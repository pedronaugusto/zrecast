//! The coverage gate. Every public name Recast and Detour declare in a header
//! has a verdict in the record, and every verdict's evidence resolves against
//! the tree: BOUND and EXTENSION name symbols `ffi/*.h` declares, LANGUAGE a
//! Zig facility from a closed list, ZIG a declaration in `src/`, INTERNAL the
//! place upstream itself marks the name internal, and UNDEFINED a header line
//! no vendored source defines. A GAP fails, and has to name the tranche of
//! work that closes it: a gap nobody has scheduled is a wish, not work.
//!
//! The record, `ci/coverage/unbound_*.txt`, is generated from three inputs and
//! checked in so a diff of it is reviewable:
//!
//!   the harvest          every public name the vendored headers declare,
//!                        read by the declaration scanner below
//!   ci/coverage/bindings.tsv   the names that are answered, and how
//!   `tranche`            which tranche closes each of the rest
//!
//! Through bay, from the repository root:
//!
//!   bay check-coverage                    the gate
//!   bay coverage -- --list                the gate, and the open gaps
//!   bay coverage -- --record              write the record
//!   bay coverage -- --check-record        the record matches its inputs
//!   bay coverage -- --names               AREA<TAB>SYMBOL, every public name
//!   bay coverage -- --classify            the INTERNAL verdicts upstream justifies
//!   bay coverage -- --summary [AREA]      verdicts per area, and one area's names
//!   bay coverage -- --audit               lines read that yielded no name
//!   bay coverage -- --collapsed           symbols covering more than one declaration
//!
//! Every order is byte order, so the record is the same file on every host.
const std = @import("std");
const Io = std.Io;
const mem = std.mem;
const Allocator = mem.Allocator;

/// The vendored upstream tree.
const recast = "libs/recastnavigation";
/// The hand-maintained half of the record.
const bindings_path = "ci/coverage/bindings.tsv";
/// C entry points no Zig wrapper calls, each with its reason.
const exceptions_path = "ci/coverage/zig_surface_exceptions.txt";
/// The largest file read.
const max_file_bytes = 1 << 22;
/// How many lines of a long list one finding prints.
const max_listed = 40;
/// How many differing lines of a stale record file are printed.
const max_diff = 20;

/// One area of the record: an upstream directory's public headers, or one
/// header of it. DetourNavMeshQuery is lifted out of Detour: dtNavMeshQuery is
/// the pathfinding engine, large enough to want its own line.
const Area = struct {
    name: []const u8,
    dir: []const u8,
    /// The one header the area holds, or with `except` the one it leaves out.
    header: ?[]const u8 = null,
    except: bool = false,
};

const areas = [_]Area{
    .{ .name = "Recast", .dir = "Recast" },
    .{ .name = "Detour", .dir = "Detour", .header = "DetourNavMeshQuery.h", .except = true },
    .{ .name = "DetourNavMeshQuery", .dir = "Detour", .header = "DetourNavMeshQuery.h" },
    .{ .name = "DetourCrowd", .dir = "DetourCrowd" },
    .{ .name = "DetourTileCache", .dir = "DetourTileCache" },
    .{ .name = "DebugUtils", .dir = "DebugUtils" },
};

/// The upstream directories whose headers carry upstream's internal markers.
const marked_dirs = [_][]const u8{ "Recast", "Detour", "DetourCrowd", "DetourTileCache" };

/// Every verdict, in byte order.
const verdicts = [_][]const u8{ "BOUND", "EXTENSION", "GAP", "INTERNAL", "LANGUAGE", "UNDEFINED", "ZIG" };
/// The verdicts a line of `ci/coverage/bindings.tsv` can carry: every one but GAP.
const binding_verdicts = [_][]const u8{ "BOUND", "EXTENSION", "LANGUAGE", "ZIG", "INTERNAL", "UNDEFINED" };

/// The Zig facilities LANGUAGE evidence may name, matched as prefixes, so the
/// verdict cannot become a home for anything inconvenient.
const facilities = [_][]const u8{
    "@Vector",     "@shuffle",      "@splat",            "@bitCast",        "@ptrCast", "@floatCast",
    "@intCast",    "@enumFromInt",  "@intFromEnum",      "@as",             "@min",     "@max",
    "@abs",        "@sqrt",         "@floor",            "@ceil",           "@cos",     "@sin",
    "@memcpy",     "@memset",       "@byteSwap",         "@fieldParentPtr", "std.math", "std.mem",
    "std.sort",    "std.ArrayList", "std.PriorityQueue", "std.heap",        "std.fmt",  "std.Thread",
    "std.testing",
};

/// Words in LANGUAGE evidence that name a facility with no `@` or `std.`.
const language_words = [_][]const u8{ "Zig slices", "defer", "errdefer", "discard" };

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    var arena: std.heap.ArenaAllocator = .init(init.gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const args = try init.minimal.args.toSlice(a);
    var out_buffer: [4096]u8 = undefined;
    var err_buffer: [4096]u8 = undefined;
    var stdout = Io.File.stdout().writer(io, &out_buffer);
    var stderr = Io.File.stderr().writer(io, &err_buffer);
    var r: Report = .{ .out = &stdout.interface, .err = &stderr.interface };
    const code = try run(.{ .io = io, .dir = Io.Dir.cwd(), .a = a }, &r, args);
    try stdout.interface.flush();
    try stderr.interface.flush();
    return code;
}

/// Runs the mode `args` names and returns the exit code.
fn run(t: Tree, r: *Report, args: []const [:0]const u8) !u8 {
    const mode: []const u8 = if (args.len > 1) args[1] else "";
    if (eql(mode, "") or eql(mode, "--list")) return gate(t, r, eql(mode, "--list"));
    if (eql(mode, "--record") or eql(mode, "--check-record")) return record(t, r, eql(mode, "--check-record"));
    if (eql(mode, "--summary")) {
        try summary(t, r.out, if (args.len > 2) args[2] else null);
        return 0;
    }
    if (eql(mode, "--names")) {
        for (areas) |area| {
            for (try symbolsOf(t.a, (try harvestArea(t, area)).decls)) |s| try r.out.print("{s}\t{s}\n", .{ area.name, s });
        }
        return 0;
    }
    if (eql(mode, "--classify")) {
        for (try classify(t, try harvestNames(t))) |p| try r.out.print("{s}\tINTERNAL\t{s}\n", .{ p.name, p.ref });
        return 0;
    }
    if (eql(mode, "--audit")) {
        for (areas) |area| {
            for ((try harvestArea(t, area)).skipped) |line| try r.out.print("{s}\n", .{line});
        }
        return 0;
    }
    if (eql(mode, "--collapsed")) {
        for (try collapsed(t)) |line| try r.out.print("{s}\n", .{line});
        return 0;
    }
    try r.err.print("unknown mode {s}; the modes are listed at the top of ci/coverage.zig\n", .{mode});
    return 2;
}

/// Where findings and the summary go, and how many checks failed.
const Report = struct {
    out: *Io.Writer,
    err: *Io.Writer,
    fails: usize = 0,

    /// Prints one failed check's headline.
    fn fail(r: *Report, comptime format: []const u8, args: anytype) !void {
        try r.err.print(format ++ "\n", args);
        r.fails += 1;
    }

    /// Prints up to `max` of `lines`, indented.
    fn listed(r: *Report, lines: []const []const u8, max: usize) !void {
        for (lines[0..@min(max, lines.len)]) |line| try r.err.print("  {s}\n", .{line});
    }
};

/// The repository the gate reads, rooted at `dir`.
const Tree = struct {
    io: Io,
    dir: Io.Dir,
    a: Allocator,

    /// A file's contents, or null when there is no such file.
    fn read(t: Tree, path: []const u8) !?[]const u8 {
        return t.dir.readFileAlloc(t.io, path, t.a, .limited(max_file_bytes)) catch |err| switch (err) {
            error.FileNotFound, error.IsDir => null,
            else => |e| e,
        };
    }

    /// The entries of directory `path` of one kind whose names carry `prefix`
    /// and `suffix`, in byte order, as a shell glob lists them; with `hidden`,
    /// dot-names too, as `find` lists them. Empty when there is no directory.
    fn list(t: Tree, path: []const u8, kind: Io.File.Kind, prefix: []const u8, suffix: []const u8, hidden: bool) ![]const []const u8 {
        var dir = t.dir.openDir(t.io, path, .{ .iterate = true }) catch |err| switch (err) {
            error.FileNotFound, error.NotDir => return &.{},
            else => |e| return e,
        };
        defer dir.close(t.io);
        var names: std.ArrayList([]const u8) = .empty;
        var it = dir.iterate();
        while (try it.next(t.io)) |entry| {
            if (entry.kind != kind or (!hidden and entry.name[0] == '.')) continue;
            if (!mem.startsWith(u8, entry.name, prefix) or !mem.endsWith(u8, entry.name, suffix)) continue;
            try names.append(t.a, try t.a.dupe(u8, entry.name));
        }
        sortBytes(names.items);
        return names.items;
    }

    /// The files `list` names, read, with their paths.
    fn files(t: Tree, path: []const u8, prefix: []const u8, suffix: []const u8) ![]const Named {
        var out: std.ArrayList(Named) = .empty;
        for (try t.list(path, .file, prefix, suffix, false)) |name| {
            const full = try t.a.print("{s}/{s}", .{ path, name });
            try out.append(t.a, .{ .name = full, .ref = (try t.read(full)) orelse continue });
        }
        return out.items;
    }
};

/// Two strings carried together: a name and what it refers to.
const Named = struct { name: []const u8, ref: []const u8 };

//===========================================================================
// The declaration scanner.
//
// Every name Recast and Detour declare in a header, by kind: functions, data
// members, enum constants, static constants and type names. Recast's surface
// is not mostly functions: dtNavMeshCreateParams has 31 data members a navmesh
// needs populated, dtStatus's twelve detail bits are `static const unsigned
// int` rather than an enum, and six enums carry no rc/dt prefix at all.
//
// A declaration spanning several physical lines is read once, in full: lines
// are joined while parentheses are unbalanced, so a continuation is never
// taken for a declaration of its own. Every brace opens a scope, so a
// function body is a scope too and nothing inside one is read as a
// declaration. Only file, class and enum scopes are harvested.
//
// What it cannot see, so silence is never mistaken for coverage:
//
//   - preprocessor macros (rcLikely, RC_SIZE_MAX, the assert macros)
//   - operator overloads, which have no name a C ABI can carry
//   - anything declared only in a .cpp, which no host can reach either
//
// `--audit` prints every harvestable line that yielded no name; that list is
// the statement of what the scanner does not understand.
//===========================================================================

/// One declaration read from a header.
const Decl = struct {
    /// func, field, enum, const or type.
    kind: []const u8,
    /// The enclosing class, struct or enum, or "-" at file scope.
    owner: []const u8,
    name: []const u8,
    /// A function's parameter count, "-" otherwise. Overloads are separate
    /// capabilities: dtNavMesh::init takes a single tile or a tile grid.
    sig: []const u8,
    /// Whether every enclosing scope is public: a public member of a
    /// privately nested type is unreachable anyway.
    public: bool,
    file: []const u8,
    line: usize,
    /// A function's parameter types, which tell apart overloads an arity
    /// cannot; empty otherwise.
    disc: []const u8,
};

const ScopeKind = enum { file, block, class, @"enum" };
const Scope = struct { kind: ScopeKind, name: []const u8, public: bool };

/// Reads declarations out of header text, one file at a time.
const Scanner = struct {
    a: Allocator,
    decls: *std.ArrayList(Decl),
    file: []const u8 = "",
    scopes: std.ArrayList(Scope) = .empty,
    /// The scope the next `{` opens, set by the declaration that introduces it.
    pending: ?Scope = null,
    in_comment: bool = false,
    /// The line the declaration being read starts on.
    start: usize = 0,
    emitted: bool = false,
    /// Harvestable lines that yielded no name, as FILE:LINE<TAB>line.
    skipped: std.ArrayList([]const u8) = .empty,

    /// Reads every declaration in one header.
    fn scan(s: *Scanner, file: []const u8, text: []const u8) !void {
        s.file = file;
        s.scopes.clearRetainingCapacity();
        s.pending = null;
        s.in_comment = false;
        var in_directive = false;
        var lines: Lines = .init(text);
        while (lines.next()) |raw| {
            var line = try s.strip(raw);
            // A directive, and every line its trailing backslash continues.
            // rcAssert's body is a braced block, and letting it into the
            // scope count would move every later declaration into a scope
            // that does not exist.
            if (in_directive or mem.startsWith(u8, trimStart(line), "#")) {
                in_directive = continued(line);
                continue;
            }
            line = dropTemplate(line);
            if (trimStart(line).len == 0) continue;
            s.start = lines.number;
            while (unbalanced(line)) {
                const next = try s.strip(lines.next() orelse break);
                if (mem.startsWith(u8, trimStart(next), "#")) continue;
                line = try mem.concat(s.a, u8, &.{ line, " ", next });
            }
            try s.step(line);
        }
    }

    /// One logical line: an access specifier, or a declaration, then the
    /// scopes its braces open and close.
    fn step(s: *Scanner, line: []const u8) !void {
        const kind: ScopeKind = if (s.top()) |t| t.kind else .file;
        if (access(line)) |public| {
            if (s.top()) |t| if (t.kind == .class) {
                t.public = public;
            };
        } else if (kind != .block) {
            s.emitted = false;
            try s.harvest(line, kind);
            if (!s.emitted) try s.skipped.append(s.a, try s.a.print("{s}:{d}\t{s}", .{ s.file, s.start, line }));
        }
        for (line) |c| switch (c) {
            '{' => {
                try s.scopes.append(s.a, s.pending orelse .{ .kind = .block, .name = "-", .public = true });
                s.pending = null;
            },
            '}' => _ = s.scopes.pop(),
            else => {},
        };
    }

    fn top(s: *Scanner) ?*Scope {
        if (s.scopes.items.len == 0) return null;
        return &s.scopes.items[s.scopes.items.len - 1];
    }

    /// The enclosing type's name, or "-".
    fn owner(s: *Scanner) []const u8 {
        const t = s.top() orelse return "-";
        return if (t.kind == .block) "-" else t.name;
    }

    /// `raw` without comments, and with string and character literals read
    /// as `0`: DT_NAVMESH_MAGIC is spelled 'D'<<24 | 'N'<<16 | ..., and a
    /// literal can carry a brace or a parenthesis that would unbalance the
    /// scope count.
    fn strip(s: *Scanner, raw: []const u8) ![]const u8 {
        const code = try stripComments(s.a, raw, &s.in_comment);
        return replaceLiterals(s.a, try replaceLiterals(s.a, code, '"'), '\'');
    }

    /// Every name one logical line declares.
    fn harvest(s: *Scanner, l: []const u8, kind: ScopeKind) !void {
        const t = trimStart(l);
        // typedef: a name a host writes though it declares no storage. A
        // function typedef's name sits inside the first parenthesised group
        // (`typedef void (dtFreeFunc)(void* ptr);`), not at the end of the
        // line, where the last identifier is a parameter.
        if (mem.startsWith(u8, t, "typedef") and t.len > 7 and isSpace(t[7])) {
            const decl = try dropBrackets(s.a, cut(l, ';'), "");
            return s.emit("type", lastIdent(innerGroup(decl) orelse decl), "-", "");
        }
        // The keyword also introduces an elaborated type specifier in an
        // ordinary declaration (`struct dtTileCacheAlloc* getAlloc()`), so
        // what follows the name decides. A body, a base clause or the end of
        // the line is a definition; `;` is a forward declaration, emitted
        // where it is defined; anything else is a use, read by the rules below.
        if (introduction(l, &.{ "class", "struct", "union" })) |intro| {
            if (opensDefinition(intro.rest) or isFinal(intro.rest)) {
                try s.emit("type", intro.name, "-", "");
                s.pending = .{ .kind = .class, .name = intro.name, .public = !eql(intro.keyword, "class") };
                return;
            }
            if (intro.rest[0] == ';') return;
        }
        // An anonymous aggregate declares its members in the enclosing type:
        // dtTileCacheObstacle's union of cylinder, box and orientedBox.
        if (anonymous(l)) |public| {
            s.pending = .{ .kind = .class, .name = s.owner(), .public = public };
            return;
        }
        if (introduction(l, &.{"enum"})) |intro| {
            if (opensDefinition(intro.rest)) {
                try s.emit("type", intro.name, "-", "");
                s.pending = .{ .kind = .@"enum", .name = intro.name, .public = true };
                return;
            }
            if (intro.rest[0] == ';') return;
        }
        if (kind == .@"enum") return s.emitList("enum", try removeFirst(s.a, cut(l, '}'), '{'));
        // `static const unsigned int DT_PARTIAL_RESULT = 1 << 6;` at file
        // scope and `static const int MAX_LOCAL_SEGS = 8;` in a class read the
        // same way. The initialiser may call a function, so this comes first.
        if (staticConst(l) or (kind == .file and fileConst(l))) {
            return s.emit("const", lastIdent(try dropBrackets(s.a, cut(cut(l, ';'), '='), "")), "-", "");
        }
        // An operator has no name a C ABI can carry, and its declaration
        // would otherwise be read as whatever call its inline body makes.
        if (wordIn(l, "operator")) return;
        if (call(l)) |c| return s.function(l, c);
        // A data member, only inside a type: Recast has no file-scope
        // variable outside `static const`.
        const excluded = [_][]const u8{ "using", "friend", "return", "}", "{" };
        if (kind == .class and mem.endsWith(u8, trimEnd(l), ";") and !startsWithAny(t, &excluded)) {
            try s.emitList("field", cut(l, ';'));
        }
    }

    /// A function is the first identifier followed by `(`: a constructor's
    /// member-initialiser list is a run of identifiers followed by `(` too,
    /// and only the leftmost is the function being declared. A `{` earlier
    /// on the line puts the match inside a body. Either way the line is done:
    /// a declarator list never contains `(` in these headers.
    fn function(s: *Scanner, l: []const u8, c: Call) !void {
        const pre: u8 = if (c.start > 0) l[c.start - 1] else 0;
        if (mem.findScalar(u8, l[0 .. c.start + 1], '{') != null) return;
        if (pre == '.' or pre == '>' or pre == ':' or keyword(c.name) or typeword(c.name)) return;
        const name = if (pre == '~') try mem.concat(s.a, u8, &.{ "~", c.name }) else c.name;
        try s.emit("func", name, try arity(s.a, l[c.start..]), try paramTypes(s.a, l[c.start..]));
    }

    fn emit(s: *Scanner, kind: []const u8, name: []const u8, sig: []const u8, disc: []const u8) !void {
        if (name.len == 0 or keyword(name) or typeword(name)) return;
        s.emitted = true;
        var public = true;
        for (s.scopes.items) |scope| {
            if (scope.kind == .class and !scope.public) public = false;
        }
        try s.decls.append(s.a, .{
            .kind = kind,
            .owner = s.owner(),
            .name = name,
            .sig = sig,
            .public = public,
            .file = s.file,
            .line = s.start,
            .disc = disc,
        });
    }

    /// Enumerators and multi-declarator members both arrive as a
    /// comma-separated list: `float cs, ch;` declares two names.
    fn emitList(s: *Scanner, kind: []const u8, decl: []const u8) !void {
        var parts = mem.splitScalar(u8, try dropBrackets(s.a, decl, ""), ',');
        while (parts.next()) |part| {
            const p = cut(cut(part, '='), ':');
            try s.emit(kind, if (eql(kind, "enum")) firstIdent(p) else lastIdent(p), "-", "");
        }
    }
};

/// The lines of a text as awk reads them: split on newline, with no empty
/// line after a final newline.
const Lines = struct {
    rest: ?[]const u8,
    number: usize = 0,

    fn init(text: []const u8) Lines {
        return .{ .rest = if (text.len == 0) null else text };
    }

    fn next(l: *Lines) ?[]const u8 {
        const rest = l.rest orelse return null;
        l.number += 1;
        const at = mem.findScalar(u8, rest, '\n') orelse {
            l.rest = null;
            return rest;
        };
        l.rest = if (at + 1 == rest.len) null else rest[at + 1 ..];
        return rest[0..at];
    }
};

/// `raw` without its comments, carrying an unterminated `/*` to the next line
/// through `in_comment`.
fn stripComments(a: Allocator, raw: []const u8, in_comment: *bool) ![]const u8 {
    var l = raw;
    if (in_comment.*) {
        const end = mem.find(u8, l, "*/") orelse return "";
        l = l[end + 2 ..];
        in_comment.* = false;
    }
    while (mem.find(u8, l, "/*")) |open| {
        const close = mem.find(u8, l[open + 2 ..], "*/") orelse {
            l = l[0..open];
            in_comment.* = true;
            break;
        };
        l = try mem.concat(a, u8, &.{ l[0..open], l[open + 2 + close + 2 ..] });
    }
    return cutText(l, "//");
}

/// `s` with each pair of `quote` and what it encloses read as `0`.
fn replaceLiterals(a: Allocator, s: []const u8, quote: u8) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;
    var from: usize = 0;
    while (mem.findScalarPos(u8, s, from, quote)) |open| {
        const close = mem.findScalarPos(u8, s, open + 1, quote) orelse break;
        try out.appendSlice(a, s[from..open]);
        try out.append(a, '0');
        from = close + 1;
    }
    try out.appendSlice(a, s[from..]);
    return out.items;
}

/// `s` with every `[...]` replaced by `with`: array bounds are not names.
fn dropBrackets(a: Allocator, s: []const u8, with: []const u8) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;
    var from: usize = 0;
    while (mem.findScalarPos(u8, s, from, '[')) |open| {
        const close = mem.findScalarPos(u8, s, open + 1, ']') orelse break;
        try out.appendSlice(a, s[from..open]);
        try out.appendSlice(a, with);
        from = close + 1;
    }
    try out.appendSlice(a, s[from..]);
    return out.items;
}

/// `s` without the first `c`.
fn removeFirst(a: Allocator, s: []const u8, c: u8) ![]const u8 {
    const at = mem.findScalar(u8, s, c) orelse return s;
    return mem.concat(a, u8, &.{ s[0..at], s[at + 1 ..] });
}

/// `line` without a leading `template<...>`.
fn dropTemplate(line: []const u8) []const u8 {
    const t = trimStart(line);
    if (!mem.startsWith(u8, t, "template")) return line;
    const args = trimStart(t["template".len..]);
    if (args.len == 0 or args[0] != '<') return line;
    const close = mem.findScalar(u8, args, '>') orelse return line;
    return trimStart(args[close + 1 ..]);
}

/// Whether a directive line continues onto the next.
fn continued(line: []const u8) bool {
    return mem.endsWith(u8, trimEnd(line), "\\");
}

/// Whether `s` opens more parentheses than it closes.
fn unbalanced(s: []const u8) bool {
    return mem.countScalar(u8, s, '(') > mem.countScalar(u8, s, ')');
}

/// The access an access specifier line sets, or null for any other line.
fn access(l: []const u8) ?bool {
    const t = trimStart(l);
    for ([_][]const u8{ "public", "private", "protected" }) |word| {
        if (mem.startsWith(u8, t, word) and mem.startsWith(u8, trimStart(t[word.len..]), ":")) return eql(word, "public");
    }
    return null;
}

/// A keyword introducing a type, with the name after it and the rest of the
/// line: the leftmost `(^|[^A-Za-z0-9_])KEYWORD[[:space:]]+IDENT`.
const Intro = struct { keyword: []const u8, name: []const u8, rest: []const u8 };

fn introduction(l: []const u8, keywords: []const []const u8) ?Intro {
    for (0..l.len) |p| {
        if (p > 0 and isWord(l[p - 1])) continue;
        for (keywords) |kw| {
            if (!mem.startsWith(u8, l[p..], kw)) continue;
            const spaces = p + kw.len;
            var j = spaces;
            while (j < l.len and isSpace(l[j])) j += 1;
            if (j == spaces or j == l.len or !isIdentStart(l[j])) continue;
            const name_start = j;
            while (j < l.len and isWord(l[j])) j += 1;
            return .{ .keyword = kw, .name = l[name_start..j], .rest = trimStart(l[j..]) };
        }
    }
    return null;
}

/// Whether what follows a type's name makes its introduction a definition.
fn opensDefinition(rest: []const u8) bool {
    return rest.len == 0 or rest[0] == '{' or rest[0] == ':';
}

fn isFinal(rest: []const u8) bool {
    if (!mem.startsWith(u8, rest, "final")) return false;
    return rest.len == 5 or isSpace(rest[5]) or rest[5] == '{' or rest[5] == ':';
}

/// For a line that opens an anonymous aggregate, whether its members are
/// public; null for any other line.
fn anonymous(l: []const u8) ?bool {
    const t = trimStart(l);
    for ([_][]const u8{ "union", "struct", "class" }) |kw| {
        if (!mem.startsWith(u8, t, kw)) continue;
        var rest = trimStart(t[kw.len..]);
        if (rest.len > 0 and rest[0] == '{') rest = trimStart(rest[1..]);
        return if (rest.len == 0) !eql(kw, "class") else null;
    }
    return null;
}

/// Whether `l` holds `static const` followed by a space.
fn staticConst(l: []const u8) bool {
    for (0..l.len) |p| {
        if (!mem.startsWith(u8, l[p..], "static") or (p > 0 and isWord(l[p - 1]))) continue;
        var j = p + "static".len;
        const spaces = j;
        while (j < l.len and isSpace(l[j])) j += 1;
        if (j > spaces and mem.startsWith(u8, l[j..], "const") and j + 5 < l.len and isSpace(l[j + 5])) return true;
    }
    return false;
}

/// Whether `l` opens with `const`, a space, and later an `=`.
fn fileConst(l: []const u8) bool {
    const t = trimStart(l);
    return mem.startsWith(u8, t, "const") and t.len > 5 and isSpace(t[5]) and mem.findScalar(u8, t[6..], '=') != null;
}

/// Whether `word` occurs in `l` as a whole word.
fn wordIn(l: []const u8, word: []const u8) bool {
    var from: usize = 0;
    while (mem.findPos(u8, l, from, word)) |at| : (from = at + 1) {
        const end = at + word.len;
        if ((at == 0 or !isWord(l[at - 1])) and (end == l.len or !isWord(l[end]))) return true;
    }
    return false;
}

/// The leftmost identifier followed, after any space, by `(`.
const Call = struct { start: usize, name: []const u8 };

fn call(l: []const u8) ?Call {
    var i: usize = 0;
    while (i < l.len) : (i += 1) {
        if (!isIdentStart(l[i])) continue;
        var j = i;
        while (j < l.len and isWord(l[j])) j += 1;
        const name_end = j;
        while (j < l.len and isSpace(l[j])) j += 1;
        if (j < l.len and l[j] == '(') return .{ .start = i, .name = l[i..name_end] };
        // Every later start inside this word ends where it does, and fails too.
        i = name_end - 1;
    }
    return null;
}

/// The number of parameters between the first balanced pair of parentheses.
/// Commas nested in parentheses or template arguments belong to a parameter.
fn arity(a: Allocator, s: []const u8) ![]const u8 {
    var i = mem.findScalar(u8, s, '(') orelse return "-";
    var parens: usize = 0;
    var angles: usize = 0;
    var commas: usize = 0;
    var seen = false;
    while (i < s.len) : (i += 1) {
        const c = s[i];
        switch (c) {
            '(' => parens += 1,
            ')' => {
                parens -= 1;
                if (parens == 0) break;
            },
            '<' => angles += 1,
            '>' => angles -|= 1,
            else => {
                if (c == ',' and parens == 1 and angles == 0) commas += 1;
                if (isWord(c) and parens == 1) seen = true;
            },
        }
    }
    if (!seen) return "0";
    return a.print("{d}", .{commas + 1});
}

/// The parameter types of the first balanced parenthesis group, as
/// `(type,type)`, with `const` after it for a const member function. An
/// arity alone cannot separate every overload: dtSwapEndian has five forms
/// that take one argument, rcRasterizeTriangles two that take eight and
/// differ in the fourth, and eleven member functions differ only in
/// constness. Types are stable across upstream reformatting; an ordinal is not.
fn paramTypes(a: Allocator, s: []const u8) ![]const u8 {
    var i = mem.findScalar(u8, s, '(') orelse return "";
    var parens: usize = 0;
    var angles: usize = 0;
    var closed = false;
    var cur: std.ArrayList(u8) = .empty;
    var out: std.ArrayList(u8) = .empty;
    while (i < s.len) : (i += 1) {
        const c = s[i];
        switch (c) {
            '(' => {
                parens += 1;
                if (parens == 1) continue;
            },
            ')' => {
                parens -= 1;
                if (parens == 0) {
                    closed = true;
                    break;
                }
            },
            '<' => angles += 1,
            '>' => angles -|= 1,
            ',' => if (parens == 1 and angles == 0) {
                try appendType(a, &out, cur.items);
                cur.clearRetainingCapacity();
                continue;
            },
            else => {},
        }
        try cur.append(a, c);
    }
    if (hasWord(cur.items)) try appendType(a, &out, cur.items);
    const tail = trimStart(if (closed) s[i + 1 ..] else "");
    const constant = mem.startsWith(u8, tail, "const") and (tail.len == 5 or !isWord(tail[5]));
    return a.print("({s}){s}", .{ out.items, if (constant) "const" else "" });
}

fn appendType(a: Allocator, out: *std.ArrayList(u8), param: []const u8) !void {
    if (out.items.len > 0) try out.append(a, ',');
    try out.appendSlice(a, try paramType(a, param));
}

/// One parameter reduced to its type: default argument dropped, array
/// extent emptied, parameter name removed and space around `*` and `&`
/// closed up. The name is the last identifier when something precedes it
/// and it is not a type word, which keeps `unsigned char` whole while taking
/// the `v` from `float* v`.
fn paramType(a: Allocator, raw: []const u8) ![]const u8 {
    var p = cut(trim(raw), '=');
    p = trim(try collapseSpace(a, try dropBrackets(a, p, "[]")));
    if (trailingIdent(p)) |at| {
        if (!typeword(p[at..]) and at > 0) p = p[0..at];
    }
    return trim(try closeUp(a, try closeUp(a, p, '*'), '&'));
}

/// Where the identifier that ends `p` starts, or null.
fn trailingIdent(p: []const u8) ?usize {
    var k = p.len;
    while (k > 0 and isWord(p[k - 1])) k -= 1;
    while (k < p.len and !isIdentStart(p[k])) k += 1;
    return if (k < p.len) k else null;
}

/// `s` with every run of space read as one space.
fn collapseSpace(a: Allocator, s: []const u8) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;
    var in_space = false;
    for (s) |c| {
        if (isSpace(c)) {
            if (!in_space) try out.append(a, ' ');
            in_space = true;
            continue;
        }
        in_space = false;
        try out.append(a, c);
    }
    return out.items;
}

/// `s` with the space on both sides of every `c` removed.
fn closeUp(a: Allocator, s: []const u8, c: u8) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;
    var after = false;
    for (s) |ch| {
        if (ch == c) {
            out.items.len = trimEnd(out.items).len;
            try out.append(a, c);
            after = true;
        } else if (!(after and isSpace(ch))) {
            after = false;
            try out.append(a, ch);
        }
    }
    return out.items;
}

/// The contents of the first parenthesis pair holding no other, or null.
fn innerGroup(s: []const u8) ?[]const u8 {
    var from: usize = 0;
    while (mem.findScalarPos(u8, s, from, '(')) |open| : (from = open + 1) {
        const next = mem.findAnyPos(u8, s, open + 1, "()") orelse return null;
        if (s[next] == ')') return s[open + 1 .. next];
    }
    return null;
}

fn lastIdent(s: []const u8) []const u8 {
    var name: []const u8 = "";
    var i: usize = 0;
    while (i < s.len) {
        if (!isIdentStart(s[i])) {
            i += 1;
            continue;
        }
        const start = i;
        while (i < s.len and isWord(s[i])) i += 1;
        name = s[start..i];
    }
    return name;
}

fn firstIdent(s: []const u8) []const u8 {
    for (s, 0..) |c, i| {
        if (!isIdentStart(c)) continue;
        var j = i;
        while (j < s.len and isWord(s[j])) j += 1;
        return s[i..j];
    }
    return "";
}

fn keyword(n: []const u8) bool {
    return oneOf(n, "", &.{
        "if",   "for",   "while",    "switch",   "return",   "sizeof",   "alignof", "static_assert",
        "do",   "else",  "catch",    "new",      "delete",   "defined",  "throw",   "goto",
        "case", "break", "continue", "template", "typename", "operator",
    });
}

/// A type keyword at the head of a declaration is not the thing declared.
fn typeword(n: []const u8) bool {
    return oneOf(n, "", &.{
        "void",   "bool",     "char",      "short",    "int",    "long",     "float",   "double",
        "signed", "unsigned", "const",     "volatile", "static", "inline",   "virtual", "explicit",
        "extern", "struct",   "class",     "enum",     "union",  "typedef",  "using",   "friend",
        "public", "private",  "protected", "size_t",   "auto",   "register", "mutable",
    });
}

/// One area's declarations, and the lines it read and took no name from.
const Harvested = struct { decls: []const Decl, skipped: []const []const u8 };

fn harvestArea(t: Tree, area: Area) !Harvested {
    const dir = try t.a.print("{s}/{s}/Include", .{ recast, area.dir });
    var decls: std.ArrayList(Decl) = .empty;
    var scanner: Scanner = .{ .a = t.a, .decls = &decls };
    for (try t.list(dir, .file, "", ".h", false)) |name| {
        if (area.header) |header| if (eql(name, header) == area.except) continue;
        const path = try t.a.print("{s}/{s}", .{ dir, name });
        try scanner.scan(path, (try t.read(path)) orelse continue);
    }
    return .{ .decls = decls.items, .skipped = scanner.skipped.items };
}

/// The public symbols of one area's declarations, in byte order, each
/// KIND:OWNER::NAME, with /ARITY on a function. The parameter types follow
/// only where an arity does not tell two forms apart, so a name that never
/// collided keeps its plain spelling.
fn symbolsOf(a: Allocator, decls: []const Decl) ![]const []const u8 {
    var forms: std.StringHashMap(usize) = .init(a);
    var seen: std.StringHashMap(void) = .init(a);
    var keys: std.ArrayList(Named) = .empty;
    for (decls) |d| {
        if (!d.public) continue;
        const symbol = if (eql(d.kind, "func"))
            try a.print("{s}:{s}::{s}/{s}", .{ d.kind, d.owner, d.name, d.sig })
        else
            try a.print("{s}:{s}::{s}", .{ d.kind, d.owner, d.name });
        try keys.append(a, .{ .name = symbol, .ref = d.disc });
        if ((try seen.getOrPut(try a.print("{s}\x1c{s}", .{ symbol, d.disc }))).found_existing) continue;
        const f = try forms.getOrPut(symbol);
        f.value_ptr.* = if (f.found_existing) f.value_ptr.* + 1 else 1;
    }
    var out: std.ArrayList([]const u8) = .empty;
    for (keys.items) |k| {
        const many = (forms.get(k.name) orelse 1) > 1;
        try out.append(a, if (many) try mem.concat(a, u8, &.{ k.name, k.ref }) else k.name);
    }
    return unique(out.items);
}

/// Every public name upstream declares, as AREA<TAB>SYMBOL in byte order.
fn harvestNames(t: Tree) ![]const []const u8 {
    var names: std.ArrayList([]const u8) = .empty;
    for (areas) |area| {
        for (try symbolsOf(t.a, (try harvestArea(t, area)).decls)) |s| {
            try names.append(t.a, try t.a.print("{s}\t{s}", .{ area.name, s }));
        }
    }
    return unique(names.items);
}

/// Two declarations reduced to one symbol after arity and parameter types
/// have both been applied: a name upstream declares twice, a typedef with an
/// #ifdef branch each way. AREA<TAB>SYMBOL<TAB>COUNT<TAB> FILE:LINE...
fn collapsed(t: Tree) ![]const []const u8 {
    const Seen = struct { count: usize = 0, where: std.ArrayList(u8) = .empty };
    var lines: std.ArrayList([]const u8) = .empty;
    for (areas) |area| {
        var seen: std.StringHashMap(Seen) = .init(t.a);
        for ((try harvestArea(t, area)).decls) |d| {
            if (!d.public) continue;
            const symbol = if (eql(d.kind, "func"))
                try t.a.print("{s}:{s}::{s}/{s}{s}", .{ d.kind, d.owner, d.name, d.sig, d.disc })
            else
                try t.a.print("{s}:{s}::{s}", .{ d.kind, d.owner, d.name });
            const s = try seen.getOrPut(symbol);
            if (!s.found_existing) s.value_ptr.* = .{};
            s.value_ptr.count += 1;
            try s.value_ptr.where.print(t.a, " {s}:{d}", .{ d.file, d.line });
        }
        var it = seen.iterator();
        while (it.next()) |s| {
            const n = s.value_ptr.count;
            if (n > 1) try lines.append(t.a, try t.a.print("{s}\t{s}\t{d}\t{s}", .{ area.name, s.key_ptr.*, n, s.value_ptr.where.items }));
        }
    }
    sortBytes(lines.items);
    return lines.items;
}

//===========================================================================
// What upstream itself justifies excluding.
//
// An exclusion is legitimate only when upstream marks the name internal.
// Recast has one phrase for that, "generally meant for internal use only",
// and it appears in two positions:
//
//   group  a doxygen @name banner: DetourNavMesh.h opens "Encoding and
//          Decoding" with it, and it holds until the closing `/// @}`
//   decl   an @note on the comment immediately above one declaration
//
// Nothing else in the vendored headers claims to be internal. A name that is
// awkward to bind is not internal, and stays a GAP until it is bound.
//===========================================================================

/// Appends NAME<TAB>FILE:LINE<TAB>group|decl for every name `text` marks.
fn markIn(a: Allocator, file: []const u8, text: []const u8, out: *std.ArrayList([]const u8)) !void {
    var group = false;
    var mark: []const u8 = "";
    var pending = false;
    var pending_mark: []const u8 = "";
    var in_comment = false;
    var lines: Lines = .init(text);
    while (lines.next()) |raw| {
        if (mem.find(u8, raw, "generally meant for internal use only") != null) {
            const where = try a.print("{s}:{d}", .{ file, lines.number });
            if (mem.find(u8, raw, "These functions are") != null) {
                group = true;
                mark = where;
            } else {
                pending = true;
                pending_mark = where;
            }
            continue;
        }
        const t = trimStart(raw);
        if (mem.startsWith(u8, t, "///")) {
            if (group and mem.startsWith(u8, trimStart(t[3..]), "@}")) group = false;
            continue;
        }
        const line = try stripComments(a, raw, &in_comment);
        if (mem.startsWith(u8, trimStart(line), "#") or mem.trim(u8, line, " \t\n").len == 0) continue;
        const c = call(line) orelse continue;
        const pre: u8 = if (c.start > 0) line[c.start - 1] else 0;
        const statements = [_][]const u8{ "if", "for", "while", "switch", "return", "sizeof", "do", "else", "operator", "new", "delete" };
        const declaration = pre != '.' and pre != '>' and pre != ':' and !oneOf(c.name, "", &statements);
        if (declaration and pending) {
            try out.append(a, try a.print("{s}\t{s}\tdecl", .{ c.name, pending_mark }));
        } else if (declaration and group) {
            try out.append(a, try a.print("{s}\t{s}\tgroup", .{ c.name, mark }));
        }
        pending = false;
    }
}

/// The INTERNAL verdict of every public name upstream marks internal, as
/// SYMBOL and evidence, in the order of `names`.
fn classify(t: Tree, names: []const []const u8) ![]const Named {
    var marks: std.ArrayList([]const u8) = .empty;
    for (marked_dirs) |dir| {
        const include = try t.a.print("{s}/{s}/Include", .{ recast, dir });
        for (try t.list(include, .file, "", ".h", true)) |name| {
            const path = try t.a.print("{s}/{s}", .{ include, name });
            try markIn(t.a, path, (try t.read(path)) orelse continue, &marks);
        }
    }
    var evidence: std.StringHashMap([]const u8) = .init(t.a);
    for (unique(marks.items)) |line| {
        var fields = mem.splitScalar(u8, line, '\t');
        const name = fields.first();
        const where = fields.next() orelse "";
        const how = fields.next() orelse "";
        const e = try evidence.getOrPut(name);
        if (!e.found_existing) e.value_ptr.* = try t.a.print("upstream {s} ({s})", .{ where, how });
    }
    var symbols: std.ArrayList([]const u8) = .empty;
    for (names) |line| try symbols.append(t.a, field(line, 1));
    var out: std.ArrayList(Named) = .empty;
    for (unique(symbols.items)) |symbol| {
        if (evidence.get(bareName(symbol))) |e| try out.append(t.a, .{ .name = symbol, .ref = e });
    }
    return out.items;
}

/// A symbol's name as the markers spell it: no KIND:OWNER:: and no
/// trailing /ARITY.
fn bareName(symbol: []const u8) []const u8 {
    var n = symbol;
    if (mem.findScalar(u8, n, ':')) |colon| {
        if (colon > 0 and lowerAlpha(n[0..colon])) if (mem.findScalarPos(u8, n, colon + 1, ':')) |second| {
            if (mem.startsWith(u8, n[second..], "::")) n = n[second + 2 ..];
        };
    }
    var from: usize = 0;
    while (mem.findScalarPos(u8, n, from, '/')) |slash| : (from = slash + 1) {
        const rest = n[slash + 1 ..];
        if (rest.len > 0 and mem.findNone(u8, rest, "0123456789-") == null) return n[0..slash];
    }
    return n;
}

//===========================================================================
// The tranches: which tranche of work closes a name no binding answers. The
// reasoning for a gap lives here, once per rule, instead of once per row, and
// is re-read every time the record is generated. The rules are ordered and
// the first match wins, so a specific name is lifted out of its owner's
// tranche by a rule above the owner's.
//===========================================================================

const t2 = "tranche 2 - deterministic cooks";
const t3a = "tranche 3a - authoring: areas, off-mesh links, poly flags";
const t3b = "tranche 3b - the staged Recast pipeline";
const t4 = "tranche 4 - query breadth";
const t5 = "tranche 5 - DetourTileCache";
const t6 = "tranche 6 - DetourCrowd";
const t7 = "tranche 7 - value math, geometry and containers";
const t8 = "tranche 8 - reading a loaded navmesh back";
const t9 = "tranche 9 - DebugUtils";

/// The tranche that closes `symbol`, or null when no rule claims it.
fn tranche(area: []const u8, symbol: []const u8) ?[]const u8 {
    const owner = ownerOf(symbol);
    const name = nameOf(symbol);
    return exception(symbol, owner, name) orelse authoring(owner, name) orelse subsystem(area, owner, name);
}

/// The named exceptions, and the families that cut across areas. Each sits
/// above the rule that would otherwise claim it, so why a name moves is
/// visible where it moves.
fn exception(symbol: []const u8, owner: []const u8, name: []const u8) ?[]const u8 {
    // A bake input, the write side of dtMeshHeader::userId: it travels with
    // the rest of the authoring parameters.
    if (eql(symbol, "field:dtNavMeshCreateParams::userId")) return t3a;
    // Plain tile data, read back beside the header's other fields.
    if (eql(symbol, "field:dtNavMeshCreateParams::buildBvTree")) return t8;
    if (eql(owner, "dtTileFlags") or eql(name, "dtTileFlags") or eql(name, "DT_TILE_FREE_DATA")) return t8;
    // Geometry with a real algorithm inside. A second Zig implementation
    // would be a liability to keep in step, so these cross as entry points.
    if (oneOf(name, "dt", &.{
        "ClosestPtPointTriangle", "DistancePtPolyEdgesSqr",  "DistancePtSegSqr2D", "IntersectSegmentPoly2D",
        "IntersectSegSeg2D",      "OverlapPolyPoly2D",       "OverlapBounds",      "OverlapQuantBounds",
        "PointInPolygon",         "RandomPointInConvexPoly", "CalcPolyCenter",     "TriArea2D",
    }) or eql(name, "rcOffsetPoly")) return t7;
    // These decode rcCompactSpan::con, so they belong with the compact
    // heightfield they read rather than with value math.
    if (oneOf(name, "rc", &.{ "GetCon", "SetCon", "GetDirOffsetX", "GetDirOffsetY", "GetDirForOffset" })) return t3b;
    // What a designer's volumes become, run inside the bake.
    if (oneOf(name, "rc", &.{ "MarkBoxArea", "MarkConvexPolyArea", "MarkCylinderArea" })) return t3a;
    // Value math in both spellings, and the scalar helpers around it.
    // dtMathAtan2f carries a digit, so the Math family takes digits.
    if ((mem.startsWith(u8, name, "rcV") or mem.startsWith(u8, name, "dtV")) and alnum(name[3..])) return t7;
    if (oneOf(name, "rc", &scalar_helpers) or oneOf(name, "dt", &scalar_helpers)) return t7;
    if (oneOf(name, "dt", &.{ "Ilog2", "NextPow2", "Align4", "Alloc" })) return t7;
    if (mem.startsWith(u8, name, "dtMath") and name.len > 6 and alnum(name[6..])) return t7;
    if (eql(name, "dtGetThenAdvanceBufferPointer")) return t7;
    // Routing upstream's asserts into a host's logger, both hook families
    // together: one alone gives a host its logger for half the library and
    // a bare abort for the other half.
    if (mem.startsWith(u8, name, "dtAssertFail") or mem.startsWith(u8, name, "rcAssertFail")) return t7;
    // The C++ container family, and the scoped-delete guard beside it.
    if (oneOf(owner, "rc", containers[0..5])) return t7;
    if (oneOf(name, "rc", &containers)) return t7;
    // Byte-order swapping, and the magic naming the image being swapped: a
    // cook that is bit-identical everywhere is what makes it worth having.
    if (oneOf(name, "dt", &.{ "SwapByte", "SwapEndian", "NavMeshDataSwapEndian", "NavMeshHeaderSwapEndian" })) return t2;
    if (eql(name, "DT_NAVMESH_MAGIC")) return t2;
    return null;
}

const scalar_helpers = [_][]const u8{ "Min", "Max", "Abs", "Sqr", "Sqrt", "Clamp", "Swap", "IgnoreUnused" };
const containers = [_][]const u8{ "VectorBase", "TempVector", "PermVector", "IntArray", "ScopedDelete", "SizeType" };

/// Authoring: off-mesh connections, per-polygon areas and flags, and the
/// per-tile state that saves and restores them.
fn authoring(owner: []const u8, name: []const u8) ?[]const u8 {
    if (eql(owner, "dtPolyTypes") or eql(name, "dtPolyTypes")) return t3a;
    if (oneOf(name, "DT_", &.{ "NAVMESH_STATE_MAGIC", "NAVMESH_STATE_VERSION", "OFFMESH_CON_BIDIR" })) return t3a;
    if (eql(owner, "dtPoly") and oneOf(name, "", &.{ "areaAndtype", "flags", "getArea", "setArea", "getType", "setType" })) return t3a;
    if (eql(owner, "dtMeshTile") and eql(name, "offMeshCons")) return t3a;
    // dtOffMeshConnection::poly and ::side are the wiring rather than the
    // capability: authoring never names them and nothing a host calls
    // reports them, so they go with reading a tile's structures back.
    if (eql(owner, "dtOffMeshConnection")) return t8;
    if (eql(owner, "dtNavMeshCreateParams") and startsWithAny(name, &.{ "offMeshCon", "polyAreas", "polyFlags" })) return t3a;
    const flags = [_][]const u8{ "getOffMeshConnection", "getPolyArea", "getPolyFlags", "setPolyArea", "setPolyFlags" };
    if (eql(owner, "dtNavMesh") and startsWithAny(name, &flags)) return t3a;
    if (eql(owner, "dtNavMesh") and oneOf(name, "", &.{ "getTileStateSize", "storeTileState", "restoreTileState" })) return t3a;
    return null;
}

/// Whole subsystems, the query internals, reading a navmesh back, and what
/// is left of each area.
fn subsystem(area: []const u8, owner: []const u8, name: []const u8) ?[]const u8 {
    if (eql(area, "DebugUtils")) return t9;
    if (eql(area, "DetourCrowd")) return t6;
    if (eql(area, "DetourTileCache")) return t5;
    if (eql(area, "DetourNavMeshQuery")) return t4;
    // The layered heightfield is what the tile cache compresses and
    // rebuilds from, so it arrives with the tile cache.
    if (oneOf(owner, "rc", &.{ "HeightfieldLayer", "HeightfieldLayerSet" })) return t5;
    if (oneOf(name, "rc", &.{ "HeightfieldLayer", "HeightfieldLayerSet" })) return t5;
    if (oneOf(name, "rc", &.{ "AllocHeightfieldLayerSet", "BuildHeightfieldLayers", "FreeHeightfieldLayerSet" })) return t5;
    if (eql(owner, "dtNavMesh") and eql(name, "getTilesAt")) return t5;
    if (oneOf(owner, "dt", &.{ "Node", "NodePool", "NodeQueue", "NodeFlags" })) return t4;
    if (oneOf(name, "dt", &.{ "Node", "NodePool", "NodeQueue", "NodeFlags", "NodeIndex" })) return t4;
    if (oneOf(name, "dt", &.{
        "Status",            "FindPathOptions",    "RaycastOptions", "StraightPathOptions",
        "StraightPathFlags", "DetailTriEdgeFlags", "PolyQuery",
    })) return t4;
    if (oneOf(name, "DT_", &.{ "NULL_IDX", "POLY_BITS", "SALT_BITS", "TILE_BITS", "IN_PROGRESS", "STATUS_DETAIL_MASK" })) return t4;
    if (oneOf(name, "DT_", &.{ "MAX_STATES_PER_NODE", "NODE_PARENT_BITS", "NODE_STATE_BITS", "RAY_CAST_LIMIT_PROPORTIONS" })) return t4;
    if (oneOf(name, "dt", &.{ "StatusInProgress", "GetDetailTriEdgeFlags", "ClosestHeightPointTriangle" })) return t4;
    const refs = [_][]const u8{ "decodePolyId", "encodePolyId", "getTileAndPolyByRef", "isValidPolyRef" };
    if (eql(owner, "dtNavMesh") and startsWithAny(name, &refs)) return t4;
    const tile = [_][]const u8{ "BVNode", "Link", "MeshHeader", "MeshTile", "Poly", "PolyDetail" };
    if (oneOf(owner, "dt", &tile) or oneOf(name, "dt", &tile)) return t8;
    if (oneOf(name, "DT_", &.{ "EXT_LINK", "NULL_LINK" }) or eql(name, "dtOppositeTile")) return t8;
    if (eql(owner, "dtNavMesh") and oneOf(name, "", &.{ "calcTileLoc", "getParams", "getPolyRefBase" })) return t8;
    // What is left of Recast is the pipeline: its stages, the containers
    // they fill, the context that logs and times them, and the constants
    // they write into a mesh.
    if (eql(area, "Recast")) return t3b;
    if (eql(area, "Detour")) return t4;
    return null;
}

/// The OWNER of KIND:OWNER::NAME, or "-".
fn ownerOf(symbol: []const u8) []const u8 {
    var rest = symbol;
    if (mem.findScalar(u8, rest, ':')) |colon| {
        if (colon > 0 and lowerAlpha(rest[0..colon])) rest = rest[colon + 1 ..];
    }
    const at = mem.find(u8, rest, "::") orelse return "-";
    return rest[0..at];
}

/// The NAME of KIND:OWNER::NAME, without arity or parameter types.
fn nameOf(symbol: []const u8) []const u8 {
    var rest = symbol;
    var from: usize = 0;
    while (mem.findScalarPos(u8, rest, from, '/')) |slash| : (from = slash + 1) {
        var j = slash + 1;
        while (j < rest.len and std.ascii.isDigit(rest[j])) j += 1;
        if (j > slash + 1 and (j == rest.len or rest[j] == '(')) {
            rest = rest[0..slash];
            break;
        }
    }
    const at = mem.find(u8, rest, "::") orelse return rest;
    return rest[at + 2 ..];
}

//===========================================================================
// The record.
//===========================================================================

/// One line of the record: AREA, SYMBOL, VERDICT, EVIDENCE.
const Row = struct { area: []const u8, symbol: []const u8, verdict: []const u8, evidence: []const u8 };

/// One file of the record, with the contents its inputs generate.
const RecordFile = struct { path: []const u8, text: []const u8 };
const Generated = struct { files: []const RecordFile, rows: []const Row };

/// The bindings, keyed by symbol; null after printing to `messages` why a
/// line is malformed. A binding for a name upstream no longer declares is
/// the re-vendor failure this catches: it would otherwise sit unreferenced.
fn readBindings(t: Tree, names: []const []const u8, messages: *std.ArrayList(u8)) !?std.StringHashMap(Named) {
    var known: std.StringHashMap(void) = .init(t.a);
    for (names) |line| try known.put(field(line, 1), {});
    var bound: std.StringHashMap(Named) = .init(t.a);
    var bad = false;
    const text = (try t.read(bindings_path)) orelse {
        try messages.print(t.a, "{s} does not exist\n", .{bindings_path});
        return null;
    };
    var lines: Lines = .init(text);
    while (lines.next()) |line| {
        if (line.len == 0 or line[0] == '#') continue;
        const at = lines.number;
        if (mem.countScalar(u8, line, '\t') != 2) {
            try messages.print(t.a, "{s}:{d}: not SYMBOL<TAB>VERDICT<TAB>EVIDENCE\n", .{ bindings_path, at });
            bad = true;
        } else if (!oneOf(field(line, 1), "", &binding_verdicts)) {
            try messages.print(t.a, "{s}:{d}: {s} is not a verdict a binding can carry\n", .{ bindings_path, at, field(line, 1) });
            bad = true;
        } else if (!known.contains(field(line, 0))) {
            try messages.print(t.a, "{s}:{d}: upstream declares no {s}\n", .{ bindings_path, at, field(line, 0) });
            bad = true;
        } else if ((try bound.getOrPut(field(line, 0))).found_existing) {
            try messages.print(t.a, "{s}:{d}: {s} is bound twice\n", .{ bindings_path, at, field(line, 0) });
            bad = true;
        } else {
            try bound.put(field(line, 0), .{ .name = field(line, 1), .ref = field(line, 2) });
        }
    }
    if (!bad) return bound;
    try messages.print(t.a, "{s} is malformed\n", .{bindings_path});
    return null;
}

/// The record the harvest, the bindings and the tranche rules generate, one
/// file per area; null after printing to `messages` why it cannot be. A name
/// with a binding takes it; a name without one is a gap, and its evidence is
/// the tranche label the rules give it.
fn generate(t: Tree, names: []const []const u8, messages: *std.ArrayList(u8)) !?Generated {
    const bound = (try readBindings(t, names, messages)) orelse return null;
    var rows: std.ArrayList(Row) = .empty;
    var unclassified = false;
    for (names) |line| {
        const area = field(line, 0);
        const symbol = field(line, 1);
        const label = tranche(area, symbol) orelse {
            try messages.print(t.a, "{s}: no tranche rule in ci/coverage.zig claims it\n", .{symbol});
            unclassified = true;
            continue;
        };
        const binding = bound.get(symbol);
        try rows.append(t.a, .{
            .area = area,
            .symbol = symbol,
            .verdict = if (binding) |b| b.name else "GAP",
            .evidence = if (binding) |b| b.ref else label,
        });
    }
    if (unclassified) {
        try messages.print(t.a, "the tranche rules in ci/coverage.zig left a name unclassified\n", .{});
        return null;
    }
    var area_names: std.ArrayList([]const u8) = .empty;
    for (rows.items) |row| try area_names.append(t.a, row.area);
    var out: std.ArrayList(RecordFile) = .empty;
    for (unique(area_names.items)) |area| {
        var lines: std.ArrayList([]const u8) = .empty;
        for (rows.items) |row| {
            if (eql(row.area, area)) try lines.append(t.a, try t.a.print("{s}\t{s}\t{s}\t{s}\n", .{ row.area, row.symbol, row.verdict, row.evidence }));
        }
        // By symbol, so a diff of the record is a diff of what changed.
        mem.sort([]const u8, lines.items, {}, lessBySymbol);
        try out.append(t.a, .{
            .path = try t.a.print("ci/coverage/unbound_{s}.txt", .{try std.ascii.allocLowerString(t.a, area)}),
            .text = try mem.concat(t.a, u8, lines.items),
        });
    }
    return .{ .files = out.items, .rows = rows.items };
}

fn lessBySymbol(_: void, x: []const u8, y: []const u8) bool {
    return switch (mem.order(u8, field(x, 1), field(y, 1))) {
        .lt => true,
        .gt => false,
        .eq => mem.lessThan(u8, x, y),
    };
}

/// Whether every generated file matches what is checked in, printing to
/// `messages` what differs in each that does not.
fn compareRecord(t: Tree, generated: []const RecordFile, messages: *std.ArrayList(u8)) !bool {
    var stale = false;
    for (generated) |file| {
        const current = try t.read(file.path);
        if (current) |c| if (eql(c, file.text)) continue;
        stale = true;
        try messages.print(t.a, "{s} is not what ci/coverage.zig generates\n", .{file.path});
        try diffLines(t.a, current orelse "", file.text, messages);
    }
    if (stale) try messages.print(t.a,
        \\
        \\Edit ci/coverage/bindings.tsv or the tranche rules in ci/coverage.zig and run it with --record;
        \\the record itself is generated and hand edits to it are lost.
        \\
    , .{});
    return !stale;
}

/// Up to `max_diff` lines only one side has, `<` checked in and `>` generated.
fn diffLines(a: Allocator, old: []const u8, new: []const u8, messages: *std.ArrayList(u8)) !void {
    var old_set: std.StringHashMap(void) = .init(a);
    var new_set: std.StringHashMap(void) = .init(a);
    var it = Lines.init(old);
    while (it.next()) |l| try old_set.put(l, {});
    it = .init(new);
    while (it.next()) |l| try new_set.put(l, {});
    var printed: usize = 0;
    for ([_]struct { []const u8, *std.StringHashMap(void), []const u8 }{ .{ old, &new_set, "<" }, .{ new, &old_set, ">" } }) |side| {
        it = .init(side[0]);
        while (it.next()) |l| {
            if (side[1].contains(l) or printed == max_diff) continue;
            try messages.print(a, "{s} {s}\n", .{ side[2], l });
            printed += 1;
        }
    }
    if (printed == 0) try messages.print(a, "  the same lines in another order\n", .{});
}

/// `--record` writes the record; `--check-record` fails when what is
/// checked in differs from it.
fn record(t: Tree, r: *Report, check: bool) !u8 {
    var messages: std.ArrayList(u8) = .empty;
    const generated = (try generate(t, try harvestNames(t), &messages)) orelse {
        try r.err.writeAll(messages.items);
        return 1;
    };
    if (check) {
        if (!try compareRecord(t, generated.files, &messages)) {
            try r.err.writeAll(messages.items);
            return 1;
        }
        try r.out.writeAll("OK  the record matches its inputs\n");
        return 0;
    }
    for (generated.files) |file| try t.dir.writeFile(t.io, .{ .sub_path = file.path, .data = file.text });
    try countVerdicts(r.out, generated.rows);
    try r.out.print("  {s:<30} {d:>5}\n", .{ "public Recast/Detour names", generated.rows.len });
    return 0;
}

/// One line per verdict the rows carry, with its count, in byte order.
fn countVerdicts(out: *Io.Writer, rows: []const Row) !void {
    for (verdicts) |verdict| {
        var n: usize = 0;
        for (rows) |row| {
            if (eql(row.verdict, verdict)) n += 1;
        }
        var lower: [16]u8 = undefined;
        if (n > 0) try out.print("    {s:<28} {d:>5}\n", .{ std.ascii.lowerString(&lower, verdict), n });
    }
}

//===========================================================================
// The gate.
//===========================================================================

/// Runs every check, prints the summary, and returns the exit code.
fn gate(t: Tree, r: *Report, list: bool) !u8 {
    const ffi = try t.files("ffi", "", ".h");
    var syms: std.StringHashMap(void) = .init(t.a);
    for (ffi) |header| {
        for (try zrecastWords(t.a, header.ref)) |word| try syms.put(word, {});
    }
    const names = try harvestNames(t);
    var provable: std.StringHashMap([]const u8) = .init(t.a);
    for (try classify(t, names)) |p| try provable.put(p.name, p.ref);

    try checkClaimed(t, r);
    const rows = try readRecord(t, r);
    var messages: std.ArrayList(u8) = .empty;
    const fresh = if (try generate(t, names, &messages)) |g| try compareRecord(t, g.files, &messages) else false;
    if (!fresh) {
        try r.err.writeAll(messages.items);
        try r.fail("ci/coverage/unbound_*.txt is not what ci/coverage.zig --record generates", .{});
    }
    try checkComplete(t.a, r, names, rows);
    var problems: std.ArrayList([]const u8) = .empty;
    const refs = try checkEvidence(t.a, .{ .syms = syms, .provable = provable }, rows, &problems);
    if (problems.items.len > 0) {
        for (problems.items[0..@min(max_listed, problems.items.len)]) |p| try r.err.print("{s}\n", .{p});
        try r.fail("{d} unusable piece(s) of evidence", .{problems.items.len});
    }
    try checkMembers(t.a, r, ffi, refs.members.items);
    try checkUndefined(t, r, refs.undefined.items);
    try checkZig(t, r, refs.zig.items);
    const entry_points = try checkSurface(t, r, ffi);
    try summarize(t.a, r, rows, list, entry_points, names.len);
    if (r.fails > 0) {
        try r.err.print("\nFAIL  {d} problem(s)\n", .{r.fails});
        return 1;
    }
    try r.out.writeAll("\nOK  every public Recast/Detour name is accounted for\n");
    return 0;
}

/// A directory of upstream that no area claims is a place a capability can
/// hide where nothing here would look, so a re-vendor that adds one fails
/// until an area covers it.
fn checkClaimed(t: Tree, r: *Report) !void {
    var unclaimed: std.ArrayList([]const u8) = .empty;
    for (try t.list(recast, .directory, "", "", true)) |dir| {
        const claimed = for (areas) |area| {
            if (eql(area.dir, dir)) break true;
        } else false;
        if (!claimed) try unclaimed.append(t.a, dir);
    }
    if (unclaimed.items.len == 0) return;
    try r.listed(unclaimed.items, unclaimed.items.len);
    try r.fail("{d} upstream directory(ies) no area claims", .{unclaimed.items.len});
}

/// The checked-in record's rows, every malformed line reported.
fn readRecord(t: Tree, r: *Report) ![]const Row {
    var rows: std.ArrayList(Row) = .empty;
    var shape: std.ArrayList([]const u8) = .empty;
    for (try t.files("ci/coverage", "unbound_", ".txt")) |file| {
        var lines: Lines = .init(file.ref);
        while (lines.next()) |line| {
            if (line.len == 0 or line[0] == '#') continue;
            const at = lines.number;
            const row: Row = .{ .area = field(line, 0), .symbol = field(line, 1), .verdict = field(line, 2), .evidence = field(line, 3) };
            const problem = if (mem.countScalar(u8, line, '\t') != 3)
                try t.a.print("{s}:{d}: not four tab-separated fields", .{ file.name, at })
            else if (!oneOf(row.verdict, "", &verdicts))
                try t.a.print("{s}:{d}: unknown verdict {s}", .{ file.name, at, row.verdict })
            else if (!eql(row.verdict, "GAP") and row.evidence.len < 8)
                try t.a.print("{s}:{d}: {s} has no evidence", .{ file.name, at, row.symbol })
            else if (eql(row.verdict, "GAP") and !trancheLabel(row.evidence))
                try t.a.print("{s}:{d}: {s} is a GAP not assigned to a tranche", .{ file.name, at, row.symbol })
            else
                null;
            if (problem) |p| try shape.append(t.a, p) else try rows.append(t.a, row);
        }
    }
    if (shape.items.len > 0) {
        for (shape.items) |p| try r.err.print("{s}\n", .{p});
        try r.fail("{d} malformed line(s)", .{shape.items.len});
    }
    return rows.items;
}

/// Whether a GAP's evidence opens `tranche N[ab] - ` and names it.
fn trancheLabel(e: []const u8) bool {
    if (!mem.startsWith(u8, e, "tranche ") or e.len < 9 or e[8] < '1' or e[8] > '9') return false;
    var i: usize = 9;
    if (i < e.len and (e[i] == 'a' or e[i] == 'b')) i += 1;
    return mem.startsWith(u8, e[i..], " - ") and e.len >= i + 6;
}

/// No name without a verdict, no verdict for a name upstream no longer
/// declares, and no name with two.
fn checkComplete(a: Allocator, r: *Report, names: []const []const u8, rows: []const Row) !void {
    var pairs: std.ArrayList([]const u8) = .empty;
    for (rows) |row| try pairs.append(a, try a.print("{s}\t{s}", .{ row.area, row.symbol }));
    sortBytes(pairs.items);
    var duplicated: std.ArrayList([]const u8) = .empty;
    for (pairs.items, 0..) |p, i| {
        if (i > 0 and eql(p, pairs.items[i - 1]) and (i < 2 or !eql(p, pairs.items[i - 2]))) try duplicated.append(a, p);
    }
    const classified = unique(try a.dupe([]const u8, pairs.items));
    const missing = try without(a, names, classified);
    if (missing.len > 0) {
        try r.listed(missing, max_listed);
        try r.fail("{d} area/name pair(s) with no verdict", .{missing.len});
    }
    const stale = try without(a, classified, names);
    if (stale.len > 0) {
        try r.listed(stale, max_listed);
        try r.fail("{d} stale line(s) — upstream no longer declares these; delete them", .{stale.len});
    }
    if (duplicated.items.len > 0) {
        try r.listed(duplicated.items, duplicated.items.len);
        try r.fail("duplicate area/name pair(s) in the record", .{});
    }
}

/// What evidence is checked against.
const Context = struct {
    /// Every zrc/Zrc/ZRC_ identifier `ffi/*.h` declares or mentions.
    syms: std.StringHashMap(void),
    /// The INTERNAL evidence upstream's markers justify, by symbol.
    provable: std.StringHashMap([]const u8),
};

/// References in evidence that are resolved against files.
const Refs = struct {
    members: std.ArrayList(Named) = .empty,
    zig: std.ArrayList(Named) = .empty,
    undefined: std.ArrayList(Named) = .empty,
};

/// One rule per verdict. Appends every unusable piece of evidence to
/// `problems` and returns the references left to resolve.
fn checkEvidence(a: Allocator, cx: Context, rows: []const Row, problems: *std.ArrayList([]const u8)) !Refs {
    var refs: Refs = .{};
    for (rows) |row| {
        const name = row.symbol;
        const e = row.evidence;
        if (eql(row.verdict, "BOUND") or eql(row.verdict, "EXTENSION")) {
            // A struct member is written Type.member, and resolved below.
            var n: usize = 0;
            var rest = e;
            while (zrecastSymbol(rest)) |m| : (n += 1) {
                const token = rest[m.start..m.end];
                rest = rest[m.end..];
                if (mem.findScalar(u8, token, '.') != null) {
                    try refs.members.append(a, .{ .name = name, .ref = token });
                } else if (!cx.syms.contains(token)) {
                    try problems.append(a, try a.print("  {s}: {s} is not in ffi/*.h", .{ name, token }));
                }
            }
            if (n == 0) try problems.append(a, try a.print("  {s}: {s} names no zrecast symbol", .{ name, row.verdict }));
        } else if (eql(row.verdict, "LANGUAGE")) {
            var hit = false;
            var rest = e;
            while (facilityToken(rest)) |m| {
                const token = rest[m.start..m.end];
                rest = rest[m.end..];
                if (startsWithAny(token, &facilities)) {
                    hit = true;
                } else {
                    try problems.append(a, try a.print("  {s}: {s} is not a facility this file recognises", .{ name, token }));
                }
            }
            const worded = for (language_words) |w| {
                if (mem.find(u8, e, w) != null) break true;
            } else false;
            if (!hit and !worded) try problems.append(a, try a.print("  {s}: LANGUAGE names no facility", .{name}));
        } else if (eql(row.verdict, "ZIG")) {
            // Only the leading src/FILE.zig:decl is the reference; the rest is prose.
            if (zigRef(e)) |end| {
                try refs.zig.append(a, .{ .name = name, .ref = e[0..end] });
            } else try problems.append(a, try a.print("  {s}: ZIG evidence is not src/FILE.zig:decl", .{name}));
        } else if (eql(row.verdict, "UNDEFINED")) {
            if (headerLine(e)) |end| {
                try refs.undefined.append(a, .{ .name = name, .ref = e[0..end] });
            } else try problems.append(a, try a.print("  {s}: UNDEFINED evidence does not open with HEADER.h:LINE", .{name}));
        } else if (eql(row.verdict, "INTERNAL")) {
            // Recomputed rather than believed: the record cannot claim an
            // exclusion the headers do not support.
            if (cx.provable.get(name)) |p| {
                if (!eql(p, e)) try problems.append(a, try a.print("  {s}: INTERNAL evidence does not match what --classify computes", .{name}));
            } else try problems.append(a, try a.print("  {s}: INTERNAL, but --classify cannot justify it", .{name}));
        }
    }
    return refs;
}

/// A span of a string.
const Span = struct { start: usize, end: usize };

/// The leftmost `(zrc|Zrc|ZRC_)[A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?`.
fn zrecastSymbol(s: []const u8) ?Span {
    for (0..s.len) |p| {
        if (!startsWithAny(s[p..], &.{ "zrc", "Zrc", "ZRC_" })) continue;
        var j = wordEnd(s, p);
        if (j + 1 < s.len and s[j] == '.' and isIdentStart(s[j + 1])) j = wordEnd(s, j + 1);
        return .{ .start = p, .end = j };
    }
    return null;
}

/// The leftmost `@[A-Za-z]+` or `std\.[A-Za-z.]+`.
fn facilityToken(s: []const u8) ?Span {
    for (0..s.len) |p| {
        const dotted = mem.startsWith(u8, s[p..], "std.");
        if (s[p] != '@' and !dotted) continue;
        const from = if (dotted) p + 4 else p + 1;
        var j = from;
        while (j < s.len and (std.ascii.isAlphabetic(s[j]) or (dotted and s[j] == '.'))) j += 1;
        if (j > from) return .{ .start = p, .end = j };
    }
    return null;
}

/// The length of a leading `src/[DIR/]FILE.zig:decl[.member]`, or null.
fn zigRef(e: []const u8) ?usize {
    if (!mem.startsWith(u8, e, "src/")) return null;
    var i = wordEnd(e, 4);
    if (i == 4) return null;
    if (i < e.len and e[i] == '/') {
        const file = i + 1;
        i = wordEnd(e, file);
        if (i == file) return null;
    }
    if (!mem.startsWith(u8, e[i..], ".zig:")) return null;
    i += 5;
    if (i == e.len or !isIdentStart(e[i])) return null;
    i = wordEnd(e, i);
    if (i + 1 < e.len and e[i] == '.' and isIdentStart(e[i + 1])) i = wordEnd(e, i + 1);
    return i;
}

/// The length of a leading `HEADER.h:LINE` followed by a space, or null.
fn headerLine(e: []const u8) ?usize {
    var i = wordEnd(e, 0);
    if (i == 0 or !mem.startsWith(u8, e[i..], ".h:")) return null;
    i += 3;
    const digits = i;
    while (i < e.len and std.ascii.isDigit(e[i])) i += 1;
    if (i == digits or i == e.len or e[i] != ' ') return null;
    return i;
}

/// Type.member evidence: the member has to sit inside that type's braces.
fn checkMembers(a: Allocator, r: *Report, ffi: []const Named, refs: []const Named) !void {
    var bodies: std.StringHashMap([]const []const u8) = .init(a);
    var missing: usize = 0;
    for (refs) |ref| {
        const dot = mem.findScalar(u8, ref.ref, '.') orelse continue;
        const ty = ref.ref[0..dot];
        const member = ref.ref[dot + 1 ..];
        const body = try bodies.getOrPut(ty);
        if (!body.found_existing) body.value_ptr.* = try bodyOf(a, ffi, ty);
        const found = for (body.value_ptr.*) |l| {
            if (wordIn(l, member)) break true;
        } else false;
        if (found) continue;
        try r.err.print("  {s}: ffi/*.h declares no {s} inside {s}\n", .{ ref.name, member, ty });
        missing += 1;
    }
    if (missing > 0) try r.fail("{d} member reference(s) pointing at nothing", .{missing});
}

/// The lines from each `typedef struct|enum|union T` through its `} T;`,
/// the headers read as one stream. A line closing `T` counts even when no
/// opening line came before it.
fn bodyOf(a: Allocator, ffi: []const Named, ty: []const u8) ![]const []const u8 {
    var lines: std.ArrayList([]const u8) = .empty;
    var inside = false;
    for (ffi) |header| {
        var it: Lines = .init(header.ref);
        while (it.next()) |l| {
            const closing = mem.startsWith(u8, l, "} ") and mem.startsWith(u8, l[2..], ty) and
                mem.startsWith(u8, l[2 + ty.len ..], ";");
            var from: usize = 0;
            while (mem.findPos(u8, l, from, "typedef ")) |at| : (from = at + 1) {
                const rest = l[at + "typedef ".len ..];
                for ([_][]const u8{ "struct ", "enum ", "union " }) |kw| {
                    if (mem.startsWith(u8, rest, kw) and mem.startsWith(u8, rest[kw.len..], ty)) inside = true;
                }
            }
            if (inside or closing) try lines.append(a, l);
            if (closing) inside = false;
        }
    }
    return lines.items;
}

/// UNDEFINED evidence: the header declares the name at that line, and no
/// vendored source defines it. A re-vendor that supplies the definition
/// turns the name into a binding to write, and fails here until it is.
fn checkUndefined(t: Tree, r: *Report, refs: []const Named) !void {
    var sources: std.ArrayList(Named) = .empty;
    for (try t.list(recast, .directory, "", "", false)) |dir| {
        try sources.appendSlice(t.a, try t.files(try t.a.print("{s}/{s}/Source", .{ recast, dir }), "", ".cpp"));
    }
    var problems: usize = 0;
    for (refs) |ref| {
        const after = if (mem.findLast(u8, ref.name, "::")) |at| ref.name[at + 2 ..] else ref.name;
        const bare = after[0 .. mem.findScalar(u8, after, '/') orelse after.len];
        const colon = mem.findScalar(u8, ref.ref, ':') orelse ref.ref.len;
        const header = try t.a.print("{s}/DebugUtils/Include/{s}", .{ recast, ref.ref[0..colon] });
        const number = ref.ref[if (mem.findScalarLast(u8, ref.ref, ':')) |at| at + 1 else 0..];
        const text = (try t.read(header)) orelse {
            try r.err.print("  {s}: {s} does not exist\n", .{ ref.name, header });
            problems += 1;
            continue;
        };
        if (!declaresAt(text, std.fmt.parseInt(usize, number, 10) catch 0, bare)) {
            try r.err.print("  {s}: {s}:{s} does not declare it\n", .{ ref.name, header, number });
            problems += 1;
        }
        const defined = for (sources.items) |source| {
            if (callsIn(source.ref, bare)) break true;
        } else false;
        if (defined) {
            try r.err.print("  {s}: a vendored source defines it after all; bind it\n", .{ref.name});
            problems += 1;
        }
    }
    if (problems > 0) try r.fail("{d} UNDEFINED line(s) that do not hold", .{problems});
}

/// Whether line `number` of `text` holds `NAME(` after a non-word character.
fn declaresAt(text: []const u8, number: usize, name: []const u8) bool {
    var lines: Lines = .init(text);
    while (lines.next()) |l| {
        if (lines.number != number) continue;
        const needle_end = name.len + 1;
        var from: usize = 1;
        while (mem.findPos(u8, l, from, name)) |at| : (from = at + 1) {
            if (!isWord(l[at - 1]) and at + needle_end <= l.len and l[at + name.len] == '(') return true;
        }
        return false;
    }
    return false;
}

/// Whether `text` holds `name` as a word followed, after any space, by `(`.
fn callsIn(text: []const u8, name: []const u8) bool {
    var from: usize = 0;
    while (mem.findPos(u8, text, from, name)) |at| : (from = at + 1) {
        if (at > 0 and isWord(text[at - 1])) continue;
        var j = at + name.len;
        while (j < text.len and isSpace(text[j])) j += 1;
        if (j < text.len and text[j] == '(') return true;
    }
    return false;
}

/// ZIG evidence: the declaration has to exist.
fn checkZig(t: Tree, r: *Report, refs: []const Named) !void {
    var missing: usize = 0;
    for (refs) |ref| {
        const colon = mem.findScalar(u8, ref.ref, ':') orelse continue;
        const file = ref.ref[0..colon];
        const decl = ref.ref[colon + 1 ..];
        const text = (try t.read(file)) orelse {
            try r.err.print("  {s}: {s} does not exist\n", .{ ref.name, file });
            missing += 1;
            continue;
        };
        if (zigDeclares(text, decl)) continue;
        if (mem.findScalar(u8, decl, '.')) |dot| {
            try r.err.print("  {s}: {s} declares no {s} inside {s}\n", .{ ref.name, file, decl[dot + 1 ..], decl[0..dot] });
        } else try r.err.print("  {s}: {s} declares no {s}\n", .{ ref.name, file, decl });
        missing += 1;
    }
    if (missing > 0) try r.fail("{d} ZIG line(s) pointing at nothing", .{missing});
}

/// Whether `text` declares `decl` publicly, or `Type.member` inside the
/// braces of `pub const Type = struct|union|enum`.
fn zigDeclares(text: []const u8, decl: []const u8) bool {
    var lines: Lines = .init(text);
    const dot = mem.findScalar(u8, decl, '.') orelse {
        while (lines.next()) |l| {
            const t = trimStart(l);
            if (mem.startsWith(u8, t, "pub ") and declares(t[4..], decl, false)) return true;
        }
        return false;
    };
    const ty = decl[0..dot];
    const member = decl[dot + 1 ..];
    var inside = false;
    var seen = false;
    var found = false;
    var opens: usize = 0;
    var closes: usize = 0;
    while (lines.next()) |l| {
        if (typeHeader(l, ty)) {
            inside = true;
            opens = 0;
            closes = 0;
        }
        if (!inside) continue;
        opens += mem.countScalar(u8, l, '{');
        closes += mem.countScalar(u8, l, '}');
        const t = trimStart(l);
        if (declares(if (mem.startsWith(u8, t, "pub ")) t[4..] else t, member, true)) found = true;
        if (closes >= opens and seen) inside = false;
        seen = true;
    }
    return found;
}

/// Whether `l` opens `fn|const|inline fn|var NAME`, followed by a word
/// boundary, or with `member` by one of ` (:=`.
fn declares(l: []const u8, name: []const u8, member: bool) bool {
    for ([_][]const u8{ "fn ", "const ", "inline fn ", "var " }) |kw| {
        if (!mem.startsWith(u8, l, kw) or !mem.startsWith(u8, l[kw.len..], name)) continue;
        const end = kw.len + name.len;
        if (member) {
            if (end < l.len and mem.findScalar(u8, " (:=", l[end]) != null) return true;
        } else if (end == l.len or !isWord(l[end])) return true;
    }
    return false;
}

/// Whether `l` opens `pub const T = [extern |packed ](struct|union|enum)`.
fn typeHeader(l: []const u8, ty: []const u8) bool {
    if (!mem.startsWith(u8, l, "pub const ") or !mem.startsWith(u8, l[10..], ty)) return false;
    var rest = l[10 + ty.len ..];
    if (!mem.startsWith(u8, rest, " = ")) return false;
    rest = rest[3..];
    if (startsWithAny(rest, &.{ "extern ", "packed " })) rest = rest[7..];
    return startsWithAny(rest, &.{ "struct", "union", "enum" });
}

/// The Zig surface. An entry point declared in C and never wrapped is
/// unreachable for a Zig host, and nothing above can see it. `src/c.zig` is
/// the extern layer, and a declaration is not use; tests do not count
/// either. Returns the number of entry points.
fn checkSurface(t: Tree, r: *Report, ffi: []const Named) !usize {
    var entries: std.ArrayList([]const u8) = .empty;
    for (ffi) |header| try entries.appendSlice(t.a, try entryPoints(t.a, header.ref));
    const entry_points = unique(entries.items);
    var calls: std.ArrayList([]const u8) = .empty;
    var src: ?Io.Dir = t.dir.openDir(t.io, "src", .{ .iterate = true }) catch |err| switch (err) {
        error.FileNotFound => null,
        else => |e| return e,
    };
    if (src) |*dir| {
        defer dir.close(t.io);
        var walker = try dir.walk(t.a);
        defer walker.deinit();
        while (try walker.next(t.io)) |entry| {
            if (entry.kind != .file or !mem.endsWith(u8, entry.basename, ".zig")) continue;
            if (eql(entry.basename, "c.zig") or mem.endsWith(u8, entry.basename, "_test.zig")) continue;
            const text = try entry.dir.readFileAlloc(t.io, entry.basename, t.a, .limited(max_file_bytes));
            var from: usize = 0;
            while (mem.findPos(u8, text, from, "zrc")) |at| {
                from = wordEnd(text, at);
                try calls.append(t.a, text[at..from]);
            }
        }
    }
    var called: std.StringHashMap(void) = .init(t.a);
    for (calls.items) |c| try called.put(c, {});
    const excused = try exceptions(t, r);
    var excused_set: std.StringHashMap(void) = .init(t.a);
    for (excused) |e| try excused_set.put(e, {});
    var unwrapped: std.ArrayList([]const u8) = .empty;
    var stranded: std.ArrayList([]const u8) = .empty;
    for (entry_points) |e| {
        if (called.contains(e)) continue;
        try unwrapped.append(t.a, e);
        if (!excused_set.contains(e)) try stranded.append(t.a, e);
    }
    if (stranded.items.len > 0) {
        try r.listed(stranded.items, stranded.items.len);
        try r.fail("{d} entry point(s) with no Zig caller", .{stranded.items.len});
    }
    const excess = try without(t.a, excused, unwrapped.items);
    if (excess.len > 0) {
        try r.listed(excess, excess.len);
        try r.fail("{d} excused entry point(s) that Zig does call, or that no longer exist", .{excess.len});
    }
    return entry_points.len;
}

/// The names `ci/coverage/zig_surface_exceptions.txt` excuses, in byte order, each
/// line NAME<TAB>reason; malformed lines are reported.
fn exceptions(t: Tree, r: *Report) ![]const []const u8 {
    const text = (try t.read(exceptions_path)) orelse return &.{};
    var names: std.ArrayList([]const u8) = .empty;
    var malformed: usize = 0;
    var lines: Lines = .init(text);
    while (lines.next()) |line| {
        if (line.len == 0 or line[0] == '#') continue;
        if (mem.countScalar(u8, line, '\t') != 1) {
            try r.err.print("  {s}: not NAME<TAB>reason\n", .{field(line, 0)});
            malformed += 1;
        } else if (field(line, 1).len < 10) {
            try r.err.print("  {s}: no reason given\n", .{field(line, 0)});
            malformed += 1;
        } else try names.append(t.a, field(line, 0));
    }
    if (malformed > 0) try r.fail("{d} malformed exception line(s)", .{malformed});
    return unique(names.items);
}

/// Every `zrc...` identifier directly followed by `(`.
fn entryPoints(a: Allocator, text: []const u8) ![]const []const u8 {
    var out: std.ArrayList([]const u8) = .empty;
    var from: usize = 0;
    while (mem.findPos(u8, text, from, "zrc")) |at| {
        const end = wordEnd(text, at);
        if (end < text.len and text[end] == '(') {
            try out.append(a, text[at..end]);
            from = end + 1;
        } else from = at + 1;
    }
    return out.items;
}

/// Every identifier opening `zrc`, `Zrc` or `ZRC_` at a word boundary.
fn zrecastWords(a: Allocator, text: []const u8) ![]const []const u8 {
    var out: std.ArrayList([]const u8) = .empty;
    var p: usize = 0;
    while (p < text.len) {
        if ((p == 0 or !isWord(text[p - 1])) and startsWithAny(text[p..], &.{ "zrc", "Zrc", "ZRC_" })) {
            const end = wordEnd(text, p);
            try out.append(a, text[p..end]);
            p = end;
        } else p += 1;
    }
    return out.items;
}

/// The open gaps when asked, then the counts.
fn summarize(a: Allocator, r: *Report, rows: []const Row, list: bool, entry_points: usize, names: usize) !void {
    var open: std.ArrayList([]const u8) = .empty;
    for (rows) |row| {
        if (eql(row.verdict, "GAP")) try open.append(a, try a.print("  {s}\t{s}", .{ row.area, row.symbol }));
    }
    sortBytes(open.items);
    if (list and open.items.len > 0) {
        try r.out.writeAll("gaps\n");
        for (open.items) |line| try r.err.print("{s}\n", .{line});
        try r.out.writeAll("\n");
    }
    try r.out.writeAll("zrecast coverage\n");
    try r.out.print("  {s:<30} {d:>5}\n", .{ "entry points exported", entry_points });
    try r.out.print("  {s:<30} {d:>5}\n", .{ "public Recast/Detour names", names });
    try countVerdicts(r.out, rows);
    if (open.items.len > 0) try r.fail("{d} gap(s) — run with --list to see them", .{open.items.len});
}

/// Names with a verdict, by verdict and area: the only completeness number
/// the record supports. A name nobody has judged is counted and named as
/// such, never folded into a percentage. With `pattern`, the names of every
/// area it matches, each with its verdict.
fn summary(t: Tree, out: *Io.Writer, pattern: ?[]const u8) !void {
    var record_rows: std.ArrayList(Row) = .empty;
    for (try t.files("ci/coverage", "unbound_", ".txt")) |file| {
        var lines: Lines = .init(file.ref);
        while (lines.next()) |l| {
            if (l.len == 0 or l[0] == '#') continue;
            try record_rows.append(t.a, .{ .area = field(l, 0), .symbol = field(l, 1), .verdict = field(l, 2), .evidence = "" });
        }
    }
    try out.writeAll("zrecast coverage of Recast and Detour\n\n");
    const columns = .{ "area", "names", "bound", "ext", "lang", "internal", "undef", "GAP" };
    try out.print("  {s:<22} {s:>6} {s:>6} {s:>6} {s:>6} {s:>6} {s:>6} {s:>6}\n", columns);
    var total: [7]usize = @splat(0);
    var judged: usize = 0;
    var audit: usize = 0;
    for (areas) |area| {
        const harvested = try harvestArea(t, area);
        audit += harvested.skipped.len;
        const names = try symbolsOf(t.a, harvested.decls);
        var pairs: std.ArrayList([]const u8) = .empty;
        for (record_rows.items) |row| {
            if (eql(row.area, area.name)) try pairs.append(t.a, try t.a.print("{s}\t{s}", .{ row.symbol, row.verdict }));
        }
        const verdict_pairs = unique(pairs.items);
        var c: [7]usize = @splat(0);
        c[0] = names.len;
        for (verdict_pairs) |p| {
            const v = field(p, 1);
            for ([_][]const []const u8{ &.{"BOUND"}, &.{"EXTENSION"}, &.{ "LANGUAGE", "ZIG" }, &.{"INTERNAL"}, &.{"UNDEFINED"}, &.{"GAP"} }, 1..) |group, i| {
                if (oneOf(v, "", group)) c[i] += 1;
            }
        }
        for (c[1..]) |n| judged += n;
        for (&total, c) |*sum, n| sum.* += n;
        try out.print("  {s:<22} {d:>6} {d:>6} {d:>6} {d:>6} {d:>6} {d:>6} {d:>6}\n", .{ area.name, c[0], c[1], c[2], c[3], c[4], c[5], c[6] });
        if (pattern) |p| if (std.ascii.findIgnoreCase(area.name, p) != null) try listArea(out, names, verdict_pairs);
    }
    const collapsed_count = (try collapsed(t)).len;
    try out.print("  {s:<22} {d:>6} {d:>6} {d:>6} {d:>6} {d:>6} {d:>6} {d:>6}\n", .{ "TOTAL", total[0], total[1], total[2], total[3], total[4], total[5], total[6] });
    if (total[0] > judged) try out.print("\n  {d} name(s) with no verdict at all — run ci/coverage.zig\n", .{total[0] - judged});
    try out.print(
        \\
        \\  blind spots
        \\    preprocessor macros are not names here (rcLikely, the assert macros)
        \\    operator overloads carry no name a C ABI can spell
        \\    {d} symbol(s) cover more than one declaration — ci/coverage.zig --collapsed
        \\    {d} line(s) the harvester did not parse — ci/coverage.zig --audit
        \\
    , .{ collapsed_count, audit });
}

/// One area's names, each with every verdict the record gives it, or GAP.
fn listArea(out: *Io.Writer, names: []const []const u8, verdict_pairs: []const []const u8) !void {
    for (names) |name| {
        var any = false;
        for (verdict_pairs) |p| {
            if (!eql(field(p, 0), name)) continue;
            try out.print("      {s}\t{s}\n", .{ name, field(p, 1) });
            any = true;
        }
        if (!any) try out.print("      {s}\tGAP\n", .{name});
    }
}

//===========================================================================
// Strings.
//===========================================================================

fn eql(x: []const u8, y: []const u8) bool {
    return mem.eql(u8, x, y);
}

/// Whether `s` is `prefix` followed by exactly one of `rest`.
fn oneOf(s: []const u8, prefix: []const u8, rest: []const []const u8) bool {
    if (!mem.startsWith(u8, s, prefix)) return false;
    for (rest) |r| {
        if (eql(s[prefix.len..], r)) return true;
    }
    return false;
}

fn startsWithAny(s: []const u8, prefixes: []const []const u8) bool {
    for (prefixes) |p| {
        if (mem.startsWith(u8, s, p)) return true;
    }
    return false;
}

/// Field `n` of a tab-separated line; empty past the last.
fn field(line: []const u8, n: usize) []const u8 {
    var fields = mem.splitScalar(u8, line, '\t');
    for (0..n) |_| _ = fields.next() orelse return "";
    return fields.next() orelse "";
}

/// `s` up to the first `c`.
fn cut(s: []const u8, c: u8) []const u8 {
    return s[0 .. mem.findScalar(u8, s, c) orelse s.len];
}

/// `s` up to the first `needle`.
fn cutText(s: []const u8, needle: []const u8) []const u8 {
    return s[0 .. mem.find(u8, s, needle) orelse s.len];
}

/// The end of the run of word characters starting at `from`.
fn wordEnd(s: []const u8, from: usize) usize {
    var j = from;
    while (j < s.len and isWord(s[j])) j += 1;
    return j;
}

fn alnum(s: []const u8) bool {
    for (s) |c| {
        if (!std.ascii.isAlphanumeric(c)) return false;
    }
    return true;
}

fn lowerAlpha(s: []const u8) bool {
    for (s) |c| {
        if (!std.ascii.isLower(c)) return false;
    }
    return true;
}

fn hasWord(s: []const u8) bool {
    for (s) |c| {
        if (isWord(c)) return true;
    }
    return false;
}

fn isWord(c: u8) bool {
    return std.ascii.isAlphanumeric(c) or c == '_';
}

fn isIdentStart(c: u8) bool {
    return std.ascii.isAlphabetic(c) or c == '_';
}

fn isSpace(c: u8) bool {
    return std.ascii.isWhitespace(c);
}

fn trim(s: []const u8) []const u8 {
    return mem.trim(u8, s, &std.ascii.whitespace);
}

fn trimStart(s: []const u8) []const u8 {
    return mem.trimStart(u8, s, &std.ascii.whitespace);
}

fn trimEnd(s: []const u8) []const u8 {
    return mem.trimEnd(u8, s, &std.ascii.whitespace);
}

fn sortBytes(items: [][]const u8) void {
    mem.sort([]const u8, items, {}, lessBytes);
}

fn lessBytes(_: void, x: []const u8, y: []const u8) bool {
    return mem.lessThan(u8, x, y);
}

/// `items` sorted, without repeats.
fn unique(items: [][]const u8) [][]const u8 {
    sortBytes(items);
    var n: usize = 0;
    for (items) |item| {
        if (n > 0 and eql(items[n - 1], item)) continue;
        items[n] = item;
        n += 1;
    }
    return items[0..n];
}

/// The lines of `xs` that `ys` does not hold, in the order of `xs`.
fn without(a: Allocator, xs: []const []const u8, ys: []const []const u8) ![]const []const u8 {
    var set: std.StringHashMap(void) = .init(a);
    for (ys) |y| try set.put(y, {});
    var out: std.ArrayList([]const u8) = .empty;
    for (xs) |x| {
        if (!set.contains(x)) try out.append(a, x);
    }
    return out.items;
}

//===========================================================================
// Tests.
//===========================================================================

const testing = std.testing;

/// The public symbols the scanner reads from `text`, one per line.
fn scanned(a: Allocator, text: []const u8) ![]const u8 {
    var decls: std.ArrayList(Decl) = .empty;
    var scanner: Scanner = .{ .a = a, .decls = &decls };
    try scanner.scan("h.h", text);
    var out: std.ArrayList(u8) = .empty;
    for (try symbolsOf(a, decls.items)) |s| try out.print(a, "{s}\n", .{s});
    return out.items;
}

/// A report whose two streams are kept in memory.
const Captured = struct {
    out: Io.Writer.Allocating,
    err: Io.Writer.Allocating,

    fn init(a: Allocator) Captured {
        return .{ .out = .init(a), .err = .init(a) };
    }

    fn report(c: *Captured) Report {
        return .{ .out = &c.out.writer, .err = &c.err.writer };
    }
};

/// Writes `files`, each a path and its contents, under `dir`.
fn plant(dir: Io.Dir, files: []const [2][]const u8) !void {
    for (files) |f| {
        if (std.fs.path.dirname(f[0])) |parent| try dir.createDirPath(testing.io, parent);
        try dir.writeFile(testing.io, .{ .sub_path = f[0], .data = f[1] });
    }
}

test "an arity and parameter types keep overloads apart" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    try testing.expectEqualStrings(
        \\func:-::dtDot/2
        \\func:-::dtSwapEndian/1(float*)
        \\func:-::dtSwapEndian/1(unsigned short*)
        \\func:-::rcGet/1
        \\func:dtTile::get/1(int)
        \\func:dtTile::get/1(int)const
        \\func:dtTile::reset/0
        \\type:-::dtTile
        \\
    , try scanned(arena.allocator(),
        \\float dtDot(const float* v1, const float* v2);
        \\inline void dtSwapEndian(unsigned short* v);
        \\inline void dtSwapEndian(float* v);
        \\int rcGet(int i = 0);
        \\struct dtTile
        \\{
        \\    int get(int i) const;
        \\    int get(int i);
        \\    void reset();
        \\};
        \\
    ));
}

test "fields, enum constants, static constants and typedefs are names" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    try testing.expectEqualStrings(
        \\const:-::DT_PARTIAL
        \\const:dtParams::MAX_SEGS
        \\enum:dtFlags::DT_FIRST
        \\enum:dtFlags::DT_SECOND
        \\field:dtParams::ch
        \\field:dtParams::cs
        \\field:dtParams::verts
        \\type:-::dtAllocFunc
        \\type:-::dtFlags
        \\type:-::dtParams
        \\type:-::dtPolyRef
        \\
    , try scanned(arena.allocator(),
        \\static const unsigned int DT_PARTIAL = 1 << 6;
        \\typedef unsigned int dtPolyRef;
        \\typedef void* (dtAllocFunc)(size_t size, int hint);
        \\enum dtFlags
        \\{
        \\    DT_FIRST = 1,
        \\    DT_SECOND,
        \\};
        \\struct dtParams
        \\{
        \\    float cs, ch;
        \\    int verts[3];
        \\    static const int MAX_SEGS = 8;
        \\};
        \\
    ));
}

test "private and protected members are not public names" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const header =
        \\class dtQuery
        \\{
        \\public:
        \\    void find(int ref);
        \\    struct Result { int count; };
        \\private:
        \\    void helper();
        \\    struct Hidden { int inner; };
        \\protected:
        \\    int m_state;
        \\};
        \\
    ;
    try testing.expectEqualStrings(
        \\func:dtQuery::find/1
        \\type:-::dtQuery
        \\type:dtQuery::Result
        \\
    , try scanned(a, header));
    var decls: std.ArrayList(Decl) = .empty;
    var scanner: Scanner = .{ .a = a, .decls = &decls };
    try scanner.scan("h.h", header);
    var hidden: std.ArrayList(u8) = .empty;
    for (decls.items) |d| {
        if (!d.public) try hidden.print(a, "{s} ", .{d.name});
    }
    try testing.expectEqualStrings("helper Hidden m_state ", hidden.items);
}

test "comments, directives and continued lines are read as a compiler reads them" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const header =
        \\/* struct dtHidden
        \\   { */
        \\#define rcAssert(x) \
        \\    do { if (!(x)) abort(); } while (0)
        \\int dtLong(int a,
        \\           int b); // dtTrailing(int c);
        \\static const unsigned int DT_MAGIC = 'D'<<24 | 'N'<<16;
        \\template<class T> void rcSwap(T& a, T& b);
        \\
    ;
    try testing.expectEqualStrings(
        \\const:-::DT_MAGIC
        \\func:-::dtLong/2
        \\func:-::rcSwap/2
        \\
    , try scanned(a, header));
    var decls: std.ArrayList(Decl) = .empty;
    var scanner: Scanner = .{ .a = a, .decls = &decls };
    try scanner.scan("h.h", header);
    try testing.expectEqual(@as(usize, 5), decls.items[0].line);
    try testing.expectEqualStrings("(int,int)", decls.items[0].disc);
    try testing.expectEqualStrings("(T&,T&)", decls.items[2].disc);
}

test "upstream's internal markers mark a group and a single declaration" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var marks: std.ArrayList([]const u8) = .empty;
    try markIn(a, "N.h",
        \\/// @name Encoding and Decoding
        \\/// These functions are generally meant for internal use only.
        \\/// @{
        \\dtPolyRef encodePolyId(unsigned int salt) const;
        \\/// @}
        \\/// @note This method is generally meant for internal use only.
        \\void decode(int x);
        \\void open(int y);
        \\
    , &marks);
    try testing.expectEqual(@as(usize, 2), marks.items.len);
    try testing.expectEqualStrings("encodePolyId\tN.h:2\tgroup", marks.items[0]);
    try testing.expectEqualStrings("decode\tN.h:6\tdecl", marks.items[1]);
    try testing.expectEqualStrings("decodePolyId", bareName("func:dtNavMesh::decodePolyId/4"));
    try testing.expectEqualStrings("get/1(int)", bareName("func:dtTile::get/1(int)"));
}

test "the first tranche rule that matches wins" {
    try testing.expectEqualStrings(t3a, tranche("Detour", "field:dtNavMeshCreateParams::userId").?);
    try testing.expectEqualStrings(t4, tranche("Detour", "field:dtNavMeshCreateParams::cs").?);
    try testing.expectEqualStrings(t7, tranche("Detour", "func:-::dtMathAtan2f/2").?);
    try testing.expectEqualStrings(t2, tranche("Detour", "func:-::dtSwapEndian/1(float*)").?);
    try testing.expectEqualStrings(t3b, tranche("Recast", "func:-::rcGetCon/2").?);
    try testing.expect(tranche("Elsewhere", "type:-::thing") == null);
    try testing.expectEqualStrings("dtSwapEndian", nameOf("func:-::dtSwapEndian/1(float*)"));
    try testing.expectEqualStrings("dtNavMesh", ownerOf("func:dtNavMesh::init/1"));
}

/// The problems `checkEvidence` finds in `rows`, one per line.
fn evidenceProblems(a: Allocator, cx: Context, rows: []const Row, refs: *Refs) ![]const u8 {
    var problems: std.ArrayList([]const u8) = .empty;
    refs.* = try checkEvidence(a, cx, rows, &problems);
    var out: std.ArrayList(u8) = .empty;
    for (problems.items) |p| try out.print(a, "{s}\n", .{p});
    return out.items;
}

test "each verdict's evidence is held to its own rule" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var cx: Context = .{ .syms = .init(a), .provable = .init(a) };
    try cx.syms.put("zrcFoo", {});
    try cx.syms.put("ZRC_BAR", {});
    try cx.provable.put("p", "upstream N.h:2 (group)");
    const rows = [_]Row{
        .{ .area = "A", .symbol = "b1", .verdict = "BOUND", .evidence = "zrcFoo mirrors it" },
        .{ .area = "A", .symbol = "b2", .verdict = "BOUND", .evidence = "ZRC_BAR and zrcMissing" },
        .{ .area = "A", .symbol = "b3", .verdict = "EXTENSION", .evidence = "a sentence naming nothing" },
        .{ .area = "A", .symbol = "b4", .verdict = "BOUND", .evidence = "ZrcSpan.reg holds it" },
        .{ .area = "A", .symbol = "l1", .verdict = "LANGUAGE", .evidence = "std.math.atan2" },
        .{ .area = "A", .symbol = "l2", .verdict = "LANGUAGE", .evidence = "@reduce over a vector" },
        .{ .area = "A", .symbol = "l3", .verdict = "LANGUAGE", .evidence = "the discard, `_ = value;`" },
        .{ .area = "A", .symbol = "z1", .verdict = "ZIG", .evidence = "src/vec.zig:scalar.abs - prose after it" },
        .{ .area = "A", .symbol = "z2", .verdict = "ZIG", .evidence = "vec.zig:abs" },
        .{ .area = "A", .symbol = "u1", .verdict = "UNDEFINED", .evidence = "RecastDebugDraw.h:34 declares it" },
        .{ .area = "A", .symbol = "u2", .verdict = "UNDEFINED", .evidence = "declared somewhere" },
        .{ .area = "A", .symbol = "p", .verdict = "INTERNAL", .evidence = "upstream N.h:2 (group)" },
        .{ .area = "B", .symbol = "p", .verdict = "INTERNAL", .evidence = "upstream N.h:9 (group)" },
        .{ .area = "A", .symbol = "q", .verdict = "INTERNAL", .evidence = "upstream N.h:2 (group)" },
    };
    var refs: Refs = .{};
    try testing.expectEqualStrings(
        \\  b2: zrcMissing is not in ffi/*.h
        \\  b3: EXTENSION names no zrecast symbol
        \\  l2: @reduce is not a facility this file recognises
        \\  l2: LANGUAGE names no facility
        \\  z2: ZIG evidence is not src/FILE.zig:decl
        \\  u2: UNDEFINED evidence does not open with HEADER.h:LINE
        \\  p: INTERNAL evidence does not match what --classify computes
        \\  q: INTERNAL, but --classify cannot justify it
        \\
    , try evidenceProblems(a, cx, &rows, &refs));
    try testing.expectEqualStrings("ZrcSpan.reg", refs.members.items[0].ref);
    try testing.expectEqualStrings("src/vec.zig:scalar.abs", refs.zig.items[0].ref);
    try testing.expectEqualStrings("RecastDebugDraw.h:34", refs.undefined.items[0].ref);
}

test "a member, a Zig declaration and an undefined name each resolve against the tree" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    try plant(tmp.dir, &.{
        .{ "ffi/zrecast.h", "typedef struct ZrcSpan\n{\n    unsigned int reg;\n} ZrcSpan;\nint area;\n" },
        .{ "src/vec.zig", "pub const scalar = struct {\n    pub fn abs(x: f32) f32 {\n        return x;\n    }\n};\npub fn dot() void {}\n" },
        .{ "libs/recastnavigation/DebugUtils/Include/D.h", "// D.h\nvoid duDraw(int x);\nvoid duDefined(int x);\n" },
        .{ "libs/recastnavigation/DebugUtils/Source/D.cpp", "void duDefined (int x) {}\n" },
    });
    const t: Tree = .{ .io = testing.io, .dir = tmp.dir, .a = a };
    var c: Captured = .init(a);
    var r = c.report();
    const ffi = try t.files("ffi", "", ".h");
    try checkMembers(a, &r, ffi, &.{ .{ .name = "m1", .ref = "ZrcSpan.reg" }, .{ .name = "m2", .ref = "ZrcSpan.area" } });
    try checkZig(t, &r, &.{
        .{ .name = "z1", .ref = "src/vec.zig:scalar.abs" },
        .{ .name = "z2", .ref = "src/vec.zig:dot" },
        .{ .name = "z3", .ref = "src/vec.zig:scalar.sqrt" },
        .{ .name = "z4", .ref = "src/none.zig:x" },
    });
    try checkUndefined(t, &r, &.{
        .{ .name = "func:-::duDraw/1", .ref = "D.h:2" },
        .{ .name = "func:-::duDraw/1", .ref = "D.h:1" },
        .{ .name = "func:-::duDefined/1", .ref = "D.h:3" },
        .{ .name = "func:-::duGone/1", .ref = "Gone.h:1" },
    });
    try testing.expectEqualStrings(
        \\  m2: ffi/*.h declares no area inside ZrcSpan
        \\1 member reference(s) pointing at nothing
        \\  z3: src/vec.zig declares no sqrt inside scalar
        \\  z4: src/none.zig does not exist
        \\2 ZIG line(s) pointing at nothing
        \\  func:-::duDraw/1: libs/recastnavigation/DebugUtils/Include/D.h:1 does not declare it
        \\  func:-::duDefined/1: a vendored source defines it after all; bind it
        \\  func:-::duGone/1: libs/recastnavigation/DebugUtils/Include/Gone.h does not exist
        \\3 UNDEFINED line(s) that do not hold
        \\
    , c.err.written());
    try testing.expectEqual(@as(usize, 3), r.fails);
}

test "a malformed line of the record is refused, and a GAP needs its tranche" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    try plant(tmp.dir, &.{.{ "ci/coverage/unbound_recast.txt", "# a comment\n" ++
        "Recast\tfunc:-::rcA/0\tBOUND\tzrcA mirrors it\n" ++
        "Recast\tfunc:-::rcB/0\tBOUND\n" ++
        "Recast\tfunc:-::rcC/0\tMAYBE\twhatever it is\n" ++
        "Recast\tfunc:-::rcD/0\tBOUND\tshort\n" ++
        "Recast\tfunc:-::rcE/0\tGAP\tsomeday\n" ++
        "Recast\tfunc:-::rcF/0\tGAP\ttranche 3b - the staged Recast pipeline\n" }});
    var c: Captured = .init(a);
    var r = c.report();
    const rows = try readRecord(.{ .io = testing.io, .dir = tmp.dir, .a = a }, &r);
    try testing.expectEqual(@as(usize, 2), rows.len);
    try testing.expectEqualStrings(
        \\ci/coverage/unbound_recast.txt:3: not four tab-separated fields
        \\ci/coverage/unbound_recast.txt:4: unknown verdict MAYBE
        \\ci/coverage/unbound_recast.txt:5: func:-::rcD/0 has no evidence
        \\ci/coverage/unbound_recast.txt:6: func:-::rcE/0 is a GAP not assigned to a tranche
        \\4 malformed line(s)
        \\
    , c.err.written());
}

test "the record is generated from the bindings and the tranche rules" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    const t: Tree = .{ .io = testing.io, .dir = tmp.dir, .a = a };
    const names = [_][]const u8{ "Recast\tfunc:-::rcA/0", "Recast\tfunc:-::rcB/0" };
    try plant(tmp.dir, &.{.{ bindings_path, "# header\nfunc:-::rcB/0\tBOUND\tzrcB mirrors it\n" }});
    var messages: std.ArrayList(u8) = .empty;
    const generated = (try generate(t, &names, &messages)).?;
    try testing.expectEqualStrings(
        "Recast\tfunc:-::rcA/0\tGAP\t" ++ t3b ++ "\nRecast\tfunc:-::rcB/0\tBOUND\tzrcB mirrors it\n",
        generated.files[0].text,
    );
    try testing.expectEqualStrings("ci/coverage/unbound_recast.txt", generated.files[0].path);
    try testing.expect(!try compareRecord(t, generated.files, &messages));
    try plant(tmp.dir, &.{.{ bindings_path, "func:-::rcA/0\tBOUND\n" ++
        "func:-::rcA/0\tGAP\ttranche 3b - x\n" ++
        "func:-::rcZ/0\tBOUND\tzrcZ mirrors it\n" ++
        "func:-::rcB/0\tBOUND\tzrcB mirrors it\n" ++
        "func:-::rcB/0\tZIG\tsrc/x.zig:b\n" }});
    messages.clearRetainingCapacity();
    try testing.expect(try generate(t, &names, &messages) == null);
    try testing.expectEqualStrings(
        \\ci/coverage/bindings.tsv:1: not SYMBOL<TAB>VERDICT<TAB>EVIDENCE
        \\ci/coverage/bindings.tsv:2: GAP is not a verdict a binding can carry
        \\ci/coverage/bindings.tsv:3: upstream declares no func:-::rcZ/0
        \\ci/coverage/bindings.tsv:5: func:-::rcB/0 is bound twice
        \\ci/coverage/bindings.tsv is malformed
        \\
    , messages.items);
}

test "every name has one verdict, and every verdict a name" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var c: Captured = .init(a);
    var r = c.report();
    const names = [_][]const u8{ "A\tx", "A\ty" };
    const rows = [_]Row{
        .{ .area = "A", .symbol = "x", .verdict = "BOUND", .evidence = "zrcX mirrors it" },
        .{ .area = "A", .symbol = "x", .verdict = "BOUND", .evidence = "zrcX mirrors it" },
        .{ .area = "A", .symbol = "z", .verdict = "BOUND", .evidence = "zrcZ mirrors it" },
    };
    try checkComplete(a, &r, &names, &rows);
    try testing.expectEqualStrings(
        "  A\ty\n1 area/name pair(s) with no verdict\n" ++
            "  A\tz\n1 stale line(s) — upstream no longer declares these; delete them\n" ++
            "  A\tx\nduplicate area/name pair(s) in the record\n",
        c.err.written(),
    );
}

test "an entry point no Zig source calls is stranded, unless excused" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    try plant(tmp.dir, &.{
        .{ "ffi/zrecast.h", "ZRC_API void zrcA(void);\nZRC_API void zrcB(void);\nZRC_API void zrcC (void);\n" },
        .{ "src/a.zig", "c.zrcA();\n" },
        .{ "src/c.zig", "pub extern fn zrcB() void;\n" },
        .{ "src/a_test.zig", "c.zrcB();\n" },
        .{ exceptions_path, "zrcA\tcalled after all, so not excusable\nzrcQ\tshort\n" },
    });
    const t: Tree = .{ .io = testing.io, .dir = tmp.dir, .a = a };
    var c: Captured = .init(a);
    var r = c.report();
    try testing.expectEqual(@as(usize, 2), try checkSurface(t, &r, try t.files("ffi", "", ".h")));
    try testing.expectEqualStrings(
        \\  zrcQ: no reason given
        \\1 malformed exception line(s)
        \\  zrcB
        \\1 entry point(s) with no Zig caller
        \\  zrcA
        \\1 excused entry point(s) that Zig does call, or that no longer exist
        \\
    , c.err.written());
}

test "an upstream directory no area claims fails" {
    var arena: std.heap.ArenaAllocator = .init(testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    try plant(tmp.dir, &.{
        .{ "libs/recastnavigation/Recast/Include/R.h", "" },
        .{ "libs/recastnavigation/Extra/E.h", "" },
    });
    var c: Captured = .init(a);
    var r = c.report();
    try checkClaimed(.{ .io = testing.io, .dir = tmp.dir, .a = a }, &r);
    try testing.expectEqualStrings("  Extra\n1 upstream directory(ies) no area claims\n", c.err.written());
}
