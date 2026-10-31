//! Mutation probes (`bay probe [filter]`): each patch in ci/probes/ breaks one
//! invariant on purpose and names the failure that must follow. A probe the
//! suite survives is a test that tests nothing; a probe that fails for another
//! reason proves nothing either, so it is a miss too. Run from the repository
//! root, under bay, against a copy of the tree in .zig-cache/probes/tree: the
//! sources here are never touched.
//!
//! A probe is a unified diff over paths relative to the root (`a/` and `b/`
//! prefixes), preceded by its header:
//!
//!   # expect: <text the failing output must contain>
//!   # run: zig build <step and options>
//!
//! The default run is the suite without the source checks, which are the
//! gate's: `zig build test -Dci-lint=false`.
//!
//! A hunk's context and removed lines must stand in the file exactly; it
//! applies where they stand nearest its stated line, and a probe whose lines
//! stand nowhere is refused as stale rather than applied to something else.
const std = @import("std");
const aegis = @import("aegis");
const Io = std.Io;

const probes_dir = "ci/probes";
const tree = ".zig-cache/probes/tree";
/// Top-level entries of the copy kept between runs: its build cache and the
/// packages it fetched, so a run stays warm.
const kept = [_][]const u8{ ".zig-cache", "zig-pkg" };
/// Directory names never copied, at any depth.
const skipped = [_][]const u8{ ".git", ".zig-cache", "zig-out", "zig-pkg" };
const max_file_bytes = 1 << 24;
const max_output_bytes = 1 << 26;

const Outcome = enum { caught, not_caught, other_failure, stale };

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    var arena: std.heap.ArenaAllocator = .init(init.gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const args = try init.minimal.args.toSlice(a);
    const filter: []const u8 = if (args.len > 1) args[1] else "";
    var buffer: [4096]u8 = undefined;
    var stdout = Io.File.stdout().writer(io, &buffer);
    const out = &stdout.interface;
    const bay = init.environ_map.get("BAY") orelse {
        try out.writeAll("probes: run through `bay probe`, which names itself in BAY\n");
        try out.flush();
        return 1;
    };

    const cwd = Io.Dir.cwd();
    try refresh(a, io, cwd);
    var copy = try cwd.openDir(io, tree, .{});
    defer copy.close(io);

    var names: std.ArrayList([]const u8) = .empty;
    var dir = try cwd.openDir(io, probes_dir, .{ .iterate = true });
    defer dir.close(io);
    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.name, ".patch") and std.mem.find(u8, entry.name, filter) != null) try names.append(a, try a.dupe(u8, entry.name));
    }
    if (names.items.len == 0) {
        try out.print("probes: no probe in {s} matches '{s}'\n", .{ probes_dir, filter });
        try out.flush();
        return 1;
    }
    std.mem.sort([]const u8, names.items, {}, lessThan);

    var missed: std.ArrayList(u8) = .empty;
    var caught: usize = 0;
    for (names.items) |name| {
        const text = try aegis.own.readFile(io, dir, name, a, .fromRaw(max_file_bytes));
        const result = try probe(a, io, init.environ_map, bay, cwd, copy, text);
        try out.print("  {s:<56} {s}\n", .{ name[0 .. name.len - ".patch".len], label(result.outcome) });
        if (result.outcome == .caught) {
            caught += 1;
            continue;
        }
        try missed.print(a, "  - {s} ({s})\n", .{ name, label(result.outcome) });
        if (result.detail.len != 0) try out.print("{s}", .{result.detail});
    }
    try out.print("\n{d} caught, {d} missed\n{s}", .{ caught, names.items.len - caught, missed.items });
    try out.flush();
    return if (caught == names.items.len) 0 else 1;
}

fn lessThan(_: void, left: []const u8, right: []const u8) bool {
    return std.mem.lessThan(u8, left, right);
}

fn label(outcome: Outcome) []const u8 {
    return switch (outcome) {
        .caught => "caught",
        .not_caught => "NOT CAUGHT",
        .other_failure => "caught by something else",
        .stale => "does not apply",
    };
}

const Result = struct { outcome: Outcome, detail: []const u8 = "" };

/// Applies one probe to the copy, runs its commands, and makes the copy the
/// repository again.
fn probe(a: std.mem.Allocator, io: Io, environ: *const std.process.Environ.Map, bay: []const u8, root: Io.Dir, copy: Io.Dir, text: []const u8) !Result {
    const parsed = parse(a, text) catch |err| return .{ .outcome = .stale, .detail = try a.print("      malformed probe: {t}\n", .{err}) };
    // A command may write files of its own, so the whole copy is made the
    // repository again, not just the files the probe patched.
    defer refresh(a, io, root) catch |err| std.debug.panic("restoring the copy: {t}", .{err});
    for (parsed.files) |file| {
        const original = try aegis.own.readFile(io, copy, file.path, a, .fromRaw(max_file_bytes));
        const patched = apply(a, original, file.hunks) catch |err| return .{ .outcome = .stale, .detail = try a.print("      {s}: {t}\n", .{ file.path, err }) };
        try copy.writeFile(io, .{ .sub_path = file.path, .data = patched });
    }
    // Every command but the last prepares (regenerating a record, say) and
    // must pass; the last is the one that has to notice.
    const preparing = parsed.runs[0 .. parsed.runs.len - 1];
    for (preparing) |command| {
        const run = try runCommand(a, io, environ, bay, copy, command);
        if (run.term != .exited or run.term.exited != 0) return .{ .outcome = .other_failure, .detail = try failure(a, command, "it to pass", run) };
    }
    const command = parsed.runs[parsed.runs.len - 1];
    const run = try runCommand(a, io, environ, bay, copy, command);
    if (run.term == .exited and run.term.exited == 0) return .{ .outcome = .not_caught };
    if (std.mem.find(u8, run.stdout, parsed.expect) != null or std.mem.find(u8, run.stderr, parsed.expect) != null) return .{ .outcome = .caught };
    return .{ .outcome = .other_failure, .detail = try failure(a, command, try a.print("to see: {s}", .{parsed.expect}), run) };
}

/// A probe's `zig build ...` command, run through bay in the copy.
fn runCommand(a: std.mem.Allocator, io: Io, environ: *const std.process.Environ.Map, bay: []const u8, copy: Io.Dir, command: []const u8) !std.process.RunResult {
    var argv: std.ArrayList([]const u8) = .empty;
    try argv.appendSlice(a, &.{ bay, "dev" });
    var words = std.mem.tokenizeScalar(u8, command["zig build".len..], ' ');
    while (words.next()) |word| try argv.append(a, word);
    return std.process.run(a, io, .{ .argv = argv.items, .cwd = .{ .dir = copy }, .environ_map = environ, .stdout_limit = .limited(max_output_bytes), .stderr_limit = .limited(max_output_bytes) });
}

fn failure(a: std.mem.Allocator, command: []const u8, wanted: []const u8, run: std.process.RunResult) ![]const u8 {
    return a.print("      `{s}`: expected {s}\n{s}{s}", .{ command, wanted, try tail(a, run.stdout, 12), try tail(a, run.stderr, 12) });
}

/// The last `count` lines of `text`, indented.
fn tail(a: std.mem.Allocator, text: []const u8, count: usize) ![]const u8 {
    var start = text.len;
    var seen: usize = 0;
    while (start > 0 and seen <= count) : (start -= 1) {
        if (text[start - 1] == '\n') seen += 1;
    }
    var out: std.ArrayList(u8) = .empty;
    var lines = std.mem.splitScalar(u8, std.mem.trim(u8, text[start..], "\n"), '\n');
    while (lines.next()) |line| try out.print(a, "      | {s}\n", .{std.mem.trimEnd(u8, line, "\r")});
    return out.items;
}

const FilePatch = struct { path: []const u8, hunks: []const u8 };
const Probe = struct { expect: []const u8, runs: []const []const u8, files: []const FilePatch };

/// The header and the per-file sections of a probe.
fn parse(a: std.mem.Allocator, text: []const u8) !Probe {
    var expect: ?[]const u8 = null;
    var runs: std.ArrayList([]const u8) = .empty;
    var files: std.ArrayList(FilePatch) = .empty;
    var lines = std.mem.splitScalar(u8, text, '\n');
    var section: ?usize = null;
    var path: []const u8 = "";
    while (lines.next()) |line| {
        const at = lines.index orelse text.len;
        if (std.mem.startsWith(u8, line, "# expect: ")) expect = line["# expect: ".len..];
        if (std.mem.startsWith(u8, line, "# run: ")) {
            const command = line["# run: ".len..];
            if (!std.mem.startsWith(u8, command, "zig build")) return error.RunIsNotZigBuild;
            try runs.append(a, command);
        }
        if (std.mem.startsWith(u8, line, "--- ")) {
            if (section) |start| try files.append(a, .{ .path = path, .hunks = text[start .. at - line.len - 1] });
            const plus = lines.next() orelse return error.MalformedPatch;
            if (!std.mem.startsWith(u8, plus, "+++ b/")) return error.MalformedPatch;
            path = plus["+++ b/".len..];
            if (!std.mem.eql(u8, line, try a.print("--- a/{s}", .{path}))) return error.MalformedPatch;
            section = lines.index orelse text.len;
        }
    }
    const start = section orelse return error.MalformedPatch;
    try files.append(a, .{ .path = path, .hunks = text[start..] });
    if (runs.items.len == 0) try runs.append(a, "zig build test -Dci-lint=false");
    return .{ .expect = expect orelse return error.NoExpectLine, .runs = runs.items, .files = files.items };
}

/// `original` with the hunks of one file applied. Every context and removed
/// line must match `original` exactly; a hunk may have moved since the probe
/// was written, and the nearest place its lines stand, after the previous
/// hunk, is where it applies.
fn apply(a: std.mem.Allocator, original: []const u8, hunks: []const u8) ![]const u8 {
    var source: std.ArrayList([]const u8) = .empty;
    var split = std.mem.splitScalar(u8, original, '\n');
    while (split.next()) |line| try source.append(a, line);
    var out: std.ArrayList([]const u8) = .empty;
    var cursor: usize = 0;
    var lines = std.mem.splitScalar(u8, hunks, '\n');
    while (lines.next()) |header| {
        if (!std.mem.startsWith(u8, header, "@@ -")) continue;
        const range = try parseRange(header);
        var old: std.ArrayList([]const u8) = .empty;
        var new: std.ArrayList([]const u8) = .empty;
        while (old.items.len < range.old or new.items.len < range.new) {
            const line = lines.next() orelse return error.MalformedPatch;
            const body = if (line.len == 0) "" else line[1..];
            switch (if (line.len == 0) ' ' else line[0]) {
                ' ' => {
                    try old.append(a, body);
                    try new.append(a, body);
                },
                '-' => try old.append(a, body),
                '+' => try new.append(a, body),
                '\\' => {},
                else => return error.MalformedPatch,
            }
        }
        const at = nearest(source.items, old.items, cursor, range.start -| 1) orelse return error.PatchMismatch;
        try out.appendSlice(a, source.items[cursor..at]);
        try out.appendSlice(a, new.items);
        cursor = at + old.items.len;
    }
    try out.appendSlice(a, source.items[cursor..]);
    return std.mem.join(a, "\n", out.items);
}

const Range = struct { start: usize, old: usize, new: usize };

/// The `@@ -start,old +start,new @@` of a hunk; an omitted count is one.
fn parseRange(header: []const u8) !Range {
    var words = std.mem.tokenizeScalar(u8, header["@@ ".len..], ' ');
    const removed = words.next() orelse return error.MalformedPatch;
    const added = words.next() orelse return error.MalformedPatch;
    if (removed[0] != '-' or added[0] != '+') return error.MalformedPatch;
    const old = try parseSpan(removed[1..]);
    const new = try parseSpan(added[1..]);
    return .{ .start = old.start, .old = old.count, .new = new.count };
}

fn parseSpan(text: []const u8) !struct { start: usize, count: usize } {
    const comma = std.mem.findScalar(u8, text, ',');
    const start = std.fmt.parseInt(usize, text[0 .. comma orelse text.len], 10) catch return error.MalformedPatch;
    const count = if (comma) |at| std.fmt.parseInt(usize, text[at + 1 ..], 10) catch return error.MalformedPatch else 1;
    return .{ .start = start, .count = count };
}

/// The index at or after `from` nearest `wanted` where `needle` stands in `lines`.
fn nearest(lines: []const []const u8, needle: []const []const u8, from: usize, wanted: usize) ?usize {
    if (needle.len > lines.len) return null;
    var best: ?usize = null;
    var index = from;
    while (index + needle.len <= lines.len) : (index += 1) {
        if (!matches(lines[index..][0..needle.len], needle)) continue;
        const distance = if (index > wanted) index - wanted else wanted - index;
        const best_distance = if (best) |b| (if (b > wanted) b - wanted else wanted - b) else std.math.maxInt(usize);
        if (distance < best_distance) best = index;
    }
    return best;
}

fn matches(have: []const []const u8, want: []const []const u8) bool {
    for (have, want) |left, right| if (!std.mem.eql(u8, left, right)) return false;
    return true;
}

/// Makes the copy the repository again: everything but `kept` is removed,
/// then every file is copied over.
fn refresh(a: std.mem.Allocator, io: Io, root: Io.Dir) !void {
    var copy = try root.createDirPathOpen(io, tree, .{ .open_options = .{ .iterate = true } });
    defer copy.close(io);
    var old: std.ArrayList([]const u8) = .empty;
    var entries = copy.iterate();
    while (try entries.next(io)) |entry| {
        if (!contains(&kept, entry.name)) try old.append(a, try a.dupe(u8, entry.name));
    }
    for (old.items) |name| try copy.deleteTree(io, name);
    var source = try root.openDir(io, ".", .{ .iterate = true });
    defer source.close(io);
    var walker = try source.walkSelectively(a);
    defer walker.deinit();
    while (try walker.next(io)) |entry| switch (entry.kind) {
        .directory => if (!contains(&skipped, entry.basename)) {
            try copy.createDirPath(io, entry.path);
            try walker.enter(io, entry);
        },
        .file => try entry.dir.copyFile(entry.basename, copy, entry.path, io, .{}),
        else => {},
    };
}

fn contains(list: []const []const u8, item: []const u8) bool {
    for (list) |candidate| if (std.mem.eql(u8, candidate, item)) return true;
    return false;
}

test "a hunk applies where its lines stand, and nowhere else" {
    var arena: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const original = "one\ntwo\nthree\nfour\n";
    try std.testing.expectEqualStrings("one\ntwo\n3\nfour\n", try apply(a, original, "@@ -2,2 +2,2 @@\n two\n-three\n+3\n"));
    // Moved, the hunk applies where its lines now stand.
    try std.testing.expectEqualStrings("one\ntwo\nthree\n4\n", try apply(a, original, "@@ -1,1 +1,1 @@\n-four\n+4\n"));
    // Lines that stand nowhere refuse the probe as stale.
    try std.testing.expectError(error.PatchMismatch, apply(a, original, "@@ -2,1 +2,1 @@\n-five\n+5\n"));
}

test "a probe names its expectation and its files" {
    var arena: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena.deinit();
    const a = arena.allocator();
    const parsed = try parse(a, "# expect: it broke\n--- a/x.c\n+++ b/x.c\n@@ -1,1 +1,1 @@\n-a\n+b\n--- a/y.c\n+++ b/y.c\n@@ -1,1 +1,1 @@\n-c\n+d\n");
    try std.testing.expectEqualStrings("it broke", parsed.expect);
    try std.testing.expectEqualStrings("zig build test -Dci-lint=false", parsed.runs[0]);
    try std.testing.expectEqual(@as(usize, 2), parsed.files.len);
    try std.testing.expectEqualStrings("y.c", parsed.files[1].path);
    try std.testing.expectEqualStrings("b\n", try apply(a, "a\n", parsed.files[0].hunks));
    try std.testing.expectError(error.NoExpectLine, parse(a, "--- a/x\n+++ b/x\n"));
    try std.testing.expectError(error.RunIsNotZigBuild, parse(a, "# expect: e\n# run: ci/check.sh\n--- a/x\n+++ b/x\n"));
}
