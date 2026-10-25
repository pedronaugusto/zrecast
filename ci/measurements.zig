//! Recomputes every count README.md publishes, as the Markdown table its
//! generated block holds (`zig build measurements`). The docs check rebuilds
//! the block from this program and fails when the two differ, so no count in
//! the README is written by hand. Run from the repository root.
const std = @import("std");
const Io = std.Io;

/// The largest file read.
const max_file_bytes = 1 << 22;

const Row = struct { value: []const u8, description: []const u8 };

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    var arena: std.heap.ArenaAllocator = .init(init.gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const cwd = Io.Dir.cwd();

    const entry_points = try countInDirectory(a, io, cwd, "ffi", ".h", "ZRC_API");
    // The boundary's externs, which src/c.zig alone declares; the surface
    // check's planted drifts declare others on purpose.
    const externs = try countLines(a, io, cwd, "src/c.zig", "pub extern fn zrc");
    var buffer: [4096]u8 = undefined;
    // The surface check pairs every declaration with its extern, so a
    // difference is a defect in this program.
    if (entry_points != externs) {
        var stderr = Io.File.stderr().writer(io, &buffer);
        try stderr.interface.print("{d} C entry points but {d} Zig externs\n", .{ entry_points, externs });
        try stderr.interface.flush();
        return 1;
    }
    const manifest = try cwd.readFileAlloc(io, "build.zig.zon", a, .limited(1 << 20));
    const ledger = try verdicts(a, io, cwd);
    const rows = [_]Row{
        .{ .value = version(manifest) orelse return error.NoVersion, .description = "version (one home: `build.zig.zon`)" },
        .{ .value = try a.print("{d}", .{entry_points}), .description = "C entry points (`ZRC_API` in `ffi/*.h`), each with its Zig extern" },
        .{ .value = try a.print("{d}", .{ledger.names}), .description = "public Recast/Detour names, each carrying a verdict in `ci/coverage/`" },
        .{ .value = try a.print("{d}", .{ledger.bound}), .description = "of them reachable through the C boundary (`BOUND`)" },
        .{ .value = try a.print("{d}", .{ledger.language}), .description = "C++-only surface a C boundary cannot carry (`LANGUAGE`), each with the reason" },
        .{ .value = try a.print("{d}", .{ledger.zig}), .description = "reimplemented on the Zig side (`ZIG`), each naming its mirror" },
        .{ .value = try a.print("{d}", .{ledger.undefined}), .description = "declared upstream and defined nowhere (`UNDEFINED`), so no host can call it" },
        .{ .value = try a.print("{d}", .{try countLines(a, io, cwd, "tests/c_smoke.c", "CHECK(")}), .description = "assertions in the standalone C smoke test" },
        .{ .value = try a.print("{d}", .{try countTranslationUnits(a, io, cwd)}), .description = "vendored recastnavigation translation units `build.zig` compiles" },
        .{ .value = try a.print("{d}", .{try countFiles(io, cwd, "ci/probes", ".patch")}), .description = "invariants `bay probe` mutates, each with the test that must notice" },
    };
    var stdout = Io.File.stdout().writer(io, &buffer);
    try stdout.interface.writeAll("| | |\n|---:|---|\n");
    for (rows) |row| try stdout.interface.print("| **{s}** | {s} |\n", .{ row.value, row.description });
    try stdout.interface.flush();
    return 0;
}

/// The `.version` string of build.zig.zon.
fn version(manifest: []const u8) ?[]const u8 {
    const key = ".version = \"";
    const at = std.mem.find(u8, manifest, key) orelse return null;
    const start = at + key.len;
    const end = std.mem.findScalarPos(u8, manifest, start, '"') orelse return null;
    return manifest[start..end];
}

const Ledger = struct { names: usize = 0, bound: usize = 0, language: usize = 0, zig: usize = 0, undefined: usize = 0 };

/// The verdicts of the coverage record, ci/coverage/unbound_*.txt: one
/// AREA<TAB>NAME<TAB>VERDICT<TAB>EVIDENCE line per public upstream name.
fn verdicts(a: std.mem.Allocator, io: Io, root: Io.Dir) !Ledger {
    var dir = try root.openDir(io, "ci/coverage", .{ .iterate = true });
    defer dir.close(io);
    var ledger: Ledger = .{};
    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind != .file or !std.mem.startsWith(u8, entry.name, "unbound_") or !std.mem.endsWith(u8, entry.name, ".txt")) continue;
        var lines = std.mem.splitScalar(u8, try dir.readFileAlloc(io, entry.name, a, .limited(max_file_bytes)), '\n');
        while (lines.next()) |line| tally(&ledger, line);
    }
    return ledger;
}

fn tally(ledger: *Ledger, line: []const u8) void {
    var fields = std.mem.splitScalar(u8, line, '\t');
    _ = fields.next();
    _ = fields.next() orelse return;
    const verdict = fields.next() orelse return;
    ledger.names += 1;
    if (std.mem.eql(u8, verdict, "BOUND")) ledger.bound += 1;
    if (std.mem.eql(u8, verdict, "LANGUAGE")) ledger.language += 1;
    if (std.mem.eql(u8, verdict, "ZIG")) ledger.zig += 1;
    if (std.mem.eql(u8, verdict, "UNDEFINED")) ledger.undefined += 1;
}

/// Lines starting with `prefix`, after indentation, in every `suffix` file of `path`.
fn countInDirectory(a: std.mem.Allocator, io: Io, root: Io.Dir, path: []const u8, suffix: []const u8, prefix: []const u8) !usize {
    var dir = try root.openDir(io, path, .{ .iterate = true });
    defer dir.close(io);
    var total: usize = 0;
    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind != .file or !std.mem.endsWith(u8, entry.name, suffix)) continue;
        total += startingWith(try dir.readFileAlloc(io, entry.name, a, .limited(max_file_bytes)), prefix);
    }
    return total;
}

fn countLines(a: std.mem.Allocator, io: Io, root: Io.Dir, path: []const u8, prefix: []const u8) !usize {
    return startingWith(try root.readFileAlloc(io, path, a, .limited(max_file_bytes)), prefix);
}

fn startingWith(text: []const u8, prefix: []const u8) usize {
    var count: usize = 0;
    var lines = std.mem.splitScalar(u8, text, '\n');
    while (lines.next()) |line| {
        if (std.mem.startsWith(u8, std.mem.trimStart(u8, line, " \t"), prefix)) count += 1;
    }
    return count;
}

/// The quoted `libs/recastnavigation/….cpp` paths in build.zig.
fn countTranslationUnits(a: std.mem.Allocator, io: Io, root: Io.Dir) !usize {
    const text = try root.readFileAlloc(io, "build.zig", a, .limited(max_file_bytes));
    var count: usize = 0;
    var from: usize = 0;
    while (std.mem.findPos(u8, text, from, "\"libs/recastnavigation/")) |at| {
        const end = std.mem.findScalarPos(u8, text, at + 1, '"') orelse break;
        if (std.mem.endsWith(u8, text[at + 1 .. end], ".cpp")) count += 1;
        from = end + 1;
    }
    return count;
}

/// The `suffix` files directly in `path`.
fn countFiles(io: Io, root: Io.Dir, path: []const u8, suffix: []const u8) !usize {
    var dir = try root.openDir(io, path, .{ .iterate = true });
    defer dir.close(io);
    var count: usize = 0;
    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.name, suffix)) count += 1;
    }
    return count;
}

test "the version is read from the manifest" {
    try std.testing.expectEqualStrings("0.1.1", version(".{\n    .version = \"0.1.1\",\n").?);
    try std.testing.expect(version(".{}") == null);
}

test "a line counts once, after its indentation" {
    try std.testing.expectEqual(@as(usize, 2), startingWith("  CHECK(a);\nCHECK(b); CHECK(c);\n// CHECK(d)\n", "CHECK("));
}
