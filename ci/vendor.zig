//! Proves libs/recastnavigation is upstream at the pinned commit, byte for
//! byte: every taken path holds the same files with the same bytes, nothing
//! else sits beside them, and the design document names the commit
//! build.zig.zon pins. Behaviour the package needs that upstream lacks lives
//! in ffi/, on the package's side of the boundary.
//!
//! zrecast-vendor VENDORED UPSTREAM MANIFEST RECORD
const std = @import("std");
const aegis = @import("aegis");
const Io = std.Io;

/// What was taken from upstream: directories whole, files as they are.
const taken = [_][]const u8{ "Recast", "Detour", "DetourCrowd", "DetourTileCache", "DebugUtils", "License.txt", "README.md", "CHANGELOG.md" };
/// File names upstream has inside the taken directories and the vendored copy
/// drops: its CMake build, which build.zig supersedes.
const excluded = [_][]const u8{"CMakeLists.txt"};
/// The largest file compared.
const max_file_bytes = 1 << 22;

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    var arena: std.heap.ArenaAllocator = .init(init.gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const args = try init.minimal.args.toSlice(a);
    if (args.len != 5) return error.Usage;
    const cwd = Io.Dir.cwd();
    var vendored = try cwd.openDir(io, args[1], .{ .iterate = true });
    defer vendored.close(io);
    var upstream = try cwd.openDir(io, args[2], .{ .iterate = true });
    defer upstream.close(io);
    var findings: std.ArrayList(u8) = .empty;
    try compareTop(a, io, vendored, upstream, &findings);
    const manifest = try aegis.own.readFile(io, cwd, args[3], a, .fromRaw(1 << 20));
    const record = try aegis.own.readFile(io, cwd, args[4], a, .fromRaw(1 << 20));
    try checkRecord(a, manifest, record, &findings);
    if (findings.items.len == 0) return 0;
    var buffer: [4096]u8 = undefined;
    var stderr = Io.File.stderr().writer(io, &buffer);
    try stderr.interface.print("libs/recastnavigation is not upstream verbatim:\n{s}", .{findings.items});
    try stderr.interface.flush();
    return 1;
}

/// Compares each taken path, and refuses a vendored entry that is not one.
fn compareTop(a: std.mem.Allocator, io: Io, vendored: Io.Dir, upstream: Io.Dir, findings: *std.ArrayList(u8)) !void {
    var it = vendored.iterate();
    while (try it.next(io)) |entry| {
        if (!contains(&taken, entry.name)) try findings.print(a, "  {s}: not taken from upstream\n", .{entry.name});
    }
    for (taken) |name| {
        const kind = (upstream.statFile(io, name, .{}) catch {
            try findings.print(a, "  {s}: upstream has no such path\n", .{name});
            continue;
        }).kind;
        if (kind == .directory) try compareTrees(a, io, vendored, upstream, name, findings) else try compareFiles(a, io, vendored, upstream, name, findings);
    }
}

/// Compares two trees under `path`, both directions, by file.
fn compareTrees(a: std.mem.Allocator, io: Io, vendored: Io.Dir, upstream: Io.Dir, path: []const u8, findings: *std.ArrayList(u8)) !void {
    const ours = try listFiles(a, io, vendored, path);
    const theirs = try listFiles(a, io, upstream, path);
    for (theirs) |name| {
        if (!contains(ours, name)) try findings.print(a, "  {s}: missing from the vendored copy\n", .{name});
    }
    for (ours) |name| {
        if (!contains(theirs, name)) try findings.print(a, "  {s}: added to the vendored copy\n", .{name}) else try compareFiles(a, io, vendored, upstream, name, findings);
    }
}

/// Every file under `path`, minus the excluded names, as paths with `/`.
fn listFiles(a: std.mem.Allocator, io: Io, root: Io.Dir, path: []const u8) ![]const []const u8 {
    var dir = root.openDir(io, path, .{ .iterate = true }) catch |err| switch (err) {
        error.FileNotFound => return &.{},
        else => |e| return e,
    };
    defer dir.close(io);
    var walker = try dir.walk(a);
    defer walker.deinit();
    var names: std.ArrayList([]const u8) = .empty;
    while (try walker.next(io)) |entry| {
        if (entry.kind != .file or contains(&excluded, entry.basename)) continue;
        const relative = try a.dupe(u8, entry.path);
        std.mem.replaceScalar(u8, relative, '\\', '/');
        try names.append(a, try a.print("{s}/{s}", .{ path, relative }));
    }
    return names.items;
}

/// Compares one file's bytes.
fn compareFiles(a: std.mem.Allocator, io: Io, vendored: Io.Dir, upstream: Io.Dir, name: []const u8, findings: *std.ArrayList(u8)) !void {
    const ours = aegis.own.readFile(io, vendored, name, a, .fromRaw(max_file_bytes)) catch |err| switch (err) {
        error.FileNotFound => return findings.print(a, "  {s}: missing from the vendored copy\n", .{name}),
        else => |e| return e,
    };
    const theirs = try aegis.own.readFile(io, upstream, name, a, .fromRaw(max_file_bytes));
    if (!std.mem.eql(u8, ours, theirs)) try findings.print(a, "  {s}: differs from upstream\n", .{name});
}

/// Requires the record to name the commit the manifest pins upstream at.
fn checkRecord(a: std.mem.Allocator, manifest: []const u8, record: []const u8, findings: *std.ArrayList(u8)) !void {
    const commit = pinnedCommit(manifest) orelse return findings.print(a, "  build.zig.zon pins recastnavigation at no commit\n", .{});
    if (std.mem.find(u8, record, commit) == null) try findings.print(a, "  the vendor record does not name the pinned commit {s}\n", .{commit});
}

/// The commit after `#` in the recastnavigation pin's URL.
fn pinnedCommit(manifest: []const u8) ?[]const u8 {
    const at = std.mem.find(u8, manifest, ".recastnavigation = .{") orelse return null;
    const url = std.mem.findPos(u8, manifest, at, "github.com/recastnavigation/recastnavigation") orelse return null;
    const hash = std.mem.findScalarPos(u8, manifest, url, '#') orelse return null;
    const end = std.mem.findScalarPos(u8, manifest, hash, '"') orelse return null;
    return manifest[hash + 1 .. end];
}

fn contains(list: []const []const u8, item: []const u8) bool {
    for (list) |candidate| if (std.mem.eql(u8, candidate, item)) return true;
    return false;
}

test "the pinned commit is read from the manifest's URL" {
    const manifest =
        \\        .recastnavigation = .{
        \\            .url = "git+https://github.com/recastnavigation/recastnavigation?ref=v1.6.0#6dc1667f",
    ;
    try std.testing.expectEqualStrings("6dc1667f", pinnedCommit(manifest).?);
    try std.testing.expect(pinnedCommit(".other = .{}") == null);
}
