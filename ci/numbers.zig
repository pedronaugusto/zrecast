//! Refuses a multi-digit number written by hand in a document. A count that
//! belongs to the tree belongs in a generated block, which the docs check
//! rebuilds; anything else stated outside one must be in `allowed` with the
//! reason it cannot go stale. Numbers in a generated block, a fenced or
//! inline code span, a link target or an identifier (x86_64, 0.17.0) are not
//! hand-written claims. An allowance no document uses fails too: it describes
//! a sentence that is no longer there.
const std = @import("std");
const Io = std.Io;

/// The numbers a document may state by hand. Two kinds qualify: arithmetic,
/// true forever, and a property of the pinned vendor, which moves only at a
/// re-vendor, whose procedure re-reads the documents.
const allowed = [_]struct { number: []const u8, reason: []const u8 }{
    .{ .number = "170", .reason = "measured 2026-09-02: the ReleaseFast size gap between a query-only C program (462 KB) and the same with one bake call (634 KB), zig c++ -O2, x86_64-windows; stated as an approximation" },
    .{ .number = "855", .reason = "size of upstream's excluded Tests/ tree, summed file bytes at the pinned commit" },
    .{ .number = "584", .reason = "size of upstream's excluded Docs/ tree, summed file bytes at the pinned commit" },
    .{ .number = "29", .reason = "arithmetic: 73856093 * 29 fits an int and 73856093 * 30 does not (hashPos2, DetourProximityGrid.cpp)" },
    .{ .number = "32767", .reason = "arithmetic: the largest magnitude whose doubled square still fits an int" },
    .{ .number = "2147352578", .reason = "arithmetic: 2 * 32767^2" },
    .{ .number = "32", .reason = "MAX_NEIS, the fixed neighbour-tile buffer at the pinned commit (DetourNavMesh.cpp:1021)" },
    .{ .number = "128", .reason = "arithmetic: an erosion radius of 128 doubles to 256, which an unsigned char wraps to 0" },
    .{ .number = "129", .reason = "arithmetic: an erosion radius of 129 doubles to 258, which wraps to 2, radius 1's threshold" },
    .{ .number = "255", .reason = "the saturation ceiling rcErodeWalkableArea clamps distances to at the pinned commit (RecastArea.cpp:111)" },
    .{ .number = "127", .reason = "arithmetic: the largest erosion radius whose doubled threshold an unsigned char can hold" },
    .{ .number = "512", .reason = "MSG_SIZE, the log-message buffer at the pinned commit (Recast.cpp)" },
    .{ .number = "511", .reason = "arithmetic: MSG_SIZE minus the terminator, where a longer message is cut" },
};

/// Documents beyond the top-level `*.md` files.
const extra_documents = [_][]const u8{"docs/design.md"};
/// The largest document read.
const max_file_bytes = 1 << 22;

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    var arena: std.heap.ArenaAllocator = .init(init.gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const cwd = Io.Dir.cwd();
    var findings: std.ArrayList(u8) = .empty;
    var used: [allowed.len]bool = @splat(false);
    var names: std.ArrayList([]const u8) = .empty;
    var root = try cwd.openDir(io, ".", .{ .iterate = true });
    defer root.close(io);
    var it = root.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.name, ".md")) try names.append(a, try a.dupe(u8, entry.name));
    }
    for (extra_documents) |name| {
        _ = cwd.statFile(io, name, .{}) catch |err| switch (err) {
            error.FileNotFound => continue,
            else => |e| return e,
        };
        try names.append(a, name);
    }
    std.mem.sort([]const u8, names.items, {}, lessThan);
    for (names.items) |name| try check(a, name, try cwd.readFileAlloc(io, name, a, .limited(max_file_bytes)), &used, &findings);
    for (allowed, used) |entry, was| {
        if (!was) try findings.print(a, "ci/numbers.zig: {s} is allowed but no document states it\n", .{entry.number});
    }
    if (findings.items.len == 0) return 0;
    var buffer: [4096]u8 = undefined;
    var stderr = Io.File.stderr().writer(io, &buffer);
    try stderr.interface.writeAll(findings.items);
    try stderr.interface.flush();
    return 1;
}

fn lessThan(_: void, left: []const u8, right: []const u8) bool {
    return std.mem.lessThan(u8, left, right);
}

/// Appends a finding for every hand-written number of `text`, marking the
/// allowances it uses.
fn check(a: std.mem.Allocator, name: []const u8, text: []const u8, used: *[allowed.len]bool, findings: *std.ArrayList(u8)) !void {
    var prose = try strip(a, text, "<!-- BEGIN GENERATED ", "<!-- END GENERATED -->", true);
    prose = try strip(a, prose, "```", "```", true);
    prose = try strip(a, prose, "`", "`", false);
    prose = try strip(a, prose, "](", ")", true);
    var number: usize = 0;
    var lines = std.mem.splitScalar(u8, prose, '\n');
    while (lines.next()) |line| {
        number += 1;
        var from: usize = 0;
        while (nextNumber(line, from)) |found| {
            from = found.end;
            const token = line[found.start..found.end];
            if (allowance(token)) |index| {
                used[index] = true;
                continue;
            }
            try findings.print(a, "{s}:{d}: the number {s} is written by hand; generate it, or allow it in ci/numbers.zig with why it cannot go stale\n", .{ name, number, token });
        }
    }
}

fn allowance(token: []const u8) ?usize {
    for (allowed, 0..) |entry, index| if (std.mem.eql(u8, entry.number, token)) return index;
    return null;
}

/// `text` with every region from `open` to the next `close` removed, keeping
/// the region's newlines so line numbers stay true. A region that would cross
/// a line ends nowhere unless `multiline`, and is left as it is.
fn strip(a: std.mem.Allocator, text: []const u8, open: []const u8, close: []const u8, multiline: bool) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;
    var at: usize = 0;
    while (std.mem.findPos(u8, text, at, open)) |start| {
        const end = std.mem.findPos(u8, text, start + open.len, close) orelse break;
        if (!multiline and std.mem.findScalar(u8, text[start..end], '\n') != null) {
            try out.appendSlice(a, text[at .. start + 1]);
            at = start + 1;
            continue;
        }
        try out.appendSlice(a, text[at..start]);
        for (text[start .. end + close.len]) |byte| if (byte == '\n') try out.append(a, '\n');
        at = end + close.len;
    }
    try out.appendSlice(a, text[at..]);
    return out.items;
}

const Span = struct { start: usize, end: usize };

/// The next number of `line` at or after `from`: two or more characters of
/// digits, spaces and commas that start and end on a digit, standing apart
/// from identifiers, paths and versions.
fn nextNumber(line: []const u8, from: usize) ?Span {
    var start = from;
    while (start < line.len) {
        if (!std.ascii.isDigit(line[start]) or (start > 0 and joined(line[start - 1]))) {
            start += 1;
            continue;
        }
        var end = start;
        while (end < line.len and (std.ascii.isDigit(line[end]) or line[end] == ' ' or line[end] == ',')) end += 1;
        var last = end;
        while (last > start + 1) : (last -= 1) {
            if (std.ascii.isDigit(line[last - 1]) and (last == line.len or !joined(line[last]))) return .{ .start = start, .end = last };
        }
        start = end;
    }
    return null;
}

/// Whether a character beside a number makes it part of a longer token.
fn joined(byte: u8) bool {
    return std.ascii.isAlphanumeric(byte) or byte == '_' or byte == '.' or byte == '-' or byte == '/';
}

test "a number apart from an identifier is found, one inside one is not" {
    try std.testing.expectEqual(Span{ .start = 4, .end = 6 }, nextNumber("has 12 corners", 0).?);
    try std.testing.expect(nextNumber("x86_64 and 0.17.0 and v2", 0) == null);
    try std.testing.expect(nextNumber("one 7 digit", 0) == null);
    try std.testing.expectEqual(Span{ .start = 0, .end = 5 }, nextNumber("1,024 bytes", 0).?);
    try std.testing.expectEqual(Span{ .start = 0, .end = 2 }, nextNumber("12 3a", 0).?);
}

test "generated blocks, code and link targets are not prose" {
    const a = std.testing.allocator;
    var arena: std.heap.ArenaAllocator = .init(a);
    defer arena.deinit();
    var used: [allowed.len]bool = @splat(false);
    var findings: std.ArrayList(u8) = .empty;
    const text = "<!-- BEGIN GENERATED x -->\n| 40 |\n<!-- END GENERATED -->\n```\n41\n```\n`42` [a](b/43)\nthen 44\n";
    try check(arena.allocator(), "f.md", text, &used, &findings);
    try std.testing.expectEqualStrings("f.md:8: the number 44 is written by hand; generate it, or allow it in ci/numbers.zig with why it cannot go stale\n", findings.items);
}
