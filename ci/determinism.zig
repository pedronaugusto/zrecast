//! Cook determinism across targets (`bay determinism [--strict] [filter]`).
//! The development build hands this program the suite built per target; it
//! runs each where this host can, natively or in a container, and collects
//! the three cook digests a run prints when ZRECAST_COOK_DIGESTS is set.
//!
//! Targets sharing a C library must agree, and a disagreement fails. Across C
//! libraries the digests are reported, not gated: upstream's qsort ties fall
//! to the implementation. See docs/design.md, Known upstream behaviour: `qsort`.
//!
//! A target ran, failed, or could not run on this host; the last is counted
//! and named, and `--strict` makes it a failure.
//!
//! Arguments after the options, four per target: the target, its C-library
//! group, its runner (`native` or `docker:<image>:<platform>`), the binary.
const std = @import("std");
const Io = std.Io;

const max_output_bytes = 1 << 26;
const pull_seconds = 60;
const run_seconds = 600;

const Target = struct { name: []const u8, group: []const u8, runner: []const u8, binary: []const u8 };
const Ran = struct { target: Target, digests: []const u8 };

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    var arena: std.heap.ArenaAllocator = .init(init.gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const args = try init.minimal.args.toSlice(a);
    var strict = false;
    var filter: []const u8 = "";
    var targets: std.ArrayList(Target) = .empty;
    var index: usize = 1;
    while (index + 4 <= args.len and !std.mem.startsWith(u8, args[index], "--")) : (index += 4) {
        try targets.append(a, .{ .name = args[index], .group = args[index + 1], .runner = args[index + 2], .binary = args[index + 3] });
    }
    for (args[index..]) |arg| {
        if (std.mem.eql(u8, arg, "--strict")) strict = true else if (!std.mem.eql(u8, arg, "--")) filter = arg;
    }

    var environ = try init.environ_map.clone(a);
    try environ.put("ZRECAST_COOK_DIGESTS", "1");
    var buffer: [4096]u8 = undefined;
    var stdout = Io.File.stdout().writer(io, &buffer);
    const out = &stdout.interface;

    var ran: std.ArrayList(Ran) = .empty;
    var failed: std.ArrayList([]const u8) = .empty;
    var unrun: std.ArrayList([]const u8) = .empty;
    for (targets.items) |target| {
        if (std.mem.find(u8, target.name, filter) == null) continue;
        const result = try cook(a, io, &environ, target);
        switch (result) {
            .ran => |digests| {
                try ran.append(a, .{ .target = target, .digests = digests });
                try out.print("  {s:<24} ran ({s})\n", .{ target.name, target.group });
            },
            .failed => |why| {
                try failed.append(a, try a.print("{s}: {s}", .{ target.name, why }));
                try out.print("  {s:<24} FAILED: {s}\n", .{ target.name, why });
            },
            .unrun => |why| {
                try unrun.append(a, try a.print("{s}: {s}", .{ target.name, why }));
                try out.print("  {s:<24} unrun ({s})\n", .{ target.name, why });
            },
        }
    }

    for (ran.items, 0..) |left, i| for (ran.items[i + 1 ..]) |right| {
        if (!std.mem.eql(u8, left.target.group, right.target.group) or std.mem.eql(u8, left.digests, right.digests)) continue;
        try failed.append(a, try a.print("{s} and {s} disagree within {s}", .{ left.target.name, right.target.name, left.target.group }));
    };
    if (ran.items.len > 1) {
        var same = true;
        for (ran.items[1..]) |r| same = same and std.mem.eql(u8, r.digests, ran.items[0].digests);
        if (same) try out.writeAll("\nevery target run produced identical bytes\n") else {
            try out.writeAll("\nthe digests differ across C libraries, at upstream's qsort ties:\n");
            for (ran.items) |r| try out.print("  {s} ({s})\n{s}", .{ r.target.name, r.target.group, r.digests });
        }
    }
    try out.print("\n{d} ran, {d} failed, {d} unrun\n", .{ ran.items.len, failed.items.len, unrun.items.len });
    for (failed.items) |line| try out.print("  - {s}\n", .{line});
    for (unrun.items) |line| try out.print("  - {s}\n", .{line});
    if (strict and unrun.items.len != 0) try out.writeAll("--strict: an unrun target is a target this says nothing about\n");
    try out.flush();
    return if (failed.items.len == 0 and !(strict and unrun.items.len != 0)) 0 else 1;
}

const Outcome = union(enum) { ran: []const u8, failed: []const u8, unrun: []const u8 };

/// Runs one target's suite and takes its digests.
fn cook(a: std.mem.Allocator, io: Io, environ: *const std.process.Environ.Map, target: Target) !Outcome {
    var argv: std.ArrayList([]const u8) = .empty;
    if (std.mem.eql(u8, target.runner, "native")) {
        try argv.append(a, target.binary);
    } else if (std.mem.startsWith(u8, target.runner, "docker:")) {
        const container = parseDocker(target.runner) orelse return .{ .failed = "malformed docker runner" };
        if (try docker(a, io, environ, container)) |why| return .{ .unrun = why };
        const mount = try a.print("{s}:/zrecast:ro", .{std.fs.path.dirname(target.binary) orelse "."});
        try argv.appendSlice(a, &.{ "docker", "run", "--rm", "--platform", container.platform, "-e", "ZRECAST_COOK_DIGESTS=1", "-v", mount, container.image, try a.print("/zrecast/{s}", .{std.fs.path.basename(target.binary)}) });
    } else return .{ .failed = "unknown runner" };

    const run = std.process.run(a, io, .{ .argv = argv.items, .environ_map = environ, .stdout_limit = .limited(max_output_bytes), .stderr_limit = .limited(max_output_bytes), .timeout = seconds(run_seconds) }) catch |err| switch (err) {
        error.Timeout => return .{ .failed = "did not finish in time" },
        error.InvalidExe, error.FileNotFound, error.AccessDenied => return .{ .unrun = "this host cannot execute the binary" },
        else => |e| return e,
    };
    const output = try std.mem.concat(a, u8, &.{ run.stdout, run.stderr });
    if (run.term != .exited or run.term.exited != 0) {
        if (std.mem.find(u8, output, "exec format error") != null or std.mem.find(u8, output, "no matching manifest") != null) return .{ .unrun = try a.print("{s} cannot execute here", .{target.runner}) };
        return .{ .failed = "the suite failed" };
    }
    const digests = try digestLines(a, output);
    if (std.mem.count(u8, digests, "\n") != 3) return .{ .failed = "the suite passed but printed no digests" };
    return .{ .ran = digests };
}

fn seconds(count: i64) Io.Timeout {
    return .{ .duration = .{ .raw = .fromSeconds(count), .clock = .awake } };
}

const Docker = struct { image: []const u8, platform: []const u8 };

/// `docker:<image>:<platform>`; the image may carry a tag of its own.
fn parseDocker(runner: []const u8) ?Docker {
    const rest = runner["docker:".len..];
    const colon = std.mem.findScalarLast(u8, rest, ':') orelse return null;
    if (colon == 0 or colon + 1 == rest.len) return null;
    return .{ .image = rest[0..colon], .platform = rest[colon + 1 ..] };
}

/// Why this host cannot run `container`, or null when it can: no docker, no
/// daemon, or an image that is not here and cannot be fetched in time.
fn docker(a: std.mem.Allocator, io: Io, environ: *const std.process.Environ.Map, container: Docker) !?[]const u8 {
    const info = std.process.run(a, io, .{ .argv = &.{ "docker", "info" }, .environ_map = environ, .timeout = seconds(pull_seconds) }) catch return "no docker on this host";
    if (info.term != .exited or info.term.exited != 0) return "the docker daemon is not running";
    const inspect = try std.process.run(a, io, .{ .argv = &.{ "docker", "image", "inspect", container.image }, .environ_map = environ, .timeout = seconds(pull_seconds) });
    if (inspect.term == .exited and inspect.term.exited == 0) return null;
    const pull = std.process.run(a, io, .{ .argv = &.{ "docker", "pull", "--platform", container.platform, container.image }, .environ_map = environ, .timeout = seconds(pull_seconds) }) catch return try a.print("{s} is not here and could not be fetched in time", .{container.image});
    if (pull.term != .exited or pull.term.exited != 0) return try a.print("{s} is not here and could not be fetched", .{container.image});
    return null;
}

/// The `cook digest` lines of a run, sorted, each ending in a newline.
fn digestLines(a: std.mem.Allocator, output: []const u8) ![]const u8 {
    var found: std.ArrayList([]const u8) = .empty;
    var lines = std.mem.splitScalar(u8, output, '\n');
    while (lines.next()) |line| {
        const trimmed = std.mem.trimEnd(u8, line, "\r");
        if (std.mem.startsWith(u8, trimmed, "cook digest ")) try found.append(a, trimmed);
    }
    std.mem.sort([]const u8, found.items, {}, lessThan);
    var out: std.ArrayList(u8) = .empty;
    for (found.items) |line| try out.print(a, "{s}\n", .{line});
    return out.items;
}

fn lessThan(_: void, left: []const u8, right: []const u8) bool {
    return std.mem.lessThan(u8, left, right);
}

test "a docker runner names an image with its tag, then a platform" {
    const parsed = parseDocker("docker:debian:bookworm-slim:linux/amd64").?;
    try std.testing.expectEqualStrings("debian:bookworm-slim", parsed.image);
    try std.testing.expectEqualStrings("linux/amd64", parsed.platform);
    try std.testing.expect(parseDocker("docker:alpine") == null);
}

test "the digests are the sorted cook digest lines of a run" {
    var arena: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena.deinit();
    const output = "1/9 test...OK\r\ncook digest tiled    b\r\ncook digest authored c\ncook digest untiled  a\nAll passed.\n";
    try std.testing.expectEqualStrings("cook digest authored c\ncook digest tiled    b\ncook digest untiled  a\n", try digestLines(arena.allocator(), output));
}
