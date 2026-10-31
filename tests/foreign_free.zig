//! A free through the allocator seam of a block the installed allocator did
//! not issue, here a sub-slice of one it did. A safe build has to stop on it;
//! a fast or small build has to leave the block alone and carry on. build.zig
//! runs this and holds it to the one its mode promises: a stop naming the
//! mistake in Debug and ReleaseSafe, a clean exit otherwise. Out of process,
//! because a stop cannot be observed from inside the program that stops.
const std = @import("std");
const zrecast = @import("zrecast");

pub const panic = std.debug.FullPanic(stop);

/// Prints the panic and exits with a code no other way out of this program
/// takes, so the run step can tell this stop from an error return.
fn stop(message: []const u8, _: ?usize) noreturn {
    std.debug.print("{s}\n", .{message});
    std.process.exit(stopped);
}

/// The exit code a stop leaves; build.zig expects it in a safe build.
const stopped = 86;

pub fn main(init: std.process.Init) !void {
    zrecast.setAllocator(init.gpa);
    defer zrecast.resetAllocator();
    var failure: zrecast.Failure = .{};
    const block = try zrecast.alloc(64, .perm, &failure);
    defer zrecast.free(block);
    zrecast.free(block[8..]);
}
