//! Bridges a Zig `std.mem.Allocator` onto Recast's and Detour's allocator seam.
//!
//! ## Why this is not a two-line shim
//!
//! Both halves of upstream free with `free(ptr)` — no size, no alignment —
//! while Zig's allocator interface requires the size back. The gap is closed by
//! allocating a little extra and stashing the length in a header placed
//! immediately before the pointer handed to upstream:
//!
//! ```text
//!   base                       returned pointer
//!    |                          |
//!    v                          v
//!   [ ... padding ... ][Header][ ..... payload ..... ]
//!    \___ prefix, a multiple of `alignment` ___/
//! ```
//!
//! The prefix is rounded up to `alignment` so the returned pointer keeps that
//! alignment, and the base pointer is recoverable by subtracting the same
//! prefix at free time.
//!
//! ## One difference from the equivalent bridge over ozz-animation
//!
//! ozz asks for an alignment per allocation, so a bridge over it has to record
//! the alignment in the header too and recompute the prefix from it. Neither
//! `rcAlloc` nor `dtAlloc` has an alignment parameter — both are modelled on
//! `malloc` — so there is exactly one alignment here, it is a compile-time
//! constant, and storing it per block would be storing a constant. So the
//! header carries the length, and the tag `Header` explains.
//!
//! `alignment` is `ZRC_ALLOC_ALIGNMENT` from the C header, mirrored in
//! `c.alloc_alignment` and checked against the compiled library by the ABI test.
//!
//! ## Global state
//!
//! `rcAllocSetCustom` and `dtAllocSetCustom` each take a bare pair of function
//! pointers with nowhere to thread a host pointer through, so the seam is
//! process-wide. That is a property of upstream, not a shortcut taken here — it
//! is surfaced rather than hidden behind a per-object allocator parameter that
//! could not be honoured.

const std = @import("std");
const aegis = @import("aegis");
const c = @import("c.zig");
const err = @import("error.zig");

/// The one alignment every block is made with. See the module comment.
const alignment: std.mem.Alignment = .fromByteUnits(c.alloc_alignment);

/// Recorded ahead of every block so `deallocate` can reconstruct the slice
/// Zig's allocator needs — and tell such a block from a pointer into the
/// middle of one.
const Header = struct {
    /// Written by `allocate`, checked by `deallocate`. A length on its own
    /// cannot say whether it was read from a header at all: the bytes ahead
    /// of a pointer into the middle of a block are payload, and payload
    /// holding a plausible length is how a heap gets corrupted quietly. The
    /// tag makes the question answerable.
    tag: usize,
    /// Total bytes taken from the backing allocator, prefix included.
    total_len: usize,
};

/// The value every `Header.tag` holds. Arbitrary — its one job is to be a
/// value payload bytes are unlikely to hold at exactly that offset — and
/// truncated so its width follows `usize` instead of assuming a pointer size.
const header_tag: usize = @truncate(@as(u64, 0x7A72_635F_6864_7200));

/// Bytes reserved before the payload: enough for the header, rounded up so the
/// payload keeps `alignment`.
const prefix_size: usize = alignment.forward(@max(@sizeOf(Header), @alignOf(Header)));

comptime {
    // The header has to fit in the prefix, and the prefix has to preserve the
    // payload's alignment. Both hold for every plausible pointer width, but
    // they are the two invariants the arithmetic below depends on.
    std.debug.assert(prefix_size >= @sizeOf(Header));
    std.debug.assert(prefix_size % c.alloc_alignment == 0);
    std.debug.assert(@backingInt(alignment) >= @backingInt(std.mem.Alignment.of(Header)));
}

fn allocate(user: ?*anyopaque, size: usize, hint: c.AllocHint) callconv(.c) ?*anyopaque {
    // The hint says how long upstream expects to keep the block. Zig's
    // allocator interface has nowhere to put it, and inventing a second arena
    // for temporaries would be a policy this package has no business setting.
    _ = hint;

    const gpa = current(user orelse return null);
    const total = (aegis.int.Checked(usize).init(prefix_size).add(size) catch return null).raw();

    const base = gpa.rawAlloc(total, alignment, @returnAddress()) orelse return null;

    const payload = base + prefix_size;
    const header: *Header = @ptrCast(@alignCast(payload - @sizeOf(Header))); // aegis: no-danger: the header sits right below the payload, in a prefix that keeps it aligned
    header.* = .{ .tag = header_tag, .total_len = total };
    return @ptrCast(payload); // aegis: no-danger: the payload, as the seam's untyped pointer
}

fn deallocate(user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void {
    const gpa = current(user orelse return);
    const payload: [*]u8 = @ptrCast(block orelse return); // aegis: no-danger: a block is a byte pointer; its header is checked below before it is trusted

    // Only the pointer `allocate` returned can be freed. A pointer into the
    // middle of a block — a sub-slice of a serialised image, classically —
    // has payload where the header should be, so freeing on that basis would
    // hand the backing allocator a base and a length it never issued.
    const header = headerOf(payload) orelse return foreign();
    const base = payload - prefix_size;
    gpa.rawFree(base[0..header.total_len], alignment, @returnAddress());
}

/// The header `allocate` wrote ahead of `payload`, or null when the tag says
/// `payload` is not a pointer `allocate` returned.
fn headerOf(payload: [*]u8) ?*const Header {
    const header: *const Header = @ptrCast(@alignCast(payload - @sizeOf(Header))); // aegis: no-danger: the header sits right below the payload, in a prefix that keeps it aligned
    return if (header.tag == header_tag) header else null;
}

/// A free of a block the bridge did not issue is the caller's bug. A safe
/// build stops on it, naming it; a fast or small build leaves the block
/// alone, so it leaks and the heap the rest of the process runs on stays
/// intact. `tests/foreign_free.zig` holds both.
fn foreign() void {
    if (std.debug.runtime_safety) aegis.assert.never(foreign_free_message);
}

/// What a safe build stops with on a foreign free.
const foreign_free_message = "zrecast: a block was freed that the installed allocator did not issue (a sub-slice of one, or a block from another allocator)";

/// The allocator upstream is currently pointed at, kept alive for as long as it
/// is installed. `user` in the C struct points at this. Upstream's hook is
/// process-wide and allocates on whichever thread bakes or loads, so the
/// allocator sits behind a lock, held only to copy it out.
var installed: aegis.Guarded(std.mem.Allocator) = .init(undefined);

/// The allocator `user` points at, read under its lock.
fn current(user: *anyopaque) std.mem.Allocator {
    const owner: *aegis.Guarded(std.mem.Allocator) = @ptrCast(@alignCast(user)); // aegis: no-danger: user is &installed, which setAllocator hands the C side
    var held = owner.acquire();
    defer held.deinit();
    return held.value().*;
}

/// Routes every subsequent Recast and Detour allocation through `gpa`.
///
/// Process-wide. Call once during start-up, before baking or loading anything,
/// and do not swap it while handles are alive — a handle is freed through
/// whichever allocator is installed when it is destroyed.
pub fn setAllocator(gpa: std.mem.Allocator) void {
    {
        var held = installed.acquire();
        defer held.deinit();
        held.value().* = gpa;
    }
    const bridge = c.Allocator{
        .allocate = allocate,
        .deallocate = deallocate,
        .user = @ptrCast(&installed), // aegis: no-danger: the guarded allocator's address, which allocate and deallocate read back
    };
    // The boundary refuses only a bridge missing a function, and this one
    // has both.
    err.map(c.zrcSetAllocator(&bridge)) catch aegis.assert.never("the Zig bridge has both functions");
}

/// Restores upstream's built-in malloc/free allocators.
///
/// Only safe once every handle allocated through the Zig allocator has been
/// destroyed.
pub fn resetAllocator() void {
    _ = c.zrcSetAllocator(null);
}

/// The lifetime hint an allocation carries through the seam. Passed through
/// to whichever allocator is installed; a host may use it to pick an arena,
/// or ignore it entirely.
pub const AllocHint = c.AllocHint;

/// Allocates through whatever allocator is installed — the Zig bridge above
/// if `setAllocator` was called, upstream's own malloc otherwise. `hint` is
/// upstream's own lifetime hint, passed straight through.
///
/// Reachable so a host already accounting for Recast's and Detour's own
/// allocations can make one on the same seam and have it accounted the same
/// way — scratch for `geom.randomPointInConvexPoly`, say.
pub fn alloc(size: usize, hint: AllocHint, failure: *err.Failure) err.Error![]u8 {
    const p = c.zrcAlloc(size, hint) orelse return err.refuse(failure, .alloc, error.OutOfMemory);
    const bytes: [*]u8 = @ptrCast(p); // aegis: no-danger: zrcAlloc returns a block of size bytes
    return bytes[0..size];
}

/// Releases a block from `alloc` back to whichever allocator is installed.
///
/// `block` must be the slice `alloc` returned and not a sub-slice of it: with
/// a Zig allocator installed, the bridge keeps a private header immediately
/// ahead of each block, and only the pointer `alloc` handed back sits where
/// that header can be found. The bridge never frees a pointer it did not
/// issue: a Debug or ReleaseSafe build stops on it, and a ReleaseFast or
/// ReleaseSmall build leaks the block. Upstream's own malloc, in use when
/// `setAllocator` was never called, has no such guard.
///
/// A serialised navmesh image does not come back through here: those arrive
/// as `navmesh.Serialized`, which owns its slice and releases it in `deinit`.
pub fn free(block: []u8) void {
    c.zrcFree(block.ptr);
}

test "the bridge round-trips a range of sizes and keeps the promised alignment" {
    const gpa = std.testing.allocator;
    setAllocator(gpa);
    defer resetAllocator();

    var size: usize = 1;
    while (size <= 4096) : (size *= 3) {
        const block = allocate(@ptrCast(&installed), size, .perm) orelse {
            return error.TestUnexpectedResult;
        };
        try std.testing.expect(@intFromPtr(block) % c.alloc_alignment == 0);
        // Write the payload so a too-small allocation trips the test allocator.
        const bytes: [*]u8 = @ptrCast(block);
        @memset(bytes[0..size], 0xAB);
        deallocate(@ptrCast(&installed), block);
    }
}

test "the bridge tolerates a zero-size request and a null free" {
    const gpa = std.testing.allocator;
    setAllocator(gpa);
    defer resetAllocator();

    const block = allocate(@ptrCast(&installed), 0, .temp) orelse {
        return error.TestUnexpectedResult;
    };
    deallocate(@ptrCast(&installed), block);
    deallocate(@ptrCast(&installed), null);
}

test "installing an incomplete allocator is refused" {
    // Straight at the C entry point: half a seam is worse than none, because
    // upstream would allocate through the host and free through malloc.
    const half = c.Allocator{
        .allocate = allocate,
        .deallocate = null,
        .user = null,
    };
    try std.testing.expectEqual(c.Result.invalid_argument, c.zrcSetAllocator(&half));
}

test "alloc and free round-trip through the installed allocator" {
    var failure: err.Failure = .{};
    const gpa = std.testing.allocator;
    setAllocator(gpa);
    defer resetAllocator();

    const block = try alloc(128, .perm, &failure);
    try std.testing.expectEqual(@as(usize, 128), block.len);
    @memset(block, 0xCD);
    free(block);
}

test "the bridge refuses a pointer it did not hand out" {
    // `block[8..]` is the mistake: payload sits where the header would be, so
    // freeing on that basis asks the backing allocator for a region it never
    // issued. The recorder stands in for that allocator, which makes the
    // attempt observable instead of destructive.
    const Recorder = struct {
        buffer: [512]u8 align(c.alloc_alignment) = undefined,
        used: usize = 0,
        frees: usize = 0,
        last: []u8 = &.{},

        fn rawAlloc(ctx: *anyopaque, len: usize, a: std.mem.Alignment, _: usize) ?[*]u8 {
            const self: *@This() = @ptrCast(@alignCast(ctx));
            const start = a.forward(self.used);
            if (start + len > self.buffer.len) return null;
            self.used = start + len;
            return self.buffer[start..].ptr;
        }

        fn rawResize(_: *anyopaque, _: []u8, _: std.mem.Alignment, _: usize, _: usize) bool {
            return false;
        }

        fn rawRemap(_: *anyopaque, _: []u8, _: std.mem.Alignment, _: usize, _: usize) ?[*]u8 {
            return null;
        }

        fn rawFree(ctx: *anyopaque, memory: []u8, _: std.mem.Alignment, _: usize) void {
            const self: *@This() = @ptrCast(@alignCast(ctx));
            self.frees += 1;
            self.last = memory;
        }

        const vtable: std.mem.Allocator.VTable = .{
            .alloc = rawAlloc,
            .resize = rawResize,
            .remap = rawRemap,
            .free = rawFree,
        };
    };

    var recorder: Recorder = .{};
    setAllocator(.{ .ptr = &recorder, .vtable = &Recorder.vtable });
    defer resetAllocator();

    const size = 64;
    const block = allocate(@ptrCast(&installed), size, .perm) orelse {
        return error.TestUnexpectedResult;
    };
    const bytes: [*]u8 = @ptrCast(block);
    // Zeroed payload, so the shifted read below cannot find the tag by luck.
    @memset(bytes[0..size], 0);

    // The tag tells the two apart. What a free of the shifted pointer then
    // does depends on the build: a safe one stops, which tests/foreign_free.zig
    // holds out of process; a fast or small one leaves the backing allocator
    // untouched, which this holds.
    try std.testing.expect(headerOf(bytes + 8) == null);
    try std.testing.expect(headerOf(bytes) != null);
    if (!std.debug.runtime_safety) {
        deallocate(@ptrCast(&installed), bytes + 8);
        try std.testing.expectEqual(@as(usize, 0), recorder.frees);
    }

    // The pointer `allocate` returned still frees, and frees the whole block.
    deallocate(@ptrCast(&installed), block);
    try std.testing.expectEqual(@as(usize, 1), recorder.frees);
    try std.testing.expectEqual(bytes - prefix_size, recorder.last.ptr);
    try std.testing.expectEqual(prefix_size + size, recorder.last.len);
}
