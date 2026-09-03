//! The README's Usage section, as a program that has to compile and run.
//!
//! `zig build examples` builds and executes this; ci/readme_usage.sh extracts
//! the marked region into README.md and ci/check-docs.sh refuses to let the
//! two drift apart. The geometry is a bare ground plane — the smallest input
//! that bakes, serialises and answers a path query.

const std = @import("std");
const zrecast = @import("zrecast");

pub fn main() !void {
    var gpa_state = std.heap.DebugAllocator(.{}){};
    defer _ = gpa_state.deinit();
    const gpa = gpa_state.allocator();

    // A 20 x 20 m ground plane: four corners, two triangles, Y up.
    const level_verts = [_]f32{
        -10, 0, -10,
        10,  0, -10,
        10,  0, 10,
        -10, 0, 10,
    };
    const level_indices = [_]i32{ 0, 2, 1, 0, 3, 2 };
    const agent_pos = [3]f32{ -8, 0, -8 };
    const target_pos = [3]f32{ 8, 0, 8 };

    // --- README:usage ---
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    // In the cook: geometry in, bytes out. `log` is optional, and the
    // difference between a diagnosable failure and a bare error code.
    var log: [1024]u8 = undefined;
    const poly = try zrecast.PolyMesh.bake(zrecast.defaultConfig(), .{
        .verts = &level_verts, // 3 floats per vertex, right-handed, Y up
        .tris = &level_indices, // 3 indices per triangle
    }, null, &log);
    defer poly.deinit();

    const baked = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer baked.deinit();
    const image = try baked.serialize();
    defer image.deinit();
    // ...write `image.bytes` wherever the pipeline puts build artefacts.

    // In the game: bytes in, answers out.
    const mesh = try zrecast.NavMesh.initFromBytes(image.bytes);
    defer mesh.deinit();
    const query = try zrecast.NavMeshQuery.init(mesh, 2048);
    defer query.deinit();

    const filter = zrecast.defaultFilter();
    const extents = [3]f32{ 2, 4, 2 };
    const from = try query.findNearestPoly(agent_pos, extents, &filter);
    const to = try query.findNearestPoly(target_pos, extents, &filter);
    if (from.ref == null or to.ref == null) return error.OffMesh;

    var corridor: [256]zrecast.PolyRef = undefined;
    const path = try query.findPath(
        from.ref.?,
        to.ref.?,
        from.point,
        to.point,
        &filter,
        &corridor,
    );

    var corners: [64][3]f32 = undefined;
    const walk = try query.findStraightPath(
        from.point,
        to.point,
        corridor[0..path.len],
        .{},
        &corners,
        null,
        null,
    );
    // Walk `corners[0..walk.len]`; `path.partial` says the goal was out of
    // reach and the corridor stops at the closest polygon instead.
    // --- README:usage ---

    if (walk.len < 2) return error.NoPath;
    if (path.partial) return error.PartialPath;
    std.debug.print(
        "zrecast usage: cooked {d} bytes, walked {d} corners\n",
        .{ image.bytes.len, walk.len },
    );
}
