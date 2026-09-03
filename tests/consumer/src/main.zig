//! Drives zrecast the way the README's example does, but from a separate
//! package that got here through `b.dependency`.

const std = @import("std");
const zrecast = @import("zrecast");

pub fn main() !void {
    var gpa_state = std.heap.DebugAllocator(.{}){};
    defer _ = gpa_state.deinit();
    const gpa = gpa_state.allocator();

    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const verts = [_]f32{
        -10, 0, -10,
        10,  0, -10,
        10,  0, 10,
        -10, 0, 10,
    };
    const tris = [_]i32{ 0, 2, 1, 0, 3, 2 };

    const poly = try zrecast.PolyMesh.bake(
        zrecast.defaultConfig(),
        .{ .verts = &verts, .tris = &tris },
        null,
        null,
    );
    defer poly.deinit();

    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    if (image.bytes.len == 0) return error.EmptyImage;

    const query = try zrecast.NavMeshQuery.init(mesh, 256);
    defer query.deinit();
    const filter = zrecast.defaultFilter();
    const near = try query.findNearestPoly(
        .{ 0, 0, 0 },
        .{ 2, 4, 2 },
        &filter,
    );
    if (near.ref == null) return error.OffMesh;

    std.debug.print("zig consumer: ok ({d}-byte image)\n", .{image.bytes.len});
}
