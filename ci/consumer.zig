//! What a project that depends on zrecast writes: bake a ground plane, build
//! a navmesh, serialise it and find the polygon under a point.
const std = @import("std");
const zrecast = @import("zrecast");

pub fn main(init: std.process.Init) !void {
    zrecast.setAllocator(init.gpa);
    defer zrecast.resetAllocator();
    var failure: zrecast.Failure = .{};

    const verts = [_]f32{ -10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10 };
    const tris = [_]i32{ 0, 2, 1, 0, 3, 2 };
    const poly = try zrecast.PolyMesh.bake(zrecast.defaultConfig(), .{ .verts = &verts, .tris = &tris }, null, null, &failure);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null, &failure);
    defer mesh.deinit();
    const image = try mesh.serialize(&failure);
    defer image.deinit();
    if (image.bytes.len == 0) return error.EmptyImage;

    const query = try zrecast.NavMeshQuery.init(mesh, 256, &failure);
    defer query.deinit();
    const filter = try zrecast.QueryFilter.initDefault(&failure);
    defer filter.deinit();
    const near = try query.findNearestPoly(.{ 0, 0, 0 }, .{ 2, 4, 2 }, filter, &failure);
    if (near.ref == null) return error.OffMesh;
}
