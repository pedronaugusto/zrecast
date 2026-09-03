//! Slice-shaped wrappers over the fourteen geometry entry points in
//! `ffi/zrecast.h`'s "Geometry primitives" section.
//!
//! The C side takes a pointer and a count that nothing proves agree with
//! each other; this side derives every count from a slice's own length, so a
//! polygon array and the number describing it cannot fall out of step.

const std = @import("std");
const zc = @import("c.zig");
const err = @import("error.zig");
const Error = err.Error;

pub const Vec3 = @import("vec.zig").Vec3;

/// What `distancePointToSegment2D` found: the squared distance, and where
/// along the segment the nearest point fell.
pub const SegmentDistance = struct { dist_sqr: f32, t: f32 };

/// Where a segment crosses a convex polygon's boundary, from
/// `intersectSegmentPoly2D`.
pub const SegmentPolyHit = struct {
    intersects: bool,
    t_min: f32,
    t_max: f32,
    seg_min: i32,
    seg_max: i32,
};

/// Where two segments cross, from `intersectSegSeg2D`.
pub const SegSegHit = struct { s: f32, t: f32 };

/// Closest point on triangle a-b-c to `point`, in 3D.
///
/// A degenerate triangle — three collinear or coincident vertices — can make
/// upstream's barycentric division produce a non-finite result, reported as
/// it comes rather than refused.
pub fn closestPointOnTriangle(point: Vec3, a: Vec3, b: Vec3, c: Vec3) Error!Vec3 {
    var out: Vec3 = undefined;
    try err.check(zc.zrcClosestPointOnTriangle(&point, &a, &b, &c, &out));
    return out;
}

/// Height of triangle a-b-c directly above or below `point`, on the
/// xz-plane, or `null` when the point is not inside the triangle at all.
///
/// A triangle degenerate on the xz-plane — its projected area under 1e-6 — is
/// not inside anything and answers `null`.
pub fn closestHeightPointTriangle(point: Vec3, a: Vec3, b: Vec3, c: Vec3) Error!?f32 {
    var height: f32 = undefined;
    var inside: zc.Bool = zc.c_false;
    try err.check(zc.zrcClosestHeightPointTriangle(&point, &a, &b, &c, &height, &inside));
    if (inside == zc.c_false) return null;
    return height;
}

/// Squared distance from `point` to the segment p-q, on the xz-plane. `t` is
/// where the nearest point falls along the segment, clamped to `[0, 1]`; a
/// zero-length segment reports `t = 0`.
pub fn distancePointToSegment2D(point: Vec3, p: Vec3, q: Vec3) Error!SegmentDistance {
    var dist_sqr: f32 = undefined;
    var t: f32 = undefined;
    try err.check(zc.zrcDistancePointToSegment2D(&point, &p, &q, &dist_sqr, &t));
    return .{ .dist_sqr = dist_sqr, .t = t };
}

/// Squared distance from `point` to every edge of a polygon, on the
/// xz-plane, and whether the point is inside it. Entry `i` of each output is
/// the edge starting at `verts[i]`.
///
/// `out_edge_dist_sqr` and `out_edge_t` must each be at least `verts.len`
/// long, checked here rather than left to the C side: `zrcDistancePointToPolyEdges`
/// takes bare pointers and cannot tell a short buffer from a long one.
pub fn distancePointToPolyEdges(
    point: Vec3,
    verts: []const Vec3,
    out_edge_dist_sqr: []f32,
    out_edge_t: []f32,
) Error!bool {
    const vert_count = std.math.cast(i32, verts.len) orelse return Error.InvalidArgument;
    if (out_edge_dist_sqr.len < verts.len or out_edge_t.len < verts.len) {
        return Error.InvalidArgument;
    }
    var inside: zc.Bool = zc.c_false;
    try err.check(zc.zrcDistancePointToPolyEdges(
        &point,
        @ptrCast(verts.ptr),
        vert_count,
        out_edge_dist_sqr.ptr,
        out_edge_t.ptr,
        &inside,
    ));
    return inside != zc.c_false;
}

/// Whether `point` lies inside a polygon, on the xz-plane.
pub fn pointInPolygon(point: Vec3, verts: []const Vec3) Error!bool {
    const vert_count = std.math.cast(i32, verts.len) orelse return Error.InvalidArgument;
    var inside: zc.Bool = zc.c_false;
    try err.check(zc.zrcPointInPolygon(&point, @ptrCast(verts.ptr), vert_count, &inside));
    return inside != zc.c_false;
}

/// Where the segment p0-p1 enters and leaves a convex polygon, on the
/// xz-plane. `t_min` and `t_max` are always written: a segment wholly inside
/// the polygon reports 0 and 1. `seg_min` and `seg_max` are the polygon edge
/// crossed at each end, or -1 where that endpoint is inside.
pub fn intersectSegmentPoly2D(p0: Vec3, p1: Vec3, verts: []const Vec3) Error!SegmentPolyHit {
    const vert_count = std.math.cast(i32, verts.len) orelse return Error.InvalidArgument;
    var t_min: f32 = undefined;
    var t_max: f32 = undefined;
    var seg_min: i32 = undefined;
    var seg_max: i32 = undefined;
    var intersects: zc.Bool = zc.c_false;
    try err.check(zc.zrcIntersectSegmentPoly2D(
        &p0,
        &p1,
        @ptrCast(verts.ptr),
        vert_count,
        &t_min,
        &t_max,
        &seg_min,
        &seg_max,
        &intersects,
    ));
    return .{
        .intersects = intersects != zc.c_false,
        .t_min = t_min,
        .t_max = t_max,
        .seg_min = seg_min,
        .seg_max = seg_max,
    };
}

/// Where two segments cross, on the xz-plane. `null` for parallel or
/// near-parallel lines, rather than a struct with a flag; otherwise `s` and
/// `t` are fractions along ap-aq and bp-bq, not clamped to `[0, 1]`, so a
/// value outside that range means the infinite lines cross beyond the
/// segment's end.
pub fn intersectSegSeg2D(ap: Vec3, aq: Vec3, bp: Vec3, bq: Vec3) Error!?SegSegHit {
    var s: f32 = undefined;
    var t: f32 = undefined;
    var intersects: zc.Bool = zc.c_false;
    try err.check(zc.zrcIntersectSegSeg2D(&ap, &aq, &bp, &bq, &s, &t, &intersects));
    if (intersects == zc.c_false) return null;
    return .{ .s = s, .t = t };
}

/// Whether two convex polygons overlap, on the xz-plane, by separating axis.
pub fn overlapPolyPoly2D(a: []const Vec3, b: []const Vec3) Error!bool {
    const count_a = std.math.cast(i32, a.len) orelse return Error.InvalidArgument;
    const count_b = std.math.cast(i32, b.len) orelse return Error.InvalidArgument;
    var overlap: zc.Bool = zc.c_false;
    try err.check(zc.zrcOverlapPolyPoly2D(
        @ptrCast(a.ptr),
        count_a,
        @ptrCast(b.ptr),
        count_b,
        &overlap,
    ));
    return overlap != zc.c_false;
}

/// Whether two axis-aligned boxes overlap. Touching counts as overlapping.
pub fn overlapBounds(amin: Vec3, amax: Vec3, bmin: Vec3, bmax: Vec3) Error!bool {
    var overlap: zc.Bool = zc.c_false;
    try err.check(zc.zrcOverlapBounds(&amin, &amax, &bmin, &bmax, &overlap));
    return overlap != zc.c_false;
}

/// Whether two boxes in a tile's quantised space overlap — the test a
/// bounding-volume tree traversal runs.
pub fn overlapQuantBounds(amin: [3]u16, amax: [3]u16, bmin: [3]u16, bmax: [3]u16) Error!bool {
    var overlap: zc.Bool = zc.c_false;
    try err.check(zc.zrcOverlapQuantBounds(&amin, &amax, &bmin, &bmax, &overlap));
    return overlap != zc.c_false;
}

/// Twice the signed area of triangle a-b-c on the xz-plane. Positive when c
/// lies to the left of the line a-b looking from a toward b, which is
/// Detour's own polygon winding direction.
pub fn triArea2D(a: Vec3, b: Vec3, c: Vec3) Error!f32 {
    var area: f32 = undefined;
    try err.check(zc.zrcTriArea2D(&a, &b, &c, &area));
    return area;
}

/// Centroid of the polygon formed by indexing `verts` with `indices`. Every
/// index must be less than `verts.len`.
pub fn polyCenter(verts: []const Vec3, indices: []const u16) Error!Vec3 {
    const vert_count = std.math.cast(i32, verts.len) orelse return Error.InvalidArgument;
    const index_count = std.math.cast(i32, indices.len) orelse return Error.InvalidArgument;
    var out: Vec3 = undefined;
    try err.check(zc.zrcPolyCenter(@ptrCast(verts.ptr), vert_count, indices.ptr, index_count, &out));
    return out;
}

/// A point inside a convex polygon, chosen from two numbers in `[0, 1]`: `s`
/// picks a triangle of the fan weighted by area, `t` picks a point within
/// it, so a placement is reproducible from a seed.
///
/// `scratch` is working storage for the fan's areas and must be at least
/// `verts.len` long; its contents on return are unspecified.
pub fn randomPointInConvexPoly(verts: []const Vec3, scratch: []f32, s: f32, t: f32) Error!Vec3 {
    const vert_count = std.math.cast(i32, verts.len) orelse return Error.InvalidArgument;
    const scratch_count = std.math.cast(i32, scratch.len) orelse return Error.InvalidArgument;
    var out: Vec3 = undefined;
    try err.check(zc.zrcRandomPointInConvexPoly(
        @ptrCast(verts.ptr),
        vert_count,
        scratch.ptr,
        scratch_count,
        s,
        t,
        &out,
    ));
    return out;
}

/// Inflates or shrinks a convex polygon by `offset`, on the xz-plane, into
/// `out`, and returns how many vertices were written. A negative offset
/// shrinks; sharp corners are bevelled, so the result can have up to
/// `2 * verts.len` vertices, and y is carried through from the source vertex
/// unchanged.
///
/// Upstream's own bound refuses to fill the last slot of the buffer it is
/// given, so `out` must be at least one longer than the result for the call
/// to succeed. A result that would exactly fill `out` is reported as
/// `Error.BufferTooSmall` rather than as a short write.
pub fn offsetPoly(verts: []const Vec3, offset: f32, out: []Vec3) Error!usize {
    const vert_count = std.math.cast(i32, verts.len) orelse return Error.InvalidArgument;
    const max_out_verts = std.math.cast(i32, out.len) orelse return Error.InvalidArgument;
    var written: i32 = undefined;
    try err.check(zc.zrcOffsetPoly(
        @ptrCast(verts.ptr),
        vert_count,
        offset,
        @ptrCast(out.ptr),
        max_out_verts,
        &written,
    ));
    return @intCast(written);
}

test "an empty polygon is rejected before Recast ever sees it" {
    try std.testing.expectError(Error.InvalidArgument, pointInPolygon(.{ 0, 0, 0 }, &.{}));
}

test "polyCenter rejects an index past the end of the vertex array" {
    const verts = [_]Vec3{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 1 } };
    const bad_indices = [_]u16{ 0, 1, 3 };
    try std.testing.expectError(Error.InvalidArgument, polyCenter(&verts, &bad_indices));
}

test "distancePointToPolyEdges rejects an output slice shorter than the polygon" {
    const verts = [_]Vec3{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 1 } };
    var short_dist_sqr: [2]f32 = undefined;
    var full_t: [3]f32 = undefined;
    try std.testing.expectError(
        Error.InvalidArgument,
        distancePointToPolyEdges(.{ 0.1, 0, 0.1 }, &verts, &short_dist_sqr, &full_t),
    );
}
