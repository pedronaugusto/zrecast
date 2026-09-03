//! Recast's bake pipeline taken apart into stages: a heightfield, a compact
//! heightfield, a contour set and a polygon mesh, each a handle the host
//! owns and can inspect, edit or replace between one call and the next.
//!
//! `bake.PolyMesh.bake` runs this same pipeline end to end; this file is for
//! a cook that wants to stop between stages — mark areas from its own data
//! after erosion, draw the heightfield, reuse one compact heightfield for
//! several region strategies.
//!
//! Every stage below refuses a container that already holds a result rather
//! than overwriting its buffers silently, the rule ffi/zrecast.h states for
//! the whole pipeline.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const memory = @import("memory.zig");
const PolyMesh = @import("bake.zig").PolyMesh;
const TriMesh = @import("bake.zig").TriMesh;
const AreaAuthoring = @import("bake.zig").AreaAuthoring;

//===----------------------------------------------------------------------===//
// The build context
//===----------------------------------------------------------------------===//

/// Which of Recast's build messages a log entry is.
pub const LogCategory = c.LogCategory;

/// The build phases Recast times, and the count that bounds them. Every
/// stage function below starts and stops the labels its upstream function
/// does, so a host that installs timers measures the same phases a C++ host
/// measures.
pub const TimerLabel = c.TimerLabel;

/// Where a build's log messages and timings go: the same seam as upstream's
/// `rcContext`, restated as a POD rather than a class with six protected
/// virtuals.
///
/// Every hook may be `null` individually, and the whole context may be
/// `null` wherever one is asked for — a null context has both flags clear,
/// while a default-initialised one has both set, the same as upstream's own
/// `rcContext()`. The flags gate more than convenience: `accumulatedTime`
/// answers -1 without calling the hook at all when `timers_enabled` is
/// false, the same as upstream.
///
/// A `log` message over 511 bytes arrives as two calls: first an error
/// reading "Log message was truncated", at a category the caller did not
/// choose, then the message cut to 511 bytes.
pub const BuildContext = struct {
    /// Passed back to every hook untouched.
    user: ?*anyopaque = null,
    log: ?*const fn (user: ?*anyopaque, category: LogCategory, message: [*:0]const u8, length: i32) callconv(.c) void = null,
    /// Discards whatever the log has accumulated.
    reset_log: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
    /// Resets every timer to unused.
    reset_timers: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
    start_timer: ?*const fn (user: ?*anyopaque, label: TimerLabel) callconv(.c) void = null,
    stop_timer: ?*const fn (user: ?*anyopaque, label: TimerLabel) callconv(.c) void = null,
    /// Total time accumulated on a label, in microseconds, or -1 for a timer
    /// that has never run.
    accumulated_time: ?*const fn (user: ?*anyopaque, label: TimerLabel) callconv(.c) i32 = null,
    /// Clear to silence `log` and `reset_log` without removing them.
    ///
    /// Defaulted on, matching upstream's own `rcContext()` constructor rather
    /// than the zeroed C struct: a host that installs a hook and gets silence
    /// has the same trouble a flagless polygon gives a query. The C side
    /// cannot default this way, because a zeroed ZrcBuildContext has to be a
    /// valid one.
    log_enabled: bool = true,
    /// Clear to silence the four timer hooks; `accumulatedTime` then always
    /// answers -1. Defaulted on for the same reason.
    timers_enabled: bool = true,

    /// The `c.BuildContext` upstream's C ABI reads, with the two flags
    /// widened to its 32-bit bool.
    pub fn toC(self: BuildContext) c.BuildContext {
        return .{
            .user = self.user,
            .log = self.log,
            .reset_log = self.reset_log,
            .reset_timers = self.reset_timers,
            .start_timer = self.start_timer,
            .stop_timer = self.stop_timer,
            .accumulated_time = self.accumulated_time,
            .log_enabled = if (self.log_enabled) c.c_true else c.c_false,
            .timers_enabled = if (self.timers_enabled) c.c_true else c.c_false,
        };
    }
};

/// Converts `context` for one C call, using `storage` to hold the widened
/// struct for the duration of that call. `null` in is `null` out.
fn toCContext(context: ?*const BuildContext, storage: *c.BuildContext) ?*const c.BuildContext {
    const ctx = context orelse return null;
    storage.* = ctx.toC();
    return storage;
}

/// Logs one message through `context`, as a Recast stage would. `message`
/// crosses already NUL-terminated: a varargs entry point cannot be checked
/// at this boundary, so formatting is the caller's.
pub fn log(context: ?*const BuildContext, category: LogCategory, message: [:0]const u8) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcBuildContextLog(toCContext(context, &context_storage), category, message.ptr));
}

pub fn resetLog(context: ?*const BuildContext) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcBuildContextResetLog(toCContext(context, &context_storage)));
}

pub fn resetTimers(context: ?*const BuildContext) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcBuildContextResetTimers(toCContext(context, &context_storage)));
}

pub fn startTimer(context: ?*const BuildContext, label: TimerLabel) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcBuildContextStartTimer(toCContext(context, &context_storage), label));
}

pub fn stopTimer(context: ?*const BuildContext, label: TimerLabel) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcBuildContextStopTimer(toCContext(context, &context_storage), label));
}

/// Microseconds accumulated on `label`, or -1 when `context`'s timers are
/// disabled or `label` has never run.
pub fn accumulatedTime(context: ?*const BuildContext, label: TimerLabel) err.Error!i32 {
    var context_storage: c.BuildContext = undefined;
    var out: i32 = 0;
    try err.check(c.zrcBuildContextAccumulatedTime(toCContext(context, &context_storage), label, &out));
    return out;
}

//===----------------------------------------------------------------------===//
// Sizing a build
//===----------------------------------------------------------------------===//

/// Cell dimensions `calcGridSize` computes and `Heightfield.init` accepts.
pub const GridSize = struct { width: i32, height: i32 };

/// Axis-aligned bounds of `mesh`'s vertices, `.{ min, max }`.
pub fn calcBounds(mesh: TriMesh) err.Error![2][3]f32 {
    const c_mesh = try mesh.toC();
    var result: [2][3]f32 = undefined;
    try err.check(c.zrcCalcBounds(&c_mesh, &result[0], &result[1]));
    return result;
}

/// Width and height, in cells, of the voxel grid `bmin`..`bmax` implies at
/// `cell_size`.
pub fn calcGridSize(bmin: [3]f32, bmax: [3]f32, cell_size: f32) err.Error!GridSize {
    var size: GridSize = undefined;
    try err.check(c.zrcCalcGridSize(&bmin, &bmax, cell_size, &size.width, &size.height));
    return size;
}

/// Writes `area_walkable` into `out_areas` for every triangle of `mesh`
/// whose slope is within `walkable_slope_angle`, leaving the rest
/// untouched.
///
/// `out_areas` is one byte per triangle: its length must equal `mesh`'s
/// triangle count, checked here since the C side only ever sees a bare
/// pointer. Untouched means untouched — zero the array first, or reuse one
/// already marked.
pub fn markWalkableTriangles(
    context: ?*const BuildContext,
    walkable_slope_angle: f32,
    mesh: TriMesh,
    out_areas: []u8,
) err.Error!void {
    const c_mesh = try mesh.toC();
    if (out_areas.len != @as(usize, @intCast(c_mesh.tri_count))) return err.Error.InvalidArgument;
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcMarkWalkableTriangles(
        toCContext(context, &context_storage),
        walkable_slope_angle,
        &c_mesh,
        out_areas.ptr,
    ));
}

/// Writes `area_null` into `io_areas` for every triangle of `mesh` steeper
/// than `walkable_slope_angle`, leaving the rest untouched — the inverse of
/// `markWalkableTriangles`, for an array already filled in. Same length
/// rule as `markWalkableTriangles`.
pub fn clearUnwalkableTriangles(
    context: ?*const BuildContext,
    walkable_slope_angle: f32,
    mesh: TriMesh,
    io_areas: []u8,
) err.Error!void {
    const c_mesh = try mesh.toC();
    if (io_areas.len != @as(usize, @intCast(c_mesh.tri_count))) return err.Error.InvalidArgument;
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcClearUnwalkableTriangles(
        toCContext(context, &context_storage),
        walkable_slope_angle,
        &c_mesh,
        io_areas.ptr,
    ));
}

//===----------------------------------------------------------------------===//
// The heightfield
//===----------------------------------------------------------------------===//

/// A voxel column's obstructed interval, copied out of upstream's packed
/// span-plus-pointer representation.
pub const Span = c.Span;

pub const HeightfieldInfo = c.HeightfieldInfo;

/// How much span storage a heightfield holds. Spans are handed out from
/// pools of `c.spans_per_pool` and returned to a free list rather than to
/// the allocator, so a field's memory is `pool_count * spans_per_pool`
/// spans however many are live; `free_count` is how many sit on that list.
pub const HeightfieldStorage = c.HeightfieldStorage;

/// The voxelisation of the input geometry: obstructed spans per column over
/// a `width` x `height` grid.
pub const Heightfield = struct {
    handle: *c.Heightfield,

    /// Allocates a `width` x `height` field over `bmin`..`bmax`, the same
    /// grid `calcGridSize` computes and the same bounds it applies.
    ///
    /// Upstream allocates `width * height` span pointers from their product
    /// in plain `int` arithmetic, which two large sizes overflow before the
    /// allocation is even attempted; `width * height` above 268435456 is
    /// refused. The vertical extent is bounded too, separately:
    /// `(bmax[1] - bmin[1]) / cell_height` must be under 8191, all a span's
    /// 13-bit extent can address, since nothing else catches a height that
    /// overflows it.
    pub fn init(
        context: ?*const BuildContext,
        width: i32,
        height: i32,
        bmin: [3]f32,
        bmax: [3]f32,
        cell_size: f32,
        cell_height: f32,
    ) err.Error!Heightfield {
        var context_storage: c.BuildContext = undefined;
        var handle: *c.Heightfield = undefined;
        try err.check(c.zrcHeightfieldCreate(
            toCContext(context, &context_storage),
            width,
            height,
            &bmin,
            &bmax,
            cell_size,
            cell_height,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Heightfield) void {
        c.zrcHeightfieldDestroy(self.handle);
    }

    pub fn info(self: Heightfield) err.Error!HeightfieldInfo {
        var out: HeightfieldInfo = undefined;
        try err.check(c.zrcHeightfieldInfo(self.handle, &out));
        return out;
    }

    pub fn storage(self: Heightfield) err.Error!HeightfieldStorage {
        var out: HeightfieldStorage = undefined;
        try err.check(c.zrcHeightfieldStorage(self.handle, &out));
        return out;
    }

    /// Copies the column at (`x`, `z`), lowest span first, into `out`, and
    /// returns how many spans the column holds in total.
    ///
    /// `out` receives however many fit; the total is always the return
    /// value, written whether or not every span fit, so a caller compares
    /// it against `out.len` to see whether a second, larger call is needed
    /// rather than treating a short column as failure. An empty `out` asks
    /// only for the count.
    pub fn column(self: Heightfield, x: i32, z: i32, out: []Span) err.Error!usize {
        const max_spans = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]Span = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        const result = c.zrcHeightfieldColumn(self.handle, x, z, ptr, max_spans, &count);
        if (result == .buffer_too_small) return @intCast(count);
        try err.check(result);
        return @intCast(count);
    }

    /// Total spans across every column. Walks the whole field.
    pub fn spanCount(self: Heightfield, context: ?*const BuildContext) err.Error!u32 {
        var context_storage: c.BuildContext = undefined;
        var out: i32 = 0;
        try err.check(c.zrcHeightfieldSpanCount(toCContext(context, &context_storage), self.handle, &out));
        return @intCast(out);
    }

    /// Adds one span to the column at (`x`, `z`), merging it with any it
    /// overlaps. `flag_merge_threshold` is how close in cells two merged
    /// spans' upper extents must be for the higher one's area to win.
    pub fn addSpan(
        self: Heightfield,
        context: ?*const BuildContext,
        x: i32,
        z: i32,
        span_min: u32,
        span_max: u32,
        area: u8,
        flag_merge_threshold: i32,
    ) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldAddSpan(
            toCContext(context, &context_storage),
            self.handle,
            x,
            z,
            span_min,
            span_max,
            area,
            flag_merge_threshold,
        ));
    }

    /// Rasterises one triangle. Every vertex must sit at a distance from
    /// the field's minimum corner that divides by `cell_size` into a cell
    /// index: a triangle wholly outside the field is skipped, but one that
    /// straddles it reaches an out-of-range float-to-int conversion, so a
    /// finite coordinate alone is not enough.
    pub fn rasterizeTriangle(
        self: Heightfield,
        context: ?*const BuildContext,
        v0: [3]f32,
        v1: [3]f32,
        v2: [3]f32,
        area: u8,
        flag_merge_threshold: i32,
    ) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldRasterizeTriangle(
            toCContext(context, &context_storage),
            self.handle,
            &v0,
            &v1,
            &v2,
            area,
            flag_merge_threshold,
        ));
    }

    /// Rasterises an indexed mesh, one area byte per triangle. `tri_areas`
    /// must be exactly `mesh`'s triangle count long, checked here since the
    /// C side only ever sees a bare pointer. Each area must be below
    /// `c.max_areas`.
    pub fn rasterizeTriangles(
        self: Heightfield,
        context: ?*const BuildContext,
        mesh: TriMesh,
        tri_areas: []const u8,
        flag_merge_threshold: i32,
    ) err.Error!void {
        const c_mesh = try mesh.toC();
        if (tri_areas.len != @as(usize, @intCast(c_mesh.tri_count))) return err.Error.InvalidArgument;
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldRasterizeTriangles(
            toCContext(context, &context_storage),
            self.handle,
            &c_mesh,
            tri_areas.ptr,
            flag_merge_threshold,
        ));
    }

    /// Rasterises an indexed mesh whose indices are 16 bits wide. `verts`
    /// and `tris` must each divide evenly by 3, and `tri_areas` must be
    /// exactly `tris.len / 3` long — the counts a bare-pointer C call
    /// cannot check for itself.
    pub fn rasterizeTrianglesU16(
        self: Heightfield,
        context: ?*const BuildContext,
        verts: []const f32,
        tris: []const u16,
        tri_areas: []const u8,
        flag_merge_threshold: i32,
    ) err.Error!void {
        if (verts.len == 0 or verts.len % 3 != 0) return err.Error.InvalidArgument;
        if (tris.len == 0 or tris.len % 3 != 0) return err.Error.InvalidArgument;
        const vert_count = std.math.cast(i32, verts.len / 3) orelse return err.Error.InvalidArgument;
        const tri_count = std.math.cast(i32, tris.len / 3) orelse return err.Error.InvalidArgument;
        if (tri_areas.len != tris.len / 3) return err.Error.InvalidArgument;
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldRasterizeTrianglesU16(
            toCContext(context, &context_storage),
            self.handle,
            verts.ptr,
            vert_count,
            tris.ptr,
            tri_areas.ptr,
            tri_count,
            flag_merge_threshold,
        ));
    }

    /// Rasterises an unindexed soup: three vertices per triangle, one area
    /// byte per triangle. `verts.len` must equal `9 * tri_areas.len` —
    /// three floats per vertex, three vertices per triangle — checked here
    /// since the C side is handed only a triangle count, not a vertex one.
    pub fn rasterizeTriangleSoup(
        self: Heightfield,
        context: ?*const BuildContext,
        verts: []const f32,
        tri_areas: []const u8,
        flag_merge_threshold: i32,
    ) err.Error!void {
        const tri_count = std.math.cast(i32, tri_areas.len) orelse return err.Error.InvalidArgument;
        if (verts.len != tri_areas.len * 9) return err.Error.InvalidArgument;
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldRasterizeTriangleSoup(
            toCContext(context, &context_storage),
            self.handle,
            verts.ptr,
            tri_areas.ptr,
            tri_count,
            flag_merge_threshold,
        ));
    }

    /// Marks spans a `walkable_climb` step reaches over as walkable, so an
    /// agent can climb a stair nosing or a kerb.
    pub fn filterLowHangingObstacles(
        self: Heightfield,
        context: ?*const BuildContext,
        walkable_climb: i32,
    ) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldFilterLowHangingObstacles(
            toCContext(context, &context_storage),
            self.handle,
            walkable_climb,
        ));
    }

    /// Drops spans on a ledge: those whose drop to a neighbour exceeds
    /// `walkable_climb`, which an agent standing there would fall off.
    pub fn filterLedgeSpans(
        self: Heightfield,
        context: ?*const BuildContext,
        walkable_height: i32,
        walkable_climb: i32,
    ) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldFilterLedgeSpans(
            toCContext(context, &context_storage),
            self.handle,
            walkable_height,
            walkable_climb,
        ));
    }

    /// Drops spans with less than `walkable_height` cells of clearance
    /// above them.
    pub fn filterWalkableLowHeightSpans(
        self: Heightfield,
        context: ?*const BuildContext,
        walkable_height: i32,
    ) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcHeightfieldFilterWalkableLowHeightSpans(
            toCContext(context, &context_storage),
            self.handle,
            walkable_height,
        ));
    }
};

//===----------------------------------------------------------------------===//
// The compact heightfield
//===----------------------------------------------------------------------===//

/// Where one column's spans start in the span array, and how many there are.
pub const CompactCell = c.CompactCell;

/// A span of open space, copied out of upstream's packed representation.
/// Decode `con` with `getCon`/`setCon` rather than by hand.
pub const CompactSpan = c.CompactSpan;

pub const CompactHeightfieldInfo = c.CompactHeightfieldInfo;

/// The walkable surface, rebuilt as open space instead of obstruction.
pub const CompactHeightfield = struct {
    handle: *c.CompactHeightfield,

    /// Builds the compact heightfield of the walkable surface in
    /// `heightfield`. `walkable_height` and `walkable_climb` are in cells.
    /// Borrows `heightfield`, which stays owned by the caller.
    pub fn init(
        context: ?*const BuildContext,
        walkable_height: i32,
        walkable_climb: i32,
        heightfield: Heightfield,
    ) err.Error!CompactHeightfield {
        var context_storage: c.BuildContext = undefined;
        var handle: *c.CompactHeightfield = undefined;
        try err.check(c.zrcCompactHeightfieldCreate(
            toCContext(context, &context_storage),
            walkable_height,
            walkable_climb,
            heightfield.handle,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CompactHeightfield) void {
        c.zrcCompactHeightfieldDestroy(self.handle);
    }

    pub fn info(self: CompactHeightfield) err.Error!CompactHeightfieldInfo {
        var out: CompactHeightfieldInfo = undefined;
        try err.check(c.zrcCompactHeightfieldInfo(self.handle, &out));
        return out;
    }

    /// Copies cells into `out`, starting at `first`. The array is
    /// `width * height` long, row-major: the cell at (x, z) is at
    /// `x + z * width`.
    pub fn cells(self: CompactHeightfield, first: u32, out: []CompactCell) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]CompactCell = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCompactHeightfieldCells(self.handle, first_i, count, ptr));
    }

    /// Copies spans into `out`, starting at `first`. The array is
    /// `info().span_count` long.
    pub fn spans(self: CompactHeightfield, first: u32, out: []CompactSpan) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]CompactSpan = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCompactHeightfieldSpans(self.handle, first_i, count, ptr));
    }

    /// Writes `spans_in` back, starting at `first`. Every packed field is
    /// checked against the width upstream gives it: a value that does not
    /// fit is silently truncated on the way into the bitfield and reads
    /// back as a different span otherwise.
    pub fn setSpans(self: CompactHeightfield, first: u32, spans_in: []const CompactSpan) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, spans_in.len) orelse return err.Error.InvalidArgument;
        try err.check(c.zrcCompactHeightfieldSetSpans(self.handle, first_i, count, spans_in.ptr));
    }

    /// Copies distance-field values into `out`, one per span, starting at
    /// `first`. `error.NotFound` before `buildDistanceField` has run: the
    /// array does not exist yet.
    pub fn distances(self: CompactHeightfield, first: u32, out: []u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCompactHeightfieldDistances(self.handle, first_i, count, ptr));
    }

    /// Copies area ids into `out`, one per span, starting at `first`.
    pub fn areas(self: CompactHeightfield, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCompactHeightfieldAreas(self.handle, first_i, count, ptr));
    }

    /// Writes area ids back, one per span, starting at `first`. Each must
    /// be below `c.max_areas`. How a host marks areas from data the volume
    /// shapes cannot express: a painted mask, a per-triangle table carried
    /// through rasterisation.
    pub fn setAreas(self: CompactHeightfield, first: u32, areas_in: []const u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, areas_in.len) orelse return err.Error.InvalidArgument;
        try err.check(c.zrcCompactHeightfieldSetAreas(self.handle, first_i, count, areas_in.ptr));
    }

    /// Marks `authoring`'s volumes into the area array, exactly as a bake
    /// does. `authoring.area_flags` is unused here: flags are assigned to
    /// polygons, which do not exist yet.
    pub fn markAreas(
        self: CompactHeightfield,
        context: ?*const BuildContext,
        authoring: AreaAuthoring,
    ) err.Error!void {
        const c_authoring = try authoring.toC();
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcCompactHeightfieldMarkAreas(
            toCContext(context, &context_storage),
            self.handle,
            &c_authoring,
        ));
    }

    /// Erodes the walkable area by `radius` cells, so the surface that
    /// survives is where an agent's centre may stand.
    pub fn erode(self: CompactHeightfield, context: ?*const BuildContext, radius: i32) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcCompactHeightfieldErode(toCContext(context, &context_storage), self.handle, radius));
    }

    /// Applies a median filter to the area ids, smoothing away single-span
    /// noise without moving a boundary.
    pub fn medianFilter(self: CompactHeightfield, context: ?*const BuildContext) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcCompactHeightfieldMedianFilter(toCContext(context, &context_storage), self.handle));
    }

    /// Builds the distance field: for every span, how far it is from the
    /// nearest unwalkable one. Required before `.watershed` regions.
    pub fn buildDistanceField(self: CompactHeightfield, context: ?*const BuildContext) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcCompactHeightfieldBuildDistanceField(toCContext(context, &context_storage), self.handle));
    }

    /// Splits the walkable surface into regions by `partition`.
    /// `border_size` is the unnavigable ring, in cells, a tiled build
    /// needs; `min_region_area` and `merge_region_area` are in cells
    /// squared, and the second is unused by `.layers`, which never merges.
    /// `.watershed` reads the distance field and is refused with
    /// `error.InvalidArgument` before `buildDistanceField` has run.
    pub fn buildRegions(
        self: CompactHeightfield,
        context: ?*const BuildContext,
        partition: c.Partition,
        border_size: i32,
        min_region_area: i32,
        merge_region_area: i32,
    ) err.Error!void {
        var context_storage: c.BuildContext = undefined;
        try err.check(c.zrcCompactHeightfieldBuildRegions(
            toCContext(context, &context_storage),
            self.handle,
            partition,
            border_size,
            min_region_area,
            merge_region_area,
        ));
    }
};

//===----------------------------------------------------------------------===//
// Contours
//===----------------------------------------------------------------------===//

pub const ContourSetInfo = c.ContourSetInfo;

/// One region's outline.
pub const ContourInfo = c.ContourInfo;

/// One contour vertex: x, y, z in cells, then the region id of the
/// neighbour across the edge that starts here, carrying `c.border_vertex`
/// and `c.area_border` above `c.contour_reg_mask`.
pub const ContourVertex = [4]i32;

/// What a contour build tessellates beyond what simplification would keep.
pub const ContourOptions = struct {
    /// Tessellate edges against impassable space.
    ///
    /// Defaulted on, because that is upstream's own default argument and what
    /// `PolyMesh.bake` therefore uses: a staged cook that turns it off
    /// produces a different mesh from a bake of the same geometry.
    tess_wall_edges: bool = true,
    /// Tessellate edges between two different areas.
    tess_area_edges: bool = false,

    fn toC(self: ContourOptions) i32 {
        var bits: i32 = 0;
        if (self.tess_wall_edges) bits |= c.contour_tess_wall_edges;
        if (self.tess_area_edges) bits |= c.contour_tess_area_edges;
        return bits;
    }
};

/// The outlines of every region, traced and simplified.
pub const ContourSet = struct {
    handle: *c.ContourSet,

    /// Traces and simplifies the outline of every region in `field`.
    /// `max_error` is how far a simplified edge may sit from the traced
    /// one, in cells; `max_edge_len` is the longest edge before it is
    /// subdivided, 0 for no limit. Borrows `field`.
    pub fn init(
        context: ?*const BuildContext,
        field: CompactHeightfield,
        max_error: f32,
        max_edge_len: i32,
        options: ContourOptions,
    ) err.Error!ContourSet {
        var context_storage: c.BuildContext = undefined;
        var handle: *c.ContourSet = undefined;
        try err.check(c.zrcContourSetCreate(
            toCContext(context, &context_storage),
            field.handle,
            max_error,
            max_edge_len,
            options.toC(),
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: ContourSet) void {
        c.zrcContourSetDestroy(self.handle);
    }

    pub fn info(self: ContourSet) err.Error!ContourSetInfo {
        var out: ContourSetInfo = undefined;
        try err.check(c.zrcContourSetInfo(self.handle, &out));
        return out;
    }

    pub fn at(self: ContourSet, index: u32) err.Error!ContourInfo {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: ContourInfo = undefined;
        try err.check(c.zrcContourAt(self.handle, i, &out));
        return out;
    }

    /// Copies simplified vertices of contour `index` into `out`, starting
    /// at `first`.
    pub fn verts(self: ContourSet, index: u32, first: u32, out: []ContourVertex) err.Error!void {
        const idx = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]i32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcContourVerts(self.handle, idx, first_i, count, ptr));
    }

    /// The same, for the raw traced outline before simplification.
    pub fn rawVerts(self: ContourSet, index: u32, first: u32, out: []ContourVertex) err.Error!void {
        const idx = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]i32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcContourRawVerts(self.handle, idx, first_i, count, ptr));
    }
};

//===----------------------------------------------------------------------===//
// The polygon mesh stages
//===----------------------------------------------------------------------===//

/// Builds polygons from `contours` into `mesh`, up to `verts_per_poly`
/// corners each. [Limit: 3 <= verts_per_poly <= c.verts_per_polygon]
/// Refused when `mesh` already holds polygons, per zrecast.h's rule that no
/// stage here overwrites a container that already holds a result.
pub fn polyMeshBuild(
    context: ?*const BuildContext,
    contours: ContourSet,
    verts_per_poly: i32,
    mesh: PolyMesh,
) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcPolyMeshBuild(
        toCContext(context, &context_storage),
        contours.handle,
        verts_per_poly,
        mesh.handle,
    ));
}

/// Builds the detail half of `mesh` from its polygons and the surface in
/// `field`. `sample_dist` and `sample_max_error` are in world units. A mesh
/// with no polygons leaves the detail half empty and succeeds — upstream's
/// own behaviour. Refused when the detail half is already filled.
pub fn polyMeshBuildDetail(
    context: ?*const BuildContext,
    mesh: PolyMesh,
    field: CompactHeightfield,
    sample_dist: f32,
    sample_max_error: f32,
) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcPolyMeshBuildDetail(
        toCContext(context, &context_storage),
        mesh.handle,
        field.handle,
        sample_dist,
        sample_max_error,
    ));
}

/// Copies the polygons of `src` into `dst`, which must be empty. The
/// detail half is not copied — rebuild it with `polyMeshBuildDetail`. The
/// agent dimensions travel with the polygons.
pub fn polyMeshCopy(context: ?*const BuildContext, src: PolyMesh, dst: PolyMesh) err.Error!void {
    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcPolyMeshCopy(toCContext(context, &context_storage), src.handle, dst.handle));
}

/// Merges `meshes` into `out`, which must be empty in both halves. Both
/// halves must be present in every input or absent from every input; every
/// input must carry its own agent dimensions and its own `verts_per_poly`,
/// and every input after the first must match the first on both — upstream
/// takes the first mesh's stride and dimensions for the whole merge. A
/// failure partway through can leave `out` holding a merged polygon half
/// and no detail half; destroy it and start again rather than reuse it.
///
/// Builds a scratch array of C handles from `meshes` through the installed
/// allocator, freed before this returns.
pub fn polyMeshMerge(
    context: ?*const BuildContext,
    meshes: []const PolyMesh,
    out: PolyMesh,
) err.Error!void {
    if (meshes.len == 0) return err.Error.InvalidArgument;
    const count = std.math.cast(i32, meshes.len) orelse return err.Error.InvalidArgument;
    const bytes_len = std.math.mul(usize, meshes.len, @sizeOf(*const c.PolyMesh)) catch
        return err.Error.InvalidArgument;
    const handle_bytes = try memory.alloc(bytes_len, .temp);
    defer memory.free(handle_bytes);
    const handles: [*]*const c.PolyMesh = @ptrCast(@alignCast(handle_bytes.ptr));
    for (meshes, 0..) |mesh, i| handles[i] = mesh.handle;

    var context_storage: c.BuildContext = undefined;
    try err.check(c.zrcPolyMeshMerge(toCContext(context, &context_storage), handles, count, out.handle));
}

//===----------------------------------------------------------------------===//
// Connection helpers
//
// Pure bit arithmetic on a `c.CompactSpan`'s packed `con` field, transcribed
// as upstream's own expressions (Recast.h: rcSetCon, rcGetCon,
// rcGetDirOffsetX, rcGetDirOffsetY, rcGetDirForOffset) so each answers
// identically to the C.
//===----------------------------------------------------------------------===//

/// The neighbour index `span` carries for `direction`, or `not_connected`
/// (0x3f) when there is none. Mirrors upstream's `rcGetCon`.
pub fn getCon(span: c.CompactSpan, direction: u2) u32 {
    const shift: u5 = @as(u5, direction) * 6;
    return (span.con >> shift) & 0x3f;
}

/// `span` with its `direction` neighbour set to `neighbor`, the other three
/// directions untouched. Mirrors upstream's `rcSetCon`; `neighbor` is
/// masked to six bits, the same as upstream's own assignment.
pub fn setCon(span: c.CompactSpan, direction: u2, neighbor: u32) c.CompactSpan {
    const shift: u5 = @as(u5, direction) * 6;
    var result = span;
    result.con = (span.con & ~(@as(u32, 0x3f) << shift)) | ((neighbor & 0x3f) << shift);
    return result;
}

/// The x step a cell takes moving in `direction`, one of the four compass
/// directions. Mirrors upstream's `rcGetDirOffsetX`; `direction` is masked
/// to its low two bits exactly as upstream masks it, so any input answers
/// rather than only 0..3.
pub fn dirOffsetX(direction: i32) i32 {
    const offset = [4]i32{ -1, 0, 1, 0 };
    return offset[@as(usize, @intCast(direction & 0x03))];
}

/// The z step a cell takes moving in `direction`. Mirrors upstream's
/// `rcGetDirOffsetY` under that name on purpose: upstream's own TODO says
/// the function should be called Z, and it already walks the z axis
/// despite the Y in its name.
pub fn dirOffsetY(direction: i32) i32 {
    const offset = [4]i32{ 0, 1, 0, -1 };
    return offset[@as(usize, @intCast(direction & 0x03))];
}

/// The direction that steps by (`offset_x`, `offset_z`) — one of the four
/// compass directions, for the four combinations where exactly one of them
/// is nonzero. Mirrors upstream's `rcGetDirForOffset`.
///
/// Upstream indexes a five-entry table with no bound of its own. Each
/// argument inside [-1, 1] is not enough on its own: a diagonal
/// combination inside that range still derives an index outside the table.
/// Both the arguments and the derived index are checked here, and
/// `error.InvalidArgument` replaces upstream's out-of-bounds read.
pub fn dirForOffset(offset_x: i32, offset_z: i32) err.Error!i32 {
    if (offset_x < -1 or offset_x > 1 or offset_z < -1 or offset_z > 1) {
        return err.Error.InvalidArgument;
    }
    const dirs = [5]i32{ 3, 0, -1, 2, 1 };
    const idx = ((offset_z + 1) << 1) + offset_x;
    if (idx < 0 or idx >= dirs.len) return err.Error.InvalidArgument;
    return dirs[@intCast(idx)];
}

test "getCon and setCon round-trip through the packed connection field" {
    var span = std.mem.zeroes(c.CompactSpan);
    inline for (0..4) |i| {
        const direction: u2 = i;
        const shift: u5 = @as(u5, direction) * 6;
        const neighbor: u32 = 0x15 + i;
        span = setCon(span, direction, neighbor);
        try std.testing.expectEqual(neighbor & 0x3f, getCon(span, direction));
        try std.testing.expectEqual(
            (neighbor & 0x3f) << shift,
            span.con & (@as(u32, 0x3f) << shift),
        );
    }
}

test "dirOffsetX and dirOffsetY match upstream's tables, and any input is safe" {
    const expected_x = [4]i32{ -1, 0, 1, 0 };
    const expected_y = [4]i32{ 0, 1, 0, -1 };
    inline for (0..4) |i| {
        try std.testing.expectEqual(expected_x[i], dirOffsetX(i));
        try std.testing.expectEqual(expected_y[i], dirOffsetY(i));
    }
    try std.testing.expectEqual(dirOffsetX(0), dirOffsetX(4));
    try std.testing.expectEqual(dirOffsetY(-1), dirOffsetY(3));
}

test "dirForOffset inverts dirOffsetX/dirOffsetY and rejects a diagonal" {
    inline for (0..4) |i| {
        const direction: i32 = i;
        const x = dirOffsetX(direction);
        const z = dirOffsetY(direction);
        try std.testing.expectEqual(direction, try dirForOffset(x, z));
    }
    try std.testing.expectError(err.Error.InvalidArgument, dirForOffset(-1, -1));
    try std.testing.expectError(err.Error.InvalidArgument, dirForOffset(1, 1));
    try std.testing.expectError(err.Error.InvalidArgument, dirForOffset(2, 0));
}

test "markWalkableTriangles rejects an area slice that does not match the triangle count" {
    const verts = [_]f32{ 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    const tris = [_]i32{ 0, 1, 2 };
    var short: [0]u8 = .{};
    try std.testing.expectError(
        err.Error.InvalidArgument,
        markWalkableTriangles(null, 45.0, TriMesh{ .verts = &verts, .tris = &tris }, &short),
    );
}

test "calcGridSize matches upstream's rounding rule" {
    const size = try calcGridSize(.{ 0, 0, 0 }, .{ 10, 0, 10 }, 1.0);
    try std.testing.expectEqual(@as(i32, 10), size.width);
    try std.testing.expectEqual(@as(i32, 10), size.height);
}
