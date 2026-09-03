//! The Detour half: asking a baked navmesh where an agent can go.
//!
//! Everything here is runtime work — no baking, no growth, no hidden
//! allocation. Each call writes into a buffer the caller owns and returns how
//! much of it was used.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const memory = @import("memory.zig");
const NavMesh = @import("navmesh.zig").NavMesh;
const Vec3 = @import("vec.zig").Vec3;

/// Identifies one polygon of one navmesh. Zero means "no polygon", which is
/// modelled as `null` everywhere it can occur.
pub const PolyRef = c.PolyRef;

/// Which polygons a query may traverse, and what each area costs.
pub const Filter = c.QueryFilter;

/// Every area cost 1.0 and every flag admitted — accepts every polygon a bake
/// produces.
pub fn defaultFilter() Filter {
    var filter: Filter = undefined;
    c.zrcQueryFilterDefault(&filter);
    return filter;
}

/// Flags on a straight-path corner.
pub const straightpath_start = c.straightpath_start;
pub const straightpath_end = c.straightpath_end;
pub const straightpath_offmesh_connection = c.straightpath_offmesh_connection;

/// Options for `NavMeshQuery.findStraightPath`. Not the flags above, despite
/// the shared prefix and overlapping values: these go in, those come out.
pub const StraightPathOptions = struct {
    /// Also emit a point wherever the corridor crosses an area boundary.
    area_crossings: bool = false,
    /// Also emit a point at every polygon edge the corridor crosses.
    all_crossings: bool = false,

    fn toC(self: StraightPathOptions) u32 {
        var bits: u32 = 0;
        if (self.area_crossings) bits |= c.straightpath_area_crossings;
        if (self.all_crossings) bits |= c.straightpath_all_crossings;
        return bits;
    }
};

/// What `findNearestPoly` found.
pub const NearestPoly = struct {
    /// The polygon, or null when nothing was in range. Not an error: an agent
    /// standing off the mesh is an ordinary situation.
    ref: ?PolyRef,
    /// The nearest point on that polygon, or the query point itself when
    /// nothing was found.
    point: [3]f32,
    /// Whether the query point's x/z actually lay inside the polygon, rather
    /// than the polygon merely being the closest one. Unset when nothing was
    /// found.
    over_poly: bool,
};

/// How much of a caller's buffer a path query filled, and whether it had to
/// stop short.
pub const PathResult = struct {
    /// The prefix of the caller's buffer that now holds the answer.
    len: usize,
    /// The answer is a best effort: either the goal was unreachable, the node
    /// pool ran out, or the buffer could not hold the whole result.
    ///
    /// This is deliberately not folded into the error set. A partial path is
    /// usable — an agent walks it and re-plans — and collapsing it into an
    /// error would push every caller into treating "no route" and "long route"
    /// the same way.
    partial: bool,
};

/// Options for `NavMeshQuery.raycast`.
pub const RaycastOptions = struct {
    /// Accumulate movement cost across the polygons crossed, into
    /// `RaycastHit.path_cost`. Without it that field stays 0.
    use_costs: bool = false,

    fn toC(self: RaycastOptions) u32 {
        return if (self.use_costs) c.raycast_use_costs else 0;
    }
};

/// Where a walkability raycast ended up.
pub const RaycastHit = struct {
    /// Fraction of the segment travelled before stopping, in [0, 1]. Detour's
    /// "no wall hit" is normalised to exactly 1.0 with `hit` clear, so `t` is
    /// always usable as a lerp parameter.
    t: f32,
    /// Point reached: start + (end - start) * t.
    position: [3]f32,
    /// Normal of the wall struck, or (0, 0, 0) when nothing was struck.
    normal: [3]f32,
    /// Whether a wall stopped the ray before the target position.
    hit: bool,
    /// Index of the polygon edge the ray crossed last, or null if it crossed
    /// none.
    hit_edge_index: ?u32,
    /// Cost of the movement along the ray, accumulated per polygon crossed.
    /// Zero unless `RaycastOptions.use_costs` was set.
    path_cost: f32,

    fn fromC(raw: c.RaycastHit) RaycastHit {
        return .{
            .t = raw.t,
            .position = raw.position,
            .normal = raw.normal,
            .hit = raw.hit != c.c_false,
            .hit_edge_index = if (raw.hit_edge_index < 0) null else @intCast(raw.hit_edge_index),
            .path_cost = raw.path_cost,
        };
    }
};

/// Where `moveAlongSurface` stopped, and how many polygons it crossed getting
/// there. `visited_len` is zero when no buffer was supplied.
pub const MoveResult = struct {
    position: [3]f32,
    visited_len: usize,
    /// The move crossed more polygons than the buffer could hold. `position` is
    /// correct regardless; only the list is short. Always false when no buffer
    /// was supplied — there is no list to have been cut short.
    truncated: bool,
};

/// What `raycast` found, and how many polygons the ray crossed. `path_len` is
/// zero when no buffer was supplied.
pub const RaycastResult = struct {
    hit: RaycastHit,
    path_len: usize,
    /// The ray crossed more polygons than the buffer could hold. `hit` is
    /// correct regardless; only the list is short.
    truncated: bool,
};

/// Options for `NavMeshQuery.slicedFindPathInit`.
pub const FindPathOptions = struct {
    /// Use raycasts to shortcut between polygons where the straight line is
    /// walkable, producing a path not confined to polygon centres. Costs are
    /// still evaluated.
    any_angle: bool = false,

    fn toC(self: FindPathOptions) u32 {
        return if (self.any_angle) c.findpath_any_angle else 0;
    }
};

/// How far a sliced search advanced, and whether more remains.
pub const SliceProgress = struct {
    /// Node expansions the update actually ran.
    iterations: u32,
    /// Whether more work remains. A finished search reports false and is
    /// ready for `NavMeshQuery.slicedFindPathFinalize`.
    in_progress: bool,
};

/// Supplies the randomness so a placement is reproducible from a seed.
pub const RandomSource = struct {
    /// Must return a value in [0, 1). Invoked synchronously, on the calling
    /// thread, before the entry point returns, an unspecified number of
    /// times.
    next: *const fn (user: ?*anyopaque) callconv(.c) f32,
    user: ?*anyopaque = null,

    fn toC(self: RandomSource) c.RandomSource {
        return .{ .next = self.next, .user = self.user };
    }
};

/// A random point on the navmesh, and the polygon it landed on.
pub const RandomPoint = struct {
    ref: PolyRef,
    point: Vec3,
};

/// How much of a caller's buffer a polygon query filled, and whether it had
/// to stop short.
pub const PolyList = struct {
    len: usize,
    truncated: bool,
};

/// Sink for `NavMeshQuery.queryPolygonsBatched`. `refs` is borrowed and valid
/// only for the duration of the call.
pub const PolySink = struct {
    process: *const fn (user: ?*anyopaque, refs: [*]const PolyRef, count: i32) callconv(.c) void,
    user: ?*anyopaque = null,

    fn toC(self: PolySink) c.PolyQuery {
        return .{ .process = self.process, .user = self.user };
    }
};

/// What `closestPointOnPoly` found.
pub const ClosestPoint = struct {
    point: Vec3,
    /// Whether the query position was already above the polygon, rather than
    /// projected onto it from outside.
    over_poly: bool,
};

/// How much of a caller's buffers a Dijkstra-style search filled, and whether
/// it had to stop short.
///
/// Several entry points below share one C-side `max_result` across two or
/// three output arrays. The Zig side takes a slice per array: an **empty**
/// slice passes NULL and means "not wanted"; a non-empty one must be exactly
/// as long as every other non-empty one in the group, or the call is
/// `error.InvalidArgument`, and `max_result` is that shared length.
/// `findLocalNeighbourhood` and `polyWallSegments` each have one array
/// upstream requires non-NULL regardless of what is wanted (`out_refs`,
/// `out_verts`); for those the required array's own length sets the capacity,
/// and only the remaining, genuinely optional array follows the
/// empty-means-NULL rule.
pub const ReachedPolys = struct {
    len: usize,
    truncated: bool,
};

/// How much of a caller's wall-segment buffers were filled, and whether it
/// had to stop short.
pub const WallSegments = struct {
    len: usize,
    truncated: bool,
};

/// What `findDistanceToWall` found.
pub const WallHit = struct {
    distance: f32,
    position: Vec3,
    normal: Vec3,
};

/// The three fields packed into a polygon reference. Only meaningful against
/// the navmesh that minted the reference, since the field widths are decided
/// when that navmesh is created.
pub const DecodedPolyRef = struct {
    salt: u32,
    tile: u32,
    poly: u32,
};

/// How full a query's node pool is.
pub const NodePoolInfo = struct {
    /// Nodes the last search used.
    node_count: u32,
    /// Nodes the pool can hold, as `NavMeshQuery.init` sized it.
    max_nodes: u32,
    /// Buckets in the pool's hash table, always a power of two.
    hash_size: u32,
    /// Bytes the pool occupies.
    bytes_used: u32,
};

/// What a search concluded about one node, as separate fields rather than
/// upstream's bit flags.
pub const NodeFlags = struct {
    /// On the open list: reached, cost not yet settled.
    open: bool,
    /// On the closed list: cost settled.
    closed: bool,
    /// Reached by a raycast shortcut, so its parent is not adjacent to it.
    parent_detached: bool,
};

/// One search node, copied out of a query's node pool.
pub const Node = struct {
    /// Position the search reached the polygon at.
    pos: Vec3,
    /// Cost of the step into this polygon.
    cost: f32,
    /// Cost from the search's start to here.
    total: f32,
    /// Polygon this node stands for.
    ref: PolyRef,
    /// The node this was reached from, or null for the search's start. Pass
    /// to `NavMeshQuery.nodeAt` to walk back towards the start.
    parent_index: ?u32,
    /// Which of the polygon's states this node is.
    state: u32,
    flags: NodeFlags,
};

fn nodeFromC(raw: c.Node) Node {
    return .{
        .pos = raw.pos,
        .cost = raw.cost,
        .total = raw.total,
        .ref = raw.ref,
        .parent_index = if (raw.parent_index == 0) null else raw.parent_index,
        .state = raw.state,
        .flags = .{
            .open = (raw.flags & c.node_open) != 0,
            .closed = (raw.flags & c.node_closed) != 0,
            .parent_detached = (raw.flags & c.node_parent_detached) != 0,
        },
    };
}

/// Turns `error.NotFound` from a node-pool lookup into `null`, letting every
/// other error through — the pattern `NavMeshQuery.findNode` and
/// `NavMeshQuery.nodeAt` share, since "not visited" is a real answer rather
/// than a failure.
fn nodeOrNotFound(result: c.Result, raw: c.Node) err.Error!?Node {
    if (result == .not_found) return null;
    try err.check(result);
    return nodeFromC(raw);
}

/// Checks a group of output slices that share one C-side `max_result`. See
/// `ReachedPolys` for the rule this enforces. Returns the shared length.
fn sharedCapacity(lens: []const usize) err.Error!i32 {
    var shared: ?usize = null;
    for (lens) |l| {
        if (l == 0) continue;
        if (shared) |s| {
            if (l != s) return err.Error.InvalidArgument;
        } else shared = l;
    }
    return std.math.cast(i32, shared orelse 0) orelse err.Error.InvalidArgument;
}

/// A navmesh plus the node pool a search needs.
///
/// Borrows its navmesh, which must outlive it. Not thread-safe: the node pool
/// is mutable state, so give each thread its own query.
pub const NavMeshQuery = struct {
    handle: *c.NavMeshQuery,

    /// `max_nodes` bounds every search: A* fails with `OutOfNodes` rather than
    /// growing. 2048 suits a small mesh.
    ///
    /// [Limit: 4 ... 65535]. Four, not one: Detour sizes the pool's hash table
    /// as `dtNextPow2(max_nodes / 4)`, which is zero for anything smaller, and
    /// every bucket index it then computes lands outside the allocation.
    pub fn init(navmesh: NavMesh, max_nodes: u32) err.Error!NavMeshQuery {
        const nodes = std.math.cast(i32, max_nodes) orelse
            return err.Error.InvalidArgument;
        var handle: *c.NavMeshQuery = undefined;
        try err.check(c.zrcNavMeshQueryCreate(navmesh.handle, nodes, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: NavMeshQuery) void {
        c.zrcNavMeshQueryDestroy(self.handle);
    }

    /// Finds the polygon nearest `center` within a box of `half_extents`.
    ///
    /// The y half-extent is usually the one that decides the answer: it has to
    /// span the distance from `center` down to the floor.
    pub fn findNearestPoly(
        self: NavMeshQuery,
        center: [3]f32,
        half_extents: [3]f32,
        filter: *const Filter,
    ) err.Error!NearestPoly {
        var ref: PolyRef = 0;
        var point: [3]f32 = undefined;
        var over_poly: c.Bool = c.c_false;
        try err.check(c.zrcFindNearestPoly(
            self.handle,
            &center,
            &half_extents,
            filter,
            &ref,
            &point,
            &over_poly,
        ));
        return .{
            .ref = if (ref == 0) null else ref,
            .point = point,
            .over_poly = over_poly != c.c_false,
        };
    }

    /// A* over the polygon graph, writing the corridor into `out_path`.
    ///
    /// The result is not a path to walk; it is the corridor a walkable path
    /// lives in. Pass it to `findStraightPath` to get corners.
    pub fn findPath(
        self: NavMeshQuery,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: [3]f32,
        end_pos: [3]f32,
        filter: *const Filter,
        out_path: []PolyRef,
    ) err.Error!PathResult {
        const capacity = std.math.cast(i32, out_path.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var partial: c.Bool = c.c_false;
        try err.check(c.zrcFindPath(
            self.handle,
            start_ref,
            end_ref,
            &start_pos,
            &end_pos,
            filter,
            out_path.ptr,
            capacity,
            &count,
            &partial,
        ));
        return .{ .len = @intCast(count), .partial = partial != c.c_false };
    }

    /// String-pulls a corridor into the corners to walk.
    ///
    /// `out_points` receives one `[3]f32` per corner. `out_flags` and
    /// `out_refs`, when given, must be at least as long as `out_points`.
    pub fn findStraightPath(
        self: NavMeshQuery,
        start_pos: [3]f32,
        end_pos: [3]f32,
        path: []const PolyRef,
        options: StraightPathOptions,
        out_points: [][3]f32,
        out_flags: ?[]u8,
        out_refs: ?[]PolyRef,
    ) err.Error!PathResult {
        const path_count = std.math.cast(i32, path.len) orelse
            return err.Error.InvalidArgument;
        const capacity = std.math.cast(i32, out_points.len) orelse
            return err.Error.InvalidArgument;
        // Detour writes these three arrays in lockstep and bounds only the
        // first; the C layer rejects a short companion, and the slice lengths
        // below are what it checks against.
        const flags_capacity = std.math.cast(i32, if (out_flags) |f| f.len else 0) orelse
            return err.Error.InvalidArgument;
        const refs_capacity = std.math.cast(i32, if (out_refs) |r| r.len else 0) orelse
            return err.Error.InvalidArgument;

        var count: i32 = 0;
        var partial: c.Bool = c.c_false;
        try err.check(c.zrcFindStraightPath(
            self.handle,
            &start_pos,
            &end_pos,
            path.ptr,
            path_count,
            options.toC(),
            // A [3]f32 array is 3 contiguous floats with no padding, which the
            // ABI test pins; this is the same memory Detour wants.
            @ptrCast(out_points.ptr),
            capacity,
            if (out_flags) |flags| flags.ptr else null,
            flags_capacity,
            if (out_refs) |refs| refs.ptr else null,
            refs_capacity,
            &count,
            &partial,
        ));
        return .{ .len = @intCast(count), .partial = partial != c.c_false };
    }

    /// Slides from `start_pos` towards `end_pos` across the surface, stopping
    /// at walls. The movement primitive for a character driven by input.
    ///
    /// Returns the position reached. `out_visited`, when given, receives the
    /// polygons crossed; `visited_len` in the result says how many.
    pub fn moveAlongSurface(
        self: NavMeshQuery,
        start_ref: PolyRef,
        start_pos: [3]f32,
        end_pos: [3]f32,
        filter: *const Filter,
        out_visited: ?[]PolyRef,
    ) err.Error!MoveResult {
        const capacity: i32 = if (out_visited) |visited|
            std.math.cast(i32, visited.len) orelse return err.Error.InvalidArgument
        else
            0;
        var position: [3]f32 = undefined;
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcMoveAlongSurface(
            self.handle,
            start_ref,
            &start_pos,
            &end_pos,
            filter,
            &position,
            if (out_visited) |visited| visited.ptr else null,
            capacity,
            &count,
            &truncated,
        ));
        return .{
            .position = position,
            .visited_len = @intCast(count),
            .truncated = truncated != c.c_false,
        };
    }

    /// Casts a walkability ray across the surface.
    ///
    /// Not a physics ray: it ignores height and reports the first navmesh
    /// boundary crossed. `prev_ref` is the polygon `start_ref` was entered
    /// from, or null for "no parent"; it affects nothing but the cost of
    /// entering `start_ref`, and only when `options.use_costs` is set.
    /// `out_path`, when given, receives the polygons crossed.
    pub fn raycast(
        self: NavMeshQuery,
        start_ref: PolyRef,
        start_pos: [3]f32,
        end_pos: [3]f32,
        filter: *const Filter,
        options: RaycastOptions,
        prev_ref: ?PolyRef,
        out_path: ?[]PolyRef,
    ) err.Error!RaycastResult {
        const capacity: i32 = if (out_path) |path|
            std.math.cast(i32, path.len) orelse return err.Error.InvalidArgument
        else
            0;
        var raw_hit: c.RaycastHit = undefined;
        var path_len: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcRaycast(
            self.handle,
            start_ref,
            &start_pos,
            &end_pos,
            filter,
            options.toC(),
            prev_ref orelse 0,
            &raw_hit,
            if (out_path) |path| path.ptr else null,
            capacity,
            &path_len,
            &truncated,
        ));
        return .{
            .hit = RaycastHit.fromC(raw_hit),
            .path_len = @intCast(path_len),
            .truncated = truncated != c.c_false,
        };
    }

    /// Begins a sliced search: the same A* `findPath` runs, driven a few
    /// iterations at a time so a frame can spend a fixed budget on it. State
    /// lives in this query; there is no second handle.
    ///
    /// `filter` is copied and need not outlive the call. Replaces any slice
    /// already in flight on this query.
    pub fn slicedFindPathInit(
        self: NavMeshQuery,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: Vec3,
        end_pos: Vec3,
        filter: *const Filter,
        options: FindPathOptions,
    ) err.Error!void {
        try err.check(c.zrcSlicedFindPathInit(
            self.handle,
            start_ref,
            end_ref,
            &start_pos,
            &end_pos,
            filter,
            options.toC(),
        ));
    }

    /// Advances the search by at most `max_iters` node expansions.
    ///
    /// A search that ran out of nodes, or whose start or end polygon stopped
    /// being valid, reports the failure here rather than at finalise.
    pub fn slicedFindPathUpdate(self: NavMeshQuery, max_iters: u32) err.Error!SliceProgress {
        const iters = std.math.cast(i32, max_iters) orelse
            return err.Error.InvalidArgument;
        var iterations: i32 = 0;
        var in_progress: c.Bool = c.c_false;
        try err.check(c.zrcSlicedFindPathUpdate(self.handle, iters, &iterations, &in_progress));
        return .{
            .iterations = @intCast(iterations),
            .in_progress = in_progress != c.c_false,
        };
    }

    /// Reads out the corridor a finished search found, and ends the slice.
    ///
    /// A second call in a row is `error.NoSearch` rather than an answer:
    /// upstream clears its own re-entry guard on the way out without clearing
    /// the state it guards, so a second call would otherwise fall into the
    /// "start and end are the same polygon" branch and report success with a
    /// one-element path holding the null reference.
    pub fn slicedFindPathFinalize(self: NavMeshQuery, out_path: []PolyRef) err.Error!PathResult {
        const capacity = std.math.cast(i32, out_path.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var partial: c.Bool = c.c_false;
        try err.check(c.zrcSlicedFindPathFinalize(
            self.handle,
            out_path.ptr,
            capacity,
            &count,
            &partial,
        ));
        return .{ .len = @intCast(count), .partial = partial != c.c_false };
    }

    /// Ends the slice early, keeping as much of `existing` — a corridor from
    /// an earlier search — as the search has confirmed: walked from its end
    /// backwards until one of its polygons is found in the search's own tree,
    /// then joined to what the search reached. Lets a host re-plan a path it
    /// is already walking without discarding it.
    pub fn slicedFindPathFinalizePartial(
        self: NavMeshQuery,
        existing: []const PolyRef,
        out_path: []PolyRef,
    ) err.Error!PathResult {
        const existing_count = std.math.cast(i32, existing.len) orelse
            return err.Error.InvalidArgument;
        const capacity = std.math.cast(i32, out_path.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var partial: c.Bool = c.c_false;
        try err.check(c.zrcSlicedFindPathFinalizePartial(
            self.handle,
            existing.ptr,
            existing_count,
            out_path.ptr,
            capacity,
            &count,
            &partial,
        ));
        return .{ .len = @intCast(count), .partial = partial != c.c_false };
    }

    /// Abandons a slice in flight, releasing the query for other searches.
    /// Succeeds whether or not a slice was in flight.
    pub fn slicedFindPathCancel(self: NavMeshQuery) err.Error!void {
        try err.check(c.zrcSlicedFindPathCancel(self.handle));
    }

    /// Whether a sliced search is in flight on this query.
    pub fn slicedFindPathActive(self: NavMeshQuery) bool {
        return c.zrcSlicedFindPathActive(self.handle) != c.c_false;
    }

    /// A random point on the navmesh, uniform by polygon area.
    pub fn findRandomPoint(
        self: NavMeshQuery,
        filter: *const Filter,
        random: RandomSource,
    ) err.Error!RandomPoint {
        const source = random.toC();
        var ref: PolyRef = 0;
        var point: Vec3 = undefined;
        try err.check(c.zrcFindRandomPoint(self.handle, filter, &source, &ref, &point));
        return .{ .ref = ref, .point = point };
    }

    /// A random point on a polygon the search reached within `max_radius` of
    /// `center`, starting from `start_ref`. Reachable, not merely near: the
    /// search walks the surface, so a polygon across a wall is not a
    /// candidate however close it is.
    ///
    /// The polygon is what the radius bounds, not the point. Upstream picks a
    /// polygon the search reached and places the point anywhere inside it, and
    /// a polygon reached near the edge of the circle extends past it.
    pub fn findRandomPointAroundCircle(
        self: NavMeshQuery,
        start_ref: PolyRef,
        center: Vec3,
        max_radius: f32,
        filter: *const Filter,
        random: RandomSource,
    ) err.Error!RandomPoint {
        const source = random.toC();
        var ref: PolyRef = 0;
        var point: Vec3 = undefined;
        try err.check(c.zrcFindRandomPointAroundCircle(
            self.handle,
            start_ref,
            &center,
            max_radius,
            filter,
            &source,
            &ref,
            &point,
        ));
        return .{ .ref = ref, .point = point };
    }

    /// Every polygon whose bounds overlap the box `center` +/- `half_extents`.
    pub fn queryPolygons(
        self: NavMeshQuery,
        center: Vec3,
        half_extents: Vec3,
        filter: *const Filter,
        out_refs: []PolyRef,
    ) err.Error!PolyList {
        const capacity = std.math.cast(i32, out_refs.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcQueryPolygons(
            self.handle,
            &center,
            &half_extents,
            filter,
            out_refs.ptr,
            capacity,
            &count,
            &truncated,
        ));
        return .{ .len = @intCast(count), .truncated = truncated != c.c_false };
    }

    /// The same query, delivered in batches instead of into a buffer: for a
    /// caller that would rather not size an array for a result it cannot
    /// predict. Batches follow upstream's own boundary — once per 32
    /// polygons within a single tile, plus once for what remains of it — not
    /// a promise this binding makes.
    pub fn queryPolygonsBatched(
        self: NavMeshQuery,
        center: Vec3,
        half_extents: Vec3,
        filter: *const Filter,
        sink: PolySink,
    ) err.Error!void {
        const raw_sink = sink.toC();
        try err.check(c.zrcQueryPolygonsBatched(self.handle, &center, &half_extents, filter, &raw_sink));
    }

    /// Closest point on `ref` to `pos`, using the polygon's detail mesh for
    /// height.
    pub fn closestPointOnPoly(self: NavMeshQuery, ref: PolyRef, pos: Vec3) err.Error!ClosestPoint {
        var point: Vec3 = undefined;
        var over_poly: c.Bool = c.c_false;
        try err.check(c.zrcClosestPointOnPoly(self.handle, ref, &pos, &point, &over_poly));
        return .{ .point = point, .over_poly = over_poly != c.c_false };
    }

    /// Closest point on `ref`'s boundary to `pos`, which is the polygon
    /// itself when `pos` is outside it and the projection onto its edge
    /// otherwise. Cheaper than `closestPointOnPoly` and does not consult the
    /// detail mesh, so the height it reports is the polygon's own.
    pub fn closestPointOnPolyBoundary(self: NavMeshQuery, ref: PolyRef, pos: Vec3) err.Error!Vec3 {
        var point: Vec3 = undefined;
        try err.check(c.zrcClosestPointOnPolyBoundary(self.handle, ref, &pos, &point));
        return point;
    }

    /// Height of `ref`'s surface directly under `pos`. `error.InvalidArgument`
    /// when `pos` is not over the polygon at all, which is the answer rather
    /// than an approximation.
    pub fn polyHeight(self: NavMeshQuery, ref: PolyRef, pos: Vec3) err.Error!f32 {
        var height: f32 = 0;
        try err.check(c.zrcPolyHeight(self.handle, ref, &pos, &height));
        return height;
    }

    /// Every polygon reachable from `start_ref` within `radius` of `center`,
    /// as a Dijkstra search over the surface rather than a route to it.
    /// `out_parents` gives the polygon each was reached through, which is
    /// what makes the result a tree rather than a set; `out_costs` gives the
    /// cost to reach each. See `ReachedPolys` for the shared-capacity rule.
    pub fn findPolysAroundCircle(
        self: NavMeshQuery,
        start_ref: PolyRef,
        center: Vec3,
        radius: f32,
        filter: *const Filter,
        out_refs: []PolyRef,
        out_parents: []PolyRef,
        out_costs: []f32,
    ) err.Error!ReachedPolys {
        const lens = [_]usize{ out_refs.len, out_parents.len, out_costs.len };
        const capacity = try sharedCapacity(&lens);
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcFindPolysAroundCircle(
            self.handle,
            start_ref,
            &center,
            radius,
            filter,
            if (out_refs.len == 0) null else out_refs.ptr,
            if (out_parents.len == 0) null else out_parents.ptr,
            if (out_costs.len == 0) null else out_costs.ptr,
            capacity,
            &count,
            &truncated,
        ));
        return .{ .len = @intCast(count), .truncated = truncated != c.c_false };
    }

    /// The same search, bounded by a convex polygon instead of a circle.
    /// `verts` is projected onto the xz-plane.
    pub fn findPolysAroundShape(
        self: NavMeshQuery,
        start_ref: PolyRef,
        verts: []const Vec3,
        filter: *const Filter,
        out_refs: []PolyRef,
        out_parents: []PolyRef,
        out_costs: []f32,
    ) err.Error!ReachedPolys {
        const vert_count = std.math.cast(i32, verts.len) orelse
            return err.Error.InvalidArgument;
        const lens = [_]usize{ out_refs.len, out_parents.len, out_costs.len };
        const capacity = try sharedCapacity(&lens);
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcFindPolysAroundShape(
            self.handle,
            start_ref,
            // A []Vec3 is tightly packed floats, the same memory Detour wants.
            @ptrCast(verts.ptr),
            vert_count,
            filter,
            if (out_refs.len == 0) null else out_refs.ptr,
            if (out_parents.len == 0) null else out_parents.ptr,
            if (out_costs.len == 0) null else out_costs.ptr,
            capacity,
            &count,
            &truncated,
        ));
        return .{ .len = @intCast(count), .truncated = truncated != c.c_false };
    }

    /// The corridor from the last Dijkstra search's start to `end_ref`. Reads
    /// the search tree `findPolysAroundCircle` or `findPolysAroundShape` left
    /// behind, so it is meaningful only right after one of them, for a
    /// polygon that search reached.
    pub fn pathFromDijkstraSearch(
        self: NavMeshQuery,
        end_ref: PolyRef,
        out_path: []PolyRef,
    ) err.Error!PolyList {
        const capacity = std.math.cast(i32, out_path.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcPathFromDijkstraSearch(
            self.handle,
            end_ref,
            out_path.ptr,
            capacity,
            &count,
            &truncated,
        ));
        return .{ .len = @intCast(count), .truncated = truncated != c.c_false };
    }

    /// Polygons touching a circle around `center`, without a real search:
    /// only the polygons adjacent to `start_ref` and their neighbours, cheap
    /// enough for a per-frame collision query and useless further than a step
    /// away. `out_refs` is required, unlike its two siblings above: upstream
    /// writes through it with no null check.
    pub fn findLocalNeighbourhood(
        self: NavMeshQuery,
        start_ref: PolyRef,
        center: Vec3,
        radius: f32,
        filter: *const Filter,
        out_refs: []PolyRef,
        out_parents: []PolyRef,
    ) err.Error!ReachedPolys {
        if (out_parents.len != 0 and out_parents.len != out_refs.len)
            return err.Error.InvalidArgument;
        const capacity = std.math.cast(i32, out_refs.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcFindLocalNeighbourhood(
            self.handle,
            start_ref,
            &center,
            radius,
            filter,
            out_refs.ptr,
            if (out_parents.len == 0) null else out_parents.ptr,
            capacity,
            &count,
            &truncated,
        ));
        return .{ .len = @intCast(count), .truncated = truncated != c.c_false };
    }

    /// The segments of `ref`'s boundary an agent under `filter` cannot cross.
    /// `out_verts` takes two points per segment; `out_refs`, when given,
    /// receives the polygon on the far side of each, or 0 where there is
    /// none.
    pub fn polyWallSegments(
        self: NavMeshQuery,
        ref: PolyRef,
        filter: *const Filter,
        out_verts: [][2]Vec3,
        out_refs: []PolyRef,
    ) err.Error!WallSegments {
        if (out_refs.len != 0 and out_refs.len != out_verts.len)
            return err.Error.InvalidArgument;
        const capacity = std.math.cast(i32, out_verts.len) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        var truncated: c.Bool = c.c_false;
        try err.check(c.zrcPolyWallSegments(
            self.handle,
            ref,
            filter,
            // A [2]Vec3 is four tightly packed floats, the same memory Detour
            // wants for a segment's two endpoints.
            @ptrCast(out_verts.ptr),
            if (out_refs.len == 0) null else out_refs.ptr,
            capacity,
            &count,
            &truncated,
        ));
        return .{ .len = @intCast(count), .truncated = truncated != c.c_false };
    }

    /// Distance from `center` to the nearest wall within `max_radius`, or
    /// null when nothing is in range.
    pub fn findDistanceToWall(
        self: NavMeshQuery,
        start_ref: PolyRef,
        center: Vec3,
        max_radius: f32,
        filter: *const Filter,
    ) err.Error!?WallHit {
        var dist: f32 = 0;
        var pos: Vec3 = undefined;
        var normal: Vec3 = undefined;
        var found: c.Bool = c.c_false;
        try err.check(c.zrcFindDistanceToWall(
            self.handle,
            start_ref,
            &center,
            max_radius,
            filter,
            &dist,
            &pos,
            &normal,
            &found,
        ));
        if (found == c.c_false) return null;
        return .{ .distance = dist, .position = pos, .normal = normal };
    }

    /// Whether `ref` names a live polygon, and — when `filter` is given —
    /// one this filter would admit. A null filter asks the plainer question:
    /// does `ref` decode to a polygon that exists, with no admission test.
    pub fn isValidPolyRef(self: NavMeshQuery, ref: PolyRef, filter: ?*const Filter) err.Error!bool {
        var valid: c.Bool = c.c_false;
        try err.check(c.zrcIsValidPolyRef(self.handle, ref, filter, &valid));
        return valid != c.c_false;
    }

    /// Whether the last search closed `ref` — visited it and settled its
    /// cost.
    pub fn isInClosedList(self: NavMeshQuery, ref: PolyRef) err.Error!bool {
        var closed: c.Bool = c.c_false;
        try err.check(c.zrcIsInClosedList(self.handle, ref, &closed));
        return closed != c.c_false;
    }

    /// The navmesh this query was created against. Borrowed; the caller
    /// still owns it.
    pub fn attachedNavMesh(self: NavMeshQuery) err.Error!NavMesh {
        var handle: *const c.NavMesh = undefined;
        try err.check(c.zrcQueryNavMesh(self.handle, &handle));
        return .{ .handle = @constCast(handle) };
    }

    /// How full this query's node pool is.
    pub fn nodePoolInfo(self: NavMeshQuery) err.Error!NodePoolInfo {
        var raw: c.NodePoolInfo = undefined;
        try err.check(c.zrcQueryNodePoolInfo(self.handle, &raw));
        return .{
            .node_count = @intCast(raw.node_count),
            .max_nodes = @intCast(raw.max_nodes),
            .hash_size = @intCast(raw.hash_size),
            .bytes_used = @intCast(raw.bytes_used),
        };
    }

    /// The node the last search made for `ref` in state `state`, if it made
    /// one. Null rather than an error when the search never reached that
    /// polygon in that state — the difference between "cost zero" and "not
    /// visited".
    pub fn findNode(self: NavMeshQuery, ref: PolyRef, state: u32) err.Error!?Node {
        var raw: c.Node = undefined;
        const result = c.zrcQueryFindNode(self.handle, ref, state, &raw);
        return nodeOrNotFound(result, raw);
    }

    /// Every node the last search made for `ref`, across its states, written
    /// into `out`. Returns how many were written, at most four — the width of
    /// upstream's per-polygon state table.
    pub fn findNodes(self: NavMeshQuery, ref: PolyRef, out: []Node) err.Error!usize {
        var raw: [c.max_node_states]c.Node = undefined;
        const capacity = std.math.cast(i32, @min(out.len, raw.len)) orelse
            return err.Error.InvalidArgument;
        var count: i32 = 0;
        try err.check(c.zrcQueryFindNodes(self.handle, ref, &raw, capacity, &count));
        const found: usize = @intCast(count);
        for (out[0..found], raw[0..found]) |*dst, src| dst.* = nodeFromC(src);
        return found;
    }

    /// The node at a one-based pool index, which is what `Node.parent_index`
    /// carries. Null for index 0, "no node".
    pub fn nodeAt(self: NavMeshQuery, index: u32) err.Error!?Node {
        var raw: c.Node = undefined;
        const result = c.zrcQueryNodeAt(self.handle, index, &raw);
        return nodeOrNotFound(result, raw);
    }
};

/// Splits a polygon reference into the three fields packed into it.
pub fn decodePolyRef(navmesh: NavMesh, ref: PolyRef) err.Error!DecodedPolyRef {
    var salt: u32 = 0;
    var tile: u32 = 0;
    var poly: u32 = 0;
    try err.check(c.zrcDecodePolyRef(navmesh.handle, ref, &salt, &tile, &poly));
    return .{ .salt = salt, .tile = tile, .poly = poly };
}

/// Packs the three fields back into a reference. `error.InvalidArgument`
/// when any of them is too wide for the navmesh's own bit layout, which
/// upstream instead truncates silently.
pub fn encodePolyRef(navmesh: NavMesh, salt: u32, tile: u32, poly: u32) err.Error!PolyRef {
    var ref: PolyRef = 0;
    try err.check(c.zrcEncodePolyRef(navmesh.handle, salt, tile, poly, &ref));
    return ref;
}

test "the default filter admits everything at neutral cost" {
    const filter = defaultFilter();
    try std.testing.expectEqual(@as(u16, 0xffff), filter.include_flags);
    try std.testing.expectEqual(@as(u16, 0), filter.exclude_flags);
    for (filter.area_cost) |cost| try std.testing.expectEqual(@as(f32, 1.0), cost);
}

test "a [3]f32 array is three tightly packed floats" {
    // findStraightPath reinterprets a []`[3]f32` as a flat float array, which
    // is only sound while this holds.
    try std.testing.expectEqual(@as(usize, 12), @sizeOf([3]f32));
    try std.testing.expectEqual(@as(usize, 24), @sizeOf([2][3]f32));
}

/// A navmesh and query pair, empty: no tiles added, enough to exercise a
/// query's own state without baking any geometry.
const EmptyWorld = struct {
    mesh: NavMesh,
    query: NavMeshQuery,

    fn init() !EmptyWorld {
        const grid: c.TileGrid = .{
            .origin = .{ 0, 0, 0 },
            .extent_max = .{ 10, 0, 10 },
            .tile_world_size = 10,
            .tile_count_x = 1,
            .tile_count_z = 1,
        };
        const mesh = try NavMesh.initTiled(grid, 1, 256);
        errdefer mesh.deinit();
        const query = try NavMeshQuery.init(mesh, 8);
        return .{ .mesh = mesh, .query = query };
    }

    fn deinit(self: EmptyWorld) void {
        self.query.deinit();
        self.mesh.deinit();
    }
};

test "a fresh query has no sliced search in flight" {
    const gpa = std.testing.allocator;
    try memory.setAllocator(gpa);
    defer memory.resetAllocator();

    const world = try EmptyWorld.init();
    defer world.deinit();

    try std.testing.expect(!world.query.slicedFindPathActive());
    try std.testing.expectError(err.Error.NoSearch, world.query.slicedFindPathUpdate(1));
}

test "findNode on a query that has run no search is null rather than an error" {
    const gpa = std.testing.allocator;
    try memory.setAllocator(gpa);
    defer memory.resetAllocator();

    const world = try EmptyWorld.init();
    defer world.deinit();

    try std.testing.expectEqual(@as(?Node, null), try world.query.findNode(1, 0));
}
