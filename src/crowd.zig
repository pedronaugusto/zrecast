//! Crowds, and the four pieces they steer with.
//!
//! A `NavMesh` says what is walkable and a `NavMeshQuery` answers where an
//! agent could go. A `Crowd` moves it: every frame it re-plans each agent's
//! local path, collects the neighbours and walls nearby, picks a velocity that
//! avoids both, and integrates under acceleration and speed limits. Position
//! and velocity are outputs; a host controls `Crowd.setAgentParams` and the
//! three move-request calls.
//!
//! `ProximityGrid`, `AvoidanceQuery`, `PathCorridor`, `LocalBoundary` and
//! `PathQueue` are the parts a crowd drives internally, each usable on its own.
//!
//! The byte-for-byte cook guarantee does not reach here: the velocity sampler
//! calls `cosf` and `sinf`. See `ffi/zrecast.h`.

const aegis = @import("aegis");
const c = @import("c.zig");
const err = @import("error.zig");
const NavMesh = @import("navmesh.zig").NavMesh;
const NavMeshQuery = @import("query.zig").NavMeshQuery;
const BorrowedNavMeshQuery = @import("query.zig").BorrowedNavMeshQuery;
const QueryFilter = @import("query.zig").QueryFilter;
const QueryFilterDesc = @import("query.zig").QueryFilterDesc;
const PolyRef = @import("query.zig").PolyRef;

//===----------------------------------------------------------------------===//
// Values
//===----------------------------------------------------------------------===//

/// Names one agent in a crowd. 0 is never a live agent.
///
/// Slot and serial packed together, so a slot reused after a removal answers
/// to a different reference and the old one is `error.NotFound`. Upstream has
/// no identity of its own: every setter it offers takes a bare pool index and
/// never asks whether that slot is live.
pub const AgentRef = c.AgentRef;

/// Names one submitted search in a `PathQueue`.
pub const PathRequestRef = c.PathRequestRef;

pub const CrowdAgentState = c.CrowdAgentState;
pub const CrowdTargetState = c.CrowdTargetState;
pub const CrowdAgentParams = c.CrowdAgentParams;
pub const CrowdAgent = c.CrowdAgent;
pub const CrowdCorner = c.CrowdCorner;
pub const CrowdNeighbour = c.CrowdNeighbour;
/// How far an agent is through an off-mesh connection.
///
/// Nothing here produces one, because nothing upstream does: a crowd drives an
/// off-mesh traversal through a private array with no accessor, so a C++ host
/// cannot read a live one either. The type is here because upstream declares
/// it publicly. What is observable is `CrowdAgent.state`.
pub const CrowdAgentAnimation = c.CrowdAgentAnimation;
pub const AvoidanceParams = c.AvoidanceParams;
pub const AvoidanceCircle = c.AvoidanceCircle;
pub const AvoidanceSegment = c.AvoidanceSegment;
pub const AvoidanceSample = c.AvoidanceSample;
pub const PathCorridorInfo = c.PathCorridorInfo;
pub const PathRequestState = c.PathRequestState;

pub const crowd_max_neighbours = c.crowd_max_neighbours;
pub const crowd_max_corners = c.crowd_max_corners;
pub const crowd_max_avoidance_params = c.crowd_max_avoidance_params;
pub const crowd_max_filters = c.crowd_max_filters;
pub const crowd_max_agents = c.crowd_max_agents;
pub const avoidance_max_pattern_divs = c.avoidance_max_pattern_divs;
pub const avoidance_max_pattern_rings = c.avoidance_max_pattern_rings;
pub const path_corridor_min_path = c.path_corridor_min_path;

/// Which steering behaviours an agent takes part in. A bitwise or of these
/// goes in `CrowdAgentParams.update_flags`.
pub const UpdateFlags = struct {
    /// Slow down before a corner instead of cutting it.
    pub const anticipate_turns = c.crowd_anticipate_turns;
    /// Steer around neighbouring agents rather than through them.
    pub const obstacle_avoidance = c.crowd_obstacle_avoidance;
    /// Push apart from neighbours already too close.
    pub const separation = c.crowd_separation;
    /// Shorten the corridor whenever a later corner is directly visible.
    pub const optimize_vis = c.crowd_optimize_vis;
    /// Re-plan the corridor locally as the world changes under it.
    pub const optimize_topo = c.crowd_optimize_topo;
};

/// What one update recorded about one agent, in and out.
///
/// `agent` and `samples` are inputs and both are optional: naming an agent
/// with no recorder still fills the two optimiser points, and a recorder with
/// no agent named collects nothing.
pub const CrowdAgentDebug = struct {
    agent: ?AgentRef = null,
    samples: ?AvoidanceDebug = null,
    /// Filled by `Crowd.update`: the segment the visibility optimiser tried.
    /// Left at the previous update's values when the agent did not optimise.
    opt_start: [3]f32 = .{ 0, 0, 0 },
    opt_end: [3]f32 = .{ 0, 0, 0 },
};

/// The configuration a crowd installs in all eight of its avoidance slots.
///
/// Not upstream's: `dtCrowd::init` writes these ten values inline and offers
/// no way to ask for them.
pub fn avoidanceParamsDefault() AvoidanceParams {
    var out: AvoidanceParams = undefined;
    c.zrcAvoidanceParamsDefault(&out);
    return out;
}

//===----------------------------------------------------------------------===//
// The crowd
//===----------------------------------------------------------------------===//

/// A group of agents steering around each other and the world.
pub const Crowd = struct {
    handle: *c.Crowd,

    /// Creates a crowd of at most `max_agents`, planning against `navmesh`.
    ///
    /// `navmesh` is borrowed and must outlive the crowd; an agent whose polygon
    /// is removed drops to `.invalid` and recovers when one is there again.
    /// `max_agents` is `[1, crowd_max_agents]`; `max_agent_radius` sizes the
    /// proximity grid and search box, and a larger agent radius is refused.
    pub fn init(navmesh: NavMesh, max_agents: u32, max_agent_radius: f32, failure: *err.Failure) err.Error!Crowd {
        const agents = aegis.int.cast(i32, max_agents) catch
            return err.refuse(failure, .crowd_create, error.InvalidArgument);
        var handle: *c.Crowd = undefined;
        try err.check(failure, .crowd_create, c.zrcCrowdCreate(navmesh.handle, agents, max_agent_radius, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Crowd) void {
        c.zrcCrowdDestroy(self.handle);
    }

    /// Re-initialises the crowd in place, as though it had just been created.
    ///
    /// Every agent is discarded and **every `AgentRef` minted before this
    /// stops resolving**, including ones the caller still holds. A failure
    /// leaves the crowd empty rather than in its previous state: the purge
    /// happens first and cannot be undone.
    pub fn reinit(self: Crowd, navmesh: NavMesh, max_agents: u32, max_agent_radius: f32, failure: *err.Failure) err.Error!void {
        const agents = aegis.int.cast(i32, max_agents) catch
            return err.refuse(failure, .crowd_init, error.InvalidArgument);
        try err.check(failure, .crowd_init, c.zrcCrowdInit(self.handle, navmesh.handle, agents, max_agent_radius));
    }

    //-- Agents ---------------------------------------------------------------

    /// Adds an agent at the navmesh position nearest `position`.
    ///
    /// When nothing is near enough it is still added, at the requested point,
    /// in state `.invalid` — it exists and is not steered.
    ///
    /// `error.CrowdFull` when every slot is taken.
    pub fn addAgent(
        self: Crowd,
        position: [3]f32,
        params: CrowdAgentParams,
        failure: *err.Failure,
    ) err.Error!AgentRef {
        var ref: AgentRef = undefined;
        try err.check(failure, .crowd_add_agent, c.zrcCrowdAddAgent(self.handle, &position, &params, &ref));
        return ref;
    }

    /// Removes an agent. Its slot becomes available and `ref` stops resolving.
    pub fn removeAgent(self: Crowd, ref: AgentRef, failure: *err.Failure) err.Error!void {
        try err.check(failure, .crowd_remove_agent, c.zrcCrowdRemoveAgent(self.handle, ref));
    }

    /// Replaces an agent's parameters. Position, velocity and target are
    /// untouched.
    pub fn setAgentParams(self: Crowd, ref: AgentRef, params: CrowdAgentParams, failure: *err.Failure) err.Error!void {
        try err.check(failure, .crowd_set_agent_params, c.zrcCrowdSetAgentParams(self.handle, ref, &params));
    }

    /// Everything about one agent except its corridor and its local boundary,
    /// which have accessors of their own.
    pub fn agent(self: Crowd, ref: AgentRef, failure: *err.Failure) err.Error!CrowdAgent {
        var out: CrowdAgent = undefined;
        try err.check(failure, .crowd_agent_info, c.zrcCrowdAgentInfo(self.handle, ref, &out));
        return out;
    }

    /// Agent slots the crowd was created for, live or not. Not a population;
    /// that is `activeAgentCount`.
    pub fn agentCapacity(self: Crowd, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .crowd_agent_capacity, c.zrcCrowdAgentCapacity(self.handle, &out));
        return c.reported(u32, out);
    }

    /// Agents currently in the crowd.
    pub fn activeAgentCount(self: Crowd, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .crowd_active_agent_count, c.zrcCrowdActiveAgentCount(self.handle, &out));
        return c.reported(u32, out);
    }

    /// Every active agent's reference.
    ///
    /// `out` receives however many fit; the total is always the return value,
    /// so a caller compares it against `out.len` to see whether a second,
    /// larger call is needed rather than treating a short buffer as failure.
    /// An empty `out` asks only for the count.
    pub fn activeAgents(self: Crowd, out: []AgentRef, failure: *err.Failure) err.Error!usize {
        const max_agents = aegis.int.cast(i32, out.len) catch
            return err.refuse(failure, .crowd_active_agents, error.InvalidArgument);
        const ptr: ?[*]AgentRef = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        const result = c.zrcCrowdActiveAgents(self.handle, ptr, max_agents, &count);
        if (result == .buffer_too_small) return c.reported(usize, count);
        try err.check(failure, .crowd_active_agents, result);
        return c.reported(usize, count);
    }

    /// The reference of the agent in slot `index`, or null when the slot is free.
    pub fn agentRefAt(self: Crowd, index: u32, failure: *err.Failure) err.Error!?AgentRef {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .crowd_agent_ref_at, error.InvalidArgument);
        var ref: c.Maybe(AgentRef) = .none;
        try err.check(failure, .crowd_agent_ref_at, c.zrcCrowdAgentRefAt(self.handle, i, &ref));
        return ref.get();
    }

    /// Copies corners of the agent's local path into `out`, starting at
    /// `first`. The total is `CrowdAgent.corner_count`, at most
    /// `crowd_max_corners`; the last is where the agent is heading.
    pub fn agentCorners(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: []CrowdCorner,
        failure: *err.Failure,
    ) err.Error!void {
        const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .crowd_agent_corners, error.InvalidArgument);
        const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .crowd_agent_corners, error.InvalidArgument);
        const ptr: ?[*]CrowdCorner = if (out.len == 0) null else out.ptr;
        try err.check(failure, .crowd_agent_corners, c.zrcCrowdAgentCorners(self.handle, ref, first_i, count, ptr));
    }

    /// The same for its neighbours, nearest first. The total is
    /// `CrowdAgent.neighbour_count`, at most `crowd_max_neighbours`.
    pub fn agentNeighbours(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: []CrowdNeighbour,
        failure: *err.Failure,
    ) err.Error!void {
        const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .crowd_agent_neighbours, error.InvalidArgument);
        const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .crowd_agent_neighbours, error.InvalidArgument);
        const ptr: ?[*]CrowdNeighbour = if (out.len == 0) null else out.ptr;
        try err.check(failure, .crowd_agent_neighbours, c.zrcCrowdAgentNeighbours(self.handle, ref, first_i, count, ptr));
    }

    /// The corridor the agent is walking, read-only. The standalone
    /// `PathCorridor` is the same object, editable, for a host steering one
    /// character itself.
    pub fn agentCorridor(self: Crowd, ref: AgentRef, failure: *err.Failure) err.Error!PathCorridorInfo {
        var out: PathCorridorInfo = undefined;
        try err.check(failure, .crowd_agent_corridor_info, c.zrcCrowdAgentCorridorInfo(self.handle, ref, &out));
        return out;
    }

    /// Copies polygons of the agent's corridor into `out`, starting at
    /// `first`, in walking order.
    pub fn agentCorridorPath(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: []PolyRef,
        failure: *err.Failure,
    ) err.Error!void {
        const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .crowd_agent_corridor_path, error.InvalidArgument);
        const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .crowd_agent_corridor_path, error.InvalidArgument);
        const ptr: ?[*]PolyRef = if (out.len == 0) null else out.ptr;
        try err.check(failure, .crowd_agent_corridor_path, c.zrcCrowdAgentCorridorPath(self.handle, ref, first_i, count, ptr));
    }

    /// The position the agent's cached walls were last collected around. See
    /// `LocalBoundary.center` for the sentinel an uncollected one reports.
    pub fn agentBoundaryCenter(self: Crowd, ref: AgentRef, failure: *err.Failure) err.Error![3]f32 {
        var out: [3]f32 = undefined;
        try err.check(failure, .crowd_agent_boundary_center, c.zrcCrowdAgentBoundaryCenter(self.handle, ref, &out));
        return out;
    }

    /// Walls the agent is steering around, at most eight.
    pub fn agentBoundarySegmentCount(self: Crowd, ref: AgentRef, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .crowd_agent_boundary_segment_count, c.zrcCrowdAgentBoundarySegmentCount(self.handle, ref, &out));
        return c.reported(u32, out);
    }

    /// Copies segments of the agent's local boundary into `out`, starting at
    /// `first`, nearest first. Each is the two endpoints.
    pub fn agentBoundarySegments(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: [][6]f32,
        failure: *err.Failure,
    ) err.Error!void {
        const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .crowd_agent_boundary_segments, error.InvalidArgument);
        const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .crowd_agent_boundary_segments, error.InvalidArgument);
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr); // aegis: no-danger: each element of the slice is an array of the pointee type, so the slice is one contiguous run of them
        try err.check(failure, .crowd_agent_boundary_segments, c.zrcCrowdAgentBoundarySegments(self.handle, ref, first_i, count, ptr));
    }

    //-- Where an agent is going ---------------------------------------------

    /// Sends the agent to `position` inside polygon `poly`.
    ///
    /// Planned during the next `update`, so the agent's target state is
    /// `.requesting` until then. `poly` of 0 is `error.InvalidArgument` rather
    /// than a way to clear the target; that is `resetMoveTarget`.
    pub fn requestMoveTarget(
        self: Crowd,
        ref: AgentRef,
        poly: PolyRef,
        position: [3]f32,
        failure: *err.Failure,
    ) err.Error!void {
        try err.check(failure, .crowd_request_move_target, c.zrcCrowdRequestMoveTarget(self.handle, ref, poly, &position));
    }

    /// Steers the agent along `velocity` instead of towards a place. Still
    /// clamped by its speed and acceleration limits, and still constrained to
    /// the navmesh.
    pub fn requestMoveVelocity(self: Crowd, ref: AgentRef, velocity: [3]f32, failure: *err.Failure) err.Error!void {
        try err.check(failure, .crowd_request_move_velocity, c.zrcCrowdRequestMoveVelocity(self.handle, ref, &velocity));
    }

    /// Clears the agent's target. It stops where it is.
    pub fn resetMoveTarget(self: Crowd, ref: AgentRef, failure: *err.Failure) err.Error!void {
        try err.check(failure, .crowd_reset_move_target, c.zrcCrowdResetMoveTarget(self.handle, ref));
    }

    //-- The frame ------------------------------------------------------------

    /// Advances every agent by `dt` seconds. `dt` must be finite and greater
    /// than zero.
    pub fn update(self: Crowd, dt: f32, failure: *err.Failure) err.Error!void {
        try err.check(failure, .crowd_update, c.zrcCrowdUpdate(self.handle, dt, null));
    }

    /// The same, recording what one agent did. `debug` is read for what to
    /// record and written with what was recorded.
    pub fn updateDebug(self: Crowd, dt: f32, debug: *CrowdAgentDebug, failure: *err.Failure) err.Error!void {
        var raw = c.CrowdAgentDebug{
            .agent = c.Maybe(AgentRef).of(debug.agent),
            .samples = if (debug.samples) |d| d.handle else null,
            .opt_start = debug.opt_start,
            .opt_end = debug.opt_end,
        };
        try err.check(failure, .crowd_update, c.zrcCrowdUpdate(self.handle, dt, &raw));
        debug.opt_start = raw.opt_start;
        debug.opt_end = raw.opt_end;
    }

    /// Candidate velocities the avoidance sampler tried during the last
    /// update, summed across every agent. A cost measure, not a state.
    pub fn velocitySampleCount(self: Crowd, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .crowd_velocity_sample_count, c.zrcCrowdVelocitySampleCount(self.handle, &out));
        return c.reported(u32, out);
    }

    //-- What the whole crowd shares -----------------------------------------

    /// Replaces one of the crowd's avoidance configurations. `index` is
    /// `[0, crowd_max_avoidance_params)`.
    pub fn setAvoidanceParams(self: Crowd, index: u32, params: AvoidanceParams, failure: *err.Failure) err.Error!void {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .crowd_set_avoidance_params, error.InvalidArgument);
        try err.check(failure, .crowd_set_avoidance_params, c.zrcCrowdSetAvoidanceParams(self.handle, i, &params));
    }

    /// Reads one back. Every index holds `avoidanceParamsDefault` until set.
    pub fn avoidanceParams(self: Crowd, index: u32, failure: *err.Failure) err.Error!AvoidanceParams {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .crowd_avoidance_params, error.InvalidArgument);
        var out: AvoidanceParams = undefined;
        try err.check(failure, .crowd_avoidance_params, c.zrcCrowdAvoidanceParams(self.handle, i, &out));
        return out;
    }

    /// Replaces one of the crowd's query filters with a copy of `value`,
    /// which need not outlive the call. `index` is `[0, crowd_max_filters)`.
    pub fn setFilter(self: Crowd, index: u32, value: QueryFilter, failure: *err.Failure) err.Error!void {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .crowd_set_filter, error.InvalidArgument);
        try err.check(failure, .crowd_set_filter, c.zrcCrowdSetFilter(self.handle, i, value.handle));
    }

    /// Reads one back.
    pub fn filter(self: Crowd, index: u32, failure: *err.Failure) err.Error!QueryFilterDesc {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .crowd_filter, error.InvalidArgument);
        var out: QueryFilterDesc = undefined;
        try err.check(failure, .crowd_filter, c.zrcCrowdFilter(self.handle, i, &out));
        return out;
    }

    /// The half extents of the box the crowd searches when it places or
    /// recovers an agent. Derived from `max_agent_radius`.
    pub fn queryHalfExtents(self: Crowd, failure: *err.Failure) err.Error![3]f32 {
        var out: [3]f32 = undefined;
        try err.check(failure, .crowd_query_half_extents, c.zrcCrowdQueryHalfExtents(self.handle, &out));
        return out;
    }

    /// The crowd's proximity grid. Borrowed: the crowd owns it, and it dies
    /// with the crowd or at the next `reinit`. Rebuilt from scratch at the
    /// start of every `update`.
    pub fn grid(self: Crowd, failure: *err.Failure) err.Error!BorrowedProximityGrid {
        var handle: *const c.ProximityGrid = undefined;
        try err.check(failure, .crowd_grid, c.zrcCrowdGrid(self.handle, &handle));
        return .{ .handle = handle };
    }

    /// The crowd's path request queue. Borrowed, same lifetime rules.
    pub fn pathQueue(self: Crowd, failure: *err.Failure) err.Error!BorrowedPathQueue {
        var handle: *const c.PathQueue = undefined;
        try err.check(failure, .crowd_path_queue, c.zrcCrowdPathQueue(self.handle, &handle));
        return .{ .handle = handle };
    }

    /// The crowd's own query object. Borrowed, same lifetime rules, and
    /// read-only: starting a sliced search on it would take the node pool out
    /// from under the crowd's per-frame searches.
    pub fn navMeshQuery(self: Crowd, failure: *err.Failure) err.Error!BorrowedNavMeshQuery {
        var handle: *const c.NavMeshQuery = undefined;
        try err.check(failure, .crowd_nav_mesh_query, c.zrcCrowdNavMeshQuery(self.handle, &handle));
        return .{ .handle = handle };
    }
};

//===----------------------------------------------------------------------===//
// The proximity grid
//===----------------------------------------------------------------------===//

/// A spatial hash over the xz-plane: put a footprint in, ask what is near a
/// box, get the ids back.
pub const ProximityGrid = ProximityGridOf(.owned);

/// A grid another object lent, read-only: what `Crowd.grid` returns. It
/// cannot be cleared, filled or freed, and lives as long as its owner.
pub const BorrowedProximityGrid = ProximityGridOf(.borrowed);

fn ProximityGridOf(comptime access: c.Access) type {
    return struct {
        const Self = @This();

        handle: c.Handle(c.ProximityGrid, access),

        /// `pool_size` is `[1, 65534]`; `cell_size` must be finite, positive and
        /// large enough that its reciprocal is finite. One item spans as many
        /// entries as its footprint covers cells.
        pub fn init(pool_size: u32, cell_size: f32, failure: *err.Failure) err.Error!ProximityGrid {
            comptime c.ownedOnly(access);
            const size = aegis.int.cast(i32, pool_size) catch return err.refuse(failure, .proximity_grid_create, error.InvalidArgument);
            var handle: *c.ProximityGrid = undefined;
            try err.check(failure, .proximity_grid_create, c.zrcProximityGridCreate(size, cell_size, &handle));
            return .{ .handle = handle };
        }

        /// Destroying the grid a crowd handed back does nothing: it belongs to the
        /// crowd.
        pub fn deinit(self: Self) void {
            comptime c.ownedOnly(access);
            c.zrcProximityGridDestroy(self.handle);
        }

        /// Empties the grid. The pool and the cell size stay as they were.
        pub fn clear(self: Self, failure: *err.Failure) err.Error!void {
            try err.check(failure, .proximity_grid_clear, c.zrcProximityGridClear(self.handle));
        }

        /// Files `id` under every cell the box covers. The two axes are the
        /// world's x and z.
        ///
        /// Silently drops items once the pool is full, which is upstream's own
        /// behaviour and makes the pool size a capacity question, not an error one.
        pub fn addItem(
            self: Self,
            id: u16,
            min_x: f32,
            min_y: f32,
            max_x: f32,
            max_y: f32,
            failure: *err.Failure,
        ) err.Error!void {
            try err.check(failure, .proximity_grid_add_item, c.zrcProximityGridAddItem(self.handle, id, min_x, min_y, max_x, max_y));
        }

        /// Every distinct id filed under any cell the box covers, into `out`.
        ///
        /// Returns how many were written. `error.BufferTooSmall` when they did not
        /// fit, so a clipped neighbourhood cannot be mistaken for the whole one —
        /// upstream reports no difference.
        pub fn queryItems(
            self: Self,
            min_x: f32,
            min_y: f32,
            max_x: f32,
            max_y: f32,
            out: []u16,
            failure: *err.Failure,
        ) err.Error!usize {
            const max_ids = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .proximity_grid_query_items, error.InvalidArgument);
            const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
            var count: i32 = 0;
            try err.check(failure, .proximity_grid_query_items, c.zrcProximityGridQueryItems(
                self.handle,
                min_x,
                min_y,
                max_x,
                max_y,
                ptr,
                max_ids,
                &count,
            ));
            return c.reported(usize, count);
        }

        /// How many entries are filed at exactly cell `(x, y)`.
        pub fn itemCountAt(self: Self, x: i32, y: i32, failure: *err.Failure) err.Error!u32 {
            var out: i32 = 0;
            try err.check(failure, .proximity_grid_item_count_at, c.zrcProximityGridItemCountAt(self.handle, x, y, &out));
            return c.reported(u32, out);
        }

        /// The cell bounds of everything added since the last clear: min x, min y,
        /// max x, max y. An empty grid reports upstream's inverted sentinel, so
        /// `min > max` is how a caller tells empty from one cell at the origin.
        pub fn bounds(self: Self, failure: *err.Failure) err.Error![4]i32 {
            var out: [4]i32 = undefined;
            try err.check(failure, .proximity_grid_bounds, c.zrcProximityGridBounds(self.handle, &out));
            return out;
        }

        pub fn cellSize(self: Self, failure: *err.Failure) err.Error!f32 {
            var out: f32 = 0;
            try err.check(failure, .proximity_grid_cell_size, c.zrcProximityGridCellSize(self.handle, &out));
            return out;
        }
    };
}

//===----------------------------------------------------------------------===//
// Obstacle avoidance
//===----------------------------------------------------------------------===//

/// Picks the velocity that gets closest to what an agent wanted without
/// running into anything moving nearby.
pub const AvoidanceQuery = struct {
    handle: *c.AvoidanceQuery,

    /// Room for `max_circles` moving obstacles and `max_segments` walls, both
    /// `[1, 65535]`. A crowd asks for 6 and 8.
    pub fn init(max_circles: u32, max_segments: u32, failure: *err.Failure) err.Error!AvoidanceQuery {
        const circles = aegis.int.cast(i32, max_circles) catch return err.refuse(failure, .avoidance_query_create, error.InvalidArgument);
        const segments = aegis.int.cast(i32, max_segments) catch return err.refuse(failure, .avoidance_query_create, error.InvalidArgument);
        var handle: *c.AvoidanceQuery = undefined;
        try err.check(failure, .avoidance_query_create, c.zrcAvoidanceQueryCreate(circles, segments, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: AvoidanceQuery) void {
        c.zrcAvoidanceQueryDestroy(self.handle);
    }

    /// Forgets every obstacle. The capacities stay as they were.
    pub fn reset(self: AvoidanceQuery, failure: *err.Failure) err.Error!void {
        try err.check(failure, .avoidance_query_reset, c.zrcAvoidanceQueryReset(self.handle));
    }

    /// Adds a moving circular obstacle. `error.BufferTooSmall` once
    /// `max_circles` are added — upstream drops it silently, and an obstacle
    /// the sampler never saw is an agent that walks through it.
    pub fn addCircle(
        self: AvoidanceQuery,
        position: [3]f32,
        radius: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        failure: *err.Failure,
    ) err.Error!void {
        try err.check(failure, .avoidance_add_circle, c.zrcAvoidanceAddCircle(
            self.handle,
            &position,
            radius,
            &velocity,
            &desired_velocity,
        ));
    }

    /// Adds a wall segment. Same capacity rule.
    pub fn addSegment(self: AvoidanceQuery, p: [3]f32, q: [3]f32, failure: *err.Failure) err.Error!void {
        try err.check(failure, .avoidance_add_segment, c.zrcAvoidanceAddSegment(self.handle, &p, &q));
    }

    /// What a sampler chose, and how many candidates it scored.
    pub const Chosen = struct {
        velocity: [3]f32,
        samples: u32,
    };

    /// Picks a velocity by scoring a regular grid of candidates.
    /// `params.grid_size` must be at least 2.
    pub fn sampleGrid(
        self: AvoidanceQuery,
        position: [3]f32,
        radius: f32,
        max_speed: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        params: AvoidanceParams,
        debug: ?AvoidanceDebug,
        failure: *err.Failure,
    ) err.Error!Chosen {
        return self.sample(
            c.zrcAvoidanceSampleGrid,
            .avoidance_sample_grid,
            position,
            radius,
            max_speed,
            velocity,
            desired_velocity,
            params,
            debug,
            failure,
        );
    }

    /// Picks a velocity by refining a ring pattern, which is what a crowd
    /// uses. The one entry point here that reaches `cosf` and `sinf`, and so
    /// the reason steering sits outside the package's cross-platform
    /// guarantee.
    pub fn sampleAdaptive(
        self: AvoidanceQuery,
        position: [3]f32,
        radius: f32,
        max_speed: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        params: AvoidanceParams,
        debug: ?AvoidanceDebug,
        failure: *err.Failure,
    ) err.Error!Chosen {
        return self.sample(
            c.zrcAvoidanceSampleAdaptive,
            .avoidance_sample_adaptive,
            position,
            radius,
            max_speed,
            velocity,
            desired_velocity,
            params,
            debug,
            failure,
        );
    }

    fn sample(
        self: AvoidanceQuery,
        comptime call: anytype,
        comptime name: err.Call,
        position: [3]f32,
        radius: f32,
        max_speed: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        params: AvoidanceParams,
        debug: ?AvoidanceDebug,
        failure: *err.Failure,
    ) err.Error!Chosen {
        var chosen: [3]f32 = .{ 0, 0, 0 };
        var samples: i32 = 0;
        try err.check(failure, name, call(
            self.handle,
            &position,
            radius,
            max_speed,
            &velocity,
            &desired_velocity,
            &params,
            if (debug) |d| d.handle else null,
            &chosen,
            &samples,
        ));
        return .{ .velocity = chosen, .samples = c.reported(u32, samples) };
    }

    pub fn circleCount(self: AvoidanceQuery, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .avoidance_circle_count, c.zrcAvoidanceCircleCount(self.handle, &out));
        return c.reported(u32, out);
    }

    /// One of them. The index is bounded here; upstream's accessor bounds
    /// nothing.
    pub fn circleAt(self: AvoidanceQuery, index: u32, failure: *err.Failure) err.Error!AvoidanceCircle {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .avoidance_circle_at, error.InvalidArgument);
        var out: AvoidanceCircle = undefined;
        try err.check(failure, .avoidance_circle_at, c.zrcAvoidanceCircleAt(self.handle, i, &out));
        return out;
    }

    pub fn segmentCount(self: AvoidanceQuery, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .avoidance_segment_count, c.zrcAvoidanceSegmentCount(self.handle, &out));
        return c.reported(u32, out);
    }

    /// One of them. Same bounding.
    pub fn segmentAt(self: AvoidanceQuery, index: u32, failure: *err.Failure) err.Error!AvoidanceSegment {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .avoidance_segment_at, error.InvalidArgument);
        var out: AvoidanceSegment = undefined;
        try err.check(failure, .avoidance_segment_at, c.zrcAvoidanceSegmentAt(self.handle, i, &out));
        return out;
    }
};

/// Records the candidate velocities a sampler tried, for a debug view.
pub const AvoidanceDebug = struct {
    handle: *c.AvoidanceDebug,

    /// `max_samples` is `[1, 65535]`.
    pub fn init(max_samples: u32, failure: *err.Failure) err.Error!AvoidanceDebug {
        const samples = aegis.int.cast(i32, max_samples) catch return err.refuse(failure, .avoidance_debug_create, error.InvalidArgument);
        var handle: *c.AvoidanceDebug = undefined;
        try err.check(failure, .avoidance_debug_create, c.zrcAvoidanceDebugCreate(samples, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: AvoidanceDebug) void {
        c.zrcAvoidanceDebugDestroy(self.handle);
    }

    /// Forgets every sample. The capacity stays as it was.
    pub fn reset(self: AvoidanceDebug, failure: *err.Failure) err.Error!void {
        try err.check(failure, .avoidance_debug_reset, c.zrcAvoidanceDebugReset(self.handle));
    }

    /// Records one candidate by hand, for a host scoring velocities itself.
    /// `error.BufferTooSmall` once full.
    pub fn addSample(self: AvoidanceDebug, sample: AvoidanceSample, failure: *err.Failure) err.Error!void {
        try err.check(failure, .avoidance_debug_add_sample, c.zrcAvoidanceDebugAddSample(self.handle, &sample));
    }

    /// Rescales every recorded penalty into `[0, 1]` against the largest.
    /// Idempotent; a recorder with no samples is a no-op.
    pub fn normalize(self: AvoidanceDebug, failure: *err.Failure) err.Error!void {
        try err.check(failure, .avoidance_debug_normalize, c.zrcAvoidanceDebugNormalize(self.handle));
    }

    pub fn sampleCount(self: AvoidanceDebug, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .avoidance_debug_sample_count, c.zrcAvoidanceDebugSampleCount(self.handle, &out));
        return c.reported(u32, out);
    }

    /// One of them. The index is bounded here; all seven of upstream's
    /// accessors index their arrays unchecked.
    pub fn sampleAt(self: AvoidanceDebug, index: u32, failure: *err.Failure) err.Error!AvoidanceSample {
        const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .avoidance_debug_sample_at, error.InvalidArgument);
        var out: AvoidanceSample = undefined;
        try err.check(failure, .avoidance_debug_sample_at, c.zrcAvoidanceDebugSampleAt(self.handle, i, &out));
        return out;
    }
};

//===----------------------------------------------------------------------===//
// The path corridor
//===----------------------------------------------------------------------===//

/// The polygons an agent is walking through, and the position and target
/// inside them, edited as the agent moves.
pub const PathCorridor = struct {
    handle: *c.PathCorridor,

    /// `max_path` is `[path_corridor_min_path, 65535]`. The minimum is not a
    /// preference: three of upstream's own operations write past a shorter
    /// buffer in every build configuration. See `ffi/zrecast.h`.
    pub fn init(max_path: u32, failure: *err.Failure) err.Error!PathCorridor {
        const size = aegis.int.cast(i32, max_path) catch return err.refuse(failure, .path_corridor_create, error.InvalidArgument);
        var handle: *c.PathCorridor = undefined;
        try err.check(failure, .path_corridor_create, c.zrcPathCorridorCreate(size, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: PathCorridor) void {
        c.zrcPathCorridorDestroy(self.handle);
    }

    /// Empties the corridor and puts it at `position` inside `poly`, which may
    /// be null for a corridor whose agent is not on the navmesh.
    pub fn reset(self: PathCorridor, poly: ?PolyRef, position: [3]f32, failure: *err.Failure) err.Error!void {
        try err.check(failure, .path_corridor_reset, c.zrcPathCorridorReset(self.handle, c.Maybe(PolyRef).of(poly), &position));
    }

    /// Replaces the corridor and its target. `path` must hold between 1 and
    /// `max_path` polygons.
    pub fn setCorridor(
        self: PathCorridor,
        target: [3]f32,
        polys: []const PolyRef,
        failure: *err.Failure,
    ) err.Error!void {
        const count = aegis.int.cast(i32, polys.len) catch return err.refuse(failure, .path_corridor_set_corridor, error.InvalidArgument);
        if (polys.len == 0) return err.refuse(failure, .path_corridor_set_corridor, error.InvalidArgument);
        try err.check(failure, .path_corridor_set_corridor, c.zrcPathCorridorSetCorridor(self.handle, &target, polys.ptr, count));
    }

    /// String-pulls the corridor ahead into the corners to walk, returning how
    /// many were produced.
    ///
    /// **At most `out.len - 1` corners come back**, upstream's own behaviour:
    /// the string-pull reserves the last slot for the corridor's end point and
    /// drops the first when the agent stands on it. `out` holds at least two.
    pub fn findCorners(
        self: PathCorridor,
        query: NavMeshQuery,
        filter: QueryFilter,
        out: []CrowdCorner,
        failure: *err.Failure,
    ) err.Error!usize {
        const max_corners = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .path_corridor_find_corners, error.InvalidArgument);
        const ptr: ?[*]CrowdCorner = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        try err.check(failure, .path_corridor_find_corners, c.zrcPathCorridorFindCorners(
            self.handle,
            query.handle,
            filter.handle,
            ptr,
            max_corners,
            &count,
        ));
        return c.reported(usize, count);
    }

    /// Shortens the corridor if `next` is directly visible from the position.
    /// `range` must be finite and positive. Left as it was when nothing could
    /// be shortened.
    pub fn optimizeVisibility(
        self: PathCorridor,
        next: [3]f32,
        range: f32,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!void {
        try err.check(failure, .path_corridor_optimize_visibility, c.zrcPathCorridorOptimizeVisibility(
            self.handle,
            &next,
            range,
            query.handle,
            filter.handle,
        ));
    }

    /// Re-plans the corridor locally, following the world rather than the
    /// straight line. Reports whether anything changed.
    pub fn optimizeTopology(
        self: PathCorridor,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!bool {
        var changed: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_optimize_topology, c.zrcPathCorridorOptimizeTopology(
            self.handle,
            query.handle,
            filter.handle,
            &changed,
        ));
        return changed != c.c_false;
    }

    /// What crossing an off-mesh connection produced.
    pub const OffMeshMove = struct {
        /// The polygon left, then the one entered.
        refs: [2]PolyRef,
        start: [3]f32,
        end: [3]f32,
        /// False when the corridor was not at that connection, which is not an
        /// error.
        moved: bool,
    };

    /// Advances the corridor across the off-mesh connection at its front.
    pub fn moveOverOffMeshConnection(
        self: PathCorridor,
        offmesh_poly: PolyRef,
        query: NavMeshQuery,
        failure: *err.Failure,
    ) err.Error!OffMeshMove {
        var out = OffMeshMove{
            .refs = .{ 0, 0 },
            .start = .{ 0, 0, 0 },
            .end = .{ 0, 0, 0 },
            .moved = false,
        };
        var moved: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_move_over_offmesh_connection, c.zrcPathCorridorMoveOverOffmeshConnection(
            self.handle,
            offmesh_poly,
            query.handle,
            &out.refs,
            &out.start,
            &out.end,
            &moved,
        ));
        out.moved = moved != c.c_false;
        return out;
    }

    /// Puts `safe_poly` at the front when the first polygon has gone.
    pub fn fixStart(
        self: PathCorridor,
        safe_poly: PolyRef,
        safe_position: [3]f32,
        failure: *err.Failure,
    ) err.Error!bool {
        var fixed: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_fix_start, c.zrcPathCorridorFixStart(self.handle, safe_poly, &safe_position, &fixed));
        return fixed != c.c_false;
    }

    /// Cuts the corridor back to the last polygon still valid, falling back to
    /// `safe_poly` when none is.
    pub fn trimInvalid(
        self: PathCorridor,
        safe_poly: PolyRef,
        safe_position: [3]f32,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!bool {
        var trimmed: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_trim_invalid, c.zrcPathCorridorTrimInvalid(
            self.handle,
            safe_poly,
            &safe_position,
            query.handle,
            filter.handle,
            &trimmed,
        ));
        return trimmed != c.c_false;
    }

    /// Whether the first `max_lookahead` polygons still exist and still pass
    /// the filter.
    pub fn isValid(
        self: PathCorridor,
        max_lookahead: u32,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!bool {
        const lookahead = aegis.int.cast(i32, max_lookahead) catch
            return err.refuse(failure, .path_corridor_is_valid, error.InvalidArgument);
        var valid: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_is_valid, c.zrcPathCorridorIsValid(
            self.handle,
            lookahead,
            query.handle,
            filter.handle,
            &valid,
        ));
        return valid != c.c_false;
    }

    /// Slides the position to `position`, dropping the polygons left behind.
    /// The position ends up on the navmesh, not necessarily where it was asked
    /// to go.
    pub fn movePosition(
        self: PathCorridor,
        position: [3]f32,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!bool {
        var moved: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_move_position, c.zrcPathCorridorMovePosition(
            self.handle,
            &position,
            query.handle,
            filter.handle,
            &moved,
        ));
        return moved != c.c_false;
    }

    /// The same for the target end of the corridor.
    pub fn moveTargetPosition(
        self: PathCorridor,
        position: [3]f32,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!bool {
        var moved: c.Bool = c.c_false;
        try err.check(failure, .path_corridor_move_target_position, c.zrcPathCorridorMoveTargetPosition(
            self.handle,
            &position,
            query.handle,
            filter.handle,
            &moved,
        ));
        return moved != c.c_false;
    }

    /// Where the corridor is, where it ends, and how long it is.
    pub fn info(self: PathCorridor, failure: *err.Failure) err.Error!PathCorridorInfo {
        var out: PathCorridorInfo = undefined;
        try err.check(failure, .path_corridor_info, c.zrcPathCorridorInfo(self.handle, &out));
        return out;
    }

    /// Copies polygons of the corridor into `out`, starting at `first`, in
    /// walking order.
    pub fn path(self: PathCorridor, first: u32, out: []PolyRef, failure: *err.Failure) err.Error!void {
        const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .path_corridor_path, error.InvalidArgument);
        const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .path_corridor_path, error.InvalidArgument);
        const ptr: ?[*]PolyRef = if (out.len == 0) null else out.ptr;
        try err.check(failure, .path_corridor_path, c.zrcPathCorridorPath(self.handle, first_i, count, ptr));
    }
};

//===----------------------------------------------------------------------===//
// Corridor splicing
//
// The three primitives a corridor is built from, on a host's own array. Each
// edits `path[0..path_count]` in place and returns the new length.
// `visited_count` above `path.len` is what produces the negative-length
// `memmove` upstream; both counts are bounded here.
//===----------------------------------------------------------------------===//

/// For a position that slid forwards.
pub fn mergeCorridorStartMoved(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
    failure: *err.Failure,
) err.Error!usize {
    return merge(c.zrcMergeCorridorStartMoved, .merge_corridor_start_moved, path, path_count, visited, failure);
}

/// For a target that slid forwards.
pub fn mergeCorridorEndMoved(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
    failure: *err.Failure,
) err.Error!usize {
    return merge(c.zrcMergeCorridorEndMoved, .merge_corridor_end_moved, path, path_count, visited, failure);
}

/// For a visibility optimisation that skipped ahead.
pub fn mergeCorridorStartShortcut(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
    failure: *err.Failure,
) err.Error!usize {
    return merge(c.zrcMergeCorridorStartShortcut, .merge_corridor_start_shortcut, path, path_count, visited, failure);
}

fn merge(
    comptime call: anytype,
    comptime name: err.Call,
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
    failure: *err.Failure,
) err.Error!usize {
    const max_path = aegis.int.cast(i32, path.len) catch return err.refuse(failure, name, error.InvalidArgument);
    const count = aegis.int.cast(i32, path_count) catch return err.refuse(failure, name, error.InvalidArgument);
    const visited_count = aegis.int.cast(i32, visited.len) catch
        return err.refuse(failure, name, error.InvalidArgument);
    var out: i32 = 0;
    try err.check(failure, name, call(path.ptr, count, max_path, visited.ptr, visited_count, &out));
    return c.reported(usize, out);
}

//===----------------------------------------------------------------------===//
// The local boundary
//===----------------------------------------------------------------------===//

/// The walls near an agent, cached so they are sampled once rather than every
/// frame.
pub const LocalBoundary = struct {
    handle: *c.LocalBoundary,

    pub fn init(failure: *err.Failure) err.Error!LocalBoundary {
        var handle: *c.LocalBoundary = undefined;
        try err.check(failure, .local_boundary_create, c.zrcLocalBoundaryCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: LocalBoundary) void {
        c.zrcLocalBoundaryDestroy(self.handle);
    }

    /// Forgets every segment and every polygon it was collected from.
    pub fn reset(self: LocalBoundary, failure: *err.Failure) err.Error!void {
        try err.check(failure, .local_boundary_reset, c.zrcLocalBoundaryReset(self.handle));
    }

    /// Re-collects the walls within `range` of `position`, starting from
    /// `poly`. Keeps the eight nearest. A null `poly` resets it instead.
    pub fn update(
        self: LocalBoundary,
        poly: ?PolyRef,
        position: [3]f32,
        range: f32,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!void {
        try err.check(failure, .local_boundary_update, c.zrcLocalBoundaryUpdate(
            self.handle,
            c.Maybe(PolyRef).of(poly),
            &position,
            range,
            query.handle,
            filter.handle,
        ));
    }

    /// Whether every polygon it was collected from still exists and still
    /// passes the filter. False means it should be updated again.
    pub fn isValid(
        self: LocalBoundary,
        query: NavMeshQuery,
        filter: QueryFilter,
        failure: *err.Failure,
    ) err.Error!bool {
        var valid: c.Bool = c.c_false;
        try err.check(failure, .local_boundary_is_valid, c.zrcLocalBoundaryIsValid(self.handle, query.handle, filter.handle, &valid));
        return valid != c.c_false;
    }

    /// The position it was last collected around.
    ///
    /// A boundary that has never been updated reports upstream's own sentinel,
    /// `FLT_MAX` on all three axes, rather than a zero.
    pub fn center(self: LocalBoundary, failure: *err.Failure) err.Error![3]f32 {
        var out: [3]f32 = undefined;
        try err.check(failure, .local_boundary_center, c.zrcLocalBoundaryCenter(self.handle, &out));
        return out;
    }

    /// Segments currently held, at most eight.
    pub fn segmentCount(self: LocalBoundary, failure: *err.Failure) err.Error!u32 {
        var out: i32 = 0;
        try err.check(failure, .local_boundary_segment_count, c.zrcLocalBoundarySegmentCount(self.handle, &out));
        return c.reported(u32, out);
    }

    /// Copies segments into `out`, starting at `first`, nearest first. Each is
    /// the two endpoints.
    pub fn segments(self: LocalBoundary, first: u32, out: [][6]f32, failure: *err.Failure) err.Error!void {
        const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .local_boundary_segments, error.InvalidArgument);
        const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .local_boundary_segments, error.InvalidArgument);
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr); // aegis: no-danger: each element of the slice is an array of the pointee type, so the slice is one contiguous run of them
        try err.check(failure, .local_boundary_segments, c.zrcLocalBoundarySegments(self.handle, first_i, count, ptr));
    }
};

//===----------------------------------------------------------------------===//
// The path queue
//===----------------------------------------------------------------------===//

/// Long searches, run a slice at a time across frames. Eight at once, which is
/// upstream's fixed capacity.
pub const PathQueue = PathQueueOf(.owned);

/// A queue another object lent, read-only: what `Crowd.pathQueue` returns.
/// It cannot take requests, run or be freed, and lives as long as its owner.
pub const BorrowedPathQueue = PathQueueOf(.borrowed);

fn PathQueueOf(comptime access: c.Access) type {
    return struct {
        const Self = @This();

        handle: c.Handle(c.PathQueue, access),

        /// `max_path_size` is the longest path one request may return and
        /// `max_search_nodes` the pool each search gets; a crowd uses 256 and
        /// 4096. Both are `[1, 65535]`. `navmesh` is borrowed and must outlive the
        /// queue.
        pub fn init(
            navmesh: NavMesh,
            max_path_size: u32,
            max_search_nodes: u32,
            failure: *err.Failure,
        ) err.Error!PathQueue {
            comptime c.ownedOnly(access);
            const path_size = aegis.int.cast(i32, max_path_size) catch
                return err.refuse(failure, .path_queue_create, error.InvalidArgument);
            const nodes = aegis.int.cast(i32, max_search_nodes) catch
                return err.refuse(failure, .path_queue_create, error.InvalidArgument);
            var handle: *c.PathQueue = undefined;
            try err.check(failure, .path_queue_create, c.zrcPathQueueCreate(navmesh.handle, path_size, nodes, &handle));
            return .{ .handle = handle };
        }

        /// Destroying the queue a crowd handed back does nothing: it belongs to
        /// the crowd.
        pub fn deinit(self: Self) void {
            comptime c.ownedOnly(access);
            c.zrcPathQueueDestroy(self.handle);
        }

        /// Advances the request at the front by at most `max_iters` iterations.
        ///
        /// One call advances one request. A request that does not finish within
        /// one call's budget holds the front and the others wait behind it, and a
        /// finished result is discarded two calls after it became ready — both
        /// upstream's behaviour.
        pub fn update(self: Self, max_iters: u32, failure: *err.Failure) err.Error!void {
            const iters = aegis.int.cast(i32, max_iters) catch return err.refuse(failure, .path_queue_update, error.InvalidArgument);
            try err.check(failure, .path_queue_update, c.zrcPathQueueUpdate(self.handle, iters));
        }

        /// Submits a search. The result is null when every slot is taken, which
        /// is not an error: it is how a host learns to try again.
        ///
        /// `filter` is copied for the request's lifetime. Upstream keeps the
        /// caller's pointer and reads through it on every later update.
        pub fn request(
            self: Self,
            start_poly: PolyRef,
            end_poly: PolyRef,
            start_position: [3]f32,
            end_position: [3]f32,
            filter: QueryFilter,
            failure: *err.Failure,
        ) err.Error!?PathRequestRef {
            var ref: c.Maybe(PathRequestRef) = .none;
            try err.check(failure, .path_queue_request, c.zrcPathQueueRequest(
                self.handle,
                start_poly,
                end_poly,
                &start_position,
                &end_position,
                filter.handle,
                &ref,
            ));
            return ref.get();
        }

        /// Where a request has got to. An unknown reference is `.unknown` rather
        /// than an error.
        pub fn requestStatus(self: Self, ref: PathRequestRef, failure: *err.Failure) err.Error!PathRequestState {
            var state: PathRequestState = .unknown;
            try err.check(failure, .path_queue_request_status, c.zrcPathQueueRequestStatus(self.handle, ref, &state));
            return state;
        }

        /// Takes a finished request's path and frees its slot; returns the length.
        ///
        /// `error.SearchInProgress` when it has not finished and `error.NotFound`
        /// for a reference no slot holds; the request survives both. The slot is
        /// freed whichever way a successful call ends, so size `out` at the
        /// queue's `max_path_size` rather than retrying after `error.BufferTooSmall`.
        pub fn result(self: Self, ref: PathRequestRef, out: []PolyRef, failure: *err.Failure) err.Error!usize {
            const max_path = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .path_queue_result, error.InvalidArgument);
            const ptr: ?[*]PolyRef = if (out.len == 0) null else out.ptr;
            var count: i32 = 0;
            try err.check(failure, .path_queue_result, c.zrcPathQueueResult(self.handle, ref, ptr, max_path, &count));
            return c.reported(usize, count);
        }

        /// The query object the queue runs its searches on. Borrowed; it dies with
        /// the queue.
        pub fn navMeshQuery(self: Self, failure: *err.Failure) err.Error!BorrowedNavMeshQuery {
            var handle: *const c.NavMeshQuery = undefined;
            try err.check(failure, .path_queue_nav_mesh_query, c.zrcPathQueueNavMeshQuery(self.handle, &handle));
            return .{ .handle = handle };
        }
    };
}
