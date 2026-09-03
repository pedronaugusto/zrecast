//! Crowds, and the four pieces they steer with.
//!
//! A `NavMesh` says what is walkable and a `NavMeshQuery` answers where an
//! agent could go. A `Crowd` moves it: every frame it re-plans each agent's
//! local path, collects the neighbours and walls nearby, picks a velocity that
//! avoids both, and integrates under acceleration and speed limits.
//!
//! The crowd owns an agent's movement. Position and velocity are outputs it
//! recomputes each frame from the parameters and the target; what a host
//! controls is `Crowd.setAgentParams` and the three move-request calls.
//!
//! `ProximityGrid`, `AvoidanceQuery`, `PathCorridor`, `LocalBoundary` and
//! `PathQueue` are the parts a crowd drives internally, each usable on its own
//! by a host steering a single character.
//!
//! The package's byte-for-byte cook guarantee does not reach here: the
//! adaptive velocity sampler calls `cosf` and `sinf`, and steering is a
//! frame-local decision recomputed the next frame. A host that needs a
//! reproducible replay should record its inputs. See `ffi/zrecast.h`.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const NavMesh = @import("navmesh.zig").NavMesh;
const NavMeshQuery = @import("query.zig").NavMeshQuery;
const Filter = @import("query.zig").Filter;
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

/// Names one submitted search in a `PathQueue`. `path_request_none` is never
/// a live request.
pub const PathRequestRef = c.PathRequestRef;
pub const path_request_none = c.path_request_none;

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
    agent: AgentRef = 0,
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
    /// `navmesh` is borrowed and must outlive the crowd. Tiles may come and go
    /// underneath: an agent whose polygon is removed drops to `.invalid` and
    /// recovers when one is there again.
    ///
    /// `max_agents` is `[1, crowd_max_agents]`; `max_agent_radius` sizes the
    /// crowd's proximity grid and its placement search box, and an agent added
    /// with a larger radius is refused.
    pub fn init(navmesh: NavMesh, max_agents: u32, max_agent_radius: f32) err.Error!Crowd {
        const agents = std.math.cast(i32, max_agents) orelse
            return err.Error.InvalidArgument;
        var handle: *c.Crowd = undefined;
        try err.check(c.zrcCrowdCreate(navmesh.handle, agents, max_agent_radius, &handle));
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
    pub fn reinit(self: Crowd, navmesh: NavMesh, max_agents: u32, max_agent_radius: f32) err.Error!void {
        const agents = std.math.cast(i32, max_agents) orelse
            return err.Error.InvalidArgument;
        try err.check(c.zrcCrowdInit(self.handle, navmesh.handle, agents, max_agent_radius));
    }

    //-- Agents ---------------------------------------------------------------

    /// Adds an agent at the navmesh position nearest `position`.
    ///
    /// The agent is placed on the navmesh, not at the point asked for. When
    /// nothing is near enough it is still added, at the requested point, in
    /// state `.invalid` — it exists and is not steered.
    ///
    /// `error.CrowdFull` when every slot is taken.
    pub fn addAgent(
        self: Crowd,
        position: [3]f32,
        params: CrowdAgentParams,
    ) err.Error!AgentRef {
        var ref: AgentRef = 0;
        try err.check(c.zrcCrowdAddAgent(self.handle, &position, &params, &ref));
        return ref;
    }

    /// Removes an agent. Its slot becomes available and `ref` stops resolving.
    pub fn removeAgent(self: Crowd, ref: AgentRef) err.Error!void {
        try err.check(c.zrcCrowdRemoveAgent(self.handle, ref));
    }

    /// Replaces an agent's parameters. Position, velocity and target are
    /// untouched.
    pub fn setAgentParams(self: Crowd, ref: AgentRef, params: CrowdAgentParams) err.Error!void {
        try err.check(c.zrcCrowdSetAgentParams(self.handle, ref, &params));
    }

    /// Everything about one agent except its corridor and its local boundary,
    /// which have accessors of their own.
    pub fn agent(self: Crowd, ref: AgentRef) err.Error!CrowdAgent {
        var out: CrowdAgent = undefined;
        try err.check(c.zrcCrowdAgentInfo(self.handle, ref, &out));
        return out;
    }

    /// Agent slots the crowd was created for, live or not. Not a population;
    /// that is `activeAgentCount`.
    pub fn agentCapacity(self: Crowd) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcCrowdAgentCapacity(self.handle, &out));
        return @intCast(out);
    }

    /// Agents currently in the crowd.
    pub fn activeAgentCount(self: Crowd) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcCrowdActiveAgentCount(self.handle, &out));
        return @intCast(out);
    }

    /// Every active agent's reference.
    ///
    /// `out` receives however many fit; the total is always the return value,
    /// so a caller compares it against `out.len` to see whether a second,
    /// larger call is needed rather than treating a short buffer as failure.
    /// An empty `out` asks only for the count.
    pub fn activeAgents(self: Crowd, out: []AgentRef) err.Error!usize {
        const max_agents = std.math.cast(i32, out.len) orelse
            return err.Error.InvalidArgument;
        const ptr: ?[*]AgentRef = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        const result = c.zrcCrowdActiveAgents(self.handle, ptr, max_agents, &count);
        if (result == .buffer_too_small) return @intCast(count);
        try err.check(result);
        return @intCast(count);
    }

    /// The reference of the agent in slot `index`, or 0 when the slot is free.
    pub fn agentRefAt(self: Crowd, index: u32) err.Error!AgentRef {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var ref: AgentRef = 0;
        try err.check(c.zrcCrowdAgentRefAt(self.handle, i, &ref));
        return ref;
    }

    /// Copies corners of the agent's local path into `out`, starting at
    /// `first`. The total is `CrowdAgent.corner_count`, at most
    /// `crowd_max_corners`; the last is where the agent is heading.
    pub fn agentCorners(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: []CrowdCorner,
    ) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]CrowdCorner = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCrowdAgentCorners(self.handle, ref, first_i, count, ptr));
    }

    /// The same for its neighbours, nearest first. The total is
    /// `CrowdAgent.neighbour_count`, at most `crowd_max_neighbours`.
    pub fn agentNeighbours(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: []CrowdNeighbour,
    ) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]CrowdNeighbour = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCrowdAgentNeighbours(self.handle, ref, first_i, count, ptr));
    }

    /// The corridor the agent is walking, read-only. The standalone
    /// `PathCorridor` is the same object, editable, for a host steering one
    /// character itself.
    pub fn agentCorridor(self: Crowd, ref: AgentRef) err.Error!PathCorridorInfo {
        var out: PathCorridorInfo = undefined;
        try err.check(c.zrcCrowdAgentCorridorInfo(self.handle, ref, &out));
        return out;
    }

    /// Copies polygons of the agent's corridor into `out`, starting at
    /// `first`, in walking order.
    pub fn agentCorridorPath(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: []PolyRef,
    ) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]PolyRef = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcCrowdAgentCorridorPath(self.handle, ref, first_i, count, ptr));
    }

    /// The position the agent's cached walls were last collected around. See
    /// `LocalBoundary.center` for the sentinel an uncollected one reports.
    pub fn agentBoundaryCenter(self: Crowd, ref: AgentRef) err.Error![3]f32 {
        var out: [3]f32 = undefined;
        try err.check(c.zrcCrowdAgentBoundaryCenter(self.handle, ref, &out));
        return out;
    }

    /// Walls the agent is steering around, at most eight.
    pub fn agentBoundarySegmentCount(self: Crowd, ref: AgentRef) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcCrowdAgentBoundarySegmentCount(self.handle, ref, &out));
        return @intCast(out);
    }

    /// Copies segments of the agent's local boundary into `out`, starting at
    /// `first`, nearest first. Each is the two endpoints.
    pub fn agentBoundarySegments(
        self: Crowd,
        ref: AgentRef,
        first: u32,
        out: [][6]f32,
    ) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcCrowdAgentBoundarySegments(self.handle, ref, first_i, count, ptr));
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
    ) err.Error!void {
        try err.check(c.zrcCrowdRequestMoveTarget(self.handle, ref, poly, &position));
    }

    /// Steers the agent along `velocity` instead of towards a place. Still
    /// clamped by its speed and acceleration limits, and still constrained to
    /// the navmesh.
    pub fn requestMoveVelocity(self: Crowd, ref: AgentRef, velocity: [3]f32) err.Error!void {
        try err.check(c.zrcCrowdRequestMoveVelocity(self.handle, ref, &velocity));
    }

    /// Clears the agent's target. It stops where it is.
    pub fn resetMoveTarget(self: Crowd, ref: AgentRef) err.Error!void {
        try err.check(c.zrcCrowdResetMoveTarget(self.handle, ref));
    }

    //-- The frame ------------------------------------------------------------

    /// Advances every agent by `dt` seconds. `dt` must be finite and greater
    /// than zero.
    pub fn update(self: Crowd, dt: f32) err.Error!void {
        try err.check(c.zrcCrowdUpdate(self.handle, dt, null));
    }

    /// The same, recording what one agent did. `debug` is read for what to
    /// record and written with what was recorded.
    pub fn updateDebug(self: Crowd, dt: f32, debug: *CrowdAgentDebug) err.Error!void {
        var raw = c.CrowdAgentDebug{
            .agent = debug.agent,
            .samples = if (debug.samples) |d| d.handle else null,
            .opt_start = debug.opt_start,
            .opt_end = debug.opt_end,
        };
        try err.check(c.zrcCrowdUpdate(self.handle, dt, &raw));
        debug.opt_start = raw.opt_start;
        debug.opt_end = raw.opt_end;
    }

    /// Candidate velocities the avoidance sampler tried during the last
    /// update, summed across every agent. A cost measure, not a state.
    pub fn velocitySampleCount(self: Crowd) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcCrowdVelocitySampleCount(self.handle, &out));
        return @intCast(out);
    }

    //-- What the whole crowd shares -----------------------------------------

    /// Replaces one of the crowd's avoidance configurations. `index` is
    /// `[0, crowd_max_avoidance_params)`.
    pub fn setAvoidanceParams(self: Crowd, index: u32, params: AvoidanceParams) err.Error!void {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        try err.check(c.zrcCrowdSetAvoidanceParams(self.handle, i, &params));
    }

    /// Reads one back. Every index holds `avoidanceParamsDefault` until set.
    pub fn avoidanceParams(self: Crowd, index: u32) err.Error!AvoidanceParams {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: AvoidanceParams = undefined;
        try err.check(c.zrcCrowdAvoidanceParams(self.handle, i, &out));
        return out;
    }

    /// Replaces one of the crowd's query filters. `index` is
    /// `[0, crowd_max_filters)`.
    pub fn setFilter(self: Crowd, index: u32, value: Filter) err.Error!void {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        try err.check(c.zrcCrowdSetFilter(self.handle, i, &value));
    }

    /// Reads one back.
    pub fn filter(self: Crowd, index: u32) err.Error!Filter {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: Filter = undefined;
        try err.check(c.zrcCrowdFilter(self.handle, i, &out));
        return out;
    }

    /// The half extents of the box the crowd searches when it places or
    /// recovers an agent. Derived from `max_agent_radius`.
    pub fn queryHalfExtents(self: Crowd) err.Error![3]f32 {
        var out: [3]f32 = undefined;
        try err.check(c.zrcCrowdQueryHalfExtents(self.handle, &out));
        return out;
    }

    /// The crowd's proximity grid. Borrowed: the crowd owns it, and it dies
    /// with the crowd or at the next `reinit`. Rebuilt from scratch at the
    /// start of every `update`.
    pub fn grid(self: Crowd) err.Error!ProximityGrid {
        var handle: *const c.ProximityGrid = undefined;
        try err.check(c.zrcCrowdGrid(self.handle, &handle));
        return .{ .handle = @constCast(handle) };
    }

    /// The crowd's path request queue. Borrowed, same lifetime rules.
    pub fn pathQueue(self: Crowd) err.Error!PathQueue {
        var handle: *const c.PathQueue = undefined;
        try err.check(c.zrcCrowdPathQueue(self.handle, &handle));
        return .{ .handle = @constCast(handle) };
    }

    /// The crowd's own query object. Borrowed, same lifetime rules, and
    /// read-only: starting a sliced search on it would take the node pool out
    /// from under the crowd's per-frame searches.
    pub fn navMeshQuery(self: Crowd) err.Error!NavMeshQuery {
        var handle: *const c.NavMeshQuery = undefined;
        try err.check(c.zrcCrowdNavMeshQuery(self.handle, &handle));
        return .{ .handle = @constCast(handle) };
    }
};

//===----------------------------------------------------------------------===//
// The proximity grid
//===----------------------------------------------------------------------===//

/// A spatial hash over the xz-plane: put a footprint in, ask what is near a
/// box, get the ids back.
pub const ProximityGrid = struct {
    handle: *c.ProximityGrid,

    /// `pool_size` is `[1, 65534]`; `cell_size` must be finite, positive and
    /// large enough that its reciprocal is finite. One item spans as many
    /// entries as its footprint covers cells.
    pub fn init(pool_size: u32, cell_size: f32) err.Error!ProximityGrid {
        const size = std.math.cast(i32, pool_size) orelse return err.Error.InvalidArgument;
        var handle: *c.ProximityGrid = undefined;
        try err.check(c.zrcProximityGridCreate(size, cell_size, &handle));
        return .{ .handle = handle };
    }

    /// Destroying the grid a crowd handed back does nothing: it belongs to the
    /// crowd.
    pub fn deinit(self: ProximityGrid) void {
        c.zrcProximityGridDestroy(self.handle);
    }

    /// Empties the grid. The pool and the cell size stay as they were.
    pub fn clear(self: ProximityGrid) err.Error!void {
        try err.check(c.zrcProximityGridClear(self.handle));
    }

    /// Files `id` under every cell the box covers. The two axes are the
    /// world's x and z.
    ///
    /// Silently drops items once the pool is full, which is upstream's own
    /// behaviour and makes the pool size a capacity question, not an error one.
    pub fn addItem(
        self: ProximityGrid,
        id: u16,
        min_x: f32,
        min_y: f32,
        max_x: f32,
        max_y: f32,
    ) err.Error!void {
        try err.check(c.zrcProximityGridAddItem(self.handle, id, min_x, min_y, max_x, max_y));
    }

    /// Every distinct id filed under any cell the box covers, into `out`.
    ///
    /// Returns how many were written. `error.BufferTooSmall` when they did not
    /// fit, so a clipped neighbourhood cannot be mistaken for the whole one —
    /// upstream reports no difference.
    pub fn queryItems(
        self: ProximityGrid,
        min_x: f32,
        min_y: f32,
        max_x: f32,
        max_y: f32,
        out: []u16,
    ) err.Error!usize {
        const max_ids = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        try err.check(c.zrcProximityGridQueryItems(
            self.handle,
            min_x,
            min_y,
            max_x,
            max_y,
            ptr,
            max_ids,
            &count,
        ));
        return @intCast(count);
    }

    /// How many entries are filed at exactly cell `(x, y)`.
    pub fn itemCountAt(self: ProximityGrid, x: i32, y: i32) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcProximityGridItemCountAt(self.handle, x, y, &out));
        return @intCast(out);
    }

    /// The cell bounds of everything added since the last clear: min x, min y,
    /// max x, max y. An empty grid reports upstream's inverted sentinel, so
    /// `min > max` is how a caller tells empty from one cell at the origin.
    pub fn bounds(self: ProximityGrid) err.Error![4]i32 {
        var out: [4]i32 = undefined;
        try err.check(c.zrcProximityGridBounds(self.handle, &out));
        return out;
    }

    pub fn cellSize(self: ProximityGrid) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zrcProximityGridCellSize(self.handle, &out));
        return out;
    }
};

//===----------------------------------------------------------------------===//
// Obstacle avoidance
//===----------------------------------------------------------------------===//

/// Picks the velocity that gets closest to what an agent wanted without
/// running into anything moving nearby.
pub const AvoidanceQuery = struct {
    handle: *c.AvoidanceQuery,

    /// Room for `max_circles` moving obstacles and `max_segments` walls, both
    /// `[1, 65535]`. A crowd asks for 6 and 8.
    pub fn init(max_circles: u32, max_segments: u32) err.Error!AvoidanceQuery {
        const circles = std.math.cast(i32, max_circles) orelse return err.Error.InvalidArgument;
        const segments = std.math.cast(i32, max_segments) orelse return err.Error.InvalidArgument;
        var handle: *c.AvoidanceQuery = undefined;
        try err.check(c.zrcAvoidanceQueryCreate(circles, segments, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: AvoidanceQuery) void {
        c.zrcAvoidanceQueryDestroy(self.handle);
    }

    /// Forgets every obstacle. The capacities stay as they were.
    pub fn reset(self: AvoidanceQuery) err.Error!void {
        try err.check(c.zrcAvoidanceQueryReset(self.handle));
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
    ) err.Error!void {
        try err.check(c.zrcAvoidanceAddCircle(
            self.handle,
            &position,
            radius,
            &velocity,
            &desired_velocity,
        ));
    }

    /// Adds a wall segment. Same capacity rule.
    pub fn addSegment(self: AvoidanceQuery, p: [3]f32, q: [3]f32) err.Error!void {
        try err.check(c.zrcAvoidanceAddSegment(self.handle, &p, &q));
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
    ) err.Error!Chosen {
        return self.sample(
            c.zrcAvoidanceSampleGrid,
            position,
            radius,
            max_speed,
            velocity,
            desired_velocity,
            params,
            debug,
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
    ) err.Error!Chosen {
        return self.sample(
            c.zrcAvoidanceSampleAdaptive,
            position,
            radius,
            max_speed,
            velocity,
            desired_velocity,
            params,
            debug,
        );
    }

    fn sample(
        self: AvoidanceQuery,
        comptime call: anytype,
        position: [3]f32,
        radius: f32,
        max_speed: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        params: AvoidanceParams,
        debug: ?AvoidanceDebug,
    ) err.Error!Chosen {
        var chosen: [3]f32 = .{ 0, 0, 0 };
        var samples: i32 = 0;
        try err.check(call(
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
        return .{ .velocity = chosen, .samples = @intCast(samples) };
    }

    pub fn circleCount(self: AvoidanceQuery) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcAvoidanceCircleCount(self.handle, &out));
        return @intCast(out);
    }

    /// One of them. The index is bounded here; upstream's accessor bounds
    /// nothing.
    pub fn circleAt(self: AvoidanceQuery, index: u32) err.Error!AvoidanceCircle {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: AvoidanceCircle = undefined;
        try err.check(c.zrcAvoidanceCircleAt(self.handle, i, &out));
        return out;
    }

    pub fn segmentCount(self: AvoidanceQuery) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcAvoidanceSegmentCount(self.handle, &out));
        return @intCast(out);
    }

    /// One of them. Same bounding.
    pub fn segmentAt(self: AvoidanceQuery, index: u32) err.Error!AvoidanceSegment {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: AvoidanceSegment = undefined;
        try err.check(c.zrcAvoidanceSegmentAt(self.handle, i, &out));
        return out;
    }
};

/// Records the candidate velocities a sampler tried, for a debug view.
pub const AvoidanceDebug = struct {
    handle: *c.AvoidanceDebug,

    /// `max_samples` is `[1, 65535]`.
    pub fn init(max_samples: u32) err.Error!AvoidanceDebug {
        const samples = std.math.cast(i32, max_samples) orelse return err.Error.InvalidArgument;
        var handle: *c.AvoidanceDebug = undefined;
        try err.check(c.zrcAvoidanceDebugCreate(samples, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: AvoidanceDebug) void {
        c.zrcAvoidanceDebugDestroy(self.handle);
    }

    /// Forgets every sample. The capacity stays as it was.
    pub fn reset(self: AvoidanceDebug) err.Error!void {
        try err.check(c.zrcAvoidanceDebugReset(self.handle));
    }

    /// Records one candidate by hand, for a host scoring velocities itself.
    /// `error.BufferTooSmall` once full.
    pub fn addSample(self: AvoidanceDebug, sample: AvoidanceSample) err.Error!void {
        try err.check(c.zrcAvoidanceDebugAddSample(self.handle, &sample));
    }

    /// Rescales every recorded penalty into `[0, 1]` against the largest.
    /// Idempotent; a recorder with no samples is a no-op.
    pub fn normalize(self: AvoidanceDebug) err.Error!void {
        try err.check(c.zrcAvoidanceDebugNormalize(self.handle));
    }

    pub fn sampleCount(self: AvoidanceDebug) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcAvoidanceDebugSampleCount(self.handle, &out));
        return @intCast(out);
    }

    /// One of them. The index is bounded here; all seven of upstream's
    /// accessors index their arrays unchecked.
    pub fn sampleAt(self: AvoidanceDebug, index: u32) err.Error!AvoidanceSample {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: AvoidanceSample = undefined;
        try err.check(c.zrcAvoidanceDebugSampleAt(self.handle, i, &out));
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
    pub fn init(max_path: u32) err.Error!PathCorridor {
        const size = std.math.cast(i32, max_path) orelse return err.Error.InvalidArgument;
        var handle: *c.PathCorridor = undefined;
        try err.check(c.zrcPathCorridorCreate(size, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: PathCorridor) void {
        c.zrcPathCorridorDestroy(self.handle);
    }

    /// Empties the corridor and puts it at `position` inside `poly`, which may
    /// be 0 for a corridor whose agent is not on the navmesh.
    pub fn reset(self: PathCorridor, poly: PolyRef, position: [3]f32) err.Error!void {
        try err.check(c.zrcPathCorridorReset(self.handle, poly, &position));
    }

    /// Replaces the corridor and its target. `path` must hold between 1 and
    /// `max_path` polygons.
    pub fn setCorridor(
        self: PathCorridor,
        target: [3]f32,
        polys: []const PolyRef,
    ) err.Error!void {
        const count = std.math.cast(i32, polys.len) orelse return err.Error.InvalidArgument;
        if (polys.len == 0) return err.Error.InvalidArgument;
        try err.check(c.zrcPathCorridorSetCorridor(self.handle, &target, polys.ptr, count));
    }

    /// String-pulls the corridor ahead into the corners to walk, returning how
    /// many were produced.
    ///
    /// **At most `out.len - 1` corners come back**, which is upstream's own
    /// behaviour rather than a rounding error: the string-pull reserves the
    /// last slot for the corridor's end point and then drops the first when
    /// the agent already stands on it. Ask for one more than needed; `out`
    /// must hold at least two.
    pub fn findCorners(
        self: PathCorridor,
        query: NavMeshQuery,
        filter: *const Filter,
        out: []CrowdCorner,
    ) err.Error!usize {
        const max_corners = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]CrowdCorner = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        try err.check(c.zrcPathCorridorFindCorners(
            self.handle,
            query.handle,
            filter,
            ptr,
            max_corners,
            &count,
        ));
        return @intCast(count);
    }

    /// Shortens the corridor if `next` is directly visible from the position.
    /// `range` must be finite and positive. Left as it was when nothing could
    /// be shortened.
    pub fn optimizeVisibility(
        self: PathCorridor,
        next: [3]f32,
        range: f32,
        query: NavMeshQuery,
        filter: *const Filter,
    ) err.Error!void {
        try err.check(c.zrcPathCorridorOptimizeVisibility(
            self.handle,
            &next,
            range,
            query.handle,
            filter,
        ));
    }

    /// Re-plans the corridor locally, following the world rather than the
    /// straight line. Reports whether anything changed.
    pub fn optimizeTopology(
        self: PathCorridor,
        query: NavMeshQuery,
        filter: *const Filter,
    ) err.Error!bool {
        var changed: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorOptimizeTopology(
            self.handle,
            query.handle,
            filter,
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
    ) err.Error!OffMeshMove {
        var out = OffMeshMove{
            .refs = .{ 0, 0 },
            .start = .{ 0, 0, 0 },
            .end = .{ 0, 0, 0 },
            .moved = false,
        };
        var moved: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorMoveOverOffmeshConnection(
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
    ) err.Error!bool {
        var fixed: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorFixStart(self.handle, safe_poly, &safe_position, &fixed));
        return fixed != c.c_false;
    }

    /// Cuts the corridor back to the last polygon still valid, falling back to
    /// `safe_poly` when none is.
    pub fn trimInvalid(
        self: PathCorridor,
        safe_poly: PolyRef,
        safe_position: [3]f32,
        query: NavMeshQuery,
        filter: *const Filter,
    ) err.Error!bool {
        var trimmed: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorTrimInvalid(
            self.handle,
            safe_poly,
            &safe_position,
            query.handle,
            filter,
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
        filter: *const Filter,
    ) err.Error!bool {
        const lookahead = std.math.cast(i32, max_lookahead) orelse
            return err.Error.InvalidArgument;
        var valid: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorIsValid(
            self.handle,
            lookahead,
            query.handle,
            filter,
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
        filter: *const Filter,
    ) err.Error!bool {
        var moved: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorMovePosition(
            self.handle,
            &position,
            query.handle,
            filter,
            &moved,
        ));
        return moved != c.c_false;
    }

    /// The same for the target end of the corridor.
    pub fn moveTargetPosition(
        self: PathCorridor,
        position: [3]f32,
        query: NavMeshQuery,
        filter: *const Filter,
    ) err.Error!bool {
        var moved: c.Bool = c.c_false;
        try err.check(c.zrcPathCorridorMoveTargetPosition(
            self.handle,
            &position,
            query.handle,
            filter,
            &moved,
        ));
        return moved != c.c_false;
    }

    /// Where the corridor is, where it ends, and how long it is.
    pub fn info(self: PathCorridor) err.Error!PathCorridorInfo {
        var out: PathCorridorInfo = undefined;
        try err.check(c.zrcPathCorridorInfo(self.handle, &out));
        return out;
    }

    /// Copies polygons of the corridor into `out`, starting at `first`, in
    /// walking order.
    pub fn path(self: PathCorridor, first: u32, out: []PolyRef) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]PolyRef = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcPathCorridorPath(self.handle, first_i, count, ptr));
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
) err.Error!usize {
    return merge(c.zrcMergeCorridorStartMoved, path, path_count, visited);
}

/// For a target that slid forwards.
pub fn mergeCorridorEndMoved(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
) err.Error!usize {
    return merge(c.zrcMergeCorridorEndMoved, path, path_count, visited);
}

/// For a visibility optimisation that skipped ahead.
pub fn mergeCorridorStartShortcut(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
) err.Error!usize {
    return merge(c.zrcMergeCorridorStartShortcut, path, path_count, visited);
}

fn merge(
    comptime call: anytype,
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
) err.Error!usize {
    const max_path = std.math.cast(i32, path.len) orelse return err.Error.InvalidArgument;
    const count = std.math.cast(i32, path_count) orelse return err.Error.InvalidArgument;
    const visited_count = std.math.cast(i32, visited.len) orelse
        return err.Error.InvalidArgument;
    var out: i32 = 0;
    try err.check(call(path.ptr, count, max_path, visited.ptr, visited_count, &out));
    return @intCast(out);
}

//===----------------------------------------------------------------------===//
// The local boundary
//===----------------------------------------------------------------------===//

/// The walls near an agent, cached so they are sampled once rather than every
/// frame.
pub const LocalBoundary = struct {
    handle: *c.LocalBoundary,

    pub fn init() err.Error!LocalBoundary {
        var handle: *c.LocalBoundary = undefined;
        try err.check(c.zrcLocalBoundaryCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: LocalBoundary) void {
        c.zrcLocalBoundaryDestroy(self.handle);
    }

    /// Forgets every segment and every polygon it was collected from.
    pub fn reset(self: LocalBoundary) err.Error!void {
        try err.check(c.zrcLocalBoundaryReset(self.handle));
    }

    /// Re-collects the walls within `range` of `position`, starting from
    /// `poly`. Keeps the eight nearest. `poly` of 0 resets it instead.
    pub fn update(
        self: LocalBoundary,
        poly: PolyRef,
        position: [3]f32,
        range: f32,
        query: NavMeshQuery,
        filter: *const Filter,
    ) err.Error!void {
        try err.check(c.zrcLocalBoundaryUpdate(
            self.handle,
            poly,
            &position,
            range,
            query.handle,
            filter,
        ));
    }

    /// Whether every polygon it was collected from still exists and still
    /// passes the filter. False means it should be updated again.
    pub fn isValid(
        self: LocalBoundary,
        query: NavMeshQuery,
        filter: *const Filter,
    ) err.Error!bool {
        var valid: c.Bool = c.c_false;
        try err.check(c.zrcLocalBoundaryIsValid(self.handle, query.handle, filter, &valid));
        return valid != c.c_false;
    }

    /// The position it was last collected around.
    ///
    /// A boundary that has never been updated reports upstream's own sentinel,
    /// `FLT_MAX` on all three axes, rather than a zero.
    pub fn center(self: LocalBoundary) err.Error![3]f32 {
        var out: [3]f32 = undefined;
        try err.check(c.zrcLocalBoundaryCenter(self.handle, &out));
        return out;
    }

    /// Segments currently held, at most eight.
    pub fn segmentCount(self: LocalBoundary) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcLocalBoundarySegmentCount(self.handle, &out));
        return @intCast(out);
    }

    /// Copies segments into `out`, starting at `first`, nearest first. Each is
    /// the two endpoints.
    pub fn segments(self: LocalBoundary, first: u32, out: [][6]f32) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcLocalBoundarySegments(self.handle, first_i, count, ptr));
    }
};

//===----------------------------------------------------------------------===//
// The path queue
//===----------------------------------------------------------------------===//

/// Long searches, run a slice at a time across frames. Eight at once, which is
/// upstream's fixed capacity.
pub const PathQueue = struct {
    handle: *c.PathQueue,

    /// `max_path_size` is the longest path one request may return and
    /// `max_search_nodes` the pool each search gets; a crowd uses 256 and
    /// 4096. Both are `[1, 65535]`. `navmesh` is borrowed and must outlive the
    /// queue.
    pub fn init(
        navmesh: NavMesh,
        max_path_size: u32,
        max_search_nodes: u32,
    ) err.Error!PathQueue {
        const path_size = std.math.cast(i32, max_path_size) orelse
            return err.Error.InvalidArgument;
        const nodes = std.math.cast(i32, max_search_nodes) orelse
            return err.Error.InvalidArgument;
        var handle: *c.PathQueue = undefined;
        try err.check(c.zrcPathQueueCreate(navmesh.handle, path_size, nodes, &handle));
        return .{ .handle = handle };
    }

    /// Destroying the queue a crowd handed back does nothing: it belongs to
    /// the crowd.
    pub fn deinit(self: PathQueue) void {
        c.zrcPathQueueDestroy(self.handle);
    }

    /// Advances the request at the front by at most `max_iters` iterations.
    ///
    /// One call advances one request, so a queue with several outstanding is
    /// drained over several calls. A request that does not finish within one
    /// call's budget holds the front and the others wait behind it, and a
    /// finished result is discarded two calls after it became ready — both
    /// upstream's behaviour.
    pub fn update(self: PathQueue, max_iters: u32) err.Error!void {
        const iters = std.math.cast(i32, max_iters) orelse return err.Error.InvalidArgument;
        try err.check(c.zrcPathQueueUpdate(self.handle, iters));
    }

    /// Submits a search. The result is `path_request_none` when every slot is
    /// taken, which is not an error: it is how a host learns to try again.
    ///
    /// `filter` is copied for the request's lifetime. Upstream keeps the
    /// caller's pointer and reads through it on every later update.
    pub fn request(
        self: PathQueue,
        start_poly: PolyRef,
        end_poly: PolyRef,
        start_position: [3]f32,
        end_position: [3]f32,
        filter: *const Filter,
    ) err.Error!PathRequestRef {
        var ref: PathRequestRef = path_request_none;
        try err.check(c.zrcPathQueueRequest(
            self.handle,
            start_poly,
            end_poly,
            &start_position,
            &end_position,
            filter,
            &ref,
        ));
        return ref;
    }

    /// Where a request has got to. An unknown reference is `.unknown` rather
    /// than an error.
    pub fn requestStatus(self: PathQueue, ref: PathRequestRef) err.Error!PathRequestState {
        var state: PathRequestState = .unknown;
        try err.check(c.zrcPathQueueRequestStatus(self.handle, ref, &state));
        return state;
    }

    /// Takes a finished request's path and frees its slot, returning its
    /// length.
    ///
    /// `error.SearchInProgress` when it has not finished and `error.NotFound`
    /// for a reference no slot holds; the request survives both. The slot is
    /// freed whichever way a successful call ends, so size `out` at the
    /// queue's `max_path_size` rather than retrying after
    /// `error.BufferTooSmall`.
    pub fn result(self: PathQueue, ref: PathRequestRef, out: []PolyRef) err.Error!usize {
        const max_path = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]PolyRef = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        try err.check(c.zrcPathQueueResult(self.handle, ref, ptr, max_path, &count));
        return @intCast(count);
    }

    /// The query object the queue runs its searches on. Borrowed; it dies with
    /// the queue.
    pub fn navMeshQuery(self: PathQueue) err.Error!NavMeshQuery {
        var handle: *const c.NavMeshQuery = undefined;
        try err.check(c.zrcPathQueueNavMeshQuery(self.handle, &handle));
        return .{ .handle = @constCast(handle) };
    }
};
