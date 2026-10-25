//! Each binding against the same Detour work written in C++
//! (bench/baseline.cpp), measured through shakedown in ReleaseFast, so the
//! Zig layer's cost is a number. Timings are evidence, never a correctness
//! gate; `--smoke` runs each row once.
const std = @import("std");
const shakedown = @import("shakedown");
const zrecast = @import("zrecast");
const baseline = @import("baseline");
const provenance = @import("preflight_bench_options");

/// The level: a square of ground this many metres a side with a box pillar
/// every `pillar_spacing` metres, baked into one tile.
const ground_side = 64;
const pillar_spacing = 8;
/// Query points, one midway between each pair of pillar rows, 8 by 8.
const point_count = 64;
/// The node pool of every query, on both sides.
const max_nodes = 2048;
/// The crowd's agents, one per point, its frame, and the frames each
/// sample steps: the first second after every agent is sent off.
const agent_radius = 0.4;
const frame = 1.0 / 60.0;
const crowd_frames = 60;

const extents = [3]f32{ 2, 4, 2 };

const Failed = error{Failed};

const Context = struct {
    gpa: std.mem.Allocator,
    image: []const u8,
    /// Set after the load rows, which run while no other handle is alive:
    /// `setAllocator` is not to be swapped under a live one.
    mesh: zrecast.NavMesh = undefined,
    query: zrecast.NavMeshQuery = undefined,
    world: *anyopaque = undefined,
    filter: zrecast.QueryFilter,
    points: [point_count][3]f32 = undefined,
    refs: [point_count]zrecast.PolyRef = undefined,
    /// Unit i of a row runs from `points[pairs[i][0]]` to `points[pairs[i][1]]`.
    pairs: [point_count][2]u32,
    crowd: zrecast.Crowd = undefined,
    crowd_c: *anyopaque = undefined,

    fn loadZig(c: *Context, n: u64) Failed!void {
        var failure: zrecast.Failure = .{};
        var tiles: u64 = 0;
        for (0..n) |_| {
            const mesh = zrecast.NavMesh.initFromBytes(.init(c.image), &failure) catch return error.Failed;
            tiles += mesh.maxTiles();
            mesh.deinit();
        }
        std.mem.doNotOptimizeAway(tiles);
    }

    fn installAllocator(c: *Context) Failed!void {
        zrecast.setAllocator(c.gpa);
    }

    fn removeAllocator(_: *Context) Failed!void {
        zrecast.resetAllocator();
    }

    fn loadValidatedC(c: *Context, n: u64) Failed!void {
        if (baseline.zrecast_baseline_load_validated(c.image.ptr, c.image.len, n) == std.math.maxInt(u64)) return error.Failed;
    }

    fn loadC(c: *Context, n: u64) Failed!void {
        if (baseline.zrecast_baseline_load(c.image.ptr, c.image.len, n) == std.math.maxInt(u64)) return error.Failed;
    }

    fn nearestZig(c: *Context, n: u64) Failed!void {
        var failure: zrecast.Failure = .{};
        var found: u64 = 0;
        for (0..n) |i| {
            const near = c.query.findNearestPoly(c.points[i % point_count], extents, c.filter, &failure) catch return error.Failed;
            found += @intFromBool(near.ref != null);
        }
        std.mem.doNotOptimizeAway(found);
    }

    fn nearestC(c: *Context, n: u64) Failed!void {
        if (baseline.zrecast_baseline_nearest(c.world, &c.points, point_count, &extents, n) == std.math.maxInt(u64)) return error.Failed;
    }

    fn routeZig(c: *Context, n: u64) Failed!void {
        var failure: zrecast.Failure = .{};
        var corridor: [256]zrecast.PolyRef = undefined;
        var corners: [64][3]f32 = undefined;
        var walked: u64 = 0;
        for (0..n) |i| {
            const pair = c.pairs[i % point_count];
            const from = c.points[pair[0]];
            const to = c.points[pair[1]];
            const path = c.query.findPath(c.refs[pair[0]], c.refs[pair[1]], from, to, c.filter, &corridor, &failure) catch return error.Failed;
            const walk = c.query.findStraightPath(from, to, corridor[0..path.len], .{}, &corners, null, null, &failure) catch return error.Failed;
            walked += walk.len;
        }
        std.mem.doNotOptimizeAway(walked);
    }

    fn routeC(c: *Context, n: u64) Failed!void {
        if (baseline.zrecast_baseline_route(c.world, &c.refs, &c.points, &c.pairs, point_count, n) == std.math.maxInt(u64)) return error.Failed;
    }

    fn raycastZig(c: *Context, n: u64) Failed!void {
        var failure: zrecast.Failure = .{};
        var sum: f32 = 0;
        for (0..n) |i| {
            const pair = c.pairs[i % point_count];
            const ray = c.query.raycast(c.refs[pair[0]], c.points[pair[0]], c.points[pair[1]], c.filter, .{}, null, null, &failure) catch return error.Failed;
            sum += ray.hit.t;
        }
        std.mem.doNotOptimizeAway(sum);
    }

    fn raycastC(c: *Context, n: u64) Failed!void {
        if (baseline.zrecast_baseline_raycast(c.world, &c.refs, &c.points, &c.pairs, point_count, n) < 0) return error.Failed;
    }

    /// A crowd ages as its agents arrive, so each batch gets a new one,
    /// every agent just sent across the level, and every batch is the same
    /// `crowd_frames` frames on both sides.
    fn openCrowd(c: *Context) Failed!void {
        var failure: zrecast.Failure = .{};
        c.crowd = zrecast.Crowd.init(c.mesh, point_count, agent_radius, &failure) catch return error.Failed;
        errdefer c.crowd.deinit();
        for (c.pairs) |pair| {
            const agent = c.crowd.addAgent(c.points[pair[0]], agentParams(), &failure) catch return error.Failed;
            c.crowd.requestMoveTarget(agent, c.refs[pair[1]], c.points[pair[1]], &failure) catch return error.Failed;
        }
    }

    fn closeCrowd(c: *Context) Failed!void {
        c.crowd.deinit();
    }

    fn crowdZig(c: *Context, n: u64) Failed!void {
        var failure: zrecast.Failure = .{};
        for (0..n) |_| c.crowd.update(frame, &failure) catch return error.Failed;
        std.mem.doNotOptimizeAway(c.crowd.activeAgentCount(&failure) catch return error.Failed);
    }

    fn openCrowdC(c: *Context) Failed!void {
        c.crowd_c = baseline.zrecast_baseline_crowd_open(c.world, &c.refs, &c.points, &c.pairs, point_count, agent_radius) orelse return error.Failed;
    }

    fn closeCrowdC(c: *Context) Failed!void {
        baseline.zrecast_baseline_crowd_close(c.crowd_c);
    }

    fn crowdC(c: *Context, n: u64) Failed!void {
        std.mem.doNotOptimizeAway(baseline.zrecast_baseline_crowd_update(c.crowd_c, frame, n));
    }
};

/// The agent parameters of both crowds, field for field.
fn agentParams() zrecast.CrowdAgentParams {
    const flags = zrecast.UpdateFlags;
    return .{
        .radius = agent_radius,
        .height = 2,
        .max_acceleration = 8,
        .max_speed = 3.5,
        .collision_query_range = agent_radius * 12,
        .path_optimization_range = agent_radius * 30,
        .separation_weight = 2,
        .update_flags = flags.anticipate_turns | flags.obstacle_avoidance | flags.separation | flags.optimize_vis | flags.optimize_topo,
        .obstacle_avoidance_type = 0,
        .query_filter_type = 0,
        .user_data = null,
    };
}

/// The level's triangles: the ground, and each pillar's four sides and top.
const Level = struct {
    verts: std.ArrayList(f32) = .empty,
    tris: std.ArrayList(i32) = .empty,

    fn vertex(level: *Level, gpa: std.mem.Allocator, x: f32, y: f32, z: f32) !i32 {
        const index: i32 = @intCast(level.verts.items.len / 3);
        try level.verts.appendSlice(gpa, &.{ x, y, z });
        return index;
    }

    /// Two triangles, wound so a quad listed (-x,-z), (-x,+z), (+x,+z),
    /// (+x,-z) faces up, as Recast's walkable test reads a normal.
    fn quad(level: *Level, gpa: std.mem.Allocator, corners: [4]i32) !void {
        try level.tris.appendSlice(gpa, &.{ corners[0], corners[1], corners[2], corners[0], corners[2], corners[3] });
    }

    fn build(gpa: std.mem.Allocator) !Level {
        var level: Level = .{};
        errdefer level.deinit(gpa);
        const side: f32 = ground_side;
        try level.quad(gpa, .{
            try level.vertex(gpa, 0, 0, 0),
            try level.vertex(gpa, 0, 0, side),
            try level.vertex(gpa, side, 0, side),
            try level.vertex(gpa, side, 0, 0),
        });
        var x: f32 = pillar_spacing;
        while (x < side) : (x += pillar_spacing) {
            var z: f32 = pillar_spacing;
            while (z < side) : (z += pillar_spacing) {
                // Corner k is +x when bit 0 is set, +z for bit 1, the top for bit 2.
                var corner: [8]i32 = undefined;
                for (&corner, 0..) |*slot, k| {
                    const dx: f32 = if (k & 1 != 0) 0.75 else -0.75;
                    const dz: f32 = if (k & 2 != 0) 0.75 else -0.75;
                    const y: f32 = if (k & 4 != 0) 3 else 0;
                    slot.* = try level.vertex(gpa, x + dx, y, z + dz);
                }
                try level.quad(gpa, .{ corner[4], corner[6], corner[7], corner[5] });
                try level.quad(gpa, .{ corner[0], corner[4], corner[5], corner[1] });
                try level.quad(gpa, .{ corner[1], corner[5], corner[7], corner[3] });
                try level.quad(gpa, .{ corner[3], corner[7], corner[6], corner[2] });
                try level.quad(gpa, .{ corner[2], corner[6], corner[4], corner[0] });
            }
        }
        return level;
    }

    fn deinit(level: *Level, gpa: std.mem.Allocator) void {
        level.verts.deinit(gpa);
        level.tris.deinit(gpa);
    }
};

pub fn main(init: std.process.Init) !void {
    var failure: zrecast.Failure = .{};
    const gpa = init.gpa;
    const args = try init.minimal.args.toSlice(init.arena.allocator());
    const options = try shakedown.bench.Options.fromArguments(args[1..]);
    const metadata: shakedown.bench.Metadata = .{ .commit = provenance.commit };
    var stdout_buffer: [4096]u8 = undefined;
    var stdout = std.Io.File.stdout().writer(init.io, &stdout_buffer);
    const Row = shakedown.bench.Row(Context, Failed);

    // The cook, untimed: one bake, and the image both sides load.
    const image = image: {
        var level = try Level.build(gpa);
        defer level.deinit(gpa);
        const poly = try zrecast.PolyMesh.bake(zrecast.defaultConfig(), .{ .verts = level.verts.items, .tris = level.tris.items }, null, null, &failure);
        defer poly.deinit();
        const baked = try zrecast.NavMesh.initFromPolyMesh(poly, null, &failure);
        defer baked.deinit();
        break :image try baked.serialize(&failure);
    };
    defer image.deinit();

    var context: Context = .{
        .gpa = gpa,
        .image = image.bytes,
        .filter = undefined,
        .pairs = undefined,
    };
    // Pair i runs to a point across the level. None is a point paired with
    // itself: that needs 36i + 11 to be a multiple of 64, and it is odd.
    for (&context.pairs, 0..) |*pair, i| pair.* = .{ @intCast(i), @intCast((i * 37 + 11) % point_count) };

    const loads = [_]Row{
        .{ .name = "load_zig", .unit = "navmesh", .run = Context.loadZig },
        .{ .name = "load_zig_allocator", .unit = "navmesh", .run = Context.loadZig, .fixture = .{ .lifetime = .row, .setup = Context.installAllocator, .teardown = Context.removeAllocator } },
        .{ .name = "load_c", .unit = "navmesh", .run = Context.loadC },
        .{ .name = "load_c_validated", .unit = "navmesh", .run = Context.loadValidatedC },
    };
    try shakedown.bench.run(Failed, gpa, init.io, &stdout.interface, &context, &loads, metadata, options);

    context.mesh = try zrecast.NavMesh.initFromBytes(.init(image.bytes), &failure);
    defer context.mesh.deinit();
    context.query = try zrecast.NavMeshQuery.init(context.mesh, max_nodes, &failure);
    defer context.query.deinit();
    context.filter = try zrecast.QueryFilter.initDefault(&failure);
    defer context.filter.deinit();
    context.world = baseline.zrecast_baseline_open(image.bytes.ptr, image.bytes.len, max_nodes) orelse return error.BaselineOpen;
    defer baseline.zrecast_baseline_close(context.world);
    for (&context.points, &context.refs, 0..) |*point, *ref, i| {
        const at = [3]f32{
            @as(f32, @floatFromInt(i % 8)) * pillar_spacing + pillar_spacing / 2,
            0,
            @as(f32, @floatFromInt(i / 8)) * pillar_spacing + pillar_spacing / 2,
        };
        const near = try context.query.findNearestPoly(at, extents, context.filter, &failure);
        ref.* = near.ref orelse return error.OffMesh;
        point.* = near.point;
    }

    const queries = [_]Row{
        .{ .name = "nearest_zig", .unit = "query", .initial = 100, .run = Context.nearestZig },
        .{ .name = "nearest_c", .unit = "query", .initial = 100, .run = Context.nearestC },
        .{ .name = "route_zig", .unit = "route", .initial = 10, .run = Context.routeZig },
        .{ .name = "route_c", .unit = "route", .initial = 10, .run = Context.routeC },
        .{ .name = "raycast_zig", .unit = "ray", .initial = 100, .run = Context.raycastZig },
        .{ .name = "raycast_c", .unit = "ray", .initial = 100, .run = Context.raycastC },
        .{ .name = "crowd_zig", .unit = "frame", .initial = crowd_frames, .grow = false, .run = Context.crowdZig, .fixture = .{ .lifetime = .batch, .setup = Context.openCrowd, .teardown = Context.closeCrowd } },
        .{ .name = "crowd_c", .unit = "frame", .initial = crowd_frames, .grow = false, .run = Context.crowdC, .fixture = .{ .lifetime = .batch, .setup = Context.openCrowdC, .teardown = Context.closeCrowdC } },
    };
    try shakedown.bench.run(Failed, gpa, init.io, &stdout.interface, &context, &queries, metadata, options);
    try stdout.interface.flush();
}
