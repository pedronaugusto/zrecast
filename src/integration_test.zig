//! End-to-end tests: bake -> navmesh -> bytes -> navmesh -> queries.
//!
//! The geometry comes from `tests/fixture.cpp`, which generates it rather than
//! shipping it, so the suite carries no third-party meshes and the assertions
//! can be exact about a shape they defined.
//!
//! Compiled only into the test binary; the fixture symbols below live in the
//! test-only `zrecast-fixture` library.

const std = @import("std");
const zrecast = @import("zrecast.zig");
const c = @import("c.zig");
const err = @import("error.zig");

//=============================================================================
// Fixture bindings (tests/fixture.h)
//=============================================================================

extern fn zrcFixtureTriMesh(out: *c.TriMesh) void;
extern fn zrcFixtureUnwalkableTriMesh(out: *c.TriMesh) void;

/// Mirrors the macros in tests/fixture.h. Asserted against the geometry the
/// fixture actually hands over, so the two cannot drift apart silently.
const fixture = struct {
    const vert_count = 24;
    const tri_count = 28;
    const ground_extent = 10.0;
    const wall_end_x = 4.0;
    const island_x = 40.0;
    const island_z = 40.0;

    const start = [3]f32{ -8, 0, -8 };
    const goal = [3]f32{ -8, 0, 8 };
    const island = [3]f32{ island_x, 0, island_z };
};

fn fixtureMesh() zrecast.TriMesh {
    var raw: c.TriMesh = undefined;
    zrcFixtureTriMesh(&raw);
    return .{
        .verts = raw.verts[0..@intCast(raw.vert_count * 3)],
        .tris = raw.tris[0..@intCast(raw.tri_count * 3)],
    };
}

fn unwalkableMesh() zrecast.TriMesh {
    var raw: c.TriMesh = undefined;
    zrcFixtureUnwalkableTriMesh(&raw);
    return .{
        .verts = raw.verts[0..@intCast(raw.vert_count * 3)],
        .tris = raw.tris[0..@intCast(raw.tri_count * 3)],
    };
}

/// The bake is not free, so most tests share one. Each caller still owns what
/// it builds; this only centralises the config.
fn bakeFixture(log: ?[]u8) !zrecast.PolyMesh {
    return zrecast.PolyMesh.bake(zrecast.defaultConfig(), fixtureMesh(), null, log);
}

const search_extents = [3]f32{ 2, 4, 2 };

//=============================================================================
// The bake
//=============================================================================

test "the fixture geometry is the shape the assertions assume" {
    const mesh = fixtureMesh();
    try std.testing.expectEqual(@as(usize, fixture.vert_count * 3), mesh.verts.len);
    try std.testing.expectEqual(@as(usize, fixture.tri_count * 3), mesh.tris.len);
    for (mesh.tris) |index| {
        try std.testing.expect(index >= 0);
        try std.testing.expect(index < fixture.vert_count);
    }
}

test "baking the fixture produces a mesh that covers the ground" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    var log_buffer: [1024]u8 = undefined;
    const poly = try bakeFixture(&log_buffer);
    defer poly.deinit();

    const baked = try poly.info();
    try std.testing.expect(baked.poly_count > 0);
    try std.testing.expect(baked.vert_count >= 3);
    // The detail mesh restores the height Recast quantised away; without it
    // Detour has no surface to project a point onto.
    try std.testing.expect(baked.detail_tri_count > 0);

    const bounds = [2][3]f32{ baked.bmin, baked.bmax };
    // The polygon mesh's bounds are the input bounds, which the island widens.
    try std.testing.expect(bounds[0][0] <= -fixture.ground_extent + 0.001);
    try std.testing.expect(bounds[1][0] >= fixture.island_x - 0.001);
    for (bounds[0], bounds[1]) |lo, hi| try std.testing.expect(lo <= hi);

    // A successful bake should not have logged an error.
    try std.testing.expect(
        std.mem.indexOf(u8, zrecast.logText(&log_buffer), "error:") == null,
    );
}

test "geometry with no walkable surface is an empty result, and says why" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    var log_buffer: [512]u8 = undefined;
    @memset(&log_buffer, 0);

    const baked = zrecast.PolyMesh.bake(
        zrecast.defaultConfig(),
        unwalkableMesh(),
        null,
        &log_buffer,
    );
    try std.testing.expectError(zrecast.Error.EmptyResult, baked);

    // The whole reason the log exists: a bare error code would not tell the
    // caller their slope threshold was the problem.
    const text = zrecast.logText(&log_buffer);
    try std.testing.expect(text.len > 0);
    try std.testing.expect(std.mem.indexOf(u8, text, "no walkable polygons") != null);
}

test "a bake log smaller than the message is truncated, not overrun" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    // A canary either side, so an overrun by even one byte is visible.
    var storage = [_]u8{0xAA} ** 32;
    const log = storage[8..16];

    const baked = zrecast.PolyMesh.bake(zrecast.defaultConfig(), unwalkableMesh(), null, log);
    try std.testing.expectError(zrecast.Error.EmptyResult, baked);

    for (storage[0..8]) |byte| try std.testing.expectEqual(@as(u8, 0xAA), byte);
    for (storage[16..]) |byte| try std.testing.expectEqual(@as(u8, 0xAA), byte);
    // Written, terminated inside its own bounds, and shorter than the slice.
    try std.testing.expect(zrecast.logText(log).len > 0);
    try std.testing.expect(zrecast.logText(log).len < log.len);
}

test "a malformed configuration is refused before anything is allocated" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const mesh = fixtureMesh();

    var zero_cell = zrecast.defaultConfig();
    zero_cell.cell_size = 0;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(zero_cell, mesh, null, null),
    );

    var nan_slope = zrecast.defaultConfig();
    nan_slope.agent_max_slope = std.math.nan(f32);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(nan_slope, mesh, null, null),
    );

    var vertical_slope = zrecast.defaultConfig();
    vertical_slope.agent_max_slope = 90;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(vertical_slope, mesh, null, null),
    );

    var too_many_verts = zrecast.defaultConfig();
    too_many_verts.verts_per_poly = c.verts_per_polygon + 1;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(too_many_verts, mesh, null, null),
    );

    // A cell so large the geometry does not fill one voxel.
    var coarse = zrecast.defaultConfig();
    coarse.cell_size = 10000;
    coarse.cell_height = 10000;
    coarse.agent_height = 40000;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(coarse, mesh, null, null),
    );
}

test "a configuration whose voxel arithmetic overflows an int is refused" {
    // Every voxel count in the bake is a caller-supplied world length divided
    // by a caller-supplied cell size. A float-to-int conversion that does not
    // fit is undefined behaviour rather than a wrap, so each of these would be
    // a sanitizer trap — and, unsanitized, an arbitrary voxel count — if the
    // conversions were not saturating. Run under `-Dsanitize_c=true` this test
    // is what proves they are.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const mesh = fixtureMesh();
    const huge: f32 = 1e30;

    // maxEdgeLen = edge_max_len / cell_size, clamped rather than rejected: an
    // edge limit longer than the grid just means "never subdivide", so the
    // bake still succeeds and produces the same mesh.
    var edge = zrecast.defaultConfig();
    edge.edge_max_len = huge;
    const edge_baked = try zrecast.PolyMesh.bake(edge, mesh, null, null);
    edge_baked.deinit();

    // minRegionArea and mergeRegionArea are squares, so they saturate sooner.
    // Every region is then smaller than the minimum and gets culled, which is
    // an empty result rather than a fault.
    var region = zrecast.defaultConfig();
    region.region_min_size = huge;
    region.region_merge_size = huge;
    try std.testing.expectError(
        zrecast.Error.EmptyResult,
        zrecast.PolyMesh.bake(region, mesh, null, null),
    );

    // The agent dimensions are rejected rather than clamped: silently baking
    // for a different agent than the caller described would be worse than an
    // error. Each bound is one of Recast's own storage limits.
    var agent = zrecast.defaultConfig();
    agent.agent_height = huge;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(agent, mesh, null, null),
    );

    var wide = zrecast.defaultConfig();
    wide.agent_radius = huge;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(wide, mesh, null, null),
    );

    var steppy = zrecast.defaultConfig();
    steppy.agent_max_climb = huge;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(steppy, mesh, null, null),
    );

    // And the grid itself: a cell so small the world does not fit in an int.
    var fine = zrecast.defaultConfig();
    fine.cell_size = 1e-30;
    fine.cell_height = 1e-30;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(fine, mesh, null, null),
    );

    // Geometry so large the grid overflows even at a sane cell size.
    const far = [_]f32{ 0, 0, 0, 1e30, 0, 0, 0, 0, 1e30 };
    const tri = [_]i32{ 0, 1, 2 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(zrecast.defaultConfig(), .{ .verts = &far, .tris = &tri }, null, null),
    );

    // The tile grid reaches rcCalcGridSize too, and reaches it first: a host
    // computes the grid before it bakes a single tile of one. The same bound
    // has to hold there or the conversion is undefined before any bake runs.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.tileGrid(tiledConfig(), .{ .verts = &far, .tris = &tri }),
    );
    var fine_tiled = tiledConfig();
    fine_tiled.cell_size = 1e-30;
    fine_tiled.cell_height = 1e-30;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.tileGrid(fine_tiled, mesh),
    );
}

test "the walkable-slope threshold is the angle it says it is" {
    // agent_max_slope becomes a cosine, and that cosine is the only
    // transcendental the bake would reach. It is computed by a polynomial here
    // rather than by the host's `cosf` so that a cook does not depend on the
    // host's libm — which is worth nothing if the polynomial is not also
    // right. A ramp at a known angle either side of the threshold is what says
    // it is: two degrees of slope decide whether this surface exists at all.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const rise: f32 = 20.0 * std.math.tan(40.0 * std.math.pi / 180.0);
    const verts = [_]f32{
        0,  0,    0,
        20, rise, 0,
        0,  0,    20,
        20, rise, 20,
    };
    const tris = [_]i32{ 0, 2, 1, 1, 2, 3 };
    const ramp = zrecast.TriMesh{ .verts = &verts, .tris = &tris };

    var gentle = zrecast.defaultConfig();
    gentle.agent_max_slope = 41;
    const walkable = try zrecast.PolyMesh.bake(gentle, ramp, null, null);
    defer walkable.deinit();
    try std.testing.expect((try walkable.info()).poly_count > 0);

    var strict = zrecast.defaultConfig();
    strict.agent_max_slope = 39;
    try std.testing.expectError(
        zrecast.Error.EmptyResult,
        zrecast.PolyMesh.bake(strict, ramp, null, null),
    );
}

test "a slice length that is not a multiple of three is refused by bake itself" {
    // bake.zig has a unit test for the conversion helper, but the helper is not
    // the entry point anyone calls. This drives the same rejection through
    // PolyMesh.bake, which is what a consumer actually reaches.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = zrecast.defaultConfig();
    const verts = [_]f32{ 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    const tris = [_]i32{ 0, 1, 2 };

    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = verts[0..8], .tris = &tris }, null, null),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = &verts, .tris = tris[0..2] }, null, null),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = &.{}, .tris = &tris }, null, null),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = &verts, .tris = &.{} }, null, null),
    );
}

test "geometry with an out-of-range index or a NaN is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = zrecast.defaultConfig();

    // Recast indexes verts[tris[i] * 3] with no bound of its own, so this is
    // the difference between an error and an out-of-bounds read.
    var verts = [_]f32{ 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    const bad_index = [_]i32{ 0, 1, 7 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = &verts, .tris = &bad_index }, null, null),
    );

    const negative_index = [_]i32{ 0, -1, 2 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = &verts, .tris = &negative_index }, null, null),
    );

    const good_index = [_]i32{ 0, 1, 2 };
    verts[4] = std.math.nan(f32);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, .{ .verts = &verts, .tris = &good_index }, null, null),
    );
}

test "every partition mode bakes the fixture" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    inline for (.{ .watershed, .monotone, .layers }) |mode| {
        var config = zrecast.defaultConfig();
        config.partition = mode;
        const poly = try zrecast.PolyMesh.bake(config, fixtureMesh(), null, null);
        defer poly.deinit();
        try std.testing.expect((try poly.info()).poly_count > 0);
    }
}

//=============================================================================
// Determinism
//=============================================================================

fn hexDigest(hash: std.crypto.hash.sha2.Sha256) [64]u8 {
    var raw: [32]u8 = undefined;
    var copy = hash;
    copy.final(&raw);
    var hex: [64]u8 = undefined;
    _ = std.fmt.bufPrint(&hex, "{x}", .{&raw}) catch unreachable;
    return hex;
}

/// Untiled: the whole fixture as one image.
fn cookUntiledDigest() ![64]u8 {
    var untiled = std.crypto.hash.sha2.Sha256.init(.{});
    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    untiled.update(image.bytes);
    return hexDigest(untiled);
}

/// Tiled: every tile of the grid, in grid order, so the digest covers the
/// per-tile path and the order it produces tiles in as well as the bytes.
fn cookTiledDigest() ![64]u8 {
    var tiled = std.crypto.hash.sha2.Sha256.init(.{});
    const config = tiledConfig();
    const geometry = fixtureMesh();
    const grid = try zrecast.tileGrid(config, geometry);
    var z: i32 = 0;
    while (z < grid.tile_count_z) : (z += 1) {
        var x: i32 = 0;
        while (x < grid.tile_count_x) : (x += 1) {
            const tile = try zrecast.PolyMesh.bakeTile(config, geometry, grid, x, z, null, null);
            if (tile == null) continue;
            defer tile.?.deinit();
            const bytes = try zrecast.buildTileData(tile.?, x, z, 0, null);
            defer bytes.deinit();
            tiled.update(bytes.bytes);
        }
    }
    return hexDigest(tiled);
}

/// Authored: all three volume shapes and a flag table, so the cook covers
/// the point-in-polygon test, the radius comparison and the box bound as
/// well as the pipeline they sit in.
fn cookAuthoredDigest() ![64]u8 {
    var authored = std.crypto.hash.sha2.Sha256.init(.{});
    var table = [_]u16{0} ** c.max_areas;
    table[zrecast.area_walkable] = zrecast.poly_flag_walkable;
    table[@intCast(marked_area)] = marked_flag;

    const diamond = [_]f32{ -7, 0, -3, -5, 0, -5, -3, 0, -3, -5, 0, -1 };
    const volumes = [_]zrecast.AreaVolume{
        try zrecast.convexVolume(marked_area, -1, 2, &diamond),
        bandVolume(marked_area),
        zrecast.cylinderVolume(zrecast.area_null, -1, 2, .{ 6, -8 }, 2),
    };
    const poly = try zrecast.PolyMesh.bake(
        zrecast.defaultConfig(),
        fixtureMesh(),
        .{ .volumes = &volumes, .area_flags = &table },
        null,
    );
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    authored.update(image.bytes);
    return hexDigest(authored);
}

// The determinism claim, scoped to what is measured. One target cooking
// twice gets the same bytes — asserted here, on every target that runs the
// suite. Targets sharing a C library also agree; `ci/determinism.sh` cooks
// on both architectures of musl and glibc and compares. Two DIFFERENT C
// libraries may disagree: upstream sorts BV items, holes and diagonals with
// qsort, and ties fall to the implementation — UPSTREAM.md has the entry.
test "a cook is deterministic" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const untiled = try cookUntiledDigest();
    const tiled = try cookTiledDigest();
    const authored = try cookAuthoredDigest();

    try std.testing.expectEqualStrings(&untiled, &(try cookUntiledDigest()));
    try std.testing.expectEqualStrings(&tiled, &(try cookTiledDigest()));
    try std.testing.expectEqualStrings(&authored, &(try cookAuthoredDigest()));

    // ci/determinism.sh sets this and collects the three lines to compare
    // across targets; without it the test stays quiet, since the build runner
    // presents any test stderr under a spurious "failed command" banner.
    if (std.testing.environ.getAlloc(gpa, "ZRECAST_COOK_DIGESTS")) |value| {
        defer gpa.free(value);
        std.debug.print(
            "\ncook digest untiled  {s}\ncook digest tiled    {s}\ncook digest authored {s}\n",
            .{ untiled, tiled, authored },
        );
    } else |_| {}
}

//=============================================================================
// Serialisation
//=============================================================================

test "a navmesh survives a round trip through bytes" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();

    const built = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer built.deinit();

    const image = try built.serialize();
    defer image.deinit();
    const bytes = image.bytes;
    try std.testing.expect(bytes.len > 0);
    try zrecast.validate(bytes);

    const loaded = try zrecast.NavMesh.initFromBytes(bytes);
    defer loaded.deinit();

    try std.testing.expectEqual(built.polyCount(), loaded.polyCount());
    try std.testing.expect(loaded.polyCount() > 0);

    const a = try built.bounds();
    const b = try loaded.bounds();
    for (a[0], b[0]) |x, y| try std.testing.expectEqual(x, y);
    for (a[1], b[1]) |x, y| try std.testing.expectEqual(x, y);

    // Re-serialising the reloaded mesh must reproduce the same bytes: anything
    // else means the loader is mutating what it was handed.
    const again = try loaded.serialize();
    defer again.deinit();
    try std.testing.expectEqualSlices(u8, bytes, again.bytes);

    // A lone tile is a 1x1 grid rather than a separate mode, so the tile entry
    // points have to find it — on a mesh that was loaded from bytes just as on
    // one that was built. A loaded mesh that reported no grid would answer
    // "outside the grid" for the only tile it holds.
    for ([_]zrecast.NavMesh{ built, loaded }) |mesh| {
        try std.testing.expectEqual(@as(u32, 1), mesh.tileCount());
        const ref = try mesh.tileRefAt(0, 0, 0);
        try std.testing.expect(ref != 0);
        const tile_bounds = try mesh.tileBounds(ref);
        const mesh_bounds = try mesh.bounds();
        for (tile_bounds[0], mesh_bounds[0]) |x, y| try std.testing.expectEqual(x, y);
        for (tile_bounds[1], mesh_bounds[1]) |x, y| try std.testing.expectEqual(x, y);
    }
}

test "an image survives a round trip through the opposite byte order" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();

    const foreign = try zrecast.swapImageEndian(image.bytes, true);
    defer foreign.deinit();

    // Really the other byte order, not a copy: the header's magic is the first
    // four bytes, and swapping it is what makes the image unreadable here.
    try std.testing.expectEqual(image.bytes.len, foreign.bytes.len);
    try std.testing.expect(!std.mem.eql(u8, image.bytes, foreign.bytes));
    try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(foreign.bytes));
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        zrecast.NavMesh.initFromBytes(foreign.bytes),
    );

    // And back again, byte for byte. A swap that lost or reordered anything
    // would show up here rather than as a navmesh that loads and misbehaves.
    const home = try zrecast.swapImageEndian(foreign.bytes, false);
    defer home.deinit();
    try std.testing.expectEqualSlices(u8, image.bytes, home.bytes);

    // The input is never touched, whichever direction was asked for.
    const again = try mesh.serialize();
    defer again.deinit();
    try std.testing.expectEqualSlices(u8, again.bytes, image.bytes);

    // Nonsense in either direction is refused rather than swapped.
    var junk = [_]u8{0xAB} ** 256;
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        zrecast.swapImageEndian(&junk, true),
    );
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        zrecast.swapImageEndian(&junk, false),
    );
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        zrecast.swapImageEndian(image.bytes[0..8], true),
    );
}

test "every truncation of a valid navmesh image is rejected" {
    // The dangerous case is not random bytes — the magic check catches those.
    // It is a VALID image cut short: the header passes, and then Detour derives
    // eight array pointers from counts that describe more data than exists.
    // dtNavMesh::init never compares those against the length it was given, so
    // without the size check here every one of these is an out-of-bounds read.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    const bytes = image.bytes;

    var len: usize = 0;
    while (len < bytes.len) : (len += 1) {
        if (zrecast.validate(bytes[0..len])) |_| {
            std.debug.print("truncated image accepted at {d} bytes\n", .{len});
            return error.TestUnexpectedResult;
        } else |_| {}
        if (zrecast.NavMesh.initFromBytes(bytes[0..len])) |loaded| {
            loaded.deinit();
            std.debug.print("truncated image loaded at {d} bytes\n", .{len});
            return error.TestUnexpectedResult;
        } else |_| {}
    }

    // And the whole image must still load, so the test cannot pass by
    // rejecting everything.
    const whole = try zrecast.NavMesh.initFromBytes(bytes);
    whole.deinit();
}

test "random image mutations never fault the validator, the loader or the queries" {
    // Most of the validator's interior checks were FOUND by fuzzing, not by
    // reading Detour — a development campaign of random mutations surfaced
    // three holes the hand-written checks had missed. This is that fuzzer,
    // kept in the suite at a size every CI leg can afford and seeded so a
    // failure reproduces. The assertions are mostly implicit: a mutant that
    // validates must also survive loading and querying, and the sanitizers
    // and the leak-checking allocator are what would object.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();

    // A point known to be on the pristine mesh, so the mutant queries start
    // from coordinates that mean something.
    const filter = zrecast.defaultFilter();
    const extents = [3]f32{ 100, 100, 100 };
    const pristine = try zrecast.NavMeshQuery.init(mesh, 256);
    defer pristine.deinit();
    const anchor = try pristine.findNearestPoly(.{ 0, 0, 0 }, extents, &filter);
    try std.testing.expect(anchor.ref != null);

    const mutant = try gpa.alloc(u8, image.bytes.len);
    defer gpa.free(mutant);

    var prng = std.Random.DefaultPrng.init(0x5eed);
    const random = prng.random();
    const rounds = 2000;
    var accepted: u32 = 0;
    for (0..rounds) |_| {
        @memcpy(mutant, image.bytes);
        for (0..random.intRangeAtMost(usize, 1, 4)) |_| {
            mutant[random.uintLessThan(usize, mutant.len)] = random.int(u8);
        }

        _ = zrecast.validate(mutant) catch continue;
        const loaded = zrecast.NavMesh.initFromBytes(mutant) catch continue;
        defer loaded.deinit();
        accepted += 1;

        const query = try zrecast.NavMeshQuery.init(loaded, 256);
        defer query.deinit();
        const near = query.findNearestPoly(anchor.point, extents, &filter) catch continue;
        const ref = near.ref orelse continue;
        var corridor: [64]zrecast.PolyRef = undefined;
        const path = query.findPath(
            ref,
            ref,
            near.point,
            anchor.point,
            &filter,
            &corridor,
        ) catch continue;
        var corners: [16][3]f32 = undefined;
        _ = query.findStraightPath(
            near.point,
            anchor.point,
            corridor[0..path.len],
            .{},
            &corners,
            null,
            null,
        ) catch {};
        _ = query.raycast(ref, near.point, anchor.point, &filter, .{}, null, null) catch {};
        _ = query.moveAlongSurface(ref, near.point, anchor.point, &filter, null) catch {};
    }

    // A fuzzer whose mutants all die at the magic check exercises nothing.
    // Measured with this seed: 783 of the 2000 mutants validate, load and
    // reach the queries. The floor is far below that on purpose — it exists
    // to notice the exercised share collapsing, not to pin the exact count.
    try std.testing.expect(accepted * 6 >= rounds);
}

test "a navmesh image with a doctored header is rejected" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    const original = image.bytes;

    // Field offsets in Detour's dtMeshHeader. Anchored by the two assertions
    // below: if the struct ever moves, magic and version stop reading as
    // themselves and this test fails before it can patch the wrong bytes.
    const off_magic = 0;
    const off_version = 4;
    const off_poly_count = 24;
    const off_vert_count = 28;
    const off_max_link_count = 32;
    const off_detail_mesh_count = 36;
    const off_off_mesh_con_count = 52;
    const off_off_mesh_base = 56;
    const off_bmin = 72;
    const off_bmax = 84;
    const off_bv_quant_factor = 96;
    const off_x = 8;
    const off_y = 12;
    const off_layer = 16;
    const dt_navmesh_magic: i32 = ('D' << 24) | ('N' << 16) | ('A' << 8) | 'V';

    try std.testing.expectEqual(
        dt_navmesh_magic,
        std.mem.readInt(i32, original[off_magic..][0..4], .little),
    );
    try std.testing.expectEqual(
        zrecast.dataVersion(),
        std.mem.readInt(i32, original[off_version..][0..4], .little),
    );

    const copy = try gpa.alloc(u8, original.len);
    defer gpa.free(copy);

    const Case = struct {
        name: []const u8,
        offset: usize,
        value: i32,
        expected: zrecast.Error,
        /// The patch leaves a well-formed tile image that only a *lone* tile
        /// may not be. `validate` answers whether the bytes are loadable at
        /// all, so it accepts these; `initFromBytes` builds a mesh with no
        /// neighbours and refuses them. Asserting both directions is what keeps
        /// the split honest instead of letting a check quietly disappear.
        lone_tile_only: bool = false,
    };
    const cases = [_]Case{
        .{ .name = "wrong magic", .offset = off_magic, .value = 0x1234, .expected = zrecast.Error.BadFormat },
        .{ .name = "future version", .offset = off_version, .value = 9999, .expected = zrecast.Error.UnsupportedVersion },
        .{ .name = "zero polygons", .offset = off_poly_count, .value = 0, .expected = zrecast.Error.BadFormat },
        .{ .name = "negative polygons", .offset = off_poly_count, .value = -1, .expected = zrecast.Error.BadFormat },
        .{ .name = "absurd polygons", .offset = off_poly_count, .value = 1 << 30, .expected = zrecast.Error.BadFormat },
        .{ .name = "too few vertices", .offset = off_vert_count, .value = 2, .expected = zrecast.Error.BadFormat },
        // maxLinkCount == 0 makes addTile write links[-1] before it ever looks
        // at the data. That write happens in every build configuration.
        .{ .name = "zero links", .offset = off_max_link_count, .value = 0, .expected = zrecast.Error.BadFormat },
        .{ .name = "detail mesh count mismatch", .offset = off_detail_mesh_count, .value = 1, .expected = zrecast.Error.BadFormat },
        // A connection claimed by a header whose offMeshBase still equals
        // polyCount: the ground and off-mesh spans no longer add up to the
        // polygon count, and the header check refuses that.
        .{ .name = "off-mesh connections claimed", .offset = off_off_mesh_con_count, .value = 1, .expected = zrecast.Error.BadFormat },
        .{ .name = "negative off-mesh count", .offset = off_off_mesh_con_count, .value = -1, .expected = zrecast.Error.BadFormat },
        .{ .name = "off-mesh base out of range", .offset = off_off_mesh_base, .value = 1 << 20, .expected = zrecast.Error.BadFormat },
        // A grid position is legitimate on a tile of a grid and impossible on a
        // lone one, so these are refused by the loader rather than by the
        // format check. A negative coordinate is neither: it would reach
        // getNeighbourTilesAt's `nx--` at INT_MIN, so both layers refuse it.
        .{ .name = "tile x set", .offset = off_x, .value = 1, .expected = zrecast.Error.BadFormat, .lone_tile_only = true },
        .{ .name = "tile y at INT_MIN", .offset = off_y, .value = -2147483648, .expected = zrecast.Error.BadFormat },
        .{ .name = "tile y negative", .offset = off_y, .value = -1, .expected = zrecast.Error.BadFormat },
        .{ .name = "tile x past the coordinate ceiling", .offset = off_x, .value = 1048577, .expected = zrecast.Error.BadFormat },
        .{ .name = "tile layer set", .offset = off_layer, .value = 3, .expected = zrecast.Error.BadFormat, .lone_tile_only = true },
        .{ .name = "tile layer past 255", .offset = off_layer, .value = 256, .expected = zrecast.Error.BadFormat },
    };

    // The float fields, which no integer patch reaches. Every one of these
    // feeds a float-to-integer conversion inside Detour on the query path.
    const FloatCase = struct {
        name: []const u8,
        offset: usize,
        value: f32,
    };
    const float_cases = [_]FloatCase{
        // bvQuantFactor scales a clamped extent into an unsigned short on every
        // single query (DetourNavMeshQuery.cpp:759).
        .{ .name = "huge quantisation factor", .offset = off_bv_quant_factor, .value = 1e30 },
        .{ .name = "zero quantisation factor", .offset = off_bv_quant_factor, .value = 0 },
        .{ .name = "negative quantisation factor", .offset = off_bv_quant_factor, .value = -1 },
        .{ .name = "NaN quantisation factor", .offset = off_bv_quant_factor, .value = std.math.nan(f32) },
        // Opposite extremes make bmax - bmin overflow to infinity, which then
        // propagates into the same conversion.
        .{ .name = "bmin at -3e38", .offset = off_bmin, .value = -3e38 },
        .{ .name = "bmax at +3e38", .offset = off_bmax, .value = 3e38 },
        .{ .name = "NaN in bmin", .offset = off_bmin, .value = std.math.nan(f32) },
        .{ .name = "infinite bmax", .offset = off_bmax, .value = std.math.inf(f32) },
        // bmin above bmax on one axis.
        .{ .name = "inverted bounds", .offset = off_bmin, .value = 1e9 },
    };

    for (cases) |case| {
        @memcpy(copy, original);
        std.mem.writeInt(i32, copy[case.offset..][0..4], case.value, .little);
        if (case.lone_tile_only) {
            zrecast.validate(copy) catch |e| {
                std.debug.print("case '{s}' is a well-formed tile and was rejected\n", .{case.name});
                return e;
            };
        } else {
            std.testing.expectError(case.expected, zrecast.validate(copy)) catch |e| {
                std.debug.print("case '{s}' was not rejected as expected\n", .{case.name});
                return e;
            };
        }
        std.testing.expectError(case.expected, zrecast.NavMesh.initFromBytes(copy)) catch |e| {
            std.debug.print("case '{s}' loaded despite validation\n", .{case.name});
            return e;
        };
    }

    for (float_cases) |case| {
        @memcpy(copy, original);
        @memcpy(copy[case.offset..][0..4], std.mem.asBytes(&case.value));
        std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy)) catch |e| {
            std.debug.print("float case '{s}' was not rejected\n", .{case.name});
            return e;
        };
        std.testing.expectError(
            zrecast.Error.BadFormat,
            zrecast.NavMesh.initFromBytes(copy),
        ) catch |e| {
            std.debug.print("float case '{s}' loaded despite validation\n", .{case.name});
            return e;
        };
    }

    // The untouched copy must still load, proving the loops above rejected the
    // patches and not the fixture.
    @memcpy(copy, original);
    const good = try zrecast.NavMesh.initFromBytes(copy);
    good.deinit();
}

test "an image whose polygons lie about their own shape is rejected" {
    // The header can be entirely self-consistent and the image still unsafe.
    // dtPoly::vertCount is one byte read straight out of the file, and four
    // Detour functions copy that many vertices into a fixed
    // float[DT_VERTS_PER_POLYGON*3] stack array — getPolyHeight,
    // closestPointOnPoly, moveAlongSurface and raycast. A value of 200 passes
    // every count check, loads, and then writes vertex coordinates from the
    // image past the end of that array.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    const original = image.bytes;

    // Layout of a tile image, mirroring dtNavMesh::addTile. Anchored below by
    // reading the polygon's real corner count back and checking it is sane, so
    // a struct change fails here rather than patching arbitrary bytes.
    const header_size = 100;
    const vert_count = std.mem.readInt(i32, original[28..][0..4], .little);
    const poly_count = std.mem.readInt(i32, original[24..][0..4], .little);
    const verts_size: usize = @intCast((12 * vert_count + 3) & ~@as(i32, 3));
    const polys_at = header_size + verts_size;
    const sizeof_poly = 32;
    const off_poly_vert_count = 30; // dtPoly::vertCount
    const off_poly_verts = 4; // dtPoly::verts[0]
    const off_poly_area_and_type = 31; // dtPoly::areaAndtype
    const off_poly_neis = 16; // dtPoly::neis[0]

    try std.testing.expect(poly_count >= 1);
    try std.testing.expect(vert_count >= 3);
    const real_corners = original[polys_at + off_poly_vert_count];
    try std.testing.expect(real_corners >= 3 and real_corners <= 6);

    const copy = try gpa.alloc(u8, original.len);
    defer gpa.free(copy);

    // Offsets of the remaining arrays, for the neighbour and detail cases.
    const links_at = polys_at + @as(usize, @intCast(poly_count)) * sizeof_poly;
    const max_link_count = std.mem.readInt(i32, original[32..][0..4], .little);
    const links_size: usize = @intCast((12 * max_link_count + 3) & ~@as(i32, 3));
    const detail_meshes_at = links_at + links_size;
    const detail_meshes_size: usize = @intCast((12 * poly_count + 3) & ~@as(i32, 3));
    const detail_verts_at = detail_meshes_at + detail_meshes_size;
    const detail_vert_count = std.mem.readInt(i32, original[40..][0..4], .little);
    const detail_verts_size: usize = @intCast((12 * detail_vert_count + 3) & ~@as(i32, 3));
    const detail_tris_at = detail_verts_at + detail_verts_size;
    const detail_tri_count = std.mem.readInt(i32, original[44..][0..4], .little);
    try std.testing.expect(detail_tri_count >= 1);
    try std.testing.expect(detail_tris_at + 4 <= original.len);

    const Byte = struct {
        name: []const u8,
        offset: usize,
        value: u8,
        /// Well formed as a tile of a grid, impossible only on a lone tile.
        lone_tile_only: bool = false,
    };
    const cases = [_]Byte{
        // The stack overrun.
        .{ .name = "vertCount 200", .offset = polys_at + off_poly_vert_count, .value = 200 },
        .{ .name = "vertCount 7", .offset = polys_at + off_poly_vert_count, .value = 7 },
        .{ .name = "vertCount 0", .offset = polys_at + off_poly_vert_count, .value = 0 },
        // A corner index past the vertex array.
        .{ .name = "corner index out of range", .offset = polys_at + off_poly_verts, .value = 0xff },
        // A ground polygon claiming to be an off-mesh connection sends
        // closestPointOnPoly down its two-vertex segment path instead.
        .{ .name = "polygon type off-mesh", .offset = polys_at + off_poly_area_and_type, .value = (1 << 6) | 63 },
        // A neighbour index past the polygon count. connectIntLinks launders it
        // into a link reference, and every traversal query then resolves that
        // reference with getTileAndPolyByRefUnsafe, which indexes tile->polys
        // and checks nothing — so this is the same stack overrun as a doctored
        // vertCount, reached the long way round. The low byte is enough: the
        // fixture has few polygons, so 0xfe is already out of range.
        .{ .name = "neighbour index out of range", .offset = polys_at + off_poly_neis, .value = 0xfe },
        // A portal edge to an adjacent tile. Legitimate on a tile of a grid, so
        // the format check accepts it and the lone-tile loader refuses it.
        .{ .name = "external link on a lone tile", .offset = polys_at + off_poly_neis + 1, .value = 0x80, .lone_tile_only = true },
    };

    for (cases) |case| {
        @memcpy(copy, original);
        copy[case.offset] = case.value;
        if (case.lone_tile_only) {
            zrecast.validate(copy) catch |e| {
                std.debug.print("interior case '{s}' is a well-formed tile and was rejected\n", .{case.name});
                return e;
            };
        } else {
            std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy)) catch |e| {
                std.debug.print("interior case '{s}' was not rejected\n", .{case.name});
                return e;
            };
        }
        std.testing.expectError(
            zrecast.Error.BadFormat,
            zrecast.NavMesh.initFromBytes(copy),
        ) catch |e| {
            std.debug.print("interior case '{s}' loaded despite validation\n", .{case.name});
            return e;
        };
    }

    // A portal whose side code is none of the four dtCreateNavMeshData emits
    // (4, 2, 0 and 6 — DetourNavMeshBuilder.cpp:534-540). connectExtLinks
    // matches only those (DetourNavMesh.cpp:312), so such an edge could never
    // link to anything and no real bake produces one. Two bytes rather than
    // one, because the flag and the side share a u16.
    @memcpy(copy, original);
    std.mem.writeInt(u16, copy[polys_at + off_poly_neis ..][0..2], 0x8000 | 3, .little);
    try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy));
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        zrecast.NavMesh.initFromBytes(copy),
    );

    // Detail edge flags with no boundary bit anywhere.
    // closestPointOnDetailEdges instantiated with onlyBoundary = true skips
    // every triangle that has none, then lerps through the null pointers it
    // never assigned. Clearing one triangle is not enough — the check is per
    // sub-mesh — so this clears the flag byte of every detail triangle.
    @memcpy(copy, original);
    for (0..@as(usize, @intCast(detail_tri_count))) |i| {
        copy[detail_tris_at + i * 4 + 3] = 0;
    }
    try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy));
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        zrecast.NavMesh.initFromBytes(copy),
    );

    // A NaN vertex coordinate. Every distance comparison against it is false,
    // so closestPointOnDetailEdges never records a nearest edge and lerps
    // through the null pointers it left unassigned.
    for ([_]f32{ std.math.nan(f32), std.math.inf(f32), 1e30 }) |poison| {
        @memcpy(copy, original);
        @memcpy(copy[header_size..][0..4], std.mem.asBytes(&poison));
        try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy));
    }
    // And one in the detail vertex array, which is a separate loop.
    if (detail_vert_count > 0) {
        @memcpy(copy, original);
        const poison = std.math.nan(f32);
        @memcpy(copy[detail_verts_at..][0..4], std.mem.asBytes(&poison));
        try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy));
    }

    // A second corner index, on the last polygon rather than the first, so the
    // pass is shown to walk the whole array and not just its head.
    @memcpy(copy, original);
    const last_poly = polys_at + sizeof_poly * @as(usize, @intCast(poly_count - 1));
    copy[last_poly + off_poly_verts] = 0xff;
    try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy));

    // Positive control: the untouched image must still load and still answer a
    // query, or every rejection above proves nothing.
    @memcpy(copy, original);
    const good = try zrecast.NavMesh.initFromBytes(copy);
    defer good.deinit();
    const query = try zrecast.NavMeshQuery.init(good, 2048);
    defer query.deinit();
    const filter = zrecast.defaultFilter();
    const near = try query.findNearestPoly(fixture.start, search_extents, &filter);
    try std.testing.expect(near.ref != null);
}

//=============================================================================
// Queries
//=============================================================================

/// Bake, build and load a navmesh plus a query, as a runtime would.
const World = struct {
    poly: zrecast.PolyMesh,
    mesh: zrecast.NavMesh,
    query: zrecast.NavMeshQuery,
    filter: zrecast.Filter,

    fn init() !World {
        const poly = try bakeFixture(null);
        errdefer poly.deinit();
        const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
        errdefer mesh.deinit();
        const query = try zrecast.NavMeshQuery.init(mesh, 2048);
        return .{ .poly = poly, .mesh = mesh, .query = query, .filter = zrecast.defaultFilter() };
    }

    fn deinit(self: World) void {
        self.query.deinit();
        self.mesh.deinit();
        self.poly.deinit();
    }
};

//=============================================================================
// Tiles
//=============================================================================

/// The tiled bake config. 32 voxels at the default 0.3 m cell size puts a tile
/// edge at 9.6 m, which divides the fixture's 20 m ground plane into several
/// tiles and leaves the island at (40, 40) in one of its own, with empty tiles
/// between. That mixture is the point: it exercises a stitched interior and the
/// empty-tile path in the same bake.
fn tiledConfig() zrecast.Config {
    var config = zrecast.defaultConfig();
    config.tile_size = 32;
    return config;
}

/// A tiled navmesh with every non-empty tile of the fixture resident.
const TiledWorld = struct {
    mesh: zrecast.NavMesh,
    query: zrecast.NavMeshQuery,
    filter: zrecast.Filter,
    grid: zrecast.TileGrid,
    baked: u32,
    empty: u32,

    fn init() !TiledWorld {
        const config = tiledConfig();
        const geometry = fixtureMesh();
        const grid = try zrecast.tileGrid(config, geometry);

        const mesh = try zrecast.NavMesh.initTiled(
            grid,
            @intCast(grid.tile_count_x * grid.tile_count_z),
            1 << 14,
        );
        errdefer mesh.deinit();

        var baked: u32 = 0;
        var empty: u32 = 0;
        var z: i32 = 0;
        while (z < grid.tile_count_z) : (z += 1) {
            var x: i32 = 0;
            while (x < grid.tile_count_x) : (x += 1) {
                const tile = try zrecast.PolyMesh.bakeTile(config, geometry, grid, x, z, null, null);
                if (tile == null) {
                    empty += 1;
                    continue;
                }
                defer tile.?.deinit();
                const bytes = try zrecast.buildTileData(tile.?, x, z, 0, null);
                defer bytes.deinit();
                _ = try mesh.addTile(bytes.bytes);
                baked += 1;
            }
        }

        const query = try zrecast.NavMeshQuery.init(mesh, 4096);
        return .{
            .mesh = mesh,
            .query = query,
            .filter = zrecast.defaultFilter(),
            .grid = grid,
            .baked = baked,
            .empty = empty,
        };
    }

    fn deinit(self: TiledWorld) void {
        self.query.deinit();
        self.mesh.deinit();
    }
};

test "the tile grid covers the geometry and divides it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = tiledConfig();
    const grid = try zrecast.tileGrid(config, fixtureMesh());

    try std.testing.expect(grid.tile_count_x > 1);
    try std.testing.expect(grid.tile_count_z > 1);
    try std.testing.expectApproxEqAbs(
        @as(f32, @floatFromInt(config.tile_size)) * config.cell_size,
        grid.tile_world_size,
        1e-4,
    );
    // The grid has to reach every corner of the input, or a tile of geometry
    // would simply never be baked.
    for (0..3) |i| try std.testing.expect(grid.origin[i] <= grid.extent_max[i]);
    const covered_x = grid.origin[0] +
        @as(f32, @floatFromInt(grid.tile_count_x)) * grid.tile_world_size;
    const covered_z = grid.origin[2] +
        @as(f32, @floatFromInt(grid.tile_count_z)) * grid.tile_world_size;
    try std.testing.expect(covered_x >= grid.extent_max[0]);
    try std.testing.expect(covered_z >= grid.extent_max[2]);

    // A single-tile config has no grid, and saying so is better than returning
    // a one-by-one one nobody asked for.
    var solo = zrecast.defaultConfig();
    solo.tile_size = 0;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.tileGrid(solo, fixtureMesh()),
    );
}

test "a tiled bake fills some tiles and leaves the rest empty, without failing" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();

    // Both halves matter. Several tiles carrying geometry proves the bake is
    // really tiled rather than one tile wearing a grid; several holding nothing
    // proves an empty tile is a fact about the world rather than an error, which
    // is the contract zrcPolyMeshBake does not share.
    try std.testing.expect(world.baked >= 2);
    try std.testing.expect(world.empty >= 1);
    try std.testing.expectEqual(world.baked, world.mesh.tileCount());
    try std.testing.expect(world.mesh.maxTiles() >= world.baked);
    try std.testing.expect(world.mesh.polyCount() > 0);

    // The tiled mesh has to sit where the geometry does. Each tile is baked
    // over a padded window and Recast removes that padding again
    // (RecastContour.cpp:837-850); getting the padding wrong shifts every tile
    // by the same amount, so the tiles still stitch to each other and only
    // their agreement with the world is lost. Comparing against the untiled
    // bake of the same geometry is what notices.
    const solo = try bakeFixture(null);
    defer solo.deinit();
    const solo_info = try solo.info();
    const solo_bounds = [2][3]f32{ solo_info.bmin, solo_info.bmax };
    const tiled_bounds = try world.mesh.bounds();
    // x and z only: the tile grid is an xz grid, and a tile's y extent comes
    // from whichever polygons landed in it.
    //
    // The minimum corner is exact, and asserting it loosely is what let a
    // one-cell shift of the tile window pass unnoticed: the grid's origin *is*
    // the untiled bake's minimum, so a tolerance wider than a cell tests
    // nothing. The maximum is a different matter — the grid covers whole tiles
    // and so reaches past the geometry — and only the direction is asserted
    // there.
    for ([_]usize{ 0, 2 }) |i| {
        try std.testing.expectApproxEqAbs(solo_bounds[0][i], tiled_bounds[0][i], 1e-3);
        try std.testing.expect(tiled_bounds[1][i] >= solo_bounds[1][i] - 0.5);
    }

    // Walking the slots finds exactly the resident tiles and nothing else, and
    // every reference it hands back resolves.
    var found: u32 = 0;
    for (0..world.mesh.maxTiles()) |slot| {
        const ref = try world.mesh.tileRefAtIndex(@intCast(slot));
        if (ref == 0) continue;
        found += 1;
        const tile_bounds = try world.mesh.tileBounds(ref);
        for (0..3) |i| try std.testing.expect(tile_bounds[0][i] <= tile_bounds[1][i]);
    }
    try std.testing.expectEqual(world.baked, found);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileRefAtIndex(world.mesh.maxTiles()),
    );

    // A tiled navmesh is not one image. Detour has no container format for a
    // set of tiles and neither does this package, so the answer is an error
    // rather than the bytes of whichever tile happened to be first.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.serialize());
}

test "a path crosses a tile boundary, and stops crossing when the tile is removed" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();

    // start and goal sit either side of the wall and, at a 9.6 m tile edge over
    // a 20 m plane, in different tiles. A path between them therefore has to
    // leave one tile and enter another, which it can only do through the portal
    // edges a tiled bake emits — so this fails if the border, the tile bounds or
    // the portal validation are wrong, not merely if tiling is missing.
    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const to = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try std.testing.expect(from.ref != null);
    try std.testing.expect(to.ref != null);

    const start_tile = try tileOf(world, fixture.start);
    const goal_tile = try tileOf(world, fixture.goal);
    try std.testing.expect(start_tile != goal_tile);

    var corridor: [256]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        from.ref.?,
        to.ref.?,
        from.point,
        to.point,
        &world.filter,
        &corridor,
    );
    try std.testing.expect(path.len > 0);
    try std.testing.expect(!path.partial);
    try std.testing.expectEqual(to.ref.?, corridor[path.len - 1]);

    // Removing the goal's tile has to break the path. If it does not, the
    // corridor never crossed the boundary and the assertion above was passing
    // for the wrong reason.
    try world.mesh.removeTile(goal_tile);
    try std.testing.expectEqual(world.baked - 1, world.mesh.tileCount());

    const after = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try std.testing.expect(after.ref == null);
}

/// The reference of the tile holding a world-space point.
fn tileOf(world: TiledWorld, point: [3]f32) !zrecast.TileRef {
    const x: i32 = @intFromFloat(
        @floor((point[0] - world.grid.origin[0]) / world.grid.tile_world_size),
    );
    const z: i32 = @intFromFloat(
        @floor((point[2] - world.grid.origin[2]) / world.grid.tile_world_size),
    );
    const ref = try world.mesh.tileRefAt(x, z, 0);
    try std.testing.expect(ref != 0);
    return ref;
}

test "a tile can be found, measured, removed, and its reference then refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();

    const ref = try tileOf(world, fixture.start);
    const bounds = try world.mesh.tileBounds(ref);
    for (0..3) |i| try std.testing.expect(bounds[0][i] <= bounds[1][i]);
    // The tile has to contain the point it was looked up by.
    try std.testing.expect(bounds[0][0] <= fixture.start[0] and fixture.start[0] <= bounds[1][0]);
    try std.testing.expect(bounds[0][2] <= fixture.start[2] and fixture.start[2] <= bounds[1][2]);

    const before = world.mesh.tileCount();
    try world.mesh.removeTile(ref);
    try std.testing.expectEqual(before - 1, world.mesh.tileCount());

    // The slot is empty, and the reference that named it is stale. A stale
    // reference must be refused rather than resolved against whatever now
    // occupies the slot — dtNavMesh::removeTile would otherwise dereference a
    // null header for a slot that was never occupied at all.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.removeTile(ref));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.tileBounds(ref));
}

test "the tile entry points refuse what they cannot place" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = tiledConfig();
    const geometry = fixtureMesh();
    const grid = try zrecast.tileGrid(config, geometry);

    // A tile that exists, so the failures below are about placement rather than
    // about the bytes.
    var tile: ?zrecast.PolyMesh = null;
    var tile_x: i32 = 0;
    var tile_z: i32 = 0;
    outer: while (tile_z < grid.tile_count_z) : (tile_z += 1) {
        tile_x = 0;
        while (tile_x < grid.tile_count_x) : (tile_x += 1) {
            tile = try zrecast.PolyMesh.bakeTile(config, geometry, grid, tile_x, tile_z, null, null);
            if (tile != null) break :outer;
        }
    }
    try std.testing.expect(tile != null);
    defer tile.?.deinit();

    const bytes = try zrecast.buildTileData(tile.?, tile_x, tile_z, 0, null);
    defer bytes.deinit();

    {
        const mesh = try zrecast.NavMesh.initTiled(grid, 1, 1 << 14);
        defer mesh.deinit();
        _ = try mesh.addTile(bytes.bytes);
        // The same grid position twice.
        try std.testing.expectError(zrecast.Error.TileOccupied, mesh.addTile(bytes.bytes));
        // A second, different position with no slot left for it.
        const elsewhere = try zrecast.buildTileData(tile.?, tile_x, tile_z + 1, 0, null);
        defer elsewhere.deinit();
        try std.testing.expectError(zrecast.Error.NavMeshFull, mesh.addTile(elsewhere.bytes));
    }

    // A tile whose grid position lies outside the navmesh's grid is refused by
    // the format check, with the navmesh's own bounds as the admission.
    {
        const outside = try zrecast.buildTileData(tile.?, grid.tile_count_x, 0, 0, null);
        defer outside.deinit();
        const mesh = try zrecast.NavMesh.initTiled(grid, 4, 1 << 14);
        defer mesh.deinit();
        try std.testing.expectError(zrecast.Error.BadFormat, mesh.addTile(outside.bytes));
        // Well formed all the same: the position is only wrong for this grid.
        try zrecast.validate(outside.bytes);
    }

    // A tile of a grid is not a lone navmesh, and a lone navmesh has no tiles
    // to serialise as a set.
    {
        const portal_tile = try zrecast.buildTileData(tile.?, tile_x, tile_z, 0, null);
        defer portal_tile.deinit();
        _ = zrecast.NavMesh.initFromBytes(portal_tile.bytes) catch |e| {
            try std.testing.expectEqual(zrecast.Error.BadFormat, e);
        };
    }

    // Out-of-range coordinates never reach Detour.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.buildTileData(tile.?, -1, 0, 0, null),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.buildTileData(tile.?, 0, 0, 256, null),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bakeTile(config, geometry, grid, grid.tile_count_x, 0, null, null),
    );
}

test "a grid that zrcTileGridCompute could not have produced is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = tiledConfig();
    const geometry = fixtureMesh();
    const good = try zrecast.tileGrid(config, geometry);

    // A grid is a value, so a host can store one in an asset and read back
    // something that is no longer one. Both halves of a tiled build check it,
    // and they fail differently if they do not: the bake turns a NaN origin
    // into an out-of-range float conversion inside Recast's rasteriser, and the
    // navmesh hands Detour a tile width it divides by.
    const doctored = [_]zrecast.TileGrid{
        blk: {
            var g = good;
            g.origin[0] = std.math.nan(f32);
            break :blk g;
        },
        blk: {
            var g = good;
            g.extent_max[1] = std.math.inf(f32);
            break :blk g;
        },
        blk: {
            var g = good;
            g.tile_world_size = 0;
            break :blk g;
        },
        blk: {
            var g = good;
            g.tile_count_x = 0;
            break :blk g;
        },
        blk: {
            var g = good;
            g.tile_count_z = -1;
            break :blk g;
        },
    };

    for (doctored) |grid| {
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.PolyMesh.bakeTile(config, geometry, grid, 0, 0, null, null),
        );
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.NavMesh.initTiled(grid, 4, 1 << 14),
        );
    }
}

/// Drives every stage by hand and hands back the polygon mesh they produce.
///
/// The parameters are the ones `buildCells` derives from `config`, which are
/// the ones `PolyMesh.bake` uses internally, so this is the same cook rather
/// than a similar one. The caller owns the result.
fn stagedCook(gpa: std.mem.Allocator, config: zrecast.Config, geometry: zrecast.TriMesh) !zrecast.PolyMesh {
    const cells = try zrecast.buildCells(config);
    const bounds = try zrecast.calcBounds(geometry);
    const grid = try zrecast.calcGridSize(bounds[0], bounds[1], config.cell_size);

    const field = try zrecast.Heightfield.init(
        null,
        grid.width,
        grid.height,
        bounds[0],
        bounds[1],
        config.cell_size,
        config.cell_height,
    );
    defer field.deinit();

    const tri_areas = try gpa.alloc(u8, geometry.tris.len / 3);
    defer gpa.free(tri_areas);
    @memset(tri_areas, zrecast.area_null);
    try zrecast.markWalkableTriangles(null, config.agent_max_slope, geometry, tri_areas);
    try field.rasterizeTriangles(null, geometry, tri_areas, cells.walkable_climb);

    if (config.filter_low_hanging_obstacles != c.c_false) {
        try field.filterLowHangingObstacles(null, cells.walkable_climb);
    }
    if (config.filter_ledge_spans != c.c_false) {
        try field.filterLedgeSpans(null, cells.walkable_height, cells.walkable_climb);
    }
    if (config.filter_walkable_low_height_spans != c.c_false) {
        try field.filterWalkableLowHeightSpans(null, cells.walkable_height);
    }

    const compact = try zrecast.CompactHeightfield.init(
        null,
        cells.walkable_height,
        cells.walkable_climb,
        field,
    );
    defer compact.deinit();
    if (cells.walkable_radius > 0) try compact.erode(null, cells.walkable_radius);

    switch (config.partition) {
        .watershed => {
            try compact.buildDistanceField(null);
            try compact.buildRegions(
                null,
                .watershed,
                cells.border_size,
                cells.min_region_area,
                cells.merge_region_area,
            );
        },
        .monotone, .layers => try compact.buildRegions(
            null,
            config.partition,
            cells.border_size,
            cells.min_region_area,
            cells.merge_region_area,
        ),
    }

    const contours = try zrecast.ContourSet.init(
        null,
        compact,
        cells.max_simplification_error,
        cells.max_edge_len,
        .{},
    );
    defer contours.deinit();

    const mesh = try zrecast.PolyMesh.initEmpty();
    errdefer mesh.deinit();
    try zrecast.polyMeshBuild(null, contours, cells.verts_per_poly, mesh);
    try zrecast.polyMeshBuildDetail(
        null,
        mesh,
        compact,
        cells.detail_sample_dist,
        cells.detail_sample_max_error,
    );

    // Recast leaves every flag zero, which no nonzero filter admits. A bake
    // assigns them from the area id; a hand-driven cook has to do the same or
    // the mesh is silently unreachable.
    const built = try mesh.info();
    const poly_count: usize = @intCast(built.poly_count);
    const areas = try gpa.alloc(u8, poly_count);
    defer gpa.free(areas);
    try mesh.polyAreas(0, areas);
    const flags = try gpa.alloc(u16, poly_count);
    defer gpa.free(flags);
    for (areas, flags) |area, *flag| {
        flag.* = if (area == zrecast.area_null) 0 else zrecast.poly_flag_walkable;
    }
    try mesh.setPolyFlags(0, flags);

    // Recast carries no agent dimensions and Detour demands three. A bake
    // quantises them to the voxel grid the mesh was actually built on rather
    // than to what was asked for.
    try mesh.setAgentDims(
        @as(f32, @floatFromInt(cells.walkable_height)) * config.cell_height,
        @as(f32, @floatFromInt(cells.walkable_radius)) * config.cell_size,
        @as(f32, @floatFromInt(cells.walkable_climb)) * config.cell_height,
    );
    return mesh;
}

test "the staged pipeline cooks the same bytes a bake does" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const geometry = fixtureMesh();

    // The strongest statement the staged pipeline can make: not that it
    // produces a usable mesh, but that it produces the same asset. Anything
    // wired in the wrong order, given the wrong parameter, or quantised
    // differently moves at least one byte of the serialised tile.
    for ([_]zrecast.Partition{ .watershed, .monotone, .layers }) |mode| {
        var config = zrecast.defaultConfig();
        config.partition = mode;

        const baked = try zrecast.PolyMesh.bake(config, geometry, null, null);
        defer baked.deinit();
        const baked_mesh = try zrecast.NavMesh.initFromPolyMesh(baked, null);
        defer baked_mesh.deinit();
        const baked_image = try baked_mesh.serialize();
        defer baked_image.deinit();

        const staged = try stagedCook(gpa, config, geometry);
        defer staged.deinit();
        const staged_mesh = try zrecast.NavMesh.initFromPolyMesh(staged, null);
        defer staged_mesh.deinit();
        const staged_image = try staged_mesh.serialize();
        defer staged_image.deinit();

        try std.testing.expectEqualSlices(u8, baked_image.bytes, staged_image.bytes);
    }
}

test "a hand-built mesh with no flags is a navmesh no query can cross" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = zrecast.defaultConfig();
    const geometry = fixtureMesh();

    // The staged cook minus its one non-Recast step. Recast allocates the flag
    // array and memsets it to zero, and dtCreateNavMeshData copies it
    // verbatim, so the tile is well formed and every polygon is invisible to
    // the default filter. That is a confusing way to get an empty path, and
    // the difference between the two meshes below is exactly the step that
    // avoids it.
    const mesh = try stagedCook(gpa, config, geometry);
    defer mesh.deinit();
    const info = try mesh.info();
    const zeros = try gpa.alloc(u16, @intCast(info.poly_count));
    defer gpa.free(zeros);
    @memset(zeros, 0);

    const flagged = try zrecast.NavMesh.initFromPolyMesh(mesh, null);
    defer flagged.deinit();
    var flagged_query = try zrecast.NavMeshQuery.init(flagged, 2048);
    defer flagged_query.deinit();
    const filter = zrecast.defaultFilter();
    const half = [3]f32{ 2, 4, 2 };
    const found = try flagged_query.findNearestPoly(fixture.start, half, &filter);
    try std.testing.expect(found.ref != 0);

    // The same geometry, with the flags Recast left alone.
    const bare = try stagedCook(gpa, config, geometry);
    defer bare.deinit();
    try bare.setPolyFlags(0, zeros);
    const unreachable_mesh = try zrecast.NavMesh.initFromPolyMesh(bare, null);
    defer unreachable_mesh.deinit();
    var bare_query = try zrecast.NavMeshQuery.init(unreachable_mesh, 2048);
    defer bare_query.deinit();
    const missing = try bare_query.findNearestPoly(fixture.start, half, &filter);
    try std.testing.expectEqual(@as(?zrecast.PolyRef, null), missing.ref);
}

test "a hand-built mesh carries the agent it was built for, or is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = zrecast.defaultConfig();
    const geometry = fixtureMesh();

    // Recast carries no agent dimensions and Detour demands three. A bake
    // fills them from its own configuration; a mesh assembled a stage at a
    // time has nowhere to get them from, and one that never received them
    // would describe an agent of size zero — a tile every agent fits through.
    const mesh = try stagedCook(gpa, config, geometry);
    defer mesh.deinit();
    const dims = try mesh.info();
    try std.testing.expect(dims.walkable_height > 0);
    try std.testing.expect(dims.walkable_climb > 0);

    const fresh = try zrecast.PolyMesh.initEmpty();
    defer fresh.deinit();
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.NavMesh.initFromPolyMesh(fresh, null),
    );
}

test "a mesh with no detail half is a navmesh Detour derives the height for" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = zrecast.defaultConfig();
    const geometry = fixtureMesh();

    // Every stage of a staged cook except the detail mesh. Upstream allows a
    // tile with no detail arrays and derives each polygon's surface from its
    // own corners; passing the empty arrays instead would describe a detail
    // mesh covering no polygon while the tile claims many.
    const cells = try zrecast.buildCells(config);
    const bounds = try zrecast.calcBounds(geometry);
    const grid = try zrecast.calcGridSize(bounds[0], bounds[1], config.cell_size);

    const field = try zrecast.Heightfield.init(
        null,
        grid.width,
        grid.height,
        bounds[0],
        bounds[1],
        config.cell_size,
        config.cell_height,
    );
    defer field.deinit();

    const tri_areas = try gpa.alloc(u8, geometry.tris.len / 3);
    defer gpa.free(tri_areas);
    @memset(tri_areas, zrecast.area_null);
    try zrecast.markWalkableTriangles(null, config.agent_max_slope, geometry, tri_areas);
    try field.rasterizeTriangles(null, geometry, tri_areas, cells.walkable_climb);
    try field.filterLowHangingObstacles(null, cells.walkable_climb);
    try field.filterLedgeSpans(null, cells.walkable_height, cells.walkable_climb);
    try field.filterWalkableLowHeightSpans(null, cells.walkable_height);

    const compact = try zrecast.CompactHeightfield.init(
        null,
        cells.walkable_height,
        cells.walkable_climb,
        field,
    );
    defer compact.deinit();
    try compact.erode(null, cells.walkable_radius);
    try compact.buildDistanceField(null);
    try compact.buildRegions(
        null,
        .watershed,
        cells.border_size,
        cells.min_region_area,
        cells.merge_region_area,
    );

    const contours = try zrecast.ContourSet.init(
        null,
        compact,
        cells.max_simplification_error,
        cells.max_edge_len,
        .{},
    );
    defer contours.deinit();

    const mesh = try zrecast.PolyMesh.initEmpty();
    defer mesh.deinit();
    try zrecast.polyMeshBuild(null, contours, cells.verts_per_poly, mesh);

    const built = try mesh.info();
    try std.testing.expect(built.poly_count > 0);
    try std.testing.expectEqual(@as(i32, 0), built.detail_mesh_count);

    const flags = try gpa.alloc(u16, @intCast(built.poly_count));
    defer gpa.free(flags);
    @memset(flags, zrecast.poly_flag_walkable);
    try mesh.setPolyFlags(0, flags);
    try mesh.setAgentDims(2, 0.6, 0.9);

    const navmesh = try zrecast.NavMesh.initFromPolyMesh(mesh, null);
    defer navmesh.deinit();
    var query = try zrecast.NavMeshQuery.init(navmesh, 2048);
    defer query.deinit();
    const filter = zrecast.defaultFilter();
    const found = try query.findNearestPoly(fixture.start, .{ 2, 4, 2 }, &filter);
    try std.testing.expect(found.ref != null);
}

/// A small heightfield with two spans in one column, for the argument
/// contracts that need a real container and nothing else.
fn scratchField() !zrecast.Heightfield {
    const bmin = [3]f32{ 0, 0, 0 };
    const bmax = [3]f32{ 4 * 0.3, 10 * 0.2, 4 * 0.3 };
    const field = try zrecast.Heightfield.init(null, 4, 4, bmin, bmax, 0.3, 0.2);
    errdefer field.deinit();
    try field.addSpan(null, 0, 0, 0, 2, zrecast.area_walkable, 1);
    try field.addSpan(null, 0, 0, 5, 8, zrecast.area_walkable, 1);
    return field;
}

test "an erosion radius Recast's threshold cannot express is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const field = try scratchField();
    defer field.deinit();
    const compact = try zrecast.CompactHeightfield.init(null, 4, 2, field);
    defer compact.deinit();

    // rcErodeWalkableArea reduces the radius to a byte before comparing it
    // against a distance field stored in bytes, so 128 wraps to eroding
    // nothing and 129 erodes as 1 would. Refused rather than quietly wrong.
    try compact.erode(null, 127);
    try std.testing.expectError(zrecast.Error.InvalidArgument, compact.erode(null, 128));
    try std.testing.expectError(zrecast.Error.InvalidArgument, compact.erode(null, -1));
}

test "a heightfield whose cell count overflows an int is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const bmin = [3]f32{ 0, 0, 0 };
    const bmax = [3]f32{ 100, 1, 100 };

    // Upstream allocates width * height span pointers with the product taken
    // in plain int, so two sizes that individually look modest overflow before
    // the allocation is attempted, and every later index into that array is
    // the same arithmetic.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.Heightfield.init(null, 32767, 32767, bmin, bmax, 0.3, 0.2),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.Heightfield.init(null, 32768, 4, bmin, bmax, 0.3, 0.2),
    );

    // The vertical extent is bounded separately, since Recast never turns it
    // into a grid dimension and a rasteriser converts a height to a cell index
    // before it clamps.
    const tall = [3]f32{ 100, 1.0e6, 100 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.Heightfield.init(null, 16, 16, bmin, tall, 0.3, 0.2),
    );
}

test "a vertex no cell index could name is refused before Recast sees it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const field = try scratchField();
    defer field.deinit();

    // Upstream skips a triangle wholly outside the field, but one that
    // straddles it reaches `(int)((triBBMax[2] - hfBBMin[2]) * inverseCellSize)`
    // with whatever the far corner holds. Finite is not enough.
    const inside = [3]f32{ 0.1, 0.1, 0.1 };
    const also_inside = [3]f32{ 0.2, 0.1, 0.1 };
    const far = [3]f32{ 1.0e17, 0.1, 1.0e17 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        field.rasterizeTriangle(null, inside, also_inside, far, zrecast.area_walkable, 1),
    );

    // The same bound on the soup form, which carries no ZrcTriMesh to route
    // through the shared geometry check.
    const soup = [_]f32{
        0.1,    0.1, 0.1,
        0.2,    0.1, 0.1,
        1.0e17, 0.1, 1.0e17,
    };
    const soup_areas = [_]u8{zrecast.area_walkable};
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        field.rasterizeTriangleSoup(null, &soup, &soup_areas, 1),
    );
}

test "a span field that does not fit its packed width is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const field = try scratchField();
    defer field.deinit();
    const compact = try zrecast.CompactHeightfield.init(null, 4, 2, field);
    defer compact.deinit();

    const info = try compact.info();
    try std.testing.expect(info.span_count > 0);

    var read: [1]zrecast.CompactSpan = undefined;
    try compact.spans(0, &read);

    // con is 24 bits upstream and h is 8. A value that does not fit is
    // truncated silently on the way into the bitfield and reads back as a
    // different span, so the whole batch is refused before any of it lands.
    var wide = read;
    wide[0].con = 1 << 24;
    try std.testing.expectError(zrecast.Error.InvalidArgument, compact.setSpans(0, &wide));
    var tall = read;
    tall[0].h = 256;
    try std.testing.expectError(zrecast.Error.InvalidArgument, compact.setSpans(0, &tall));

    // Nothing was written by either refusal.
    var again: [1]zrecast.CompactSpan = undefined;
    try compact.spans(0, &again);
    try std.testing.expectEqual(read[0].con, again[0].con);
    try std.testing.expectEqual(read[0].h, again[0].h);

    // A value that does fit round-trips.
    var ok = read;
    ok[0].con = (1 << 24) - 1;
    ok[0].h = 255;
    try compact.setSpans(0, &ok);
    try compact.spans(0, &again);
    try std.testing.expectEqual(@as(u32, (1 << 24) - 1), again[0].con);
    try std.testing.expectEqual(@as(u32, 255), again[0].h);
}

test "merging two meshes that pack their polygons differently is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = zrecast.defaultConfig();
    const geometry = fixtureMesh();
    const cells = try zrecast.buildCells(config);
    const bounds = try zrecast.calcBounds(geometry);
    const grid = try zrecast.calcGridSize(bounds[0], bounds[1], config.cell_size);

    const field = try zrecast.Heightfield.init(
        null,
        grid.width,
        grid.height,
        bounds[0],
        bounds[1],
        config.cell_size,
        config.cell_height,
    );
    defer field.deinit();
    const tri_areas = try gpa.alloc(u8, geometry.tris.len / 3);
    defer gpa.free(tri_areas);
    @memset(tri_areas, zrecast.area_null);
    try zrecast.markWalkableTriangles(null, config.agent_max_slope, geometry, tri_areas);
    try field.rasterizeTriangles(null, geometry, tri_areas, cells.walkable_climb);
    const compact = try zrecast.CompactHeightfield.init(
        null,
        cells.walkable_height,
        cells.walkable_climb,
        field,
    );
    defer compact.deinit();
    try compact.buildDistanceField(null);
    try compact.buildRegions(null, .watershed, 0, cells.min_region_area, cells.merge_region_area);
    const contours = try zrecast.ContourSet.init(
        null,
        compact,
        cells.max_simplification_error,
        cells.max_edge_len,
        .{},
    );
    defer contours.deinit();

    // Same contours, two polygon packings.
    const wide = try zrecast.PolyMesh.initEmpty();
    defer wide.deinit();
    try zrecast.polyMeshBuild(null, contours, 6, wide);
    try wide.setAgentDims(2, 0.6, 0.9);

    const narrow = try zrecast.PolyMesh.initEmpty();
    defer narrow.deinit();
    try zrecast.polyMeshBuild(null, contours, 3, narrow);
    try narrow.setAgentDims(2, 0.6, 0.9);

    try std.testing.expectEqual(@as(i32, 6), (try wide.info()).verts_per_poly);
    try std.testing.expectEqual(@as(i32, 3), (try narrow.info()).verts_per_poly);

    // Upstream takes the first mesh's stride as the stride into every input,
    // which reads past the end of any input that packs its polygons at a
    // smaller one.
    const merged = try zrecast.PolyMesh.initEmpty();
    defer merged.deinit();
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.polyMeshMerge(null, &.{ wide, narrow }, merged),
    );

    // Two that agree merge, and the result holds both.
    const twin = try zrecast.PolyMesh.initEmpty();
    defer twin.deinit();
    try zrecast.polyMeshCopy(null, wide, twin);
    const both = try zrecast.PolyMesh.initEmpty();
    defer both.deinit();
    try zrecast.polyMeshMerge(null, &.{ wide, twin }, both);
    try std.testing.expectEqual(
        (try wide.info()).poly_count * 2,
        (try both.info()).poly_count,
    );
}

/// What the build-context hooks below recorded.
const LogSpy = struct {
    var calls: u32 = 0;
    var last_category: zrecast.LogCategory = .progress;
    var last_length: i32 = 0;
    var last_first_byte: u8 = 0;
    var resets: u32 = 0;
    var starts: u32 = 0;
    var stops: u32 = 0;
    var user_seen: ?*anyopaque = null;

    fn reset() void {
        calls = 0;
        last_category = .progress;
        last_length = 0;
        last_first_byte = 0;
        resets = 0;
        starts = 0;
        stops = 0;
        user_seen = null;
    }

    fn onLog(
        user: ?*anyopaque,
        category: zrecast.LogCategory,
        message: [*:0]const u8,
        length: i32,
    ) callconv(.c) void {
        calls += 1;
        user_seen = user;
        last_category = category;
        last_length = length;
        last_first_byte = message[0];
    }

    fn onResetLog(_: ?*anyopaque) callconv(.c) void {
        resets += 1;
    }

    fn onStart(_: ?*anyopaque, _: zrecast.TimerLabel) callconv(.c) void {
        starts += 1;
    }

    fn onStop(_: ?*anyopaque, _: zrecast.TimerLabel) callconv(.c) void {
        stops += 1;
    }

    fn onAccumulated(_: ?*anyopaque, _: zrecast.TimerLabel) callconv(.c) i32 {
        return 42;
    }
};

test "a build context's hooks reach the host, and the flags gate them" {
    LogSpy.reset();
    var marker: u32 = 7;
    const context = zrecast.BuildContext{
        .user = &marker,
        .log = LogSpy.onLog,
        .reset_log = LogSpy.onResetLog,
        .start_timer = LogSpy.onStart,
        .stop_timer = LogSpy.onStop,
        .accumulated_time = LogSpy.onAccumulated,
    };

    try zrecast.log(&context, .warning, "a short message");
    try std.testing.expectEqual(@as(u32, 1), LogSpy.calls);
    try std.testing.expectEqual(zrecast.LogCategory.warning, LogSpy.last_category);
    try std.testing.expectEqual(@as(i32, "a short message".len), LogSpy.last_length);
    try std.testing.expectEqual(@as(u8, 'a'), LogSpy.last_first_byte);
    try std.testing.expectEqual(@as(?*anyopaque, &marker), LogSpy.user_seen);

    // A host's text is data, not a format string. Upstream's log() is
    // varargs and formats what it is given; "%%" would arrive as a single
    // per-cent sign if the message were passed as the format itself.
    LogSpy.reset();
    try zrecast.log(&context, .progress, "%%");
    try std.testing.expectEqual(@as(i32, 2), LogSpy.last_length);
    try std.testing.expectEqual(@as(u8, '%'), LogSpy.last_first_byte);

    LogSpy.reset();
    try zrecast.resetLog(&context);
    try std.testing.expectEqual(@as(u32, 1), LogSpy.resets);
    try zrecast.startTimer(&context, .total);
    try zrecast.stopTimer(&context, .total);
    try std.testing.expectEqual(@as(u32, 1), LogSpy.starts);
    try std.testing.expectEqual(@as(u32, 1), LogSpy.stops);
    try std.testing.expectEqual(@as(i32, 42), try zrecast.accumulatedTime(&context, .total));

    // A message of 512 bytes or more arrives as two calls: an error reading
    // "Log message was truncated", at a category the caller did not choose,
    // then the message cut to 511 bytes. A host counting entries or trusting
    // the category would otherwise be wrong.
    LogSpy.reset();
    var long: [600:0]u8 = undefined;
    @memset(long[0..600], 'x');
    long[600] = 0;
    try zrecast.log(&context, .progress, &long);
    try std.testing.expectEqual(@as(u32, 2), LogSpy.calls);
    try std.testing.expectEqual(zrecast.LogCategory.progress, LogSpy.last_category);
    try std.testing.expectEqual(@as(i32, 511), LogSpy.last_length);

    // The flags gate before the hook is reached. That is what makes them part
    // of the struct rather than a convenience a host could apply itself.
    LogSpy.reset();
    var silent = context;
    silent.log_enabled = false;
    silent.timers_enabled = false;
    try zrecast.log(&silent, .@"error", "unheard");
    try zrecast.resetLog(&silent);
    try zrecast.startTimer(&silent, .total);
    try std.testing.expectEqual(@as(u32, 0), LogSpy.calls);
    try std.testing.expectEqual(@as(u32, 0), LogSpy.resets);
    try std.testing.expectEqual(@as(u32, 0), LogSpy.starts);
    // accumulatedTime answers -1 without asking the hook at all.
    try std.testing.expectEqual(@as(i32, -1), try zrecast.accumulatedTime(&silent, .total));

    // A null context is one with both flags clear, and every entry point
    // takes it.
    try zrecast.log(null, .warning, "nowhere");
    try zrecast.resetTimers(null);
    try std.testing.expectEqual(@as(i32, -1), try zrecast.accumulatedTime(null, .total));

    // The count of labels is not a label.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.startTimer(&context, .max_timers),
    );
}

//=============================================================================
// The tile cache
//=============================================================================

/// A codec that stores its input unchanged.
///
/// No codec is bundled with the package and no container format is invented,
/// so the suite supplies the simplest one that satisfies the interface. It is
/// enough to prove the seam: every byte a cook compresses comes back out of
/// the decompressor a rebuild calls.
const StoreCodec = struct {
    var compress_calls: u32 = 0;
    var decompress_calls: u32 = 0;

    fn reset() void {
        compress_calls = 0;
        decompress_calls = 0;
    }

    fn maxSize(_: ?*anyopaque, buffer_size: i32) callconv(.c) i32 {
        return buffer_size;
    }

    fn compress(
        _: ?*anyopaque,
        buffer: [*]const u8,
        buffer_size: i32,
        out: [*]u8,
        max_out: i32,
        out_size: *i32,
    ) callconv(.c) c.Result {
        if (buffer_size > max_out) return .buffer_too_small;
        const n: usize = @intCast(buffer_size);
        @memcpy(out[0..n], buffer[0..n]);
        out_size.* = buffer_size;
        compress_calls += 1;
        return .ok;
    }

    fn decompress(
        _: ?*anyopaque,
        compressed: [*]const u8,
        compressed_size: i32,
        out: [*]u8,
        max_out: i32,
        out_size: *i32,
    ) callconv(.c) c.Result {
        if (compressed_size > max_out) return .buffer_too_small;
        const n: usize = @intCast(compressed_size);
        @memcpy(out[0..n], compressed[0..n]);
        out_size.* = compressed_size;
        decompress_calls += 1;
        return .ok;
    }

    fn compressor() zrecast.Compressor {
        return .{
            .max_compressed_size = maxSize,
            .compress = compress,
            .decompress = decompress,
        };
    }
};

/// Gives every polygon of a rebuilt tile the walkable flag.
///
/// Recast leaves the flags zero and Detour copies them verbatim, so a tile
/// cache with no callback produces tiles no nonzero filter admits. The areas
/// arrive carrying the ids the cook wrote, so this reads them to decide.
const WalkableFlags = struct {
    var calls: u32 = 0;

    fn process(_: ?*anyopaque, params: *zrecast.TileCacheBuildParams) callconv(.c) c.Result {
        calls += 1;
        const n: usize = @intCast(params.poly_count);
        const areas = params.areas[0..n];
        const flags = params.flags[0..n];
        for (areas, flags) |area, *flag| {
            flag.* = if (area == zrecast.area_null) 0 else zrecast.poly_flag_walkable;
        }
        return .ok;
    }
};

/// A tile cache cooked from the fixture, with every layer resident and every
/// tile built into a navmesh.
const CachedWorld = struct {
    cache: zrecast.TileCache,
    mesh: zrecast.NavMesh,
    query: zrecast.NavMeshQuery,
    filter: zrecast.Filter,
    layers: u32,

    /// Cooks one tile of the grid into compressed layers and adds them.
    ///
    /// The same tile bounds zrcPolyMeshBakeTile computes, taken to the
    /// compact heightfield and then cut into layers rather than into
    /// contours: a layer is what a tile cache stores and recarves.
    fn cookTile(
        gpa: std.mem.Allocator,
        cache: zrecast.TileCache,
        config: zrecast.Config,
        cells: zrecast.BuildCells,
        grid: zrecast.TileGrid,
        geometry: zrecast.TriMesh,
        tile_x: i32,
        tile_y: i32,
    ) !u32 {
        const edge = grid.tile_world_size;
        const border_world = @as(f32, @floatFromInt(cells.border_size)) * config.cell_size;
        const bmin = [3]f32{
            grid.origin[0] + @as(f32, @floatFromInt(tile_x)) * edge - border_world,
            grid.origin[1],
            grid.origin[2] + @as(f32, @floatFromInt(tile_y)) * edge - border_world,
        };
        const bmax = [3]f32{
            grid.origin[0] + @as(f32, @floatFromInt(tile_x + 1)) * edge + border_world,
            grid.extent_max[1],
            grid.origin[2] + @as(f32, @floatFromInt(tile_y + 1)) * edge + border_world,
        };
        const side = config.tile_size + cells.border_size * 2;

        const field = try zrecast.Heightfield.init(
            null,
            side,
            side,
            bmin,
            bmax,
            config.cell_size,
            config.cell_height,
        );
        defer field.deinit();

        const tri_areas = try gpa.alloc(u8, geometry.tris.len / 3);
        defer gpa.free(tri_areas);
        @memset(tri_areas, zrecast.area_null);
        try zrecast.markWalkableTriangles(null, config.agent_max_slope, geometry, tri_areas);
        try field.rasterizeTriangles(null, geometry, tri_areas, cells.walkable_climb);
        try field.filterLowHangingObstacles(null, cells.walkable_climb);
        try field.filterLedgeSpans(null, cells.walkable_height, cells.walkable_climb);
        try field.filterWalkableLowHeightSpans(null, cells.walkable_height);

        const compact = try zrecast.CompactHeightfield.init(
            null,
            cells.walkable_height,
            cells.walkable_climb,
            field,
        );
        defer compact.deinit();
        if (cells.walkable_radius > 0) try compact.erode(null, cells.walkable_radius);

        const layers = try zrecast.HeightfieldLayerSet.init(
            null,
            compact,
            cells.border_size,
            cells.walkable_height,
        );
        defer layers.deinit();

        const layer_count = try layers.count();
        var added: u32 = 0;
        var i: u32 = 0;
        while (i < layer_count) : (i += 1) {
            const layer = try layers.at(i);
            const n: usize = @intCast(layer.width * layer.height);
            const heights = try gpa.alloc(u8, n);
            defer gpa.free(heights);
            const areas = try gpa.alloc(u8, n);
            defer gpa.free(areas);
            const cons = try gpa.alloc(u8, n);
            defer gpa.free(cons);
            try layers.heights(i, 0, heights);
            try layers.areas(i, 0, areas);
            try layers.cons(i, 0, cons);

            const header = zrecast.TileCacheLayerHeader{
                .tile_x = tile_x,
                .tile_y = tile_y,
                .tile_layer = @intCast(i),
                .bmin = layer.bmin,
                .bmax = layer.bmax,
                .height_min = layer.height_min,
                .height_max = layer.height_max,
                .width = layer.width,
                .height = layer.height,
                .min_x = layer.min_x,
                .max_x = layer.max_x,
                .min_z = layer.min_z,
                .max_z = layer.max_z,
            };
            var codec = StoreCodec.compressor();
            const bytes = try zrecast.buildTileCacheLayer(&codec, header, heights, areas, cons);
            defer bytes.deinit();
            _ = try cache.addTile(bytes.bytes);
            added += 1;
        }
        return added;
    }

    fn init(gpa: std.mem.Allocator) !CachedWorld {
        const config = tiledConfig();
        const geometry = fixtureMesh();
        const grid = try zrecast.tileGrid(config, geometry);
        const cells = try zrecast.buildCells(config);

        const tiles_per_axis: u32 = @intCast(grid.tile_count_x * grid.tile_count_z);
        const cache_params = zrecast.TileCacheParams{
            .origin = grid.origin,
            .cell_size = config.cell_size,
            .cell_height = config.cell_height,
            .width = config.tile_size,
            .height = config.tile_size,
            .walkable_height = @as(f32, @floatFromInt(cells.walkable_height)) * config.cell_height,
            .walkable_radius = @as(f32, @floatFromInt(cells.walkable_radius)) * config.cell_size,
            .walkable_climb = @as(f32, @floatFromInt(cells.walkable_climb)) * config.cell_height,
            .max_simplification_error = cells.max_simplification_error,
            .max_tiles = @intCast(tiles_per_axis * 4),
            .max_obstacles = 64,
        };

        var codec = StoreCodec.compressor();
        const cache = try zrecast.TileCache.init(
            cache_params,
            &codec,
            null,
            .{ .process = WalkableFlags.process },
        );
        errdefer cache.deinit();

        var layers: u32 = 0;
        var z: i32 = 0;
        while (z < grid.tile_count_z) : (z += 1) {
            var x: i32 = 0;
            while (x < grid.tile_count_x) : (x += 1) {
                layers += try cookTile(gpa, cache, config, cells, grid, geometry, x, z);
            }
        }

        const mesh = try zrecast.NavMesh.initTiled(grid, tiles_per_axis * 4, 1 << 14);
        errdefer mesh.deinit();
        z = 0;
        while (z < grid.tile_count_z) : (z += 1) {
            var x: i32 = 0;
            while (x < grid.tile_count_x) : (x += 1) {
                try cache.buildNavMeshTilesAt(x, z, mesh);
            }
        }

        const query = try zrecast.NavMeshQuery.init(mesh, 4096);
        return .{
            .cache = cache,
            .mesh = mesh,
            .query = query,
            .filter = zrecast.defaultFilter(),
            .layers = layers,
        };
    }

    fn deinit(self: *CachedWorld) void {
        self.query.deinit();
        self.mesh.deinit();
        self.cache.deinit();
    }

    /// Drives the update loop to completion, the way a frame would spend a
    /// budget on it. Bounded so a cache that never settles fails the test
    /// rather than hanging it.
    fn settle(self: *CachedWorld) !void {
        var spins: u32 = 0;
        while (spins < 512) : (spins += 1) {
            if (try self.cache.update(self.mesh)) return;
        }
        return error.TileCacheNeverSettled;
    }

    /// Whether a complete corridor exists from the fixture's start to its
    /// goal, which is only true while the wall's gap is open.
    fn goalReachable(self: *CachedWorld) !bool {
        const half = [3]f32{ 2, 4, 2 };
        const from = try self.query.findNearestPoly(fixture.start, half, &self.filter);
        const to = try self.query.findNearestPoly(fixture.goal, half, &self.filter);
        if (from.ref == null or to.ref == null) return false;
        var corridor: [256]zrecast.PolyRef = undefined;
        const path = try self.query.findPath(
            from.ref.?,
            to.ref.?,
            from.point,
            to.point,
            &self.filter,
            &corridor,
        );
        return !path.partial;
    }
};

test "an obstacle in the wall's gap closes the way, and removing it opens it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    StoreCodec.reset();
    WalkableFlags.calls = 0;

    var world = try CachedWorld.init(gpa);
    defer world.deinit();

    // The cook and the first build both went through the host's codec, and
    // every tile went through the host's mesh-process callback. Without the
    // callback the tiles would be well formed and invisible to every filter.
    try std.testing.expect(world.layers > 0);
    try std.testing.expect(StoreCodec.compress_calls > 0);
    try std.testing.expect(StoreCodec.decompress_calls > 0);
    try std.testing.expect(WalkableFlags.calls > 0);

    // The fixture's wall runs from the -x edge to wall_end_x, so the only way
    // from start to goal is round its far end.
    try std.testing.expect(try world.goalReachable());

    const gap_min = [3]f32{ fixture.wall_end_x - 1.0, -1.0, -1.5 };
    const gap_max = [3]f32{ fixture.ground_extent + 1.0, 3.0, 1.5 };
    const plug = try world.cache.addBoxObstacle(gap_min, gap_max);

    // Queued, not applied. The tiles it overlaps are not even known yet:
    // upstream fills `touched` on the first update that processes the
    // request, so a host reading the obstacle back before then sees the
    // shape it asked for and nothing about where it lands.
    const queued = try world.cache.obstacleInfo(plug);
    try std.testing.expectEqual(zrecast.ObstacleShape.box, queued.shape);
    try std.testing.expectEqual(zrecast.ObstacleState.processing, queued.state);
    try std.testing.expectEqual(@as(i32, 0), queued.touched_count);
    try std.testing.expect(try world.goalReachable());

    try world.settle();
    const carved = try world.cache.obstacleInfo(plug);
    try std.testing.expectEqual(zrecast.ObstacleState.processed, carved.state);
    try std.testing.expect(carved.touched_count > 0);
    try std.testing.expect(!try world.goalReachable());

    // Removing it restores the mesh the cook produced.
    try world.cache.removeObstacle(plug);
    try world.settle();
    try std.testing.expect(try world.goalReachable());
}

test "a layer built from a header reads that header back, field for field" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    // `TileCacheLayer.header` is spelled apart from the `info` family because
    // it hands back the same struct `buildTileCacheLayer` takes rather than a
    // shape invented for reading. That asymmetry is only worth carrying if the
    // round trip is exact, so this asserts the whole struct at once.
    const header = zrecast.TileCacheLayerHeader{
        .tile_x = 3,
        .tile_y = 5,
        .tile_layer = 2,
        .bmin = .{ -4.5, 0.25, -8.0 },
        .bmax = .{ 4.5, 3.25, 8.0 },
        .height_min = 1,
        .height_max = 40,
        .width = 8,
        .height = 6,
        .min_x = 1,
        .max_x = 6,
        .min_z = 0,
        .max_z = 5,
    };
    const n: usize = @intCast(header.width * header.height);
    const heights = try gpa.alloc(u8, n);
    defer gpa.free(heights);
    const areas = try gpa.alloc(u8, n);
    defer gpa.free(areas);
    const cons = try gpa.alloc(u8, n);
    defer gpa.free(cons);
    for (heights, areas, cons, 0..) |*h, *a, *con, i| {
        h.* = @intCast(i % 32);
        a.* = if (i % 3 == 0) 0 else 1;
        con.* = @intCast(i % 16);
    }

    var codec = StoreCodec.compressor();
    const bytes = try zrecast.buildTileCacheLayer(&codec, header, heights, areas, cons);
    defer bytes.deinit();

    const layer = try zrecast.TileCacheLayer.initFromBytes(&codec, null, bytes.bytes);
    defer layer.deinit();
    try std.testing.expectEqual(header, try layer.header());

    // The three grids come back as they went in, which is what makes the
    // header's width and height mean anything at all.
    const back = try gpa.alloc(u8, n);
    defer gpa.free(back);
    try layer.heights(0, back);
    try std.testing.expectEqualSlices(u8, heights, back);
    try layer.areas(0, back);
    try std.testing.expectEqualSlices(u8, areas, back);
    try layer.cons(0, back);
    try std.testing.expectEqualSlices(u8, cons, back);
}

test "bytes that are not a compressed layer never reach the tile cache" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    var world = try CachedWorld.init(gpa);
    defer world.deinit();

    // Upstream casts the buffer to a layer header and reads its magic before
    // comparing the length against anything, then stores a compressed length
    // computed by subtraction — negative for a short buffer, and handed to
    // the host's codec as a plain int. Checked here before upstream sees it.
    var stub = [_]u8{0} ** 8;
    try std.testing.expectError(zrecast.Error.BadFormat, world.cache.addTile(&stub));

    // Long enough to be a header, but not one.
    var wrong = [_]u8{0} ** 128;
    try std.testing.expectError(zrecast.Error.BadFormat, world.cache.addTile(&wrong));

    // The sharp case: a buffer carrying the right magic and version but too
    // short to be a header. Upstream tests the magic before it looks at the
    // length at all, so this is what reaches the subtraction that goes
    // negative and the read that runs off the end.
    var short_but_labelled = [_]u8{0} ** 8;
    std.mem.writeInt(i32, short_but_labelled[0..4], c.tilecache_magic, .little);
    std.mem.writeInt(i32, short_but_labelled[4..8], c.tilecache_version, .little);
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        world.cache.addTile(&short_but_labelled),
    );

    // The length dtTileCacheHeaderSwapEndian takes and discards before
    // dereferencing the header.
    try std.testing.expectError(zrecast.Error.BadFormat, zrecast.swapTileCacheHeaderEndian(&stub));

    // Upstream's own index accessors bound nothing at all.
    const params = try world.cache.params();
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.cache.tileRefAt(@intCast(params.max_tiles)),
    );
    _ = try world.cache.tileRefAt(@intCast(params.max_tiles - 1));
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.cache.obstacleRefAt(@intCast(params.max_obstacles)),
    );

    // A reference naming a slot no obstacle occupies is not there, rather
    // than a struct read out of a free slot.
    const free_slot = try world.cache.obstacleRefAt(@intCast(params.max_obstacles - 1));
    try std.testing.expectEqual(@as(zrecast.ObstacleRef, 0), free_slot);

    // An emptied slot is never reported as empty through a reference:
    // upstream turns the salt over in the same statement that sets the state,
    // so the reference stops resolving first. The free slot above is the only
    // way to observe emptiness.
    const doomed = try world.cache.addBoxObstacle(
        .{ fixture.wall_end_x, -1, -1 },
        .{ fixture.wall_end_x + 1, 2, 1 },
    );
    try world.settle();
    try world.cache.removeObstacle(doomed);
    try world.settle();
    try std.testing.expectError(zrecast.Error.NotFound, world.cache.obstacleInfo(doomed));
}

test "an obstacle touching more tiles than the cache can track is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    var world = try CachedWorld.init(gpa);
    defer world.deinit();

    // Upstream tracks eight touched tiles per obstacle and carves an obstacle
    // that overlaps more into the first eight, leaving the rest untouched
    // without saying so. Refused here instead: a hole in the navmesh nothing
    // reports is worse than an error.
    const whole_world = [3]f32{ 1000, 1000, 1000 };
    const everything_min = [3]f32{ -1000, -1000, -1000 };
    var refs: [64]zrecast.CompressedTileRef = undefined;
    const overlapped = try world.cache.queryTiles(everything_min, whole_world, &refs);
    try std.testing.expect(overlapped > 8);

    // A short buffer still reports how many there were. Upstream writes only
    // what fits and returns success either way, so a caller that trusted the
    // count it got back would size its next allocation from a number that had
    // already been clipped to the buffer it was complaining about.
    var one: [1]zrecast.CompressedTileRef = undefined;
    const still_all = try world.cache.queryTiles(everything_min, whole_world, &one);
    try std.testing.expectEqual(overlapped, still_all);
    try std.testing.expectEqual(refs[0], one[0]);

    // The navmesh's own stacked lookup answers the same question on the
    // other side of the cache: a tile cache is what puts more than one layer
    // at a grid position, and this is how a host enumerates them without
    // knowing how many there are.
    var navmesh_layers: [16]zrecast.TileRef = undefined;
    const resident = try world.mesh.tileRefsAt(0, 0, &navmesh_layers);
    try std.testing.expect(resident > 0);
    var no_room: [0]zrecast.TileRef = undefined;
    try std.testing.expectEqual(resident, try world.mesh.tileRefsAt(0, 0, &no_room));

    // The same for the stacked-layer lookup, which truncates the same way.
    // An empty buffer is the sharpest form of the question: it asks only for
    // the count, and a count clipped to the buffer would be zero.
    var layers_here: [16]zrecast.CompressedTileRef = undefined;
    const stacked = try world.cache.tilesAt(0, 0, &layers_here);
    try std.testing.expect(stacked > 0);
    var none: [0]zrecast.CompressedTileRef = undefined;
    try std.testing.expectEqual(stacked, try world.cache.tilesAt(0, 0, &none));
    try std.testing.expectEqual(overlapped, try world.cache.queryTiles(everything_min, whole_world, &none));
    try std.testing.expectError(
        zrecast.Error.BufferTooSmall,
        world.cache.addBoxObstacle(everything_min, whole_world),
    );

    // One that fits is accepted, so the refusal is about the count rather
    // than about box obstacles.
    const small_min = [3]f32{ fixture.wall_end_x, -1, -1 };
    const small_max = [3]f32{ fixture.wall_end_x + 1, 2, 1 };
    _ = try world.cache.addBoxObstacle(small_min, small_max);
}

test "a tile baked from one config into another config's grid is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const geometry = fixtureMesh();

    // A grid carries the tile edge it was computed from, and a bake recomputes
    // it from the config it is handed. Nothing else compares the two, so a
    // host that computes a grid with one config and bakes with another gets
    // tiles cooked at one spacing under a navmesh addressing them at another,
    // with every call reporting success.
    const coarse = tiledConfig();
    var fine = coarse;
    fine.cell_size = coarse.cell_size / 2;

    const coarse_grid = try zrecast.tileGrid(coarse, geometry);
    const fine_grid = try zrecast.tileGrid(fine, geometry);

    // The misplacement is measured rather than assumed: these two configs
    // really do lay tiles out at different spacings.
    try std.testing.expect(coarse_grid.tile_world_size != fine_grid.tile_world_size);

    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bakeTile(fine, geometry, coarse_grid, 0, 0, null, null),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bakeTile(coarse, geometry, fine_grid, 0, 0, null, null),
    );

    // What is checked is the spacing, not the two fields separately: halving
    // the cell and doubling the tile leaves the same tile edge, and a config
    // that agrees on the edge bakes into the grid whatever its fields say.
    var same_edge = fine;
    same_edge.tile_size = coarse.tile_size * 2;
    const same_edge_grid = try zrecast.tileGrid(same_edge, geometry);
    try std.testing.expectEqual(coarse_grid.tile_world_size, same_edge_grid.tile_world_size);

    const tile = try zrecast.PolyMesh.bakeTile(same_edge, geometry, coarse_grid, 0, 0, null, null);
    if (tile) |mesh| mesh.deinit();
}

test "a reference naming a free slot is refused rather than dereferenced" {
    // The upstream hazard UPSTREAM.md records. dtNavMesh::getTileByRef checks
    // the slot index and the salt and stops there, so it returns free slots as
    // well as occupied ones — and a free slot's header is null, which
    // removeTile then reads through (DetourNavMesh.cpp:1253). Every slot of a
    // freshly created navmesh carries salt 1, so a reference that names an
    // unused slot of one passes both of removeTile's checks.
    //
    // Such a reference is built here rather than described: two tiles added in
    // succession occupy consecutive slots, so the difference between their
    // references is exactly one slot, and one more step lands on a slot that
    // was never occupied and still carries the original salt.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();

    var refs: [2]zrecast.TileRef = .{ 0, 0 };
    var slots: [2]u32 = .{ 0, 0 };
    var found: usize = 0;
    var free_slot: ?u32 = null;
    for (0..world.mesh.maxTiles()) |i| {
        const slot: u32 = @intCast(i);
        const ref = try world.mesh.tileRefAtIndex(slot);
        if (ref == 0) {
            if (free_slot == null) free_slot = slot;
            continue;
        }
        if (found < 2) {
            refs[found] = ref;
            slots[found] = slot;
            found += 1;
        }
    }
    try std.testing.expectEqual(@as(usize, 2), found);
    // Above the first resident slot, so the step below is a forward one and the
    // unsigned arithmetic cannot wrap into a reference naming something else.
    try std.testing.expect(free_slot != null and free_slot.? > slots[0]);

    const span = refs[1] - refs[0];
    const slot_step = span / (slots[1] - slots[0]);
    try std.testing.expect(slot_step > 0);
    try std.testing.expectEqual(span, slot_step * (slots[1] - slots[0]));

    // Same salt as the two resident references, an index that has never held a
    // tile: everything getTileByRef looks at, and nothing it does not.
    const forged = refs[0] +% slot_step *% (free_slot.? -% slots[0]);
    try std.testing.expect(forged != refs[0] and forged != refs[1]);

    const before = world.mesh.tileCount();
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.removeTile(forged));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.tileBounds(forged));
    // If the forged reference had happened to name an occupied slot, the
    // removal above would have succeeded and this is what would say so.
    try std.testing.expectEqual(before, world.mesh.tileCount());
}

test "a tile count and polygon count that leave Detour no salt are refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const grid = try zrecast.tileGrid(tiledConfig(), fixtureMesh());

    // Detour splits 32 reference bits between a tile index, a polygon index and
    // a salt, and refuses fewer than 10 bits of salt (DetourNavMesh.cpp:255-261).
    // Rounded up to powers of two the first two may claim 22 bits together;
    // 2^12 tiles of 2^12 polygons claims 24.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.NavMesh.initTiled(grid, 1 << 12, 1 << 12),
    );

    // Refused at the door, not by Detour. dtNavMesh::init reaches the same
    // verdict, but only after allocating a navmesh and half-initialising it,
    // and unwinding that is the delicate path UPSTREAM.md describes. Counting
    // allocations is what tells the two apart: the check here has to happen
    // before anything is taken.
    {
        var counting = std.testing.FailingAllocator.init(gpa, .{});
        try zrecast.setAllocator(counting.allocator());
        defer zrecast.setAllocator(gpa) catch {};
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.NavMesh.initTiled(grid, 1 << 12, 1 << 12),
        );
        try std.testing.expectEqual(@as(usize, 0), counting.alloc_index);
    }
    // One bit less on each side fits exactly.
    {
        const mesh = try zrecast.NavMesh.initTiled(grid, 1 << 11, 1 << 11);
        defer mesh.deinit();
        try std.testing.expectEqual(@as(u32, 0), mesh.tileCount());
        try std.testing.expectEqual(@as(u32, 1 << 11), mesh.maxTiles());
        // 0 polygons is what a navmesh with no tile resident answers, so it
        // cannot double as the failure a NULL handle reports. The C boundary
        // gives that -1 and tests/c_smoke.c reads it back.
        try std.testing.expectEqual(@as(u32, 0), mesh.polyCount());
    }
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.NavMesh.initTiled(grid, 0, 1 << 11),
    );
}

test "findNearestPoly locates the ground and reports nothing off the mesh" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);
    // The projected point must be on the floor, near the requested point.
    try std.testing.expectApproxEqAbs(fixture.start[0], near.point[0], 1.0);
    try std.testing.expectApproxEqAbs(fixture.start[2], near.point[2], 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0), near.point[1], 0.5);

    // Far outside the mesh, with a search box too small to reach it.
    const nowhere = try world.query.findNearestPoly(
        .{ 500, 500, 500 },
        search_extents,
        &world.filter,
    );
    try std.testing.expect(nowhere.ref == null);
    // Not finding anything is not an error, and the point is left as asked.
    try std.testing.expectEqual(@as(f32, 500), nowhere.point[0]);
}

test "a filter that admits nothing finds nothing" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    var blind = zrecast.defaultFilter();
    blind.include_flags = 0;
    const near = try world.query.findNearestPoly(fixture.start, search_extents, &blind);
    try std.testing.expect(near.ref == null);

    // And excluding exactly the flag the bake assigns has the same effect,
    // which is the check that the bake really did assign it.
    var excluded = zrecast.defaultFilter();
    excluded.exclude_flags = zrecast.poly_flag_walkable;
    const also_none = try world.query.findNearestPoly(fixture.start, search_extents, &excluded);
    try std.testing.expect(also_none.ref == null);
}

test "a path across the wall has to round its end" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const to = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try std.testing.expect(from.ref != null);
    try std.testing.expect(to.ref != null);

    var corridor: [256]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        from.ref.?,
        to.ref.?,
        from.point,
        to.point,
        &world.filter,
        &corridor,
    );
    try std.testing.expect(path.len > 0);
    try std.testing.expect(!path.partial);
    try std.testing.expectEqual(from.ref.?, corridor[0]);
    try std.testing.expectEqual(to.ref.?, corridor[path.len - 1]);
    // No reference in a corridor may be zero.
    for (corridor[0..path.len]) |ref| try std.testing.expect(ref != 0);

    var corners: [64][3]f32 = undefined;
    var flags: [64]u8 = undefined;
    var refs: [64]zrecast.PolyRef = undefined;
    const straight = try world.query.findStraightPath(
        from.point,
        to.point,
        corridor[0..path.len],
        .{},
        &corners,
        &flags,
        &refs,
    );

    try std.testing.expect(straight.len >= 2);
    try std.testing.expect(!straight.partial);
    try std.testing.expect(flags[0] & zrecast.straightpath_start != 0);
    try std.testing.expect(flags[straight.len - 1] & zrecast.straightpath_end != 0);

    // The wall makes a straight line impossible: there must be at least one
    // corner between the ends, and it must be past the wall's tip.
    try std.testing.expect(straight.len >= 3);
    var rounded_the_wall = false;
    for (corners[1 .. straight.len - 1]) |corner| {
        if (corner[0] > fixture.wall_end_x - 0.5) rounded_the_wall = true;
    }
    try std.testing.expect(rounded_the_wall);

    // Consecutive corners must actually differ, and every one must be finite
    // and inside the world.
    for (corners[0..straight.len]) |corner| {
        for (corner) |v| try std.testing.expect(std.math.isFinite(v));
        try std.testing.expect(@abs(corner[0]) <= fixture.ground_extent + 1);
        try std.testing.expect(@abs(corner[2]) <= fixture.ground_extent + 1);
    }
}

test "an unreachable goal yields a partial path rather than a lie" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const island = try world.query.findNearestPoly(fixture.island, search_extents, &world.filter);
    try std.testing.expect(from.ref != null);
    // The island must have baked, or this test proves nothing.
    try std.testing.expect(island.ref != null);
    try std.testing.expect(from.ref.? != island.ref.?);

    var corridor: [256]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        from.ref.?,
        island.ref.?,
        from.point,
        island.point,
        &world.filter,
        &corridor,
    );
    try std.testing.expect(path.partial);
    try std.testing.expect(path.len > 0);
    // A partial corridor still starts where asked and never reaches the goal.
    try std.testing.expectEqual(from.ref.?, corridor[0]);
    try std.testing.expect(corridor[path.len - 1] != island.ref.?);
}

test "a corridor buffer that is too small reports partial, not overflow" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const to = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);

    var single: [1]zrecast.PolyRef = .{0};
    const path = try world.query.findPath(
        from.ref.?,
        to.ref.?,
        from.point,
        to.point,
        &world.filter,
        &single,
    );
    try std.testing.expectEqual(@as(usize, 1), path.len);
    try std.testing.expect(path.partial);
    try std.testing.expectEqual(from.ref.?, single[0]);
}

test "moveAlongSurface slides along the wall instead of through it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(from.ref != null);

    var visited: [32]zrecast.PolyRef = undefined;
    const moved = try world.query.moveAlongSurface(
        from.ref.?,
        from.point,
        fixture.goal,
        &world.filter,
        &visited,
    );

    for (moved.position) |v| try std.testing.expect(std.math.isFinite(v));
    try std.testing.expect(moved.visited_len > 0);
    try std.testing.expect(moved.visited_len <= visited.len);
    // It must have moved towards the goal but been stopped by the wall well
    // short of it: the wall sits at z = 0 and the goal is at z = +8.
    try std.testing.expect(moved.position[2] > from.point[2]);
    try std.testing.expect(moved.position[2] < 0.5);

    // A visited buffer too small to hold the crossing must say so rather than
    // look like a complete list: Detour reports the clip as a plain success.
    var single: [1]zrecast.PolyRef = undefined;
    const clipped = try world.query.moveAlongSurface(
        from.ref.?,
        from.point,
        fixture.goal,
        &world.filter,
        &single,
    );
    try std.testing.expect(clipped.truncated);
    try std.testing.expectEqual(@as(usize, 1), clipped.visited_len);
    // The position reached is unaffected by the buffer being short.
    for (moved.position, clipped.position) |a, b| {
        try std.testing.expectApproxEqAbs(a, b, 1e-5);
    }
    try std.testing.expect(!moved.truncated);

    // The same call without a visited buffer must behave identically.
    const again = try world.query.moveAlongSurface(
        from.ref.?,
        from.point,
        fixture.goal,
        &world.filter,
        null,
    );
    try std.testing.expectEqual(@as(usize, 0), again.visited_len);
    // No buffer means no list to report as cut short, even though the move
    // itself crosses more polygons than the internal discard buffer holds.
    try std.testing.expect(!again.truncated);
    for (moved.position, again.position) |a, b| {
        try std.testing.expectApproxEqAbs(a, b, 1e-5);
    }
}

test "a raycast stops at the wall and passes through open ground" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(from.ref != null);

    var crossed: [32]zrecast.PolyRef = undefined;
    const blocked_result = try world.query.raycast(
        from.ref.?,
        from.point,
        fixture.goal,
        &world.filter,
        .{},
        null,
        &crossed,
    );
    const blocked = blocked_result.hit;
    // The ray crossed at least the polygon it started in, and every reported
    // reference is real.
    try std.testing.expect(blocked_result.path_len > 0);
    try std.testing.expect(blocked_result.path_len <= crossed.len);
    try std.testing.expectEqual(from.ref.?, crossed[0]);
    try std.testing.expect(!blocked_result.truncated);

    // And a path buffer too short to hold the crossing reports the clip.
    var one: [1]zrecast.PolyRef = undefined;
    const clipped = try world.query.raycast(
        from.ref.?,
        from.point,
        fixture.goal,
        &world.filter,
        .{},
        null,
        &one,
    );
    try std.testing.expect(clipped.truncated);
    try std.testing.expectEqual(@as(usize, 1), clipped.path_len);
    // The hit itself is unaffected by the buffer being short.
    try std.testing.expectApproxEqAbs(blocked.t, clipped.hit.t, 1e-6);
    try std.testing.expect(blocked.hit);
    try std.testing.expect(blocked.t > 0);
    try std.testing.expect(blocked.t < 1);
    // The normal of a wall on the xz plane is horizontal and unit length.
    const n = blocked.normal;
    const length = @sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    try std.testing.expectApproxEqAbs(@as(f32, 1), length, 1e-3);
    // The reported position must be the lerp its own `t` describes.
    for (blocked.position, 0..) |p, i| {
        const expected = from.point[i] + (fixture.goal[i] - from.point[i]) * blocked.t;
        try std.testing.expectApproxEqAbs(expected, p, 1e-4);
    }

    // A short ray entirely inside open ground hits nothing, and then `t` must
    // be exactly 1 rather than Detour's FLT_MAX.
    var short_target = from.point;
    short_target[0] += 1.0;
    const clear_result = try world.query.raycast(
        from.ref.?,
        from.point,
        short_target,
        &world.filter,
        .{},
        null,
        null,
    );
    const clear = clear_result.hit;
    try std.testing.expectEqual(@as(usize, 0), clear_result.path_len);
    try std.testing.expect(!clear.hit);
    try std.testing.expectEqual(@as(f32, 1.0), clear.t);
    for (clear.normal) |v| try std.testing.expectEqual(@as(f32, 0), v);
    for (clear.position, short_target) |p, expected| {
        try std.testing.expectApproxEqAbs(expected, p, 1e-4);
    }
}

test "queries reject the arguments that would fault inside Detour" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const to = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);

    const nan = std.math.nan(f32);
    var corridor: [16]zrecast.PolyRef = undefined;

    // A NaN position would poison every distance comparison in the search.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.query.findPath(
        from.ref.?,
        to.ref.?,
        .{ nan, 0, 0 },
        to.point,
        &world.filter,
        &corridor,
    ));

    // A zero reference is "no polygon" and cannot start a search.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.query.findPath(
        0,
        to.ref.?,
        from.point,
        to.point,
        &world.filter,
        &corridor,
    ));

    // An empty output buffer.
    var none: [0]zrecast.PolyRef = .{};
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.query.findPath(
        from.ref.?,
        to.ref.?,
        from.point,
        to.point,
        &world.filter,
        &none,
    ));

    // A negative area cost makes A* incoherent.
    var negative = zrecast.defaultFilter();
    negative.area_cost[zrecast.area_walkable] = -1;
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.query.findPath(
        from.ref.?,
        to.ref.?,
        from.point,
        to.point,
        &negative,
        &corridor,
    ));

    // A companion array shorter than the point array: Detour writes all three
    // in lockstep and bounds-checks only the first.
    var corners: [8][3]f32 = undefined;
    var short_flags: [2]u8 = undefined;
    try std.testing.expectError(zrecast.Error.BufferTooSmall, world.query.findStraightPath(
        from.point,
        to.point,
        &.{from.ref.?},
        .{},
        &corners,
        &short_flags,
        null,
    ));

    // A corridor containing a zero reference.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.query.findStraightPath(
        from.point,
        to.point,
        &.{ from.ref.?, 0 },
        .{},
        &corners,
        null,
        null,
    ));

    // A node pool of zero, and one past what Detour can index.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.NavMeshQuery.init(world.mesh, 0),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.NavMeshQuery.init(world.mesh, 65536),
    );

    // And 1 through 3, which Detour accepts and then corrupts the heap over:
    // its node pool sizes its hash table as dtNextPow2(max_nodes / 4), which is
    // zero below four, after which every bucket index is out of bounds. The
    // smallest accepted value must still produce a query that works.
    for ([_]u32{ 1, 2, 3 }) |too_few| {
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.NavMeshQuery.init(world.mesh, too_few),
        );
    }
    const smallest = try zrecast.NavMeshQuery.init(world.mesh, 4);
    defer smallest.deinit();
    const found = try smallest.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(found.ref != null);
}

test "a stale polygon reference from another navmesh is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const from = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(from.ref != null);

    // A reference with a plausible index but a salt that belongs to nothing.
    const bogus: zrecast.PolyRef = from.ref.? ^ 0x8000_0000;
    var corridor: [16]zrecast.PolyRef = undefined;
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.query.findPath(
        bogus,
        from.ref.?,
        from.point,
        from.point,
        &world.filter,
        &corridor,
    ));
}

//=============================================================================
// Area authoring
//=============================================================================

/// An area id for the tests below, clear of both `area_walkable` and
/// `area_null` so a polygon carrying it cannot be either by accident.
const marked_area: i32 = 7;

/// A flag bit no bake sets by default, so a filter keyed on it is testing the
/// authored table rather than the default one.
const marked_flag: u16 = 0x0002;

/// A band across the open half of the fixture, south of the wall.
///
/// It spans the straight line from (-8, -8) to (8, -8) and stops at z = -6,
/// which leaves the strip between it and the wall open. That is what makes a
/// detour possible, and so what makes a cost difference observable: with the
/// band cheap the corridor crosses it, with the band expensive it does not.
fn bandVolume(area: i32) zrecast.AreaVolume {
    return zrecast.boxVolume(area, -1, 2, .{ -3, -12 }, .{ 3, -6 });
}

/// A block over the gap at the wall's +X end — the only way from one half of
/// the fixture to the other.
fn gapVolume(area: i32) zrecast.AreaVolume {
    return zrecast.boxVolume(area, -1, 4, .{ 3, -3 }, .{ 12, 3 });
}

/// Bake, load and query the fixture, with authoring applied.
const AuthoredWorld = struct {
    poly: zrecast.PolyMesh,
    mesh: zrecast.NavMesh,
    query: zrecast.NavMeshQuery,

    fn init(authoring: zrecast.AreaAuthoring) !AuthoredWorld {
        const poly = try zrecast.PolyMesh.bake(
            zrecast.defaultConfig(),
            fixtureMesh(),
            authoring,
            null,
        );
        errdefer poly.deinit();
        const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
        errdefer mesh.deinit();
        const query = try zrecast.NavMeshQuery.init(mesh, 2048);
        return .{ .poly = poly, .mesh = mesh, .query = query };
    }

    fn deinit(self: AuthoredWorld) void {
        self.query.deinit();
        self.mesh.deinit();
        self.poly.deinit();
    }
};

/// Two points in the open half, on a straight line that crosses `bandVolume`.
const south_west = [3]f32{ -8, 0, -8 };
const south_east = [3]f32{ 8, 0, -8 };

/// How many polygons of a corridor carry `area`.
fn corridorAreaCount(
    mesh: zrecast.NavMesh,
    corridor: []const zrecast.PolyRef,
    area: i32,
) !usize {
    var found: usize = 0;
    for (corridor) |ref| {
        if (try mesh.polyArea(ref) == area) found += 1;
    }
    return found;
}

/// Finds a corridor between two points, or fails the test if either end is off
/// the mesh.
fn corridorBetween(
    world: AuthoredWorld,
    filter: *const zrecast.Filter,
    from: [3]f32,
    to: [3]f32,
    out: []zrecast.PolyRef,
) !zrecast.PathResult {
    const start = try world.query.findNearestPoly(from, search_extents, filter);
    const end = try world.query.findNearestPoly(to, search_extents, filter);
    try std.testing.expect(start.ref != null);
    try std.testing.expect(end.ref != null);
    return world.query.findPath(
        start.ref.?,
        end.ref.?,
        start.point,
        end.point,
        filter,
        out,
    );
}

test "a costly area sends the path around it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const volumes = [_]zrecast.AreaVolume{bandVolume(marked_area)};
    const world = try AuthoredWorld.init(.{ .volumes = &volumes });
    defer world.deinit();

    var corridor: [128]zrecast.PolyRef = undefined;

    // Cheap: the corridor is free to cross the band, and the shortest route
    // does. Without this the expensive case below proves nothing.
    const cheap = zrecast.defaultFilter();
    const through = try corridorBetween(world, &cheap, south_west, south_east, &corridor);
    try std.testing.expect(!through.partial);
    const crossed = try corridorAreaCount(world.mesh, corridor[0..through.len], marked_area);
    try std.testing.expect(crossed > 0);

    // Expensive: the same query, the same navmesh, a filter that charges more
    // for the band than the detour costs.
    var costly = zrecast.defaultFilter();
    costly.area_cost[@intCast(marked_area)] = 50;
    const around = try corridorBetween(world, &costly, south_west, south_east, &corridor);
    try std.testing.expect(!around.partial);
    const avoided = try corridorAreaCount(world.mesh, corridor[0..around.len], marked_area);
    try std.testing.expectEqual(@as(usize, 0), avoided);
}

test "a volume marked unwalkable carves a hole a path cannot cross" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const filter = zrecast.defaultFilter();
    var corridor: [128]zrecast.PolyRef = undefined;

    // The gap at the wall's end is the only route between the two halves, and
    // an unauthored bake finds it.
    {
        const open = try AuthoredWorld.init(.{});
        defer open.deinit();
        const path = try corridorBetween(open, &filter, fixture.start, fixture.goal, &corridor);
        try std.testing.expect(!path.partial);
    }

    // Blocking it with an area_null volume leaves the goal unreachable, which
    // Detour reports as a partial path rather than a short one.
    const volumes = [_]zrecast.AreaVolume{gapVolume(zrecast.area_null)};
    const blocked = try AuthoredWorld.init(.{ .volumes = &volumes });
    defer blocked.deinit();
    const path = try corridorBetween(blocked, &filter, fixture.start, fixture.goal, &corridor);
    try std.testing.expect(path.partial);
}

test "the area-to-flag table decides which polygons a filter admits" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    var table = [_]u16{0} ** c.max_areas;
    table[zrecast.area_walkable] = zrecast.poly_flag_walkable;
    table[@intCast(marked_area)] = marked_flag;

    const volumes = [_]zrecast.AreaVolume{bandVolume(marked_area)};
    const world = try AuthoredWorld.init(.{
        .volumes = &volumes,
        .area_flags = &table,
    });
    defer world.deinit();

    // A point inside the band carries the table's flag, not the default one.
    const inside = [3]f32{ 0, 0, -8 };
    var admits_all = zrecast.defaultFilter();
    admits_all.include_flags = zrecast.poly_flag_walkable | marked_flag;
    const found = try world.query.findNearestPoly(inside, search_extents, &admits_all);
    try std.testing.expect(found.ref != null);
    try std.testing.expectEqual(marked_area, try world.mesh.polyArea(found.ref.?));
    try std.testing.expectEqual(marked_flag, try world.mesh.polyFlags(found.ref.?));

    // A filter that excludes that flag cannot see the polygon at all, which is
    // the difference between an area and a flag: one costs, the other refuses.
    var excludes_band = zrecast.defaultFilter();
    excludes_band.include_flags = zrecast.poly_flag_walkable | marked_flag;
    excludes_band.exclude_flags = marked_flag;
    const hidden = try world.query.findNearestPoly(inside, search_extents, &excludes_band);
    if (hidden.ref) |ref| {
        try std.testing.expect(try world.mesh.polyArea(ref) != marked_area);
    }
}

test "a polygon's area and flags can be rewritten after the bake" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);
    const ref = near.ref.?;

    // What the bake decided.
    try std.testing.expectEqual(@as(i32, zrecast.area_walkable), try world.mesh.polyArea(ref));
    try std.testing.expectEqual(zrecast.poly_flag_walkable, try world.mesh.polyFlags(ref));

    try world.mesh.setPolyArea(ref, marked_area);
    try std.testing.expectEqual(marked_area, try world.mesh.polyArea(ref));

    // Clearing the flags takes the polygon out of every query that filters on
    // them, which is what makes this a runtime mechanism rather than a label.
    try world.mesh.setPolyFlags(ref, 0);
    try std.testing.expectEqual(@as(u16, 0), try world.mesh.polyFlags(ref));
    const after = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    if (after.ref) |other| try std.testing.expect(other != ref);
}

test "the polygon accessors refuse a reference they cannot resolve" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);
    const ref = near.ref.?;

    // Zero is "no polygon" everywhere else in the API, and is not a query that
    // failed.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.polyArea(0));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.polyFlags(0));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.setPolyArea(0, 1));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.setPolyFlags(0, 1));

    // An area id Detour would silently truncate into its six-bit field.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.setPolyArea(ref, c.max_areas),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.setPolyArea(ref, -1),
    );

    // A plausible index carrying a salt that belongs to nothing.
    const bogus: zrecast.PolyRef = ref ^ 0x8000_0000;
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.polyArea(bogus));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.setPolyFlags(bogus, 1));
}

test "area authoring Recast could not read is refused before anything is baked" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const mesh = fixtureMesh();
    const config = zrecast.defaultConfig();

    const square = [_]f32{ -1, 0, -1, 1, 0, -1, 1, 0, 1, -1, 0, 1 };
    const good_convex = try zrecast.convexVolume(marked_area, -1, 2, &square);

    // Each of these is a way for a volume to describe geometry Recast would
    // read past the end of, rasterise as a NaN comparison, or index an
    // area-cost table with.
    var out_of_range_area = good_convex;
    out_of_range_area.area = c.max_areas;

    var negative_area = good_convex;
    negative_area.area = -1;

    var inverted_extent = good_convex;
    inverted_extent.y_min = 2;
    inverted_extent.y_max = -1;

    var nan_extent = good_convex;
    nan_extent.y_max = std.math.nan(f32);

    const nan_square = [_]f32{ -1, 0, -1, 1, 0, -1, 1, std.math.nan(f32), 1, -1, 0, 1 };
    const nan_vertex = try zrecast.convexVolume(marked_area, -1, 2, &nan_square);

    var no_verts = good_convex;
    no_verts.verts = null;

    var too_few_verts = good_convex;
    too_few_verts.vert_count = 2;

    const zero_radius = zrecast.cylinderVolume(marked_area, -1, 2, .{ 0, 0 }, 0);
    const nan_radius = zrecast.cylinderVolume(marked_area, -1, 2, .{ 0, 0 }, std.math.nan(f32));
    const inverted_box = zrecast.boxVolume(marked_area, -1, 2, .{ 3, 3 }, .{ -3, -3 });

    const bad = [_]zrecast.AreaVolume{
        out_of_range_area, negative_area, inverted_extent, nan_extent,
        nan_vertex,        no_verts,      too_few_verts,   zero_radius,
        nan_radius,        inverted_box,
    };
    for (bad) |volume| {
        const volumes = [_]zrecast.AreaVolume{volume};
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.PolyMesh.bake(config, mesh, .{ .volumes = &volumes }, null),
        );
    }

    // A flag on area_null admits a polygon nothing may stand on.
    var walkable_null = [_]u16{0} ** c.max_areas;
    walkable_null[zrecast.area_null] = zrecast.poly_flag_walkable;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.PolyMesh.bake(config, mesh, .{ .area_flags = &walkable_null }, null),
    );

    // A vertex array that is not whole vertices cannot become a footprint.
    const ragged = [_]f32{ 0, 0, 0, 1, 0 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.convexVolume(marked_area, -1, 2, &ragged),
    );
    const two_verts = [_]f32{ 0, 0, 0, 1, 0, 1 };
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        zrecast.convexVolume(marked_area, -1, 2, &two_verts),
    );

    // The positive control: the same volume, unbroken, bakes.
    const ok_volumes = [_]zrecast.AreaVolume{good_convex};
    const baked = try zrecast.PolyMesh.bake(config, mesh, .{ .volumes = &ok_volumes }, null);
    baked.deinit();
}

test "a tile bake applies the same authoring the whole-world bake does" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const config = tiledConfig();
    const grid = try zrecast.tileGrid(config, fixtureMesh());

    const volumes = [_]zrecast.AreaVolume{bandVolume(marked_area)};
    const authoring = zrecast.AreaAuthoring{ .volumes = &volumes };

    const mesh = try zrecast.NavMesh.initTiled(grid, 64, 1 << 14);
    defer mesh.deinit();

    var z: i32 = 0;
    while (z < grid.tile_count_z) : (z += 1) {
        var x: i32 = 0;
        while (x < grid.tile_count_x) : (x += 1) {
            const tile = try zrecast.PolyMesh.bakeTile(
                config,
                fixtureMesh(),
                grid,
                x,
                z,
                authoring,
                null,
            ) orelse continue;
            defer tile.deinit();

            const bytes = try zrecast.buildTileData(tile, x, z, 0, null);
            defer bytes.deinit();
            _ = try mesh.addTile(bytes.bytes);
        }
    }

    // The band lies inside the tiled world, so the polygon under a point in it
    // carries the authored area rather than the default one.
    const query = try zrecast.NavMeshQuery.init(mesh, 2048);
    defer query.deinit();
    const filter = zrecast.defaultFilter();
    const found = try query.findNearestPoly(.{ 0, 0, -8 }, search_extents, &filter);
    try std.testing.expect(found.ref != null);
    try std.testing.expectEqual(marked_area, try mesh.polyArea(found.ref.?));
}

//=============================================================================
// Off-mesh connections
//=============================================================================

/// A jump over the wall, well clear of its ends.
///
/// Both endpoints sit on ground the fixture really has — the wall occupies
/// z in [-0.3, 0.3] and erosion clears a little more — so each end attaches to
/// a polygon, and the pair is far shorter than rounding the wall at x = 4.
fn wallJump(bidirectional: bool) zrecast.OffMeshConnection {
    return .{
        .start = .{ -8, 0, -2 },
        .end = .{ -8, 0, 2 },
        .radius = 2,
        .area = zrecast.area_walkable,
        .flags = zrecast.poly_flag_walkable,
        .bidirectional = if (bidirectional) c.c_true else c.c_false,
        .user_id = 0x5AFE,
    };
}

/// Bake, load and query the fixture with `authoring` on its single tile.
const LinkedWorld = struct {
    poly: zrecast.PolyMesh,
    mesh: zrecast.NavMesh,
    query: zrecast.NavMeshQuery,
    filter: zrecast.Filter,

    fn init(authoring: ?zrecast.TileAuthoring) !LinkedWorld {
        const poly = try bakeFixture(null);
        errdefer poly.deinit();
        const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, authoring);
        errdefer mesh.deinit();
        const query = try zrecast.NavMeshQuery.init(mesh, 2048);
        return .{
            .poly = poly,
            .mesh = mesh,
            .query = query,
            .filter = zrecast.defaultFilter(),
        };
    }

    fn deinit(self: LinkedWorld) void {
        self.query.deinit();
        self.mesh.deinit();
        self.poly.deinit();
    }

    /// Whether a path between two points routes through an off-mesh
    /// connection, judged two ways at once: a corner the string-pull flags,
    /// and a polygon in the corridor whose type says so. Both have to agree.
    fn crossesLink(self: LinkedWorld, from: [3]f32, to: [3]f32) !bool {
        const start = try self.query.findNearestPoly(from, search_extents, &self.filter);
        const end = try self.query.findNearestPoly(to, search_extents, &self.filter);
        try std.testing.expect(start.ref != null);
        try std.testing.expect(end.ref != null);

        var corridor: [128]zrecast.PolyRef = undefined;
        const path = try self.query.findPath(
            start.ref.?,
            end.ref.?,
            start.point,
            end.point,
            &self.filter,
            &corridor,
        );
        try std.testing.expect(!path.partial);

        var by_type = false;
        for (corridor[0..path.len]) |ref| {
            if (try self.mesh.polyType(ref) == .offmesh_connection) by_type = true;
        }

        var points: [32][3]f32 = undefined;
        var flags: [32]u8 = undefined;
        const pulled = try self.query.findStraightPath(
            start.point,
            end.point,
            corridor[0..path.len],
            .{},
            &points,
            &flags,
            null,
        );
        var by_flag = false;
        for (flags[0..pulled.len]) |flag| {
            if (flag & zrecast.straightpath_offmesh_connection != 0) by_flag = true;
        }

        try std.testing.expectEqual(by_type, by_flag);
        return by_type;
    }
};

test "an off-mesh connection is a corner a path can be told to animate" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    // Without one, the only route is around the wall's end and nothing in the
    // package can produce a ZRC_STRAIGHTPATH_OFFMESH_CONNECTION corner.
    {
        const plain = try LinkedWorld.init(null);
        defer plain.deinit();
        try std.testing.expect(!try plain.crossesLink(fixture.start, fixture.goal));
    }

    const links = [_]zrecast.OffMeshConnection{wallJump(true)};
    const linked = try LinkedWorld.init(.{ .connections = &links });
    defer linked.deinit();
    try std.testing.expect(try linked.crossesLink(fixture.start, fixture.goal));
}

test "a one-way connection cannot be discovered from the far end" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const links = [_]zrecast.OffMeshConnection{wallJump(false)};
    const world = try LinkedWorld.init(.{ .connections = &links });
    defer world.deinit();

    // A to B is what the connection is for.
    try std.testing.expect(try world.crossesLink(fixture.start, fixture.goal));
    // B to A is the same two points the other way round. DT_OFFMESH_CON_BIDIR
    // gates exactly one link — the entry onto the connection from B's ground
    // polygon — so without it pathfinding never finds the connection from that
    // side and takes the long way round instead.
    try std.testing.expect(!try world.crossesLink(fixture.goal, fixture.start));
}

test "a connection reads back as it was authored" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const authored = wallJump(true);
    const links = [_]zrecast.OffMeshConnection{authored};
    const world = try LinkedWorld.init(.{ .connections = &links, .user_id = 0xC0FFEE });
    defer world.deinit();

    // The connection owns the tile's last polygon, so walking a corridor that
    // uses it is how to find its reference without assuming an encoding.
    const start = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const end = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try std.testing.expect(start.ref != null and end.ref != null);
    var corridor: [128]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        start.ref.?,
        end.ref.?,
        start.point,
        end.point,
        &world.filter,
        &corridor,
    );
    var link_ref: ?zrecast.PolyRef = null;
    var ground_ref: ?zrecast.PolyRef = null;
    var before_link: zrecast.PolyRef = 0;
    var previous: zrecast.PolyRef = 0;
    for (corridor[0..path.len]) |ref| {
        switch (try world.mesh.polyType(ref)) {
            .offmesh_connection => {
                link_ref = ref;
                before_link = previous;
            },
            .ground => ground_ref = ref,
        }
        previous = ref;
    }
    try std.testing.expect(link_ref != null);
    try std.testing.expect(ground_ref != null);
    try std.testing.expect(before_link != 0);

    // An off-mesh polygon is two vertices and no detail sub-mesh. The detail
    // array is sized for ground polygons only, so reading one for this polygon
    // would run past its end.
    const link_poly = try world.mesh.polyInfo(link_ref.?);
    try std.testing.expectEqual(@as(u8, 2), link_poly.vert_count);
    try std.testing.expectEqual(@as(u8, 0), link_poly.detail_tri_count);
    try std.testing.expectEqual(@as(u8, 0), link_poly.detail_vert_count);
    try std.testing.expectEqual(@as(u32, 0), link_poly.detail_tri_base);
    try std.testing.expectEqual(@as(u32, 0), link_poly.detail_vert_base);
    const ground_poly = try world.mesh.polyInfo(ground_ref.?);
    try std.testing.expect(ground_poly.detail_tri_count > 0);

    const read = try world.mesh.offMeshConnection(link_ref.?);
    try std.testing.expectEqual(authored.start, read.start);
    try std.testing.expectEqual(authored.end, read.end);
    try std.testing.expectEqual(authored.radius, read.radius);
    try std.testing.expectEqual(authored.user_id, read.user_id);
    try std.testing.expectEqual(c.c_true, read.bidirectional);
    try std.testing.expectEqual(authored.flags, read.flags);
    try std.testing.expectEqual(authored.area, read.area);
    // Both endpoints are inside the one tile, which is the only case a
    // single-tile navmesh wires at both ends.
    try std.testing.expectEqual(@as(i32, 255), read.end_side);

    // The endpoints Detour attached to are near the authored ones but snapped
    // onto polygons, so they agree in xz to well inside the search radius.
    // `prev_ref` is the corridor's polygon before the link, which is what makes
    // the pair come back the way round it was authored.
    const ends = try world.mesh.offMeshConnectionEndPoints(before_link, link_ref.?);
    for ([_]usize{ 0, 2 }) |axis| {
        try std.testing.expect(@abs(ends[0][axis] - authored.start[axis]) < authored.radius);
        try std.testing.expect(@abs(ends[1][axis] - authored.end[axis]) < authored.radius);
    }

    // Zero is not a neutral previous polygon: it is "arrived from somewhere
    // that is not endpoint A", so the pair comes back reversed. Asserting that
    // is what keeps the header's rule from being a sentence nothing checks.
    const reversed = try world.mesh.offMeshConnectionEndPoints(0, link_ref.?);
    try std.testing.expectEqual(ends[0], reversed[1]);
    try std.testing.expectEqual(ends[1], reversed[0]);

    // The tile carries the opaque id it was built with.
    const tile = try world.mesh.tileRefAtIndex(0);
    try std.testing.expectEqual(@as(u32, 0xC0FFEE), try world.mesh.tileUserId(tile));

    // Refusals. A ground polygon is not a connection, and zero is not a
    // polygon at all.
    try std.testing.expectError(
        zrecast.Error.QueryFailed,
        world.mesh.offMeshConnection(ground_ref.?),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.offMeshConnection(0),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.offMeshConnectionEndPoints(0, 0),
    );
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.polyType(0));
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileUserId(0),
    );
}

test "a connection Detour could not wire is refused before anything is built" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();

    const good = wallJump(true);

    var nan_start = good;
    nan_start.start[1] = std.math.nan(f32);

    var infinite_end = good;
    infinite_end.end[0] = std.math.inf(f32);

    var zero_radius = good;
    zero_radius.radius = 0;

    var negative_radius = good;
    negative_radius.radius = -1;

    var nan_radius = good;
    nan_radius.radius = std.math.nan(f32);

    var out_of_range_area = good;
    out_of_range_area.area = c.max_areas;

    var negative_area = good;
    negative_area.area = -1;

    const bad = [_]zrecast.OffMeshConnection{
        nan_start,  infinite_end,      zero_radius,   negative_radius,
        nan_radius, out_of_range_area, negative_area,
    };
    for (bad) |connection| {
        const links = [_]zrecast.OffMeshConnection{connection};
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.NavMesh.initFromPolyMesh(poly, .{ .connections = &links }),
        );
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            zrecast.buildTileData(poly, 0, 0, 0, .{ .connections = &links }),
        );
    }

    // The positive control: the same connection, unbroken, builds.
    const ok = [_]zrecast.OffMeshConnection{good};
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, .{ .connections = &ok });
    mesh.deinit();
}

test "an image whose off-mesh header fields disagree is rejected" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const links = [_]zrecast.OffMeshConnection{wallJump(true)};
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, .{ .connections = &links });
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    const original = image.bytes;

    // The same dtMeshHeader offsets the doctored-header test anchors.
    const off_poly_count = 24;
    const off_detail_mesh_count = 36;
    const off_bv_node_count = 48;
    const off_off_mesh_con_count = 52;
    const off_off_mesh_base = 56;

    const poly_count = std.mem.readInt(i32, original[off_poly_count..][0..4], .little);
    const off_mesh_base = std.mem.readInt(i32, original[off_off_mesh_base..][0..4], .little);
    const con_count = std.mem.readInt(i32, original[off_off_mesh_con_count..][0..4], .little);

    // The image really does carry the connection, or nothing below is testing
    // the off-mesh path at all.
    try std.testing.expectEqual(@as(i32, 1), con_count);
    try std.testing.expectEqual(poly_count, off_mesh_base + con_count);
    try std.testing.expectEqual(
        off_mesh_base,
        std.mem.readInt(i32, original[off_detail_mesh_count..][0..4], .little),
    );
    try std.testing.expectEqual(
        off_mesh_base * 2,
        std.mem.readInt(i32, original[off_bv_node_count..][0..4], .little),
    );

    const Case = struct { name: []const u8, offset: usize, value: i32 };
    const cases = [_]Case{
        // offMeshBase and offMeshConCount must together cover polyCount
        // exactly: a gap leaves ground polygons typed as connections, and an
        // overlap leaves connections indexing off the end of offMeshCons.
        .{ .name = "offMeshBase one low", .offset = off_off_mesh_base, .value = off_mesh_base - 1 },
        .{ .name = "offMeshBase one high", .offset = off_off_mesh_base, .value = off_mesh_base + 1 },
        // The value that would make the sum itself overflow, in the check
        // written to catch it.
        .{ .name = "offMeshBase at INT_MAX", .offset = off_off_mesh_base, .value = std.math.maxInt(i32) },
        .{ .name = "a negative offMeshBase", .offset = off_off_mesh_base, .value = -1 },
        .{ .name = "a negative connection count", .offset = off_off_mesh_con_count, .value = -1 },
        .{ .name = "one connection too many", .offset = off_off_mesh_con_count, .value = con_count + 1 },
        // detailMeshCount covers ground polygons only; claiming one per
        // polygon would read a sub-mesh past the end of the array.
        .{ .name = "a detail mesh per polygon", .offset = off_detail_mesh_count, .value = poly_count },
        // The BV tree is two nodes per ground polygon, not per polygon.
        .{ .name = "a BV tree sized for every polygon", .offset = off_bv_node_count, .value = poly_count * 2 },
    };

    const copy = try gpa.alloc(u8, original.len);
    defer gpa.free(copy);
    for (cases) |case| {
        @memcpy(copy, original);
        std.mem.writeInt(i32, copy[case.offset..][0..4], case.value, .little);
        std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy)) catch |e| {
            std.debug.print("accepted: {s}\n", .{case.name});
            return e;
        };
    }

    // Every count above also feeds the layout arithmetic, so patching one in
    // place leaves an image whose total size no longer matches its header —
    // and that is what rejects it, not the invariant under test. The cases
    // below shrink the array to match the count, so the layout still adds up
    // and the only thing left to notice is the count itself.
    //
    // dtMeshHeader is 100 bytes and every array before the detail meshes has a
    // stride that is already a multiple of four, so the alignment padding
    // TileLayoutOf inserts is zero here.
    const off_vert_count = 28;
    const off_max_link_count = 32;
    const vert_count = std.mem.readInt(i32, original[off_vert_count..][0..4], .little);
    const max_link_count = std.mem.readInt(i32, original[off_max_link_count..][0..4], .little);
    const detail_meshes: usize = @intCast(100 + 12 * vert_count + 32 * poly_count +
        12 * max_link_count);

    {
        // One detail sub-mesh short of a ground polygon. Detour indexes
        // detailMeshes by polygon index with no bound of its own, so the last
        // ground polygon would read a sub-mesh past the end of the array.
        const short = try gpa.alloc(u8, original.len - 12);
        defer gpa.free(short);
        const cut = detail_meshes + 12 * @as(usize, @intCast(off_mesh_base - 1));
        @memcpy(short[0..cut], original[0..cut]);
        @memcpy(short[cut..], original[cut + 12 ..]);
        std.mem.writeInt(i32, short[off_detail_mesh_count..][0..4], off_mesh_base - 1, .little);

        // Filled with a plausible dtPolyDetail — no vertices of its own, one
        // triangle borrowed from the front of the array — so the sub-mesh that
        // polygon now reads is not rejected for some incidental reason.
        //
        // The header's own detailMeshCount check is not what rejects this, and
        // no mutation of it makes this test fail: an array short of a polygon
        // is caught downstream by the detail-mesh bounds, and one longer than
        // the polygons is never read. That check is a well-formedness
        // assertion, kept because it makes the rejection certain rather than
        // incidental, and ci/probes carries nothing claiming otherwise.
        @memset(short[cut..][0..12], 0);
        short[cut + 9] = 1;
        try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(short));
    }

    // The untouched image must still load, or every rejection above proves
    // nothing.
    @memcpy(copy, original);
    const reloaded = try zrecast.NavMesh.initFromBytes(copy);
    reloaded.deinit();
}

test "an image whose off-mesh connection lies about itself is rejected" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const links = [_]zrecast.OffMeshConnection{wallJump(true)};
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, .{ .connections = &links });
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();
    const original = image.bytes;

    // dtOffMeshConnection is the last array in a tile image and is 36 bytes, so
    // the single connection this navmesh carries is its final record. Field
    // offsets within it, anchored by the assertions below.
    const record = original.len - 36;
    const off_pos = 0;
    const off_rad = 24;
    const off_poly = 28;
    const off_flags = 30;
    const off_side = 31;
    const off_off_mesh_base = 56;

    const off_mesh_base = std.mem.readInt(i32, original[off_off_mesh_base..][0..4], .little);
    // The record really is where it is assumed to be: connection 0 owns the
    // first off-mesh polygon, its side says "inside this tile", and its only
    // flag is the bidirectional one it was authored with.
    try std.testing.expectEqual(
        @as(u16, @intCast(off_mesh_base)),
        std.mem.readInt(u16, original[record + off_poly ..][0..2], .little),
    );
    try std.testing.expectEqual(@as(u8, 255), original[record + off_side]);
    try std.testing.expectEqual(@as(u8, 1), original[record + off_flags]);

    const copy = try gpa.alloc(u8, original.len);
    defer gpa.free(copy);

    const Patch = struct { name: []const u8, offset: usize, bytes: []const u8 };
    const nan = std.mem.toBytes(std.math.nan(f32));
    const negative = std.mem.toBytes(@as(f32, -1));
    const patches = [_]Patch{
        // The sharpest one. baseOffMeshLinks indexes tile->polys with this and
        // then writes through tile->verts[poly->verts[0] * 3], so a wrong value
        // is an out-of-bounds write and not merely a wrong answer.
        .{ .name = "a connection pointing at a ground polygon", .offset = record + off_poly, .bytes = &.{ 0, 0 } },
        .{ .name = "a connection pointing past the polygons", .offset = record + off_poly, .bytes = &.{ 0xff, 0xff } },
        // classifyOffMeshPoint emits 0-7 or 255 and nothing else.
        .{ .name = "a side code no classification produces", .offset = record + off_side, .bytes = &.{8} },
        .{ .name = "a side code just below the inside marker", .offset = record + off_side, .bytes = &.{254} },
        // DT_OFFMESH_CON_BIDIR is the only bit the builder ever sets.
        .{ .name = "a connection flag Detour never sets", .offset = record + off_flags, .bytes = &.{2} },
        // A radius that is not a number makes every distance comparison in
        // baseOffMeshLinks false, and a negative one an inverted search box.
        .{ .name = "a radius that is not a number", .offset = record + off_rad, .bytes = &nan },
        .{ .name = "a negative radius", .offset = record + off_rad, .bytes = &negative },
        .{ .name = "an endpoint that is not a number", .offset = record + off_pos, .bytes = &nan },
    };

    for (patches) |patch| {
        @memcpy(copy, original);
        @memcpy(copy[patch.offset..][0..patch.bytes.len], patch.bytes);
        std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy)) catch |e| {
            std.debug.print("accepted: {s}\n", .{patch.name});
            return e;
        };
    }

    // A BV-tree leaf naming an off-mesh polygon. The tree sits immediately
    // before the connection array, sixteen bytes per node with the polygon
    // index in the last four. createBVTree only ever emits ground leaves, and
    // the BV branch of queryPolygonsInTile — unlike its linear one — does not
    // filter off-mesh polygons out, so a leaf past offMeshBase would let
    // findNearestPolyInTile resolve a connection's own polygon as the ground
    // it lands on.
    const off_bv_node_count = 48;
    const bv_node_count = std.mem.readInt(i32, original[off_bv_node_count..][0..4], .little);
    try std.testing.expect(bv_node_count > 0);
    const bvtree = record - @as(usize, @intCast(bv_node_count)) * 16;
    var leaf: ?usize = null;
    for (0..@intCast(bv_node_count)) |i| {
        const at = bvtree + i * 16 + 12;
        if (std.mem.readInt(i32, original[at..][0..4], .little) >= 0) {
            leaf = at;
            break;
        }
    }
    try std.testing.expect(leaf != null);
    @memcpy(copy, original);
    std.mem.writeInt(i32, copy[leaf.?..][0..4], off_mesh_base, .little);
    try std.testing.expectError(zrecast.Error.BadFormat, zrecast.validate(copy));

    // The untouched image must still load.
    @memcpy(copy, original);
    const reloaded = try zrecast.NavMesh.initFromBytes(copy);
    reloaded.deinit();
}

//=============================================================================
// Tile state
//=============================================================================

test "a tile's state saves and restores the flags a query can see" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const tile = try world.mesh.tileRefAtIndex(0);
    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);
    const ref = near.ref.?;

    try world.mesh.setPolyFlags(ref, marked_flag);
    try world.mesh.setPolyArea(ref, marked_area);

    const size = try world.mesh.tileStateSize(tile);
    try std.testing.expect(size > 0);
    const saved = try gpa.alloc(u8, size);
    defer gpa.free(saved);
    try world.mesh.storeTileState(tile, saved);

    // Change both, and observe the change through a query rather than through
    // the accessor that wrote it.
    try world.mesh.setPolyFlags(ref, 0);
    try world.mesh.setPolyArea(ref, zrecast.area_walkable);
    const gone = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    if (gone.ref) |other| try std.testing.expect(other != ref);

    try world.mesh.restoreTileState(tile, saved);
    try std.testing.expectEqual(marked_flag, try world.mesh.polyFlags(ref));
    try std.testing.expectEqual(marked_area, try world.mesh.polyArea(ref));

    // The length has to be exact, because nothing else would catch a blob and
    // a tile that have since disagreed: upstream sizes the polygon array it
    // reads back from the live tile, not from the blob.
    try std.testing.expectError(
        zrecast.Error.BufferTooSmall,
        world.mesh.storeTileState(tile, saved[0 .. size - 1]),
    );
    const oversized = try gpa.alloc(u8, size + 1);
    defer gpa.free(oversized);
    @memcpy(oversized[0..size], saved);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.storeTileState(tile, oversized),
    );
    try std.testing.expectError(
        zrecast.Error.BufferTooSmall,
        world.mesh.restoreTileState(tile, saved[0 .. size - 1]),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.restoreTileState(tile, oversized),
    );

    // A reference to no tile at all.
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.tileStateSize(0));
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.storeTileState(0, saved),
    );

    // The blob carries a magic and a version of its own, distinct from the
    // tile image's, and restoring one that is not this format is refused
    // before any polygon is touched.
    const doctored = try gpa.alloc(u8, size);
    defer gpa.free(doctored);

    @memcpy(doctored, saved);
    std.mem.writeInt(i32, doctored[0..4], 0, .little);
    try std.testing.expectError(
        zrecast.Error.BadFormat,
        world.mesh.restoreTileState(tile, doctored),
    );

    @memcpy(doctored, saved);
    std.mem.writeInt(i32, doctored[4..8], 99, .little);
    try std.testing.expectError(
        zrecast.Error.UnsupportedVersion,
        world.mesh.restoreTileState(tile, doctored),
    );

    // And the untouched blob still restores, so the two rejections above are
    // about what was changed rather than about the blob at large.
    try world.mesh.restoreTileState(tile, saved);
    try std.testing.expectEqual(marked_flag, try world.mesh.polyFlags(ref));
}

test "state from one tile does not restore onto another" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();
    try std.testing.expect(world.mesh.tileCount() >= 2);

    const first = try world.mesh.tileRefAtIndex(0);
    const second = try world.mesh.tileRefAtIndex(1);

    const size = try world.mesh.tileStateSize(first);
    const saved = try gpa.alloc(u8, size);
    defer gpa.free(saved);
    try world.mesh.storeTileState(first, saved);

    // The blob carries the reference it came from. Restoring it onto a
    // different tile is refused even when the two happen to be the same size,
    // which is the only thing standing between a save file and a tile it does
    // not describe.
    const second_size = try world.mesh.tileStateSize(second);
    if (second_size == size) {
        try std.testing.expectError(
            zrecast.Error.InvalidArgument,
            world.mesh.restoreTileState(second, saved),
        );
    }
    try world.mesh.restoreTileState(first, saved);
}

//=============================================================================
// Reading a navmesh back
//=============================================================================

// Every accessor below is checked against something that reached the same fact
// by another route — a query, a bake count, a tile bound — rather than against
// another accessor. A read-back that only agrees with itself would pass while
// reporting the wrong array.

test "a polygon read back is the polygon a query found" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);
    const ref = near.ref.?;

    const info = try world.mesh.polyInfo(ref);
    try std.testing.expect(info.vert_count >= 3);
    try std.testing.expect(info.vert_count <= c.verts_per_polygon);
    try std.testing.expectEqual(zrecast.PolyType.ground, @as(zrecast.PolyType, @enumFromInt(info.type)));

    // The area and flags reached two ways: through the accessor this tranche
    // adds, and through the one tranche 3a added.
    try std.testing.expectEqual(try world.mesh.polyArea(ref), @as(i32, info.area));
    try std.testing.expectEqual(try world.mesh.polyFlags(ref), info.flags);

    // The point the query snapped to has to lie inside the corners the
    // accessor reports, or the vertex indices name the wrong polygon.
    const tile = try world.mesh.tileRefAtIndex(0);
    var corners: [c.verts_per_polygon][3]f32 = undefined;
    var lo = [2]f32{ std.math.floatMax(f32), std.math.floatMax(f32) };
    var hi = [2]f32{ -std.math.floatMax(f32), -std.math.floatMax(f32) };
    for (0..info.vert_count) |k| {
        try world.mesh.tileVerts(tile, info.verts[k], corners[k .. k + 1]);
        for ([_]usize{ 0, 2 }, 0..) |axis, a| {
            lo[a] = @min(lo[a], corners[k][axis]);
            hi[a] = @max(hi[a], corners[k][axis]);
        }
    }
    const eps: f32 = 1e-3;
    try std.testing.expect(near.point[0] >= lo[0] - eps and near.point[0] <= hi[0] + eps);
    try std.testing.expect(near.point[2] >= lo[1] - eps and near.point[2] <= hi[1] + eps);

    // Every corner index is inside the tile's vertex array, and every internal
    // neighbour inside its polygon array.
    const tile_info = try world.mesh.tileInfo(tile);
    for (0..info.vert_count) |k| {
        try std.testing.expect(info.verts[k] < tile_info.vert_count);
        const nei = info.neis[k];
        if (nei == 0) continue;
        if (nei & zrecast.ext_link != 0) continue;
        try std.testing.expect(nei <= tile_info.poly_count);
    }
}

test "the links read back are the links a path walked" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const start = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const end = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try std.testing.expect(start.ref != null and end.ref != null);

    var corridor: [128]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        start.ref.?,
        end.ref.?,
        start.point,
        end.point,
        &world.filter,
        &corridor,
    );
    try std.testing.expect(!path.partial);
    try std.testing.expect(path.len >= 2);

    // Consecutive polygons of a corridor are, by construction, linked. Walking
    // the chain the accessors report has to find each step, or the link array
    // being read is not the one the pathfinder used.
    const tile = try world.mesh.tileRefAtIndex(0);
    const link_capacity: u32 = @intCast((try world.mesh.tileInfo(tile)).max_link_count);
    for (corridor[0 .. path.len - 1], corridor[1..path.len]) |from, to| {
        const info = try world.mesh.polyInfo(from);
        var at = info.first_link;
        var found = false;
        var steps: u32 = 0;
        while (at != zrecast.null_link) : (steps += 1) {
            // A chain longer than the tile's whole link array is a cycle.
            try std.testing.expect(steps <= link_capacity);
            const link = try world.mesh.tileLink(tile, at);
            if (link.ref == to) found = true;
            at = link.next;
        }
        try std.testing.expect(found);
    }
}

test "a tile read back is the tile the bake placed" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();

    var total_polys: i32 = 0;
    var index: u32 = 0;
    while (index < world.mesh.tileCount()) : (index += 1) {
        const ref = try world.mesh.tileRefAtIndex(index);
        const info = try world.mesh.tileInfo(ref);

        // The bounds two ways.
        const bounds = try world.mesh.tileBounds(ref);
        try std.testing.expectEqual(bounds[0], info.bmin);
        try std.testing.expectEqual(bounds[1], info.bmax);

        // The grid position the tile was cooked at is the one calcTileLoc gives
        // for its own centre.
        const centre = [3]f32{
            (info.bmin[0] + info.bmax[0]) * 0.5,
            0,
            (info.bmin[2] + info.bmax[2]) * 0.5,
        };
        const at = try world.mesh.calcTileLoc(centre);
        try std.testing.expectEqual(info.tile_x, at.x);
        try std.testing.expectEqual(info.tile_z, at.z);

        // Ground polygons and off-mesh polygons together are the whole tile,
        // and a bake with no connections has only the first kind.
        try std.testing.expectEqual(@as(i32, 0), info.off_mesh_con_count);
        try std.testing.expectEqual(info.poly_count, info.ground_poly_count);
        try std.testing.expectEqual(info.ground_poly_count, info.detail_mesh_count);
        try std.testing.expect(info.vert_count >= 3);
        try std.testing.expectEqual(zrecast.tile_free_data, info.flags);
        total_polys += info.poly_count;
    }

    // Summed across the tiles, the accessor agrees with the count the navmesh
    // reports for itself.
    try std.testing.expectEqual(world.mesh.polyCount(), @as(u32, @intCast(total_polys)));

    // Grid parameters, against the grid the tiles were baked from.
    const params = try world.mesh.params();
    try std.testing.expectEqual(world.grid.origin[0], params.origin[0]);
    try std.testing.expectEqual(world.grid.origin[2], params.origin[2]);
    try std.testing.expectEqual(world.grid.tile_world_size, params.tile_width);
    try std.testing.expectEqual(world.grid.tile_world_size, params.tile_height);
}

test "the detail mesh read back covers every polygon exactly once" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();

    const tile = try mesh.tileRefAtIndex(0);
    const info = try mesh.tileInfo(tile);

    // The sub-meshes tile against the detail triangle array with no gaps and no
    // overlaps, which is what makes a height query well defined. Checked
    // against the count the bake reported for the polygon mesh, not against the
    // tile's own header.
    var covered: u32 = 0;
    var index: i32 = 0;
    while (index < info.ground_poly_count) : (index += 1) {
        const p = try mesh.polyInfo(try mesh.tilePolyRef(tile, @intCast(index)));
        try std.testing.expect(p.detail_tri_count >= 1);
        try std.testing.expectEqual(covered, p.detail_tri_base);
        covered += p.detail_tri_count;
    }
    try std.testing.expectEqual(@as(u32, @intCast(info.detail_tri_count)), covered);
    try std.testing.expectEqual(
        @as(u32, @intCast((try poly.info()).detail_tri_count)),
        covered,
    );
}

test "a range outside an array is an error rather than a short read" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const tile = try world.mesh.tileRefAtIndex(0);
    const info = try world.mesh.tileInfo(tile);

    var one: [1][3]f32 = undefined;
    // The last entry is readable; one past it is not.
    try world.mesh.tileVerts(tile, @intCast(info.vert_count - 1), &one);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileVerts(tile, @intCast(info.vert_count), &one),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileVerts(tile, std.math.maxInt(u32), &one),
    );

    var many: [4][3]f32 = undefined;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileVerts(tile, @intCast(info.vert_count - 2), &many),
    );

    // An empty range is a success that reads nothing.
    var none: [0][3]f32 = undefined;
    try world.mesh.tileVerts(tile, @intCast(info.vert_count), &none);

    var tri: [1][4]u8 = undefined;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileDetailTris(tile, @intCast(info.detail_tri_count), &tri),
    );

    // Indices into the link and BV arrays are bounded the same way.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileLink(tile, @intCast(info.max_link_count)),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileBvNode(tile, @intCast(info.bv_node_count)),
    );
    // A range whose end does not fit in the int the C side counts with. Added
    // to the count as an int this wraps negative and passes a bound it should
    // fail, so the arithmetic has to be done wider than the fields.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tileVerts(tile, std.math.maxInt(i32), &one),
    );

    // Every polygon of the tile is nameable, and one past the last is not.
    var index: u32 = 0;
    while (index < @as(u32, @intCast(info.poly_count))) : (index += 1) {
        const ref = try world.mesh.tilePolyRef(tile, index);
        try std.testing.expect(ref != 0);
        // The reference really names that polygon, and not merely some
        // polygon: its area is what the accessor keyed by the same index says.
        _ = try world.mesh.polyInfo(ref);
    }
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.tilePolyRef(tile, @intCast(info.poly_count)),
    );

    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.tileInfo(0));
    try std.testing.expectError(zrecast.Error.InvalidArgument, world.mesh.polyInfo(0));
}

test "a tile built without a bounding-volume tree answers the same questions" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();

    const with = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer with.deinit();
    const without = try zrecast.NavMesh.initFromPolyMesh(poly, .{ .skip_bv_tree = true });
    defer without.deinit();

    const with_info = try with.tileInfo(try with.tileRefAtIndex(0));
    const without_info = try without.tileInfo(try without.tileRefAtIndex(0));
    try std.testing.expect(with_info.bv_node_count > 0);
    try std.testing.expectEqual(@as(i32, 0), without_info.bv_node_count);

    // Smaller on disk, because the tree is the only thing that changed.
    const with_bytes = try with.serialize();
    defer with_bytes.deinit();
    const without_bytes = try without.serialize();
    defer without_bytes.deinit();
    try std.testing.expect(without_bytes.bytes.len < with_bytes.bytes.len);

    // And the same answers: without the tree Detour scans every polygon
    // instead, which is slower and not different.
    const filter = zrecast.defaultFilter();
    const a = try zrecast.NavMeshQuery.init(with, 2048);
    defer a.deinit();
    const b = try zrecast.NavMeshQuery.init(without, 2048);
    defer b.deinit();
    for ([_][3]f32{ fixture.start, fixture.goal, .{ 0, 0, -8 } }) |at| {
        const found_a = try a.findNearestPoly(at, search_extents, &filter);
        const found_b = try b.findNearestPoly(at, search_extents, &filter);
        try std.testing.expect(found_a.ref != null and found_b.ref != null);
        try std.testing.expectEqual(found_a.point, found_b.point);
    }
}

test "a position no tile coordinate could name is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();

    // Inside the grid, and outside it, are both answers rather than errors:
    // the coordinate a position would have is well defined either way.
    _ = try world.mesh.calcTileLoc(fixture.start);
    _ = try world.mesh.calcTileLoc(.{ 1000, 0, 1000 });

    // A position far enough out that the tile index does not fit in an int.
    // Upstream divides and casts without checking, and a float-to-int
    // conversion whose value does not fit is undefined behaviour, so this has
    // to be refused before Detour sees it rather than after.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.calcTileLoc(.{ 1e30, 0, 0 }),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.calcTileLoc(.{ 0, 0, -1e30 }),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.calcTileLoc(.{ std.math.nan(f32), 0, 0 }),
    );
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        world.mesh.calcTileLoc(.{ std.math.inf(f32), 0, 0 }),
    );
}

test "a tile side has an opposite, and it is an involution" {
    var side: i32 = 0;
    while (side < 8) : (side += 1) {
        const opposite = try zrecast.oppositeTileSide(side);
        try std.testing.expect(opposite >= 0 and opposite < 8);
        try std.testing.expect(opposite != side);
        try std.testing.expectEqual(side, try zrecast.oppositeTileSide(opposite));
    }
    try std.testing.expectError(zrecast.Error.InvalidArgument, zrecast.oppositeTileSide(-1));
    try std.testing.expectError(zrecast.Error.InvalidArgument, zrecast.oppositeTileSide(8));
}

//=============================================================================
// Value math
//=============================================================================

// src/vec.zig re-implements arithmetic that already exists in C, which is only
// worth doing if the two answer identically. `tests/reference.cpp` gives each
// upstream inline function a linkable name; every assertion below compares
// against one of those rather than against a number written here, so the C is
// the oracle and not a second opinion.

extern fn zrcRefDtVcross(d: *[3]f32, a: *const [3]f32, b: *const [3]f32) void;
extern fn zrcRefDtVdot(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefDtVmad(d: *[3]f32, a: *const [3]f32, b: *const [3]f32, s: f32) void;
extern fn zrcRefDtVlerp(d: *[3]f32, a: *const [3]f32, b: *const [3]f32, t: f32) void;
extern fn zrcRefDtVadd(d: *[3]f32, a: *const [3]f32, b: *const [3]f32) void;
extern fn zrcRefDtVsub(d: *[3]f32, a: *const [3]f32, b: *const [3]f32) void;
extern fn zrcRefDtVscale(d: *[3]f32, v: *const [3]f32, t: f32) void;
extern fn zrcRefDtVmin(mn: *[3]f32, v: *const [3]f32) void;
extern fn zrcRefDtVmax(mx: *[3]f32, v: *const [3]f32) void;
extern fn zrcRefDtVset(d: *[3]f32, x: f32, y: f32, z: f32) void;
extern fn zrcRefDtVcopy(d: *[3]f32, a: *const [3]f32) void;
extern fn zrcRefDtVlen(v: *const [3]f32) f32;
extern fn zrcRefDtVlenSqr(v: *const [3]f32) f32;
extern fn zrcRefDtVdist(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefDtVdistSqr(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefDtVdist2D(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefDtVdist2DSqr(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefDtVnormalize(v: *[3]f32) void;
extern fn zrcRefDtVequal(a: *const [3]f32, b: *const [3]f32) c_int;
extern fn zrcRefDtVisfinite(v: *const [3]f32) c_int;
extern fn zrcRefDtVisfinite2D(v: *const [3]f32) c_int;
extern fn zrcRefDtVdot2D(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefDtVperp2D(a: *const [3]f32, b: *const [3]f32) f32;

extern fn zrcRefRcVcross(d: *[3]f32, a: *const [3]f32, b: *const [3]f32) void;
extern fn zrcRefRcVdot(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefRcVmad(d: *[3]f32, a: *const [3]f32, b: *const [3]f32, s: f32) void;
extern fn zrcRefRcVadd(d: *[3]f32, a: *const [3]f32, b: *const [3]f32) void;
extern fn zrcRefRcVsub(d: *[3]f32, a: *const [3]f32, b: *const [3]f32) void;
extern fn zrcRefRcVmin(mn: *[3]f32, v: *const [3]f32) void;
extern fn zrcRefRcVmax(mx: *[3]f32, v: *const [3]f32) void;
extern fn zrcRefRcVcopy(d: *[3]f32, v: *const [3]f32) void;
extern fn zrcRefRcVdist(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefRcVdistSqr(a: *const [3]f32, b: *const [3]f32) f32;
extern fn zrcRefRcVnormalize(v: *[3]f32) void;

extern fn zrcRefDtNextPow2(v: u32) u32;
extern fn zrcRefDtIlog2(v: u32) u32;
extern fn zrcRefDtAlign4(x: i32) i32;
extern fn zrcRefDtMinF(a: f32, b: f32) f32;
extern fn zrcRefDtMaxF(a: f32, b: f32) f32;
extern fn zrcRefDtAbsF(a: f32) f32;
extern fn zrcRefDtSqrF(a: f32) f32;
extern fn zrcRefDtClampF(v: f32, mn: f32, mx: f32) f32;
extern fn zrcRefRcMinF(a: f32, b: f32) f32;
extern fn zrcRefRcMaxF(a: f32, b: f32) f32;
extern fn zrcRefRcAbsF(a: f32) f32;
extern fn zrcRefRcSqrF(a: f32) f32;
extern fn zrcRefRcClampF(v: f32, mn: f32, mx: f32) f32;
extern fn zrcRefRcSqrt(x: f32) f32;
extern fn zrcRefDtMathSqrtf(x: f32) f32;
extern fn zrcRefDtMathFabsf(x: f32) f32;
extern fn zrcRefDtMathFloorf(x: f32) f32;
extern fn zrcRefDtMathCeilf(x: f32) f32;
extern fn zrcRefDtMathCosf(x: f32) f32;
extern fn zrcRefDtMathSinf(x: f32) f32;
extern fn zrcRefDtMathAtan2f(y: f32, x: f32) f32;
extern fn zrcRefDtMathIsfinite(x: f32) c_int;

extern fn zrcRefTileArrayOffsets(data: *const anyopaque, out_offsets: *[8]i64) void;

const vec = zrecast.vec;

/// Bit-for-bit, not approximately. A tolerance here would hide exactly the
/// difference this whole file exists to rule out: a fused multiply-add rounds
/// once where the separate operations round twice, and the gap is one ulp.
fn expectSameBits(want: f32, got: f32) !void {
    const w: u32 = @bitCast(want);
    const g: u32 = @bitCast(got);
    if (w == g) return;
    std.debug.print(
        "C gave {x:0>8} ({d}), Zig gave {x:0>8} ({d})\n",
        .{ w, want, g, got },
    );
    return error.NotBitIdentical;
}

fn expectSameVec(want: [3]f32, got: [3]f32) !void {
    for (want, got) |w, g| try expectSameBits(w, g);
}

/// Finite inputs only, and deliberately so: the strict claim is that every
/// finite argument produces identical bits. Non-finite arguments are a
/// separate test below, because a NaN produced by two different square-root
/// instructions may carry a different payload while being equally a NaN, and
/// asserting on that would be asserting about the hardware.
const parity_table = [_][3]f32{
    .{ 0, 0, 0 },
    .{ -0.0, -0.0, -0.0 },
    .{ 1, 2, 3 },
    .{ -1, -2, -3 },
    .{ 0.1, 0.2, 0.3 },
    .{ 3, -0.0, 0 },
    // Denormals: the smallest positive float, and one just above it.
    .{ std.math.floatTrueMin(f32), -std.math.floatTrueMin(f32), 2 * std.math.floatTrueMin(f32) },
    // The bottom and top of the normal range in one vector.
    .{ std.math.floatMin(f32), 1, std.math.floatMax(f32) },
    .{ 1e18, -1e-18, 1e-30 },
    // The threshold dtVequal is written around, and its square.
    .{ 1.0 / 16384.0, -1.0 / 16384.0, 1.0 / 268435456.0 },
    .{ 16384, -16384, 0.5 },
    .{ 1e-4, 1e4, -7.25 },
};

const parity_scalars = [_]f32{ 0, -0.0, 1, -1, 0.5, -0.25, 3, 1e-30, 1e18 };

test "the Zig vector math is bit-identical to the C, for every finite input" {
    var out: [3]f32 = undefined;

    for (parity_table) |a| {
        for (parity_table) |b| {
            zrcRefDtVadd(&out, &a, &b);
            try expectSameVec(out, vec.add(a, b));
            zrcRefDtVsub(&out, &a, &b);
            try expectSameVec(out, vec.sub(a, b));
            zrcRefDtVcross(&out, &a, &b);
            try expectSameVec(out, vec.cross(a, b));

            try expectSameBits(zrcRefDtVdot(&a, &b), vec.dot(a, b));
            try expectSameBits(zrcRefDtVdot2D(&a, &b), vec.dot2D(a, b));
            try expectSameBits(zrcRefDtVperp2D(&a, &b), vec.perp2D(a, b));
            try expectSameBits(zrcRefDtVdist(&a, &b), vec.dist(a, b));
            try expectSameBits(zrcRefDtVdistSqr(&a, &b), vec.distSqr(a, b));
            try expectSameBits(zrcRefDtVdist2D(&a, &b), vec.dist2D(a, b));
            try expectSameBits(zrcRefDtVdist2DSqr(&a, &b), vec.dist2DSqr(a, b));

            // dtVmin and dtVmax mutate their first argument, and the operand
            // order decides the answer at a tie, so the copy has to be `a`.
            out = a;
            zrcRefDtVmin(&out, &b);
            try expectSameVec(out, vec.min(a, b));
            out = a;
            zrcRefDtVmax(&out, &b);
            try expectSameVec(out, vec.max(a, b));

            try std.testing.expectEqual(
                zrcRefDtVequal(&a, &b) != 0,
                vec.equal(a, b),
            );

            for (parity_scalars) |s| {
                zrcRefDtVmad(&out, &a, &b, s);
                try expectSameVec(out, vec.mad(a, b, s));
                zrcRefDtVlerp(&out, &a, &b, s);
                try expectSameVec(out, vec.lerp(a, b, s));
            }
        }

        try expectSameBits(zrcRefDtVlen(&a), vec.len(a));
        try expectSameBits(zrcRefDtVlenSqr(&a), vec.lenSqr(a));
        try std.testing.expectEqual(zrcRefDtVisfinite(&a) != 0, vec.isFinite(a));
        try std.testing.expectEqual(zrcRefDtVisfinite2D(&a) != 0, vec.isFinite2D(a));

        for (parity_scalars) |s| {
            zrcRefDtVscale(&out, &a, s);
            try expectSameVec(out, vec.scale(a, s));
        }

        // A zero vector normalises to NaN in both, which the payload-blind
        // comparison below covers; every other row is finite and compared bit
        // for bit.
        out = a;
        zrcRefDtVnormalize(&out);
        const zig_normalized = vec.normalize(a);
        for (out, zig_normalized) |w, g| {
            if (std.math.isNan(w)) {
                try std.testing.expect(std.math.isNan(g));
            } else {
                try expectSameBits(w, g);
            }
        }
    }
}

test "a set and a copy are the value semantics of a Vec3" {
    var out: [3]f32 = undefined;
    for (parity_table) |a| {
        zrcRefDtVset(&out, a[0], a[1], a[2]);
        try expectSameVec(out, vec.Vec3{ a[0], a[1], a[2] });
        var copied: [3]f32 = undefined;
        zrcRefDtVcopy(&copied, &a);
        const assigned: vec.Vec3 = a;
        try expectSameVec(copied, assigned);
    }
}

test "the rc and dt spellings of the same vector function are the same function" {
    // Both families are bound to one Zig implementation each, which is only
    // honest if upstream's two spellings agree. They are separate inline
    // definitions in separate headers, so nothing but this checks it.
    var dt_out: [3]f32 = undefined;
    var rc_out: [3]f32 = undefined;

    for (parity_table) |a| {
        for (parity_table) |b| {
            zrcRefDtVadd(&dt_out, &a, &b);
            zrcRefRcVadd(&rc_out, &a, &b);
            try expectSameVec(dt_out, rc_out);
            zrcRefDtVsub(&dt_out, &a, &b);
            zrcRefRcVsub(&rc_out, &a, &b);
            try expectSameVec(dt_out, rc_out);
            zrcRefDtVcross(&dt_out, &a, &b);
            zrcRefRcVcross(&rc_out, &a, &b);
            try expectSameVec(dt_out, rc_out);

            try expectSameBits(zrcRefDtVdot(&a, &b), zrcRefRcVdot(&a, &b));
            try expectSameBits(zrcRefDtVdist(&a, &b), zrcRefRcVdist(&a, &b));
            try expectSameBits(zrcRefDtVdistSqr(&a, &b), zrcRefRcVdistSqr(&a, &b));

            dt_out = a;
            zrcRefDtVmin(&dt_out, &b);
            rc_out = a;
            zrcRefRcVmin(&rc_out, &b);
            try expectSameVec(dt_out, rc_out);
            dt_out = a;
            zrcRefDtVmax(&dt_out, &b);
            rc_out = a;
            zrcRefRcVmax(&rc_out, &b);
            try expectSameVec(dt_out, rc_out);

            for (parity_scalars) |s| {
                zrcRefDtVmad(&dt_out, &a, &b, s);
                zrcRefRcVmad(&rc_out, &a, &b, s);
                try expectSameVec(dt_out, rc_out);
            }
        }

        zrcRefDtVcopy(&dt_out, &a);
        zrcRefRcVcopy(&rc_out, &a);
        try expectSameVec(dt_out, rc_out);

        dt_out = a;
        zrcRefDtVnormalize(&dt_out);
        rc_out = a;
        zrcRefRcVnormalize(&rc_out);
        for (dt_out, rc_out) |d, r| {
            if (std.math.isNan(d)) {
                try std.testing.expect(std.math.isNan(r));
            } else {
                try expectSameBits(d, r);
            }
        }
    }
}

test "a non-finite argument is answered the same way, NaN for NaN" {
    const nan = std.math.nan(f32);
    const inf = std.math.inf(f32);
    const rows = [_][3]f32{
        .{ nan, 0, 0 },
        .{ 0, inf, 0 },
        .{ -inf, 1, 2 },
        .{ nan, inf, -inf },
    };
    for (rows) |v| {
        try std.testing.expectEqual(zrcRefDtVisfinite(&v) != 0, vec.isFinite(v));
        try std.testing.expectEqual(zrcRefDtVisfinite2D(&v) != 0, vec.isFinite2D(v));
        // A length over a non-finite component is non-finite in both, and
        // whether it lands on NaN or on infinity has to agree.
        const c_len = zrcRefDtVlen(&v);
        const zig_len = vec.len(v);
        try std.testing.expectEqual(std.math.isNan(c_len), std.math.isNan(zig_len));
        if (!std.math.isNan(c_len)) try expectSameBits(c_len, zig_len);
    }
}

test "the scalar helpers answer what upstream answers, not what Zig would" {
    const nan = std.math.nan(f32);

    // Each of these is a value at which the obvious Zig spelling gives a
    // different answer, so each one is the reason its function is written out
    // rather than delegated to a builtin.
    try expectSameBits(zrcRefDtMinF(3, nan), vec.scalar.min(@as(f32, 3), nan));
    try expectSameBits(zrcRefDtMinF(nan, 3), vec.scalar.min(nan, @as(f32, 3)));
    try expectSameBits(zrcRefDtMaxF(3, nan), vec.scalar.max(@as(f32, 3), nan));
    try expectSameBits(zrcRefDtMaxF(nan, 3), vec.scalar.max(nan, @as(f32, 3)));
    // The asymmetry itself: swapping the operands changes the answer, which is
    // a promise @min and @max do not make.
    try std.testing.expect(std.math.isNan(zrcRefDtMinF(3, nan)));
    try std.testing.expect(!std.math.isNan(zrcRefDtMinF(nan, 3)));

    try expectSameBits(zrcRefDtAbsF(-0.0), vec.scalar.abs(@as(f32, -0.0)));
    try std.testing.expectEqual(
        @as(u32, 0x80000000),
        @as(u32, @bitCast(vec.scalar.abs(@as(f32, -0.0)))),
    );

    try expectSameBits(zrcRefDtClampF(nan, 0, 1), vec.scalar.clamp(nan, @as(f32, 0), @as(f32, 1)));
    try std.testing.expect(std.math.isNan(vec.scalar.clamp(nan, @as(f32, 0), @as(f32, 1))));

    for (parity_scalars) |s| {
        try expectSameBits(zrcRefDtSqrF(s), vec.scalar.sqr(s));
        try expectSameBits(zrcRefRcSqrF(s), vec.scalar.sqr(s));
        try expectSameBits(zrcRefRcAbsF(s), vec.scalar.abs(s));
        for (parity_scalars) |t| {
            try expectSameBits(zrcRefDtMinF(s, t), vec.scalar.min(s, t));
            try expectSameBits(zrcRefRcMinF(s, t), vec.scalar.min(s, t));
            try expectSameBits(zrcRefDtMaxF(s, t), vec.scalar.max(s, t));
            try expectSameBits(zrcRefRcMaxF(s, t), vec.scalar.max(s, t));
            try expectSameBits(zrcRefDtClampF(s, t, 1), vec.scalar.clamp(s, t, @as(f32, 1)));
            try expectSameBits(zrcRefRcClampF(s, t, 1), vec.scalar.clamp(s, t, @as(f32, 1)));
        }
    }

    // rcSqrt and dtMathSqrtf are both a plain sqrtf, which IEEE-754 requires
    // to be correctly rounded, so @sqrt is exact parity rather than close.
    for (parity_scalars) |s| {
        if (s < 0) continue;
        try expectSameBits(zrcRefRcSqrt(s), @sqrt(s));
        try expectSameBits(zrcRefDtMathSqrtf(s), @sqrt(s));
        try expectSameBits(zrcRefDtMathFabsf(s), @abs(s));
        try expectSameBits(zrcRefDtMathFloorf(s), @floor(s));
        try expectSameBits(zrcRefDtMathCeilf(s), @ceil(s));
    }
    try std.testing.expect(zrcRefDtMathIsfinite(1) != 0);
    try std.testing.expect(zrcRefDtMathIsfinite(nan) == 0);
}

test "the integer helpers answer at zero, where the obvious spelling does not" {
    // dtNextPow2 reaches 0 for 0 through an unsigned decrement that wraps to
    // 0xFFFFFFFF and an increment that wraps back. std.math.ceilPowerOfTwo
    // answers 1, and a plain `-` panics.
    try std.testing.expectEqual(@as(u32, 0), zrcRefDtNextPow2(0));
    try std.testing.expectEqual(zrcRefDtNextPow2(0), vec.scalar.nextPow2(0));

    // dtIlog2 answers 0 for 0. `31 - @clz(v)` answers -1.
    try std.testing.expectEqual(@as(u32, 0), zrcRefDtIlog2(0));
    try std.testing.expectEqual(zrcRefDtIlog2(0), vec.scalar.ilog2(0));

    var v: u32 = 0;
    while (v < 4096) : (v += 1) {
        try std.testing.expectEqual(zrcRefDtNextPow2(v), vec.scalar.nextPow2(v));
        try std.testing.expectEqual(zrcRefDtIlog2(v), vec.scalar.ilog2(v));
    }
    for ([_]u32{ 0xffff, 0x10000, 0x7fffffff, 0x80000000, 0xffffffff }) |big| {
        try std.testing.expectEqual(zrcRefDtNextPow2(big), vec.scalar.nextPow2(big));
        try std.testing.expectEqual(zrcRefDtIlog2(big), vec.scalar.ilog2(big));
    }

    for ([_]i32{ -8, -1, 0, 1, 2, 3, 4, 5, 1024, std.math.maxInt(i32) - 3 }) |x| {
        try std.testing.expectEqual(zrcRefDtAlign4(x), vec.scalar.align4(x));
    }
    // Past that the C is not an oracle: `x + 3` is signed overflow, which the
    // undefined-behaviour sanitizer traps rather than answering. So the Zig
    // side is checked against the wrap it defines, and the C is not called.
    try std.testing.expectEqual(
        @as(i32, std.math.minInt(i32)),
        vec.scalar.align4(std.math.maxInt(i32)),
    );
}

//=============================================================================
// Geometry primitives
//=============================================================================

// Each one is checked against a fact reached through a different subsystem —
// a polygon a query snapped to, a bounding-volume node read out of a tile, a
// tile's own bounds — rather than against a shape written here. A primitive
// that only agrees with a literal in this file would pass while disagreeing
// with the navmesh it is meant to reason about.

const geom = zrecast.geom;

/// The corners of a live navmesh polygon, in world space, read back through
/// the tile accessors.
fn polygonCorners(
    mesh: zrecast.NavMesh,
    ref: zrecast.PolyRef,
    out: *[c.verts_per_polygon][3]f32,
) ![]const [3]f32 {
    const info = try mesh.polyInfo(ref);
    const tile = try mesh.tileRefAtIndex(0);
    for (0..info.vert_count) |k| {
        try mesh.tileVerts(tile, info.verts[k], out[k .. k + 1]);
    }
    return out[0..info.vert_count];
}

test "the point a query snapped to is inside the polygon it snapped to" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);

    var storage: [c.verts_per_polygon][3]f32 = undefined;
    const corners = try polygonCorners(world.mesh, near.ref.?, &storage);
    try std.testing.expect(corners.len >= 3);

    // Detour placed the point on this polygon. If dtPointInPolygon disagrees,
    // either the vertex indices name a different polygon or the test is
    // reading the wrong array.
    try std.testing.expect(try geom.pointInPolygon(near.point, corners));

    // The centroid of the same corners has to be inside it too, which ties
    // dtCalcPolyCenter to the same polygon.
    var indices: [c.verts_per_polygon]u16 = undefined;
    for (0..corners.len) |k| indices[k] = @intCast(k);
    const full_centre = try geom.polyCenter(corners, indices[0..corners.len]);
    try std.testing.expect(try geom.pointInPolygon(full_centre, corners));

    // A point far outside is outside.
    try std.testing.expect(!try geom.pointInPolygon(fixture.island, corners));
}

test "a polygon read back has the winding Detour wraps in" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    var storage: [c.verts_per_polygon][3]f32 = undefined;
    const corners = try polygonCorners(world.mesh, near.ref.?, &storage);

    // Fanned from the first corner, every triangle of a convex polygon wound
    // Detour's way has positive signed area, and the total is the polygon's.
    var total: f32 = 0;
    for (1..corners.len - 1) |k| {
        const area = try geom.triArea2D(corners[0], corners[k], corners[k + 1]);
        try std.testing.expect(area >= 0);
        total += area;
    }
    try std.testing.expect(total > 0);

    // Reversing two corners negates the sign, exactly. That property is what
    // makes the function usable as a left-of-line test.
    const forward = try geom.triArea2D(corners[0], corners[1], corners[2]);
    const reversed = try geom.triArea2D(corners[0], corners[2], corners[1]);
    try std.testing.expectEqual(forward, -reversed);
}

test "a random point in a polygon is in the polygon" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    var storage: [c.verts_per_polygon][3]f32 = undefined;
    const corners = try polygonCorners(world.mesh, near.ref.?, &storage);

    // Scratch through the package's own allocator seam, which is the reason
    // that seam is reachable at all: upstream's dtRandomPointInConvexPoly
    // takes a working buffer it does not allocate.
    const scratch_bytes = try zrecast.alloc(corners.len * @sizeOf(f32), .perm);
    defer zrecast.free(scratch_bytes);
    const scratch: []f32 = @alignCast(std.mem.bytesAsSlice(f32, scratch_bytes));

    // Containment is in the CLOSED polygon, and the boundary is reachable
    // rather than exotic: t = 0 makes upstream's `v = sqrt(t)` zero, which
    // places the point exactly on the polygon's first vertex, and s = 1 sends
    // it onto the last edge because the area weights are floored at 0.001
    // while the running total is not. dtPointInPolygon is a strict crossing
    // test, so both land outside it.
    var edge_dist: [c.verts_per_polygon]f32 = undefined;
    var edge_t: [c.verts_per_polygon]f32 = undefined;
    var s: f32 = 0;
    while (s <= 1.0) : (s += 0.25) {
        var t: f32 = 0;
        while (t <= 1.0) : (t += 0.25) {
            const point = try geom.randomPointInConvexPoly(corners, scratch, s, t);
            if (try geom.pointInPolygon(point, corners)) continue;
            _ = try geom.distancePointToPolyEdges(
                point,
                corners,
                edge_dist[0..corners.len],
                edge_t[0..corners.len],
            );
            var nearest: f32 = std.math.floatMax(f32);
            for (edge_dist[0..corners.len]) |d| nearest = @min(nearest, d);
            try std.testing.expect(nearest < 1e-6);
        }
    }

    // Strictly inside, away from those ends.
    const middle = try geom.randomPointInConvexPoly(corners, scratch, 0.5, 0.5);
    try std.testing.expect(try geom.pointInPolygon(middle, corners));

    // The same two numbers give the same point: a placement is reproducible
    // from a seed rather than from hidden state.
    const first = try geom.randomPointInConvexPoly(corners, scratch, 0.3, 0.7);
    const again = try geom.randomPointInConvexPoly(corners, scratch, 0.3, 0.7);
    try std.testing.expectEqual(first, again);

    // Fewer than three vertices is upstream's read from before the array.
    try std.testing.expectError(
        error.InvalidArgument,
        geom.randomPointInConvexPoly(corners[0..2], scratch, 0.5, 0.5),
    );
    // Scratch shorter than the polygon is a write past the end of it.
    try std.testing.expectError(
        error.InvalidArgument,
        geom.randomPointInConvexPoly(corners, scratch[0 .. corners.len - 1], 0.5, 0.5),
    );
    // s and t are fractions; upstream takes the square root of t.
    try std.testing.expectError(
        error.InvalidArgument,
        geom.randomPointInConvexPoly(corners, scratch, 0.5, -0.5),
    );
}

test "a segment out of a polygon leaves it at the boundary" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    var storage: [c.verts_per_polygon][3]f32 = undefined;
    const corners = try polygonCorners(world.mesh, near.ref.?, &storage);

    var indices: [c.verts_per_polygon]u16 = undefined;
    for (0..corners.len) |k| indices[k] = @intCast(k);
    const inside = try geom.polyCenter(corners, indices[0..corners.len]);
    const outside = zrecast.Vec3{ fixture.island_x, inside[1], fixture.island_z };

    const hit = try geom.intersectSegmentPoly2D(inside, outside, corners);
    try std.testing.expect(hit.intersects);
    // Starting inside, the segment never enters; it only leaves.
    try std.testing.expectEqual(@as(f32, 0), hit.t_min);
    try std.testing.expect(hit.t_max > 0 and hit.t_max < 1);
    try std.testing.expect(hit.seg_max >= 0);
    try std.testing.expect(hit.seg_max < @as(i32, @intCast(corners.len)));

    // The point where it leaves is on the polygon's boundary, which the edge
    // distances have to agree with.
    const exit = vec.lerp(inside, outside, hit.t_max);
    var edge_dist: [c.verts_per_polygon]f32 = undefined;
    var edge_t: [c.verts_per_polygon]f32 = undefined;
    _ = try geom.distancePointToPolyEdges(
        exit,
        corners,
        edge_dist[0..corners.len],
        edge_t[0..corners.len],
    );
    var nearest: f32 = std.math.floatMax(f32);
    for (edge_dist[0..corners.len]) |d| nearest = @min(nearest, d);
    try std.testing.expect(nearest < 1e-3);

    // A segment wholly inside reports the whole of itself.
    const whole = try geom.intersectSegmentPoly2D(inside, inside, corners);
    try std.testing.expect(whole.intersects);
    try std.testing.expectEqual(@as(f32, 0), whole.t_min);
    try std.testing.expectEqual(@as(f32, 1), whole.t_max);
}

test "two segments meet at the point both parametrisations name" {
    const a0 = zrecast.Vec3{ -1, 0, 0 };
    const a1 = zrecast.Vec3{ 1, 0, 0 };
    const b0 = zrecast.Vec3{ 0, 5, -1 };
    const b1 = zrecast.Vec3{ 0, -5, 1 };

    const hit = (try geom.intersectSegSeg2D(a0, a1, b0, b1)).?;
    const from_a = vec.lerp(a0, a1, hit.s);
    const from_b = vec.lerp(b0, b1, hit.t);
    // The crossing is on the xz-plane, so only x and z have to agree; y is
    // whatever each segment carries.
    try std.testing.expectApproxEqAbs(from_a[0], from_b[0], 1e-5);
    try std.testing.expectApproxEqAbs(from_a[2], from_b[2], 1e-5);

    // Parallel lines have no crossing, reported as no answer rather than as a
    // flag beside two meaningless numbers.
    const parallel = try geom.intersectSegSeg2D(
        a0,
        a1,
        .{ -1, 0, 1 },
        .{ 1, 0, 1 },
    );
    try std.testing.expect(parallel == null);
}

test "a shrunk polygon lies inside the one it came from" {
    const square = [_]zrecast.Vec3{
        .{ -4, 0, -4 },
        .{ 4, 0, -4 },
        .{ 4, 0, 4 },
        .{ -4, 0, 4 },
    };

    var out: [16]zrecast.Vec3 = undefined;
    const written = try geom.offsetPoly(&square, -1.0, &out);
    try std.testing.expect(written >= square.len);

    for (out[0..written]) |v| {
        try std.testing.expect(try geom.pointInPolygon(v, &square));
    }

    // Grown the other way, every original corner is inside the result.
    const grown = try geom.offsetPoly(&square, 1.0, &out);
    for (square) |v| {
        try std.testing.expect(try geom.pointInPolygon(v, out[0..grown]));
    }

    // Upstream's own bound refuses to fill the last slot, so a buffer sized
    // exactly to the result is reported as too small rather than filled.
    var exact: [4]zrecast.Vec3 = undefined;
    try std.testing.expectError(
        error.BufferTooSmall,
        geom.offsetPoly(&square, -1.0, &exact),
    );
}

test "a bounding-volume node overlaps the leaves beneath it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const tile = try world.mesh.tileRefAtIndex(0);
    const info = try world.mesh.tileInfo(tile);
    try std.testing.expect(info.bv_node_count > 0);

    const root = try world.mesh.tileBvNode(tile, 0);

    // A tile reserves two nodes per polygon and the tree fills 2n - 1 of them,
    // so the last is spare. zrecast seals it as an internal node; upstream
    // leaves it zeroed, which reads as a leaf naming polygon 0. See UPSTREAM.md.
    const tree_nodes: u32 = @intCast(info.bv_node_count - 1);
    const sealed = try world.mesh.tileBvNode(tile, tree_nodes);
    try std.testing.expect(sealed.i < 0);

    // Every node of a bounding-volume hierarchy is inside its root, so the
    // test Detour runs during a query has to say so for all of them.
    var index: u32 = 0;
    while (index < tree_nodes) : (index += 1) {
        const node = try world.mesh.tileBvNode(tile, index);
        try std.testing.expect(try geom.overlapQuantBounds(
            root.bmin,
            root.bmax,
            node.bmin,
            node.bmax,
        ));
    }

    // Two boxes on opposite sides of the quantised space do not overlap.
    try std.testing.expect(!try geom.overlapQuantBounds(
        .{ 0, 0, 0 },
        .{ 1, 1, 1 },
        .{ 100, 100, 100 },
        .{ 200, 200, 200 },
    ));
}

test "a tile overlaps itself and not a tile across the world" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try TiledWorld.init();
    defer world.deinit();
    try std.testing.expect(world.mesh.tileCount() >= 2);

    const first = try world.mesh.tileBounds(try world.mesh.tileRefAtIndex(0));
    try std.testing.expect(try geom.overlapBounds(first[0], first[1], first[0], first[1]));

    // The fixture's island sits at (40, 40) with the ground plane around the
    // origin, so some pair of resident tiles is far enough apart to be
    // disjoint. Finding one proves the test is not vacuous.
    var disjoint = false;
    var i: u32 = 0;
    while (i < world.mesh.tileCount()) : (i += 1) {
        const a = try world.mesh.tileBounds(try world.mesh.tileRefAtIndex(i));
        var j: u32 = 0;
        while (j < world.mesh.tileCount()) : (j += 1) {
            const b = try world.mesh.tileBounds(try world.mesh.tileRefAtIndex(j));
            if (!try geom.overlapBounds(a[0], a[1], b[0], b[1])) disjoint = true;
        }
    }
    try std.testing.expect(disjoint);
}

test "the closest point on a triangle is the point itself when it is on it" {
    const a = zrecast.Vec3{ 0, 0, 0 };
    const b = zrecast.Vec3{ 4, 0, 0 };
    const t = zrecast.Vec3{ 0, 0, 4 };

    const inside = zrecast.Vec3{ 1, 0, 1 };
    const on = try geom.closestPointOnTriangle(inside, a, b, t);
    for (inside, on) |want, got| try std.testing.expectApproxEqAbs(want, got, 1e-5);

    // Outside past a corner, the closest point is that corner exactly.
    const past_a = zrecast.Vec3{ -5, 0, -5 };
    try std.testing.expectEqual(a, try geom.closestPointOnTriangle(past_a, a, b, t));

    // Outside across an edge, the closest point is on that edge, so its
    // distance to the segment is zero.
    const past_edge = zrecast.Vec3{ 2, 0, -5 };
    const on_edge = try geom.closestPointOnTriangle(past_edge, a, b, t);
    const to_ab = try geom.distancePointToSegment2D(on_edge, a, b);
    try std.testing.expect(to_ab.dist_sqr < 1e-6);
    try std.testing.expect(to_ab.t > 0 and to_ab.t < 1);
}

test "an array length the C cannot check is checked here" {
    const square = [_]zrecast.Vec3{
        .{ -1, 0, -1 },
        .{ 1, 0, -1 },
        .{ 1, 0, 1 },
        .{ -1, 0, 1 },
    };
    var dist: [4]f32 = undefined;
    var t: [4]f32 = undefined;

    // Both output arrays take one entry per edge. The C entry point is handed
    // bare pointers and cannot tell that either is short.
    _ = try geom.distancePointToPolyEdges(.{ 0, 0, 0 }, &square, &dist, &t);
    try std.testing.expectError(
        error.InvalidArgument,
        geom.distancePointToPolyEdges(.{ 0, 0, 0 }, &square, dist[0..3], &t),
    );
    try std.testing.expectError(
        error.InvalidArgument,
        geom.distancePointToPolyEdges(.{ 0, 0, 0 }, &square, &dist, t[0..3]),
    );

    // An index past the end of the vertex array: upstream's dtCalcPolyCenter
    // takes no vertex count at all and would read through it.
    try std.testing.expectError(
        error.InvalidArgument,
        geom.polyCenter(&square, &[_]u16{ 0, 1, 4 }),
    );
    try std.testing.expectError(error.InvalidArgument, geom.polyCenter(&square, &[_]u16{}));

    // Two vertices is not a polygon, and for the overlap test it is upstream's
    // read of the first vertex of an empty one.
    try std.testing.expectError(
        error.InvalidArgument,
        geom.pointInPolygon(.{ 0, 0, 0 }, square[0..2]),
    );
    try std.testing.expectError(
        error.InvalidArgument,
        geom.overlapPolyPoly2D(&square, square[0..1]),
    );
}

test "two polygons overlap when they share ground" {
    const a = [_]zrecast.Vec3{
        .{ 0, 0, 0 },
        .{ 2, 0, 0 },
        .{ 2, 0, 2 },
        .{ 0, 0, 2 },
    };
    const shifted = [_]zrecast.Vec3{
        .{ 1, 0, 1 },
        .{ 3, 0, 1 },
        .{ 3, 0, 3 },
        .{ 1, 0, 3 },
    };
    const far = [_]zrecast.Vec3{
        .{ 10, 0, 10 },
        .{ 12, 0, 10 },
        .{ 12, 0, 12 },
        .{ 10, 0, 12 },
    };
    try std.testing.expect(try geom.overlapPolyPoly2D(&a, &shifted));
    try std.testing.expect(!try geom.overlapPolyPoly2D(&a, &far));
}

//=============================================================================
// The layout of a tile image
//=============================================================================

test "the tile layout is the one Detour derives from the same bytes" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();

    const layout = try zrecast.tileLayout(image.bytes);
    try std.testing.expectEqual(image.bytes.len, layout.total_size);

    // The oracle walks the same buffer with dtGetThenAdvanceBufferPointer, the
    // primitive dtNavMesh::addTile itself lays a tile out with. Reproducing
    // the arithmetic and reproducing the walk have to agree.
    var offsets: [8]i64 = undefined;
    zrcRefTileArrayOffsets(image.bytes.ptr, &offsets);
    const reported = [_]usize{
        layout.verts.offset,
        layout.polys.offset,
        layout.links.offset,
        layout.detail_meshes.offset,
        layout.detail_verts.offset,
        layout.detail_tris.offset,
        layout.bv_tree.offset,
        layout.off_mesh_cons.offset,
    };
    for (offsets, reported) |want, got| {
        try std.testing.expectEqual(@as(usize, @intCast(want)), got);
    }

    // Every range is inside the image and none of them overlaps the next.
    var at: usize = layout.verts.offset;
    for ([_]zrecast.Range{
        layout.verts,        layout.polys,
        layout.links,        layout.detail_meshes,
        layout.detail_verts, layout.detail_tris,
        layout.bv_tree,      layout.off_mesh_cons,
    }) |range| {
        try std.testing.expect(range.offset >= at);
        try std.testing.expect(range.offset + range.len <= image.bytes.len);
        at = range.offset + range.len;
    }
}

test "the vertices at the layout's offset are the vertices the accessor reports" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    defer mesh.deinit();
    const image = try mesh.serialize();
    defer image.deinit();

    const loaded = try zrecast.NavMesh.initFromBytes(image.bytes);
    defer loaded.deinit();
    const tile = try loaded.tileRefAtIndex(0);
    const info = try loaded.tileInfo(tile);

    const layout = try zrecast.tileLayout(image.bytes);
    try std.testing.expectEqual(
        @as(usize, @intCast(info.vert_count)) * 3 * @sizeOf(f32),
        layout.verts.len,
    );

    // Read the raw bytes at the offset the layout names, and compare against
    // what the tile accessor hands back from the loaded navmesh. Two routes to
    // the same floats: one through the image, one through dtNavMesh.
    const raw = layout.verts.of(image.bytes);
    const from_image: []align(1) const f32 = std.mem.bytesAsSlice(f32, raw);

    const from_tile = try gpa.alloc([3]f32, @intCast(info.vert_count));
    defer gpa.free(from_tile);
    try loaded.tileVerts(tile, 0, from_tile);

    for (from_tile, 0..) |want, k| {
        for (want, 0..) |component, axis| {
            try std.testing.expectEqual(component, from_image[k * 3 + axis]);
        }
    }
}

test "a buffer that is not a tile image has no layout" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    var junk = [_]u8{0} ** 256;
    try std.testing.expectError(error.BadFormat, zrecast.tileLayout(&junk));
    try std.testing.expectError(error.BadFormat, zrecast.tileLayout(junk[0..4]));
}

//=============================================================================
// The seams
//=============================================================================

test "a block from the allocator seam is the host's own block" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    // The test allocator fails the test if this leaks or is freed wrongly,
    // which is what makes the round trip an assertion rather than a smoke
    // test: zrcAlloc has to reach the installed Zig allocator, and zrcFree has
    // to unwind the private header the bridge puts ahead of the payload.
    var size: usize = 1;
    while (size <= 4096) : (size *= 4) {
        const block = try zrecast.alloc(size, .perm);
        try std.testing.expectEqual(size, block.len);
        try std.testing.expect(@intFromPtr(block.ptr) % c.alloc_alignment == 0);
        @memset(block, 0x5A);
        zrecast.free(block);
    }

    const temp = try zrecast.alloc(64, .temp);
    zrecast.free(temp);
}

test "the assertion seam reports the build it was compiled into" {
    // The only thing tying the C entry point to how build.zig compiled the
    // library. Upstream's whole hook family is inside `#else NDEBUG`, so a
    // release build cannot call a handler at all, and a host that could not
    // ask would have no way to know.
    try std.testing.expectEqual(
        zrecast.options.enable_asserts,
        zrecast.assertsEnabled(),
    );

    // Nothing here can make an assertion fire: every entry point checks its
    // arguments before upstream sees them, which is the whole point of the
    // validation layer, and in a release build the assertions are not compiled
    // in at all. What is checkable is that the seam holds a handler and gives
    // it back, and that it reports the build honestly.
    const Sink = struct {
        fn fail(user: ?*anyopaque, failure: zrecast.AssertFailure) void {
            _ = user;
            _ = failure;
        }
    };

    var context: u32 = 7;
    try zrecast.setAssertHandler(.{ .fail = Sink.fail, .user = &context });
    const installed = (try zrecast.assertHandler()) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(installed.fail == &Sink.fail);
    try std.testing.expectEqual(@as(?*anyopaque, &context), installed.user);

    try zrecast.setAssertHandler(null);
    try std.testing.expect((try zrecast.assertHandler()) == null);
}

test "a query box no tile range could hold is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    // Two finite floats whose sum is not. dtVisfinite is upstream's whole
    // check, and it looks at each argument on its own, so this pair reaches
    // dtNavMesh::calcTileLoc as an infinity and converts it to an int — which
    // the undefined-behaviour sanitizer reports as "outside the range of
    // representable values of type 'int'". Reachable from findNearestPoly,
    // which shipped before this check existed.
    const half = std.math.floatMax(f32) * 0.5;
    try std.testing.expectError(error.InvalidArgument, world.query.findNearestPoly(
        .{ half, 0, 0 },
        .{ half, 1, 1 },
        &world.filter,
    ));

    // Finite the whole way, and still not a query: the box spans more grid
    // cells than Detour's scan could walk in any reasonable time, because that
    // scan visits every cell of the range whether a tile is there or not. Far
    // enough out that a single axis is over the limit.
    try std.testing.expectError(error.InvalidArgument, world.query.findNearestPoly(
        .{ 0, 0, 0 },
        .{ 1e9, 1, 1e9 },
        &world.filter,
    ));

    // And again with each axis comfortably under the limit and only their
    // product over it, which is the case a per-axis bound alone would miss:
    // the fixture's tile is about 53 m, so this is roughly 3,800 cells on a
    // side and 14 million in total.
    try std.testing.expectError(error.InvalidArgument, world.query.findNearestPoly(
        .{ 0, 0, 0 },
        .{ 1e5, 1, 1e5 },
        &world.filter,
    ));

    // A negative extent is not a box at all.
    try std.testing.expectError(error.InvalidArgument, world.query.findNearestPoly(
        fixture.start,
        .{ -1, 1, 1 },
        &world.filter,
    ));

    // And an extent large enough to cover the whole navmesh several times over
    // is still an ordinary query.
    const generous = try world.query.findNearestPoly(fixture.start, .{ 500, 500, 500 }, &world.filter);
    try std.testing.expect(generous.ref != null);
}

//=============================================================================
// Sliced pathfinding
//=============================================================================

/// A start and goal on opposite sides of the fixture's wall, snapped to the
/// navmesh. The corridor between them is several polygons long, which is what
/// makes a sliced search take more than one update.
const Endpoints = struct {
    start: zrecast.NearestPoly,
    goal: zrecast.NearestPoly,

    fn find(world: World) !Endpoints {
        const s = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
        const g = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
        try std.testing.expect(s.ref != null and g.ref != null);
        return .{ .start = s, .goal = g };
    }
};

test "a sliced path is the path found all at once" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const ends = try Endpoints.find(world);

    var whole: [256]zrecast.PolyRef = undefined;
    const one_shot = try world.query.findPath(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        &whole,
    );
    try std.testing.expect(!one_shot.partial);
    try std.testing.expect(one_shot.len > 2);

    try std.testing.expect(!world.query.slicedFindPathActive());
    try world.query.slicedFindPathInit(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        .{},
    );
    try std.testing.expect(world.query.slicedFindPathActive());

    // Two iterations at a time, so the budget is doing something: a search
    // that finished in one round would prove nothing about slicing.
    var rounds: u32 = 0;
    var total_iterations: u32 = 0;
    while (rounds < 1000) : (rounds += 1) {
        const progress = try world.query.slicedFindPathUpdate(2);
        total_iterations += progress.iterations;
        if (!progress.in_progress) break;
    }
    try std.testing.expect(rounds > 1);
    try std.testing.expect(total_iterations > 0);

    var sliced: [256]zrecast.PolyRef = undefined;
    const finished = try world.query.slicedFindPathFinalize(&sliced);
    try std.testing.expect(!finished.partial);
    try std.testing.expectEqualSlices(
        zrecast.PolyRef,
        whole[0..one_shot.len],
        sliced[0..finished.len],
    );
    try std.testing.expect(!world.query.slicedFindPathActive());
}

test "a search that would clear the pool under a slice is refused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const ends = try Endpoints.find(world);

    try world.query.slicedFindPathInit(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        .{},
    );
    defer world.query.slicedFindPathCancel() catch {};

    // The five that begin by clearing the shared node pool. Detour's own class
    // comment says every const method is safe alongside a slice; these are the
    // five for which that is false.
    var refs: [16]zrecast.PolyRef = undefined;
    var parents: [16]zrecast.PolyRef = undefined;
    var costs: [16]f32 = undefined;
    var path: [16]zrecast.PolyRef = undefined;
    try std.testing.expectError(error.SearchInProgress, world.query.findPath(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        &path,
    ));
    try std.testing.expectError(error.SearchInProgress, world.query.findPolysAroundCircle(
        ends.start.ref.?,
        ends.start.point,
        5,
        &world.filter,
        &refs,
        &parents,
        &costs,
    ));
    const shape = [_]zrecast.Vec3{
        .{ -1, 0, -1 }, .{ 1, 0, -1 }, .{ 1, 0, 1 }, .{ -1, 0, 1 },
    };
    try std.testing.expectError(error.SearchInProgress, world.query.findPolysAroundShape(
        ends.start.ref.?,
        &shape,
        &world.filter,
        &refs,
        &parents,
        &costs,
    ));
    try std.testing.expectError(error.SearchInProgress, world.query.findDistanceToWall(
        ends.start.ref.?,
        ends.start.point,
        5,
        &world.filter,
    ));
    try std.testing.expectError(error.SearchInProgress, world.query.findRandomPointAroundCircle(
        ends.start.ref.?,
        ends.start.point,
        5,
        &world.filter,
        fixedRandom,
    ));

    // And the ones that touch no shared node keep working, because refusing
    // them would be a restriction the defect does not justify: a game loop
    // calls these between slices.
    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expectEqual(ends.start.ref, near.ref);
    var boxed: [64]zrecast.PolyRef = undefined;
    _ = try world.query.queryPolygons(fixture.start, search_extents, &world.filter, &boxed);
    _ = try world.query.raycast(
        ends.start.ref.?,
        ends.start.point,
        fixture.goal,
        &world.filter,
        .{},
        null,
        null,
    );
    var visited: [16]zrecast.PolyRef = undefined;
    _ = try world.query.moveAlongSurface(
        ends.start.ref.?,
        ends.start.point,
        fixture.goal,
        &world.filter,
        &visited,
    );
    var local: [16]zrecast.PolyRef = undefined;
    _ = try world.query.findLocalNeighbourhood(
        ends.start.ref.?,
        ends.start.point,
        3,
        &world.filter,
        &local,
        &.{},
    );
    _ = try world.query.findRandomPoint(&world.filter, fixedRandom);

    // The slice survived all of that.
    try std.testing.expect(world.query.slicedFindPathActive());
    var sliced: [256]zrecast.PolyRef = undefined;
    var rounds: u32 = 0;
    while (rounds < 1000) : (rounds += 1) {
        if (!(try world.query.slicedFindPathUpdate(64)).in_progress) break;
    }
    const finished = try world.query.slicedFindPathFinalize(&sliced);
    try std.testing.expect(finished.len > 0);
    try std.testing.expectEqual(ends.start.ref.?, sliced[0]);
    try std.testing.expectEqual(ends.goal.ref.?, sliced[finished.len - 1]);
}

test "a slice finalised twice is refused, and cancelling releases it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const ends = try Endpoints.find(world);

    // Nothing in flight: there is nothing to advance or finalise.
    var path: [64]zrecast.PolyRef = undefined;
    try std.testing.expectError(error.NoSearch, world.query.slicedFindPathUpdate(4));
    try std.testing.expectError(error.NoSearch, world.query.slicedFindPathFinalize(&path));

    try world.query.slicedFindPathInit(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        .{},
    );
    var rounds: u32 = 0;
    while (rounds < 1000) : (rounds += 1) {
        if (!(try world.query.slicedFindPathUpdate(64)).in_progress) break;
    }
    _ = try world.query.slicedFindPathFinalize(&path);

    // Upstream would zero its own query state here, misread that as "start and
    // end are the same polygon" — both being 0 — and hand back a one-element
    // path holding the null reference, reported as success.
    try std.testing.expectError(error.NoSearch, world.query.slicedFindPathFinalize(&path));

    // A bad buffer is an error and nothing more: the slice is not thrown away
    // as a side effect of getting the arguments wrong.
    try world.query.slicedFindPathInit(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        .{},
    );
    try std.testing.expectError(error.InvalidArgument, world.query.slicedFindPathFinalize(&.{}));
    try std.testing.expect(world.query.slicedFindPathActive());

    try world.query.slicedFindPathCancel();
    try std.testing.expect(!world.query.slicedFindPathActive());
    // Cancelling twice is not an error, and the pool is free again.
    try world.query.slicedFindPathCancel();
    _ = try world.query.findPath(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        &path,
    );
}

test "a slice finalised early keeps the corridor it was re-planning" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const ends = try Endpoints.find(world);

    var whole: [256]zrecast.PolyRef = undefined;
    const one_shot = try world.query.findPath(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        &whole,
    );

    // Re-plan from the goal back to the start while walking the corridor the
    // other way, then stop the search early and splice what it has onto what
    // is already known.
    try world.query.slicedFindPathInit(
        ends.goal.ref.?,
        ends.start.ref.?,
        ends.goal.point,
        ends.start.point,
        &world.filter,
        .{},
    );
    _ = try world.query.slicedFindPathUpdate(1);

    var joined: [256]zrecast.PolyRef = undefined;
    const partial = try world.query.slicedFindPathFinalizePartial(
        whole[0..one_shot.len],
        &joined,
    );
    try std.testing.expect(partial.len > 0);
    // Whatever it returns starts at the polygon the search started from.
    try std.testing.expectEqual(ends.goal.ref.?, joined[0]);
    try std.testing.expect(!world.query.slicedFindPathActive());
}

//=============================================================================
// Query breadth
//=============================================================================

/// A random source with no randomness in it: two numbers that step through a
/// fixed cycle. Placement has to be a pure function of what this returns, and
/// a fixed cycle is what makes that testable.
var fixed_random_state: u32 = 0;
fn fixedRandomNext(user: ?*anyopaque) callconv(.c) f32 {
    _ = user;
    fixed_random_state = fixed_random_state *% 1664525 +% 1013904223;
    return @as(f32, @floatFromInt(fixed_random_state >> 8)) / 16777216.0;
}
const fixedRandom = zrecast.RandomSource{ .next = fixedRandomNext };

test "a random point is on the navmesh, and the same numbers give the same point" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    fixed_random_state = 12345;
    const first = try world.query.findRandomPoint(&world.filter, fixedRandom);
    try std.testing.expect(first.ref != 0);

    // On the navmesh, checked through a different entry point entirely.
    const snapped = try world.query.findNearestPoly(first.point, .{ 1, 1, 1 }, &world.filter);
    try std.testing.expectEqual(first.ref, snapped.ref.?);
    try std.testing.expect(snapped.over_poly);

    // The same sequence of numbers places the same point, so a host can seed
    // a placement and reproduce it.
    fixed_random_state = 12345;
    const again = try world.query.findRandomPoint(&world.filter, fixedRandom);
    try std.testing.expectEqual(first.ref, again.ref);
    try std.testing.expectEqual(first.point, again.point);

    // A different sequence eventually places a different point.
    var moved = false;
    var seed: u32 = 1;
    while (seed < 40 and !moved) : (seed += 1) {
        fixed_random_state = seed;
        const other = try world.query.findRandomPoint(&world.filter, fixedRandom);
        if (other.ref != first.ref or !std.meta.eql(other.point, first.point)) moved = true;
    }
    try std.testing.expect(moved);

    // Around a circle, the result has to be reachable from the start, not
    // merely near it — and it is the polygon the radius bounds, not the
    // point: upstream places the point anywhere inside a polygon the search
    // reached, and a polygon reached at the edge of the circle extends past
    // it. What is checkable is that the point is on the polygon it names.
    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    fixed_random_state = 7;
    const around = try world.query.findRandomPointAroundCircle(
        near.ref.?,
        near.point,
        3.0,
        &world.filter,
        fixedRandom,
    );
    try std.testing.expect(around.ref != 0);
    const on_poly = try world.query.closestPointOnPoly(around.ref, around.point);
    for (around.point, on_poly.point) |want, got| {
        try std.testing.expectApproxEqAbs(want, got, 1e-3);
    }
    // The polygon is genuinely close, even where the point on it is not.
    const poly_centre = try world.query.closestPointOnPoly(around.ref, near.point);
    try std.testing.expect(zrecast.vec.dist2D(near.point, poly_centre.point) <= 3.0 + 1e-3);
}

test "every polygon a box returns overlaps the box" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const center = zrecast.Vec3{ fixture.start[0], 0.2, fixture.start[2] };
    const extents = zrecast.Vec3{ 3, 2, 3 };
    var refs: [128]zrecast.PolyRef = undefined;
    const found = try world.query.queryPolygons(center, extents, &world.filter, &refs);
    try std.testing.expect(found.len > 0);
    try std.testing.expect(!found.truncated);

    const box_min = zrecast.vec.sub(center, extents);
    const box_max = zrecast.vec.add(center, extents);
    const tile = try world.mesh.tileRefAtIndex(0);
    for (refs[0..found.len]) |ref| {
        // Reached by a different route: the polygon's own corners, read out of
        // the tile, must have bounds that meet the box Detour was given.
        const info = try world.mesh.polyInfo(ref);
        var lo = zrecast.Vec3{ std.math.floatMax(f32), std.math.floatMax(f32), std.math.floatMax(f32) };
        var hi = zrecast.Vec3{ -std.math.floatMax(f32), -std.math.floatMax(f32), -std.math.floatMax(f32) };
        var corner: [1][3]f32 = undefined;
        for (0..info.vert_count) |k| {
            try world.mesh.tileVerts(tile, info.verts[k], &corner);
            lo = zrecast.vec.min(lo, corner[0]);
            hi = zrecast.vec.max(hi, corner[0]);
        }
        try std.testing.expect(try zrecast.geom.overlapBounds(box_min, box_max, lo, hi));
    }

    // A buffer too small reports the truncation rather than a short answer.
    var one: [1]zrecast.PolyRef = undefined;
    const clipped = try world.query.queryPolygons(center, extents, &world.filter, &one);
    try std.testing.expect(clipped.truncated);
    try std.testing.expectEqual(@as(usize, 1), clipped.len);
}

test "the batched box query finds what the buffered one finds" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const center = zrecast.Vec3{ 0, 0.2, 0 };
    const extents = zrecast.Vec3{ 12, 4, 12 };
    var refs: [256]zrecast.PolyRef = undefined;
    const buffered = try world.query.queryPolygons(center, extents, &world.filter, &refs);
    try std.testing.expect(buffered.len > 0);
    try std.testing.expect(!buffered.truncated);

    const Collector = struct {
        var seen: [256]zrecast.PolyRef = undefined;
        var count: usize = 0;
        var batches: usize = 0;
        fn process(user: ?*anyopaque, batch: [*]const zrecast.PolyRef, n: i32) callconv(.c) void {
            _ = user;
            batches += 1;
            for (batch[0..@intCast(n)]) |r| {
                if (count < seen.len) {
                    seen[count] = r;
                    count += 1;
                }
            }
        }
    };
    Collector.count = 0;
    Collector.batches = 0;
    try world.query.queryPolygonsBatched(center, extents, &world.filter, .{
        .process = Collector.process,
    });

    try std.testing.expect(Collector.batches > 0);
    try std.testing.expectEqualSlices(
        zrecast.PolyRef,
        refs[0..buffered.len],
        Collector.seen[0..Collector.count],
    );
}

/// Whether a point is in a polygon or on its boundary.
///
/// `dtPointInPolygon` is a strict crossing test, so a point Detour placed
/// exactly on an edge — which is what closest-point and random-placement both
/// produce — reports as outside. The closed question is the one those answers
/// actually satisfy.
fn onClosedPolygon(point: zrecast.Vec3, corners: []const [3]f32) !bool {
    if (try zrecast.geom.pointInPolygon(point, corners)) return true;
    var edge_dist: [c.verts_per_polygon]f32 = undefined;
    var edge_t: [c.verts_per_polygon]f32 = undefined;
    _ = try zrecast.geom.distancePointToPolyEdges(
        point,
        corners,
        edge_dist[0..corners.len],
        edge_t[0..corners.len],
    );
    var nearest: f32 = std.math.floatMax(f32);
    for (edge_dist[0..corners.len]) |d| nearest = @min(nearest, d);
    return nearest < 1e-6;
}

test "the closest point on a polygon is on that polygon" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const ref = near.ref.?;

    var storage: [c.verts_per_polygon][3]f32 = undefined;
    const corners = try polygonCorners(world.mesh, ref, &storage);

    // A point already on it comes back unchanged and reported as over it.
    const on = try world.query.closestPointOnPoly(ref, near.point);
    try std.testing.expect(on.over_poly);
    for (near.point, on.point) |want, got| {
        try std.testing.expectApproxEqAbs(want, got, 1e-3);
    }

    // A point far outside is pulled onto the polygon — onto its boundary,
    // which dtPointInPolygon's strict crossing test reports as outside, so
    // what is checkable is that it lies on the closed polygon.
    const far = zrecast.Vec3{ fixture.island_x, 0, fixture.island_z };
    const pulled = try world.query.closestPointOnPoly(ref, far);
    try std.testing.expect(!pulled.over_poly);
    try std.testing.expect(try onClosedPolygon(pulled.point, corners));

    // The boundary variant lands on an edge, so its distance to the nearest
    // edge is zero.
    const boundary = try world.query.closestPointOnPolyBoundary(ref, far);
    var edge_dist: [c.verts_per_polygon]f32 = undefined;
    var edge_t: [c.verts_per_polygon]f32 = undefined;
    _ = try zrecast.geom.distancePointToPolyEdges(
        boundary,
        corners,
        edge_dist[0..corners.len],
        edge_t[0..corners.len],
    );
    var nearest: f32 = std.math.floatMax(f32);
    for (edge_dist[0..corners.len]) |d| nearest = @min(nearest, d);
    try std.testing.expect(nearest < 1e-3);

    // The height under a point over the polygon is the height the closest
    // point reports, reached two ways.
    const height = try world.query.polyHeight(ref, near.point);
    try std.testing.expectApproxEqAbs(on.point[1], height, 1e-3);
    // And a point nowhere near it has no height on it.
    try std.testing.expectError(error.InvalidArgument, world.query.polyHeight(ref, far));
}

test "the polygons around a circle form a tree the corridor walks back up" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const start = near.ref.?;

    var refs: [128]zrecast.PolyRef = undefined;
    var parents: [128]zrecast.PolyRef = undefined;
    var costs: [128]f32 = undefined;
    const reached = try world.query.findPolysAroundCircle(
        start,
        near.point,
        6.0,
        &world.filter,
        &refs,
        &parents,
        &costs,
    );
    try std.testing.expect(reached.len > 1);
    try std.testing.expect(!reached.truncated);

    // The search starts where it was told to, at no cost, with no parent.
    try std.testing.expectEqual(start, refs[0]);
    try std.testing.expectEqual(@as(zrecast.PolyRef, 0), parents[0]);
    try std.testing.expectEqual(@as(f32, 0), costs[0]);

    for (refs[0..reached.len], parents[0..reached.len], costs[0..reached.len], 0..) |ref, parent, cost, i| {
        try std.testing.expect(ref != 0);
        try std.testing.expect(cost >= 0);
        if (i == 0) continue;
        // Every other polygon was reached through one already in the list, so
        // the parents really do form a tree rooted at the start.
        var found_parent = false;
        for (refs[0..reached.len]) |candidate| {
            if (candidate == parent) found_parent = true;
        }
        try std.testing.expect(found_parent);
    }

    // The corridor the search leaves behind, back to one of the polygons it
    // reached, is linked end to end — checked against the tile's own link
    // array rather than against the search that produced it.
    const target = refs[reached.len - 1];
    var corridor: [128]zrecast.PolyRef = undefined;
    const path = try world.query.pathFromDijkstraSearch(target, &corridor);
    try std.testing.expect(path.len >= 1);
    try std.testing.expectEqual(start, corridor[0]);
    try std.testing.expectEqual(target, corridor[path.len - 1]);

    const tile = try world.mesh.tileRefAtIndex(0);
    const link_capacity: u32 = @intCast((try world.mesh.tileInfo(tile)).max_link_count);
    for (corridor[0 .. path.len - 1], corridor[1..path.len]) |from, to| {
        const info = try world.mesh.polyInfo(from);
        var at = info.first_link;
        var linked = false;
        var steps: u32 = 0;
        while (at != zrecast.null_link) : (steps += 1) {
            try std.testing.expect(steps <= link_capacity);
            const link = try world.mesh.tileLink(tile, at);
            if (link.ref == to) linked = true;
            at = link.next;
        }
        try std.testing.expect(linked);
    }

    // Every polygon the search closed is reported closed, and one it never
    // reached is not.
    try std.testing.expect(try world.query.isInClosedList(start));

    // The optional arrays are optional, and mismatched ones are refused.
    const counted = try world.query.findPolysAroundCircle(
        start,
        near.point,
        6.0,
        &world.filter,
        &refs,
        &.{},
        &.{},
    );
    try std.testing.expectEqual(reached.len, counted.len);
    try std.testing.expectError(error.InvalidArgument, world.query.findPolysAroundCircle(
        start,
        near.point,
        6.0,
        &world.filter,
        refs[0..8],
        parents[0..4],
        &.{},
    ));
}

test "the distance to a wall is the distance to the segments the wall is made of" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    const ref = near.ref.?;

    var verts: [16][2]zrecast.Vec3 = undefined;
    var neighbours: [16]zrecast.PolyRef = undefined;
    const walls = try world.query.polyWallSegments(ref, &world.filter, &verts, &neighbours);
    try std.testing.expect(walls.len > 0);
    try std.testing.expect(!walls.truncated);

    // The nearest wall within reach, and the nearest of those segments, are
    // the same distance away — two routes to one fact.
    const hit = try world.query.findDistanceToWall(ref, near.point, 20.0, &world.filter);
    try std.testing.expect(hit != null);

    var nearest_segment: f32 = std.math.floatMax(f32);
    for (verts[0..walls.len], neighbours[0..walls.len]) |segment, neighbour| {
        // A segment with a neighbour is not a wall; only the unlinked ones are.
        if (neighbour != 0) continue;
        const d = try zrecast.geom.distancePointToSegment2D(near.point, segment[0], segment[1]);
        nearest_segment = @min(nearest_segment, @sqrt(d.dist_sqr));
    }
    try std.testing.expect(nearest_segment < std.math.floatMax(f32));
    try std.testing.expectApproxEqAbs(nearest_segment, hit.?.distance, 1e-2);

    // The normal points away from the wall and is a unit vector.
    const n = hit.?.normal;
    try std.testing.expectApproxEqAbs(@as(f32, 1), zrecast.vec.len(n), 1e-3);

    // Nothing within range is an absent answer, not a hit at distance zero —
    // upstream reports success either way and normalises over an untouched
    // position.
    const nothing = try world.query.findDistanceToWall(ref, near.point, 0.001, &world.filter);
    try std.testing.expect(nothing == null);
}

test "an option a query takes changes what it answers" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const ends = try Endpoints.find(world);

    // Costs are accumulated only when asked for; without the option upstream
    // reports 0 rather than "not computed".
    const plain = try world.query.raycast(
        ends.start.ref.?,
        ends.start.point,
        fixture.goal,
        &world.filter,
        .{},
        null,
        null,
    );
    try std.testing.expectEqual(@as(f32, 0), plain.hit.path_cost);
    const costed = try world.query.raycast(
        ends.start.ref.?,
        ends.start.point,
        fixture.goal,
        &world.filter,
        .{ .use_costs = true },
        null,
        null,
    );
    try std.testing.expect(costed.hit.path_cost > 0);
    // The hit itself is the same either way.
    try std.testing.expectApproxEqAbs(plain.hit.t, costed.hit.t, 1e-6);
    try std.testing.expect(costed.hit.hit_edge_index != null);

    // Every crossing produces at least as many corners as only the turns do.
    var corridor: [256]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        &corridor,
    );
    var turns: [64][3]f32 = undefined;
    var crossings: [64][3]f32 = undefined;
    const by_turn = try world.query.findStraightPath(
        ends.start.point,
        ends.goal.point,
        corridor[0..path.len],
        .{},
        &turns,
        null,
        null,
    );
    const by_crossing = try world.query.findStraightPath(
        ends.start.point,
        ends.goal.point,
        corridor[0..path.len],
        .{ .all_crossings = true },
        &crossings,
        null,
        null,
    );
    try std.testing.expect(by_crossing.len >= by_turn.len);
    try std.testing.expect(by_crossing.len > 2);
}

test "the node pool remembers the corridor the search walked" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();
    const ends = try Endpoints.find(world);

    const before = try world.query.nodePoolInfo();
    try std.testing.expect(before.max_nodes > 0);

    var corridor: [256]zrecast.PolyRef = undefined;
    const path = try world.query.findPath(
        ends.start.ref.?,
        ends.goal.ref.?,
        ends.start.point,
        ends.goal.point,
        &world.filter,
        &corridor,
    );
    try std.testing.expect(path.len > 2);

    const after = try world.query.nodePoolInfo();
    try std.testing.expect(after.node_count > 0);
    try std.testing.expect(after.node_count <= after.max_nodes);
    try std.testing.expectEqual(before.max_nodes, after.max_nodes);

    // Walking the parent chain from the end node has to reproduce the corridor
    // findPath returned, backwards. That is the search's own bookkeeping
    // checked against the answer it handed out.
    const end_node = (try world.query.findNode(ends.goal.ref.?, 0)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(end_node.flags.closed);
    try std.testing.expectEqual(ends.goal.ref.?, end_node.ref);

    var walked: [256]zrecast.PolyRef = undefined;
    var n: usize = 0;
    var node: ?zrecast.Node = end_node;
    while (node) |current| {
        try std.testing.expect(n < walked.len);
        walked[n] = current.ref;
        n += 1;
        node = if (current.parent_index) |idx| try world.query.nodeAt(idx) else null;
    }
    try std.testing.expectEqual(path.len, n);
    for (corridor[0..path.len], 0..) |ref, i| {
        try std.testing.expectEqual(ref, walked[n - 1 - i]);
    }

    // A polygon no search reached has no node; index 0 is not a node either.
    try std.testing.expect((try world.query.nodeAt(0)) == null);
    var states: [4]zrecast.Node = undefined;
    const found = try world.query.findNodes(ends.goal.ref.?, &states);
    try std.testing.expect(found >= 1);
    try std.testing.expectEqual(ends.goal.ref.?, states[0].ref);
}

test "a polygon reference splits into the fields packed into it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const tile = try world.mesh.tileRefAtIndex(0);
    const info = try world.mesh.tileInfo(tile);

    // Every polygon of the tile, split and rebuilt.
    var index: u32 = 0;
    while (index < @as(u32, @intCast(info.poly_count))) : (index += 1) {
        const ref = try world.mesh.tilePolyRef(tile, index);
        const parts = try zrecast.decodePolyRef(world.mesh, ref);
        try std.testing.expectEqual(index, parts.poly);
        const rebuilt = try zrecast.encodePolyRef(world.mesh, parts.salt, parts.tile, parts.poly);
        try std.testing.expectEqual(ref, rebuilt);
        try std.testing.expect(try world.query.isValidPolyRef(ref, null));
    }

    // A field wider than this navmesh's own layout is refused, where upstream
    // would shift it away and mint a reference to something else.
    try std.testing.expectError(
        error.InvalidArgument,
        zrecast.encodePolyRef(world.mesh, 0, 0, 0xffff),
    );

    // The query and the navmesh answer the same validity question differently:
    // with a filter that admits nothing, a live polygon is not valid.
    const near = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    var blind = zrecast.defaultFilter();
    blind.include_flags = 0;
    try std.testing.expect(try world.query.isValidPolyRef(near.ref.?, null));
    try std.testing.expect(!try world.query.isValidPolyRef(near.ref.?, &blind));

    // And the navmesh the query hands back is the one it was made from.
    const attached = try world.query.attachedNavMesh();
    try std.testing.expectEqual(world.mesh.polyCount(), attached.polyCount());
}

//=============================================================================
// Out of memory
//=============================================================================

/// Serialise a navmesh, check the image, and load it back. Every allocation
/// inside is either zrecast's own or one upstream genuinely checks
/// (`dtAllocNavMesh`, and the two arrays in `dtNavMesh::init`, which report
/// `DT_OUT_OF_MEMORY`).
fn runOwnedArc(mesh: zrecast.NavMesh) !void {
    const image = try mesh.serialize();
    defer image.deinit();
    try zrecast.validate(image.bytes);

    const loaded = try zrecast.NavMesh.initFromBytes(image.bytes);
    defer loaded.deinit();

    if (loaded.polyCount() == 0) return error.EmptyReload;
}

test "a failure at any allocation zrecast owns is reported and leaks nothing" {
    // What makes the OutOfMemory paths more than decorative. The sweep runs the
    // arc once per allocation site, failing that site and every one after it,
    // and requires three things each time: an error rather than a success, the
    // error the API documents, and every byte taken given back. It is the test
    // that exercises the multi-step rollback inside zrcNavMeshDeserialize,
    // where a buffer, a handle and a dtNavMesh are acquired in turn and any of
    // the three can fail.
    //
    // The scope is narrow on purpose, and the cause is upstream's: **neither
    // Recast nor Detour survives an allocation failure.** Three places, all
    // unfixable from outside:
    //
    //   * `rcVectorBase::push_back` calls `allocate_and_copy`, which returns
    //     null when `rcAlloc` fails, then runs `construct(data + m_size, value)`
    //     on it unconditionally (RecastAlloc.h:214) — a placement-new through a
    //     null pointer. So the bake happens before the sweep.
    //   * `createBVTree` allocates its item array and indexes it without a
    //     check (DetourNavMeshBuilder.cpp:175), so `dtCreateNavMeshData` — and
    //     with it `NavMesh.initFromPolyMesh` — is outside the sweep too.
    //   * `dtNodePool` and `dtNodeQueue` check their allocations with
    //     `dtAssert` and then memset the result, which under NDEBUG is the same
    //     null dereference. So is building a `NavMeshQuery`.
    //
    // None of this is silenced; all of it is recorded in UPSTREAM.md. What
    // remains below is the part zrecast is actually responsible for, and it is
    // held to the full standard.
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const poly = try bakeFixture(null);
    defer poly.deinit();
    const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
    // Both are freed through `gpa`, which the loop restores every iteration.
    defer mesh.deinit();

    // The sweep's own allocations are backed by an arena rather than by the
    // testing allocator, because exactly one index leaks by design (see the
    // tally at the end) and the testing allocator would report that upstream
    // limitation as a defect in this test. The arena reclaims it wholesale at
    // scope exit; `failing`'s byte counters still catch any leak from zrecast.
    var arena = std.heap.ArenaAllocator.init(gpa);
    defer arena.deinit();
    const sweep_gpa = arena.allocator();

    const total = count: {
        var counting = std.testing.FailingAllocator.init(sweep_gpa, .{});
        try zrecast.setAllocator(counting.allocator());
        defer zrecast.setAllocator(gpa) catch {};
        try runOwnedArc(mesh);
        break :count counting.alloc_index;
    };
    try std.testing.expect(total > 0);

    var leaking_indices: usize = 0;
    var index: usize = 0;
    while (index < total) : (index += 1) {
        var failing = std.testing.FailingAllocator.init(sweep_gpa, .{ .fail_index = index });
        try zrecast.setAllocator(failing.allocator());
        const outcome = runOwnedArc(mesh);
        try zrecast.setAllocator(gpa);

        if (outcome) |_| {
            std.debug.print(
                "allocation {d}/{d} failed but the arc reported success\n",
                .{ index, total },
            );
            return error.SwallowedOutOfMemory;
        } else |e| switch (e) {
            error.OutOfMemory => {},
            else => {
                std.debug.print("allocation {d}/{d} gave {t}\n", .{ index, total, e });
                return e;
            },
        }

        if (failing.allocated_bytes != failing.freed_bytes) {
            leaking_indices += 1;
            // The first one is upstream's and expected; anything beyond it is
            // a regression here, so only then is the detail worth printing.
            if (leaking_indices > 1) {
                std.debug.print(
                    "allocation {d}/{d} leaked: {d} allocated, {d} freed\n",
                    .{ index, total, failing.allocated_bytes, failing.freed_bytes },
                );
            }
        }
    }

    // Exactly one index may leak, and it is upstream's, pinned here so that a
    // regression in zrecast's own rollback still shows up as a second one.
    //
    // dtNavMesh::init allocates m_tiles and then m_posLookup, and memsets
    // neither until both have succeeded. When the second fails, the first
    // cannot be reclaimed: running ~dtNavMesh would read uninitialised tile
    // flags and free the garbage pointers behind them, so
    // ffi/zrecast_navmesh.cpp releases the object without its destructor and
    // accepts the one array. See UPSTREAM.md.
    try std.testing.expectEqual(@as(usize, 1), leaking_indices);
}

test "a query that reaches no polygon reports none" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    // The tile's own minimum corner, which is where the defect this guards
    // against lives: every query box is clamped to the tile's bounds before
    // being quantised, so a box at or below that corner quantises its minimum
    // to zero on all three axes — and a bounding-volume node of all zeroes
    // overlaps exactly that. Upstream reserves one such node past the end of
    // the tree and walks into it, which made this call answer polygon 0, whose
    // nearest point is metres outside the half-extents asked for.
    const bounds = try world.mesh.bounds();
    const corner = bounds[0];
    const at_corner = try world.query.findNearestPoly(corner, .{ 0.05, 0.05, 0.05 }, &world.filter);
    try std.testing.expect(at_corner.ref == null);

    // Well outside the mesh entirely, which never reached a tile at all and so
    // was never affected — kept so the test above cannot pass for that reason.
    const far = try world.query.findNearestPoly(
        .{ -1000, -1000, -1000 },
        .{ 1, 1, 1 },
        &world.filter,
    );
    try std.testing.expect(far.ref == null);

    // And the search still finds what it should.
    const on_ground = try world.query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(on_ground.ref != null);
}

//=============================================================================
// Crowds
//=============================================================================

fn crowdAgentParams(radius: f32) zrecast.CrowdAgentParams {
    return .{
        .radius = radius,
        .height = 2.0,
        .max_acceleration = 8.0,
        .max_speed = 3.5,
        .collision_query_range = radius * 12.0,
        .path_optimization_range = radius * 30.0,
        .separation_weight = 2.0,
        .update_flags = zrecast.UpdateFlags.anticipate_turns |
            zrecast.UpdateFlags.obstacle_avoidance |
            zrecast.UpdateFlags.separation |
            zrecast.UpdateFlags.optimize_vis |
            zrecast.UpdateFlags.optimize_topo,
        .obstacle_avoidance_type = 0,
        .query_filter_type = 0,
        .user_data = null,
    };
}

test "an agent reference outlives its slot only until the slot is reused" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const crowd = try zrecast.Crowd.init(world.mesh, 4, 0.6);
    defer crowd.deinit();

    const params = crowdAgentParams(0.6);
    try std.testing.expectEqual(@as(u32, 4), try crowd.agentCapacity());
    try std.testing.expectEqual(@as(u32, 0), try crowd.activeAgentCount());

    const first = try crowd.addAgent(fixture.start, params);
    const first_ever = first;
    try std.testing.expect(first != 0);
    try std.testing.expectEqual(@as(u32, 1), try crowd.activeAgentCount());
    try std.testing.expectEqual(
        zrecast.CrowdAgentState.walking,
        (try crowd.agent(first)).state,
    );

    // Upstream's removeAgent only clears a flag, and addAgent hands back the
    // lowest free slot, so the same index answers for whoever took it next.
    // A reference carries a serial the crowd never reissues.
    try crowd.removeAgent(first);
    try std.testing.expectError(zrecast.Error.NotFound, crowd.agent(first));
    try std.testing.expectError(zrecast.Error.NotFound, crowd.removeAgent(first));

    const second = try crowd.addAgent(fixture.start, params);
    try std.testing.expect(second != first);
    _ = try crowd.agent(second);
    try std.testing.expectError(zrecast.Error.NotFound, crowd.agent(first));
    try std.testing.expectError(zrecast.Error.NotFound, crowd.setAgentParams(first, params));

    // The slot is the same one, which is what makes the serial load-bearing.
    try std.testing.expectEqual(second, try crowd.agentRefAt(0));
    try std.testing.expectEqual(@as(zrecast.AgentRef, 0), try crowd.agentRefAt(1));
    try std.testing.expectError(zrecast.Error.InvalidArgument, crowd.agentRefAt(4));

    // Re-initialising purges upstream, so every reference minted before it
    // must stop resolving even once the slot is filled again.
    try crowd.reinit(world.mesh, 4, 0.6);
    try std.testing.expectEqual(@as(u32, 0), try crowd.activeAgentCount());
    try std.testing.expectError(zrecast.Error.NotFound, crowd.agent(second));
    const third = try crowd.addAgent(fixture.start, params);
    try std.testing.expect(third != second);
    try std.testing.expectError(zrecast.Error.NotFound, crowd.agent(second));
    // And the very first reference, whose serial the counter would hand out
    // again if a re-initialisation reset it. This is the reference the reset
    // would collide with, not `second`.
    try std.testing.expect(third != first_ever);
    try std.testing.expectError(zrecast.Error.NotFound, crowd.agent(first_ever));

    // A full crowd says so rather than answering with a slot it does not have.
    for (0..3) |_| _ = try crowd.addAgent(fixture.start, params);
    try std.testing.expectError(
        zrecast.Error.CrowdFull,
        crowd.addAgent(fixture.start, params),
    );

    // The total comes back even when the buffer cannot hold it.
    var refs: [4]zrecast.AgentRef = undefined;
    try std.testing.expectEqual(@as(usize, 4), try crowd.activeAgents(&refs));
    try std.testing.expectEqual(@as(usize, 4), try crowd.activeAgents(refs[0..2]));
    try std.testing.expectEqual(@as(usize, 4), try crowd.activeAgents(&.{}));
}

test "an agent's parameters are checked where upstream indexes unchecked" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const crowd = try zrecast.Crowd.init(world.mesh, 4, 0.6);
    defer crowd.deinit();

    // The two fields upstream reads as array indices at fifteen call sites
    // between them, bounding neither. Both are unsigned char, so every value
    // from the table's length to 255 is a read past its end, every frame.
    var params = crowdAgentParams(0.6);
    params.query_filter_type = @intCast(zrecast.crowd_max_filters);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );
    params.query_filter_type = 255;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );
    params = crowdAgentParams(0.6);
    params.obstacle_avoidance_type = @intCast(zrecast.crowd_max_avoidance_params);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );

    // A zero maximum speed gives an infinite off-mesh traversal budget, and a
    // zero collision range is a reciprocal upstream takes without a guard.
    params = crowdAgentParams(0.6);
    params.max_speed = 0;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );
    params = crowdAgentParams(0.6);
    params.collision_query_range = 0;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );

    // A radius the crowd was not sized for, and one no cell index could name.
    params = crowdAgentParams(5.0);
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );
    params = crowdAgentParams(std.math.nan(f32));
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.addAgent(fixture.start, params),
    );

    // A frame is a positive number. Upstream multiplies acceleration by it and
    // asserts nothing, so a NaN frame poisons every position from then on.
    _ = try crowd.addAgent(fixture.start, crowdAgentParams(0.6));
    try std.testing.expectError(zrecast.Error.InvalidArgument, crowd.update(0));
    try std.testing.expectError(zrecast.Error.InvalidArgument, crowd.update(-0.1));
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.update(std.math.nan(f32)),
    );
    try crowd.update(1.0 / 60.0);

    // Every shared table is bounded here, where upstream answers with null.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.setFilter(@intCast(zrecast.crowd_max_filters), zrecast.defaultFilter()),
    );
    try crowd.setFilter(3, zrecast.defaultFilter());
    var avoidance = zrecast.avoidanceParamsDefault();
    try std.testing.expect(avoidance.grid_size >= 2);
    try crowd.setAvoidanceParams(5, avoidance);
    try std.testing.expectEqual(avoidance, try crowd.avoidanceParams(5));
    avoidance.horiz_time = 0;
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.setAvoidanceParams(5, avoidance),
    );
}

/// Fails when the step from `prev` to `now` passed through the fixture's wall.
///
/// The wall lies along z = 0 from the -x edge to `wall_end_x`. Crossing z = 0
/// is how an agent reaches the goal, so the claim is about *where* it crosses:
/// only past the wall's far end is legitimate. A snapshot cannot say that — an
/// agent that has already rounded the wall is back at a small x on the far
/// side — so the crossing point is interpolated from the step.
fn expectNoTunnel(prev: [3]f32, now: [3]f32) !void {
    if ((prev[2] < 0) == (now[2] < 0)) return;
    const dz = now[2] - prev[2];
    const t = if (dz == 0) 0 else (0 - prev[2]) / dz;
    const x = prev[0] + (now[0] - prev[0]) * t;
    try std.testing.expect(x > fixture.wall_end_x - 0.5);
}

test "two agents on the same spot separate, and neither crosses the wall" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const crowd = try zrecast.Crowd.init(world.mesh, 8, 0.6);
    defer crowd.deinit();

    const goal = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try std.testing.expect(goal.ref != null);

    const params = crowdAgentParams(0.6);
    const a_ref = try crowd.addAgent(fixture.start, params);
    const b_ref = try crowd.addAgent(fixture.start, params);
    try crowd.requestMoveTarget(a_ref, goal.ref.?, goal.point);
    try crowd.requestMoveTarget(b_ref, goal.ref.?, goal.point);

    // A target of no polygon is refused; clearing one is a separate call.
    try std.testing.expectError(
        zrecast.Error.InvalidArgument,
        crowd.requestMoveTarget(a_ref, 0, goal.point),
    );

    var a = try crowd.agent(a_ref);
    var b = try crowd.agent(b_ref);
    var a_prev = a.position;
    var b_prev = b.position;
    var frame: u32 = 0;
    while (frame < 600) : (frame += 1) {
        try crowd.update(1.0 / 60.0);
        a = try crowd.agent(a_ref);
        b = try crowd.agent(b_ref);
        try expectNoTunnel(a_prev, a.position);
        try expectNoTunnel(b_prev, b.position);
        a_prev = a.position;
        b_prev = b.position;
        for ([_]zrecast.CrowdAgent{ a, b }) |agent| {
            // Nothing leaves the ground plane, and nothing goes non-finite:
            // a poisoned position would be an infinity or a NaN that every
            // later frame carries.
            try std.testing.expect(@abs(agent.position[0]) <= fixture.ground_extent + 1);
            try std.testing.expect(@abs(agent.position[2]) <= fixture.ground_extent + 1);
            try std.testing.expect(std.math.isFinite(agent.position[0]));
            try std.testing.expect(std.math.isFinite(agent.position[1]));
            try std.testing.expect(std.math.isFinite(agent.position[2]));
        }
    }

    // Started on top of each other, they end at least a radius apart.
    const dx = a.position[0] - b.position[0];
    const dz = a.position[2] - b.position[2];
    try std.testing.expect(dx * dx + dz * dz > params.radius * params.radius);

    // At least one rounded the wall, so the crowd actually pathed rather than
    // shuffling in place.
    try std.testing.expect(a.position[2] > 0 or b.position[2] > 0);
    try std.testing.expect(try crowd.velocitySampleCount() > 0);

    // The corners and the corridor are readable, and the corridor's first
    // polygon is the one the agent is standing in.
    var corners: [zrecast.crowd_max_corners]zrecast.CrowdCorner = undefined;
    try crowd.agentCorners(a_ref, 0, corners[0..@intCast(a.corner_count)]);
    var neighbours: [zrecast.crowd_max_neighbours]zrecast.CrowdNeighbour = undefined;
    try crowd.agentNeighbours(a_ref, 0, neighbours[0..@intCast(a.neighbour_count)]);

    const corridor = try crowd.agentCorridor(a_ref);
    try std.testing.expect(corridor.path_count > 0);
    try std.testing.expect(corridor.first_poly != 0);
    const under = try world.query.findNearestPoly(a.position, search_extents, &world.filter);
    try std.testing.expectEqual(under.ref.?, corridor.first_poly);

    // The boundary was collected, so its centre is no longer the sentinel an
    // untouched one carries.
    const centre = try crowd.agentBoundaryCenter(a_ref);
    try std.testing.expect(centre[0] != std.math.floatMax(f32));

    // Clearing the target stops the agent where it is.
    try crowd.resetMoveTarget(a_ref);
    try std.testing.expectEqual(
        zrecast.CrowdTargetState.none,
        (try crowd.agent(a_ref)).target_state,
    );
}

test "a debug view that outlived its agent does not stop the crowd" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const crowd = try zrecast.Crowd.init(world.mesh, 4, 0.6);
    defer crowd.deinit();

    const watched = try crowd.addAgent(fixture.start, crowdAgentParams(0.6));
    const other = try crowd.addAgent(fixture.start, crowdAgentParams(0.6));

    const samples = try zrecast.AvoidanceDebug.init(256);
    defer samples.deinit();
    var debug = zrecast.CrowdAgentDebug{ .agent = watched, .samples = samples };

    const goal = try world.query.findNearestPoly(fixture.goal, search_extents, &world.filter);
    try crowd.requestMoveTarget(other, goal.ref.?, goal.point);
    try crowd.updateDebug(1.0 / 60.0, &debug);

    // The watched agent goes; the reference in the debug struct does not.
    try crowd.removeAgent(watched);

    // Upstream's own idx is an index into a snapshot that shifts as agents
    // come and go, so a host keeping a reference across frames is the
    // ordinary case rather than a misuse. The frame has to run anyway.
    const before = try crowd.agent(other);
    var frame: u32 = 0;
    while (frame < 120) : (frame += 1) {
        try crowd.updateDebug(1.0 / 60.0, &debug);
    }
    const after = try crowd.agent(other);

    // The surviving agent covered ground, which is what says the updates ran
    // rather than returning early on the reference the debug struct still
    // carries.
    const dx = after.position[0] - before.position[0];
    const dz = after.position[2] - before.position[2];
    try std.testing.expect(dx * dx + dz * dz > 0.25);
    try std.testing.expect(try crowd.velocitySampleCount() > 0);
}

test "an agent removed mid-jump does not carry the next agent with it" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const links = [_]zrecast.OffMeshConnection{wallJump(true)};
    const world = try LinkedWorld.init(.{ .connections = &links });
    defer world.deinit();

    const crowd = try zrecast.Crowd.init(world.mesh, 4, 0.6);
    defer crowd.deinit();

    // Onto the connection's near side, heading for its far side.
    const near = [3]f32{ -8, 0, -2 };
    const far = [3]f32{ -8, 0, 2 };
    const jumper = try crowd.addAgent(near, crowdAgentParams(0.6));
    const target = try world.query.findNearestPoly(far, search_extents, &world.filter);
    try std.testing.expect(target.ref != null);
    try crowd.requestMoveTarget(jumper, target.ref.?, target.point);

    // Drive it until it is part way across.
    var crossing = false;
    var frame: u32 = 0;
    while (frame < 600) : (frame += 1) {
        try crowd.update(1.0 / 60.0);
        if ((try crowd.agent(jumper)).state == .offmesh) {
            crossing = true;
            break;
        }
    }
    try std.testing.expect(crossing);

    // Removed mid-crossing. Upstream's own removeAgent frees the slot here and
    // leaves the traversal live in a parallel array update() walks by slot, so
    // the next agent added would resume it — its position overwritten every
    // frame by a lerp between the two endpoints above.
    try crowd.removeAgent(jumper);
    try std.testing.expectError(zrecast.Error.NotFound, crowd.agent(jumper));

    const settler = try crowd.addAgent(fixture.start, crowdAgentParams(0.6));
    const placed = (try crowd.agent(settler)).position;
    frame = 0;
    while (frame < 120) : (frame += 1) {
        try crowd.update(1.0 / 60.0);
        const now = (try crowd.agent(settler)).position;
        // It has no target, so it stays where it was put. The failure this
        // guards against teleports it four metres, onto the connection.
        const dx = now[0] - placed[0];
        const dz = now[2] - placed[2];
        try std.testing.expect(dx * dx + dz * dz < 1.0);
    }

    // And the slot the jumper held comes back once its crossing finished, so
    // holding it is a delay rather than a leak.
    try std.testing.expectEqual(@as(u32, 1), try crowd.activeAgentCount());
    var refs: [4]zrecast.AgentRef = undefined;
    try std.testing.expectEqual(@as(usize, 1), try crowd.activeAgents(&refs));
    for (0..3) |_| _ = try crowd.addAgent(fixture.start, crowdAgentParams(0.6));
    try std.testing.expectEqual(@as(u32, 4), try crowd.activeAgentCount());
}

test "a crowd's borrowed pieces are the ones it is driving" {
    const gpa = std.testing.allocator;
    try zrecast.setAllocator(gpa);
    defer zrecast.resetAllocator();

    const world = try World.init();
    defer world.deinit();

    const crowd = try zrecast.Crowd.init(world.mesh, 4, 0.6);
    defer crowd.deinit();
    _ = try crowd.addAgent(fixture.start, crowdAgentParams(0.6));
    try crowd.update(1.0 / 60.0);

    // The grid is rebuilt from every agent at the start of each update, so
    // after one it holds the agent that was there.
    const grid = try crowd.grid();
    const bounds = try grid.bounds();
    try std.testing.expect(bounds[0] <= bounds[2]);
    try std.testing.expect(try grid.cellSize() > 0);

    // The query the crowd plans with is against the crowd's own navmesh.
    const query = try crowd.navMeshQuery();
    const near = try query.findNearestPoly(fixture.start, search_extents, &world.filter);
    try std.testing.expect(near.ref != null);

    // And the placement box is derived from the radius the crowd was made for.
    const extents = try crowd.queryHalfExtents();
    try std.testing.expect(extents[0] > 0 and extents[1] > 0 and extents[2] > 0);

    _ = try crowd.pathQueue();
}
