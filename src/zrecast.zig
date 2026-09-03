//! zrecast — Zig bindings for Recast (navmesh baking) and Detour (navmesh
//! queries).
//!
//! The package has two halves, and the split is the point:
//!
//!   * `bake` is Recast, a **build-time** tool. It turns a triangle soup into a
//!     polygon mesh. Slow, allocation-heavy, and its output is an artefact.
//!   * `navmesh` and `query` are Detour, the **runtime**. They load that
//!     artefact and answer questions about it, with no baking involved.
//!
//! A cook uses the first and serialises the result; a game loads the bytes and
//! only ever touches the second.
//!
//! ```zig
//! // Cook:
//! try zrecast.setAllocator(gpa);
//! defer zrecast.resetAllocator();
//!
//! const poly = try zrecast.PolyMesh.bake(zrecast.defaultConfig(), .{
//!     .verts = level_verts,
//!     .tris = level_indices,
//! }, null, null);
//! defer poly.deinit();
//!
//! const mesh = try zrecast.NavMesh.initFromPolyMesh(poly, null);
//! defer mesh.deinit();
//!
//! const image = try mesh.serialize();
//! defer image.deinit();
//!
//! // Runtime:
//! const loaded = try zrecast.NavMesh.initFromBytes(image.bytes);
//! defer loaded.deinit();
//!
//! const query = try zrecast.NavMeshQuery.init(loaded, 2048);
//! defer query.deinit();
//!
//! const filter = zrecast.defaultFilter();
//! const near = try query.findNearestPoly(pos, .{ 2, 4, 2 }, &filter);
//! ```

const std = @import("std");

pub const c = @import("c.zig");

const error_mod = @import("error.zig");
const memory_mod = @import("memory.zig");
const bake_mod = @import("bake.zig");
const navmesh_mod = @import("navmesh.zig");
const query_mod = @import("query.zig");
const vec_mod = @import("vec.zig");
const geom_mod = @import("geom.zig");
const asserts_mod = @import("asserts.zig");
const pipeline_mod = @import("pipeline.zig");
const tilecache_mod = @import("tilecache.zig");
const crowd_mod = @import("crowd.zig");

//=============================================================================
// Public surface
//=============================================================================

pub const Error = error_mod.Error;
pub const resultName = error_mod.name;

pub const setAllocator = memory_mod.setAllocator;
pub const resetAllocator = memory_mod.resetAllocator;
pub const alloc = memory_mod.alloc;
pub const free = memory_mod.free;
pub const AllocHint = memory_mod.AllocHint;

// The assertion seam, alongside the allocator seam above.
pub const AssertFailure = asserts_mod.Failure;
pub const AssertHandler = asserts_mod.Handler;
pub const assertsEnabled = asserts_mod.assertsEnabled;
pub const setAssertHandler = asserts_mod.setHandler;
pub const assertHandler = asserts_mod.handler;

// Build time.
pub const Config = bake_mod.Config;
pub const Partition = bake_mod.Partition;
pub const TriMesh = bake_mod.TriMesh;
pub const PolyMesh = bake_mod.PolyMesh;
pub const PolyMeshInfo = bake_mod.PolyMeshInfo;
pub const defaultConfig = bake_mod.defaultConfig;
pub const logText = bake_mod.logText;
pub const BuildCells = bake_mod.BuildCells;
pub const buildCells = bake_mod.buildCells;

// The seam.
pub const NavMesh = navmesh_mod.NavMesh;
pub const Serialized = navmesh_mod.Serialized;
pub const validate = navmesh_mod.validate;
pub const swapImageEndian = navmesh_mod.swapImageEndian;
pub const TileRef = navmesh_mod.TileRef;
pub const TileGrid = bake_mod.TileGrid;
pub const tileGrid = bake_mod.tileGrid;
pub const VolumeShape = bake_mod.VolumeShape;
pub const AreaVolume = bake_mod.AreaVolume;
pub const AreaAuthoring = bake_mod.AreaAuthoring;
pub const convexVolume = bake_mod.convexVolume;
pub const boxVolume = bake_mod.boxVolume;
pub const cylinderVolume = bake_mod.cylinderVolume;
pub const buildTileData = navmesh_mod.buildTileData;
pub const dataVersion = navmesh_mod.dataVersion;
pub const PolyType = navmesh_mod.PolyType;
pub const OffMeshConnection = navmesh_mod.OffMeshConnection;
pub const TileAuthoring = navmesh_mod.TileAuthoring;
pub const NavMeshParams = navmesh_mod.NavMeshParams;
pub const TileInfo = navmesh_mod.TileInfo;
pub const PolyInfo = navmesh_mod.PolyInfo;
pub const Link = navmesh_mod.Link;
pub const BvNode = navmesh_mod.BvNode;
pub const TileLoc = navmesh_mod.TileLoc;
pub const oppositeTileSide = navmesh_mod.oppositeTileSide;
pub const Range = navmesh_mod.Range;
pub const TileLayout = navmesh_mod.TileLayout;
pub const tileLayout = navmesh_mod.tileLayout;
pub const ext_link = c.ext_link;
pub const null_link = c.null_link;
pub const tile_free_data = c.tile_free_data;
pub const detail_edge_boundary = navmesh_mod.detail_edge_boundary;
pub const detailTriEdgeFlags = navmesh_mod.detailTriEdgeFlags;

// Runtime.
pub const PolyRef = query_mod.PolyRef;
pub const Filter = query_mod.Filter;
pub const NavMeshQuery = query_mod.NavMeshQuery;
pub const NearestPoly = query_mod.NearestPoly;
pub const PathResult = query_mod.PathResult;
pub const RaycastHit = query_mod.RaycastHit;
pub const MoveResult = query_mod.MoveResult;
pub const RaycastResult = query_mod.RaycastResult;
pub const defaultFilter = query_mod.defaultFilter;
pub const straightpath_start = query_mod.straightpath_start;
pub const straightpath_end = query_mod.straightpath_end;
pub const straightpath_offmesh_connection = query_mod.straightpath_offmesh_connection;
pub const RaycastOptions = query_mod.RaycastOptions;
pub const StraightPathOptions = query_mod.StraightPathOptions;
pub const FindPathOptions = query_mod.FindPathOptions;
pub const SliceProgress = query_mod.SliceProgress;
pub const RandomSource = query_mod.RandomSource;
pub const RandomPoint = query_mod.RandomPoint;
pub const PolyList = query_mod.PolyList;
pub const PolySink = query_mod.PolySink;
pub const ClosestPoint = query_mod.ClosestPoint;
pub const ReachedPolys = query_mod.ReachedPolys;
pub const WallSegments = query_mod.WallSegments;
pub const WallHit = query_mod.WallHit;
pub const DecodedPolyRef = query_mod.DecodedPolyRef;
pub const NodePoolInfo = query_mod.NodePoolInfo;
pub const NodeFlags = query_mod.NodeFlags;
pub const Node = query_mod.Node;
pub const decodePolyRef = query_mod.decodePolyRef;
pub const encodePolyRef = query_mod.encodePolyRef;

// The staged Recast pipeline.
pub const LogCategory = pipeline_mod.LogCategory;
pub const TimerLabel = pipeline_mod.TimerLabel;
pub const BuildContext = pipeline_mod.BuildContext;
pub const log = pipeline_mod.log;
pub const resetLog = pipeline_mod.resetLog;
pub const resetTimers = pipeline_mod.resetTimers;
pub const startTimer = pipeline_mod.startTimer;
pub const stopTimer = pipeline_mod.stopTimer;
pub const accumulatedTime = pipeline_mod.accumulatedTime;
pub const GridSize = pipeline_mod.GridSize;
pub const calcBounds = pipeline_mod.calcBounds;
pub const calcGridSize = pipeline_mod.calcGridSize;
pub const markWalkableTriangles = pipeline_mod.markWalkableTriangles;
pub const clearUnwalkableTriangles = pipeline_mod.clearUnwalkableTriangles;
pub const Span = pipeline_mod.Span;
pub const HeightfieldInfo = pipeline_mod.HeightfieldInfo;
pub const HeightfieldStorage = pipeline_mod.HeightfieldStorage;
pub const Heightfield = pipeline_mod.Heightfield;
pub const CompactCell = pipeline_mod.CompactCell;
pub const CompactSpan = pipeline_mod.CompactSpan;
pub const CompactHeightfieldInfo = pipeline_mod.CompactHeightfieldInfo;
pub const CompactHeightfield = pipeline_mod.CompactHeightfield;
pub const ContourSetInfo = pipeline_mod.ContourSetInfo;
pub const ContourInfo = pipeline_mod.ContourInfo;
pub const ContourVertex = pipeline_mod.ContourVertex;
pub const ContourOptions = pipeline_mod.ContourOptions;
pub const ContourSet = pipeline_mod.ContourSet;
pub const polyMeshBuild = pipeline_mod.polyMeshBuild;
pub const polyMeshBuildDetail = pipeline_mod.polyMeshBuildDetail;
pub const polyMeshCopy = pipeline_mod.polyMeshCopy;
pub const polyMeshMerge = pipeline_mod.polyMeshMerge;
pub const getCon = pipeline_mod.getCon;
pub const setCon = pipeline_mod.setCon;
pub const dirOffsetX = pipeline_mod.dirOffsetX;
pub const dirOffsetY = pipeline_mod.dirOffsetY;
pub const dirForOffset = pipeline_mod.dirForOffset;

// The layered heightfield, the tile cache, and the layer builder.
pub const CompressedTileRef = tilecache_mod.CompressedTileRef;
pub const ObstacleRef = tilecache_mod.ObstacleRef;
pub const HeightfieldLayer = tilecache_mod.HeightfieldLayer;
pub const HeightfieldLayerSet = tilecache_mod.HeightfieldLayerSet;
pub const TileCacheParams = tilecache_mod.TileCacheParams;
pub const Compressor = tilecache_mod.Compressor;
pub const Allocator = tilecache_mod.Allocator;
pub const TileCacheBuildParams = tilecache_mod.TileCacheBuildParams;
pub const MeshProcess = tilecache_mod.MeshProcess;
pub const ObstacleShape = tilecache_mod.ObstacleShape;
pub const ObstacleState = tilecache_mod.ObstacleState;
pub const ObstacleInfo = tilecache_mod.ObstacleInfo;
pub const CompressedTileInfo = tilecache_mod.CompressedTileInfo;
pub const TileCache = tilecache_mod.TileCache;
pub const TileCacheLayerHeader = tilecache_mod.TileCacheLayerHeader;
pub const buildTileCacheLayer = tilecache_mod.buildTileCacheLayer;
pub const swapTileCacheHeaderEndian = tilecache_mod.swapTileCacheHeaderEndian;
pub const TileCacheLayer = tilecache_mod.TileCacheLayer;
pub const TileCacheContourInfo = tilecache_mod.TileCacheContourInfo;
pub const TileCacheContourVertex = tilecache_mod.TileCacheContourVertex;
pub const TileCacheContourSet = tilecache_mod.TileCacheContourSet;
pub const TileCachePolyMeshInfo = tilecache_mod.TileCachePolyMeshInfo;
pub const TileCachePolyMesh = tilecache_mod.TileCachePolyMesh;
pub const layerCon = tilecache_mod.layerCon;
pub const layerPortal = tilecache_mod.layerPortal;

// Crowds, and the five pieces they steer with.
pub const AgentRef = crowd_mod.AgentRef;
pub const PathRequestRef = crowd_mod.PathRequestRef;
pub const path_request_none = crowd_mod.path_request_none;
pub const CrowdAgentState = crowd_mod.CrowdAgentState;
pub const CrowdTargetState = crowd_mod.CrowdTargetState;
pub const CrowdAgentParams = crowd_mod.CrowdAgentParams;
pub const CrowdAgent = crowd_mod.CrowdAgent;
pub const CrowdCorner = crowd_mod.CrowdCorner;
pub const CrowdNeighbour = crowd_mod.CrowdNeighbour;
pub const CrowdAgentAnimation = crowd_mod.CrowdAgentAnimation;
pub const CrowdAgentDebug = crowd_mod.CrowdAgentDebug;
pub const UpdateFlags = crowd_mod.UpdateFlags;
pub const Crowd = crowd_mod.Crowd;
pub const ProximityGrid = crowd_mod.ProximityGrid;
pub const AvoidanceParams = crowd_mod.AvoidanceParams;
pub const AvoidanceCircle = crowd_mod.AvoidanceCircle;
pub const AvoidanceSegment = crowd_mod.AvoidanceSegment;
pub const AvoidanceSample = crowd_mod.AvoidanceSample;
pub const AvoidanceQuery = crowd_mod.AvoidanceQuery;
pub const AvoidanceDebug = crowd_mod.AvoidanceDebug;
pub const avoidanceParamsDefault = crowd_mod.avoidanceParamsDefault;
pub const PathCorridorInfo = crowd_mod.PathCorridorInfo;
pub const PathCorridor = crowd_mod.PathCorridor;
pub const mergeCorridorStartMoved = crowd_mod.mergeCorridorStartMoved;
pub const mergeCorridorEndMoved = crowd_mod.mergeCorridorEndMoved;
pub const mergeCorridorStartShortcut = crowd_mod.mergeCorridorStartShortcut;
pub const LocalBoundary = crowd_mod.LocalBoundary;
pub const PathRequestState = crowd_mod.PathRequestState;
pub const PathQueue = crowd_mod.PathQueue;
pub const crowd_max_neighbours = crowd_mod.crowd_max_neighbours;
pub const crowd_max_corners = crowd_mod.crowd_max_corners;
pub const crowd_max_avoidance_params = crowd_mod.crowd_max_avoidance_params;
pub const crowd_max_filters = crowd_mod.crowd_max_filters;
pub const crowd_max_agents = crowd_mod.crowd_max_agents;
pub const avoidance_max_pattern_divs = crowd_mod.avoidance_max_pattern_divs;
pub const avoidance_max_pattern_rings = crowd_mod.avoidance_max_pattern_rings;
pub const path_corridor_min_path = crowd_mod.path_corridor_min_path;

// Geometry and value math.
pub const vec = vec_mod;
pub const geom = geom_mod;

/// The currency of the whole API: every position, extent and bound crossing
/// the boundary is one of these.
pub const Vec3 = vec_mod.Vec3;

/// The area id a bake assigns to walkable surface, and the flag that goes with
/// it. A polygon with no flags is invisible to every query.
pub const area_walkable = c.area_walkable;
pub const area_null = c.area_null;
pub const poly_flag_walkable = c.poly_flag_walkable;

/// Build options the C library was actually compiled with, so a consumer can
/// branch on them instead of assuming.
pub const options = @import("zrecast_options");

//=============================================================================
// Versions
//=============================================================================

pub const Version = struct {
    major: u8,
    minor: u8,
    patch: u8,

    fn unpack(packed_value: u32) Version {
        return .{
            .major = @truncate(packed_value >> 16),
            .minor = @truncate(packed_value >> 8),
            .patch = @truncate(packed_value),
        };
    }

    pub fn format(self: Version, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}.{d}.{d}", .{ self.major, self.minor, self.patch });
    }
};

/// Version of these bindings.
pub fn version() Version {
    return Version.unpack(c.zrcVersion());
}

/// Version of the vendored recastnavigation tree.
pub fn recastVersion() Version {
    return Version.unpack(c.zrcRecastVersion());
}

//=============================================================================
// Tests
//=============================================================================

test {
    // Pull every module in so its own tests are discovered and run.
    _ = error_mod;
    _ = memory_mod;
    _ = bake_mod;
    _ = navmesh_mod;
    _ = query_mod;
    _ = vec_mod;
    _ = geom_mod;
    _ = asserts_mod;
    _ = pipeline_mod;
    _ = tilecache_mod;
    // Only reachable in a test build, where the fixture library is linked.
    _ = @import("integration_test.zig");
}

test "the C library agrees with the extern declarations in c.zig" {
    // This is the guard that makes hand-written externs safe. Every field the
    // Zig side believes in is checked against what the C++ translation unit
    // compiled to. A reordered field fails here rather than in production.
    var layout: c.AbiLayout = undefined;
    c.zrcAbiLayout(&layout);

    try std.testing.expectEqual(@as(u32, @sizeOf(c.AbiLayout)), layout.layout_size);

    try std.testing.expectEqual(@as(u32, @sizeOf(c.TriMesh)), layout.trimesh_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.TriMesh)), layout.trimesh_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TriMesh, "verts")),
        layout.trimesh_offset_verts,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TriMesh, "vert_count")),
        layout.trimesh_offset_vert_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TriMesh, "tris")),
        layout.trimesh_offset_tris,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TriMesh, "tri_count")),
        layout.trimesh_offset_tri_count,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.BakeConfig)), layout.bake_config_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.BakeConfig)), layout.bake_config_align);
    // Every field, not a sample, and looked up by NAME rather than by position.
    //
    // ZrcBakeConfig is ten consecutive floats followed by seven more 4-byte
    // fields. Two of them swapping places between the C header and the externs
    // in c.zig leaves the size, the alignment, and — this is the part that
    // matters — the whole *sequence* of offsets unchanged, because each side
    // reports its own declaration order. Only asking "where does the field
    // called X live on each side" can see it. Every bake would otherwise
    // quietly use the wrong agent dimensions.
    //
    // This list is the field order of ZrcBakeConfig in ffi/zrecast.h, which is
    // the same order zrcAbiLayout fills its offsets array in.
    const bake_field_order = [_][]const u8{
        "cell_size",
        "cell_height",
        "agent_height",
        "agent_radius",
        "agent_max_climb",
        "agent_max_slope",
        "region_min_size",
        "region_merge_size",
        "edge_max_len",
        "edge_max_error",
        "verts_per_poly",
        "detail_sample_dist",
        "detail_sample_max_error",
        "partition",
        "filter_low_hanging_obstacles",
        "filter_ledge_spans",
        "filter_walkable_low_height_spans",
        "tile_size",
        "border_size",
    };
    const bake_fields = @typeInfo(c.BakeConfig).@"struct".fields;
    // A field added to one side and not the other, in either direction.
    try std.testing.expectEqual(
        @as(u32, bake_field_order.len),
        layout.bake_config_field_count,
    );
    try std.testing.expectEqual(bake_field_order.len, bake_fields.len);
    try std.testing.expect(bake_field_order.len <= c.abi_max_fields);
    inline for (bake_field_order, 0..) |name, i| {
        try std.testing.expectEqual(
            @as(u32, @offsetOf(c.BakeConfig, name)),
            layout.bake_config_offsets[i],
        );
    }

    try std.testing.expectEqual(@as(u32, @sizeOf(c.BakeLog)), layout.bake_log_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.BakeLog)), layout.bake_log_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.BakeLog, "buffer")),
        layout.bake_log_offset_buffer,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.BakeLog, "capacity")),
        layout.bake_log_offset_capacity,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.QueryFilter)), layout.query_filter_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.QueryFilter)), layout.query_filter_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.QueryFilter, "area_cost")),
        layout.query_filter_offset_area_cost,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.QueryFilter, "include_flags")),
        layout.query_filter_offset_include_flags,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.QueryFilter, "exclude_flags")),
        layout.query_filter_offset_exclude_flags,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.RaycastHit)), layout.raycast_hit_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.RaycastHit)), layout.raycast_hit_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.RaycastHit, "t")),
        layout.raycast_hit_offset_t,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.RaycastHit, "position")),
        layout.raycast_hit_offset_position,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.RaycastHit, "normal")),
        layout.raycast_hit_offset_normal,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.RaycastHit, "hit")),
        layout.raycast_hit_offset_hit,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.Allocator)), layout.allocator_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.Allocator)), layout.allocator_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Allocator, "allocate")),
        layout.allocator_offset_allocate,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Allocator, "deallocate")),
        layout.allocator_offset_deallocate,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Allocator, "user")),
        layout.allocator_offset_user,
    );

    // Two float triples, a float and two ints: swapping the triples leaves the
    // size and the alignment identical, so each offset is checked by name the
    // way BakeConfig's are.
    try std.testing.expectEqual(@as(u32, @sizeOf(c.TileGrid)), layout.tile_grid_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.TileGrid)), layout.tile_grid_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileGrid, "origin")),
        layout.tile_grid_offset_origin,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileGrid, "extent_max")),
        layout.tile_grid_offset_extent_max,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileGrid, "tile_world_size")),
        layout.tile_grid_offset_tile_world_size,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileGrid, "tile_count_x")),
        layout.tile_grid_offset_tile_count_x,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileGrid, "tile_count_z")),
        layout.tile_grid_offset_tile_count_z,
    );

    // A host fills these in, and a reorder of y_min against y_max, or xz_min
    // against xz_max, changes neither the size nor the alignment while
    // inverting every volume authored through them.
    try std.testing.expectEqual(@as(u32, @sizeOf(c.AreaVolume)), layout.area_volume_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.AreaVolume)), layout.area_volume_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "shape")),
        layout.area_volume_offset_shape,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "area")),
        layout.area_volume_offset_area,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "y_min")),
        layout.area_volume_offset_y_min,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "y_max")),
        layout.area_volume_offset_y_max,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "verts")),
        layout.area_volume_offset_verts,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "vert_count")),
        layout.area_volume_offset_vert_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "xz_min")),
        layout.area_volume_offset_xz_min,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "xz_max")),
        layout.area_volume_offset_xz_max,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaVolume, "radius")),
        layout.area_volume_offset_radius,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.AreaAuthoring)),
        layout.area_authoring_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.AreaAuthoring)),
        layout.area_authoring_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaAuthoring, "volumes")),
        layout.area_authoring_offset_volumes,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaAuthoring, "volume_count")),
        layout.area_authoring_offset_volume_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AreaAuthoring, "area_flags")),
        layout.area_authoring_offset_area_flags,
    );

    // ZrcOffMeshConnection mixes two float triples with a float, an i32, a
    // u16, a ZrcBool and a u32: every offset is checked by name, as
    // ZrcAreaVolume's are, because a reorder among same-sized fields would
    // leave the size and the alignment unchanged.
    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.OffMeshConnection)),
        layout.off_mesh_connection_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.OffMeshConnection)),
        layout.off_mesh_connection_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "start")),
        layout.off_mesh_connection_offset_start,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "end")),
        layout.off_mesh_connection_offset_end,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "radius")),
        layout.off_mesh_connection_offset_radius,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "area")),
        layout.off_mesh_connection_offset_area,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "flags")),
        layout.off_mesh_connection_offset_flags,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "bidirectional")),
        layout.off_mesh_connection_offset_bidirectional,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "user_id")),
        layout.off_mesh_connection_offset_user_id,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.OffMeshConnection, "end_side")),
        layout.off_mesh_connection_offset_end_side,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.TileAuthoring)),
        layout.tile_authoring_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.TileAuthoring)),
        layout.tile_authoring_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileAuthoring, "connections")),
        layout.tile_authoring_offset_connections,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileAuthoring, "connection_count")),
        layout.tile_authoring_offset_connection_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileAuthoring, "user_id")),
        layout.tile_authoring_offset_user_id,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileAuthoring, "skip_bv_tree")),
        layout.tile_authoring_offset_skip_bv_tree,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.NavMeshParams)),
        layout.nav_mesh_params_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.NavMeshParams)),
        layout.nav_mesh_params_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.NavMeshParams, "origin")),
        layout.nav_mesh_params_offset_origin,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.NavMeshParams, "tile_width")),
        layout.nav_mesh_params_offset_tile_width,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.NavMeshParams, "tile_height")),
        layout.nav_mesh_params_offset_tile_height,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.NavMeshParams, "max_tiles")),
        layout.nav_mesh_params_offset_max_tiles,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.NavMeshParams, "max_polys")),
        layout.nav_mesh_params_offset_max_polys,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.TileInfo)), layout.tile_info_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.TileInfo)), layout.tile_info_align);
    // Every field, by name, for the same reason ZrcBakeConfig's list is: a
    // struct this wide can swap two same-sized fields without moving its size
    // or its alignment.
    const tile_info_field_order = [_][]const u8{
        "tile_x",
        "tile_z",
        "tile_layer",
        "user_id",
        "poly_count",
        "ground_poly_count",
        "off_mesh_con_count",
        "vert_count",
        "detail_mesh_count",
        "detail_vert_count",
        "detail_tri_count",
        "bv_node_count",
        "max_link_count",
        "walkable_height",
        "walkable_radius",
        "walkable_climb",
        "bmin",
        "bmax",
        "bv_quant_factor",
        "magic",
        "flags",
        "salt",
        "links_free_list",
        "next_tile",
    };
    const tile_info_fields = @typeInfo(c.TileInfo).@"struct".fields;
    try std.testing.expectEqual(
        @as(u32, tile_info_field_order.len),
        layout.tile_info_field_count,
    );
    try std.testing.expectEqual(tile_info_field_order.len, tile_info_fields.len);
    try std.testing.expect(tile_info_field_order.len <= c.abi_max_fields);
    inline for (tile_info_field_order, 0..) |name, i| {
        try std.testing.expectEqual(
            @as(u32, @offsetOf(c.TileInfo, name)),
            layout.tile_info_offsets[i],
        );
    }

    try std.testing.expectEqual(@as(u32, @sizeOf(c.PolyInfo)), layout.poly_info_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.PolyInfo)), layout.poly_info_align);
    const poly_info_field_order = [_][]const u8{
        "verts",
        "neis",
        "flags",
        "vert_count",
        "area",
        "type",
        "first_link",
        "detail_vert_base",
        "detail_tri_base",
        "detail_vert_count",
        "detail_tri_count",
    };
    const poly_info_fields = @typeInfo(c.PolyInfo).@"struct".fields;
    try std.testing.expectEqual(
        @as(u32, poly_info_field_order.len),
        layout.poly_info_field_count,
    );
    try std.testing.expectEqual(poly_info_field_order.len, poly_info_fields.len);
    try std.testing.expect(poly_info_field_order.len <= c.abi_max_fields);
    inline for (poly_info_field_order, 0..) |name, i| {
        try std.testing.expectEqual(
            @as(u32, @offsetOf(c.PolyInfo, name)),
            layout.poly_info_offsets[i],
        );
    }

    try std.testing.expectEqual(@as(u32, @sizeOf(c.Link)), layout.link_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.Link)), layout.link_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Link, "ref")),
        layout.link_offset_ref,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Link, "next")),
        layout.link_offset_next,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Link, "edge")),
        layout.link_offset_edge,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Link, "side")),
        layout.link_offset_side,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Link, "bmin")),
        layout.link_offset_bmin,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.Link, "bmax")),
        layout.link_offset_bmax,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.BvNode)), layout.bv_node_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.BvNode)), layout.bv_node_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.BvNode, "bmin")),
        layout.bv_node_offset_bmin,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.BvNode, "bmax")),
        layout.bv_node_offset_bmax,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.BvNode, "i")),
        layout.bv_node_offset_i,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.PolyRef)), layout.poly_ref_size);
    try std.testing.expectEqual(@as(u32, @sizeOf(c.TileRef)), layout.tile_ref_size);

    // Constants the Zig side mirrors rather than reads.
    try std.testing.expectEqual(@as(u32, c.max_areas), layout.max_areas);
    try std.testing.expectEqual(@as(u32, c.verts_per_polygon), layout.verts_per_polygon);
    try std.testing.expectEqual(@as(u32, c.alloc_alignment), layout.alloc_alignment);

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.AssertHandler)),
        layout.assert_handler_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.AssertHandler)),
        layout.assert_handler_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AssertHandler, "fail")),
        layout.assert_handler_offset_fail,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.AssertHandler, "user")),
        layout.assert_handler_offset_user,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.TileLayout)), layout.tile_layout_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.TileLayout)), layout.tile_layout_align);
    // Every field, by name, for the same reason ZrcBakeConfig's list is:
    // ZrcTileLayout is seventeen fields of one type, the worst case for it —
    // any two of them trading places leaves the size, the alignment and the
    // whole sequence of offsets unchanged.
    const tile_layout_field_order = [_][]const u8{
        "verts_offset",
        "verts_size",
        "polys_offset",
        "polys_size",
        "links_offset",
        "links_size",
        "detail_meshes_offset",
        "detail_meshes_size",
        "detail_verts_offset",
        "detail_verts_size",
        "detail_tris_offset",
        "detail_tris_size",
        "bv_tree_offset",
        "bv_tree_size",
        "off_mesh_cons_offset",
        "off_mesh_cons_size",
        "total_size",
    };
    const tile_layout_fields = @typeInfo(c.TileLayout).@"struct".fields;
    try std.testing.expectEqual(
        @as(u32, tile_layout_field_order.len),
        layout.tile_layout_field_count,
    );
    try std.testing.expectEqual(tile_layout_field_order.len, tile_layout_fields.len);
    try std.testing.expect(tile_layout_field_order.len <= c.abi_max_fields);
    inline for (tile_layout_field_order, 0..) |name, i| {
        try std.testing.expectEqual(
            @as(u32, @offsetOf(c.TileLayout, name)),
            layout.tile_layout_offsets[i],
        );
    }

    // The staged Recast pipeline's structs. The five wide ones report every
    // offset by name for the reason ZrcBakeConfig does; the narrow ones are
    // checked field by field.
    const build_context_field_order = [_][]const u8{
        "user",           "log",        "reset_log",        "reset_timers",
        "start_timer",    "stop_timer", "accumulated_time", "log_enabled",
        "timers_enabled",
    };
    try expectFieldOffsets(
        c.BuildContext,
        &build_context_field_order,
        layout.build_context_size,
        layout.build_context_align,
        layout.build_context_field_count,
        &layout.build_context_offsets,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.Span)), layout.span_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.Span)), layout.span_align);
    try std.testing.expectEqual(@as(u32, @offsetOf(c.Span, "smin")), layout.span_offset_smin);
    try std.testing.expectEqual(@as(u32, @offsetOf(c.Span, "smax")), layout.span_offset_smax);
    try std.testing.expectEqual(@as(u32, @offsetOf(c.Span, "area")), layout.span_offset_area);

    const heightfield_info_field_order = [_][]const u8{
        "width", "height", "bmin", "bmax", "cell_size", "cell_height",
    };
    try expectFieldOffsets(
        c.HeightfieldInfo,
        &heightfield_info_field_order,
        layout.heightfield_info_size,
        layout.heightfield_info_align,
        layout.heightfield_info_field_count,
        &layout.heightfield_info_offsets,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.HeightfieldStorage)),
        layout.heightfield_storage_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.HeightfieldStorage)),
        layout.heightfield_storage_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.HeightfieldStorage, "pool_count")),
        layout.heightfield_storage_offset_pool_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.HeightfieldStorage, "free_count")),
        layout.heightfield_storage_offset_free_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.HeightfieldStorage, "spans_per_pool")),
        layout.heightfield_storage_offset_spans_per_pool,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.CompactCell)), layout.compact_cell_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.CompactCell)), layout.compact_cell_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.CompactCell, "index")),
        layout.compact_cell_offset_index,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.CompactCell, "count")),
        layout.compact_cell_offset_count,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.CompactSpan)), layout.compact_span_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.CompactSpan)), layout.compact_span_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.CompactSpan, "y")),
        layout.compact_span_offset_y,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.CompactSpan, "reg")),
        layout.compact_span_offset_reg,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.CompactSpan, "con")),
        layout.compact_span_offset_con,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.CompactSpan, "h")),
        layout.compact_span_offset_h,
    );

    const compact_heightfield_info_field_order = [_][]const u8{
        "width",              "height",         "span_count",
        "walkable_height",    "walkable_climb", "border_size",
        "max_distance",       "max_regions",    "bmin",
        "bmax",               "cell_size",      "cell_height",
        "has_distance_field",
    };
    try expectFieldOffsets(
        c.CompactHeightfieldInfo,
        &compact_heightfield_info_field_order,
        layout.compact_heightfield_info_size,
        layout.compact_heightfield_info_align,
        layout.compact_heightfield_info_field_count,
        &layout.compact_heightfield_info_offsets,
    );

    const contour_set_info_field_order = [_][]const u8{
        "contour_count", "bmin",  "bmax",   "cell_size",
        "cell_height",   "width", "height", "border_size",
        "max_error",
    };
    try expectFieldOffsets(
        c.ContourSetInfo,
        &contour_set_info_field_order,
        layout.contour_set_info_size,
        layout.contour_set_info_align,
        layout.contour_set_info_field_count,
        &layout.contour_set_info_offsets,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.ContourInfo)), layout.contour_info_size);
    try std.testing.expectEqual(@as(u32, @alignOf(c.ContourInfo)), layout.contour_info_align);
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.ContourInfo, "vert_count")),
        layout.contour_info_offset_vert_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.ContourInfo, "raw_vert_count")),
        layout.contour_info_offset_raw_vert_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.ContourInfo, "region")),
        layout.contour_info_offset_region,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.ContourInfo, "area")),
        layout.contour_info_offset_area,
    );

    const poly_mesh_info_field_order = [_][]const u8{
        "vert_count",       "poly_count",        "max_polys",
        "verts_per_poly",   "bmin",              "bmax",
        "cell_size",        "cell_height",       "border_size",
        "max_edge_error",   "detail_mesh_count", "detail_vert_count",
        "detail_tri_count", "walkable_height",   "walkable_radius",
        "walkable_climb",
    };
    try expectFieldOffsets(
        c.PolyMeshInfo,
        &poly_mesh_info_field_order,
        layout.poly_mesh_info_size,
        layout.poly_mesh_info_align,
        layout.poly_mesh_info_field_count,
        &layout.poly_mesh_info_offsets,
    );

    const build_cells_field_order = [_][]const u8{
        "walkable_height",    "walkable_climb",           "walkable_radius",
        "max_edge_len",       "min_region_area",          "merge_region_area",
        "border_size",        "max_simplification_error", "verts_per_poly",
        "detail_sample_dist", "detail_sample_max_error",
    };
    try expectFieldOffsets(
        c.BuildCells,
        &build_cells_field_order,
        layout.build_cells_size,
        layout.build_cells_align,
        layout.build_cells_field_count,
        &layout.build_cells_offsets,
    );

    // The layered heightfield and the tile cache.
    const heightfield_layer_field_order = [_][]const u8{
        "bmin",  "bmax",   "cell_size",  "cell_height",
        "width", "height", "min_x",      "max_x",
        "min_z", "max_z",  "height_min", "height_max",
    };
    try expectFieldOffsets(
        c.HeightfieldLayer,
        &heightfield_layer_field_order,
        layout.heightfield_layer_size,
        layout.heightfield_layer_align,
        layout.heightfield_layer_field_count,
        &layout.heightfield_layer_offsets,
    );

    const tile_cache_params_field_order = [_][]const u8{
        "origin",          "cell_size",      "cell_height",
        "width",           "height",         "walkable_height",
        "walkable_radius", "walkable_climb", "max_simplification_error",
        "max_tiles",       "max_obstacles",
    };
    try expectFieldOffsets(
        c.TileCacheParams,
        &tile_cache_params_field_order,
        layout.tile_cache_params_size,
        layout.tile_cache_params_align,
        layout.tile_cache_params_field_count,
        &layout.tile_cache_params_offsets,
    );

    const tile_cache_compressor_field_order = [_][]const u8{
        "user", "max_compressed_size", "compress", "decompress",
    };
    try expectFieldOffsets(
        c.TileCacheCompressor,
        &tile_cache_compressor_field_order,
        layout.tile_cache_compressor_size,
        layout.tile_cache_compressor_align,
        layout.tile_cache_compressor_field_count,
        &layout.tile_cache_compressor_offsets,
    );

    const tile_cache_allocator_field_order = [_][]const u8{
        "user", "reset", "allocate", "deallocate",
    };
    try expectFieldOffsets(
        c.TileCacheAllocator,
        &tile_cache_allocator_field_order,
        layout.tile_cache_allocator_size,
        layout.tile_cache_allocator_align,
        layout.tile_cache_allocator_field_count,
        &layout.tile_cache_allocator_offsets,
    );

    const tile_cache_build_params_field_order = [_][]const u8{
        "areas", "flags", "poly_count", "user_id", "connections", "connection_count",
    };
    try expectFieldOffsets(
        c.TileCacheBuildParams,
        &tile_cache_build_params_field_order,
        layout.tile_cache_build_params_size,
        layout.tile_cache_build_params_align,
        layout.tile_cache_build_params_field_count,
        &layout.tile_cache_build_params_offsets,
    );

    const compressed_tile_info_field_order = [_][]const u8{
        "tile_x", "tile_y",     "tile_layer", "bmin",
        "bmax",   "height_min", "height_max", "width",
        "height", "min_x",      "max_x",      "min_z",
        "max_z",  "data_size",
    };
    try expectFieldOffsets(
        c.CompressedTileInfo,
        &compressed_tile_info_field_order,
        layout.compressed_tile_info_size,
        layout.compressed_tile_info_align,
        layout.compressed_tile_info_field_count,
        &layout.compressed_tile_info_offsets,
    );

    const obstacle_info_field_order = [_][]const u8{
        "shape",        "state",     "position",      "radius",
        "height",       "bmin",      "bmax",          "center",
        "half_extents", "y_radians", "touched_count", "pending_count",
    };
    try expectFieldOffsets(
        c.ObstacleInfo,
        &obstacle_info_field_order,
        layout.obstacle_info_size,
        layout.obstacle_info_align,
        layout.obstacle_info_field_count,
        &layout.obstacle_info_offsets,
    );

    const tile_cache_layer_header_field_order = [_][]const u8{
        "tile_x", "tile_y",     "tile_layer", "bmin",
        "bmax",   "height_min", "height_max", "width",
        "height", "min_x",      "max_x",      "min_z",
        "max_z",
    };
    try expectFieldOffsets(
        c.TileCacheLayerHeader,
        &tile_cache_layer_header_field_order,
        layout.tile_cache_layer_header_size,
        layout.tile_cache_layer_header_align,
        layout.tile_cache_layer_header_field_count,
        &layout.tile_cache_layer_header_offsets,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.TileCacheContourInfo)),
        layout.tile_cache_contour_info_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.TileCacheContourInfo)),
        layout.tile_cache_contour_info_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileCacheContourInfo, "vert_count")),
        layout.tile_cache_contour_info_offset_vert_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileCacheContourInfo, "region")),
        layout.tile_cache_contour_info_offset_region,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileCacheContourInfo, "area")),
        layout.tile_cache_contour_info_offset_area,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.TileCachePolyMeshInfo)),
        layout.tile_cache_poly_mesh_info_size,
    );
    try std.testing.expectEqual(
        @as(u32, @alignOf(c.TileCachePolyMeshInfo)),
        layout.tile_cache_poly_mesh_info_align,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileCachePolyMeshInfo, "vert_count")),
        layout.tile_cache_poly_mesh_info_offset_vert_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileCachePolyMeshInfo, "poly_count")),
        layout.tile_cache_poly_mesh_info_offset_poly_count,
    );
    try std.testing.expectEqual(
        @as(u32, @offsetOf(c.TileCachePolyMeshInfo, "verts_per_poly")),
        layout.tile_cache_poly_mesh_info_offset_verts_per_poly,
    );

    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.CompressedTileRef)),
        layout.compressed_tile_ref_size,
    );
    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.ObstacleRef)),
        layout.obstacle_ref_size,
    );

    // The crowd's cross-boundary structs. ZrcCrowdAgentParams and
    // ZrcCrowdAgentDebug each carry a pointer, so their size and several
    // offsets differ between a 32-bit and a 64-bit target — the disagreement a
    // hand-written extern struct can get wrong with no compiler noticing.
    const crowd_agent_params_field_order = [_][]const u8{
        "radius",
        "height",
        "max_acceleration",
        "max_speed",
        "collision_query_range",
        "path_optimization_range",
        "separation_weight",
        "update_flags",
        "obstacle_avoidance_type",
        "query_filter_type",
        "user_data",
    };
    try expectFieldOffsets(
        c.CrowdAgentParams,
        &crowd_agent_params_field_order,
        layout.crowd_agent_params_size,
        layout.crowd_agent_params_align,
        layout.crowd_agent_params_field_count,
        &layout.crowd_agent_params_offsets,
    );

    const crowd_agent_field_order = [_][]const u8{
        "state",
        "target_state",
        "partial",
        "position",
        "velocity",
        "desired_velocity",
        "avoided_velocity",
        "displacement",
        "desired_speed",
        "target_ref",
        "target_position",
        "target_replan",
        "target_replan_time",
        "topology_opt_time",
        "corner_count",
        "neighbour_count",
        "params",
    };
    try expectFieldOffsets(
        c.CrowdAgent,
        &crowd_agent_field_order,
        layout.crowd_agent_size,
        layout.crowd_agent_align,
        layout.crowd_agent_field_count,
        &layout.crowd_agent_offsets,
    );

    const crowd_corner_field_order = [_][]const u8{
        "position",
        "flags",
        "poly",
    };
    try expectFieldOffsets(
        c.CrowdCorner,
        &crowd_corner_field_order,
        layout.crowd_corner_size,
        layout.crowd_corner_align,
        layout.crowd_corner_field_count,
        &layout.crowd_corner_offsets,
    );

    const crowd_neighbour_field_order = [_][]const u8{
        "agent",
        "distance",
    };
    try expectFieldOffsets(
        c.CrowdNeighbour,
        &crowd_neighbour_field_order,
        layout.crowd_neighbour_size,
        layout.crowd_neighbour_align,
        layout.crowd_neighbour_field_count,
        &layout.crowd_neighbour_offsets,
    );

    const crowd_agent_animation_field_order = [_][]const u8{
        "active",
        "init_position",
        "start_position",
        "end_position",
        "poly",
        "t",
        "t_max",
    };
    try expectFieldOffsets(
        c.CrowdAgentAnimation,
        &crowd_agent_animation_field_order,
        layout.crowd_agent_animation_size,
        layout.crowd_agent_animation_align,
        layout.crowd_agent_animation_field_count,
        &layout.crowd_agent_animation_offsets,
    );

    const crowd_agent_debug_field_order = [_][]const u8{
        "agent",
        "samples",
        "opt_start",
        "opt_end",
    };
    try expectFieldOffsets(
        c.CrowdAgentDebug,
        &crowd_agent_debug_field_order,
        layout.crowd_agent_debug_size,
        layout.crowd_agent_debug_align,
        layout.crowd_agent_debug_field_count,
        &layout.crowd_agent_debug_offsets,
    );

    const avoidance_params_field_order = [_][]const u8{
        "vel_bias",
        "weight_desired_vel",
        "weight_current_vel",
        "weight_side",
        "weight_toi",
        "horiz_time",
        "grid_size",
        "adaptive_divs",
        "adaptive_rings",
        "adaptive_depth",
    };
    try expectFieldOffsets(
        c.AvoidanceParams,
        &avoidance_params_field_order,
        layout.avoidance_params_size,
        layout.avoidance_params_align,
        layout.avoidance_params_field_count,
        &layout.avoidance_params_offsets,
    );

    const avoidance_circle_field_order = [_][]const u8{
        "position",
        "velocity",
        "desired_velocity",
        "radius",
    };
    try expectFieldOffsets(
        c.AvoidanceCircle,
        &avoidance_circle_field_order,
        layout.avoidance_circle_size,
        layout.avoidance_circle_align,
        layout.avoidance_circle_field_count,
        &layout.avoidance_circle_offsets,
    );

    const avoidance_segment_field_order = [_][]const u8{
        "p",
        "q",
        "touching",
    };
    try expectFieldOffsets(
        c.AvoidanceSegment,
        &avoidance_segment_field_order,
        layout.avoidance_segment_size,
        layout.avoidance_segment_align,
        layout.avoidance_segment_field_count,
        &layout.avoidance_segment_offsets,
    );

    const avoidance_sample_field_order = [_][]const u8{
        "velocity",
        "size",
        "penalty",
        "desired_velocity_penalty",
        "current_velocity_penalty",
        "preferred_side_penalty",
        "collision_time_penalty",
    };
    try expectFieldOffsets(
        c.AvoidanceSample,
        &avoidance_sample_field_order,
        layout.avoidance_sample_size,
        layout.avoidance_sample_align,
        layout.avoidance_sample_field_count,
        &layout.avoidance_sample_offsets,
    );

    const path_corridor_info_field_order = [_][]const u8{
        "position",
        "target",
        "first_poly",
        "last_poly",
        "path_count",
    };
    try expectFieldOffsets(
        c.PathCorridorInfo,
        &path_corridor_info_field_order,
        layout.path_corridor_info_size,
        layout.path_corridor_info_align,
        layout.path_corridor_info_field_count,
        &layout.path_corridor_info_offsets,
    );

    try std.testing.expectEqual(@as(u32, @sizeOf(c.AgentRef)), layout.agent_ref_size);
    try std.testing.expectEqual(
        @as(u32, @sizeOf(c.PathRequestRef)),
        layout.path_request_ref_size,
    );

    // ZRC_MAX_TIMERS is the last enumerator, so the count includes it.
    try std.testing.expectEqual(
        @as(u32, @typeInfo(c.TimerLabel).@"enum".fields.len),
        layout.timer_label_count,
    );

    // The Zig error mapping must cover every C result.
    const result_fields = @typeInfo(c.Result).@"enum".fields;
    try std.testing.expectEqual(@as(u32, result_fields.len), layout.result_count);
}

/// Checks a struct's size, alignment, field count and every offset by name.
///
/// The by-name lookup is the point, the same as it is for ZrcBakeConfig: two
/// same-sized fields trading places between the C header and the externs leaves
/// the size, the alignment and the whole sequence of offsets unchanged, because
/// each side reports its own declaration order.
fn expectFieldOffsets(
    comptime T: type,
    comptime order: []const []const u8,
    reported_size: u32,
    reported_align: u32,
    reported_count: u32,
    offsets: *const [c.abi_max_fields]u32,
) !void {
    try std.testing.expectEqual(@as(u32, @sizeOf(T)), reported_size);
    try std.testing.expectEqual(@as(u32, @alignOf(T)), reported_align);
    try std.testing.expectEqual(@as(u32, order.len), reported_count);
    try std.testing.expectEqual(order.len, @typeInfo(T).@"struct".fields.len);
    try std.testing.expect(order.len <= c.abi_max_fields);
    inline for (order, 0..) |name, i| {
        try std.testing.expectEqual(@as(u32, @offsetOf(T, name)), offsets[i]);
    }
}

test "version reporting is wired up" {
    // The version's one home is build.zig.zon; the ZRC_VERSION_* macros in
    // ffi/zrecast.h restate it for C consumers, and zrcVersion carries what
    // they compiled to, so this comparison is what stops the two drifting.
    const v = version();
    var rendered: [16]u8 = undefined;
    const got = try std.fmt.bufPrint(
        &rendered,
        "{d}.{d}.{d}",
        .{ v.major, v.minor, v.patch },
    );
    try std.testing.expectEqualStrings(options.package_version, got);

    const upstream = recastVersion();
    try std.testing.expectEqual(@as(u8, 1), upstream.major);
    try std.testing.expectEqual(@as(u8, 6), upstream.minor);
}

test "result names are never empty" {
    inline for (@typeInfo(c.Result).@"enum".fields) |field| {
        const name = resultName(@enumFromInt(field.value));
        try std.testing.expect(name.len > 0);
    }
}
