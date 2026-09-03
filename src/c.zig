//! Hand-written declarations mirroring `ffi/zrecast.h`.
//!
//! These are written by hand rather than produced by `@cImport` so the package
//! stays translate-c-free and every type is exactly the shape the rest of the
//! wrapper wants. The cost of hand-writing is drift: nothing in either compiler
//! checks that this file still agrees with the header. That gap is closed by
//! `zrcAbiLayout`, asserted in the test at the bottom of `zrecast.zig` — if a
//! field moves on either side, the test fails loudly instead of corrupting
//! memory quietly.

const std = @import("std");

//=============================================================================
// Results
//=============================================================================

pub const Result = enum(c_int) {
    ok = 0,
    invalid_argument = 1,
    out_of_memory = 2,
    bad_format = 3,
    unsupported_version = 4,
    buffer_too_small = 5,
    bake_failed = 6,
    empty_result = 7,
    query_failed = 8,
    out_of_nodes = 9,
    tile_occupied = 10,
    navmesh_full = 11,
    search_in_progress = 12,
    no_search = 13,
    not_found = 14,
    already_built = 15,
    crowd_full = 16,
};

//=============================================================================
// Constants mirrored from the header
//=============================================================================

pub const max_areas = 64;
pub const verts_per_polygon = 6;
pub const alloc_alignment = 16;

/// Capacity of `AbiLayout.bake_config_offsets`; mirrors ZRC_ABI_MAX_FIELDS.
pub const abi_max_fields = 32;

pub const area_null: u8 = 0;
pub const area_walkable: u8 = 63;
pub const poly_flag_walkable: u16 = 0x0001;

pub const straightpath_start: u8 = 0x01;
pub const straightpath_end: u8 = 0x02;
pub const straightpath_offmesh_connection: u8 = 0x04;

/// Options for `zrcFindStraightPath`, distinct from the straightpath_* flags
/// above despite the shared prefix: these go in, those come out.
pub const straightpath_area_crossings: u32 = 0x01;
pub const straightpath_all_crossings: u32 = 0x02;

/// Options for `zrcSlicedFindPathInit`.
pub const findpath_any_angle: u32 = 0x02;

/// How far an any-angle shortcut ray may reach, as a multiple of the agent
/// radius the navmesh was baked for.
pub const raycast_limit_proportions: f32 = 50.0;

/// Options for `zrcRaycast`.
pub const raycast_use_costs: u32 = 0x01;

/// How many distinct search states one polygon may carry at once.
pub const max_node_states = 4;

/// What a search concluded about one polygon (ZrcNode.flags).
pub const node_open: u32 = 0x01;
pub const node_closed: u32 = 0x02;
pub const node_parent_detached: u32 = 0x04;

/// A polygon reference. Zero means "no polygon".
pub const PolyRef = u32;

/// A tile reference. Zero means "no tile".
pub const TileRef = u32;

/// Bounds on a tile's place in the grid; mirrors ZRC_MAX_TILE_COORD and
/// ZRC_MAX_TILE_LAYER.
pub const max_tile_coord = 1048576;
pub const max_tile_layer = 255;

/// The C ABI's boolean: a 32-bit int, not Zig's `bool`, so the layout is fixed.
pub const Bool = i32;
pub const c_false: Bool = 0;
pub const c_true: Bool = 1;

/// Options for `zrcContourSetCreate`, bitwise ORed into `flags`.
pub const contour_none: i32 = 0;
pub const contour_tess_wall_edges: i32 = 0x01;
pub const contour_tess_area_edges: i32 = 0x02;

/// Bits Recast gives a span's lower and upper extent.
pub const span_height_bits = 13;
/// Largest value either extent can hold.
pub const span_max_height = 8191;
/// Spans in one of a heightfield's allocation pools.
pub const spans_per_pool = 2048;

/// The value CompactSpan.con holds for a direction with no neighbour.
pub const not_connected: u32 = 0x3f;
/// Region id bit marking a border region, whose spans are unwalkable.
pub const border_reg: u16 = 0x8000;

/// Contour vertex flag: the vertex lies on a tile border.
pub const border_vertex: i32 = 0x10000;
/// Contour vertex flag: the vertex lies on the border of an area.
pub const area_border: i32 = 0x20000;
/// Mask isolating the region id from a contour vertex's fourth component.
pub const contour_reg_mask: i32 = 0xffff;

/// The vertex index Recast writes where a polygon has no further corner and
/// no neighbour.
pub const mesh_null_idx: u16 = 0xffff;
/// The region id of a polygon merged from several regions.
pub const multiple_regs: u16 = 0;

/// Magic and version of a compressed layer's header, for a host to recognise
/// its own asset.
pub const tilecache_magic: i32 = 0x44544C52;
pub const tilecache_version: i32 = 1;

/// How many tiles one obstacle may overlap. Upstream's own limit: an obstacle
/// touching more is carved into the first eight and silently missing from the
/// rest.
pub const max_touched_tiles = 8;

/// Names one compressed layer in a tile cache. Salt-protected the same way a
/// `TileRef` is.
pub const CompressedTileRef = u32;

/// Names one obstacle. Salt-protected, and the salt turns over when the
/// deferred removal completes rather than when it is requested.
pub const ObstacleRef = u32;

/// The area id a tile-cache layer gives unwalkable surface, and the one it
/// gives walkable surface.
pub const tilecache_area_null: u8 = 0;
pub const tilecache_area_walkable: u8 = 63;

/// The index a tile-cache polygon writes where it has no further corner and
/// no neighbour.
pub const tilecache_null_idx: u16 = 0xffff;

//=============================================================================
// Plain data
//=============================================================================

pub const AllocHint = enum(c_int) {
    perm = 0,
    temp = 1,
};

pub const Allocator = extern struct {
    allocate: ?*const fn (user: ?*anyopaque, size: usize, hint: AllocHint) callconv(.c) ?*anyopaque,
    deallocate: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void,
    user: ?*anyopaque,
};

pub const AssertFailFunc = ?*const fn (?*anyopaque, [*:0]const u8, [*:0]const u8, i32) callconv(.c) void;

pub const AssertHandler = extern struct {
    fail: AssertFailFunc,
    user: ?*anyopaque,
};

pub const TriMesh = extern struct {
    verts: [*]const f32,
    vert_count: i32,
    tris: [*]const i32,
    tri_count: i32,
};

pub const Partition = enum(i32) {
    watershed = 0,
    monotone = 1,
    layers = 2,
};

pub const BakeConfig = extern struct {
    cell_size: f32,
    cell_height: f32,
    agent_height: f32,
    agent_radius: f32,
    agent_max_climb: f32,
    agent_max_slope: f32,
    region_min_size: f32,
    region_merge_size: f32,
    edge_max_len: f32,
    edge_max_error: f32,
    verts_per_poly: i32,
    detail_sample_dist: f32,
    detail_sample_max_error: f32,
    partition: Partition,
    filter_low_hanging_obstacles: Bool,
    filter_ledge_spans: Bool,
    filter_walkable_low_height_spans: Bool,
    tile_size: i32,
    border_size: i32,
};

pub const TileGrid = extern struct {
    origin: [3]f32,
    extent_max: [3]f32,
    tile_world_size: f32,
    tile_count_x: i32,
    tile_count_z: i32,
};

pub const BuildCells = extern struct {
    walkable_height: i32,
    walkable_climb: i32,
    walkable_radius: i32,
    max_edge_len: i32,
    min_region_area: i32,
    merge_region_area: i32,
    border_size: i32,
    max_simplification_error: f32,
    verts_per_poly: i32,
    detail_sample_dist: f32,
    detail_sample_max_error: f32,
};

pub const BakeLog = extern struct {
    buffer: ?[*]u8,
    capacity: usize,
};

pub const VolumeShape = enum(c_int) {
    convex = 0,
    box = 1,
    cylinder = 2,
};

pub const AreaVolume = extern struct {
    shape: VolumeShape,
    area: i32,
    y_min: f32,
    y_max: f32,
    verts: ?[*]const f32,
    vert_count: i32,
    xz_min: [2]f32,
    xz_max: [2]f32,
    radius: f32,
};

pub const AreaAuthoring = extern struct {
    volumes: ?[*]const AreaVolume,
    volume_count: i32,
    area_flags: ?*const [max_areas]u16,
};

/// Which of Recast's build messages a log entry is.
pub const LogCategory = enum(c_int) {
    progress = 1,
    warning = 2,
    @"error" = 3,
};

/// The build phases Recast times, and the count that bounds them.
pub const TimerLabel = enum(c_int) {
    total = 0,
    temp = 1,
    rasterize_triangles = 2,
    build_compactheightfield = 3,
    build_contours = 4,
    build_contours_trace = 5,
    build_contours_simplify = 6,
    filter_border = 7,
    filter_walkable = 8,
    median_area = 9,
    filter_low_obstacles = 10,
    build_polymesh = 11,
    merge_polymesh = 12,
    erode_area = 13,
    mark_box_area = 14,
    mark_cylinder_area = 15,
    mark_convexpoly_area = 16,
    build_distancefield = 17,
    build_distancefield_dist = 18,
    build_distancefield_blur = 19,
    build_regions = 20,
    build_regions_watershed = 21,
    build_regions_expand = 22,
    build_regions_flood = 23,
    build_regions_filter = 24,
    build_layers = 25,
    build_polymeshdetail = 26,
    merge_polymeshdetail = 27,
    /// One past the last label, and the length of a table indexed by one.
    max_timers = 28,
};

/// A build's log and timer hooks. Every hook may be NULL individually, and
/// the whole struct may be NULL, which is a context with both flags clear.
pub const BuildContext = extern struct {
    user: ?*anyopaque,
    log: ?*const fn (user: ?*anyopaque, category: LogCategory, message: [*:0]const u8, length: i32) callconv(.c) void,
    reset_log: ?*const fn (user: ?*anyopaque) callconv(.c) void,
    reset_timers: ?*const fn (user: ?*anyopaque) callconv(.c) void,
    start_timer: ?*const fn (user: ?*anyopaque, label: TimerLabel) callconv(.c) void,
    stop_timer: ?*const fn (user: ?*anyopaque, label: TimerLabel) callconv(.c) void,
    accumulated_time: ?*const fn (user: ?*anyopaque, label: TimerLabel) callconv(.c) i32,
    /// Clear to silence `log` and `reset_log` without removing them.
    log_enabled: Bool,
    /// Clear to silence the four timer hooks.
    timers_enabled: Bool,
};

/// A voxel column's obstructed interval, copied out.
pub const Span = extern struct {
    smin: u32,
    smax: u32,
    area: u8,
};

pub const HeightfieldInfo = extern struct {
    /// Cells along x and z.
    width: i32,
    height: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    cell_size: f32,
    cell_height: f32,
};

/// How much span storage a heightfield holds.
pub const HeightfieldStorage = extern struct {
    pool_count: i32,
    /// How many pooled spans are on the free list.
    free_count: i32,
    spans_per_pool: i32,
};

/// Where one column's spans start in the span array, and how many there are.
pub const CompactCell = extern struct {
    index: u32,
    count: u32,
};

/// A span of open space, copied out of upstream's packed representation.
pub const CompactSpan = extern struct {
    /// Lower extent, in cells above the field's minimum corner.
    y: u16,
    /// Region id, or 0 for none. `border_reg` marks a border region.
    reg: u16,
    /// Four 6-bit neighbour indices, one per direction, `not_connected` where
    /// there is no neighbour.
    con: u32,
    /// Height of the open space above `y`, in cells.
    h: u32,
};

pub const CompactHeightfieldInfo = extern struct {
    width: i32,
    height: i32,
    /// Length of the span, distance and area arrays.
    span_count: i32,
    walkable_height: i32,
    walkable_climb: i32,
    border_size: i32,
    /// Largest value in the distance field, or 0 before it is built.
    max_distance: u16,
    /// Largest region id assigned, or 0 before regions are built.
    max_regions: u16,
    bmin: [3]f32,
    bmax: [3]f32,
    cell_size: f32,
    cell_height: f32,
    /// Whether the distance field has been built.
    has_distance_field: Bool,
};

pub const ContourSetInfo = extern struct {
    contour_count: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    cell_size: f32,
    cell_height: f32,
    width: i32,
    height: i32,
    border_size: i32,
    /// The simplification error the set was built with, in cells.
    max_error: f32,
};

/// One region's outline.
pub const ContourInfo = extern struct {
    /// Vertices of the simplified outline.
    vert_count: i32,
    /// Vertices of the raw traced outline, before simplification.
    raw_vert_count: i32,
    /// The region this outline encloses.
    region: u16,
    /// The area id of that region.
    area: u8,
};

pub const PolyMeshInfo = extern struct {
    vert_count: i32,
    poly_count: i32,
    /// Polygons the arrays have room for, which is what indexes `polys`.
    max_polys: i32,
    /// Corners per polygon, and half the stride of a `polys` entry.
    verts_per_poly: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    cell_size: f32,
    cell_height: f32,
    border_size: i32,
    max_edge_error: f32,
    /// The detail half. All three are 0 until `zrcPolyMeshBuildDetail` has
    /// run.
    detail_mesh_count: i32,
    detail_vert_count: i32,
    detail_tri_count: i32,
    /// The dimensions `zrcPolyMeshSetAgentDims` recorded, world units.
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
};

/// What kind of polygon a reference names.
pub const PolyType = enum(c_int) {
    ground = 0,
    offmesh_connection = 1,
};

/// One point-to-point link between two places the surface does not join.
pub const OffMeshConnection = extern struct {
    start: [3]f32,
    end: [3]f32,
    radius: f32,
    area: i32,
    flags: u16,
    bidirectional: Bool,
    user_id: u32,
    end_side: i32 = 0,
};

/// What a tile carries beyond the shape of its surface.
pub const TileAuthoring = extern struct {
    connections: ?[*]const OffMeshConnection,
    connection_count: i32,
    user_id: u32,
    skip_bv_tree: Bool,
};

/// Marks a polygon edge as crossing into an adjacent tile (DT_EXT_LINK).
pub const ext_link: u16 = 0x8000;
/// The end of a link chain (DT_NULL_LINK).
pub const null_link: u32 = 0xffffffff;
/// The navmesh owns a tile's bytes and frees them with it (DT_TILE_FREE_DATA).
pub const tile_free_data: u32 = 0x01;

/// How a navmesh's tile grid was sized, as `NavMesh.initTiled` set it up.
pub const NavMeshParams = extern struct {
    origin: [3]f32,
    tile_width: f32,
    tile_height: f32,
    max_tiles: i32,
    max_polys: i32,
};

/// Everything a tile's header says about it, copied out.
pub const TileInfo = extern struct {
    tile_x: i32,
    tile_z: i32,
    tile_layer: i32,
    user_id: u32,
    poly_count: i32,
    ground_poly_count: i32,
    off_mesh_con_count: i32,
    vert_count: i32,
    detail_mesh_count: i32,
    detail_vert_count: i32,
    detail_tri_count: i32,
    bv_node_count: i32,
    max_link_count: i32,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    bmin: [3]f32,
    bmax: [3]f32,
    bv_quant_factor: f32,
    magic: i32,
    flags: u32,
    salt: u32,
    links_free_list: u32,
    next_tile: TileRef,
};

/// One polygon, copied out of the tile that holds it.
pub const PolyInfo = extern struct {
    verts: [verts_per_polygon]u16,
    neis: [verts_per_polygon]u16,
    flags: u16,
    vert_count: u8,
    area: u8,
    /// A PolyType value, stored as a plain i32 the way the header does.
    type: i32,
    first_link: u32,
    detail_vert_base: u32,
    detail_tri_base: u32,
    detail_vert_count: u8,
    detail_tri_count: u8,
};

/// One link: an edge from a polygon to a neighbour it can be walked to.
pub const Link = extern struct {
    ref: PolyRef,
    next: u32,
    edge: u8,
    side: u8,
    bmin: u8,
    bmax: u8,
};

/// One node of a tile's bounding-volume tree.
pub const BvNode = extern struct {
    bmin: [3]u16,
    bmax: [3]u16,
    i: i32,
};

/// Where each of a tile image's eight arrays begins, and how many bytes it
/// occupies, both relative to the start of the image.
pub const TileLayout = extern struct {
    verts_offset: i64,
    verts_size: i64,
    polys_offset: i64,
    polys_size: i64,
    links_offset: i64,
    links_size: i64,
    detail_meshes_offset: i64,
    detail_meshes_size: i64,
    detail_verts_offset: i64,
    detail_verts_size: i64,
    detail_tris_offset: i64,
    detail_tris_size: i64,
    bv_tree_offset: i64,
    bv_tree_size: i64,
    off_mesh_cons_offset: i64,
    off_mesh_cons_size: i64,
    total_size: i64,
};

pub const QueryFilter = extern struct {
    area_cost: [max_areas]f32,
    include_flags: u16,
    exclude_flags: u16,
};

pub const RaycastHit = extern struct {
    t: f32,
    position: [3]f32,
    normal: [3]f32,
    hit: Bool,
    /// Index of the polygon edge the ray crossed last, or -1 if it crossed
    /// none.
    hit_edge_index: i32,
    /// Cost of the movement along the ray. Zero unless `raycast_use_costs`
    /// was passed.
    path_cost: f32,
};

/// Source of randomness for `zrcFindRandomPoint` and
/// `zrcFindRandomPointAroundCircle`. `next` must return a value in [0, 1) and
/// is invoked synchronously, on the calling thread, before the entry point
/// returns.
pub const RandomSource = extern struct {
    next: ?*const fn (user: ?*anyopaque) callconv(.c) f32,
    user: ?*anyopaque,
};

/// Sink for `zrcQueryPolygonsBatched`. `refs` is borrowed and valid only for
/// the duration of the call.
pub const PolyQuery = extern struct {
    process: ?*const fn (user: ?*anyopaque, refs: [*]const PolyRef, count: i32) callconv(.c) void,
    user: ?*anyopaque,
};

/// One search node, copied out of a query's node pool.
pub const Node = extern struct {
    pos: [3]f32,
    cost: f32,
    total: f32,
    ref: PolyRef,
    /// One-based index of the node this was reached from, or 0 for the
    /// start. Pass to `zrcQueryNodeAt` to walk back towards the start.
    parent_index: u32,
    /// Which of the polygon's states this node is, below `max_node_states`.
    state: u32,
    /// A combination of the node_* flags.
    flags: u32,
};

/// How full a query's node pool is.
pub const NodePoolInfo = extern struct {
    node_count: i32,
    max_nodes: i32,
    hash_size: i32,
    bytes_used: i32,
};

/// One sheet's extent and where its usable data sits inside it.
pub const HeightfieldLayer = extern struct {
    bmin: [3]f32,
    bmax: [3]f32,
    cell_size: f32,
    cell_height: f32,
    /// Cells along x and z. The three arrays are `width * height` long.
    width: i32,
    height: i32,
    /// The sub-region holding data, in cells.
    min_x: i32,
    max_x: i32,
    min_z: i32,
    max_z: i32,
    /// The layer's own height range, in cells above `bmin[1]`.
    height_min: i32,
    height_max: i32,
};

pub const TileCacheParams = extern struct {
    /// Minimum corner of the whole world, the same origin the navmesh uses.
    origin: [3]f32,
    cell_size: f32,
    cell_height: f32,
    /// One tile's edge, in cells.
    width: i32,
    height: i32,
    /// The agent, in world units.
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    /// How far a simplified contour may sit from the traced one, in cells.
    max_simplification_error: f32,
    max_tiles: i32,
    max_obstacles: i32,
};

/// The codec a host supplies. Upstream calls these while rebuilding a tile; a
/// hook that returns anything but `.ok` aborts the operation that called it.
pub const TileCacheCompressor = extern struct {
    user: ?*anyopaque,
    /// An upper bound on what `compress` may write for a buffer this size.
    max_compressed_size: ?*const fn (user: ?*anyopaque, buffer_size: i32) callconv(.c) i32,
    compress: ?*const fn (
        user: ?*anyopaque,
        buffer: [*]const u8,
        buffer_size: i32,
        compressed: [*]u8,
        max_compressed_size: i32,
        out_compressed_size: *i32,
    ) callconv(.c) Result,
    decompress: ?*const fn (
        user: ?*anyopaque,
        compressed: [*]const u8,
        compressed_size: i32,
        buffer: [*]u8,
        max_buffer_size: i32,
        out_size: *i32,
    ) callconv(.c) Result,
};

/// Optional scratch allocator for a tile rebuild, reset once per tile.
pub const TileCacheAllocator = extern struct {
    user: ?*anyopaque,
    /// Called once at the start of every tile rebuild, before any allocation.
    reset: ?*const fn (user: ?*anyopaque) callconv(.c) void,
    allocate: ?*const fn (user: ?*anyopaque, size: usize) callconv(.c) ?*anyopaque,
    deallocate: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void,
};

/// What a mesh-process callback may change about a tile before it is built.
pub const TileCacheBuildParams = extern struct {
    /// One area id per polygon, writable.
    areas: [*]u8,
    /// One flag word per polygon, writable. Every entry arrives zero.
    flags: [*]u16,
    poly_count: i32,
    /// Copied into the tile's header, for a host to recognise it later.
    user_id: u32,
    /// Off-mesh connections for this tile, or NULL for none.
    connections: ?[*]const OffMeshConnection,
    connection_count: i32,
};

/// Called once per tile rebuild, after the polygons exist and before the tile
/// is built. NULL leaves every polygon at area 0 and flags 0.
pub const TileCacheMeshProcess = ?*const fn (user: ?*anyopaque, params: *TileCacheBuildParams) callconv(.c) Result;

/// What one compressed layer holds, copied out.
pub const CompressedTileInfo = extern struct {
    tile_x: i32,
    tile_y: i32,
    tile_layer: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    /// The layer's height range, in cells above `bmin[1]`.
    height_min: i32,
    height_max: i32,
    /// Cells along x and z.
    width: i32,
    height: i32,
    /// The sub-region holding data, in cells.
    min_x: i32,
    max_x: i32,
    min_z: i32,
    max_z: i32,
    /// Bytes the cache holds for this layer, header included.
    data_size: i32,
};

/// The three obstacle shapes, mirroring ObstacleType.
pub const ObstacleShape = enum(c_int) {
    cylinder = 0,
    box = 1,
    oriented_box = 2,
};

/// Where an obstacle is in the add/remove cycle, mirroring ObstacleState.
pub const ObstacleState = enum(c_int) {
    empty = 0,
    processing = 1,
    processed = 2,
    removing = 3,
};

pub const ObstacleInfo = extern struct {
    shape: ObstacleShape,
    state: ObstacleState,
    /// `.cylinder`: base centre, radius, height.
    position: [3]f32,
    radius: f32,
    height: f32,
    /// `.box`: the two corners.
    bmin: [3]f32,
    bmax: [3]f32,
    /// `.oriented_box`: centre, half extents, and the rotation.
    center: [3]f32,
    half_extents: [3]f32,
    y_radians: f32,
    /// How many tiles it overlaps, and how many of those still owe a rebuild.
    touched_count: i32,
    pending_count: i32,
};

/// A layer's identity and extent, as it crosses into and out of compression.
pub const TileCacheLayerHeader = extern struct {
    tile_x: i32,
    tile_y: i32,
    tile_layer: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    /// The layer's height range, in cells above `bmin[1]`.
    height_min: i32,
    height_max: i32,
    /// Cells along x and z.
    width: i32,
    height: i32,
    /// The sub-region holding usable data, in cells.
    min_x: i32,
    max_x: i32,
    min_z: i32,
    max_z: i32,
};

pub const TileCacheContourInfo = extern struct {
    vert_count: i32,
    region: u8,
    area: u8,
};

pub const TileCachePolyMeshInfo = extern struct {
    vert_count: i32,
    poly_count: i32,
    /// Corners per polygon, and half the stride of a `polys` entry. Always
    /// `verts_per_polygon`.
    verts_per_poly: i32,
};

pub const AbiLayout = extern struct {
    layout_size: u32,

    trimesh_size: u32,
    trimesh_align: u32,
    trimesh_offset_verts: u32,
    trimesh_offset_vert_count: u32,
    trimesh_offset_tris: u32,
    trimesh_offset_tri_count: u32,

    bake_config_size: u32,
    bake_config_align: u32,
    bake_config_field_count: u32,
    bake_config_offsets: [abi_max_fields]u32,

    bake_log_size: u32,
    bake_log_align: u32,
    bake_log_offset_buffer: u32,
    bake_log_offset_capacity: u32,

    query_filter_size: u32,
    query_filter_align: u32,
    query_filter_offset_area_cost: u32,
    query_filter_offset_include_flags: u32,
    query_filter_offset_exclude_flags: u32,

    raycast_hit_size: u32,
    raycast_hit_align: u32,
    raycast_hit_offset_t: u32,
    raycast_hit_offset_position: u32,
    raycast_hit_offset_normal: u32,
    raycast_hit_offset_hit: u32,

    allocator_size: u32,
    allocator_align: u32,
    allocator_offset_allocate: u32,
    allocator_offset_deallocate: u32,
    allocator_offset_user: u32,

    tile_grid_size: u32,
    tile_grid_align: u32,
    tile_grid_offset_origin: u32,
    tile_grid_offset_extent_max: u32,
    tile_grid_offset_tile_world_size: u32,
    tile_grid_offset_tile_count_x: u32,
    tile_grid_offset_tile_count_z: u32,

    area_volume_size: u32,
    area_volume_align: u32,
    area_volume_offset_shape: u32,
    area_volume_offset_area: u32,
    area_volume_offset_y_min: u32,
    area_volume_offset_y_max: u32,
    area_volume_offset_verts: u32,
    area_volume_offset_vert_count: u32,
    area_volume_offset_xz_min: u32,
    area_volume_offset_xz_max: u32,
    area_volume_offset_radius: u32,

    area_authoring_size: u32,
    area_authoring_align: u32,
    area_authoring_offset_volumes: u32,
    area_authoring_offset_volume_count: u32,
    area_authoring_offset_area_flags: u32,

    off_mesh_connection_size: u32,
    off_mesh_connection_align: u32,
    off_mesh_connection_offset_start: u32,
    off_mesh_connection_offset_end: u32,
    off_mesh_connection_offset_radius: u32,
    off_mesh_connection_offset_area: u32,
    off_mesh_connection_offset_flags: u32,
    off_mesh_connection_offset_bidirectional: u32,
    off_mesh_connection_offset_user_id: u32,
    off_mesh_connection_offset_end_side: u32,

    tile_authoring_size: u32,
    tile_authoring_align: u32,
    tile_authoring_offset_connections: u32,
    tile_authoring_offset_connection_count: u32,
    tile_authoring_offset_user_id: u32,
    tile_authoring_offset_skip_bv_tree: u32,

    nav_mesh_params_size: u32,
    nav_mesh_params_align: u32,
    nav_mesh_params_offset_origin: u32,
    nav_mesh_params_offset_tile_width: u32,
    nav_mesh_params_offset_tile_height: u32,
    nav_mesh_params_offset_max_tiles: u32,
    nav_mesh_params_offset_max_polys: u32,

    tile_info_size: u32,
    tile_info_align: u32,
    tile_info_field_count: u32,
    tile_info_offsets: [abi_max_fields]u32,

    poly_info_size: u32,
    poly_info_align: u32,
    poly_info_field_count: u32,
    poly_info_offsets: [abi_max_fields]u32,

    link_size: u32,
    link_align: u32,
    link_offset_ref: u32,
    link_offset_next: u32,
    link_offset_edge: u32,
    link_offset_side: u32,
    link_offset_bmin: u32,
    link_offset_bmax: u32,

    bv_node_size: u32,
    bv_node_align: u32,
    bv_node_offset_bmin: u32,
    bv_node_offset_bmax: u32,
    bv_node_offset_i: u32,

    poly_ref_size: u32,
    tile_ref_size: u32,
    result_count: u32,
    max_areas: u32,
    verts_per_polygon: u32,
    alloc_alignment: u32,

    assert_handler_size: u32,
    assert_handler_align: u32,
    assert_handler_offset_fail: u32,
    assert_handler_offset_user: u32,

    tile_layout_size: u32,
    tile_layout_align: u32,
    tile_layout_field_count: u32,
    tile_layout_offsets: [abi_max_fields]u32,

    // Appended after the freeze. New structs go here rather than beside the
    // block they are thematically closest to, so every earlier offset keeps
    // the value it already has.

    random_source_size: u32,
    random_source_align: u32,
    random_source_offset_next: u32,
    random_source_offset_user: u32,

    poly_query_size: u32,
    poly_query_align: u32,
    poly_query_offset_process: u32,
    poly_query_offset_user: u32,

    node_size: u32,
    node_align: u32,
    node_offset_pos: u32,
    node_offset_cost: u32,
    node_offset_total: u32,
    node_offset_ref: u32,
    node_offset_parent_index: u32,
    node_offset_state: u32,
    node_offset_flags: u32,

    node_pool_info_size: u32,
    node_pool_info_align: u32,
    node_pool_info_offset_node_count: u32,
    node_pool_info_offset_max_nodes: u32,
    node_pool_info_offset_hash_size: u32,
    node_pool_info_offset_bytes_used: u32,

    raycast_hit_offset_hit_edge_index: u32,
    raycast_hit_offset_path_cost: u32,

    // The staged Recast pipeline's structs.

    build_context_size: u32,
    build_context_align: u32,
    build_context_field_count: u32,
    build_context_offsets: [abi_max_fields]u32,

    span_size: u32,
    span_align: u32,
    span_offset_smin: u32,
    span_offset_smax: u32,
    span_offset_area: u32,

    heightfield_info_size: u32,
    heightfield_info_align: u32,
    heightfield_info_field_count: u32,
    heightfield_info_offsets: [abi_max_fields]u32,

    heightfield_storage_size: u32,
    heightfield_storage_align: u32,
    heightfield_storage_offset_pool_count: u32,
    heightfield_storage_offset_free_count: u32,
    heightfield_storage_offset_spans_per_pool: u32,

    compact_cell_size: u32,
    compact_cell_align: u32,
    compact_cell_offset_index: u32,
    compact_cell_offset_count: u32,

    compact_span_size: u32,
    compact_span_align: u32,
    compact_span_offset_y: u32,
    compact_span_offset_reg: u32,
    compact_span_offset_con: u32,
    compact_span_offset_h: u32,

    compact_heightfield_info_size: u32,
    compact_heightfield_info_align: u32,
    compact_heightfield_info_field_count: u32,
    compact_heightfield_info_offsets: [abi_max_fields]u32,

    contour_set_info_size: u32,
    contour_set_info_align: u32,
    contour_set_info_field_count: u32,
    contour_set_info_offsets: [abi_max_fields]u32,

    contour_info_size: u32,
    contour_info_align: u32,
    contour_info_offset_vert_count: u32,
    contour_info_offset_raw_vert_count: u32,
    contour_info_offset_region: u32,
    contour_info_offset_area: u32,

    poly_mesh_info_size: u32,
    poly_mesh_info_align: u32,
    poly_mesh_info_field_count: u32,
    poly_mesh_info_offsets: [abi_max_fields]u32,

    timer_label_count: u32,

    build_cells_size: u32,
    build_cells_align: u32,
    build_cells_field_count: u32,
    build_cells_offsets: [abi_max_fields]u32,

    // The layered heightfield and the tile cache.

    heightfield_layer_size: u32,
    heightfield_layer_align: u32,
    heightfield_layer_field_count: u32,
    heightfield_layer_offsets: [abi_max_fields]u32,

    tile_cache_params_size: u32,
    tile_cache_params_align: u32,
    tile_cache_params_field_count: u32,
    tile_cache_params_offsets: [abi_max_fields]u32,

    tile_cache_compressor_size: u32,
    tile_cache_compressor_align: u32,
    tile_cache_compressor_field_count: u32,
    tile_cache_compressor_offsets: [abi_max_fields]u32,

    tile_cache_allocator_size: u32,
    tile_cache_allocator_align: u32,
    tile_cache_allocator_field_count: u32,
    tile_cache_allocator_offsets: [abi_max_fields]u32,

    tile_cache_build_params_size: u32,
    tile_cache_build_params_align: u32,
    tile_cache_build_params_field_count: u32,
    tile_cache_build_params_offsets: [abi_max_fields]u32,

    compressed_tile_info_size: u32,
    compressed_tile_info_align: u32,
    compressed_tile_info_field_count: u32,
    compressed_tile_info_offsets: [abi_max_fields]u32,

    obstacle_info_size: u32,
    obstacle_info_align: u32,
    obstacle_info_field_count: u32,
    obstacle_info_offsets: [abi_max_fields]u32,

    tile_cache_layer_header_size: u32,
    tile_cache_layer_header_align: u32,
    tile_cache_layer_header_field_count: u32,
    tile_cache_layer_header_offsets: [abi_max_fields]u32,

    tile_cache_contour_info_size: u32,
    tile_cache_contour_info_align: u32,
    tile_cache_contour_info_offset_vert_count: u32,
    tile_cache_contour_info_offset_region: u32,
    tile_cache_contour_info_offset_area: u32,

    tile_cache_poly_mesh_info_size: u32,
    tile_cache_poly_mesh_info_align: u32,
    tile_cache_poly_mesh_info_offset_vert_count: u32,
    tile_cache_poly_mesh_info_offset_poly_count: u32,
    tile_cache_poly_mesh_info_offset_verts_per_poly: u32,

    compressed_tile_ref_size: u32,
    obstacle_ref_size: u32,

    crowd_agent_params_size: u32,
    crowd_agent_params_align: u32,
    crowd_agent_params_field_count: u32,
    crowd_agent_params_offsets: [abi_max_fields]u32,

    crowd_agent_size: u32,
    crowd_agent_align: u32,
    crowd_agent_field_count: u32,
    crowd_agent_offsets: [abi_max_fields]u32,

    crowd_corner_size: u32,
    crowd_corner_align: u32,
    crowd_corner_field_count: u32,
    crowd_corner_offsets: [abi_max_fields]u32,

    crowd_neighbour_size: u32,
    crowd_neighbour_align: u32,
    crowd_neighbour_field_count: u32,
    crowd_neighbour_offsets: [abi_max_fields]u32,

    crowd_agent_animation_size: u32,
    crowd_agent_animation_align: u32,
    crowd_agent_animation_field_count: u32,
    crowd_agent_animation_offsets: [abi_max_fields]u32,

    crowd_agent_debug_size: u32,
    crowd_agent_debug_align: u32,
    crowd_agent_debug_field_count: u32,
    crowd_agent_debug_offsets: [abi_max_fields]u32,

    avoidance_params_size: u32,
    avoidance_params_align: u32,
    avoidance_params_field_count: u32,
    avoidance_params_offsets: [abi_max_fields]u32,

    avoidance_circle_size: u32,
    avoidance_circle_align: u32,
    avoidance_circle_field_count: u32,
    avoidance_circle_offsets: [abi_max_fields]u32,

    avoidance_segment_size: u32,
    avoidance_segment_align: u32,
    avoidance_segment_field_count: u32,
    avoidance_segment_offsets: [abi_max_fields]u32,

    avoidance_sample_size: u32,
    avoidance_sample_align: u32,
    avoidance_sample_field_count: u32,
    avoidance_sample_offsets: [abi_max_fields]u32,

    path_corridor_info_size: u32,
    path_corridor_info_align: u32,
    path_corridor_info_field_count: u32,
    path_corridor_info_offsets: [abi_max_fields]u32,

    agent_ref_size: u32,
    path_request_ref_size: u32,
};

//=============================================================================
// Opaque handles
//=============================================================================

pub const PolyMesh = opaque {};
pub const Heightfield = opaque {};
pub const CompactHeightfield = opaque {};
pub const ContourSet = opaque {};
pub const NavMesh = opaque {};
pub const NavMeshQuery = opaque {};
pub const HeightfieldLayerSet = opaque {};
pub const TileCache = opaque {};
pub const TileCacheLayer = opaque {};
pub const TileCacheContourSet = opaque {};
pub const TileCachePolyMesh = opaque {};

//=============================================================================
// Entry points
//=============================================================================

pub extern fn zrcVersion() u32;
pub extern fn zrcRecastVersion() u32;
pub extern fn zrcNavMeshDataVersion() i32;
pub extern fn zrcResultName(result: Result) [*:0]const u8;
pub extern fn zrcSetAllocator(alloc: ?*const Allocator) Result;
pub extern fn zrcAlloc(size: usize, hint: AllocHint) ?*anyopaque;
pub extern fn zrcFree(block: ?*anyopaque) void;
pub extern fn zrcAbiLayout(out: *AbiLayout) void;

pub extern fn zrcAssertsEnabled() Bool;
pub extern fn zrcSetAssertHandler(handler: ?*const AssertHandler) Result;
pub extern fn zrcAssertHandler(out: *AssertHandler) Result;

pub extern fn zrcBakeConfigDefault(out: *BakeConfig) void;
pub extern fn zrcPolyMeshBake(
    config: *const BakeConfig,
    mesh: *const TriMesh,
    authoring: ?*const AreaAuthoring,
    log: ?*BakeLog,
    out: **PolyMesh,
) Result;
pub extern fn zrcBakeConfigCells(config: *const BakeConfig, out: *BuildCells) Result;

pub extern fn zrcTileGridCompute(
    config: *const BakeConfig,
    mesh: *const TriMesh,
    out: *TileGrid,
) Result;
pub extern fn zrcPolyMeshBakeTile(
    config: *const BakeConfig,
    mesh: *const TriMesh,
    grid: *const TileGrid,
    tile_x: i32,
    tile_z: i32,
    authoring: ?*const AreaAuthoring,
    log: ?*BakeLog,
    out: *?*PolyMesh,
) Result;
pub extern fn zrcPolyMeshDestroy(mesh: ?*PolyMesh) void;

pub extern fn zrcBuildContextLog(
    context: ?*const BuildContext,
    category: LogCategory,
    message: [*:0]const u8,
) Result;
pub extern fn zrcBuildContextResetLog(context: ?*const BuildContext) Result;
pub extern fn zrcBuildContextResetTimers(context: ?*const BuildContext) Result;
pub extern fn zrcBuildContextStartTimer(context: ?*const BuildContext, label: TimerLabel) Result;
pub extern fn zrcBuildContextStopTimer(context: ?*const BuildContext, label: TimerLabel) Result;
pub extern fn zrcBuildContextAccumulatedTime(
    context: ?*const BuildContext,
    label: TimerLabel,
    out_time: *i32,
) Result;

//=============================================================================
// Sizing a build
//=============================================================================

pub extern fn zrcCalcBounds(mesh: *const TriMesh, bmin: *[3]f32, bmax: *[3]f32) Result;
pub extern fn zrcCalcGridSize(
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    cell_size: f32,
    out_width: *i32,
    out_height: *i32,
) Result;
pub extern fn zrcMarkWalkableTriangles(
    context: ?*const BuildContext,
    walkable_slope_angle: f32,
    mesh: *const TriMesh,
    out_areas: [*]u8,
) Result;
pub extern fn zrcClearUnwalkableTriangles(
    context: ?*const BuildContext,
    walkable_slope_angle: f32,
    mesh: *const TriMesh,
    io_areas: [*]u8,
) Result;

//=============================================================================
// The heightfield
//=============================================================================

pub extern fn zrcHeightfieldCreate(
    context: ?*const BuildContext,
    width: i32,
    height: i32,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    cell_size: f32,
    cell_height: f32,
    out: **Heightfield,
) Result;
pub extern fn zrcHeightfieldDestroy(heightfield: ?*Heightfield) void;
pub extern fn zrcHeightfieldInfo(heightfield: *const Heightfield, out: *HeightfieldInfo) Result;
pub extern fn zrcHeightfieldStorage(heightfield: *const Heightfield, out: *HeightfieldStorage) Result;
pub extern fn zrcHeightfieldColumn(
    heightfield: *const Heightfield,
    x: i32,
    z: i32,
    out: ?[*]Span,
    max_spans: i32,
    out_count: *i32,
) Result;
pub extern fn zrcHeightfieldSpanCount(
    context: ?*const BuildContext,
    heightfield: *const Heightfield,
    out_count: *i32,
) Result;
pub extern fn zrcHeightfieldAddSpan(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    x: i32,
    z: i32,
    span_min: u32,
    span_max: u32,
    area: u8,
    flag_merge_threshold: i32,
) Result;
pub extern fn zrcHeightfieldRasterizeTriangle(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    v0: *const [3]f32,
    v1: *const [3]f32,
    v2: *const [3]f32,
    area: u8,
    flag_merge_threshold: i32,
) Result;
pub extern fn zrcHeightfieldRasterizeTriangles(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    mesh: *const TriMesh,
    tri_areas: [*]const u8,
    flag_merge_threshold: i32,
) Result;
pub extern fn zrcHeightfieldRasterizeTrianglesU16(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    verts: [*]const f32,
    vert_count: i32,
    tris: [*]const u16,
    tri_areas: [*]const u8,
    tri_count: i32,
    flag_merge_threshold: i32,
) Result;
pub extern fn zrcHeightfieldRasterizeTriangleSoup(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    verts: [*]const f32,
    tri_areas: [*]const u8,
    tri_count: i32,
    flag_merge_threshold: i32,
) Result;
pub extern fn zrcHeightfieldFilterLowHangingObstacles(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    walkable_climb: i32,
) Result;
pub extern fn zrcHeightfieldFilterLedgeSpans(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    walkable_height: i32,
    walkable_climb: i32,
) Result;
pub extern fn zrcHeightfieldFilterWalkableLowHeightSpans(
    context: ?*const BuildContext,
    heightfield: *Heightfield,
    walkable_height: i32,
) Result;

//=============================================================================
// The compact heightfield
//=============================================================================

pub extern fn zrcCompactHeightfieldCreate(
    context: ?*const BuildContext,
    walkable_height: i32,
    walkable_climb: i32,
    heightfield: *const Heightfield,
    out: **CompactHeightfield,
) Result;
pub extern fn zrcCompactHeightfieldDestroy(field: ?*CompactHeightfield) void;
pub extern fn zrcCompactHeightfieldInfo(
    field: *const CompactHeightfield,
    out: *CompactHeightfieldInfo,
) Result;
pub extern fn zrcCompactHeightfieldCells(
    field: *const CompactHeightfield,
    first: i32,
    count: i32,
    out: ?[*]CompactCell,
) Result;
pub extern fn zrcCompactHeightfieldSpans(
    field: *const CompactHeightfield,
    first: i32,
    count: i32,
    out: ?[*]CompactSpan,
) Result;
pub extern fn zrcCompactHeightfieldSetSpans(
    field: *CompactHeightfield,
    first: i32,
    count: i32,
    spans: [*]const CompactSpan,
) Result;
pub extern fn zrcCompactHeightfieldDistances(
    field: *const CompactHeightfield,
    first: i32,
    count: i32,
    out: ?[*]u16,
) Result;
pub extern fn zrcCompactHeightfieldAreas(
    field: *const CompactHeightfield,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcCompactHeightfieldSetAreas(
    field: *CompactHeightfield,
    first: i32,
    count: i32,
    areas: [*]const u8,
) Result;
pub extern fn zrcCompactHeightfieldMarkAreas(
    context: ?*const BuildContext,
    field: *CompactHeightfield,
    authoring: *const AreaAuthoring,
) Result;
pub extern fn zrcCompactHeightfieldErode(
    context: ?*const BuildContext,
    field: *CompactHeightfield,
    radius: i32,
) Result;
pub extern fn zrcCompactHeightfieldMedianFilter(
    context: ?*const BuildContext,
    field: *CompactHeightfield,
) Result;
pub extern fn zrcCompactHeightfieldBuildDistanceField(
    context: ?*const BuildContext,
    field: *CompactHeightfield,
) Result;
pub extern fn zrcCompactHeightfieldBuildRegions(
    context: ?*const BuildContext,
    field: *CompactHeightfield,
    partition: Partition,
    border_size: i32,
    min_region_area: i32,
    merge_region_area: i32,
) Result;

//=============================================================================
// Contours
//=============================================================================

pub extern fn zrcContourSetCreate(
    context: ?*const BuildContext,
    field: *const CompactHeightfield,
    max_error: f32,
    max_edge_len: i32,
    flags: i32,
    out: **ContourSet,
) Result;
pub extern fn zrcContourSetDestroy(contours: ?*ContourSet) void;
pub extern fn zrcContourSetInfo(contours: *const ContourSet, out: *ContourSetInfo) Result;
pub extern fn zrcContourAt(contours: *const ContourSet, index: i32, out: *ContourInfo) Result;
pub extern fn zrcContourVerts(
    contours: *const ContourSet,
    index: i32,
    first: i32,
    count: i32,
    out: ?[*]i32,
) Result;
pub extern fn zrcContourRawVerts(
    contours: *const ContourSet,
    index: i32,
    first: i32,
    count: i32,
    out: ?[*]i32,
) Result;

//=============================================================================
// The polygon mesh, built by hand
//=============================================================================

pub extern fn zrcPolyMeshCreate(out: **PolyMesh) Result;
pub extern fn zrcPolyMeshBuild(
    context: ?*const BuildContext,
    contours: *const ContourSet,
    verts_per_poly: i32,
    mesh: *PolyMesh,
) Result;
pub extern fn zrcPolyMeshBuildDetail(
    context: ?*const BuildContext,
    mesh: *PolyMesh,
    field: *const CompactHeightfield,
    sample_dist: f32,
    sample_max_error: f32,
) Result;
pub extern fn zrcPolyMeshSetAgentDims(
    mesh: *PolyMesh,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
) Result;
pub extern fn zrcPolyMeshInfo(mesh: *const PolyMesh, out: *PolyMeshInfo) Result;
pub extern fn zrcPolyMeshCopy(
    context: ?*const BuildContext,
    src: *const PolyMesh,
    dst: *PolyMesh,
) Result;
pub extern fn zrcPolyMeshMerge(
    context: ?*const BuildContext,
    meshes: [*]const *const PolyMesh,
    count: i32,
    out: *PolyMesh,
) Result;
pub extern fn zrcPolyMeshVerts(mesh: *const PolyMesh, first: i32, count: i32, out: ?[*]u16) Result;
pub extern fn zrcPolyMeshPolys(mesh: *const PolyMesh, first: i32, count: i32, out: ?[*]u16) Result;
pub extern fn zrcPolyMeshRegions(mesh: *const PolyMesh, first: i32, count: i32, out: ?[*]u16) Result;
pub extern fn zrcPolyMeshPolyAreas(mesh: *const PolyMesh, first: i32, count: i32, out: ?[*]u8) Result;
pub extern fn zrcPolyMeshSetPolyAreas(
    mesh: *PolyMesh,
    first: i32,
    count: i32,
    areas: [*]const u8,
) Result;
pub extern fn zrcPolyMeshPolyFlags(mesh: *const PolyMesh, first: i32, count: i32, out: ?[*]u16) Result;
pub extern fn zrcPolyMeshSetPolyFlags(
    mesh: *PolyMesh,
    first: i32,
    count: i32,
    flags: [*]const u16,
) Result;
pub extern fn zrcPolyMeshDetailMeshes(
    mesh: *const PolyMesh,
    first: i32,
    count: i32,
    out: ?[*]u32,
) Result;
pub extern fn zrcPolyMeshDetailVerts(
    mesh: *const PolyMesh,
    first: i32,
    count: i32,
    out: ?[*]f32,
) Result;
pub extern fn zrcPolyMeshDetailTris(
    mesh: *const PolyMesh,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;

pub extern fn zrcNavMeshCreate(
    mesh: *const PolyMesh,
    authoring: ?*const TileAuthoring,
    out: **NavMesh,
) Result;
pub extern fn zrcNavMeshDestroy(navmesh: ?*NavMesh) void;
pub extern fn zrcNavMeshSerialize(
    navmesh: *const NavMesh,
    out_data: *?*anyopaque,
    out_size: *usize,
) Result;
pub extern fn zrcNavMeshDeserialize(data: ?*const anyopaque, size: usize, out: **NavMesh) Result;
pub extern fn zrcNavMeshValidate(data: ?*const anyopaque, size: usize) Result;
pub extern fn zrcTileLayout(data: ?*const anyopaque, size: usize, out: *TileLayout) Result;

pub extern fn zrcNavMeshImageSwapEndian(
    data: ?*const anyopaque,
    size: usize,
    from_native: Bool,
    out_data: *?*anyopaque,
    out_size: *usize,
) Result;
pub extern fn zrcNavMeshPolyCount(navmesh: ?*const NavMesh) i32;
pub extern fn zrcNavMeshBounds(navmesh: ?*const NavMesh, bmin: *[3]f32, bmax: *[3]f32) Result;

pub extern fn zrcNavMeshCreateTiled(
    grid: *const TileGrid,
    max_tiles: i32,
    max_polys_per_tile: i32,
    out: **NavMesh,
) Result;
pub extern fn zrcTileDataBuild(
    mesh: *const PolyMesh,
    tile_x: i32,
    tile_z: i32,
    tile_layer: i32,
    authoring: ?*const TileAuthoring,
    out_data: *?*anyopaque,
    out_size: *usize,
) Result;
pub extern fn zrcNavMeshAddTile(
    navmesh: ?*NavMesh,
    data: ?*const anyopaque,
    size: usize,
    out_ref: ?*TileRef,
) Result;
pub extern fn zrcNavMeshRemoveTile(navmesh: ?*NavMesh, ref: TileRef) Result;
pub extern fn zrcNavMeshTileRefAt(
    navmesh: ?*const NavMesh,
    tile_x: i32,
    tile_z: i32,
    tile_layer: i32,
    out_ref: *TileRef,
) Result;
pub extern fn zrcNavMeshTileRefsAt(
    navmesh: *const NavMesh,
    tile_x: i32,
    tile_z: i32,
    out: ?[*]TileRef,
    max_tiles: i32,
    out_count: *i32,
) Result;

pub extern fn zrcNavMeshTileCount(navmesh: ?*const NavMesh) i32;
pub extern fn zrcNavMeshTileRefAtIndex(
    navmesh: ?*const NavMesh,
    index: i32,
    out_ref: *TileRef,
) Result;
pub extern fn zrcNavMeshMaxTiles(navmesh: ?*const NavMesh) i32;
pub extern fn zrcNavMeshTileBounds(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    bmin: *[3]f32,
    bmax: *[3]f32,
) Result;

pub extern fn zrcNavMeshGetPolyArea(
    navmesh: ?*const NavMesh,
    ref: PolyRef,
    out_area: *i32,
) Result;
pub extern fn zrcNavMeshSetPolyArea(navmesh: ?*NavMesh, ref: PolyRef, area: i32) Result;
pub extern fn zrcNavMeshGetPolyFlags(
    navmesh: ?*const NavMesh,
    ref: PolyRef,
    out_flags: *u16,
) Result;
pub extern fn zrcNavMeshSetPolyFlags(navmesh: ?*NavMesh, ref: PolyRef, flags: u16) Result;
pub extern fn zrcNavMeshGetPolyType(navmesh: ?*const NavMesh, ref: PolyRef, out_type: *i32) Result;

pub extern fn zrcNavMeshOffMeshConnectionEndPoints(
    navmesh: ?*const NavMesh,
    prev_ref: PolyRef,
    poly_ref: PolyRef,
    out_start: *[3]f32,
    out_end: *[3]f32,
) Result;
pub extern fn zrcNavMeshOffMeshConnection(
    navmesh: ?*const NavMesh,
    ref: PolyRef,
    out: *OffMeshConnection,
) Result;
pub extern fn zrcNavMeshTileUserId(navmesh: ?*const NavMesh, ref: TileRef, out_user_id: *u32) Result;

pub extern fn zrcNavMeshParams(navmesh: ?*const NavMesh, out: *NavMeshParams) Result;
pub extern fn zrcNavMeshCalcTileLoc(
    navmesh: ?*const NavMesh,
    pos: *const [3]f32,
    out_tile_x: *i32,
    out_tile_z: *i32,
) Result;
pub extern fn zrcOppositeTileSide(side: i32, out_side: *i32) Result;
pub extern fn zrcNavMeshTileInfo(navmesh: ?*const NavMesh, ref: TileRef, out: *TileInfo) Result;
pub extern fn zrcNavMeshPolyInfo(navmesh: ?*const NavMesh, ref: PolyRef, out: *PolyInfo) Result;
pub extern fn zrcNavMeshTilePolyRef(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    index: i32,
    out: *PolyRef,
) Result;
pub extern fn zrcNavMeshTileLink(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    index: i32,
    out: *Link,
) Result;
pub extern fn zrcNavMeshTileBvNode(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    index: i32,
    out: *BvNode,
) Result;
pub extern fn zrcNavMeshTileVerts(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    first: i32,
    count: i32,
    out: ?[*]f32,
) Result;
pub extern fn zrcNavMeshTileDetailVerts(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    first: i32,
    count: i32,
    out: ?[*]f32,
) Result;
pub extern fn zrcNavMeshTileDetailTris(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;

pub extern fn zrcNavMeshTileStateSize(navmesh: ?*const NavMesh, ref: TileRef, out_size: *usize) Result;
pub extern fn zrcNavMeshStoreTileState(
    navmesh: ?*const NavMesh,
    ref: TileRef,
    data: ?*anyopaque,
    size: usize,
) Result;
pub extern fn zrcNavMeshRestoreTileState(
    navmesh: ?*NavMesh,
    ref: TileRef,
    data: ?*const anyopaque,
    size: usize,
) Result;

pub extern fn zrcClosestPointOnTriangle(
    point: *const [3]f32,
    a: *const [3]f32,
    b: *const [3]f32,
    c: *const [3]f32,
    out_closest: *[3]f32,
) Result;
pub extern fn zrcClosestHeightPointTriangle(
    point: *const [3]f32,
    a: *const [3]f32,
    b: *const [3]f32,
    c: *const [3]f32,
    out_height: *f32,
    out_inside: *Bool,
) Result;
pub extern fn zrcDistancePointToSegment2D(
    point: *const [3]f32,
    p: *const [3]f32,
    q: *const [3]f32,
    out_dist_sqr: *f32,
    out_t: *f32,
) Result;
pub extern fn zrcDistancePointToPolyEdges(
    point: *const [3]f32,
    verts: [*]const f32,
    vert_count: i32,
    out_edge_dist_sqr: [*]f32,
    out_edge_t: [*]f32,
    out_inside: *Bool,
) Result;
pub extern fn zrcPointInPolygon(
    point: *const [3]f32,
    verts: [*]const f32,
    vert_count: i32,
    out_inside: *Bool,
) Result;
pub extern fn zrcIntersectSegmentPoly2D(
    p0: *const [3]f32,
    p1: *const [3]f32,
    verts: [*]const f32,
    vert_count: i32,
    out_t_min: *f32,
    out_t_max: *f32,
    out_seg_min: *i32,
    out_seg_max: *i32,
    out_intersects: *Bool,
) Result;
pub extern fn zrcIntersectSegSeg2D(
    ap: *const [3]f32,
    aq: *const [3]f32,
    bp: *const [3]f32,
    bq: *const [3]f32,
    out_s: *f32,
    out_t: *f32,
    out_intersects: *Bool,
) Result;
pub extern fn zrcOverlapPolyPoly2D(
    poly_a: [*]const f32,
    count_a: i32,
    poly_b: [*]const f32,
    count_b: i32,
    out_overlap: *Bool,
) Result;
pub extern fn zrcOverlapBounds(
    amin: *const [3]f32,
    amax: *const [3]f32,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    out_overlap: *Bool,
) Result;
pub extern fn zrcOverlapQuantBounds(
    amin: *const [3]u16,
    amax: *const [3]u16,
    bmin: *const [3]u16,
    bmax: *const [3]u16,
    out_overlap: *Bool,
) Result;
pub extern fn zrcTriArea2D(
    a: *const [3]f32,
    b: *const [3]f32,
    c: *const [3]f32,
    out_area: *f32,
) Result;
pub extern fn zrcPolyCenter(
    verts: [*]const f32,
    vert_count: i32,
    indices: [*]const u16,
    index_count: i32,
    out_center: *[3]f32,
) Result;
pub extern fn zrcRandomPointInConvexPoly(
    verts: [*]const f32,
    vert_count: i32,
    scratch: [*]f32,
    scratch_count: i32,
    s: f32,
    t: f32,
    out_point: *[3]f32,
) Result;
pub extern fn zrcOffsetPoly(
    verts: [*]const f32,
    vert_count: i32,
    offset: f32,
    out_verts: [*]f32,
    max_out_verts: i32,
    out_vert_count: *i32,
) Result;

pub extern fn zrcQueryFilterDefault(out: *QueryFilter) void;
pub extern fn zrcNavMeshQueryCreate(
    navmesh: *const NavMesh,
    max_nodes: i32,
    out: **NavMeshQuery,
) Result;
pub extern fn zrcNavMeshQueryDestroy(query: ?*NavMeshQuery) void;

pub extern fn zrcFindNearestPoly(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    out_ref: *PolyRef,
    out_point: ?*[3]f32,
    out_over_poly: ?*Bool,
) Result;

pub extern fn zrcFindPath(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    out_path: [*]PolyRef,
    max_path: i32,
    out_count: *i32,
    out_partial: ?*Bool,
) Result;

pub extern fn zrcFindStraightPath(
    query: *const NavMeshQuery,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    path: [*]const PolyRef,
    path_count: i32,
    options: u32,
    out_points: [*]f32,
    max_points: i32,
    out_flags: ?[*]u8,
    max_flags: i32,
    out_refs: ?[*]PolyRef,
    max_refs: i32,
    out_count: *i32,
    out_partial: ?*Bool,
) Result;

pub extern fn zrcMoveAlongSurface(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    out_pos: *[3]f32,
    out_visited: ?[*]PolyRef,
    max_visited: i32,
    out_visited_count: ?*i32,
    out_truncated: ?*Bool,
) Result;

pub extern fn zrcRaycast(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    options: u32,
    prev_ref: PolyRef,
    out_hit: *RaycastHit,
    out_path: ?[*]PolyRef,
    max_path: i32,
    out_path_count: ?*i32,
    out_truncated: ?*Bool,
) Result;

//=============================================================================
// Sliced pathfinding
//=============================================================================

pub extern fn zrcSlicedFindPathInit(
    query: *NavMeshQuery,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    options: u32,
) Result;
pub extern fn zrcSlicedFindPathUpdate(
    query: *NavMeshQuery,
    max_iters: i32,
    out_iters: ?*i32,
    out_in_progress: ?*Bool,
) Result;
pub extern fn zrcSlicedFindPathFinalize(
    query: *NavMeshQuery,
    out_path: [*]PolyRef,
    max_path: i32,
    out_count: *i32,
    out_partial: ?*Bool,
) Result;
pub extern fn zrcSlicedFindPathFinalizePartial(
    query: *NavMeshQuery,
    existing: [*]const PolyRef,
    existing_count: i32,
    out_path: [*]PolyRef,
    max_path: i32,
    out_count: *i32,
    out_partial: ?*Bool,
) Result;
pub extern fn zrcSlicedFindPathCancel(query: *NavMeshQuery) Result;
pub extern fn zrcSlicedFindPathActive(query: ?*const NavMeshQuery) Bool;

//=============================================================================
// Random points
//=============================================================================

pub extern fn zrcFindRandomPoint(
    query: *const NavMeshQuery,
    filter: *const QueryFilter,
    random: *const RandomSource,
    out_ref: *PolyRef,
    out_point: *[3]f32,
) Result;
pub extern fn zrcFindRandomPointAroundCircle(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center: *const [3]f32,
    max_radius: f32,
    filter: *const QueryFilter,
    random: *const RandomSource,
    out_ref: *PolyRef,
    out_point: *[3]f32,
) Result;

//=============================================================================
// Polygons in a box
//=============================================================================

pub extern fn zrcQueryPolygons(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    out_refs: [*]PolyRef,
    max_refs: i32,
    out_count: *i32,
    out_truncated: ?*Bool,
) Result;
pub extern fn zrcQueryPolygonsBatched(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    sink: *const PolyQuery,
) Result;

//=============================================================================
// A point against one polygon
//=============================================================================

pub extern fn zrcClosestPointOnPoly(
    query: *const NavMeshQuery,
    ref: PolyRef,
    pos: *const [3]f32,
    out_point: *[3]f32,
    out_over_poly: ?*Bool,
) Result;
pub extern fn zrcClosestPointOnPolyBoundary(
    query: *const NavMeshQuery,
    ref: PolyRef,
    pos: *const [3]f32,
    out_point: *[3]f32,
) Result;
pub extern fn zrcPolyHeight(
    query: *const NavMeshQuery,
    ref: PolyRef,
    pos: *const [3]f32,
    out_height: *f32,
) Result;

//=============================================================================
// Searching outwards
//=============================================================================

pub extern fn zrcFindPolysAroundCircle(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center: *const [3]f32,
    radius: f32,
    filter: *const QueryFilter,
    out_refs: ?[*]PolyRef,
    out_parents: ?[*]PolyRef,
    out_costs: ?[*]f32,
    max_result: i32,
    out_count: *i32,
    out_truncated: ?*Bool,
) Result;
pub extern fn zrcFindPolysAroundShape(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    verts: [*]const f32,
    vert_count: i32,
    filter: *const QueryFilter,
    out_refs: ?[*]PolyRef,
    out_parents: ?[*]PolyRef,
    out_costs: ?[*]f32,
    max_result: i32,
    out_count: *i32,
    out_truncated: ?*Bool,
) Result;
pub extern fn zrcPathFromDijkstraSearch(
    query: *const NavMeshQuery,
    end_ref: PolyRef,
    out_path: [*]PolyRef,
    max_path: i32,
    out_count: *i32,
    out_truncated: ?*Bool,
) Result;
pub extern fn zrcFindLocalNeighbourhood(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center: *const [3]f32,
    radius: f32,
    filter: *const QueryFilter,
    out_refs: [*]PolyRef,
    out_parents: ?[*]PolyRef,
    max_result: i32,
    out_count: *i32,
    out_truncated: ?*Bool,
) Result;

//=============================================================================
// Walls
//=============================================================================

pub extern fn zrcPolyWallSegments(
    query: *const NavMeshQuery,
    ref: PolyRef,
    filter: *const QueryFilter,
    out_verts: [*]f32,
    out_refs: ?[*]PolyRef,
    max_segments: i32,
    out_count: *i32,
    out_truncated: ?*Bool,
) Result;
pub extern fn zrcFindDistanceToWall(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center: *const [3]f32,
    max_radius: f32,
    filter: *const QueryFilter,
    out_dist: *f32,
    out_pos: *[3]f32,
    out_normal: *[3]f32,
    out_found: *Bool,
) Result;

//=============================================================================
// What a reference means
//=============================================================================

pub extern fn zrcIsValidPolyRef(
    query: *const NavMeshQuery,
    ref: PolyRef,
    filter: ?*const QueryFilter,
    out_valid: *Bool,
) Result;
pub extern fn zrcIsInClosedList(
    query: *const NavMeshQuery,
    ref: PolyRef,
    out_closed: *Bool,
) Result;
pub extern fn zrcQueryNavMesh(
    query: *const NavMeshQuery,
    out_navmesh: **const NavMesh,
) Result;
pub extern fn zrcDecodePolyRef(
    navmesh: *const NavMesh,
    ref: PolyRef,
    out_salt: ?*u32,
    out_tile: ?*u32,
    out_poly: ?*u32,
) Result;
pub extern fn zrcEncodePolyRef(
    navmesh: *const NavMesh,
    salt: u32,
    tile: u32,
    poly: u32,
    out_ref: *PolyRef,
) Result;

//=============================================================================
// The search's own node pool
//=============================================================================

pub extern fn zrcQueryNodePoolInfo(
    query: *const NavMeshQuery,
    out: *NodePoolInfo,
) Result;
pub extern fn zrcQueryFindNode(
    query: *const NavMeshQuery,
    ref: PolyRef,
    state: u32,
    out: *Node,
) Result;
pub extern fn zrcQueryFindNodes(
    query: *const NavMeshQuery,
    ref: PolyRef,
    out: [*]Node,
    max_nodes: i32,
    out_count: *i32,
) Result;
pub extern fn zrcQueryNodeAt(
    query: *const NavMeshQuery,
    index: u32,
    out: *Node,
) Result;

//=============================================================================
// The layered heightfield
//=============================================================================

pub extern fn zrcHeightfieldLayerSetCreate(
    context: ?*const BuildContext,
    field: *const CompactHeightfield,
    border_size: i32,
    walkable_height: i32,
    out: **HeightfieldLayerSet,
) Result;
pub extern fn zrcHeightfieldLayerSetDestroy(layers: ?*HeightfieldLayerSet) void;
pub extern fn zrcHeightfieldLayerSetCount(layers: *const HeightfieldLayerSet, out_count: *i32) Result;
pub extern fn zrcHeightfieldLayerAt(
    layers: *const HeightfieldLayerSet,
    index: i32,
    out: *HeightfieldLayer,
) Result;
pub extern fn zrcHeightfieldLayerHeights(
    layers: *const HeightfieldLayerSet,
    index: i32,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcHeightfieldLayerAreas(
    layers: *const HeightfieldLayerSet,
    index: i32,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcHeightfieldLayerCons(
    layers: *const HeightfieldLayerSet,
    index: i32,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;

//=============================================================================
// The tile cache — obstacles carved into a baked mesh at runtime
//=============================================================================

pub extern fn zrcTileCacheCreate(
    params: *const TileCacheParams,
    compressor: *const TileCacheCompressor,
    allocator: ?*const TileCacheAllocator,
    mesh_process: TileCacheMeshProcess,
    mesh_process_user: ?*anyopaque,
    out: **TileCache,
) Result;
pub extern fn zrcTileCacheDestroy(cache: ?*TileCache) void;
pub extern fn zrcTileCacheParams(cache: *const TileCache, out: *TileCacheParams) Result;

pub extern fn zrcTileCacheAddTile(
    cache: *TileCache,
    data: ?*const anyopaque,
    size: usize,
    out_ref: *CompressedTileRef,
) Result;
pub extern fn zrcTileCacheRemoveTile(
    cache: *TileCache,
    ref: CompressedTileRef,
    out_data: ?*?*anyopaque,
    out_size: ?*usize,
) Result;

pub extern fn zrcTileCacheTileInfo(
    cache: *const TileCache,
    ref: CompressedTileRef,
    out: *CompressedTileInfo,
) Result;
pub extern fn zrcTileCacheTileAt(
    cache: *const TileCache,
    tile_x: i32,
    tile_y: i32,
    tile_layer: i32,
    out_ref: *CompressedTileRef,
) Result;
pub extern fn zrcTileCacheTilesAt(
    cache: *const TileCache,
    tile_x: i32,
    tile_y: i32,
    out: [*]CompressedTileRef,
    max_tiles: i32,
    out_count: *i32,
) Result;
pub extern fn zrcTileCacheTileRefAt(
    cache: *const TileCache,
    index: i32,
    out_ref: *CompressedTileRef,
) Result;

pub extern fn zrcTileCacheObstacleInfo(
    cache: *const TileCache,
    ref: ObstacleRef,
    out: *ObstacleInfo,
) Result;
pub extern fn zrcTileCacheObstacleRefAt(
    cache: *const TileCache,
    index: i32,
    out_ref: *ObstacleRef,
) Result;

pub extern fn zrcTileCacheAddCylinderObstacle(
    cache: *TileCache,
    position: *const [3]f32,
    radius: f32,
    height: f32,
    out_ref: *ObstacleRef,
) Result;
pub extern fn zrcTileCacheAddBoxObstacle(
    cache: *TileCache,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    out_ref: *ObstacleRef,
) Result;
pub extern fn zrcTileCacheAddOrientedBoxObstacle(
    cache: *TileCache,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    y_radians: f32,
    out_ref: *ObstacleRef,
) Result;
pub extern fn zrcTileCacheRemoveObstacle(cache: *TileCache, ref: ObstacleRef) Result;

pub extern fn zrcTileCacheQueryTiles(
    cache: *const TileCache,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    out: [*]CompressedTileRef,
    max_tiles: i32,
    out_count: *i32,
) Result;

pub extern fn zrcTileCacheUpdate(
    cache: *TileCache,
    navmesh: *NavMesh,
    out_up_to_date: ?*Bool,
) Result;
pub extern fn zrcTileCacheBuildNavMeshTile(
    cache: *TileCache,
    ref: CompressedTileRef,
    navmesh: *NavMesh,
) Result;
pub extern fn zrcTileCacheBuildNavMeshTilesAt(
    cache: *TileCache,
    tile_x: i32,
    tile_y: i32,
    navmesh: *NavMesh,
) Result;

//=============================================================================
// Building a compressed layer, and taking one apart
//=============================================================================

pub extern fn zrcTileCacheLayerBuild(
    compressor: *const TileCacheCompressor,
    header: *const TileCacheLayerHeader,
    heights: [*]const u8,
    areas: [*]const u8,
    cons: [*]const u8,
    out_data: *?*anyopaque,
    out_size: *usize,
) Result;
pub extern fn zrcTileCacheHeaderSwapEndian(data: ?*anyopaque, size: usize) Result;

pub extern fn zrcTileCacheLayerDecompress(
    compressor: *const TileCacheCompressor,
    allocator: ?*const TileCacheAllocator,
    data: ?*const anyopaque,
    size: usize,
    out: **TileCacheLayer,
) Result;
pub extern fn zrcTileCacheLayerDestroy(layer: ?*TileCacheLayer) void;
pub extern fn zrcTileCacheLayerHeaderOf(layer: *const TileCacheLayer, out: *TileCacheLayerHeader) Result;
pub extern fn zrcTileCacheLayerRegionCount(layer: *const TileCacheLayer, out_count: *i32) Result;

pub extern fn zrcTileCacheLayerHeights(
    layer: *const TileCacheLayer,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcTileCacheLayerAreas(
    layer: *const TileCacheLayer,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcTileCacheLayerSetAreas(
    layer: *TileCacheLayer,
    first: i32,
    count: i32,
    areas: [*]const u8,
) Result;
pub extern fn zrcTileCacheLayerCons(
    layer: *const TileCacheLayer,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcTileCacheLayerRegions(
    layer: *const TileCacheLayer,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;

pub extern fn zrcTileCacheLayerMarkCylinder(
    layer: *TileCacheLayer,
    origin: *const [3]f32,
    cell_size: f32,
    cell_height: f32,
    position: *const [3]f32,
    radius: f32,
    height: f32,
    area: u8,
) Result;
pub extern fn zrcTileCacheLayerMarkBox(
    layer: *TileCacheLayer,
    origin: *const [3]f32,
    cell_size: f32,
    cell_height: f32,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    area: u8,
) Result;
pub extern fn zrcTileCacheLayerMarkOrientedBox(
    layer: *TileCacheLayer,
    origin: *const [3]f32,
    cell_size: f32,
    cell_height: f32,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    y_radians: f32,
    area: u8,
) Result;
pub extern fn zrcTileCacheLayerBuildRegions(layer: *TileCacheLayer, walkable_climb: i32) Result;

pub extern fn zrcTileCacheContourSetCreate(
    allocator: ?*const TileCacheAllocator,
    layer: *TileCacheLayer,
    walkable_climb: i32,
    max_error: f32,
    out: **TileCacheContourSet,
) Result;
pub extern fn zrcTileCacheContourSetDestroy(contours: ?*TileCacheContourSet) void;
pub extern fn zrcTileCacheContourSetCount(contours: *const TileCacheContourSet, out_count: *i32) Result;
pub extern fn zrcTileCacheContourAt(
    contours: *const TileCacheContourSet,
    index: i32,
    out: *TileCacheContourInfo,
) Result;
pub extern fn zrcTileCacheContourVerts(
    contours: *const TileCacheContourSet,
    index: i32,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;

pub extern fn zrcTileCachePolyMeshCreate(
    allocator: ?*const TileCacheAllocator,
    contours: *const TileCacheContourSet,
    out: **TileCachePolyMesh,
) Result;
pub extern fn zrcTileCachePolyMeshDestroy(mesh: ?*TileCachePolyMesh) void;
pub extern fn zrcTileCachePolyMeshInfo(mesh: *const TileCachePolyMesh, out: *TileCachePolyMeshInfo) Result;
pub extern fn zrcTileCachePolyMeshVerts(
    mesh: *const TileCachePolyMesh,
    first: i32,
    count: i32,
    out: ?[*]u16,
) Result;
pub extern fn zrcTileCachePolyMeshPolys(
    mesh: *const TileCachePolyMesh,
    first: i32,
    count: i32,
    out: ?[*]u16,
) Result;
pub extern fn zrcTileCachePolyMeshAreas(
    mesh: *const TileCachePolyMesh,
    first: i32,
    count: i32,
    out: ?[*]u8,
) Result;
pub extern fn zrcTileCachePolyMeshFlags(
    mesh: *const TileCachePolyMesh,
    first: i32,
    count: i32,
    out: ?[*]u16,
) Result;

//=============================================================================
// Crowds — many agents steering around each other and the world
//=============================================================================

pub const crowd_max_neighbours: i32 = 6;
pub const crowd_max_corners: i32 = 4;
pub const crowd_max_avoidance_params: i32 = 8;
pub const crowd_max_filters: i32 = 16;
pub const avoidance_max_pattern_divs: i32 = 32;
pub const avoidance_max_pattern_rings: i32 = 4;
pub const crowd_max_agents: i32 = 16383;

/// A bitwise or of these goes in `CrowdAgentParams.update_flags`.
pub const crowd_anticipate_turns: u8 = 1;
pub const crowd_obstacle_avoidance: u8 = 2;
pub const crowd_separation: u8 = 4;
pub const crowd_optimize_vis: u8 = 8;
pub const crowd_optimize_topo: u8 = 16;

/// Where a path corridor is and what it holds. Every crowd agent carries
/// one, read with `zrcCrowdAgentCorridorInfo`.
pub const PathCorridorInfo = extern struct {
    position: [3]f32,
    target: [3]f32,
    first_poly: PolyRef,
    last_poly: PolyRef,
    path_count: i32,
};

/// How the velocity sampler weighs its candidates. A crowd holds
/// `crowd_max_avoidance_params` of these and each agent names one.
pub const AvoidanceParams = extern struct {
    vel_bias: f32,
    weight_desired_vel: f32,
    weight_current_vel: f32,
    weight_side: f32,
    weight_toi: f32,
    horiz_time: f32,
    grid_size: u8,
    adaptive_divs: u8,
    adaptive_rings: u8,
    adaptive_depth: u8,
};

/// A reference to one agent in a crowd. 0 is never a live agent.
pub const AgentRef = u64;

/// What kind of surface an agent is on.
pub const CrowdAgentState = enum(c_int) {
    invalid = 0,
    walking = 1,
    offmesh = 2,
};

/// Where an agent's move request has got to.
pub const CrowdTargetState = enum(c_int) {
    none = 0,
    failed = 1,
    valid = 2,
    requesting = 3,
    waiting_for_queue = 4,
    waiting_for_path = 5,
    velocity = 6,
};

/// How one agent behaves. Entirely a host's to set.
pub const CrowdAgentParams = extern struct {
    radius: f32,
    height: f32,
    max_acceleration: f32,
    max_speed: f32,
    collision_query_range: f32,
    path_optimization_range: f32,
    separation_weight: f32,
    update_flags: u8,
    obstacle_avoidance_type: u8,
    query_filter_type: u8,
    user_data: ?*anyopaque,
};

/// One agent, by value. Everything but its corridor and its local boundary,
/// which have accessors of their own.
pub const CrowdAgent = extern struct {
    state: CrowdAgentState,
    target_state: CrowdTargetState,
    partial: Bool,
    position: [3]f32,
    velocity: [3]f32,
    desired_velocity: [3]f32,
    avoided_velocity: [3]f32,
    displacement: [3]f32,
    desired_speed: f32,
    target_ref: PolyRef,
    target_position: [3]f32,
    target_replan: Bool,
    target_replan_time: f32,
    topology_opt_time: f32,
    corner_count: i32,
    neighbour_count: i32,
    params: CrowdAgentParams,
};

/// One corner of an agent's local path.
pub const CrowdCorner = extern struct {
    position: [3]f32,
    /// A bitwise or of the straightpath_* flags.
    flags: u8,
    poly: PolyRef,
};

/// One neighbour an agent is steering around.
pub const CrowdNeighbour = extern struct {
    agent: AgentRef,
    distance: f32,
};

/// How far an agent is through an off-mesh connection. Only meaningful
/// while its state is `.offmesh`.
pub const CrowdAgentAnimation = extern struct {
    active: Bool,
    init_position: [3]f32,
    start_position: [3]f32,
    end_position: [3]f32,
    poly: PolyRef,
    t: f32,
    t_max: f32,
};

/// What one update recorded about one agent, in and out.
pub const CrowdAgentDebug = extern struct {
    /// In: the agent to record. 0 records nothing.
    agent: AgentRef,
    /// In: where the avoidance sampler's candidate velocities land, or NULL.
    /// Borrowed for the call only.
    samples: ?*AvoidanceDebug,
    /// Out: the segment ZRC_CROWD_OPTIMIZE_VIS tried to shortcut this frame.
    opt_start: [3]f32,
    opt_end: [3]f32,
};

pub const Crowd = opaque {};
pub const ProximityGrid = opaque {};
pub const AvoidanceQuery = opaque {};
pub const AvoidanceDebug = opaque {};
pub const PathCorridor = opaque {};
pub const LocalBoundary = opaque {};
pub const PathQueue = opaque {};

pub extern fn zrcAvoidanceParamsDefault(out: *AvoidanceParams) void;

pub extern fn zrcCrowdCreate(
    navmesh: *const NavMesh,
    max_agents: i32,
    max_agent_radius: f32,
    out: **Crowd,
) Result;
pub extern fn zrcCrowdInit(
    crowd: *Crowd,
    navmesh: *const NavMesh,
    max_agents: i32,
    max_agent_radius: f32,
) Result;
pub extern fn zrcCrowdDestroy(crowd: ?*Crowd) void;

//=============================================================================
// Agents
//=============================================================================

pub extern fn zrcCrowdAddAgent(
    crowd: *Crowd,
    position: *const [3]f32,
    params: *const CrowdAgentParams,
    out_ref: *AgentRef,
) Result;
pub extern fn zrcCrowdRemoveAgent(crowd: *Crowd, ref: AgentRef) Result;
pub extern fn zrcCrowdSetAgentParams(
    crowd: *Crowd,
    ref: AgentRef,
    params: *const CrowdAgentParams,
) Result;
pub extern fn zrcCrowdAgentInfo(crowd: *const Crowd, ref: AgentRef, out: *CrowdAgent) Result;
pub extern fn zrcCrowdAgentCapacity(crowd: *const Crowd, out_count: *i32) Result;
pub extern fn zrcCrowdActiveAgentCount(crowd: *const Crowd, out_count: *i32) Result;
pub extern fn zrcCrowdActiveAgents(
    crowd: *const Crowd,
    out: ?[*]AgentRef,
    max_agents: i32,
    out_count: *i32,
) Result;
pub extern fn zrcCrowdAgentRefAt(crowd: *const Crowd, index: i32, out_ref: *AgentRef) Result;
pub extern fn zrcCrowdAgentCorners(
    crowd: *const Crowd,
    ref: AgentRef,
    first: i32,
    count: i32,
    out: ?[*]CrowdCorner,
) Result;
pub extern fn zrcCrowdAgentNeighbours(
    crowd: *const Crowd,
    ref: AgentRef,
    first: i32,
    count: i32,
    out: ?[*]CrowdNeighbour,
) Result;
pub extern fn zrcCrowdAgentCorridorInfo(
    crowd: *const Crowd,
    ref: AgentRef,
    out: *PathCorridorInfo,
) Result;
pub extern fn zrcCrowdAgentCorridorPath(
    crowd: *const Crowd,
    ref: AgentRef,
    first: i32,
    count: i32,
    out: ?[*]PolyRef,
) Result;
pub extern fn zrcCrowdAgentBoundaryCenter(crowd: *const Crowd, ref: AgentRef, out: *[3]f32) Result;
pub extern fn zrcCrowdAgentBoundarySegmentCount(
    crowd: *const Crowd,
    ref: AgentRef,
    out_count: *i32,
) Result;
pub extern fn zrcCrowdAgentBoundarySegments(
    crowd: *const Crowd,
    ref: AgentRef,
    first: i32,
    count: i32,
    out: ?[*]f32,
) Result;

//=============================================================================
// Where an agent is going
//=============================================================================

pub extern fn zrcCrowdRequestMoveTarget(
    crowd: *Crowd,
    ref: AgentRef,
    poly: PolyRef,
    position: *const [3]f32,
) Result;
pub extern fn zrcCrowdRequestMoveVelocity(
    crowd: *Crowd,
    ref: AgentRef,
    velocity: *const [3]f32,
) Result;
pub extern fn zrcCrowdResetMoveTarget(crowd: *Crowd, ref: AgentRef) Result;

//=============================================================================
// The frame
//=============================================================================

pub extern fn zrcCrowdUpdate(crowd: *Crowd, dt: f32, debug: ?*CrowdAgentDebug) Result;
pub extern fn zrcCrowdVelocitySampleCount(crowd: *const Crowd, out_count: *i32) Result;

//=============================================================================
// What the whole crowd shares
//=============================================================================

pub extern fn zrcCrowdSetAvoidanceParams(
    crowd: *Crowd,
    index: i32,
    params: *const AvoidanceParams,
) Result;
pub extern fn zrcCrowdAvoidanceParams(crowd: *const Crowd, index: i32, out: *AvoidanceParams) Result;
pub extern fn zrcCrowdSetFilter(crowd: *Crowd, index: i32, filter: *const QueryFilter) Result;
pub extern fn zrcCrowdFilter(crowd: *const Crowd, index: i32, out: *QueryFilter) Result;
pub extern fn zrcCrowdQueryHalfExtents(crowd: *const Crowd, out: *[3]f32) Result;
pub extern fn zrcCrowdGrid(crowd: *const Crowd, out: **const ProximityGrid) Result;
pub extern fn zrcCrowdPathQueue(crowd: *const Crowd, out: **const PathQueue) Result;
pub extern fn zrcCrowdNavMeshQuery(crowd: *const Crowd, out: **const NavMeshQuery) Result;

//=============================================================================
// The proximity grid
//=============================================================================

pub extern fn zrcProximityGridCreate(pool_size: i32, cell_size: f32, out: **ProximityGrid) Result;
pub extern fn zrcProximityGridDestroy(grid: ?*ProximityGrid) void;
pub extern fn zrcProximityGridClear(grid: *ProximityGrid) Result;
pub extern fn zrcProximityGridAddItem(
    grid: *ProximityGrid,
    id: u16,
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
) Result;
pub extern fn zrcProximityGridQueryItems(
    grid: *const ProximityGrid,
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
    out_ids: ?[*]u16,
    max_ids: i32,
    out_count: *i32,
) Result;
pub extern fn zrcProximityGridItemCountAt(
    grid: *const ProximityGrid,
    x: i32,
    y: i32,
    out_count: *i32,
) Result;
pub extern fn zrcProximityGridBounds(grid: *const ProximityGrid, out: *[4]i32) Result;
pub extern fn zrcProximityGridCellSize(grid: *const ProximityGrid, out: *f32) Result;

//=============================================================================
// Obstacle avoidance
//=============================================================================

/// A moving circular obstacle the sampler is avoiding.
pub const AvoidanceCircle = extern struct {
    position: [3]f32,
    velocity: [3]f32,
    desired_velocity: [3]f32,
    radius: f32,
};

/// A static segment the sampler is avoiding — a wall.
pub const AvoidanceSegment = extern struct {
    p: [3]f32,
    q: [3]f32,
    touching: Bool,
};

/// One candidate velocity the sampler tried, and why it scored as it did.
pub const AvoidanceSample = extern struct {
    velocity: [3]f32,
    size: f32,
    penalty: f32,
    desired_velocity_penalty: f32,
    current_velocity_penalty: f32,
    preferred_side_penalty: f32,
    collision_time_penalty: f32,
};

pub extern fn zrcAvoidanceQueryCreate(
    max_circles: i32,
    max_segments: i32,
    out: **AvoidanceQuery,
) Result;
pub extern fn zrcAvoidanceQueryDestroy(query: ?*AvoidanceQuery) void;
pub extern fn zrcAvoidanceQueryReset(query: *AvoidanceQuery) Result;
pub extern fn zrcAvoidanceAddCircle(
    query: *AvoidanceQuery,
    position: *const [3]f32,
    radius: f32,
    velocity: *const [3]f32,
    desired_velocity: *const [3]f32,
) Result;
pub extern fn zrcAvoidanceAddSegment(
    query: *AvoidanceQuery,
    p: *const [3]f32,
    q: *const [3]f32,
) Result;
pub extern fn zrcAvoidanceSampleGrid(
    query: *AvoidanceQuery,
    position: *const [3]f32,
    radius: f32,
    max_speed: f32,
    velocity: *const [3]f32,
    desired_velocity: *const [3]f32,
    params: *const AvoidanceParams,
    debug: ?*AvoidanceDebug,
    out_velocity: *[3]f32,
    out_samples: ?*i32,
) Result;
pub extern fn zrcAvoidanceSampleAdaptive(
    query: *AvoidanceQuery,
    position: *const [3]f32,
    radius: f32,
    max_speed: f32,
    velocity: *const [3]f32,
    desired_velocity: *const [3]f32,
    params: *const AvoidanceParams,
    debug: ?*AvoidanceDebug,
    out_velocity: *[3]f32,
    out_samples: ?*i32,
) Result;
pub extern fn zrcAvoidanceCircleCount(query: *const AvoidanceQuery, out_count: *i32) Result;
pub extern fn zrcAvoidanceCircleAt(
    query: *const AvoidanceQuery,
    index: i32,
    out: *AvoidanceCircle,
) Result;
pub extern fn zrcAvoidanceSegmentCount(query: *const AvoidanceQuery, out_count: *i32) Result;
pub extern fn zrcAvoidanceSegmentAt(
    query: *const AvoidanceQuery,
    index: i32,
    out: *AvoidanceSegment,
) Result;

pub extern fn zrcAvoidanceDebugCreate(max_samples: i32, out: **AvoidanceDebug) Result;
pub extern fn zrcAvoidanceDebugDestroy(debug: ?*AvoidanceDebug) void;
pub extern fn zrcAvoidanceDebugReset(debug: *AvoidanceDebug) Result;
pub extern fn zrcAvoidanceDebugAddSample(
    debug: *AvoidanceDebug,
    sample: *const AvoidanceSample,
) Result;
pub extern fn zrcAvoidanceDebugNormalize(debug: *AvoidanceDebug) Result;
pub extern fn zrcAvoidanceDebugSampleCount(debug: *const AvoidanceDebug, out_count: *i32) Result;
pub extern fn zrcAvoidanceDebugSampleAt(
    debug: *const AvoidanceDebug,
    index: i32,
    out: *AvoidanceSample,
) Result;

//=============================================================================
// The path corridor
//=============================================================================

pub const path_corridor_min_path: i32 = 32;

pub extern fn zrcPathCorridorCreate(max_path: i32, out: **PathCorridor) Result;
pub extern fn zrcPathCorridorDestroy(corridor: ?*PathCorridor) void;
pub extern fn zrcPathCorridorReset(
    corridor: *PathCorridor,
    poly: PolyRef,
    position: *const [3]f32,
) Result;
pub extern fn zrcPathCorridorSetCorridor(
    corridor: *PathCorridor,
    target: *const [3]f32,
    path: [*]const PolyRef,
    path_count: i32,
) Result;
pub extern fn zrcPathCorridorFindCorners(
    corridor: *PathCorridor,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out: ?[*]CrowdCorner,
    max_corners: i32,
    out_count: *i32,
) Result;
pub extern fn zrcPathCorridorOptimizeVisibility(
    corridor: *PathCorridor,
    next: *const [3]f32,
    range: f32,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
) Result;
pub extern fn zrcPathCorridorOptimizeTopology(
    corridor: *PathCorridor,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out_optimized: ?*Bool,
) Result;
pub extern fn zrcPathCorridorMoveOverOffmeshConnection(
    corridor: *PathCorridor,
    offmesh_poly: PolyRef,
    query: *NavMeshQuery,
    out_refs: *[2]PolyRef,
    out_start: *[3]f32,
    out_end: *[3]f32,
    out_moved: *Bool,
) Result;
pub extern fn zrcPathCorridorFixStart(
    corridor: *PathCorridor,
    safe_poly: PolyRef,
    safe_position: *const [3]f32,
    out_fixed: ?*Bool,
) Result;
pub extern fn zrcPathCorridorTrimInvalid(
    corridor: *PathCorridor,
    safe_poly: PolyRef,
    safe_position: *const [3]f32,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out_trimmed: ?*Bool,
) Result;
pub extern fn zrcPathCorridorIsValid(
    corridor: *PathCorridor,
    max_lookahead: i32,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out_valid: *Bool,
) Result;
pub extern fn zrcPathCorridorMovePosition(
    corridor: *PathCorridor,
    position: *const [3]f32,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out_moved: ?*Bool,
) Result;
pub extern fn zrcPathCorridorMoveTargetPosition(
    corridor: *PathCorridor,
    position: *const [3]f32,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out_moved: ?*Bool,
) Result;
pub extern fn zrcPathCorridorInfo(corridor: *const PathCorridor, out: *PathCorridorInfo) Result;
pub extern fn zrcPathCorridorPath(
    corridor: *const PathCorridor,
    first: i32,
    count: i32,
    out: ?[*]PolyRef,
) Result;

pub extern fn zrcMergeCorridorStartMoved(
    path: [*]PolyRef,
    path_count: i32,
    max_path: i32,
    visited: [*]const PolyRef,
    visited_count: i32,
    out_count: *i32,
) Result;
pub extern fn zrcMergeCorridorEndMoved(
    path: [*]PolyRef,
    path_count: i32,
    max_path: i32,
    visited: [*]const PolyRef,
    visited_count: i32,
    out_count: *i32,
) Result;
pub extern fn zrcMergeCorridorStartShortcut(
    path: [*]PolyRef,
    path_count: i32,
    max_path: i32,
    visited: [*]const PolyRef,
    visited_count: i32,
    out_count: *i32,
) Result;

//=============================================================================
// The local boundary
//=============================================================================

pub extern fn zrcLocalBoundaryCreate(out: **LocalBoundary) Result;
pub extern fn zrcLocalBoundaryDestroy(boundary: ?*LocalBoundary) void;
pub extern fn zrcLocalBoundaryReset(boundary: *LocalBoundary) Result;
pub extern fn zrcLocalBoundaryUpdate(
    boundary: *LocalBoundary,
    poly: PolyRef,
    position: *const [3]f32,
    range: f32,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
) Result;
pub extern fn zrcLocalBoundaryIsValid(
    boundary: *LocalBoundary,
    query: *NavMeshQuery,
    filter: *const QueryFilter,
    out_valid: *Bool,
) Result;
pub extern fn zrcLocalBoundaryCenter(boundary: *const LocalBoundary, out: *[3]f32) Result;
pub extern fn zrcLocalBoundarySegmentCount(boundary: *const LocalBoundary, out_count: *i32) Result;
pub extern fn zrcLocalBoundarySegments(
    boundary: *const LocalBoundary,
    first: i32,
    count: i32,
    out: ?[*]f32,
) Result;

//=============================================================================
// The path queue
//=============================================================================

/// A submitted search. 0 (`path_request_none`) is never a live request.
pub const PathRequestRef = u32;
pub const path_request_none: PathRequestRef = 0;

/// Where one request has got to.
pub const PathRequestState = enum(c_int) {
    unknown = 0,
    running = 1,
    ready = 2,
    failed = 3,
};

pub extern fn zrcPathQueueCreate(
    navmesh: *const NavMesh,
    max_path_size: i32,
    max_search_nodes: i32,
    out: **PathQueue,
) Result;
pub extern fn zrcPathQueueDestroy(queue: ?*PathQueue) void;
pub extern fn zrcPathQueueUpdate(queue: *PathQueue, max_iters: i32) Result;
pub extern fn zrcPathQueueRequest(
    queue: *PathQueue,
    start_poly: PolyRef,
    end_poly: PolyRef,
    start_position: *const [3]f32,
    end_position: *const [3]f32,
    filter: *const QueryFilter,
    out_ref: *PathRequestRef,
) Result;
pub extern fn zrcPathQueueRequestStatus(
    queue: *const PathQueue,
    ref: PathRequestRef,
    out_state: *PathRequestState,
) Result;
pub extern fn zrcPathQueueResult(
    queue: *PathQueue,
    ref: PathRequestRef,
    out: ?[*]PolyRef,
    max_path: i32,
    out_count: *i32,
) Result;
pub extern fn zrcPathQueueNavMeshQuery(queue: *const PathQueue, out: **const NavMeshQuery) Result;
