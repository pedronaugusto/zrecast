//===----------------------------------------------------------------------===//
// zrecast — compile-time layout assertions and the runtime layout report.
//
// Two independent guards live here:
//
//   1. static_asserts that fail the BUILD if a re-vendored Recast or Detour
//      changes a constant this package mirrors, a type it hands to upstream, or
//      a struct whose size decides how a serialised navmesh is laid out.
//   2. zrcAbiLayout(), which lets the Zig wrapper assert in a TEST that its
//      hand-written externs still match this translation unit.
//
// Together they cover both directions of drift: C++ vs upstream, and Zig vs C.
//===----------------------------------------------------------------------===//

#include <stddef.h>

#include "zrecast_internal.h"

namespace {

/// Compile-time type identity, without pulling in <type_traits> (which is
/// heavier than the one thing needed from it).
template <typename A, typename B>
struct SameType {
  static const bool value = false;
};
template <typename A>
struct SameType<A, A> {
  static const bool value = true;
};

//===----------------------------------------------------------------------===//
// Polygon references
//
// zrecast_query.cpp passes ZrcPolyRef and ZrcPolyRef arrays straight to Detour
// with no cast, which is only sound while the two are literally the same type.
// Upstream can change this by defining DT_POLYREF64, which zrecast does not.
//===----------------------------------------------------------------------===//

static_assert(SameType<ZrcPolyRef, dtPolyRef>::value,
              "ZrcPolyRef must be exactly dtPolyRef; DT_POLYREF64 is not "
              "supported by this binding");
static_assert(sizeof(ZrcPolyRef) == 4, "dtPolyRef is expected to be 32 bits");
static_assert(SameType<ZrcTileRef, dtTileRef>::value,
              "ZrcTileRef must be exactly dtTileRef");
static_assert(sizeof(ZrcTileRef) == 4, "dtTileRef is expected to be 32 bits");

//===----------------------------------------------------------------------===//
// Mirrored upstream constants
//
// Each of these is written out in zrecast.h so a C consumer need not include an
// upstream header. That duplication is only safe if it is checked.
//===----------------------------------------------------------------------===//

static_assert(ZRC_MAX_AREAS == DT_MAX_AREAS, "DT_MAX_AREAS changed");
static_assert(ZRC_VERTS_PER_POLYGON == DT_VERTS_PER_POLYGON,
              "DT_VERTS_PER_POLYGON changed");
static_assert(ZRC_AREA_NULL == RC_NULL_AREA, "RC_NULL_AREA changed");
static_assert(ZRC_AREA_WALKABLE == RC_WALKABLE_AREA, "RC_WALKABLE_AREA changed");

// The staged Recast pipeline mirrors these so a C consumer can size a span, a
// pool or a contour vertex without a Recast header.
static_assert(ZRC_SPAN_HEIGHT_BITS == RC_SPAN_HEIGHT_BITS,
              "RC_SPAN_HEIGHT_BITS changed");
static_assert(ZRC_SPAN_MAX_HEIGHT == RC_SPAN_MAX_HEIGHT,
              "RC_SPAN_MAX_HEIGHT changed");
static_assert(ZRC_SPANS_PER_POOL == RC_SPANS_PER_POOL,
              "RC_SPANS_PER_POOL changed");
static_assert(ZRC_NOT_CONNECTED == RC_NOT_CONNECTED, "RC_NOT_CONNECTED changed");
static_assert(ZRC_BORDER_REG == RC_BORDER_REG, "RC_BORDER_REG changed");
static_assert(ZRC_BORDER_VERTEX == RC_BORDER_VERTEX, "RC_BORDER_VERTEX changed");
static_assert(ZRC_AREA_BORDER == RC_AREA_BORDER, "RC_AREA_BORDER changed");
static_assert(ZRC_CONTOUR_REG_MASK == RC_CONTOUR_REG_MASK,
              "RC_CONTOUR_REG_MASK changed");
static_assert(ZRC_MESH_NULL_IDX == RC_MESH_NULL_IDX, "RC_MESH_NULL_IDX changed");
static_assert(ZRC_MULTIPLE_REGS == RC_MULTIPLE_REGS, "RC_MULTIPLE_REGS changed");

// zrc::HostContext casts between these enums rather than switching, so their
// enumerators have to agree one for one. ZRC_MAX_TIMERS is the length of a
// table indexed by a label, and zrc::IsTimerLabel refuses it as a label.
static_assert(static_cast<int>(ZRC_LOG_PROGRESS) ==
                  static_cast<int>(RC_LOG_PROGRESS),
              "rcLogCategory enumerator values changed");
static_assert(static_cast<int>(ZRC_LOG_WARNING) ==
                  static_cast<int>(RC_LOG_WARNING),
              "rcLogCategory enumerator values changed");
static_assert(static_cast<int>(ZRC_LOG_ERROR) == static_cast<int>(RC_LOG_ERROR),
              "rcLogCategory enumerator values changed");
static_assert(static_cast<int>(ZRC_TIMER_TOTAL) ==
                  static_cast<int>(RC_TIMER_TOTAL),
              "rcTimerLabel enumerator values changed");
static_assert(static_cast<int>(ZRC_TIMER_MERGE_POLYMESHDETAIL) ==
                  static_cast<int>(RC_TIMER_MERGE_POLYMESHDETAIL),
              "rcTimerLabel enumerator values changed");
static_assert(static_cast<int>(ZRC_MAX_TIMERS) ==
                  static_cast<int>(RC_MAX_TIMERS),
              "RC_MAX_TIMERS changed");

// The tile cache's own mirrored constants and reference types.
static_assert(SameType<ZrcCompressedTileRef, dtCompressedTileRef>::value,
              "ZrcCompressedTileRef must be exactly dtCompressedTileRef");
static_assert(SameType<ZrcObstacleRef, dtObstacleRef>::value,
              "ZrcObstacleRef must be exactly dtObstacleRef");
static_assert(ZRC_TILECACHE_MAGIC == DT_TILECACHE_MAGIC,
              "DT_TILECACHE_MAGIC changed");
static_assert(ZRC_TILECACHE_VERSION == DT_TILECACHE_VERSION,
              "DT_TILECACHE_VERSION changed");
static_assert(ZRC_MAX_TOUCHED_TILES == DT_MAX_TOUCHED_TILES,
              "DT_MAX_TOUCHED_TILES changed");
static_assert(ZRC_TILECACHE_AREA_NULL == DT_TILECACHE_NULL_AREA,
              "DT_TILECACHE_NULL_AREA changed");
static_assert(ZRC_TILECACHE_AREA_WALKABLE == DT_TILECACHE_WALKABLE_AREA,
              "DT_TILECACHE_WALKABLE_AREA changed");
static_assert(ZRC_TILECACHE_NULL_IDX == DT_TILECACHE_NULL_IDX,
              "DT_TILECACHE_NULL_IDX changed");

// zrcTileCacheObstacleInfo casts between these rather than switching, so their
// enumerators have to agree one for one.
static_assert(static_cast<int>(ZRC_OBSTACLE_CYLINDER) ==
                  static_cast<int>(DT_OBSTACLE_CYLINDER),
              "ObstacleType enumerator values changed");
static_assert(static_cast<int>(ZRC_OBSTACLE_BOX) ==
                  static_cast<int>(DT_OBSTACLE_BOX),
              "ObstacleType enumerator values changed");
static_assert(static_cast<int>(ZRC_OBSTACLE_ORIENTED_BOX) ==
                  static_cast<int>(DT_OBSTACLE_ORIENTED_BOX),
              "ObstacleType enumerator values changed");
static_assert(static_cast<int>(ZRC_OBSTACLE_EMPTY) ==
                  static_cast<int>(DT_OBSTACLE_EMPTY),
              "ObstacleState enumerator values changed");
static_assert(static_cast<int>(ZRC_OBSTACLE_PROCESSING) ==
                  static_cast<int>(DT_OBSTACLE_PROCESSING),
              "ObstacleState enumerator values changed");
static_assert(static_cast<int>(ZRC_OBSTACLE_PROCESSED) ==
                  static_cast<int>(DT_OBSTACLE_PROCESSED),
              "ObstacleState enumerator values changed");
static_assert(static_cast<int>(ZRC_OBSTACLE_REMOVING) ==
                  static_cast<int>(DT_OBSTACLE_REMOVING),
              "ObstacleState enumerator values changed");

// zrcContourSetCreate passes ZrcContourFlags straight through as the buildFlags
// int rcBuildContours takes.
static_assert(static_cast<int>(ZRC_CONTOUR_TESS_WALL_EDGES) ==
                  static_cast<int>(RC_CONTOUR_TESS_WALL_EDGES),
              "rcBuildContoursFlags enumerator values changed");
static_assert(static_cast<int>(ZRC_CONTOUR_TESS_AREA_EDGES) ==
                  static_cast<int>(RC_CONTOUR_TESS_AREA_EDGES),
              "rcBuildContoursFlags enumerator values changed");

// The allocator thunks in zrecast_core.cpp map ZrcAllocHint onto both of
// upstream's hint enums by value.
static_assert(static_cast<int>(ZRC_ALLOC_PERM) == static_cast<int>(RC_ALLOC_PERM),
              "rcAllocHint enumerator values changed");
static_assert(static_cast<int>(ZRC_ALLOC_TEMP) == static_cast<int>(RC_ALLOC_TEMP),
              "rcAllocHint enumerator values changed");
static_assert(static_cast<int>(ZRC_ALLOC_PERM) == static_cast<int>(DT_ALLOC_PERM),
              "dtAllocHint enumerator values changed");
static_assert(static_cast<int>(ZRC_ALLOC_TEMP) == static_cast<int>(DT_ALLOC_TEMP),
              "dtAllocHint enumerator values changed");

//===----------------------------------------------------------------------===//
// Serialised navmesh layout
//
// zrc::ValidateNavMeshImage reproduces dtNavMesh::addTile's pointer arithmetic in
// order to bounds-check an image before Detour walks it. That reproduction is
// only correct while these types have the sizes it assumes, and the image
// format only stays readable while the on-disk version is the pinned one.
//===----------------------------------------------------------------------===//

static_assert(DT_NAVMESH_VERSION == 7,
              "the serialised navmesh format version changed; update "
              "UPSTREAM.md and expect existing images to be rejected");
static_assert(sizeof(dtMeshHeader) == 100, "dtMeshHeader layout changed");
static_assert(sizeof(dtPoly) == 32, "dtPoly layout changed");
static_assert(sizeof(dtLink) == 12, "dtLink layout changed");
static_assert(sizeof(dtPolyDetail) == 12, "dtPolyDetail layout changed");
static_assert(sizeof(dtBVNode) == 16, "dtBVNode layout changed");
static_assert(sizeof(dtOffMeshConnection) == 36,
              "dtOffMeshConnection layout changed");

//===----------------------------------------------------------------------===//
// Allocator seam
//===----------------------------------------------------------------------===//

static_assert(sizeof(ZrcAllocator) == 3 * sizeof(void*),
              "ZrcAllocator is expected to be three pointers");
static_assert(ZRC_ALLOC_ALIGNMENT >= alignof(void*),
              "ZRC_ALLOC_ALIGNMENT must cover a pointer");
static_assert(ZRC_ALLOC_ALIGNMENT >= alignof(double),
              "ZRC_ALLOC_ALIGNMENT must cover the widest scalar");
static_assert(ZRC_ALLOC_ALIGNMENT >= alignof(dtMeshHeader),
              "ZRC_ALLOC_ALIGNMENT must cover a tile header, which is placed at "
              "the start of an allocated image");

//===----------------------------------------------------------------------===//
// Assertion seam
//===----------------------------------------------------------------------===//

static_assert(sizeof(ZrcAssertHandler) == 2 * sizeof(void*),
              "ZrcAssertHandler is expected to be two pointers");

//===----------------------------------------------------------------------===//
// Plain-data types crossing the boundary
//
// These are mirrored field for field by hand-written Zig externs, so their
// layout is part of the ABI even though nothing casts them to an upstream type.
//===----------------------------------------------------------------------===//

// Every ZrcBakeConfig field is four bytes, so the size pins the field count the
// list in zrcAbiLayout must enumerate. Adding a field without adding it there
// fails here.
static_assert(sizeof(ZrcBakeConfig) == 19 * 4,
              "ZrcBakeConfig gained, lost or resized a field; update the offset "
              "list in zrcAbiLayout to match");
static_assert(19 <= ZRC_ABI_MAX_FIELDS, "ZRC_ABI_MAX_FIELDS is too small");

static_assert(sizeof(ZrcQueryFilter) == ZRC_MAX_AREAS * sizeof(float) + 4,
              "ZrcQueryFilter gained padding or a field");
static_assert(offsetof(ZrcQueryFilter, area_cost) == 0, "field moved");
static_assert(sizeof(ZrcRaycastHit) == 4 + 12 + 12 + 4 + 4 + 4,
              "ZrcRaycastHit gained padding or a field");
static_assert(offsetof(ZrcRaycastHit, t) == 0, "field moved");
static_assert(sizeof(ZrcBool) == 4, "ZrcBool must stay a 32-bit int");

// ZrcAreaVolume is handed to the bake by a host, so a Zig extern that disagrees
// about it authors the wrong volume rather than failing to compile. The size is
// pinned so that adding a field without adding it to zrcAbiLayout fails here.
static_assert(sizeof(ZrcAreaVolume) == 48,
              "ZrcAreaVolume gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcAreaVolume, shape) == 0, "field moved");
static_assert(sizeof(ZrcAreaAuthoring) == 24,
              "ZrcAreaAuthoring gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcAreaAuthoring, volumes) == 0, "field moved");

// ZrcOffMeshConnection and ZrcTileAuthoring are handed to zrcNavMeshCreate and
// zrcTileDataBuild by a host, so a Zig extern that disagrees about either
// authors the wrong connection rather than failing to compile. The sizes are
// pinned so that adding a field without adding it to zrcAbiLayout fails here.
static_assert(sizeof(ZrcOffMeshConnection) == 48,
              "ZrcOffMeshConnection gained, lost or resized a field; update "
              "the offsets reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcOffMeshConnection, start) == 0, "field moved");
static_assert(sizeof(ZrcTileAuthoring) == 24,
              "ZrcTileAuthoring gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcTileAuthoring, connections) == 0, "field moved");

//===----------------------------------------------------------------------===//
// Queries: sliced pathfinding, random points, node pool
//
// ZrcRandomSource and ZrcPolyQuery are handed to this library by a host, the
// same as ZrcAllocator and ZrcAssertHandler; ZrcNode and ZrcNodePoolInfo are
// read out of a live dtNodePool by value, the same as ZrcNavMeshParams.
//===----------------------------------------------------------------------===//

static_assert(sizeof(ZrcRandomSource) == 2 * sizeof(void*),
              "ZrcRandomSource is expected to be two pointers");
static_assert(offsetof(ZrcRandomSource, next) == 0, "field moved");

static_assert(sizeof(ZrcPolyQuery) == 2 * sizeof(void*),
              "ZrcPolyQuery is expected to be two pointers");
static_assert(offsetof(ZrcPolyQuery, process) == 0, "field moved");

static_assert(sizeof(ZrcNode) == 36,
              "ZrcNode gained, lost or resized a field; update the offsets "
              "reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcNode, pos) == 0, "field moved");

static_assert(sizeof(ZrcNodePoolInfo) == 16,
              "ZrcNodePoolInfo gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcNodePoolInfo, node_count) == 0, "field moved");

//===----------------------------------------------------------------------===//
// Reading a loaded navmesh back
//
// These five are read out of a live dtNavMesh by value, so a Zig extern that
// disagrees about any of them reads the wrong bytes rather than failing to
// compile. Sizes are pinned the same way the authoring types above are.
//===----------------------------------------------------------------------===//

static_assert(sizeof(ZrcNavMeshParams) == 28,
              "ZrcNavMeshParams gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcNavMeshParams, origin) == 0, "field moved");

// ZrcTileInfo is wide enough that a handful of spot-checked offsets is not a
// guard, the same reasoning ZrcBakeConfig's offset list rests on: two
// same-sized fields swapping declaration order leaves the size and the
// alignment identical.
static_assert(sizeof(ZrcTileInfo) == 112,
              "ZrcTileInfo gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(24 <= ZRC_ABI_MAX_FIELDS, "ZRC_ABI_MAX_FIELDS is too small");

static_assert(sizeof(ZrcPolyInfo) == 48,
              "ZrcPolyInfo gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(11 <= ZRC_ABI_MAX_FIELDS, "ZRC_ABI_MAX_FIELDS is too small");

static_assert(sizeof(ZrcLink) == 12,
              "ZrcLink gained, lost or resized a field; update the offsets "
              "reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcLink, ref) == 0, "field moved");

static_assert(sizeof(ZrcBvNode) == 16,
              "ZrcBvNode gained, lost or resized a field; update the offsets "
              "reported by zrcAbiLayout to match");
static_assert(offsetof(ZrcBvNode, bmin) == 0, "field moved");

// ZrcTileLayout is eight offset/size pairs plus a total, all int64_t, so the
// same reasoning as tile_info_offsets applies: two fields trading places
// leaves the size and the alignment identical.
static_assert(sizeof(ZrcTileLayout) == 17 * 8,
              "ZrcTileLayout gained, lost or resized a field; update the "
              "offsets reported by zrcAbiLayout to match");
static_assert(17 <= ZRC_ABI_MAX_FIELDS, "ZRC_ABI_MAX_FIELDS is too small");

//===----------------------------------------------------------------------===//
// Crowds and obstacle avoidance
//
// DetourCrowd fixes each of these capacities at compile time; a Zig extern
// that disagrees about any of them sizes its own buffers to the wrong length.
//===----------------------------------------------------------------------===//

static_assert(ZRC_CROWD_MAX_NEIGHBOURS == DT_CROWDAGENT_MAX_NEIGHBOURS,
              "DT_CROWDAGENT_MAX_NEIGHBOURS changed");
static_assert(ZRC_CROWD_MAX_CORNERS == DT_CROWDAGENT_MAX_CORNERS,
              "DT_CROWDAGENT_MAX_CORNERS changed");
static_assert(ZRC_CROWD_MAX_AVOIDANCE_PARAMS ==
                  DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS,
              "DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS changed");
static_assert(ZRC_CROWD_MAX_FILTERS == DT_CROWD_MAX_QUERY_FILTER_TYPE,
              "DT_CROWD_MAX_QUERY_FILTER_TYPE changed");
static_assert(ZRC_AVOIDANCE_MAX_PATTERN_DIVS == DT_MAX_PATTERN_DIVS,
              "DT_MAX_PATTERN_DIVS changed");
static_assert(ZRC_AVOIDANCE_MAX_PATTERN_RINGS == DT_MAX_PATTERN_RINGS,
              "DT_MAX_PATTERN_RINGS changed");
static_assert(ZRC_PATH_REQUEST_NONE == DT_PATHQ_INVALID,
              "DT_PATHQ_INVALID changed");
static_assert(sizeof(ZrcPathRequestRef) == sizeof(dtPathQueueRef),
              "dtPathQueueRef changed size");

// zrcCrowdAgentInfo casts DetourCrowd's CrowdAgentState straight into
// ZrcCrowdAgentState rather than switching, so the two enums' values have to
// agree one for one.
static_assert(static_cast<int>(ZRC_CROWD_AGENT_INVALID) ==
                  static_cast<int>(DT_CROWDAGENT_STATE_INVALID),
              "CrowdAgentState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_AGENT_WALKING) ==
                  static_cast<int>(DT_CROWDAGENT_STATE_WALKING),
              "CrowdAgentState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_AGENT_OFFMESH) ==
                  static_cast<int>(DT_CROWDAGENT_STATE_OFFMESH),
              "CrowdAgentState enumerator values changed");

// Same for the move-request state.
static_assert(static_cast<int>(ZRC_CROWD_TARGET_NONE) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_NONE),
              "MoveRequestState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_TARGET_FAILED) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_FAILED),
              "MoveRequestState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_TARGET_VALID) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_VALID),
              "MoveRequestState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_TARGET_REQUESTING) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_REQUESTING),
              "MoveRequestState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_TARGET_WAITING_FOR_QUEUE) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE),
              "MoveRequestState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_TARGET_WAITING_FOR_PATH) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_WAITING_FOR_PATH),
              "MoveRequestState enumerator values changed");
static_assert(static_cast<int>(ZRC_CROWD_TARGET_VELOCITY) ==
                  static_cast<int>(DT_CROWDAGENT_TARGET_VELOCITY),
              "MoveRequestState enumerator values changed");

// ZrcCrowdAgentParams::update_flags is a bitwise or of these, passed straight
// through as dtCrowdAgentParams::updateFlags.
static_assert(static_cast<int>(ZRC_CROWD_ANTICIPATE_TURNS) ==
                  static_cast<int>(DT_CROWD_ANTICIPATE_TURNS),
              "DT_CROWD_ANTICIPATE_TURNS changed");
static_assert(static_cast<int>(ZRC_CROWD_OBSTACLE_AVOIDANCE) ==
                  static_cast<int>(DT_CROWD_OBSTACLE_AVOIDANCE),
              "DT_CROWD_OBSTACLE_AVOIDANCE changed");
static_assert(static_cast<int>(ZRC_CROWD_SEPARATION) ==
                  static_cast<int>(DT_CROWD_SEPARATION),
              "DT_CROWD_SEPARATION changed");
static_assert(static_cast<int>(ZRC_CROWD_OPTIMIZE_VIS) ==
                  static_cast<int>(DT_CROWD_OPTIMIZE_VIS),
              "DT_CROWD_OPTIMIZE_VIS changed");
static_assert(static_cast<int>(ZRC_CROWD_OPTIMIZE_TOPO) ==
                  static_cast<int>(DT_CROWD_OPTIMIZE_TOPO),
              "DT_CROWD_OPTIMIZE_TOPO changed");

// dtProximityGrid stores a pool index in an unsigned short and reserves
// 0xffff as its end-of-chain marker (DetourProximityGrid.cpp:120-127); the
// four-entries-per-agent pool a crowd builds has to stay under that.
static_assert((ZRC_CROWD_MAX_AGENTS * 4) <= 0xfffe,
              "ZRC_CROWD_MAX_AGENTS overflows dtProximityGrid's unsigned "
              "short pool index");

/// One past the last ZrcResult enumerator. The switch in zrcResultName is
/// compiled with -Wswitch, so adding a result without handling it there is
/// already a warning; this is what lets a consumer check the same thing.
const uint32_t kResultCount =
    static_cast<uint32_t>(ZRC_ERR_CROWD_FULL) + 1u;

}  // namespace

extern "C" {

void zrcAbiLayout(ZrcAbiLayout* out) {
  if (out == nullptr) return;
  out->layout_size = static_cast<uint32_t>(sizeof(ZrcAbiLayout));

  out->trimesh_size = static_cast<uint32_t>(sizeof(ZrcTriMesh));
  out->trimesh_align = static_cast<uint32_t>(alignof(ZrcTriMesh));
  out->trimesh_offset_verts =
      static_cast<uint32_t>(offsetof(ZrcTriMesh, verts));
  out->trimesh_offset_vert_count =
      static_cast<uint32_t>(offsetof(ZrcTriMesh, vert_count));
  out->trimesh_offset_tris = static_cast<uint32_t>(offsetof(ZrcTriMesh, tris));
  out->trimesh_offset_tri_count =
      static_cast<uint32_t>(offsetof(ZrcTriMesh, tri_count));

  out->bake_config_size = static_cast<uint32_t>(sizeof(ZrcBakeConfig));
  out->bake_config_align = static_cast<uint32_t>(alignof(ZrcBakeConfig));

  // Every field, in declaration order. The list is written out rather than
  // sampled because a sampled offset cannot see two same-sized fields swap
  // places, which is precisely the drift that would go unnoticed.
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, cell_height)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, agent_height)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, agent_radius)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, agent_max_climb)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, agent_max_slope)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, region_min_size)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, region_merge_size)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, edge_max_len)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, edge_max_error)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, verts_per_poly)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, detail_sample_dist)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, detail_sample_max_error)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, partition)),
        static_cast<uint32_t>(
            offsetof(ZrcBakeConfig, filter_low_hanging_obstacles)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, filter_ledge_spans)),
        static_cast<uint32_t>(
            offsetof(ZrcBakeConfig, filter_walkable_low_height_spans)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, tile_size)),
        static_cast<uint32_t>(offsetof(ZrcBakeConfig, border_size)),
    };
    const uint32_t count = static_cast<uint32_t>(sizeof(offsets) /
                                                 sizeof(offsets[0]));
    out->bake_config_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->bake_config_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->bake_log_size = static_cast<uint32_t>(sizeof(ZrcBakeLog));
  out->bake_log_align = static_cast<uint32_t>(alignof(ZrcBakeLog));
  out->bake_log_offset_buffer =
      static_cast<uint32_t>(offsetof(ZrcBakeLog, buffer));
  out->bake_log_offset_capacity =
      static_cast<uint32_t>(offsetof(ZrcBakeLog, capacity));

  out->query_filter_size = static_cast<uint32_t>(sizeof(ZrcQueryFilter));
  out->query_filter_align = static_cast<uint32_t>(alignof(ZrcQueryFilter));
  out->query_filter_offset_area_cost =
      static_cast<uint32_t>(offsetof(ZrcQueryFilter, area_cost));
  out->query_filter_offset_include_flags =
      static_cast<uint32_t>(offsetof(ZrcQueryFilter, include_flags));
  out->query_filter_offset_exclude_flags =
      static_cast<uint32_t>(offsetof(ZrcQueryFilter, exclude_flags));

  out->raycast_hit_size = static_cast<uint32_t>(sizeof(ZrcRaycastHit));
  out->raycast_hit_align = static_cast<uint32_t>(alignof(ZrcRaycastHit));
  out->raycast_hit_offset_t =
      static_cast<uint32_t>(offsetof(ZrcRaycastHit, t));
  out->raycast_hit_offset_position =
      static_cast<uint32_t>(offsetof(ZrcRaycastHit, position));
  out->raycast_hit_offset_normal =
      static_cast<uint32_t>(offsetof(ZrcRaycastHit, normal));
  out->raycast_hit_offset_hit =
      static_cast<uint32_t>(offsetof(ZrcRaycastHit, hit));
  // Added after the freeze; the fields themselves live at the end of
  // ZrcAbiLayout (see the struct's own comment), reported here beside the
  // rest of ZrcRaycastHit for readability.
  out->raycast_hit_offset_hit_edge_index =
      static_cast<uint32_t>(offsetof(ZrcRaycastHit, hit_edge_index));
  out->raycast_hit_offset_path_cost =
      static_cast<uint32_t>(offsetof(ZrcRaycastHit, path_cost));

  out->allocator_size = static_cast<uint32_t>(sizeof(ZrcAllocator));
  out->allocator_align = static_cast<uint32_t>(alignof(ZrcAllocator));
  out->allocator_offset_allocate =
      static_cast<uint32_t>(offsetof(ZrcAllocator, allocate));
  out->allocator_offset_deallocate =
      static_cast<uint32_t>(offsetof(ZrcAllocator, deallocate));
  out->allocator_offset_user =
      static_cast<uint32_t>(offsetof(ZrcAllocator, user));

  out->tile_grid_size = static_cast<uint32_t>(sizeof(ZrcTileGrid));
  out->tile_grid_align = static_cast<uint32_t>(alignof(ZrcTileGrid));
  out->tile_grid_offset_origin =
      static_cast<uint32_t>(offsetof(ZrcTileGrid, origin));
  out->tile_grid_offset_extent_max =
      static_cast<uint32_t>(offsetof(ZrcTileGrid, extent_max));
  out->tile_grid_offset_tile_world_size =
      static_cast<uint32_t>(offsetof(ZrcTileGrid, tile_world_size));
  out->tile_grid_offset_tile_count_x =
      static_cast<uint32_t>(offsetof(ZrcTileGrid, tile_count_x));
  out->tile_grid_offset_tile_count_z =
      static_cast<uint32_t>(offsetof(ZrcTileGrid, tile_count_z));

  out->area_volume_size = static_cast<uint32_t>(sizeof(ZrcAreaVolume));
  out->area_volume_align = static_cast<uint32_t>(alignof(ZrcAreaVolume));
  out->area_volume_offset_shape =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, shape));
  out->area_volume_offset_area =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, area));
  out->area_volume_offset_y_min =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, y_min));
  out->area_volume_offset_y_max =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, y_max));
  out->area_volume_offset_verts =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, verts));
  out->area_volume_offset_vert_count =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, vert_count));
  out->area_volume_offset_xz_min =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, xz_min));
  out->area_volume_offset_xz_max =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, xz_max));
  out->area_volume_offset_radius =
      static_cast<uint32_t>(offsetof(ZrcAreaVolume, radius));

  out->area_authoring_size = static_cast<uint32_t>(sizeof(ZrcAreaAuthoring));
  out->area_authoring_align = static_cast<uint32_t>(alignof(ZrcAreaAuthoring));
  out->area_authoring_offset_volumes =
      static_cast<uint32_t>(offsetof(ZrcAreaAuthoring, volumes));
  out->area_authoring_offset_volume_count =
      static_cast<uint32_t>(offsetof(ZrcAreaAuthoring, volume_count));
  out->area_authoring_offset_area_flags =
      static_cast<uint32_t>(offsetof(ZrcAreaAuthoring, area_flags));

  out->off_mesh_connection_size =
      static_cast<uint32_t>(sizeof(ZrcOffMeshConnection));
  out->off_mesh_connection_align =
      static_cast<uint32_t>(alignof(ZrcOffMeshConnection));
  out->off_mesh_connection_offset_start =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, start));
  out->off_mesh_connection_offset_end =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, end));
  out->off_mesh_connection_offset_radius =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, radius));
  out->off_mesh_connection_offset_area =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, area));
  out->off_mesh_connection_offset_flags =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, flags));
  out->off_mesh_connection_offset_bidirectional =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, bidirectional));
  out->off_mesh_connection_offset_user_id =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, user_id));
  out->off_mesh_connection_offset_end_side =
      static_cast<uint32_t>(offsetof(ZrcOffMeshConnection, end_side));

  out->tile_authoring_size = static_cast<uint32_t>(sizeof(ZrcTileAuthoring));
  out->tile_authoring_align = static_cast<uint32_t>(alignof(ZrcTileAuthoring));
  out->tile_authoring_offset_connections =
      static_cast<uint32_t>(offsetof(ZrcTileAuthoring, connections));
  out->tile_authoring_offset_connection_count =
      static_cast<uint32_t>(offsetof(ZrcTileAuthoring, connection_count));
  out->tile_authoring_offset_user_id =
      static_cast<uint32_t>(offsetof(ZrcTileAuthoring, user_id));
  out->tile_authoring_offset_skip_bv_tree =
      static_cast<uint32_t>(offsetof(ZrcTileAuthoring, skip_bv_tree));

  out->nav_mesh_params_size = static_cast<uint32_t>(sizeof(ZrcNavMeshParams));
  out->nav_mesh_params_align = static_cast<uint32_t>(alignof(ZrcNavMeshParams));
  out->nav_mesh_params_offset_origin =
      static_cast<uint32_t>(offsetof(ZrcNavMeshParams, origin));
  out->nav_mesh_params_offset_tile_width =
      static_cast<uint32_t>(offsetof(ZrcNavMeshParams, tile_width));
  out->nav_mesh_params_offset_tile_height =
      static_cast<uint32_t>(offsetof(ZrcNavMeshParams, tile_height));
  out->nav_mesh_params_offset_max_tiles =
      static_cast<uint32_t>(offsetof(ZrcNavMeshParams, max_tiles));
  out->nav_mesh_params_offset_max_polys =
      static_cast<uint32_t>(offsetof(ZrcNavMeshParams, max_polys));

  out->tile_info_size = static_cast<uint32_t>(sizeof(ZrcTileInfo));
  out->tile_info_align = static_cast<uint32_t>(alignof(ZrcTileInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileInfo, tile_x)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, tile_z)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, tile_layer)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, user_id)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, poly_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, ground_poly_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, off_mesh_con_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, vert_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, detail_mesh_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, detail_vert_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, detail_tri_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, bv_node_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, max_link_count)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, walkable_height)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, walkable_radius)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, walkable_climb)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, bv_quant_factor)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, magic)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, flags)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, salt)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, links_free_list)),
        static_cast<uint32_t>(offsetof(ZrcTileInfo, next_tile)),
    };
    const uint32_t count = static_cast<uint32_t>(sizeof(offsets) /
                                                 sizeof(offsets[0]));
    out->tile_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->poly_info_size = static_cast<uint32_t>(sizeof(ZrcPolyInfo));
  out->poly_info_align = static_cast<uint32_t>(alignof(ZrcPolyInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, verts)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, neis)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, flags)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, vert_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, area)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, type)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, first_link)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, detail_vert_base)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, detail_tri_base)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, detail_vert_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyInfo, detail_tri_count)),
    };
    const uint32_t count = static_cast<uint32_t>(sizeof(offsets) /
                                                 sizeof(offsets[0]));
    out->poly_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->poly_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->link_size = static_cast<uint32_t>(sizeof(ZrcLink));
  out->link_align = static_cast<uint32_t>(alignof(ZrcLink));
  out->link_offset_ref = static_cast<uint32_t>(offsetof(ZrcLink, ref));
  out->link_offset_next = static_cast<uint32_t>(offsetof(ZrcLink, next));
  out->link_offset_edge = static_cast<uint32_t>(offsetof(ZrcLink, edge));
  out->link_offset_side = static_cast<uint32_t>(offsetof(ZrcLink, side));
  out->link_offset_bmin = static_cast<uint32_t>(offsetof(ZrcLink, bmin));
  out->link_offset_bmax = static_cast<uint32_t>(offsetof(ZrcLink, bmax));

  out->bv_node_size = static_cast<uint32_t>(sizeof(ZrcBvNode));
  out->bv_node_align = static_cast<uint32_t>(alignof(ZrcBvNode));
  out->bv_node_offset_bmin = static_cast<uint32_t>(offsetof(ZrcBvNode, bmin));
  out->bv_node_offset_bmax = static_cast<uint32_t>(offsetof(ZrcBvNode, bmax));
  out->bv_node_offset_i = static_cast<uint32_t>(offsetof(ZrcBvNode, i));

  out->poly_ref_size = static_cast<uint32_t>(sizeof(ZrcPolyRef));
  out->tile_ref_size = static_cast<uint32_t>(sizeof(ZrcTileRef));
  out->result_count = kResultCount;
  out->max_areas = static_cast<uint32_t>(ZRC_MAX_AREAS);
  out->verts_per_polygon = static_cast<uint32_t>(ZRC_VERTS_PER_POLYGON);
  out->alloc_alignment = static_cast<uint32_t>(ZRC_ALLOC_ALIGNMENT);

  out->assert_handler_size = static_cast<uint32_t>(sizeof(ZrcAssertHandler));
  out->assert_handler_align = static_cast<uint32_t>(alignof(ZrcAssertHandler));
  out->assert_handler_offset_fail =
      static_cast<uint32_t>(offsetof(ZrcAssertHandler, fail));
  out->assert_handler_offset_user =
      static_cast<uint32_t>(offsetof(ZrcAssertHandler, user));

  out->tile_layout_size = static_cast<uint32_t>(sizeof(ZrcTileLayout));
  out->tile_layout_align = static_cast<uint32_t>(alignof(ZrcTileLayout));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileLayout, verts_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, verts_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, polys_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, polys_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, links_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, links_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, detail_meshes_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, detail_meshes_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, detail_verts_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, detail_verts_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, detail_tris_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, detail_tris_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, bv_tree_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, bv_tree_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, off_mesh_cons_offset)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, off_mesh_cons_size)),
        static_cast<uint32_t>(offsetof(ZrcTileLayout, total_size)),
    };
    const uint32_t count = static_cast<uint32_t>(sizeof(offsets) /
                                                 sizeof(offsets[0]));
    out->tile_layout_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_layout_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->random_source_size = static_cast<uint32_t>(sizeof(ZrcRandomSource));
  out->random_source_align = static_cast<uint32_t>(alignof(ZrcRandomSource));
  out->random_source_offset_next =
      static_cast<uint32_t>(offsetof(ZrcRandomSource, next));
  out->random_source_offset_user =
      static_cast<uint32_t>(offsetof(ZrcRandomSource, user));

  out->poly_query_size = static_cast<uint32_t>(sizeof(ZrcPolyQuery));
  out->poly_query_align = static_cast<uint32_t>(alignof(ZrcPolyQuery));
  out->poly_query_offset_process =
      static_cast<uint32_t>(offsetof(ZrcPolyQuery, process));
  out->poly_query_offset_user =
      static_cast<uint32_t>(offsetof(ZrcPolyQuery, user));

  out->node_size = static_cast<uint32_t>(sizeof(ZrcNode));
  out->node_align = static_cast<uint32_t>(alignof(ZrcNode));
  out->node_offset_pos = static_cast<uint32_t>(offsetof(ZrcNode, pos));
  out->node_offset_cost = static_cast<uint32_t>(offsetof(ZrcNode, cost));
  out->node_offset_total = static_cast<uint32_t>(offsetof(ZrcNode, total));
  out->node_offset_ref = static_cast<uint32_t>(offsetof(ZrcNode, ref));
  out->node_offset_parent_index =
      static_cast<uint32_t>(offsetof(ZrcNode, parent_index));
  out->node_offset_state = static_cast<uint32_t>(offsetof(ZrcNode, state));
  out->node_offset_flags = static_cast<uint32_t>(offsetof(ZrcNode, flags));

  out->node_pool_info_size = static_cast<uint32_t>(sizeof(ZrcNodePoolInfo));
  out->node_pool_info_align =
      static_cast<uint32_t>(alignof(ZrcNodePoolInfo));
  out->node_pool_info_offset_node_count =
      static_cast<uint32_t>(offsetof(ZrcNodePoolInfo, node_count));
  out->node_pool_info_offset_max_nodes =
      static_cast<uint32_t>(offsetof(ZrcNodePoolInfo, max_nodes));
  out->node_pool_info_offset_hash_size =
      static_cast<uint32_t>(offsetof(ZrcNodePoolInfo, hash_size));
  out->node_pool_info_offset_bytes_used =
      static_cast<uint32_t>(offsetof(ZrcNodePoolInfo, bytes_used));

  out->build_context_size = static_cast<uint32_t>(sizeof(ZrcBuildContext));
  out->build_context_align = static_cast<uint32_t>(alignof(ZrcBuildContext));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcBuildContext, user)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, log)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, reset_log)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, reset_timers)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, start_timer)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, stop_timer)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, accumulated_time)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, log_enabled)),
        static_cast<uint32_t>(offsetof(ZrcBuildContext, timers_enabled)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->build_context_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->build_context_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->span_size = static_cast<uint32_t>(sizeof(ZrcSpan));
  out->span_align = static_cast<uint32_t>(alignof(ZrcSpan));
  out->span_offset_smin = static_cast<uint32_t>(offsetof(ZrcSpan, smin));
  out->span_offset_smax = static_cast<uint32_t>(offsetof(ZrcSpan, smax));
  out->span_offset_area = static_cast<uint32_t>(offsetof(ZrcSpan, area));

  out->heightfield_info_size =
      static_cast<uint32_t>(sizeof(ZrcHeightfieldInfo));
  out->heightfield_info_align =
      static_cast<uint32_t>(alignof(ZrcHeightfieldInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcHeightfieldInfo, width)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldInfo, height)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldInfo, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldInfo, cell_height)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->heightfield_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->heightfield_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->heightfield_storage_size =
      static_cast<uint32_t>(sizeof(ZrcHeightfieldStorage));
  out->heightfield_storage_align =
      static_cast<uint32_t>(alignof(ZrcHeightfieldStorage));
  out->heightfield_storage_offset_pool_count =
      static_cast<uint32_t>(offsetof(ZrcHeightfieldStorage, pool_count));
  out->heightfield_storage_offset_free_count =
      static_cast<uint32_t>(offsetof(ZrcHeightfieldStorage, free_count));
  out->heightfield_storage_offset_spans_per_pool =
      static_cast<uint32_t>(offsetof(ZrcHeightfieldStorage, spans_per_pool));

  out->compact_cell_size = static_cast<uint32_t>(sizeof(ZrcCompactCell));
  out->compact_cell_align = static_cast<uint32_t>(alignof(ZrcCompactCell));
  out->compact_cell_offset_index =
      static_cast<uint32_t>(offsetof(ZrcCompactCell, index));
  out->compact_cell_offset_count =
      static_cast<uint32_t>(offsetof(ZrcCompactCell, count));

  out->compact_span_size = static_cast<uint32_t>(sizeof(ZrcCompactSpan));
  out->compact_span_align = static_cast<uint32_t>(alignof(ZrcCompactSpan));
  out->compact_span_offset_y =
      static_cast<uint32_t>(offsetof(ZrcCompactSpan, y));
  out->compact_span_offset_reg =
      static_cast<uint32_t>(offsetof(ZrcCompactSpan, reg));
  out->compact_span_offset_con =
      static_cast<uint32_t>(offsetof(ZrcCompactSpan, con));
  out->compact_span_offset_h =
      static_cast<uint32_t>(offsetof(ZrcCompactSpan, h));

  out->compact_heightfield_info_size =
      static_cast<uint32_t>(sizeof(ZrcCompactHeightfieldInfo));
  out->compact_heightfield_info_align =
      static_cast<uint32_t>(alignof(ZrcCompactHeightfieldInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, width)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, height)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, span_count)),
        static_cast<uint32_t>(
            offsetof(ZrcCompactHeightfieldInfo, walkable_height)),
        static_cast<uint32_t>(
            offsetof(ZrcCompactHeightfieldInfo, walkable_climb)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, border_size)),
        static_cast<uint32_t>(
            offsetof(ZrcCompactHeightfieldInfo, max_distance)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, max_regions)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcCompactHeightfieldInfo, cell_height)),
        static_cast<uint32_t>(
            offsetof(ZrcCompactHeightfieldInfo, has_distance_field)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->compact_heightfield_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->compact_heightfield_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->contour_set_info_size =
      static_cast<uint32_t>(sizeof(ZrcContourSetInfo));
  out->contour_set_info_align =
      static_cast<uint32_t>(alignof(ZrcContourSetInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, contour_count)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, cell_height)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, width)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, height)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, border_size)),
        static_cast<uint32_t>(offsetof(ZrcContourSetInfo, max_error)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->contour_set_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->contour_set_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->contour_info_size = static_cast<uint32_t>(sizeof(ZrcContourInfo));
  out->contour_info_align = static_cast<uint32_t>(alignof(ZrcContourInfo));
  out->contour_info_offset_vert_count =
      static_cast<uint32_t>(offsetof(ZrcContourInfo, vert_count));
  out->contour_info_offset_raw_vert_count =
      static_cast<uint32_t>(offsetof(ZrcContourInfo, raw_vert_count));
  out->contour_info_offset_region =
      static_cast<uint32_t>(offsetof(ZrcContourInfo, region));
  out->contour_info_offset_area =
      static_cast<uint32_t>(offsetof(ZrcContourInfo, area));

  out->poly_mesh_info_size = static_cast<uint32_t>(sizeof(ZrcPolyMeshInfo));
  out->poly_mesh_info_align = static_cast<uint32_t>(alignof(ZrcPolyMeshInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, vert_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, poly_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, max_polys)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, verts_per_poly)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, cell_height)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, border_size)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, max_edge_error)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, detail_mesh_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, detail_vert_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, detail_tri_count)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, walkable_height)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, walkable_radius)),
        static_cast<uint32_t>(offsetof(ZrcPolyMeshInfo, walkable_climb)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->poly_mesh_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->poly_mesh_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->timer_label_count = static_cast<uint32_t>(ZRC_MAX_TIMERS) + 1u;

  out->build_cells_size = static_cast<uint32_t>(sizeof(ZrcBuildCells));
  out->build_cells_align = static_cast<uint32_t>(alignof(ZrcBuildCells));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcBuildCells, walkable_height)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, walkable_climb)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, walkable_radius)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, max_edge_len)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, min_region_area)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, merge_region_area)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, border_size)),
        static_cast<uint32_t>(
            offsetof(ZrcBuildCells, max_simplification_error)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, verts_per_poly)),
        static_cast<uint32_t>(offsetof(ZrcBuildCells, detail_sample_dist)),
        static_cast<uint32_t>(
            offsetof(ZrcBuildCells, detail_sample_max_error)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->build_cells_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->build_cells_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->heightfield_layer_size = static_cast<uint32_t>(sizeof(ZrcHeightfieldLayer));
  out->heightfield_layer_align = static_cast<uint32_t>(alignof(ZrcHeightfieldLayer));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, bmin)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, bmax)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, cell_height)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, width)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, height)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, min_x)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, max_x)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, min_z)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, max_z)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, height_min)),
        static_cast<uint32_t>(offsetof(ZrcHeightfieldLayer, height_max)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->heightfield_layer_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->heightfield_layer_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->tile_cache_params_size = static_cast<uint32_t>(sizeof(ZrcTileCacheParams));
  out->tile_cache_params_align = static_cast<uint32_t>(alignof(ZrcTileCacheParams));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, origin)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, cell_size)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, cell_height)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, width)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, height)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, walkable_height)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, walkable_radius)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, walkable_climb)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, max_simplification_error)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, max_tiles)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheParams, max_obstacles)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->tile_cache_params_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_cache_params_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->tile_cache_compressor_size = static_cast<uint32_t>(sizeof(ZrcTileCacheCompressor));
  out->tile_cache_compressor_align = static_cast<uint32_t>(alignof(ZrcTileCacheCompressor));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileCacheCompressor, user)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheCompressor, max_compressed_size)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheCompressor, compress)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheCompressor, decompress)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->tile_cache_compressor_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_cache_compressor_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->tile_cache_allocator_size = static_cast<uint32_t>(sizeof(ZrcTileCacheAllocator));
  out->tile_cache_allocator_align = static_cast<uint32_t>(alignof(ZrcTileCacheAllocator));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileCacheAllocator, user)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheAllocator, reset)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheAllocator, allocate)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheAllocator, deallocate)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->tile_cache_allocator_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_cache_allocator_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->tile_cache_build_params_size = static_cast<uint32_t>(sizeof(ZrcTileCacheBuildParams));
  out->tile_cache_build_params_align = static_cast<uint32_t>(alignof(ZrcTileCacheBuildParams));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileCacheBuildParams, areas)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheBuildParams, flags)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheBuildParams, poly_count)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheBuildParams, user_id)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheBuildParams, connections)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheBuildParams, connection_count)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->tile_cache_build_params_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_cache_build_params_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->compressed_tile_info_size = static_cast<uint32_t>(sizeof(ZrcCompressedTileInfo));
  out->compressed_tile_info_align = static_cast<uint32_t>(alignof(ZrcCompressedTileInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, tile_x)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, tile_y)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, tile_layer)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, height_min)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, height_max)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, width)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, height)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, min_x)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, max_x)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, min_z)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, max_z)),
        static_cast<uint32_t>(offsetof(ZrcCompressedTileInfo, data_size)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->compressed_tile_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->compressed_tile_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->obstacle_info_size = static_cast<uint32_t>(sizeof(ZrcObstacleInfo));
  out->obstacle_info_align = static_cast<uint32_t>(alignof(ZrcObstacleInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, shape)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, state)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, position)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, radius)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, height)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, bmin)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, bmax)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, center)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, half_extents)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, y_radians)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, touched_count)),
        static_cast<uint32_t>(offsetof(ZrcObstacleInfo, pending_count)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->obstacle_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->obstacle_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->tile_cache_layer_header_size = static_cast<uint32_t>(sizeof(ZrcTileCacheLayerHeader));
  out->tile_cache_layer_header_align = static_cast<uint32_t>(alignof(ZrcTileCacheLayerHeader));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, tile_x)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, tile_y)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, tile_layer)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, bmin)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, bmax)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, height_min)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, height_max)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, width)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, height)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, min_x)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, max_x)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, min_z)),
        static_cast<uint32_t>(offsetof(ZrcTileCacheLayerHeader, max_z)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->tile_cache_layer_header_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->tile_cache_layer_header_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->tile_cache_contour_info_size =
      static_cast<uint32_t>(sizeof(ZrcTileCacheContourInfo));
  out->tile_cache_contour_info_align =
      static_cast<uint32_t>(alignof(ZrcTileCacheContourInfo));
  out->tile_cache_contour_info_offset_vert_count =
      static_cast<uint32_t>(offsetof(ZrcTileCacheContourInfo, vert_count));
  out->tile_cache_contour_info_offset_region =
      static_cast<uint32_t>(offsetof(ZrcTileCacheContourInfo, region));
  out->tile_cache_contour_info_offset_area =
      static_cast<uint32_t>(offsetof(ZrcTileCacheContourInfo, area));

  out->tile_cache_poly_mesh_info_size =
      static_cast<uint32_t>(sizeof(ZrcTileCachePolyMeshInfo));
  out->tile_cache_poly_mesh_info_align =
      static_cast<uint32_t>(alignof(ZrcTileCachePolyMeshInfo));
  out->tile_cache_poly_mesh_info_offset_vert_count = static_cast<uint32_t>(
      offsetof(ZrcTileCachePolyMeshInfo, vert_count));
  out->tile_cache_poly_mesh_info_offset_poly_count = static_cast<uint32_t>(
      offsetof(ZrcTileCachePolyMeshInfo, poly_count));
  out->tile_cache_poly_mesh_info_offset_verts_per_poly =
      static_cast<uint32_t>(
          offsetof(ZrcTileCachePolyMeshInfo, verts_per_poly));

  out->compressed_tile_ref_size =
      static_cast<uint32_t>(sizeof(ZrcCompressedTileRef));
  out->obstacle_ref_size = static_cast<uint32_t>(sizeof(ZrcObstacleRef));

  out->crowd_agent_params_size =
      static_cast<uint32_t>(sizeof(ZrcCrowdAgentParams));
  out->crowd_agent_params_align =
      static_cast<uint32_t>(alignof(ZrcCrowdAgentParams));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentParams, radius)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentParams, height)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentParams, max_acceleration)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentParams, max_speed)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentParams, collision_query_range)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentParams, path_optimization_range)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentParams, separation_weight)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentParams, update_flags)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentParams, obstacle_avoidance_type)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentParams, query_filter_type)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentParams, user_data)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->crowd_agent_params_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->crowd_agent_params_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->crowd_agent_size = static_cast<uint32_t>(sizeof(ZrcCrowdAgent));
  out->crowd_agent_align = static_cast<uint32_t>(alignof(ZrcCrowdAgent));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, state)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, target_state)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, partial)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, position)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, velocity)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, desired_velocity)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, avoided_velocity)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, displacement)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, desired_speed)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, target_ref)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, target_position)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, target_replan)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, target_replan_time)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, topology_opt_time)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, corner_count)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, neighbour_count)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgent, params)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->crowd_agent_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->crowd_agent_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->crowd_corner_size = static_cast<uint32_t>(sizeof(ZrcCrowdCorner));
  out->crowd_corner_align = static_cast<uint32_t>(alignof(ZrcCrowdCorner));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCrowdCorner, position)),
        static_cast<uint32_t>(offsetof(ZrcCrowdCorner, flags)),
        static_cast<uint32_t>(offsetof(ZrcCrowdCorner, poly)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->crowd_corner_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->crowd_corner_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->crowd_neighbour_size =
      static_cast<uint32_t>(sizeof(ZrcCrowdNeighbour));
  out->crowd_neighbour_align =
      static_cast<uint32_t>(alignof(ZrcCrowdNeighbour));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCrowdNeighbour, agent)),
        static_cast<uint32_t>(offsetof(ZrcCrowdNeighbour, distance)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->crowd_neighbour_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->crowd_neighbour_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->crowd_agent_animation_size =
      static_cast<uint32_t>(sizeof(ZrcCrowdAgentAnimation));
  out->crowd_agent_animation_align =
      static_cast<uint32_t>(alignof(ZrcCrowdAgentAnimation));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentAnimation, active)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentAnimation, init_position)),
        static_cast<uint32_t>(
            offsetof(ZrcCrowdAgentAnimation, start_position)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentAnimation, end_position)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentAnimation, poly)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentAnimation, t)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentAnimation, t_max)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->crowd_agent_animation_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->crowd_agent_animation_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->crowd_agent_debug_size =
      static_cast<uint32_t>(sizeof(ZrcCrowdAgentDebug));
  out->crowd_agent_debug_align =
      static_cast<uint32_t>(alignof(ZrcCrowdAgentDebug));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentDebug, agent)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentDebug, samples)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentDebug, opt_start)),
        static_cast<uint32_t>(offsetof(ZrcCrowdAgentDebug, opt_end)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->crowd_agent_debug_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->crowd_agent_debug_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->avoidance_params_size =
      static_cast<uint32_t>(sizeof(ZrcAvoidanceParams));
  out->avoidance_params_align =
      static_cast<uint32_t>(alignof(ZrcAvoidanceParams));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, vel_bias)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceParams, weight_desired_vel)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceParams, weight_current_vel)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, weight_side)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, weight_toi)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, horiz_time)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, grid_size)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, adaptive_divs)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, adaptive_rings)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceParams, adaptive_depth)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->avoidance_params_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->avoidance_params_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->avoidance_circle_size =
      static_cast<uint32_t>(sizeof(ZrcAvoidanceCircle));
  out->avoidance_circle_align =
      static_cast<uint32_t>(alignof(ZrcAvoidanceCircle));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcAvoidanceCircle, position)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceCircle, velocity)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceCircle, desired_velocity)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceCircle, radius)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->avoidance_circle_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->avoidance_circle_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->avoidance_segment_size =
      static_cast<uint32_t>(sizeof(ZrcAvoidanceSegment));
  out->avoidance_segment_align =
      static_cast<uint32_t>(alignof(ZrcAvoidanceSegment));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcAvoidanceSegment, p)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceSegment, q)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceSegment, touching)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->avoidance_segment_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->avoidance_segment_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->avoidance_sample_size =
      static_cast<uint32_t>(sizeof(ZrcAvoidanceSample));
  out->avoidance_sample_align =
      static_cast<uint32_t>(alignof(ZrcAvoidanceSample));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcAvoidanceSample, velocity)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceSample, size)),
        static_cast<uint32_t>(offsetof(ZrcAvoidanceSample, penalty)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceSample, desired_velocity_penalty)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceSample, current_velocity_penalty)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceSample, preferred_side_penalty)),
        static_cast<uint32_t>(
            offsetof(ZrcAvoidanceSample, collision_time_penalty)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->avoidance_sample_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->avoidance_sample_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->path_corridor_info_size =
      static_cast<uint32_t>(sizeof(ZrcPathCorridorInfo));
  out->path_corridor_info_align =
      static_cast<uint32_t>(alignof(ZrcPathCorridorInfo));
  {
    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(ZrcPathCorridorInfo, position)),
        static_cast<uint32_t>(offsetof(ZrcPathCorridorInfo, target)),
        static_cast<uint32_t>(offsetof(ZrcPathCorridorInfo, first_poly)),
        static_cast<uint32_t>(offsetof(ZrcPathCorridorInfo, last_poly)),
        static_cast<uint32_t>(offsetof(ZrcPathCorridorInfo, path_count)),
    };
    const uint32_t count =
        static_cast<uint32_t>(sizeof(offsets) / sizeof(offsets[0]));
    out->path_corridor_info_field_count = count;
    for (uint32_t i = 0; i < ZRC_ABI_MAX_FIELDS; ++i) {
      out->path_corridor_info_offsets[i] = i < count ? offsets[i] : 0u;
    }
  }

  out->agent_ref_size = static_cast<uint32_t>(sizeof(ZrcAgentRef));
  out->path_request_ref_size =
      static_cast<uint32_t>(sizeof(ZrcPathRequestRef));
}

}  // extern "C"
