//===----------------------------------------------------------------------===//
// zrecast — a C ABI over Recast (navmesh baking) and Detour (navmesh queries).
//
// This header is the ONLY contract between the C++ implementation and any
// consumer (the Zig wrapper in ../src, or a plain C host). It is deliberately
// free of C++: opaque handles, POD structs with fixed layout, and a flat result
// enum. No exceptions cross this boundary; Recast and Detour throw none.
//
// TWO LIFECYCLES, and the split runs through this header:
//
//   Recast is a BAKER. It turns a triangle soup into a polygon mesh, and it
//   belongs in a tools/cook pipeline: it is slow, allocates heavily, and its
//   output is an artefact you ship. Everything named zrcPolyMesh* is bake-time.
//
//   Detour is the RUNTIME. It loads a baked navmesh and answers queries with no
//   further baking. Everything named zrcNavMesh* / zrcNavMeshQuery* is runtime.
//
// A game only needs the second half, and because each concern is its own
// translation unit it gets to link only that half: a program that calls no
// zrcPolyMesh* entry point pulls in none of Recast.
//
// Ownership rules, uniformly:
//   *Bake / *Create /   allocate through the installed allocator and yield a
//   *Deserialize        handle the caller owns.
//   *Destroy            accepts NULL and does nothing.
//   Query accessors     never allocate; returned pointers borrow from the
//                       handle and die with it.
//
// Thread safety: handles are not internally synchronised. A ZrcNavMesh may be
// shared read-only by several ZrcNavMeshQuery objects, but a single
// ZrcNavMeshQuery holds a mutable node pool and must not be used from two
// threads at once — give each thread its own.
//===----------------------------------------------------------------------===//

#ifndef ZRECAST_H_
#define ZRECAST_H_

#include <stddef.h>
#include <stdint.h>

// Only the entry points below are exported. The library is compiled with
// -fvisibility=hidden where the toolchain supports it, so a host that links its
// own copy of Recast or Detour is not silently interposed by the one vendored
// here — without that, a shared build exports every rc*/dt* symbol as well.
#if defined(ZRECAST_SHARED)
#if defined(_MSC_VER)
#ifdef ZRECAST_BUILD
#define ZRC_API __declspec(dllexport)
#else
#define ZRC_API __declspec(dllimport)
#endif
#elif defined(__GNUC__)
#define ZRC_API __attribute__((visibility("default")))
#else
#define ZRC_API
#endif
#else
#define ZRC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Version
//===----------------------------------------------------------------------===//

#define ZRC_VERSION_MAJOR 0
#define ZRC_VERSION_MINOR 1
#define ZRC_VERSION_PATCH 0

/// Version of the zrecast binding itself, packed as (major<<16)|(minor<<8)|patch.
/// Compare against the ZRC_VERSION_* macros to detect a header/library skew.
ZRC_API uint32_t zrcVersion(void);

/// Version of the vendored recastnavigation tree, same packing.
ZRC_API uint32_t zrcRecastVersion(void);

/// Format version of a serialised navmesh image (Detour's DT_NAVMESH_VERSION).
/// An image produced by a different value will be rejected by
/// zrcNavMeshDeserialize.
ZRC_API int32_t zrcNavMeshDataVersion(void);

//===----------------------------------------------------------------------===//
// Basic types
//===----------------------------------------------------------------------===//

/// A boolean in the ABI. `int` rather than C99 `_Bool` so the layout is the
/// same under every compiler that reads this header, C++ included.
typedef int32_t ZrcBool;
#define ZRC_FALSE 0
#define ZRC_TRUE 1

//===----------------------------------------------------------------------===//
// Results
//===----------------------------------------------------------------------===//

typedef enum ZrcResult {
  ZRC_OK = 0,
  /// A NULL handle, an out-of-range count, or a non-finite / out-of-domain
  /// scalar was passed in.
  ZRC_ERR_INVALID_ARGUMENT = 1,
  /// The installed allocator returned NULL.
  ZRC_ERR_OUT_OF_MEMORY = 2,
  /// Serialised navmesh bytes that are not a navmesh, or whose header does not
  /// describe the buffer that carries it.
  ZRC_ERR_BAD_FORMAT = 3,
  /// Serialised navmesh bytes in a format version this build does not speak.
  ZRC_ERR_UNSUPPORTED_VERSION = 4,
  /// A caller-provided output buffer was too small for the result.
  ZRC_ERR_BUFFER_TOO_SMALL = 5,
  /// A Recast build stage failed. Pass a ZrcBakeLog to find out which.
  ZRC_ERR_BAKE_FAILED = 6,
  /// The bake succeeded but produced no walkable polygons — usually a cell
  /// size too coarse for the geometry, or an agent that fits nowhere.
  ZRC_ERR_EMPTY_RESULT = 7,
  /// Detour reported DT_FAILURE for a reason with no more specific mapping.
  ZRC_ERR_QUERY_FAILED = 8,
  /// The query's node pool was exhausted. Raise max_nodes.
  ZRC_ERR_OUT_OF_NODES = 9,
  /// A tile already occupies that grid position. Remove it first.
  ZRC_ERR_TILE_OCCUPIED = 10,
  /// The navmesh has no free tile slot. It was created for max_tiles, and that
  /// many are resident.
  ZRC_ERR_NAVMESH_FULL = 11,
  /// A sliced path search is in flight on this query, and what was asked for
  /// would clear the node pool underneath it. Finalise or cancel it first.
  ZRC_ERR_SEARCH_IN_PROGRESS = 12,
  /// No sliced path search is in flight on this query, so there is nothing to
  /// advance, finalise or read.
  ZRC_ERR_NO_SEARCH = 13,
  /// The thing asked for is not there — a polygon the last search never
  /// reached, or a pool index no node occupies. Distinct from an argument
  /// error: the question was well formed and the answer is nothing.
  ZRC_ERR_NOT_FOUND = 14,
  /// A build stage was pointed at a container that already holds a result.
  /// Recast overwrites the buffer pointers without freeing them, so the stage
  /// is refused instead. Destroy the handle and make a new one.
  ZRC_ERR_ALREADY_BUILT = 15,
  /// The crowd has no free agent slot. It was created for max_agents, and that
  /// many are active.
  ZRC_ERR_CROWD_FULL = 16,
} ZrcResult;

/// Static, never-NULL description of a result code. Borrowed; do not free.
ZRC_API const char* zrcResultName(ZrcResult result);

//===----------------------------------------------------------------------===//
// Allocator seam
//
// Recast and Detour each route every allocation through one global pair of
// function pointers (rcAllocSetCustom / dtAllocSetCustom). zrecast installs
// both from a single ZrcAllocator so a host can account for navigation memory
// in its own budget.
//
// Note the asymmetry inherited from upstream, and note what is NOT here:
//
//   * `deallocate` receives only the block pointer — no size. A host allocator
//     that needs the size back (Zig's std.mem.Allocator does) must record it in
//     a header of its own. ../src/memory.zig does exactly that.
//
//   * `allocate` receives no alignment, because neither rcAlloc nor dtAlloc has
//     an alignment parameter: both are modelled on malloc. The contract is
//     therefore malloc's — a returned block must be aligned for any scalar type
//     the platform has. ZRC_ALLOC_ALIGNMENT below is that guarantee, stated.
//===----------------------------------------------------------------------===//

/// Minimum alignment every block from ZrcAllocator.allocate must satisfy.
///
/// Detour casts raw tile bytes to structs containing 4-byte scalars, and
/// Recast's pools hold pointers; 16 covers both with the same margin malloc
/// gives, on every target this package builds for.
#define ZRC_ALLOC_ALIGNMENT 16

/// How long the allocation is expected to live. Passed through from upstream
/// verbatim; a host may use it to pick an arena, or ignore it entirely.
typedef enum ZrcAllocHint {
  /// Persists after the call returns — part of a result the caller keeps.
  ZRC_ALLOC_PERM = 0,
  /// Scratch, freed before the call that made it returns.
  ZRC_ALLOC_TEMP = 1,
} ZrcAllocHint;

typedef struct ZrcAllocator {
  /// Must return a block of at least `size` bytes aligned to at least
  /// ZRC_ALLOC_ALIGNMENT, or NULL on failure. `size` may be 0.
  void* (*allocate)(void* user, size_t size, ZrcAllocHint hint);
  /// Frees a block from `allocate`. Must tolerate a NULL block.
  void (*deallocate)(void* user, void* block);
  /// Opaque host pointer, passed back unmodified.
  void* user;
} ZrcAllocator;

/// Installs a process-wide allocator for all subsequent Recast and Detour
/// allocations.
///
/// This is global state, mirroring upstream's own design. Call it before
/// building anything, and do not swap it while live handles exist — those
/// handles will be freed through whichever allocator is installed at
/// destruction time. Passing NULL restores upstream's default (malloc/free).
///
/// `alloc` is copied by value; the caller need not keep it alive, but `user`
/// must outlive every handle allocated through it.
///
/// Returns ZRC_ERR_INVALID_ARGUMENT if either function pointer is NULL, in
/// which case the previously installed allocator is left untouched.
ZRC_API ZrcResult zrcSetAllocator(const ZrcAllocator* alloc);

/// Allocates through the installed allocator, with upstream's own lifetime hint.
///
/// This is dtAlloc, reachable. A host that installed a ZrcAllocator can already
/// see every allocation Recast and Detour make; this is how it makes one on the
/// same seam and has it accounted the same way — scratch for
/// zrcRandomPointInConvexPoly, say.
///
/// Returns NULL when the allocator does. A `size` of 0 is passed through
/// unchanged, so whether that yields NULL is the allocator's answer, not this
/// function's.
ZRC_API void* zrcAlloc(size_t size, ZrcAllocHint hint);

/// Frees a block from zrcAlloc, or a buffer returned by zrcNavMeshSerialize.
///
/// Such a block comes from the installed allocator, which the caller may not
/// be able to call directly (the Zig bridge, for one, stores a private header
/// ahead of each block). This is the matching free. Tolerates NULL.
ZRC_API void zrcFree(void* block);

//===----------------------------------------------------------------------===//
// Assertions
//
// Recast and Detour each assert their internal invariants through one global
// hook (rcAssertFailSetCustom, dtAssertFailSetCustom), and with no hook
// installed a failure reaches assert() and takes the process down without a
// word to the host. zrecast installs both from a single ZrcAssertHandler.
//
// Both, in one call, deliberately. The two halves are separate seams upstream,
// and a host that installed only one would get its own logger for half the
// library and a bare abort for the other half.
//
// The hook exists only in a build that kept the assertions: upstream compiles
// the whole family out under NDEBUG — the type, the setter and the getter are
// all inside the #else. zrcAssertsEnabled is the seam that says which build
// this is, and it is not a formality. With assertions compiled out, `rcAssert`
// and `dtAssert` expand to `(void)sizeof(x)` and nothing is ever checked.
//===----------------------------------------------------------------------===//

/// Called when an assertion inside Recast or Detour fails.
///
/// `expression`, `file` and `line` are upstream's own. Both strings are string
/// literals with static storage duration, so they outlive the call.
///
/// Returning is allowed, and execution then continues into the state the
/// assertion was guarding against — which is upstream's own behaviour for a
/// custom handler, not something added here. A handler that means to stop must
/// not return.
typedef void (*ZrcAssertFailFunc)(void* user, const char* expression,
                                  const char* file, int32_t line);

typedef struct ZrcAssertHandler {
  /// Invoked on a failed assertion. Must not be NULL.
  ZrcAssertFailFunc fail;
  /// Opaque host pointer, passed back unmodified.
  void* user;
} ZrcAssertHandler;

/// Whether this build kept Recast's and Detour's internal assertions.
///
/// ZRC_FALSE in a build compiled with NDEBUG, where zrcSetAssertHandler still
/// records the handler and nothing will ever call it.
ZRC_API ZrcBool zrcAssertsEnabled(void);

/// Installs a process-wide assertion handler for Recast and Detour both.
///
/// Global state, mirroring upstream's own design, and with the same caveat as
/// zrcSetAllocator: `handler` is copied by value, but `user` must outlive it.
/// Passing NULL restores upstream's default, which is assert().
///
/// Returns ZRC_ERR_INVALID_ARGUMENT if `handler->fail` is NULL, in which case
/// the previously installed handler is left untouched.
ZRC_API ZrcResult zrcSetAssertHandler(const ZrcAssertHandler* handler);

/// Reads back what zrcSetAssertHandler installed, whether or not this build can
/// call it. A cleared handler reports both fields NULL.
ZRC_API ZrcResult zrcAssertHandler(ZrcAssertHandler* out);

//===----------------------------------------------------------------------===//
// Geometry input
//===----------------------------------------------------------------------===//

/// An indexed triangle soup in Recast's coordinate convention: right-handed,
/// **Y up**, distances in world units.
///
/// Nothing about winding matters except for slope: a triangle's walkability is
/// decided from its normal, and the normal comes from the winding, so a
/// counter-clockwise-when-seen-from-above floor is what registers as a floor.
///
/// The arrays are borrowed for the duration of the bake call only.
typedef struct ZrcTriMesh {
  /// Vertex positions, `3 * vert_count` floats, (x, y, z) interleaved.
  const float* verts;
  int vert_count;
  /// Triangle corner indices into `verts`, `3 * tri_count` entries.
  const int* tris;
  int tri_count;
} ZrcTriMesh;

//===----------------------------------------------------------------------===//
// Baking (Recast — build time)
//===----------------------------------------------------------------------===//

/// How the walkable surface is split into regions before contours are traced.
typedef enum ZrcPartition {
  /// Watershed. Best-quality regions, slowest. The default.
  ZRC_PARTITION_WATERSHED = 0,
  /// Monotone. Fastest and never produces holes, but can produce long thin
  /// polygons.
  ZRC_PARTITION_MONOTONE = 1,
  /// Layer-based. A middle ground, and the one tiled meshes usually want.
  ZRC_PARTITION_LAYERS = 2,
} ZrcPartition;

/// Bake parameters, in **world units** except where noted.
///
/// Recast's own rcConfig mixes world units and voxels, which is a reliable
/// source of mistakes; this struct takes world units and derives the voxel
/// counts. Fill it with zrcBakeConfigDefault and adjust, rather than zeroing
/// it — a zeroed config is rejected, since cell_size 0 has no meaning.
///
/// Ratios matter as much as values. Each agent dimension is divided by a cell
/// size to reach voxels, and Recast stores those voxel counts in fields with
/// real limits — 13 bits for a span extent, a byte for the erosion threshold —
/// and squares them in plain `int` arithmetic. A ratio outside that range is
/// therefore rejected with ZRC_ERR_INVALID_ARGUMENT rather than clamped, since
/// clamping would bake a mesh for a different agent than the one described.
/// The bake log names the offending field.
typedef struct ZrcBakeConfig {
  /// Voxel size on the xz plane. The single most important knob: everything
  /// below is quantised to it. Roughly agent_radius / 2 is a common starting
  /// point. [Limit: > 0]
  float cell_size;
  /// Voxel size along y. Roughly cell_size / 2. [Limit: > 0]
  float cell_height;

  /// Agent height: the vertical clearance a walkable span must have.
  float agent_height;
  /// Agent radius: walkable area is eroded by this much away from obstacles.
  /// May be 0 for a point agent.
  float agent_radius;
  /// Tallest step the agent can climb without it counting as a wall.
  float agent_max_climb;
  /// Steepest floor the agent can stand on. [Limit: 0 <= value < 90] [Degrees]
  float agent_max_slope;

  /// Square root of the smallest region to keep, in voxels. Regions smaller
  /// than region_min_size^2 voxels are discarded as noise.
  float region_min_size;
  /// Square root of the largest region that may be merged into a neighbour, in
  /// voxels.
  float region_merge_size;

  /// Longest contour edge before it is subdivided. 0 disables subdivision.
  float edge_max_len;
  /// How far a simplified contour may deviate from the raw one. [Voxels]
  float edge_max_error;
  /// Maximum corners per polygon. [Limit: 3 <= value <= ZRC_VERTS_PER_POLYGON]
  int32_t verts_per_poly;

  /// Detail-mesh height sampling distance, as a multiple of cell_size.
  /// [Limit: 0 (off) or >= 0.9]
  float detail_sample_dist;
  /// How far the detail mesh may deviate from the heightfield, as a multiple
  /// of cell_height.
  float detail_sample_max_error;

  /// One of ZrcPartition.
  int32_t partition;

  /// Let the agent walk over small ledges such as stair nosings.
  ZrcBool filter_low_hanging_obstacles;
  /// Drop spans whose neighbours are too far below to step down to.
  ZrcBool filter_ledge_spans;
  /// Drop spans without agent_height of clearance above them.
  ZrcBool filter_walkable_low_height_spans;

  /// Tile edge, in **voxels** — the one field here that is not world units.
  ///
  /// Voxels rather than world units because the grid has to divide exactly: a
  /// tile edge of `tile_size * cell_size` puts every tile boundary on a cell
  /// boundary, and a rounded world-unit edge would not. 0 bakes a single tile
  /// covering the whole input, which is what zrcPolyMeshBake does.
  /// [Limit: 0, or 16 <= value <= 1024]
  int32_t tile_size;

  /// Width of the unnavigable border each tile is baked with, in voxels.
  ///
  /// A tile has to see a little of its neighbours to trace contours that meet
  /// theirs. That margin is discarded once the contours are traced, leaving a
  /// ring of the tile unnavigable.
  /// Negative derives it as the agent radius in voxels plus 3, the value
  /// Recast's own tiled build uses. Ignored when tile_size is 0.
  /// [Limit: negative (derive), or 0 <= value <= 255]
  int32_t border_size;
} ZrcBakeConfig;

/// Writes a configuration that bakes a sensible mesh for a human-sized agent
/// (0.6 m radius, 2.0 m tall) at 0.3 m resolution.
///
/// These are the values Recast's own solo-mesh sample ships with, and they are
/// a starting point rather than an answer: cell_size in particular has to suit
/// the scale of your geometry.
ZRC_API void zrcBakeConfigDefault(ZrcBakeConfig* out);

/// Optional destination for the messages Recast produces while baking.
///
/// Recast reports failure as a bare `false` and puts the reason in a log
/// callback, so without this a failed bake is undiagnosable. `buffer` is filled
/// with NUL-terminated text, truncated to fit, and is always NUL-terminated on
/// return provided `capacity` is non-zero.
typedef struct ZrcBakeLog {
  char* buffer;
  size_t capacity;
} ZrcBakeLog;

/// A baked polygon mesh: the artefact a cook produces and a navmesh is made
/// from. Holds Recast's polygon mesh and its detail (height) mesh together,
/// because Detour needs both, plus the agent dimensions in world units that
/// Recast does not carry and Detour demands.
///
/// zrcPolyMeshBake produces one in a single call. The staged pipeline further
/// down produces the same handle a stage at a time, and the two are
/// interchangeable everywhere.
typedef struct ZrcPolyMesh ZrcPolyMesh;

/// The tile grid a tiled bake and its navmesh share.
///
/// Both halves need the same grid or the tiles do not line up, so it is
/// computed once by zrcTileGridCompute and passed to both. Treat it as opaque
/// arithmetic rather than something to fill in by hand.
typedef struct ZrcTileGrid {
  /// Minimum corner of the whole input, and the navmesh's origin.
  float origin[3];
  /// Maximum corner of the whole input.
  float extent_max[3];
  /// Edge of one tile in world units: `tile_size * cell_size`.
  float tile_world_size;
  /// Tiles along x and z. Together they bound `tile_x` and `tile_z`.
  int32_t tile_count_x;
  int32_t tile_count_z;
} ZrcTileGrid;

/// Computes the tile grid `config` and `mesh` imply.
///
/// Returns ZRC_ERR_INVALID_ARGUMENT when `config->tile_size` is 0, because a
/// single-tile bake has no grid — use zrcPolyMeshBake for that.
ZRC_API ZrcResult zrcTileGridCompute(const ZrcBakeConfig* config,
                                     const ZrcTriMesh* mesh,
                                     ZrcTileGrid* out);

//===----------------------------------------------------------------------===//
// Constants mirrored from upstream
//
// Written out here so a C consumer need not include a Recast or Detour header.
// Every one is pinned to its upstream original by a static_assert in
// ffi/zrecast_abi.cpp, and reported by zrcAbiLayout so a binding can check it
// too. They sit ahead of the first declaration that uses one.
//===----------------------------------------------------------------------===//

/// Corners per polygon Detour supports (DT_VERTS_PER_POLYGON).
#define ZRC_VERTS_PER_POLYGON 6

/// Distinct area ids (DT_MAX_AREAS).
#define ZRC_MAX_AREAS 64

/// The area id Recast gives an unwalkable surface.
#define ZRC_AREA_NULL 0
/// The area id zrcPolyMeshBake gives a walkable surface.
#define ZRC_AREA_WALKABLE 63

/// The polygon flag zrcPolyMeshBake sets on every walkable polygon.
#define ZRC_POLY_FLAG_WALKABLE 0x0001

//===----------------------------------------------------------------------===//
// Area authoring
//
// A bake with no authoring gives every walkable polygon area ZRC_AREA_WALKABLE
// and flag ZRC_POLY_FLAG_WALKABLE, which is uniform navigable space. Volumes
// are what make one part of a level different from another: water to swim, mud
// that costs more to cross, a doorway only some agents may use, a ledge nothing
// may stand on.
//
// The two halves do different jobs and a host usually wants both. An **area id**
// is what the query filter charges for, through ZrcQueryFilter.area_cost, so it
// makes a region expensive rather than forbidden. A **polygon flag** is what the
// filter admits or refuses outright, through include_flags and exclude_flags.
//===----------------------------------------------------------------------===//

typedef enum ZrcVolumeShape {
  /// A convex polygon in the xz plane, extruded between y_min and y_max.
  ZRC_VOLUME_CONVEX = 0,
  /// An axis-aligned box.
  ZRC_VOLUME_BOX = 1,
  /// A vertical cylinder standing on its base.
  ZRC_VOLUME_CYLINDER = 2,
} ZrcVolumeShape;

/// One volume of the world, and the area id the surface inside it takes.
typedef struct ZrcAreaVolume {
  /// Which of the fields below are read.
  ZrcVolumeShape shape;

  /// Area id written where this volume covers walkable surface.
  /// [Limit: 0 <= area < ZRC_MAX_AREAS]
  ///
  /// ZRC_AREA_NULL is not merely an id: it makes the surface unwalkable, which
  /// is how a volume carves a hole rather than colouring one.
  int32_t area;

  /// The volume's vertical extent, world units. Every shape uses both.
  float y_min;
  float y_max;

  /// ZRC_VOLUME_CONVEX: the footprint, `3 * vert_count` floats laid out
  /// (x, y, z). The y of each vertex is **ignored** — y_min and y_max carry the
  /// extent — so the same array a renderer holds can be passed straight in.
  /// Every float must still be finite, y included: a NaN reaching Recast's
  /// point-in-polygon test makes the comparison's outcome a property of the
  /// platform. NULL for the other shapes.
  const float* verts;
  /// [Limit: 3 <= vert_count <= 1048576 for a convex volume]
  int32_t vert_count;

  /// ZRC_VOLUME_BOX: the two corners in xz, `{x, z}` each.
  /// ZRC_VOLUME_CYLINDER: `xz_min` is the base centre; `xz_max` is unused.
  /// Both unused by ZRC_VOLUME_CONVEX.
  float xz_min[2];
  float xz_max[2];
  /// ZRC_VOLUME_CYLINDER only. [Limit: > 0]
  float radius;
} ZrcAreaVolume;

/// What a bake writes into the mesh beyond the shape of the geometry.
///
/// Every field is optional and the whole struct may be NULL, which is the
/// uniform-space default described above.
typedef struct ZrcAreaAuthoring {
  /// Applied in order, after the walkable surface has been eroded by the agent
  /// radius and before it is cut into regions — the same point Recast's own
  /// pipeline applies them at. Later volumes overwrite earlier ones where they
  /// overlap, so the order is the layering.
  const ZrcAreaVolume* volumes;
  /// [Limit: 0 <= volume_count <= 65536, and volumes is non-NULL above zero]
  ///
  /// The ceiling is arbitrary but stated: a count read from a file is a count
  /// that can be wrong, and a wrong one should be an error rather than a very
  /// long loop.
  int32_t volume_count;

  /// The polygon flags each area id receives, indexed by area id, with exactly
  /// ZRC_MAX_AREAS entries.
  ///
  /// NULL keeps the default: ZRC_POLY_FLAG_WALKABLE for every area except
  /// ZRC_AREA_NULL, which gets none. Flags are assigned from the area id rather
  /// than from the volume because Recast carries only the area id through the
  /// pipeline — by the time polygons exist, which volume produced one is no
  /// longer knowable.
  ///
  /// [Limit: entry ZRC_AREA_NULL must be 0 — an unwalkable polygon that some
  /// filter still admits is a polygon a path can cross]
  const uint16_t* area_flags;
} ZrcAreaAuthoring;

/// Runs the whole Recast pipeline over `mesh`:
///
///   heightfield -> filters -> compact heightfield -> erode -> **volumes** ->
///   regions -> contours -> polygon mesh -> detail mesh
///
/// `authoring` and `log` may both be NULL. On failure `*out` is left NULL.
///
/// With no authoring, every polygon that comes out walkable is given area
/// ZRC_AREA_WALKABLE and flags ZRC_POLY_FLAG_WALKABLE, so the default query
/// filter accepts it. A polygon with zero flags is invisible to every query,
/// which is a confusing way to get an empty path; assigning them here avoids it.
ZRC_API ZrcResult zrcPolyMeshBake(const ZrcBakeConfig* config,
                                  const ZrcTriMesh* mesh,
                                  const ZrcAreaAuthoring* authoring,
                                  ZrcBakeLog* log, ZrcPolyMesh** out);

/// Bakes one tile of a tiled navmesh.
///
/// Same pipeline as zrcPolyMeshBake, restricted to the tile at
/// (`tile_x`, `tile_z`) and padded by the configured border so its contours
/// meet its neighbours'. `mesh` is the whole world's geometry every time;
/// rasterisation clips it to the tile.
///
/// **An empty tile is success, not failure.** Most tiles of a real world hold
/// no walkable surface, and `*out` is then NULL with ZRC_OK returned. This is
/// the one place the two bake entry points differ in contract:
/// zrcPolyMeshBake reports ZRC_ERR_EMPTY_RESULT, because a whole-world bake
/// that found nothing walkable is a mistake rather than a fact about geography.
///
/// `authoring` is the whole world's, same as `mesh`: a volume outside this tile
/// marks nothing, and a volume crossing a tile edge marks each side from the
/// same numbers, so the tiles agree without being told about each other.
///
/// `config` must be the one `grid` was computed from: `tile_size * cell_size`
/// has to equal the grid's `tile_world_size` exactly, or the tiles land at one
/// spacing under a navmesh addressing them at another and nothing later fails.
/// A mismatch is ZRC_ERR_INVALID_ARGUMENT.
ZRC_API ZrcResult zrcPolyMeshBakeTile(const ZrcBakeConfig* config,
                                      const ZrcTriMesh* mesh,
                                      const ZrcTileGrid* grid, int32_t tile_x,
                                      int32_t tile_z,
                                      const ZrcAreaAuthoring* authoring,
                                      ZrcBakeLog* log, ZrcPolyMesh** out);

ZRC_API void zrcPolyMeshDestroy(ZrcPolyMesh* mesh);

//===----------------------------------------------------------------------===//
// The staged Recast pipeline
//
// zrcPolyMeshBake runs Recast end to end and hands back the result. This is the
// same pipeline taken apart: each stage is its own entry point, each
// intermediate container is a handle a host owns, and every stage can be
// inspected, edited or replaced between one call and the next. It is what a
// tool needs — a visualiser that draws the heightfield, a cook that marks areas
// from its own data after erosion, a build that reuses one compact heightfield
// for several region strategies.
//
// The stages, in the only order Recast accepts:
//
//   zrcHeightfieldCreate
//     -> rasterize (a triangle, an indexed mesh, or a soup) / add spans
//     -> filters
//   zrcCompactHeightfieldCreate
//     -> erode -> median filter -> mark areas
//     -> build the distance field -> build regions
//   zrcContourSetCreate
//   zrcPolyMeshCreate -> zrcPolyMeshBuild -> zrcPolyMeshBuildDetail
//
// Recast's stages assume their output container is empty and overwrite its
// buffer pointers without checking, which leaks every allocation the container
// held. Two of them check with rcAssert, which compiles to nothing in a release
// build. Handing a host the containers is what makes that reachable, so every
// stage here refuses a container that already holds a result, with
// ZRC_ERR_ALREADY_BUILT, in every build configuration. See UPSTREAM.md.
//
// How a container answers "what is in you" is uniform here and among the tile
// cache's containers below. One carrying aggregate data beyond its element
// count answers through zrc<Container>Info, filling a Zrc<Container>Info
// struct. One that is nothing but a sized array answers through
// zrc<Container>Count, a struct around a single int being ceremony rather than
// symmetry: zrcHeightfieldLayerSetCount and zrcTileCacheContourSetCount are
// the two on that arm, and rcHeightfieldLayerSet and dtTileCacheContourSet
// hold nothing else to report.
//===----------------------------------------------------------------------===//

/// Which of Recast's build messages a log entry is.
typedef enum ZrcLogCategory {
  ZRC_LOG_PROGRESS = 1,
  ZRC_LOG_WARNING = 2,
  ZRC_LOG_ERROR = 3,
} ZrcLogCategory;

/// The build phases Recast times, and the count that bounds them.
///
/// Every stage entry point below starts and stops the labels its upstream
/// function does, so a host that installs timers measures the same phases a C++
/// host measures.
typedef enum ZrcTimerLabel {
  /// The whole build. Recast never starts this one: it is the host's to use.
  ZRC_TIMER_TOTAL = 0,
  /// Also the host's, for whatever it wants to measure.
  ZRC_TIMER_TEMP = 1,
  ZRC_TIMER_RASTERIZE_TRIANGLES = 2,
  ZRC_TIMER_BUILD_COMPACTHEIGHTFIELD = 3,
  ZRC_TIMER_BUILD_CONTOURS = 4,
  ZRC_TIMER_BUILD_CONTOURS_TRACE = 5,
  ZRC_TIMER_BUILD_CONTOURS_SIMPLIFY = 6,
  ZRC_TIMER_FILTER_BORDER = 7,
  ZRC_TIMER_FILTER_WALKABLE = 8,
  ZRC_TIMER_MEDIAN_AREA = 9,
  ZRC_TIMER_FILTER_LOW_OBSTACLES = 10,
  ZRC_TIMER_BUILD_POLYMESH = 11,
  ZRC_TIMER_MERGE_POLYMESH = 12,
  ZRC_TIMER_ERODE_AREA = 13,
  ZRC_TIMER_MARK_BOX_AREA = 14,
  ZRC_TIMER_MARK_CYLINDER_AREA = 15,
  ZRC_TIMER_MARK_CONVEXPOLY_AREA = 16,
  ZRC_TIMER_BUILD_DISTANCEFIELD = 17,
  ZRC_TIMER_BUILD_DISTANCEFIELD_DIST = 18,
  ZRC_TIMER_BUILD_DISTANCEFIELD_BLUR = 19,
  ZRC_TIMER_BUILD_REGIONS = 20,
  ZRC_TIMER_BUILD_REGIONS_WATERSHED = 21,
  ZRC_TIMER_BUILD_REGIONS_EXPAND = 22,
  ZRC_TIMER_BUILD_REGIONS_FLOOD = 23,
  ZRC_TIMER_BUILD_REGIONS_FILTER = 24,
  ZRC_TIMER_BUILD_LAYERS = 25,
  ZRC_TIMER_BUILD_POLYMESHDETAIL = 26,
  ZRC_TIMER_MERGE_POLYMESHDETAIL = 27,
  /// One past the last label, and the length of a table indexed by one.
  ZRC_MAX_TIMERS = 28,
} ZrcTimerLabel;

/// Where a build's log messages and timings go.
///
/// Recast's rcContext is a class with two enable flags and six protected
/// virtuals; this is the same object as a POD. Every hook may be NULL
/// individually, and the whole struct may be NULL wherever one is asked for,
/// which is a context with both flags clear.
///
/// The flags are not a convenience. Upstream consults them before it reaches a
/// hook, and `accumulated_time` answers -1 without asking when
/// `timers_enabled` is false, so a struct of hooks alone would not behave the
/// way subclassing rcContext behaves.
typedef struct ZrcBuildContext {
  /// Passed back to every hook untouched.
  void* user;

  /// Called for each message. `message` is NUL-terminated and `length` is its
  /// length, both borrowed for the duration of the call.
  ///
  /// A message longer than 511 bytes arrives as **two** calls: first a
  /// ZRC_LOG_ERROR reading "Log message was truncated", at a category the
  /// caller did not choose, then the message cut to 511 bytes. A host counting
  /// entries, or trusting the category, has to expect it.
  void (*log)(void* user, ZrcLogCategory category, const char* message,
              int32_t length);
  /// Discard whatever the log has accumulated.
  void (*reset_log)(void* user);

  /// Reset every timer to unused.
  void (*reset_timers)(void* user);
  void (*start_timer)(void* user, ZrcTimerLabel label);
  void (*stop_timer)(void* user, ZrcTimerLabel label);
  /// Total time accumulated on `label`, in **microseconds**, or -1 for a timer
  /// that has never run.
  int32_t (*accumulated_time)(void* user, ZrcTimerLabel label);

  /// Clear to silence `log` and `reset_log` without removing them.
  ZrcBool log_enabled;
  /// Clear to silence the four timer hooks. `accumulated_time` is then never
  /// called and answers -1.
  ZrcBool timers_enabled;
} ZrcBuildContext;

/// Logs a message through `context`, as a Recast stage would.
///
/// `message` is NUL-terminated. Truncation is upstream's, described on
/// ZrcBuildContext.log; the message crosses already formatted, because a
/// varargs entry point cannot be checked at a C ABI boundary.
ZRC_API ZrcResult zrcBuildContextLog(const ZrcBuildContext* context,
                                     ZrcLogCategory category,
                                     const char* message);

ZRC_API ZrcResult zrcBuildContextResetLog(const ZrcBuildContext* context);
ZRC_API ZrcResult zrcBuildContextResetTimers(const ZrcBuildContext* context);
ZRC_API ZrcResult zrcBuildContextStartTimer(const ZrcBuildContext* context,
                                            ZrcTimerLabel label);
ZRC_API ZrcResult zrcBuildContextStopTimer(const ZrcBuildContext* context,
                                           ZrcTimerLabel label);

/// Microseconds accumulated on `label`, or -1 when timers are disabled or the
/// timer has never run.
ZRC_API ZrcResult zrcBuildContextAccumulatedTime(const ZrcBuildContext* context,
                                                 ZrcTimerLabel label,
                                                 int32_t* out_time);

//===----------------------------------------------------------------------===//
// Sizing a build
//===----------------------------------------------------------------------===//

/// The voxel quantities a ZrcBakeConfig implies.
///
/// The staged entry points take voxels, because Recast's own stages do.
/// ZrcBakeConfig describes an agent in world units. This is the conversion
/// between them, and it is the conversion zrcPolyMeshBake performs internally,
/// so a host that drives the stages with these numbers reproduces a bake
/// exactly rather than approximately.
typedef struct ZrcBuildCells {
  /// For zrcCompactHeightfieldCreate and two of the filters.
  int32_t walkable_height;
  /// For zrcCompactHeightfieldCreate, two of the filters, and the merge
  /// threshold the rasterisers take.
  int32_t walkable_climb;
  /// For zrcCompactHeightfieldErode. Zero means no erosion is needed.
  int32_t walkable_radius;
  /// For zrcContourSetCreate. Clamped to the per-axis voxel ceiling, which is
  /// the same as no subdivision at all.
  int32_t max_edge_len;
  /// For zrcCompactHeightfieldBuildRegions, in cells squared.
  int32_t min_region_area;
  int32_t merge_region_area;
  /// For zrcCompactHeightfieldBuildRegions. Derived from the walkable radius
  /// when the configuration asked for that; 0 for an untiled build.
  int32_t border_size;
  /// For zrcContourSetCreate, in cells.
  float max_simplification_error;
  /// For zrcPolyMeshBuild.
  int32_t verts_per_poly;
  /// For zrcPolyMeshBuildDetail, in world units.
  float detail_sample_dist;
  float detail_sample_max_error;
} ZrcBuildCells;

/// Converts `config` into the voxel quantities the staged entry points take.
///
/// Fails with ZRC_ERR_INVALID_ARGUMENT for exactly the configurations
/// zrcPolyMeshBake rejects, including the ratios that would leave a quantity
/// too large for the field Recast stores it in.
ZRC_API ZrcResult zrcBakeConfigCells(const ZrcBakeConfig* config,
                                     ZrcBuildCells* out);

/// Axis-aligned bounds of a triangle mesh's vertices. Both arrays are 3 floats.
ZRC_API ZrcResult zrcCalcBounds(const ZrcTriMesh* mesh, float* bmin,
                                float* bmax);

/// Width and height, in cells, of the voxel grid `bmin`..`bmax` implies at
/// `cell_size`.
ZRC_API ZrcResult zrcCalcGridSize(const float* bmin, const float* bmax,
                                  float cell_size, int32_t* out_width,
                                  int32_t* out_height);

/// Writes ZRC_AREA_WALKABLE into `out_areas` for every triangle whose slope is
/// within `walkable_slope_angle`, and leaves the rest untouched.
///
/// `out_areas` has one byte per triangle. Untouched means untouched: a host
/// zeroes the array first, or reuses one it has already marked.
///
/// The slope threshold is computed from a polynomial cosine rather than the
/// host's `cosf`, which the C standard does not require to be correctly
/// rounded. Two platforms disagreeing in the last bit of that threshold would
/// disagree about whether a surface is navigable, and the cook would depend
/// on the host's libm. zrcPolyMeshBake uses the same threshold,
/// so a mesh assembled a stage at a time matches one it bakes.
ZRC_API ZrcResult zrcMarkWalkableTriangles(const ZrcBuildContext* context,
                                           float walkable_slope_angle,
                                           const ZrcTriMesh* mesh,
                                           uint8_t* out_areas);

/// Writes ZRC_AREA_NULL into `io_areas` for every triangle steeper than
/// `walkable_slope_angle`, and leaves the rest untouched. The inverse of
/// zrcMarkWalkableTriangles, for an array a host has already filled in, and
/// with the same platform-stable slope threshold.
ZRC_API ZrcResult zrcClearUnwalkableTriangles(const ZrcBuildContext* context,
                                              float walkable_slope_angle,
                                              const ZrcTriMesh* mesh,
                                              uint8_t* io_areas);

//===----------------------------------------------------------------------===//
// The heightfield
//===----------------------------------------------------------------------===//

/// Bits Recast gives a span's lower and upper extent (RC_SPAN_HEIGHT_BITS).
#define ZRC_SPAN_HEIGHT_BITS 13
/// Largest value either extent can hold (RC_SPAN_MAX_HEIGHT).
#define ZRC_SPAN_MAX_HEIGHT 8191
/// Spans in one of a heightfield's allocation pools (RC_SPANS_PER_POOL).
#define ZRC_SPANS_PER_POOL 2048

/// A voxel column's obstructed interval, copied out.
///
/// Upstream packs these three into one 32-bit word with a `next` pointer;
/// crossing the boundary they are whole fields, because a bitfield's layout is
/// the compiler's business and a host cannot rely on it.
typedef struct ZrcSpan {
  /// Lower extent, in cells above the heightfield's minimum corner.
  /// [Limit: < smax]
  uint32_t smin;
  /// Upper extent. [Limit: <= ZRC_SPAN_MAX_HEIGHT]
  uint32_t smax;
  /// [Limit: 0 <= area < ZRC_MAX_AREAS]
  uint8_t area;
} ZrcSpan;

/// A heightfield: the voxelisation of the input geometry.
typedef struct ZrcHeightfield ZrcHeightfield;

typedef struct ZrcHeightfieldInfo {
  /// Cells along x and z.
  int32_t width;
  int32_t height;
  float bmin[3];
  float bmax[3];
  float cell_size;
  float cell_height;
} ZrcHeightfieldInfo;

/// How much span storage a heightfield holds.
///
/// Spans are handed out from pools of ZRC_SPANS_PER_POOL and returned to a free
/// list rather than to the allocator, so a heightfield's memory is
/// `pool_count * ZRC_SPANS_PER_POOL` spans however many are live. `free_count`
/// is how many of those are on the free list.
typedef struct ZrcHeightfieldStorage {
  int32_t pool_count;
  int32_t free_count;
  int32_t spans_per_pool;
} ZrcHeightfieldStorage;

/// Allocates a heightfield of `width` x `height` cells over `bmin`..`bmax`.
///
/// The same grid zrcCalcGridSize computes and the same bounds it applies, so a
/// host that sizes a field from a mesh and one that names the size itself are
/// held to one rule. Upstream stores both sizes as they arrive and allocates
/// `width * height` span pointers from their product in plain `int`
/// arithmetic, which two large sizes overflow before the allocation is even
/// attempted.
///
/// The vertical extent is bounded too, and separately: Recast never turns it
/// into a grid size, so nothing else would catch it, and a rasteriser converts
/// a height to a cell index before it clamps. `(bmax[1] - bmin[1]) /
/// cell_height` must be under ZRC_SPAN_MAX_HEIGHT, which is all a span's
/// 13-bit extent can address in any case.
///
/// [Limit: 0 < width, height <= 32767; width * height <= 268435456; bmin
/// no greater than bmax on every axis; every float finite; cell_size and
/// cell_height above 0]
ZRC_API ZrcResult zrcHeightfieldCreate(const ZrcBuildContext* context,
                                       int32_t width, int32_t height,
                                       const float* bmin, const float* bmax,
                                       float cell_size, float cell_height,
                                       ZrcHeightfield** out);

ZRC_API void zrcHeightfieldDestroy(ZrcHeightfield* heightfield);

ZRC_API ZrcResult zrcHeightfieldInfo(const ZrcHeightfield* heightfield,
                                     ZrcHeightfieldInfo* out);

ZRC_API ZrcResult zrcHeightfieldStorage(const ZrcHeightfield* heightfield,
                                        ZrcHeightfieldStorage* out);

/// Copies the spans of the column at (`x`, `z`), lowest first, into `out`.
///
/// `*out_count` is how many the column holds, written whether or not they fit.
/// A column with more spans than `max_spans` writes the first `max_spans` and
/// returns ZRC_ERR_BUFFER_TOO_SMALL, so a second call with the right size
/// finishes the job. `out` may be NULL when `max_spans` is 0, which asks only
/// for the count.
ZRC_API ZrcResult zrcHeightfieldColumn(const ZrcHeightfield* heightfield,
                                       int32_t x, int32_t z, ZrcSpan* out,
                                       int32_t max_spans, int32_t* out_count);

/// Total spans across every column. Walks the whole field.
ZRC_API ZrcResult zrcHeightfieldSpanCount(const ZrcBuildContext* context,
                                          const ZrcHeightfield* heightfield,
                                          int32_t* out_count);

/// Adds one span to the column at (`x`, `z`), merging it with any it overlaps.
///
/// `flag_merge_threshold` is how close in cells two merged spans' upper extents
/// must be for the higher one's area to win.
///
/// Upstream indexes `spans[x + z * width]` with no bound of its own, so the
/// coordinates are checked here. [Limit: 0 <= x < width, 0 <= z < height,
/// span_min < span_max <= ZRC_SPAN_MAX_HEIGHT, area < ZRC_MAX_AREAS]
ZRC_API ZrcResult zrcHeightfieldAddSpan(const ZrcBuildContext* context,
                                        ZrcHeightfield* heightfield, int32_t x,
                                        int32_t z, uint32_t span_min,
                                        uint32_t span_max, uint8_t area,
                                        int32_t flag_merge_threshold);

/// Rasterises one triangle into the heightfield. Each vertex is 3 floats.
///
/// Every vertex must sit at a distance from the field's minimum corner that
/// divides by `cell_size` into a cell index. Upstream skips a triangle wholly
/// outside the field, but one that straddles it reaches an out-of-range
/// float-to-int conversion with whatever the far corner holds, so a finite
/// coordinate is not on its own enough.
ZRC_API ZrcResult zrcHeightfieldRasterizeTriangle(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    const float* v0, const float* v1, const float* v2, uint8_t area,
    int32_t flag_merge_threshold);

/// Rasterises an indexed mesh, one area byte per triangle.
///
/// Upstream reads `verts[tris[i] * 3]` without checking the index against
/// `vert_count`; every index is bounded here first, and every vertex against
/// the field it is going into, as zrcHeightfieldRasterizeTriangle describes.
/// Each area must be below ZRC_MAX_AREAS.
ZRC_API ZrcResult zrcHeightfieldRasterizeTriangles(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    const ZrcTriMesh* mesh, const uint8_t* tri_areas,
    int32_t flag_merge_threshold);

/// Rasterises an indexed mesh whose indices are 16 bits wide.
///
/// Same contract as zrcHeightfieldRasterizeTriangles; separate because upstream
/// keeps the two index widths as separate functions and a host holding 16-bit
/// indices should not have to widen them.
ZRC_API ZrcResult zrcHeightfieldRasterizeTrianglesU16(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    const float* verts, int32_t vert_count, const uint16_t* tris,
    const uint8_t* tri_areas, int32_t tri_count,
    int32_t flag_merge_threshold);

/// Rasterises an unindexed soup: `3 * tri_count` vertices, three per triangle.
ZRC_API ZrcResult zrcHeightfieldRasterizeTriangleSoup(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    const float* verts, const uint8_t* tri_areas, int32_t tri_count,
    int32_t flag_merge_threshold);

/// Marks spans that a `walkable_climb` step reaches over as walkable, so an
/// agent can climb a stair nosing or a kerb.
ZRC_API ZrcResult zrcHeightfieldFilterLowHangingObstacles(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    int32_t walkable_climb);

/// Drops spans on a ledge: those whose drop to a neighbour exceeds
/// `walkable_climb`, which an agent standing there would fall off.
ZRC_API ZrcResult zrcHeightfieldFilterLedgeSpans(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    int32_t walkable_height, int32_t walkable_climb);

/// Drops spans with less than `walkable_height` cells of clearance above them.
ZRC_API ZrcResult zrcHeightfieldFilterWalkableLowHeightSpans(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    int32_t walkable_height);

//===----------------------------------------------------------------------===//
// The compact heightfield
//===----------------------------------------------------------------------===//

/// The value ZrcCompactSpan's connection field holds for a direction with no
/// neighbour (RC_NOT_CONNECTED).
#define ZRC_NOT_CONNECTED 0x3f
/// Region id bit marking a border region, whose spans are unwalkable
/// (RC_BORDER_REG).
#define ZRC_BORDER_REG 0x8000

/// Where one column's spans start in the span array, and how many there are.
typedef struct ZrcCompactCell {
  /// [Limit: < 2^24]
  uint32_t index;
  /// [Limit: < 256]
  uint32_t count;
} ZrcCompactCell;

/// A span of open space, copied out of upstream's packed representation.
typedef struct ZrcCompactSpan {
  /// Lower extent, in cells above the field's minimum corner.
  uint16_t y;
  /// Region id, or 0 for none. ZRC_BORDER_REG marks a border region.
  uint16_t reg;
  /// Four 6-bit neighbour indices, one per direction, each ZRC_NOT_CONNECTED
  /// where there is no neighbour. Decode with the connection helpers rather
  /// than by hand. [Limit: < 2^24]
  uint32_t con;
  /// Height of the open space above `y`, in cells. [Limit: < 256]
  uint32_t h;
} ZrcCompactSpan;

/// The walkable surface, rebuilt as open space instead of obstruction.
typedef struct ZrcCompactHeightfield ZrcCompactHeightfield;

typedef struct ZrcCompactHeightfieldInfo {
  int32_t width;
  int32_t height;
  /// Length of the span, distance and area arrays.
  int32_t span_count;
  int32_t walkable_height;
  int32_t walkable_climb;
  int32_t border_size;
  /// Largest value in the distance field, or 0 before it is built.
  uint16_t max_distance;
  /// Largest region id assigned, or 0 before regions are built.
  uint16_t max_regions;
  float bmin[3];
  float bmax[3];
  float cell_size;
  float cell_height;
  /// Whether the distance field has been built, and so whether
  /// zrcCompactHeightfieldDistances has anything to report.
  ZrcBool has_distance_field;
} ZrcCompactHeightfieldInfo;

/// Builds the compact heightfield of the walkable surface in `heightfield`.
///
/// `walkable_height` and `walkable_climb` are in cells.
ZRC_API ZrcResult zrcCompactHeightfieldCreate(
    const ZrcBuildContext* context, int32_t walkable_height,
    int32_t walkable_climb, const ZrcHeightfield* heightfield,
    ZrcCompactHeightfield** out);

ZRC_API void zrcCompactHeightfieldDestroy(ZrcCompactHeightfield* field);

ZRC_API ZrcResult zrcCompactHeightfieldInfo(
    const ZrcCompactHeightfield* field, ZrcCompactHeightfieldInfo* out);

/// Copies `count` cells from `first`. The array is `width * height` long, in
/// row-major order: the cell at (x, z) is at `x + z * width`.
ZRC_API ZrcResult zrcCompactHeightfieldCells(const ZrcCompactHeightfield* field,
                                             int32_t first, int32_t count,
                                             ZrcCompactCell* out);

/// Copies `count` spans from `first`. The array is `span_count` long.
ZRC_API ZrcResult zrcCompactHeightfieldSpans(const ZrcCompactHeightfield* field,
                                             int32_t first, int32_t count,
                                             ZrcCompactSpan* out);

/// Writes `count` spans back, starting at `first`.
///
/// Every packed field is checked against the width upstream gives it, because
/// a value that does not fit is silently truncated on the way into the
/// bitfield and reads back as a different span.
ZRC_API ZrcResult zrcCompactHeightfieldSetSpans(ZrcCompactHeightfield* field,
                                                int32_t first, int32_t count,
                                                const ZrcCompactSpan* spans);

/// Copies `count` distance-field values from `first`, one per span.
///
/// ZRC_ERR_NOT_FOUND before zrcCompactHeightfieldBuildDistanceField has run:
/// the array does not exist yet, and upstream would read through a null
/// pointer.
ZRC_API ZrcResult zrcCompactHeightfieldDistances(
    const ZrcCompactHeightfield* field, int32_t first, int32_t count,
    uint16_t* out);

/// Copies `count` area ids from `first`, one per span.
ZRC_API ZrcResult zrcCompactHeightfieldAreas(const ZrcCompactHeightfield* field,
                                             int32_t first, int32_t count,
                                             uint8_t* out);

/// Writes `count` area ids back, starting at `first`. Each must be less than
/// ZRC_MAX_AREAS.
///
/// This is how a host marks areas from data the volume shapes cannot express —
/// a painted mask, a per-triangle table carried through rasterisation.
ZRC_API ZrcResult zrcCompactHeightfieldSetAreas(ZrcCompactHeightfield* field,
                                                int32_t first, int32_t count,
                                                const uint8_t* areas);

/// Marks the volumes in `authoring` into the field's area array, exactly as a
/// bake does. `authoring->area_flags` is unused here: flags are assigned to
/// polygons, which do not exist yet.
ZRC_API ZrcResult zrcCompactHeightfieldMarkAreas(
    const ZrcBuildContext* context, ZrcCompactHeightfield* field,
    const ZrcAreaAuthoring* authoring);

/// Erodes the walkable area by `radius` cells, so the surface that survives is
/// where an agent's centre may stand.
///
/// [Limit: 0 <= radius <= 127] Upstream compares distances against
/// `(unsigned char)(radius * 2)`, so a radius of 128 wraps to eroding nothing
/// and 129 erodes as 1 would. See UPSTREAM.md.
ZRC_API ZrcResult zrcCompactHeightfieldErode(const ZrcBuildContext* context,
                                             ZrcCompactHeightfield* field,
                                             int32_t radius);

/// Applies a median filter to the area ids, smoothing away single-span noise
/// without moving a boundary.
ZRC_API ZrcResult zrcCompactHeightfieldMedianFilter(
    const ZrcBuildContext* context, ZrcCompactHeightfield* field);

/// Builds the distance field: for every span, how far it is from the nearest
/// unwalkable one. Required before ZRC_PARTITION_WATERSHED regions.
ZRC_API ZrcResult zrcCompactHeightfieldBuildDistanceField(
    const ZrcBuildContext* context, ZrcCompactHeightfield* field);

/// Splits the walkable surface into regions, by one of the three strategies
/// ZrcPartition names.
///
/// `border_size` is the unnavigable ring, in cells, that a tiled build needs.
/// `min_region_area` and `merge_region_area` are in cells squared;
/// `merge_region_area` is unused by ZRC_PARTITION_LAYERS, which never merges.
/// [Limit: 0 <= border_size <= 255, the same ceiling a bake configuration
/// carries; the two areas non-negative]
///
/// ZRC_PARTITION_WATERSHED reads the distance field and is refused with
/// ZRC_ERR_INVALID_ARGUMENT before it has been built, rather than reading
/// through the null pointer upstream would.
ZRC_API ZrcResult zrcCompactHeightfieldBuildRegions(
    const ZrcBuildContext* context, ZrcCompactHeightfield* field,
    ZrcPartition partition, int32_t border_size, int32_t min_region_area,
    int32_t merge_region_area);

//===----------------------------------------------------------------------===//
// Contours
//===----------------------------------------------------------------------===//

/// Contour vertex flag: the vertex lies on a tile border (RC_BORDER_VERTEX).
#define ZRC_BORDER_VERTEX 0x10000
/// Contour vertex flag: the vertex lies on the border of an area
/// (RC_AREA_BORDER).
#define ZRC_AREA_BORDER 0x20000
/// Mask isolating the region id from a contour vertex's fourth component
/// (RC_CONTOUR_REG_MASK).
#define ZRC_CONTOUR_REG_MASK 0xffff

/// What a contour build tessellates beyond what simplification would keep.
///
/// ZRC_CONTOUR_TESS_WALL_EDGES, not 0, is what upstream's own default argument
/// supplies and what zrcPolyMeshBake therefore uses. A staged cook that passes
/// 0 produces a different mesh from a bake of the same geometry — a legitimate
/// choice, but not the one a C++ host who omitted the argument would get.
typedef enum ZrcContourFlags {
  ZRC_CONTOUR_NONE = 0,
  /// Tessellate edges against impassable space.
  ZRC_CONTOUR_TESS_WALL_EDGES = 0x01,
  /// Tessellate edges between two different areas.
  ZRC_CONTOUR_TESS_AREA_EDGES = 0x02,
} ZrcContourFlags;

/// The outlines of every region, traced and simplified.
typedef struct ZrcContourSet ZrcContourSet;

typedef struct ZrcContourSetInfo {
  int32_t contour_count;
  float bmin[3];
  float bmax[3];
  float cell_size;
  float cell_height;
  int32_t width;
  int32_t height;
  int32_t border_size;
  /// The simplification error the set was built with, in cells.
  float max_error;
} ZrcContourSetInfo;

/// One region's outline.
typedef struct ZrcContourInfo {
  /// Vertices of the simplified outline.
  int32_t vert_count;
  /// Vertices of the raw traced outline, before simplification.
  int32_t raw_vert_count;
  /// The region this outline encloses.
  uint16_t region;
  /// The area id of that region.
  uint8_t area;
} ZrcContourInfo;

/// Traces and simplifies the outline of every region in `field`.
///
/// `max_error` is how far a simplified edge may sit from the traced one, in
/// cells; `max_edge_len` is the longest edge before it is subdivided, 0 for no
/// limit. `flags` is a bitwise or of ZrcContourFlags — pass
/// ZRC_CONTOUR_TESS_WALL_EDGES to match what a bake does.
ZRC_API ZrcResult zrcContourSetCreate(const ZrcBuildContext* context,
                                      const ZrcCompactHeightfield* field,
                                      float max_error, int32_t max_edge_len,
                                      int32_t flags, ZrcContourSet** out);

ZRC_API void zrcContourSetDestroy(ZrcContourSet* contours);

ZRC_API ZrcResult zrcContourSetInfo(const ZrcContourSet* contours,
                                    ZrcContourSetInfo* out);

ZRC_API ZrcResult zrcContourAt(const ZrcContourSet* contours, int32_t index,
                               ZrcContourInfo* out);

/// Copies `count` simplified vertices of contour `index`, from `first`, four
/// ints each: x, y, z in cells, then the region id of the neighbour across the
/// edge that starts here, carrying ZRC_BORDER_VERTEX and ZRC_AREA_BORDER above
/// ZRC_CONTOUR_REG_MASK.
ZRC_API ZrcResult zrcContourVerts(const ZrcContourSet* contours, int32_t index,
                                  int32_t first, int32_t count, int32_t* out);

/// The same for the raw traced outline, which has not been simplified.
ZRC_API ZrcResult zrcContourRawVerts(const ZrcContourSet* contours,
                                     int32_t index, int32_t first,
                                     int32_t count, int32_t* out);

//===----------------------------------------------------------------------===//
// The polygon mesh, built by hand
//
// ZrcPolyMesh is the same handle zrcPolyMeshBake produces, so a mesh assembled
// stage by stage goes into zrcTileDataBuild exactly as a baked one does. It
// holds two halves — the polygons and the detail heights — filled by two
// stages, and the agent dimensions Detour needs and Recast does not carry.
//===----------------------------------------------------------------------===//

/// The vertex index Recast writes where a polygon has no further corner and no
/// neighbour (RC_MESH_NULL_IDX).
#define ZRC_MESH_NULL_IDX 0xffff
/// The region id of a polygon merged from several regions (RC_MULTIPLE_REGS).
#define ZRC_MULTIPLE_REGS 0

typedef struct ZrcPolyMeshInfo {
  int32_t vert_count;
  int32_t poly_count;
  /// Polygons the arrays have room for, which is what indexes `polys`.
  int32_t max_polys;
  /// Corners per polygon, and half the stride of a `polys` entry.
  int32_t verts_per_poly;
  float bmin[3];
  float bmax[3];
  float cell_size;
  float cell_height;
  int32_t border_size;
  float max_edge_error;
  /// The detail half. All three are 0 until zrcPolyMeshBuildDetail has run.
  int32_t detail_mesh_count;
  int32_t detail_vert_count;
  int32_t detail_tri_count;
  /// The dimensions zrcPolyMeshSetAgentDims recorded, world units.
  float walkable_height;
  float walkable_radius;
  float walkable_climb;
} ZrcPolyMeshInfo;

/// Allocates an empty polygon mesh, for zrcPolyMeshBuild, zrcPolyMeshCopy or
/// zrcPolyMeshMerge to fill.
ZRC_API ZrcResult zrcPolyMeshCreate(ZrcPolyMesh** out);

/// Builds polygons from `contours`, up to `verts_per_poly` corners each.
///
/// [Limit: 3 <= verts_per_poly <= ZRC_VERTS_PER_POLYGON] ZRC_ERR_ALREADY_BUILT
/// when `mesh` already holds polygons.
ZRC_API ZrcResult zrcPolyMeshBuild(const ZrcBuildContext* context,
                                   const ZrcContourSet* contours,
                                   int32_t verts_per_poly, ZrcPolyMesh* mesh);

/// Builds the detail half from the polygons already in `mesh` and the surface
/// in `field`.
///
/// `sample_dist` and `sample_max_error` are in world units. A mesh with no
/// polygons leaves the detail half empty and succeeds, which is upstream's own
/// behaviour. ZRC_ERR_ALREADY_BUILT when the detail half is already filled.
ZRC_API ZrcResult zrcPolyMeshBuildDetail(const ZrcBuildContext* context,
                                         ZrcPolyMesh* mesh,
                                         const ZrcCompactHeightfield* field,
                                         float sample_dist,
                                         float sample_max_error);

/// Records the agent the mesh was built for, in world units.
///
/// Recast does not carry these and Detour demands them, so a mesh assembled by
/// hand has to be told. zrcTileDataBuild refuses a mesh that has not been.
/// [Limit: every value finite and >= 0]
ZRC_API ZrcResult zrcPolyMeshSetAgentDims(ZrcPolyMesh* mesh,
                                          float walkable_height,
                                          float walkable_radius,
                                          float walkable_climb);

ZRC_API ZrcResult zrcPolyMeshInfo(const ZrcPolyMesh* mesh,
                                  ZrcPolyMeshInfo* out);

/// Copies the polygons of `src` into `dst`, which must be empty.
///
/// The detail half is not copied: upstream has no function that copies one, and
/// inventing an equivalent here would be a second implementation of a format
/// upstream owns. Rebuild it with zrcPolyMeshBuildDetail. The agent dimensions
/// travel with the polygons.
ZRC_API ZrcResult zrcPolyMeshCopy(const ZrcBuildContext* context,
                                  const ZrcPolyMesh* src, ZrcPolyMesh* dst);

/// Merges `count` meshes into `out`, which must be empty in both halves.
///
/// Both halves are merged together, so every input must have both, or none may.
/// Every input must also have been given its agent dimensions.
/// Every input must agree on `verts_per_poly`: upstream takes the first
/// mesh's as the stride into all of them, which reads past the end of any input
/// that packs its polygons differently.
///
/// The agent dimensions are taken from the first input, and every other input
/// must match it — merging meshes baked for two different agents produces one
/// mesh that describes neither.
///
/// A failure partway through can leave `out` holding a merged polygon half and
/// no detail half. It is not reusable after that; destroy it and start again.
ZRC_API ZrcResult zrcPolyMeshMerge(const ZrcBuildContext* context,
                                   const ZrcPolyMesh* const* meshes,
                                   int32_t count, ZrcPolyMesh* out);

/// Copies `count` vertices from `first`, three `uint16_t` each, in cells above
/// the mesh's minimum corner.
ZRC_API ZrcResult zrcPolyMeshVerts(const ZrcPolyMesh* mesh, int32_t first,
                                   int32_t count, uint16_t* out);

/// Copies `count` polygons from `first`, `2 * verts_per_poly` entries each.
///
/// The first half is the corner indices, ZRC_MESH_NULL_IDX past the last
/// corner. The second is the neighbour across each edge: ZRC_MESH_NULL_IDX for
/// none, otherwise a polygon index plus one, or a portal flag where the edge
/// leaves the tile.
ZRC_API ZrcResult zrcPolyMeshPolys(const ZrcPolyMesh* mesh, int32_t first,
                                   int32_t count, uint16_t* out);

/// Copies `count` region ids from `first`, one per polygon.
/// ZRC_MULTIPLE_REGS where a polygon came from more than one region.
ZRC_API ZrcResult zrcPolyMeshRegions(const ZrcPolyMesh* mesh, int32_t first,
                                     int32_t count, uint16_t* out);

/// Copies `count` area ids from `first`, one per polygon.
ZRC_API ZrcResult zrcPolyMeshPolyAreas(const ZrcPolyMesh* mesh, int32_t first,
                                       int32_t count, uint8_t* out);

/// Writes `count` area ids back, starting at `first`.
/// [Limit: each < ZRC_MAX_AREAS]
ZRC_API ZrcResult zrcPolyMeshSetPolyAreas(ZrcPolyMesh* mesh, int32_t first,
                                          int32_t count, const uint8_t* areas);

/// Copies `count` polygon flags from `first`, one per polygon.
ZRC_API ZrcResult zrcPolyMeshPolyFlags(const ZrcPolyMesh* mesh, int32_t first,
                                       int32_t count, uint16_t* out);

/// Writes `count` polygon flags back, starting at `first`.
///
/// A polygon mesh built by hand comes out with every flag zero, which no
/// nonzero query filter admits, so this is not optional for a mesh a host
/// assembles itself.
ZRC_API ZrcResult zrcPolyMeshSetPolyFlags(ZrcPolyMesh* mesh, int32_t first,
                                          int32_t count,
                                          const uint16_t* flags);

/// Copies `count` detail sub-meshes from `first`, four `uint32_t` each: the
/// first vertex and vertex count, then the first triangle and triangle count,
/// of the polygon at the same index.
ZRC_API ZrcResult zrcPolyMeshDetailMeshes(const ZrcPolyMesh* mesh,
                                          int32_t first, int32_t count,
                                          uint32_t* out);

/// Copies `count` detail vertices from `first`, three floats each, in world
/// units.
ZRC_API ZrcResult zrcPolyMeshDetailVerts(const ZrcPolyMesh* mesh, int32_t first,
                                         int32_t count, float* out);

/// Copies `count` detail triangles from `first`, four bytes each: three corner
/// indices into the owning sub-mesh's vertices, then the edge flags.
ZRC_API ZrcResult zrcPolyMeshDetailTris(const ZrcPolyMesh* mesh, int32_t first,
                                        int32_t count, uint8_t* out);

//===----------------------------------------------------------------------===//
// Navmesh (Detour — runtime)
//===----------------------------------------------------------------------===//

/// A navigation mesh ready to be queried, holding one tile or a grid of them.
typedef struct ZrcNavMesh ZrcNavMesh;

//===----------------------------------------------------------------------===//
// Off-mesh connections
//
// A bake produces surface an agent can walk across, and only that. An off-mesh
// connection joins two places the surface does not: a jump down a ledge, a
// ladder, a door between two rooms, a teleporter. It is a two-vertex polygon of
// its own, linked into the graph at both ends, so a path can route through it
// and a straight path reports it as a corner flagged
// ZRC_STRAIGHTPATH_OFFMESH_CONNECTION — which is how a game knows to play an
// animation instead of walking.
//
// Connections belong to a tile, and are supplied when that tile is built. They
// cannot be added to a live navmesh: Detour wires them during addTile and has
// no path to do it later.
//===----------------------------------------------------------------------===//

/// One point-to-point link between two places the surface does not join.
typedef struct ZrcOffMeshConnection {
  /// Endpoint A, world space. The travel direction is A to B.
  float start[3];
  /// Endpoint B, world space.
  float end[3];

  /// How far from each endpoint Detour searches for the polygon to attach to.
  /// An endpoint with no walkable surface within this distance is left
  /// unattached, silently, which is upstream's behaviour and not an error.
  /// [Limit: > 0, finite]
  float radius;

  /// Area id of the connection's own polygon, so a filter can charge for
  /// crossing it. [Limit: 0 <= area < ZRC_MAX_AREAS]
  int32_t area;

  /// Query flags of the connection's own polygon. Zero makes the connection
  /// invisible to every query, which is a working way to disable one.
  uint16_t flags;

  /// Non-zero for travel in both directions, zero for A to B only.
  ///
  /// One-way means B's ground polygon never lists the connection as a
  /// neighbour, so pathfinding cannot discover it from that side. It does not
  /// mean the links are absent: a path already on the connection can still
  /// leave at either end.
  ZrcBool bidirectional;

  /// Opaque to zrecast. Stored in the tile and returned by
  /// zrcNavMeshOffMeshConnection, which is how a host recognises which
  /// connection a path is crossing.
  uint32_t user_id;

  /// **Output only**, and ignored when this struct is passed to a build.
  ///
  /// Which tile edge endpoint B fell beyond, 0 to 7, or 255 when it lies
  /// inside the same tile as endpoint A. That distinction is the one the
  /// section above warns about: only a connection reporting 255 is wired at
  /// both ends by a single-tile navmesh.
  int32_t end_side;
} ZrcOffMeshConnection;

/// What a tile carries beyond the shape of its surface.
///
/// Every field is optional and the whole struct may be NULL.
typedef struct ZrcTileAuthoring {
  /// The connections this tile owns.
  ///
  /// A connection is stored by the tile that contains **endpoint A**; an
  /// endpoint outside the tile is wired from the neighbouring tile that does
  /// contain it. In a single-tile navmesh there is no neighbour, so a
  /// connection whose endpoint B lies outside the tile is wired at A only:
  /// enterable, with no way off at the far end. Keep both endpoints inside one
  /// tile, or build a tiled navmesh.
  const ZrcOffMeshConnection* connections;
  /// [Limit: 0 <= connection_count <= 65536, and connections is non-NULL above
  /// zero]
  int32_t connection_count;

  /// Opaque to zrecast, stored in the tile header and readable back with
  /// zrcNavMeshTileUserId. Zero when no authoring is given.
  uint32_t user_id;

  /// Non-zero omits the tile's bounding-volume tree.
  ///
  /// Stated as a negative so that a zeroed struct, and a NULL one, both mean
  /// "build it" — which is what a query wants. Without the tree Detour finds
  /// the polygons overlapping a box by scanning every one of them, which is a
  /// smaller tile and a slower query.
  ZrcBool skip_bv_tree;
} ZrcTileAuthoring;

/// Converts a baked polygon mesh into a queryable navmesh.
///
/// `authoring` may be NULL. The navmesh takes a private copy of the data, so
/// `mesh` may be destroyed immediately afterwards.
ZRC_API ZrcResult zrcNavMeshCreate(const ZrcPolyMesh* mesh,
                                   const ZrcTileAuthoring* authoring,
                                   ZrcNavMesh** out);

ZRC_API void zrcNavMeshDestroy(ZrcNavMesh* navmesh);

//===----------------------------------------------------------------------===//
// Tiles
//
// A tiled navmesh is how a world larger than one bake gets navigated: each
// tile is cooked on its own, shipped as its own asset, and added to or removed
// from a live navmesh as the camera moves. Tiles stitch to their neighbours
// through portal edges, and each is baked with a border so those edges meet.
//
// There is deliberately no whole-navmesh image for a tiled mesh. Detour has no
// such format either — its own demo invents a container — and inventing one
// here would be a format this package then has to keep. Cook each tile with
// zrcTileDataBuild instead; the tile bytes are the asset.
//===----------------------------------------------------------------------===//

/// Identifies one polygon of one tile of one navmesh. 0 is "no polygon".
///
/// A reference embeds a salt that changes when a tile is replaced, so a stale
/// reference is detected rather than silently pointing at whatever now occupies
/// the slot. References are only meaningful to the navmesh that issued them.
typedef uint32_t ZrcPolyRef;

/// Identifies one tile of one navmesh. 0 is "no tile".
///
/// Like ZrcPolyRef, a reference carries a salt that changes when the slot is
/// reused, so a reference held across a removal is detected rather than
/// silently pointing at whatever now occupies the slot.
typedef uint32_t ZrcTileRef;

/// Largest tile-grid coordinate, on either axis.
///
/// Far beyond any real world, and chosen well inside `int` so that Detour's
/// own `nx--` / `nx++` neighbour walk (DetourNavMesh.cpp:1078-1089) cannot
/// overflow. A coordinate outside it is ZRC_ERR_INVALID_ARGUMENT.
#define ZRC_MAX_TILE_COORD 1048576

/// Largest tile layer. Layers stack tiles at one grid position; a bake
/// produces layer 0, and DetourTileCache is what uses the rest.
#define ZRC_MAX_TILE_LAYER 255

/// Creates an empty navmesh sized for a tile grid.
///
/// `max_tiles` bounds how many tiles may be resident at once, and
/// `max_polys_per_tile` how many polygons any one of them may hold. Both feed
/// Detour's polygon reference, which spends 32 bits across a tile index, a
/// polygon index and a salt and refuses fewer than 10 bits of salt: rounded up
/// to powers of two, the two together may claim at most 22 bits. Exceeding
/// that is ZRC_ERR_INVALID_ARGUMENT rather than a failure from inside Detour.
ZRC_API ZrcResult zrcNavMeshCreateTiled(const ZrcTileGrid* grid,
                                        int32_t max_tiles,
                                        int32_t max_polys_per_tile,
                                        ZrcNavMesh** out);

/// Cooks a baked tile into the bytes a navmesh accepts.
///
/// The caller owns the buffer and frees it with zrcFree. This is the cook
/// step: write those bytes to disk, and hand them to zrcNavMeshAddTile at
/// runtime. The grid coordinates are stored in the image, so a tile knows
/// where it belongs without being told again.
ZRC_API ZrcResult zrcTileDataBuild(const ZrcPolyMesh* mesh, int32_t tile_x,
                                   int32_t tile_z, int32_t tile_layer,
                                   const ZrcTileAuthoring* authoring,
                                   void** out_data, size_t* out_size);

/// Adds a tile to a live navmesh.
///
/// `data` is validated in full before Detour sees any of it (see
/// zrcNavMeshValidate) and then **copied**, so the caller's buffer stays the
/// caller's and may be freed immediately.
///
/// Returns ZRC_ERR_INVALID_ARGUMENT if the tile's grid position lies outside
/// the navmesh's grid, or if the tile holds more polygons than the navmesh was
/// sized for; ZRC_ERR_TILE_OCCUPIED if a tile already sits at that position;
/// ZRC_ERR_NAVMESH_FULL if every slot is taken. `out_ref` may be NULL.
ZRC_API ZrcResult zrcNavMeshAddTile(ZrcNavMesh* navmesh, const void* data,
                                    size_t size, ZrcTileRef* out_ref);

/// Removes a tile and releases the copy the navmesh made of it.
///
/// A reference from a previous residency of the same slot is rejected with
/// ZRC_ERR_INVALID_ARGUMENT rather than removing whatever is there now.
ZRC_API ZrcResult zrcNavMeshRemoveTile(ZrcNavMesh* navmesh, ZrcTileRef ref);

/// The reference of the tile at a grid position, or 0 if the slot is empty.
///
/// An empty slot is ZRC_OK with `*out_ref` 0 — a fact about the world, not an
/// error. A coordinate outside the navmesh's grid is
/// ZRC_ERR_INVALID_ARGUMENT.
ZRC_API ZrcResult zrcNavMeshTileRefAt(const ZrcNavMesh* navmesh, int32_t tile_x,
                                      int32_t tile_z, int32_t tile_layer,
                                      ZrcTileRef* out_ref);

/// Every tile stacked at a grid position, across all layers, lowest first.
///
/// `zrcNavMeshTileRefAt` answers for one named layer; this answers when a host
/// does not know how many layers are there. A tile cache is what stacks them:
/// each compressed layer of a position becomes its own navmesh tile.
///
/// `*out_count` is how many are resident there, whether or not they fit;
/// ZRC_ERR_BUFFER_TOO_SMALL when they do not, with the first `max_tiles`
/// written. `out` may be NULL when `max_tiles` is 0, which asks only for the
/// count.
ZRC_API ZrcResult zrcNavMeshTileRefsAt(const ZrcNavMesh* navmesh,
                                       int32_t tile_x, int32_t tile_z,
                                       ZrcTileRef* out, int32_t max_tiles,
                                       int32_t* out_count);

/// Tiles currently resident. Returns -1 for a NULL navmesh.
ZRC_API int32_t zrcNavMeshTileCount(const ZrcNavMesh* navmesh);

/// The reference of the tile in slot `index`, or 0 if the slot is empty.
///
/// Slots are the navmesh's own storage, `zrcNavMeshMaxTiles` of them, and
/// walking them is how a host enumerates what is resident without knowing where
/// to look. An empty slot is ZRC_OK with `*out_ref` 0; an index outside
/// [0, maxTiles) is ZRC_ERR_INVALID_ARGUMENT.
ZRC_API ZrcResult zrcNavMeshTileRefAtIndex(const ZrcNavMesh* navmesh,
                                           int32_t index, ZrcTileRef* out_ref);

/// Tiles the navmesh was sized to hold. Returns -1 for a NULL navmesh.
ZRC_API int32_t zrcNavMeshMaxTiles(const ZrcNavMesh* navmesh);

/// World-space bounds of one resident tile. Both arrays are 3 floats.
ZRC_API ZrcResult zrcNavMeshTileBounds(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                       float* bmin, float* bmax);

/// Serialises a single-tile `navmesh` into a freshly allocated buffer, which
/// the caller owns and frees with zrcFree.
///
/// A tiled navmesh has no single image and this returns
/// ZRC_ERR_INVALID_ARGUMENT for one: cook each tile with zrcTileDataBuild
/// instead, which is the shape a streaming host wants anyway.
///
/// The image is Detour's own tile format: architecture-independent as to
/// pointer width, but **native-endian**, and tied to the struct layouts of the
/// vendored Detour. Treat it as a build artefact keyed to this library's
/// version, not as an interchange format. zrcNavMeshDataVersion reports the
/// format version an image will carry.
///
/// An image this package produced round-trips byte for byte. An image from
/// another tool does not quite: loading it seals the spare bounding-volume node
/// upstream leaves zeroed (see ZrcBvNode), so serialising it again differs in
/// that one int.
ZRC_API ZrcResult zrcNavMeshSerialize(const ZrcNavMesh* navmesh,
                                      void** out_data, size_t* out_size);

/// Rebuilds a single-tile navmesh from bytes produced by zrcNavMeshSerialize.
///
/// The buffer is validated in full before Detour sees any of it (see
/// zrcNavMeshValidate) and then copied, so it need not outlive the call.
///
/// A tile image that belongs to a grid — one carrying grid coordinates or
/// portal edges — is ZRC_ERR_INVALID_ARGUMENT here, because a single-tile mesh
/// has no neighbours for its portals to reach. Build the grid with
/// zrcNavMeshCreateTiled and add that image with zrcNavMeshAddTile.
ZRC_API ZrcResult zrcNavMeshDeserialize(const void* data, size_t size,
                                        ZrcNavMesh** out);

//===----------------------------------------------------------------------===//
// Polygon areas and flags, at runtime
//
// The bake decides these, and a game changes its mind: a door opens, a bridge
// burns, a zone floods. Both are per polygon and both are cheap — a write into
// the tile a reference names, with no rebuild — which is what makes them the
// mechanism for a world that changes shape without being re-cooked.
//
// A reference from a tile that has since been removed is refused rather than
// followed, the same as everywhere else a ZrcPolyRef is taken.
//===----------------------------------------------------------------------===//

/// The area id of one polygon, 0 to ZRC_MAX_AREAS - 1.
///
/// A zero reference is ZRC_ERR_INVALID_ARGUMENT, as it is everywhere else a
/// handle is taken. All four of these refuse a reference whose tile has been
/// removed or replaced, which the salt makes detectable.
ZRC_API ZrcResult zrcNavMeshGetPolyArea(const ZrcNavMesh* navmesh,
                                        ZrcPolyRef ref, int32_t* out_area);

/// Rewrites the area id of one polygon, changing what the query filter charges
/// to cross it. [Limit: 0 <= area < ZRC_MAX_AREAS]
ZRC_API ZrcResult zrcNavMeshSetPolyArea(ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                        int32_t area);

/// The query flags of one polygon.
ZRC_API ZrcResult zrcNavMeshGetPolyFlags(const ZrcNavMesh* navmesh,
                                         ZrcPolyRef ref, uint16_t* out_flags);

/// Rewrites the query flags of one polygon, changing whether a filter admits it
/// at all. Zero makes the polygon invisible to every query.
ZRC_API ZrcResult zrcNavMeshSetPolyFlags(ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                         uint16_t flags);

/// What kind of polygon a reference names.
typedef enum ZrcPolyType {
  /// Ordinary walkable surface.
  ZRC_POLY_GROUND = 0,
  /// The two-vertex polygon of an off-mesh connection.
  ZRC_POLY_OFFMESH_CONNECTION = 1,
} ZrcPolyType;

/// Which kind of polygon a reference names, as a ZrcPolyType.
///
/// There is no setter. A polygon's type decides how every query treats it and
/// which of a tile's arrays it may index; changing it on a live navmesh would
/// invalidate the bounds this package checked when the tile was admitted. A
/// host chooses the type by supplying connections at build time.
ZRC_API ZrcResult zrcNavMeshGetPolyType(const ZrcNavMesh* navmesh,
                                        ZrcPolyRef ref, int32_t* out_type);

//===----------------------------------------------------------------------===//
// Reading off-mesh connections back
//===----------------------------------------------------------------------===//

/// The two endpoints of an off-mesh connection, in the direction of travel.
///
/// `prev_ref` is the polygon the path arrives from, and is what decides which
/// endpoint is called the start. Detour compares it against the connection's
/// own link back to endpoint A's landing polygon: pass **that** polygon and the
/// endpoints come back as authored, A then B. Pass anything else — including 0 —
/// and they come back **reversed**, because arriving from anywhere that is not
/// A means arriving from B.
///
/// So 0 is not a neutral value here. Walking a corridor, `prev_ref` is the
/// polygon immediately before the connection in it. Both arrays are 3 floats.
///
/// ZRC_ERR_INVALID_ARGUMENT when `poly_ref` names no polygon, and
/// ZRC_ERR_QUERY_FAILED when it names one that is not an off-mesh connection.
ZRC_API ZrcResult zrcNavMeshOffMeshConnectionEndPoints(
    const ZrcNavMesh* navmesh, ZrcPolyRef prev_ref, ZrcPolyRef poly_ref,
    float* out_start, float* out_end);

/// The connection a polygon reference names, as it is stored.
///
/// `out->flags` and `out->area` come from the connection's polygon, so a
/// zrcNavMeshSetPolyFlags since the tile was added is reflected here. The
/// endpoints are the stored ones, which is not always where the connection
/// attached: Detour snaps each end to the nearest polygon within `radius` when
/// the tile is added, and zrcNavMeshOffMeshConnectionEndPoints reports where it
/// landed.
///
/// ZRC_ERR_INVALID_ARGUMENT when `ref` names no polygon, and
/// ZRC_ERR_QUERY_FAILED when it names one that is not an off-mesh connection.
ZRC_API ZrcResult zrcNavMeshOffMeshConnection(const ZrcNavMesh* navmesh,
                                              ZrcPolyRef ref,
                                              ZrcOffMeshConnection* out);

/// The tile's opaque user id, as given to zrcTileDataBuild or zrcNavMeshCreate.
ZRC_API ZrcResult zrcNavMeshTileUserId(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                       uint32_t* out_user_id);

//===----------------------------------------------------------------------===//
// Reading a loaded navmesh back
//
// A tile is a packed image with eight arrays in it, and Detour hands out raw
// pointers into that image. These entry points copy **by value** instead: a
// polygon read back stays valid after the tile it came from is removed, which a
// pointer would not.
//
// Every array is addressed by a half-open range so a host reads what it wants
// in one call rather than one entry at a time. A range outside the array is
// ZRC_ERR_INVALID_ARGUMENT rather than a short read, because a silently short
// read is indistinguishable from an empty tile.
//===----------------------------------------------------------------------===//

/// Marks a polygon's edge as crossing into an adjacent tile (DT_EXT_LINK).
///
/// ZrcPolyInfo.neis is one entry per corner, encoded three ways: 0 for an edge
/// with nothing across it, this bit set for a tile boundary, and otherwise a
/// **one-based** index of the neighbouring polygon in the same tile. The
/// one-based part is why a raw entry cannot be used as an index unchanged.
#define ZRC_EXT_LINK 0x8000

/// The end of a polygon's link chain (DT_NULL_LINK).
#define ZRC_NULL_LINK 0xffffffffu

/// The navmesh owns this tile's bytes and frees them with it (DT_TILE_FREE_DATA).
///
/// Always set for a tile added through zrcNavMeshAddTile, which copies.
#define ZRC_TILE_FREE_DATA 0x01

/// How a navmesh's tile grid was sized, as zrcNavMeshCreateTiled set it up.
typedef struct ZrcNavMeshParams {
  /// World-space origin of the tile grid.
  float origin[3];
  /// Tile dimensions in world units. A single-tile navmesh reports the tile's
  /// own extent rather than zero, because that is the grid it has.
  float tile_width;
  float tile_height;
  /// How many tiles may be resident at once.
  int32_t max_tiles;
  /// How many polygons any one tile may hold.
  int32_t max_polys;
} ZrcNavMeshParams;

ZRC_API ZrcResult zrcNavMeshParams(const ZrcNavMesh* navmesh,
                                   ZrcNavMeshParams* out);

/// Which tile of the grid a world position falls in.
///
/// Answers for any position inside the grid or outside it: the result is the
/// coordinate the position would have, not a promise that a tile is there.
/// `pos` is 3 floats, all of which must be finite.
///
/// ZRC_ERR_INVALID_ARGUMENT for a position so far from the origin that the
/// tile it would occupy cannot be named by an int. That is a real limit rather
/// than a formality: upstream casts the quotient without checking it, and a
/// float-to-int conversion whose value does not fit is undefined behaviour.
ZRC_API ZrcResult zrcNavMeshCalcTileLoc(const ZrcNavMesh* navmesh,
                                        const float* pos, int32_t* out_tile_x,
                                        int32_t* out_tile_z);

/// The side code facing a given one, for the four values a portal edge carries.
///
/// A portal on side 0 of one tile meets side 4 of its neighbour. Returns
/// ZRC_ERR_INVALID_ARGUMENT outside 0 to 7.
ZRC_API ZrcResult zrcOppositeTileSide(int32_t side, int32_t* out_side);

/// Everything a tile's header says about it, copied out.
typedef struct ZrcTileInfo {
  /// Where the tile sits in the grid.
  int32_t tile_x;
  int32_t tile_z;
  int32_t tile_layer;

  /// As given to zrcTileDataBuild or zrcNavMeshCreate.
  uint32_t user_id;

  /// Total polygons, ground and off-mesh together.
  int32_t poly_count;
  /// Ground polygons, which are the first `ground_poly_count` of them. The
  /// off-mesh connections own the rest (dtMeshHeader::offMeshBase).
  int32_t ground_poly_count;
  int32_t off_mesh_con_count;

  int32_t vert_count;
  int32_t detail_mesh_count;
  int32_t detail_vert_count;
  int32_t detail_tri_count;
  int32_t bv_node_count;
  /// Links the tile was built with. Detour allocates them all up front and
  /// hands them out as polygons are wired.
  int32_t max_link_count;

  /// The agent the tile was cooked for, in world units.
  float walkable_height;
  float walkable_radius;
  float walkable_climb;

  /// World-space bounds. Both arrays are 3 floats.
  float bmin[3];
  float bmax[3];

  /// Scale from world units to the quantised bounds a ZrcBvNode carries: the
  /// reciprocal of the cell size the tile was cooked at. Written whether or not
  /// the tile has a tree, so `bv_node_count` is what says the tree is there.
  float bv_quant_factor;

  /// The format identifier at the front of the image (DT_NAVMESH_MAGIC).
  int32_t magic;

  /// ZRC_TILE_FREE_DATA, and nothing else this package produces.
  uint32_t flags;

  //--- The navmesh's own bookkeeping, reported because a C++ host can read it.
  //--- None of it is an invitation to walk the structures it describes; the
  //--- entry points above address every array by index instead.

  /// Salt of the slot this tile occupies. Bumped when the slot is reused,
  /// which is what makes a reference held across a removal detectable.
  uint32_t salt;
  /// Head of the tile's list of unused links, or ZRC_NULL_LINK.
  uint32_t links_free_list;
  /// The next tile in the same hash bucket, or 0.
  ///
  /// This is Detour's own spatial lookup, not a list of anything a host would
  /// want: two tiles at unrelated grid positions share a bucket whenever their
  /// coordinates hash alike, so following it finds neighbours and strangers
  /// alike. Reported because a C++ host can read it. Use zrcNavMeshTileRefAt
  /// to reach the layers of a column, and zrcNavMeshTileRefAtIndex to walk
  /// every tile.
  ZrcTileRef next_tile;
} ZrcTileInfo;

ZRC_API ZrcResult zrcNavMeshTileInfo(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                     ZrcTileInfo* out);

/// One polygon, copied out of the tile that holds it.
typedef struct ZrcPolyInfo {
  /// Corner vertices, `vert_count` of them, as indices into the tile's vertex
  /// array. An off-mesh connection has exactly two.
  uint16_t verts[ZRC_VERTS_PER_POLYGON];

  /// One entry per corner. See ZRC_EXT_LINK for the encoding.
  uint16_t neis[ZRC_VERTS_PER_POLYGON];

  /// The flags a query filter admits or refuses this polygon by.
  uint16_t flags;
  uint8_t vert_count;
  /// The area a query filter charges for crossing it.
  uint8_t area;
  /// A ZrcPolyType.
  int32_t type;

  /// Head of this polygon's link chain, or ZRC_NULL_LINK. Follow it with
  /// zrcNavMeshTileLink.
  uint32_t first_link;

  //--- The polygon's slice of the tile's detail mesh (dtPolyDetail), which is
  //--- the finer triangulation a height query samples.

  uint32_t detail_vert_base;
  uint32_t detail_tri_base;
  uint8_t detail_vert_count;
  uint8_t detail_tri_count;
} ZrcPolyInfo;

ZRC_API ZrcResult zrcNavMeshPolyInfo(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                     ZrcPolyInfo* out);

/// The reference naming one polygon of a tile, by index.
///
/// Without this a polygon is reachable only through a reference some query
/// already handed back, so a tile's polygons could not be walked at all. The
/// index runs over the whole tile: `[0, ZrcTileInfo.ground_poly_count)` are the
/// ground polygons and the rest belong to off-mesh connections.
///
/// The reference stays valid until the tile is removed, at which point the
/// salt it carries makes it detectably stale rather than silently wrong.
ZRC_API ZrcResult zrcNavMeshTilePolyRef(const ZrcNavMesh* navmesh,
                                        ZrcTileRef ref, int32_t index,
                                        ZrcPolyRef* out);

/// One link: an edge from a polygon to a neighbour it can be walked to.
typedef struct ZrcLink {
  /// The polygon on the other side.
  ZrcPolyRef ref;
  /// The next link of the same polygon, or ZRC_NULL_LINK.
  uint32_t next;
  /// Which corner of the owning polygon this link leaves by. 0xff on an
  /// off-mesh connection's own entry link.
  uint8_t edge;
  /// The tile side the link crosses, or 0xff when it stays inside the tile.
  uint8_t side;
  /// How much of the shared edge the neighbour covers, in 1/255ths, when the
  /// link crosses a tile boundary. Both zero otherwise.
  uint8_t bmin;
  uint8_t bmax;
} ZrcLink;

/// Reads one of a tile's links by index.
///
/// Indices come from ZrcPolyInfo.first_link and from ZrcLink.next; there is no
/// reason to iterate the array itself, and a link not on any polygon's chain is
/// free storage rather than an edge.
ZRC_API ZrcResult zrcNavMeshTileLink(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                     int32_t index, ZrcLink* out);

/// One node of a tile's bounding-volume tree.
///
/// The tree occupies ZrcTileInfo.bv_node_count **minus one** nodes. A tile
/// reserves two per polygon and the tree fills `2n - 1` of them, because every
/// node is either a leaf or a split into two children. The last reserved node
/// is spare, and this package seals it as an internal node so no query can
/// report it; upstream leaves it zeroed, which reads as a leaf naming polygon
/// 0 and matches any query reaching the tile's minimum corner. See UPSTREAM.md.
typedef struct ZrcBvNode {
  /// Quantised bounds, in the tile's own coordinates. Divide by
  /// ZrcTileInfo.bv_quant_factor and add ZrcTileInfo.bmin for world units.
  uint16_t bmin[3];
  uint16_t bmax[3];
  /// A polygon index when zero or positive; otherwise the negated number of
  /// nodes to skip to leave this subtree.
  int32_t i;
} ZrcBvNode;

ZRC_API ZrcResult zrcNavMeshTileBvNode(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                       int32_t index, ZrcBvNode* out);

/// Copies `count` vertices from `first`, three floats each, into `out`.
ZRC_API ZrcResult zrcNavMeshTileVerts(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                      int32_t first, int32_t count, float* out);

/// Copies `count` detail vertices from `first`, three floats each.
ZRC_API ZrcResult zrcNavMeshTileDetailVerts(const ZrcNavMesh* navmesh,
                                            ZrcTileRef ref, int32_t first,
                                            int32_t count, float* out);

/// Copies `count` detail triangles from `first`, four bytes each.
///
/// Three corner indices and an edge-flag byte. A corner below the owning
/// polygon's `vert_count` names one of its own corners; at or above it, name
/// `detail_vert_base + corner - vert_count` in the detail vertex array.
ZRC_API ZrcResult zrcNavMeshTileDetailTris(const ZrcNavMesh* navmesh,
                                           ZrcTileRef ref, int32_t first,
                                           int32_t count, uint8_t* out);

/// Where each of a tile image's eight arrays begins, and how many bytes it
/// occupies, both relative to the start of the image.
///
/// This is the layout dtNavMesh::addTile derives, reported instead of walked:
/// every array of a tile is carved out of the raw buffer by one primitive
/// (dtGetThenAdvanceBufferPointer) applied eight times in a fixed order, and
/// this is that arithmetic's result. A host parsing an image itself — a tool
/// that patches polygon flags in a cooked asset, say — needs exactly these
/// numbers and cannot derive them from the header alone, because the padding
/// rule is upstream's rather than the host's.
///
/// The image is validated first, so every offset named here is inside `size`
/// and every array is fully contained. An array with no elements has a size of
/// 0 and an offset equal to the array that follows it.
typedef struct ZrcTileLayout {
  /// The header itself always begins at offset 0; its size is the first
  /// array's offset.
  int64_t verts_offset;
  int64_t verts_size;
  int64_t polys_offset;
  int64_t polys_size;
  int64_t links_offset;
  int64_t links_size;
  int64_t detail_meshes_offset;
  int64_t detail_meshes_size;
  int64_t detail_verts_offset;
  int64_t detail_verts_size;
  int64_t detail_tris_offset;
  int64_t detail_tris_size;
  int64_t bv_tree_offset;
  int64_t bv_tree_size;
  int64_t off_mesh_cons_offset;
  int64_t off_mesh_cons_size;
  /// Total bytes the header's counts imply, which a well-formed image matches
  /// exactly.
  int64_t total_size;
} ZrcTileLayout;

/// Reports the layout of a serialised tile image.
///
/// `data` and `size` are the same bytes zrcNavMeshAddTile takes. Returns
/// ZRC_ERR_BAD_FORMAT for anything zrcNavMeshValidate would reject, so a
/// successful call means the offsets describe the buffer that was passed.
ZRC_API ZrcResult zrcTileLayout(const void* data, size_t size,
                                ZrcTileLayout* out);

//===----------------------------------------------------------------------===//
// Tile state
//
// The areas and flags of one tile's polygons, as bytes: what a save file has to
// carry so a door left open is still open on reload. Nothing structural is in
// it — not vertices, not links, not the connections themselves — so it is small
// and it is only meaningful against the very tile it came from.
//===----------------------------------------------------------------------===//

/// Bytes zrcNavMeshStoreTileState needs for this tile.
ZRC_API ZrcResult zrcNavMeshTileStateSize(const ZrcNavMesh* navmesh,
                                          ZrcTileRef ref, size_t* out_size);

/// Writes the tile's polygon areas and flags into `data`.
///
/// `size` must be exactly zrcNavMeshTileStateSize's answer, not merely at least
/// it. Upstream accepts any buffer at least large enough and then reads back a
/// polygon count it takes from the live tile rather than from the blob, so a
/// blob and a tile that disagree are only caught by insisting the two lengths
/// match. Too small is ZRC_ERR_BUFFER_TOO_SMALL, too large is
/// ZRC_ERR_INVALID_ARGUMENT.
ZRC_API ZrcResult zrcNavMeshStoreTileState(const ZrcNavMesh* navmesh,
                                           ZrcTileRef ref, void* data,
                                           size_t size);

/// Puts stored areas and flags back onto the tile they came from.
///
/// The blob carries the tile reference it was taken from, and restoring it onto
/// any other tile is ZRC_ERR_INVALID_ARGUMENT. A tile removed and re-added gets
/// a new salt, so its old state no longer applies — which is the point.
ZRC_API ZrcResult zrcNavMeshRestoreTileState(ZrcNavMesh* navmesh, ZrcTileRef ref,
                                             const void* data, size_t size);

/// Rewrites a cooked tile image into the opposite byte order, as a new buffer.
///
/// The image Detour cooks is native-endian, so a host that cooks on one
/// architecture and loads on another of the opposite byte order has to convert
/// on one side or the other. This is that conversion. The caller owns the
/// result and frees it with zrcFree; `data` is never modified, so a refusal
/// leaves nothing half-converted.
///
/// `from_native` says which way round. ZRC_TRUE: `data` is an image this build
/// can load, and the result is for a machine of the opposite byte order.
/// ZRC_FALSE: `data` came from such a machine, and the result is loadable here.
///
/// The direction is not cosmetic. Upstream's two halves have to run in opposite
/// orders — the body is swapped while the header still reads natively on the
/// way out, and the header is swapped first on the way in — because the body's
/// array bounds are counts stored in the header. Running them the wrong way
/// round reads those counts through the wrong byte order.
///
/// The result is validated as thoroughly as zrcNavMeshValidate validates any
/// image, in whichever byte order it ends up native to.
ZRC_API ZrcResult zrcNavMeshImageSwapEndian(const void* data, size_t size,
                                            ZrcBool from_native,
                                            void** out_data, size_t* out_size);

/// Checks that `data` is a tile image this build can load, without building
/// anything.
///
/// One rule, and it is about memory safety rather than about which navmesh the
/// tile belongs to: a tile of a grid passes here just as a lone tile does. The
/// extra conditions live where they matter — zrcNavMeshDeserialize requires a
/// tile with no neighbours, zrcNavMeshAddTile requires one that fits its
/// grid — so there is one definition of "well formed" and two of "admissible".
///
/// This exists because Detour's own dtNavMesh::init checks two fields — magic
/// and version — by dereferencing the buffer *before* looking at its length,
/// and then trusts every count in the header to address the rest of it. A
/// truncated or doctored image is therefore an out-of-bounds read inside
/// Detour, not an error return. This function closes that: it bounds-checks the
/// header itself, rejects negative or inconsistent counts, and requires the
/// total size implied by those counts to equal `size` exactly.
///
/// It then walks the arrays and bounds every index Detour will dereference:
/// each polygon's corner count, corner indices and neighbour indices, each
/// detail sub-mesh's base and extent, each detail triangle's corner
/// resolution, each BV-tree node's polygon index or escape jump, and each
/// off-mesh connection's polygon, side code and flags, with every coordinate
/// required finite. That second pass is not optional —
/// a `dtPoly::vertCount` of 200 in an otherwise untouched image passes every
/// header check and then overruns a fixed stack array in four Detour functions
/// with coordinates taken from the image.
///
/// What it still does not do is decide whether the mesh is *sensible*: a fully
/// in-bounds image can describe degenerate polygons, a nonsense BV tree, or a
/// navmesh whose geometry corresponds to nothing. So this is a guarantee about
/// memory safety, not about the mesh being usable or trustworthy. For untrusted
/// input, wrap the image in a length-and-signature container and check that
/// first.
ZRC_API ZrcResult zrcNavMeshValidate(const void* data, size_t size);

/// Total polygons across the mesh. Returns -1 for a NULL navmesh.
///
/// 0 is a legitimate answer — a navmesh with no tile resident has no polygons —
/// so a NULL handle reports the same -1 zrcNavMeshTileCount and
/// zrcNavMeshMaxTiles do.
ZRC_API int32_t zrcNavMeshPolyCount(const ZrcNavMesh* navmesh);

/// World-space bounds. Both arrays are 3 floats.
ZRC_API ZrcResult zrcNavMeshBounds(const ZrcNavMesh* navmesh, float* bmin,
                                   float* bmax);

//===----------------------------------------------------------------------===//
// Geometry primitives
//
// The computational geometry Detour runs on, callable directly. These are what
// a host needs when it reasons about a navmesh itself rather than asking a
// query: whether a point is in a polygon, where a segment leaves one, how far
// an agent is from an edge. Answering those questions with a second
// implementation would mean answering them differently from Detour at the
// boundaries, which is where every one of them matters.
//
// The vector arithmetic underneath is deliberately NOT here. A three-float add
// across an FFI boundary is a call to add three floats; the Zig side spells
// those out and proves them bit-identical to the C instead. What crosses the
// boundary is only the part with an algorithm in it.
//
// Every array is a caller's own, so every count is checked against what the
// function will dereference rather than trusted — upstream checks none of
// them. Positions are (x, y, z) triples and must be finite; a polygon is
// `3 * vert_count` floats and must have at least three vertices, which is the
// smallest shape each of these has a defined answer for.
//
// Where upstream projects onto the xz-plane and ignores y, so does this, and
// each entry point says so.
//===----------------------------------------------------------------------===//

/// Closest point on a triangle to `point`, in 3D. All four arrays are 3 floats.
///
/// A degenerate triangle — three collinear or coincident vertices — can make
/// upstream's barycentric division produce a non-finite result. That is
/// reported as it comes rather than refused, because refusing it would answer
/// differently from the C for an input the C accepts.
ZRC_API ZrcResult zrcClosestPointOnTriangle(const float* point, const float* a,
                                            const float* b, const float* c,
                                            float* out_closest);

/// Height of triangle a-b-c directly above or below `point`, on the xz-plane.
///
/// `out_height` is written only when the point lies inside the triangle, which
/// `*out_inside` reports. A triangle degenerate on the xz-plane — its
/// projected area under 1e-6 — is not inside anything and reports ZRC_FALSE.
ZRC_API ZrcResult zrcClosestHeightPointTriangle(const float* point,
                                                const float* a, const float* b,
                                                const float* c,
                                                float* out_height,
                                                ZrcBool* out_inside);

/// Squared distance from `point` to the segment p-q, on the xz-plane.
///
/// `out_t` receives where the nearest point falls along the segment, clamped
/// to [0, 1]. A zero-length segment reports t = 0.
ZRC_API ZrcResult zrcDistancePointToSegment2D(const float* point,
                                              const float* p, const float* q,
                                              float* out_dist_sqr,
                                              float* out_t);

/// Squared distance from `point` to every edge of a polygon, on the xz-plane,
/// and whether the point is inside it.
///
/// `out_edge_dist_sqr` and `out_edge_t` each receive `vert_count` entries, one
/// per edge, indexed by the edge's **starting** vertex.
ZRC_API ZrcResult zrcDistancePointToPolyEdges(const float* point,
                                              const float* verts,
                                              int32_t vert_count,
                                              float* out_edge_dist_sqr,
                                              float* out_edge_t,
                                              ZrcBool* out_inside);

/// Whether `point` lies inside a polygon, on the xz-plane.
ZRC_API ZrcResult zrcPointInPolygon(const float* point, const float* verts,
                                    int32_t vert_count, ZrcBool* out_inside);

/// Where the segment p0-p1 enters and leaves a convex polygon, on the
/// xz-plane.
///
/// `out_t_min` and `out_t_max` are fractions along the segment and are always
/// written: a segment wholly inside the polygon reports 0 and 1. The two
/// segment indices are the polygon edge crossed at each end, or -1 where the
/// endpoint is inside. `out_intersects` is ZRC_FALSE when the segment misses
/// the polygon entirely, in which case the other four outputs hold the state
/// the search had reached and mean nothing.
ZRC_API ZrcResult zrcIntersectSegmentPoly2D(const float* p0, const float* p1,
                                            const float* verts,
                                            int32_t vert_count,
                                            float* out_t_min, float* out_t_max,
                                            int32_t* out_seg_min,
                                            int32_t* out_seg_max,
                                            ZrcBool* out_intersects);

/// Where two segments cross, on the xz-plane.
///
/// `out_s` and `out_t` are fractions along ap-aq and bp-bq respectively, and
/// are **not** clamped: a value outside [0, 1] means the infinite lines cross
/// beyond the segment's end. `out_intersects` is ZRC_FALSE for parallel or
/// near-parallel lines, and both fractions are then 0 — upstream writes neither
/// on that path, so the value is this binding's rather than Detour's.
ZRC_API ZrcResult zrcIntersectSegSeg2D(const float* ap, const float* aq,
                                       const float* bp, const float* bq,
                                       float* out_s, float* out_t,
                                       ZrcBool* out_intersects);

/// Whether two convex polygons overlap, on the xz-plane, by separating axis.
ZRC_API ZrcResult zrcOverlapPolyPoly2D(const float* poly_a, int32_t count_a,
                                       const float* poly_b, int32_t count_b,
                                       ZrcBool* out_overlap);

/// Whether two axis-aligned boxes overlap. All four arrays are 3 floats.
///
/// Touching counts as overlapping: the test is inclusive at both ends.
ZRC_API ZrcResult zrcOverlapBounds(const float* amin, const float* amax,
                                   const float* bmin, const float* bmax,
                                   ZrcBool* out_overlap);

/// Whether two boxes in a tile's quantised space overlap.
///
/// This is the test a BV-tree traversal runs, so it is what a host needs to
/// walk a tree read back through zrcNavMeshTileBvNode and reach the same
/// polygons Detour would. All four arrays are 3 uint16.
ZRC_API ZrcResult zrcOverlapQuantBounds(const uint16_t* amin,
                                        const uint16_t* amax,
                                        const uint16_t* bmin,
                                        const uint16_t* bmax,
                                        ZrcBool* out_overlap);

/// Twice the signed area of triangle a-b-c on the xz-plane.
///
/// The sign is the winding: positive when c lies to the left of the line a-b
/// looking from a toward b, which is the direction Detour wraps a polygon in.
/// All three arrays are 3 floats.
ZRC_API ZrcResult zrcTriArea2D(const float* a, const float* b, const float* c,
                               float* out_area);

/// Centroid of the polygon formed by indexing `verts` with `indices`.
///
/// `out_center` is 3 floats. Every index must be less than `vert_count`, and
/// `index_count` must be at least 1.
///
/// Upstream's dtCalcPolyCenter takes no vertex count at all and so cannot
/// check either — it trusts every value in the index array as an offset, and
/// divides by the index count without testing it. Both are real parameters
/// here.
ZRC_API ZrcResult zrcPolyCenter(const float* verts, int32_t vert_count,
                                const uint16_t* indices, int32_t index_count,
                                float* out_center);

/// A point inside a convex polygon, chosen from two numbers in [0, 1].
///
/// `s` picks a triangle of the fan weighted by area and `t` picks a point
/// within it, so a caller supplies its own randomness and this stays a pure
/// function of its arguments — which is what makes a placement reproducible
/// from a seed.
///
/// `scratch` is working storage for the fan's areas and must hold at least
/// `vert_count` floats. Its contents on return are unspecified.
///
/// `vert_count` must be at least 3. Upstream accepts 1 and then reads three
/// floats **before** the start of the vertex array: with a single vertex its
/// triangulation loop never runs, the chosen triangle index stays 0, and the
/// second corner is read from index -1.
ZRC_API ZrcResult zrcRandomPointInConvexPoly(const float* verts,
                                             int32_t vert_count, float* scratch,
                                             int32_t scratch_count, float s,
                                             float t, float* out_point);

/// Inflates or shrinks a convex polygon by `offset`, on the xz-plane.
///
/// A negative offset shrinks. Sharp corners are bevelled rather than mitred,
/// so the result can have up to `2 * vert_count` vertices; y is carried
/// through from the source vertex unchanged.
///
/// `out_verts` holds `3 * max_out_verts` floats and `out_vert_count` receives
/// how many were written. Note the capacity upstream actually requires:
/// its bounds check refuses to fill the last slot, so `max_out_verts` must
/// exceed the result by at least one. Returns ZRC_ERR_BUFFER_TOO_SMALL when
/// the result did not fit, which is upstream's `0` given a name.
ZRC_API ZrcResult zrcOffsetPoly(const float* verts, int32_t vert_count,
                                float offset, float* out_verts,
                                int32_t max_out_verts,
                                int32_t* out_vert_count);

//===----------------------------------------------------------------------===//
// Queries (Detour — runtime)
//===----------------------------------------------------------------------===//

/// Which polygons a query may traverse, and what each area costs.
///
/// A POD mirror of Detour's dtQueryFilter, which is a C++ class. Each call
/// converts this into a dtQueryFilter on the stack; that is a fixed ~260-byte
/// copy against a graph search, and it keeps a class out of the ABI.
typedef struct ZrcQueryFilter {
  /// Cost multiplier per area id. 1.0 is neutral; larger discourages.
  float area_cost[ZRC_MAX_AREAS];
  /// A polygon is visitable only if it shares a bit with this.
  uint16_t include_flags;
  /// A polygon is skipped if it shares a bit with this.
  uint16_t exclude_flags;
} ZrcQueryFilter;

/// Every area cost 1.0, include_flags 0xFFFF, exclude_flags 0 — accepts every
/// polygon zrcPolyMeshBake produces.
ZRC_API void zrcQueryFilterDefault(ZrcQueryFilter* out);

/// A query object: a navmesh plus the node pool a search needs.
///
/// Holds a borrowed pointer to its navmesh, which must outlive it. Not
/// thread-safe; give each thread its own.
typedef struct ZrcNavMeshQuery ZrcNavMeshQuery;

/// `max_nodes` bounds every search this query can run: A* stops with
/// ZRC_ERR_OUT_OF_NODES rather than growing. 2048 suits a small mesh.
///
/// [Limit: 4 <= max_nodes <= 65535]. The lower bound is four rather than one
/// because Detour sizes the pool's hash table as `dtNextPow2(max_nodes / 4)`,
/// which is zero — not one — for anything smaller, and every bucket index it
/// then computes is out of bounds.
ZRC_API ZrcResult zrcNavMeshQueryCreate(const ZrcNavMesh* navmesh,
                                        int32_t max_nodes,
                                        ZrcNavMeshQuery** out);

ZRC_API void zrcNavMeshQueryDestroy(ZrcNavMeshQuery* query);

/// Finds the polygon nearest `center` within a box of `half_extents`.
///
/// On success with nothing in range, `*out_ref` is 0 and `out_point` is
/// untouched — that is a legitimate answer, not an error. `out_point` may be
/// NULL. Both float arrays are 3 elements.
///
/// half_extents[1] (the y component) is usually the one that matters: it has to
/// cover the distance from `center` down to the floor.
///
/// `*out_over_poly`, when not NULL, reports whether `center`'s x/z actually
/// lies inside the nearest polygon, as opposed to the polygon merely being the
/// closest one. Unchanged when nothing is found, the same as `out_point`.
ZRC_API ZrcResult zrcFindNearestPoly(const ZrcNavMeshQuery* query,
                                     const float* center,
                                     const float* half_extents,
                                     const ZrcQueryFilter* filter,
                                     ZrcPolyRef* out_ref, float* out_point,
                                     ZrcBool* out_over_poly);

/// A* over the polygon graph, producing the corridor of polygons to cross.
///
/// This is not yet a path to walk — it is the corridor a walkable path lives
/// in. Feed it to zrcFindStraightPath to get points.
///
/// `*out_partial` is set when the corridor stops short of `end_ref` (the goal
/// is unreachable, or the node pool ran out); the corridor returned is then the
/// best effort towards it. Distinguishing that from success matters: a partial
/// path looks exactly like a complete one otherwise. May be NULL.
ZRC_API ZrcResult zrcFindPath(const ZrcNavMeshQuery* query,
                              ZrcPolyRef start_ref, ZrcPolyRef end_ref,
                              const float* start_pos, const float* end_pos,
                              const ZrcQueryFilter* filter,
                              ZrcPolyRef* out_path, int32_t max_path,
                              int32_t* out_count, ZrcBool* out_partial);

/// Flags on a straight-path point (Detour's dtStraightPathFlags).
enum {
  /// The point is the path's start.
  ZRC_STRAIGHTPATH_START = 0x01,
  /// The point is the path's end.
  ZRC_STRAIGHTPATH_END = 0x02,
  /// The point enters an off-mesh connection.
  ZRC_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04,
};

/// Options for zrcFindStraightPath (Detour's dtStraightPathOptions).
///
/// Not the flags above, despite the shared prefix and the overlapping values:
/// these go in, those come out. Without either, a corner is emitted only where
/// the corridor actually turns.
enum {
  /// Also emit a point wherever the corridor crosses an area boundary.
  ZRC_STRAIGHTPATH_AREA_CROSSINGS = 0x01,
  /// Also emit a point at every polygon edge the corridor crosses.
  ZRC_STRAIGHTPATH_ALL_CROSSINGS = 0x02,
};

/// String-pulls a polygon corridor into the list of corners to walk.
///
/// `out_points` receives `3 * (*out_count)` floats, so it holds `max_points`
/// corners. `out_flags` and `out_refs` may each be NULL; when given, each
/// carries its own capacity, and each must be at least `max_points`.
///
/// The capacities are separate parameters rather than a documented promise
/// because Detour's appendVertex writes all three arrays at the same index,
/// bounded only by the first array's limit
/// (DetourNavMeshQuery.cpp:1689-1709). A short companion array is a real
/// overflow, so the contract is checked here instead of being asserted in a
/// comment. Too small a companion is ZRC_ERR_BUFFER_TOO_SMALL.
///
/// `*out_partial` is set when the corridor had more corners than `max_points`
/// could hold.
ZRC_API ZrcResult zrcFindStraightPath(
    const ZrcNavMeshQuery* query, const float* start_pos, const float* end_pos,
    const ZrcPolyRef* path, int32_t path_count, uint32_t options,
    float* out_points, int32_t max_points, uint8_t* out_flags,
    int32_t max_flags, ZrcPolyRef* out_refs, int32_t max_refs,
    int32_t* out_count, ZrcBool* out_partial);

/// Slides from `start_pos` towards `end_pos` across the surface, stopping at
/// walls — the movement primitive for a character being pushed by input.
///
/// Does no search, unlike zrcFindPath. `out_visited`, `out_visited_count` and
/// `out_truncated` may each be NULL if not needed.
///
/// `*out_truncated` is set when the move crossed more polygons than
/// `max_visited` could hold. The position reached is correct either way; only
/// the list is short. Without this a caller cannot tell a complete list from a
/// clipped one, since Detour reports the clip as a success.
///
/// It stays clear when `out_visited` is NULL. Detour requires a real buffer, so
/// a small internal one is used and discarded; reporting *that* as truncation
/// would be reporting on a list the caller never asked for.
ZRC_API ZrcResult zrcMoveAlongSurface(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* start_pos,
    const float* end_pos, const ZrcQueryFilter* filter, float* out_pos,
    ZrcPolyRef* out_visited, int32_t max_visited, int32_t* out_visited_count,
    ZrcBool* out_truncated);

/// Where a walkability raycast ended up.
typedef struct ZrcRaycastHit {
  /// Fraction of the segment travelled before stopping, in [0, 1].
  ///
  /// Detour reports FLT_MAX for "no wall hit"; this normalises that to exactly
  /// 1.0 with `hit` clear, so `t` is always usable as a lerp parameter. A wall
  /// struck exactly at the segment's end still counts as a hit.
  float t;
  /// Point reached: start + (end - start) * t.
  float position[3];
  /// Normal of the wall struck, or (0, 0, 0) when nothing was struck.
  float normal[3];
  /// Whether a wall stopped the ray before `end_pos`.
  ZrcBool hit;
  /// Index of the polygon edge the ray crossed last, or -1 if it crossed none.
  ///
  /// Upstream leaves this uninitialised when the very first polygon's
  /// intersection test fails, and neither its own overload nor its caller
  /// zeroes it first (DetourNavMeshQuery.cpp:2527-2532). Seeded here, so the
  /// value is always this binding's rather than the caller's stack.
  int32_t hit_edge_index;
  /// Cost of the movement along the ray, accumulated per polygon crossed.
  ///
  /// Zero unless ZRC_RAYCAST_USE_COSTS was passed: upstream only accumulates
  /// under that option, and reports 0 rather than "not computed" otherwise.
  float path_cost;
} ZrcRaycastHit;

/// Casts a walkability ray across the surface. Not a physics ray: it ignores
/// height entirely and reports the first navmesh boundary crossed.
///
/// Options for zrcRaycast (Detour's dtRaycastOptions).
enum {
  /// Accumulate movement cost across the polygons crossed, into
  /// ZrcRaycastHit.path_cost. Without it that field stays 0.
  ZRC_RAYCAST_USE_COSTS = 0x01,
};

/// `out_path`, `out_path_count` and `out_truncated` may each be NULL.
/// `*out_truncated` is set when the ray crossed more polygons than `max_path`
/// could hold; the hit itself is correct either way.
///
/// `prev_ref` is the polygon `start_ref` was entered from, or 0 for "no
/// parent" — upstream's own default. It affects nothing but the cost of
/// entering `start_ref`, and only when ZRC_RAYCAST_USE_COSTS is set.
ZRC_API ZrcResult zrcRaycast(const ZrcNavMeshQuery* query,
                             ZrcPolyRef start_ref, const float* start_pos,
                             const float* end_pos,
                             const ZrcQueryFilter* filter, uint32_t options,
                             ZrcPolyRef prev_ref, ZrcRaycastHit* out_hit,
                             ZrcPolyRef* out_path, int32_t max_path,
                             int32_t* out_path_count, ZrcBool* out_truncated);

//===----------------------------------------------------------------------===//
// Sliced pathfinding
//
// The same search zrcFindPath runs, driven a few iterations at a time so a
// frame can spend a fixed budget on it. The state lives in the query object;
// there is no second handle.
//
// Two things the C++ API leaves to a host, which this one does not:
//
//   * **The filter must outlive the call.** initSlicedFindPath stores the
//     filter as a raw pointer and every later update and finalise reads
//     through it (DetourNavMeshQuery.cpp:1233). The start and end positions
//     are copied; the filter is not. ZrcNavMeshQuery keeps its own copy for
//     the life of the slice, so a caller may pass a stack filter here the way
//     it does everywhere else.
//
//   * **One search at a time.** Detour's own class comment promises that const
//     methods have "no impact on an in-progress sliced path query"
//     (DetourNavMeshQuery.cpp:133). It is false. zrcFindPath,
//     zrcFindPolysAroundCircle, zrcFindPolysAroundShape,
//     zrcFindRandomPointAroundCircle and zrcFindDistanceToWall each begin by
//     clearing the shared node pool, and dtNodePool::clear resets the count
//     without clearing the storage — so the slice's best-node pointer stays
//     valid while the nodes behind it are handed to the next search.
//     Finalising then walks a chain belonging to that other search and
//     returns plausible, wrong references. Those five are refused while a
//     slice is in flight, with ZRC_ERR_SEARCH_IN_PROGRESS.
//
//     Refused, and not more: zrcFindNearestPoly, zrcQueryPolygons,
//     zrcRaycast, zrcMoveAlongSurface, zrcFindLocalNeighbourhood and
//     zrcFindRandomPoint touch no shared node, so a game loop may keep using
//     them between slices of a path.
//===----------------------------------------------------------------------===//

/// Options for zrcSlicedFindPathInit (Detour's dtFindPathOptions).
enum {
  /// Use raycasts to shortcut between polygons where the straight line is
  /// walkable, which produces a path that is not confined to polygon centres.
  /// Costs are still evaluated.
  ZRC_FINDPATH_ANY_ANGLE = 0x02,
};

/// How far an any-angle shortcut ray may reach, as a multiple of the agent
/// radius the navmesh was baked for (Detour's DT_RAY_CAST_LIMIT_PROPORTIONS).
#define ZRC_RAYCAST_LIMIT_PROPORTIONS 50.0f

/// Begins a sliced search. `filter` is copied and need not outlive the call.
///
/// Replaces any slice already in flight on this query, which is the one thing
/// a second init may do: abandoning a slice is what zrcSlicedFindPathCancel is
/// for, and starting a new one implies it.
ZRC_API ZrcResult zrcSlicedFindPathInit(ZrcNavMeshQuery* query,
                                        ZrcPolyRef start_ref,
                                        ZrcPolyRef end_ref,
                                        const float* start_pos,
                                        const float* end_pos,
                                        const ZrcQueryFilter* filter,
                                        uint32_t options);

/// Advances the search by at most `max_iters` node expansions.
///
/// `*out_iters` receives how many it actually ran, and `*out_in_progress` says
/// whether more remain; both may be NULL. A search that has finished reports
/// ZRC_FALSE and is ready for zrcSlicedFindPathFinalize.
///
/// A search that ran out of nodes, or whose start or end polygon stopped being
/// valid, reports the failure here rather than at finalise.
ZRC_API ZrcResult zrcSlicedFindPathUpdate(ZrcNavMeshQuery* query,
                                          int32_t max_iters,
                                          int32_t* out_iters,
                                          ZrcBool* out_in_progress);

/// Reads out the corridor a finished search found, and ends the slice.
///
/// `*out_partial` is set when the search could not reach the end polygon and
/// the corridor is the best effort towards it, or when `max_path` could not
/// hold the whole of it.
///
/// Calling this twice is ZRC_ERR_NO_SEARCH rather than an answer. Upstream
/// memsets its query state on the way out and guards re-entry with
/// dtStatusFailed, which is false for the zero that memset just wrote — so a
/// second call falls into the "start and end are the same polygon" branch,
/// both being 0, and returns success with a one-element path holding the null
/// reference (DetourNavMeshQuery.cpp:1516-1520, 1579).
ZRC_API ZrcResult zrcSlicedFindPathFinalize(ZrcNavMeshQuery* query,
                                            ZrcPolyRef* out_path,
                                            int32_t max_path,
                                            int32_t* out_count,
                                            ZrcBool* out_partial);

/// Ends the slice early, keeping as much of an existing corridor as the search
/// has confirmed.
///
/// `existing` is a corridor from an earlier search, walked from its end
/// backwards until one of its polygons is found in the search's own tree; the
/// result is that prefix joined to what the search reached. This is how a host
/// re-plans a path it is already walking without discarding it.
ZRC_API ZrcResult zrcSlicedFindPathFinalizePartial(
    ZrcNavMeshQuery* query, const ZrcPolyRef* existing, int32_t existing_count,
    ZrcPolyRef* out_path, int32_t max_path, int32_t* out_count,
    ZrcBool* out_partial);

/// Abandons a slice in flight, releasing the query for other searches.
///
/// Upstream has no such call — a slice ends only by being finalised — so a
/// host that changed its mind had to finalise into a buffer it then threw
/// away. Returns ZRC_OK whether or not a slice was in flight.
ZRC_API ZrcResult zrcSlicedFindPathCancel(ZrcNavMeshQuery* query);

/// Whether a sliced search is in flight on this query.
ZRC_API ZrcBool zrcSlicedFindPathActive(const ZrcNavMeshQuery* query);

//===----------------------------------------------------------------------===//
// Random points
//
// Both entry points take the randomness rather than making it, so a placement
// is reproducible from a seed and a host's own generator is the one used.
//
// Upstream's parameter is a bare `float (*)()` with nowhere to put a context
// (DetourNavMeshQuery.h:460), so the seam here carries one and the FFI layer
// bridges the two. The bridge is thread-local, which is what makes two threads
// with two generators safe; the header says so rather than leaving it to be
// discovered.
//===----------------------------------------------------------------------===//

typedef struct ZrcRandomSource {
  /// Must return a value in [0, 1). Invoked synchronously, on the calling
  /// thread, before the entry point returns, an unspecified number of times:
  /// once per resident tile and once per candidate polygon for
  /// zrcFindRandomPoint, once per polygon the search visits for
  /// zrcFindRandomPointAroundCircle, plus two to place the point inside the
  /// triangle it settled on.
  float (*next)(void* user);
  /// Opaque host pointer, passed back unmodified.
  void* user;
} ZrcRandomSource;

/// A random point on the navmesh, uniform by polygon area.
ZRC_API ZrcResult zrcFindRandomPoint(const ZrcNavMeshQuery* query,
                                     const ZrcQueryFilter* filter,
                                     const ZrcRandomSource* random,
                                     ZrcPolyRef* out_ref, float* out_point);

/// A random point on a polygon the search reached within `max_radius` of
/// `center`, starting from `start_ref`.
///
/// Reachable, not merely near: the search walks the surface, so a polygon
/// across a wall is not a candidate however close it is.
///
/// The **polygon** is what the radius bounds, not the point. Upstream picks a
/// polygon the search reached and then places the point anywhere inside it
/// (DetourNavMeshQuery.cpp:478-492), and a polygon reached near the edge of
/// the circle extends past it. A host that needs the point itself inside a
/// radius has to check it.
ZRC_API ZrcResult zrcFindRandomPointAroundCircle(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float max_radius, const ZrcQueryFilter* filter,
    const ZrcRandomSource* random, ZrcPolyRef* out_ref, float* out_point);

//===----------------------------------------------------------------------===//
// Polygons in a box
//===----------------------------------------------------------------------===//

/// Every polygon whose bounds overlap the box `center` +/- `half_extents`.
///
/// `*out_truncated` is set when more polygons overlapped than `max_refs` could
/// hold. May be NULL.
///
/// The box is limited: the tile range it implies must be nameable by an int
/// and must span fewer than four million grid cells, because Detour walks
/// every cell of that range whether a tile is there or not. A box meant to
/// cover a whole navmesh should be built from zrcNavMeshBounds rather than
/// from an arbitrarily large extent.
ZRC_API ZrcResult zrcQueryPolygons(const ZrcNavMeshQuery* query,
                                   const float* center,
                                   const float* half_extents,
                                   const ZrcQueryFilter* filter,
                                   ZrcPolyRef* out_refs, int32_t max_refs,
                                   int32_t* out_count, ZrcBool* out_truncated);

typedef struct ZrcPolyQuery {
  /// Called with a batch of polygon references. `refs` is borrowed and valid
  /// only for the duration of the call.
  ///
  /// Called more than once per query, and the batching is upstream's rather
  /// than this binding's: once per **32 polygons within a single tile**, plus
  /// once for whatever is left when that tile is finished. A batch never spans
  /// two tiles, so a small tile produces a small batch.
  void (*process)(void* user, const ZrcPolyRef* refs, int32_t count);
  /// Opaque host pointer, passed back unmodified.
  void* user;
} ZrcPolyQuery;

/// The same query, delivered in batches instead of into a buffer.
///
/// For a host that would rather not size an array for a result it cannot
/// predict. Upstream hands its callback the tile and the polygon structures
/// too; those are pointers into a tile's live storage, so only the references
/// cross this boundary. Everything about a polygon is readable from its
/// reference through zrcNavMeshPolyInfo.
///
/// One truncation this cannot report, because upstream does not: a grid column
/// stacked more than 32 tiles deep silently drops the rest from every query
/// that touches it (DetourNavMeshQuery.cpp:944, DetourNavMesh.cpp:1133).
ZRC_API ZrcResult zrcQueryPolygonsBatched(const ZrcNavMeshQuery* query,
                                          const float* center,
                                          const float* half_extents,
                                          const ZrcQueryFilter* filter,
                                          const ZrcPolyQuery* sink);

//===----------------------------------------------------------------------===//
// A point against one polygon
//===----------------------------------------------------------------------===//

/// Closest point on `ref` to `pos`, using the polygon's detail mesh for
/// height. `*out_over_poly` says whether `pos` was already above it; may be
/// NULL.
ZRC_API ZrcResult zrcClosestPointOnPoly(const ZrcNavMeshQuery* query,
                                        ZrcPolyRef ref, const float* pos,
                                        float* out_point,
                                        ZrcBool* out_over_poly);

/// Closest point on `ref`'s boundary to `pos`, which is the polygon itself
/// when `pos` is outside it and the projection onto its edge otherwise.
///
/// Cheaper than zrcClosestPointOnPoly and does not consult the detail mesh, so
/// the height it reports is the polygon's own.
ZRC_API ZrcResult zrcClosestPointOnPolyBoundary(const ZrcNavMeshQuery* query,
                                                ZrcPolyRef ref,
                                                const float* pos,
                                                float* out_point);

/// Height of `ref`'s surface directly under `pos`.
///
/// ZRC_ERR_INVALID_ARGUMENT when `pos` is not over the polygon at all, which
/// is the answer rather than an approximation.
ZRC_API ZrcResult zrcPolyHeight(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                                const float* pos, float* out_height);

//===----------------------------------------------------------------------===//
// Searching outwards
//
// Three Dijkstra searches over the surface, each answering "what can I reach"
// rather than "how do I get there". All three fill several arrays from one
// capacity, so each array given must hold `max_result` entries.
//===----------------------------------------------------------------------===//

/// Every polygon reachable from `start_ref` within `radius` of `center`.
///
/// `out_refs`, `out_parents` and `out_costs` may each be NULL; any that is not
/// must hold `max_result` entries. `out_parents` gives the polygon each was
/// reached through, which is what makes the result a tree rather than a set.
ZRC_API ZrcResult zrcFindPolysAroundCircle(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float radius, const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
    ZrcPolyRef* out_parents, float* out_costs, int32_t max_result,
    int32_t* out_count, ZrcBool* out_truncated);

/// The same, bounded by a convex polygon instead of a circle.
///
/// `verts` is `3 * vert_count` floats, projected onto the xz-plane.
ZRC_API ZrcResult zrcFindPolysAroundShape(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* verts,
    int32_t vert_count, const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
    ZrcPolyRef* out_parents, float* out_costs, int32_t max_result,
    int32_t* out_count, ZrcBool* out_truncated);

/// The corridor from the last Dijkstra search's start to `end_ref`.
///
/// Reads the search tree the two entry points above leave behind, so it is
/// only meaningful immediately after one of them, and only for a polygon that
/// search reached.
ZRC_API ZrcResult zrcPathFromDijkstraSearch(const ZrcNavMeshQuery* query,
                                            ZrcPolyRef end_ref,
                                            ZrcPolyRef* out_path,
                                            int32_t max_path,
                                            int32_t* out_count,
                                            ZrcBool* out_truncated);

/// Polygons touching a circle around `center`, without a real search.
///
/// Walks only the polygons adjacent to `start_ref` and their neighbours, which
/// makes it cheap enough for a per-frame collision query and useless for
/// anything further than a step away.
///
/// `out_refs` must be non-NULL, unlike its two siblings above: upstream writes
/// through it without checking (DetourNavMeshQuery.cpp:3135, 3252) while both
/// of them guard the same write.
ZRC_API ZrcResult zrcFindLocalNeighbourhood(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float radius, const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
    ZrcPolyRef* out_parents, int32_t max_result, int32_t* out_count,
    ZrcBool* out_truncated);

//===----------------------------------------------------------------------===//
// Walls
//===----------------------------------------------------------------------===//

/// The segments of `ref`'s boundary an agent under `filter` cannot cross.
///
/// `out_verts` receives `6 * (*out_count)` floats — two points per segment —
/// and `out_refs`, which may be NULL, receives the polygon on the far side of
/// each, or 0 where there is none. Both arrays are filled from `max_segments`,
/// so both must hold that many.
ZRC_API ZrcResult zrcPolyWallSegments(const ZrcNavMeshQuery* query,
                                      ZrcPolyRef ref,
                                      const ZrcQueryFilter* filter,
                                      float* out_verts, ZrcPolyRef* out_refs,
                                      int32_t max_segments, int32_t* out_count,
                                      ZrcBool* out_truncated);

/// Distance from `center` to the nearest wall within `max_radius`.
///
/// `*out_found` is ZRC_FALSE when nothing is within range, and that is a real
/// distinction upstream does not draw: it returns success either way, leaves
/// `hitPos` untouched, and then normalises `center - hitPos` over whatever was
/// there (DetourNavMeshQuery.cpp:3648-3650). With nothing found, `*out_dist`
/// is `max_radius`, and the position and normal are zeroed here rather than
/// left as the caller's own memory.
///
/// The normal is also NaN in upstream whenever the agent stands exactly on the
/// wall, because dtVnormalize has no zero-length guard. That case reports the
/// hit with a zeroed normal instead.
ZRC_API ZrcResult zrcFindDistanceToWall(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float max_radius, const ZrcQueryFilter* filter, float* out_dist,
    float* out_pos, float* out_normal, ZrcBool* out_found);

//===----------------------------------------------------------------------===//
// What a reference means
//===----------------------------------------------------------------------===//

/// Whether `ref` names a live polygon, and — when `filter` is given — one this
/// filter would admit.
///
/// `filter` may be NULL, which asks the plainer question: does `ref` decode to
/// a polygon that exists, with no admission test. A non-NULL filter asks the
/// stricter one upstream calls "valid": exists, and passes the filter.
///
/// Upstream dereferences the filter with no null check, alone among the
/// filtered methods in its file (DetourNavMeshQuery.cpp:3663) — which is
/// exactly why the NULL case here has to take its own path rather than a
/// filter that admits everything.
ZRC_API ZrcResult zrcIsValidPolyRef(const ZrcNavMeshQuery* query,
                                    ZrcPolyRef ref,
                                    const ZrcQueryFilter* filter,
                                    ZrcBool* out_valid);

/// Whether the last search closed `ref` — visited it and settled its cost.
ZRC_API ZrcResult zrcIsInClosedList(const ZrcNavMeshQuery* query,
                                    ZrcPolyRef ref, ZrcBool* out_closed);

/// The navmesh a query was created against. Borrowed; the caller still owns it.
ZRC_API ZrcResult zrcQueryNavMesh(const ZrcNavMeshQuery* query,
                                  const ZrcNavMesh** out_navmesh);

/// Splits a polygon reference into the three fields packed into it.
///
/// The widths are decided when the navmesh is created, from its tile and
/// polygon counts, so a reference is only meaningful against the navmesh that
/// minted it. Any out parameter may be NULL.
ZRC_API ZrcResult zrcDecodePolyRef(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                   uint32_t* out_salt, uint32_t* out_tile,
                                   uint32_t* out_poly);

/// Packs the three fields back into a reference.
///
/// ZRC_ERR_INVALID_ARGUMENT when any of them is too wide for the navmesh's own
/// bit layout, which upstream instead truncates silently.
ZRC_API ZrcResult zrcEncodePolyRef(const ZrcNavMesh* navmesh, uint32_t salt,
                                   uint32_t tile, uint32_t poly,
                                   ZrcPolyRef* out_ref);

/// Which edges of a detail triangle lie on the polygon's own boundary.
///
/// `tri_flags` is ZrcTileInfo-relative detail triangle data as
/// zrcNavMeshTileDetailTris reports it: every fourth byte. `edge_index` is 0,
/// 1 or 2. Test the result against ZRC_DETAIL_EDGE_BOUNDARY.
#define ZRC_DETAIL_EDGE_BOUNDARY 0x01

//===----------------------------------------------------------------------===//
// The search's own node pool
//
// A query owns a fixed pool of search nodes, sized at creation. Everything
// here reads it: how full it is, and what the last search concluded about a
// particular polygon. Nothing here writes to it — the searches do that, and a
// host reaching in to allocate or reorder nodes would be corrupting the search
// that owns them rather than exercising a capability.
//
// Useful for one thing in particular: ZRC_ERR_OUT_OF_NODES says a search ran
// out, and this says how close the ones that succeeded came.
//===----------------------------------------------------------------------===//

/// How many distinct search states one polygon may carry at once
/// (Detour's DT_MAX_STATES_PER_NODE).
#define ZRC_MAX_NODE_STATES 4

/// What a search concluded about one polygon (Detour's dtNodeFlags).
enum {
  /// On the open list: reached, cost not yet settled.
  ZRC_NODE_OPEN = 0x01,
  /// On the closed list: cost settled.
  ZRC_NODE_CLOSED = 0x02,
  /// Reached by a raycast shortcut, so its parent is not adjacent to it.
  ZRC_NODE_PARENT_DETACHED = 0x04,
};

/// One search node, copied out. Upstream packs the last three into bitfields
/// of a single word; they are whole fields here.
typedef struct ZrcNode {
  /// Position the search reached the polygon at.
  float pos[3];
  /// Cost of the step into this polygon.
  float cost;
  /// Cost from the search's start to here.
  float total;
  /// Polygon this node stands for.
  ZrcPolyRef ref;
  /// One-based index of the node this was reached from, or 0 for the start.
  /// Pass it to zrcQueryNodeAt to walk back towards the start.
  uint32_t parent_index;
  /// Which of the polygon's states this node is, below ZRC_MAX_NODE_STATES.
  uint32_t state;
  /// A combination of the ZRC_NODE_* flags.
  uint32_t flags;
} ZrcNode;

/// How full the query's node pool is.
typedef struct ZrcNodePoolInfo {
  /// Nodes the last search used.
  int32_t node_count;
  /// Nodes the pool can hold, as zrcNavMeshQueryCreate sized it.
  int32_t max_nodes;
  /// Buckets in the pool's hash table, always a power of two.
  int32_t hash_size;
  /// Bytes the pool occupies.
  int32_t bytes_used;
} ZrcNodePoolInfo;

ZRC_API ZrcResult zrcQueryNodePoolInfo(const ZrcNavMeshQuery* query,
                                       ZrcNodePoolInfo* out);

/// The node the last search made for `ref` in state `state`, if it made one.
///
/// ZRC_ERR_NOT_FOUND when the search never reached that polygon, which is the
/// difference between "cost zero" and "not visited".
ZRC_API ZrcResult zrcQueryFindNode(const ZrcNavMeshQuery* query,
                                   ZrcPolyRef ref, uint32_t state,
                                   ZrcNode* out);

/// Every node the last search made for `ref`, across its states.
///
/// `out` holds `max_nodes` entries; `*out_count` receives how many were
/// written, at most ZRC_MAX_NODE_STATES.
ZRC_API ZrcResult zrcQueryFindNodes(const ZrcNavMeshQuery* query,
                                    ZrcPolyRef ref, ZrcNode* out,
                                    int32_t max_nodes, int32_t* out_count);

/// The node at a one-based pool index, which is what ZrcNode.parent_index
/// carries. Index 0 is "no node" and is ZRC_ERR_NOT_FOUND.
ZRC_API ZrcResult zrcQueryNodeAt(const ZrcNavMeshQuery* query, uint32_t index,
                                 ZrcNode* out);

//===----------------------------------------------------------------------===//
// The layered heightfield
//
// One more container of the staged pipeline, held back until now because its
// consumer is the tile cache below rather than the polygon mesh. Where a
// compact heightfield is one surface with overlaps folded together, a layer
// set separates them: each layer is a sheet of walkable surface at one level,
// small enough to compress and carve obstacles into at runtime.
//===----------------------------------------------------------------------===//

/// A set of walkable sheets cut out of a compact heightfield.
typedef struct ZrcHeightfieldLayerSet ZrcHeightfieldLayerSet;

/// One sheet's extent and where its usable data sits inside it.
typedef struct ZrcHeightfieldLayer {
  float bmin[3];
  float bmax[3];
  float cell_size;
  float cell_height;
  /// Cells along x and z. The three arrays are `width * height` long.
  int32_t width;
  int32_t height;
  /// The sub-region holding data, in cells. Outside it the arrays are filled
  /// but meaningless, so the bounds are reported rather than assumed.
  int32_t min_x;
  int32_t max_x;
  /// Upstream spells these `miny` and `maxy` while meaning the z axis, the
  /// same wart rcGetDirOffsetY carries.
  int32_t min_z;
  int32_t max_z;
  /// The layer's own height range, in cells above `bmin[1]`.
  int32_t height_min;
  int32_t height_max;
} ZrcHeightfieldLayer;

/// Cuts `field` into layers, each an unobstructed sheet of walkable surface.
///
/// `border_size` is the unnavigable ring in cells a tiled build carries, and
/// `walkable_height` is in cells. Upstream stops at 63 overlapping platforms
/// and fails the whole build when a field has more, naming the reason in the
/// log — so pass a build context if this returns ZRC_ERR_BAKE_FAILED and the
/// geometry has many stacked floors.
/// [Limit: 0 <= border_size <= 255, 0 <= walkable_height <= ZRC_SPAN_MAX_HEIGHT]
ZRC_API ZrcResult zrcHeightfieldLayerSetCreate(const ZrcBuildContext* context,
                                               const ZrcCompactHeightfield* field,
                                               int32_t border_size,
                                               int32_t walkable_height,
                                               ZrcHeightfieldLayerSet** out);

ZRC_API void zrcHeightfieldLayerSetDestroy(ZrcHeightfieldLayerSet* layers);

/// How many layers the set holds. Zero is a legitimate answer for a field
/// with no walkable surface.
ZRC_API ZrcResult zrcHeightfieldLayerSetCount(const ZrcHeightfieldLayerSet* layers,
                                              int32_t* out_count);

ZRC_API ZrcResult zrcHeightfieldLayerAt(const ZrcHeightfieldLayerSet* layers,
                                        int32_t index,
                                        ZrcHeightfieldLayer* out);

/// Copies `count` height samples of layer `index` from `first`, one byte each.
/// The array is `width * height` long, row-major.
ZRC_API ZrcResult zrcHeightfieldLayerHeights(const ZrcHeightfieldLayerSet* layers,
                                             int32_t index, int32_t first,
                                             int32_t count, uint8_t* out);

/// The same for the area ids, one per sample.
ZRC_API ZrcResult zrcHeightfieldLayerAreas(const ZrcHeightfieldLayerSet* layers,
                                           int32_t index, int32_t first,
                                           int32_t count, uint8_t* out);

/// The same for the packed neighbour connections: the low nibble carries one
/// bit per direction for a neighbour inside this layer, and the high nibble
/// one bit per direction for a portal into a different layer. Decode with the
/// layer connection helpers rather than by hand.
ZRC_API ZrcResult zrcHeightfieldLayerCons(const ZrcHeightfieldLayerSet* layers,
                                          int32_t index, int32_t first,
                                          int32_t count, uint8_t* out);

//===----------------------------------------------------------------------===//
// The tile cache — obstacles carved into a baked mesh at runtime
//
// A baked navmesh is fixed at cook time. A tile cache keeps each tile's
// walkable surface in a compressed layer beside the navmesh, and rebuilds a
// tile from that layer whenever an obstacle over it appears or goes away. It
// is how a game does a door, a destructible, or a bridge that collapses.
//
// The shape a host works in:
//
//   cook time   a layer per tile, from zrcHeightfieldLayerSetCreate,
//               compressed with the host's codec by zrcTileCacheLayerBuild
//   load time   zrcTileCacheCreate, then zrcTileCacheAddTile per layer
//   run time    zrcTileCacheAddCylinderObstacle and friends, then
//               zrcTileCacheUpdate in a loop until it reports up to date
//
// No codec is bundled and no container format is invented. Upstream's own
// interface is three functions and the vendored tree ships no implementation
// of it, so this package supplies none either: a host picks a codec, and the
// bytes a cook produces are the bytes that codec produced.
//
// Tile-cache tiles carry no bounding-volume tree — upstream sets
// buildBvTree false when it builds one — so every query against one falls
// back to a linear scan of the tile. That is upstream's design decision, and
// it is what makes tile size worth choosing carefully.
//===----------------------------------------------------------------------===//

/// Magic and version of a compressed layer's header (DT_TILECACHE_MAGIC,
/// DT_TILECACHE_VERSION). Mirrored so a host can recognise its own asset.
#define ZRC_TILECACHE_MAGIC 0x44544C52
#define ZRC_TILECACHE_VERSION 1

/// How many tiles one obstacle may overlap (DT_MAX_TOUCHED_TILES).
///
/// Upstream's own limit, and a hard one: an obstacle touching more tiles is
/// carved into the first eight and silently missing from the rest. This
/// package refuses such an obstacle instead — see zrcTileCacheAddBoxObstacle.
#define ZRC_MAX_TOUCHED_TILES 8

/// A live tile cache: the compressed layers, the obstacles over them, and the
/// queue of tiles waiting to be rebuilt.
typedef struct ZrcTileCache ZrcTileCache;

/// Names one compressed layer in a tile cache. Salt-protected the same way a
/// ZrcTileRef is: removing a tile invalidates every reference to it.
typedef uint32_t ZrcCompressedTileRef;

/// Names one obstacle. Salt-protected, and the salt turns over when the
/// deferred removal completes rather than when it is requested.
typedef uint32_t ZrcObstacleRef;

typedef struct ZrcTileCacheParams {
  /// Minimum corner of the whole world, the same origin the navmesh uses.
  float origin[3];
  float cell_size;
  float cell_height;
  /// One tile's edge, in cells. [Limit: 1 <= value <= 255, because a layer
  /// header stores each as a single byte]
  int32_t width;
  int32_t height;
  /// The agent, in world units. Detour needs these for every tile the cache
  /// rebuilds, and they are the ones a cook used.
  float walkable_height;
  float walkable_radius;
  float walkable_climb;
  /// How far a simplified contour may sit from the traced one, in cells.
  float max_simplification_error;
  /// [Limit: 1 <= max_tiles, 1 <= max_obstacles, and max_tiles must leave
  /// Detour enough salt bits — the same rule zrcNavMeshCreateTiled applies]
  int32_t max_tiles;
  int32_t max_obstacles;
} ZrcTileCacheParams;

/// The codec a host supplies. Every field is required.
///
/// Upstream calls these while rebuilding a tile, which happens inside
/// zrcTileCacheUpdate and zrcTileCacheLayerBuild. A hook that returns
/// anything but ZRC_OK aborts the operation that called it.
typedef struct ZrcTileCacheCompressor {
  void* user;
  /// An upper bound on what `compress` may write for a buffer this size.
  /// Must not be less than `buffer_size`, and must not overflow an int32.
  int32_t (*max_compressed_size)(void* user, int32_t buffer_size);
  ZrcResult (*compress)(void* user, const uint8_t* buffer, int32_t buffer_size,
                        uint8_t* compressed, int32_t max_compressed_size,
                        int32_t* out_compressed_size);
  /// `compressed_size` is what the caller stored, and is checked to be
  /// positive before it reaches here: upstream computes it as a subtraction
  /// that goes negative for a short buffer and hands the result over as an
  /// int. See UPSTREAM.md.
  ZrcResult (*decompress)(void* user, const uint8_t* compressed,
                          int32_t compressed_size, uint8_t* buffer,
                          int32_t max_buffer_size, int32_t* out_size);
} ZrcTileCacheCompressor;

/// Optional scratch allocator for a tile rebuild, reset once per tile.
///
/// Upstream's own default forwards to the same allocator zrcSetAllocator
/// installs and its reset does nothing, which is what NULL selects here. A
/// host supplying one gets a linear arena per tile instead, which is what the
/// seam exists for.
typedef struct ZrcTileCacheAllocator {
  void* user;
  /// Called once at the start of every tile rebuild, before any allocation.
  void (*reset)(void* user);
  void* (*allocate)(void* user, size_t size);
  void (*deallocate)(void* user, void* block);
} ZrcTileCacheAllocator;

/// What a mesh-process callback may change about a tile before it is built.
///
/// Upstream hands its callback a mutable dtNavMeshCreateParams and then feeds
/// the result straight to dtNavMesh::addTile, which is the one path into a
/// navmesh that skips every check zrcNavMeshAddTile applies. Narrowing the
/// callback to the fields it is actually for closes that by construction: a
/// host can colour the tile, name it, and give it off-mesh connections, and
/// cannot describe a tile whose polygon count disagrees with its polygons.
typedef struct ZrcTileCacheBuildParams {
  /// One area id per polygon, writable. [Limit: each < ZRC_MAX_AREAS]
  uint8_t* areas;
  /// One flag word per polygon, writable.
  ///
  /// **Every entry arrives zero.** Upstream allocates this array and memsets
  /// it with a comment saying the user is responsible for filling it
  /// (DetourTileCacheBuilder.cpp:1790-1791), and dtCreateNavMeshData copies it
  /// verbatim. The `areas` above are not zeroed the same way: they carry the
  /// ids the cook wrote into the layer.
  uint16_t* flags;
  int32_t poly_count;
  /// Copied into the tile's header, for a host to recognise it later.
  uint32_t user_id;
  /// Off-mesh connections for this tile, or NULL for none. The array is
  /// borrowed for the duration of the callback and validated on return, by
  /// the same rules zrcTileDataBuild applies.
  const ZrcOffMeshConnection* connections;
  int32_t connection_count;
} ZrcTileCacheBuildParams;

/// Called once per tile rebuild, after the polygons exist and before the tile
/// is built.
///
/// NULL leaves every polygon's **flags** at 0, which no nonzero query filter
/// admits — a navmesh that is silently unreachable rather than one that fails.
/// The areas are not zeroed: they carry the ids the cook wrote into the layer,
/// so a callback that only wants to set flags can read `areas` to decide them.
typedef ZrcResult (*ZrcTileCacheMeshProcess)(void* user,
                                             ZrcTileCacheBuildParams* params);

/// Creates a tile cache. `compressor` is required; `allocator` and
/// `mesh_process` may both be NULL.
///
/// A cache cannot be re-initialised: upstream's init has no purge and leaks
/// every array if called twice, so there is one create and one destroy.
ZRC_API ZrcResult zrcTileCacheCreate(const ZrcTileCacheParams* params,
                                     const ZrcTileCacheCompressor* compressor,
                                     const ZrcTileCacheAllocator* allocator,
                                     ZrcTileCacheMeshProcess mesh_process,
                                     void* mesh_process_user,
                                     ZrcTileCache** out);

ZRC_API void zrcTileCacheDestroy(ZrcTileCache* cache);

ZRC_API ZrcResult zrcTileCacheParams(const ZrcTileCache* cache,
                                     ZrcTileCacheParams* out);

/// Adds one compressed layer, produced by zrcTileCacheLayerBuild.
///
/// The bytes are copied, so the caller keeps ownership of `data`. Upstream
/// reads the header off the buffer before comparing `size` against anything
/// and then stores a compressed length computed by subtraction, which goes
/// negative for a short buffer and is handed to the host codec as an int; the
/// header and the length are checked here first. See UPSTREAM.md.
///
/// ZRC_ERR_TILE_OCCUPIED when a layer already sits at that position.
ZRC_API ZrcResult zrcTileCacheAddTile(ZrcTileCache* cache, const void* data,
                                      size_t size,
                                      ZrcCompressedTileRef* out_ref);

/// Removes a layer. `out_data` and `out_size` may be NULL to discard it;
/// otherwise they receive a copy the caller owns and frees with zrcFree.
ZRC_API ZrcResult zrcTileCacheRemoveTile(ZrcTileCache* cache,
                                         ZrcCompressedTileRef ref,
                                         void** out_data, size_t* out_size);

/// What one compressed layer holds, copied out.
typedef struct ZrcCompressedTileInfo {
  int32_t tile_x;
  int32_t tile_y;
  int32_t tile_layer;
  float bmin[3];
  float bmax[3];
  /// The layer's height range, in cells above `bmin[1]`.
  int32_t height_min;
  int32_t height_max;
  /// Cells along x and z.
  int32_t width;
  int32_t height;
  /// The sub-region holding data, in cells.
  int32_t min_x;
  int32_t max_x;
  int32_t min_z;
  int32_t max_z;
  /// Bytes the cache holds for this layer, header included.
  int32_t data_size;
} ZrcCompressedTileInfo;

ZRC_API ZrcResult zrcTileCacheTileInfo(const ZrcTileCache* cache,
                                       ZrcCompressedTileRef ref,
                                       ZrcCompressedTileInfo* out);

/// The layer at (`tile_x`, `tile_y`, `tile_layer`), or 0 in `*out_ref` when
/// that position is empty — a legitimate answer, not an error.
ZRC_API ZrcResult zrcTileCacheTileAt(const ZrcTileCache* cache, int32_t tile_x,
                                     int32_t tile_y, int32_t tile_layer,
                                     ZrcCompressedTileRef* out_ref);

/// Every layer stacked at (`tile_x`, `tile_y`), lowest first. `*out_count` is
/// how many exist, whether or not they fit; ZRC_ERR_BUFFER_TOO_SMALL when
/// they do not.
ZRC_API ZrcResult zrcTileCacheTilesAt(const ZrcTileCache* cache,
                                      int32_t tile_x, int32_t tile_y,
                                      ZrcCompressedTileRef* out,
                                      int32_t max_tiles, int32_t* out_count);

/// The reference of the slot at `index`, or 0 when the slot is free.
///
/// Slots run from 0 to `max_tiles - 1`. Upstream's own accessor bounds
/// nothing at all, so the index is checked here.
ZRC_API ZrcResult zrcTileCacheTileRefAt(const ZrcTileCache* cache,
                                        int32_t index,
                                        ZrcCompressedTileRef* out_ref);

/// The three obstacle shapes, mirroring ObstacleType.
typedef enum ZrcObstacleShape {
  ZRC_OBSTACLE_CYLINDER = 0,
  ZRC_OBSTACLE_BOX = 1,
  ZRC_OBSTACLE_ORIENTED_BOX = 2,
} ZrcObstacleShape;

/// Where an obstacle is in the add/remove cycle, mirroring ObstacleState.
///
/// zrcTileCacheObstacleInfo never reports ZRC_OBSTACLE_EMPTY. Upstream turns
/// the slot's salt over in the same statement that sets the state, so by the
/// time a slot is empty every reference to it has stopped resolving and the
/// lookup answers ZRC_ERR_NOT_FOUND instead. An empty slot is observed through
/// zrcTileCacheObstacleRefAt, which reports 0 for one.
typedef enum ZrcObstacleState {
  /// The slot holds no obstacle. Present because upstream's enum has it, and
  /// unreachable through a reference for the reason above.
  ZRC_OBSTACLE_EMPTY = 0,
  /// Added, and at least one of its tiles has not been rebuilt yet.
  ZRC_OBSTACLE_PROCESSING = 1,
  /// Every tile it touches has been rebuilt; it is carved into the navmesh.
  ZRC_OBSTACLE_PROCESSED = 2,
  /// Removal requested, and at least one of its tiles has not been rebuilt.
  ZRC_OBSTACLE_REMOVING = 3,
} ZrcObstacleState;

typedef struct ZrcObstacleInfo {
  ZrcObstacleShape shape;
  ZrcObstacleState state;
  /// ZRC_OBSTACLE_CYLINDER: base centre, radius, height.
  float position[3];
  float radius;
  float height;
  /// ZRC_OBSTACLE_BOX: the two corners.
  float bmin[3];
  float bmax[3];
  /// ZRC_OBSTACLE_ORIENTED_BOX: centre, half extents, and the rotation.
  ///
  /// Upstream stores the angle as two derived terms rather than the angle,
  /// so this is recovered by inverting them and can differ from what was
  /// passed in the last bits. It is an accessor, not part of a cook.
  float center[3];
  float half_extents[3];
  float y_radians;
  /// How many tiles it overlaps, and how many of those still owe a rebuild.
  ///
  /// **Both are 0 until the first zrcTileCacheUpdate that processes the
  /// request.** Adding an obstacle queues it; upstream works out which tiles
  /// it lands on only when it comes to carve them, so an obstacle read back
  /// immediately reports the shape it was given and nothing about where it
  /// went.
  int32_t touched_count;
  int32_t pending_count;
} ZrcObstacleInfo;

ZRC_API ZrcResult zrcTileCacheObstacleInfo(const ZrcTileCache* cache,
                                           ZrcObstacleRef ref,
                                           ZrcObstacleInfo* out);

/// The reference of the obstacle slot at `index`, or 0 when it is free.
/// Upstream's own accessor bounds nothing; the index is checked here.
ZRC_API ZrcResult zrcTileCacheObstacleRefAt(const ZrcTileCache* cache,
                                            int32_t index,
                                            ZrcObstacleRef* out_ref);

/// A vertical cylinder standing on its base.
///
/// The obstacle is queued, not applied: it appears in the navmesh once
/// zrcTileCacheUpdate has rebuilt every tile it touches.
///
/// **An obstacle overlapping more than ZRC_MAX_TOUCHED_TILES tiles is
/// refused** with ZRC_ERR_BUFFER_TOO_SMALL. Upstream carves it into the first
/// eight and leaves the rest untouched without saying so, which is a hole in
/// the navmesh nothing reports; splitting it into several obstacles is the
/// fix. See UPSTREAM.md.
///
/// ZRC_ERR_NAVMESH_FULL when every obstacle slot is taken, and
/// ZRC_ERR_BUFFER_TOO_SMALL when the request queue is full. Upstream reports
/// the first as DT_OUT_OF_MEMORY, which a host would read as an allocator
/// failure; it is the cache being full, and the cache was created for
/// `max_obstacles`.
ZRC_API ZrcResult zrcTileCacheAddCylinderObstacle(ZrcTileCache* cache,
                                                  const float* position,
                                                  float radius, float height,
                                                  ZrcObstacleRef* out_ref);

/// An axis-aligned box. Same queueing and same limits as the cylinder.
ZRC_API ZrcResult zrcTileCacheAddBoxObstacle(ZrcTileCache* cache,
                                             const float* bmin,
                                             const float* bmax,
                                             ZrcObstacleRef* out_ref);

/// A box rotated about the y axis. Same queueing and same limits.
ZRC_API ZrcResult zrcTileCacheAddOrientedBoxObstacle(ZrcTileCache* cache,
                                                     const float* center,
                                                     const float* half_extents,
                                                     float y_radians,
                                                     ZrcObstacleRef* out_ref);

/// Queues an obstacle's removal. The reference stays resolvable until
/// zrcTileCacheUpdate has rebuilt every tile it touched, and only then does
/// its salt turn over — after which zrcTileCacheObstacleInfo answers
/// ZRC_ERR_NOT_FOUND for it rather than reporting an empty slot.
ZRC_API ZrcResult zrcTileCacheRemoveObstacle(ZrcTileCache* cache,
                                             ZrcObstacleRef ref);

/// Every layer overlapping the box, which is how a host sizes an obstacle
/// against ZRC_MAX_TOUCHED_TILES before adding one.
///
/// `*out_count` is how many overlap, whether or not they fit;
/// ZRC_ERR_BUFFER_TOO_SMALL when they do not. Upstream drops the overflow and
/// reports success.
ZRC_API ZrcResult zrcTileCacheQueryTiles(const ZrcTileCache* cache,
                                         const float* bmin, const float* bmax,
                                         ZrcCompressedTileRef* out,
                                         int32_t max_tiles,
                                         int32_t* out_count);

/// Processes queued obstacle requests and rebuilds **one** tile.
///
/// That is the throttle rather than a batch API: a host calls this in a loop
/// until `*out_up_to_date` is true, spending as much of a frame on it as it
/// can afford. `out_up_to_date` may be NULL.
///
/// A mesh-process callback that fails is reported here, but the tile it was
/// called for has already been committed by then: upstream's callback returns
/// void and the rebuild continues past it regardless. The tile is well formed
/// — it carries whatever the callback wrote before failing, and no off-mesh
/// connections — so the error means "rebuild that tile", not "nothing
/// happened".
///
/// The tiles this produces go into `navmesh` through Detour directly, without
/// the image validation zrcNavMeshAddTile applies. That is upstream's design
/// and not something a binding can insert itself into: the bytes are built by
/// upstream from a layer this package already checked on the way in, from
/// parameters it checked at creation, and from a mesh-process callback it
/// narrows to the fields the callback is for. Those three are where a host's
/// data enters, and those three are checked.
ZRC_API ZrcResult zrcTileCacheUpdate(ZrcTileCache* cache, ZrcNavMesh* navmesh,
                                     ZrcBool* out_up_to_date);

/// Rebuilds one named tile immediately, outside the update loop — how a host
/// commits a layer it has just added without waiting for an obstacle.
ZRC_API ZrcResult zrcTileCacheBuildNavMeshTile(ZrcTileCache* cache,
                                               ZrcCompressedTileRef ref,
                                               ZrcNavMesh* navmesh);

/// The same for every layer stacked at one grid position.
ZRC_API ZrcResult zrcTileCacheBuildNavMeshTilesAt(ZrcTileCache* cache,
                                                  int32_t tile_x,
                                                  int32_t tile_y,
                                                  ZrcNavMesh* navmesh);

//===----------------------------------------------------------------------===//
// Building a compressed layer, and taking one apart
//
// The cook-time half of the tile cache: turn one layer of a layer set into
// the compressed bytes a cache stores, and — for a tool that wants to see
// what a cache will do — take those bytes back apart and drive the same
// stages the cache drives internally.
//===----------------------------------------------------------------------===//

/// The area id a tile-cache layer gives unwalkable surface, and the one it
/// gives walkable surface (DT_TILECACHE_NULL_AREA, DT_TILECACHE_WALKABLE_AREA).
#define ZRC_TILECACHE_AREA_NULL 0
#define ZRC_TILECACHE_AREA_WALKABLE 63

/// The index a tile-cache polygon writes where it has no further corner and
/// no neighbour (DT_TILECACHE_NULL_IDX).
#define ZRC_TILECACHE_NULL_IDX 0xffff

/// A layer's identity and extent, as it crosses into and out of compression.
typedef struct ZrcTileCacheLayerHeader {
  int32_t tile_x;
  int32_t tile_y;
  int32_t tile_layer;
  float bmin[3];
  float bmax[3];
  /// The layer's height range, in cells above `bmin[1]`.
  int32_t height_min;
  int32_t height_max;
  /// Cells along x and z. [Limit: 1 <= value <= 255 — upstream stores each as
  /// a single byte, and the three grids are `width * height` long]
  int32_t width;
  int32_t height;
  /// The sub-region holding usable data, in cells.
  /// [Limit: 0 <= min_x <= max_x < width, 0 <= min_z <= max_z < height]
  int32_t min_x;
  int32_t max_x;
  int32_t min_z;
  int32_t max_z;
} ZrcTileCacheLayerHeader;

/// Compresses one layer into the bytes zrcTileCacheAddTile takes.
///
/// `heights`, `areas` and `cons` are each `width * height` bytes, the three
/// arrays zrcHeightfieldLayerHeights and its siblings report. `*out_data` is
/// the caller's to free with zrcFree.
ZRC_API ZrcResult zrcTileCacheLayerBuild(
    const ZrcTileCacheCompressor* compressor,
    const ZrcTileCacheLayerHeader* header, const uint8_t* heights,
    const uint8_t* areas, const uint8_t* cons, void** out_data,
    size_t* out_size);

/// Swaps a compressed layer header between endiannesses, in place.
///
/// For a host reading an asset cooked on a machine of the other byte order.
/// Upstream takes a length and discards it before dereferencing the header;
/// the length is checked here. Returns ZRC_ERR_BAD_FORMAT for bytes that are
/// not a layer header in either order.
ZRC_API ZrcResult zrcTileCacheHeaderSwapEndian(void* data, size_t size);

/// One decompressed layer: the three grids, plus the regions once they are
/// built.
typedef struct ZrcTileCacheLayer ZrcTileCacheLayer;

/// Decompresses a layer, through the same codec that produced it.
ZRC_API ZrcResult zrcTileCacheLayerDecompress(
    const ZrcTileCacheCompressor* compressor,
    const ZrcTileCacheAllocator* allocator, const void* data, size_t size,
    ZrcTileCacheLayer** out);

ZRC_API void zrcTileCacheLayerDestroy(ZrcTileCacheLayer* layer);

/// The layer's own header, by value.
///
/// Spelled HeaderOf rather than Info because it hands back the same
/// ZrcTileCacheLayerHeader zrcTileCacheLayerBuild takes, not a shape invented
/// for reading: build a layer from a header and read it back and the two agree
/// field for field.
ZRC_API ZrcResult zrcTileCacheLayerHeaderOf(const ZrcTileCacheLayer* layer,
                                            ZrcTileCacheLayerHeader* out);

/// How many regions zrcTileCacheLayerBuildRegions found, or 0 before it runs.
ZRC_API ZrcResult zrcTileCacheLayerRegionCount(const ZrcTileCacheLayer* layer,
                                               int32_t* out_count);

/// Copies `count` height samples from `first`. The array is
/// `width * height` long, row-major.
ZRC_API ZrcResult zrcTileCacheLayerHeights(const ZrcTileCacheLayer* layer,
                                           int32_t first, int32_t count,
                                           uint8_t* out);

/// The same for the area ids, one per sample.
ZRC_API ZrcResult zrcTileCacheLayerAreas(const ZrcTileCacheLayer* layer,
                                         int32_t first, int32_t count,
                                         uint8_t* out);

/// Writes `count` area ids back from `first`.
/// [Limit: each < ZRC_MAX_AREAS]
ZRC_API ZrcResult zrcTileCacheLayerSetAreas(ZrcTileCacheLayer* layer,
                                            int32_t first, int32_t count,
                                            const uint8_t* areas);

/// The same for the packed connections: the low nibble is one bit per
/// direction for a neighbour inside this layer, the high nibble one bit per
/// direction for a portal into another.
ZRC_API ZrcResult zrcTileCacheLayerCons(const ZrcTileCacheLayer* layer,
                                        int32_t first, int32_t count,
                                        uint8_t* out);

/// The region id of each sample, once regions have been built.
/// ZRC_ERR_NOT_FOUND before zrcTileCacheLayerBuildRegions has run.
ZRC_API ZrcResult zrcTileCacheLayerRegions(const ZrcTileCacheLayer* layer,
                                           int32_t first, int32_t count,
                                           uint8_t* out);

/// Carves a cylinder into the layer's area ids, the same way an obstacle
/// does. `origin`, `cell_size` and `cell_height` are the layer's own, from
/// the tile cache's parameters.
ZRC_API ZrcResult zrcTileCacheLayerMarkCylinder(
    ZrcTileCacheLayer* layer, const float* origin, float cell_size,
    float cell_height, const float* position, float radius, float height,
    uint8_t area);

/// The same for an axis-aligned box.
ZRC_API ZrcResult zrcTileCacheLayerMarkBox(ZrcTileCacheLayer* layer,
                                           const float* origin,
                                           float cell_size, float cell_height,
                                           const float* bmin,
                                           const float* bmax, uint8_t area);

/// The same for a box rotated about the y axis.
ZRC_API ZrcResult zrcTileCacheLayerMarkOrientedBox(
    ZrcTileCacheLayer* layer, const float* origin, float cell_size,
    float cell_height, const float* center, const float* half_extents,
    float y_radians, uint8_t area);

/// Splits the layer's walkable surface into regions, in place.
/// `walkable_climb` is in cells.
ZRC_API ZrcResult zrcTileCacheLayerBuildRegions(ZrcTileCacheLayer* layer,
                                                int32_t walkable_climb);

/// The traced outlines of a layer's regions.
typedef struct ZrcTileCacheContourSet ZrcTileCacheContourSet;

typedef struct ZrcTileCacheContourInfo {
  int32_t vert_count;
  uint8_t region;
  uint8_t area;
} ZrcTileCacheContourInfo;

/// Traces and simplifies the outline of every region in `layer`, which must
/// have had its regions built. `max_error` is in cells.
ZRC_API ZrcResult zrcTileCacheContourSetCreate(
    const ZrcTileCacheAllocator* allocator, ZrcTileCacheLayer* layer,
    int32_t walkable_climb, float max_error, ZrcTileCacheContourSet** out);

ZRC_API void zrcTileCacheContourSetDestroy(ZrcTileCacheContourSet* contours);

ZRC_API ZrcResult zrcTileCacheContourSetCount(
    const ZrcTileCacheContourSet* contours, int32_t* out_count);

ZRC_API ZrcResult zrcTileCacheContourAt(const ZrcTileCacheContourSet* contours,
                                        int32_t index,
                                        ZrcTileCacheContourInfo* out);

/// Copies `count` vertices of contour `index` from `first`, four bytes each:
/// x, y, z in cells, then the packed connection of the edge starting here.
ZRC_API ZrcResult zrcTileCacheContourVerts(
    const ZrcTileCacheContourSet* contours, int32_t index, int32_t first,
    int32_t count, uint8_t* out);

/// The polygons a layer's contours become — what a tile cache hands Detour.
typedef struct ZrcTileCachePolyMesh ZrcTileCachePolyMesh;

typedef struct ZrcTileCachePolyMeshInfo {
  int32_t vert_count;
  int32_t poly_count;
  /// Corners per polygon, and half the stride of a `polys` entry. Always
  /// ZRC_VERTS_PER_POLYGON: upstream's tile-cache mesh builder has no other.
  int32_t verts_per_poly;
} ZrcTileCachePolyMeshInfo;

ZRC_API ZrcResult zrcTileCachePolyMeshCreate(
    const ZrcTileCacheAllocator* allocator,
    const ZrcTileCacheContourSet* contours, ZrcTileCachePolyMesh** out);

ZRC_API void zrcTileCachePolyMeshDestroy(ZrcTileCachePolyMesh* mesh);

ZRC_API ZrcResult zrcTileCachePolyMeshInfo(const ZrcTileCachePolyMesh* mesh,
                                           ZrcTileCachePolyMeshInfo* out);

/// Copies `count` vertices from `first`, three `uint16_t` each, in cells.
ZRC_API ZrcResult zrcTileCachePolyMeshVerts(const ZrcTileCachePolyMesh* mesh,
                                            int32_t first, int32_t count,
                                            uint16_t* out);

/// Copies `count` polygons from `first`, `2 * verts_per_poly` entries each:
/// the corner indices, then the neighbour across each edge.
/// ZRC_TILECACHE_NULL_IDX marks an absent corner or neighbour.
ZRC_API ZrcResult zrcTileCachePolyMeshPolys(const ZrcTileCachePolyMesh* mesh,
                                            int32_t first, int32_t count,
                                            uint16_t* out);

/// Copies `count` area ids from `first`, one per polygon.
ZRC_API ZrcResult zrcTileCachePolyMeshAreas(const ZrcTileCachePolyMesh* mesh,
                                            int32_t first, int32_t count,
                                            uint8_t* out);

/// Copies `count` polygon flags from `first`. Every entry is 0 unless a
/// mesh-process callback has written it.
ZRC_API ZrcResult zrcTileCachePolyMeshFlags(const ZrcTileCachePolyMesh* mesh,
                                            int32_t first, int32_t count,
                                            uint16_t* out);

//===----------------------------------------------------------------------===//
// Crowds — many agents steering around each other and the world
//
// A query answers where an agent could go. A crowd moves it: every frame it
// re-plans each agent's local path, looks at its neighbours and the walls
// nearby, picks a velocity that avoids both, and integrates. The navmesh says
// what is walkable; the crowd decides who walks where.
//
// The shape a host works in:
//
//   zrcCrowdCreate            once, against a navmesh that must outlive it
//   zrcCrowdAddAgent          per agent, giving position and parameters
//   zrcCrowdRequestMoveTarget per agent, whenever the destination changes
//   zrcCrowdUpdate            once a frame, with the frame's delta time
//   zrcCrowdAgentInfo         per agent, to read back position and velocity
//
// **The crowd owns an agent's movement.** Position and velocity are outputs it
// recomputes every frame from the parameters and the target; there is no entry
// point that writes them, because writing them is how a C++ host desynchronises
// an agent from its own path corridor. Upstream says the same thing in prose
// (DetourCrowd.h, "you give up direct control of the agent's movement") and
// then hands out a mutable pointer anyway. What a host controls is what
// zrcCrowdSetAgentParams and the three move-request entry points take.
//
// **Steering is not a cook.** The cross-platform guarantee this package makes
// covers baked assets: the same geometry produces the same navmesh bytes on
// every target (see zrcPolyMeshBake and UPSTREAM.md). It does not extend here.
// dtObstacleAvoidanceQuery::sampleVelocityAdaptive reaches cosf and sinf
// (DetourObstacleAvoidance.cpp:499-500, 535-536) — the only transcendentals
// anywhere in DetourCrowd — and replacing them with the exact polynomial the
// bake path uses would cost a hot loop every frame for a decision that is
// recomputed the next frame anyway. A host that needs a reproducible replay
// should record its inputs, not assume the steering repeats.
//
// **Agents are referenced, not indexed.** Upstream identifies an agent by its
// slot in a fixed pool: removeAgent only clears a flag, addAgent reuses the
// lowest free slot, and every setter bounds-checks the index without ever
// asking whether the slot is live. A stale index therefore drives whichever
// agent took the slot next, silently. ZrcAgentRef carries a serial number
// alongside the slot so a stale one is refused with ZRC_ERR_NOT_FOUND, the
// same shape ZrcTileRef and ZrcObstacleRef already have.
//===----------------------------------------------------------------------===//

/// Neighbours an agent steers around (DT_CROWDAGENT_MAX_NEIGHBOURS).
#define ZRC_CROWD_MAX_NEIGHBOURS 6

/// Corners of its local path an agent looks ahead to
/// (DT_CROWDAGENT_MAX_CORNERS). The last is the one it is heading for, so the
/// number of useful corners is one less.
#define ZRC_CROWD_MAX_CORNERS 4

/// Avoidance configurations a crowd holds (DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS).
#define ZRC_CROWD_MAX_AVOIDANCE_PARAMS 8

/// Query filters a crowd holds (DT_CROWD_MAX_QUERY_FILTER_TYPE).
#define ZRC_CROWD_MAX_FILTERS 16

/// Angular divisions the adaptive velocity sampler may use (DT_MAX_PATTERN_DIVS).
#define ZRC_AVOIDANCE_MAX_PATTERN_DIVS 32

/// Rings the adaptive velocity sampler may use (DT_MAX_PATTERN_RINGS).
#define ZRC_AVOIDANCE_MAX_PATTERN_RINGS 4

/// Cell coordinates a crowd's proximity grid addresses without overflowing.
///
/// Upstream hashes a cell by multiplying each coordinate by a large prime in
/// int arithmetic (DetourProximityGrid.cpp:43-46); 73856093 * 30 is already
/// past INT_MAX, so a coordinate outside this is signed overflow. The
/// coordinate is an agent's world position divided by the grid's cell size,
/// and a crowd sizes that cell at `max_agent_radius * 3` — so a world wider
/// than about this many cells reaches it.
///
/// **Not defended against, because there is no seam to defend at**: dtCrowd
/// calls its grid itself, from inside update(), with positions this package
/// never sees. A C++ host is in exactly the same position. Refusing large
/// worlds at the door would remove a capability upstream has rather than bind
/// it. Named here so a host building with UBSan knows what a trap in
/// hashPos2 is, and recorded in UPSTREAM.md so a re-vendor can check whether
/// upstream has fixed it. The standalone grid entry points below do not bound
/// it either: a C++ host passing the same coordinate gets the same behaviour.
#define ZRC_PROXIMITY_GRID_SAFE_CELL 29

/// Agents one crowd may be created for.
///
/// zrecast's own bound, not upstream's, and it comes from the proximity grid a
/// crowd builds for `max_agents * 4` entries. dtProximityGrid stores a pool
/// index in an `unsigned short` and reserves 0xffff as its end-of-chain marker
/// (DetourProximityGrid.cpp:120-127), so a pool of more than 65534 entries
/// truncates every index past that and links buckets to the wrong items —
/// silently, and only for the agents past the limit. See UPSTREAM.md.
#define ZRC_CROWD_MAX_AGENTS 16383

/// The handles the sections below define, forward-declared here because a
/// crowd owns or borrows one of each.
typedef struct ZrcProximityGrid ZrcProximityGrid;
typedef struct ZrcAvoidanceQuery ZrcAvoidanceQuery;
typedef struct ZrcAvoidanceDebug ZrcAvoidanceDebug;
typedef struct ZrcPathCorridor ZrcPathCorridor;
typedef struct ZrcLocalBoundary ZrcLocalBoundary;
typedef struct ZrcPathQueue ZrcPathQueue;

/// Where a path corridor is and what it holds.
///
/// Declared here rather than beside the corridor because every crowd agent
/// has one and zrcCrowdAgentCorridorInfo reports it.
typedef struct ZrcPathCorridorInfo {
  /// The current position, inside the first polygon.
  float position[3];
  /// The target, inside the last.
  float target[3];
  /// The polygon holding the position, or 0 when the corridor is empty.
  ZrcPolyRef first_poly;
  /// The polygon holding the target, or 0 when the corridor is empty.
  ZrcPolyRef last_poly;
  /// Polygons in the corridor.
  int32_t path_count;
} ZrcPathCorridorInfo;


///
/// Declared here rather than beside the sampler because a crowd holds
/// ZRC_CROWD_MAX_AVOIDANCE_PARAMS of these and each agent names one.
/// zrcAvoidanceParamsDefault fills in what a crowd installs.
typedef struct ZrcAvoidanceParams {
  /// How far towards the desired velocity the search starts, in [0, 1].
  float vel_bias;
  /// Weight on staying close to the velocity the path wants.
  float weight_desired_vel;
  /// Weight on staying close to the velocity already being travelled.
  float weight_current_vel;
  /// Weight on preferring one side of an obstacle over the other.
  float weight_side;
  /// Weight on how soon a candidate would collide.
  float weight_toi;
  /// How far ahead, in seconds, a collision still counts. [Limit: > 0]
  ///
  /// Refused at zero rather than passed through: upstream takes its reciprocal
  /// without a guard (DetourObstacleAvoidance.cpp:443, 515), and the infinity
  /// that follows makes every candidate score NaN. Every comparison against
  /// NaN is false, so the sampler keeps its zero-initialised answer and
  /// reports success — an agent that silently stops steering.
  float horiz_time;
  /// Candidates per axis for zrcAvoidanceSampleGrid. [Limit: >= 2]
  ///
  /// Upstream divides by `grid_size - 1` and clamps nothing
  /// (DetourObstacleAvoidance.cpp:454): 1 is a division by zero that ends in
  /// the same silent NaN, and 0 scores no candidate at all while still
  /// reporting success.
  uint8_t grid_size;
  /// Angular divisions per ring for zrcAvoidanceSampleAdaptive.
  /// [Limit: 1 to ZRC_AVOIDANCE_MAX_PATTERN_DIVS]
  uint8_t adaptive_divs;
  /// Rings for zrcAvoidanceSampleAdaptive.
  /// [Limit: 1 to ZRC_AVOIDANCE_MAX_PATTERN_RINGS]
  uint8_t adaptive_rings;
  /// Refinement passes for zrcAvoidanceSampleAdaptive. [Limit: >= 1]
  ///
  /// Upstream clamps the two above and not this one
  /// (DetourObstacleAvoidance.cpp:530-533), so 0 skips the search entirely and
  /// hands back the desired velocity scaled by `vel_bias` — avoidance turned
  /// off, reported as success. Large values are allowed and cost linearly.
  uint8_t adaptive_depth;
} ZrcAvoidanceParams;

/// The configuration a crowd installs in all eight of its slots.
///
/// Not upstream's: dtCrowd::init writes these ten values inline
/// (DetourCrowd.cpp:404-417) and offers no way to ask for them. Having them by
/// name is what lets a host change one field of a working configuration
/// instead of restating all ten.
ZRC_API void zrcAvoidanceParamsDefault(ZrcAvoidanceParams* out);

/// A reference to one agent in a crowd. 0 is never a live agent.
///
/// Slot and serial packed together: the low 16 bits are the pool slot plus
/// one, the high 48 are a counter the crowd never reuses. Removing an agent
/// and adding another puts a different serial in the same slot, so the old
/// reference stops resolving. Re-initialising a crowd retires every reference
/// it has ever minted, for the same reason.
typedef uint64_t ZrcAgentRef;

/// What kind of surface an agent is on (Detour's CrowdAgentState).
typedef enum ZrcCrowdAgentState {
  /// Not on the navmesh at all: the position it was added at projected onto
  /// nothing, or the polygon under it has since been removed. It will not be
  /// steered until it recovers.
  ZRC_CROWD_AGENT_INVALID = 0,
  /// On an ordinary polygon.
  ZRC_CROWD_AGENT_WALKING = 1,
  /// Traversing an off-mesh connection. Steering is suspended for the
  /// duration, and how far along it is cannot be read: upstream keeps that in
  /// a private array with no accessor. See ZrcCrowdAgentAnimation.
  ZRC_CROWD_AGENT_OFFMESH = 2,
} ZrcCrowdAgentState;

/// Where an agent's move request has got to (Detour's MoveRequestState).
typedef enum ZrcCrowdTargetState {
  /// No target. The agent stands still.
  ZRC_CROWD_TARGET_NONE = 0,
  /// The last request could not be planned.
  ZRC_CROWD_TARGET_FAILED = 1,
  /// A path to the target is in the corridor and being followed.
  ZRC_CROWD_TARGET_VALID = 2,
  /// A target was submitted and the next update will plan towards it.
  ZRC_CROWD_TARGET_REQUESTING = 3,
  /// The quick local plan fell short; the request is queued for a full search.
  ZRC_CROWD_TARGET_WAITING_FOR_QUEUE = 4,
  /// The queued search is running.
  ZRC_CROWD_TARGET_WAITING_FOR_PATH = 5,
  /// Steering towards a velocity rather than a place, from
  /// zrcCrowdRequestMoveVelocity.
  ZRC_CROWD_TARGET_VELOCITY = 6,
} ZrcCrowdTargetState;

/// Which steering behaviours an agent takes part in (Detour's UpdateFlags).
/// A bitwise or of these goes in ZrcCrowdAgentParams::update_flags.
enum {
  /// Slow down before a corner instead of cutting it.
  ZRC_CROWD_ANTICIPATE_TURNS = 1,
  /// Steer around neighbouring agents rather than through them.
  ZRC_CROWD_OBSTACLE_AVOIDANCE = 2,
  /// Push apart from neighbours that are already too close.
  ZRC_CROWD_SEPARATION = 4,
  /// Shorten the corridor whenever a later corner is directly visible.
  ZRC_CROWD_OPTIMIZE_VIS = 8,
  /// Re-plan the corridor locally as the world changes under it.
  ZRC_CROWD_OPTIMIZE_TOPO = 16,
};

/// How one agent behaves. Entirely a host's to set.
///
/// Every float must be finite, and the bounds below are enforced by
/// zrcCrowdAddAgent and zrcCrowdSetAgentParams rather than asserted. Upstream
/// documents most of them in a comment and checks none of them; several are
/// reachable memory-safety faults rather than merely wrong numbers, and each
/// one says which below.
typedef struct ZrcCrowdAgentParams {
  /// [Limit: > 0, and no larger than the crowd's max_agent_radius]
  ///
  /// A crowd files each agent into its proximity grid as a box of this radius
  /// around its position and floors the result into an int
  /// (DetourProximityGrid.cpp:106-109). A non-finite or enormous radius makes
  /// that conversion undefined on the first update after the agent is added,
  /// before it has moved at all.
  float radius;
  /// [Limit: > 0]
  float height;
  /// [Limit: >= 0] Zero is an agent whose velocity never changes.
  float max_acceleration;
  /// [Limit: > 0]
  ///
  /// Zero rather than positive is refused, which upstream's own comment
  /// permits. An agent that reaches an off-mesh connection at zero maximum
  /// speed gets an infinite traversal budget (DetourCrowd.cpp:1164) and never
  /// finishes crossing it: it stops being steered, for good. An agent that
  /// should not move is one with no target.
  float max_speed;
  /// How far to look for neighbours and walls to steer around. Often a
  /// multiple of the radius. [Limit: > 0]
  ///
  /// Upstream takes its reciprocal when ZRC_CROWD_SEPARATION is set
  /// (DetourCrowd.cpp:1215) and checks nothing, so zero becomes an infinite
  /// separation weight, then an infinite velocity, then a position no longer
  /// on the navmesh — and the next frame floors that into the proximity grid.
  float collision_query_range;
  /// How far ahead ZRC_CROWD_OPTIMIZE_VIS looks for a shortcut. [Limit: > 0]
  float path_optimization_range;
  /// How hard ZRC_CROWD_SEPARATION pushes. [Limit: >= 0]
  float separation_weight;
  /// A bitwise or of the ZRC_CROWD_* update flags above.
  uint8_t update_flags;
  /// Which of the crowd's avoidance configurations to steer with.
  /// [Limit: < ZRC_CROWD_MAX_AVOIDANCE_PARAMS]
  ///
  /// Refused here rather than passed through. Upstream indexes an
  /// eight-element array with this byte and never bounds it
  /// (DetourCrowd.cpp:1293), so any value from 8 up reads past the array every
  /// frame, for every agent carrying it, and hands whatever it found to the
  /// velocity sampler as a configuration.
  uint8_t obstacle_avoidance_type;
  /// Which of the crowd's query filters to plan with.
  /// [Limit: < ZRC_CROWD_MAX_FILTERS]
  ///
  /// Refused here for the same reason, and it is the worse of the two:
  /// upstream indexes a sixteen-element array of dtQueryFilter with this byte
  /// at fourteen separate call sites and bounds it at none of them
  /// (DetourCrowd.cpp:534, 707, 775, 934, 961, 968, 998, 1004, 1018, 1085,
  /// 1088, 1110, 1117, 1397). A dtQueryFilter is 264 bytes, so the largest
  /// value reads roughly 63 kilobytes past the end of the crowd.
  uint8_t query_filter_type;
  /// Whatever a host wants to hang off the agent. Never dereferenced here.
  void* user_data;
} ZrcCrowdAgentParams;

/// One agent, by value. Everything but its corridor and its local boundary,
/// which have accessors of their own.
///
/// Upstream's dtCrowdAgent embeds a dtPathCorridor and a dtLocalBoundary by
/// value, both non-copyable. dtCrowd placement-news each slot rather than
/// assigning it for that reason, and neither member can cross a boundary
/// inside a copied-out struct. zrcCrowdAgentCorridorInfo and
/// zrcCrowdAgentBoundaryCenter read them instead.
typedef struct ZrcCrowdAgent {
  /// What the agent is standing on.
  ZrcCrowdAgentState state;
  /// Where its move request has got to.
  ZrcCrowdTargetState target_state;
  /// Set when the corridor leads somewhere short of the requested target: the
  /// goal was unreachable, or the search ran out of nodes. The agent walks the
  /// best path found. Without this a partial path looks like a complete one.
  ZrcBool partial;
  /// Current position, on the navmesh.
  float position[3];
  /// Velocity actually applied this frame, after acceleration limiting.
  float velocity[3];
  /// Velocity the path wants, before avoidance.
  float desired_velocity[3];
  /// Velocity after avoidance, before acceleration limiting.
  float avoided_velocity[3];
  /// Scratch, not a contract.
  ///
  /// Upstream accumulates the collision-resolution push here across the
  /// iterations of one update and leaves whatever the last iteration wrote.
  /// Reported rather than hidden, because a debug view wants it; nothing else
  /// should read it.
  float displacement[3];
  /// Speed the path wants this frame.
  float desired_speed;
  /// The move request's target polygon, or 0 for none.
  ZrcPolyRef target_ref;
  /// The move request's target position, or the requested velocity when
  /// `target_state` is ZRC_CROWD_TARGET_VELOCITY.
  float target_position[3];
  /// Whether the current path is being re-planned rather than planned fresh.
  ZrcBool target_replan;
  /// Seconds since the target was last re-planned.
  float target_replan_time;
  /// Seconds since the corridor was last topology-optimised.
  float topology_opt_time;
  /// Corners of the local path ahead, readable with zrcCrowdAgentCorners.
  int32_t corner_count;
  /// Neighbours it is steering around, readable with zrcCrowdAgentNeighbours.
  int32_t neighbour_count;
  /// The parameters it was added or last updated with.
  ZrcCrowdAgentParams params;
} ZrcCrowdAgent;

/// One corner of an agent's local path.
typedef struct ZrcCrowdCorner {
  float position[3];
  /// A bitwise or of the ZRC_STRAIGHTPATH_* flags.
  uint8_t flags;
  /// The polygon entered at this corner.
  ZrcPolyRef poly;
} ZrcCrowdCorner;

/// One neighbour an agent is steering around.
typedef struct ZrcCrowdNeighbour {
  /// The neighbour itself. Always live: the crowd fills these during an update
  /// from agents it has just confirmed active.
  ZrcAgentRef agent;
  /// Distance between the two, in world units.
  float distance;
} ZrcCrowdNeighbour;

/// How far an agent is through an off-mesh connection.
///
/// **Nothing here produces one, because nothing upstream does either.** A
/// crowd drives an off-mesh traversal through a private array with no
/// accessor (dtCrowd::m_agentAnims), so a C++ host cannot read a live one and
/// neither can a Zig host. The type crosses because upstream declares it
/// publicly and a host modelling the same data wants the same shape; adding a
/// way to reach one would be a capability this package invented rather than
/// bound. What is observable is ZrcCrowdAgent::state, which reports
/// ZRC_CROWD_AGENT_OFFMESH for the duration.
typedef struct ZrcCrowdAgentAnimation {
  /// Whether a traversal is in progress.
  ZrcBool active;
  /// Where the agent stood when it entered the connection.
  float init_position[3];
  /// The connection's two endpoints, in the order it is crossing them.
  float start_position[3];
  float end_position[3];
  /// The off-mesh connection's polygon.
  ZrcPolyRef poly;
  /// Seconds elapsed, and the total the traversal was budgeted, so `t / t_max`
  /// is the fraction done. `t_max` is derived from the connection's length and
  /// the agent's maximum speed.
  float t;
  float t_max;
} ZrcCrowdAgentAnimation;

/// What one update recorded about one agent, in and out.
///
/// Both halves are optional and independent: naming an agent with no sample
/// buffer still fills the two optimiser points, and a sample buffer with no
/// agent named collects nothing.
typedef struct ZrcCrowdAgentDebug {
  /// In: the agent to record. 0 records nothing.
  ///
  /// A reference, not an index. Upstream's own field is compared against a
  /// position in the list of agents that happened to be active this frame
  /// (DetourCrowd.cpp:1050, 1120), which shifts as agents come and go — so the
  /// value that named one agent last frame names a different one this frame.
  /// This is resolved to that position on the way in.
  ZrcAgentRef agent;
  /// In: where the avoidance sampler's candidate velocities land, or NULL. Its
  /// previous contents are discarded. Borrowed for the call only.
  ZrcAvoidanceDebug* samples;
  /// Out: the segment ZRC_CROWD_OPTIMIZE_VIS tried to shortcut this frame.
  /// Both are left at the previous update's values when the agent did not
  /// optimise, so a viewer should clear them itself between frames if that
  /// matters.
  float opt_start[3];
  float opt_end[3];
} ZrcCrowdAgentDebug;

/// A group of agents steering around each other.
typedef struct ZrcCrowd ZrcCrowd;

/// Creates a crowd of at most `max_agents`, planning against `navmesh`.
///
/// **`navmesh` is borrowed and must outlive the crowd.** The crowd builds two
/// query objects against it — one with a 512-node pool for its per-frame local
/// searches, one with 4096 nodes inside the path queue — and never mutates it.
/// Tiles may come and go underneath: an agent whose polygon is removed drops to
/// ZRC_CROWD_AGENT_INVALID and recovers when one is there again.
///
/// `max_agent_radius` sizes the crowd's proximity grid and its placement
/// search box; an agent added with a larger radius is refused.
/// [Limit: > 0, and no larger than a world coordinate] — upstream multiplies
/// it by three for the grid's cell size and by two for the placement box
/// (DetourCrowd.cpp:389, 395), and a radius near the top of float range
/// overflows both to infinity.
///
/// `max_agents` is [1, ZRC_CROWD_MAX_AGENTS].
ZRC_API ZrcResult zrcCrowdCreate(const ZrcNavMesh* navmesh, int32_t max_agents,
                                 float max_agent_radius, ZrcCrowd** out);

/// Re-initialises a crowd in place, as though it had just been created.
///
/// Every agent is discarded, every avoidance configuration and filter returns
/// to its default, and **every ZrcAgentRef minted before this stops
/// resolving** — including any the caller still holds. The crowd may be
/// pointed at a different navmesh and sized differently.
///
/// Unlike zrcTileCacheCreate's counterpart, upstream's dtCrowd::init is
/// documented as re-callable and purges before it allocates
/// (DetourCrowd.cpp:383), so this is a supported operation rather than a leak.
///
/// A failure leaves the crowd empty and initialised for no agents rather than
/// in its previous state: the purge happens first and cannot be undone.
ZRC_API ZrcResult zrcCrowdInit(ZrcCrowd* crowd, const ZrcNavMesh* navmesh,
                               int32_t max_agents, float max_agent_radius);

ZRC_API void zrcCrowdDestroy(ZrcCrowd* crowd);

//===----------------------------------------------------------------------===//
// Agents
//===----------------------------------------------------------------------===//

/// Adds an agent at the navmesh position nearest `position`.
///
/// The agent is placed on the navmesh, not at the point asked for: the crowd
/// projects `position` into the nearest polygon within its placement box. When
/// nothing is near enough the agent is still added, at the requested point,
/// in state ZRC_CROWD_AGENT_INVALID — it exists and is not steered.
///
/// ZRC_ERR_CROWD_FULL when every slot is taken. A slot still draining from
/// zrcCrowdRemoveAgent counts as taken until its crossing finishes.
ZRC_API ZrcResult zrcCrowdAddAgent(ZrcCrowd* crowd, const float* position,
                                   const ZrcCrowdAgentParams* params,
                                   ZrcAgentRef* out_ref);

/// Removes an agent. `ref` stops resolving immediately.
///
/// Its slot becomes available immediately too, except for an agent part way
/// across an off-mesh connection: that slot is held until the crossing
/// finishes, a fraction of a second later. Upstream's own removeAgent clears
/// the agent's active flag and nothing else, while the traversal lives in a
/// parallel array update() walks by raw pool slot — so the next agent added
/// into that slot resumes the dead one's crossing, its position overwritten by
/// a lerp between coordinates it has never been near (DetourCrowd.cpp:572-578,
/// 1411-1447). Holding the slot is how that is closed without reaching into
/// dtCrowd. See UPSTREAM.md.
ZRC_API ZrcResult zrcCrowdRemoveAgent(ZrcCrowd* crowd, ZrcAgentRef ref);

/// Replaces an agent's parameters. Its position, velocity and target are
/// untouched.
ZRC_API ZrcResult zrcCrowdSetAgentParams(ZrcCrowd* crowd, ZrcAgentRef ref,
                                         const ZrcCrowdAgentParams* params);

/// Everything about one agent except its corridor and its local boundary.
ZRC_API ZrcResult zrcCrowdAgentInfo(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                    ZrcCrowdAgent* out);

/// Agent slots the crowd was created for, live or not.
///
/// Not how many agents there are — that is zrcCrowdActiveAgentCount. Upstream
/// spells both as getAgentCount(), which returns the capacity; a host reading
/// it as a population iterates over free slots.
ZRC_API ZrcResult zrcCrowdAgentCapacity(const ZrcCrowd* crowd,
                                        int32_t* out_count);

/// Agents currently in the crowd.
ZRC_API ZrcResult zrcCrowdActiveAgentCount(const ZrcCrowd* crowd,
                                           int32_t* out_count);

/// Every active agent's reference.
///
/// `*out_count` is how many are active, whether or not they fit;
/// ZRC_ERR_BUFFER_TOO_SMALL when they do not, with the first `max_agents`
/// written. `out` may be NULL when `max_agents` is 0, which asks only for the
/// count.
ZRC_API ZrcResult zrcCrowdActiveAgents(const ZrcCrowd* crowd, ZrcAgentRef* out,
                                       int32_t max_agents, int32_t* out_count);

/// The reference of the agent in slot `index`, or 0 when the slot is free.
///
/// Walking the slots is how a host enumerates a crowd without allocating for
/// zrcCrowdActiveAgents. An index outside [0, zrcCrowdAgentCapacity) is
/// ZRC_ERR_INVALID_ARGUMENT.
ZRC_API ZrcResult zrcCrowdAgentRefAt(const ZrcCrowd* crowd, int32_t index,
                                     ZrcAgentRef* out_ref);

/// Copies `count` corners of the agent's local path from `first`, at most
/// ZRC_CROWD_MAX_CORNERS in all. The last is where it is heading.
ZRC_API ZrcResult zrcCrowdAgentCorners(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                       int32_t first, int32_t count,
                                       ZrcCrowdCorner* out);

/// Copies `count` of the agent's neighbours from `first`, at most
/// ZRC_CROWD_MAX_NEIGHBOURS in all, nearest first.
ZRC_API ZrcResult zrcCrowdAgentNeighbours(const ZrcCrowd* crowd,
                                          ZrcAgentRef ref, int32_t first,
                                          int32_t count,
                                          ZrcCrowdNeighbour* out);

/// The corridor the agent is walking.
///
/// Read-only through a crowd. A C++ host reaches the same object through
/// dtCrowd::getEditableAgent and can drive it directly; doing so desynchronises
/// the agent from the crowd, which recomputes the corridor from the agent's
/// position and target on the next update and would overwrite the edit. The
/// standalone ZrcPathCorridor below is the same object, editable, for a host
/// steering one character itself.
ZRC_API ZrcResult zrcCrowdAgentCorridorInfo(const ZrcCrowd* crowd,
                                            ZrcAgentRef ref,
                                            ZrcPathCorridorInfo* out);

/// Copies `count` polygons of the agent's corridor from `first`, in walking
/// order. The total is ZrcPathCorridorInfo::path_count.
ZRC_API ZrcResult zrcCrowdAgentCorridorPath(const ZrcCrowd* crowd,
                                            ZrcAgentRef ref, int32_t first,
                                            int32_t count, ZrcPolyRef* out);

/// The position the agent's cached walls were last collected around, three
/// floats. See zrcLocalBoundaryCenter for the sentinel an uncollected
/// boundary reports.
ZRC_API ZrcResult zrcCrowdAgentBoundaryCenter(const ZrcCrowd* crowd,
                                              ZrcAgentRef ref, float* out);

/// Walls the agent is steering around, at most eight.
ZRC_API ZrcResult zrcCrowdAgentBoundarySegmentCount(const ZrcCrowd* crowd,
                                                    ZrcAgentRef ref,
                                                    int32_t* out_count);

/// Copies `count` of them from `first`, six floats each: the two endpoints.
/// Nearest first.
ZRC_API ZrcResult zrcCrowdAgentBoundarySegments(const ZrcCrowd* crowd,
                                                ZrcAgentRef ref, int32_t first,
                                                int32_t count, float* out);

//===----------------------------------------------------------------------===//
// Where an agent is going
//===----------------------------------------------------------------------===//

/// Sends the agent to `position` inside polygon `ref`.
///
/// The request is planned during the next zrcCrowdUpdate, so the agent's
/// target state is ZRC_CROWD_TARGET_REQUESTING until then. `ref` must be a
/// polygon of the crowd's navmesh; 0 is ZRC_ERR_INVALID_ARGUMENT rather than
/// a way to clear the target, which is what zrcCrowdResetMoveTarget is for.
ZRC_API ZrcResult zrcCrowdRequestMoveTarget(ZrcCrowd* crowd, ZrcAgentRef ref,
                                            ZrcPolyRef poly,
                                            const float* position);

/// Steers the agent along `velocity` instead of towards a place.
///
/// The velocity is a direction and a speed, still clamped by the agent's
/// max_speed and max_acceleration and still constrained to the navmesh.
ZRC_API ZrcResult zrcCrowdRequestMoveVelocity(ZrcCrowd* crowd, ZrcAgentRef ref,
                                              const float* velocity);

/// Clears the agent's target. It stops where it is.
ZRC_API ZrcResult zrcCrowdResetMoveTarget(ZrcCrowd* crowd, ZrcAgentRef ref);

//===----------------------------------------------------------------------===//
// The frame
//===----------------------------------------------------------------------===//

/// Advances every agent by `dt` seconds.
///
/// `dt` must be finite and greater than zero, and upstream asserts neither.
/// A NaN frame scales the acceleration clamp by NaN, and because every
/// comparison against NaN is false the clamp is skipped and the position it
/// integrates is NaN from then on (DetourCrowd.cpp:58-74). A zero frame does
/// not corrupt anything: it freezes every velocity where it stands, for good.
/// A negative one inverts the clamp and walks agents backwards.
///
/// `debug` may be NULL. When it is not, it is read for what to record and
/// written with what was recorded; see ZrcCrowdAgentDebug. A `debug->agent`
/// that no longer resolves records nothing and the frame still runs, the same
/// as 0: a debug view that outlived its agent must not stop a crowd.
ZRC_API ZrcResult zrcCrowdUpdate(ZrcCrowd* crowd, float dt,
                                 ZrcCrowdAgentDebug* debug);

/// Candidate velocities the avoidance sampler tried during the last update,
/// summed across every agent. A cost measure, not a state.
ZRC_API ZrcResult zrcCrowdVelocitySampleCount(const ZrcCrowd* crowd,
                                              int32_t* out_count);

//===----------------------------------------------------------------------===//
// What the whole crowd shares
//===----------------------------------------------------------------------===//

/// Replaces one of the crowd's avoidance configurations.
/// `index` is [0, ZRC_CROWD_MAX_AVOIDANCE_PARAMS).
ZRC_API ZrcResult zrcCrowdSetAvoidanceParams(
    ZrcCrowd* crowd, int32_t index, const ZrcAvoidanceParams* params);

/// Reads one back. Every index holds zrcAvoidanceParamsDefault until set.
ZRC_API ZrcResult zrcCrowdAvoidanceParams(const ZrcCrowd* crowd, int32_t index,
                                          ZrcAvoidanceParams* out);

/// Replaces one of the crowd's query filters.
/// `index` is [0, ZRC_CROWD_MAX_FILTERS).
ZRC_API ZrcResult zrcCrowdSetFilter(ZrcCrowd* crowd, int32_t index,
                                    const ZrcQueryFilter* filter);

/// Reads one back. Every index holds zrcQueryFilterDefault until set.
ZRC_API ZrcResult zrcCrowdFilter(const ZrcCrowd* crowd, int32_t index,
                                 ZrcQueryFilter* out);

/// The half extents of the box the crowd searches when it places or recovers
/// an agent, three floats. Derived from `max_agent_radius`.
ZRC_API ZrcResult zrcCrowdQueryHalfExtents(const ZrcCrowd* crowd, float* out);

/// The crowd's proximity grid, borrowed. The caller must not destroy it, and
/// it dies with the crowd or at the next zrcCrowdInit.
///
/// Rebuilt from scratch at the start of every zrcCrowdUpdate, so what it holds
/// between frames is the last frame's agent positions.
ZRC_API ZrcResult zrcCrowdGrid(const ZrcCrowd* crowd,
                               const ZrcProximityGrid** out);

/// The crowd's path request queue, borrowed. Same lifetime rules.
ZRC_API ZrcResult zrcCrowdPathQueue(const ZrcCrowd* crowd,
                                    const ZrcPathQueue** out);

/// The crowd's own query object, borrowed. Same lifetime rules.
///
/// Read-only on purpose: it is the query the crowd runs its per-frame searches
/// on, and starting a sliced search on it would take the node pool out from
/// under them. A host that wants one makes its own with zrcNavMeshQueryCreate.
ZRC_API ZrcResult zrcCrowdNavMeshQuery(const ZrcCrowd* crowd,
                                       const ZrcNavMeshQuery** out);

//===----------------------------------------------------------------------===//
// The proximity grid
//
// A spatial hash over the xz-plane: put an item's footprint in, ask what is
// near a box, get the ids back. A crowd keeps one and rebuilds it every update
// so each agent can find its neighbours without testing every other agent.
//
// Usable on its own for anything else that wants the same answer, and upstream
// gives it an allocation pair of its own on those terms.
//===----------------------------------------------------------------------===//

/// Creates a grid sized for `pool_size` cell entries of `cell_size` each.
///
/// One item spans as many entries as its footprint covers cells, so a pool
/// holds fewer items than its size when they are large. A crowd asks for four
/// entries per agent.
///
/// `pool_size` is [1, 65534]. `cell_size` must be finite, greater than zero,
/// and large enough that its reciprocal is finite — a subnormal one makes
/// every cell coordinate infinite or NaN. Both are asserted upstream and asserts compile away
/// (DetourProximityGrid.cpp:64-65): a pool size of zero rounds to zero hash
/// buckets and every bucket index computed after that is out of bounds, and a
/// cell size of zero makes the reciprocal infinite so every coordinate
/// collapses to one cell. The upper bound is the 0xffff end-of-chain marker
/// described at ZRC_CROWD_MAX_AGENTS.
ZRC_API ZrcResult zrcProximityGridCreate(int32_t pool_size, float cell_size,
                                         ZrcProximityGrid** out);

/// Destroying the grid a crowd handed back does nothing: it belongs to the
/// crowd. zrcCrowdGrid returns it const, so reaching this at all takes a cast.
ZRC_API void zrcProximityGridDestroy(ZrcProximityGrid* grid);

/// Empties the grid. The pool and the cell size stay as they were.
ZRC_API ZrcResult zrcProximityGridClear(ZrcProximityGrid* grid);

/// Files `id` under every cell the box [min_x, min_y] - [max_x, max_y] covers.
///
/// The two axes are the world's x and z; upstream calls them x and y because
/// the grid is two-dimensional, and this keeps its spelling.
///
/// The four bounds must be finite, min <= max on both axes, and the box must
/// cover at most ZRC_PROXIMITY_GRID_MAX_SPAN cells on each axis. Upstream
/// floors each bound into an int and loops between them
/// (DetourProximityGrid.cpp:104-112): a non-finite bound makes that conversion
/// undefined, and a box spanning billions of cells is a loop that does not
/// end. A cell coordinate outside [-32768, 32767] is refused too — upstream
/// stores it in a `short` and compares the truncated copy against the full
/// value when reading back, so items outside that range are filed and then
/// never found.
///
/// **Silently drops items once the pool is full**, which is upstream's own
/// behaviour and is why the pool size is a capacity question rather than an
/// error one.
ZRC_API ZrcResult zrcProximityGridAddItem(ZrcProximityGrid* grid, uint16_t id,
                                          float min_x, float min_y,
                                          float max_x, float max_y);

/// Every distinct id filed under any cell the box covers.
///
/// The same bounds rules as zrcProximityGridAddItem.
///
/// `*out_count` is how many were written. Upstream stops at `max_ids` and
/// returns that many with no signal, so a full buffer here is
/// ZRC_ERR_BUFFER_TOO_SMALL with the ids that fit already written: a caller
/// cannot mistake a clipped neighbourhood for the whole one.
ZRC_API ZrcResult zrcProximityGridQueryItems(const ZrcProximityGrid* grid,
                                             float min_x, float min_y,
                                             float max_x, float max_y,
                                             uint16_t* out_ids,
                                             int32_t max_ids,
                                             int32_t* out_count);

/// How many entries are filed at exactly cell (x, y). Counts an id once per
/// entry, so an id filed twice at one cell counts twice.
ZRC_API ZrcResult zrcProximityGridItemCountAt(const ZrcProximityGrid* grid,
                                              int32_t x, int32_t y,
                                              int32_t* out_count);

/// The cell bounds of everything added since the last clear, four ints:
/// min x, min y, max x, max y.
///
/// An empty grid reports upstream's own inverted sentinel — 0xffff, 0xffff,
/// -0xffff, -0xffff — rather than a zero box, so `min > max` is how a caller
/// tells empty from a single cell at the origin.
ZRC_API ZrcResult zrcProximityGridBounds(const ZrcProximityGrid* grid,
                                         int32_t* out);

/// The cell size the grid was created with.
ZRC_API ZrcResult zrcProximityGridCellSize(const ZrcProximityGrid* grid,
                                           float* out);

//===----------------------------------------------------------------------===//
// Obstacle avoidance
//
// Given where an agent is, how fast it wants to go, and what is moving nearby,
// pick the velocity that gets closest to what it wanted without running into
// anything. This is the part of a crowd's steering that considers other
// agents' velocities rather than only their positions.
//
// A crowd drives this itself, once per agent per frame, using the
// configuration at that agent's obstacle_avoidance_type. Everything here is
// also callable directly, which is what a host steering one character outside
// a crowd wants.
//===----------------------------------------------------------------------===//

/// A moving circular obstacle the sampler is avoiding.
typedef struct ZrcAvoidanceCircle {
  float position[3];
  /// Its actual velocity.
  float velocity[3];
  /// The velocity it wants, which the sampler weighs differently.
  float desired_velocity[3];
  float radius;
} ZrcAvoidanceCircle;

/// A static segment the sampler is avoiding — a wall.
typedef struct ZrcAvoidanceSegment {
  float p[3];
  float q[3];
  /// Whether the agent is already close enough to be touching it.
  ZrcBool touching;
} ZrcAvoidanceSegment;

/// One candidate velocity the sampler tried, and why it scored as it did.
typedef struct ZrcAvoidanceSample {
  float velocity[3];
  /// Side length of the sampling cell this candidate came from.
  float size;
  /// The total, which is the sum of the four below.
  float penalty;
  float desired_velocity_penalty;
  float current_velocity_penalty;
  float preferred_side_penalty;
  float collision_time_penalty;
} ZrcAvoidanceSample;

/// Creates a sampler with room for `max_circles` moving obstacles and
/// `max_segments` walls. Both are [1, 65535]. A crowd asks for 6 and 8.
ZRC_API ZrcResult zrcAvoidanceQueryCreate(int32_t max_circles,
                                          int32_t max_segments,
                                          ZrcAvoidanceQuery** out);

ZRC_API void zrcAvoidanceQueryDestroy(ZrcAvoidanceQuery* query);

/// Forgets every obstacle. The capacities stay as they were.
ZRC_API ZrcResult zrcAvoidanceQueryReset(ZrcAvoidanceQuery* query);

/// Adds a moving circular obstacle.
///
/// **Silently ignored once `max_circles` are added**, which is upstream's
/// behaviour (DetourObstacleAvoidance.cpp:214-215) and is reported here as
/// ZRC_ERR_BUFFER_TOO_SMALL instead: an obstacle the sampler never saw is an
/// agent that walks through it.
ZRC_API ZrcResult zrcAvoidanceAddCircle(ZrcAvoidanceQuery* query,
                                        const float* position, float radius,
                                        const float* velocity,
                                        const float* desired_velocity);

/// Adds a wall segment. Same capacity rule as zrcAvoidanceAddCircle.
ZRC_API ZrcResult zrcAvoidanceAddSegment(ZrcAvoidanceQuery* query,
                                         const float* p, const float* q);

/// Picks a velocity by scoring a regular grid of candidates.
///
/// `out_velocity` receives three floats. `*out_samples` is how many candidates
/// were scored; it may be NULL. `debug` may be NULL; when given, every
/// candidate is recorded into it and whatever it held is discarded.
///
/// `params->grid_size` must be at least 2. Upstream divides by
/// `grid_size - 1` (DetourObstacleAvoidance.cpp:466) and clamps nothing, so 1
/// is a division by zero and 0 scores no candidate at all while still
/// returning success.
ZRC_API ZrcResult zrcAvoidanceSampleGrid(
    ZrcAvoidanceQuery* query, const float* position, float radius,
    float max_speed, const float* velocity, const float* desired_velocity,
    const ZrcAvoidanceParams* params, ZrcAvoidanceDebug* debug,
    float* out_velocity, int32_t* out_samples);

/// Picks a velocity by refining a ring pattern, which is what a crowd uses.
///
/// The same arguments and the same output. `params->adaptive_divs`,
/// `adaptive_rings` and `adaptive_depth` steer the refinement; upstream clamps
/// the first two to their ZRC_AVOIDANCE_MAX_PATTERN_* limits and clamps the
/// depth to nothing, so a depth of 0 returns the agent's current velocity
/// unchanged rather than avoiding anything. Refused here.
///
/// This is the entry point that reaches cosf and sinf, so it is the reason
/// steering is outside the package's cross-platform guarantee. See the
/// section banner above.
ZRC_API ZrcResult zrcAvoidanceSampleAdaptive(
    ZrcAvoidanceQuery* query, const float* position, float radius,
    float max_speed, const float* velocity, const float* desired_velocity,
    const ZrcAvoidanceParams* params, ZrcAvoidanceDebug* debug,
    float* out_velocity, int32_t* out_samples);

/// Moving obstacles currently added.
ZRC_API ZrcResult zrcAvoidanceCircleCount(const ZrcAvoidanceQuery* query,
                                          int32_t* out_count);

/// One of them. The index is bounded here; upstream's accessor bounds nothing.
ZRC_API ZrcResult zrcAvoidanceCircleAt(const ZrcAvoidanceQuery* query,
                                       int32_t index,
                                       ZrcAvoidanceCircle* out);

/// Wall segments currently added.
ZRC_API ZrcResult zrcAvoidanceSegmentCount(const ZrcAvoidanceQuery* query,
                                           int32_t* out_count);

/// One of them. Same bounding.
ZRC_API ZrcResult zrcAvoidanceSegmentAt(const ZrcAvoidanceQuery* query,
                                        int32_t index,
                                        ZrcAvoidanceSegment* out);

/// Creates a recorder with room for `max_samples` candidate velocities.
///
/// [Limit: 1 to 65535], the same ceiling zrcAvoidanceQueryCreate carries and
/// for the same reason: upstream stores the count unclamped and multiplies it
/// by a struct size for the allocation, which on a 32-bit size_t wraps to a
/// small buffer while the count stays large.
ZRC_API ZrcResult zrcAvoidanceDebugCreate(int32_t max_samples,
                                          ZrcAvoidanceDebug** out);

ZRC_API void zrcAvoidanceDebugDestroy(ZrcAvoidanceDebug* debug);

/// Forgets every sample. The capacity stays as it was.
ZRC_API ZrcResult zrcAvoidanceDebugReset(ZrcAvoidanceDebug* debug);

/// Records one candidate by hand, for a host scoring velocities itself.
/// Silently ignored once full upstream; ZRC_ERR_BUFFER_TOO_SMALL here.
ZRC_API ZrcResult zrcAvoidanceDebugAddSample(ZrcAvoidanceDebug* debug,
                                             const ZrcAvoidanceSample* sample);

/// Rescales every recorded penalty into [0, 1] against the largest, so a
/// viewer can colour them. Idempotent; a recorder with no samples is a no-op.
ZRC_API ZrcResult zrcAvoidanceDebugNormalize(ZrcAvoidanceDebug* debug);

/// Samples currently recorded.
ZRC_API ZrcResult zrcAvoidanceDebugSampleCount(const ZrcAvoidanceDebug* debug,
                                               int32_t* out_count);

/// One of them. The index is bounded here; all seven of upstream's accessors
/// index their arrays unchecked (DetourObstacleAvoidance.h:56-62).
ZRC_API ZrcResult zrcAvoidanceDebugSampleAt(const ZrcAvoidanceDebug* debug,
                                            int32_t index,
                                            ZrcAvoidanceSample* out);

//===----------------------------------------------------------------------===//
// The path corridor
//
// The polygons an agent is walking through, and the position and target inside
// them. Every crowd agent has one; a host steering a single character can use
// one on its own, which is what upstream intends by giving it its own header.
//
// What makes it a corridor rather than a path: it is edited as the agent
// moves. zrcPathCorridorMovePosition slides the position along and drops the
// polygons behind it, the optimisers shorten what is ahead, and
// zrcPathCorridorTrimInvalid cuts it back when the world changes underneath.
//===----------------------------------------------------------------------===//

/// The shortest corridor upstream can be driven safely.
///
/// Not a preference. Three of upstream's own operations write past the end of
/// a shorter buffer, in every build configuration:
///
///   - fixPathStart writes m_path[2] and sets the length to 3 with no bound
///     of any kind, not even an assert (DetourPathCorridor.cpp:529-532).
///   - movePosition merges up to 16 newly visited polygons, and
///     optimizePathVisibility and optimizePathTopology up to 32. Each merge
///     computes `size = maxPath - req` in a signed int and passes it to
///     memmove as a length (DetourPathCorridor.cpp:60-63, 146-149). When the
///     corridor is shorter than the merge, that subtraction goes negative and
///     the conversion to size_t makes it enormous.
///
/// 32 is the largest of those three, so a corridor at least this long cannot
/// reach any of them. See UPSTREAM.md.
#define ZRC_PATH_CORRIDOR_MIN_PATH 32

/// Creates a corridor that can hold `max_path` polygons.
///
/// [Limit: ZRC_PATH_CORRIDOR_MIN_PATH to 65535]. A crowd gives each of its
/// agents 256, which is also the practical ceiling on how far ahead a crowd
/// plans.
ZRC_API ZrcResult zrcPathCorridorCreate(int32_t max_path,
                                        ZrcPathCorridor** out);

ZRC_API void zrcPathCorridorDestroy(ZrcPathCorridor* corridor);

/// Empties the corridor and puts it at `position` inside polygon `poly`.
///
/// The corridor then holds that one polygon with the target equal to the
/// position. `poly` may be 0, which is how a corridor for an agent that is not
/// on the navmesh is spelled.
ZRC_API ZrcResult zrcPathCorridorReset(ZrcPathCorridor* corridor,
                                       ZrcPolyRef poly, const float* position);

/// Replaces the corridor with `path` and its target with `target`.
///
/// `path_count` must be in [1, max_path]. Upstream copies whatever it is
/// given, bounded only by an assert that compiles away
/// (DetourPathCorridor.cpp:515): a longer path overruns the corridor's buffer,
/// and a negative one becomes an enormous unsigned length passed to memcpy.
/// An empty corridor is refused too, because moveTargetPosition then reads
/// m_path[-1] (DetourPathCorridor.cpp:487).
ZRC_API ZrcResult zrcPathCorridorSetCorridor(ZrcPathCorridor* corridor,
                                             const float* target,
                                             const ZrcPolyRef* path,
                                             int32_t path_count);

/// String-pulls the corridor ahead into the corners to walk.
///
/// **At most `max_corners - 1` corners are produced**, which is upstream's
/// own behaviour rather than a rounding error: the string-pull reserves the
/// last slot for the corridor's end point and then drops the first corner when
/// the agent is already standing on it (DetourPathCorridor.cpp:265-277). Ask
/// for one more than needed. `max_corners` must be at least 2.
///
/// `*out_count` is how many corners were produced, and **no truncation is
/// reported**, which is the one place in this package a buffer that filled up
/// is not distinguished from one that was long enough. Upstream discards the
/// status of the string-pull underneath (DetourPathCorridor.cpp:261-262), so
/// the fact needed to tell the two apart is gone before this boundary can see
/// it. Ask with a buffer larger than the corners wanted, or read
/// ZrcPathCorridorInfo::path_count first.
ZRC_API ZrcResult zrcPathCorridorFindCorners(
    ZrcPathCorridor* corridor, ZrcNavMeshQuery* query,
    const ZrcQueryFilter* filter, ZrcCrowdCorner* out, int32_t max_corners,
    int32_t* out_count);

/// Shortens the corridor if `next` is directly visible from the position.
///
/// `range` is how far ahead to look and must be finite and greater than zero.
/// The corridor is left as it was when nothing could be shortened.
ZRC_API ZrcResult zrcPathCorridorOptimizeVisibility(
    ZrcPathCorridor* corridor, const float* next, float range,
    ZrcNavMeshQuery* query, const ZrcQueryFilter* filter);

/// Re-plans the corridor locally, following the world rather than the
/// straight line. `*out_optimized` says whether anything changed; may be NULL.
ZRC_API ZrcResult zrcPathCorridorOptimizeTopology(
    ZrcPathCorridor* corridor, ZrcNavMeshQuery* query,
    const ZrcQueryFilter* filter, ZrcBool* out_optimized);

/// Advances the corridor across the off-mesh connection at its front.
///
/// `out_refs` receives two polygons — the one left and the one entered — and
/// `out_start` and `out_end` three floats each. `*out_moved` is ZRC_FALSE when
/// the corridor was not at that connection, which is not an error.
///
/// One case upstream gets wrong and this cannot repair from outside: when the
/// connection is already the corridor's *first* polygon, its search loop exits
/// before advancing and the prune that follows removes nothing
/// (DetourPathCorridor.cpp:389-406), so the position moves to the far side
/// while the corridor still starts at the connection. Reported as moved,
/// because it did move. See UPSTREAM.md.
ZRC_API ZrcResult zrcPathCorridorMoveOverOffmeshConnection(
    ZrcPathCorridor* corridor, ZrcPolyRef offmesh_poly, ZrcNavMeshQuery* query,
    ZrcPolyRef* out_refs, float* out_start, float* out_end,
    ZrcBool* out_moved);

/// Puts `safe_poly` at the front of the corridor when the first polygon has
/// gone. `*out_fixed` may be NULL.
ZRC_API ZrcResult zrcPathCorridorFixStart(ZrcPathCorridor* corridor,
                                          ZrcPolyRef safe_poly,
                                          const float* safe_position,
                                          ZrcBool* out_fixed);

/// Cuts the corridor back to the last polygon that is still valid, falling
/// back to `safe_poly` when none is. `*out_trimmed` may be NULL.
ZRC_API ZrcResult zrcPathCorridorTrimInvalid(ZrcPathCorridor* corridor,
                                             ZrcPolyRef safe_poly,
                                             const float* safe_position,
                                             ZrcNavMeshQuery* query,
                                             const ZrcQueryFilter* filter,
                                             ZrcBool* out_trimmed);

/// Whether the first `max_lookahead` polygons of the corridor still exist and
/// still pass the filter. `max_lookahead` is [0, path_count]; upstream clamps
/// it to the corridor's length and answers "valid" for a negative one, which
/// is a question nothing was asked.
ZRC_API ZrcResult zrcPathCorridorIsValid(ZrcPathCorridor* corridor,
                                         int32_t max_lookahead,
                                         ZrcNavMeshQuery* query,
                                         const ZrcQueryFilter* filter,
                                         ZrcBool* out_valid);

/// Slides the position to `position`, dropping the polygons left behind.
///
/// The position ends up on the navmesh, which is not necessarily where it was
/// asked to go. `*out_moved` may be NULL.
ZRC_API ZrcResult zrcPathCorridorMovePosition(ZrcPathCorridor* corridor,
                                              const float* position,
                                              ZrcNavMeshQuery* query,
                                              const ZrcQueryFilter* filter,
                                              ZrcBool* out_moved);

/// The same for the target end of the corridor.
ZRC_API ZrcResult zrcPathCorridorMoveTargetPosition(
    ZrcPathCorridor* corridor, const float* position, ZrcNavMeshQuery* query,
    const ZrcQueryFilter* filter, ZrcBool* out_moved);

/// Where the corridor is, where it ends, and how long it is.
ZRC_API ZrcResult zrcPathCorridorInfo(const ZrcPathCorridor* corridor,
                                      ZrcPathCorridorInfo* out);

/// Copies `count` polygons of the corridor from `first`, in walking order.
ZRC_API ZrcResult zrcPathCorridorPath(const ZrcPathCorridor* corridor,
                                      int32_t first, int32_t count,
                                      ZrcPolyRef* out);

/// The three corridor-splicing primitives the corridor itself is built from,
/// callable on a host's own polygon array.
///
/// Each edits `path` in place and reports the new length in `*out_count`.
/// `path_count` is the current length, `max_path` the array's capacity, and
/// `visited` the polygons crossed since the last edit, in the order they were
/// crossed. `path_count` and `visited_count` must both be in [0, max_path].
///
/// **`visited_count` above `max_path` is what corrupts memory upstream.** Two
/// of the three compute a merge width, subtract it from the capacity in a
/// signed int, and hand the difference to memmove as a length
/// (DetourPathCorridor.cpp:60-63, 146-149). A width wider than the capacity
/// makes that difference negative, and memmove reads it as roughly 2^64 bytes
/// from a pointer already past the end of the array. The third asserts the
/// bound and the assert compiles away (DetourPathCorridor.cpp:103).
///
/// A merge whose result would not fit is truncated by upstream with no signal;
/// `*out_count` is the length that resulted either way.
///
/// StartMoved is for a position that slid forwards, EndMoved for a target that
/// did, StartShortcut for a visibility optimisation that skipped ahead.
ZRC_API ZrcResult zrcMergeCorridorStartMoved(ZrcPolyRef* path,
                                             int32_t path_count,
                                             int32_t max_path,
                                             const ZrcPolyRef* visited,
                                             int32_t visited_count,
                                             int32_t* out_count);

ZRC_API ZrcResult zrcMergeCorridorEndMoved(ZrcPolyRef* path,
                                           int32_t path_count,
                                           int32_t max_path,
                                           const ZrcPolyRef* visited,
                                           int32_t visited_count,
                                           int32_t* out_count);

ZRC_API ZrcResult zrcMergeCorridorStartShortcut(ZrcPolyRef* path,
                                                int32_t path_count,
                                                int32_t max_path,
                                                const ZrcPolyRef* visited,
                                                int32_t visited_count,
                                                int32_t* out_count);

//===----------------------------------------------------------------------===//
// The local boundary
//
// The walls near an agent, cached. Sampling them every frame would mean a
// navmesh query per agent per frame; a boundary collects them once and stays
// usable until the agent has moved far enough to need new ones.
//===----------------------------------------------------------------------===//

ZRC_API ZrcResult zrcLocalBoundaryCreate(ZrcLocalBoundary** out);

ZRC_API void zrcLocalBoundaryDestroy(ZrcLocalBoundary* boundary);

/// Forgets every segment and every polygon it was collected from.
ZRC_API ZrcResult zrcLocalBoundaryReset(ZrcLocalBoundary* boundary);

/// Re-collects the walls within `range` of `position`, starting from `poly`.
///
/// Keeps the eight nearest, which is upstream's fixed capacity. `poly` may be
/// 0, which resets the boundary instead of collecting anything. `range` must
/// be finite and greater than zero.
ZRC_API ZrcResult zrcLocalBoundaryUpdate(ZrcLocalBoundary* boundary,
                                         ZrcPolyRef poly,
                                         const float* position, float range,
                                         ZrcNavMeshQuery* query,
                                         const ZrcQueryFilter* filter);

/// Whether every polygon the boundary was collected from still exists and
/// still passes the filter. False means it should be updated again.
ZRC_API ZrcResult zrcLocalBoundaryIsValid(ZrcLocalBoundary* boundary,
                                          ZrcNavMeshQuery* query,
                                          const ZrcQueryFilter* filter,
                                          ZrcBool* out_valid);

/// The position the boundary was last collected around, three floats.
ZRC_API ZrcResult zrcLocalBoundaryCenter(const ZrcLocalBoundary* boundary,
                                         float* out);

/// Segments currently held, at most eight.
ZRC_API ZrcResult zrcLocalBoundarySegmentCount(
    const ZrcLocalBoundary* boundary, int32_t* out_count);

/// Copies `count` segments from `first`, six floats each: the two endpoints.
/// Nearest first.
ZRC_API ZrcResult zrcLocalBoundarySegments(const ZrcLocalBoundary* boundary,
                                           int32_t first, int32_t count,
                                           float* out);

//===----------------------------------------------------------------------===//
// The path queue
//
// Long searches, run a slice at a time across frames. A crowd puts an agent's
// request here when the quick local plan falls short of the target, and reads
// the result back whenever it is ready — which may be several frames later.
//
// This is the same sliced search zrcSlicedFindPathInit drives directly, with
// the bookkeeping for several outstanding requests on top. Eight at a time,
// which is upstream's fixed capacity.
//===----------------------------------------------------------------------===//

/// A submitted search. 0 is never a live request.
typedef uint32_t ZrcPathRequestRef;

/// What zrcPathQueueRequest returns when the queue is full.
#define ZRC_PATH_REQUEST_NONE 0

/// Where one request has got to.
typedef enum ZrcPathRequestState {
  /// No such request: never submitted, or its slot has been recycled.
  ZRC_PATH_REQUEST_UNKNOWN = 0,
  /// Submitted, not finished.
  ZRC_PATH_REQUEST_RUNNING = 1,
  /// Finished; zrcPathQueueResult will hand back the path.
  ZRC_PATH_REQUEST_READY = 2,
  /// Finished without a path.
  ZRC_PATH_REQUEST_FAILED = 3,
} ZrcPathRequestState;

/// Creates a queue whose searches run against `navmesh`.
///
/// `max_path_size` is the longest path one request may return and
/// `max_search_nodes` the pool each search gets; a crowd uses 256 and 4096.
/// `max_path_size` is [1, 65535] and `max_search_nodes` is [4, 65535] — the
/// same floor zrcNavMeshQueryCreate carries, because the queue forwards it to
/// the same dtNavMeshQuery::init, which sizes its hash table as
/// dtNextPow2(nodes / 4) and lands every bucket index outside the allocation
/// for anything smaller. **`navmesh` is borrowed and must outlive the queue.**
ZRC_API ZrcResult zrcPathQueueCreate(const ZrcNavMesh* navmesh,
                                     int32_t max_path_size,
                                     int32_t max_search_nodes,
                                     ZrcPathQueue** out);

ZRC_API void zrcPathQueueDestroy(ZrcPathQueue* queue);

/// Advances the request at the front of the queue by at most `max_iters`
/// search iterations. [Limit: >= 0]
///
/// One call advances one request, so a queue with several outstanding is
/// drained over several calls. A crowd makes one per update.
///
/// Two of upstream's behaviours are worth planning around. A request that does
/// not finish within one call's budget is not stepped past, so it holds the
/// front of the queue and the other seven wait behind it
/// (DetourPathQueue.cpp:128-131). And a finished result is discarded two calls
/// after it became ready, whether or not anything collected it
/// (DetourPathQueue.cpp:100-105): a host that does not read its results within
/// two updates loses them, and the reference stops resolving.
ZRC_API ZrcResult zrcPathQueueUpdate(ZrcPathQueue* queue, int32_t max_iters);

/// Submits a search from `start_poly` to `end_poly`.
///
/// `*out_ref` is ZRC_PATH_REQUEST_NONE when every slot is taken, which is not
/// an error: it is how a host learns to try again next frame.
///
/// `filter` is copied. Upstream keeps the caller's pointer and reads through
/// it on every later update — its own header calls that "potentially
/// dangerous" (DetourPathQueue.h:47) — so a filter that goes out of scope
/// before the search finishes is a dangling read there and is not one here. A
/// host that mutates its filter mid-search does not see the change, which is
/// the same choice zrcSlicedFindPathInit makes.
ZRC_API ZrcResult zrcPathQueueRequest(ZrcPathQueue* queue,
                                      ZrcPolyRef start_poly,
                                      ZrcPolyRef end_poly,
                                      const float* start_position,
                                      const float* end_position,
                                      const ZrcQueryFilter* filter,
                                      ZrcPathRequestRef* out_ref);

/// Where a request has got to. An unknown reference is
/// ZRC_PATH_REQUEST_UNKNOWN rather than an error.
///
/// One upstream failure cannot be told from an unknown reference and is
/// reported as ZRC_PATH_REQUEST_UNKNOWN: a search whose start or end polygon
/// is invalidated while it is queued sets a bare DT_FAILURE
/// (DetourNavMeshQuery.cpp:1288, 1324), which is the same value
/// dtPathQueue::getRequestStatus returns for a reference no slot holds
/// (DetourPathQueue.cpp:178). A C++ host cannot separate them either. Every
/// other failure carries a detail bit and is reported as
/// ZRC_PATH_REQUEST_FAILED.
ZRC_API ZrcResult zrcPathQueueRequestStatus(const ZrcPathQueue* queue,
                                            ZrcPathRequestRef ref,
                                            ZrcPathRequestState* out_state);

/// Takes a finished request's path and frees its slot.
///
/// ZRC_ERR_SEARCH_IN_PROGRESS when the search has not finished, and
/// ZRC_ERR_NOT_FOUND for a reference no slot holds. Upstream checks neither:
/// it frees the slot and reports success with whatever length the slot
/// happened to hold, which for a running search is an empty path
/// indistinguishable from a genuine one (DetourPathQueue.cpp:190-196). The
/// request survives the refusal here and can be collected when it is ready.
///
/// `*out_count` is the path's length; ZRC_ERR_BUFFER_TOO_SMALL when `max_path`
/// could not hold it, with the polygons that fit already written. Upstream
/// clamps and reports success either way, so a caller cannot otherwise tell a
/// whole path from its first half. The slot is freed whichever way *that*
/// ends, so size the buffer at `max_path_size` rather than retrying.
ZRC_API ZrcResult zrcPathQueueResult(ZrcPathQueue* queue,
                                     ZrcPathRequestRef ref, ZrcPolyRef* out,
                                     int32_t max_path, int32_t* out_count);

/// The query object the queue runs its searches on, borrowed. The caller must
/// not destroy it, and it dies with the queue.
ZRC_API ZrcResult zrcPathQueueNavMeshQuery(const ZrcPathQueue* queue,
                                           const ZrcNavMeshQuery** out);

//===----------------------------------------------------------------------===//
// ABI layout guard
//
// The Zig wrapper hand-declares `extern struct`s mirroring the POD types above.
// Nothing in either compiler checks that those two declarations agree — a field
// reordered here and not there is silent memory corruption, not a build error.
// zrcAbiLayout reports what the C++ side actually compiled to so the other side
// can assert against it in a test.
//
// Its counterpart lives in ffi/zrecast_abi.cpp: static_asserts that fail the
// BUILD if a re-vendored Recast or Detour changes a type this package casts to,
// reads from a serialised image, or sizes a buffer against.
//===----------------------------------------------------------------------===//

/// Capacity of every `*_offsets` array below (ZrcAbiLayout::bake_config_offsets,
/// ::tile_info_offsets, ::poly_info_offsets). Larger than any of them needs, so
/// adding a field does not change this ABI — only the reported count.
#define ZRC_ABI_MAX_FIELDS 32

typedef struct ZrcAbiLayout {
  /// sizeof(ZrcAbiLayout). Read this first: if it disagrees with the consumer's
  /// own sizeof, the struct itself has changed and no field below can be
  /// trusted.
  uint32_t layout_size;

  uint32_t trimesh_size;
  uint32_t trimesh_align;
  uint32_t trimesh_offset_verts;
  uint32_t trimesh_offset_vert_count;
  uint32_t trimesh_offset_tris;
  uint32_t trimesh_offset_tri_count;

  uint32_t bake_config_size;
  uint32_t bake_config_align;
  /// Number of fields in ZrcBakeConfig, and the offset of every one of them in
  /// declaration order.
  ///
  /// A handful of spot-checked offsets is not a guard: ZrcBakeConfig is ten
  /// consecutive floats followed by seven more 4-byte fields, so swapping any
  /// two of the floats between this header and the Zig externs leaves the size,
  /// the alignment and any sampled offset identical — and every bake then
  /// silently uses the wrong agent dimensions. Reporting all of them, plus the
  /// count, is what makes a reorder or an added field fail the test.
  uint32_t bake_config_field_count;
  uint32_t bake_config_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t bake_log_size;
  uint32_t bake_log_align;
  uint32_t bake_log_offset_buffer;
  uint32_t bake_log_offset_capacity;

  uint32_t query_filter_size;
  uint32_t query_filter_align;
  uint32_t query_filter_offset_area_cost;
  uint32_t query_filter_offset_include_flags;
  uint32_t query_filter_offset_exclude_flags;

  uint32_t raycast_hit_size;
  uint32_t raycast_hit_align;
  uint32_t raycast_hit_offset_t;
  uint32_t raycast_hit_offset_position;
  uint32_t raycast_hit_offset_normal;
  uint32_t raycast_hit_offset_hit;

  uint32_t allocator_size;
  uint32_t allocator_align;
  uint32_t allocator_offset_allocate;
  uint32_t allocator_offset_deallocate;
  uint32_t allocator_offset_user;

  /// ZrcTileGrid is two float triples followed by a float and two ints, so its
  /// size and alignment stay the same if the two triples change places. Every
  /// offset is reported for the same reason ZrcBakeConfig's are.
  uint32_t tile_grid_size;
  uint32_t tile_grid_align;
  uint32_t tile_grid_offset_origin;
  uint32_t tile_grid_offset_extent_max;
  uint32_t tile_grid_offset_tile_world_size;
  uint32_t tile_grid_offset_tile_count_x;
  uint32_t tile_grid_offset_tile_count_z;

  /// ZrcAreaVolume is a run of same-sized scalars around one pointer, so its
  /// size and alignment survive almost any reorder: y_min and y_max changing
  /// places, or xz_min and xz_max, would leave both identical and silently
  /// invert every volume a host authors. Every offset is reported for that
  /// reason, as ZrcBakeConfig's and ZrcTileGrid's are.
  uint32_t area_volume_size;
  uint32_t area_volume_align;
  uint32_t area_volume_offset_shape;
  uint32_t area_volume_offset_area;
  uint32_t area_volume_offset_y_min;
  uint32_t area_volume_offset_y_max;
  uint32_t area_volume_offset_verts;
  uint32_t area_volume_offset_vert_count;
  uint32_t area_volume_offset_xz_min;
  uint32_t area_volume_offset_xz_max;
  uint32_t area_volume_offset_radius;

  uint32_t area_authoring_size;
  uint32_t area_authoring_align;
  uint32_t area_authoring_offset_volumes;
  uint32_t area_authoring_offset_volume_count;
  uint32_t area_authoring_offset_area_flags;

  /// ZrcOffMeshConnection mixes two float triples with scalars of three
  /// different sizes (a float, an int32, a uint16, a uint32, a ZrcBool), so a
  /// reorder among fields of matching size would not show up in the size or
  /// the alignment alone. Every offset is reported for that reason, as
  /// ZrcAreaVolume's are.
  uint32_t off_mesh_connection_size;
  uint32_t off_mesh_connection_align;
  uint32_t off_mesh_connection_offset_start;
  uint32_t off_mesh_connection_offset_end;
  uint32_t off_mesh_connection_offset_radius;
  uint32_t off_mesh_connection_offset_area;
  uint32_t off_mesh_connection_offset_flags;
  uint32_t off_mesh_connection_offset_bidirectional;
  uint32_t off_mesh_connection_offset_user_id;
  uint32_t off_mesh_connection_offset_end_side;

  uint32_t tile_authoring_size;
  uint32_t tile_authoring_align;
  uint32_t tile_authoring_offset_connections;
  uint32_t tile_authoring_offset_connection_count;
  uint32_t tile_authoring_offset_user_id;
  uint32_t tile_authoring_offset_skip_bv_tree;

  uint32_t nav_mesh_params_size;
  uint32_t nav_mesh_params_align;
  uint32_t nav_mesh_params_offset_origin;
  uint32_t nav_mesh_params_offset_tile_width;
  uint32_t nav_mesh_params_offset_tile_height;
  uint32_t nav_mesh_params_offset_max_tiles;
  uint32_t nav_mesh_params_offset_max_polys;

  uint32_t tile_info_size;
  uint32_t tile_info_align;
  /// Number of fields in ZrcTileInfo, and the offset of every one of them in
  /// declaration order, the same precedent as bake_config_offsets: a struct
  /// this wide can reorder two same-sized fields without moving its size or
  /// its alignment, so only a per-field, by-name check would notice.
  uint32_t tile_info_field_count;
  uint32_t tile_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t poly_info_size;
  uint32_t poly_info_align;
  /// Same precedent as tile_info_offsets.
  uint32_t poly_info_field_count;
  uint32_t poly_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t link_size;
  uint32_t link_align;
  uint32_t link_offset_ref;
  uint32_t link_offset_next;
  uint32_t link_offset_edge;
  uint32_t link_offset_side;
  uint32_t link_offset_bmin;
  uint32_t link_offset_bmax;

  uint32_t bv_node_size;
  uint32_t bv_node_align;
  uint32_t bv_node_offset_bmin;
  uint32_t bv_node_offset_bmax;
  uint32_t bv_node_offset_i;

  uint32_t poly_ref_size;
  uint32_t tile_ref_size;

  /// Number of enumerators in ZrcResult, so a consumer can assert its own error
  /// mapping is exhaustive.
  uint32_t result_count;

  /// Upstream constants the wrapper mirrors as its own compile-time values.
  uint32_t max_areas;
  uint32_t verts_per_polygon;
  uint32_t alloc_alignment;

  uint32_t assert_handler_size;
  uint32_t assert_handler_align;
  uint32_t assert_handler_offset_fail;
  uint32_t assert_handler_offset_user;

  uint32_t tile_layout_size;
  uint32_t tile_layout_align;
  /// Number of fields in ZrcTileLayout, and the offset of every one of them in
  /// declaration order, the same precedent as tile_info_offsets: eight
  /// offset/size pairs plus a total, all the same width, so only a per-field
  /// check would notice two of them trading places.
  uint32_t tile_layout_field_count;
  uint32_t tile_layout_offsets[ZRC_ABI_MAX_FIELDS];

  //--- Appended after the freeze. New structs go here rather than beside the
  //--- block they are thematically closest to, so every earlier offset keeps
  //--- the value it already has.

  uint32_t random_source_size;
  uint32_t random_source_align;
  uint32_t random_source_offset_next;
  uint32_t random_source_offset_user;

  uint32_t poly_query_size;
  uint32_t poly_query_align;
  uint32_t poly_query_offset_process;
  uint32_t poly_query_offset_user;

  uint32_t node_size;
  uint32_t node_align;
  uint32_t node_offset_pos;
  uint32_t node_offset_cost;
  uint32_t node_offset_total;
  uint32_t node_offset_ref;
  uint32_t node_offset_parent_index;
  uint32_t node_offset_state;
  uint32_t node_offset_flags;

  uint32_t node_pool_info_size;
  uint32_t node_pool_info_align;
  uint32_t node_pool_info_offset_node_count;
  uint32_t node_pool_info_offset_max_nodes;
  uint32_t node_pool_info_offset_hash_size;
  uint32_t node_pool_info_offset_bytes_used;

  /// ZrcRaycastHit's two fields added after the freeze. Appended here, beside
  /// the rest of the post-freeze additions, rather than beside
  /// raycast_hit_offset_hit above, for the same reason as this whole block.
  uint32_t raycast_hit_offset_hit_edge_index;
  uint32_t raycast_hit_offset_path_cost;

  //--- The staged Recast pipeline's structs.

  /// ZrcBuildContext is a user pointer, six function pointers and two
  /// ZrcBool flags. Six pointers of one width in a row is the worst case for a
  /// spot check: any two of them trading places leaves size, alignment and
  /// every sampled offset identical, and a build's timings then arrive at the
  /// log hook. Every offset is reported for that reason.
  uint32_t build_context_size;
  uint32_t build_context_align;
  uint32_t build_context_field_count;
  uint32_t build_context_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t span_size;
  uint32_t span_align;
  uint32_t span_offset_smin;
  uint32_t span_offset_smax;
  uint32_t span_offset_area;

  /// Two float triples and two floats among four ints, all four bytes wide.
  uint32_t heightfield_info_size;
  uint32_t heightfield_info_align;
  uint32_t heightfield_info_field_count;
  uint32_t heightfield_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t heightfield_storage_size;
  uint32_t heightfield_storage_align;
  uint32_t heightfield_storage_offset_pool_count;
  uint32_t heightfield_storage_offset_free_count;
  uint32_t heightfield_storage_offset_spans_per_pool;

  uint32_t compact_cell_size;
  uint32_t compact_cell_align;
  uint32_t compact_cell_offset_index;
  uint32_t compact_cell_offset_count;

  uint32_t compact_span_size;
  uint32_t compact_span_align;
  uint32_t compact_span_offset_y;
  uint32_t compact_span_offset_reg;
  uint32_t compact_span_offset_con;
  uint32_t compact_span_offset_h;

  /// Same precedent as heightfield_info_offsets, and more of it.
  uint32_t compact_heightfield_info_size;
  uint32_t compact_heightfield_info_align;
  uint32_t compact_heightfield_info_field_count;
  uint32_t compact_heightfield_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t contour_set_info_size;
  uint32_t contour_set_info_align;
  uint32_t contour_set_info_field_count;
  uint32_t contour_set_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t contour_info_size;
  uint32_t contour_info_align;
  uint32_t contour_info_offset_vert_count;
  uint32_t contour_info_offset_raw_vert_count;
  uint32_t contour_info_offset_region;
  uint32_t contour_info_offset_area;

  /// Sixteen fields, thirteen of them four bytes wide. Same precedent again.
  uint32_t poly_mesh_info_size;
  uint32_t poly_mesh_info_align;
  uint32_t poly_mesh_info_field_count;
  uint32_t poly_mesh_info_offsets[ZRC_ABI_MAX_FIELDS];

  /// Number of enumerators in ZrcTimerLabel, RC_MAX_TIMERS included, so a
  /// consumer can assert its own timer table is the right length.
  uint32_t timer_label_count;

  /// Eleven fields, nine of them four-byte ints among two floats. Every offset
  /// is reported for the same reason ZrcBakeConfig's are: a reorder among
  /// same-sized fields would leave size and alignment untouched and quantise
  /// an agent into the wrong stage argument.
  uint32_t build_cells_size;
  uint32_t build_cells_align;
  uint32_t build_cells_field_count;
  uint32_t build_cells_offsets[ZRC_ABI_MAX_FIELDS];

  //--- The layered heightfield and the tile cache. Every one reports all of
  //--- its offsets: each is a run of same-sized fields where a reorder would
  //--- leave the size and the alignment untouched.

  uint32_t heightfield_layer_size;
  uint32_t heightfield_layer_align;
  uint32_t heightfield_layer_field_count;
  uint32_t heightfield_layer_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t tile_cache_params_size;
  uint32_t tile_cache_params_align;
  uint32_t tile_cache_params_field_count;
  uint32_t tile_cache_params_offsets[ZRC_ABI_MAX_FIELDS];

  /// Three function pointers in a row after a user pointer: the worst case
  /// for a spot check, since any two trading places compresses with the
  /// decompressor and reads back garbage.
  uint32_t tile_cache_compressor_size;
  uint32_t tile_cache_compressor_align;
  uint32_t tile_cache_compressor_field_count;
  uint32_t tile_cache_compressor_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t tile_cache_allocator_size;
  uint32_t tile_cache_allocator_align;
  uint32_t tile_cache_allocator_field_count;
  uint32_t tile_cache_allocator_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t tile_cache_build_params_size;
  uint32_t tile_cache_build_params_align;
  uint32_t tile_cache_build_params_field_count;
  uint32_t tile_cache_build_params_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t compressed_tile_info_size;
  uint32_t compressed_tile_info_align;
  uint32_t compressed_tile_info_field_count;
  uint32_t compressed_tile_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t obstacle_info_size;
  uint32_t obstacle_info_align;
  uint32_t obstacle_info_field_count;
  uint32_t obstacle_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t tile_cache_layer_header_size;
  uint32_t tile_cache_layer_header_align;
  uint32_t tile_cache_layer_header_field_count;
  uint32_t tile_cache_layer_header_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t tile_cache_contour_info_size;
  uint32_t tile_cache_contour_info_align;
  uint32_t tile_cache_contour_info_offset_vert_count;
  uint32_t tile_cache_contour_info_offset_region;
  uint32_t tile_cache_contour_info_offset_area;

  uint32_t tile_cache_poly_mesh_info_size;
  uint32_t tile_cache_poly_mesh_info_align;
  uint32_t tile_cache_poly_mesh_info_offset_vert_count;
  uint32_t tile_cache_poly_mesh_info_offset_poly_count;
  uint32_t tile_cache_poly_mesh_info_offset_verts_per_poly;

  uint32_t compressed_tile_ref_size;
  uint32_t obstacle_ref_size;

  /// The crowd's cross-boundary structs, every offset reported.
  ///
  /// ZrcCrowdAgentParams and ZrcCrowdAgentDebug each carry a pointer, so their
  /// size and several offsets differ between a 32-bit and a 64-bit target —
  /// which is exactly the disagreement a hand-written Zig extern struct can
  /// get wrong without any compiler noticing.
  uint32_t crowd_agent_params_size;
  uint32_t crowd_agent_params_align;
  uint32_t crowd_agent_params_field_count;
  uint32_t crowd_agent_params_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t crowd_agent_size;
  uint32_t crowd_agent_align;
  uint32_t crowd_agent_field_count;
  uint32_t crowd_agent_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t crowd_corner_size;
  uint32_t crowd_corner_align;
  uint32_t crowd_corner_field_count;
  uint32_t crowd_corner_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t crowd_neighbour_size;
  uint32_t crowd_neighbour_align;
  uint32_t crowd_neighbour_field_count;
  uint32_t crowd_neighbour_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t crowd_agent_animation_size;
  uint32_t crowd_agent_animation_align;
  uint32_t crowd_agent_animation_field_count;
  uint32_t crowd_agent_animation_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t crowd_agent_debug_size;
  uint32_t crowd_agent_debug_align;
  uint32_t crowd_agent_debug_field_count;
  uint32_t crowd_agent_debug_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t avoidance_params_size;
  uint32_t avoidance_params_align;
  uint32_t avoidance_params_field_count;
  uint32_t avoidance_params_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t avoidance_circle_size;
  uint32_t avoidance_circle_align;
  uint32_t avoidance_circle_field_count;
  uint32_t avoidance_circle_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t avoidance_segment_size;
  uint32_t avoidance_segment_align;
  uint32_t avoidance_segment_field_count;
  uint32_t avoidance_segment_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t avoidance_sample_size;
  uint32_t avoidance_sample_align;
  uint32_t avoidance_sample_field_count;
  uint32_t avoidance_sample_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t path_corridor_info_size;
  uint32_t path_corridor_info_align;
  uint32_t path_corridor_info_field_count;
  uint32_t path_corridor_info_offsets[ZRC_ABI_MAX_FIELDS];

  uint32_t agent_ref_size;
  uint32_t path_request_ref_size;
} ZrcAbiLayout;

/// Fills `out` with the layout the library was compiled with. Never fails.
ZRC_API void zrcAbiLayout(ZrcAbiLayout* out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZRECAST_H_
