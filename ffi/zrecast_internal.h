//===----------------------------------------------------------------------===//
// zrecast — implementation-private declarations shared by the ffi/*.cpp units.
//
// Not installed and not part of the ABI. Nothing here may appear in zrecast.h.
//===----------------------------------------------------------------------===//

#ifndef ZRECAST_INTERNAL_H_
#define ZRECAST_INTERNAL_H_

#include <math.h>
#include <new>
#include <stdint.h>
#include <string.h>

#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "DetourLocalBoundary.h"
#include "DetourObstacleAvoidance.h"
#include "DetourPathCorridor.h"
#include "DetourPathQueue.h"
#include "DetourProximityGrid.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourStatus.h"
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "zrecast.h"

namespace zrc {

//===----------------------------------------------------------------------===//
// Allocation
//
// zrecast's own handles go through dtAlloc/dtFree; the Recast container types
// further down go through rcAlloc/rcFree, because Recast releases their
// internal buffers with rcFree and the pairing should be honest. zrcSetAllocator
// has already pointed both at the same host, so there is one seam and one
// accounting whichever half of upstream asked for the memory.
//===----------------------------------------------------------------------===//

inline void* Alloc(size_t size, dtAllocHint hint) { return dtAlloc(size, hint); }
inline void Free(void* block) { dtFree(block); }

/// Checked placement-new. Recast's rcNew, and several raw
/// `new (dtAlloc(...)) T` sites in Detour, construct without testing the
/// allocation first — so a failure there is a construction at address zero.
/// These make the ZRC_ERR_OUT_OF_MEMORY paths reachable rather than decorative.
template <typename T, typename... Args>
T* New(Args&&... args) {
  void* block = Alloc(sizeof(T), DT_ALLOC_PERM);
  if (block == nullptr) return nullptr;
  return new (block) T(static_cast<Args&&>(args)...);
}

template <typename T>
void Delete(T* object) {
  if (object == nullptr) return;
  object->~T();
  Free(object);
}

/// Allocates one of Recast's container types.
///
/// Recast's own rcAllocPolyMesh and friends do exactly this minus the null
/// test: rcNew is `new (rcAlloc(...)) T()`, so a failed allocation constructs
/// at address zero. The container comes from rcAlloc rather than dtAlloc
/// because Recast releases its internals with rcFree, and pairing the two
/// keeps the ownership story honest even though both reach the same host.
template <typename T>
T* RcNew() {
  void* block = rcAlloc(sizeof(T), RC_ALLOC_PERM);
  if (block == nullptr) return nullptr;
  return new (block) T();
}

/// Releases one of Recast's container types, through upstream's own free
/// function for that type.
///
/// This is an overload set rather than a `~T(); rcFree(p)` template on purpose.
/// Four of Recast's five containers free their internal buffers in a
/// destructor, so the generic form would work for them — but rcPolyMeshDetail
/// has no user-declared destructor at all, and its buffers are released only by
/// rcFreePolyMeshDetail. A generic delete therefore leaks three allocations per
/// detail mesh, silently, because nothing in the type system distinguishes it
/// from the other four. Deferring to upstream's own free function per type
/// keeps that asymmetry upstream's business instead of something this package
/// has to remember. See UPSTREAM.md.
inline void RcFree(rcHeightfield* p) { rcFreeHeightField(p); }
inline void RcFree(rcCompactHeightfield* p) { rcFreeCompactHeightfield(p); }
inline void RcFree(rcContourSet* p) { rcFreeContourSet(p); }
inline void RcFree(rcHeightfieldLayerSet* p) { rcFreeHeightfieldLayerSet(p); }
inline void RcFree(rcPolyMesh* p) { rcFreePolyMesh(p); }
inline void RcFree(rcPolyMeshDetail* p) { rcFreePolyMeshDetail(p); }

//===----------------------------------------------------------------------===//
// Scalar validation
//
// Recast and Detour assert in debug and read on regardless in release, so a NaN
// or a wild index from a caller becomes a fault deep inside upstream rather
// than an error at the door. These are the door.
//===----------------------------------------------------------------------===//

/// True for a finite float. Written as a self-comparison plus a magnitude test
/// rather than std::isfinite so it behaves the same under -ffast-math hosts.
inline bool IsFinite(float v) { return v == v && v <= 3.402823466e+38f && v >= -3.402823466e+38f; }

inline bool IsFiniteVec3(const float* v) {
  return v != nullptr && IsFinite(v[0]) && IsFinite(v[1]) && IsFinite(v[2]);
}

/// Cosine of an angle in degrees, to double precision, using only the four
/// operations IEEE-754 specifies exactly.
///
/// A cook must not depend on the host's libm, and `cosf` is the one thing in
/// the bake path that would tie it there. The standard does not require a
/// correctly rounded cosine, so two C libraries may legitimately differ in the
/// last bit — and the walkable-slope threshold that comes out of it decides,
/// per triangle, whether a surface is navigable at all.
///
/// The argument is bounded: zrecast refuses a slope outside [0, 90), so x stays
/// within [0, pi/2) where the Taylor series through x^16 is accurate to about
/// 5e-13, four orders below the precision of the float it becomes. Every step
/// is an add, a subtract, a multiply or a divide, each of which IEEE-754
/// defines as correctly rounded, so the result is bit-identical everywhere the
/// package is compiled with -ffp-contract=off.
inline double CosDegrees(double degrees) {
  const double pi = 3.14159265358979323846;
  const double x = degrees / 180.0 * pi;
  const double x2 = x * x;
  // Horner, from the smallest term outwards. The coefficients are 1/(2k)!.
  double r = -1.0 / 20922789888000.0;         // x^16
  r = r * x2 + 1.0 / 87178291200.0;           // x^14
  r = r * x2 - 1.0 / 479001600.0;             // x^12
  r = r * x2 + 1.0 / 3628800.0;               // x^10
  r = r * x2 - 1.0 / 40320.0;                 // x^8
  r = r * x2 + 1.0 / 720.0;                   // x^6
  r = r * x2 - 1.0 / 24.0;                    // x^4
  r = r * x2 + 1.0 / 2.0;                     // x^2
  return 1.0 - r * x2;
}

/// Coordinates must stay well inside float range: `bmax - bmin` is computed as
/// a float in several places, and two opposite extremes overflow it to
/// infinity, which then propagates into the BV quantisation of a tile image.
const float kMaxCoordinate = 1e18f;

/// A tile grid handed back by a caller, checked before either half of a tiled
/// build trusts it.
///
/// zrcTileGridCompute is where one comes from, but nothing stops a host storing
/// it in an asset and reading it back, so both the bake and the navmesh check
/// rather than assume. Both, because they fail differently: a NaN origin is an
/// out-of-range float conversion inside rcRasterizeTriangles on one side, and a
/// tile width Detour divides by on the other.
inline ZrcResult ValidateTileGrid(const ZrcTileGrid& grid) {
  if (!IsFiniteVec3(grid.origin) || !IsFiniteVec3(grid.extent_max)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsFinite(grid.tile_world_size) || !(grid.tile_world_size > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (grid.origin[i] < -kMaxCoordinate ||
        grid.extent_max[i] > kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  if (grid.tile_count_x < 1 || grid.tile_count_z < 1) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (grid.tile_count_x > ZRC_MAX_TILE_COORD ||
      grid.tile_count_z > ZRC_MAX_TILE_COORD) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Serialised navmesh images
//===----------------------------------------------------------------------===//

/// What a caller will accept beyond the image being well formed.
///
/// The two are separate questions. "Well formed" is about memory safety and is
/// the same for every tile. "Admissible" depends on the navmesh the tile is
/// going into: a single-tile mesh has no neighbours, so a portal edge in one of
/// its polygons points nowhere, while a tile of a grid must land inside that
/// grid. Keeping them apart means one definition of well formed and a
/// per-caller definition of admissible, rather than three validators.
struct TileAdmission {
  /// Largest grid coordinate the header may carry, inclusive.
  int max_tile_x;
  int max_tile_z;
  /// Largest layer the header may carry, inclusive. A lone tile is layer 0.
  int max_tile_layer;
  /// Whether a polygon may carry a portal edge to a neighbouring tile.
  bool allow_portals;
};

/// Full validation of a serialised tile image, header and interior. Shared by
/// zrcNavMeshValidate, zrcNavMeshDeserialize and zrcNavMeshAddTile so there is
/// exactly one definition of "well formed".
///
/// Two layers, and both are needed. The first recomputes, line for line, the
/// pointer arithmetic dtNavMesh::addTile performs on the same buffer — addTile
/// derives eight array pointers from the header's counts and never compares
/// them against the buffer length it was handed. The second walks those arrays
/// and bounds every index Detour will dereference, because a count that fits
/// the buffer says nothing about the indices stored inside it.
ZrcResult ValidateNavMeshImage(const void* data, size_t size,
                               const TileAdmission& admission);

/// The admission a lone tile gets: grid position pinned to the origin and no
/// portal edges, since there is nothing on the other side of one.
TileAdmission SoleTileAdmission();

/// The admission any well-formed tile gets, used by zrcNavMeshValidate, which
/// answers "can this be loaded at all" rather than "does it belong here".
TileAdmission AnyTileAdmission();

//===----------------------------------------------------------------------===//
// Tiles
//===----------------------------------------------------------------------===//

/// Cooks a baked polygon mesh into Detour tile bytes at a grid position.
///
/// Shared by zrcTileDataBuild and zrcNavMeshCreate: the single-tile mesh is the
/// grid position (0, 0, 0) with no neighbours, not a different code path. The
/// buffer comes from Detour's allocator and the caller owns it. `authoring`
/// may be NULL.
ZrcResult BuildTileData(const ZrcPolyMesh& mesh, int32_t tile_x, int32_t tile_z,
                        int32_t tile_layer, const ZrcTileAuthoring* authoring,
                        unsigned char** out_data, int* out_size);

//===----------------------------------------------------------------------===//
// Detour status mapping
//===----------------------------------------------------------------------===//

/// Collapses a dtStatus into a ZrcResult.
///
/// Detour packs a success/failure bit together with detail bits, and some
/// detail bits ride along with success (DT_PARTIAL_RESULT, and
/// DT_BUFFER_TOO_SMALL from the path queries, which fill what they can). Those
/// are not failures and are reported through the separate `partial` out
/// parameters instead; only a status with DT_FAILURE set reaches the mapping
/// below.
ZrcResult ResultFromStatus(dtStatus status);

/// True when a *successful* status says the answer was cut short — the goal was
/// not reached, or the caller's buffer could not hold the whole result.
inline bool StatusIsPartial(dtStatus status) {
  return (status & (DT_PARTIAL_RESULT | DT_BUFFER_TOO_SMALL)) != 0;
}

/// Scratch bytes from the host allocator, released on scope exit. A `size` of
/// zero allocates nothing, so a caller need not special-case the empty case.
class TempBuffer {
 public:
  explicit TempBuffer(size_t size)
      : ptr_(size > 0 ? Alloc(size, DT_ALLOC_TEMP) : nullptr) {}
  ~TempBuffer() { Free(ptr_); }

  void* get() const { return ptr_; }

 private:
  TempBuffer(const TempBuffer&);
  TempBuffer& operator=(const TempBuffer&);
  void* ptr_;
};

/// Largest voxel count along one axis of a heightfield.
///
/// Recast squares voxel quantities in plain `int` arithmetic, and — this is the
/// part that decides the number — it *sums* two of those squares:
/// simplifyContour compares `dx*dx + dz*dz` against `maxEdgeLen*maxEdgeLen`
/// (RecastContour.cpp:399). So the bound is not the largest integer whose
/// square fits an int (46340) but the largest whose square fits twice over.
/// 2 * 32767^2 is 2147352578, just under INT_MAX; 32768 would not fit.
///
/// It is not a practical limit. A single-tile mesh 32 000 voxels across is
/// already far past the point where a tiled mesh is the right answer, and
/// tiling is the documented way to go bigger.
const int kMaxAxisCells = 32767;

/// Largest unnavigable border, in voxels, a build stage will accept.
///
/// The width a compact heightfield's own borderSize field is documented to
/// hold, and the same ceiling ZrcBakeConfig::border_size carries. Shared so a
/// bake, a staged region build and a layer build answer to one number.
const int kMaxBorderSizeCells = 255;

/// Largest agent radius, in voxels, that Recast's erosion can express.
///
/// rcErodeWalkableArea reduces the radius to `(unsigned char)(radius * 2)` and
/// compares it against a distance field stored in bytes, so 127 is the point
/// past which the threshold stops meaning what it says.
const int kMaxWalkableRadiusCells = 127;

/// What a world extent and a cell size add up to, as a voxel grid.
enum class GridExtent {
  /// A grid Recast can allocate and address.
  kOk,
  /// The cell is larger than the geometry: fewer than one cell on an axis.
  kTooCoarse,
  /// The cell is small enough that the grid is past kMaxAxisCells on an axis,
  /// or past 2^28 cells in total.
  kTooFine,
};

/// Bounds the grid rcCalcGridSize would compute from `bmin`, `bmax` and `cs`.
///
/// rcCalcGridSize converts each span to `(int)(span / cs + 0.5f)`, which is
/// undefined once the quotient leaves int range, and rcCreateHeightfield then
/// allocates `width * height` span pointers from the result. Both bounds are
/// applied in double, before either cast can happen.
inline GridExtent CheckGridExtentFit(const float* bmin, const float* bmax,
                                     float cs) {
  const double span_x =
      static_cast<double>(bmax[0]) - static_cast<double>(bmin[0]);
  const double span_z =
      static_cast<double>(bmax[2]) - static_cast<double>(bmin[2]);
  const double cells_x = span_x / static_cast<double>(cs) + 0.5;
  const double cells_z = span_z / static_cast<double>(cs) + 0.5;

  if (!(cells_x >= 1.0) || !(cells_z >= 1.0)) return GridExtent::kTooCoarse;

  const double kMaxCells = 268435456.0;  // 2^28 in total...
  const double kMaxAxis = static_cast<double>(kMaxAxisCells);  // ...and per axis
  if (!(cells_x <= kMaxAxis) || !(cells_z <= kMaxAxis) ||
      cells_x * cells_z > kMaxCells) {
    return GridExtent::kTooFine;
  }
  return GridExtent::kOk;
}

/// Largest vertex or triangle count a mesh crossing this boundary may carry.
///
/// Not a Recast limit: it guards the index arithmetic on both sides, here and
/// upstream, against overflowing an int before any of it is performed.
const int kMaxTriMeshCount = 1 << 28;

/// Rejects geometry Recast would read out of bounds or turn into NaN.
///
/// Recast indexes `verts[tris[i] * 3]` with no range test of its own — an index
/// past the end is an out-of-bounds read inside the rasteriser, in every build
/// configuration. A NaN coordinate is worse: rcCalcBounds propagates it into
/// the grid size, and the heightfield allocation that follows is computed from
/// garbage.
///
/// Shared rather than per-file: the bake, the staged rasterisers and the
/// triangle-area markers all hand the same arrays to the same upstream
/// functions, and a second copy of this rule is a second place for it to drift.
inline ZrcResult ValidateTriMesh(const ZrcTriMesh& mesh) {
  if (mesh.verts == nullptr || mesh.tris == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (mesh.vert_count < 3 || mesh.tri_count < 1) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Guard the index arithmetic below, and Recast's own, against overflow.
  if (mesh.vert_count > kMaxTriMeshCount ||
      mesh.tri_count > kMaxTriMeshCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  for (int i = 0; i < mesh.vert_count * 3; ++i) {
    if (!IsFinite(mesh.verts[i])) return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < mesh.tri_count * 3; ++i) {
    const int index = mesh.tris[i];
    if (index < 0 || index >= mesh.vert_count) return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// Rejects area authoring Recast would misread: an out-of-range shape or area
/// id, a non-finite extent, or a shape whose own fields do not describe
/// geometry Recast can rasterise.
///
/// Shared with the staged pipeline: a bake and a hand-driven
/// zrcCompactHeightfieldMarkAreas apply the same volumes to the same upstream
/// markers, so they answer to one rule rather than two copies of it.
inline ZrcResult ValidateAreaAuthoring(const ZrcAreaAuthoring& authoring) {
  if (authoring.volume_count < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (authoring.volume_count > 0 && authoring.volumes == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // An arbitrary but stated ceiling, so a wild count is an error rather than a
  // very long loop.
  if (authoring.volume_count > 65536) return ZRC_ERR_INVALID_ARGUMENT;

  for (int i = 0; i < authoring.volume_count; ++i) {
    const ZrcAreaVolume& v = authoring.volumes[i];
    if (v.shape != ZRC_VOLUME_CONVEX && v.shape != ZRC_VOLUME_BOX &&
        v.shape != ZRC_VOLUME_CYLINDER) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (v.area < 0 || v.area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
    if (!IsFinite(v.y_min) || !IsFinite(v.y_max) || v.y_min > v.y_max) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }

    switch (v.shape) {
      case ZRC_VOLUME_CONVEX:
        if (v.verts == nullptr || v.vert_count < 3 ||
            v.vert_count > (1 << 20)) {
          return ZRC_ERR_INVALID_ARGUMENT;
        }
        for (int k = 0; k < v.vert_count * 3; ++k) {
          if (!IsFinite(v.verts[k])) return ZRC_ERR_INVALID_ARGUMENT;
        }
        break;
      case ZRC_VOLUME_BOX:
        if (!IsFinite(v.xz_min[0]) || !IsFinite(v.xz_min[1]) ||
            !IsFinite(v.xz_max[0]) || !IsFinite(v.xz_max[1])) {
          return ZRC_ERR_INVALID_ARGUMENT;
        }
        if (v.xz_min[0] > v.xz_max[0] || v.xz_min[1] > v.xz_max[1]) {
          return ZRC_ERR_INVALID_ARGUMENT;
        }
        break;
      case ZRC_VOLUME_CYLINDER:
        if (!IsFinite(v.xz_min[0]) || !IsFinite(v.xz_min[1])) {
          return ZRC_ERR_INVALID_ARGUMENT;
        }
        if (!IsFinite(v.radius) || !(v.radius > 0.f)) {
          return ZRC_ERR_INVALID_ARGUMENT;
        }
        break;
    }
  }

  // An unwalkable polygon that some filter still admits is a polygon a path
  // can cross.
  if (authoring.area_flags != nullptr &&
      authoring.area_flags[ZRC_AREA_NULL] != 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// Colours a compact heightfield's area ids with the authored volumes, in
/// array order, so a later volume overwrites an earlier one where they
/// overlap.
///
/// `authoring` must already have passed ValidateAreaAuthoring. Shared so that
/// a bake and a hand-driven mark produce bit-identical fields: the order, the
/// argument construction and the choice of upstream marker are all decided
/// here, once.
inline void MarkAreaVolumes(rcContext& ctx, const ZrcAreaAuthoring& authoring,
                            rcCompactHeightfield& chf) {
  for (int i = 0; i < authoring.volume_count; ++i) {
    const ZrcAreaVolume& v = authoring.volumes[i];
    const unsigned char area_id = static_cast<unsigned char>(v.area);
    switch (v.shape) {
      case ZRC_VOLUME_CONVEX:
        rcMarkConvexPolyArea(&ctx, v.verts, v.vert_count, v.y_min, v.y_max,
                             area_id, chf);
        break;
      case ZRC_VOLUME_BOX: {
        const float bmin[3] = {v.xz_min[0], v.y_min, v.xz_min[1]};
        const float bmax[3] = {v.xz_max[0], v.y_max, v.xz_max[1]};
        rcMarkBoxArea(&ctx, bmin, bmax, area_id, chf);
        break;
      }
      case ZRC_VOLUME_CYLINDER: {
        const float pos[3] = {v.xz_min[0], v.y_min, v.xz_min[1]};
        rcMarkCylinderArea(&ctx, pos, v.radius, v.y_max - v.y_min, area_id,
                           chf);
        break;
      }
    }
  }
}

/// The same work rcMarkWalkableTriangles does, with upstream's own vector
/// helpers over upstream's own edge cross product, and one difference:
/// upstream takes the threshold from `cosf`, which the C standard does not
/// require to be correctly rounded. Two platforms may therefore disagree in the
/// last bit of the threshold, and the comparison it feeds decides per triangle
/// whether a surface is navigable — so a cook would depend on the host's libm.
/// CosDegrees computes the same threshold using only exactly specified
/// operations, so the cook does not depend on the host's libm.
///
/// Shared with the staged pipeline for exactly that reason: a host that marks
/// triangles a stage at a time must get the platform-stable threshold too, or
/// the libm independence holds only for whole-bake callers.
inline void MarkWalkableTriangles(float slope_degrees, const ZrcTriMesh& mesh,
                                  unsigned char* tri_areas) {
  const float threshold =
      static_cast<float>(CosDegrees(static_cast<double>(slope_degrees)));
  for (int i = 0; i < mesh.tri_count; ++i) {
    const int* tri = &mesh.tris[i * 3];
    float e0[3];
    float e1[3];
    float normal[3];
    rcVsub(e0, &mesh.verts[tri[1] * 3], &mesh.verts[tri[0] * 3]);
    rcVsub(e1, &mesh.verts[tri[2] * 3], &mesh.verts[tri[0] * 3]);
    rcVcross(normal, e0, e1);
    rcVnormalize(normal);
    if (normal[1] > threshold) tri_areas[i] = RC_WALKABLE_AREA;
  }
}

/// The inverse, matching rcClearUnwalkableTriangles: every triangle steeper
/// than the threshold is cleared to the null area and the rest are untouched.
/// Same platform-stable threshold, for the same reason.
inline void ClearUnwalkableTriangles(float slope_degrees,
                                     const ZrcTriMesh& mesh,
                                     unsigned char* tri_areas) {
  const float threshold =
      static_cast<float>(CosDegrees(static_cast<double>(slope_degrees)));
  for (int i = 0; i < mesh.tri_count; ++i) {
    const int* tri = &mesh.tris[i * 3];
    float e0[3];
    float e1[3];
    float normal[3];
    rcVsub(e0, &mesh.verts[tri[1] * 3], &mesh.verts[tri[0] * 3]);
    rcVsub(e1, &mesh.verts[tri[2] * 3], &mesh.verts[tri[0] * 3]);
    rcVcross(normal, e0, e1);
    rcVnormalize(normal);
    if (normal[1] <= threshold) tri_areas[i] = RC_NULL_AREA;
  }
}

/// Rejects one off-mesh connection Detour would misread: a non-finite or
/// out-of-domain endpoint, a radius that is not a positive finite number, or
/// an area id outside range.
///
/// Shared because a connection reaches Detour from two directions — a tile a
/// host authors, and a tile the cache rebuilds through its mesh-process
/// callback — and both must hold it to the same rule.
inline ZrcResult ValidateOffMeshConnection(const ZrcOffMeshConnection& con) {
  if (!IsFiniteVec3(con.start) || !IsFiniteVec3(con.end)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int k = 0; k < 3; ++k) {
    if (con.start[k] < -kMaxCoordinate || con.start[k] > kMaxCoordinate ||
        con.end[k] < -kMaxCoordinate || con.end[k] > kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  if (!IsFinite(con.radius) || !(con.radius > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (con.area < 0 || con.area >= ZRC_MAX_AREAS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// Largest number of off-mesh connections one tile may carry.
///
/// Arbitrary but stated, so a count read from a file is an error rather than a
/// very long loop.
const int kMaxOffMeshConnections = 65536;

/// True when a vertical extent, in cells, is one a Recast span can address.
///
/// Easy to miss, because Recast never turns the vertical extent into a grid
/// dimension. rasterizeTri computes `(int)ceilf(spanMax * inverseCellHeight)`
/// and only *then* clamps the result to RC_SPAN_MAX_HEIGHT
/// (RecastRasterization.cpp:445-446), so a tall triangle over a fine cell
/// height is an out-of-range float conversion before the clamp ever runs. One
/// vertex at y = 4e8 with a 0.2 cell height is enough.
///
/// The bound is RC_SPAN_MAX_HEIGHT rather than INT_MAX because that is all a
/// 13-bit span extent can address. Beyond it Recast silently flattens
/// everything into the topmost span, so refusing turns a quietly wrong mesh
/// into an answerable error as well as removing the undefined behaviour.
inline bool VerticalExtentFits(float bmin_y, float bmax_y, float ch) {
  const double span_y = static_cast<double>(bmax_y) - static_cast<double>(bmin_y);
  const double cells_y = span_y / static_cast<double>(ch) + 1.0;
  return cells_y <= static_cast<double>(RC_SPAN_MAX_HEIGHT);
}

/// True when a vertex maps to grid coordinates rasterizeTri can compute.
///
/// rasterizeTri converts a triangle's own bounds to cell indices with
/// `(int)((triBBMin[2] - hfBBMin[2]) * inverseCellSize)`
/// (RecastRasterization.cpp:335-336, 384-385) and clamps only afterwards, so
/// the conversion is undefined before the clamp can help. Upstream's
/// overlapBounds test rejects a triangle wholly outside the field, but one
/// that straddles it reaches the cast with whatever the far vertex holds.
inline bool RasterVertexFits(const rcHeightfield& hf, const float* v) {
  if (!IsFiniteVec3(v)) return false;
  for (int k = 0; k < 2; ++k) {
    const int axis = k == 0 ? 0 : 2;
    const double cells = (static_cast<double>(v[axis]) -
                          static_cast<double>(hf.bmin[axis])) /
                         static_cast<double>(hf.cs);
    if (!(cells >= -2147483648.0 && cells < 2147483648.0)) return false;
  }
  return true;
}

//===----------------------------------------------------------------------===//
// The tile cache's three host-supplied interfaces
//
// Upstream takes each as an abstract class. Each of these wraps the POD the C
// ABI carries, and is built on the stack or held by the handle for exactly as
// long as upstream needs it.
//===----------------------------------------------------------------------===//

/// A caller's ZrcTileCacheCompressor, wearing upstream's interface.
///
/// The three hooks are required, so none is null-checked per call: the entry
/// point that accepts a ZrcTileCacheCompressor rejects an incomplete one.
class HostCompressor : public dtTileCacheCompressor {
 public:
  explicit HostCompressor(const ZrcTileCacheCompressor& hooks);

  int maxCompressedSize(const int bufferSize) override;
  dtStatus compress(const unsigned char* buffer, const int bufferSize,
                    unsigned char* compressed, const int maxCompressedSize,
                    int* compressedSize) override;
  dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                      unsigned char* buffer, const int maxBufferSize,
                      int* bufferSize) override;

 private:
  ZrcTileCacheCompressor hooks_;
};

/// A caller's ZrcTileCacheAllocator, or upstream's own default when the
/// caller supplied none.
///
/// The default's alloc and free already reach dtAlloc and dtFree, which is the
/// seam zrcSetAllocator installs, so a NULL allocator is already a host's
/// allocator. Only `reset` is a capability the default does not offer.
class HostTileCacheAlloc : public dtTileCacheAlloc {
 public:
  explicit HostTileCacheAlloc(const ZrcTileCacheAllocator* hooks);

  void reset() override;
  void* alloc(const size_t size) override;
  void free(void* ptr) override;

 private:
  const ZrcTileCacheAllocator* hooks_;
};

/// A caller's ZrcTileCacheMeshProcess, narrowed.
///
/// Upstream hands its callback the whole mutable dtNavMeshCreateParams and
/// feeds the result straight to dtNavMesh::addTile, which is the one path into
/// a navmesh that skips zrcNavMeshAddTile's validation. This shows the caller
/// only the fields the callback is for, validates what comes back, and writes
/// it into the params itself. See UPSTREAM.md.
class HostMeshProcess : public dtTileCacheMeshProcess {
 public:
  HostMeshProcess(ZrcTileCacheMeshProcess hook, void* user);

  void process(dtNavMeshCreateParams* params, unsigned char* polyAreas,
               unsigned short* polyFlags) override;

  /// What the last process() call returned, so the entry point that drove it
  /// can report a callback's failure rather than discarding it: upstream's
  /// process() returns void and has nowhere to put one.
  ZrcResult LastResult() const { return last_result_; }
  void ClearLastResult() { last_result_ = ZRC_OK; }

 private:
  ZrcTileCacheMeshProcess hook_;
  void* user_;
  ZrcResult last_result_;
  /// The connection arrays the last callback asked for, unpacked into the six
  /// parallel arrays dtNavMeshCreateParams wants and owned until the next
  /// call replaces them or the handle dies.
  TempBuffer con_verts_;
  TempBuffer con_rad_;
  TempBuffer con_flags_;
  TempBuffer con_areas_;
  TempBuffer con_dir_;
  TempBuffer con_user_id_;
};

//===----------------------------------------------------------------------===//
// The staged Recast pipeline
//===----------------------------------------------------------------------===//

/// A caller's ZrcBuildContext, wearing the rcContext every Recast stage takes.
///
/// Built on the stack for the duration of one stage call rather than stored, so
/// a host that swaps a hook between calls is obeyed, and a NULL ZrcBuildContext
/// is a context with both flags clear. Upstream consults m_logEnabled and
/// m_timerEnabled before it reaches a virtual, so the flags are set here rather
/// than tested in each hook.
class HostContext : public rcContext {
 public:
  explicit HostContext(const ZrcBuildContext* hooks);

 protected:
  void doResetLog() override;
  void doLog(rcLogCategory category, const char* msg, int len) override;
  void doResetTimers() override;
  void doStartTimer(rcTimerLabel label) override;
  void doStopTimer(rcTimerLabel label) override;
  int doGetAccumulatedTime(rcTimerLabel label) const override;

 private:
  const ZrcBuildContext* hooks_;
};

/// Bounds a `(first, count)` window against an array of `length` entries.
///
/// A window outside the array is an error rather than a short read, so a caller
/// cannot mistake a truncated answer for a complete one. A zero count is
/// accepted at any `first` within the array, and at `first == length`.
inline ZrcResult CheckRange(int32_t first, int32_t count, int32_t length) {
  if (first < 0 || count < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (first > length) return ZRC_ERR_INVALID_ARGUMENT;
  if (count > length - first) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// True when a Recast timer label names one of the twenty-eight phases.
///
/// RC_MAX_TIMERS is the length of the table upstream indexes with a label, not
/// a label, so it is refused here rather than passed through as one.
inline bool IsTimerLabel(ZrcTimerLabel label) {
  return label >= 0 && label < ZRC_MAX_TIMERS;
}

inline bool IsLogCategory(ZrcLogCategory category) {
  return category == ZRC_LOG_PROGRESS || category == ZRC_LOG_WARNING ||
         category == ZRC_LOG_ERROR;
}


//===----------------------------------------------------------------------===//
// Query filters
//===----------------------------------------------------------------------===//

/// Translates the POD filter into Detour's class.
///
/// dtQueryFilter is a C++ type with private members and no aggregate
/// initialisation, so it cannot appear in the ABI. Building one per call costs
/// a fixed ~260-byte fill, which is noise beside the graph search it feeds.
///
/// Shared rather than per-unit: five translation units now hand a filter to
/// Detour, and a cost check that holds in one and not another is the kind of
/// divergence this package has grown before.
inline bool BuildFilter(const ZrcQueryFilter& in, dtQueryFilter* out) {
  for (int i = 0; i < ZRC_MAX_AREAS; ++i) {
    const float cost = in.area_cost[i];
    // A negative or non-finite cost makes A* incoherent: the open list stops
    // being a priority queue in any useful sense, and a negative edge can make
    // the search loop.
    if (!IsFinite(cost) || cost < 0.f) return false;
    out->setAreaCost(i, cost);
  }
  out->setIncludeFlags(in.include_flags);
  out->setExcludeFlags(in.exclude_flags);
  return true;
}

/// The reverse, for reading a filter a crowd holds back out.
inline void ReadFilter(const dtQueryFilter& in, ZrcQueryFilter* out) {
  for (int i = 0; i < ZRC_MAX_AREAS; ++i) {
    out->area_cost[i] = in.getAreaCost(i);
  }
  out->include_flags = in.getIncludeFlags();
  out->exclude_flags = in.getExcludeFlags();
}

//===----------------------------------------------------------------------===//
// Crowds and the pieces they steer with
//===----------------------------------------------------------------------===//

/// Agent slots one crowd may hold. Mirrors ZRC_CROWD_MAX_AGENTS.
const int kMaxCrowdAgents = ZRC_CROWD_MAX_AGENTS;

/// Entries dtProximityGrid can address.
///
/// It stores a pool index in an unsigned short and reserves 0xffff as the
/// end-of-chain marker (DetourProximityGrid.cpp:120-127), so the last usable
/// index is 0xfffe and a pool may hold that many entries.
const int kMaxProximityGridPool = 0xfffe;

/// Cell coordinates dtProximityGrid can store faithfully.
///
/// An item records its cell as a `short` and every read back compares that
/// truncated copy against the full int (DetourProximityGrid.cpp:117-118,
/// 145-146, 180). Beyond this an item is filed and then never found, which is
/// a neighbour an agent does not steer around rather than an error.
const int kMaxProximityGridCell = 32767;

/// Cells one proximity-grid box may span on an axis.
///
/// addItem and queryItems loop from the floored minimum to the floored maximum
/// on both axes (DetourProximityGrid.cpp:110-112, 138-141) with nothing
/// bounding the count. Two coordinates far enough apart is a loop that does
/// not end rather than a bad answer.
const int kMaxProximityGridSpan = 4096;

/// Bounds a proximity-grid box, before either loop turns it into cells.
inline ZrcResult CheckProximityBox(float min_x, float min_y, float max_x,
                                   float max_y, float inv_cell_size) {
  if (!IsFinite(min_x) || !IsFinite(min_y) || !IsFinite(max_x) ||
      !IsFinite(max_y)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // A cell size small enough that its reciprocal overflows leaves every
  // product below either infinite or, for a bound of exactly zero, NaN — and
  // every comparison against NaN is false, so the range tests would pass it
  // through to the conversion they exist to prevent.
  if (!IsFinite(inv_cell_size) || !(inv_cell_size > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (min_x > max_x || min_y > max_y) return ZRC_ERR_INVALID_ARGUMENT;
  // In double, because the product of two finite floats is what overflows.
  const double lo_x = floor(static_cast<double>(min_x) * inv_cell_size);
  const double lo_y = floor(static_cast<double>(min_y) * inv_cell_size);
  const double hi_x = floor(static_cast<double>(max_x) * inv_cell_size);
  const double hi_y = floor(static_cast<double>(max_y) * inv_cell_size);
  const double limit = static_cast<double>(kMaxProximityGridCell);
  if (lo_x < -limit || hi_x > limit || lo_y < -limit || hi_y > limit) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (hi_x - lo_x >= kMaxProximityGridSpan ||
      hi_y - lo_y >= kMaxProximityGridSpan) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// True for a float a caller may pass as a length, a weight or a duration.
inline bool IsPositiveFinite(float v) { return IsFinite(v) && v > 0.f; }
inline bool IsNonNegativeFinite(float v) { return IsFinite(v) && v >= 0.f; }

/// True for a world position this package will let reach upstream.
inline bool IsPosition(const float* p) {
  if (!IsFiniteVec3(p)) return false;
  for (int i = 0; i < 3; ++i) {
    if (p[i] < -kMaxCoordinate || p[i] > kMaxCoordinate) return false;
  }
  return true;
}

/// One agent's parameters, checked before dtCrowd stores them.
///
/// Every bound here is one upstream documents in a comment and enforces
/// nowhere; the two index fields are heap over-reads rather than wrong
/// numbers. ffi/zrecast.h names the call sites.
inline ZrcResult ValidateCrowdAgentParams(const ZrcCrowdAgentParams& params,
                                          float max_agent_radius) {
  if (!IsPositiveFinite(params.radius) ||
      params.radius > max_agent_radius) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsPositiveFinite(params.height)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!IsNonNegativeFinite(params.max_acceleration)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsPositiveFinite(params.max_speed)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!IsPositiveFinite(params.collision_query_range)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsPositiveFinite(params.path_optimization_range)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsNonNegativeFinite(params.separation_weight)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params.obstacle_avoidance_type >= ZRC_CROWD_MAX_AVOIDANCE_PARAMS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params.query_filter_type >= ZRC_CROWD_MAX_FILTERS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// The velocity sampler's configuration, checked before it is stored or used.
inline ZrcResult ValidateAvoidanceParams(const ZrcAvoidanceParams& params) {
  if (!IsFinite(params.vel_bias) || params.vel_bias < 0.f ||
      params.vel_bias > 1.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsNonNegativeFinite(params.weight_desired_vel) ||
      !IsNonNegativeFinite(params.weight_current_vel) ||
      !IsNonNegativeFinite(params.weight_side) ||
      !IsNonNegativeFinite(params.weight_toi)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!IsPositiveFinite(params.horiz_time)) return ZRC_ERR_INVALID_ARGUMENT;
  if (params.grid_size < 2) return ZRC_ERR_INVALID_ARGUMENT;
  if (params.adaptive_divs < 1 ||
      params.adaptive_divs > ZRC_AVOIDANCE_MAX_PATTERN_DIVS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params.adaptive_rings < 1 ||
      params.adaptive_rings > ZRC_AVOIDANCE_MAX_PATTERN_RINGS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params.adaptive_depth < 1) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// Copies the ZrcAvoidanceParams POD onto upstream's identically shaped
/// struct. Field by field rather than a memcpy: the two are laid out the same
/// today and nothing but this function would notice if that changed.
inline void ToDtAvoidanceParams(const ZrcAvoidanceParams& in,
                                dtObstacleAvoidanceParams* out) {
  out->velBias = in.vel_bias;
  out->weightDesVel = in.weight_desired_vel;
  out->weightCurVel = in.weight_current_vel;
  out->weightSide = in.weight_side;
  out->weightToi = in.weight_toi;
  out->horizTime = in.horiz_time;
  out->gridSize = in.grid_size;
  out->adaptiveDivs = in.adaptive_divs;
  out->adaptiveRings = in.adaptive_rings;
  out->adaptiveDepth = in.adaptive_depth;
}

inline void FromDtAvoidanceParams(const dtObstacleAvoidanceParams& in,
                                  ZrcAvoidanceParams* out) {
  out->vel_bias = in.velBias;
  out->weight_desired_vel = in.weightDesVel;
  out->weight_current_vel = in.weightCurVel;
  out->weight_side = in.weightSide;
  out->weight_toi = in.weightToi;
  out->horiz_time = in.horizTime;
  out->grid_size = in.gridSize;
  out->adaptive_divs = in.adaptiveDivs;
  out->adaptive_rings = in.adaptiveRings;
  out->adaptive_depth = in.adaptiveDepth;
}

/// Requests dtPathQueue can hold at once.
///
/// dtPathQueue::MAX_QUEUE is private (DetourPathQueue.h:46), so the number is
/// restated here rather than read from upstream.
const int kPathQueueSlots = 8;

//===----------------------------------------------------------------------===//
// Agent references
//
// Upstream identifies an agent by its pool slot and nothing else, so a slot
// reused after a removal answers to the old index. A reference carries the
// slot in its low 16 bits and a serial the crowd never reissues in the rest.
//===----------------------------------------------------------------------===//

const int kAgentSlotBits = 16;
const uint64_t kAgentSlotMask = 0xffffu;

inline ZrcAgentRef MakeAgentRef(int32_t slot, uint64_t serial) {
  return (serial << kAgentSlotBits) |
         (static_cast<uint64_t>(slot + 1) & kAgentSlotMask);
}

/// The slot a reference names, or -1 when it names none.
inline int32_t AgentRefSlot(ZrcAgentRef ref) {
  const uint64_t slot = ref & kAgentSlotMask;
  if (slot == 0) return -1;
  return static_cast<int32_t>(slot - 1);
}

inline uint64_t AgentRefSerial(ZrcAgentRef ref) { return ref >> kAgentSlotBits; }

}  // namespace zrc

//===----------------------------------------------------------------------===//
// Handle types (global namespace — they must match the C tag names)
//
// The C header forward-declares these as opaque tags; defining them as real C++
// types here means every accessor is statically typed. No reinterpret_cast
// crosses the boundary, so a handle mix-up is a compile error rather than
// memory corruption.
//===----------------------------------------------------------------------===//

/// Recast's polygon mesh and its detail mesh, kept together because Detour
/// needs both to build a tile, plus the agent dimensions in world units that
/// rcPolyMesh does not carry but dtNavMeshCreateParams demands.
struct ZrcPolyMesh {
  rcPolyMesh* poly;
  rcPolyMeshDetail* detail;
  float walkable_height;
  float walkable_radius;
  float walkable_climb;
  /// Whether the three dimensions above have been supplied.
  ///
  /// A bake fills them from its own configuration. A mesh assembled stage by
  /// stage has nowhere to get them from — Recast carries no agent dimensions —
  /// so zrcPolyMeshSetAgentDims is how one arrives, and a tile built from a
  /// mesh that never received them would describe an agent of size zero.
  bool has_agent_dims;
};

/// The voxelisation of the input geometry, and its span pools.
struct ZrcHeightfield {
  rcHeightfield* impl;
};

/// The walkable surface as open space rather than obstruction.
struct ZrcCompactHeightfield {
  rcCompactHeightfield* impl;
};

/// The traced outlines of every region.
struct ZrcContourSet {
  rcContourSet* impl;
};

/// The walkable surface cut into separately compressible sheets.
struct ZrcHeightfieldLayerSet {
  rcHeightfieldLayerSet* impl;
};

/// A dtNavMesh and the tile grid it was sized for.
///
/// A single-tile mesh is a 1x1 grid rather than a separate mode, so every tile
/// entry point works on both and there is one set of range checks.
struct ZrcNavMesh {
  dtNavMesh* impl;
  int32_t tile_count_x;
  int32_t tile_count_z;
};

/// A live tile cache: upstream's object, the three interfaces it borrows for
/// its whole lifetime, and the navmesh grid a rebuilt tile has to land in.
struct ZrcTileCache {
  dtTileCache* impl;
  zrc::HostCompressor compressor;
  zrc::HostTileCacheAlloc allocator;
  zrc::HostMeshProcess mesh_process;
};

/// One decompressed layer, and the allocator that owns its buffer.
///
/// dtFreeTileCacheLayer needs the same dtTileCacheAlloc that produced the
/// layer, so the handle carries it rather than asking the caller to hand the
/// right one back.
struct ZrcTileCacheLayer {
  dtTileCacheLayer* impl;
  zrc::HostTileCacheAlloc allocator;
};

struct ZrcTileCacheContourSet {
  dtTileCacheContourSet* impl;
  zrc::HostTileCacheAlloc allocator;
};

struct ZrcTileCachePolyMesh {
  dtTileCachePolyMesh* impl;
  zrc::HostTileCacheAlloc allocator;
};

struct ZrcNavMeshQuery;

/// A group of steered agents, and the bookkeeping upstream does not keep.
///
/// `serial` per slot is what makes a reference outlive nothing: addAgent
/// stamps the next value into the slot it took, removeAgent clears it, and a
/// reference resolves only when the two still agree. `next_serial` counts up
/// across re-initialisations as well, so a crowd pointed at a new navmesh
/// retires every reference minted against the old one.
struct ZrcCrowd {
  dtCrowd* impl;
  /// The navmesh the crowd plans against, borrowed.
  const ZrcNavMesh* navmesh;
  /// One per slot, 0 for a free slot. `capacity` entries.
  uint64_t* serial;
  int32_t capacity;
  uint64_t next_serial;
  /// Wrappers over the objects dtCrowd owns, handed out borrowed. Made once
  /// so a caller gets the same pointer every time and never a fresh
  /// allocation it might think it owns.
  ZrcProximityGrid* grid;
  ZrcPathQueue* path_queue;
  ZrcNavMeshQuery* query;
};

/// A spatial hash. `owns` is false for the one a crowd hands back.
struct ZrcProximityGrid {
  dtProximityGrid* impl;
  bool owns;
};

struct ZrcAvoidanceQuery {
  dtObstacleAvoidanceQuery* impl;
  int32_t max_circles;
  int32_t max_segments;
};

struct ZrcAvoidanceDebug {
  dtObstacleAvoidanceDebugData* impl;
  int32_t max_samples;
};

/// A corridor and the buffer length upstream will not report back.
///
/// dtPathCorridor keeps m_maxPath privately and offers no accessor, and every
/// bound this package places on setCorridor and the merge helpers needs it.
struct ZrcPathCorridor {
  dtPathCorridor* impl;
  int32_t max_path;
};

struct ZrcLocalBoundary {
  dtLocalBoundary* impl;
};

/// A queue and the navmesh its searches run against, borrowed. `owns` is
/// false for the one a crowd hands back.
struct ZrcPathQueue {
  dtPathQueue* impl;
  const ZrcNavMesh* navmesh;
  /// Handed back by zrcPathQueueNavMeshQuery, const only.
  ZrcNavMeshQuery* query;
  int32_t max_path_size;
  bool owns;
  /// One filter per queue slot, owned for the request's lifetime.
  ///
  /// dtPathQueue stores the caller's dtQueryFilter as a raw pointer and reads
  /// through it on every later update, which its own header calls
  /// "potentially dangerous" (DetourPathQueue.h:43). `filter_owner` records
  /// which request each is lent to, so a slot upstream has recycled after its
  /// keep-alive window can be reclaimed.
  dtQueryFilter filters[zrc::kPathQueueSlots];
  ZrcPathRequestRef filter_owner[zrc::kPathQueueSlots];
};

struct ZrcNavMeshQuery {
  dtNavMeshQuery* impl;
  /// The navmesh this query was created against, borrowed. zrcQueryNavMesh
  /// hands it back; the caller still owns it.
  const ZrcNavMesh* navmesh;
  /// The filter a sliced search runs under, owned for the slice's lifetime.
  ///
  /// No mutation probe covers this one, deliberately. Replacing this with a
  /// stack local reproduces upstream's hazard exactly — and the result is
  /// undefined behaviour rather than a wrong answer, so whether any test
  /// notices depends on what the next call happens to write over the dead
  /// frame. A probe that passes for that reason would be worse than none.
  ///
  /// dtNavMeshQuery::initSlicedFindPath stores the filter it is given as a raw
  /// pointer and every later update and finalise reads through it
  /// (DetourNavMeshQuery.cpp:1233). Every other entry point here builds a
  /// dtQueryFilter on the stack, which would dangle the moment init returned.
  dtQueryFilter slice_filter;
  /// Whether a sliced search is in flight.
  bool slicing;
};

#endif  // ZRECAST_INTERNAL_H_
