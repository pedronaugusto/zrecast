//===----------------------------------------------------------------------===//
// zrecast — the Recast bake: triangle soup in, polygon mesh out.
//
// This is the BUILD-TIME half of the package. Nothing here is meant to run in
// a frame; it belongs in a cook step whose output is serialised and shipped.
//===----------------------------------------------------------------------===//

#include <stdio.h>

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Build context
//
// Recast reports failure as a bare `false` and sends the reason to a log
// callback on rcContext. Without capturing that, "the bake failed" is the whole
// diagnosis. This subclass appends messages into the caller's buffer, so a
// failed bake can say *which* stage failed and why.
//===----------------------------------------------------------------------===//

class BakeContext : public rcContext {
 public:
  BakeContext(char* buffer, size_t capacity)
      : rcContext(true), buffer_(buffer), capacity_(capacity), used_(0) {
    // Timers cost a clock read per stage and nothing here reports them.
    enableTimer(false);
    if (buffer_ != nullptr && capacity_ != 0) buffer_[0] = '\0';
  }

  /// Records a stage failure in the same buffer the caller reads, so a `false`
  /// return from a Recast entry point that logged nothing still says something.
  void note(const char* text) { append(text, strlen(text)); }

 protected:
  void doLog(const rcLogCategory category, const char* msg,
             const int len) override {
    if (len <= 0) return;
    switch (category) {
      case RC_LOG_PROGRESS:
        // Progress chatter would drown the errors in a small buffer.
        return;
      case RC_LOG_WARNING:
        append("warning: ", 9);
        break;
      case RC_LOG_ERROR:
        append("error: ", 7);
        break;
    }
    append(msg, static_cast<size_t>(len));
    append("\n", 1);
  }

  void doResetLog() override {
    used_ = 0;
    if (buffer_ != nullptr && capacity_ != 0) buffer_[0] = '\0';
  }

 private:
  /// Appends what fits and keeps the buffer NUL-terminated. Truncation is
  /// silent by design: a diagnostic that overflows must not become a defect.
  void append(const char* text, size_t len) {
    if (buffer_ == nullptr || capacity_ == 0) return;
    const size_t room = capacity_ - 1 - used_;
    if (room == 0) return;
    const size_t n = len < room ? len : room;
    memcpy(buffer_ + used_, text, n);
    used_ += n;
    buffer_[used_] = '\0';
  }

  char* buffer_;
  size_t capacity_;
  size_t used_;
};

//===----------------------------------------------------------------------===//
// Input validation
//===----------------------------------------------------------------------===//

/// Rejects a configuration that would divide by zero, allocate a heightfield
/// the size of the world, or hand Detour a polygon it cannot represent.
ZrcResult ValidateConfig(const ZrcBakeConfig& config) {
  const float floats[] = {
      config.cell_size,        config.cell_height,
      config.agent_height,     config.agent_radius,
      config.agent_max_climb,  config.agent_max_slope,
      config.region_min_size,  config.region_merge_size,
      config.edge_max_len,     config.edge_max_error,
      config.detail_sample_dist, config.detail_sample_max_error,
  };
  for (size_t i = 0; i < sizeof(floats) / sizeof(floats[0]); ++i) {
    if (!zrc::IsFinite(floats[i])) return ZRC_ERR_INVALID_ARGUMENT;
  }

  if (!(config.cell_size > 0.f) || !(config.cell_height > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!(config.agent_height > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (config.agent_radius < 0.f || config.agent_max_climb < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // rcMarkWalkableTriangles takes the cosine of this; outside [0, 90) the
  // threshold stops meaning anything.
  if (config.agent_max_slope < 0.f || config.agent_max_slope >= 90.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (config.region_min_size < 0.f || config.region_merge_size < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (config.edge_max_len < 0.f || config.edge_max_error < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (config.verts_per_poly < 3 ||
      config.verts_per_poly > ZRC_VERTS_PER_POLYGON) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Recast documents 0 (off) or >= 0.9; anything between is a sampling grid
  // finer than a cell, which makes the detail build pathologically slow.
  if (config.detail_sample_dist < 0.f ||
      (config.detail_sample_dist > 0.f && config.detail_sample_dist < 0.9f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (config.detail_sample_max_error < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  if (config.partition != ZRC_PARTITION_WATERSHED &&
      config.partition != ZRC_PARTITION_MONOTONE &&
      config.partition != ZRC_PARTITION_LAYERS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // 0 is the single-tile sentinel; an explicit edge below Recast's own tiled
  // build minimum or above the header's documented ceiling is rejected.
  if (config.tile_size != 0 &&
      (config.tile_size < 16 || config.tile_size > 1024)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Negative derives the border and is always accepted; an explicit width is
  // bounded to what the header documents.
  if (config.border_size > zrc::kMaxBorderSizeCells) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Scratch that frees itself
//
// The pipeline holds four intermediates that are dead by the time it returns.
// Recast's own samples leak them on every early exit; a scope guard is cheaper
// than eleven cleanup labels.
//===----------------------------------------------------------------------===//

template <typename T>
class RcOwned {
 public:
  RcOwned() : ptr_(zrc::RcNew<T>()) {}
  ~RcOwned() { zrc::RcFree(ptr_); }

  T* get() const { return ptr_; }
  explicit operator bool() const { return ptr_ != nullptr; }

  /// Hands ownership to the caller.
  T* release() {
    T* out = ptr_;
    ptr_ = nullptr;
    return out;
  }

 private:
  RcOwned(const RcOwned&);
  RcOwned& operator=(const RcOwned&);
  T* ptr_;
};

/// Shared with zrecast_navmesh.cpp and the staged pipeline.
using zrc::TempBuffer;

/// Largest value a float can hold that is still below INT_MAX, so converting it
/// to int is defined.
const float kMaxIntAsFloat = 2147483520.0f;

/// Largest voxel count zrecast will accept along one axis, and the largest
/// voxel length it will pass to Recast.
///
/// Both bounds live in zrecast_internal.h, shared with the staged pipeline's
/// entry points, which apply the same rules to a grid a host sizes itself.
using zrc::kMaxAxisCells;
using zrc::kMaxWalkableRadiusCells;

/// Converts a float to an int without overflowing.
///
/// Every quantity derived below is a caller-supplied world length divided by a
/// caller-supplied cell size, and a float-to-int conversion whose value does
/// not fit the destination is undefined behaviour — not a wrap. So "does not
/// fit" is one bad configuration away rather than a theoretical concern, and
/// Zig's C sanitizer traps it (__ubsan_handle_float_cast_overflow) precisely
/// because it is real.
///
/// Saturating instead is safe in every use here: each result is a voxel count
/// or a voxel area, and a saturated one makes Recast produce an empty or
/// rejected build, which is the honest answer to an absurd configuration.
int SaturateToInt(float v) {
  // Also catches NaN, which every caller has already excluded, but this is the
  // function that must not be wrong.
  if (!(v > 0.f)) return 0;
  if (v >= kMaxIntAsFloat) return static_cast<int>(kMaxIntAsFloat);
  return static_cast<int>(v);
}

/// Converts a world-unit length to a whole number of voxels, rounding the way
/// Recast's own samples do for each quantity.
int VoxelsCeil(float world, float cell) {
  return SaturateToInt(ceilf(world / cell));
}

int VoxelsFloor(float world, float cell) {
  return SaturateToInt(floorf(world / cell));
}

//===----------------------------------------------------------------------===//
// The pipeline shared by a whole-world bake and one tile of a tiled bake.
//===----------------------------------------------------------------------===//

/// The half of an rcConfig that does not depend on which bake it is: the agent
/// dimensions converted to voxels, and the simplification limits.
///
/// Bounds and grid size are the caller's, because they are the only part a tile
/// and a whole world disagree about. Sharing the rest means a correction to one
/// of these conversions cannot reach one entry point and miss the other.
ZrcResult FillAgentConfig(const ZrcBakeConfig& config, rcConfig* out,
                          BakeContext& ctx) {
  rcConfig& cfg = *out;
  cfg.cs = config.cell_size;
  cfg.ch = config.cell_height;
  cfg.walkableSlopeAngle = config.agent_max_slope;
  cfg.walkableHeight = VoxelsCeil(config.agent_height, config.cell_height);
  cfg.walkableClimb = VoxelsFloor(config.agent_max_climb, config.cell_height);
  cfg.walkableRadius = VoxelsCeil(config.agent_radius, config.cell_size);
  // Clamped rather than rejected: an edge-length limit longer than the whole
  // grid means "never subdivide", which is exactly what the clamped value does.
  cfg.maxEdgeLen = SaturateToInt(config.edge_max_len / config.cell_size);
  if (cfg.maxEdgeLen > kMaxAxisCells) cfg.maxEdgeLen = kMaxAxisCells;
  cfg.maxSimplificationError = config.edge_max_error;
  cfg.minRegionArea =
      SaturateToInt(config.region_min_size * config.region_min_size);
  cfg.mergeRegionArea =
      SaturateToInt(config.region_merge_size * config.region_merge_size);
  cfg.maxVertsPerPoly = config.verts_per_poly;
  cfg.detailSampleDist = config.detail_sample_dist < 0.9f
                             ? 0.f
                             : config.cell_size * config.detail_sample_dist;
  cfg.detailSampleMaxError =
      config.cell_height * config.detail_sample_max_error;

  // rcBuildCompactHeightfield requires at least three voxels of clearance for
  // the walkable-height test to mean anything.
  if (cfg.walkableHeight < 3) {
    ctx.note(
        "agent_height is less than three voxels of cell_height; lower "
        "cell_height or raise agent_height\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // The agent dimensions are rejected rather than clamped when they leave the
  // range Recast's own storage can represent, because clamping would silently
  // bake a mesh for a different agent than the caller asked for.
  //
  // Each bound is upstream's, not invented here: span extents are 13-bit
  // (RC_SPAN_MAX_HEIGHT), and rcErodeWalkableArea computes its threshold as
  // `(unsigned char)(radius * 2)` — which both overflows an int for a large
  // radius and wraps in a byte well before that.
  if (cfg.walkableHeight > RC_SPAN_MAX_HEIGHT ||
      cfg.walkableClimb > RC_SPAN_MAX_HEIGHT) {
    ctx.note(
        "agent_height or agent_max_climb is more voxels tall than a Recast "
        "span can represent; raise cell_height\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (cfg.walkableRadius > kMaxWalkableRadiusCells) {
    ctx.note(
        "agent_radius is more voxels wide than Recast's erosion threshold can "
        "represent; raise cell_size\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  return ZRC_OK;
}

/// Marks every triangle whose face normal is no steeper than `slope_degrees`.
///
/// Both markers live in zrecast_internal.h, shared with the staged pipeline's
/// entry points so that a cook driven a stage at a time is the same bytes on
/// every platform a whole bake is.
using zrc::MarkWalkableTriangles;

/// The voxel grid rcCalcGridSize would compute, bounded before it computes it.
///
/// rcCalcGridSize is `(int)((bmax - bmin) / cs + 0.5f)` (Recast.cpp:302-303),
/// and that conversion is undefined — not merely wrong — once the quotient
/// exceeds INT_MAX, which a large world and a small cell_size reach easily.
/// Doing the arithmetic in double means the check happens where it is still
/// safe, so every caller of rcCalcGridSize goes through here first.
ZrcResult CheckGridExtent(const float* bmin, const float* bmax, float cs,
                          BakeContext& ctx) {
  switch (zrc::CheckGridExtentFit(bmin, bmax, cs)) {
    case zrc::GridExtent::kOk:
      return ZRC_OK;
    case zrc::GridExtent::kTooCoarse:
      ctx.note("cell_size is larger than the geometry's extent\n");
      return ZRC_ERR_INVALID_ARGUMENT;
    case zrc::GridExtent::kTooFine:
      ctx.note(
          "cell_size is too small for the geometry's extent; a mesh this "
          "large needs to be tiled\n");
      return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_ERR_INVALID_ARGUMENT;
}

/// The vertical extent, checked against what a Recast span can address.
ZrcResult CheckVerticalExtent(const rcConfig& cfg, BakeContext& ctx) {
  if (!zrc::VerticalExtentFits(cfg.bmin[1], cfg.bmax[1], cfg.ch)) {
    ctx.note(
        "the geometry's vertical extent is more voxels tall than a Recast "
        "span can address; raise cell_height\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// Runs heightfield -> filters -> compact heightfield -> erode -> regions ->
/// contours -> polygon mesh -> detail mesh over an already-filled `cfg`.
///
/// `config` supplies the fields `cfg` does not carry: the filter toggles, the
/// partition choice, and the world-unit scale the resulting handle's agent
/// dimensions are reported in. `authoring` may be NULL; when given, its
/// volumes are marked into the compact heightfield and its area-to-flag table
/// decides `poly->flags`. `empty_is_ok` is the one place a whole-world bake
/// and a tile bake disagree: zero polygons is ZRC_ERR_EMPTY_RESULT for the
/// former and ZRC_OK with `*out` left NULL for the latter. On any other
/// failure `*out` is left NULL.
ZrcResult RunBakePipeline(const rcConfig& cfg, const ZrcBakeConfig& config,
                          const ZrcTriMesh& mesh,
                          const ZrcAreaAuthoring* authoring, BakeContext& ctx,
                          bool empty_is_ok, ZrcPolyMesh** out) {
  //---------------------------------------------------------------------------
  // Stage 1: rasterise the soup into a solid heightfield.
  //---------------------------------------------------------------------------

  RcOwned<rcHeightfield> hf;
  if (!hf) return ZRC_ERR_OUT_OF_MEMORY;
  if (!rcCreateHeightfield(&ctx, *hf.get(), cfg.width, cfg.height, cfg.bmin,
                           cfg.bmax, cfg.cs, cfg.ch)) {
    ctx.note("rcCreateHeightfield failed\n");
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  {
    TempBuffer areas(static_cast<size_t>(mesh.tri_count));
    if (areas.get() == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
    unsigned char* tri_areas = static_cast<unsigned char*>(areas.get());
    memset(tri_areas, 0, static_cast<size_t>(mesh.tri_count));

    MarkWalkableTriangles(cfg.walkableSlopeAngle, mesh, tri_areas);
    if (!rcRasterizeTriangles(&ctx, mesh.verts, mesh.vert_count, mesh.tris,
                              tri_areas, mesh.tri_count, *hf.get(),
                              cfg.walkableClimb)) {
      ctx.note("rcRasterizeTriangles failed\n");
      return ZRC_ERR_OUT_OF_MEMORY;
    }
  }

  //---------------------------------------------------------------------------
  // Stage 2: filter out what the agent cannot stand on.
  //---------------------------------------------------------------------------

  if (config.filter_low_hanging_obstacles) {
    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf.get());
  }
  if (config.filter_ledge_spans) {
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf.get());
  }
  if (config.filter_walkable_low_height_spans) {
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf.get());
  }

  //---------------------------------------------------------------------------
  // Stage 3: compact heightfield, eroded by the agent radius.
  //---------------------------------------------------------------------------

  RcOwned<rcCompactHeightfield> chf;
  if (!chf) return ZRC_ERR_OUT_OF_MEMORY;
  if (!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb,
                                 *hf.get(), *chf.get())) {
    ctx.note("rcBuildCompactHeightfield failed\n");
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  if (cfg.walkableRadius > 0 &&
      !rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf.get())) {
    ctx.note("rcErodeWalkableArea failed\n");
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  //---------------------------------------------------------------------------
  // Authored volumes: colour the walkable surface with area ids, in array
  // order, so a later volume overwrites an earlier one where they overlap.
  //
  // Applied here because the surface has already been eroded by the agent
  // radius, so a volume colours only surface an agent could actually stand
  // on, and because it runs before the surface is cut into regions, so a
  // region never spans two areas.
  //---------------------------------------------------------------------------

  if (authoring != nullptr) {
    zrc::MarkAreaVolumes(ctx, *authoring, *chf.get());
  }

  //---------------------------------------------------------------------------
  // Stage 4: partition the walkable surface into regions.
  //---------------------------------------------------------------------------

  switch (config.partition) {
    case ZRC_PARTITION_WATERSHED:
      if (!rcBuildDistanceField(&ctx, *chf.get())) {
        ctx.note("rcBuildDistanceField failed\n");
        return ZRC_ERR_OUT_OF_MEMORY;
      }
      if (!rcBuildRegions(&ctx, *chf.get(), cfg.borderSize, cfg.minRegionArea,
                          cfg.mergeRegionArea)) {
        ctx.note("rcBuildRegions failed\n");
        return ZRC_ERR_BAKE_FAILED;
      }
      break;
    case ZRC_PARTITION_MONOTONE:
      if (!rcBuildRegionsMonotone(&ctx, *chf.get(), cfg.borderSize,
                                  cfg.minRegionArea, cfg.mergeRegionArea)) {
        ctx.note("rcBuildRegionsMonotone failed\n");
        return ZRC_ERR_BAKE_FAILED;
      }
      break;
    case ZRC_PARTITION_LAYERS:
      if (!rcBuildLayerRegions(&ctx, *chf.get(), cfg.borderSize,
                               cfg.minRegionArea)) {
        ctx.note("rcBuildLayerRegions failed\n");
        return ZRC_ERR_BAKE_FAILED;
      }
      break;
    default:
      // Unreachable: ValidateConfig has already rejected anything else.
      return ZRC_ERR_INVALID_ARGUMENT;
  }

  //---------------------------------------------------------------------------
  // Stage 5: trace contours, then turn them into convex polygons.
  //---------------------------------------------------------------------------

  RcOwned<rcContourSet> cset;
  if (!cset) return ZRC_ERR_OUT_OF_MEMORY;
  if (!rcBuildContours(&ctx, *chf.get(), cfg.maxSimplificationError,
                       cfg.maxEdgeLen, *cset.get())) {
    ctx.note("rcBuildContours failed\n");
    return ZRC_ERR_BAKE_FAILED;
  }

  RcOwned<rcPolyMesh> pmesh;
  if (!pmesh) return ZRC_ERR_OUT_OF_MEMORY;
  if (!rcBuildPolyMesh(&ctx, *cset.get(), cfg.maxVertsPerPoly, *pmesh.get())) {
    ctx.note("rcBuildPolyMesh failed\n");
    return ZRC_ERR_BAKE_FAILED;
  }

  //---------------------------------------------------------------------------
  // Stage 6: the detail mesh, which restores the height Recast quantised away.
  //---------------------------------------------------------------------------

  RcOwned<rcPolyMeshDetail> dmesh;
  if (!dmesh) return ZRC_ERR_OUT_OF_MEMORY;
  if (!rcBuildPolyMeshDetail(&ctx, *pmesh.get(), *chf.get(),
                             cfg.detailSampleDist, cfg.detailSampleMaxError,
                             *dmesh.get())) {
    ctx.note("rcBuildPolyMeshDetail failed\n");
    return ZRC_ERR_BAKE_FAILED;
  }

  //---------------------------------------------------------------------------
  // An empty mesh is not a failure upstream, but it is never what the caller
  // wanted, and it becomes a navmesh that silently answers every query with
  // "nowhere to go".
  //---------------------------------------------------------------------------

  if (pmesh.get()->npolys <= 0) {
    if (empty_is_ok) return ZRC_OK;
    ctx.note(
        "the bake produced no walkable polygons: check cell_size against the "
        "scale of the geometry, and that some surface is flatter than "
        "agent_max_slope\n");
    return ZRC_ERR_EMPTY_RESULT;
  }

  //---------------------------------------------------------------------------
  // Flags. Recast leaves them zero; the default Detour filter admits a polygon
  // only if it shares a bit with include_flags, so a zero-flag mesh is
  // invisible to every query. With no area-to-flag table every walkable
  // polygon gets ZRC_POLY_FLAG_WALKABLE, which is what makes the result usable
  // without a second, undocumented step. With one, each polygon's flags come
  // from its area id; the table has ZRC_MAX_AREAS entries and `areas[i]` is
  // read as an index into it, so an id past the table falls back to 0 rather
  // than reading out of bounds.
  //---------------------------------------------------------------------------

  rcPolyMesh* poly = pmesh.get();
  const uint16_t* area_flags =
      authoring != nullptr ? authoring->area_flags : nullptr;
  for (int i = 0; i < poly->npolys; ++i) {
    if (area_flags != nullptr) {
      poly->flags[i] =
          poly->areas[i] < ZRC_MAX_AREAS ? area_flags[poly->areas[i]] : 0;
    } else {
      poly->flags[i] =
          poly->areas[i] == RC_NULL_AREA ? 0 : ZRC_POLY_FLAG_WALKABLE;
    }
  }

  ZrcPolyMesh* handle = zrc::New<ZrcPolyMesh>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->poly = pmesh.release();
  handle->detail = dmesh.release();
  // Detour wants these back in world units, and quantised to the voxel grid
  // the mesh was actually built on rather than to what was asked for.
  handle->walkable_height =
      static_cast<float>(cfg.walkableHeight) * config.cell_height;
  handle->walkable_radius =
      static_cast<float>(cfg.walkableRadius) * config.cell_size;
  handle->walkable_climb =
      static_cast<float>(cfg.walkableClimb) * config.cell_height;
  handle->has_agent_dims = true;

  *out = handle;
  return ZRC_OK;
}

}  // namespace

extern "C" {

void zrcBakeConfigDefault(ZrcBakeConfig* out) {
  if (out == nullptr) return;
  // Recast's solo-mesh sample defaults, which describe a human-sized agent on
  // metre-scale geometry.
  out->cell_size = 0.3f;
  out->cell_height = 0.2f;
  out->agent_height = 2.0f;
  out->agent_radius = 0.6f;
  out->agent_max_climb = 0.9f;
  out->agent_max_slope = 45.0f;
  out->region_min_size = 8.0f;
  out->region_merge_size = 20.0f;
  out->edge_max_len = 12.0f;
  out->edge_max_error = 1.3f;
  out->verts_per_poly = ZRC_VERTS_PER_POLYGON;
  out->detail_sample_dist = 6.0f;
  out->detail_sample_max_error = 1.0f;
  out->partition = ZRC_PARTITION_WATERSHED;
  out->filter_low_hanging_obstacles = ZRC_TRUE;
  out->filter_ledge_spans = ZRC_TRUE;
  out->filter_walkable_low_height_spans = ZRC_TRUE;
  out->tile_size = 0;
  out->border_size = -1;
}

ZrcResult zrcBakeConfigCells(const ZrcBakeConfig* config, ZrcBuildCells* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (config == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult config_valid = ValidateConfig(*config);
  if (config_valid != ZRC_OK) return config_valid;

  // The same conversion RunBakePipeline runs, through the same helper, so a
  // host driving the stages by hand and zrcPolyMeshBake quantise identically.
  // The log goes nowhere: FillAgentConfig's notes name the offending field,
  // and this entry point reports only whether the configuration converts.
  BakeContext ctx(nullptr, 0);
  rcConfig cfg;
  memset(&cfg, 0, sizeof(cfg));
  const ZrcResult agent_valid = FillAgentConfig(*config, &cfg, ctx);
  if (agent_valid != ZRC_OK) return agent_valid;

  out->walkable_height = cfg.walkableHeight;
  out->walkable_climb = cfg.walkableClimb;
  out->walkable_radius = cfg.walkableRadius;
  out->max_edge_len = cfg.maxEdgeLen;
  out->min_region_area = cfg.minRegionArea;
  out->merge_region_area = cfg.mergeRegionArea;
  // An untiled build has no neighbouring tile for a contour to meet, so it
  // carries no border; a tiled one derives the same width zrcPolyMeshBakeTile
  // does.
  out->border_size = config->tile_size == 0
                         ? 0
                         : (config->border_size < 0
                                ? cfg.walkableRadius + 3
                                : config->border_size);
  out->max_simplification_error = cfg.maxSimplificationError;
  out->verts_per_poly = cfg.maxVertsPerPoly;
  out->detail_sample_dist = cfg.detailSampleDist;
  out->detail_sample_max_error = cfg.detailSampleMaxError;
  return ZRC_OK;
}

ZrcResult zrcTileGridCompute(const ZrcBakeConfig* config,
                             const ZrcTriMesh* mesh, ZrcTileGrid* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (config == nullptr || mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult config_valid = ValidateConfig(*config);
  if (config_valid != ZRC_OK) return config_valid;
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) return mesh_valid;

  // A single-tile bake has no grid; zrcPolyMeshBake covers that case.
  if (config->tile_size == 0) return ZRC_ERR_INVALID_ARGUMENT;

  float bmin[3];
  float bmax[3];
  rcCalcBounds(mesh->verts, mesh->vert_count, bmin, bmax);

  // No log to write into here, but the same bound: the grid this reports is
  // the grid a bake will walk, so it has to be a grid rcCalcGridSize can
  // compute at all.
  BakeContext ctx(nullptr, 0);
  const ZrcResult extent_valid =
      CheckGridExtent(bmin, bmax, config->cell_size, ctx);
  if (extent_valid != ZRC_OK) return extent_valid;

  int grid_width = 0;
  int grid_height = 0;
  rcCalcGridSize(bmin, bmax, config->cell_size, &grid_width, &grid_height);

  // Upstream's own tile-count formula, so the grid this reports and the voxel
  // grid a bake actually walks cannot disagree through float rounding.
  int tile_count_x = (grid_width + config->tile_size - 1) / config->tile_size;
  int tile_count_z = (grid_height + config->tile_size - 1) / config->tile_size;
  if (tile_count_x < 1) tile_count_x = 1;
  if (tile_count_z < 1) tile_count_z = 1;

  out->origin[0] = bmin[0];
  out->origin[1] = bmin[1];
  out->origin[2] = bmin[2];
  out->extent_max[0] = bmax[0];
  out->extent_max[1] = bmax[1];
  out->extent_max[2] = bmax[2];
  out->tile_world_size = static_cast<float>(config->tile_size) * config->cell_size;
  out->tile_count_x = tile_count_x;
  out->tile_count_z = tile_count_z;
  return ZRC_OK;
}

ZrcResult zrcPolyMeshBake(const ZrcBakeConfig* config, const ZrcTriMesh* mesh,
                          const ZrcAreaAuthoring* authoring, ZrcBakeLog* log,
                          ZrcPolyMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (config == nullptr || mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (log != nullptr && log->buffer == nullptr && log->capacity != 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  BakeContext ctx(log != nullptr ? log->buffer : nullptr,
                  log != nullptr ? log->capacity : 0);

  const ZrcResult config_valid = ValidateConfig(*config);
  if (config_valid != ZRC_OK) {
    ctx.note("invalid bake configuration\n");
    return config_valid;
  }
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) {
    ctx.note("invalid input geometry\n");
    return mesh_valid;
  }
  if (authoring != nullptr) {
    const ZrcResult authoring_valid = zrc::ValidateAreaAuthoring(*authoring);
    if (authoring_valid != ZRC_OK) {
      ctx.note("invalid area authoring\n");
      return authoring_valid;
    }
  }

  //---------------------------------------------------------------------------
  // Configuration, in voxels.
  //---------------------------------------------------------------------------

  rcConfig cfg;
  memset(&cfg, 0, sizeof(cfg));
  const ZrcResult agent_valid = FillAgentConfig(*config, &cfg, ctx);
  if (agent_valid != ZRC_OK) return agent_valid;
  // No border ring: this bake covers the whole input, so there is no
  // neighbouring tile for a contour to meet. zrcPolyMeshBakeTile is the entry
  // point that sets both.
  cfg.tileSize = 0;
  cfg.borderSize = 0;

  rcCalcBounds(mesh->verts, mesh->vert_count, cfg.bmin, cfg.bmax);

  const ZrcResult extent_valid =
      CheckGridExtent(cfg.bmin, cfg.bmax, cfg.cs, ctx);
  if (extent_valid != ZRC_OK) return extent_valid;
  const ZrcResult vertical_valid = CheckVerticalExtent(cfg, ctx);
  if (vertical_valid != ZRC_OK) return vertical_valid;

  rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);
  if (cfg.width <= 0 || cfg.height <= 0) {
    ctx.note("cell_size is larger than the geometry's extent\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  return RunBakePipeline(cfg, *config, *mesh, authoring, ctx,
                         /*empty_is_ok=*/false, out);
}

ZrcResult zrcPolyMeshBakeTile(const ZrcBakeConfig* config,
                              const ZrcTriMesh* mesh, const ZrcTileGrid* grid,
                              int32_t tile_x, int32_t tile_z,
                              const ZrcAreaAuthoring* authoring,
                              ZrcBakeLog* log, ZrcPolyMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (config == nullptr || mesh == nullptr || grid == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (log != nullptr && log->buffer == nullptr && log->capacity != 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  BakeContext ctx(log != nullptr ? log->buffer : nullptr,
                  log != nullptr ? log->capacity : 0);

  const ZrcResult config_valid = ValidateConfig(*config);
  if (config_valid != ZRC_OK) {
    ctx.note("invalid bake configuration\n");
    return config_valid;
  }
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) {
    ctx.note("invalid input geometry\n");
    return mesh_valid;
  }
  if (authoring != nullptr) {
    const ZrcResult authoring_valid = zrc::ValidateAreaAuthoring(*authoring);
    if (authoring_valid != ZRC_OK) {
      ctx.note("invalid area authoring\n");
      return authoring_valid;
    }
  }

  const ZrcResult grid_valid = zrc::ValidateTileGrid(*grid);
  if (grid_valid != ZRC_OK) {
    ctx.note("the tile grid is not one zrcTileGridCompute could have made\n");
    return grid_valid;
  }
  if (config->tile_size == 0) {
    ctx.note("tile_size is 0; zrcPolyMeshBake bakes a single tile instead\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tile_x < 0 || tile_x >= grid->tile_count_x || tile_z < 0 ||
      tile_z >= grid->tile_count_z) {
    ctx.note("tile_x or tile_z is outside the tile grid\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  //---------------------------------------------------------------------------
  // Configuration, in voxels.
  //---------------------------------------------------------------------------

  rcConfig cfg;
  memset(&cfg, 0, sizeof(cfg));
  const ZrcResult agent_valid = FillAgentConfig(*config, &cfg, ctx);
  if (agent_valid != ZRC_OK) return agent_valid;

  // Negative derives the border from the walkable radius, matching Recast's
  // own tiled build; an explicit width passes through unchanged.
  cfg.tileSize = config->tile_size;
  cfg.borderSize =
      config->border_size < 0 ? cfg.walkableRadius + 3 : config->border_size;
  cfg.width = cfg.tileSize + cfg.borderSize * 2;
  cfg.height = cfg.tileSize + cfg.borderSize * 2;

  // walkableRadius is already bounded to kMaxWalkableRadiusCells above, so a
  // derived border cannot overflow — but tile_size and an explicit
  // border_size can still combine past kMaxAxisCells, the same per-axis voxel
  // limit the whole-world path bounds rcCalcGridSize's result to.
  if (cfg.width > kMaxAxisCells || cfg.height > kMaxAxisCells) {
    ctx.note(
        "tile_size plus twice border_size is larger than Recast's per-axis "
        "voxel limit; lower tile_size or border_size\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const float tile_edge = static_cast<float>(cfg.tileSize) * cfg.cs;

  // The grid and the bake carry the tile edge separately, and nothing else
  // compares them. zrcTileGridCompute derived tile_world_size from the config
  // it was given, and zrcNavMeshCreateTiled lays the navmesh out on that
  // value; baking with a different tile_size or cell_size cooks tiles at one
  // spacing under a navmesh addressing them at another, and every later call
  // succeeds. Both sides evaluate the same product of the same two config
  // fields, so they agree bit for bit when they agree at all and a tolerance
  // would only let a drift through.
  if (tile_edge != grid->tile_world_size) {
    ctx.note(
        "tile_size times cell_size is not the grid's tile_world_size; the "
        "grid was computed from a different configuration\n");
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  cfg.bmin[0] = grid->origin[0] + static_cast<float>(tile_x) * tile_edge;
  cfg.bmin[1] = grid->origin[1];
  cfg.bmin[2] = grid->origin[2] + static_cast<float>(tile_z) * tile_edge;
  cfg.bmax[0] = grid->origin[0] + static_cast<float>(tile_x + 1) * tile_edge;
  cfg.bmax[1] = grid->extent_max[1];
  cfg.bmax[2] = grid->origin[2] + static_cast<float>(tile_z + 1) * tile_edge;

  const float border_world = static_cast<float>(cfg.borderSize) * cfg.cs;
  cfg.bmin[0] -= border_world;
  cfg.bmin[2] -= border_world;
  cfg.bmax[0] += border_world;
  cfg.bmax[2] += border_world;
  const ZrcResult vertical_valid = CheckVerticalExtent(cfg, ctx);
  if (vertical_valid != ZRC_OK) return vertical_valid;

  return RunBakePipeline(cfg, *config, *mesh, authoring, ctx,
                         /*empty_is_ok=*/true, out);
}

void zrcPolyMeshDestroy(ZrcPolyMesh* mesh) {
  if (mesh == nullptr) return;
  zrc::RcFree(mesh->poly);
  zrc::RcFree(mesh->detail);
  zrc::Delete(mesh);
}

int32_t zrcPolyMeshPolyCount(const ZrcPolyMesh* mesh) {
  return mesh == nullptr ? 0 : mesh->poly->npolys;
}

int32_t zrcPolyMeshVertCount(const ZrcPolyMesh* mesh) {
  return mesh == nullptr ? 0 : mesh->poly->nverts;
}

int32_t zrcPolyMeshDetailVertCount(const ZrcPolyMesh* mesh) {
  return mesh == nullptr ? 0 : mesh->detail->nverts;
}

int32_t zrcPolyMeshDetailTriCount(const ZrcPolyMesh* mesh) {
  return mesh == nullptr ? 0 : mesh->detail->ntris;
}

ZrcResult zrcPolyMeshBounds(const ZrcPolyMesh* mesh, float* bmin, float* bmax) {
  if (mesh == nullptr || bmin == nullptr || bmax == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    bmin[i] = mesh->poly->bmin[i];
    bmax[i] = mesh->poly->bmax[i];
  }
  return ZRC_OK;
}

}  // extern "C"
