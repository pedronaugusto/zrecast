//===----------------------------------------------------------------------===//
// zrecast — the staged Recast pipeline's containers: the heightfield, the
// compact heightfield, the contour set and the polygon mesh, plus every
// accessor onto them.
//
// This file owns the lifetime of the four container handles — allocation
// fused with the build that fills them, or with an empty start for
// zrcPolyMeshCreate — and the range accessors that read and write their
// arrays. The mutating build stages that run *between* creation and
// destruction (rasterisation, filters, erosion, region building, the polygon
// and detail-mesh builders) live elsewhere; this file only ever calls the one
// upstream function that fills a container at the moment it is created.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Argument validation
//===----------------------------------------------------------------------===//

ZrcResult ValidateHeightfieldCreateArgs(int32_t width, int32_t height,
                                        const float* bmin, const float* bmax,
                                        float cell_size, float cell_height) {
  // The same per-axis and total ceilings zrc::CheckGridExtentFit applies to a
  // grid derived from a mesh. rcCreateHeightfield allocates
  // `sizeof(rcSpan*) * width * height` with the product taken in plain `int`
  // (Recast.cpp:318), so two sizes that individually look modest overflow
  // before the allocation is attempted; and every later index into that array
  // is `x + z * width` in `int` as well.
  if (width <= 0 || width > zrc::kMaxAxisCells) return ZRC_ERR_INVALID_ARGUMENT;
  if (height <= 0 || height > zrc::kMaxAxisCells) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (static_cast<int64_t>(width) * static_cast<int64_t>(height) >
      static_cast<int64_t>(1) << 28) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(bmin) || !zrc::IsFiniteVec3(bmax)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (bmin[i] < -zrc::kMaxCoordinate || bmin[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (bmax[i] < -zrc::kMaxCoordinate || bmax[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (bmin[i] > bmax[i]) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cell_size) || !(cell_size > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cell_height) || !(cell_height > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // The vertical extent is the one dimension Recast never turns into a grid
  // size, and the one a rasteriser converts to a cell index without clamping
  // first. The bake bounds it from the geometry; a field a host sizes itself
  // is bounded here, by the same rule.
  if (!zrc::VerticalExtentFits(bmin[1], bmax[1], cell_height)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

// rcBuildCompactHeightfield (Recast.cpp:403-536 in this vendored tree — the
// upstream source keeps the compact-heightfield builder in Recast.cpp itself
// rather than a separate RecastCompact.cpp) takes walkableHeight and
// walkableClimb as plain `int` parameters. Neither is stored in a bitfield
// anywhere reachable from this call: rcCompactHeightfield::walkableHeight and
// ::walkableClimb are themselves full `int` fields, so there is no packed
// width for a value to be truncated into.
//
// The one real hazard is Recast.cpp:423 —
//   compactHeightfield.bmax[1] += walkableHeight * heightfield.ch;
// — a float computation that can push bmax[1] toward infinity for a large
// enough walkableHeight, which then propagates into every stage downstream
// that trusts bmax. The bound applied here is [0, ZRC_SPAN_MAX_HEIGHT]: the
// same ceiling a span's own vertical extent already carries elsewhere in this
// API (zrcHeightfieldAddSpan's span_max limit), which keeps a walkable height
// consistent with the tallest span it could ever actually measure against and
// keeps the multiplication well inside float range for any realistic
// cell_height.
ZrcResult ValidateCompactHeightfieldCreateArgs(int32_t walkable_height,
                                               int32_t walkable_climb) {
  if (walkable_height < 0 || walkable_height > ZRC_SPAN_MAX_HEIGHT) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (walkable_climb < 0 || walkable_climb > ZRC_SPAN_MAX_HEIGHT) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

ZrcResult ValidateContourSetCreateArgs(float max_error, int32_t max_edge_len,
                                       int32_t flags) {
  if (!zrc::IsFinite(max_error) || max_error < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_edge_len < 0) return ZRC_ERR_INVALID_ARGUMENT;
  const int32_t known_flags =
      ZRC_CONTOUR_TESS_WALL_EDGES | ZRC_CONTOUR_TESS_AREA_EDGES;
  if ((flags & ~known_flags) != 0) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// The heightfield
//===----------------------------------------------------------------------===//

ZrcResult zrcHeightfieldCreate(const ZrcBuildContext* context, int32_t width,
                               int32_t height, const float* bmin,
                               const float* bmax, float cell_size,
                               float cell_height, ZrcHeightfield** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult args_ok = ValidateHeightfieldCreateArgs(
      width, height, bmin, bmax, cell_size, cell_height);
  if (args_ok != ZRC_OK) return args_ok;

  rcHeightfield* impl = zrc::RcNew<rcHeightfield>();
  if (impl == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  zrc::HostContext ctx(context);
  if (!rcCreateHeightfield(&ctx, *impl, width, height, bmin, bmax, cell_size,
                           cell_height)) {
    zrc::RcFree(impl);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  ZrcHeightfield* handle = zrc::New<ZrcHeightfield>();
  if (handle == nullptr) {
    zrc::RcFree(impl);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->impl = impl;
  *out = handle;
  return ZRC_OK;
}

void zrcHeightfieldDestroy(ZrcHeightfield* heightfield) {
  if (heightfield == nullptr) return;
  zrc::RcFree(heightfield->impl);
  zrc::Delete(heightfield);
}

ZrcResult zrcHeightfieldInfo(const ZrcHeightfield* heightfield,
                             ZrcHeightfieldInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcHeightfield& hf = *heightfield->impl;
  out->width = hf.width;
  out->height = hf.height;
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = hf.bmin[i];
    out->bmax[i] = hf.bmax[i];
  }
  out->cell_size = hf.cs;
  out->cell_height = hf.ch;
  return ZRC_OK;
}

ZrcResult zrcHeightfieldStorage(const ZrcHeightfield* heightfield,
                                ZrcHeightfieldStorage* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcHeightfield& hf = *heightfield->impl;
  int32_t pool_count = 0;
  for (const rcSpanPool* pool = hf.pools; pool != nullptr; pool = pool->next) {
    ++pool_count;
  }
  int32_t free_count = 0;
  for (const rcSpan* span = hf.freelist; span != nullptr; span = span->next) {
    ++free_count;
  }
  out->pool_count = pool_count;
  out->free_count = free_count;
  out->spans_per_pool = ZRC_SPANS_PER_POOL;
  return ZRC_OK;
}

// `x + z * width` in plain `int`, the same arithmetic upstream's own addSpan
// performs (RecastRasterization.cpp:122). Safe because zrcHeightfieldCreate
// bounds the product of the two sizes well inside int range before any
// heightfield with these dimensions can exist.
ZrcResult zrcHeightfieldColumn(const ZrcHeightfield* heightfield, int32_t x,
                               int32_t z, ZrcSpan* out, int32_t max_spans,
                               int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_spans < 0) return ZRC_ERR_INVALID_ARGUMENT;
  const rcHeightfield& hf = *heightfield->impl;
  if (x < 0 || x >= hf.width || z < 0 || z >= hf.height) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_spans > 0 && out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  int32_t total = 0;
  for (const rcSpan* span = hf.spans[x + z * hf.width]; span != nullptr;
       span = span->next) {
    if (total < max_spans) {
      out[total].smin = static_cast<uint32_t>(span->smin);
      out[total].smax = static_cast<uint32_t>(span->smax);
      out[total].area = static_cast<uint8_t>(span->area);
    }
    ++total;
  }
  *out_count = total;
  if (total > max_spans) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The compact heightfield
//===----------------------------------------------------------------------===//

ZrcResult zrcCompactHeightfieldCreate(const ZrcBuildContext* context,
                                      int32_t walkable_height,
                                      int32_t walkable_climb,
                                      const ZrcHeightfield* heightfield,
                                      ZrcCompactHeightfield** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const ZrcResult args_ok =
      ValidateCompactHeightfieldCreateArgs(walkable_height, walkable_climb);
  if (args_ok != ZRC_OK) return args_ok;

  rcCompactHeightfield* impl = zrc::RcNew<rcCompactHeightfield>();
  if (impl == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  zrc::HostContext ctx(context);
  if (!rcBuildCompactHeightfield(&ctx, walkable_height, walkable_climb,
                                 *heightfield->impl, *impl)) {
    zrc::RcFree(impl);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  ZrcCompactHeightfield* handle = zrc::New<ZrcCompactHeightfield>();
  if (handle == nullptr) {
    zrc::RcFree(impl);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->impl = impl;
  *out = handle;
  return ZRC_OK;
}

void zrcCompactHeightfieldDestroy(ZrcCompactHeightfield* field) {
  if (field == nullptr) return;
  zrc::RcFree(field->impl);
  zrc::Delete(field);
}

ZrcResult zrcCompactHeightfieldInfo(const ZrcCompactHeightfield* field,
                                    ZrcCompactHeightfieldInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcCompactHeightfield& chf = *field->impl;
  out->width = chf.width;
  out->height = chf.height;
  out->span_count = chf.spanCount;
  out->walkable_height = chf.walkableHeight;
  out->walkable_climb = chf.walkableClimb;
  out->border_size = chf.borderSize;
  out->max_distance = chf.maxDistance;
  out->max_regions = chf.maxRegions;
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = chf.bmin[i];
    out->bmax[i] = chf.bmax[i];
  }
  out->cell_size = chf.cs;
  out->cell_height = chf.ch;
  out->has_distance_field = chf.dist != nullptr ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldCells(const ZrcCompactHeightfield* field,
                                     int32_t first, int32_t count,
                                     ZrcCompactCell* out) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcCompactHeightfield& chf = *field->impl;
  // Upstream's own `xSize * zSize` (Recast.cpp:426). The product
  // fits an int because the sizes came from a heightfield zrcHeightfieldCreate
  // bounded, and rcBuildCompactHeightfield copies them across unchanged.
  const int32_t length = chf.width * chf.height;
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || chf.cells == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  for (int32_t i = 0; i < count; ++i) {
    const rcCompactCell& src = chf.cells[first + i];
    out[i].index = static_cast<uint32_t>(src.index);
    out[i].count = static_cast<uint32_t>(src.count);
  }
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldSpans(const ZrcCompactHeightfield* field,
                                     int32_t first, int32_t count,
                                     ZrcCompactSpan* out) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcCompactHeightfield& chf = *field->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, chf.spanCount);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || chf.spans == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  for (int32_t i = 0; i < count; ++i) {
    const rcCompactSpan& src = chf.spans[first + i];
    out[i].y = static_cast<uint16_t>(src.y);
    out[i].reg = static_cast<uint16_t>(src.reg);
    out[i].con = static_cast<uint32_t>(src.con);
    out[i].h = static_cast<uint32_t>(src.h);
  }
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldSetSpans(ZrcCompactHeightfield* field,
                                        int32_t first, int32_t count,
                                        const ZrcCompactSpan* spans) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  rcCompactHeightfield& chf = *field->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, chf.spanCount);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (spans == nullptr || chf.spans == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // con is a 24-bit bitfield and h an 8-bit one (rcCompactSpan, Recast.h); a
  // value that does not fit is truncated silently on the way in and reads
  // back as a different span. The whole batch is checked before any of it is
  // written.
  for (int32_t i = 0; i < count; ++i) {
    if (spans[i].con >= (1u << 24) || spans[i].h >= 256u) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  for (int32_t i = 0; i < count; ++i) {
    rcCompactSpan& dst = chf.spans[first + i];
    dst.y = spans[i].y;
    dst.reg = spans[i].reg;
    dst.con = spans[i].con;
    dst.h = spans[i].h;
  }
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldDistances(const ZrcCompactHeightfield* field,
                                         int32_t first, int32_t count,
                                         uint16_t* out) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcCompactHeightfield& chf = *field->impl;
  // Checked before CheckRange: with no distance array there is no range to
  // bound against.
  if (chf.dist == nullptr) return ZRC_ERR_NOT_FOUND;
  const ZrcResult range_ok = zrc::CheckRange(first, count, chf.spanCount);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, chf.dist + first, sizeof(uint16_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldAreas(const ZrcCompactHeightfield* field,
                                     int32_t first, int32_t count,
                                     uint8_t* out) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcCompactHeightfield& chf = *field->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, chf.spanCount);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || chf.areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, chf.areas + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldSetAreas(ZrcCompactHeightfield* field,
                                        int32_t first, int32_t count,
                                        const uint8_t* areas) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  rcCompactHeightfield& chf = *field->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, chf.spanCount);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (areas == nullptr || chf.areas == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  for (int32_t i = 0; i < count; ++i) {
    if (areas[i] >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  }
  memcpy(chf.areas + first, areas, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Contours
//===----------------------------------------------------------------------===//

ZrcResult zrcContourSetCreate(const ZrcBuildContext* context,
                              const ZrcCompactHeightfield* field,
                              float max_error, int32_t max_edge_len,
                              int32_t flags, ZrcContourSet** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const ZrcResult args_ok =
      ValidateContourSetCreateArgs(max_error, max_edge_len, flags);
  if (args_ok != ZRC_OK) return args_ok;

  rcContourSet* impl = zrc::RcNew<rcContourSet>();
  if (impl == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  zrc::HostContext ctx(context);
  if (!rcBuildContours(&ctx, *field->impl, max_error, max_edge_len, *impl,
                       flags)) {
    zrc::RcFree(impl);
    return ZRC_ERR_BAKE_FAILED;
  }

  ZrcContourSet* handle = zrc::New<ZrcContourSet>();
  if (handle == nullptr) {
    zrc::RcFree(impl);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->impl = impl;
  *out = handle;
  return ZRC_OK;
}

void zrcContourSetDestroy(ZrcContourSet* contours) {
  if (contours == nullptr) return;
  zrc::RcFree(contours->impl);
  zrc::Delete(contours);
}

ZrcResult zrcContourSetInfo(const ZrcContourSet* contours,
                            ZrcContourSetInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (contours == nullptr || contours->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcContourSet& cset = *contours->impl;
  out->contour_count = cset.nconts;
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = cset.bmin[i];
    out->bmax[i] = cset.bmax[i];
  }
  out->cell_size = cset.cs;
  out->cell_height = cset.ch;
  out->width = cset.width;
  out->height = cset.height;
  out->border_size = cset.borderSize;
  out->max_error = cset.maxError;
  return ZRC_OK;
}

ZrcResult zrcContourAt(const ZrcContourSet* contours, int32_t index,
                       ZrcContourInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (contours == nullptr || contours->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcContourSet& cset = *contours->impl;
  if (index < 0 || index >= cset.nconts) return ZRC_ERR_INVALID_ARGUMENT;

  const rcContour& c = cset.conts[index];
  out->vert_count = c.nverts;
  out->raw_vert_count = c.nrverts;
  out->region = static_cast<uint16_t>(c.reg);
  out->area = static_cast<uint8_t>(c.area);
  return ZRC_OK;
}

ZrcResult zrcContourVerts(const ZrcContourSet* contours, int32_t index,
                          int32_t first, int32_t count, int32_t* out) {
  if (contours == nullptr || contours->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcContourSet& cset = *contours->impl;
  if (index < 0 || index >= cset.nconts) return ZRC_ERR_INVALID_ARGUMENT;

  const rcContour& c = cset.conts[index];
  const ZrcResult range_ok = zrc::CheckRange(first, count, c.nverts);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || c.verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, c.verts + static_cast<int64_t>(first) * 4,
         sizeof(int32_t) * 4 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcContourRawVerts(const ZrcContourSet* contours, int32_t index,
                             int32_t first, int32_t count, int32_t* out) {
  if (contours == nullptr || contours->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcContourSet& cset = *contours->impl;
  if (index < 0 || index >= cset.nconts) return ZRC_ERR_INVALID_ARGUMENT;

  const rcContour& c = cset.conts[index];
  const ZrcResult range_ok = zrc::CheckRange(first, count, c.nrverts);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || c.rverts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, c.rverts + static_cast<int64_t>(first) * 4,
         sizeof(int32_t) * 4 * static_cast<size_t>(count));
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The polygon mesh, built by hand
//===----------------------------------------------------------------------===//

ZrcResult zrcPolyMeshCreate(ZrcPolyMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;

  rcPolyMesh* poly = zrc::RcNew<rcPolyMesh>();
  if (poly == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  rcPolyMeshDetail* detail = zrc::RcNew<rcPolyMeshDetail>();
  if (detail == nullptr) {
    zrc::RcFree(poly);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  ZrcPolyMesh* handle = zrc::New<ZrcPolyMesh>();
  if (handle == nullptr) {
    zrc::RcFree(poly);
    zrc::RcFree(detail);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->poly = poly;
  handle->detail = detail;
  handle->walkable_height = 0.f;
  handle->walkable_radius = 0.f;
  handle->walkable_climb = 0.f;
  handle->has_agent_dims = false;
  *out = handle;
  return ZRC_OK;
}

ZrcResult zrcPolyMeshSetAgentDims(ZrcPolyMesh* mesh, float walkable_height,
                                  float walkable_radius,
                                  float walkable_climb) {
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(walkable_height) || walkable_height < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(walkable_radius) || walkable_radius < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(walkable_climb) || walkable_climb < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  mesh->walkable_height = walkable_height;
  mesh->walkable_radius = walkable_radius;
  mesh->walkable_climb = walkable_climb;
  mesh->has_agent_dims = true;
  return ZRC_OK;
}

ZrcResult zrcPolyMeshInfo(const ZrcPolyMesh* mesh, ZrcPolyMeshInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (mesh == nullptr || mesh->poly == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMesh& poly = *mesh->poly;
  const rcPolyMeshDetail& detail = *mesh->detail;

  out->vert_count = poly.nverts;
  out->poly_count = poly.npolys;
  out->max_polys = poly.maxpolys;
  out->verts_per_poly = poly.nvp;
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = poly.bmin[i];
    out->bmax[i] = poly.bmax[i];
  }
  out->cell_size = poly.cs;
  out->cell_height = poly.ch;
  out->border_size = poly.borderSize;
  out->max_edge_error = poly.maxEdgeError;

  out->detail_mesh_count = detail.nmeshes;
  out->detail_vert_count = detail.nverts;
  out->detail_tri_count = detail.ntris;

  out->walkable_height = mesh->walkable_height;
  out->walkable_radius = mesh->walkable_radius;
  out->walkable_climb = mesh->walkable_climb;
  return ZRC_OK;
}

ZrcResult zrcPolyMeshVerts(const ZrcPolyMesh* mesh, int32_t first,
                           int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMesh& poly = *mesh->poly;
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.nverts);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || poly.verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, poly.verts + static_cast<int64_t>(first) * 3,
         sizeof(uint16_t) * 3 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshPolys(const ZrcPolyMesh* mesh, int32_t first,
                           int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMesh& poly = *mesh->poly;
  // The array holds room for maxpolys, not npolys — that is what indexes it.
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.maxpolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || poly.polys == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const int32_t stride = 2 * poly.nvp;
  memcpy(out, poly.polys + static_cast<int64_t>(first) * stride,
         sizeof(uint16_t) * static_cast<size_t>(stride) *
             static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshRegions(const ZrcPolyMesh* mesh, int32_t first,
                             int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMesh& poly = *mesh->poly;
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.maxpolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || poly.regs == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, poly.regs + first, sizeof(uint16_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshPolyAreas(const ZrcPolyMesh* mesh, int32_t first,
                               int32_t count, uint8_t* out) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMesh& poly = *mesh->poly;
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.maxpolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || poly.areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, poly.areas + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshSetPolyAreas(ZrcPolyMesh* mesh, int32_t first,
                                  int32_t count, const uint8_t* areas) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  rcPolyMesh& poly = *mesh->poly;
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.maxpolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (areas == nullptr || poly.areas == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  for (int32_t i = 0; i < count; ++i) {
    if (areas[i] >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  }
  memcpy(poly.areas + first, areas, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshPolyFlags(const ZrcPolyMesh* mesh, int32_t first,
                               int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMesh& poly = *mesh->poly;
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.maxpolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || poly.flags == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  memcpy(out, poly.flags + first, sizeof(uint16_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshSetPolyFlags(ZrcPolyMesh* mesh, int32_t first,
                                  int32_t count, const uint16_t* flags) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  rcPolyMesh& poly = *mesh->poly;
  const ZrcResult range_ok = zrc::CheckRange(first, count, poly.maxpolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (flags == nullptr || poly.flags == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(poly.flags + first, flags, sizeof(uint16_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshDetailMeshes(const ZrcPolyMesh* mesh, int32_t first,
                                  int32_t count, uint32_t* out) {
  if (mesh == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMeshDetail& detail = *mesh->detail;
  const ZrcResult range_ok = zrc::CheckRange(first, count, detail.nmeshes);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || detail.meshes == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(out, detail.meshes + static_cast<int64_t>(first) * 4,
         sizeof(uint32_t) * 4 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshDetailVerts(const ZrcPolyMesh* mesh, int32_t first,
                                 int32_t count, float* out) {
  if (mesh == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMeshDetail& detail = *mesh->detail;
  const ZrcResult range_ok = zrc::CheckRange(first, count, detail.nverts);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || detail.verts == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(out, detail.verts + static_cast<int64_t>(first) * 3,
         sizeof(float) * 3 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcPolyMeshDetailTris(const ZrcPolyMesh* mesh, int32_t first,
                                int32_t count, uint8_t* out) {
  if (mesh == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcPolyMeshDetail& detail = *mesh->detail;
  const ZrcResult range_ok = zrc::CheckRange(first, count, detail.ntris);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || detail.tris == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(out, detail.tris + static_cast<int64_t>(first) * 4,
         sizeof(uint8_t) * 4 * static_cast<size_t>(count));
  return ZRC_OK;
}

}  // extern "C"
