//===----------------------------------------------------------------------===//
// zrecast — the staged Recast pipeline: one entry point per build stage.
//
// zrecast_bake.cpp runs heightfield -> compact heightfield -> regions ->
// contours -> polygon mesh as one call. This file exposes the same stages
// separately, over host-owned containers, for a tool that needs to inspect or
// replace what happens between two of them. See zrecast.h's "The staged
// Recast pipeline" section for the contract each entry point promises.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Bounds shared with the bake
//
// zrc::ValidateTriMesh, zrc::ValidateAreaAuthoring, zrc::MarkAreaVolumes,
// zrc::zrc::kMaxTriMeshCount and zrc::zrc::kMaxWalkableRadiusCells live in
// zrecast_internal.h, applied identically by zrecast_bake.cpp. A raw stage and
// a whole bake answer to one copy of each rule.
//===----------------------------------------------------------------------===//


//===----------------------------------------------------------------------===//
// The output-container check
//
// rcBuildPolyMesh and rcMergePolyMeshes assign fresh allocations straight
// over a destination's members with no check of their own (RecastMesh.cpp);
// rcMergePolyMeshDetails does the same (RecastMeshDetail.cpp). A destination
// that already holds a result loses every buffer it held. "Filled" is defined
// in zrecast.h's staged-pipeline section: any of the poly half's five arrays,
// or the detail half's three, non-null.
//===----------------------------------------------------------------------===//

bool PolyHalfFilled(const rcPolyMesh& poly) {
  return poly.verts != nullptr || poly.polys != nullptr ||
         poly.regs != nullptr || poly.flags != nullptr ||
         poly.areas != nullptr;
}

bool DetailHalfFilled(const rcPolyMeshDetail& detail) {
  return detail.meshes != nullptr || detail.verts != nullptr ||
         detail.tris != nullptr;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Rasterisation and spans
//===----------------------------------------------------------------------===//

ZrcResult zrcHeightfieldAddSpan(const ZrcBuildContext* context,
                                ZrcHeightfield* heightfield, int32_t x,
                                int32_t z, uint32_t span_min,
                                uint32_t span_max, uint8_t area,
                                int32_t flag_merge_threshold) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // rcAddSpan indexes spans[x + z * width] with no bound of its own
  // (RecastRasterization.cpp:122); x and z are checked against the
  // heightfield's own extent here.
  if (x < 0 || x >= heightfield->impl->width || z < 0 ||
      z >= heightfield->impl->height) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // smin/smax are 13-bit fields and area is 6 bits (ZRC_SPAN_HEIGHT_BITS,
  // ZRC_MAX_AREAS); a value that does not fit is truncated silently on the
  // way into rcSpan's bitfields and reads back as a different span.
  if (span_min >= span_max || span_max > ZRC_SPAN_MAX_HEIGHT) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  if (flag_merge_threshold < 0) return ZRC_ERR_INVALID_ARGUMENT;

  zrc::HostContext ctx(context);
  const bool ok = rcAddSpan(&ctx, *heightfield->impl, x, z,
                            static_cast<unsigned short>(span_min),
                            static_cast<unsigned short>(span_max), area,
                            flag_merge_threshold);
  // addSpan's only false path is allocSpan returning null (RecastRasterization.cpp:113-116).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcHeightfieldRasterizeTriangle(const ZrcBuildContext* context,
                                          ZrcHeightfield* heightfield,
                                          const float* v0, const float* v1,
                                          const float* v2, uint8_t area,
                                          int32_t flag_merge_threshold) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Finiteness is not enough: rasterizeTri converts a corner's distance from
  // the field's own minimum into a cell index before it clamps, so a huge but
  // finite coordinate on a triangle that straddles the field is an
  // out-of-range conversion. See zrc::RasterVertexFits.
  const rcHeightfield& hf = *heightfield->impl;
  if (!zrc::RasterVertexFits(hf, v0) || !zrc::RasterVertexFits(hf, v1) ||
      !zrc::RasterVertexFits(hf, v2)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  if (flag_merge_threshold < 0) return ZRC_ERR_INVALID_ARGUMENT;

  zrc::HostContext ctx(context);
  const bool ok = rcRasterizeTriangle(&ctx, v0, v1, v2, area,
                                      *heightfield->impl,
                                      flag_merge_threshold);
  // rcRasterizeTriangle's only false path is the rasterizeTri/addSpan chain
  // running out of memory (RecastRasterization.cpp:469-473).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcHeightfieldRasterizeTriangles(const ZrcBuildContext* context,
                                           ZrcHeightfield* heightfield,
                                           const ZrcTriMesh* mesh,
                                           const uint8_t* tri_areas,
                                           int32_t flag_merge_threshold) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) return mesh_valid;
  if (tri_areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (flag_merge_threshold < 0) return ZRC_ERR_INVALID_ARGUMENT;
  // The area lands in a 6-bit field; the whole array is checked before any
  // triangle is rasterised.
  for (int i = 0; i < mesh->tri_count; ++i) {
    if (tri_areas[i] >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  }
  // zrc::ValidateTriMesh bounds the indices and rejects a non-finite
  // coordinate; this bounds each vertex against the field it is going into,
  // which the bake never needs because it sizes the field from the same mesh.
  for (int i = 0; i < mesh->vert_count; ++i) {
    if (!zrc::RasterVertexFits(*heightfield->impl, mesh->verts + i * 3)) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }

  zrc::HostContext ctx(context);
  const bool ok = rcRasterizeTriangles(&ctx, mesh->verts, mesh->vert_count,
                                       mesh->tris, tri_areas, mesh->tri_count,
                                       *heightfield->impl,
                                       flag_merge_threshold);
  // Every triangle goes through the same rasterizeTri/addSpan chain as the
  // single-triangle form; its only false path is out of memory
  // (RecastRasterization.cpp:495-499). ffi/zrecast_bake.cpp maps this
  // overload's failure the same way (zrecast_bake.cpp:525-530).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcHeightfieldRasterizeTrianglesU16(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    const float* verts, int32_t vert_count, const uint16_t* tris,
    const uint8_t* tri_areas, int32_t tri_count,
    int32_t flag_merge_threshold) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // The same counts zrc::ValidateTriMesh accepts for the 32-bit index form,
  // so the two overloads answer alike.
  if (vert_count < 3 || vert_count > zrc::kMaxTriMeshCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tri_count < 1 || tri_count > zrc::kMaxTriMeshCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (flag_merge_threshold < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (tri_areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (vert_count > 0 && verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (tri_count > 0 && tris == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  // rcRasterizeTriangles reads verts[tris[i] * 3] with no bound of its own
  // (RecastRasterization.cpp:519-521); every index is checked here, and every
  // coordinate is bounded the way a heightfield's own extent is, since this
  // form has no ZrcTriMesh to route through ValidateTriMesh.
  for (int32_t i = 0; i < vert_count; ++i) {
    if (!zrc::RasterVertexFits(*heightfield->impl, verts + i * 3)) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  for (int32_t i = 0; i < tri_count * 3; ++i) {
    if (tris[i] >= vert_count) return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int32_t i = 0; i < tri_count; ++i) {
    if (tri_areas[i] >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  }

  zrc::HostContext ctx(context);
  const bool ok = rcRasterizeTriangles(&ctx, verts, vert_count, tris,
                                       tri_areas, tri_count,
                                       *heightfield->impl,
                                       flag_merge_threshold);
  // Same rasterizeTri/addSpan chain as the 32-bit index overload; its only
  // false path is out of memory (RecastRasterization.cpp:522-526).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcHeightfieldRasterizeTriangleSoup(const ZrcBuildContext* context,
                                              ZrcHeightfield* heightfield,
                                              const float* verts,
                                              const uint8_t* tri_areas,
                                              int32_t tri_count,
                                              int32_t flag_merge_threshold) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Bounded the same way ValidateTriMesh bounds a triangle count.
  if (tri_count < 1 || tri_count > zrc::kMaxTriMeshCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (verts == nullptr || tri_areas == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (flag_merge_threshold < 0) return ZRC_ERR_INVALID_ARGUMENT;

  // 9 * tri_count (three vertices of three floats each) can exceed int32
  // range at tri_count near zrc::kMaxTriMeshCount, so each triangle's base offset
  // is computed in 64 bits rather than precomputed as one int32 bound.
  for (int32_t t = 0; t < tri_count; ++t) {
    const float* tri_verts = verts + static_cast<int64_t>(t) * 9;
    for (int k = 0; k < 3; ++k) {
      if (!zrc::RasterVertexFits(*heightfield->impl, tri_verts + k * 3)) {
        return ZRC_ERR_INVALID_ARGUMENT;
      }
    }
  }
  for (int32_t i = 0; i < tri_count; ++i) {
    if (tri_areas[i] >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  }

  zrc::HostContext ctx(context);
  const bool ok = rcRasterizeTriangles(&ctx, verts, tri_areas, tri_count,
                                       *heightfield->impl,
                                       flag_merge_threshold);
  // Same rasterizeTri/addSpan chain as the indexed forms; its only false path
  // is out of memory (RecastRasterization.cpp:548-552).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcHeightfieldSpanCount(const ZrcBuildContext* context,
                                  const ZrcHeightfield* heightfield,
                                  int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostContext ctx(context);
  *out_count = rcGetHeightFieldSpanCount(&ctx, *heightfield->impl);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Filters
//===----------------------------------------------------------------------===//

ZrcResult zrcHeightfieldFilterLowHangingObstacles(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    int32_t walkable_climb) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (walkable_climb < 0) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  rcFilterLowHangingWalkableObstacles(&ctx, walkable_climb,
                                      *heightfield->impl);
  return ZRC_OK;
}

ZrcResult zrcHeightfieldFilterLedgeSpans(const ZrcBuildContext* context,
                                         ZrcHeightfield* heightfield,
                                         int32_t walkable_height,
                                         int32_t walkable_climb) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (walkable_height < 0 || walkable_climb < 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostContext ctx(context);
  rcFilterLedgeSpans(&ctx, walkable_height, walkable_climb,
                     *heightfield->impl);
  return ZRC_OK;
}

ZrcResult zrcHeightfieldFilterWalkableLowHeightSpans(
    const ZrcBuildContext* context, ZrcHeightfield* heightfield,
    int32_t walkable_height) {
  if (heightfield == nullptr || heightfield->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (walkable_height < 0) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  rcFilterWalkableLowHeightSpans(&ctx, walkable_height, *heightfield->impl);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Compact-field stages
//===----------------------------------------------------------------------===//

ZrcResult zrcCompactHeightfieldErode(const ZrcBuildContext* context,
                                     ZrcCompactHeightfield* field,
                                     int32_t radius) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // rcErodeWalkableArea reduces the radius to (unsigned char)(radius * 2)
  // (RecastArea.cpp:210); past zrc::kMaxWalkableRadiusCells that wraps and erodes
  // less than asked instead of failing.
  if (radius < 0 || radius > zrc::kMaxWalkableRadiusCells) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostContext ctx(context);
  const bool ok = rcErodeWalkableArea(&ctx, radius, *field->impl);
  // Its only false path is the scratch distance buffer failing to allocate
  // (RecastArea.cpp:45-50), matching zrecast_bake.cpp's own mapping
  // (zrecast_bake.cpp:558-562).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcCompactHeightfieldMedianFilter(const ZrcBuildContext* context,
                                            ZrcCompactHeightfield* field) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostContext ctx(context);
  const bool ok = rcMedianFilterWalkableArea(&ctx, *field->impl);
  // Its only false path is its scratch area buffer failing to allocate
  // (RecastArea.cpp:247-252).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcCompactHeightfieldMarkAreas(const ZrcBuildContext* context,
                                         ZrcCompactHeightfield* field,
                                         const ZrcAreaAuthoring* authoring) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (authoring == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult authoring_valid = zrc::ValidateAreaAuthoring(*authoring);
  if (authoring_valid != ZRC_OK) return authoring_valid;

  zrc::HostContext ctx(context);
  zrc::MarkAreaVolumes(ctx, *authoring, *field->impl);
  return ZRC_OK;
}

ZrcResult zrcCompactHeightfieldBuildDistanceField(
    const ZrcBuildContext* context, ZrcCompactHeightfield* field) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostContext ctx(context);
  const bool ok = rcBuildDistanceField(&ctx, *field->impl);
  // Both of its false paths are its two scratch buffers failing to allocate
  // (RecastRegion.cpp:1265-1277), matching zrecast_bake.cpp's own mapping
  // (zrecast_bake.cpp:605-608).
  return ok ? ZRC_OK : ZRC_ERR_OUT_OF_MEMORY;
}

ZrcResult zrcCompactHeightfieldBuildRegions(const ZrcBuildContext* context,
                                            ZrcCompactHeightfield* field,
                                            ZrcPartition partition,
                                            int32_t border_size,
                                            int32_t min_region_area,
                                            int32_t merge_region_area) {
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (partition != ZRC_PARTITION_WATERSHED &&
      partition != ZRC_PARTITION_MONOTONE &&
      partition != ZRC_PARTITION_LAYERS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (border_size < 0 || border_size > zrc::kMaxBorderSizeCells) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (min_region_area < 0 || merge_region_area < 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // rcBuildRegions reads chf.dist before rcBuildDistanceField has allocated
  // it (RecastRegion.cpp:339,381,494, reached from rcBuildRegions via
  // sortCellsByLevel/expandRegions/floodRegion). rcBuildRegionsMonotone and
  // rcBuildLayerRegions never read chf.dist, so only the watershed partition
  // is guarded here.
  if (partition == ZRC_PARTITION_WATERSHED && field->impl->dist == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  zrc::HostContext ctx(context);
  bool ok = false;
  switch (partition) {
    case ZRC_PARTITION_WATERSHED:
      ok = rcBuildRegions(&ctx, *field->impl, border_size, min_region_area,
                          merge_region_area);
      break;
    case ZRC_PARTITION_MONOTONE:
      ok = rcBuildRegionsMonotone(&ctx, *field->impl, border_size,
                                  min_region_area, merge_region_area);
      break;
    case ZRC_PARTITION_LAYERS:
      ok = rcBuildLayerRegions(&ctx, *field->impl, border_size,
                               min_region_area);
      break;
  }
  // All three mix allocation failure with region-id overflow and merge
  // failure; zrecast_bake.cpp maps every false from any of them to
  // ZRC_ERR_BAKE_FAILED without distinguishing the cause
  // (zrecast_bake.cpp:604-631), and this follows the same mapping.
  return ok ? ZRC_OK : ZRC_ERR_BAKE_FAILED;
}

//===----------------------------------------------------------------------===//
// Polygon-mesh stages
//===----------------------------------------------------------------------===//

ZrcResult zrcPolyMeshBuild(const ZrcBuildContext* context,
                           const ZrcContourSet* contours,
                           int32_t verts_per_poly, ZrcPolyMesh* mesh) {
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (contours == nullptr || contours->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (verts_per_poly < 3 || verts_per_poly > ZRC_VERTS_PER_POLYGON) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (PolyHalfFilled(*mesh->poly)) return ZRC_ERR_ALREADY_BUILT;

  zrc::HostContext ctx(context);
  const bool ok = rcBuildPolyMesh(&ctx, *contours->impl, verts_per_poly,
                                  *mesh->poly);
  // rcBuildPolyMesh's false paths mix allocation failure with a vertex or
  // polygon count too large to index (RecastMesh.cpp:1014-1097);
  // zrecast_bake.cpp maps every one of them to ZRC_ERR_BAKE_FAILED
  // (zrecast_bake.cpp:648-651), and this follows the same mapping.
  return ok ? ZRC_OK : ZRC_ERR_BAKE_FAILED;
}

ZrcResult zrcPolyMeshBuildDetail(const ZrcBuildContext* context,
                                 ZrcPolyMesh* mesh,
                                 const ZrcCompactHeightfield* field,
                                 float sample_dist, float sample_max_error) {
  if (mesh == nullptr || mesh->poly == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(sample_dist) || sample_dist < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(sample_max_error) || sample_max_error < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (DetailHalfFilled(*mesh->detail)) return ZRC_ERR_ALREADY_BUILT;

  zrc::HostContext ctx(context);
  const bool ok = rcBuildPolyMeshDetail(&ctx, *mesh->poly, *field->impl,
                                        sample_dist, sample_max_error,
                                        *mesh->detail);
  // A mesh with no polygons returns true before allocating anything
  // (RecastMeshDetail.cpp:1178-1179), leaving the detail half empty; every
  // false path otherwise is zrecast_bake.cpp's ZRC_ERR_BAKE_FAILED mapping
  // (zrecast_bake.cpp:659-664).
  return ok ? ZRC_OK : ZRC_ERR_BAKE_FAILED;
}

ZrcResult zrcPolyMeshCopy(const ZrcBuildContext* context,
                          const ZrcPolyMesh* src, ZrcPolyMesh* dst) {
  if (src == nullptr || src->poly == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (dst == nullptr || dst->poly == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (PolyHalfFilled(*dst->poly)) return ZRC_ERR_ALREADY_BUILT;

  zrc::HostContext ctx(context);
  const bool ok = rcCopyPolyMesh(&ctx, *src->poly, *dst->poly);
  // Every one of rcCopyPolyMesh's false paths is one of its five allocations
  // failing (RecastMesh.cpp:1508-1546); nothing else can make it fail.
  if (!ok) return ZRC_ERR_OUT_OF_MEMORY;

  // Upstream carries no agent dimensions; the header says they travel with
  // the polygons.
  dst->walkable_height = src->walkable_height;
  dst->walkable_radius = src->walkable_radius;
  dst->walkable_climb = src->walkable_climb;
  dst->has_agent_dims = src->has_agent_dims;
  return ZRC_OK;
}

ZrcResult zrcPolyMeshMerge(const ZrcBuildContext* context,
                           const ZrcPolyMesh* const* meshes, int32_t count,
                           ZrcPolyMesh* out) {
  if (out == nullptr || out->poly == nullptr || out->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (meshes == nullptr || count < 1) return ZRC_ERR_INVALID_ARGUMENT;
  for (int32_t i = 0; i < count; ++i) {
    if (meshes[i] == nullptr || meshes[i]->poly == nullptr ||
        meshes[i]->detail == nullptr) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  // rcMergePolyMeshes takes meshes[0]->nvp as the stride into every input's
  // polys array (RecastMesh.cpp:1317,1424), which reads past the end of any
  // input packing its polygons at a different stride.
  const int first_nvp = meshes[0]->poly->nvp;
  for (int32_t i = 1; i < count; ++i) {
    if (meshes[i]->poly->nvp != first_nvp) return ZRC_ERR_INVALID_ARGUMENT;
  }

  // Both halves are merged together: every input must have a built detail
  // half, or none may, or the merged mesh's detail arrays would describe only
  // some of its polygons.
  const bool merge_detail = DetailHalfFilled(*meshes[0]->detail);
  for (int32_t i = 1; i < count; ++i) {
    if (DetailHalfFilled(*meshes[i]->detail) != merge_detail) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }

  if (!meshes[0]->has_agent_dims) return ZRC_ERR_INVALID_ARGUMENT;
  for (int32_t i = 1; i < count; ++i) {
    if (!meshes[i]->has_agent_dims ||
        meshes[i]->walkable_height != meshes[0]->walkable_height ||
        meshes[i]->walkable_radius != meshes[0]->walkable_radius ||
        meshes[i]->walkable_climb != meshes[0]->walkable_climb) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }

  if (PolyHalfFilled(*out->poly) || DetailHalfFilled(*out->detail)) {
    return ZRC_ERR_ALREADY_BUILT;
  }

  zrc::HostContext ctx(context);

  // rcMergePolyMeshes takes rcPolyMesh** (non-const) but only ever reads
  // through it (RecastMesh.cpp:1401-1465; the loop declares its element
  // `const rcPolyMesh* pmesh`). meshes[i]->poly is itself a plain, non-const
  // rcPolyMesh* member, so copying it out of a `const ZrcPolyMesh*` already
  // yields a mutable pointer with no cast needed.
  {
    zrc::TempBuffer scratch(static_cast<size_t>(count) * sizeof(rcPolyMesh*));
    if (scratch.get() == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
    rcPolyMesh** poly_ptrs = static_cast<rcPolyMesh**>(scratch.get());
    for (int32_t i = 0; i < count; ++i) poly_ptrs[i] = meshes[i]->poly;
    const bool ok = rcMergePolyMeshes(&ctx, poly_ptrs, count, *out->poly);
    // Every false path of rcMergePolyMeshes and the adjacency build it calls
    // (buildMeshAdjacency, RecastMesh.cpp:42) is an allocation failure
    // (RecastMesh.cpp:1336-1397).
    if (!ok) return ZRC_ERR_OUT_OF_MEMORY;
  }

  if (merge_detail) {
    zrc::TempBuffer scratch(static_cast<size_t>(count) *
                          sizeof(rcPolyMeshDetail*));
    if (scratch.get() == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
    rcPolyMeshDetail** detail_ptrs =
        static_cast<rcPolyMeshDetail**>(scratch.get());
    for (int32_t i = 0; i < count; ++i) detail_ptrs[i] = meshes[i]->detail;
    // rcMergePolyMeshDetails has no null-meshes guard of its own
    // (RecastMeshDetail.cpp:1389-1405); `meshes` is checked non-null and
    // `count >= 1` above, so the array it indexes is never null here.
    const bool ok =
        rcMergePolyMeshDetails(&ctx, detail_ptrs, count, *out->detail);
    // Every false path is one of its three allocations failing
    // (RecastMeshDetail.cpp:1408-1428).
    if (!ok) return ZRC_ERR_OUT_OF_MEMORY;
  }

  out->walkable_height = meshes[0]->walkable_height;
  out->walkable_radius = meshes[0]->walkable_radius;
  out->walkable_climb = meshes[0]->walkable_climb;
  out->has_agent_dims = true;
  return ZRC_OK;
}

}  // extern "C"
