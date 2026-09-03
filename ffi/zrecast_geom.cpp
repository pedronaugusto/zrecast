//===----------------------------------------------------------------------===//
// zrecast — computational geometry primitives, callable directly.
//
// Thin wrappers over Detour's point/segment/polygon tests
// (Detour/Include/DetourCommon.h) and Recast's polygon offset
// (Recast/Source/RecastArea.cpp), with the input validation upstream does not
// do: every count is checked against what the function will dereference, and
// every position is finite before it reaches a barycentric divide or a
// square root. See zrecast.h's "Geometry primitives" section for what each
// entry point answers.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

#include "DetourCommon.h"

namespace {

/// Largest vertex count a polygon argument may carry.
///
/// Upstream indexes a vertex array as `verts[i * 3]` with `int` arithmetic, so
/// the product has to stay inside int32. Nothing upstream bounds it.
const int32_t kMaxPolyVerts = 0x7fffffff / 3;

/// Shared by every entry point that takes a `(verts, vert_count)` array:
/// non-null, in range, and every float finite. `min_count` is 3 for a polygon
/// and 1 for zrcPolyCenter, which upstream's dtCalcPolyCenter accepts down to
/// a single indexed vertex.
ZrcResult ValidateVertexArray(const float* verts, int32_t vert_count,
                              int32_t min_count) {
  if (vert_count < min_count || vert_count > kMaxPolyVerts) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const int32_t float_count = vert_count * 3;
  for (int32_t i = 0; i < float_count; ++i) {
    if (!zrc::IsFinite(verts[i])) return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// At least 3, at most kMaxPolyVerts, verts non-null, every vertex finite.
ZrcResult ValidatePolygon(const float* verts, int32_t vert_count) {
  return ValidateVertexArray(verts, vert_count, 3);
}

}  // namespace

extern "C" {

ZrcResult zrcClosestPointOnTriangle(const float* point, const float* a,
                                    const float* b, const float* c,
                                    float* out_closest) {
  if (out_closest == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_closest[0] = out_closest[1] = out_closest[2] = 0.f;
  if (!zrc::IsFiniteVec3(point) || !zrc::IsFiniteVec3(a) ||
      !zrc::IsFiniteVec3(b) || !zrc::IsFiniteVec3(c)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // A collinear or coincident triangle can drive upstream's final
  // `1.0f / (va + vb + vc)` to divide by zero. That result is reported as
  // upstream computes it rather than refused, since refusing it would answer
  // differently from the C for an input the C accepts.
  dtClosestPtPointTriangle(out_closest, point, a, b, c);
  return ZRC_OK;
}

ZrcResult zrcClosestHeightPointTriangle(const float* point, const float* a,
                                        const float* b, const float* c,
                                        float* out_height,
                                        ZrcBool* out_inside) {
  if (out_height == nullptr || out_inside == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_height = 0.f;
  *out_inside = ZRC_FALSE;
  if (!zrc::IsFiniteVec3(point) || !zrc::IsFiniteVec3(a) ||
      !zrc::IsFiniteVec3(b) || !zrc::IsFiniteVec3(c)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  float height = 0.f;
  if (dtClosestHeightPointTriangle(point, a, b, c, height)) {
    *out_height = height;
    *out_inside = ZRC_TRUE;
  }
  return ZRC_OK;
}

ZrcResult zrcDistancePointToSegment2D(const float* point, const float* p,
                                      const float* q, float* out_dist_sqr,
                                      float* out_t) {
  if (out_dist_sqr == nullptr || out_t == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_dist_sqr = 0.f;
  *out_t = 0.f;
  if (!zrc::IsFiniteVec3(point) || !zrc::IsFiniteVec3(p) ||
      !zrc::IsFiniteVec3(q)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  float t = 0.f;
  *out_dist_sqr = dtDistancePtSegSqr2D(point, p, q, t);
  *out_t = t;
  return ZRC_OK;
}

ZrcResult zrcDistancePointToPolyEdges(const float* point, const float* verts,
                                      int32_t vert_count,
                                      float* out_edge_dist_sqr,
                                      float* out_edge_t,
                                      ZrcBool* out_inside) {
  if (out_inside == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_inside = ZRC_FALSE;
  if (out_edge_dist_sqr == nullptr || out_edge_t == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(point)) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult poly_valid = ValidatePolygon(verts, vert_count);
  if (poly_valid != ZRC_OK) return poly_valid;

  // Writes vert_count entries into both arrays, indexed by each edge's
  // starting vertex; both are already sized for that by the caller's contract.
  const bool inside =
      dtDistancePtPolyEdgesSqr(point, verts, vert_count, out_edge_dist_sqr,
                               out_edge_t);
  *out_inside = inside ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPointInPolygon(const float* point, const float* verts,
                            int32_t vert_count, ZrcBool* out_inside) {
  if (out_inside == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_inside = ZRC_FALSE;
  if (!zrc::IsFiniteVec3(point)) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult poly_valid = ValidatePolygon(verts, vert_count);
  if (poly_valid != ZRC_OK) return poly_valid;
  *out_inside = dtPointInPolygon(point, verts, vert_count) ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcIntersectSegmentPoly2D(const float* p0, const float* p1,
                                    const float* verts, int32_t vert_count,
                                    float* out_t_min, float* out_t_max,
                                    int32_t* out_seg_min, int32_t* out_seg_max,
                                    ZrcBool* out_intersects) {
  if (out_t_min == nullptr || out_t_max == nullptr || out_seg_min == nullptr ||
      out_seg_max == nullptr || out_intersects == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_t_min = 0.f;
  *out_t_max = 1.f;
  *out_seg_min = -1;
  *out_seg_max = -1;
  *out_intersects = ZRC_FALSE;
  if (!zrc::IsFiniteVec3(p0) || !zrc::IsFiniteVec3(p1)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const ZrcResult poly_valid = ValidatePolygon(verts, vert_count);
  if (poly_valid != ZRC_OK) return poly_valid;

  // Upstream sets all four of these up front and always leaves them written,
  // a miss included, so they are copied out on every path below rather than
  // only on success.
  float tmin = 0.f;
  float tmax = 1.f;
  int seg_min = -1;
  int seg_max = -1;
  const bool intersects = dtIntersectSegmentPoly2D(p0, p1, verts, vert_count,
                                                   tmin, tmax, seg_min, seg_max);
  *out_t_min = tmin;
  *out_t_max = tmax;
  *out_seg_min = seg_min;
  *out_seg_max = seg_max;
  *out_intersects = intersects ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcIntersectSegSeg2D(const float* ap, const float* aq,
                               const float* bp, const float* bq, float* out_s,
                               float* out_t, ZrcBool* out_intersects) {
  if (out_s == nullptr || out_t == nullptr || out_intersects == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_s = 0.f;
  *out_t = 0.f;
  *out_intersects = ZRC_FALSE;
  if (!zrc::IsFiniteVec3(ap) || !zrc::IsFiniteVec3(aq) ||
      !zrc::IsFiniteVec3(bp) || !zrc::IsFiniteVec3(bq)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  float s = 0.f;
  float t = 0.f;
  // Near-parallel lines leave s and t untouched by upstream, so nothing is
  // written on the false path beyond the zeroing above.
  const bool intersects = dtIntersectSegSeg2D(ap, aq, bp, bq, s, t);
  if (intersects) {
    *out_s = s;
    *out_t = t;
    *out_intersects = ZRC_TRUE;
  }
  return ZRC_OK;
}

ZrcResult zrcOverlapPolyPoly2D(const float* poly_a, int32_t count_a,
                               const float* poly_b, int32_t count_b,
                               ZrcBool* out_overlap) {
  if (out_overlap == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_overlap = ZRC_FALSE;
  const ZrcResult a_valid = ValidatePolygon(poly_a, count_a);
  if (a_valid != ZRC_OK) return a_valid;
  const ZrcResult b_valid = ValidatePolygon(poly_b, count_b);
  if (b_valid != ZRC_OK) return b_valid;
  // Upstream's projectPoly reads poly[0..2] before its loop starts, unguarded
  // by a count of its own; the >= 3 floor on both polygons above is what
  // keeps that read inside the array.
  *out_overlap =
      dtOverlapPolyPoly2D(poly_a, count_a, poly_b, count_b) ? ZRC_TRUE
                                                             : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcOverlapBounds(const float* amin, const float* amax,
                           const float* bmin, const float* bmax,
                           ZrcBool* out_overlap) {
  if (out_overlap == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_overlap = ZRC_FALSE;
  if (!zrc::IsFiniteVec3(amin) || !zrc::IsFiniteVec3(amax) ||
      !zrc::IsFiniteVec3(bmin) || !zrc::IsFiniteVec3(bmax)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_overlap = dtOverlapBounds(amin, amax, bmin, bmax) ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcOverlapQuantBounds(const uint16_t* amin, const uint16_t* amax,
                                const uint16_t* bmin, const uint16_t* bmax,
                                ZrcBool* out_overlap) {
  if (out_overlap == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_overlap = ZRC_FALSE;
  if (amin == nullptr || amax == nullptr || bmin == nullptr ||
      bmax == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_overlap =
      dtOverlapQuantBounds(amin, amax, bmin, bmax) ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcTriArea2D(const float* a, const float* b, const float* c,
                       float* out_area) {
  if (out_area == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_area = 0.f;
  if (!zrc::IsFiniteVec3(a) || !zrc::IsFiniteVec3(b) || !zrc::IsFiniteVec3(c)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_area = dtTriArea2D(a, b, c);
  return ZRC_OK;
}

ZrcResult zrcPolyCenter(const float* verts, int32_t vert_count,
                        const uint16_t* indices, int32_t index_count,
                        float* out_center) {
  if (out_center == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_center[0] = out_center[1] = out_center[2] = 0.f;
  // Upstream's dtCalcPolyCenter takes no vertex count at all, so it trusts
  // every idx[j] as an offset into verts, and divides by nidx with no test —
  // an empty index list yields infinity times zero, i.e. NaN. Both are real
  // parameters here: every index is bounded below vert_count, and index_count
  // must be at least 1.
  const ZrcResult verts_valid = ValidateVertexArray(verts, vert_count, 1);
  if (verts_valid != ZRC_OK) return verts_valid;
  if (index_count < 1 || index_count > kMaxPolyVerts) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (indices == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  for (int32_t j = 0; j < index_count; ++j) {
    if (indices[j] >= vert_count) return ZRC_ERR_INVALID_ARGUMENT;
  }

  dtCalcPolyCenter(out_center, indices, index_count, verts);
  return ZRC_OK;
}

ZrcResult zrcRandomPointInConvexPoly(const float* verts, int32_t vert_count,
                                     float* scratch, int32_t scratch_count,
                                     float s, float t, float* out_point) {
  if (out_point == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_point[0] = out_point[1] = out_point[2] = 0.f;
  // Upstream's fan loop is `for (int i = 2; i < npts; i++)`, which never runs
  // for npts == 1: the chosen triangle index stays npts - 1, and the second
  // corner is then read from &pts[-3] — three floats before the caller's
  // buffer. The >= 3 floor below closes it.
  const ZrcResult poly_valid = ValidatePolygon(verts, vert_count);
  if (poly_valid != ZRC_OK) return poly_valid;
  if (scratch == nullptr || scratch_count < vert_count) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(s) || s < 0.f || s > 1.f) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(t) || t < 0.f || t > 1.f) return ZRC_ERR_INVALID_ARGUMENT;

  dtRandomPointInConvexPoly(verts, vert_count, scratch, s, t, out_point);
  return ZRC_OK;
}

ZrcResult zrcOffsetPoly(const float* verts, int32_t vert_count, float offset,
                        float* out_verts, int32_t max_out_verts,
                        int32_t* out_vert_count) {
  if (out_vert_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_vert_count = 0;
  const ZrcResult poly_valid = ValidatePolygon(verts, vert_count);
  if (poly_valid != ZRC_OK) return poly_valid;
  if (!zrc::IsFinite(offset)) return ZRC_ERR_INVALID_ARGUMENT;
  if (out_verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_out_verts < 1 || max_out_verts > kMaxPolyVerts) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // rcOffsetPoly's own bound is `n + 1 >= maxOutVerts`, one slot stricter
  // than the buffer it is given, so a result that exactly fills the caller's
  // buffer is still reported as 0. That 0 becomes ZRC_ERR_BUFFER_TOO_SMALL
  // here rather than being distinguished from "nothing written".
  const int written =
      rcOffsetPoly(verts, vert_count, offset, out_verts, max_out_verts);
  if (written == 0) return ZRC_ERR_BUFFER_TOO_SMALL;
  *out_vert_count = written;
  return ZRC_OK;
}

}  // extern "C"
