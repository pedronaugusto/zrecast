//===----------------------------------------------------------------------===//
// zrecast — runtime navmesh queries.
//
// This is the half a shipping game links. Nothing here bakes anything; every
// entry point reads an immutable navmesh and a mutable node pool.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

#include "DetourCommon.h"
#include "DetourNode.h"

namespace {

/// Upper bound Detour itself enforces on the node pool (DT_NULL_IDX).
const int32_t kMaxNodes = 0xffff;

/// Lower bound: below 4, dtNavMeshQuery::init builds a node pool with a
/// zero-byte hash table, and the first search writes far past it.
/// See docs/design.md, Known upstream behaviour: dtNavMeshQuery::init.
const int32_t kMinNodes = 4;

/// Scratch used when a caller asks for a move but not for the polygons it
/// crossed. dtNavMeshQuery::moveAlongSurface rejects a null `visited`, so
/// "don't care" has to be spelled as a buffer that is thrown away.
const int32_t kDiscardVisited = 16;

/// The most grid cells one query may scan.
///
/// dtNavMeshQuery::queryPolygons walks every cell of the caller's box,
/// occupied or not, bounded only by the int a tile coordinate is cast to
/// (DetourNavMeshQuery.cpp:947-956). At about ten million cells a second
/// (measured), this ceiling costs a few tenths of a second.
const int64_t kMaxQueryCells = 1 << 22;

/// The largest tile coordinate a query box may reach, exclusive, on either
/// side of zero. Detour computes the coordinate in float and converts it to
/// int; the check computes it in double. Near 2^31 floats are 128 apart, so a
/// double just under 2^31 can round to 2^31 in float and overflow. Half that
/// is beyond any rounding and any grid a navmesh can address.
const double kMaxTileCoord = 1073741824.0;

/// floor() for a value already known to lie within kMaxTileCoord: one
/// conversion and a compare, without a call into libm.
int64_t FloorToInt(double v) {
  const int64_t t = static_cast<int64_t>(v);
  return static_cast<double>(t) > v ? t - 1 : t;
}

/// A query box, checked before Detour turns it into a tile range: each corner
/// must name a tile an int holds, and the range must stay within
/// kMaxQueryCells. Upstream checks `center` and `halfExtents` are finite, but
/// their sum can still be infinite. Scales by the grid zrc::BindQuery read at
/// creation; dividing here measured a tenth of a findNearestPoly.
/// See docs/design.md, Known upstream behaviour: dtNavMesh::calcTileLoc.
ZrcResult CheckQueryBox(const ZrcNavMeshQuery& query, const float* center,
                        const float* half_extents) {
  for (int i = 0; i < 3; ++i) {
    if (half_extents[i] < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  }

  // The same two sums Detour makes, in float, because it is the float sum that
  // overflows and a double one would not reproduce the failure being guarded.
  int64_t span[2] = {1, 1};
  for (int k = 0; k < 2; ++k) {
    const int axis = k == 0 ? 0 : 2;
    const float lo_f = center[axis] - half_extents[axis];
    const float hi_f = center[axis] + half_extents[axis];
    // The tile coordinate in double, so the bound is tested on a value that
    // cannot itself overflow on the way to being tested. The sums above are
    // not separately tested for finiteness: an infinite one scales to an
    // infinite coordinate and a NaN one to a NaN, and the range test below
    // rejects both, since every comparison against a NaN is false. So does a
    // grid zrc::BindQuery could not read, whose scale is NaN.
    const double lo =
        (static_cast<double>(lo_f) - query.grid_origin[k]) * query.grid_scale[k];
    const double hi =
        (static_cast<double>(hi_f) - query.grid_origin[k]) * query.grid_scale[k];
    if (!(lo > -kMaxTileCoord && lo < kMaxTileCoord)) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (!(hi > -kMaxTileCoord && hi < kMaxTileCoord)) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    span[k] = FloorToInt(hi) - FloorToInt(lo) + 1;
    // Each axis first, so the product below cannot overflow int64.
    if (span[k] > kMaxQueryCells) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (span[0] * span[1] > kMaxQueryCells) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// Shared preamble: a live query and two finite positions.
ZrcResult CheckQuery(const ZrcNavMeshQuery* query, const float* a,
                     const float* b) {
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(a) || !zrc::IsFiniteVec3(b)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// The same, plus the filter the traversal queries need.
ZrcResult BeginQuery(const ZrcNavMeshQuery* query,
                     const ZrcQueryFilter* filter, const float* a,
                     const float* b) {
  const ZrcResult ready = CheckQuery(query, a, b);
  if (ready != ZRC_OK) return ready;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// CheckQuery's shape for an entry point with one position rather than a
/// start/end pair.
ZrcResult CheckQueryPoint(const ZrcNavMeshQuery* query, const float* pos) {
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// BeginQuery's shape for an entry point with one position rather than a
/// start/end pair.
ZrcResult BeginQueryPoint(const ZrcNavMeshQuery* query,
                          const ZrcQueryFilter* filter, const float* pos) {
  const ZrcResult ready = CheckQueryPoint(query, pos);
  if (ready != ZRC_OK) return ready;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// Refuses a call that would clear the node pool a sliced search is using:
/// the gate the five pool-clearing queries take, since a cleared pool hands
/// the slice's nodes to the next search. See docs/design.md, Known upstream
/// behaviour: dtNavMeshQuery's class comment.
ZrcResult CheckNotSlicing(const ZrcNavMeshQuery* query) {
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return query->slicing ? ZRC_ERR_SEARCH_IN_PROGRESS : ZRC_OK;
}

/// Bridges upstream's context-free `float (*)()` random source to
/// ZrcRandomSource, which carries one. Thread-local so two threads with two
/// generators do not trample each other's slot.
thread_local const ZrcRandomSource* g_random = nullptr;

float RandomThunk() {
  if (g_random == nullptr || g_random->next == nullptr) return 0.f;
  return g_random->next(g_random->user);
}

/// Runs `random` under g_random for the duration of one call, restoring
/// whatever was there before rather than clearing it — a handler that
/// re-enters this layer is then not surprised by a NULL slot.
class ScopedRandomSource {
 public:
  explicit ScopedRandomSource(const ZrcRandomSource* random)
      : previous_(g_random) {
    g_random = random;
  }
  ~ScopedRandomSource() { g_random = previous_; }

 private:
  const ZrcRandomSource* previous_;
};

/// Forwards upstream's queryPolygons callback to a host's ZrcPolyQuery.
///
/// Only the references cross the boundary: `tile` and `polys` are live
/// internals of the navmesh's own storage, and everything about a polygon a
/// host might want is reachable from its reference through
/// zrcNavMeshPolyInfo.
class CollectQuery final : public dtPolyQuery {
 public:
  explicit CollectQuery(const ZrcPolyQuery& sink) : sink_(sink) {}

  void process(const dtMeshTile* /*tile*/, dtPoly** /*polys*/,
              dtPolyRef* refs, int count) override {
    sink_.process(sink_.user, refs, count);
  }

 private:
  const ZrcPolyQuery& sink_;
};

/// Largest vertex count zrcFindPolysAroundShape accepts.
///
/// Upstream indexes the vertex array as `verts[i * 3]` with `int` arithmetic,
/// so the product has to stay inside int32.
const int32_t kMaxShapeVerts = 0x7fffffff / 3;

/// Copies a dtNode's bitfields out into ZrcNode's whole fields.
void CopyNode(const dtNode& node, ZrcNode* out) {
  out->pos[0] = node.pos[0];
  out->pos[1] = node.pos[1];
  out->pos[2] = node.pos[2];
  out->cost = node.cost;
  out->total = node.total;
  out->ref = node.id;
  out->parent_index = node.pidx;
  out->state = node.state;
  out->flags = node.flags;
}

}  // namespace

extern "C" {

void zrcQueryFilterDescDefault(ZrcQueryFilterDesc* out) {
  if (out == nullptr) return;
  for (int i = 0; i < ZRC_MAX_AREAS; ++i) out->area_cost[i] = 1.0f;
  out->include_flags = 0xffff;
  out->exclude_flags = 0;
}

ZrcResult zrcQueryFilterCreate(const ZrcQueryFilterDesc* desc,
                               ZrcQueryFilter** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (desc == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  ZrcQueryFilter* handle = zrc::New<ZrcQueryFilter>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  if (!zrc::BuildFilter(*desc, &handle->impl)) {
    zrc::Delete(handle);
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out = handle;
  return ZRC_OK;
}

void zrcQueryFilterDestroy(ZrcQueryFilter* filter) { zrc::Delete(filter); }

ZrcResult zrcQueryFilterDescribe(const ZrcQueryFilter* filter,
                                 ZrcQueryFilterDesc* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::DescribeFilter(filter->impl, out);
  return ZRC_OK;
}

ZrcResult zrcQueryFilterSetAreaCost(ZrcQueryFilter* filter, uint8_t area,
                                    float cost) {
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsAreaCost(cost)) return ZRC_ERR_INVALID_ARGUMENT;
  filter->impl.setAreaCost(area, cost);
  return ZRC_OK;
}

ZrcResult zrcQueryFilterSetIncludeFlags(ZrcQueryFilter* filter,
                                        uint16_t flags) {
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  filter->impl.setIncludeFlags(flags);
  return ZRC_OK;
}

ZrcResult zrcQueryFilterSetExcludeFlags(ZrcQueryFilter* filter,
                                        uint16_t flags) {
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  filter->impl.setExcludeFlags(flags);
  return ZRC_OK;
}

ZrcResult zrcNavMeshQueryCreate(const ZrcNavMesh* navmesh, int32_t max_nodes,
                                ZrcNavMeshQuery** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_nodes < kMinNodes || max_nodes > kMaxNodes) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  ZrcNavMeshQuery* handle = zrc::New<ZrcNavMeshQuery>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = dtAllocNavMeshQuery();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  const dtStatus status = handle->impl->init(navmesh->impl, max_nodes);
  if (dtStatusFailed(status)) {
    dtFreeNavMeshQuery(handle->impl);
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }

  zrc::BindQuery(handle, handle->impl, navmesh);

  *out = handle;
  return ZRC_OK;
}

void zrcNavMeshQueryDestroy(ZrcNavMeshQuery* query) {
  if (query == nullptr) return;
  dtFreeNavMeshQuery(query->impl);
  zrc::Delete(query);
}

ZrcResult zrcFindNearestPoly(const ZrcNavMeshQuery* query, const float* center,
                             const float* half_extents,
                             const ZrcQueryFilter* filter, ZrcPolyRef* out_ref,
                             float* out_point, ZrcBool* out_over_poly) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (out_over_poly != nullptr) *out_over_poly = ZRC_FALSE;
  if (center == nullptr || half_extents == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult ready =
      BeginQuery(query, filter, center, half_extents);
  if (ready != ZRC_OK) return ready;

  const ZrcResult box = CheckQueryBox(*query, center, half_extents);
  if (box != ZRC_OK) return box;

  // Seeded with the query point, which Detour leaves untouched when nothing is
  // found. The scratch lets a caller have isOverPoly without the point.
  // See docs/design.md, Known upstream behaviour: findNearestPoly.
  float scratch[3];
  float* nearest = out_point != nullptr ? out_point : scratch;
  for (int i = 0; i < 3; ++i) nearest[i] = center[i];

  bool over_poly = false;
  const dtStatus status = query->impl->findNearestPoly(
      center, half_extents, &filter->impl, out_ref, nearest, &over_poly);
  if (dtStatusFailed(status)) {
    *out_ref = 0;
    return zrc::ResultFromStatus(status);
  }
  if (out_over_poly != nullptr) {
    *out_over_poly = over_poly ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcFindPath(const ZrcNavMeshQuery* query, ZrcPolyRef start_ref,
                      ZrcPolyRef end_ref, const float* start_pos,
                      const float* end_pos, const ZrcQueryFilter* filter,
                      ZrcPolyRef* out_path, int32_t max_path,
                      int32_t* out_count, ZrcBool* out_partial) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_partial != nullptr) *out_partial = ZRC_FALSE;
  if (out_path == nullptr || max_path < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (start_pos == nullptr || end_pos == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (start_ref == 0 || end_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;

  // findPath is one of the five entry points that clears the shared node
  // pool (DetourNavMeshQuery.cpp:1003); see CheckNotSlicing.
  const ZrcResult slicing = CheckNotSlicing(query);
  if (slicing != ZRC_OK) return slicing;

  const ZrcResult ready =
      BeginQuery(query, filter, start_pos, end_pos);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  // ZrcPolyRef and dtPolyRef are the same type; zrecast_abi.cpp fails the build
  // if a re-vendor changes that, so no cast is needed or wanted here.
  const dtStatus status =
      query->impl->findPath(start_ref, end_ref, start_pos, end_pos, &filter->impl,
                            out_path, &count, max_path);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  if (count < 0 || count > max_path) {
    // Defensive: a count outside the buffer would make the caller read past it.
    return ZRC_ERR_QUERY_FAILED;
  }
  *out_count = count;
  if (out_partial != nullptr) {
    *out_partial = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcFindStraightPath(const ZrcNavMeshQuery* query,
                              const float* start_pos, const float* end_pos,
                              const ZrcPolyRef* path, int32_t path_count,
                              uint32_t options, float* out_points,
                              int32_t max_points, uint8_t* out_flags,
                              int32_t max_flags, ZrcPolyRef* out_refs,
                              int32_t max_refs, int32_t* out_count,
                              ZrcBool* out_partial) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_partial != nullptr) *out_partial = ZRC_FALSE;
  if (out_points == nullptr || max_points < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (path == nullptr || path_count < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (start_pos == nullptr || end_pos == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const uint32_t kStraightPathOptionsMask =
      static_cast<uint32_t>(ZRC_STRAIGHTPATH_AREA_CROSSINGS) |
      static_cast<uint32_t>(ZRC_STRAIGHTPATH_ALL_CROSSINGS);
  if ((options & ~kStraightPathOptionsMask) != 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // appendVertex writes all three output arrays at the same index and bounds
  // only the first, so a short companion is an overflow rather than a truncated
  // result. This is the check that makes the C boundary safe on its own.
  if (out_flags != nullptr && max_flags < max_points) {
    return ZRC_ERR_BUFFER_TOO_SMALL;
  }
  if (out_refs != nullptr && max_refs < max_points) {
    return ZRC_ERR_BUFFER_TOO_SMALL;
  }
  // The corridor's references are not scanned here: findStraightPath already
  // validates every element (DetourNavMeshQuery.cpp:1807, 2255), so an
  // O(path_count) scan would repeat that work per frame per agent and catch
  // nothing.

  // The string-pull walks a corridor that findPath already filtered, so it
  // takes no filter of its own — hence CheckQuery rather than BeginQuery.
  const ZrcResult ready = CheckQuery(query, start_pos, end_pos);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  const dtStatus status = query->impl->findStraightPath(
      start_pos, end_pos, path, path_count, out_points, out_flags, out_refs,
      &count, max_points, static_cast<int>(options));
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  if (count < 0 || count > max_points) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_partial != nullptr) {
    *out_partial = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcMoveAlongSurface(const ZrcNavMeshQuery* query,
                              ZrcPolyRef start_ref, const float* start_pos,
                              const float* end_pos,
                              const ZrcQueryFilter* filter, float* out_pos,
                              ZrcPolyRef* out_visited, int32_t max_visited,
                              int32_t* out_visited_count,
                              ZrcBool* out_truncated) {
  if (out_visited_count != nullptr) *out_visited_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (out_pos == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (start_pos == nullptr || end_pos == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (out_visited != nullptr && max_visited < 1) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult ready =
      BeginQuery(query, filter, start_pos, end_pos);
  if (ready != ZRC_OK) return ready;

  ZrcPolyRef discard[kDiscardVisited];
  ZrcPolyRef* visited = out_visited != nullptr ? out_visited : discard;
  const int capacity = out_visited != nullptr ? max_visited : kDiscardVisited;

  int count = 0;
  const dtStatus status =
      query->impl->moveAlongSurface(start_ref, start_pos, end_pos, &filter->impl,
                                    out_pos, visited, &count, capacity);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  if (count < 0 || count > capacity) return ZRC_ERR_QUERY_FAILED;
  if (out_visited_count != nullptr) {
    *out_visited_count = out_visited != nullptr ? count : 0;
  }
  // Detour reports a clipped visited list as DT_SUCCESS | DT_BUFFER_TOO_SMALL,
  // which would otherwise be indistinguishable from a complete one.
  if (out_truncated != nullptr && out_visited != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcRaycast(const ZrcNavMeshQuery* query, ZrcPolyRef start_ref,
                     const float* start_pos, const float* end_pos,
                     const ZrcQueryFilter* filter, uint32_t options,
                     ZrcPolyRef prev_ref, ZrcRaycastHit* out_hit,
                     ZrcPolyRef* out_path, int32_t max_path,
                     int32_t* out_path_count, ZrcBool* out_truncated) {
  if (out_hit == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out_hit, 0, sizeof(*out_hit));
  // Upstream can return without ever writing hitEdgeIndex, when the very
  // first polygon's intersection test fails (DetourNavMeshQuery.cpp:2527-2532).
  // Seeded here rather than left as whatever memset above happened to leave.
  out_hit->hit_edge_index = -1;
  if (out_path_count != nullptr) *out_path_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (start_pos == nullptr || end_pos == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (out_path != nullptr && max_path < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if ((options & ~static_cast<uint32_t>(ZRC_RAYCAST_USE_COSTS)) != 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult ready =
      BeginQuery(query, filter, start_pos, end_pos);
  if (ready != ZRC_OK) return ready;

  dtRaycastHit hit;
  memset(&hit, 0, sizeof(hit));
  hit.path = out_path;
  hit.maxPath = out_path != nullptr ? max_path : 0;

  const dtStatus status = query->impl->raycast(
      start_ref, start_pos, end_pos, &filter->impl, options, &hit, prev_ref);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  // Detour reports "no hit" as t == FLT_MAX, which is no lerp parameter; it is
  // normalised to 1 and reported through `hit`. The compare is <= because
  // FLT_MAX is the only "no hit" value, so a wall exactly at the segment's end
  // is a hit; a NaN compares false and reads as no hit, the safe reading.
  const bool struck = hit.t <= 1.0f;
  const float t = struck ? hit.t : 1.0f;

  out_hit->t = t;
  out_hit->hit = struck ? ZRC_TRUE : ZRC_FALSE;
  for (int i = 0; i < 3; ++i) {
    out_hit->position[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * t;
    out_hit->normal[i] = struck ? hit.hitNormal[i] : 0.f;
  }
  out_hit->hit_edge_index = hit.hitEdgeIndex;
  out_hit->path_cost = hit.pathCost;

  if (hit.pathCount < 0 || hit.pathCount > hit.maxPath) {
    return ZRC_ERR_QUERY_FAILED;
  }
  if (out_path_count != nullptr) *out_path_count = hit.pathCount;
  if (out_truncated != nullptr && out_path != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Sliced pathfinding
//===----------------------------------------------------------------------===//

ZrcResult zrcSlicedFindPathInit(ZrcNavMeshQuery* query, ZrcPolyRef start_ref,
                                ZrcPolyRef end_ref, const float* start_pos,
                                const float* end_pos,
                                const ZrcQueryFilter* filter,
                                uint32_t options) {
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(start_pos) || !zrc::IsFiniteVec3(end_pos)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (start_ref == 0 || end_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if ((options & ~static_cast<uint32_t>(ZRC_FINDPATH_ANY_ANGLE)) != 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  // Copied into the query's own: initSlicedFindPath stores this pointer and
  // every later update and finalise reads through it, so the caller's filter
  // would dangle the moment the caller destroyed it.
  query->slice_filter = filter->impl;

  const dtStatus status = query->impl->initSlicedFindPath(
      start_ref, end_ref, start_pos, end_pos, &query->slice_filter, options);
  if (dtStatusFailed(status)) {
    query->slicing = false;
    return zrc::ResultFromStatus(status);
  }
  query->slicing = true;
  return ZRC_OK;
}

ZrcResult zrcSlicedFindPathUpdate(ZrcNavMeshQuery* query, int32_t max_iters,
                                  int32_t* out_iters,
                                  ZrcBool* out_in_progress) {
  if (out_iters != nullptr) *out_iters = 0;
  if (out_in_progress != nullptr) *out_in_progress = ZRC_FALSE;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!query->slicing) return ZRC_ERR_NO_SEARCH;
  if (max_iters < 1) return ZRC_ERR_INVALID_ARGUMENT;

  int iters = 0;
  const dtStatus status =
      query->impl->updateSlicedFindPath(max_iters, &iters);
  if (out_iters != nullptr) *out_iters = iters;
  if (out_in_progress != nullptr) {
    *out_in_progress = dtStatusInProgress(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  if (dtStatusFailed(status)) {
    // The start or end polygon stopped being valid mid-flight; upstream
    // fails right here rather than waiting for finalise to notice.
    query->slicing = false;
    return zrc::ResultFromStatus(status);
  }
  return ZRC_OK;
}

ZrcResult zrcSlicedFindPathFinalize(ZrcNavMeshQuery* query,
                                    ZrcPolyRef* out_path, int32_t max_path,
                                    int32_t* out_count,
                                    ZrcBool* out_partial) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_partial != nullptr) *out_partial = ZRC_FALSE;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!query->slicing) return ZRC_ERR_NO_SEARCH;
  // Arguments first, and the flag only once they are known good: a caller who
  // gets the buffer wrong should get an error, not an error and a search
  // silently thrown away.
  if (out_path == nullptr || max_path < 1) return ZRC_ERR_INVALID_ARGUMENT;

  // Cleared before upstream is called, on every path from here on. This is
  // what closes the double-finalise hazard: with the flag already clear, a
  // second call is ZRC_ERR_NO_SEARCH above and never reaches upstream's own
  // zeroed-state branch, which would otherwise misread as "start and end are
  // the same polygon" and return a bogus one-element path
  // (DetourNavMeshQuery.cpp:1516-1520, 1579).
  query->slicing = false;

  int count = 0;
  const dtStatus status =
      query->impl->finalizeSlicedFindPath(out_path, &count, max_path);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_path) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_partial != nullptr) {
    *out_partial = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcSlicedFindPathFinalizePartial(
    ZrcNavMeshQuery* query, const ZrcPolyRef* existing, int32_t existing_count,
    ZrcPolyRef* out_path, int32_t max_path, int32_t* out_count,
    ZrcBool* out_partial) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_partial != nullptr) *out_partial = ZRC_FALSE;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!query->slicing) return ZRC_ERR_NO_SEARCH;
  if (existing == nullptr || existing_count < 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (out_path == nullptr || max_path < 1) return ZRC_ERR_INVALID_ARGUMENT;
  // Same argument-order and hazard-closing rationale as
  // zrcSlicedFindPathFinalize above.
  query->slicing = false;

  int count = 0;
  const dtStatus status = query->impl->finalizeSlicedFindPathPartial(
      existing, existing_count, out_path, &count, max_path);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_path) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_partial != nullptr) {
    *out_partial = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcSlicedFindPathCancel(ZrcNavMeshQuery* query) {
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  query->slicing = false;
  return ZRC_OK;
}

ZrcBool zrcSlicedFindPathActive(const ZrcNavMeshQuery* query) {
  if (query == nullptr) return ZRC_FALSE;
  return query->slicing ? ZRC_TRUE : ZRC_FALSE;
}

//===----------------------------------------------------------------------===//
// Random points
//===----------------------------------------------------------------------===//

ZrcResult zrcFindRandomPoint(const ZrcNavMeshQuery* query,
                             const ZrcQueryFilter* filter,
                             const ZrcRandomSource* random,
                             ZrcPolyRef* out_ref, float* out_point) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (out_point != nullptr) {
    out_point[0] = out_point[1] = out_point[2] = 0.f;
  }
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (out_point == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (random == nullptr || random->next == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  ScopedRandomSource scoped(random);
  const dtStatus status = query->impl->findRandomPoint(&filter->impl, RandomThunk,
                                                        out_ref, out_point);
  if (dtStatusFailed(status)) {
    *out_ref = 0;
    return zrc::ResultFromStatus(status);
  }
  return ZRC_OK;
}

ZrcResult zrcFindRandomPointAroundCircle(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float max_radius, const ZrcQueryFilter* filter,
    const ZrcRandomSource* random, ZrcPolyRef* out_ref, float* out_point) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (out_point != nullptr) {
    out_point[0] = out_point[1] = out_point[2] = 0.f;
  }
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (out_point == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(max_radius) || max_radius < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (random == nullptr || random->next == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // One of the five entry points that clears the shared node pool
  // (DetourNavMeshQuery.cpp:340); see CheckNotSlicing.
  const ZrcResult slicing = CheckNotSlicing(query);
  if (slicing != ZRC_OK) return slicing;

  const ZrcResult ready = BeginQueryPoint(query, filter, center);
  if (ready != ZRC_OK) return ready;

  ScopedRandomSource scoped(random);
  const dtStatus status = query->impl->findRandomPointAroundCircle(
      start_ref, center, max_radius, &filter->impl, RandomThunk, out_ref,
      out_point);
  if (dtStatusFailed(status)) {
    *out_ref = 0;
    return zrc::ResultFromStatus(status);
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Polygons in a box
//===----------------------------------------------------------------------===//

ZrcResult zrcQueryPolygons(const ZrcNavMeshQuery* query, const float* center,
                           const float* half_extents,
                           const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
                           int32_t max_refs, int32_t* out_count,
                           ZrcBool* out_truncated) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (out_refs == nullptr || max_refs < 0) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult ready =
      BeginQuery(query, filter, center, half_extents);
  if (ready != ZRC_OK) return ready;

  const ZrcResult box = CheckQueryBox(*query, center, half_extents);
  if (box != ZRC_OK) return box;

  int count = 0;
  const dtStatus status = query->impl->queryPolygons(
      center, half_extents, &filter->impl, out_refs, &count, max_refs);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_refs) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_truncated != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcQueryPolygonsBatched(const ZrcNavMeshQuery* query,
                                  const float* center,
                                  const float* half_extents,
                                  const ZrcQueryFilter* filter,
                                  const ZrcPolyQuery* sink) {
  if (sink == nullptr || sink->process == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const ZrcResult ready =
      BeginQuery(query, filter, center, half_extents);
  if (ready != ZRC_OK) return ready;

  const ZrcResult box = CheckQueryBox(*query, center, half_extents);
  if (box != ZRC_OK) return box;

  CollectQuery collect(*sink);
  const dtStatus status =
      query->impl->queryPolygons(center, half_extents, &filter->impl, &collect);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// A point against one polygon
//===----------------------------------------------------------------------===//

ZrcResult zrcClosestPointOnPoly(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                                const float* pos, float* out_point,
                                ZrcBool* out_over_poly) {
  if (out_point == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_point[0] = out_point[1] = out_point[2] = 0.f;
  if (out_over_poly != nullptr) *out_over_poly = ZRC_FALSE;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;
  if (ref == 0) return ZRC_ERR_INVALID_ARGUMENT;

  bool over_poly = false;
  const dtStatus status =
      query->impl->closestPointOnPoly(ref, pos, out_point, &over_poly);
  if (dtStatusFailed(status)) {
    out_point[0] = out_point[1] = out_point[2] = 0.f;
    return zrc::ResultFromStatus(status);
  }
  if (out_over_poly != nullptr) {
    *out_over_poly = over_poly ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcClosestPointOnPolyBoundary(const ZrcNavMeshQuery* query,
                                        ZrcPolyRef ref, const float* pos,
                                        float* out_point) {
  if (out_point == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_point[0] = out_point[1] = out_point[2] = 0.f;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;
  if (ref == 0) return ZRC_ERR_INVALID_ARGUMENT;

  const dtStatus status =
      query->impl->closestPointOnPolyBoundary(ref, pos, out_point);
  if (dtStatusFailed(status)) {
    out_point[0] = out_point[1] = out_point[2] = 0.f;
    return zrc::ResultFromStatus(status);
  }
  return ZRC_OK;
}

ZrcResult zrcPolyHeight(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                        const float* pos, float* out_height) {
  if (out_height == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_height = 0.f;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;
  if (ref == 0) return ZRC_ERR_INVALID_ARGUMENT;

  const dtStatus status = query->impl->getPolyHeight(ref, pos, out_height);
  if (dtStatusFailed(status)) {
    *out_height = 0.f;
    return zrc::ResultFromStatus(status);
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Searching outwards
//===----------------------------------------------------------------------===//

ZrcResult zrcFindPolysAroundCircle(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float radius, const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
    ZrcPolyRef* out_parents, float* out_costs, int32_t max_result,
    int32_t* out_count, ZrcBool* out_truncated) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(radius) || radius < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_result < 0) return ZRC_ERR_INVALID_ARGUMENT;

  // One of the five entry points that clears the shared node pool
  // (DetourNavMeshQuery.cpp:2742); see CheckNotSlicing.
  const ZrcResult slicing = CheckNotSlicing(query);
  if (slicing != ZRC_OK) return slicing;

  const ZrcResult ready = BeginQueryPoint(query, filter, center);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  const dtStatus status = query->impl->findPolysAroundCircle(
      start_ref, center, radius, &filter->impl, out_refs, out_parents,
      out_costs, &count, max_result);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_result) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_truncated != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcFindPolysAroundShape(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* verts,
    int32_t vert_count, const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
    ZrcPolyRef* out_parents, float* out_costs, int32_t max_result,
    int32_t* out_count, ZrcBool* out_truncated) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (vert_count < 3 || vert_count > kMaxShapeVerts) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  for (int32_t i = 0; i < vert_count * 3; ++i) {
    if (!zrc::IsFinite(verts[i])) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_result < 0) return ZRC_ERR_INVALID_ARGUMENT;

  // One of the five entry points that clears the shared node pool
  // (DetourNavMeshQuery.cpp:2919); see CheckNotSlicing.
  const ZrcResult slicing = CheckNotSlicing(query);
  if (slicing != ZRC_OK) return slicing;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  int count = 0;
  const dtStatus status = query->impl->findPolysAroundShape(
      start_ref, verts, vert_count, &filter->impl, out_refs, out_parents,
      out_costs, &count, max_result);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_result) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_truncated != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcPathFromDijkstraSearch(const ZrcNavMeshQuery* query,
                                    ZrcPolyRef end_ref, ZrcPolyRef* out_path,
                                    int32_t max_path, int32_t* out_count,
                                    ZrcBool* out_truncated) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (out_path == nullptr || max_path < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (end_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;

  int count = 0;
  const dtStatus status = query->impl->getPathFromDijkstraSearch(
      end_ref, out_path, &count, max_path);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_path) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_truncated != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcFindLocalNeighbourhood(
    const ZrcNavMeshQuery* query, ZrcPolyRef start_ref, const float* center,
    float radius, const ZrcQueryFilter* filter, ZrcPolyRef* out_refs,
    ZrcPolyRef* out_parents, int32_t max_result, int32_t* out_count,
    ZrcBool* out_truncated) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  // Upstream writes resultRef[0] unconditionally once maxResult >= 1, with no
  // null check of its own (DetourNavMeshQuery.cpp:3135), unlike its two
  // Dijkstra siblings above which guard every write through resultRef.
  if (out_refs == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(radius) || radius < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_result < 0) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult ready = BeginQueryPoint(query, filter, center);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  const dtStatus status = query->impl->findLocalNeighbourhood(
      start_ref, center, radius, &filter->impl, out_refs, out_parents, &count,
      max_result);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_result) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_truncated != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Walls
//===----------------------------------------------------------------------===//

ZrcResult zrcPolyWallSegments(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                              const ZrcQueryFilter* filter, float* out_verts,
                              ZrcPolyRef* out_refs, int32_t max_segments,
                              int32_t* out_count, ZrcBool* out_truncated) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out_truncated != nullptr) *out_truncated = ZRC_FALSE;
  if (out_verts == nullptr || max_segments < 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  int count = 0;
  const dtStatus status = query->impl->getPolyWallSegments(
      ref, &filter->impl, out_verts, out_refs, &count, max_segments);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (count < 0 || count > max_segments) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  if (out_truncated != nullptr) {
    *out_truncated = zrc::StatusIsPartial(status) ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcFindDistanceToWall(const ZrcNavMeshQuery* query,
                                ZrcPolyRef start_ref, const float* center,
                                float max_radius, const ZrcQueryFilter* filter,
                                float* out_dist, float* out_pos,
                                float* out_normal, ZrcBool* out_found) {
  if (out_found == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_found = ZRC_FALSE;
  if (out_dist == nullptr || out_pos == nullptr || out_normal == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_dist = 0.f;
  out_pos[0] = out_pos[1] = out_pos[2] = 0.f;
  out_normal[0] = out_normal[1] = out_normal[2] = 0.f;
  if (start_ref == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(max_radius) || max_radius < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // One of the five entry points that clears the shared node pool
  // (DetourNavMeshQuery.cpp:3487); see CheckNotSlicing.
  const ZrcResult slicing = CheckNotSlicing(query);
  if (slicing != ZRC_OK) return slicing;

  const ZrcResult ready = BeginQueryPoint(query, filter, center);
  if (ready != ZRC_OK) return ready;

  // Upstream leaves hitPos untouched when nothing is in range and then
  // unconditionally computes hitNormal = normalize(centerPos - hitPos) over
  // whatever was there, reporting DT_SUCCESS either way
  // (DetourNavMeshQuery.cpp:3648-3652). Seeding hitPos with center makes that
  // subtraction (0, 0, 0) rather than garbage; *out_dist == max_radius (its
  // documented "nothing found" convention) is the signal used below.
  float hit_pos[3] = {center[0], center[1], center[2]};
  float hit_normal[3] = {0.f, 0.f, 0.f};
  float hit_dist = 0.f;
  const dtStatus status = query->impl->findDistanceToWall(
      start_ref, center, max_radius, &filter->impl, &hit_dist, hit_pos,
      hit_normal);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  if (hit_dist >= max_radius) {
    *out_dist = max_radius;
    return ZRC_OK;
  }

  *out_dist = hit_dist;
  *out_found = ZRC_TRUE;
  out_pos[0] = hit_pos[0];
  out_pos[1] = hit_pos[1];
  out_pos[2] = hit_pos[2];
  // dtVnormalize has no zero-length guard, so an agent standing exactly on
  // the wall (hitPos == centerPos) gets a NaN normal. Reported as the zeroed
  // normal already seeded above instead, with the hit still reported found.
  if (zrc::IsFiniteVec3(hit_normal)) {
    out_normal[0] = hit_normal[0];
    out_normal[1] = hit_normal[1];
    out_normal[2] = hit_normal[2];
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// What a reference means
//===----------------------------------------------------------------------===//

ZrcResult zrcIsValidPolyRef(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                            const ZrcQueryFilter* filter,
                            ZrcBool* out_valid) {
  if (out_valid == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_valid = ZRC_FALSE;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (filter == nullptr) {
    const dtNavMesh* nav = query->impl->getAttachedNavMesh();
    if (nav == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
    *out_valid = nav->isValidPolyRef(ref) ? ZRC_TRUE : ZRC_FALSE;
    return ZRC_OK;
  }
  *out_valid =
      query->impl->isValidPolyRef(ref, &filter->impl) ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcIsInClosedList(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                            ZrcBool* out_closed) {
  if (out_closed == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_closed = ZRC_FALSE;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_closed = query->impl->isInClosedList(ref) ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcQueryNavMesh(const ZrcNavMeshQuery* query,
                          const ZrcNavMesh** out_navmesh) {
  if (out_navmesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_navmesh = nullptr;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_navmesh = query->navmesh;
  return ZRC_OK;
}

ZrcResult zrcDecodePolyRef(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                           uint32_t* out_salt, uint32_t* out_tile,
                           uint32_t* out_poly) {
  if (out_salt != nullptr) *out_salt = 0;
  if (out_tile != nullptr) *out_tile = 0;
  if (out_poly != nullptr) *out_poly = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  unsigned int salt = 0;
  unsigned int tile = 0;
  unsigned int poly = 0;
  navmesh->impl->decodePolyId(ref, salt, tile, poly);
  if (out_salt != nullptr) *out_salt = salt;
  if (out_tile != nullptr) *out_tile = tile;
  if (out_poly != nullptr) *out_poly = poly;
  return ZRC_OK;
}

ZrcResult zrcEncodePolyRef(const ZrcNavMesh* navmesh, uint32_t salt,
                           uint32_t tile, uint32_t poly,
                           ZrcPolyRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtNavMeshParams* params = navmesh->impl->getParams();
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  // The same arithmetic dtNavMesh::init uses to size a reference's three
  // fields (DetourNavMesh.cpp:255-260); upstream keeps the widths themselves
  // as private members with no accessor, so they are rederived here.
  const unsigned int tile_bits =
      dtIlog2(dtNextPow2(static_cast<unsigned int>(params->maxTiles)));
  const unsigned int poly_bits =
      dtIlog2(dtNextPow2(static_cast<unsigned int>(params->maxPolys)));
  const unsigned int wide = 32u - tile_bits - poly_bits;
  const unsigned int salt_bits = 31u < wide ? 31u : wide;

  // Upstream truncates a field that does not fit silently; refused here
  // instead. The `< 32` guards keep `1u << width` defined; zrcNavMeshCreateTiled
  // already bounds tile_bits + poly_bits to 22, so all three widths are well
  // under 32 for any navmesh this package created.
  if (tile_bits < 32 && tile >= (1u << tile_bits)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (poly_bits < 32 && poly >= (1u << poly_bits)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (salt_bits < 32 && salt >= (1u << salt_bits)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  *out_ref = navmesh->impl->encodePolyId(salt, tile, poly);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The search's own node pool
//===----------------------------------------------------------------------===//

ZrcResult zrcQueryNodePoolInfo(const ZrcNavMeshQuery* query,
                               ZrcNodePoolInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out->node_count = 0;
  out->max_nodes = 0;
  out->hash_size = 0;
  out->bytes_used = 0;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtNodePool* pool = query->impl->getNodePool();
  if (pool == nullptr) return ZRC_OK;
  out->node_count = pool->getNodeCount();
  out->max_nodes = pool->getMaxNodes();
  out->hash_size = pool->getHashSize();
  out->bytes_used = pool->getMemUsed();
  return ZRC_OK;
}

ZrcResult zrcQueryFindNode(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                           uint32_t state, ZrcNode* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (state >= ZRC_MAX_NODE_STATES) return ZRC_ERR_INVALID_ARGUMENT;
  dtNodePool* pool = query->impl->getNodePool();
  if (pool == nullptr) return ZRC_ERR_NOT_FOUND;
  dtNode* node = pool->findNode(ref, static_cast<unsigned char>(state));
  if (node == nullptr) return ZRC_ERR_NOT_FOUND;
  CopyNode(*node, out);
  return ZRC_OK;
}

ZrcResult zrcQueryFindNodes(const ZrcNavMeshQuery* query, ZrcPolyRef ref,
                            ZrcNode* out, int32_t max_nodes,
                            int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (out == nullptr || max_nodes < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  dtNodePool* pool = query->impl->getNodePool();
  if (pool == nullptr) return ZRC_ERR_NOT_FOUND;

  const int32_t capacity =
      max_nodes < ZRC_MAX_NODE_STATES ? max_nodes : ZRC_MAX_NODE_STATES;
  dtNode* scratch[ZRC_MAX_NODE_STATES];
  const unsigned int found = pool->findNodes(ref, scratch, capacity);
  for (unsigned int i = 0; i < found; ++i) {
    CopyNode(*scratch[i], &out[i]);
  }
  *out_count = static_cast<int32_t>(found);
  return ZRC_OK;
}

ZrcResult zrcQueryNodeAt(const ZrcNavMeshQuery* query, uint32_t index,
                         ZrcNode* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index == 0) return ZRC_ERR_NOT_FOUND;
  dtNodePool* pool = query->impl->getNodePool();
  if (pool == nullptr) return ZRC_ERR_NOT_FOUND;
  // getNodeAtIdx bounds-checks only index == 0 and otherwise indexes straight
  // into the pool's storage (DetourNode.h:68-72); an index beyond what the
  // last search actually populated reads live but uninitialised memory rather
  // than faulting. Bounding against the node count the pool itself reports
  // keeps this to what a search has actually written.
  if (index > static_cast<uint32_t>(pool->getNodeCount())) {
    return ZRC_ERR_NOT_FOUND;
  }
  dtNode* node = pool->getNodeAtIdx(index);
  if (node == nullptr) return ZRC_ERR_NOT_FOUND;
  CopyNode(*node, out);
  return ZRC_OK;
}

}  // extern "C"
