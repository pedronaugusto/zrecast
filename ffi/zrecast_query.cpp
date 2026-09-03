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

/// Lower bound, and it is not cosmetic.
///
/// dtNavMeshQuery::init builds its node pool as
/// `dtNodePool(maxNodes, dtNextPow2(maxNodes / 4))`. For maxNodes below 4 that
/// second argument is `dtNextPow2(0)`, and dtNextPow2 returns **0** for 0 (it
/// decrements, smears the bits, and increments back through the wrap). The pool
/// then allocates a zero-byte hash table, and its own
/// `dtAssert(dtNextPow2(m_hashSize) == m_hashSize)` passes because 0 == 0, so
/// even an asserts-on build says nothing.
///
/// Every later bucket index is `hash & (m_hashSize - 1)` — `hash & 0xFFFFFFFF`
/// — so the first search reads, and then *writes*, gigabytes past a zero-byte
/// allocation. Four is the smallest value for which the hash table exists.
const int32_t kMinNodes = 4;

/// Scratch used when a caller asks for a move but not for the polygons it
/// crossed. dtNavMeshQuery::moveAlongSurface rejects a null `visited`, so
/// "don't care" has to be spelled as a buffer that is thrown away.
const int32_t kDiscardVisited = 16;

// Filter translation moved to zrc::BuildFilter in ffi/zrecast_internal.h when
// the crowd's three units needed it too. Named here so the call sites below
// read as they did.
using zrc::BuildFilter;

/// The most grid cells one query may scan.
///
/// dtNavMeshQuery::queryPolygons turns the caller's box into a tile range and
/// walks every cell of it, calling getTilesAt on each — including cells no tile
/// has ever occupied (DetourNavMeshQuery.cpp:947-956). Nothing bounds that
/// range except the int a tile coordinate is cast to. Measured here at roughly
/// ten million cells a second, so this ceiling is a few tenths of a second; a
/// half-extent of 1e7 over a 50 m tile would be 1.4e11 lookups, which is not
/// undefined behaviour, merely a call that never returns.
const int64_t kMaxQueryCells = 1 << 22;

/// A query box, checked before Detour turns it into a tile range.
///
/// Two separate failures, and the first is reachable from surface this package
/// has already shipped. queryPolygons computes `center - halfExtents` and
/// `center + halfExtents` in float and hands both to dtNavMesh::calcTileLoc,
/// which is `(int)floorf((pos[0] - orig[0]) / tileWidth)` with no test of its
/// own (DetourNavMesh.cpp:1193). Upstream checks only that `center` and
/// `halfExtents` are each finite — but two finite floats sum to infinity, and
/// converting an infinite float to an int is undefined. The sanitizer catches
/// it as "outside the range of representable values of type 'int'".
///
/// The second is the cell count above. Both are decided here rather than per
/// entry point, because every box query in this file reaches the same code.
ZrcResult CheckQueryBox(const dtNavMeshQuery& query, const float* center,
                        const float* half_extents) {
  const dtNavMesh* nav = query.getAttachedNavMesh();
  if (nav == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtNavMeshParams* params = nav->getParams();
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(params->tileWidth) || !(params->tileWidth > 0.f) ||
      !zrc::IsFinite(params->tileHeight) || !(params->tileHeight > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

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
    // The quotient in double, so the bound is tested on a value that cannot
    // itself overflow on the way to being tested. The sums above are not
    // separately tested for finiteness: an infinite one divides to an infinite
    // quotient and a NaN one to a NaN, and the range test below rejects both,
    // since every comparison against a NaN is false.
    const double size = k == 0 ? params->tileWidth : params->tileHeight;
    const double origin = params->orig[axis];
    const double lo = (static_cast<double>(lo_f) - origin) / size;
    const double hi = (static_cast<double>(hi_f) - origin) / size;
    if (!(lo >= -2147483648.0 && lo < 2147483648.0)) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (!(hi >= -2147483648.0 && hi < 2147483648.0)) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    const int64_t first = static_cast<int64_t>(floor(lo));
    const int64_t last = static_cast<int64_t>(floor(hi));
    span[k] = last - first + 1;
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

/// The same, plus the filter conversion the traversal queries need.
ZrcResult BeginQuery(const ZrcNavMeshQuery* query,
                     const ZrcQueryFilter* filter, const float* a,
                     const float* b, dtQueryFilter* out_filter) {
  const ZrcResult ready = CheckQuery(query, a, b);
  if (ready != ZRC_OK) return ready;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!BuildFilter(*filter, out_filter)) return ZRC_ERR_INVALID_ARGUMENT;
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
                          const ZrcQueryFilter* filter, const float* pos,
                          dtQueryFilter* out_filter) {
  const ZrcResult ready = CheckQueryPoint(query, pos);
  if (ready != ZRC_OK) return ready;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!BuildFilter(*filter, out_filter)) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

/// Refuses a call that would clear the node pool a sliced search is using.
///
/// dtNavMeshQuery's own class comment promises that const methods have "no
/// impact on an in-progress sliced path query" (DetourNavMeshQuery.cpp:133).
/// It is false for exactly five of them: each begins by clearing the shared
/// node pool, and dtNodePool::clear resets the count without clearing the
/// storage, so a slice's best-node pointer stays valid while the nodes behind
/// it are handed to the next search. Finalising then walks a chain belonging
/// to that other search. This is the gate those five take.
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

void zrcQueryFilterDefault(ZrcQueryFilter* out) {
  if (out == nullptr) return;
  for (int i = 0; i < ZRC_MAX_AREAS; ++i) out->area_cost[i] = 1.0f;
  out->include_flags = 0xffff;
  out->exclude_flags = 0;
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

  handle->navmesh = navmesh;
  handle->slicing = false;

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

  dtQueryFilter dt_filter;
  const ZrcResult ready =
      BeginQuery(query, filter, center, half_extents, &dt_filter);
  if (ready != ZRC_OK) return ready;

  const ZrcResult box = CheckQueryBox(*query->impl, center, half_extents);
  if (box != ZRC_OK) return box;

  // Detour leaves nearestPt untouched when nothing is found, so seed it with
  // the query point: a caller that ignores the reference still reads something
  // meaningful rather than whatever was on its stack.
  //
  // The scratch is not spare: upstream writes isOverPoly only inside the
  // branch that also writes nearestPt (DetourNavMeshQuery.cpp:722-725), so a
  // caller wanting the flag and not the point would otherwise always be told
  // false. Handing it a point it then discards is what makes the two outputs
  // independent here.
  float scratch[3];
  float* nearest = out_point != nullptr ? out_point : scratch;
  for (int i = 0; i < 3; ++i) nearest[i] = center[i];

  bool over_poly = false;
  const dtStatus status = query->impl->findNearestPoly(
      center, half_extents, &dt_filter, out_ref, nearest, &over_poly);
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

  dtQueryFilter dt_filter;
  const ZrcResult ready =
      BeginQuery(query, filter, start_pos, end_pos, &dt_filter);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  // ZrcPolyRef and dtPolyRef are the same type; zrecast_abi.cpp fails the build
  // if a re-vendor changes that, so no cast is needed or wanted here.
  const dtStatus status =
      query->impl->findPath(start_ref, end_ref, start_pos, end_pos, &dt_filter,
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
  // The corridor's references are NOT scanned here. Detour validates them
  // itself and does so on every element: findStraightPath rejects a zero
  // path[0] outright, resolves path[0] and path[pathSize-1] through
  // closestPointOnPolyBoundary, and reaches every interior transition through
  // getPortalPoints, which calls the bounds-checking getTileAndPolyByRef on
  // both ends (DetourNavMeshQuery.cpp:1807, 2255). An O(path_count) scan here
  // would repeat that work once per frame per agent and catch nothing.

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

  dtQueryFilter dt_filter;
  const ZrcResult ready =
      BeginQuery(query, filter, start_pos, end_pos, &dt_filter);
  if (ready != ZRC_OK) return ready;

  ZrcPolyRef discard[kDiscardVisited];
  ZrcPolyRef* visited = out_visited != nullptr ? out_visited : discard;
  const int capacity = out_visited != nullptr ? max_visited : kDiscardVisited;

  int count = 0;
  const dtStatus status =
      query->impl->moveAlongSurface(start_ref, start_pos, end_pos, &dt_filter,
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

  dtQueryFilter dt_filter;
  const ZrcResult ready =
      BeginQuery(query, filter, start_pos, end_pos, &dt_filter);
  if (ready != ZRC_OK) return ready;

  dtRaycastHit hit;
  memset(&hit, 0, sizeof(hit));
  hit.path = out_path;
  hit.maxPath = out_path != nullptr ? max_path : 0;

  const dtStatus status = query->impl->raycast(
      start_ref, start_pos, end_pos, &dt_filter, options, &hit, prev_ref);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  // Detour reports "reached the end without hitting anything" as t == FLT_MAX,
  // which is not a lerp parameter. Normalise it to exactly 1 and say so through
  // the `hit` flag instead, so `t` is always usable arithmetic.
  //
  // The comparison is <= rather than <: FLT_MAX is the only "no hit" value, so
  // a wall struck exactly at the segment's end is a hit like any other. A NaN
  // compares false and is reported as no hit, which is the safe reading.
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
  // Built directly into the query's own copy: initSlicedFindPath stores this
  // pointer and every later update and finalise reads through it, so a stack
  // filter here would dangle the moment this call returned.
  if (!BuildFilter(*filter, &query->slice_filter)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

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
  dtQueryFilter dt_filter;
  if (!BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  ScopedRandomSource scoped(random);
  const dtStatus status = query->impl->findRandomPoint(&dt_filter, RandomThunk,
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

  dtQueryFilter dt_filter;
  const ZrcResult ready = BeginQueryPoint(query, filter, center, &dt_filter);
  if (ready != ZRC_OK) return ready;

  ScopedRandomSource scoped(random);
  const dtStatus status = query->impl->findRandomPointAroundCircle(
      start_ref, center, max_radius, &dt_filter, RandomThunk, out_ref,
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

  dtQueryFilter dt_filter;
  const ZrcResult ready =
      BeginQuery(query, filter, center, half_extents, &dt_filter);
  if (ready != ZRC_OK) return ready;

  const ZrcResult box = CheckQueryBox(*query->impl, center, half_extents);
  if (box != ZRC_OK) return box;

  int count = 0;
  const dtStatus status = query->impl->queryPolygons(
      center, half_extents, &dt_filter, out_refs, &count, max_refs);
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
  dtQueryFilter dt_filter;
  const ZrcResult ready =
      BeginQuery(query, filter, center, half_extents, &dt_filter);
  if (ready != ZRC_OK) return ready;

  const ZrcResult box = CheckQueryBox(*query->impl, center, half_extents);
  if (box != ZRC_OK) return box;

  CollectQuery collect(*sink);
  const dtStatus status =
      query->impl->queryPolygons(center, half_extents, &dt_filter, &collect);
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

  dtQueryFilter dt_filter;
  const ZrcResult ready = BeginQueryPoint(query, filter, center, &dt_filter);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  const dtStatus status = query->impl->findPolysAroundCircle(
      start_ref, center, radius, &dt_filter, out_refs, out_parents,
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
  dtQueryFilter dt_filter;
  if (!BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  int count = 0;
  const dtStatus status = query->impl->findPolysAroundShape(
      start_ref, verts, vert_count, &dt_filter, out_refs, out_parents,
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

  dtQueryFilter dt_filter;
  const ZrcResult ready = BeginQueryPoint(query, filter, center, &dt_filter);
  if (ready != ZRC_OK) return ready;

  int count = 0;
  const dtStatus status = query->impl->findLocalNeighbourhood(
      start_ref, center, radius, &dt_filter, out_refs, out_parents, &count,
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
  dtQueryFilter dt_filter;
  if (!BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  int count = 0;
  const dtStatus status = query->impl->getPolyWallSegments(
      ref, &dt_filter, out_verts, out_refs, &count, max_segments);
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

  dtQueryFilter dt_filter;
  const ZrcResult ready = BeginQueryPoint(query, filter, center, &dt_filter);
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
      start_ref, center, max_radius, &dt_filter, &hit_dist, hit_pos,
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
  dtQueryFilter dt_filter;
  if (!BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;
  *out_valid =
      query->impl->isValidPolyRef(ref, &dt_filter) ? ZRC_TRUE : ZRC_FALSE;
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
