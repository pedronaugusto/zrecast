//===----------------------------------------------------------------------===//
// zrecast — the path corridor, the local boundary and the path queue: the
// three pieces DetourCrowd builds an agent's local movement from, each usable
// on its own by a host steering movement by hand.
//
// A corridor is the polygons an agent is walking through, edited as it moves.
// A boundary is the nearby walls, cached until the agent has moved far enough
// to need new ones. A queue runs long searches a slice at a time across
// frames and hands results back whenever they are ready.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Search-in-progress gate
//===----------------------------------------------------------------------===//

/// Refuses a call that would clear the node pool a sliced search is using.
///
/// Mirrors the file-local gate of the same name in ffi/zrecast_query.cpp:
/// dtNavMeshQuery's own class comment promises that a const method has "no
/// impact on an in-progress sliced path query", and that promise is false for
/// five of them, each of which clears the shared node pool a slice's best-node
/// pointer still points into. Every entry point below that reaches one of
/// those five through a corridor or a boundary shares the same hazard.
ZrcResult CheckNotSlicing(const ZrcNavMeshQuery* query) {
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return query->slicing ? ZRC_ERR_SEARCH_IN_PROGRESS : ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Corridor-splicing argument checks
//===----------------------------------------------------------------------===//

/// Shared by the three merge free functions below.
///
/// `path` is an in/out buffer of `max_path` capacity: every one of upstream's
/// three implementations can write into it even when `path_count` is 0, so it
/// must be non-NULL whenever there is any capacity to write into
/// (DetourPathCorridor.cpp:56-67, 140-153). `visited` need only be non-NULL
/// when there is something in it to read.
ZrcResult CheckMergeArgs(const ZrcPolyRef* path, int32_t path_count,
                         int32_t max_path, const ZrcPolyRef* visited,
                         int32_t visited_count) {
  if (max_path < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (path_count < 0 || path_count > max_path) return ZRC_ERR_INVALID_ARGUMENT;
  if (visited_count < 0 || visited_count > max_path) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_path > 0 && path == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (visited_count > 0 && visited == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Node-pool floor
//===----------------------------------------------------------------------===//

/// Same hazard, same floor, as ffi/zrecast_query.cpp's own kMinNodes: a
/// dtPathQueue::init call this file makes reaches the identical
/// dtNavMeshQuery::init as zrcNavMeshQueryCreate, and a node count under four
/// hits the same dtNextPow2(0) collapse to a zero-byte hash table there
/// (DetourNavMeshQuery.cpp, dtNodePool construction). Declared again here
/// rather than shared because zrecast_query.cpp's copy is file-local.
const int32_t kMinSearchNodes = 4;
const int32_t kMaxSearchNodes = 65535;

//===----------------------------------------------------------------------===//
// Per-queue bookkeeping this package owns because ZrcPathQueue has no field
// for it.
//
// zrecast_internal.h's ZrcPathQueue carries only `impl`, `navmesh`,
// `max_path_size` and `owns` — no room for the per-slot filter ownership
//===----------------------------------------------------------------------===//

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// The path corridor
//===----------------------------------------------------------------------===//

ZrcResult zrcPathCorridorCreate(int32_t max_path, ZrcPathCorridor** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (max_path < ZRC_PATH_CORRIDOR_MIN_PATH || max_path > 65535) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  ZrcPathCorridor* handle = zrc::New<ZrcPathCorridor>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = zrc::New<dtPathCorridor>();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  // init() never frees a previous allocation of its own
  // (DetourPathCorridor.cpp:215-224, guarded only by an assert that compiles
  // away), so this handle's impl is init()'d exactly once, here, and offers
  // no re-init entry point.
  if (!handle->impl->init(max_path)) {
    zrc::Delete(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->max_path = max_path;
  *out = handle;
  return ZRC_OK;
}

void zrcPathCorridorDestroy(ZrcPathCorridor* corridor) {
  if (corridor == nullptr) return;
  zrc::Delete(corridor->impl);
  zrc::Delete(corridor);
}

ZrcResult zrcPathCorridorReset(ZrcPathCorridor* corridor, ZrcPolyRef poly,
                               const float* position) {
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;
  corridor->impl->reset(poly, position);
  return ZRC_OK;
}

ZrcResult zrcPathCorridorSetCorridor(ZrcPathCorridor* corridor,
                                     const float* target,
                                     const ZrcPolyRef* path,
                                     int32_t path_count) {
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(target)) return ZRC_ERR_INVALID_ARGUMENT;
  if (path == nullptr || path_count < 1 || path_count > corridor->max_path) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  corridor->impl->setCorridor(target, path, path_count);
  return ZRC_OK;
}

ZrcResult zrcPathCorridorFindCorners(ZrcPathCorridor* corridor,
                                     ZrcNavMeshQuery* query,
                                     const ZrcQueryFilter* filter,
                                     ZrcCrowdCorner* out, int32_t max_corners,
                                     int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Bounded above as well as below, for the reason every capacity in this
  // package is: the three scratch buffers below multiply it by a struct size,
  // and on a target whose size_t is 32 bits a large enough count wraps that
  // product to a small allocation while upstream still writes the full count
  // into it.
  if (out == nullptr || max_corners < 2 || max_corners > 65535) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  zrc::TempBuffer verts_buf(sizeof(float) * 3 *
                            static_cast<size_t>(max_corners));
  zrc::TempBuffer flags_buf(sizeof(unsigned char) *
                            static_cast<size_t>(max_corners));
  zrc::TempBuffer polys_buf(sizeof(dtPolyRef) *
                            static_cast<size_t>(max_corners));
  if (verts_buf.get() == nullptr || flags_buf.get() == nullptr ||
      polys_buf.get() == nullptr) {
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  float* verts = static_cast<float*>(verts_buf.get());
  unsigned char* flags = static_cast<unsigned char*>(flags_buf.get());
  dtPolyRef* polys = static_cast<dtPolyRef*>(polys_buf.get());

  const int ncorners = corridor->impl->findCorners(
      verts, flags, polys, max_corners, query->impl, &dt_filter);
  // No truncation check goes here: findCorners calls
  // navquery->findStraightPath without ever capturing its return value
  // (DetourPathCorridor.cpp:261), so neither a clipped string-pull nor a
  // failed one is distinguishable from a short-but-complete result. Unlike
  // zrcFindStraightPath, ZRC_ERR_BUFFER_TOO_SMALL is not reachable here.
  if (ncorners < 0 || ncorners > max_corners) return ZRC_ERR_QUERY_FAILED;

  for (int i = 0; i < ncorners; ++i) {
    out[i].position[0] = verts[i * 3 + 0];
    out[i].position[1] = verts[i * 3 + 1];
    out[i].position[2] = verts[i * 3 + 2];
    out[i].flags = flags[i];
    out[i].poly = static_cast<ZrcPolyRef>(polys[i]);
  }
  *out_count = ncorners;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorOptimizeVisibility(ZrcPathCorridor* corridor,
                                            const float* next, float range,
                                            ZrcNavMeshQuery* query,
                                            const ZrcQueryFilter* filter) {
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(next)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPositiveFinite(range)) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  corridor->impl->optimizePathVisibility(next, range, query->impl,
                                         &dt_filter);
  return ZRC_OK;
}

ZrcResult zrcPathCorridorOptimizeTopology(ZrcPathCorridor* corridor,
                                          ZrcNavMeshQuery* query,
                                          const ZrcQueryFilter* filter,
                                          ZrcBool* out_optimized) {
  if (out_optimized != nullptr) *out_optimized = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool optimized =
      corridor->impl->optimizePathTopology(query->impl, &dt_filter);
  if (out_optimized != nullptr) {
    *out_optimized = optimized ? ZRC_TRUE : ZRC_FALSE;
  }
  return ZRC_OK;
}

ZrcResult zrcPathCorridorMoveOverOffmeshConnection(
    ZrcPathCorridor* corridor, ZrcPolyRef offmesh_poly, ZrcNavMeshQuery* query,
    ZrcPolyRef* out_refs, float* out_start, float* out_end,
    ZrcBool* out_moved) {
  if (out_moved != nullptr) *out_moved = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (out_refs == nullptr || out_start == nullptr || out_end == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;

  // Untouched by upstream whenever the connection is not found
  // (DetourPathCorridor.cpp:397-401): seeded here so a "not moved" answer
  // still hands back zeroed output rather than whatever the caller's stack
  // held.
  out_refs[0] = 0;
  out_refs[1] = 0;
  out_start[0] = out_start[1] = out_start[2] = 0.f;
  out_end[0] = out_end[1] = out_end[2] = 0.f;

  const bool moved = corridor->impl->moveOverOffmeshConnection(
      offmesh_poly, out_refs, out_start, out_end, query->impl);
  if (out_moved != nullptr) *out_moved = moved ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorFixStart(ZrcPathCorridor* corridor,
                                  ZrcPolyRef safe_poly,
                                  const float* safe_position,
                                  ZrcBool* out_fixed) {
  if (out_fixed != nullptr) *out_fixed = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(safe_position)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool fixed = corridor->impl->fixPathStart(safe_poly, safe_position);
  if (out_fixed != nullptr) *out_fixed = fixed ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorTrimInvalid(ZrcPathCorridor* corridor,
                                     ZrcPolyRef safe_poly,
                                     const float* safe_position,
                                     ZrcNavMeshQuery* query,
                                     const ZrcQueryFilter* filter,
                                     ZrcBool* out_trimmed) {
  if (out_trimmed != nullptr) *out_trimmed = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(safe_position)) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool trimmed = corridor->impl->trimInvalidPath(
      safe_poly, safe_position, query->impl, &dt_filter);
  if (out_trimmed != nullptr) *out_trimmed = trimmed ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorIsValid(ZrcPathCorridor* corridor,
                                 int32_t max_lookahead, ZrcNavMeshQuery* query,
                                 const ZrcQueryFilter* filter,
                                 ZrcBool* out_valid) {
  if (out_valid == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_valid = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_lookahead < 0) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool valid =
      corridor->impl->isValid(max_lookahead, query->impl, &dt_filter);
  *out_valid = valid ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorMovePosition(ZrcPathCorridor* corridor,
                                      const float* position,
                                      ZrcNavMeshQuery* query,
                                      const ZrcQueryFilter* filter,
                                      ZrcBool* out_moved) {
  if (out_moved != nullptr) *out_moved = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool moved =
      corridor->impl->movePosition(position, query->impl, &dt_filter);
  if (out_moved != nullptr) *out_moved = moved ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorMoveTargetPosition(ZrcPathCorridor* corridor,
                                            const float* position,
                                            ZrcNavMeshQuery* query,
                                            const ZrcQueryFilter* filter,
                                            ZrcBool* out_moved) {
  if (out_moved != nullptr) *out_moved = ZRC_FALSE;
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // moveTargetPosition reads m_path[m_npath-1] with no guard
  // (DetourPathCorridor.cpp:487); an empty corridor makes that an
  // out-of-bounds read.
  if (corridor->impl->getPathCount() == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool moved =
      corridor->impl->moveTargetPosition(position, query->impl, &dt_filter);
  if (out_moved != nullptr) *out_moved = moved ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcPathCorridorInfo(const ZrcPathCorridor* corridor,
                              ZrcPathCorridorInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtPathCorridor& impl = *corridor->impl;
  memcpy(out->position, impl.getPos(), sizeof(out->position));
  memcpy(out->target, impl.getTarget(), sizeof(out->target));
  out->first_poly = static_cast<ZrcPolyRef>(impl.getFirstPoly());
  out->last_poly = static_cast<ZrcPolyRef>(impl.getLastPoly());
  out->path_count = impl.getPathCount();
  return ZRC_OK;
}

ZrcResult zrcPathCorridorPath(const ZrcPathCorridor* corridor, int32_t first,
                              int32_t count, ZrcPolyRef* out) {
  if (corridor == nullptr || corridor->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtPathCorridor& impl = *corridor->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, impl.getPathCount());
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const dtPolyRef* path = impl.getPath();
  for (int32_t i = 0; i < count; ++i) {
    out[i] = static_cast<ZrcPolyRef>(path[first + i]);
  }
  return ZRC_OK;
}

ZrcResult zrcMergeCorridorStartMoved(ZrcPolyRef* path, int32_t path_count,
                                     int32_t max_path,
                                     const ZrcPolyRef* visited,
                                     int32_t visited_count,
                                     int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  const ZrcResult args_ok =
      CheckMergeArgs(path, path_count, max_path, visited, visited_count);
  if (args_ok != ZRC_OK) return args_ok;

  *out_count = dtMergeCorridorStartMoved(path, path_count, max_path, visited,
                                         visited_count);
  return ZRC_OK;
}

ZrcResult zrcMergeCorridorEndMoved(ZrcPolyRef* path, int32_t path_count,
                                   int32_t max_path, const ZrcPolyRef* visited,
                                   int32_t visited_count, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  const ZrcResult args_ok =
      CheckMergeArgs(path, path_count, max_path, visited, visited_count);
  if (args_ok != ZRC_OK) return args_ok;

  *out_count = dtMergeCorridorEndMoved(path, path_count, max_path, visited,
                                       visited_count);
  return ZRC_OK;
}

ZrcResult zrcMergeCorridorStartShortcut(ZrcPolyRef* path, int32_t path_count,
                                        int32_t max_path,
                                        const ZrcPolyRef* visited,
                                        int32_t visited_count,
                                        int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  const ZrcResult args_ok =
      CheckMergeArgs(path, path_count, max_path, visited, visited_count);
  if (args_ok != ZRC_OK) return args_ok;

  *out_count = dtMergeCorridorStartShortcut(path, path_count, max_path,
                                            visited, visited_count);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The local boundary
//===----------------------------------------------------------------------===//

ZrcResult zrcLocalBoundaryCreate(ZrcLocalBoundary** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;

  ZrcLocalBoundary* handle = zrc::New<ZrcLocalBoundary>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  // dtLocalBoundary has no init(): its constructor already leaves it usable
  // (DetourLocalBoundary.cpp:27-32), so this is the only allocation step.
  handle->impl = zrc::New<dtLocalBoundary>();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  *out = handle;
  return ZRC_OK;
}

void zrcLocalBoundaryDestroy(ZrcLocalBoundary* boundary) {
  if (boundary == nullptr) return;
  zrc::Delete(boundary->impl);
  zrc::Delete(boundary);
}

ZrcResult zrcLocalBoundaryReset(ZrcLocalBoundary* boundary) {
  if (boundary == nullptr || boundary->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  boundary->impl->reset();
  return ZRC_OK;
}

ZrcResult zrcLocalBoundaryUpdate(ZrcLocalBoundary* boundary, ZrcPolyRef poly,
                                 const float* position, float range,
                                 ZrcNavMeshQuery* query,
                                 const ZrcQueryFilter* filter) {
  if (boundary == nullptr || boundary->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Validated before upstream is touched: update() copies pos into m_center
  // before checking anything, including when ref is later found invalid
  // (DetourLocalBoundary.cpp:97), so a non-finite position would otherwise be
  // stored permanently.
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPositiveFinite(range)) return ZRC_ERR_INVALID_ARGUMENT;
  // No CheckNotSlicing here, unlike every other entry point in this file that
  // takes a ZrcNavMeshQuery*: findLocalNeighbourhood uses the query's
  // separate 64-node tiny pool (DetourNavMeshQuery.cpp:197, 3117), not the
  // main pool a slice occupies, so this call cannot disturb one.
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  boundary->impl->update(poly, position, range, query->impl, &dt_filter);
  return ZRC_OK;
}

ZrcResult zrcLocalBoundaryIsValid(ZrcLocalBoundary* boundary,
                                  ZrcNavMeshQuery* query,
                                  const ZrcQueryFilter* filter,
                                  ZrcBool* out_valid) {
  if (out_valid == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_valid = ZRC_FALSE;
  if (boundary == nullptr || boundary->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const ZrcResult slicing_ok = CheckNotSlicing(query);
  if (slicing_ok != ZRC_OK) return slicing_ok;
  // isValid calls dtNavMeshQuery::isValidPolyRef, which dereferences the
  // filter with no null check of its own (DetourNavMeshQuery.cpp:3666).
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  dtQueryFilter dt_filter;
  if (!zrc::BuildFilter(*filter, &dt_filter)) return ZRC_ERR_INVALID_ARGUMENT;

  const bool valid = boundary->impl->isValid(query->impl, &dt_filter);
  *out_valid = valid ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

ZrcResult zrcLocalBoundaryCenter(const ZrcLocalBoundary* boundary,
                                 float* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out[0] = out[1] = out[2] = 0.f;
  if (boundary == nullptr || boundary->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Copied out verbatim, sentinel included: a boundary never updated reports
  // FLT_MAX on all three axes (DetourLocalBoundary.cpp:31), and the header
  // documents that rather than asking this file to normalise it away.
  memcpy(out, boundary->impl->getCenter(), sizeof(float) * 3);
  return ZRC_OK;
}

ZrcResult zrcLocalBoundarySegmentCount(const ZrcLocalBoundary* boundary,
                                       int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (boundary == nullptr || boundary->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_count = boundary->impl->getSegmentCount();
  return ZRC_OK;
}

ZrcResult zrcLocalBoundarySegments(const ZrcLocalBoundary* boundary,
                                   int32_t first, int32_t count, float* out) {
  if (boundary == nullptr || boundary->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const ZrcResult range_ok =
      zrc::CheckRange(first, count, boundary->impl->getSegmentCount());
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  for (int32_t i = 0; i < count; ++i) {
    const float* seg = boundary->impl->getSegment(first + i);
    memcpy(out + i * 6, seg, sizeof(float) * 6);
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The path queue
//===----------------------------------------------------------------------===//

ZrcResult zrcPathQueueCreate(const ZrcNavMesh* navmesh, int32_t max_path_size,
                             int32_t max_search_nodes, ZrcPathQueue** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_path_size < 1 || max_path_size > 65535) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_search_nodes < kMinSearchNodes ||
      max_search_nodes > kMaxSearchNodes) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  ZrcPathQueue* handle = zrc::New<ZrcPathQueue>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = zrc::New<dtPathQueue>();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  // dtPathQueue's constructor zeroes only each slot's path pointer
  // (DetourPathQueue.cpp:27-35); init() is what makes the rest of the object
  // readable, and is called exactly once per handle, here, with no re-init
  // entry point offered.
  //
  // navmesh->impl is already a plain dtNavMesh*: a pointer member's own
  // constness does not propagate to what it points at, so reading it through
  // `const ZrcNavMesh* navmesh` yields the same mutable pointer type
  // dtPathQueue::init declares — no cast needed, the same reasoning
  // zrecast_crowd.cpp's InitCrowdImpl already documents for dtCrowd::init.
  if (!handle->impl->init(max_path_size, max_search_nodes, navmesh->impl)) {
    zrc::Delete(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  handle->query = zrc::New<ZrcNavMeshQuery>();
  if (handle->query == nullptr) {
    zrc::Delete(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  // getNavQuery() returns const (DetourPathQueue.h:71); this wrapper is only
  // ever handed out as `const ZrcNavMeshQuery*`, so nothing reachable from
  // here gets a path to mutate the object through.
  handle->query->impl = const_cast<dtNavMeshQuery*>(handle->impl->getNavQuery());
  handle->query->navmesh = navmesh;
  handle->query->slicing = false;

  for (int32_t i = 0; i < zrc::kPathQueueSlots; ++i) {
    handle->filter_owner[i] = ZRC_PATH_REQUEST_NONE;
  }
  handle->navmesh = navmesh;
  handle->max_path_size = max_path_size;
  handle->owns = true;
  *out = handle;
  return ZRC_OK;
}

void zrcPathQueueDestroy(ZrcPathQueue* queue) {
  if (queue == nullptr) return;
  // A crowd hands its own queue out with owns == false; freeing it here would
  // free memory dtCrowd still uses until the crowd itself is destroyed or
  // re-initialised. Mirrors zrcProximityGridDestroy in
  // ffi/zrecast_steering.cpp.
  if (!queue->owns) return;
  zrc::Delete(queue->query);
  zrc::Delete(queue->impl);
  zrc::Delete(queue);
}

ZrcResult zrcPathQueueUpdate(ZrcPathQueue* queue, int32_t max_iters) {
  if (queue == nullptr || queue->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_iters < 0) return ZRC_ERR_INVALID_ARGUMENT;
  queue->impl->update(max_iters);
  return ZRC_OK;
}

// A crowd hands its path queue back as `const ZrcPathQueue*`
// (zrcCrowdPathQueue), and every entry point below that mutates a queue —
// this one included — takes a non-const one. C++ does not implicitly drop
// that const, so nothing reachable through the public API can submit to,
// update, or collect from a crowd's own queue: only a queue this file's own
// zrcPathQueueCreate returned can reach here, which is exactly what
// dtCrowd::getPathQueue's const return already prevents in C++.
ZrcResult zrcPathQueueRequest(ZrcPathQueue* queue, ZrcPolyRef start_poly,
                              ZrcPolyRef end_poly,
                              const float* start_position,
                              const float* end_position,
                              const ZrcQueryFilter* filter,
                              ZrcPathRequestRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = ZRC_PATH_REQUEST_NONE;
  if (queue == nullptr || queue->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (start_poly == 0 || end_poly == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(start_position) || !zrc::IsFiniteVec3(end_position)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  // Reclaim first: getRequestStatus answers DT_FAILURE for a reference no
  // slot holds, which is how a slot upstream has already recycled past its
  // keep-alive window is detected (DetourPathQueue.cpp:171-179, 99-105).
  for (int32_t i = 0; i < zrc::kPathQueueSlots; ++i) {
    if (queue->filter_owner[i] == ZRC_PATH_REQUEST_NONE) continue;
    const dtPathQueueRef held =
        static_cast<dtPathQueueRef>(queue->filter_owner[i]);
    if (queue->impl->getRequestStatus(held) == DT_FAILURE) {
      queue->filter_owner[i] = ZRC_PATH_REQUEST_NONE;
    }
  }

  int32_t slot = -1;
  for (int32_t i = 0; i < zrc::kPathQueueSlots; ++i) {
    if (queue->filter_owner[i] == ZRC_PATH_REQUEST_NONE) {
      slot = i;
      break;
    }
  }
  if (slot == -1) {
    // Every slot outstanding: upstream would refuse too
    // (DetourPathQueue.cpp:150-151). Not an error.
    return ZRC_OK;
  }

  if (!zrc::BuildFilter(*filter, &queue->filters[slot])) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const dtPathQueueRef ref =
      queue->impl->request(start_poly, end_poly, start_position,
                           end_position, &queue->filters[slot]);
  if (ref == DT_PATHQ_INVALID) return ZRC_OK;
  queue->filter_owner[slot] = static_cast<ZrcPathRequestRef>(ref);
  *out_ref = static_cast<ZrcPathRequestRef>(ref);
  return ZRC_OK;
}

ZrcResult zrcPathQueueRequestStatus(const ZrcPathQueue* queue,
                                    ZrcPathRequestRef ref,
                                    ZrcPathRequestState* out_state) {
  if (out_state == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_state = ZRC_PATH_REQUEST_UNKNOWN;
  if (queue == nullptr || queue->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (ref == ZRC_PATH_REQUEST_NONE) return ZRC_OK;

  const dtStatus status =
      queue->impl->getRequestStatus(static_cast<dtPathQueueRef>(ref));
  // Checked ahead of dtStatusFailed: DT_FAILURE alone is upstream's "no such
  // reference" answer (DetourPathQueue.cpp:178), and dtStatusFailed would
  // also match it, misreporting an unknown reference as a failed search.
  if (status == DT_FAILURE) {
    *out_state = ZRC_PATH_REQUEST_UNKNOWN;
  } else if (dtStatusSucceed(status)) {
    *out_state = ZRC_PATH_REQUEST_READY;
  } else if (dtStatusFailed(status)) {
    *out_state = ZRC_PATH_REQUEST_FAILED;
  } else {
    *out_state = ZRC_PATH_REQUEST_RUNNING;
  }
  return ZRC_OK;
}

ZrcResult zrcPathQueueResult(ZrcPathQueue* queue, ZrcPathRequestRef ref,
                             ZrcPolyRef* out, int32_t max_path,
                             int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (queue == nullptr || queue->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (out == nullptr || max_path < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (ref == ZRC_PATH_REQUEST_NONE) return ZRC_ERR_NOT_FOUND;

  // Status checked before getPathResult is called at all: getPathResult frees
  // the slot whether or not the search finished, reporting success with
  // whatever length happened to be there (DetourPathQueue.cpp:181-200). The
  // request survives both refusals below and can be collected once ready.
  const dtStatus status =
      queue->impl->getRequestStatus(static_cast<dtPathQueueRef>(ref));
  if (status == DT_FAILURE) return ZRC_ERR_NOT_FOUND;
  if (!dtStatusSucceed(status) && !dtStatusFailed(status)) {
    return ZRC_ERR_SEARCH_IN_PROGRESS;
  }

  int count = 0;
  queue->impl->getPathResult(static_cast<dtPathQueueRef>(ref), out, &count,
                             max_path);
  for (int32_t i = 0; i < zrc::kPathQueueSlots; ++i) {
    if (queue->filter_owner[i] == ref) {
      queue->filter_owner[i] = ZRC_PATH_REQUEST_NONE;
    }
  }

  if (count < 0 || count > max_path) return ZRC_ERR_QUERY_FAILED;
  *out_count = count;
  // getPathResult clamps to maxPath with no truncation signal of its own
  // (DetourPathQueue.cpp:193-195): filling the buffer exactly is reported as
  // possibly clipped, since a path exactly max_path long is indistinguishable
  // from a longer one cut short.
  if (count == max_path) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

ZrcResult zrcPathQueueNavMeshQuery(const ZrcPathQueue* queue,
                                   const ZrcNavMeshQuery** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (queue == nullptr || queue->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  if (queue->query == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = queue->query;
  return ZRC_OK;
}

}  // extern "C"
