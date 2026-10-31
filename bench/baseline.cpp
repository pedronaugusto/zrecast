// The work of each bench row, written in C++ against Detour directly: the
// number the binding's own row is read against. Each loops `times` times
// inside C++, so no call crosses from Zig per unit, and each sets Detour up
// exactly as the binding's boundary does (ffi/zrecast_navmesh.cpp,
// ffi/zrecast_query.cpp, ffi/zrecast_crowd.cpp).
#include <stdint.h>
#include <string.h>

#include "DetourAlloc.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "zrecast.h"

namespace {

/// A navmesh loaded from a single-tile image, as InitSingleTile loads one.
dtNavMesh* Load(const unsigned char* image, size_t size) {
  unsigned char* copy = static_cast<unsigned char*>(dtAlloc(size, DT_ALLOC_PERM));
  if (copy == nullptr) return nullptr;
  memcpy(copy, image, size);
  dtMeshHeader header;
  memcpy(&header, copy, sizeof(header));
  dtNavMeshParams params;
  memset(&params, 0, sizeof(params));
  params.orig[0] = header.bmin[0];
  params.orig[1] = header.bmin[1];
  params.orig[2] = header.bmin[2];
  params.tileWidth = header.bmax[0] - header.bmin[0];
  params.tileHeight = header.bmax[2] - header.bmin[2];
  params.maxTiles = 1;
  params.maxPolys = header.polyCount;
  dtNavMesh* mesh = dtAllocNavMesh();
  if (mesh == nullptr || dtStatusFailed(mesh->init(&params)) ||
      dtStatusFailed(mesh->addTile(copy, static_cast<int>(size), DT_TILE_FREE_DATA, 0, nullptr))) {
    dtFreeNavMesh(mesh);
    return nullptr;
  }
  return mesh;
}

/// zrcQueryFilterDefault's filter: every area at cost 1, every flag admitted.
dtQueryFilter DefaultFilter() {
  dtQueryFilter filter;
  for (int i = 0; i < DT_MAX_AREAS; ++i) filter.setAreaCost(i, 1.0f);
  filter.setIncludeFlags(0xffff);
  filter.setExcludeFlags(0);
  return filter;
}

struct World {
  dtNavMesh* mesh;
  dtNavMeshQuery* query;
};

}  // namespace

extern "C" {

void* zrecast_baseline_open(const unsigned char* image, size_t size, int32_t max_nodes) {
  World* world = static_cast<World*>(dtAlloc(sizeof(World), DT_ALLOC_PERM));
  if (world == nullptr) return nullptr;
  world->mesh = Load(image, size);
  world->query = dtAllocNavMeshQuery();
  if (world->mesh == nullptr || world->query == nullptr ||
      dtStatusFailed(world->query->init(world->mesh, max_nodes))) {
    dtFreeNavMeshQuery(world->query);
    dtFreeNavMesh(world->mesh);
    dtFree(world);
    return nullptr;
  }
  return world;
}

void zrecast_baseline_close(void* opened) {
  World* world = static_cast<World*>(opened);
  dtFreeNavMeshQuery(world->query);
  dtFreeNavMesh(world->mesh);
  dtFree(world);
}

uint64_t zrecast_baseline_load(const unsigned char* image, size_t size, uint64_t times) {
  uint64_t tiles = 0;
  for (uint64_t i = 0; i < times; ++i) {
    dtNavMesh* mesh = Load(image, size);
    if (mesh == nullptr) return UINT64_MAX;
    tiles += static_cast<uint64_t>(mesh->getMaxTiles());
    dtFreeNavMesh(mesh);
  }
  return tiles;
}

/// The same load after the check the binding makes first: the whole image
/// validated (zrcNavMeshValidate) before Detour reads any of it. The binding's
/// load is read against this row; zrecast_baseline_load is Detour trusting
/// its bytes.
uint64_t zrecast_baseline_load_validated(const unsigned char* image, size_t size, uint64_t times) {
  uint64_t tiles = 0;
  for (uint64_t i = 0; i < times; ++i) {
    if (zrcNavMeshValidate(image, size) != ZRC_OK) return UINT64_MAX;
    dtNavMesh* mesh = Load(image, size);
    if (mesh == nullptr) return UINT64_MAX;
    tiles += static_cast<uint64_t>(mesh->getMaxTiles());
    dtFreeNavMesh(mesh);
  }
  return tiles;
}

uint64_t zrecast_baseline_nearest(void* opened, const float* points, size_t count,
                                  const float* extents, uint64_t times) {
  const World* world = static_cast<const World*>(opened);
  const dtQueryFilter filter = DefaultFilter();
  uint64_t found = 0;
  for (uint64_t i = 0; i < times; ++i) {
    const float* point = points + (i % count) * 3;
    dtPolyRef ref = 0;
    float nearest[3];
    if (dtStatusFailed(world->query->findNearestPoly(point, extents, &filter, &ref, nearest))) return UINT64_MAX;
    found += ref != 0;
  }
  return found;
}

uint64_t zrecast_baseline_route(void* opened, const uint32_t* refs, const float* points,
                                const uint32_t* pairs, size_t count, uint64_t times) {
  const World* world = static_cast<const World*>(opened);
  const dtQueryFilter filter = DefaultFilter();
  dtPolyRef corridor[256];
  float corners[64 * 3];
  uint64_t walked = 0;
  for (uint64_t i = 0; i < times; ++i) {
    const uint32_t from = pairs[(i % count) * 2];
    const uint32_t to = pairs[(i % count) * 2 + 1];
    int length = 0;
    if (dtStatusFailed(world->query->findPath(refs[from], refs[to], points + from * 3, points + to * 3,
                                              &filter, corridor, &length, 256))) {
      return UINT64_MAX;
    }
    int corner_count = 0;
    if (dtStatusFailed(world->query->findStraightPath(points + from * 3, points + to * 3, corridor, length,
                                                      corners, nullptr, nullptr, &corner_count, 64, 0))) {
      return UINT64_MAX;
    }
    walked += static_cast<uint64_t>(corner_count);
  }
  return walked;
}

float zrecast_baseline_raycast(void* opened, const uint32_t* refs, const float* points,
                               const uint32_t* pairs, size_t count, uint64_t times) {
  const World* world = static_cast<const World*>(opened);
  const dtQueryFilter filter = DefaultFilter();
  float sum = 0;
  for (uint64_t i = 0; i < times; ++i) {
    const uint32_t from = pairs[(i % count) * 2];
    const uint32_t to = pairs[(i % count) * 2 + 1];
    dtRaycastHit hit;
    memset(&hit, 0, sizeof(hit));
    if (dtStatusFailed(world->query->raycast(refs[from], points + from * 3, points + to * 3, &filter, 0, &hit, 0))) {
      return -1;
    }
    sum += hit.t;
  }
  return sum;
}

/// The binding's nearest row with Zig taken out: the same zrcFindNearestPoly
/// the wrapper calls, on the binding's own query and filter, from a C++ loop.
/// Against zrecast_baseline_nearest it times the boundary's checks; against
/// the Zig row, the wrapper and the crossing.
uint64_t zrecast_baseline_nearest_boundary(const ZrcNavMeshQuery* query, const ZrcQueryFilter* filter,
                                           const float* points, size_t count, const float* extents,
                                           uint64_t times) {
  uint64_t found = 0;
  for (uint64_t i = 0; i < times; ++i) {
    ZrcPolyRef ref = 0;
    float nearest[3];
    ZrcBool over = 0;
    if (zrcFindNearestPoly(query, points + (i % count) * 3, extents, filter, &ref, nearest, &over) != ZRC_OK) {
      return UINT64_MAX;
    }
    found += ref != 0;
  }
  return found;
}

/// The raycast row the same way: zrcRaycast from a C++ loop.
float zrecast_baseline_raycast_boundary(const ZrcNavMeshQuery* query, const ZrcQueryFilter* filter,
                                        const uint32_t* refs, const float* points, const uint32_t* pairs,
                                        size_t count, uint64_t times) {
  float sum = 0;
  for (uint64_t i = 0; i < times; ++i) {
    const uint32_t from = pairs[(i % count) * 2];
    const uint32_t to = pairs[(i % count) * 2 + 1];
    ZrcRaycastHit hit;
    if (zrcRaycast(query, refs[from], points + from * 3, points + to * 3, filter, 0, 0, &hit, nullptr, 0, nullptr,
                   nullptr) != ZRC_OK) {
      return -1;
    }
    sum += hit.t;
  }
  return sum;
}

/// A crowd of `count` agents at `points`, each sent to its pair's other end.
/// The agent parameters are the binding's row's, field for field
/// (ToDtAgentParams).
void* zrecast_baseline_crowd_open(void* opened, const uint32_t* refs, const float* points,
                                  const uint32_t* pairs, size_t count, float radius) {
  World* world = static_cast<World*>(opened);
  dtCrowd* crowd = dtAllocCrowd();
  if (crowd == nullptr || !crowd->init(static_cast<int>(count), radius, world->mesh)) {
    dtFreeCrowd(crowd);
    return nullptr;
  }
  dtCrowdAgentParams params;
  memset(&params, 0, sizeof(params));
  params.radius = radius;
  params.height = 2.0f;
  params.maxAcceleration = 8.0f;
  params.maxSpeed = 3.5f;
  params.collisionQueryRange = radius * 12.0f;
  params.pathOptimizationRange = radius * 30.0f;
  params.separationWeight = 2.0f;
  params.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OBSTACLE_AVOIDANCE | DT_CROWD_SEPARATION |
                       DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO;
  for (size_t i = 0; i < count; ++i) {
    const int agent = crowd->addAgent(points + pairs[i * 2] * 3, &params);
    const uint32_t to = pairs[i * 2 + 1];
    if (agent < 0 || !crowd->requestMoveTarget(agent, refs[to], points + to * 3)) {
      dtFreeCrowd(crowd);
      return nullptr;
    }
  }
  return crowd;
}

/// `times` frames of `dt`; the agents still in the crowd.
uint64_t zrecast_baseline_crowd_update(void* opened, float dt, uint64_t times) {
  dtCrowd* crowd = static_cast<dtCrowd*>(opened);
  for (uint64_t i = 0; i < times; ++i) crowd->update(dt, nullptr);
  return static_cast<uint64_t>(crowd->getAgentCount());
}

void zrecast_baseline_crowd_close(void* opened) { dtFreeCrowd(static_cast<dtCrowd*>(opened)); }

}  // extern "C"
