//===----------------------------------------------------------------------===//
// zrecast — crowds: many agents steering around each other and the world,
// and the proximity grid, path queue and query object a crowd borrows out
// while it does it.
//
// Upstream identifies an agent by a pool slot it silently reuses: removeAgent
// only clears a flag, addAgent hands the lowest free slot to whoever asks
// next, and every setter bounds-checks the index without ever asking whether
// the slot is live. Every entry point below resolves a ZrcAgentRef through
// the slot/serial scheme zrecast_internal.h defines instead, so a stale
// reference is ZRC_ERR_NOT_FOUND rather than someone else's agent.
// zrcCrowdAddAgent and zrcCrowdSetAgentParams are the only paths a
// ZrcCrowdAgentParams reaches dtCrowd through, and both refuse the two
// unbounded index fields upstream never checks (DetourCrowd.cpp:534 and
// 1293, among fourteen call sites) before it does.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Argument validation
//===----------------------------------------------------------------------===//

ZrcResult ValidateCrowdCreateArgs(const ZrcNavMesh* navmesh, int32_t max_agents,
                                  float max_agent_radius) {
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_agents < 1 || max_agents > ZRC_CROWD_MAX_AGENTS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Bounded above as well as below: dtCrowd::init multiplies this by 3 for the
  // proximity grid's cell size and by 2 for the placement box
  // (DetourCrowd.cpp:389, 395), and a radius near the top of float range
  // overflows both to infinity. The same ceiling every world coordinate in
  // this package carries.
  if (!zrc::IsPositiveFinite(max_agent_radius) ||
      max_agent_radius > zrc::kMaxCoordinate) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Agent references
//===----------------------------------------------------------------------===//

/// The pool slot a reference names, live only when its serial still matches
/// the one addAgent stamped there. A malformed ref — out of range, or naming
/// a free or reused slot — is ZRC_ERR_NOT_FOUND: the reference is well
/// formed, but nothing it names is here any more.
ZrcResult ResolveAgent(const ZrcCrowd* crowd, ZrcAgentRef ref, int32_t* out_slot) {
  const int32_t slot = zrc::AgentRefSlot(ref);
  if (slot < 0 || slot >= crowd->capacity) return ZRC_ERR_NOT_FOUND;
  if (crowd->serial[slot] == 0) return ZRC_ERR_NOT_FOUND;
  if (crowd->serial[slot] != zrc::AgentRefSerial(ref)) return ZRC_ERR_NOT_FOUND;
  *out_slot = slot;
  return ZRC_OK;
}

/// dtCrowd keeps maxAgentRadius private with no accessor (DetourCrowd.h:224),
/// so it is recovered from the placement box it derives from it:
/// getQueryHalfExtents()[0] is maxAgentRadius*2.0f exactly
/// (DetourCrowd.cpp:389), and halving a finite float by a power of two
/// introduces no rounding.
float MaxAgentRadius(const ZrcCrowd& crowd) {
  return crowd.impl->getQueryHalfExtents()[0] * 0.5f;
}

//===----------------------------------------------------------------------===//
// dtCrowdAgentParams <-> ZrcCrowdAgentParams
//
// Field by field rather than a memcpy, matching zrc::ToDtAvoidanceParams: the
// two layouts agree today, and only a function that names every field would
// notice if that changed.
//===----------------------------------------------------------------------===//

void ToDtAgentParams(const ZrcCrowdAgentParams& in, dtCrowdAgentParams* out) {
  out->radius = in.radius;
  out->height = in.height;
  out->maxAcceleration = in.max_acceleration;
  out->maxSpeed = in.max_speed;
  out->collisionQueryRange = in.collision_query_range;
  out->pathOptimizationRange = in.path_optimization_range;
  out->separationWeight = in.separation_weight;
  out->updateFlags = in.update_flags;
  out->obstacleAvoidanceType = in.obstacle_avoidance_type;
  out->queryFilterType = in.query_filter_type;
  out->userData = in.user_data;
}

void FromDtAgentParams(const dtCrowdAgentParams& in, ZrcCrowdAgentParams* out) {
  out->radius = in.radius;
  out->height = in.height;
  out->max_acceleration = in.maxAcceleration;
  out->max_speed = in.maxSpeed;
  out->collision_query_range = in.collisionQueryRange;
  out->path_optimization_range = in.pathOptimizationRange;
  out->separation_weight = in.separationWeight;
  out->update_flags = in.updateFlags;
  out->obstacle_avoidance_type = in.obstacleAvoidanceType;
  out->query_filter_type = in.queryFilterType;
  out->user_data = in.userData;
}

//===----------------------------------------------------------------------===//
// Lifecycle
//===----------------------------------------------------------------------===//

/// Tears down the three borrowed wrappers a crowd hands out, but never the
/// objects they point at: those belong to dtCrowd and die when it purges or
/// is freed. Safe to call on a crowd whose wrappers were never built.
void DestroyCrowdSubHandles(ZrcCrowd* crowd) {
  zrc::Delete(crowd->grid);
  crowd->grid = nullptr;
  if (crowd->path_queue != nullptr) {
    // The queue is borrowed, so zrcPathQueueDestroy declines to free it and
    // its own query wrapper with it. This is where that wrapper is released.
    zrc::Delete(crowd->path_queue->query);
    crowd->path_queue->query = nullptr;
  }
  zrc::Delete(crowd->path_queue);
  crowd->path_queue = nullptr;
  zrc::Delete(crowd->query);
  crowd->query = nullptr;
}

/// Builds the three borrowed wrappers fresh from what dtCrowd::init just
/// allocated. Self-contained: a failure partway through frees whatever this
/// call itself allocated before returning, so a caller need not know how far
/// it got.
ZrcResult BuildCrowdSubHandles(ZrcCrowd* crowd) {
  crowd->grid = zrc::New<ZrcProximityGrid>();
  if (crowd->grid == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  // getGrid/getPathQueue/getNavMeshQuery all return a const pointer; each
  // const_cast below only satisfies the wrapper field's declared type. The
  // grid and queue are handed out through zrcCrowdGrid/zrcCrowdPathQueue with
  // `owns` false, and the query only as `const ZrcNavMeshQuery*`
  // (zrcCrowdNavMeshQuery), so nothing here gets a path to mutate through it.
  crowd->grid->impl = const_cast<dtProximityGrid*>(crowd->impl->getGrid());
  crowd->grid->owns = false;

  crowd->path_queue = zrc::New<ZrcPathQueue>();
  if (crowd->path_queue == nullptr) {
    zrc::Delete(crowd->grid);
    crowd->grid = nullptr;
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  crowd->path_queue->impl = const_cast<dtPathQueue*>(crowd->impl->getPathQueue());
  crowd->path_queue->navmesh = crowd->navmesh;
  // dtCrowd::init always sizes its queue for 256 path slots
  // (DetourCrowd.cpp:421, m_maxPathResult).
  crowd->path_queue->max_path_size = 256;
  crowd->path_queue->owns = false;
  // A borrowed queue still answers zrcPathQueueNavMeshQuery, which a C++ host
  // reaches through dtPathQueue::getNavQuery. That is a different object from
  // the crowd's own query below: dtCrowd gives the path queue a 4096-node pool
  // of its own and keeps 512 for its per-frame work (DetourCrowd.cpp:420-426,
  // 455-459).
  crowd->path_queue->query = zrc::New<ZrcNavMeshQuery>();
  if (crowd->path_queue->query == nullptr) {
    zrc::Delete(crowd->path_queue);
    crowd->path_queue = nullptr;
    zrc::Delete(crowd->grid);
    crowd->grid = nullptr;
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  crowd->path_queue->query->impl =
      const_cast<dtNavMeshQuery*>(crowd->path_queue->impl->getNavQuery());
  crowd->path_queue->query->navmesh = crowd->navmesh;
  crowd->path_queue->query->slicing = false;

  crowd->query = zrc::New<ZrcNavMeshQuery>();
  if (crowd->query == nullptr) {
    zrc::Delete(crowd->path_queue->query);
    zrc::Delete(crowd->path_queue);
    crowd->path_queue = nullptr;
    zrc::Delete(crowd->grid);
    crowd->grid = nullptr;
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  crowd->query->impl = const_cast<dtNavMeshQuery*>(crowd->impl->getNavMeshQuery());
  crowd->query->navmesh = crowd->navmesh;
  crowd->query->slicing = false;

  return ZRC_OK;
}

/// Runs dtCrowd::init and then the bookkeeping this package layers on top of
/// it: the per-slot serial array and the three borrowed wrappers. Shared by
/// zrcCrowdCreate and zrcCrowdInit, which differ only in whether a fresh
/// ZrcCrowd needs allocating first.
///
/// A failure at any step after upstream's own init succeeds leaves `crowd`
/// with zero capacity and no wrappers rather than a half-built one: the next
/// call to touch the crowd sees a consistently empty handle, not a handle
/// whose serial array or wrappers might or might not exist.
ZrcResult InitCrowdImpl(ZrcCrowd* crowd, const ZrcNavMesh* navmesh,
                        int32_t max_agents, float max_agent_radius) {
  // dtCrowd::init takes nav as dtNavMesh* despite never mutating it: it only
  // forwards the pointer to dtPathQueue::init and dtNavMeshQuery::init
  // (DetourCrowd.cpp:426, 458), and the second of those declares it const
  // (DetourNavMeshQuery.h:175) — the first just forwards it again, to its own
  // internal dtNavMeshQuery::init (DetourPathQueue.cpp:60), still unmutated.
  // navmesh->impl already carries that mutable pointer type through a const
  // ZrcNavMesh*, since a pointer field's own constness never propagates to
  // what it points at, so no cast is needed to hand it over.
  if (!crowd->impl->init(max_agents, max_agent_radius, navmesh->impl)) {
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  crowd->navmesh = navmesh;

  const size_t serial_bytes = sizeof(uint64_t) * static_cast<size_t>(max_agents);
  uint64_t* serial = static_cast<uint64_t*>(zrc::Alloc(serial_bytes, DT_ALLOC_PERM));
  if (serial == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  memset(serial, 0, serial_bytes);
  crowd->serial = serial;
  crowd->capacity = max_agents;

  const ZrcResult sub_ok = BuildCrowdSubHandles(crowd);
  if (sub_ok != ZRC_OK) {
    zrc::Free(crowd->serial);
    crowd->serial = nullptr;
    crowd->capacity = 0;
    return sub_ok;
  }
  return ZRC_OK;
}

}  // namespace

extern "C" {

void zrcAvoidanceParamsDefault(ZrcAvoidanceParams* out) {
  if (out == nullptr) return;
  out->vel_bias = 0.4f;
  out->weight_desired_vel = 2.0f;
  out->weight_current_vel = 0.75f;
  out->weight_side = 0.75f;
  out->weight_toi = 2.5f;
  out->horiz_time = 2.5f;
  out->grid_size = 33;
  out->adaptive_divs = 7;
  out->adaptive_rings = 2;
  out->adaptive_depth = 5;
}

//===----------------------------------------------------------------------===//
// The crowd
//===----------------------------------------------------------------------===//

ZrcResult zrcCrowdCreate(const ZrcNavMesh* navmesh, int32_t max_agents,
                         float max_agent_radius, ZrcCrowd** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult args_ok =
      ValidateCrowdCreateArgs(navmesh, max_agents, max_agent_radius);
  if (args_ok != ZRC_OK) return args_ok;

  ZrcCrowd* handle = zrc::New<ZrcCrowd>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = dtAllocCrowd();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  // The one point at which this counter starts: never reset again, including
  // by zrcCrowdInit, so a reference minted against any past state of this
  // handle never resolves again once it is gone.
  handle->next_serial = 1;

  const ZrcResult init_ok =
      InitCrowdImpl(handle, navmesh, max_agents, max_agent_radius);
  if (init_ok != ZRC_OK) {
    dtFreeCrowd(handle->impl);
    zrc::Delete(handle);
    return init_ok;
  }

  *out = handle;
  return ZRC_OK;
}

ZrcResult zrcCrowdInit(ZrcCrowd* crowd, const ZrcNavMesh* navmesh,
                       int32_t max_agents, float max_agent_radius) {
  if (crowd == nullptr || crowd->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult args_ok =
      ValidateCrowdCreateArgs(navmesh, max_agents, max_agent_radius);
  if (args_ok != ZRC_OK) return args_ok;

  // dtCrowd::init purges unconditionally before it allocates anything new
  // (DetourCrowd.cpp:383), so every wrapper this handle has ever handed out
  // dangles by the time init returns, success or not. Torn down here rather
  // than left for a caller to dereference through a stale pointer.
  zrc::Free(crowd->serial);
  crowd->serial = nullptr;
  crowd->capacity = 0;
  DestroyCrowdSubHandles(crowd);

  return InitCrowdImpl(crowd, navmesh, max_agents, max_agent_radius);
}

void zrcCrowdDestroy(ZrcCrowd* crowd) {
  if (crowd == nullptr) return;
  DestroyCrowdSubHandles(crowd);
  zrc::Free(crowd->serial);
  dtFreeCrowd(crowd->impl);
  zrc::Delete(crowd);
}

//===----------------------------------------------------------------------===//
// Agents
//===----------------------------------------------------------------------===//

ZrcResult zrcCrowdAddAgent(ZrcCrowd* crowd, const float* position,
                           const ZrcCrowdAgentParams* params,
                           ZrcAgentRef* out_ref) {
  if (out_ref != nullptr) *out_ref = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult params_ok =
      zrc::ValidateCrowdAgentParams(*params, MaxAgentRadius(*crowd));
  if (params_ok != ZRC_OK) return params_ok;

  dtCrowdAgentParams native;
  memset(&native, 0, sizeof(native));
  ToDtAgentParams(*params, &native);

  const int slot = crowd->impl->addAgent(position, &native);
  if (slot == -1) return ZRC_ERR_CROWD_FULL;

  const uint64_t serial = crowd->next_serial++;
  crowd->serial[slot] = serial;
  if (out_ref != nullptr) *out_ref = zrc::MakeAgentRef(slot, serial);
  return ZRC_OK;
}

ZrcResult zrcCrowdRemoveAgent(ZrcCrowd* crowd, ZrcAgentRef ref) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  // Retired here whatever happens: the host's reference stops resolving now.
  crowd->serial[slot] = 0;

  // An agent part way across an off-mesh connection is not handed back to
  // upstream yet. dtCrowd::removeAgent clears the agent's active flag and
  // touches nothing else, while the traversal lives in a parallel array
  // dtCrowd::update walks by raw pool slot, consulting only its own active
  // flag (DetourCrowd.cpp:572-578, 1411-1447). addAgent then hands the lowest
  // free slot to the next caller, who resumes the dead agent's traversal: its
  // position overwritten by a lerp between coordinates it has never been near.
  //
  // Leaving the slot occupied until the traversal finishes closes that without
  // reaching into dtCrowd. The agent stops seeking, upstream finishes carrying
  // it across, and zrcCrowdUpdate hands the slot back below. The cost is a
  // slot that stays busy for the rest of the crossing, which zrcCrowdAddAgent
  // documents.
  const dtCrowdAgent* agent = crowd->impl->getAgent(slot);
  if (agent != nullptr && agent->active &&
      agent->state == DT_CROWDAGENT_STATE_OFFMESH) {
    crowd->impl->resetMoveTarget(slot);
    return ZRC_OK;
  }

  crowd->impl->removeAgent(slot);
  return ZRC_OK;
}

namespace {

/// Hands back the slots zrcCrowdRemoveAgent held open until their off-mesh
/// traversal finished. A slot is draining when this package has retired its
/// reference but upstream still has the agent active.
void ReleaseDrainedSlots(ZrcCrowd* crowd) {
  for (int32_t i = 0; i < crowd->capacity; ++i) {
    if (crowd->serial[i] != 0) continue;
    const dtCrowdAgent* agent = crowd->impl->getAgent(i);
    if (agent == nullptr || !agent->active) continue;
    if (agent->state == DT_CROWDAGENT_STATE_OFFMESH) continue;
    crowd->impl->removeAgent(i);
  }
}

}  // namespace

ZrcResult zrcCrowdSetAgentParams(ZrcCrowd* crowd, ZrcAgentRef ref,
                                 const ZrcCrowdAgentParams* params) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult params_ok =
      zrc::ValidateCrowdAgentParams(*params, MaxAgentRadius(*crowd));
  if (params_ok != ZRC_OK) return params_ok;
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  dtCrowdAgentParams native;
  memset(&native, 0, sizeof(native));
  ToDtAgentParams(*params, &native);
  crowd->impl->updateAgentParameters(slot, &native);
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentInfo(const ZrcCrowd* crowd, ZrcAgentRef ref,
                            ZrcCrowdAgent* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  // getAgent bounds-checks and returns null only for an index outside
  // [0, getAgentCount()); ResolveAgent has already placed slot inside
  // [0, crowd->capacity), which is that same range.
  const dtCrowdAgent* ag = crowd->impl->getAgent(slot);
  out->state = static_cast<ZrcCrowdAgentState>(ag->state);
  out->target_state = static_cast<ZrcCrowdTargetState>(ag->targetState);
  out->partial = ag->partial ? ZRC_TRUE : ZRC_FALSE;
  memcpy(out->position, ag->npos, sizeof(out->position));
  memcpy(out->velocity, ag->vel, sizeof(out->velocity));
  memcpy(out->desired_velocity, ag->dvel, sizeof(out->desired_velocity));
  memcpy(out->avoided_velocity, ag->nvel, sizeof(out->avoided_velocity));
  memcpy(out->displacement, ag->disp, sizeof(out->displacement));
  out->desired_speed = ag->desiredSpeed;
  out->target_ref = static_cast<ZrcPolyRef>(ag->targetRef);
  memcpy(out->target_position, ag->targetPos, sizeof(out->target_position));
  out->target_replan = ag->targetReplan ? ZRC_TRUE : ZRC_FALSE;
  out->target_replan_time = ag->targetReplanTime;
  out->topology_opt_time = ag->topologyOptTime;
  out->corner_count = ag->ncorners;
  out->neighbour_count = ag->nneis;
  FromDtAgentParams(ag->params, &out->params);
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentCapacity(const ZrcCrowd* crowd, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_count = crowd->capacity;
  return ZRC_OK;
}

ZrcResult zrcCrowdActiveAgentCount(const ZrcCrowd* crowd, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t n = 0;
  for (int32_t i = 0; i < crowd->capacity; ++i) {
    if (crowd->serial[i] != 0) ++n;
  }
  *out_count = n;
  return ZRC_OK;
}

ZrcResult zrcCrowdActiveAgents(const ZrcCrowd* crowd, ZrcAgentRef* out,
                               int32_t max_agents, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_agents < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_agents > 0 && out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  // Walking crowd->serial directly is the same answer getActiveAgents would
  // give (DetourCrowd.cpp:683-690 packs in the same slot order this loop
  // visits) at the cost of an array of dtCrowdAgent* this package has no
  // other use for.
  int32_t total = 0;
  int32_t written = 0;
  for (int32_t i = 0; i < crowd->capacity; ++i) {
    if (crowd->serial[i] == 0) continue;
    if (written < max_agents) {
      out[written] = zrc::MakeAgentRef(i, crowd->serial[i]);
      ++written;
    }
    ++total;
  }
  *out_count = total;
  if (total > max_agents) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentRefAt(const ZrcCrowd* crowd, int32_t index,
                             ZrcAgentRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index < 0 || index >= crowd->capacity) return ZRC_ERR_INVALID_ARGUMENT;
  if (crowd->serial[index] == 0) return ZRC_OK;  // a free slot, not an error
  *out_ref = zrc::MakeAgentRef(index, crowd->serial[index]);
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentCorners(const ZrcCrowd* crowd, ZrcAgentRef ref,
                               int32_t first, int32_t count,
                               ZrcCrowdCorner* out) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  const dtCrowdAgent* ag = crowd->impl->getAgent(slot);
  const ZrcResult range_ok = zrc::CheckRange(first, count, ag->ncorners);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  for (int32_t i = 0; i < count; ++i) {
    const int32_t src = first + i;
    memcpy(out[i].position, &ag->cornerVerts[src * 3], sizeof(out[i].position));
    out[i].flags = ag->cornerFlags[src];
    out[i].poly = static_cast<ZrcPolyRef>(ag->cornerPolys[src]);
  }
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentNeighbours(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                  int32_t first, int32_t count,
                                  ZrcCrowdNeighbour* out) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  const dtCrowdAgent* ag = crowd->impl->getAgent(slot);
  const ZrcResult range_ok = zrc::CheckRange(first, count, ag->nneis);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  for (int32_t i = 0; i < count; ++i) {
    const dtCrowdNeighbour& nei = ag->neis[first + i];
    // idx is a pool index by the time update() hands it back, remapped from
    // a position in the active-agent snapshot at DetourCrowd.cpp:1110-1111;
    // always a currently active slot, since a crowd fills neis only from
    // agents its own update just confirmed active.
    out[i].agent = zrc::MakeAgentRef(nei.idx, crowd->serial[nei.idx]);
    // dist is squared despite its name and doc comment: getNeighbours passes
    // dtVlenSqr(diff) straight through addNeighbour into it
    // (DetourCrowd.cpp:211, 182) and never takes the root. Rooted here so
    // ZrcCrowdNeighbour::distance holds what it says — world units.
    out[i].distance = sqrtf(nei.dist);
  }
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentCorridorInfo(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                    ZrcPathCorridorInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  const dtPathCorridor& corridor = crowd->impl->getAgent(slot)->corridor;
  memcpy(out->position, corridor.getPos(), sizeof(out->position));
  memcpy(out->target, corridor.getTarget(), sizeof(out->target));
  out->first_poly = static_cast<ZrcPolyRef>(corridor.getFirstPoly());
  out->last_poly = static_cast<ZrcPolyRef>(corridor.getLastPoly());
  out->path_count = corridor.getPathCount();
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentCorridorPath(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                    int32_t first, int32_t count,
                                    ZrcPolyRef* out) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  const dtPathCorridor& corridor = crowd->impl->getAgent(slot)->corridor;
  const ZrcResult range_ok = zrc::CheckRange(first, count, corridor.getPathCount());
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const dtPolyRef* path = corridor.getPath();
  for (int32_t i = 0; i < count; ++i) {
    out[i] = static_cast<ZrcPolyRef>(path[first + i]);
  }
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentBoundaryCenter(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                      float* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out[0] = out[1] = out[2] = 0.f;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  // getCenter reports upstream's own FLT_MAX,FLT_MAX,FLT_MAX sentinel
  // (DetourLocalBoundary.cpp:27-32) when the boundary has never been
  // collected; copied out as-is rather than normalised to zero, since a
  // silent zero would read as a real position at the origin.
  const dtLocalBoundary& boundary = crowd->impl->getAgent(slot)->boundary;
  memcpy(out, boundary.getCenter(), sizeof(float) * 3);
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentBoundarySegmentCount(const ZrcCrowd* crowd,
                                            ZrcAgentRef ref,
                                            int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  *out_count = crowd->impl->getAgent(slot)->boundary.getSegmentCount();
  return ZRC_OK;
}

ZrcResult zrcCrowdAgentBoundarySegments(const ZrcCrowd* crowd, ZrcAgentRef ref,
                                        int32_t first, int32_t count,
                                        float* out) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  const dtLocalBoundary& boundary = crowd->impl->getAgent(slot)->boundary;
  const ZrcResult range_ok =
      zrc::CheckRange(first, count, boundary.getSegmentCount());
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  for (int32_t i = 0; i < count; ++i) {
    // getSegment is unchecked upstream (DetourLocalBoundary.h:58); the range
    // check above is what keeps this call in bounds.
    memcpy(&out[i * 6], boundary.getSegment(first + i), sizeof(float) * 6);
  }
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Where an agent is going
//===----------------------------------------------------------------------===//

ZrcResult zrcCrowdRequestMoveTarget(ZrcCrowd* crowd, ZrcAgentRef ref,
                                    ZrcPolyRef poly, const float* position) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Upstream itself refuses a zero ref (DetourCrowd.cpp:627-628); checked
  // here too, since a zero poly is a malformed call by this entry point's own
  // contract — clearing a target is zrcCrowdResetMoveTarget's job, not this
  // one's — rather than the generic ZRC_ERR_QUERY_FAILED below.
  if (poly == 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  // requestMoveTarget returns false only for a bad index or a zero ref
  // (DetourCrowd.cpp:625-628), both already excluded above; a false here
  // means upstream disagreed with a call this validation passed.
  if (!crowd->impl->requestMoveTarget(slot, static_cast<dtPolyRef>(poly), position)) {
    return ZRC_ERR_QUERY_FAILED;
  }
  return ZRC_OK;
}

ZrcResult zrcCrowdRequestMoveVelocity(ZrcCrowd* crowd, ZrcAgentRef ref,
                                      const float* velocity) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(velocity)) return ZRC_ERR_INVALID_ARGUMENT;
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  if (!crowd->impl->requestMoveVelocity(slot, velocity)) {
    return ZRC_ERR_QUERY_FAILED;
  }
  return ZRC_OK;
}

ZrcResult zrcCrowdResetMoveTarget(ZrcCrowd* crowd, ZrcAgentRef ref) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  int32_t slot = -1;
  const ZrcResult resolve_ok = ResolveAgent(crowd, ref, &slot);
  if (resolve_ok != ZRC_OK) return resolve_ok;

  if (!crowd->impl->resetMoveTarget(slot)) return ZRC_ERR_QUERY_FAILED;
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The frame
//===----------------------------------------------------------------------===//

ZrcResult zrcCrowdUpdate(ZrcCrowd* crowd, float dt, ZrcCrowdAgentDebug* debug) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPositiveFinite(dt)) return ZRC_ERR_INVALID_ARGUMENT;

  dtCrowdAgentDebugInfo native_debug;
  memset(&native_debug, 0, sizeof(native_debug));
  native_debug.idx = -1;

  if (debug != nullptr) {
    // Seeded from the caller's own struct so a corner this frame does not
    // touch reads as whatever this function last wrote there, matching
    // upstream's own contract for a debug object reused frame to frame
    // (DetourCrowd.cpp:1136-1150 only ever touches these inside a branch).
    memcpy(native_debug.optStart, debug->opt_start, sizeof(native_debug.optStart));
    memcpy(native_debug.optEnd, debug->opt_end, sizeof(native_debug.optEnd));

    // A reference that no longer resolves records nothing, exactly as 0 does,
    // and the frame still runs. Returning an error here instead would let a
    // debug view that outlived its agent stop the whole crowd advancing —
    // a host holding a reference across frames and removing that agent is the
    // ordinary case, not a misuse.
    int32_t slot = -1;
    if (debug->agent != 0 &&
        ResolveAgent(crowd, debug->agent, &slot) == ZRC_OK) {
      // Upstream's own idx names a position in the active-agent snapshot
      // update() builds this frame (DetourCrowd.cpp:1066, 1136, 1302), not a
      // pool slot: counted here as how many lower slots are currently active,
      // the same order getActiveAgents packs that snapshot in.
      int32_t position = 0;
      for (int32_t i = 0; i < slot; ++i) {
        if (crowd->serial[i] != 0) ++position;
      }
      native_debug.idx = position;
    }
    native_debug.vod = debug->samples != nullptr ? debug->samples->impl : nullptr;
  }

  crowd->impl->update(dt, debug != nullptr ? &native_debug : nullptr);
  ReleaseDrainedSlots(crowd);

  if (debug != nullptr) {
    memcpy(debug->opt_start, native_debug.optStart, sizeof(debug->opt_start));
    memcpy(debug->opt_end, native_debug.optEnd, sizeof(debug->opt_end));
  }
  return ZRC_OK;
}

ZrcResult zrcCrowdVelocitySampleCount(const ZrcCrowd* crowd, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_count = crowd->impl->getVelocitySampleCount();
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// What the whole crowd shares
//===----------------------------------------------------------------------===//

ZrcResult zrcCrowdSetAvoidanceParams(ZrcCrowd* crowd, int32_t index,
                                     const ZrcAvoidanceParams* params) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index < 0 || index >= ZRC_CROWD_MAX_AVOIDANCE_PARAMS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult params_ok = zrc::ValidateAvoidanceParams(*params);
  if (params_ok != ZRC_OK) return params_ok;

  dtObstacleAvoidanceParams native;
  memset(&native, 0, sizeof(native));
  zrc::ToDtAvoidanceParams(*params, &native);
  crowd->impl->setObstacleAvoidanceParams(index, &native);
  return ZRC_OK;
}

ZrcResult zrcCrowdAvoidanceParams(const ZrcCrowd* crowd, int32_t index,
                                  ZrcAvoidanceParams* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index < 0 || index >= ZRC_CROWD_MAX_AVOIDANCE_PARAMS) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // getObstacleAvoidanceParams bounds-checks and returns null only outside
  // that same range, already excluded above.
  const dtObstacleAvoidanceParams* native = crowd->impl->getObstacleAvoidanceParams(index);
  zrc::FromDtAvoidanceParams(*native, out);
  return ZRC_OK;
}

ZrcResult zrcCrowdSetFilter(ZrcCrowd* crowd, int32_t index,
                            const ZrcQueryFilter* filter) {
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index < 0 || index >= ZRC_CROWD_MAX_FILTERS) return ZRC_ERR_INVALID_ARGUMENT;
  if (filter == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  dtQueryFilter native;
  if (!zrc::BuildFilter(*filter, &native)) return ZRC_ERR_INVALID_ARGUMENT;

  // getEditableFilter bounds-checks and returns null only outside that same
  // range, already excluded above.
  *crowd->impl->getEditableFilter(index) = native;
  return ZRC_OK;
}

ZrcResult zrcCrowdFilter(const ZrcCrowd* crowd, int32_t index,
                         ZrcQueryFilter* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index < 0 || index >= ZRC_CROWD_MAX_FILTERS) return ZRC_ERR_INVALID_ARGUMENT;

  zrc::ReadFilter(*crowd->impl->getFilter(index), out);
  return ZRC_OK;
}

ZrcResult zrcCrowdQueryHalfExtents(const ZrcCrowd* crowd, float* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out[0] = out[1] = out[2] = 0.f;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  memcpy(out, crowd->impl->getQueryHalfExtents(), sizeof(float) * 3);
  return ZRC_OK;
}

ZrcResult zrcCrowdGrid(const ZrcCrowd* crowd, const ZrcProximityGrid** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out = crowd->grid;
  return ZRC_OK;
}

ZrcResult zrcCrowdPathQueue(const ZrcCrowd* crowd, const ZrcPathQueue** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out = crowd->path_queue;
  return ZRC_OK;
}

ZrcResult zrcCrowdNavMeshQuery(const ZrcCrowd* crowd, const ZrcNavMeshQuery** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (crowd == nullptr || crowd->impl == nullptr || crowd->serial == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out = crowd->query;
  return ZRC_OK;
}

}  // extern "C"
