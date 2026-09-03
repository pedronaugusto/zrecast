//===----------------------------------------------------------------------===//
// zrecast — the proximity grid and the obstacle-avoidance velocity sampler:
// a spatial hash over moving footprints, and the per-agent search that
// scores candidate velocities against nearby circles and walls.
//
// This file owns the lifetime of all three handles here — the grid, the
// sampler and its debug recorder — allocation fused with upstream's own
// init, and every accessor onto what they hold. dtProximityGrid and
// dtObstacleAvoidanceQuery/dtObstacleAvoidanceDebugData are the upstream
// types every function below answers to; zrcAvoidanceParamsDefault and the
// crowd that drives these per frame live elsewhere.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Shared bounds
//===----------------------------------------------------------------------===//

/// Ceiling on every count dtObstacleAvoidanceQuery::init and
/// dtObstacleAvoidanceDebugData::init multiply into an allocation of their
/// own, with no bound of their own (DetourObstacleAvoidance.cpp:232, 239,
/// 116-134). On a 32-bit size_t a large count can wrap the product to a
/// small allocation while the stored count stays large, and every element
/// accepted after that writes past the buffer it got. 65535 keeps every
/// such product well inside range on any width of size_t.
const int32_t kMaxAvoidanceCapacity = 65535;

//===----------------------------------------------------------------------===//
// Argument validation
//===----------------------------------------------------------------------===//

/// True when a live circle sits exactly at the sampling position.
///
/// dtObstacleAvoidanceQuery::prepare takes the vector from `pos` to each
/// circle and normalises it in place (DetourObstacleAvoidance.cpp:289-290);
/// dtVnormalize divides by the vector's own length with no zero guard
/// (DetourCommon.h:263-269), and that length is the full three-axis
/// distance, not a planar one. Exact coincidence on x, y and z at once is
/// what zeroes it — a circle merely close, or level with the agent but
/// offset in x or z, still normalises to a finite direction — so the
/// squared distance is compared against zero rather than an epsilon.
bool AnyCircleAtPosition(dtObstacleAvoidanceQuery& impl, const float* position) {
  const int count = impl.getObstacleCircleCount();
  for (int i = 0; i < count; ++i) {
    const dtObstacleCircle* circle = impl.getObstacleCircle(i);
    const float dx = circle->p[0] - position[0];
    const float dy = circle->p[1] - position[1];
    const float dz = circle->p[2] - position[2];
    if (dx * dx + dy * dy + dz * dz == 0.f) return true;
  }
  return false;
}

/// Shared by zrcAvoidanceSampleGrid and zrcAvoidanceSampleAdaptive: both take
/// the same eight inputs and both reach sampleVelocityGrid/Adaptive, which
/// memcpy `params` unconditionally (DetourObstacleAvoidance.cpp:442, 514)
/// and take 1/horizTime with no guard (:443, :515).
ZrcResult ValidateSampleArgs(ZrcAvoidanceQuery* query, const float* position,
                             float radius, float max_speed,
                             const float* velocity,
                             const float* desired_velocity,
                             const ZrcAvoidanceParams* params,
                             const ZrcAvoidanceDebug* debug) {
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPositiveFinite(radius)) return ZRC_ERR_INVALID_ARGUMENT;
  // vmax is guarded upstream with `vmax > 0 ? 1/vmax : FLT_MAX`
  // (DetourObstacleAvoidance.cpp:445), but a zero or negative max_speed
  // still describes an agent with no achievable velocity, so it is refused
  // here rather than silently substituted for.
  if (!zrc::IsPositiveFinite(max_speed)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(velocity) || !zrc::IsFiniteVec3(desired_velocity)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult params_ok = zrc::ValidateAvoidanceParams(*params);
  if (params_ok != ZRC_OK) return params_ok;
  if (debug != nullptr && debug->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (AnyCircleAtPosition(*query->impl, position)) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// The proximity grid
//===----------------------------------------------------------------------===//

ZrcResult zrcProximityGridCreate(int32_t pool_size, float cell_size,
                                 ZrcProximityGrid** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (pool_size < 1 || pool_size > zrc::kMaxProximityGridPool) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsPositiveFinite(cell_size)) return ZRC_ERR_INVALID_ARGUMENT;

  ZrcProximityGrid* handle = zrc::New<ZrcProximityGrid>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = dtAllocProximityGrid();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  // init() never frees a previous allocation of its own
  // (DetourProximityGrid.cpp:66-90), so this handle's impl is init()'d
  // exactly once, here, and offers no re-init entry point.
  if (!handle->impl->init(pool_size, cell_size)) {
    dtFreeProximityGrid(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->owns = true;
  *out = handle;
  return ZRC_OK;
}

void zrcProximityGridDestroy(ZrcProximityGrid* grid) {
  if (grid == nullptr) return;
  // A crowd hands its own grid out with owns == false; freeing it here would
  // free memory dtCrowd still uses until the crowd itself is destroyed or
  // re-initialised.
  if (!grid->owns) return;
  dtFreeProximityGrid(grid->impl);
  zrc::Delete(grid);
}

ZrcResult zrcProximityGridClear(ZrcProximityGrid* grid) {
  if (grid == nullptr || grid->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  grid->impl->clear();
  return ZRC_OK;
}

ZrcResult zrcProximityGridAddItem(ZrcProximityGrid* grid, uint16_t id,
                                  float min_x, float min_y, float max_x,
                                  float max_y) {
  if (grid == nullptr || grid->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const float inv_cell_size = 1.0f / grid->impl->getCellSize();
  const ZrcResult box_ok =
      zrc::CheckProximityBox(min_x, min_y, max_x, max_y, inv_cell_size);
  if (box_ok != ZRC_OK) return box_ok;

  grid->impl->addItem(id, min_x, min_y, max_x, max_y);
  return ZRC_OK;
}

ZrcResult zrcProximityGridQueryItems(const ZrcProximityGrid* grid, float min_x,
                                     float min_y, float max_x, float max_y,
                                     uint16_t* out_ids, int32_t max_ids,
                                     int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (grid == nullptr || grid->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_ids < 0 || max_ids > zrc::kMaxProximityGridPool) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_ids > 0 && out_ids == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const float inv_cell_size = 1.0f / grid->impl->getCellSize();
  const ZrcResult box_ok =
      zrc::CheckProximityBox(min_x, min_y, max_x, max_y, inv_cell_size);
  if (box_ok != ZRC_OK) return box_ok;

  // queryItems stops at maxIds and returns that many with no signal
  // (DetourProximityGrid.cpp:166-167), so a return of exactly maxIds is
  // ambiguous between "that many exist" and "more exist and it stopped".
  // Asking for one more than the caller's buffer resolves it: anything at
  // or under max_ids is the whole answer, and anything past it means the
  // real neighbourhood does not fit.
  const int32_t capacity = max_ids + 1;
  zrc::TempBuffer scratch(sizeof(unsigned short) *
                          static_cast<size_t>(capacity));
  unsigned short* ids = static_cast<unsigned short*>(scratch.get());
  if (ids == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  const int found =
      grid->impl->queryItems(min_x, min_y, max_x, max_y, ids, capacity);
  const bool clipped = found > max_ids;
  const int32_t to_copy = clipped ? max_ids : found;
  if (to_copy > 0) {
    memcpy(out_ids, ids, sizeof(uint16_t) * static_cast<size_t>(to_copy));
  }
  *out_count = to_copy;
  return clipped ? ZRC_ERR_BUFFER_TOO_SMALL : ZRC_OK;
}

ZrcResult zrcProximityGridItemCountAt(const ZrcProximityGrid* grid, int32_t x,
                                      int32_t y, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (grid == nullptr || grid->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  // getItemCountAt hashes x and y without bounding either
  // (DetourProximityGrid.cpp:179-183); the mask is safe once init() has
  // run, since m_bucketsSize is a power of two, but no item can ever have
  // been filed this far out, so it is refused rather than answered as zero.
  if (x < -zrc::kMaxProximityGridCell || x > zrc::kMaxProximityGridCell ||
      y < -zrc::kMaxProximityGridCell || y > zrc::kMaxProximityGridCell) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_count = grid->impl->getItemCountAt(x, y);
  return ZRC_OK;
}

ZrcResult zrcProximityGridBounds(const ZrcProximityGrid* grid, int32_t* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  if (grid == nullptr || grid->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const int* bounds = grid->impl->getBounds();
  out[0] = bounds[0];
  out[1] = bounds[1];
  out[2] = bounds[2];
  out[3] = bounds[3];
  return ZRC_OK;
}

ZrcResult zrcProximityGridCellSize(const ZrcProximityGrid* grid, float* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = 0.f;
  if (grid == nullptr || grid->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = grid->impl->getCellSize();
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Obstacle avoidance
//===----------------------------------------------------------------------===//

ZrcResult zrcAvoidanceQueryCreate(int32_t max_circles, int32_t max_segments,
                                  ZrcAvoidanceQuery** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (max_circles < 1 || max_circles > kMaxAvoidanceCapacity ||
      max_segments < 1 || max_segments > kMaxAvoidanceCapacity) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  ZrcAvoidanceQuery* handle = zrc::New<ZrcAvoidanceQuery>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = dtAllocObstacleAvoidanceQuery();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  // init() never frees a previous allocation either
  // (DetourObstacleAvoidance.cpp:228-245); one init per handle, here.
  if (!handle->impl->init(max_circles, max_segments)) {
    dtFreeObstacleAvoidanceQuery(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->max_circles = max_circles;
  handle->max_segments = max_segments;
  *out = handle;
  return ZRC_OK;
}

void zrcAvoidanceQueryDestroy(ZrcAvoidanceQuery* query) {
  if (query == nullptr) return;
  dtFreeObstacleAvoidanceQuery(query->impl);
  zrc::Delete(query);
}

ZrcResult zrcAvoidanceQueryReset(ZrcAvoidanceQuery* query) {
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  query->impl->reset();
  return ZRC_OK;
}

ZrcResult zrcAvoidanceAddCircle(ZrcAvoidanceQuery* query, const float* position,
                                float radius, const float* velocity,
                                const float* desired_velocity) {
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPosition(position)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPositiveFinite(radius)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(velocity) || !zrc::IsFiniteVec3(desired_velocity)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (query->impl->getObstacleCircleCount() >= query->max_circles) {
    return ZRC_ERR_BUFFER_TOO_SMALL;
  }
  query->impl->addCircle(position, radius, velocity, desired_velocity);
  return ZRC_OK;
}

ZrcResult zrcAvoidanceAddSegment(ZrcAvoidanceQuery* query, const float* p,
                                 const float* q) {
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsPosition(p) || !zrc::IsPosition(q)) return ZRC_ERR_INVALID_ARGUMENT;
  if (query->impl->getObstacleSegmentCount() >= query->max_segments) {
    return ZRC_ERR_BUFFER_TOO_SMALL;
  }
  query->impl->addSegment(p, q);
  return ZRC_OK;
}

ZrcResult zrcAvoidanceSampleGrid(ZrcAvoidanceQuery* query, const float* position,
                                 float radius, float max_speed,
                                 const float* velocity,
                                 const float* desired_velocity,
                                 const ZrcAvoidanceParams* params,
                                 ZrcAvoidanceDebug* debug, float* out_velocity,
                                 int32_t* out_samples) {
  if (out_velocity == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_velocity[0] = 0.f;
  out_velocity[1] = 0.f;
  out_velocity[2] = 0.f;
  if (out_samples != nullptr) *out_samples = 0;

  const ZrcResult args_ok =
      ValidateSampleArgs(query, position, radius, max_speed, velocity,
                         desired_velocity, params, debug);
  if (args_ok != ZRC_OK) return args_ok;

  dtObstacleAvoidanceParams dt_params;
  zrc::ToDtAvoidanceParams(*params, &dt_params);
  dtObstacleAvoidanceDebugData* debug_impl =
      debug != nullptr ? debug->impl : nullptr;

  const int samples = query->impl->sampleVelocityGrid(
      position, radius, max_speed, velocity, desired_velocity, out_velocity,
      &dt_params, debug_impl);
  if (out_samples != nullptr) *out_samples = samples;
  return ZRC_OK;
}

ZrcResult zrcAvoidanceSampleAdaptive(ZrcAvoidanceQuery* query,
                                     const float* position, float radius,
                                     float max_speed, const float* velocity,
                                     const float* desired_velocity,
                                     const ZrcAvoidanceParams* params,
                                     ZrcAvoidanceDebug* debug,
                                     float* out_velocity,
                                     int32_t* out_samples) {
  if (out_velocity == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out_velocity[0] = 0.f;
  out_velocity[1] = 0.f;
  out_velocity[2] = 0.f;
  if (out_samples != nullptr) *out_samples = 0;

  const ZrcResult args_ok =
      ValidateSampleArgs(query, position, radius, max_speed, velocity,
                         desired_velocity, params, debug);
  if (args_ok != ZRC_OK) return args_ok;

  dtObstacleAvoidanceParams dt_params;
  zrc::ToDtAvoidanceParams(*params, &dt_params);
  dtObstacleAvoidanceDebugData* debug_impl =
      debug != nullptr ? debug->impl : nullptr;

  const int samples = query->impl->sampleVelocityAdaptive(
      position, radius, max_speed, velocity, desired_velocity, out_velocity,
      &dt_params, debug_impl);
  if (out_samples != nullptr) *out_samples = samples;
  return ZRC_OK;
}

ZrcResult zrcAvoidanceCircleCount(const ZrcAvoidanceQuery* query,
                                  int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = query->impl->getObstacleCircleCount();
  return ZRC_OK;
}

ZrcResult zrcAvoidanceCircleAt(const ZrcAvoidanceQuery* query, int32_t index,
                               ZrcAvoidanceCircle* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtObstacleAvoidanceQuery& impl = *query->impl;
  if (index < 0 || index >= impl.getObstacleCircleCount()) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // getObstacleCircle indexes m_circles unchecked and is not const-qualified
  // (DetourObstacleAvoidance.h:123) though it only returns a pointer into
  // that array; the cast reaches through a read-only borrow to call it.
  const dtObstacleCircle* circle =
      const_cast<dtObstacleAvoidanceQuery&>(impl).getObstacleCircle(index);
  for (int i = 0; i < 3; ++i) {
    out->position[i] = circle->p[i];
    out->velocity[i] = circle->vel[i];
    out->desired_velocity[i] = circle->dvel[i];
  }
  out->radius = circle->rad;
  return ZRC_OK;
}

ZrcResult zrcAvoidanceSegmentCount(const ZrcAvoidanceQuery* query,
                                   int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = query->impl->getObstacleSegmentCount();
  return ZRC_OK;
}

ZrcResult zrcAvoidanceSegmentAt(const ZrcAvoidanceQuery* query, int32_t index,
                                ZrcAvoidanceSegment* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (query == nullptr || query->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtObstacleAvoidanceQuery& impl = *query->impl;
  if (index < 0 || index >= impl.getObstacleSegmentCount()) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // Same shape as zrcAvoidanceCircleAt: getObstacleSegment is unchecked and
  // not const-qualified (DetourObstacleAvoidance.h:126).
  const dtObstacleSegment* seg =
      const_cast<dtObstacleAvoidanceQuery&>(impl).getObstacleSegment(index);
  for (int i = 0; i < 3; ++i) {
    out->p[i] = seg->p[i];
    out->q[i] = seg->q[i];
  }
  out->touching = seg->touch ? ZRC_TRUE : ZRC_FALSE;
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Avoidance debug recording
//===----------------------------------------------------------------------===//

ZrcResult zrcAvoidanceDebugCreate(int32_t max_samples, ZrcAvoidanceDebug** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (max_samples < 1 || max_samples > kMaxAvoidanceCapacity) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  ZrcAvoidanceDebug* handle = zrc::New<ZrcAvoidanceDebug>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = dtAllocObstacleAvoidanceDebugData();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  // init() allocates seven parallel arrays and can fail partway through
  // them (DetourObstacleAvoidance.cpp:111-138); the destructor frees
  // whatever it got, so a false return is handled by destroying the object
  // rather than by trying it again.
  if (!handle->impl->init(max_samples)) {
    dtFreeObstacleAvoidanceDebugData(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->max_samples = max_samples;
  *out = handle;
  return ZRC_OK;
}

void zrcAvoidanceDebugDestroy(ZrcAvoidanceDebug* debug) {
  if (debug == nullptr) return;
  dtFreeObstacleAvoidanceDebugData(debug->impl);
  zrc::Delete(debug);
}

ZrcResult zrcAvoidanceDebugReset(ZrcAvoidanceDebug* debug) {
  if (debug == nullptr || debug->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  debug->impl->reset();
  return ZRC_OK;
}

ZrcResult zrcAvoidanceDebugAddSample(ZrcAvoidanceDebug* debug,
                                     const ZrcAvoidanceSample* sample) {
  if (debug == nullptr || debug->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (sample == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(sample->velocity)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(sample->size) || !zrc::IsFinite(sample->penalty) ||
      !zrc::IsFinite(sample->desired_velocity_penalty) ||
      !zrc::IsFinite(sample->current_velocity_penalty) ||
      !zrc::IsFinite(sample->preferred_side_penalty) ||
      !zrc::IsFinite(sample->collision_time_penalty)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (debug->impl->getSampleCount() >= debug->max_samples) {
    return ZRC_ERR_BUFFER_TOO_SMALL;
  }
  debug->impl->addSample(sample->velocity, sample->size, sample->penalty,
                         sample->desired_velocity_penalty,
                         sample->current_velocity_penalty,
                         sample->preferred_side_penalty,
                         sample->collision_time_penalty);
  return ZRC_OK;
}

ZrcResult zrcAvoidanceDebugNormalize(ZrcAvoidanceDebug* debug) {
  if (debug == nullptr || debug->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  debug->impl->normalizeSamples();
  return ZRC_OK;
}

ZrcResult zrcAvoidanceDebugSampleCount(const ZrcAvoidanceDebug* debug,
                                       int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (debug == nullptr || debug->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = debug->impl->getSampleCount();
  return ZRC_OK;
}

ZrcResult zrcAvoidanceDebugSampleAt(const ZrcAvoidanceDebug* debug,
                                    int32_t index, ZrcAvoidanceSample* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (debug == nullptr || debug->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtObstacleAvoidanceDebugData& impl = *debug->impl;
  if (index < 0 || index >= impl.getSampleCount()) return ZRC_ERR_INVALID_ARGUMENT;

  const float* velocity = impl.getSampleVelocity(index);
  out->velocity[0] = velocity[0];
  out->velocity[1] = velocity[1];
  out->velocity[2] = velocity[2];
  out->size = impl.getSampleSize(index);
  out->penalty = impl.getSamplePenalty(index);
  out->desired_velocity_penalty = impl.getSampleDesiredVelocityPenalty(index);
  out->current_velocity_penalty = impl.getSampleCurrentVelocityPenalty(index);
  out->preferred_side_penalty = impl.getSamplePreferredSidePenalty(index);
  out->collision_time_penalty = impl.getSampleCollisionTimePenalty(index);
  return ZRC_OK;
}

}  // extern "C"
