//===----------------------------------------------------------------------===//
// zrecast — the caller-supplied build context, and the entry points that size
// a build before any voxel exists.
//
// zrc::HostContext is what every staged Recast entry point in this package
// hands upstream in place of a bare rcContext: it wraps a caller's
// ZrcBuildContext so upstream's log and timer virtuals reach the caller's own
// hooks, with upstream's own enable-flag gating intact. The six
// zrcBuildContext* entry points below drive that same context directly, for a
// caller logging a message or reading a timer with no build stage in flight.
// The four sizing entry points answer what a caller needs before a
// heightfield exists at all: a mesh's bounds, the voxel grid those bounds
// imply at a given cell size, and which triangles are walkable at a given
// slope.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

/// Same bound zrecast_bake.cpp's ValidateConfig applies to agent_max_slope:
/// rcMarkWalkableTriangles and rcClearUnwalkableTriangles both take the
/// cosine of this angle, and outside [0, 90) the threshold stops meaning
/// anything.
bool IsValidSlopeAngle(float degrees) {
  return zrc::IsFinite(degrees) && degrees >= 0.f && degrees < 90.f;
}

}  // namespace

namespace zrc {

HostContext::HostContext(const ZrcBuildContext* hooks)
    : rcContext(false), hooks_(hooks) {
  if (hooks != nullptr) {
    enableLog(hooks->log_enabled != ZRC_FALSE);
    enableTimer(hooks->timers_enabled != ZRC_FALSE);
  }
}

void HostContext::doResetLog() {
  if (hooks_ != nullptr && hooks_->reset_log != nullptr) {
    hooks_->reset_log(hooks_->user);
  }
}

void HostContext::doLog(rcLogCategory category, const char* msg, int len) {
  if (hooks_ != nullptr && hooks_->log != nullptr) {
    hooks_->log(hooks_->user, static_cast<ZrcLogCategory>(category), msg, len);
  }
}

void HostContext::doResetTimers() {
  if (hooks_ != nullptr && hooks_->reset_timers != nullptr) {
    hooks_->reset_timers(hooks_->user);
  }
}

void HostContext::doStartTimer(rcTimerLabel label) {
  if (hooks_ != nullptr && hooks_->start_timer != nullptr) {
    hooks_->start_timer(hooks_->user, static_cast<ZrcTimerLabel>(label));
  }
}

void HostContext::doStopTimer(rcTimerLabel label) {
  if (hooks_ != nullptr && hooks_->stop_timer != nullptr) {
    hooks_->stop_timer(hooks_->user, static_cast<ZrcTimerLabel>(label));
  }
}

int HostContext::doGetAccumulatedTime(rcTimerLabel label) const {
  if (hooks_ != nullptr && hooks_->accumulated_time != nullptr) {
    return hooks_->accumulated_time(hooks_->user,
                                    static_cast<ZrcTimerLabel>(label));
  }
  return -1;
}

}  // namespace zrc

extern "C" {

ZrcResult zrcBuildContextLog(const ZrcBuildContext* context,
                             ZrcLogCategory category, const char* message) {
  if (message == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsLogCategory(category)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  // "%s" rather than passing message as the format itself: a caller's text
  // becomes data, not a format string upstream's varargs log() would parse.
  ctx.log(static_cast<rcLogCategory>(category), "%s", message);
  return ZRC_OK;
}

ZrcResult zrcBuildContextResetLog(const ZrcBuildContext* context) {
  zrc::HostContext ctx(context);
  ctx.resetLog();
  return ZRC_OK;
}

ZrcResult zrcBuildContextResetTimers(const ZrcBuildContext* context) {
  zrc::HostContext ctx(context);
  ctx.resetTimers();
  return ZRC_OK;
}

ZrcResult zrcBuildContextStartTimer(const ZrcBuildContext* context,
                                    ZrcTimerLabel label) {
  // ZRC_MAX_TIMERS is the length of the table upstream indexes with a label,
  // not a label; passing it through would index one past the table.
  if (!zrc::IsTimerLabel(label)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  ctx.startTimer(static_cast<rcTimerLabel>(label));
  return ZRC_OK;
}

ZrcResult zrcBuildContextStopTimer(const ZrcBuildContext* context,
                                   ZrcTimerLabel label) {
  if (!zrc::IsTimerLabel(label)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  ctx.stopTimer(static_cast<rcTimerLabel>(label));
  return ZRC_OK;
}

ZrcResult zrcBuildContextAccumulatedTime(const ZrcBuildContext* context,
                                         ZrcTimerLabel label,
                                         int32_t* out_time) {
  if (out_time == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_time = -1;
  if (!zrc::IsTimerLabel(label)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  *out_time = ctx.getAccumulatedTime(static_cast<rcTimerLabel>(label));
  return ZRC_OK;
}

ZrcResult zrcCalcBounds(const ZrcTriMesh* mesh, float* bmin, float* bmax) {
  if (bmin == nullptr || bmax == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  bmin[0] = bmin[1] = bmin[2] = 0.f;
  bmax[0] = bmax[1] = bmax[2] = 0.f;
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) return mesh_valid;

  rcCalcBounds(mesh->verts, mesh->vert_count, bmin, bmax);
  return ZRC_OK;
}

ZrcResult zrcCalcGridSize(const float* bmin, const float* bmax,
                          float cell_size, int32_t* out_width,
                          int32_t* out_height) {
  if (out_width == nullptr || out_height == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_width = 0;
  *out_height = 0;
  if (!zrc::IsFiniteVec3(bmin) || !zrc::IsFiniteVec3(bmax)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cell_size) || !(cell_size > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (zrc::CheckGridExtentFit(bmin, bmax, cell_size) != zrc::GridExtent::kOk) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  int width = 0;
  int height = 0;
  rcCalcGridSize(bmin, bmax, cell_size, &width, &height);
  *out_width = width;
  *out_height = height;
  return ZRC_OK;
}

ZrcResult zrcMarkWalkableTriangles(const ZrcBuildContext* context,
                                   float walkable_slope_angle,
                                   const ZrcTriMesh* mesh,
                                   uint8_t* out_areas) {
  if (mesh == nullptr || out_areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) return mesh_valid;
  if (!IsValidSlopeAngle(walkable_slope_angle)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // zrc::MarkWalkableTriangles rather than rcMarkWalkableTriangles: the two do
  // the same work over the same vector helpers, but upstream takes its
  // threshold from cosf, which the C standard does not require to be
  // correctly rounded. A cook that used it would depend on the host's libm in
  // the one comparison that decides whether a surface is navigable. The
  // context is unused because neither version logs or times anything.
  (void)context;
  zrc::MarkWalkableTriangles(walkable_slope_angle, *mesh, out_areas);
  return ZRC_OK;
}

ZrcResult zrcClearUnwalkableTriangles(const ZrcBuildContext* context,
                                      float walkable_slope_angle,
                                      const ZrcTriMesh* mesh,
                                      uint8_t* io_areas) {
  if (mesh == nullptr || io_areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult mesh_valid = zrc::ValidateTriMesh(*mesh);
  if (mesh_valid != ZRC_OK) return mesh_valid;
  if (!IsValidSlopeAngle(walkable_slope_angle)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // The same substitution, for the same reason, as zrcMarkWalkableTriangles.
  (void)context;
  zrc::ClearUnwalkableTriangles(walkable_slope_angle, *mesh, io_areas);
  return ZRC_OK;
}

}  // extern "C"
