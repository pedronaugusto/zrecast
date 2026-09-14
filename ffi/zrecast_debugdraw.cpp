//===----------------------------------------------------------------------===//
// zrecast — DebugUtils: the renderer interface, the primitives, the display
// list, and every draw call over a Recast or Detour container.
//
// Upstream's drawing code is defensive about its renderer and its arrays — it
// returns without drawing when either is null — and silent about everything
// else. That silence is what this unit removes: a null renderer, an
// incompletely filled one, an index outside a layer set or a container that
// was never built are refused here with a result, rather than drawing nothing
// and reporting success.
//
// No entry point in this unit allocates, and none mutates what it is given.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

/// Every hook but `area_to_col` is required: upstream declares all nine
/// virtuals pure but one, and a draw call reaches whichever of them the shape
/// it is drawing needs. A partially filled table would therefore fail on some
/// containers and not on others.
ZrcResult ValidateDebugDraw(const ZrcDebugDraw* dd) {
  if (dd == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (dd->depth_mask == nullptr || dd->texture == nullptr ||
      dd->begin == nullptr || dd->vertex == nullptr ||
      dd->vertex_xyz == nullptr || dd->vertex_uv == nullptr ||
      dd->vertex_xyz_uv == nullptr || dd->end == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

bool IsPrimitive(ZrcDebugDrawPrimitive prim) {
  return prim == ZRC_DEBUG_DRAW_POINTS || prim == ZRC_DEBUG_DRAW_LINES ||
         prim == ZRC_DEBUG_DRAW_TRIS || prim == ZRC_DEBUG_DRAW_QUADS;
}

/// Every float a shape's geometry is built from has to be finite: upstream
/// multiplies and adds them straight into the vertices it emits, so a NaN
/// reaches a host's vertex buffer rather than being caught anywhere.
bool AllFinite(const float* values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!zrc::IsFinite(values[i])) return false;
  }
  return true;
}

}  // namespace

namespace zrc {

//===----------------------------------------------------------------------===//
// The renderer adapter, declared in zrecast_internal.h
//===----------------------------------------------------------------------===//

HostDebugDraw::HostDebugDraw(const ZrcDebugDraw& hooks) : hooks_(hooks) {}

void HostDebugDraw::depthMask(bool state) {
  hooks_.depth_mask(hooks_.user, state ? ZRC_TRUE : ZRC_FALSE);
}

void HostDebugDraw::texture(bool state) {
  hooks_.texture(hooks_.user, state ? ZRC_TRUE : ZRC_FALSE);
}

void HostDebugDraw::begin(duDebugDrawPrimitives prim, float size) {
  hooks_.begin(hooks_.user, static_cast<ZrcDebugDrawPrimitive>(prim), size);
}

void HostDebugDraw::vertex(const float* pos, unsigned int color) {
  hooks_.vertex(hooks_.user, pos, color);
}

void HostDebugDraw::vertex(const float x, const float y, const float z,
                           unsigned int color) {
  hooks_.vertex_xyz(hooks_.user, x, y, z, color);
}

void HostDebugDraw::vertex(const float* pos, unsigned int color,
                           const float* uv) {
  hooks_.vertex_uv(hooks_.user, pos, color, uv);
}

void HostDebugDraw::vertex(const float x, const float y, const float z,
                           unsigned int color, const float u, const float v) {
  hooks_.vertex_xyz_uv(hooks_.user, x, y, z, color, u, v);
}

void HostDebugDraw::end() { hooks_.end(hooks_.user); }

unsigned int HostDebugDraw::areaToCol(unsigned int area) {
  if (hooks_.area_to_col == nullptr) return duDebugDraw::areaToCol(area);
  return hooks_.area_to_col(hooks_.user, area);
}

}  // namespace zrc

//===----------------------------------------------------------------------===//
// Colours
//
// These are inline in upstream's header, so they cross as entry points rather
// than as a mirror: an inline definition has no symbol a C host could link,
// and a second implementation here would be one more thing to keep in step
// with a re-vendor.
//===----------------------------------------------------------------------===//

ZRC_API uint32_t zrcDebugRgba(int32_t r, int32_t g, int32_t b, int32_t a) {
  return duRGBA(r, g, b, a);
}

ZRC_API uint32_t zrcDebugRgbaFloat(float r, float g, float b, float a) {
  return duRGBAf(r, g, b, a);
}

ZRC_API uint32_t zrcDebugIntToCol(int32_t i, int32_t a) {
  return duIntToCol(i, a);
}

ZRC_API ZrcResult zrcDebugIntToColFloat(int32_t i, float* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  duIntToCol(i, out);
  return ZRC_OK;
}

ZRC_API uint32_t zrcDebugMultCol(uint32_t col, uint32_t d) {
  return duMultCol(col, d);
}

ZRC_API uint32_t zrcDebugDarkenCol(uint32_t col) { return duDarkenCol(col); }

ZRC_API uint32_t zrcDebugLerpCol(uint32_t a, uint32_t b, uint32_t u) {
  return duLerpCol(a, b, u);
}

ZRC_API uint32_t zrcDebugTransCol(uint32_t c, uint32_t a) {
  return duTransCol(c, a);
}

ZRC_API ZrcResult zrcDebugCalcBoxColors(uint32_t* out, uint32_t top,
                                        uint32_t side) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  duCalcBoxColors(out, top, side);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Primitives
//===----------------------------------------------------------------------===//

ZRC_API ZrcResult zrcDebugDrawBoxWire(const ZrcDebugDraw* dd, float minx,
                                      float miny, float minz, float maxx,
                                      float maxy, float maxz, uint32_t col,
                                      float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[7] = {minx, miny, minz, maxx, maxy, maxz, line_width};
  if (!AllFinite(box, 7)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawBoxWire(&draw, minx, miny, minz, maxx, maxy, maxz, col,
                     line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCylinderWire(const ZrcDebugDraw* dd, float minx,
                                           float miny, float minz, float maxx,
                                           float maxy, float maxz, uint32_t col,
                                           float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[7] = {minx, miny, minz, maxx, maxy, maxz, line_width};
  if (!AllFinite(box, 7)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCylinderWire(&draw, minx, miny, minz, maxx, maxy, maxz, col,
                          line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawArc(const ZrcDebugDraw* dd, float x0, float y0,
                                  float z0, float x1, float y1, float z1,
                                  float h, float as0, float as1, uint32_t col,
                                  float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[10] = {x0, y0, z0, x1, y1, z1, h, as0, as1, line_width};
  if (!AllFinite(args, 10)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawArc(&draw, x0, y0, z0, x1, y1, z1, h, as0, as1, col, line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawArrow(const ZrcDebugDraw* dd, float x0, float y0,
                                    float z0, float x1, float y1, float z1,
                                    float as0, float as1, uint32_t col,
                                    float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[9] = {x0, y0, z0, x1, y1, z1, as0, as1, line_width};
  if (!AllFinite(args, 9)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawArrow(&draw, x0, y0, z0, x1, y1, z1, as0, as1, col, line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCircle(const ZrcDebugDraw* dd, float x, float y,
                                     float z, float r, uint32_t col,
                                     float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[5] = {x, y, z, r, line_width};
  if (!AllFinite(args, 5)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCircle(&draw, x, y, z, r, col, line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCross(const ZrcDebugDraw* dd, float x, float y,
                                    float z, float size, uint32_t col,
                                    float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[5] = {x, y, z, size, line_width};
  if (!AllFinite(args, 5)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCross(&draw, x, y, z, size, col, line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawBox(const ZrcDebugDraw* dd, float minx,
                                  float miny, float minz, float maxx,
                                  float maxy, float maxz,
                                  const uint32_t* fcol) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (fcol == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawBox(&draw, minx, miny, minz, maxx, maxy, maxz, fcol);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCylinder(const ZrcDebugDraw* dd, float minx,
                                       float miny, float minz, float maxx,
                                       float maxy, float maxz, uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCylinder(&draw, minx, miny, minz, maxx, maxy, maxz, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawGridXZ(const ZrcDebugDraw* dd, float ox, float oy,
                                     float oz, int32_t w, int32_t h, float size,
                                     uint32_t col, float line_width) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (w < 0 || h < 0) return ZRC_ERR_INVALID_ARGUMENT;
  const float args[5] = {ox, oy, oz, size, line_width};
  if (!AllFinite(args, 5)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawGridXZ(&draw, ox, oy, oz, w, h, size, col, line_width);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendBoxWire(const ZrcDebugDraw* dd, float minx,
                                        float miny, float minz, float maxx,
                                        float maxy, float maxz, uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendBoxWire(&draw, minx, miny, minz, maxx, maxy, maxz, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendBoxPoints(const ZrcDebugDraw* dd, float minx,
                                          float miny, float minz, float maxx,
                                          float maxy, float maxz,
                                          uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendBoxPoints(&draw, minx, miny, minz, maxx, maxy, maxz, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendCylinderWire(const ZrcDebugDraw* dd, float minx,
                                             float miny, float minz, float maxx,
                                             float maxy, float maxz,
                                             uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendCylinderWire(&draw, minx, miny, minz, maxx, maxy, maxz, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendArc(const ZrcDebugDraw* dd, float x0, float y0,
                                    float z0, float x1, float y1, float z1,
                                    float h, float as0, float as1,
                                    uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[9] = {x0, y0, z0, x1, y1, z1, h, as0, as1};
  if (!AllFinite(args, 9)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendArc(&draw, x0, y0, z0, x1, y1, z1, h, as0, as1, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendArrow(const ZrcDebugDraw* dd, float x0,
                                      float y0, float z0, float x1, float y1,
                                      float z1, float as0, float as1,
                                      uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[8] = {x0, y0, z0, x1, y1, z1, as0, as1};
  if (!AllFinite(args, 8)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendArrow(&draw, x0, y0, z0, x1, y1, z1, as0, as1, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendCircle(const ZrcDebugDraw* dd, float x, float y,
                                       float z, float r, uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[4] = {x, y, z, r};
  if (!AllFinite(args, 4)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendCircle(&draw, x, y, z, r, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendCross(const ZrcDebugDraw* dd, float x, float y,
                                      float z, float size, uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float args[4] = {x, y, z, size};
  if (!AllFinite(args, 4)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendCross(&draw, x, y, z, size, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendBox(const ZrcDebugDraw* dd, float minx,
                                    float miny, float minz, float maxx,
                                    float maxy, float maxz,
                                    const uint32_t* fcol) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (fcol == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendBox(&draw, minx, miny, minz, maxx, maxy, maxz, fcol);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugAppendCylinder(const ZrcDebugDraw* dd, float minx,
                                         float miny, float minz, float maxx,
                                         float maxy, float maxz, uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  const float box[6] = {minx, miny, minz, maxx, maxy, maxz};
  if (!AllFinite(box, 6)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duAppendCylinder(&draw, minx, miny, minz, maxx, maxy, maxz, col);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The display list
//===----------------------------------------------------------------------===//

namespace {

/// The four hooks a ZrcDebugDraw over a display list needs. `user` is the
/// ZrcDisplayList, so the table carries no state of its own and stays valid
/// for as long as the list does.
void ListDepthMask(void* user, ZrcBool state) {
  static_cast<ZrcDisplayList*>(user)->impl->depthMask(state != ZRC_FALSE);
}

void ListTexture(void* user, ZrcBool state) {
  (void)user;
  (void)state;
}

void ListBegin(void* user, ZrcDebugDrawPrimitive prim, float size) {
  static_cast<ZrcDisplayList*>(user)->impl->begin(
      static_cast<duDebugDrawPrimitives>(prim), size);
}

void ListVertex(void* user, const float* pos, uint32_t color) {
  static_cast<ZrcDisplayList*>(user)->impl->vertex(pos, color);
}

void ListVertexXYZ(void* user, float x, float y, float z, uint32_t color) {
  static_cast<ZrcDisplayList*>(user)->impl->vertex(x, y, z, color);
}

void ListVertexUV(void* user, const float* pos, uint32_t color,
                  const float* uv) {
  (void)uv;
  static_cast<ZrcDisplayList*>(user)->impl->vertex(pos, color);
}

void ListVertexXYZUV(void* user, float x, float y, float z, uint32_t color,
                     float u, float v) {
  (void)u;
  (void)v;
  static_cast<ZrcDisplayList*>(user)->impl->vertex(x, y, z, color);
}

void ListEnd(void* user) { static_cast<ZrcDisplayList*>(user)->impl->end(); }

}  // namespace

ZRC_API ZrcResult zrcDisplayListCreate(int32_t capacity,
                                       ZrcDisplayList** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  // Upstream multiplies the capacity by three for the position array and
  // allocates `new float[cap*3]`, so a capacity near INT_MAX overflows the
  // subscript before any allocation is attempted. The bound is the largest
  // value whose triple still fits an int with room to double once, which is
  // the growth step duDisplayList::vertex takes.
  if (capacity < 0 || capacity > 0x10000000) return ZRC_ERR_INVALID_ARGUMENT;

  ZrcDisplayList* list = zrc::New<ZrcDisplayList>();
  if (list == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  list->impl = zrc::New<zrc::ConcreteDisplayList>(capacity);
  if (list->impl == nullptr) {
    zrc::Delete(list);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  *out = list;
  return ZRC_OK;
}

ZRC_API void zrcDisplayListDestroy(ZrcDisplayList* list) {
  if (list == nullptr) return;
  zrc::Delete(list->impl);
  zrc::Delete(list);
}

ZRC_API ZrcResult zrcDisplayListRecorder(ZrcDisplayList* list,
                                         ZrcDebugDraw* out) {
  if (list == nullptr || out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out->user = list;
  out->depth_mask = ListDepthMask;
  out->texture = ListTexture;
  out->begin = ListBegin;
  out->vertex = ListVertex;
  out->vertex_xyz = ListVertexXYZ;
  out->vertex_uv = ListVertexUV;
  out->vertex_xyz_uv = ListVertexXYZUV;
  out->end = ListEnd;
  out->area_to_col = nullptr;
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListDepthMask(ZrcDisplayList* list,
                                          ZrcBool state) {
  if (list == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  list->impl->depthMask(state != ZRC_FALSE);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListBegin(ZrcDisplayList* list,
                                      ZrcDebugDrawPrimitive prim, float size) {
  if (list == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!IsPrimitive(prim)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(size)) return ZRC_ERR_INVALID_ARGUMENT;
  list->impl->begin(static_cast<duDebugDrawPrimitives>(prim), size);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListVertex(ZrcDisplayList* list, const float* pos,
                                       uint32_t color) {
  if (list == nullptr || pos == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;
  list->impl->vertex(pos, color);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListVertexXYZ(ZrcDisplayList* list, float x,
                                          float y, float z, uint32_t color) {
  if (list == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const float pos[3] = {x, y, z};
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;
  list->impl->vertex(x, y, z, color);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListEnd(ZrcDisplayList* list) {
  if (list == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  list->impl->end();
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListClear(ZrcDisplayList* list) {
  if (list == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  list->impl->clearRecorded();
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListDraw(const ZrcDisplayList* list,
                                     const ZrcDebugDraw* dd) {
  if (list == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  zrc::HostDebugDraw draw(*dd);
  list->impl->draw(&draw);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDisplayListVertexCount(const ZrcDisplayList* list,
                                            int32_t* out) {
  if (list == nullptr || out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = list->impl->recorded();
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Drawing what Recast built
//===----------------------------------------------------------------------===//

ZRC_API ZrcResult zrcDebugDrawTriMesh(const ZrcDebugDraw* dd,
                                      const ZrcTriMesh* mesh,
                                      const float* normals,
                                      const uint8_t* flags, float tex_scale) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult geometry = zrc::ValidateTriMesh(*mesh);
  if (geometry != ZRC_OK) return geometry;
  // Upstream returns without drawing when normals is null, which is a silent
  // no-op for a caller who simply has not computed them yet.
  if (normals == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(tex_scale)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!AllFinite(normals, mesh->tri_count * 3)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawTriMesh(&draw, mesh->verts, mesh->vert_count, mesh->tris, normals,
                     mesh->tri_count, flags, tex_scale);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawTriMeshSlope(const ZrcDebugDraw* dd,
                                           const ZrcTriMesh* mesh,
                                           const float* normals,
                                           float walkable_slope_angle,
                                           float tex_scale) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult geometry = zrc::ValidateTriMesh(*mesh);
  if (geometry != ZRC_OK) return geometry;
  if (normals == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(tex_scale)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(walkable_slope_angle) || walkable_slope_angle < 0.f ||
      walkable_slope_angle > 90.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!AllFinite(normals, mesh->tri_count * 3)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawTriMeshSlope(&draw, mesh->verts, mesh->vert_count, mesh->tris,
                          normals, mesh->tri_count, walkable_slope_angle,
                          tex_scale);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawHeightfieldSolid(const ZrcDebugDraw* dd,
                                               const ZrcHeightfield* hf) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (hf == nullptr || hf->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawHeightfieldSolid(&draw, *hf->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawHeightfieldWalkable(const ZrcDebugDraw* dd,
                                                  const ZrcHeightfield* hf) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (hf == nullptr || hf->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawHeightfieldWalkable(&draw, *hf->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCompactHeightfieldSolid(
    const ZrcDebugDraw* dd, const ZrcCompactHeightfield* chf) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (chf == nullptr || chf->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCompactHeightfieldSolid(&draw, *chf->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCompactHeightfieldRegions(
    const ZrcDebugDraw* dd, const ZrcCompactHeightfield* chf) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (chf == nullptr || chf->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCompactHeightfieldRegions(&draw, *chf->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawCompactHeightfieldDistance(
    const ZrcDebugDraw* dd, const ZrcCompactHeightfield* chf) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (chf == nullptr || chf->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  // Upstream dereferences chf.dist unconditionally, and it is null until
  // zrcCompactHeightfieldBuildDistanceField has run.
  if (chf->impl->dist == nullptr) return ZRC_ERR_EMPTY_RESULT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawCompactHeightfieldDistance(&draw, *chf->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawHeightfieldLayer(
    const ZrcDebugDraw* dd, const ZrcHeightfieldLayerSet* layers,
    int32_t index) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (index < 0 || index >= layers->impl->nlayers) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawHeightfieldLayer(&draw, layers->impl->layers[index], index);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawHeightfieldLayers(
    const ZrcDebugDraw* dd, const ZrcHeightfieldLayerSet* layers) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawHeightfieldLayers(&draw, *layers->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawRegionConnections(const ZrcDebugDraw* dd,
                                                const ZrcContourSet* cset,
                                                float alpha) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (cset == nullptr || cset->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(alpha) || alpha < 0.f || alpha > 1.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawRegionConnections(&draw, *cset->impl, alpha);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawRawContours(const ZrcDebugDraw* dd,
                                          const ZrcContourSet* cset,
                                          float alpha) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (cset == nullptr || cset->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(alpha) || alpha < 0.f || alpha > 1.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawRawContours(&draw, *cset->impl, alpha);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawContours(const ZrcDebugDraw* dd,
                                       const ZrcContourSet* cset, float alpha) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (cset == nullptr || cset->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(alpha) || alpha < 0.f || alpha > 1.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawContours(&draw, *cset->impl, alpha);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawPolyMesh(const ZrcDebugDraw* dd,
                                       const ZrcPolyMesh* mesh) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawPolyMesh(&draw, *mesh->poly);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawPolyMeshDetail(const ZrcDebugDraw* dd,
                                             const ZrcPolyMesh* mesh) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawPolyMeshDetail(&draw, *mesh->detail);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Drawing what Detour loaded
//===----------------------------------------------------------------------===//

ZRC_API ZrcResult zrcDebugDrawNavMesh(const ZrcDebugDraw* dd,
                                      const ZrcNavMesh* mesh, uint8_t flags) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMesh(&draw, *mesh->impl, flags);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawNavMeshWithClosedList(
    const ZrcDebugDraw* dd, const ZrcNavMesh* mesh,
    const ZrcNavMeshQuery* query, uint8_t flags) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMeshWithClosedList(&draw, *mesh->impl, *query->impl, flags);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawNavMeshNodes(const ZrcDebugDraw* dd,
                                           const ZrcNavMeshQuery* query) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (query == nullptr || query->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMeshNodes(&draw, *query->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawNavMeshBVTree(const ZrcDebugDraw* dd,
                                            const ZrcNavMesh* mesh) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMeshBVTree(&draw, *mesh->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawNavMeshPortals(const ZrcDebugDraw* dd,
                                             const ZrcNavMesh* mesh) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMeshPortals(&draw, *mesh->impl);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawNavMeshPolysWithFlags(const ZrcDebugDraw* dd,
                                                    const ZrcNavMesh* mesh,
                                                    uint16_t poly_flags,
                                                    uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMeshPolysWithFlags(&draw, *mesh->impl, poly_flags, col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawNavMeshPoly(const ZrcDebugDraw* dd,
                                          const ZrcNavMesh* mesh,
                                          ZrcPolyRef ref, uint32_t col) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawNavMeshPoly(&draw, *mesh->impl, static_cast<dtPolyRef>(ref), col);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawTileCacheLayerAreas(
    const ZrcDebugDraw* dd, const ZrcTileCacheLayer* layer, float cs,
    float ch) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (layer == nullptr || layer->impl == nullptr ||
      layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cs) || !(cs > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(ch) || !(ch > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawTileCacheLayerAreas(&draw, *layer->impl, cs, ch);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawTileCacheLayerRegions(
    const ZrcDebugDraw* dd, const ZrcTileCacheLayer* layer, float cs,
    float ch) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (layer == nullptr || layer->impl == nullptr ||
      layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cs) || !(cs > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(ch) || !(ch > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawTileCacheLayerRegions(&draw, *layer->impl, cs, ch);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawTileCacheContours(
    const ZrcDebugDraw* dd, const ZrcTileCacheContourSet* cset,
    const float* origin, float cs, float ch) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (cset == nullptr || cset->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (origin == nullptr || !zrc::IsFiniteVec3(origin)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cs) || !(cs > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(ch) || !(ch > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawTileCacheContours(&draw, *cset->impl, origin, cs, ch);
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDebugDrawTileCachePolyMesh(
    const ZrcDebugDraw* dd, const ZrcTileCachePolyMesh* mesh,
    const float* origin, float cs, float ch) {
  const ZrcResult check = ValidateDebugDraw(dd);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (origin == nullptr || !zrc::IsFiniteVec3(origin)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cs) || !(cs > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(ch) || !(ch > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostDebugDraw draw(*dd);
  duDebugDrawTileCachePolyMesh(&draw, *mesh->impl, origin, cs, ch);
  return ZRC_OK;
}
