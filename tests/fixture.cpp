//===----------------------------------------------------------------------===//
// zrecast — synthetic test geometry, generated rather than shipped.
//===----------------------------------------------------------------------===//

#include "fixture.h"

namespace {

//===----------------------------------------------------------------------===//
// A tiny mesh builder over fixed storage.
//
// No allocation: the tests assert that the host allocator's counts balance, and
// geometry construction must not appear in that accounting.
//===----------------------------------------------------------------------===//

template <int kMaxVerts, int kMaxTris>
struct Builder {
  float verts[kMaxVerts * 3];
  int tris[kMaxTris * 3];
  int vert_count;
  int tri_count;

  void reset() {
    vert_count = 0;
    tri_count = 0;
  }

  int vertex(float x, float y, float z) {
    const int index = vert_count++;
    verts[index * 3 + 0] = x;
    verts[index * 3 + 1] = y;
    verts[index * 3 + 2] = z;
    return index;
  }

  void triangle(int a, int b, int c) {
    const int index = tri_count++;
    tris[index * 3 + 0] = a;
    tris[index * 3 + 1] = b;
    tris[index * 3 + 2] = c;
  }

  /// A quad as two triangles, keeping the corner order the caller chose.
  void quad(int a, int b, int c, int d) {
    triangle(a, b, c);
    triangle(a, c, d);
  }

  void publish(ZrcTriMesh* out) const {
    out->verts = verts;
    out->vert_count = vert_count;
    out->tris = tris;
    out->tri_count = tri_count;
  }
};

/// Adds a horizontal quad at height `y`, wound so its normal points up.
///
/// The winding is not decoration. Recast decides walkability from the triangle
/// normal, and the normal comes from the winding, so a floor wound the other
/// way is a ceiling and rasterises to nothing walkable.
template <typename B>
void AddFloor(B& b, float min_x, float max_x, float min_z, float max_z,
              float y) {
  const int v0 = b.vertex(min_x, y, min_z);
  const int v1 = b.vertex(min_x, y, max_z);
  const int v2 = b.vertex(max_x, y, max_z);
  const int v3 = b.vertex(max_x, y, min_z);
  b.quad(v0, v1, v2, v3);
}

/// Adds an axis-aligned box with outward-facing triangles.
template <typename B>
void AddBox(B& b, float min_x, float max_x, float min_y, float max_y,
            float min_z, float max_z) {
  const int v000 = b.vertex(min_x, min_y, min_z);
  const int v001 = b.vertex(min_x, min_y, max_z);
  const int v011 = b.vertex(min_x, max_y, max_z);
  const int v010 = b.vertex(min_x, max_y, min_z);
  const int v100 = b.vertex(max_x, min_y, min_z);
  const int v101 = b.vertex(max_x, min_y, max_z);
  const int v111 = b.vertex(max_x, max_y, max_z);
  const int v110 = b.vertex(max_x, max_y, min_z);

  b.quad(v010, v011, v111, v110);  // +Y, up
  b.quad(v000, v100, v101, v001);  // -Y, down
  b.quad(v001, v101, v111, v011);  // +Z
  b.quad(v100, v000, v010, v110);  // -Z
  b.quad(v000, v001, v011, v010);  // -X
  b.quad(v101, v100, v110, v111);  // +X
}

typedef Builder<ZRC_FIXTURE_VERT_COUNT, ZRC_FIXTURE_TRI_COUNT> WorldBuilder;
typedef Builder<6, 4> TentBuilder;

WorldBuilder& World() {
  static WorldBuilder builder;
  static bool built = false;
  if (!built) {
    builder.reset();

    const float e = ZRC_FIXTURE_GROUND_EXTENT;
    AddFloor(builder, -e, e, -e, e, 0.0f);

    // The wall: tall enough that its top is not a place the agent can reach,
    // and thin enough that the top survives no erosion at all.
    AddBox(builder, -e, ZRC_FIXTURE_WALL_END_X, 0.0f, 3.0f, -0.3f, 0.3f);

    // A free-standing pillar on the +Z side, so the open half of the map is
    // not a featureless rectangle.
    AddBox(builder, 2.0f, 4.0f, 0.0f, 3.0f, -6.0f, -4.0f);

    // An island with no bridge to anything.
    const float ix = ZRC_FIXTURE_ISLAND_X;
    const float iz = ZRC_FIXTURE_ISLAND_Z;
    const float ie = ZRC_FIXTURE_ISLAND_EXTENT;
    AddFloor(builder, ix - ie, ix + ie, iz - ie, iz + ie, 0.0f);

    built = true;
  }
  return builder;
}

TentBuilder& Tent() {
  static TentBuilder builder;
  static bool built = false;
  if (!built) {
    builder.reset();
    // Apex line along x at y = 3, base edges at z = +/-1: a 71-degree slope on
    // both faces, steeper than any sane agent_max_slope.
    const int a0 = builder.vertex(-5.0f, 3.0f, 0.0f);
    const int a1 = builder.vertex(5.0f, 3.0f, 0.0f);
    const int bn = builder.vertex(-5.0f, 0.0f, -1.0f);
    const int bp = builder.vertex(-5.0f, 0.0f, 1.0f);
    const int cn = builder.vertex(5.0f, 0.0f, -1.0f);
    const int cp = builder.vertex(5.0f, 0.0f, 1.0f);
    builder.quad(bn, cn, a1, a0);  // -Z face
    builder.quad(a0, a1, cp, bp);  // +Z face
    built = true;
  }
  return builder;
}

}  // namespace

extern "C" {

void zrcFixtureTriMesh(ZrcTriMesh* out) {
  if (out == nullptr) return;
  World().publish(out);
}

void zrcFixtureUnwalkableTriMesh(ZrcTriMesh* out) {
  if (out == nullptr) return;
  Tent().publish(out);
}

}  // extern "C"
