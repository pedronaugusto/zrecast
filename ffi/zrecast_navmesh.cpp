//===----------------------------------------------------------------------===//
// zrecast — turning a baked polygon mesh into a Detour navmesh, and moving that
// navmesh in and out of bytes.
//
// This is the seam between the two lifecycles: zrcNavMeshCreate is the last
// step of a cook, zrcNavMeshDeserialize is the first step of a runtime.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

#include "DetourCommon.h"

namespace {

/// The largest polygon count any tile can carry.
///
/// dtNavMesh spends 32 bits on a polygon reference, split between tile index,
/// polygon index and a salt, and refuses to initialise if fewer than 10 bits
/// are left for the salt. One tile leaves all 22 non-salt bits to the polygon
/// index, so this is the loosest ceiling a well-formed tile can meet, and it is
/// the one image validation applies: whether a particular *navmesh* can hold
/// that many is a second question, answered by ReferenceBitsFit against the
/// tile count that navmesh was created for.
const int64_t kMaxPolyCount = (int64_t)1 << 22;

/// Whether a tile count and a per-tile polygon count leave Detour enough salt.
///
/// dtNavMesh::init computes tileBits and polyBits from the next power of two of
/// each, takes the salt from what is left of 32, and fails below 10
/// (DetourNavMesh.cpp:255-261). Recomputing it here turns that into a named
/// argument error at the door instead of a status from inside a half-built
/// object.
bool ReferenceBitsFit(int32_t max_tiles, int32_t max_polys) {
  const unsigned int tile_bits =
      dtIlog2(dtNextPow2(static_cast<unsigned int>(max_tiles)));
  const unsigned int poly_bits =
      dtIlog2(dtNextPow2(static_cast<unsigned int>(max_polys)));
  if (tile_bits + poly_bits > 32u) return false;
  const unsigned int salt_bits = 32u - tile_bits - poly_bits;
  return salt_bits >= 10u;
}

/// Tiles currently resident, counted by inspection.
///
/// dtNavMesh keeps no public count and a mirrored one here could drift from it,
/// so this walks the slots. It is O(maxTiles) pointer tests against a tile that
/// took orders of magnitude longer to bake.
int32_t ResidentTiles(const dtNavMesh& mesh) {
  int32_t n = 0;
  for (int i = 0; i < mesh.getMaxTiles(); ++i) {
    const dtMeshTile* tile = mesh.getTile(i);
    if (tile != nullptr && tile->header != nullptr) ++n;
  }
  return n;
}

/// Resolves a tile reference to a *resident* tile, or null.
///
/// dtNavMesh::getTileByRef checks the slot index and the salt and stops there,
/// so it returns free slots as well as occupied ones — and a free slot has a
/// null header. dtNavMesh::removeTile then dereferences that header without a
/// test of its own (DetourNavMesh.cpp:1253), which makes a reference to an
/// unused slot of a freshly created navmesh a null dereference rather than an
/// error. Every entry point that takes a ZrcTileRef goes through here.
const dtMeshTile* ResidentTileByRef(const dtNavMesh& mesh, ZrcTileRef ref) {
  if (ref == 0) return nullptr;
  const dtMeshTile* tile = mesh.getTileByRef(static_cast<dtTileRef>(ref));
  if (tile == nullptr || tile->header == nullptr) return nullptr;
  return tile;
}

/// Bounds a half-open [first, first + count) range against a tile array's own
/// length.
///
/// The sum is computed in int64_t rather than as two int32_t operands: adding
/// two in-range int32_t values can itself overflow int before the result is
/// ever compared against array_count.
ZrcResult ValidateTileRange(int32_t first, int32_t count, int32_t array_count) {
  if (first < 0 || count < 0) return ZRC_ERR_INVALID_ARGUMENT;
  const int64_t end = static_cast<int64_t>(first) + static_cast<int64_t>(count);
  if (end > static_cast<int64_t>(0x7fffffff)) return ZRC_ERR_INVALID_ARGUMENT;
  if (end > static_cast<int64_t>(array_count)) return ZRC_ERR_INVALID_ARGUMENT;
  return ZRC_OK;
}

int64_t Align4(int64_t x) { return (x + 3) & ~(int64_t)3; }

/// Byte offset of each array inside a tile image, and the total it implies.
struct TileLayout {
  int64_t verts;
  int64_t polys;
  int64_t links;
  int64_t detail_meshes;
  int64_t detail_verts;
  int64_t detail_tris;
  int64_t bvtree;
  int64_t offmesh;
  int64_t total;
};

/// Brings a dtNavMesh up far enough to hold one tile.
///
/// This is `dtNavMesh::init(data, size, flags)` split in two, and the split is
/// what makes the failure recoverable. That single-call form runs
/// `init(&params)` and then `addTile`, and reports both failures the same way —
/// but only the first leaves an object that cannot be destroyed. Doing the two
/// steps here attributes each failure exactly instead of inferring it from a
/// status bit that both can set.
///
/// On failure `*out_destructible` says whether the caller may run ~dtNavMesh.
///
/// It is false in exactly one case: `init(&params)` failing to *allocate*. That
/// function sets m_maxTiles before allocating the m_tiles array the destructor
/// walks, and memsets that array only after a second allocation also succeeds,
/// so an out-of-memory there leaves the destructor either dereferencing null or
/// reading uninitialised tile flags and calling dtFree on the garbage behind
/// them (DetourNavMesh.cpp:211, 232-244). Neither is survivable, and neither is
/// fixable from outside the class; releasing the storage without the destructor
/// leaks at most the first array.
///
/// `init(&params)`'s *other* failure — too few salt bits — happens after both
/// arrays are live and initialised, so it is destructible like any addTile
/// failure. Reporting it as though it were not would leak both arrays instead.
dtStatus InitSingleTile(dtNavMesh* mesh, const dtMeshHeader& header,
                        unsigned char* data, int size,
                        bool* out_destructible) {
  dtNavMeshParams params;
  memset(&params, 0, sizeof(params));
  params.orig[0] = header.bmin[0];
  params.orig[1] = header.bmin[1];
  params.orig[2] = header.bmin[2];
  params.tileWidth = header.bmax[0] - header.bmin[0];
  params.tileHeight = header.bmax[2] - header.bmin[2];
  params.maxTiles = 1;
  params.maxPolys = header.polyCount;

  *out_destructible = false;
  dtStatus status = mesh->init(&params);
  if (dtStatusFailed(status)) {
    // Only an allocation failure can leave the arrays half-built.
    *out_destructible = !dtStatusDetail(status, DT_OUT_OF_MEMORY);
    return status;
  }

  // Past this point both internal arrays are live and initialised, so the
  // destructor is safe whatever addTile reports.
  *out_destructible = true;
  return mesh->addTile(data, size, DT_TILE_FREE_DATA, 0, nullptr);
}

/// Releases a dtNavMesh that failed to initialise, running its destructor only
/// when InitSingleTile says that is safe.
void FreeFailedNavMesh(dtNavMesh* mesh, bool destructible) {
  if (mesh == nullptr) return;
  if (destructible) {
    dtFreeNavMesh(mesh);
  } else {
    dtFree(mesh);
  }
}

/// Locates the single tile of a single-tile mesh.
const dtMeshTile* SoleTile(const dtNavMesh& mesh) {
  const dtMeshTile* found = nullptr;
  for (int i = 0; i < mesh.getMaxTiles(); ++i) {
    const dtMeshTile* tile = mesh.getTile(i);
    if (tile == nullptr || tile->header == nullptr) continue;
    if (found != nullptr) return nullptr;  // more than one: not supported here
    found = tile;
  }
  return found;
}

/// Shared with the bake and the staged pipeline. A `size` of zero allocates
/// nothing, so the off-mesh arrays below need not special-case the empty case.
using zrc::TempBuffer;

/// Rejects tile authoring Detour would misread: an out-of-range count, a
/// non-finite or out-of-domain endpoint, a radius that is not a positive
/// finite number, or an area id outside range.
ZrcResult ValidateTileAuthoring(const ZrcTileAuthoring& authoring) {
  if (authoring.connection_count < 0 ||
      authoring.connection_count > zrc::kMaxOffMeshConnections) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (authoring.connection_count > 0 && authoring.connections == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int32_t i = 0; i < authoring.connection_count; ++i) {
    const ZrcResult valid =
        zrc::ValidateOffMeshConnection(authoring.connections[i]);
    if (valid != ZRC_OK) return valid;
  }
  return ZRC_OK;
}

}  // namespace

namespace zrc {

bool TileLayoutOf(const dtMeshHeader& header, TileLayout* out) {
  // Every count must already be non-negative; the caller checks that first, but
  // this is the function the arithmetic depends on, so it checks too.
  const int64_t counts[] = {
      header.vertCount,       header.polyCount,      header.maxLinkCount,
      header.detailMeshCount, header.detailVertCount, header.detailTriCount,
      header.bvNodeCount,     header.offMeshConCount,
  };
  for (size_t i = 0; i < sizeof(counts) / sizeof(counts[0]); ++i) {
    if (counts[i] < 0) return false;
  }

  // Eight terms, each under 2^37 for any count that passed the header checks,
  // so int64 cannot overflow here.
  int64_t at = Align4(static_cast<int64_t>(sizeof(dtMeshHeader)));
  out->verts = at;
  at += Align4(static_cast<int64_t>(sizeof(float)) * 3 * header.vertCount);
  out->polys = at;
  at += Align4(static_cast<int64_t>(sizeof(dtPoly)) * header.polyCount);
  out->links = at;
  at += Align4(static_cast<int64_t>(sizeof(dtLink)) * header.maxLinkCount);
  out->detail_meshes = at;
  at += Align4(static_cast<int64_t>(sizeof(dtPolyDetail)) *
               header.detailMeshCount);
  out->detail_verts = at;
  at += Align4(static_cast<int64_t>(sizeof(float)) * 3 * header.detailVertCount);
  out->detail_tris = at;
  at += Align4(static_cast<int64_t>(4) * header.detailTriCount);
  out->bvtree = at;
  at += Align4(static_cast<int64_t>(sizeof(dtBVNode)) * header.bvNodeCount);
  out->offmesh = at;
  at += Align4(static_cast<int64_t>(sizeof(dtOffMeshConnection)) *
               header.offMeshConCount);
  out->total = at;
  return true;
}

/// Reads one POD out of the image without assuming the caller's buffer is
/// aligned for it.
template <typename T>
T ReadAt(const unsigned char* data, int64_t offset, int index) {
  T value;
  memcpy(&value, data + offset + static_cast<int64_t>(sizeof(T)) * index,
         sizeof(T));
  return value;
}

/// Neutralises the one bounding-volume node upstream reserves and never fills.
///
/// dtCreateNavMeshData sets bvNodeCount to polyCount * 2
/// (DetourNavMeshBuilder.cpp:507), while the tree createBVTree builds over n
/// polygons occupies exactly 2n - 1 nodes: subdivide emits one node per call
/// and either makes a leaf or splits into two children, so there are no unary
/// nodes and the count is fixed. The last reserved node is therefore always
/// spare, and dtCreateNavMeshData leaves it as the zeroes it memset the whole
/// buffer with.
///
/// queryPolygonsInTile ends its traversal at `&bvTree[bvNodeCount]` rather
/// than at the tree (DetourNavMeshQuery.cpp:744), so it reaches that node.
/// A zeroed node has `i == 0`, which reads as a leaf naming polygon 0, and
/// bounds of {0,0,0}..{0,0,0}, which dtOverlapQuantBounds reports as
/// overlapping any query box whose quantised minimum is 0 on all three axes —
/// that is, any query reaching the tile's own minimum corner, including every
/// query that falls outside the tile and is clamped to it. Polygon 0 then
/// comes back as a candidate for a box it is nowhere near. Measured, not
/// inferred: findNearestPoly at the fixture's minimum corner with 0.05 m
/// half-extents returned a polygon 9 m away.
///
/// Marking the node internal is what makes it unreachable as a result: the
/// traversal emits a polygon only for a node with a non-negative `i`, and
/// steps past this one either way. -1 is inside the escape range image
/// validation already allows.
void SealBvSentinel(unsigned char* data, const dtMeshHeader& header,
                    const TileLayout& layout) {
  if (header.bvNodeCount <= 0) return;
  const int last = header.bvNodeCount - 1;
  dtBVNode node = ReadAt<dtBVNode>(data, layout.bvtree, last);
  if (node.i < 0) return;
  node.i = -1;
  memcpy(data + layout.bvtree + static_cast<int64_t>(sizeof(dtBVNode)) * last,
         &node, sizeof(node));
}

/// Validates every index inside the tile that Detour will dereference.
///
/// The header checks alone are not enough, and the gap is not theoretical: a
/// `dtPoly::vertCount` of 200 in an otherwise untouched image passes every
/// header check, loads without complaint, and then overruns a fixed
/// `float verts[DT_VERTS_PER_POLYGON*3]` stack array in four separate Detour
/// functions — `getPolyHeight` (DetourNavMesh.cpp:687), `closestPointOnPoly`,
/// `moveAlongSurface` and `raycast` (DetourNavMeshQuery.cpp:549, 2091, 2490).
/// The overrun is with vertex coordinates taken from the image, past the saved
/// return address.
///
/// So this walks the arrays and bounds every index Detour trusts. It is
/// O(polyCount + detailTriCount + bvNodeCount) memcpys at load time, against a
/// mesh that took orders of magnitude longer to bake.
ZrcResult ValidateTileInterior(const unsigned char* data,
                               const dtMeshHeader& header,
                               const TileLayout& layout,
                               const TileAdmission& admission) {
  const int poly_count = header.polyCount;
  const int vert_count = header.vertCount;
  const int detail_vert_count = header.detailVertCount;
  const int detail_tri_count = header.detailTriCount;

  // Coordinates first, because a single NaN among them is enough.
  //
  // closestPointOnDetailEdges tracks the nearest edge with `if (d < dmin)` and
  // leaves its pmin/pmax pointers null when nothing ever wins, then calls
  // dtVlerp on them unconditionally (DetailNavMesh.cpp:673). Every comparison
  // against a NaN distance is false, so one NaN coordinate anywhere in a
  // polygon's detail edges is a null dereference on the next closest-point
  // query. Infinities produce NaN the same way, through inf - inf.
  for (int i = 0; i < vert_count * 3; ++i) {
    const float v = ReadAt<float>(data, layout.verts, i);
    if (!IsFinite(v) || v < -kMaxCoordinate || v > kMaxCoordinate) {
      return ZRC_ERR_BAD_FORMAT;
    }
  }
  for (int i = 0; i < detail_vert_count * 3; ++i) {
    const float v = ReadAt<float>(data, layout.detail_verts, i);
    if (!IsFinite(v) || v < -kMaxCoordinate || v > kMaxCoordinate) {
      return ZRC_ERR_BAD_FORMAT;
    }
  }

  for (int i = 0; i < poly_count; ++i) {
    const dtPoly poly = ReadAt<dtPoly>(data, layout.polys, i);
    const bool is_offmesh = i >= header.offMeshBase;

    // A polygon below offMeshBase must be ground, one at or above it must be
    // an off-mesh connection endpoint, and no other value of getType() — a
    // 2-bit field with two undefined values — is acceptable.
    //
    // dtNavMesh::getOffMeshConnectionByRef computes
    // `idx = ip - tile->header->offMeshBase` with no check that
    // `ip >= offMeshBase` (DetourNavMesh.cpp:1521): unsigned, so a ground
    // polygon tagged off-mesh underflows idx to a huge value and the function
    // returns `&tile->offMeshCons[idx]`, a wild pointer. The only guard
    // upstream has is dtAssert, which compiles to `(void)sizeof(x)` under
    // NDEBUG (DetourAssert.h:25-29) — nothing at all in release.
    const unsigned char type = poly.getType();
    if (is_offmesh) {
      if (type != DT_POLYTYPE_OFFMESH_CONNECTION) return ZRC_ERR_BAD_FORMAT;
    } else if (type != DT_POLYTYPE_GROUND) {
      return ZRC_ERR_BAD_FORMAT;
    }

    // The bound that matters. Detour copies vertCount vertices into a
    // DT_VERTS_PER_POLYGON-sized stack array with no check of its own. An
    // off-mesh polygon is always the two vertices dtCreateNavMeshData
    // appended for it (DetourNavMeshBuilder.cpp:584-586); a ground polygon
    // needs at least a triangle.
    if (is_offmesh) {
      if (poly.vertCount != 2) return ZRC_ERR_BAD_FORMAT;
    } else if (poly.vertCount < 3 || poly.vertCount > DT_VERTS_PER_POLYGON) {
      return ZRC_ERR_BAD_FORMAT;
    }

    for (int k = 0; k < poly.vertCount; ++k) {
      if (poly.verts[k] >= vert_count) return ZRC_ERR_BAD_FORMAT;
    }

    // Neighbour entries, which are how a polygon index gets *laundered* into
    // something nothing bounds again.
    //
    // connectIntLinks turns `neis[j]` into a link whose ref is
    // `base | (neis[j] - 1)` (DetourNavMesh.cpp:549) with no range test. Every
    // traversal query then follows that link with
    // `getTileAndPolyByRefUnsafe`, which is unsafe exactly as named: it
    // indexes `tile->polys` with the decoded value and checks nothing. So a
    // neighbour index past polyCount is an out-of-bounds dtPoly read whose
    // vertCount then drives the vertex-gathering loops — the same stack
    // overrun a doctored vertCount gives, reached the long way round.
    //
    // Reasoning that "Detour validates refs" is what missed this: the safe
    // getTileAndPolyByRef is used on the *caller's* references, and the unsafe
    // one on references the tile produced for itself.
    for (int k = 0; k < poly.vertCount; ++k) {
      const unsigned short nei = poly.neis[k];
      if (nei == 0) continue;  // hard edge
      if ((nei & DT_EXT_LINK) != 0) {
        // A portal edge to an adjacent tile. A caller that has no neighbouring
        // tiles refuses these outright; one that does bounds the side code.
        //
        // dtCreateNavMeshData writes exactly four of the eight side values —
        // 4, 2, 0 and 6 for x-, z+, x+ and z-
        // (DetourNavMeshBuilder.cpp:534-540) — and dtNavMesh::connectExtLinks
        // matches on `DT_EXT_LINK | side` for those same four
        // (DetourNavMesh.cpp:312). Any other value describes an edge that can
        // never link to anything, which is not something a real bake produces.
        if (!admission.allow_portals) return ZRC_ERR_BAD_FORMAT;
        const unsigned short side = nei & 0xff;
        if (side != 0 && side != 2 && side != 4 && side != 6) {
          return ZRC_ERR_BAD_FORMAT;
        }
        continue;
      }
      // Internal neighbour: stored one-based, so this bounds `nei - 1`.
      if (nei > poly_count) return ZRC_ERR_BAD_FORMAT;
    }

    // Off-mesh polygons have no detail sub-mesh: detailMeshCount covers
    // ground polygons only, and this array has exactly that many entries (see
    // ValidateTileImageHeader).
    if (is_offmesh) continue;

    // The detail sub-mesh for this polygon addresses two further arrays.
    const dtPolyDetail pd = ReadAt<dtPolyDetail>(data, layout.detail_meshes, i);
    const int64_t vert_end =
        static_cast<int64_t>(pd.vertBase) + pd.vertCount;
    const int64_t tri_end = static_cast<int64_t>(pd.triBase) + pd.triCount;
    if (vert_end > detail_vert_count) return ZRC_ERR_BAD_FORMAT;
    if (tri_end > detail_tri_count) return ZRC_ERR_BAD_FORMAT;
    // A sub-mesh with no triangles is the other way closestPointOnDetailEdges
    // reaches dtVlerp with null pointers: its loop simply never runs.
    // rcBuildPolyMeshDetail always emits at least one triangle per polygon, so
    // this rejects nothing a real bake produces.
    if (pd.triCount < 1) return ZRC_ERR_BAD_FORMAT;

    // Each detail triangle corner is either one of the polygon's own vertices
    // or an index into detailVerts biased by vertBase. Only the second form can
    // leave the array.
    // The fourth byte of a detail triangle packs three 2-bit edge flags. At
    // least one edge somewhere in the sub-mesh has to be marked as a boundary,
    // for the same null-pointer reason as above: closestPointOnDetailEdges
    // instantiated with onlyBoundary = true skips every triangle that has no
    // boundary bit at all, so a sub-mesh with none leaves pmin/pmax null and
    // dtVlerp dereferences them. A polygon's detail mesh always has boundary
    // edges — they are its outline — so this rejects nothing real.
    const unsigned char kAnyBoundaryEdge =
        (DT_DETAIL_EDGE_BOUNDARY << 0) | (DT_DETAIL_EDGE_BOUNDARY << 2) |
        (DT_DETAIL_EDGE_BOUNDARY << 4);
    bool has_boundary_edge = false;

    for (int j = 0; j < pd.triCount; ++j) {
      const int64_t tri_at =
          layout.detail_tris + (static_cast<int64_t>(pd.triBase) + j) * 4;
      unsigned char corner[4];
      memcpy(corner, data + tri_at, sizeof(corner));
      for (int k = 0; k < 3; ++k) {
        if (corner[k] < poly.vertCount) continue;
        const int64_t detail_index =
            static_cast<int64_t>(pd.vertBase) + corner[k] - poly.vertCount;
        if (detail_index < 0 || detail_index >= detail_vert_count) {
          return ZRC_ERR_BAD_FORMAT;
        }
      }
      if ((corner[3] & kAnyBoundaryEdge) != 0) has_boundary_edge = true;
    }
    if (!has_boundary_edge) return ZRC_ERR_BAD_FORMAT;
  }

  // The BV tree. queryPolygonsInTile indexes tile->polys by a leaf's `i`
  // directly (DetourNavMeshQuery.cpp:776), and advances by `-i` for an internal
  // node, so both directions need a bound. The bound is offMeshBase rather
  // than polyCount: createBVTree never emits a leaf outside the ground range
  // (DetourNavMeshBuilder.cpp:195-200), and the BV branch of
  // queryPolygonsInTile — unlike its linear-scan branch — does not filter
  // off-mesh polygons out (DetourNavMesh.cpp:808-889), so a doctored leaf
  // past offMeshBase could make findNearestPolyInTile resolve an off-mesh
  // connection's own polygon as its landing polygon.
  for (int i = 0; i < header.bvNodeCount; ++i) {
    const dtBVNode node = ReadAt<dtBVNode>(data, layout.bvtree, i);
    if (node.i >= 0) {
      if (node.i >= header.offMeshBase) return ZRC_ERR_BAD_FORMAT;
    } else {
      // An escape jump of zero would not terminate; one past the end is fine,
      // because the traversal simply runs off and stops.
      if (node.i < -header.bvNodeCount) return ZRC_ERR_BAD_FORMAT;
    }
    for (int k = 0; k < 3; ++k) {
      if (node.bmin[k] > node.bmax[k]) return ZRC_ERR_BAD_FORMAT;
    }
  }

  // Off-mesh connections. dtNavMesh::baseOffMeshLinks does
  // `dtPoly* poly = &tile->polys[con->poly];` with no bound at all
  // (DetourNavMesh.cpp:571), then writes through
  // `tile->verts[poly->verts[0]*3]` — an out-of-bounds write primitive driven
  // straight from image bytes when con->poly is wrong. connectExtOffMeshLinks
  // repeats the same unguarded index (DetourNavMesh.cpp:468, 485-486).
  for (int i = 0; i < header.offMeshConCount; ++i) {
    const dtOffMeshConnection con =
        ReadAt<dtOffMeshConnection>(data, layout.offmesh, i);

    // The sharpest bound of the three, and it is an equality rather than a
    // range. dtCreateNavMeshData walks the stored connections and the off-mesh
    // polygons together, writing `con->poly = offMeshPolyBase + n` from the
    // same counter that indexes the connection array
    // (DetourNavMeshBuilder.cpp:660-662), so connection i owns polygon
    // offMeshBase + i exactly. Checking that covers the bound, the ordering,
    // and two connections claiming one polygon, in one comparison.
    if (static_cast<int>(con.poly) != header.offMeshBase + i) {
      return ZRC_ERR_BAD_FORMAT;
    }

    for (int k = 0; k < 6; ++k) {
      if (!IsFinite(con.pos[k]) || con.pos[k] < -kMaxCoordinate ||
          con.pos[k] > kMaxCoordinate) {
        return ZRC_ERR_BAD_FORMAT;
      }
    }
    if (!IsFinite(con.rad) || con.rad < 0.f) return ZRC_ERR_BAD_FORMAT;

    // classifyOffMeshPoint produces exactly these nine values
    // (DetourNavMeshBuilder.cpp:280-292). Any other side is inert rather than
    // unsafe, but no genuine image carries one.
    switch (con.side) {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 255:
        break;
      default:
        return ZRC_ERR_BAD_FORMAT;
    }

    if ((con.flags & ~static_cast<unsigned char>(DT_OFFMESH_CON_BIDIR)) != 0) {
      return ZRC_ERR_BAD_FORMAT;
    }
  }

  return ZRC_OK;
}

TileAdmission SoleTileAdmission() {
  TileAdmission admission;
  admission.max_tile_x = 0;
  admission.max_tile_z = 0;
  admission.max_tile_layer = 0;
  admission.allow_portals = false;
  return admission;
}

TileAdmission AnyTileAdmission() {
  TileAdmission admission;
  admission.max_tile_x = ZRC_MAX_TILE_COORD;
  admission.max_tile_z = ZRC_MAX_TILE_COORD;
  admission.max_tile_layer = ZRC_MAX_TILE_LAYER;
  admission.allow_portals = true;
  return admission;
}

ZrcResult ValidateTileImageHeader(const void* data, size_t size,
                                  const TileAdmission& admission,
                                  dtMeshHeader* out_header,
                                  TileLayout* out_layout) {
  if (data == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  // Detour hands the buffer to addTile as an `int`, and every count inside is
  // an int too.
  if (size > (size_t)0x7fffffff) return ZRC_ERR_BAD_FORMAT;
  if (size < sizeof(dtMeshHeader)) return ZRC_ERR_BAD_FORMAT;

  // Copy the header out rather than casting the caller's pointer to
  // dtMeshHeader*: the caller's buffer may be a byte slice with no particular
  // alignment, and a misaligned struct read is undefined behaviour on the way
  // to being a fault on some targets.
  dtMeshHeader header;
  memcpy(&header, data, sizeof(header));

  if (header.magic != DT_NAVMESH_MAGIC) return ZRC_ERR_BAD_FORMAT;
  if (header.version != DT_NAVMESH_VERSION) return ZRC_ERR_UNSUPPORTED_VERSION;

  // A tile with no polygons cannot be queried, and Detour's reference-bit
  // arithmetic assumes at least one.
  if (header.polyCount < 1 || header.polyCount > kMaxPolyCount) {
    return ZRC_ERR_BAD_FORMAT;
  }
  // Fewer than three vertices is not a polygon.
  if (header.vertCount < 3) return ZRC_ERR_BAD_FORMAT;
  // addTile writes links[maxLinkCount - 1] unconditionally while building the
  // free list. A zero here is a write one element before the array.
  if (header.maxLinkCount < 1) return ZRC_ERR_BAD_FORMAT;
  if (header.detailVertCount < 0 || header.detailTriCount < 0) {
    return ZRC_ERR_BAD_FORMAT;
  }
  // Off-mesh connections are polygons appended after the ground ones, so this
  // count feeds the same layout arithmetic polyCount does and is bounded the
  // same way.
  if (header.offMeshConCount < 0 || header.offMeshConCount > kMaxPolyCount) {
    return ZRC_ERR_BAD_FORMAT;
  }
  // dtCreateNavMeshData sets offMeshBase to the ground polygon count and
  // polyCount to offMeshBase plus the stored connection count
  // (DetourNavMeshBuilder.cpp:493, 502, 506): offMeshBase is where the
  // off-mesh polygons begin, and the two spans together must exactly cover
  // polyCount. With no connections this reduces to offMeshBase == polyCount,
  // the invariant this check replaces.
  //
  // Bounded against polyCount before the addition, not merely against zero: an
  // offMeshBase of INT_MAX would make the sum itself signed overflow, which is
  // undefined behaviour in the check meant to catch it.
  if (header.offMeshBase < 0 || header.offMeshBase > header.polyCount ||
      header.offMeshBase + header.offMeshConCount != header.polyCount) {
    return ZRC_ERR_BAD_FORMAT;
  }
  // Detour indexes detailMeshes by polygon index with no bound of its own. The
  // builder writes detailMeshCount = params->polyCount
  // (DetourNavMeshBuilder.cpp:498) — the *ground* polygon count, since
  // off-mesh polygons carry no detail sub-mesh.
  if (header.detailMeshCount != header.offMeshBase) return ZRC_ERR_BAD_FORMAT;
  // The BV tree is optional, but when present Detour expects two nodes per
  // ground polygon: createBVTree loops to params->polyCount and the tree it
  // builds is sized params->polyCount*2, both the ground count rather than the
  // total including off-mesh polygons (DetourNavMeshBuilder.cpp:200, and the
  // bvTreeSize computation above it).
  if (header.bvNodeCount != 0 &&
      header.bvNodeCount != header.offMeshBase * 2) {
    return ZRC_ERR_BAD_FORMAT;
  }

  // The tile's own place in the grid, bounded from both ends.
  //
  // The upper bound is the caller's — a lone tile pins all three to zero, a
  // tile of a grid may sit anywhere inside it. The lower bound is Detour's:
  // getNeighbourTilesAt walks to a neighbour with a bare `nx--` or `ny--`
  // (DetourNavMesh.cpp:1078-1089), so a coordinate at INT_MIN is signed
  // overflow. ZRC_MAX_TILE_COORD keeps every such step far inside range.
  if (header.x < 0 || header.x > admission.max_tile_x) return ZRC_ERR_BAD_FORMAT;
  if (header.y < 0 || header.y > admission.max_tile_z) return ZRC_ERR_BAD_FORMAT;
  if (header.layer < 0 || header.layer > admission.max_tile_layer) {
    return ZRC_ERR_BAD_FORMAT;
  }

  if (!IsFinite(header.walkableHeight) || !IsFinite(header.walkableRadius) ||
      !IsFinite(header.walkableClimb) || !IsFinite(header.bvQuantFactor)) {
    return ZRC_ERR_BAD_FORMAT;
  }
  if (!IsFiniteVec3(header.bmin) || !IsFiniteVec3(header.bmax)) {
    return ZRC_ERR_BAD_FORMAT;
  }
  for (int i = 0; i < 3; ++i) {
    if (header.bmin[i] > header.bmax[i]) return ZRC_ERR_BAD_FORMAT;
    if (header.bmin[i] < -kMaxCoordinate || header.bmax[i] > kMaxCoordinate) {
      return ZRC_ERR_BAD_FORMAT;
    }
  }

  // The BV quantisation factor is not merely a number that must be finite.
  // queryPolygonsInTile computes `(unsigned short)(qfac * extent)` on every
  // query (DetourNavMeshQuery.cpp:759-764), and a float-to-integer conversion
  // whose value does not fit is undefined behaviour. The condition below is
  // exactly the one that keeps all six of those conversions in range.
  if (header.bvNodeCount != 0) {
    if (!(header.bvQuantFactor > 0.f)) return ZRC_ERR_BAD_FORMAT;
    for (int i = 0; i < 3; ++i) {
      const double extent = static_cast<double>(header.bmax[i]) -
                            static_cast<double>(header.bmin[i]);
      if (static_cast<double>(header.bvQuantFactor) * extent + 1.0 >= 65536.0) {
        return ZRC_ERR_BAD_FORMAT;
      }
    }
  }

  // The last bounds check before the interior. dtNavMesh::addTile derives eight
  // array pointers from the counts above and never compares them against the
  // length it was given, so anything but an exact match is an out-of-bounds
  // access waiting to happen — a short buffer reads past the end, and a long
  // one means the header is not describing these bytes.
  TileLayout layout;
  if (!TileLayoutOf(header, &layout)) return ZRC_ERR_BAD_FORMAT;
  if (layout.total != static_cast<int64_t>(size)) return ZRC_ERR_BAD_FORMAT;

  *out_header = header;
  *out_layout = layout;
  return ZRC_OK;
}

ZrcResult ValidateNavMeshImage(const void* data, size_t size,
                               const TileAdmission& admission) {
  dtMeshHeader header;
  TileLayout layout;
  const ZrcResult header_valid =
      ValidateTileImageHeader(data, size, admission, &header, &layout);
  if (header_valid != ZRC_OK) return header_valid;

  // With the arrays known to be inside the buffer, the indices inside them can
  // be checked. This is the part that makes a doctored image safe rather than
  // merely well-sized.
  return ValidateTileInterior(static_cast<const unsigned char*>(data), header,
                              layout, admission);
}

ZrcResult BuildTileData(const ZrcPolyMesh& mesh, int32_t tile_x, int32_t tile_z,
                        int32_t tile_layer, const ZrcTileAuthoring* authoring,
                        unsigned char** out_data, int* out_size) {
  *out_data = nullptr;
  *out_size = 0;
  if (mesh.poly == nullptr || mesh.detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Recast carries no agent dimensions and dtNavMeshCreateParams demands
  // three. A bake fills them from its own configuration; a mesh assembled
  // stage by stage receives them from zrcPolyMeshSetAgentDims, and one that
  // never did would describe an agent of size zero — a tile whose polygons
  // every agent fits through.
  if (!mesh.has_agent_dims) return ZRC_ERR_INVALID_ARGUMENT;
  if (authoring != nullptr) {
    const ZrcResult authoring_valid = ValidateTileAuthoring(*authoring);
    if (authoring_valid != ZRC_OK) return authoring_valid;
  }

  const rcPolyMesh& poly = *mesh.poly;
  const rcPolyMeshDetail& detail = *mesh.detail;

  if (poly.npolys < 1 || poly.nverts < 3) return ZRC_ERR_EMPTY_RESULT;
  // The same reference-bit ceiling zrcNavMeshValidate applies to a loaded
  // image. Without it dtNavMesh::init would be reached with maxPolys past the
  // salt-bit boundary, which is the one init() failure that is not an
  // allocation failure — reachable only through this entry point.
  if (poly.npolys > kMaxPolyCount) return ZRC_ERR_INVALID_ARGUMENT;
  // dtCreateNavMeshData packs vertex indices into 16 bits and reserves 0xffff.
  if (poly.nverts >= 0xffff) return ZRC_ERR_INVALID_ARGUMENT;
  if (poly.nvp > ZRC_VERTS_PER_POLYGON) return ZRC_ERR_INVALID_ARGUMENT;

  dtNavMeshCreateParams params;
  memset(&params, 0, sizeof(params));
  params.verts = poly.verts;
  params.vertCount = poly.nverts;
  params.polys = poly.polys;
  params.polyAreas = poly.areas;
  params.polyFlags = poly.flags;
  params.polyCount = poly.npolys;
  params.nvp = poly.nvp;

  // An empty detail half is how a host says "no detail mesh": Detour then
  // derives each polygon's detail from its own corners. Passing the empty
  // arrays instead would describe a detail mesh covering no polygon while the
  // tile claims many, which dtCreateNavMeshData lays out and every later
  // height query reads past.
  const bool has_detail = detail.meshes != nullptr && detail.nmeshes > 0;
  if (has_detail && detail.nmeshes != poly.npolys) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  params.detailMeshes = has_detail ? detail.meshes : nullptr;
  params.detailVerts = has_detail ? detail.verts : nullptr;
  params.detailVertsCount = has_detail ? detail.nverts : 0;
  params.detailTris = has_detail ? detail.tris : nullptr;
  params.detailTriCount = has_detail ? detail.ntris : 0;

  // Off-mesh connections, unpacked from the caller's array of structs into
  // the six parallel arrays dtNavMeshCreateParams wants. The buffers are
  // scratch — dtCreateNavMeshData copies out of them before returning — and
  // TempBuffer's destructor frees them on every exit path below, including
  // the allocation and dtCreateNavMeshData failures further down. No
  // authoring, or authoring with zero connections, leaves every one of these
  // fields at the zeroed "none" dtNavMeshCreateParams expects.
  const int32_t con_count =
      authoring != nullptr ? authoring->connection_count : 0;
  TempBuffer con_verts_buf(sizeof(float) * 6 * static_cast<size_t>(con_count));
  TempBuffer con_rad_buf(sizeof(float) * static_cast<size_t>(con_count));
  TempBuffer con_flags_buf(sizeof(unsigned short) *
                           static_cast<size_t>(con_count));
  TempBuffer con_areas_buf(static_cast<size_t>(con_count));
  TempBuffer con_dir_buf(static_cast<size_t>(con_count));
  TempBuffer con_user_id_buf(sizeof(unsigned int) *
                             static_cast<size_t>(con_count));
  if (con_count > 0) {
    if (con_verts_buf.get() == nullptr || con_rad_buf.get() == nullptr ||
        con_flags_buf.get() == nullptr || con_areas_buf.get() == nullptr ||
        con_dir_buf.get() == nullptr || con_user_id_buf.get() == nullptr) {
      return ZRC_ERR_OUT_OF_MEMORY;
    }
    float* con_verts = static_cast<float*>(con_verts_buf.get());
    float* con_rad = static_cast<float*>(con_rad_buf.get());
    unsigned short* con_flags = static_cast<unsigned short*>(con_flags_buf.get());
    unsigned char* con_areas = static_cast<unsigned char*>(con_areas_buf.get());
    unsigned char* con_dir = static_cast<unsigned char*>(con_dir_buf.get());
    unsigned int* con_user_id = static_cast<unsigned int*>(con_user_id_buf.get());
    for (int32_t i = 0; i < con_count; ++i) {
      const ZrcOffMeshConnection& con = authoring->connections[i];
      float* v = &con_verts[i * 6];
      v[0] = con.start[0];
      v[1] = con.start[1];
      v[2] = con.start[2];
      v[3] = con.end[0];
      v[4] = con.end[1];
      v[5] = con.end[2];
      con_rad[i] = con.radius;
      con_flags[i] = con.flags;
      con_areas[i] = static_cast<unsigned char>(con.area);
      con_dir[i] = con.bidirectional != ZRC_FALSE ? 1 : 0;
      con_user_id[i] = con.user_id;
    }
    params.offMeshConVerts = con_verts;
    params.offMeshConRad = con_rad;
    params.offMeshConFlags = con_flags;
    params.offMeshConAreas = con_areas;
    params.offMeshConDir = con_dir;
    params.offMeshConUserID = con_user_id;
  }
  params.offMeshConCount = con_count;
  params.userId = authoring != nullptr ? authoring->user_id : 0u;

  params.walkableHeight = mesh.walkable_height;
  params.walkableRadius = mesh.walkable_radius;
  params.walkableClimb = mesh.walkable_climb;
  memcpy(params.bmin, poly.bmin, sizeof(params.bmin));
  memcpy(params.bmax, poly.bmax, sizeof(params.bmax));
  params.cs = poly.cs;
  params.ch = poly.ch;
  // Recast has already removed the border padding from the polygon mesh's
  // bounds and shifted its vertices to match (RecastContour.cpp:837-850), so
  // these are the tile's own extent rather than the padded one it was baked
  // over.
  params.tileX = tile_x;
  params.tileY = tile_z;
  params.tileLayer = tile_layer;
  // Without the BV tree every query degrades to a linear scan of the tile.
  // NULL authoring, and a zeroed one, both build it.
  params.buildBvTree =
      authoring != nullptr ? authoring->skip_bv_tree == ZRC_FALSE : true;

  unsigned char* data = nullptr;
  int data_size = 0;
  if (!dtCreateNavMeshData(&params, &data, &data_size)) {
    // Upstream returns false both for a rejected parameter and for a failed
    // allocation, without distinguishing them. Every parameter above has
    // already been checked, which leaves allocation as the live possibility.
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  if (data == nullptr || data_size <= 0) return ZRC_ERR_OUT_OF_MEMORY;

  // Before the bytes leave this function, so that every image zrecast produces
  // carries the sealed node and a host handing one straight to Detour gets the
  // same protection as one loading it back through here.
  dtMeshHeader built;
  memcpy(&built, data, sizeof(built));
  TileLayout built_layout;
  if (TileLayoutOf(built, &built_layout)) {
    SealBvSentinel(data, built, built_layout);
  }

  *out_data = data;
  *out_size = data_size;
  return ZRC_OK;
}

}  // namespace zrc

extern "C" {

ZrcResult zrcNavMeshCreate(const ZrcPolyMesh* mesh,
                           const ZrcTileAuthoring* authoring,
                           ZrcNavMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  // A lone tile is the grid position (0, 0, 0) of a one-by-one grid, not a
  // different kind of object. Every tile entry point then works on it, and
  // there is one set of range checks rather than two.
  unsigned char* data = nullptr;
  int data_size = 0;
  const ZrcResult built =
      zrc::BuildTileData(*mesh, 0, 0, 0, authoring, &data, &data_size);
  if (built != ZRC_OK) return built;

  ZrcNavMesh* handle = zrc::New<ZrcNavMesh>();
  if (handle == nullptr) {
    dtFree(data);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->tile_count_x = 1;
  handle->tile_count_z = 1;
  handle->impl = dtAllocNavMesh();
  if (handle->impl == nullptr) {
    dtFree(data);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  // The header dtCreateNavMeshData just wrote is what the tile is sized from.
  dtMeshHeader header;
  memcpy(&header, data, sizeof(header));

  // DT_TILE_FREE_DATA hands `data` to the navmesh, which frees it on destroy.
  bool destructible = false;
  const dtStatus status =
      InitSingleTile(handle->impl, header, data, data_size, &destructible);
  if (dtStatusFailed(status)) {
    // Ownership only transfers on success, so the buffer is still ours.
    dtFree(data);
    FreeFailedNavMesh(handle->impl, destructible);
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }

  *out = handle;
  return ZRC_OK;
}

ZrcResult zrcNavMeshCreateTiled(const ZrcTileGrid* grid, int32_t max_tiles,
                                int32_t max_polys_per_tile, ZrcNavMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (grid == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult grid_valid = zrc::ValidateTileGrid(*grid);
  if (grid_valid != ZRC_OK) return grid_valid;
  if (max_tiles < 1 || max_polys_per_tile < 1) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_polys_per_tile > kMaxPolyCount) return ZRC_ERR_INVALID_ARGUMENT;
  if (!ReferenceBitsFit(max_tiles, max_polys_per_tile)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  dtNavMeshParams params;
  memset(&params, 0, sizeof(params));
  params.orig[0] = grid->origin[0];
  params.orig[1] = grid->origin[1];
  params.orig[2] = grid->origin[2];
  params.tileWidth = grid->tile_world_size;
  params.tileHeight = grid->tile_world_size;
  params.maxTiles = max_tiles;
  params.maxPolys = max_polys_per_tile;

  ZrcNavMesh* handle = zrc::New<ZrcNavMesh>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->tile_count_x = grid->tile_count_x;
  handle->tile_count_z = grid->tile_count_z;
  handle->impl = dtAllocNavMesh();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  const dtStatus status = handle->impl->init(&params);
  if (dtStatusFailed(status)) {
    // The same asymmetry InitSingleTile documents: init() sets m_maxTiles
    // before the arrays the destructor walks exist, so only an allocation
    // failure leaves an object that must not be destructed.
    FreeFailedNavMesh(handle->impl, !dtStatusDetail(status, DT_OUT_OF_MEMORY));
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }

  *out = handle;
  return ZRC_OK;
}

ZrcResult zrcTileDataBuild(const ZrcPolyMesh* mesh, int32_t tile_x,
                           int32_t tile_z, int32_t tile_layer,
                           const ZrcTileAuthoring* authoring, void** out_data,
                           size_t* out_size) {
  if (out_data == nullptr || out_size == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_data = nullptr;
  *out_size = 0;
  if (mesh == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (tile_x < 0 || tile_x > ZRC_MAX_TILE_COORD) return ZRC_ERR_INVALID_ARGUMENT;
  if (tile_z < 0 || tile_z > ZRC_MAX_TILE_COORD) return ZRC_ERR_INVALID_ARGUMENT;
  if (tile_layer < 0 || tile_layer > ZRC_MAX_TILE_LAYER) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  unsigned char* data = nullptr;
  int data_size = 0;
  const ZrcResult built = zrc::BuildTileData(*mesh, tile_x, tile_z, tile_layer,
                                             authoring, &data, &data_size);
  if (built != ZRC_OK) return built;

  *out_data = data;
  *out_size = static_cast<size_t>(data_size);
  return ZRC_OK;
}

ZrcResult zrcNavMeshAddTile(ZrcNavMesh* navmesh, const void* data, size_t size,
                            ZrcTileRef* out_ref) {
  if (out_ref != nullptr) *out_ref = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  zrc::TileAdmission admission;
  admission.max_tile_x = navmesh->tile_count_x - 1;
  admission.max_tile_z = navmesh->tile_count_z - 1;
  admission.max_tile_layer = ZRC_MAX_TILE_LAYER;
  admission.allow_portals = true;
  const ZrcResult valid = zrc::ValidateNavMeshImage(data, size, admission);
  if (valid != ZRC_OK) return valid;

  dtNavMesh& mesh = *navmesh->impl;
  dtMeshHeader header;
  memcpy(&header, data, sizeof(header));

  // Both conditions Detour reports as something else. An occupied slot comes
  // back as DT_ALREADY_OCCUPIED, which has no ZrcResult of its own to map to
  // through the generic path; a full navmesh comes back as
  // DT_FAILURE | DT_OUT_OF_MEMORY (DetourNavMesh.cpp:938), which a host would
  // read as an allocator failure. Both are the caller's to fix, and neither is
  // out of memory.
  if (mesh.getTileAt(header.x, header.y, header.layer) != nullptr) {
    return ZRC_ERR_TILE_OCCUPIED;
  }
  if (ResidentTiles(mesh) >= mesh.getMaxTiles()) return ZRC_ERR_NAVMESH_FULL;

  // Detour keeps the buffer and writes link indices into it, so the caller's
  // bytes are copied rather than borrowed. That also gives the tile the
  // alignment the header struct wants, which a caller's slice may not have.
  unsigned char* copy =
      static_cast<unsigned char*>(zrc::Alloc(size, DT_ALLOC_PERM));
  if (copy == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  memcpy(copy, data, size);

  // Sealed in this package's own copy rather than in the caller's bytes, so an
  // image from another tool gets the same protection without being altered
  // where it lies. See SealBvSentinel.
  {
    dtMeshHeader loaded;
    memcpy(&loaded, copy, sizeof(loaded));
    TileLayout loaded_layout;
    if (zrc::TileLayoutOf(loaded, &loaded_layout)) {
      zrc::SealBvSentinel(copy, loaded, loaded_layout);
    }
  }

  dtTileRef ref = 0;
  const dtStatus status = mesh.addTile(copy, static_cast<int>(size),
                                       DT_TILE_FREE_DATA, 0, &ref);
  if (dtStatusFailed(status)) {
    // Ownership only transfers on success.
    zrc::Free(copy);
    return zrc::ResultFromStatus(status);
  }

  if (out_ref != nullptr) *out_ref = static_cast<ZrcTileRef>(ref);
  return ZRC_OK;
}

ZrcResult zrcNavMeshRemoveTile(ZrcNavMesh* navmesh, ZrcTileRef ref) {
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (ResidentTileByRef(*navmesh->impl, ref) == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Null data and size: the tile was added with DT_TILE_FREE_DATA, so Detour
  // releases the copy it holds rather than handing it back.
  const dtStatus status =
      navmesh->impl->removeTile(static_cast<dtTileRef>(ref), nullptr, nullptr);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileRefAt(const ZrcNavMesh* navmesh, int32_t tile_x,
                              int32_t tile_z, int32_t tile_layer,
                              ZrcTileRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tile_x < 0 || tile_x >= navmesh->tile_count_x) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tile_z < 0 || tile_z >= navmesh->tile_count_z) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tile_layer < 0 || tile_layer > ZRC_MAX_TILE_LAYER) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const dtNavMesh& mesh = *navmesh->impl;
  const dtMeshTile* tile = mesh.getTileAt(tile_x, tile_z, tile_layer);
  if (tile == nullptr) return ZRC_OK;  // an empty slot, not an error
  *out_ref = static_cast<ZrcTileRef>(mesh.getTileRef(tile));
  return ZRC_OK;
}

int32_t zrcNavMeshTileCount(const ZrcNavMesh* navmesh) {
  if (navmesh == nullptr || navmesh->impl == nullptr) return -1;
  return ResidentTiles(*navmesh->impl);
}

ZrcResult zrcNavMeshTileRefsAt(const ZrcNavMesh* navmesh, int32_t tile_x,
                               int32_t tile_z, ZrcTileRef* out,
                               int32_t max_tiles, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tile_x < 0 || tile_x >= navmesh->tile_count_x) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (tile_z < 0 || tile_z >= navmesh->tile_count_z) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (max_tiles < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_tiles > 0 && out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const dtNavMesh& mesh = *navmesh->impl;
  // Asked for with the navmesh's own capacity so the count is the real one:
  // upstream writes only what fits and returns that, which a caller sizing a
  // second call from would read as the whole answer.
  const int capacity = mesh.getMaxTiles();
  TempBuffer scratch(sizeof(const dtMeshTile*) * static_cast<size_t>(capacity));
  const dtMeshTile** all = static_cast<const dtMeshTile**>(scratch.get());
  if (capacity > 0 && all == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  const int total = mesh.getTilesAt(tile_x, tile_z, all, capacity);
  const int32_t to_copy = total < max_tiles ? total : max_tiles;
  for (int32_t i = 0; i < to_copy; ++i) {
    out[i] = static_cast<ZrcTileRef>(mesh.getTileRef(all[i]));
  }
  *out_count = total;
  if (total > max_tiles) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileRefAtIndex(const ZrcNavMesh* navmesh, int32_t index,
                                   ZrcTileRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtNavMesh& mesh = *navmesh->impl;
  if (index < 0 || index >= mesh.getMaxTiles()) return ZRC_ERR_INVALID_ARGUMENT;
  const dtMeshTile* tile = mesh.getTile(index);
  if (tile == nullptr || tile->header == nullptr) return ZRC_OK;
  *out_ref = static_cast<ZrcTileRef>(mesh.getTileRef(tile));
  return ZRC_OK;
}

int32_t zrcNavMeshMaxTiles(const ZrcNavMesh* navmesh) {
  if (navmesh == nullptr || navmesh->impl == nullptr) return -1;
  return navmesh->impl->getMaxTiles();
}

ZrcResult zrcNavMeshTileBounds(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                               float* bmin, float* bmax) {
  if (navmesh == nullptr || navmesh->impl == nullptr || bmin == nullptr ||
      bmax == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = tile->header->bmin[i];
    bmax[i] = tile->header->bmax[i];
  }
  return ZRC_OK;
}

ZrcResult zrcNavMeshGetPolyArea(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                int32_t* out_area) {
  if (out_area == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_area = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  unsigned char area = 0;
  const dtStatus status =
      navmesh->impl->getPolyArea(static_cast<dtPolyRef>(ref), &area);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  *out_area = area;
  return ZRC_OK;
}

ZrcResult zrcNavMeshSetPolyArea(ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                int32_t area) {
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (area < 0 || area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  const dtStatus status = navmesh->impl->setPolyArea(
      static_cast<dtPolyRef>(ref), static_cast<unsigned char>(area));
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcNavMeshGetPolyFlags(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                 uint16_t* out_flags) {
  if (out_flags == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_flags = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  unsigned short flags = 0;
  const dtStatus status =
      navmesh->impl->getPolyFlags(static_cast<dtPolyRef>(ref), &flags);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  *out_flags = flags;
  return ZRC_OK;
}

ZrcResult zrcNavMeshSetPolyFlags(ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                 uint16_t flags) {
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtStatus status =
      navmesh->impl->setPolyFlags(static_cast<dtPolyRef>(ref), flags);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcNavMeshGetPolyType(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                                int32_t* out_type) {
  if (out_type == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_type = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = nullptr;
  const dtPoly* poly = nullptr;
  const dtStatus status = navmesh->impl->getTileAndPolyByRef(
      static_cast<dtPolyRef>(ref), &tile, &poly);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  *out_type = poly->getType();
  return ZRC_OK;
}

ZrcResult zrcNavMeshOffMeshConnectionEndPoints(const ZrcNavMesh* navmesh,
                                               ZrcPolyRef prev_ref,
                                               ZrcPolyRef poly_ref,
                                               float* out_start,
                                               float* out_end) {
  if (out_start == nullptr || out_end == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Upstream's own zero check returns a bare DT_FAILURE with no detail bits
  // (DetourNavMesh.cpp:1463-1464), which zrc::ResultFromStatus can only map to
  // the generic ZRC_ERR_QUERY_FAILED. Refusing here first gives the more
  // specific answer.
  if (navmesh == nullptr || navmesh->impl == nullptr || poly_ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtStatus status = navmesh->impl->getOffMeshConnectionPolyEndPoints(
      static_cast<dtPolyRef>(prev_ref), static_cast<dtPolyRef>(poly_ref),
      out_start, out_end);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcNavMeshOffMeshConnection(const ZrcNavMesh* navmesh,
                                      ZrcPolyRef ref,
                                      ZrcOffMeshConnection* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtNavMesh& mesh = *navmesh->impl;

  // Resolved through the safe accessor first, rather than
  // getOffMeshConnectionByRef alone, because that one returns NULL both for a
  // reference that does not resolve at all and for one that resolves to a
  // ground polygon — and only the second of those is ZRC_ERR_QUERY_FAILED
  // under this contract; the first is ZRC_ERR_INVALID_ARGUMENT, as a stale or
  // out-of-range reference is everywhere else in this file.
  const dtMeshTile* tile = nullptr;
  const dtPoly* poly = nullptr;
  const dtStatus status =
      mesh.getTileAndPolyByRef(static_cast<dtPolyRef>(ref), &tile, &poly);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION) {
    return ZRC_ERR_QUERY_FAILED;
  }

  // The reference is live and names an off-mesh polygon, so this cannot fail.
  const dtOffMeshConnection* con =
      mesh.getOffMeshConnectionByRef(static_cast<dtPolyRef>(ref));
  if (con == nullptr) return ZRC_ERR_QUERY_FAILED;

  out->start[0] = con->pos[0];
  out->start[1] = con->pos[1];
  out->start[2] = con->pos[2];
  out->end[0] = con->pos[3];
  out->end[1] = con->pos[4];
  out->end[2] = con->pos[5];
  out->radius = con->rad;
  // Flags and area come from the polygon, not the connection record, so a
  // zrcNavMeshSetPolyFlags since the tile was added is reflected here.
  out->area = poly->getArea();
  out->flags = poly->flags;
  out->bidirectional =
      (con->flags & DT_OFFMESH_CON_BIDIR) != 0 ? ZRC_TRUE : ZRC_FALSE;
  out->user_id = con->userId;
  out->end_side = con->side;
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileUserId(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                               uint32_t* out_user_id) {
  if (out_user_id == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_user_id = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_user_id = tile->header->userId;
  return ZRC_OK;
}

ZrcResult zrcNavMeshParams(const ZrcNavMesh* navmesh, ZrcNavMeshParams* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtNavMeshParams* params = navmesh->impl->getParams();
  out->origin[0] = params->orig[0];
  out->origin[1] = params->orig[1];
  out->origin[2] = params->orig[2];
  out->tile_width = params->tileWidth;
  out->tile_height = params->tileHeight;
  out->max_tiles = params->maxTiles;
  out->max_polys = params->maxPolys;
  return ZRC_OK;
}

ZrcResult zrcNavMeshCalcTileLoc(const ZrcNavMesh* navmesh, const float* pos,
                                int32_t* out_tile_x, int32_t* out_tile_z) {
  if (out_tile_x == nullptr || out_tile_z == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_tile_x = 0;
  *out_tile_z = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr || pos == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFiniteVec3(pos)) return ZRC_ERR_INVALID_ARGUMENT;

  // A finite position is not enough. calcTileLoc divides by the tile size and
  // casts the quotient to int with no check of its own
  // (DetourNavMesh.cpp:1063-1067), so a position far enough from the origin —
  // or a degenerate tile size, which makes the quotient infinite — turns that
  // conversion into undefined behaviour rather than an out-of-range answer.
  // The same division here in double is only ever a number, and the tile the
  // position would occupy has to be one an int can name.
  const dtNavMeshParams& params = *navmesh->impl->getParams();
  const double at_x = static_cast<double>(pos[0]) - params.orig[0];
  const double at_z = static_cast<double>(pos[2]) - params.orig[2];
  const double tile_x = at_x / params.tileWidth;
  const double tile_z = at_z / params.tileHeight;
  // The bound is on the quotient rather than on its floor, which is the same
  // test: flooring a value in [-2^31, 2^31) lands in [-2^31, 2^31 - 1], the
  // exact range an int can hold. Written as a negation so a NaN quotient,
  // which compares false against everything, is refused rather than let
  // through.
  if (!(tile_x >= -2147483648.0 && tile_x < 2147483648.0) ||
      !(tile_z >= -2147483648.0 && tile_z < 2147483648.0)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  navmesh->impl->calcTileLoc(pos, out_tile_x, out_tile_z);
  return ZRC_OK;
}

ZrcResult zrcOppositeTileSide(int32_t side, int32_t* out_side) {
  if (out_side == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_side = 0;
  if (side < 0 || side > 7) return ZRC_ERR_INVALID_ARGUMENT;
  *out_side = dtOppositeTile(side);
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileInfo(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                             ZrcTileInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtMeshHeader& header = *tile->header;

  out->tile_x = header.x;
  out->tile_z = header.y;
  out->tile_layer = header.layer;
  out->user_id = header.userId;
  out->poly_count = header.polyCount;
  out->ground_poly_count = header.offMeshBase;
  out->off_mesh_con_count = header.offMeshConCount;
  out->vert_count = header.vertCount;
  out->detail_mesh_count = header.detailMeshCount;
  out->detail_vert_count = header.detailVertCount;
  out->detail_tri_count = header.detailTriCount;
  out->bv_node_count = header.bvNodeCount;
  out->max_link_count = header.maxLinkCount;
  out->walkable_height = header.walkableHeight;
  out->walkable_radius = header.walkableRadius;
  out->walkable_climb = header.walkableClimb;
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = header.bmin[i];
    out->bmax[i] = header.bmax[i];
  }
  out->bv_quant_factor = header.bvQuantFactor;
  out->magic = header.magic;
  out->flags = static_cast<uint32_t>(tile->flags);
  out->salt = tile->salt;
  out->links_free_list = tile->linksFreeList;
  out->next_tile = tile->next != nullptr
                       ? static_cast<ZrcTileRef>(navmesh->impl->getTileRef(tile->next))
                       : 0u;
  return ZRC_OK;
}

ZrcResult zrcNavMeshPolyInfo(const ZrcNavMesh* navmesh, ZrcPolyRef ref,
                             ZrcPolyInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  // Upstream's own zero check returns a bare DT_FAILURE with no detail bits,
  // which zrc::ResultFromStatus can only map to the generic ZRC_ERR_QUERY_FAILED.
  if (navmesh == nullptr || navmesh->impl == nullptr || ref == 0) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = nullptr;
  const dtPoly* poly = nullptr;
  const dtStatus status = navmesh->impl->getTileAndPolyByRef(
      static_cast<dtPolyRef>(ref), &tile, &poly);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  memcpy(out->verts, poly->verts, sizeof(out->verts));
  memcpy(out->neis, poly->neis, sizeof(out->neis));
  out->flags = poly->flags;
  out->vert_count = poly->vertCount;
  out->area = poly->getArea();
  out->type = poly->getType();
  out->first_link = poly->firstLink;

  // An off-mesh connection's polygon has no detail sub-mesh: detailMeshCount
  // covers the ground polygons only (offMeshBase of them), and indexing
  // tile->detailMeshes by an off-mesh polygon's index reads past the end of
  // that array.
  const unsigned int poly_index =
      navmesh->impl->decodePolyIdPoly(static_cast<dtPolyRef>(ref));
  if (static_cast<int>(poly_index) < tile->header->offMeshBase) {
    const dtPolyDetail& detail = tile->detailMeshes[poly_index];
    out->detail_vert_base = detail.vertBase;
    out->detail_tri_base = detail.triBase;
    out->detail_vert_count = detail.vertCount;
    out->detail_tri_count = detail.triCount;
  }
  return ZRC_OK;
}

ZrcResult zrcNavMeshTilePolyRef(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                int32_t index, ZrcPolyRef* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (index < 0 || index >= tile->header->polyCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // getPolyRefBase encodes the tile's index and salt and leaves the polygon
  // bits clear, so the index bounded above completes the reference. Composing
  // it here rather than exposing the base keeps reference arithmetic on this
  // side of the boundary, where the bound is checked.
  const dtPolyRef base = navmesh->impl->getPolyRefBase(tile);
  *out = static_cast<ZrcPolyRef>(base | static_cast<dtPolyRef>(index));
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileLink(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                             int32_t index, ZrcLink* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (index < 0 || index >= tile->header->maxLinkCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtLink& link = tile->links[index];
  out->ref = static_cast<ZrcPolyRef>(link.ref);
  out->next = link.next;
  out->edge = link.edge;
  out->side = link.side;
  out->bmin = link.bmin;
  out->bmax = link.bmax;
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileBvNode(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                               int32_t index, ZrcBvNode* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  // A tile built with skip_bv_tree has bvNodeCount 0 and bvTree NULL, so the
  // bound has to be checked before bvTree is ever indexed.
  if (index < 0 || index >= tile->header->bvNodeCount) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtBVNode& node = tile->bvTree[index];
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = node.bmin[i];
    out->bmax[i] = node.bmax[i];
  }
  out->i = node.i;
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileVerts(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                              int32_t first, int32_t count, float* out) {
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult range_valid =
      ValidateTileRange(first, count, tile->header->vertCount);
  if (range_valid != ZRC_OK) return range_valid;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, tile->verts + static_cast<int64_t>(first) * 3,
         sizeof(float) * 3 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileDetailVerts(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                    int32_t first, int32_t count, float* out) {
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult range_valid =
      ValidateTileRange(first, count, tile->header->detailVertCount);
  if (range_valid != ZRC_OK) return range_valid;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, tile->detailVerts + static_cast<int64_t>(first) * 3,
         sizeof(float) * 3 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileDetailTris(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                   int32_t first, int32_t count,
                                   uint8_t* out) {
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult range_valid =
      ValidateTileRange(first, count, tile->header->detailTriCount);
  if (range_valid != ZRC_OK) return range_valid;
  if (count == 0) return ZRC_OK;
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, tile->detailTris + static_cast<int64_t>(first) * 4,
         sizeof(uint8_t) * 4 * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcNavMeshTileStateSize(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                  size_t* out_size) {
  if (out_size == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_size = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_size = static_cast<size_t>(navmesh->impl->getTileStateSize(tile));
  return ZRC_OK;
}

ZrcResult zrcNavMeshStoreTileState(const ZrcNavMesh* navmesh, ZrcTileRef ref,
                                   void* data, size_t size) {
  if (navmesh == nullptr || navmesh->impl == nullptr || data == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const int state_size = navmesh->impl->getTileStateSize(tile);
  // Exact length, not merely "big enough" as upstream itself requires:
  // restoreTileState sizes its polygon-state array from the *live tile's*
  // polyCount, not from anything inside the blob — dtTileState carries no
  // polygon count of its own — so this is the only thing that would catch a
  // blob and a tile that have since disagreed. Comparing against `state_size`
  // rather than casting the caller's `size` to int also means a `size` past
  // INT_MAX is simply unequal to it rather than a truncating cast.
  if (size < static_cast<size_t>(state_size)) return ZRC_ERR_BUFFER_TOO_SMALL;
  if (size > static_cast<size_t>(state_size)) return ZRC_ERR_INVALID_ARGUMENT;

  const dtStatus status = navmesh->impl->storeTileState(
      tile, static_cast<unsigned char*>(data), state_size);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcNavMeshRestoreTileState(ZrcNavMesh* navmesh, ZrcTileRef ref,
                                     const void* data, size_t size) {
  if (navmesh == nullptr || navmesh->impl == nullptr || data == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtMeshTile* tile = ResidentTileByRef(*navmesh->impl, ref);
  if (tile == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  // Same exact-length rule as zrcNavMeshStoreTileState, for the same reason.
  const int state_size = navmesh->impl->getTileStateSize(tile);
  if (size < static_cast<size_t>(state_size)) return ZRC_ERR_BUFFER_TOO_SMALL;
  if (size > static_cast<size_t>(state_size)) return ZRC_ERR_INVALID_ARGUMENT;

  // dtNavMesh::restoreTileState needs a mutable tile, but ResidentTileByRef —
  // the one helper this file uses to resolve a ZrcTileRef, because it is also
  // the one that rejects a reference to a free slot — is built on the public
  // getTileByRef, which only has a const-returning overload; the non-const
  // dtNavMesh::getTile(int) is private (DetourNavMesh.h:617). The tile itself
  // is not const data, though: the navmesh owns it as mutable storage, and
  // only the accessor used to reach it here is const-qualified. Stripping
  // that qualifier back off is therefore well defined.
  dtMeshTile* mutable_tile = const_cast<dtMeshTile*>(tile);
  const dtStatus status = navmesh->impl->restoreTileState(
      mutable_tile, static_cast<const unsigned char*>(data), state_size);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

void zrcNavMeshDestroy(ZrcNavMesh* navmesh) {
  if (navmesh == nullptr) return;
  dtFreeNavMesh(navmesh->impl);
  zrc::Delete(navmesh);
}

ZrcResult zrcNavMeshSerialize(const ZrcNavMesh* navmesh, void** out_data,
                              size_t* out_size) {
  if (out_data == nullptr || out_size == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_data = nullptr;
  *out_size = 0;
  if (navmesh == nullptr || navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  // A tiled navmesh has no single image, and inventing a container format for
  // one is a format this package would then have to keep. The tile bytes are
  // the asset: cook each with zrcTileDataBuild.
  if (ResidentTiles(*navmesh->impl) > 1) return ZRC_ERR_INVALID_ARGUMENT;
  const dtMeshTile* tile = SoleTile(*navmesh->impl);
  if (tile == nullptr || tile->data == nullptr || tile->dataSize <= 0) {
    return ZRC_ERR_EMPTY_RESULT;
  }

  const size_t size = static_cast<size_t>(tile->dataSize);
  void* copy = zrc::Alloc(size, DT_ALLOC_PERM);
  if (copy == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  memcpy(copy, tile->data, size);

  *out_data = copy;
  *out_size = size;
  return ZRC_OK;
}

ZrcResult zrcNavMeshImageSwapEndian(const void* data, size_t size,
                                    ZrcBool from_native, void** out_data,
                                    size_t* out_size) {
  if (out_data == nullptr || out_size == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_data = nullptr;
  *out_size = 0;
  if (data == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (size > (size_t)0x7fffffff || size < sizeof(dtMeshHeader)) {
    return ZRC_ERR_BAD_FORMAT;
  }

  // Swapping happens on a copy, so the caller's buffer is untouched whatever
  // goes wrong and no half-swapped image can escape this function.
  unsigned char* copy =
      static_cast<unsigned char*>(zrc::Alloc(size, DT_ALLOC_PERM));
  if (copy == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  memcpy(copy, data, size);

  const int bytes = static_cast<int>(size);
  const zrc::TileAdmission admission = zrc::AnyTileAdmission();
  ZrcResult result = ZRC_OK;

  if (from_native != ZRC_FALSE) {
    // Readable here, so it is checked in full before anything is touched.
    result = zrc::ValidateNavMeshImage(copy, size, admission);
    if (result == ZRC_OK && !dtNavMeshDataSwapEndian(copy, bytes)) {
      result = ZRC_ERR_BAD_FORMAT;
    }
    // Last, because dtNavMeshDataSwapEndian reads the header to find the body.
    if (result == ZRC_OK && !dtNavMeshHeaderSwapEndian(copy, bytes)) {
      result = ZRC_ERR_BAD_FORMAT;
    }
  } else {
    // Nothing here is readable until the header is, so that goes first. Its own
    // magic test accepts either byte order, which is what makes it safe to run
    // on bytes this build cannot otherwise interpret.
    if (!dtNavMeshHeaderSwapEndian(copy, bytes)) result = ZRC_ERR_BAD_FORMAT;
    // The header is native now. Its counts have to be known to address only
    // this buffer before dtNavMeshDataSwapEndian walks the body by them.
    if (result == ZRC_OK) {
      dtMeshHeader header;
      TileLayout layout;
      result = zrc::ValidateTileImageHeader(copy, size, admission, &header,
                                            &layout);
    }
    if (result == ZRC_OK && !dtNavMeshDataSwapEndian(copy, bytes)) {
      result = ZRC_ERR_BAD_FORMAT;
    }
    // Both halves native: now the indices inside the body can be checked.
    if (result == ZRC_OK) {
      result = zrc::ValidateNavMeshImage(copy, size, admission);
    }
  }

  if (result != ZRC_OK) {
    zrc::Free(copy);
    return result;
  }
  *out_data = copy;
  *out_size = size;
  return ZRC_OK;
}

ZrcResult zrcNavMeshValidate(const void* data, size_t size) {
  return zrc::ValidateNavMeshImage(data, size, zrc::AnyTileAdmission());
}

ZrcResult zrcTileLayout(const void* data, size_t size, ZrcTileLayout* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (data == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const ZrcResult valid =
      zrc::ValidateNavMeshImage(data, size, zrc::AnyTileAdmission());
  if (valid != ZRC_OK) return valid;

  dtMeshHeader header;
  memcpy(&header, data, sizeof(header));

  // Offsets come from the same arithmetic dtNavMesh::addTile derives them
  // with; sizes come from the header's own counts directly, not by
  // differencing offsets, since an empty array's offset equals the one that
  // follows it.
  // Checked rather than assumed even though the validation above already
  // rejects every count that could fail it: the false path writes nothing, so
  // an unchecked call would copy an uninitialised stack struct to the caller.
  TileLayout layout;
  if (!zrc::TileLayoutOf(header, &layout)) return ZRC_ERR_BAD_FORMAT;

  out->verts_offset = layout.verts;
  out->verts_size = static_cast<int64_t>(sizeof(float)) * 3 * header.vertCount;
  out->polys_offset = layout.polys;
  out->polys_size = static_cast<int64_t>(sizeof(dtPoly)) * header.polyCount;
  out->links_offset = layout.links;
  out->links_size = static_cast<int64_t>(sizeof(dtLink)) * header.maxLinkCount;
  out->detail_meshes_offset = layout.detail_meshes;
  out->detail_meshes_size =
      static_cast<int64_t>(sizeof(dtPolyDetail)) * header.detailMeshCount;
  out->detail_verts_offset = layout.detail_verts;
  out->detail_verts_size =
      static_cast<int64_t>(sizeof(float)) * 3 * header.detailVertCount;
  out->detail_tris_offset = layout.detail_tris;
  out->detail_tris_size = static_cast<int64_t>(4) * header.detailTriCount;
  out->bv_tree_offset = layout.bvtree;
  out->bv_tree_size = static_cast<int64_t>(sizeof(dtBVNode)) * header.bvNodeCount;
  out->off_mesh_cons_offset = layout.offmesh;
  out->off_mesh_cons_size =
      static_cast<int64_t>(sizeof(dtOffMeshConnection)) * header.offMeshConCount;
  out->total_size = layout.total;

  return ZRC_OK;
}

ZrcResult zrcNavMeshDeserialize(const void* data, size_t size,
                                ZrcNavMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;

  // Everything structural is settled here, before Detour is allowed to look at
  // a single byte.
  const ZrcResult valid =
      zrc::ValidateNavMeshImage(data, size, zrc::SoleTileAdmission());
  if (valid != ZRC_OK) return valid;

  // Detour keeps the buffer and writes link indices into it, so the caller's
  // bytes are copied rather than borrowed. That also gives the tile the
  // alignment the header struct wants, which a caller's slice may not have.
  unsigned char* copy =
      static_cast<unsigned char*>(zrc::Alloc(size, DT_ALLOC_PERM));
  if (copy == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  memcpy(copy, data, size);

  // Sealed in this package's own copy rather than in the caller's bytes, so an
  // image from another tool gets the same protection without being altered
  // where it lies. See SealBvSentinel.
  {
    dtMeshHeader loaded;
    memcpy(&loaded, copy, sizeof(loaded));
    TileLayout loaded_layout;
    if (zrc::TileLayoutOf(loaded, &loaded_layout)) {
      zrc::SealBvSentinel(copy, loaded, loaded_layout);
    }
  }

  ZrcNavMesh* handle = zrc::New<ZrcNavMesh>();
  if (handle == nullptr) {
    zrc::Free(copy);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  // A lone tile is a 1x1 grid, not a separate mode, so the tile entry points
  // find it at (0, 0) the same way they find a tile of a larger grid.
  handle->tile_count_x = 1;
  handle->tile_count_z = 1;
  handle->impl = dtAllocNavMesh();
  if (handle->impl == nullptr) {
    zrc::Free(copy);
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  dtMeshHeader header;
  memcpy(&header, copy, sizeof(header));

  bool destructible = false;
  const dtStatus status = InitSingleTile(handle->impl, header, copy,
                                         static_cast<int>(size), &destructible);
  if (dtStatusFailed(status)) {
    zrc::Free(copy);
    FreeFailedNavMesh(handle->impl, destructible);
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }

  *out = handle;
  return ZRC_OK;
}

int32_t zrcNavMeshPolyCount(const ZrcNavMesh* navmesh) {
  // -1, not 0: a navmesh with no tile resident has no polygons and answers 0
  // legitimately, so the failure has to be a value no mesh can produce. Same
  // sentinel zrcNavMeshTileCount and zrcNavMeshMaxTiles report.
  if (navmesh == nullptr || navmesh->impl == nullptr) return -1;
  // Through a const dtNavMesh, so the public const getTile overload is the one
  // selected — the non-const overload is private.
  const dtNavMesh& mesh = *navmesh->impl;
  int32_t total = 0;
  for (int i = 0; i < mesh.getMaxTiles(); ++i) {
    const dtMeshTile* tile = mesh.getTile(i);
    if (tile == nullptr || tile->header == nullptr) continue;
    total += tile->header->polyCount;
  }
  return total;
}

ZrcResult zrcNavMeshBounds(const ZrcNavMesh* navmesh, float* bmin,
                           float* bmax) {
  if (navmesh == nullptr || navmesh->impl == nullptr || bmin == nullptr ||
      bmax == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // The union of the resident tiles, which for a lone tile is that tile. The
  // grid the navmesh was created for would give a wider answer covering tiles
  // that are not loaded, and a streaming host wants what it can actually query.
  const dtNavMesh& mesh = *navmesh->impl;
  bool any = false;
  for (int t = 0; t < mesh.getMaxTiles(); ++t) {
    const dtMeshTile* tile = mesh.getTile(t);
    if (tile == nullptr || tile->header == nullptr) continue;
    for (int i = 0; i < 3; ++i) {
      if (!any || tile->header->bmin[i] < bmin[i]) bmin[i] = tile->header->bmin[i];
      if (!any || tile->header->bmax[i] > bmax[i]) bmax[i] = tile->header->bmax[i];
    }
    any = true;
  }
  if (!any) return ZRC_ERR_EMPTY_RESULT;
  return ZRC_OK;
}

}  // extern "C"
