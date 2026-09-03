//===----------------------------------------------------------------------===//
// zrecast — the tile cache: compressed per-tile layers, the obstacles carved
// into them at runtime, and the cook-time half that builds a layer and takes
// one apart for inspection.
//
// The mesh-process callback is the one place a host's data reaches a navmesh
// without the checks zrcNavMeshAddTile applies: dtTileCache::buildNavMeshTile
// feeds its result straight to dtNavMesh::addTile. HostMeshProcess narrows
// what a callback may touch to close that by construction; see its definition
// below.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

#include "DetourCommon.h"

namespace {

//===----------------------------------------------------------------------===//
// Argument validation
//===----------------------------------------------------------------------===//

ZrcResult ValidateCompressorHooks(const ZrcTileCacheCompressor* compressor) {
  if (compressor == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (compressor->max_compressed_size == nullptr || compressor->compress == nullptr ||
      compressor->decompress == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// NULL selects upstream's own default allocator. A non-NULL one must supply
/// both alloc and free: HostTileCacheAlloc forwards each independently, and a
/// caller that filled only one would silently mix a host allocation with a
/// default deallocation, or the reverse.
ZrcResult ValidateAllocatorHooks(const ZrcTileCacheAllocator* allocator) {
  if (allocator == nullptr) return ZRC_OK;
  if (allocator->allocate == nullptr || allocator->deallocate == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

/// Bounds ZrcTileCacheParams the way zrcTileCacheCreate requires.
///
/// The salt-bit check mirrors dtTileCache::init's own formula exactly
/// (DetourTileCache.cpp:164-168), which differs from a navmesh's: a tile
/// cache reference encodes only a salt and a tile index, no polygon index.
/// The walkable_climb / cell_height bound guards buildNavMeshTile's
/// `(int)(m_params.walkableClimb / m_params.ch)` (DetourTileCache.cpp:673),
/// undefined once the quotient leaves int range.
ZrcResult ValidateTileCacheParams(const ZrcTileCacheParams& params) {
  if (!zrc::IsFiniteVec3(params.origin)) return ZRC_ERR_INVALID_ARGUMENT;
  for (int i = 0; i < 3; ++i) {
    if (params.origin[i] < -zrc::kMaxCoordinate ||
        params.origin[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  if (!zrc::IsFinite(params.cell_size) || !(params.cell_size > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(params.cell_height) || !(params.cell_height > 0.f)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params.width < 1 || params.width > 255) return ZRC_ERR_INVALID_ARGUMENT;
  if (params.height < 1 || params.height > 255) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(params.walkable_height) || params.walkable_height < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(params.walkable_radius) || params.walkable_radius < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(params.walkable_climb) || params.walkable_climb < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(params.max_simplification_error) ||
      params.max_simplification_error < 0.f) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (params.max_tiles < 1 || params.max_obstacles < 1) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  const unsigned int tile_bits =
      dtIlog2(dtNextPow2(static_cast<unsigned int>(params.max_tiles)));
  const unsigned int salt_bits = dtMin(31u, 32u - tile_bits);
  if (salt_bits < 10u) return ZRC_ERR_INVALID_ARGUMENT;

  const double climb_cells = static_cast<double>(params.walkable_climb) /
                             static_cast<double>(params.cell_height);
  if (!(climb_cells < 2147483648.0)) return ZRC_ERR_INVALID_ARGUMENT;

  return ZRC_OK;
}

/// Bounds a ZrcTileCacheLayerHeader the way zrcTileCacheLayerBuild requires.
/// width and height are limited to [1, 255] because dtTileCacheLayerHeader
/// stores each as a single byte, and the sub-region fields index that grid.
ZrcResult ValidateLayerHeader(const ZrcTileCacheLayerHeader& header) {
  if (!zrc::IsFiniteVec3(header.bmin) || !zrc::IsFiniteVec3(header.bmax)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (header.bmin[i] < -zrc::kMaxCoordinate ||
        header.bmin[i] > zrc::kMaxCoordinate ||
        header.bmax[i] < -zrc::kMaxCoordinate ||
        header.bmax[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (header.bmin[i] > header.bmax[i]) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (header.width < 1 || header.width > 255) return ZRC_ERR_INVALID_ARGUMENT;
  if (header.height < 1 || header.height > 255) return ZRC_ERR_INVALID_ARGUMENT;
  if (header.height_min < 0 || header.height_max < header.height_min ||
      header.height_max > 0xffff) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (header.min_x < 0 || header.min_x > header.max_x ||
      header.max_x >= header.width) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (header.min_z < 0 || header.min_z > header.max_z ||
      header.max_z >= header.height) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}


//===----------------------------------------------------------------------===//
// Salt-protected reference resolution
//
// dtCompressedTileRef and dtObstacleRef are both salt-protected, but neither
// accessor upstream exposes rejects a reference that merely *decodes* to an
// unoccupied slot: every slot starts with salt 1 at init, so a forged
// reference built from a never-issued salt can resolve to a free slot rather
// than failing outright. dtTileCache::removeTile then dereferences
// tile->header unconditionally (DetourTileCache.cpp:306), which is a null
// dereference for such a slot — the same class of hazard ResidentTileByRef in
// zrecast_navmesh.cpp guards against for dtNavMesh. The helpers below apply
// the same guard here: a tile's occupancy is header != nullptr, an
// obstacle's is state != DT_OBSTACLE_EMPTY.
//===----------------------------------------------------------------------===//

const dtCompressedTile* ResidentCompressedTileByRef(dtTileCache& cache,
                                                    ZrcCompressedTileRef ref) {
  if (ref == 0) return nullptr;
  const dtCompressedTile* tile =
      cache.getTileByRef(static_cast<dtCompressedTileRef>(ref));
  if (tile == nullptr || tile->header == nullptr) return nullptr;
  return tile;
}

const dtTileCacheObstacle* ResidentObstacleByRef(dtTileCache& cache,
                                                 ZrcObstacleRef ref) {
  if (ref == 0) return nullptr;
  const dtTileCacheObstacle* ob =
      cache.getObstacleByRef(static_cast<dtObstacleRef>(ref));
  if (ob == nullptr || ob->state == DT_OBSTACLE_EMPTY) return nullptr;
  return ob;
}

/// Occupied obstacle slots, counted by inspection — dtTileCache keeps no
/// public count, the same reason zrecast_navmesh.cpp's ResidentTiles walks
/// the tile array instead of trusting one.
int32_t ResidentObstacles(const dtTileCache& cache) {
  int32_t n = 0;
  for (int i = 0; i < cache.getObstacleCount(); ++i) {
    const dtTileCacheObstacle* ob = cache.getObstacle(i);
    if (ob != nullptr && ob->state != DT_OBSTACLE_EMPTY) ++n;
  }
  return n;
}

//===----------------------------------------------------------------------===//
// Obstacle bounds, reproduced from dtTileCache::getObstacleBounds
// (DetourTileCache.cpp:795-825) so the pre-flight overlap check below bounds
// exactly the box upstream will carve. The box shape needs no helper: its
// bounds are the caller's own bmin/bmax, unchanged.
//===----------------------------------------------------------------------===//

void CylinderObstacleBounds(const float* position, float radius, float height,
                            float* bmin, float* bmax) {
  bmin[0] = position[0] - radius;
  bmin[1] = position[1];
  bmin[2] = position[2] - radius;
  bmax[0] = position[0] + radius;
  bmax[1] = position[1] + height;
  bmax[2] = position[2] + radius;
}

void OrientedBoxObstacleBounds(const float* center, const float* half_extents,
                               float* bmin, float* bmax) {
  const float maxr = 1.41f * dtMax(half_extents[0], half_extents[2]);
  bmin[0] = center[0] - maxr;
  bmax[0] = center[0] + maxr;
  bmin[1] = center[1] - half_extents[1];
  bmax[1] = center[1] + half_extents[1];
  bmin[2] = center[2] - maxr;
  bmax[2] = center[2] + maxr;
}

/// Refuses an obstacle that would overlap more than ZRC_MAX_TOUCHED_TILES
/// tiles. Upstream carves such an obstacle into the first eight touched
/// tiles and leaves the rest untouched, silently (DetourTileCache.cpp:544-560,
/// where ob->touched is sized DT_MAX_TOUCHED_TILES and queryTiles is called
/// with that as maxResults). Querying with one more slot than the limit is
/// enough to tell whether it would have been dropped.
ZrcResult CheckObstacleWontOverflowTiles(dtTileCache& cache, const float* bmin,
                                         const float* bmax) {
  dtCompressedTileRef touched[ZRC_MAX_TOUCHED_TILES + 1];
  int touched_count = 0;
  const dtStatus status =
      cache.queryTiles(bmin, bmax, touched, &touched_count, ZRC_MAX_TOUCHED_TILES + 1);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (touched_count > ZRC_MAX_TOUCHED_TILES) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Decompressed layers
//===----------------------------------------------------------------------===//

int32_t LayerGridLength(const dtTileCacheLayer& layer) {
  return static_cast<int32_t>(layer.header->width) *
         static_cast<int32_t>(layer.header->height);
}

/// Frees and reconstructs a TempBuffer in place at a new size. TempBuffer has
/// no reset of its own — it allocates once at construction and frees at
/// destruction — so changing its size between calls means destroying and
/// placement-constructing a fresh one over the same storage.
void Reset(zrc::TempBuffer& buffer, size_t size) {
  buffer.~TempBuffer();
  new (&buffer) zrc::TempBuffer(size);
}

}  // namespace

//===----------------------------------------------------------------------===//
// The tile cache's three host-supplied interfaces, declared in
// zrecast_internal.h
//===----------------------------------------------------------------------===//

namespace zrc {

HostCompressor::HostCompressor(const ZrcTileCacheCompressor& hooks) : hooks_(hooks) {}

int HostCompressor::maxCompressedSize(const int bufferSize) {
  return hooks_.max_compressed_size(hooks_.user, bufferSize);
}

dtStatus HostCompressor::compress(const unsigned char* buffer, const int bufferSize,
                                  unsigned char* compressed,
                                  const int maxCompressedSize, int* compressedSize) {
  const ZrcResult result = hooks_.compress(hooks_.user, buffer, bufferSize, compressed,
                                           maxCompressedSize, compressedSize);
  return result == ZRC_OK ? DT_SUCCESS : DT_FAILURE;
}

dtStatus HostCompressor::decompress(const unsigned char* compressed,
                                    const int compressedSize, unsigned char* buffer,
                                    const int maxBufferSize, int* bufferSize) {
  const ZrcResult result = hooks_.decompress(hooks_.user, compressed, compressedSize,
                                             buffer, maxBufferSize, bufferSize);
  return result == ZRC_OK ? DT_SUCCESS : DT_FAILURE;
}

HostTileCacheAlloc::HostTileCacheAlloc(const ZrcTileCacheAllocator* hooks)
    : hooks_(hooks) {}

void HostTileCacheAlloc::reset() {
  if (hooks_ != nullptr && hooks_->reset != nullptr) hooks_->reset(hooks_->user);
}

void* HostTileCacheAlloc::alloc(const size_t size) {
  if (hooks_ != nullptr) return hooks_->allocate(hooks_->user, size);
  return dtAlloc(size, DT_ALLOC_TEMP);
}

void HostTileCacheAlloc::free(void* ptr) {
  if (hooks_ != nullptr) {
    hooks_->deallocate(hooks_->user, ptr);
    return;
  }
  dtFree(ptr);
}

HostMeshProcess::HostMeshProcess(ZrcTileCacheMeshProcess hook, void* user)
    : hook_(hook),
      user_(user),
      last_result_(ZRC_OK),
      con_verts_(0),
      con_rad_(0),
      con_flags_(0),
      con_areas_(0),
      con_dir_(0),
      con_user_id_(0) {}

/// Narrows what a callback may change about a tile to exactly the fields
/// ZrcTileCacheBuildParams exposes, then writes the validated result back
/// into `params` itself. `polyAreas` and `polyFlags` are the same arrays
/// `params->polyAreas` and `params->polyFlags` point at — passed separately,
/// non-const, because dtNavMeshCreateParams declares those two fields const
/// (DetourNavMeshBuilder.h:38,37) while dtTileCache::buildNavMeshTile hands
/// process() the mutable arrays it built them from
/// (DetourTileCache.cpp:740-741,758) — so these are used here rather than
/// `params->polyAreas`/`params->polyFlags`, to reach the same memory without
/// a const_cast.
///
/// Upstream's process() returns void and feeds whatever `params` holds on
/// return straight to dtCreateNavMeshData, so a callback's writes into
/// `polyAreas`/`polyFlags` are already committed by the time a failure here
/// is noticed — there is no way to undo that from inside this function. What
/// this function controls is whether `params` gains a pointer or a count the
/// callback chose, and whether the failure becomes visible at all:
/// last_result_ is what makes it visible, since upstream would otherwise
/// discard it and commit the tile as though nothing had gone wrong.
void HostMeshProcess::process(dtNavMeshCreateParams* params, unsigned char* polyAreas,
                              unsigned short* polyFlags) {
  last_result_ = ZRC_OK;

  ZrcTileCacheBuildParams build_params;
  build_params.areas = polyAreas;
  build_params.flags = polyFlags;
  build_params.poly_count = params->polyCount;
  build_params.user_id = params->userId;
  build_params.connections = nullptr;
  build_params.connection_count = 0;

  const ZrcResult hook_result = hook_(user_, &build_params);
  if (hook_result != ZRC_OK) {
    last_result_ = hook_result;
    return;
  }

  for (int32_t i = 0; i < build_params.poly_count; ++i) {
    if (build_params.areas[i] >= ZRC_MAX_AREAS) {
      last_result_ = ZRC_ERR_INVALID_ARGUMENT;
      return;
    }
  }

  const int32_t con_count = build_params.connection_count;
  if (con_count < 0 || con_count > zrc::kMaxOffMeshConnections) {
    last_result_ = ZRC_ERR_INVALID_ARGUMENT;
    return;
  }
  if (con_count > 0 && build_params.connections == nullptr) {
    last_result_ = ZRC_ERR_INVALID_ARGUMENT;
    return;
  }
  for (int32_t i = 0; i < con_count; ++i) {
    const ZrcResult con_ok = zrc::ValidateOffMeshConnection(build_params.connections[i]);
    if (con_ok != ZRC_OK) {
      last_result_ = con_ok;
      return;
    }
  }

  // Unpacked into the six parallel arrays dtNavMeshCreateParams wants, the
  // same shape zrc::BuildTileData uses for hand-authored connections
  // (ffi/zrecast_navmesh.cpp). The buffers must outlive this call — Detour
  // reads them inside dtCreateNavMeshData, after process() returns — so they
  // live on the handle rather than the stack, reallocated here each call.
  Reset(con_verts_, sizeof(float) * 6 * static_cast<size_t>(con_count));
  Reset(con_rad_, sizeof(float) * static_cast<size_t>(con_count));
  Reset(con_flags_, sizeof(unsigned short) * static_cast<size_t>(con_count));
  Reset(con_areas_, static_cast<size_t>(con_count));
  Reset(con_dir_, static_cast<size_t>(con_count));
  Reset(con_user_id_, sizeof(unsigned int) * static_cast<size_t>(con_count));

  if (con_count > 0) {
    if (con_verts_.get() == nullptr || con_rad_.get() == nullptr ||
        con_flags_.get() == nullptr || con_areas_.get() == nullptr ||
        con_dir_.get() == nullptr || con_user_id_.get() == nullptr) {
      last_result_ = ZRC_ERR_OUT_OF_MEMORY;
      return;
    }
    float* con_verts = static_cast<float*>(con_verts_.get());
    float* con_rad = static_cast<float*>(con_rad_.get());
    unsigned short* con_flags = static_cast<unsigned short*>(con_flags_.get());
    unsigned char* con_areas = static_cast<unsigned char*>(con_areas_.get());
    unsigned char* con_dir = static_cast<unsigned char*>(con_dir_.get());
    unsigned int* con_user_id = static_cast<unsigned int*>(con_user_id_.get());
    for (int32_t i = 0; i < con_count; ++i) {
      const ZrcOffMeshConnection& con = build_params.connections[i];
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
    params->offMeshConVerts = con_verts;
    params->offMeshConRad = con_rad;
    params->offMeshConFlags = con_flags;
    params->offMeshConAreas = con_areas;
    params->offMeshConDir = con_dir;
    params->offMeshConUserID = con_user_id;
  }
  params->offMeshConCount = con_count;
  params->userId = build_params.user_id;
}

}  // namespace zrc

extern "C" {

//===----------------------------------------------------------------------===//
// The tile cache
//===----------------------------------------------------------------------===//

ZrcResult zrcTileCacheCreate(const ZrcTileCacheParams* params,
                             const ZrcTileCacheCompressor* compressor,
                             const ZrcTileCacheAllocator* allocator,
                             ZrcTileCacheMeshProcess mesh_process,
                             void* mesh_process_user, ZrcTileCache** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (params == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult params_ok = ValidateTileCacheParams(*params);
  if (params_ok != ZRC_OK) return params_ok;
  const ZrcResult compressor_ok = ValidateCompressorHooks(compressor);
  if (compressor_ok != ZRC_OK) return compressor_ok;
  const ZrcResult allocator_ok = ValidateAllocatorHooks(allocator);
  if (allocator_ok != ZRC_OK) return allocator_ok;

  // ZrcTileCache aggregates a HostMeshProcess, and HostMeshProcess holds six
  // TempBuffer members that disable copy and move. zrc::New<T> constructs
  // with parentheses, which for an aggregate with no viable constructor of
  // its own — every member here has one, but none of them is a default or a
  // copy/move constructor — is ill-formed in C++17. Direct-list-init with
  // braces performs member-wise aggregate initialisation instead, and each
  // element here is a prvalue of the member's own type, so mandatory copy
  // elision constructs it in place without ever invoking a copy or move.
  void* block = zrc::Alloc(sizeof(ZrcTileCache), DT_ALLOC_PERM);
  if (block == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  ZrcTileCache* handle = new (block) ZrcTileCache{
      nullptr,
      zrc::HostCompressor(*compressor),
      zrc::HostTileCacheAlloc(allocator),
      zrc::HostMeshProcess(mesh_process, mesh_process_user),
  };

  handle->impl = dtAllocTileCache();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }

  dtTileCacheParams native;
  memset(&native, 0, sizeof(native));
  memcpy(native.orig, params->origin, sizeof(native.orig));
  native.cs = params->cell_size;
  native.ch = params->cell_height;
  native.width = params->width;
  native.height = params->height;
  native.walkableHeight = params->walkable_height;
  native.walkableRadius = params->walkable_radius;
  native.walkableClimb = params->walkable_climb;
  native.maxSimplificationError = params->max_simplification_error;
  native.maxTiles = params->max_tiles;
  native.maxObstacles = params->max_obstacles;

  // A tile cache cannot be re-initialised, so this is the one call: NULL
  // tmproc when the caller supplied no hook, which is also what makes
  // HostMeshProcess::process unreachable in that case rather than merely
  // untriggered.
  const dtStatus status = handle->impl->init(
      &native, &handle->allocator, &handle->compressor,
      mesh_process != nullptr ? &handle->mesh_process : nullptr);
  if (dtStatusFailed(status)) {
    dtFreeTileCache(handle->impl);
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }

  *out = handle;
  return ZRC_OK;
}

void zrcTileCacheDestroy(ZrcTileCache* cache) {
  if (cache == nullptr) return;
  dtFreeTileCache(cache->impl);
  zrc::Delete(cache);
}

ZrcResult zrcTileCacheParams(const ZrcTileCache* cache, ZrcTileCacheParams* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheParams& p = *cache->impl->getParams();
  memcpy(out->origin, p.orig, sizeof(out->origin));
  out->cell_size = p.cs;
  out->cell_height = p.ch;
  out->width = p.width;
  out->height = p.height;
  out->walkable_height = p.walkableHeight;
  out->walkable_radius = p.walkableRadius;
  out->walkable_climb = p.walkableClimb;
  out->max_simplification_error = p.maxSimplificationError;
  out->max_tiles = p.maxTiles;
  out->max_obstacles = p.maxObstacles;
  return ZRC_OK;
}

/// Upstream reads magic/version off `data` before comparing `size` against
/// anything and then stores `compressedSize = dataSize - headerSize`, which
/// goes negative for a short buffer and reaches the host codec as an int
/// (DetourTileCache.cpp:247-285); requiring `size >= dtAlign4(sizeof(header))`
/// before either read keeps that subtraction non-negative. Occupancy is
/// checked here too, ahead of the copy: upstream's own check
/// (`if (getTileAt(...)) return DT_FAILURE;`) reports a bare DT_FAILURE with
/// no detail bit, which zrc::ResultFromStatus can only map to
/// ZRC_ERR_QUERY_FAILED, not the documented ZRC_ERR_TILE_OCCUPIED.
ZrcResult zrcTileCacheAddTile(ZrcTileCache* cache, const void* data, size_t size,
                              ZrcCompressedTileRef* out_ref) {
  if (out_ref != nullptr) *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (data == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (size > static_cast<size_t>(0x7fffffff)) return ZRC_ERR_BAD_FORMAT;
  const size_t header_size =
      static_cast<size_t>(dtAlign4(static_cast<int>(sizeof(dtTileCacheLayerHeader))));
  if (size < header_size) return ZRC_ERR_BAD_FORMAT;

  dtTileCacheLayerHeader header;
  memcpy(&header, data, sizeof(header));
  if (header.magic != DT_TILECACHE_MAGIC) return ZRC_ERR_BAD_FORMAT;
  if (header.version != DT_TILECACHE_VERSION) return ZRC_ERR_UNSUPPORTED_VERSION;

  if (cache->impl->getTileAt(header.tx, header.ty, header.tlayer) != nullptr) {
    return ZRC_ERR_TILE_OCCUPIED;
  }

  // The header says the caller keeps ownership of `data`, so it is copied
  // rather than borrowed, and added with the flag that makes the cache free
  // its own copy.
  unsigned char* copy = static_cast<unsigned char*>(zrc::Alloc(size, DT_ALLOC_PERM));
  if (copy == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  memcpy(copy, data, size);

  dtCompressedTileRef ref = 0;
  const dtStatus status =
      cache->impl->addTile(copy, static_cast<int>(size), DT_COMPRESSEDTILE_FREE_DATA, &ref);
  if (dtStatusFailed(status)) {
    zrc::Free(copy);
    return zrc::ResultFromStatus(status);
  }
  if (out_ref != nullptr) *out_ref = static_cast<ZrcCompressedTileRef>(ref);
  return ZRC_OK;
}

/// dtTileCache::removeTile always frees the tile's own bytes when the
/// DT_COMPRESSEDTILE_FREE_DATA flag is set — which zrcTileCacheAddTile always
/// sets — regardless of whether `data`/`dataSize` out parameters are given
/// (DetourTileCache.cpp:293-337). So upstream can never hand ownership of the
/// tile's own storage back to a caller here; instead, a fresh copy is taken
/// before removing, and it is that copy — not the tile's original storage —
/// that becomes the caller's to free with zrcFree.
ZrcResult zrcTileCacheRemoveTile(ZrcTileCache* cache, ZrcCompressedTileRef ref,
                                 void** out_data, size_t* out_size) {
  if (out_data != nullptr) *out_data = nullptr;
  if (out_size != nullptr) *out_size = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const dtCompressedTile* tile = ResidentCompressedTileByRef(*cache->impl, ref);
  if (tile == nullptr) return ZRC_ERR_NOT_FOUND;

  void* copy = nullptr;
  size_t copy_size = 0;
  if (out_data != nullptr || out_size != nullptr) {
    copy_size = static_cast<size_t>(tile->dataSize);
    copy = zrc::Alloc(copy_size, DT_ALLOC_PERM);
    if (copy == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
    memcpy(copy, tile->data, copy_size);
  }

  const dtStatus status =
      cache->impl->removeTile(static_cast<dtCompressedTileRef>(ref), nullptr, nullptr);
  if (dtStatusFailed(status)) {
    zrc::Free(copy);
    return zrc::ResultFromStatus(status);
  }

  if (out_data != nullptr) *out_data = copy;
  if (out_size != nullptr) *out_size = copy_size;
  return ZRC_OK;
}

ZrcResult zrcTileCacheTileInfo(const ZrcTileCache* cache, ZrcCompressedTileRef ref,
                               ZrcCompressedTileInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtCompressedTile* tile = ResidentCompressedTileByRef(*cache->impl, ref);
  if (tile == nullptr) return ZRC_ERR_NOT_FOUND;

  const dtTileCacheLayerHeader& h = *tile->header;
  out->tile_x = h.tx;
  out->tile_y = h.ty;
  out->tile_layer = h.tlayer;
  memcpy(out->bmin, h.bmin, sizeof(out->bmin));
  memcpy(out->bmax, h.bmax, sizeof(out->bmax));
  out->height_min = h.hmin;
  out->height_max = h.hmax;
  out->width = h.width;
  out->height = h.height;
  out->min_x = h.minx;
  out->max_x = h.maxx;
  out->min_z = h.miny;
  out->max_z = h.maxy;
  out->data_size = tile->dataSize;
  return ZRC_OK;
}

ZrcResult zrcTileCacheTileAt(const ZrcTileCache* cache, int32_t tile_x, int32_t tile_y,
                             int32_t tile_layer, ZrcCompressedTileRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtCompressedTile* tile = cache->impl->getTileAt(tile_x, tile_y, tile_layer);
  if (tile == nullptr) return ZRC_OK;  // an empty slot, not an error
  *out_ref = static_cast<ZrcCompressedTileRef>(cache->impl->getTileRef(tile));
  return ZRC_OK;
}

/// Upstream's own getTilesAt increments its count only while it is below the
/// caller's own `maxTiles` (DetourTileCache.cpp:173-193), so the count it
/// returns is capped at whatever buffer size it was given rather than
/// reporting a true total past that cap. Querying first with a buffer sized
/// to the cache's own tile capacity — the true upper bound on how many
/// layers can stack at one position — recovers the real count, the same fix
/// zrcTileCacheQueryTiles applies to the identical shape of truncation in
/// dtTileCache::queryTiles.
ZrcResult zrcTileCacheTilesAt(const ZrcTileCache* cache, int32_t tile_x, int32_t tile_y,
                              ZrcCompressedTileRef* out, int32_t max_tiles,
                              int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_tiles < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_tiles > 0 && out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  dtTileCache& tc = *cache->impl;
  const int capacity = tc.getTileCount();
  zrc::TempBuffer scratch(sizeof(dtCompressedTileRef) * static_cast<size_t>(capacity));
  dtCompressedTileRef* all = static_cast<dtCompressedTileRef*>(scratch.get());
  if (capacity > 0 && all == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  const int total = tc.getTilesAt(tile_x, tile_y, all, capacity);
  const int32_t to_copy = total < max_tiles ? total : max_tiles;
  for (int32_t i = 0; i < to_copy; ++i) {
    out[i] = static_cast<ZrcCompressedTileRef>(all[i]);
  }
  *out_count = total;
  if (total > max_tiles) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

/// Slots run [0, max_tiles); upstream's own getTile(int) bounds nothing.
ZrcResult zrcTileCacheTileRefAt(const ZrcTileCache* cache, int32_t index,
                                ZrcCompressedTileRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCache& tc = *cache->impl;
  if (index < 0 || index >= tc.getTileCount()) return ZRC_ERR_INVALID_ARGUMENT;
  const dtCompressedTile* tile = tc.getTile(index);
  if (tile == nullptr || tile->header == nullptr) return ZRC_OK;  // free slot
  *out_ref = static_cast<ZrcCompressedTileRef>(tc.getTileRef(tile));
  return ZRC_OK;
}

ZrcResult zrcTileCacheObstacleInfo(const ZrcTileCache* cache, ZrcObstacleRef ref,
                                   ZrcObstacleInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheObstacle* ob = ResidentObstacleByRef(*cache->impl, ref);
  if (ob == nullptr) return ZRC_ERR_NOT_FOUND;

  if (ob->type == DT_OBSTACLE_CYLINDER) {
    out->shape = ZRC_OBSTACLE_CYLINDER;
    memcpy(out->position, ob->cylinder.pos, sizeof(out->position));
    out->radius = ob->cylinder.radius;
    out->height = ob->cylinder.height;
  } else if (ob->type == DT_OBSTACLE_BOX) {
    out->shape = ZRC_OBSTACLE_BOX;
    memcpy(out->bmin, ob->box.bmin, sizeof(out->bmin));
    memcpy(out->bmax, ob->box.bmax, sizeof(out->bmax));
  } else if (ob->type == DT_OBSTACLE_ORIENTED_BOX) {
    out->shape = ZRC_OBSTACLE_ORIENTED_BOX;
    memcpy(out->center, ob->orientedBox.center, sizeof(out->center));
    memcpy(out->half_extents, ob->orientedBox.halfExtents, sizeof(out->half_extents));
    // dtTileCache::addBoxObstacle stores rotAux[0] = cos(a/2)*sin(-a/2) and
    // rotAux[1] = cos(a/2)*cos(a/2) - 0.5 rather than the angle itself
    // (DetourTileCache.cpp:451-454). By the half-angle and product-to-sum
    // identities, cos(a/2)*sin(-a/2) = -sin(a)/2 and
    // cos^2(a/2) - 0.5 = cos(a)/2, so the angle is recovered rather than
    // stored, and can differ from the input in its last bits.
    const float rot0 = ob->orientedBox.rotAux[0];
    const float rot1 = ob->orientedBox.rotAux[1];
    out->y_radians = atan2f(-2.f * rot0, 2.f * rot1);
  }

  if (ob->state == DT_OBSTACLE_EMPTY) {
    out->state = ZRC_OBSTACLE_EMPTY;
  } else if (ob->state == DT_OBSTACLE_PROCESSING) {
    out->state = ZRC_OBSTACLE_PROCESSING;
  } else if (ob->state == DT_OBSTACLE_PROCESSED) {
    out->state = ZRC_OBSTACLE_PROCESSED;
  } else if (ob->state == DT_OBSTACLE_REMOVING) {
    out->state = ZRC_OBSTACLE_REMOVING;
  }
  out->touched_count = ob->ntouched;
  out->pending_count = ob->npending;
  return ZRC_OK;
}

/// Slots run [0, max_obstacles); upstream's own getObstacle(int) bounds
/// nothing.
ZrcResult zrcTileCacheObstacleRefAt(const ZrcTileCache* cache, int32_t index,
                                    ZrcObstacleRef* out_ref) {
  if (out_ref == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCache& tc = *cache->impl;
  if (index < 0 || index >= tc.getObstacleCount()) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheObstacle* ob = tc.getObstacle(index);
  if (ob == nullptr || ob->state == DT_OBSTACLE_EMPTY) return ZRC_OK;  // free slot
  *out_ref = static_cast<ZrcObstacleRef>(tc.getObstacleRef(ob));
  return ZRC_OK;
}

ZrcResult zrcTileCacheAddCylinderObstacle(ZrcTileCache* cache, const float* position,
                                          float radius, float height,
                                          ZrcObstacleRef* out_ref) {
  if (out_ref != nullptr) *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(position)) return ZRC_ERR_INVALID_ARGUMENT;
  for (int i = 0; i < 3; ++i) {
    if (position[i] < -zrc::kMaxCoordinate || position[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
  }
  if (!zrc::IsFinite(radius) || !(radius > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(height) || height < 0.f) return ZRC_ERR_INVALID_ARGUMENT;

  float bmin[3];
  float bmax[3];
  CylinderObstacleBounds(position, radius, height, bmin, bmax);
  const ZrcResult overlap_ok = CheckObstacleWontOverflowTiles(*cache->impl, bmin, bmax);
  if (overlap_ok != ZRC_OK) return overlap_ok;
  // dtTileCache::addObstacle returns DT_OUT_OF_MEMORY when the obstacle free
  // list is empty (DetourTileCache.cpp:371-372), which a host would read as
  // an allocator failure rather than the cache being full of obstacles it
  // was sized for. Counted here first, the way zrcNavMeshAddTile counts
  // resident tiles.
  if (ResidentObstacles(*cache->impl) >= cache->impl->getObstacleCount()) {
    return ZRC_ERR_NAVMESH_FULL;
  }

  dtObstacleRef ref = 0;
  const dtStatus status = cache->impl->addObstacle(position, radius, height, &ref);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (out_ref != nullptr) *out_ref = static_cast<ZrcObstacleRef>(ref);
  return ZRC_OK;
}

ZrcResult zrcTileCacheAddBoxObstacle(ZrcTileCache* cache, const float* bmin,
                                     const float* bmax, ZrcObstacleRef* out_ref) {
  if (out_ref != nullptr) *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(bmin) || !zrc::IsFiniteVec3(bmax)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (bmin[i] < -zrc::kMaxCoordinate || bmin[i] > zrc::kMaxCoordinate ||
        bmax[i] < -zrc::kMaxCoordinate || bmax[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (bmin[i] > bmax[i]) return ZRC_ERR_INVALID_ARGUMENT;
  }

  // getObstacleBounds for a box obstacle is the caller's own bmin/bmax,
  // unchanged (DetourTileCache.cpp:808-812).
  const ZrcResult overlap_ok = CheckObstacleWontOverflowTiles(*cache->impl, bmin, bmax);
  if (overlap_ok != ZRC_OK) return overlap_ok;
  if (ResidentObstacles(*cache->impl) >= cache->impl->getObstacleCount()) {
    return ZRC_ERR_NAVMESH_FULL;
  }

  dtObstacleRef ref = 0;
  const dtStatus status = cache->impl->addBoxObstacle(bmin, bmax, &ref);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (out_ref != nullptr) *out_ref = static_cast<ZrcObstacleRef>(ref);
  return ZRC_OK;
}

ZrcResult zrcTileCacheAddOrientedBoxObstacle(ZrcTileCache* cache, const float* center,
                                             const float* half_extents,
                                             float y_radians, ZrcObstacleRef* out_ref) {
  if (out_ref != nullptr) *out_ref = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(center) || !zrc::IsFiniteVec3(half_extents)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (center[i] < -zrc::kMaxCoordinate || center[i] > zrc::kMaxCoordinate) {
      return ZRC_ERR_INVALID_ARGUMENT;
    }
    if (half_extents[i] < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(y_radians)) return ZRC_ERR_INVALID_ARGUMENT;

  float bmin[3];
  float bmax[3];
  OrientedBoxObstacleBounds(center, half_extents, bmin, bmax);
  const ZrcResult overlap_ok = CheckObstacleWontOverflowTiles(*cache->impl, bmin, bmax);
  if (overlap_ok != ZRC_OK) return overlap_ok;
  if (ResidentObstacles(*cache->impl) >= cache->impl->getObstacleCount()) {
    return ZRC_ERR_NAVMESH_FULL;
  }

  dtObstacleRef ref = 0;
  const dtStatus status = cache->impl->addBoxObstacle(center, half_extents, y_radians, &ref);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (out_ref != nullptr) *out_ref = static_cast<ZrcObstacleRef>(ref);
  return ZRC_OK;
}

ZrcResult zrcTileCacheRemoveObstacle(ZrcTileCache* cache, ZrcObstacleRef ref) {
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  // Upstream's own removeObstacle treats a zero reference as "nothing to
  // remove" rather than an error (DetourTileCache.cpp:469-470); preserved
  // here rather than folded into the salt-protection check below, since a
  // caller safely re-calling this with a ref it never set is a working
  // pattern this package should not break.
  if (ref == 0) return ZRC_OK;
  if (ResidentObstacleByRef(*cache->impl, ref) == nullptr) return ZRC_ERR_NOT_FOUND;
  const dtStatus status = cache->impl->removeObstacle(static_cast<dtObstacleRef>(ref));
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

/// Upstream's own queryTiles increments its count only while it is below the
/// caller's own `maxResults` (DetourTileCache.cpp:482-521), so it reports
/// what fit rather than the true total and still returns DT_SUCCESS. Querying
/// first with a buffer sized to the cache's own tile capacity recovers the
/// true count, the same upper bound zrcTileCacheTilesAt uses.
ZrcResult zrcTileCacheQueryTiles(const ZrcTileCache* cache, const float* bmin,
                                 const float* bmax, ZrcCompressedTileRef* out,
                                 int32_t max_tiles, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (cache == nullptr || cache->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(bmin) || !zrc::IsFiniteVec3(bmax)) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_tiles < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (max_tiles > 0 && out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  dtTileCache& tc = *cache->impl;
  const int capacity = tc.getTileCount();
  zrc::TempBuffer scratch(sizeof(dtCompressedTileRef) * static_cast<size_t>(capacity));
  dtCompressedTileRef* all = static_cast<dtCompressedTileRef*>(scratch.get());
  if (capacity > 0 && all == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  int total = 0;
  const dtStatus status = tc.queryTiles(bmin, bmax, all, &total, capacity);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);

  const int32_t to_copy = total < max_tiles ? total : max_tiles;
  for (int32_t i = 0; i < to_copy; ++i) {
    out[i] = static_cast<ZrcCompressedTileRef>(all[i]);
  }
  *out_count = total;
  if (total > max_tiles) return ZRC_ERR_BUFFER_TOO_SMALL;
  return ZRC_OK;
}

ZrcResult zrcTileCacheUpdate(ZrcTileCache* cache, ZrcNavMesh* navmesh,
                             ZrcBool* out_up_to_date) {
  if (out_up_to_date != nullptr) *out_up_to_date = ZRC_FALSE;
  if (cache == nullptr || cache->impl == nullptr || navmesh == nullptr ||
      navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  cache->mesh_process.ClearLastResult();
  bool up_to_date = false;
  const dtStatus status = cache->impl->update(0.f, navmesh->impl, &up_to_date);
  if (out_up_to_date != nullptr) *out_up_to_date = up_to_date ? ZRC_TRUE : ZRC_FALSE;
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return cache->mesh_process.LastResult();
}

ZrcResult zrcTileCacheBuildNavMeshTile(ZrcTileCache* cache, ZrcCompressedTileRef ref,
                                       ZrcNavMesh* navmesh) {
  if (cache == nullptr || cache->impl == nullptr || navmesh == nullptr ||
      navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // buildNavMeshTile has `if (idx > (unsigned)m_params.maxTiles)`, one past
  // the array bound every other check in the file applies with `>=`
  // (DetourTileCache.cpp:662-663). Resolving through getTileByRef first —
  // which bounds correctly — keeps this call from ever reaching that off-by-
  // one with an out-of-range index.
  if (ResidentCompressedTileByRef(*cache->impl, ref) == nullptr) {
    return ZRC_ERR_NOT_FOUND;
  }
  cache->mesh_process.ClearLastResult();
  const dtStatus status =
      cache->impl->buildNavMeshTile(static_cast<dtCompressedTileRef>(ref), navmesh->impl);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return cache->mesh_process.LastResult();
}

ZrcResult zrcTileCacheBuildNavMeshTilesAt(ZrcTileCache* cache, int32_t tile_x,
                                          int32_t tile_y, ZrcNavMesh* navmesh) {
  if (cache == nullptr || cache->impl == nullptr || navmesh == nullptr ||
      navmesh->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // Every ref this drives comes from upstream's own getTilesAt at (tile_x,
  // tile_y), not from a caller, so the off-by-one buildNavMeshTile guards
  // above does not apply here.
  cache->mesh_process.ClearLastResult();
  const dtStatus status = cache->impl->buildNavMeshTilesAt(tile_x, tile_y, navmesh->impl);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return cache->mesh_process.LastResult();
}

//===----------------------------------------------------------------------===//
// Building a compressed layer, and taking one apart
//===----------------------------------------------------------------------===//

ZrcResult zrcTileCacheLayerBuild(const ZrcTileCacheCompressor* compressor,
                                 const ZrcTileCacheLayerHeader* header,
                                 const uint8_t* heights, const uint8_t* areas,
                                 const uint8_t* cons, void** out_data,
                                 size_t* out_size) {
  if (out_data == nullptr || out_size == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_data = nullptr;
  *out_size = 0;
  const ZrcResult compressor_ok = ValidateCompressorHooks(compressor);
  if (compressor_ok != ZRC_OK) return compressor_ok;
  if (header == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const ZrcResult header_ok = ValidateLayerHeader(*header);
  if (header_ok != ZRC_OK) return header_ok;
  if (heights == nullptr || areas == nullptr || cons == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  dtTileCacheLayerHeader native;
  memset(&native, 0, sizeof(native));
  native.magic = DT_TILECACHE_MAGIC;
  native.version = DT_TILECACHE_VERSION;
  native.tx = header->tile_x;
  native.ty = header->tile_y;
  native.tlayer = header->tile_layer;
  memcpy(native.bmin, header->bmin, sizeof(native.bmin));
  memcpy(native.bmax, header->bmax, sizeof(native.bmax));
  native.hmin = static_cast<unsigned short>(header->height_min);
  native.hmax = static_cast<unsigned short>(header->height_max);
  native.width = static_cast<unsigned char>(header->width);
  native.height = static_cast<unsigned char>(header->height);
  native.minx = static_cast<unsigned char>(header->min_x);
  native.maxx = static_cast<unsigned char>(header->max_x);
  native.miny = static_cast<unsigned char>(header->min_z);
  native.maxy = static_cast<unsigned char>(header->max_z);

  zrc::HostCompressor comp(*compressor);
  unsigned char* data = nullptr;
  int data_size = 0;
  const dtStatus status =
      dtBuildTileCacheLayer(&comp, &native, heights, areas, cons, &data, &data_size);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  if (data == nullptr || data_size <= 0) return ZRC_ERR_OUT_OF_MEMORY;

  *out_data = data;
  *out_size = static_cast<size_t>(data_size);
  return ZRC_OK;
}

/// Upstream takes a length and discards it with dtIgnoreUnused before
/// dereferencing the header (DetourTileCacheBuilder.cpp:2223-2226); the
/// length is checked here first.
ZrcResult zrcTileCacheHeaderSwapEndian(void* data, size_t size) {
  if (data == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (size > static_cast<size_t>(0x7fffffff)) return ZRC_ERR_INVALID_ARGUMENT;
  if (size < sizeof(dtTileCacheLayerHeader)) return ZRC_ERR_BAD_FORMAT;
  const bool swapped = dtTileCacheHeaderSwapEndian(static_cast<unsigned char*>(data),
                                                    static_cast<int>(size));
  if (!swapped) return ZRC_ERR_BAD_FORMAT;
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerDecompress(const ZrcTileCacheCompressor* compressor,
                                      const ZrcTileCacheAllocator* allocator,
                                      const void* data, size_t size,
                                      ZrcTileCacheLayer** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult compressor_ok = ValidateCompressorHooks(compressor);
  if (compressor_ok != ZRC_OK) return compressor_ok;
  const ZrcResult allocator_ok = ValidateAllocatorHooks(allocator);
  if (allocator_ok != ZRC_OK) return allocator_ok;
  if (data == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (size > static_cast<size_t>(0x7fffffff)) return ZRC_ERR_BAD_FORMAT;
  const size_t header_size =
      static_cast<size_t>(dtAlign4(static_cast<int>(sizeof(dtTileCacheLayerHeader))));
  if (size < header_size) return ZRC_ERR_BAD_FORMAT;

  dtTileCacheLayerHeader header;
  memcpy(&header, data, sizeof(header));
  if (header.magic != DT_TILECACHE_MAGIC) return ZRC_ERR_BAD_FORMAT;
  if (header.version != DT_TILECACHE_VERSION) return ZRC_ERR_UNSUPPORTED_VERSION;

  void* block = zrc::Alloc(sizeof(ZrcTileCacheLayer), DT_ALLOC_PERM);
  if (block == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  ZrcTileCacheLayer* handle =
      new (block) ZrcTileCacheLayer{nullptr, zrc::HostTileCacheAlloc(allocator)};

  zrc::HostCompressor comp(*compressor);
  dtTileCacheLayer* layer = nullptr;
  // dtDecompressTileCacheLayer only reads through `compressed`
  // (DetourTileCacheBuilder.cpp:2163-2219); its declared parameter is
  // non-const regardless, so this reaches the same well-defined situation
  // zrcNavMeshRestoreTileState documents for its own const_cast.
  const dtStatus status = dtDecompressTileCacheLayer(
      &handle->allocator, &comp,
      static_cast<unsigned char*>(const_cast<void*>(data)), static_cast<int>(size), &layer);
  if (dtStatusFailed(status)) {
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }
  handle->impl = layer;
  *out = handle;
  return ZRC_OK;
}

/// dtFreeTileCacheLayer, unlike its contour-set and poly-mesh siblings, has
/// no null check of its own before calling through to the allocator
/// (DetourTileCacheBuilder.cpp:2156-2161: `alloc->free(layer);` with no
/// `if (!layer) return;` first) — so a null impl is never handed to it here.
void zrcTileCacheLayerDestroy(ZrcTileCacheLayer* layer) {
  if (layer == nullptr) return;
  if (layer->impl != nullptr) dtFreeTileCacheLayer(&layer->allocator, layer->impl);
  zrc::Delete(layer);
}

ZrcResult zrcTileCacheLayerHeaderOf(const ZrcTileCacheLayer* layer,
                                    ZrcTileCacheLayerHeader* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (layer == nullptr || layer->impl == nullptr || layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const dtTileCacheLayerHeader& h = *layer->impl->header;
  out->tile_x = h.tx;
  out->tile_y = h.ty;
  out->tile_layer = h.tlayer;
  memcpy(out->bmin, h.bmin, sizeof(out->bmin));
  memcpy(out->bmax, h.bmax, sizeof(out->bmax));
  out->height_min = h.hmin;
  out->height_max = h.hmax;
  out->width = h.width;
  out->height = h.height;
  out->min_x = h.minx;
  out->max_x = h.maxx;
  out->min_z = h.miny;
  out->max_z = h.maxy;
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerRegionCount(const ZrcTileCacheLayer* layer, int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (layer == nullptr || layer->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = layer->impl->regCount;
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerHeights(const ZrcTileCacheLayer* layer, int32_t first,
                                   int32_t count, uint8_t* out) {
  if (layer == nullptr || layer->impl == nullptr || layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const int32_t length = LayerGridLength(*layer->impl);
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer->impl->heights == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, layer->impl->heights + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerAreas(const ZrcTileCacheLayer* layer, int32_t first,
                                 int32_t count, uint8_t* out) {
  if (layer == nullptr || layer->impl == nullptr || layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const int32_t length = LayerGridLength(*layer->impl);
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer->impl->areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, layer->impl->areas + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerSetAreas(ZrcTileCacheLayer* layer, int32_t first,
                                    int32_t count, const uint8_t* areas) {
  if (layer == nullptr || layer->impl == nullptr || layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const int32_t length = LayerGridLength(*layer->impl);
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (areas == nullptr || layer->impl->areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  for (int32_t i = 0; i < count; ++i) {
    if (areas[i] >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;
  }
  memcpy(layer->impl->areas + first, areas, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerCons(const ZrcTileCacheLayer* layer, int32_t first,
                                int32_t count, uint8_t* out) {
  if (layer == nullptr || layer->impl == nullptr || layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const int32_t length = LayerGridLength(*layer->impl);
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer->impl->cons == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, layer->impl->cons + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

/// regCount is 0 both before zrcTileCacheLayerBuildRegions has run and for a
/// layer that legitimately has no walkable cells — the same ambiguity
/// zrcTileCacheLayerRegionCount's own doc comment describes ("or 0 before it
/// runs"). That is the signal the header documents, so it is the one used
/// here; dtTileCacheLayer carries no separate built/not-built flag.
ZrcResult zrcTileCacheLayerRegions(const ZrcTileCacheLayer* layer, int32_t first,
                                   int32_t count, uint8_t* out) {
  if (layer == nullptr || layer->impl == nullptr || layer->impl->header == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (layer->impl->regCount == 0) return ZRC_ERR_NOT_FOUND;
  const int32_t length = LayerGridLength(*layer->impl);
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer->impl->regs == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, layer->impl->regs + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerMarkCylinder(ZrcTileCacheLayer* layer, const float* origin,
                                        float cell_size, float cell_height,
                                        const float* position, float radius,
                                        float height, uint8_t area) {
  if (layer == nullptr || layer->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(origin) || !zrc::IsFiniteVec3(position)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cell_size) || !(cell_size > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(cell_height) || !(cell_height > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(radius) || !(radius > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(height) || height < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  if (area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;

  const dtStatus status = dtMarkCylinderArea(*layer->impl, origin, cell_size, cell_height,
                                             position, radius, height, area);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerMarkBox(ZrcTileCacheLayer* layer, const float* origin,
                                   float cell_size, float cell_height,
                                   const float* bmin, const float* bmax, uint8_t area) {
  if (layer == nullptr || layer->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(origin) || !zrc::IsFiniteVec3(bmin) || !zrc::IsFiniteVec3(bmax)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (bmin[i] > bmax[i]) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cell_size) || !(cell_size > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(cell_height) || !(cell_height > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;

  const dtStatus status =
      dtMarkBoxArea(*layer->impl, origin, cell_size, cell_height, bmin, bmax, area);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerMarkOrientedBox(ZrcTileCacheLayer* layer, const float* origin,
                                           float cell_size, float cell_height,
                                           const float* center, const float* half_extents,
                                           float y_radians, uint8_t area) {
  if (layer == nullptr || layer->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFiniteVec3(origin) || !zrc::IsFiniteVec3(center) ||
      !zrc::IsFiniteVec3(half_extents)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  for (int i = 0; i < 3; ++i) {
    if (half_extents[i] < 0.f) return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (!zrc::IsFinite(cell_size) || !(cell_size > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(cell_height) || !(cell_height > 0.f)) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(y_radians)) return ZRC_ERR_INVALID_ARGUMENT;
  if (area >= ZRC_MAX_AREAS) return ZRC_ERR_INVALID_ARGUMENT;

  // Same identity as zrcTileCacheObstacleInfo, forward rather than inverted:
  // dtTileCache::addBoxObstacle computes rotAux this way from an angle
  // (DetourTileCache.cpp:451-454); dtMarkBoxArea's oriented overload takes
  // rotAux directly rather than an angle, so it is computed here to match.
  const float coshalf = cosf(0.5f * y_radians);
  const float sinhalf = sinf(-0.5f * y_radians);
  const float rot_aux[2] = {coshalf * sinhalf, coshalf * coshalf - 0.5f};

  const dtStatus status = dtMarkBoxArea(*layer->impl, origin, cell_size, cell_height,
                                        center, half_extents, rot_aux, area);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

ZrcResult zrcTileCacheLayerBuildRegions(ZrcTileCacheLayer* layer, int32_t walkable_climb) {
  if (layer == nullptr || layer->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (walkable_climb < 0) return ZRC_ERR_INVALID_ARGUMENT;
  const dtStatus status =
      dtBuildTileCacheRegions(&layer->allocator, *layer->impl, walkable_climb);
  if (dtStatusFailed(status)) return zrc::ResultFromStatus(status);
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// Contours
//===----------------------------------------------------------------------===//

ZrcResult zrcTileCacheContourSetCreate(const ZrcTileCacheAllocator* allocator,
                                       ZrcTileCacheLayer* layer, int32_t walkable_climb,
                                       float max_error, ZrcTileCacheContourSet** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult allocator_ok = ValidateAllocatorHooks(allocator);
  if (allocator_ok != ZRC_OK) return allocator_ok;
  if (layer == nullptr || layer->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (walkable_climb < 0) return ZRC_ERR_INVALID_ARGUMENT;
  if (!zrc::IsFinite(max_error) || max_error < 0.f) return ZRC_ERR_INVALID_ARGUMENT;

  void* block = zrc::Alloc(sizeof(ZrcTileCacheContourSet), DT_ALLOC_PERM);
  if (block == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  ZrcTileCacheContourSet* handle =
      new (block) ZrcTileCacheContourSet{nullptr, zrc::HostTileCacheAlloc(allocator)};

  dtTileCacheContourSet* cset = dtAllocTileCacheContourSet(&handle->allocator);
  if (cset == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  const dtStatus status =
      dtBuildTileCacheContours(&handle->allocator, *layer->impl, walkable_climb, max_error, *cset);
  if (dtStatusFailed(status)) {
    dtFreeTileCacheContourSet(&handle->allocator, cset);
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }
  handle->impl = cset;
  *out = handle;
  return ZRC_OK;
}

void zrcTileCacheContourSetDestroy(ZrcTileCacheContourSet* contours) {
  if (contours == nullptr) return;
  dtFreeTileCacheContourSet(&contours->allocator, contours->impl);
  zrc::Delete(contours);
}

ZrcResult zrcTileCacheContourSetCount(const ZrcTileCacheContourSet* contours,
                                      int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (contours == nullptr || contours->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = contours->impl->nconts;
  return ZRC_OK;
}

ZrcResult zrcTileCacheContourAt(const ZrcTileCacheContourSet* contours, int32_t index,
                                ZrcTileCacheContourInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (contours == nullptr || contours->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheContourSet& cset = *contours->impl;
  if (index < 0 || index >= cset.nconts) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheContour& c = cset.conts[index];
  out->vert_count = c.nverts;
  out->region = c.reg;
  out->area = c.area;
  return ZRC_OK;
}

ZrcResult zrcTileCacheContourVerts(const ZrcTileCacheContourSet* contours, int32_t index,
                                   int32_t first, int32_t count, uint8_t* out) {
  if (contours == nullptr || contours->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheContourSet& cset = *contours->impl;
  if (index < 0 || index >= cset.nconts) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCacheContour& c = cset.conts[index];
  const ZrcResult range_ok = zrc::CheckRange(first, count, c.nverts);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || c.verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, c.verts + static_cast<int64_t>(first) * 4,
         sizeof(uint8_t) * 4 * static_cast<size_t>(count));
  return ZRC_OK;
}

//===----------------------------------------------------------------------===//
// The tile-cache polygon mesh
//===----------------------------------------------------------------------===//

ZrcResult zrcTileCachePolyMeshCreate(const ZrcTileCacheAllocator* allocator,
                                     const ZrcTileCacheContourSet* contours,
                                     ZrcTileCachePolyMesh** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult allocator_ok = ValidateAllocatorHooks(allocator);
  if (allocator_ok != ZRC_OK) return allocator_ok;
  if (contours == nullptr || contours->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  void* block = zrc::Alloc(sizeof(ZrcTileCachePolyMesh), DT_ALLOC_PERM);
  if (block == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  ZrcTileCachePolyMesh* handle =
      new (block) ZrcTileCachePolyMesh{nullptr, zrc::HostTileCacheAlloc(allocator)};

  dtTileCachePolyMesh* mesh = dtAllocTileCachePolyMesh(&handle->allocator);
  if (mesh == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  // contours->impl is a plain (non-const) pointer member, so dereferencing it
  // through the caller's const ZrcTileCacheContourSet* needs no cast: only
  // the field is read-only here, not what it points to.
  const dtStatus status = dtBuildTileCachePolyMesh(&handle->allocator, *contours->impl, *mesh);
  if (dtStatusFailed(status)) {
    dtFreeTileCachePolyMesh(&handle->allocator, mesh);
    zrc::Delete(handle);
    return zrc::ResultFromStatus(status);
  }
  handle->impl = mesh;
  *out = handle;
  return ZRC_OK;
}

void zrcTileCachePolyMeshDestroy(ZrcTileCachePolyMesh* mesh) {
  if (mesh == nullptr) return;
  dtFreeTileCachePolyMesh(&mesh->allocator, mesh->impl);
  zrc::Delete(mesh);
}

ZrcResult zrcTileCachePolyMeshInfo(const ZrcTileCachePolyMesh* mesh,
                                   ZrcTileCachePolyMeshInfo* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  out->vert_count = mesh->impl->nverts;
  out->poly_count = mesh->impl->npolys;
  out->verts_per_poly = mesh->impl->nvp;
  return ZRC_OK;
}

ZrcResult zrcTileCachePolyMeshVerts(const ZrcTileCachePolyMesh* mesh, int32_t first,
                                    int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCachePolyMesh& m = *mesh->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, m.nverts);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || m.verts == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, m.verts + static_cast<int64_t>(first) * 3,
         sizeof(uint16_t) * 3 * static_cast<size_t>(count));
  return ZRC_OK;
}

/// Bounded against npolys rather than any larger allocated capacity:
/// dtBuildTileCachePolyMesh sizes mesh.polys for the worst-case triangle
/// count before merging (DetourTileCacheBuilder.cpp:1779), so the array often
/// has room past npolys holding nothing but the initial 0xff fill.
ZrcResult zrcTileCachePolyMeshPolys(const ZrcTileCachePolyMesh* mesh, int32_t first,
                                    int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCachePolyMesh& m = *mesh->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, m.npolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || m.polys == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const int32_t stride = 2 * m.nvp;
  memcpy(out, m.polys + static_cast<int64_t>(first) * stride,
         sizeof(uint16_t) * static_cast<size_t>(stride) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcTileCachePolyMeshAreas(const ZrcTileCachePolyMesh* mesh, int32_t first,
                                    int32_t count, uint8_t* out) {
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCachePolyMesh& m = *mesh->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, m.npolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || m.areas == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, m.areas + first, sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcTileCachePolyMeshFlags(const ZrcTileCachePolyMesh* mesh, int32_t first,
                                    int32_t count, uint16_t* out) {
  if (mesh == nullptr || mesh->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  const dtTileCachePolyMesh& m = *mesh->impl;
  const ZrcResult range_ok = zrc::CheckRange(first, count, m.npolys);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || m.flags == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memcpy(out, m.flags + first, sizeof(uint16_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

}  // extern "C"
