//===----------------------------------------------------------------------===//
// zrecast — the layered heightfield: a compact heightfield cut into
// separately compressible sheets, and every accessor onto them.
//
// This file owns the lifetime of the layer-set handle — allocation fused
// with the build that fills it, the shape ffi/zrecast_pipeline.cpp uses for
// its four containers — and the range accessors onto a layer's parallel
// height, area and connection arrays. rcBuildHeightfieldLayers is the only
// upstream function called here to fill a container, at the moment it is
// created.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Argument validation
//===----------------------------------------------------------------------===//

ZrcResult ValidateHeightfieldLayerSetCreateArgs(int32_t border_size,
                                                int32_t walkable_height) {
  if (border_size < 0 || border_size > zrc::kMaxBorderSizeCells) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  // walkableHeight is scaled by 4 into an unsigned short merge threshold
  // (RecastLayers.cpp:388); ZRC_SPAN_MAX_HEIGHT keeps that product well
  // inside range and matches the ceiling a span's own vertical extent
  // already carries elsewhere in this API.
  if (walkable_height < 0 || walkable_height > ZRC_SPAN_MAX_HEIGHT) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// The layered heightfield
//===----------------------------------------------------------------------===//

ZrcResult zrcHeightfieldLayerSetCreate(const ZrcBuildContext* context,
                                       const ZrcCompactHeightfield* field,
                                       int32_t border_size,
                                       int32_t walkable_height,
                                       ZrcHeightfieldLayerSet** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (field == nullptr || field->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const ZrcResult args_ok =
      ValidateHeightfieldLayerSetCreateArgs(border_size, walkable_height);
  if (args_ok != ZRC_OK) return args_ok;

  // RecastLayers.cpp:497-498 computes lw = chf.width - borderSize*2 and
  // lh = chf.height - borderSize*2 in plain int, with no bound of their own.
  // A border at least half of either extent drives both negative; every
  // write loop below is bounded by lw/lh so nothing is written out of
  // bounds, but the gridSize allocation at RecastLayers.cpp:527 takes their
  // product, and a doubly-negative pair multiplies back to a positive,
  // nonsensical grid size rather than failing.
  if (border_size * 2 >= field->impl->width ||
      border_size * 2 >= field->impl->height) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  rcHeightfieldLayerSet* impl = zrc::RcNew<rcHeightfieldLayerSet>();
  if (impl == nullptr) return ZRC_ERR_OUT_OF_MEMORY;

  zrc::HostContext ctx(context);
  if (!rcBuildHeightfieldLayers(&ctx, *field->impl, border_size,
                                walkable_height, *impl)) {
    zrc::RcFree(impl);
    // rcBuildHeightfieldLayers mixes allocation failure across seven
    // separate rcAlloc calls (RecastLayers.cpp:116-128, 236-239, 512-549)
    // with region-id overflow (RecastLayers.cpp:212-216) and the
    // 63-overlapping-platform layer overflow the header names
    // (RecastLayers.cpp:303-308, 372-377, 456-461).
    // zrcCompactHeightfieldBuildRegions in ffi/zrecast_stages.cpp maps the
    // same kind of mixed false, from rcBuildRegions and its siblings, to
    // ZRC_ERR_BAKE_FAILED without separating the causes; this follows it.
    return ZRC_ERR_BAKE_FAILED;
  }

  ZrcHeightfieldLayerSet* handle = zrc::New<ZrcHeightfieldLayerSet>();
  if (handle == nullptr) {
    zrc::RcFree(impl);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  handle->impl = impl;
  *out = handle;
  return ZRC_OK;
}

void zrcHeightfieldLayerSetDestroy(ZrcHeightfieldLayerSet* layers) {
  if (layers == nullptr) return;
  zrc::RcFree(layers->impl);
  zrc::Delete(layers);
}

ZrcResult zrcHeightfieldLayerSetCount(const ZrcHeightfieldLayerSet* layers,
                                      int32_t* out_count) {
  if (out_count == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out_count = 0;
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  *out_count = layers->impl->nlayers;
  return ZRC_OK;
}

ZrcResult zrcHeightfieldLayerAt(const ZrcHeightfieldLayerSet* layers,
                                int32_t index, ZrcHeightfieldLayer* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  memset(out, 0, sizeof(*out));
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcHeightfieldLayerSet& lset = *layers->impl;
  if (index < 0 || index >= lset.nlayers) return ZRC_ERR_INVALID_ARGUMENT;
  if (lset.layers == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const rcHeightfieldLayer& layer = lset.layers[index];
  for (int i = 0; i < 3; ++i) {
    out->bmin[i] = layer.bmin[i];
    out->bmax[i] = layer.bmax[i];
  }
  out->cell_size = layer.cs;
  out->cell_height = layer.ch;
  out->width = layer.width;
  out->height = layer.height;
  out->min_x = layer.minx;
  out->max_x = layer.maxx;
  // Upstream spells these miny/maxy while meaning the z axis; zrecast.h
  // documents the rename onto ZrcHeightfieldLayer::min_z / max_z.
  out->min_z = layer.miny;
  out->max_z = layer.maxy;
  out->height_min = layer.hmin;
  out->height_max = layer.hmax;
  return ZRC_OK;
}

ZrcResult zrcHeightfieldLayerHeights(const ZrcHeightfieldLayerSet* layers,
                                     int32_t index, int32_t first,
                                     int32_t count, uint8_t* out) {
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcHeightfieldLayerSet& lset = *layers->impl;
  if (index < 0 || index >= lset.nlayers) return ZRC_ERR_INVALID_ARGUMENT;
  if (lset.layers == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const rcHeightfieldLayer& layer = lset.layers[index];
  // Upstream's own idx = x + y*lw (RecastLayers.cpp:609). The product fits
  // int because width and height are a compact heightfield's own extent,
  // itself bounded by zrc::CheckGridExtentFit, shrunk only by border_size.
  const int32_t length = layer.width * layer.height;
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer.heights == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(out, layer.heights + first,
        sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcHeightfieldLayerAreas(const ZrcHeightfieldLayerSet* layers,
                                   int32_t index, int32_t first,
                                   int32_t count, uint8_t* out) {
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcHeightfieldLayerSet& lset = *layers->impl;
  if (index < 0 || index >= lset.nlayers) return ZRC_ERR_INVALID_ARGUMENT;
  if (lset.layers == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const rcHeightfieldLayer& layer = lset.layers[index];
  // Same idx = x + y*lw indexing rcBuildHeightfieldLayers writes areas
  // through (RecastLayers.cpp:611), and the same array length as heights.
  const int32_t length = layer.width * layer.height;
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer.areas == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(out, layer.areas + first,
        sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

ZrcResult zrcHeightfieldLayerCons(const ZrcHeightfieldLayerSet* layers,
                                  int32_t index, int32_t first,
                                  int32_t count, uint8_t* out) {
  if (layers == nullptr || layers->impl == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  const rcHeightfieldLayerSet& lset = *layers->impl;
  if (index < 0 || index >= lset.nlayers) return ZRC_ERR_INVALID_ARGUMENT;
  if (lset.layers == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  const rcHeightfieldLayer& layer = lset.layers[index];
  // Same idx = x + y*lw indexing rcBuildHeightfieldLayers writes the packed
  // connection byte through (RecastLayers.cpp:644), and the same array
  // length as heights and areas.
  const int32_t length = layer.width * layer.height;
  const ZrcResult range_ok = zrc::CheckRange(first, count, length);
  if (range_ok != ZRC_OK) return range_ok;
  if (count == 0) return ZRC_OK;
  if (out == nullptr || layer.cons == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  memcpy(out, layer.cons + first,
        sizeof(uint8_t) * static_cast<size_t>(count));
  return ZRC_OK;
}

}  // extern "C"
