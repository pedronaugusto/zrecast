//===----------------------------------------------------------------------===//
// zrecast — C-level smoke test.
//
// Proves the boundary in ffi/zrecast.h stands on its own as a C contract: it
// compiles as C11, links without libc++ symbols leaking into the caller, and
// behaves correctly with a host allocator that knows nothing about Zig.
//
// It walks the whole arc a real host walks — bake, build, serialise, reload,
// query — with a plain malloc allocator, and finishes by asserting that every
// byte taken from that allocator was given back.
//
// Deliberately dependency-free: no test framework, no asset files.
//===----------------------------------------------------------------------===//

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fixture.h"
#include "zrecast.h"

static int failures = 0;

#define CHECK(cond)                                                   \
  do {                                                                \
    if (!(cond)) {                                                    \
      fprintf(stderr, "%s:%d: FAIL %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                     \
    }                                                                 \
  } while (0)

//===----------------------------------------------------------------------===//
// A host allocator that counts, so the seam is proven to actually be in use
// rather than silently bypassed.
//===----------------------------------------------------------------------===//

typedef struct Counters {
  size_t allocations;
  size_t frees;
} Counters;

static void* count_allocate(void* user, size_t size, ZrcAllocHint hint) {
  /* The hint is upstream's; a real host might route TEMP into an arena. */
  (void)hint;
  Counters* counters = (Counters*)user;

  /* Reserve room for the base pointer, then round the payload up to the
     alignment the header promises. Mirrors what a host on a runtime without
     aligned_alloc has to do. */
  const size_t prefix =
      ZRC_ALLOC_ALIGNMENT > sizeof(void*) ? ZRC_ALLOC_ALIGNMENT : sizeof(void*);
  void* base = malloc(prefix + size + ZRC_ALLOC_ALIGNMENT);
  if (base == NULL) return NULL;

  uintptr_t raw = (uintptr_t)base + prefix;
  uintptr_t aligned =
      (raw + (ZRC_ALLOC_ALIGNMENT - 1)) & ~(uintptr_t)(ZRC_ALLOC_ALIGNMENT - 1);
  char* payload = (char*)aligned;
  memcpy(payload - sizeof(void*), &base, sizeof(void*));

  ++counters->allocations;
  return payload;
}

static void count_deallocate(void* user, void* block) {
  if (block == NULL) return;
  Counters* counters = (Counters*)user;
  ++counters->frees;

  void* base = NULL;
  memcpy(&base, (char*)block - sizeof(void*), sizeof(void*));
  free(base);
}

//===----------------------------------------------------------------------===//

static void test_version(void) {
  const uint32_t v = zrcVersion();
  CHECK(((v >> 16) & 0xFF) == ZRC_VERSION_MAJOR);
  CHECK(((v >> 8) & 0xFF) == ZRC_VERSION_MINOR);
  CHECK((v & 0xFF) == ZRC_VERSION_PATCH);
  CHECK(zrcRecastVersion() != 0);
  CHECK(zrcNavMeshDataVersion() > 0);
}

static void test_result_names(void) {
  for (int i = 0; i <= (int)ZRC_ERR_OUT_OF_NODES; ++i) {
    const char* name = zrcResultName((ZrcResult)i);
    CHECK(name != NULL);
    CHECK(strlen(name) > 0);
  }
}

static void test_abi_layout(void) {
  ZrcAbiLayout layout;
  memset(&layout, 0, sizeof(layout));
  zrcAbiLayout(&layout);

  CHECK(layout.layout_size == (uint32_t)sizeof(ZrcAbiLayout));
  CHECK(layout.trimesh_size == (uint32_t)sizeof(ZrcTriMesh));
  CHECK(layout.bake_config_size == (uint32_t)sizeof(ZrcBakeConfig));
  CHECK(layout.query_filter_size == (uint32_t)sizeof(ZrcQueryFilter));
  CHECK(layout.raycast_hit_size == (uint32_t)sizeof(ZrcRaycastHit));
  CHECK(layout.allocator_size == (uint32_t)sizeof(ZrcAllocator));
  CHECK(layout.tile_grid_size == (uint32_t)sizeof(ZrcTileGrid));
  CHECK(layout.tile_grid_align == (uint32_t)_Alignof(ZrcTileGrid));
  CHECK(layout.tile_grid_offset_origin == (uint32_t)offsetof(ZrcTileGrid, origin));
  CHECK(layout.tile_grid_offset_extent_max ==
        (uint32_t)offsetof(ZrcTileGrid, extent_max));
  CHECK(layout.tile_grid_offset_tile_world_size ==
        (uint32_t)offsetof(ZrcTileGrid, tile_world_size));
  CHECK(layout.tile_grid_offset_tile_count_x ==
        (uint32_t)offsetof(ZrcTileGrid, tile_count_x));
  CHECK(layout.tile_grid_offset_tile_count_z ==
        (uint32_t)offsetof(ZrcTileGrid, tile_count_z));
  CHECK(layout.area_volume_size == (uint32_t)sizeof(ZrcAreaVolume));
  CHECK(layout.area_volume_align == (uint32_t)_Alignof(ZrcAreaVolume));
  CHECK(layout.area_volume_offset_shape == (uint32_t)offsetof(ZrcAreaVolume, shape));
  CHECK(layout.area_volume_offset_area == (uint32_t)offsetof(ZrcAreaVolume, area));
  CHECK(layout.area_volume_offset_y_min == (uint32_t)offsetof(ZrcAreaVolume, y_min));
  CHECK(layout.area_volume_offset_y_max == (uint32_t)offsetof(ZrcAreaVolume, y_max));
  CHECK(layout.area_volume_offset_verts == (uint32_t)offsetof(ZrcAreaVolume, verts));
  CHECK(layout.area_volume_offset_vert_count ==
        (uint32_t)offsetof(ZrcAreaVolume, vert_count));
  CHECK(layout.area_volume_offset_xz_min == (uint32_t)offsetof(ZrcAreaVolume, xz_min));
  CHECK(layout.area_volume_offset_xz_max == (uint32_t)offsetof(ZrcAreaVolume, xz_max));
  CHECK(layout.area_volume_offset_radius == (uint32_t)offsetof(ZrcAreaVolume, radius));
  CHECK(layout.area_authoring_size == (uint32_t)sizeof(ZrcAreaAuthoring));
  CHECK(layout.area_authoring_align == (uint32_t)_Alignof(ZrcAreaAuthoring));
  CHECK(layout.area_authoring_offset_volumes ==
        (uint32_t)offsetof(ZrcAreaAuthoring, volumes));
  CHECK(layout.area_authoring_offset_volume_count ==
        (uint32_t)offsetof(ZrcAreaAuthoring, volume_count));
  CHECK(layout.area_authoring_offset_area_flags ==
        (uint32_t)offsetof(ZrcAreaAuthoring, area_flags));
  CHECK(layout.off_mesh_connection_size == (uint32_t)sizeof(ZrcOffMeshConnection));
  CHECK(layout.off_mesh_connection_align ==
        (uint32_t)_Alignof(ZrcOffMeshConnection));
  CHECK(layout.off_mesh_connection_offset_start ==
        (uint32_t)offsetof(ZrcOffMeshConnection, start));
  CHECK(layout.off_mesh_connection_offset_end ==
        (uint32_t)offsetof(ZrcOffMeshConnection, end));
  CHECK(layout.off_mesh_connection_offset_radius ==
        (uint32_t)offsetof(ZrcOffMeshConnection, radius));
  CHECK(layout.off_mesh_connection_offset_area ==
        (uint32_t)offsetof(ZrcOffMeshConnection, area));
  CHECK(layout.off_mesh_connection_offset_flags ==
        (uint32_t)offsetof(ZrcOffMeshConnection, flags));
  CHECK(layout.off_mesh_connection_offset_bidirectional ==
        (uint32_t)offsetof(ZrcOffMeshConnection, bidirectional));
  CHECK(layout.off_mesh_connection_offset_user_id ==
        (uint32_t)offsetof(ZrcOffMeshConnection, user_id));
  CHECK(layout.off_mesh_connection_offset_end_side ==
        (uint32_t)offsetof(ZrcOffMeshConnection, end_side));
  CHECK(layout.tile_authoring_size == (uint32_t)sizeof(ZrcTileAuthoring));
  CHECK(layout.tile_authoring_align == (uint32_t)_Alignof(ZrcTileAuthoring));
  CHECK(layout.tile_authoring_offset_connections ==
        (uint32_t)offsetof(ZrcTileAuthoring, connections));
  CHECK(layout.tile_authoring_offset_connection_count ==
        (uint32_t)offsetof(ZrcTileAuthoring, connection_count));
  CHECK(layout.tile_authoring_offset_user_id ==
        (uint32_t)offsetof(ZrcTileAuthoring, user_id));
  CHECK(layout.tile_authoring_offset_skip_bv_tree ==
        (uint32_t)offsetof(ZrcTileAuthoring, skip_bv_tree));
  CHECK(layout.nav_mesh_params_size == (uint32_t)sizeof(ZrcNavMeshParams));
  CHECK(layout.nav_mesh_params_align == (uint32_t)_Alignof(ZrcNavMeshParams));
  CHECK(layout.nav_mesh_params_offset_origin ==
        (uint32_t)offsetof(ZrcNavMeshParams, origin));
  CHECK(layout.nav_mesh_params_offset_tile_width ==
        (uint32_t)offsetof(ZrcNavMeshParams, tile_width));
  CHECK(layout.nav_mesh_params_offset_tile_height ==
        (uint32_t)offsetof(ZrcNavMeshParams, tile_height));
  CHECK(layout.nav_mesh_params_offset_max_tiles ==
        (uint32_t)offsetof(ZrcNavMeshParams, max_tiles));
  CHECK(layout.nav_mesh_params_offset_max_polys ==
        (uint32_t)offsetof(ZrcNavMeshParams, max_polys));
  CHECK(layout.tile_info_size == (uint32_t)sizeof(ZrcTileInfo));
  CHECK(layout.tile_info_align == (uint32_t)_Alignof(ZrcTileInfo));
  CHECK(layout.poly_info_size == (uint32_t)sizeof(ZrcPolyInfo));
  CHECK(layout.poly_info_align == (uint32_t)_Alignof(ZrcPolyInfo));
  CHECK(layout.link_size == (uint32_t)sizeof(ZrcLink));
  CHECK(layout.link_align == (uint32_t)_Alignof(ZrcLink));
  CHECK(layout.link_offset_ref == (uint32_t)offsetof(ZrcLink, ref));
  CHECK(layout.link_offset_next == (uint32_t)offsetof(ZrcLink, next));
  CHECK(layout.link_offset_edge == (uint32_t)offsetof(ZrcLink, edge));
  CHECK(layout.link_offset_side == (uint32_t)offsetof(ZrcLink, side));
  CHECK(layout.link_offset_bmin == (uint32_t)offsetof(ZrcLink, bmin));
  CHECK(layout.link_offset_bmax == (uint32_t)offsetof(ZrcLink, bmax));
  CHECK(layout.bv_node_size == (uint32_t)sizeof(ZrcBvNode));
  CHECK(layout.bv_node_align == (uint32_t)_Alignof(ZrcBvNode));
  CHECK(layout.bv_node_offset_bmin == (uint32_t)offsetof(ZrcBvNode, bmin));
  CHECK(layout.bv_node_offset_bmax == (uint32_t)offsetof(ZrcBvNode, bmax));
  CHECK(layout.bv_node_offset_i == (uint32_t)offsetof(ZrcBvNode, i));
  CHECK(layout.poly_ref_size == (uint32_t)sizeof(ZrcPolyRef));
  CHECK(layout.tile_ref_size == (uint32_t)sizeof(ZrcTileRef));
  CHECK(layout.result_count == (uint32_t)ZRC_ERR_CROWD_FULL + 1u);
  CHECK(layout.max_areas == ZRC_MAX_AREAS);
  CHECK(layout.verts_per_polygon == ZRC_VERTS_PER_POLYGON);
  CHECK(layout.alloc_alignment == ZRC_ALLOC_ALIGNMENT);
  CHECK(layout.assert_handler_size == (uint32_t)sizeof(ZrcAssertHandler));
  CHECK(layout.assert_handler_align == (uint32_t)_Alignof(ZrcAssertHandler));
  CHECK(layout.tile_layout_size == (uint32_t)sizeof(ZrcTileLayout));
  CHECK(layout.tile_layout_align == (uint32_t)_Alignof(ZrcTileLayout));

  /* The staged Recast pipeline's structs. */

  CHECK(layout.build_context_size == (uint32_t)sizeof(ZrcBuildContext));
  CHECK(layout.build_context_align == (uint32_t)_Alignof(ZrcBuildContext));
  CHECK(layout.build_context_field_count == 9u);
  CHECK(layout.build_context_offsets[0] ==
        (uint32_t)offsetof(ZrcBuildContext, user));
  CHECK(layout.build_context_offsets[1] ==
        (uint32_t)offsetof(ZrcBuildContext, log));
  CHECK(layout.build_context_offsets[2] ==
        (uint32_t)offsetof(ZrcBuildContext, reset_log));
  CHECK(layout.build_context_offsets[3] ==
        (uint32_t)offsetof(ZrcBuildContext, reset_timers));
  CHECK(layout.build_context_offsets[4] ==
        (uint32_t)offsetof(ZrcBuildContext, start_timer));
  CHECK(layout.build_context_offsets[5] ==
        (uint32_t)offsetof(ZrcBuildContext, stop_timer));
  CHECK(layout.build_context_offsets[6] ==
        (uint32_t)offsetof(ZrcBuildContext, accumulated_time));
  CHECK(layout.build_context_offsets[7] ==
        (uint32_t)offsetof(ZrcBuildContext, log_enabled));
  CHECK(layout.build_context_offsets[8] ==
        (uint32_t)offsetof(ZrcBuildContext, timers_enabled));

  CHECK(layout.span_size == (uint32_t)sizeof(ZrcSpan));
  CHECK(layout.span_align == (uint32_t)_Alignof(ZrcSpan));
  CHECK(layout.span_offset_smin == (uint32_t)offsetof(ZrcSpan, smin));
  CHECK(layout.span_offset_smax == (uint32_t)offsetof(ZrcSpan, smax));
  CHECK(layout.span_offset_area == (uint32_t)offsetof(ZrcSpan, area));

  CHECK(layout.heightfield_info_size == (uint32_t)sizeof(ZrcHeightfieldInfo));
  CHECK(layout.heightfield_info_align ==
        (uint32_t)_Alignof(ZrcHeightfieldInfo));
  CHECK(layout.heightfield_info_field_count == 6u);
  CHECK(layout.heightfield_info_offsets[0] ==
        (uint32_t)offsetof(ZrcHeightfieldInfo, width));
  CHECK(layout.heightfield_info_offsets[1] ==
        (uint32_t)offsetof(ZrcHeightfieldInfo, height));
  CHECK(layout.heightfield_info_offsets[2] ==
        (uint32_t)offsetof(ZrcHeightfieldInfo, bmin));
  CHECK(layout.heightfield_info_offsets[3] ==
        (uint32_t)offsetof(ZrcHeightfieldInfo, bmax));
  CHECK(layout.heightfield_info_offsets[4] ==
        (uint32_t)offsetof(ZrcHeightfieldInfo, cell_size));
  CHECK(layout.heightfield_info_offsets[5] ==
        (uint32_t)offsetof(ZrcHeightfieldInfo, cell_height));

  CHECK(layout.heightfield_storage_size ==
        (uint32_t)sizeof(ZrcHeightfieldStorage));
  CHECK(layout.heightfield_storage_align ==
        (uint32_t)_Alignof(ZrcHeightfieldStorage));
  CHECK(layout.heightfield_storage_offset_pool_count ==
        (uint32_t)offsetof(ZrcHeightfieldStorage, pool_count));
  CHECK(layout.heightfield_storage_offset_free_count ==
        (uint32_t)offsetof(ZrcHeightfieldStorage, free_count));
  CHECK(layout.heightfield_storage_offset_spans_per_pool ==
        (uint32_t)offsetof(ZrcHeightfieldStorage, spans_per_pool));

  CHECK(layout.compact_cell_size == (uint32_t)sizeof(ZrcCompactCell));
  CHECK(layout.compact_cell_align == (uint32_t)_Alignof(ZrcCompactCell));
  CHECK(layout.compact_cell_offset_index ==
        (uint32_t)offsetof(ZrcCompactCell, index));
  CHECK(layout.compact_cell_offset_count ==
        (uint32_t)offsetof(ZrcCompactCell, count));

  CHECK(layout.compact_span_size == (uint32_t)sizeof(ZrcCompactSpan));
  CHECK(layout.compact_span_align == (uint32_t)_Alignof(ZrcCompactSpan));
  CHECK(layout.compact_span_offset_y ==
        (uint32_t)offsetof(ZrcCompactSpan, y));
  CHECK(layout.compact_span_offset_reg ==
        (uint32_t)offsetof(ZrcCompactSpan, reg));
  CHECK(layout.compact_span_offset_con ==
        (uint32_t)offsetof(ZrcCompactSpan, con));
  CHECK(layout.compact_span_offset_h ==
        (uint32_t)offsetof(ZrcCompactSpan, h));

  CHECK(layout.compact_heightfield_info_size ==
        (uint32_t)sizeof(ZrcCompactHeightfieldInfo));
  CHECK(layout.compact_heightfield_info_align ==
        (uint32_t)_Alignof(ZrcCompactHeightfieldInfo));
  CHECK(layout.compact_heightfield_info_field_count == 13u);
  CHECK(layout.compact_heightfield_info_offsets[0] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, width));
  CHECK(layout.compact_heightfield_info_offsets[1] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, height));
  CHECK(layout.compact_heightfield_info_offsets[2] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, span_count));
  CHECK(layout.compact_heightfield_info_offsets[3] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, walkable_height));
  CHECK(layout.compact_heightfield_info_offsets[4] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, walkable_climb));
  CHECK(layout.compact_heightfield_info_offsets[5] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, border_size));
  CHECK(layout.compact_heightfield_info_offsets[6] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, max_distance));
  CHECK(layout.compact_heightfield_info_offsets[7] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, max_regions));
  CHECK(layout.compact_heightfield_info_offsets[8] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, bmin));
  CHECK(layout.compact_heightfield_info_offsets[9] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, bmax));
  CHECK(layout.compact_heightfield_info_offsets[10] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, cell_size));
  CHECK(layout.compact_heightfield_info_offsets[11] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, cell_height));
  CHECK(layout.compact_heightfield_info_offsets[12] ==
        (uint32_t)offsetof(ZrcCompactHeightfieldInfo, has_distance_field));

  CHECK(layout.contour_set_info_size == (uint32_t)sizeof(ZrcContourSetInfo));
  CHECK(layout.contour_set_info_align ==
        (uint32_t)_Alignof(ZrcContourSetInfo));
  CHECK(layout.contour_set_info_field_count == 9u);
  CHECK(layout.contour_set_info_offsets[0] ==
        (uint32_t)offsetof(ZrcContourSetInfo, contour_count));
  CHECK(layout.contour_set_info_offsets[1] ==
        (uint32_t)offsetof(ZrcContourSetInfo, bmin));
  CHECK(layout.contour_set_info_offsets[2] ==
        (uint32_t)offsetof(ZrcContourSetInfo, bmax));
  CHECK(layout.contour_set_info_offsets[3] ==
        (uint32_t)offsetof(ZrcContourSetInfo, cell_size));
  CHECK(layout.contour_set_info_offsets[4] ==
        (uint32_t)offsetof(ZrcContourSetInfo, cell_height));
  CHECK(layout.contour_set_info_offsets[5] ==
        (uint32_t)offsetof(ZrcContourSetInfo, width));
  CHECK(layout.contour_set_info_offsets[6] ==
        (uint32_t)offsetof(ZrcContourSetInfo, height));
  CHECK(layout.contour_set_info_offsets[7] ==
        (uint32_t)offsetof(ZrcContourSetInfo, border_size));
  CHECK(layout.contour_set_info_offsets[8] ==
        (uint32_t)offsetof(ZrcContourSetInfo, max_error));

  CHECK(layout.contour_info_size == (uint32_t)sizeof(ZrcContourInfo));
  CHECK(layout.contour_info_align == (uint32_t)_Alignof(ZrcContourInfo));
  CHECK(layout.contour_info_offset_vert_count ==
        (uint32_t)offsetof(ZrcContourInfo, vert_count));
  CHECK(layout.contour_info_offset_raw_vert_count ==
        (uint32_t)offsetof(ZrcContourInfo, raw_vert_count));
  CHECK(layout.contour_info_offset_region ==
        (uint32_t)offsetof(ZrcContourInfo, region));
  CHECK(layout.contour_info_offset_area ==
        (uint32_t)offsetof(ZrcContourInfo, area));

  CHECK(layout.poly_mesh_info_size == (uint32_t)sizeof(ZrcPolyMeshInfo));
  CHECK(layout.poly_mesh_info_align == (uint32_t)_Alignof(ZrcPolyMeshInfo));
  CHECK(layout.poly_mesh_info_field_count == 16u);
  CHECK(layout.poly_mesh_info_offsets[0] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, vert_count));
  CHECK(layout.poly_mesh_info_offsets[1] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, poly_count));
  CHECK(layout.poly_mesh_info_offsets[2] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, max_polys));
  CHECK(layout.poly_mesh_info_offsets[3] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, verts_per_poly));
  CHECK(layout.poly_mesh_info_offsets[4] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, bmin));
  CHECK(layout.poly_mesh_info_offsets[5] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, bmax));
  CHECK(layout.poly_mesh_info_offsets[6] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, cell_size));
  CHECK(layout.poly_mesh_info_offsets[7] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, cell_height));
  CHECK(layout.poly_mesh_info_offsets[8] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, border_size));
  CHECK(layout.poly_mesh_info_offsets[9] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, max_edge_error));
  CHECK(layout.poly_mesh_info_offsets[10] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, detail_mesh_count));
  CHECK(layout.poly_mesh_info_offsets[11] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, detail_vert_count));
  CHECK(layout.poly_mesh_info_offsets[12] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, detail_tri_count));
  CHECK(layout.poly_mesh_info_offsets[13] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, walkable_height));
  CHECK(layout.poly_mesh_info_offsets[14] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, walkable_radius));
  CHECK(layout.poly_mesh_info_offsets[15] ==
        (uint32_t)offsetof(ZrcPolyMeshInfo, walkable_climb));

  /* One past the last timer label, and the length of a table indexed by
     one. */
  CHECK(layout.timer_label_count == (uint32_t)ZRC_MAX_TIMERS + 1u);

  /* The layered heightfield and the tile cache. */

  CHECK(layout.heightfield_layer_size == (uint32_t)sizeof(ZrcHeightfieldLayer));
  CHECK(layout.heightfield_layer_align ==
        (uint32_t)_Alignof(ZrcHeightfieldLayer));
  CHECK(layout.heightfield_layer_field_count == 12u);
  CHECK(layout.heightfield_layer_offsets[0] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, bmin));
  CHECK(layout.heightfield_layer_offsets[1] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, bmax));
  CHECK(layout.heightfield_layer_offsets[2] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, cell_size));
  CHECK(layout.heightfield_layer_offsets[3] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, cell_height));
  CHECK(layout.heightfield_layer_offsets[4] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, width));
  CHECK(layout.heightfield_layer_offsets[5] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, height));
  CHECK(layout.heightfield_layer_offsets[6] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, min_x));
  CHECK(layout.heightfield_layer_offsets[7] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, max_x));
  CHECK(layout.heightfield_layer_offsets[8] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, min_z));
  CHECK(layout.heightfield_layer_offsets[9] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, max_z));
  CHECK(layout.heightfield_layer_offsets[10] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, height_min));
  CHECK(layout.heightfield_layer_offsets[11] ==
        (uint32_t)offsetof(ZrcHeightfieldLayer, height_max));

  CHECK(layout.tile_cache_params_size == (uint32_t)sizeof(ZrcTileCacheParams));
  CHECK(layout.tile_cache_params_align ==
        (uint32_t)_Alignof(ZrcTileCacheParams));
  CHECK(layout.tile_cache_params_field_count == 11u);
  CHECK(layout.tile_cache_params_offsets[0] ==
        (uint32_t)offsetof(ZrcTileCacheParams, origin));
  CHECK(layout.tile_cache_params_offsets[1] ==
        (uint32_t)offsetof(ZrcTileCacheParams, cell_size));
  CHECK(layout.tile_cache_params_offsets[2] ==
        (uint32_t)offsetof(ZrcTileCacheParams, cell_height));
  CHECK(layout.tile_cache_params_offsets[3] ==
        (uint32_t)offsetof(ZrcTileCacheParams, width));
  CHECK(layout.tile_cache_params_offsets[4] ==
        (uint32_t)offsetof(ZrcTileCacheParams, height));
  CHECK(layout.tile_cache_params_offsets[5] ==
        (uint32_t)offsetof(ZrcTileCacheParams, walkable_height));
  CHECK(layout.tile_cache_params_offsets[6] ==
        (uint32_t)offsetof(ZrcTileCacheParams, walkable_radius));
  CHECK(layout.tile_cache_params_offsets[7] ==
        (uint32_t)offsetof(ZrcTileCacheParams, walkable_climb));
  CHECK(layout.tile_cache_params_offsets[8] ==
        (uint32_t)offsetof(ZrcTileCacheParams, max_simplification_error));
  CHECK(layout.tile_cache_params_offsets[9] ==
        (uint32_t)offsetof(ZrcTileCacheParams, max_tiles));
  CHECK(layout.tile_cache_params_offsets[10] ==
        (uint32_t)offsetof(ZrcTileCacheParams, max_obstacles));

  CHECK(layout.tile_cache_compressor_size ==
        (uint32_t)sizeof(ZrcTileCacheCompressor));
  CHECK(layout.tile_cache_compressor_align ==
        (uint32_t)_Alignof(ZrcTileCacheCompressor));
  CHECK(layout.tile_cache_compressor_field_count == 4u);
  CHECK(layout.tile_cache_compressor_offsets[0] ==
        (uint32_t)offsetof(ZrcTileCacheCompressor, user));
  CHECK(layout.tile_cache_compressor_offsets[1] ==
        (uint32_t)offsetof(ZrcTileCacheCompressor, max_compressed_size));
  CHECK(layout.tile_cache_compressor_offsets[2] ==
        (uint32_t)offsetof(ZrcTileCacheCompressor, compress));
  CHECK(layout.tile_cache_compressor_offsets[3] ==
        (uint32_t)offsetof(ZrcTileCacheCompressor, decompress));

  CHECK(layout.tile_cache_allocator_size ==
        (uint32_t)sizeof(ZrcTileCacheAllocator));
  CHECK(layout.tile_cache_allocator_align ==
        (uint32_t)_Alignof(ZrcTileCacheAllocator));
  CHECK(layout.tile_cache_allocator_field_count == 4u);
  CHECK(layout.tile_cache_allocator_offsets[0] ==
        (uint32_t)offsetof(ZrcTileCacheAllocator, user));
  CHECK(layout.tile_cache_allocator_offsets[1] ==
        (uint32_t)offsetof(ZrcTileCacheAllocator, reset));
  CHECK(layout.tile_cache_allocator_offsets[2] ==
        (uint32_t)offsetof(ZrcTileCacheAllocator, allocate));
  CHECK(layout.tile_cache_allocator_offsets[3] ==
        (uint32_t)offsetof(ZrcTileCacheAllocator, deallocate));

  CHECK(layout.tile_cache_build_params_size ==
        (uint32_t)sizeof(ZrcTileCacheBuildParams));
  CHECK(layout.tile_cache_build_params_align ==
        (uint32_t)_Alignof(ZrcTileCacheBuildParams));
  CHECK(layout.tile_cache_build_params_field_count == 6u);
  CHECK(layout.tile_cache_build_params_offsets[0] ==
        (uint32_t)offsetof(ZrcTileCacheBuildParams, areas));
  CHECK(layout.tile_cache_build_params_offsets[1] ==
        (uint32_t)offsetof(ZrcTileCacheBuildParams, flags));
  CHECK(layout.tile_cache_build_params_offsets[2] ==
        (uint32_t)offsetof(ZrcTileCacheBuildParams, poly_count));
  CHECK(layout.tile_cache_build_params_offsets[3] ==
        (uint32_t)offsetof(ZrcTileCacheBuildParams, user_id));
  CHECK(layout.tile_cache_build_params_offsets[4] ==
        (uint32_t)offsetof(ZrcTileCacheBuildParams, connections));
  CHECK(layout.tile_cache_build_params_offsets[5] ==
        (uint32_t)offsetof(ZrcTileCacheBuildParams, connection_count));

  CHECK(layout.compressed_tile_info_size ==
        (uint32_t)sizeof(ZrcCompressedTileInfo));
  CHECK(layout.compressed_tile_info_align ==
        (uint32_t)_Alignof(ZrcCompressedTileInfo));
  CHECK(layout.compressed_tile_info_field_count == 14u);
  CHECK(layout.compressed_tile_info_offsets[0] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, tile_x));
  CHECK(layout.compressed_tile_info_offsets[1] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, tile_y));
  CHECK(layout.compressed_tile_info_offsets[2] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, tile_layer));
  CHECK(layout.compressed_tile_info_offsets[3] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, bmin));
  CHECK(layout.compressed_tile_info_offsets[4] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, bmax));
  CHECK(layout.compressed_tile_info_offsets[5] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, height_min));
  CHECK(layout.compressed_tile_info_offsets[6] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, height_max));
  CHECK(layout.compressed_tile_info_offsets[7] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, width));
  CHECK(layout.compressed_tile_info_offsets[8] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, height));
  CHECK(layout.compressed_tile_info_offsets[9] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, min_x));
  CHECK(layout.compressed_tile_info_offsets[10] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, max_x));
  CHECK(layout.compressed_tile_info_offsets[11] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, min_z));
  CHECK(layout.compressed_tile_info_offsets[12] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, max_z));
  CHECK(layout.compressed_tile_info_offsets[13] ==
        (uint32_t)offsetof(ZrcCompressedTileInfo, data_size));

  CHECK(layout.obstacle_info_size == (uint32_t)sizeof(ZrcObstacleInfo));
  CHECK(layout.obstacle_info_align == (uint32_t)_Alignof(ZrcObstacleInfo));
  CHECK(layout.obstacle_info_field_count == 12u);
  CHECK(layout.obstacle_info_offsets[0] ==
        (uint32_t)offsetof(ZrcObstacleInfo, shape));
  CHECK(layout.obstacle_info_offsets[1] ==
        (uint32_t)offsetof(ZrcObstacleInfo, state));
  CHECK(layout.obstacle_info_offsets[2] ==
        (uint32_t)offsetof(ZrcObstacleInfo, position));
  CHECK(layout.obstacle_info_offsets[3] ==
        (uint32_t)offsetof(ZrcObstacleInfo, radius));
  CHECK(layout.obstacle_info_offsets[4] ==
        (uint32_t)offsetof(ZrcObstacleInfo, height));
  CHECK(layout.obstacle_info_offsets[5] ==
        (uint32_t)offsetof(ZrcObstacleInfo, bmin));
  CHECK(layout.obstacle_info_offsets[6] ==
        (uint32_t)offsetof(ZrcObstacleInfo, bmax));
  CHECK(layout.obstacle_info_offsets[7] ==
        (uint32_t)offsetof(ZrcObstacleInfo, center));
  CHECK(layout.obstacle_info_offsets[8] ==
        (uint32_t)offsetof(ZrcObstacleInfo, half_extents));
  CHECK(layout.obstacle_info_offsets[9] ==
        (uint32_t)offsetof(ZrcObstacleInfo, y_radians));
  CHECK(layout.obstacle_info_offsets[10] ==
        (uint32_t)offsetof(ZrcObstacleInfo, touched_count));
  CHECK(layout.obstacle_info_offsets[11] ==
        (uint32_t)offsetof(ZrcObstacleInfo, pending_count));

  CHECK(layout.tile_cache_layer_header_size ==
        (uint32_t)sizeof(ZrcTileCacheLayerHeader));
  CHECK(layout.tile_cache_layer_header_align ==
        (uint32_t)_Alignof(ZrcTileCacheLayerHeader));
  CHECK(layout.tile_cache_layer_header_field_count == 13u);
  CHECK(layout.tile_cache_layer_header_offsets[0] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, tile_x));
  CHECK(layout.tile_cache_layer_header_offsets[1] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, tile_y));
  CHECK(layout.tile_cache_layer_header_offsets[2] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, tile_layer));
  CHECK(layout.tile_cache_layer_header_offsets[3] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, bmin));
  CHECK(layout.tile_cache_layer_header_offsets[4] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, bmax));
  CHECK(layout.tile_cache_layer_header_offsets[5] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, height_min));
  CHECK(layout.tile_cache_layer_header_offsets[6] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, height_max));
  CHECK(layout.tile_cache_layer_header_offsets[7] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, width));
  CHECK(layout.tile_cache_layer_header_offsets[8] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, height));
  CHECK(layout.tile_cache_layer_header_offsets[9] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, min_x));
  CHECK(layout.tile_cache_layer_header_offsets[10] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, max_x));
  CHECK(layout.tile_cache_layer_header_offsets[11] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, min_z));
  CHECK(layout.tile_cache_layer_header_offsets[12] ==
        (uint32_t)offsetof(ZrcTileCacheLayerHeader, max_z));

  CHECK(layout.tile_cache_contour_info_size ==
        (uint32_t)sizeof(ZrcTileCacheContourInfo));
  CHECK(layout.tile_cache_contour_info_align ==
        (uint32_t)_Alignof(ZrcTileCacheContourInfo));
  CHECK(layout.tile_cache_contour_info_offset_vert_count ==
        (uint32_t)offsetof(ZrcTileCacheContourInfo, vert_count));
  CHECK(layout.tile_cache_contour_info_offset_region ==
        (uint32_t)offsetof(ZrcTileCacheContourInfo, region));
  CHECK(layout.tile_cache_contour_info_offset_area ==
        (uint32_t)offsetof(ZrcTileCacheContourInfo, area));

  CHECK(layout.tile_cache_poly_mesh_info_size ==
        (uint32_t)sizeof(ZrcTileCachePolyMeshInfo));
  CHECK(layout.tile_cache_poly_mesh_info_align ==
        (uint32_t)_Alignof(ZrcTileCachePolyMeshInfo));
  CHECK(layout.tile_cache_poly_mesh_info_offset_vert_count ==
        (uint32_t)offsetof(ZrcTileCachePolyMeshInfo, vert_count));
  CHECK(layout.tile_cache_poly_mesh_info_offset_poly_count ==
        (uint32_t)offsetof(ZrcTileCachePolyMeshInfo, poly_count));
  CHECK(layout.tile_cache_poly_mesh_info_offset_verts_per_poly ==
        (uint32_t)offsetof(ZrcTileCachePolyMeshInfo, verts_per_poly));

  CHECK(layout.compressed_tile_ref_size ==
        (uint32_t)sizeof(ZrcCompressedTileRef));
  CHECK(layout.obstacle_ref_size == (uint32_t)sizeof(ZrcObstacleRef));

  /* The crowd's cross-boundary structs. */

  CHECK(layout.crowd_agent_params_size ==
        (uint32_t)sizeof(ZrcCrowdAgentParams));
  CHECK(layout.crowd_agent_params_align ==
        (uint32_t)_Alignof(ZrcCrowdAgentParams));
  CHECK(layout.crowd_agent_params_field_count == 11u);
  CHECK(layout.crowd_agent_params_offsets[0] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, radius));
  CHECK(layout.crowd_agent_params_offsets[1] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, height));
  CHECK(layout.crowd_agent_params_offsets[2] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, max_acceleration));
  CHECK(layout.crowd_agent_params_offsets[3] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, max_speed));
  CHECK(layout.crowd_agent_params_offsets[4] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, collision_query_range));
  CHECK(layout.crowd_agent_params_offsets[5] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, path_optimization_range));
  CHECK(layout.crowd_agent_params_offsets[6] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, separation_weight));
  CHECK(layout.crowd_agent_params_offsets[7] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, update_flags));
  CHECK(layout.crowd_agent_params_offsets[8] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, obstacle_avoidance_type));
  CHECK(layout.crowd_agent_params_offsets[9] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, query_filter_type));
  CHECK(layout.crowd_agent_params_offsets[10] ==
        (uint32_t)offsetof(ZrcCrowdAgentParams, user_data));

  CHECK(layout.crowd_agent_size == (uint32_t)sizeof(ZrcCrowdAgent));
  CHECK(layout.crowd_agent_align == (uint32_t)_Alignof(ZrcCrowdAgent));
  CHECK(layout.crowd_agent_field_count == 17u);
  CHECK(layout.crowd_agent_offsets[0] ==
        (uint32_t)offsetof(ZrcCrowdAgent, state));
  CHECK(layout.crowd_agent_offsets[1] ==
        (uint32_t)offsetof(ZrcCrowdAgent, target_state));
  CHECK(layout.crowd_agent_offsets[2] ==
        (uint32_t)offsetof(ZrcCrowdAgent, partial));
  CHECK(layout.crowd_agent_offsets[3] ==
        (uint32_t)offsetof(ZrcCrowdAgent, position));
  CHECK(layout.crowd_agent_offsets[4] ==
        (uint32_t)offsetof(ZrcCrowdAgent, velocity));
  CHECK(layout.crowd_agent_offsets[5] ==
        (uint32_t)offsetof(ZrcCrowdAgent, desired_velocity));
  CHECK(layout.crowd_agent_offsets[6] ==
        (uint32_t)offsetof(ZrcCrowdAgent, avoided_velocity));
  CHECK(layout.crowd_agent_offsets[7] ==
        (uint32_t)offsetof(ZrcCrowdAgent, displacement));
  CHECK(layout.crowd_agent_offsets[8] ==
        (uint32_t)offsetof(ZrcCrowdAgent, desired_speed));
  CHECK(layout.crowd_agent_offsets[9] ==
        (uint32_t)offsetof(ZrcCrowdAgent, target_ref));
  CHECK(layout.crowd_agent_offsets[10] ==
        (uint32_t)offsetof(ZrcCrowdAgent, target_position));
  CHECK(layout.crowd_agent_offsets[11] ==
        (uint32_t)offsetof(ZrcCrowdAgent, target_replan));
  CHECK(layout.crowd_agent_offsets[12] ==
        (uint32_t)offsetof(ZrcCrowdAgent, target_replan_time));
  CHECK(layout.crowd_agent_offsets[13] ==
        (uint32_t)offsetof(ZrcCrowdAgent, topology_opt_time));
  CHECK(layout.crowd_agent_offsets[14] ==
        (uint32_t)offsetof(ZrcCrowdAgent, corner_count));
  CHECK(layout.crowd_agent_offsets[15] ==
        (uint32_t)offsetof(ZrcCrowdAgent, neighbour_count));
  CHECK(layout.crowd_agent_offsets[16] ==
        (uint32_t)offsetof(ZrcCrowdAgent, params));

  CHECK(layout.crowd_corner_size == (uint32_t)sizeof(ZrcCrowdCorner));
  CHECK(layout.crowd_corner_align == (uint32_t)_Alignof(ZrcCrowdCorner));
  CHECK(layout.crowd_corner_field_count == 3u);
  CHECK(layout.crowd_corner_offsets[0] ==
        (uint32_t)offsetof(ZrcCrowdCorner, position));
  CHECK(layout.crowd_corner_offsets[1] ==
        (uint32_t)offsetof(ZrcCrowdCorner, flags));
  CHECK(layout.crowd_corner_offsets[2] ==
        (uint32_t)offsetof(ZrcCrowdCorner, poly));

  CHECK(layout.crowd_neighbour_size == (uint32_t)sizeof(ZrcCrowdNeighbour));
  CHECK(layout.crowd_neighbour_align ==
        (uint32_t)_Alignof(ZrcCrowdNeighbour));
  CHECK(layout.crowd_neighbour_field_count == 2u);
  CHECK(layout.crowd_neighbour_offsets[0] ==
        (uint32_t)offsetof(ZrcCrowdNeighbour, agent));
  CHECK(layout.crowd_neighbour_offsets[1] ==
        (uint32_t)offsetof(ZrcCrowdNeighbour, distance));

  CHECK(layout.crowd_agent_animation_size ==
        (uint32_t)sizeof(ZrcCrowdAgentAnimation));
  CHECK(layout.crowd_agent_animation_align ==
        (uint32_t)_Alignof(ZrcCrowdAgentAnimation));
  CHECK(layout.crowd_agent_animation_field_count == 7u);
  CHECK(layout.crowd_agent_animation_offsets[0] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, active));
  CHECK(layout.crowd_agent_animation_offsets[1] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, init_position));
  CHECK(layout.crowd_agent_animation_offsets[2] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, start_position));
  CHECK(layout.crowd_agent_animation_offsets[3] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, end_position));
  CHECK(layout.crowd_agent_animation_offsets[4] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, poly));
  CHECK(layout.crowd_agent_animation_offsets[5] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, t));
  CHECK(layout.crowd_agent_animation_offsets[6] ==
        (uint32_t)offsetof(ZrcCrowdAgentAnimation, t_max));

  CHECK(layout.crowd_agent_debug_size ==
        (uint32_t)sizeof(ZrcCrowdAgentDebug));
  CHECK(layout.crowd_agent_debug_align ==
        (uint32_t)_Alignof(ZrcCrowdAgentDebug));
  CHECK(layout.crowd_agent_debug_field_count == 4u);
  CHECK(layout.crowd_agent_debug_offsets[0] ==
        (uint32_t)offsetof(ZrcCrowdAgentDebug, agent));
  CHECK(layout.crowd_agent_debug_offsets[1] ==
        (uint32_t)offsetof(ZrcCrowdAgentDebug, samples));
  CHECK(layout.crowd_agent_debug_offsets[2] ==
        (uint32_t)offsetof(ZrcCrowdAgentDebug, opt_start));
  CHECK(layout.crowd_agent_debug_offsets[3] ==
        (uint32_t)offsetof(ZrcCrowdAgentDebug, opt_end));

  CHECK(layout.avoidance_params_size ==
        (uint32_t)sizeof(ZrcAvoidanceParams));
  CHECK(layout.avoidance_params_align ==
        (uint32_t)_Alignof(ZrcAvoidanceParams));
  CHECK(layout.avoidance_params_field_count == 10u);
  CHECK(layout.avoidance_params_offsets[0] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, vel_bias));
  CHECK(layout.avoidance_params_offsets[1] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, weight_desired_vel));
  CHECK(layout.avoidance_params_offsets[2] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, weight_current_vel));
  CHECK(layout.avoidance_params_offsets[3] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, weight_side));
  CHECK(layout.avoidance_params_offsets[4] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, weight_toi));
  CHECK(layout.avoidance_params_offsets[5] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, horiz_time));
  CHECK(layout.avoidance_params_offsets[6] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, grid_size));
  CHECK(layout.avoidance_params_offsets[7] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, adaptive_divs));
  CHECK(layout.avoidance_params_offsets[8] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, adaptive_rings));
  CHECK(layout.avoidance_params_offsets[9] ==
        (uint32_t)offsetof(ZrcAvoidanceParams, adaptive_depth));

  CHECK(layout.avoidance_circle_size ==
        (uint32_t)sizeof(ZrcAvoidanceCircle));
  CHECK(layout.avoidance_circle_align ==
        (uint32_t)_Alignof(ZrcAvoidanceCircle));
  CHECK(layout.avoidance_circle_field_count == 4u);
  CHECK(layout.avoidance_circle_offsets[0] ==
        (uint32_t)offsetof(ZrcAvoidanceCircle, position));
  CHECK(layout.avoidance_circle_offsets[1] ==
        (uint32_t)offsetof(ZrcAvoidanceCircle, velocity));
  CHECK(layout.avoidance_circle_offsets[2] ==
        (uint32_t)offsetof(ZrcAvoidanceCircle, desired_velocity));
  CHECK(layout.avoidance_circle_offsets[3] ==
        (uint32_t)offsetof(ZrcAvoidanceCircle, radius));

  CHECK(layout.avoidance_segment_size ==
        (uint32_t)sizeof(ZrcAvoidanceSegment));
  CHECK(layout.avoidance_segment_align ==
        (uint32_t)_Alignof(ZrcAvoidanceSegment));
  CHECK(layout.avoidance_segment_field_count == 3u);
  CHECK(layout.avoidance_segment_offsets[0] ==
        (uint32_t)offsetof(ZrcAvoidanceSegment, p));
  CHECK(layout.avoidance_segment_offsets[1] ==
        (uint32_t)offsetof(ZrcAvoidanceSegment, q));
  CHECK(layout.avoidance_segment_offsets[2] ==
        (uint32_t)offsetof(ZrcAvoidanceSegment, touching));

  CHECK(layout.avoidance_sample_size ==
        (uint32_t)sizeof(ZrcAvoidanceSample));
  CHECK(layout.avoidance_sample_align ==
        (uint32_t)_Alignof(ZrcAvoidanceSample));
  CHECK(layout.avoidance_sample_field_count == 7u);
  CHECK(layout.avoidance_sample_offsets[0] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, velocity));
  CHECK(layout.avoidance_sample_offsets[1] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, size));
  CHECK(layout.avoidance_sample_offsets[2] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, penalty));
  CHECK(layout.avoidance_sample_offsets[3] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, desired_velocity_penalty));
  CHECK(layout.avoidance_sample_offsets[4] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, current_velocity_penalty));
  CHECK(layout.avoidance_sample_offsets[5] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, preferred_side_penalty));
  CHECK(layout.avoidance_sample_offsets[6] ==
        (uint32_t)offsetof(ZrcAvoidanceSample, collision_time_penalty));

  CHECK(layout.path_corridor_info_size ==
        (uint32_t)sizeof(ZrcPathCorridorInfo));
  CHECK(layout.path_corridor_info_align ==
        (uint32_t)_Alignof(ZrcPathCorridorInfo));
  CHECK(layout.path_corridor_info_field_count == 5u);
  CHECK(layout.path_corridor_info_offsets[0] ==
        (uint32_t)offsetof(ZrcPathCorridorInfo, position));
  CHECK(layout.path_corridor_info_offsets[1] ==
        (uint32_t)offsetof(ZrcPathCorridorInfo, target));
  CHECK(layout.path_corridor_info_offsets[2] ==
        (uint32_t)offsetof(ZrcPathCorridorInfo, first_poly));
  CHECK(layout.path_corridor_info_offsets[3] ==
        (uint32_t)offsetof(ZrcPathCorridorInfo, last_poly));
  CHECK(layout.path_corridor_info_offsets[4] ==
        (uint32_t)offsetof(ZrcPathCorridorInfo, path_count));

  CHECK(layout.agent_ref_size == (uint32_t)sizeof(ZrcAgentRef));
  CHECK(layout.path_request_ref_size ==
        (uint32_t)sizeof(ZrcPathRequestRef));
}

static void test_allocator_rejects_incomplete(void) {
  ZrcAllocator bad;
  bad.allocate = NULL;
  bad.deallocate = count_deallocate;
  bad.user = NULL;
  CHECK(zrcSetAllocator(&bad) == ZRC_ERR_INVALID_ARGUMENT);
}

/// A rejected install must not disturb the one already in place, and NULL must
/// genuinely put upstream's malloc/free back.
static void test_allocator_swapping(Counters* counters) {
  const size_t before = counters->allocations;

  /* A half-filled allocator is worse than none: upstream would allocate
     through the host and free through malloc. Rejecting it has to leave the
     working one installed. */
  ZrcAllocator half;
  half.allocate = count_allocate;
  half.deallocate = NULL;
  half.user = counters;
  CHECK(zrcSetAllocator(&half) == ZRC_ERR_INVALID_ARGUMENT);

  /* Any call that allocates will do. */
  {
    ZrcPolyMesh* poly = NULL;
    ZrcBakeConfig cfg;
    ZrcTriMesh mesh;
    zrcBakeConfigDefault(&cfg);
    zrcFixtureTriMesh(&mesh);
    if (zrcPolyMeshBake(&cfg, &mesh, NULL, NULL, &poly) == ZRC_OK) {
      zrcPolyMeshDestroy(poly);
    }
  }
  CHECK(counters->allocations > before);

  /* NULL restores upstream's own malloc/free, so nothing further reaches the
     counters. */
  CHECK(zrcSetAllocator(NULL) == ZRC_OK);
  const size_t after_reset = counters->allocations;
  {
    ZrcPolyMesh* poly = NULL;
    ZrcBakeConfig cfg;
    ZrcTriMesh mesh;
    zrcBakeConfigDefault(&cfg);
    zrcFixtureTriMesh(&mesh);
    if (zrcPolyMeshBake(&cfg, &mesh, NULL, NULL, &poly) == ZRC_OK) {
      zrcPolyMeshDestroy(poly);
    }
  }
  CHECK(counters->allocations == after_reset);

  /* And back, so the balance assertion at the end still means something. */
  ZrcAllocator good;
  good.allocate = count_allocate;
  good.deallocate = count_deallocate;
  good.user = counters;
  CHECK(zrcSetAllocator(&good) == ZRC_OK);
}

static void test_alloc_free(void) {
  void* block = zrcAlloc(64, ZRC_ALLOC_PERM);
  CHECK(block != NULL);
  memset(block, 0xAB, 64);
  zrcFree(block);

  /* A size of 0 is passed through unchanged; whatever the installed
     allocator answers for it is the answer, freed the same way. */
  void* zero = zrcAlloc(0, ZRC_ALLOC_TEMP);
  zrcFree(zero);
}

static void test_null_arguments(void) {
  /* Out-parameters and handles that a caller may plausibly get wrong. None of
     these may fault, and each must report rather than pretend. */
  float a[3], b[3];
  ZrcPolyMeshInfo poly_bounds_info;
  CHECK(zrcPolyMeshInfo(NULL, &poly_bounds_info) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshBounds(NULL, a, b) == ZRC_ERR_INVALID_ARGUMENT);

  void* data = (void*)0x1;
  size_t size = 7;
  CHECK(zrcNavMeshSerialize(NULL, &data, &size) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(data == NULL);
  CHECK(size == 0);
  CHECK(zrcNavMeshSerialize(NULL, NULL, &size) == ZRC_ERR_INVALID_ARGUMENT);

  ZrcQueryFilter f;
  zrcQueryFilterDefault(&f);
  ZrcPolyRef ref = 9;
  CHECK(zrcFindNearestPoly(NULL, a, b, &f, &ref, NULL, NULL) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(ref == 0);

  ZrcNavMeshQuery* q = (ZrcNavMeshQuery*)0x1;
  CHECK(zrcNavMeshQueryCreate(NULL, 2048, &q) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(q == NULL);
}

static void test_null_handles_are_safe(void) {
  /* Destroying NULL is defined and must not crash. */
  zrcPolyMeshDestroy(NULL);
  zrcNavMeshDestroy(NULL);
  zrcNavMeshQueryDestroy(NULL);
  zrcFree(NULL);

  ZrcPolyMeshInfo poly_info_null;
  CHECK(zrcPolyMeshInfo(NULL, &poly_info_null) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(poly_info_null.poly_count == 0);
  /* The three counts a NULL navmesh answers all report the same -1, and none
     of them reports 0: an empty navmesh answers 0 legitimately. */
  CHECK(zrcNavMeshPolyCount(NULL) == -1);
  CHECK(zrcNavMeshTileCount(NULL) == -1);
  CHECK(zrcNavMeshMaxTiles(NULL) == -1);

  /* And so is asking for a layout or a default through a NULL pointer. */
  zrcAbiLayout(NULL);
  zrcBakeConfigDefault(NULL);
  zrcQueryFilterDefault(NULL);
}

static void my_assert_fail(void* user, const char* expression,
                            const char* file, int32_t line) {
  (void)user;
  (void)expression;
  (void)file;
  (void)line;
}

static void test_assert_handler(void) {
  const ZrcBool enabled = zrcAssertsEnabled();
  CHECK(enabled == ZRC_TRUE || enabled == ZRC_FALSE);

  /* A handler with no fail function is refused. */
  ZrcAssertHandler bad;
  bad.fail = NULL;
  bad.user = NULL;
  CHECK(zrcSetAssertHandler(&bad) == ZRC_ERR_INVALID_ARGUMENT);

  /* A valid one installs and reads back exactly what was given. */
  ZrcAssertHandler good;
  good.fail = my_assert_fail;
  good.user = (void*)0x1234;
  CHECK(zrcSetAssertHandler(&good) == ZRC_OK);

  ZrcAssertHandler out;
  memset(&out, 0, sizeof(out));
  CHECK(zrcAssertHandler(&out) == ZRC_OK);
  CHECK(out.fail == my_assert_fail);
  CHECK(out.user == (void*)0x1234);

  /* NULL clears it back to upstream's own assert(). */
  CHECK(zrcSetAssertHandler(NULL) == ZRC_OK);
  memset(&out, 0xAB, sizeof(out));
  CHECK(zrcAssertHandler(&out) == ZRC_OK);
  CHECK(out.fail == NULL);
  CHECK(out.user == NULL);
}

static void test_point_in_polygon(void) {
  const float square[12] = {0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                            1.f, 0.f, 1.f, 0.f, 0.f, 1.f};
  const float inside[3] = {0.5f, 0.f, 0.5f};
  const float outside[3] = {2.f, 0.f, 2.f};

  ZrcBool result = ZRC_FALSE;
  CHECK(zrcPointInPolygon(inside, square, 4, &result) == ZRC_OK);
  CHECK(result == ZRC_TRUE);

  CHECK(zrcPointInPolygon(outside, square, 4, &result) == ZRC_OK);
  CHECK(result == ZRC_FALSE);

  /* A two-vertex polygon has no defined inside; Detour's own loop would
     read out of bounds for it. */
  const float segment[6] = {0.f, 0.f, 0.f, 1.f, 0.f, 0.f};
  CHECK(zrcPointInPolygon(inside, segment, 2, &result) ==
        ZRC_ERR_INVALID_ARGUMENT);
}

static void test_bad_bake_input(void) {
  ZrcBakeConfig config;
  zrcBakeConfigDefault(&config);

  ZrcTriMesh mesh;
  zrcFixtureTriMesh(&mesh);

  ZrcPolyMesh* poly = (ZrcPolyMesh*)0x1;
  CHECK(zrcPolyMeshBake(NULL, &mesh, NULL, NULL, &poly) == ZRC_ERR_INVALID_ARGUMENT);
  /* The out-parameter must be cleared even on failure. */
  CHECK(poly == NULL);

  poly = (ZrcPolyMesh*)0x1;
  CHECK(zrcPolyMeshBake(&config, NULL, NULL, NULL, &poly) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(poly == NULL);

  ZrcBakeConfig broken = config;
  broken.cell_size = 0.f;
  CHECK(zrcPolyMeshBake(&broken, &mesh, NULL, NULL, &poly) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* Geometry with an index past the end of the vertex array: Recast would
     read out of bounds, so this has to be caught at the door. */
  const float verts[9] = {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
  const int bad_tris[3] = {0, 1, 99};
  ZrcTriMesh bad;
  bad.verts = verts;
  bad.vert_count = 3;
  bad.tris = bad_tris;
  bad.tri_count = 1;
  CHECK(zrcPolyMeshBake(&config, &bad, NULL, NULL, &poly) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* Geometry that is real but has nowhere to stand. */
  ZrcTriMesh tent;
  zrcFixtureUnwalkableTriMesh(&tent);
  char log[256];
  ZrcBakeLog bake_log;
  bake_log.buffer = log;
  bake_log.capacity = sizeof(log);
  log[0] = 'x';
  CHECK(zrcPolyMeshBake(&config, &tent, NULL, &bake_log, &poly) ==
        ZRC_ERR_EMPTY_RESULT);
  CHECK(poly == NULL);
  CHECK(strlen(log) > 0);
}

static void test_bad_navmesh_input(void) {
  ZrcNavMesh* mesh = (ZrcNavMesh*)0x1;
  const char garbage[128] = "this is not a navigation mesh at all, honestly";

  CHECK(zrcNavMeshDeserialize(garbage, sizeof(garbage), &mesh) ==
        ZRC_ERR_BAD_FORMAT);
  CHECK(mesh == NULL);
  CHECK(zrcNavMeshDeserialize(NULL, 0, &mesh) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshValidate(garbage, 4) == ZRC_ERR_BAD_FORMAT);
  CHECK(zrcNavMeshValidate(NULL, 128) == ZRC_ERR_INVALID_ARGUMENT);
}

/// The whole arc: bake, build, serialise, reload, query.
/* Area authoring, from C: what a Zig host cannot reach because its VolumeShape
   enum is exhaustive, plus the runtime accessors. */
static void test_area_authoring(void) {
  ZrcBakeConfig config;
  zrcBakeConfigDefault(&config);

  ZrcTriMesh mesh;
  zrcFixtureTriMesh(&mesh);

  ZrcAreaVolume volume;
  memset(&volume, 0, sizeof(volume));
  volume.shape = ZRC_VOLUME_BOX;
  volume.area = 7;
  volume.y_min = -1.0f;
  volume.y_max = 2.0f;
  volume.xz_min[0] = -3.0f;
  volume.xz_min[1] = -12.0f;
  volume.xz_max[0] = 3.0f;
  volume.xz_max[1] = -6.0f;

  ZrcAreaAuthoring authoring;
  authoring.volumes = &volume;
  authoring.volume_count = 1;
  authoring.area_flags = NULL;

  /* A shape outside the three enumerators. The C enum can hold it; the
     validator is the only thing that stops Recast being handed it. */
  ZrcAreaVolume unknown = volume;
  unknown.shape = (ZrcVolumeShape)3;
  ZrcAreaAuthoring bad_shape;
  bad_shape.volumes = &unknown;
  bad_shape.volume_count = 1;
  bad_shape.area_flags = NULL;

  ZrcPolyMesh* poly = NULL;
  CHECK(zrcPolyMeshBake(&config, &mesh, &bad_shape, NULL, &poly) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(poly == NULL);

  /* A count with no array, and a negative count. */
  ZrcAreaAuthoring no_array = authoring;
  no_array.volumes = NULL;
  CHECK(zrcPolyMeshBake(&config, &mesh, &no_array, NULL, &poly) ==
        ZRC_ERR_INVALID_ARGUMENT);
  ZrcAreaAuthoring negative = authoring;
  negative.volume_count = -1;
  CHECK(zrcPolyMeshBake(&config, &mesh, &negative, NULL, &poly) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* The volume itself bakes, and the polygon under a point inside it carries
     the authored area rather than the default one. */
  CHECK(zrcPolyMeshBake(&config, &mesh, &authoring, NULL, &poly) == ZRC_OK);
  if (poly == NULL) {
    ++failures;
    return;
  }

  ZrcNavMesh* navmesh = NULL;
  CHECK(zrcNavMeshCreate(poly, NULL, &navmesh) == ZRC_OK);
  zrcPolyMeshDestroy(poly);
  if (navmesh == NULL) {
    ++failures;
    return;
  }

  ZrcNavMeshQuery* query = NULL;
  CHECK(zrcNavMeshQueryCreate(navmesh, 2048, &query) == ZRC_OK);
  if (query == NULL) {
    zrcNavMeshDestroy(navmesh);
    ++failures;
    return;
  }

  ZrcQueryFilter filter;
  zrcQueryFilterDefault(&filter);
  const float inside[3] = {0.0f, 0.0f, -8.0f};
  const float extents[3] = {2.0f, 4.0f, 2.0f};
  ZrcPolyRef ref = 0;
  float nearest[3];
  ZrcBool over_poly = ZRC_FALSE;
  CHECK(zrcFindNearestPoly(query, inside, extents, &filter, &ref, nearest,
                           &over_poly) == ZRC_OK);
  /* The point was chosen inside the ground plane, so it is over the polygon
     rather than merely nearest to it. */
  CHECK(over_poly == ZRC_TRUE);

  /* And the same answer without asking for the point. Upstream writes
     isOverPoly only inside the branch that also writes nearestPt, so this is
     the case that needs the boundary to hand it scratch of its own. */
  ZrcBool over_alone = ZRC_FALSE;
  CHECK(zrcFindNearestPoly(query, inside, extents, &filter, &ref, NULL,
                           &over_alone) == ZRC_OK);
  CHECK(over_alone == ZRC_TRUE);
  CHECK(ref != 0);

  int32_t area = 0;
  CHECK(zrcNavMeshGetPolyArea(navmesh, ref, &area) == ZRC_OK);
  CHECK(area == 7);

  /* Rewriting takes effect, and is read back. */
  CHECK(zrcNavMeshSetPolyArea(navmesh, ref, 5) == ZRC_OK);
  CHECK(zrcNavMeshGetPolyArea(navmesh, ref, &area) == ZRC_OK);
  CHECK(area == 5);

  uint16_t flags = 0;
  CHECK(zrcNavMeshGetPolyFlags(navmesh, ref, &flags) == ZRC_OK);
  CHECK(flags == ZRC_POLY_FLAG_WALKABLE);
  CHECK(zrcNavMeshSetPolyFlags(navmesh, ref, 0) == ZRC_OK);
  CHECK(zrcNavMeshGetPolyFlags(navmesh, ref, &flags) == ZRC_OK);
  CHECK(flags == 0);

  /* Refusals: a zero reference, an out-of-range area, a NULL out-pointer. */
  CHECK(zrcNavMeshGetPolyArea(navmesh, 0, &area) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshGetPolyFlags(navmesh, 0, &flags) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshSetPolyArea(navmesh, 0, 1) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshSetPolyFlags(navmesh, 0, 1) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshSetPolyArea(navmesh, ref, ZRC_MAX_AREAS) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshSetPolyArea(navmesh, ref, -1) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshGetPolyArea(navmesh, ref, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshGetPolyFlags(navmesh, ref, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshGetPolyArea(NULL, ref, &area) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcNavMeshSetPolyFlags(NULL, ref, 0) == ZRC_ERR_INVALID_ARGUMENT);

  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);
}

static void test_pipeline(void) {
  ZrcBakeConfig config;
  zrcBakeConfigDefault(&config);

  ZrcTriMesh mesh;
  zrcFixtureTriMesh(&mesh);
  CHECK(mesh.vert_count == ZRC_FIXTURE_VERT_COUNT);
  CHECK(mesh.tri_count == ZRC_FIXTURE_TRI_COUNT);

  char log[1024];
  ZrcBakeLog bake_log;
  bake_log.buffer = log;
  bake_log.capacity = sizeof(log);

  ZrcPolyMesh* poly = NULL;
  const ZrcResult baked = zrcPolyMeshBake(&config, &mesh, NULL, &bake_log, &poly);
  if (baked != ZRC_OK) {
    fprintf(stderr, "bake failed: %s\n%s\n", zrcResultName(baked), log);
    ++failures;
    return;
  }
  ZrcPolyMeshInfo baked_info;
  memset(&baked_info, 0, sizeof(baked_info));
  CHECK(zrcPolyMeshInfo(poly, &baked_info) == ZRC_OK);
  CHECK(baked_info.poly_count > 0);
  CHECK(baked_info.vert_count >= 3);
  CHECK(baked_info.detail_tri_count > 0);
  CHECK(baked_info.bmin[0] <= baked_info.bmax[0] &&
        baked_info.bmin[1] <= baked_info.bmax[1] &&
        baked_info.bmin[2] <= baked_info.bmax[2]);

  ZrcNavMesh* navmesh = NULL;
  CHECK(zrcNavMeshCreate(poly, NULL, &navmesh) == ZRC_OK);
  zrcPolyMeshDestroy(poly);
  if (navmesh == NULL) {
    ++failures;
    return;
  }
  CHECK(zrcNavMeshPolyCount(navmesh) > 0);

  /* Serialise, check the image stands on its own, and reload it. */
  void* image = NULL;
  size_t image_size = 0;
  CHECK(zrcNavMeshSerialize(navmesh, &image, &image_size) == ZRC_OK);
  CHECK(image != NULL);
  CHECK(image_size > 0);
  CHECK(zrcNavMeshValidate(image, image_size) == ZRC_OK);
  /* One byte short must be refused: Detour would read past the end. */
  CHECK(zrcNavMeshValidate(image, image_size - 1) == ZRC_ERR_BAD_FORMAT);

  /* The layout's own total must equal the buffer it was computed from. */
  ZrcTileLayout tile_layout;
  memset(&tile_layout, 0, sizeof(tile_layout));
  CHECK(zrcTileLayout(image, image_size, &tile_layout) == ZRC_OK);
  CHECK(tile_layout.total_size == (int64_t)image_size);

  ZrcNavMesh* reloaded = NULL;
  CHECK(zrcNavMeshDeserialize(image, image_size, &reloaded) == ZRC_OK);
  zrcFree(image);
  if (reloaded == NULL) {
    zrcNavMeshDestroy(navmesh);
    ++failures;
    return;
  }
  CHECK(zrcNavMeshPolyCount(reloaded) == zrcNavMeshPolyCount(navmesh));
  zrcNavMeshDestroy(navmesh);

  /* Query the reloaded mesh, which is what a runtime actually has. */
  ZrcNavMeshQuery* query = NULL;
  CHECK(zrcNavMeshQueryCreate(reloaded, 2048, &query) == ZRC_OK);
  if (query == NULL) {
    zrcNavMeshDestroy(reloaded);
    ++failures;
    return;
  }

  ZrcQueryFilter filter;
  zrcQueryFilterDefault(&filter);

  const float start[3] = {ZRC_FIXTURE_START_X, ZRC_FIXTURE_START_Y,
                          ZRC_FIXTURE_START_Z};
  const float goal[3] = {ZRC_FIXTURE_GOAL_X, ZRC_FIXTURE_GOAL_Y,
                         ZRC_FIXTURE_GOAL_Z};
  const float extents[3] = {2.f, 4.f, 2.f};

  ZrcPolyRef start_ref = 0, goal_ref = 0;
  float start_pt[3], goal_pt[3];
  CHECK(zrcFindNearestPoly(query, start, extents, &filter, &start_ref,
                           start_pt, NULL) == ZRC_OK);
  CHECK(zrcFindNearestPoly(query, goal, extents, &filter, &goal_ref, goal_pt,
                           NULL) == ZRC_OK);
  CHECK(start_ref != 0);
  CHECK(goal_ref != 0);

  if (start_ref != 0 && goal_ref != 0) {
    ZrcPolyRef corridor[256];
    int corridor_len = 0;
    ZrcBool partial = ZRC_TRUE;
    CHECK(zrcFindPath(query, start_ref, goal_ref, start_pt, goal_pt, &filter,
                      corridor, 256, &corridor_len, &partial) == ZRC_OK);
    CHECK(corridor_len > 0);
    CHECK(partial == ZRC_FALSE);
    CHECK(corridor[0] == start_ref);
    CHECK(corridor[corridor_len - 1] == goal_ref);

    float corners[64 * 3];
    unsigned char corner_flags[64];
    int corner_count = 0;
    CHECK(zrcFindStraightPath(query, start_pt, goal_pt, corridor, corridor_len,
                              0, corners, 64, corner_flags, 64, NULL, 0,
                              &corner_count, NULL) == ZRC_OK);
    /* The wall makes a two-point straight line impossible. */
    CHECK(corner_count >= 3);
    if (corner_count >= 3) {
      CHECK((corner_flags[0] & ZRC_STRAIGHTPATH_START) != 0);
      CHECK((corner_flags[corner_count - 1] & ZRC_STRAIGHTPATH_END) != 0);
    }

    /* A companion array shorter than the point array is an overflow inside
       Detour, so the C boundary has to refuse it on its own — and clear the
       out-parameter while doing so. */
    int rejected_count = 7;
    CHECK(zrcFindStraightPath(query, start_pt, goal_pt, corridor, corridor_len,
                              0, corners, 64, corner_flags, 2, NULL, 0,
                              &rejected_count, NULL) == ZRC_ERR_BUFFER_TOO_SMALL);
    CHECK(rejected_count == 0);
    ZrcPolyRef short_refs[2];
    CHECK(zrcFindStraightPath(query, start_pt, goal_pt, corridor, corridor_len,
                              0, corners, 64, NULL, 0, short_refs, 2,
                              &rejected_count, NULL) == ZRC_ERR_BUFFER_TOO_SMALL);

    /* The same corridor, found a few iterations at a time. Driven to
       completion it has to agree with the one-shot search exactly, or the
       slice is reading a different node pool than it wrote. */
    CHECK(zrcSlicedFindPathActive(query) == ZRC_FALSE);
    CHECK(zrcSlicedFindPathInit(query, start_ref, goal_ref, start_pt, goal_pt,
                                &filter, 0) == ZRC_OK);
    CHECK(zrcSlicedFindPathActive(query) == ZRC_TRUE);

    /* While a slice is in flight, a search that would clear the node pool
       underneath it is refused rather than corrupting it. */
    int blocked_len = 0;
    CHECK(zrcFindPath(query, start_ref, goal_ref, start_pt, goal_pt, &filter,
                      corridor, 256, &blocked_len,
                      NULL) == ZRC_ERR_SEARCH_IN_PROGRESS);
    /* And one that does not touch the pool still works. */
    ZrcPolyRef during = 0;
    CHECK(zrcFindNearestPoly(query, start, extents, &filter, &during, NULL,
                             NULL) == ZRC_OK);
    CHECK(during == start_ref);

    ZrcBool in_progress = ZRC_TRUE;
    int rounds = 0;
    while (in_progress == ZRC_TRUE && rounds < 1000) {
      int did = 0;
      CHECK(zrcSlicedFindPathUpdate(query, 4, &did, &in_progress) == ZRC_OK);
      rounds++;
    }
    CHECK(in_progress == ZRC_FALSE);
    /* Several rounds, or the budget was not being honoured. */
    CHECK(rounds > 1);

    ZrcPolyRef sliced[256];
    int sliced_len = 0;
    ZrcBool sliced_partial = ZRC_TRUE;
    CHECK(zrcSlicedFindPathFinalize(query, sliced, 256, &sliced_len,
                                    &sliced_partial) == ZRC_OK);
    CHECK(sliced_partial == ZRC_FALSE);
    CHECK(sliced_len == corridor_len);
    for (int i = 0; i < sliced_len; ++i) CHECK(sliced[i] == corridor[i]);

    /* Finalising again is refused. Upstream would zero its own state, misread
       that as "start and end are the same polygon" and hand back a one-element
       path holding the null reference. */
    int again_len = 7;
    CHECK(zrcSlicedFindPathFinalize(query, sliced, 256, &again_len,
                                    NULL) == ZRC_ERR_NO_SEARCH);
    CHECK(again_len == 0);
    CHECK(zrcSlicedFindPathActive(query) == ZRC_FALSE);
    /* And the pool is free again. */
    CHECK(zrcFindPath(query, start_ref, goal_ref, start_pt, goal_pt, &filter,
                      corridor, 256, &corridor_len, NULL) == ZRC_OK);

    /* A reference splits into the three fields packed into it and rebuilds
       from them. */
    unsigned int salt = 0, tile_index = 0, poly_index = 0;
    CHECK(zrcDecodePolyRef(reloaded, start_ref, &salt, &tile_index,
                           &poly_index) == ZRC_OK);
    ZrcPolyRef rebuilt = 0;
    CHECK(zrcEncodePolyRef(reloaded, salt, tile_index, poly_index,
                           &rebuilt) == ZRC_OK);
    CHECK(rebuilt == start_ref);
    /* A field too wide for this navmesh's own layout is refused, where
       upstream would shift it away silently. */
    CHECK(zrcEncodePolyRef(reloaded, salt, tile_index, 0xffffu, &rebuilt) ==
          ZRC_ERR_INVALID_ARGUMENT);
    CHECK(rebuilt == 0);

    ZrcBool valid = ZRC_FALSE;
    CHECK(zrcIsValidPolyRef(query, start_ref, &filter, &valid) == ZRC_OK);
    CHECK(valid == ZRC_TRUE);
    /* A NULL filter is the liveness question on its own, which upstream
       answers through a different function entirely. */
    CHECK(zrcIsValidPolyRef(query, start_ref, NULL, &valid) == ZRC_OK);
    CHECK(valid == ZRC_TRUE);
    CHECK(zrcIsValidPolyRef(query, 0, NULL, &valid) == ZRC_OK);
    CHECK(valid == ZRC_FALSE);

    /* The search that just ran left its node pool behind. */
    ZrcNodePoolInfo pool;
    CHECK(zrcQueryNodePoolInfo(query, &pool) == ZRC_OK);
    CHECK(pool.max_nodes > 0);
    CHECK(pool.node_count > 0);
    CHECK(pool.node_count <= pool.max_nodes);
    ZrcNode node;
    CHECK(zrcQueryFindNode(query, start_ref, 0, &node) == ZRC_OK);
    CHECK(node.ref == start_ref);
    /* Index 0 means no node, and an index past what the search populated is
       not a node either — upstream would read uninitialised storage. */
    CHECK(zrcQueryNodeAt(query, 0, &node) == ZRC_ERR_NOT_FOUND);
    CHECK(zrcQueryNodeAt(query, (unsigned int)pool.node_count + 1u, &node) ==
          ZRC_ERR_NOT_FOUND);

    float moved[3];
    ZrcPolyRef visited[32];
    int visited_count = 0;
    ZrcBool clipped = ZRC_TRUE;
    CHECK(zrcMoveAlongSurface(query, start_ref, start_pt, goal, &filter, moved,
                              visited, 32, &visited_count,
                              &clipped) == ZRC_OK);
    CHECK(clipped == ZRC_FALSE);
    CHECK(visited_count > 0);
    /* Stopped by the wall at z = 0, well short of the goal at z = +8. */
    CHECK(moved[2] < 0.5f);

    ZrcRaycastHit hit;
    CHECK(zrcRaycast(query, start_ref, start_pt, goal, &filter, 0, 0, &hit,
                     NULL, 0, NULL, NULL) == ZRC_OK);
    CHECK(hit.hit == ZRC_TRUE);
    CHECK(hit.t > 0.f && hit.t < 1.f);
  }

  /* Arguments that would fault inside Detour. */
  ZrcPolyRef corridor[4];
  int len = 0;
  CHECK(zrcFindPath(query, 0, goal_ref, start_pt, goal_pt, &filter, corridor, 4,
                    &len, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcFindPath(query, start_ref, goal_ref, start_pt, goal_pt, &filter,
                    corridor, 0, &len, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  /* Into a separate handle: a rejected create clears its out-parameter, which
     would otherwise drop the live query on the floor. */
  ZrcNavMeshQuery* rejected = (ZrcNavMeshQuery*)0x1;
  CHECK(zrcNavMeshQueryCreate(reloaded, 0, &rejected) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(rejected == NULL);

  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(reloaded);
}

//===----------------------------------------------------------------------===//
// The staged pipeline: the same bake, taken apart into its own entry points
// over host-owned containers.
//===----------------------------------------------------------------------===//

static int pipeline_log_calls = 0;
static int pipeline_timer_start_calls = 0;
static int pipeline_timer_stop_calls = 0;

static void pipeline_log(void* user, ZrcLogCategory category,
                          const char* message, int32_t length) {
  (void)user;
  (void)category;
  (void)message;
  (void)length;
  ++pipeline_log_calls;
}

static void pipeline_start_timer(void* user, ZrcTimerLabel label) {
  (void)user;
  (void)label;
  ++pipeline_timer_start_calls;
}

static void pipeline_stop_timer(void* user, ZrcTimerLabel label) {
  (void)user;
  (void)label;
  ++pipeline_timer_stop_calls;
}

/// Builds a polygon mesh stage by stage from the fixture geometry, with a
/// build context installed throughout, and proves the result is a real
/// navmesh rather than a set of containers that merely filled in.
static void test_staged_pipeline(void) {
  pipeline_log_calls = 0;
  pipeline_timer_start_calls = 0;
  pipeline_timer_stop_calls = 0;

  ZrcBuildContext build_ctx;
  memset(&build_ctx, 0, sizeof(build_ctx));
  build_ctx.log = pipeline_log;
  build_ctx.start_timer = pipeline_start_timer;
  build_ctx.stop_timer = pipeline_stop_timer;
  build_ctx.log_enabled = ZRC_TRUE;
  build_ctx.timers_enabled = ZRC_TRUE;

  ZrcTriMesh mesh;
  zrcFixtureTriMesh(&mesh);

  float bmin[3], bmax[3];
  CHECK(zrcCalcBounds(&mesh, bmin, bmax) == ZRC_OK);

  /// The same human-sized agent zrcBakeConfigDefault describes, converted to
  /// voxels by hand: the staged entry points take cells, not world units.
  const float cell_size = 0.3f;
  const float cell_height = 0.2f;
  const float agent_max_slope = 45.0f;
  const int32_t walkable_height = (int32_t)ceilf(2.0f / cell_height);
  const int32_t walkable_climb = (int32_t)floorf(0.9f / cell_height);
  const int32_t walkable_radius = (int32_t)ceilf(0.6f / cell_size);

  int32_t width = 0, height = 0;
  CHECK(zrcCalcGridSize(bmin, bmax, cell_size, &width, &height) == ZRC_OK);
  CHECK(width > 0 && height > 0);

  ZrcHeightfield* hf = NULL;
  CHECK(zrcHeightfieldCreate(&build_ctx, width, height, bmin, bmax, cell_size,
                             cell_height, &hf) == ZRC_OK);
  if (hf == NULL) {
    ++failures;
    return;
  }

  uint8_t* tri_areas = (uint8_t*)malloc((size_t)mesh.tri_count);
  CHECK(tri_areas != NULL);
  memset(tri_areas, 0, (size_t)mesh.tri_count);
  CHECK(zrcMarkWalkableTriangles(&build_ctx, agent_max_slope, &mesh,
                                 tri_areas) == ZRC_OK);
  CHECK(zrcHeightfieldRasterizeTriangles(&build_ctx, hf, &mesh, tri_areas,
                                         walkable_climb) == ZRC_OK);
  free(tri_areas);

  CHECK(zrcHeightfieldFilterLowHangingObstacles(&build_ctx, hf,
                                                walkable_climb) == ZRC_OK);
  CHECK(zrcHeightfieldFilterLedgeSpans(&build_ctx, hf, walkable_height,
                                       walkable_climb) == ZRC_OK);
  CHECK(zrcHeightfieldFilterWalkableLowHeightSpans(&build_ctx, hf,
                                                   walkable_height) ==
        ZRC_OK);

  ZrcCompactHeightfield* chf = NULL;
  CHECK(zrcCompactHeightfieldCreate(&build_ctx, walkable_height,
                                    walkable_climb, hf, &chf) == ZRC_OK);
  zrcHeightfieldDestroy(hf);
  if (chf == NULL) {
    ++failures;
    return;
  }

  CHECK(zrcCompactHeightfieldErode(&build_ctx, chf, walkable_radius) ==
        ZRC_OK);
  CHECK(zrcCompactHeightfieldBuildDistanceField(&build_ctx, chf) == ZRC_OK);
  CHECK(zrcCompactHeightfieldBuildRegions(&build_ctx, chf,
                                          ZRC_PARTITION_WATERSHED, 0, 64,
                                          400) == ZRC_OK);

  ZrcContourSet* contours = NULL;
  CHECK(zrcContourSetCreate(&build_ctx, chf, 1.3f, 40,
                            ZRC_CONTOUR_TESS_WALL_EDGES, &contours) ==
        ZRC_OK);
  if (contours == NULL) {
    zrcCompactHeightfieldDestroy(chf);
    ++failures;
    return;
  }

  ZrcPolyMesh* poly = NULL;
  CHECK(zrcPolyMeshCreate(&poly) == ZRC_OK);
  if (poly == NULL) {
    zrcContourSetDestroy(contours);
    zrcCompactHeightfieldDestroy(chf);
    ++failures;
    return;
  }

  CHECK(zrcPolyMeshBuild(&build_ctx, contours, ZRC_VERTS_PER_POLYGON, poly) ==
        ZRC_OK);
  CHECK(zrcPolyMeshBuildDetail(&build_ctx, poly, chf, 1.8f, 0.2f) == ZRC_OK);

  zrcContourSetDestroy(contours);
  zrcCompactHeightfieldDestroy(chf);

  ZrcPolyMeshInfo poly_info;
  memset(&poly_info, 0, sizeof(poly_info));
  CHECK(zrcPolyMeshInfo(poly, &poly_info) == ZRC_OK);
  CHECK(poly_info.poly_count > 0);
  CHECK(poly_info.detail_tri_count > 0);

  /* Recast does not carry these; a mesh assembled by hand has to be told, or
     zrcNavMeshCreate refuses it. Quantised to the voxel grid actually built,
     as the whole-bake pipeline does. */
  CHECK(zrcPolyMeshSetAgentDims(poly, (float)walkable_height * cell_height,
                                (float)walkable_radius * cell_size,
                                (float)walkable_climb * cell_height) ==
        ZRC_OK);

  /* A hand-built mesh comes out with every flag zero, which no query filter
     admits. */
  uint16_t* flags =
      (uint16_t*)malloc(sizeof(uint16_t) * (size_t)poly_info.poly_count);
  CHECK(flags != NULL);
  for (int32_t i = 0; i < poly_info.poly_count; ++i) {
    flags[i] = ZRC_POLY_FLAG_WALKABLE;
  }
  CHECK(zrcPolyMeshSetPolyFlags(poly, 0, poly_info.poly_count, flags) ==
        ZRC_OK);
  free(flags);

  ZrcNavMesh* navmesh = NULL;
  CHECK(zrcNavMeshCreate(poly, NULL, &navmesh) == ZRC_OK);
  zrcPolyMeshDestroy(poly);
  if (navmesh == NULL) {
    ++failures;
    return;
  }

  ZrcNavMeshQuery* query = NULL;
  CHECK(zrcNavMeshQueryCreate(navmesh, 2048, &query) == ZRC_OK);
  if (query == NULL) {
    zrcNavMeshDestroy(navmesh);
    ++failures;
    return;
  }

  /* The proof the staged mesh is a real navmesh, not merely a struct that
     filled in: a query that actually finds a polygon. */
  ZrcQueryFilter filter;
  zrcQueryFilterDefault(&filter);
  const float center[3] = {0.0f, 0.0f, -8.0f};
  const float extents[3] = {2.0f, 4.0f, 2.0f};
  ZrcPolyRef ref = 0;
  float nearest[3];
  CHECK(zrcFindNearestPoly(query, center, extents, &filter, &ref, nearest,
                           NULL) == ZRC_OK);
  CHECK(ref != 0);

  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);

  /* A clean build logs nothing: Recast's log hook fires only for warnings
     and errors, and this fixture produces neither. The timer hooks are what
     actually fired, once per stage started and stopped above. */
  CHECK(pipeline_log_calls == 0);
  CHECK(pipeline_timer_start_calls > 0);
  CHECK(pipeline_timer_stop_calls > 0);
  CHECK(pipeline_timer_start_calls == pipeline_timer_stop_calls);
}

/// The error contracts a tool assembling a mesh by hand depends on most: a
/// container that refuses to be built into twice, a partition that refuses to
/// read a distance field that was never built, and the bounds checks a
/// heightfield's own span storage relies on.
static void test_staged_pipeline_errors(void) {
  const float cell_size = 0.3f;
  const float cell_height = 0.2f;

  /* zrcPolyMeshBuild twice into the same mesh: the second call has to be
     refused, or the first call's buffers would be leaked out from under it.
     A real build is needed so the poly half is genuinely filled — an empty
     contour set would leave a mesh indistinguishable from a fresh one. */
  ZrcTriMesh mesh;
  zrcFixtureTriMesh(&mesh);
  float mesh_bmin[3], mesh_bmax[3];
  CHECK(zrcCalcBounds(&mesh, mesh_bmin, mesh_bmax) == ZRC_OK);
  int32_t mesh_width = 0, mesh_height = 0;
  CHECK(zrcCalcGridSize(mesh_bmin, mesh_bmax, cell_size, &mesh_width,
                        &mesh_height) == ZRC_OK);

  ZrcHeightfield* build_hf = NULL;
  CHECK(zrcHeightfieldCreate(NULL, mesh_width, mesh_height, mesh_bmin,
                             mesh_bmax, cell_size, cell_height, &build_hf) ==
        ZRC_OK);
  uint8_t* areas = (uint8_t*)malloc((size_t)mesh.tri_count);
  CHECK(areas != NULL);
  memset(areas, 0, (size_t)mesh.tri_count);
  CHECK(zrcMarkWalkableTriangles(NULL, 45.0f, &mesh, areas) == ZRC_OK);
  CHECK(zrcHeightfieldRasterizeTriangles(NULL, build_hf, &mesh, areas, 4) ==
        ZRC_OK);
  free(areas);

  ZrcCompactHeightfield* build_chf = NULL;
  CHECK(zrcCompactHeightfieldCreate(NULL, 10, 4, build_hf, &build_chf) ==
        ZRC_OK);
  zrcHeightfieldDestroy(build_hf);
  CHECK(zrcCompactHeightfieldBuildDistanceField(NULL, build_chf) == ZRC_OK);
  CHECK(zrcCompactHeightfieldBuildRegions(NULL, build_chf,
                                          ZRC_PARTITION_WATERSHED, 0, 64,
                                          400) == ZRC_OK);

  ZrcContourSet* contours = NULL;
  CHECK(zrcContourSetCreate(NULL, build_chf, 1.3f, 40,
                            ZRC_CONTOUR_TESS_WALL_EDGES, &contours) ==
        ZRC_OK);
  zrcCompactHeightfieldDestroy(build_chf);
  if (contours == NULL) {
    ++failures;
    return;
  }

  ZrcPolyMesh* poly = NULL;
  CHECK(zrcPolyMeshCreate(&poly) == ZRC_OK);
  if (poly == NULL) {
    zrcContourSetDestroy(contours);
    ++failures;
    return;
  }
  CHECK(zrcPolyMeshBuild(NULL, contours, ZRC_VERTS_PER_POLYGON, poly) ==
        ZRC_OK);
  CHECK(zrcPolyMeshBuild(NULL, contours, ZRC_VERTS_PER_POLYGON, poly) ==
        ZRC_ERR_ALREADY_BUILT);
  zrcContourSetDestroy(contours);
  zrcPolyMeshDestroy(poly);

  /* A small heightfield of its own, for the span and column contracts: two
     spans in one column, well short of the field's own extent. */
  const float small_bmin[3] = {0.f, 0.f, 0.f};
  const float small_bmax[3] = {4.f * cell_size, 10.f * cell_height,
                               4.f * cell_size};
  ZrcHeightfield* hf = NULL;
  CHECK(zrcHeightfieldCreate(NULL, 4, 4, small_bmin, small_bmax, cell_size,
                             cell_height, &hf) == ZRC_OK);
  if (hf == NULL) {
    ++failures;
    return;
  }

  CHECK(zrcHeightfieldAddSpan(NULL, hf, 0, 0, 0, 2, ZRC_AREA_WALKABLE, 1) ==
        ZRC_OK);
  CHECK(zrcHeightfieldAddSpan(NULL, hf, 0, 0, 5, 8, ZRC_AREA_WALKABLE, 1) ==
        ZRC_OK);

  /* Width is 4, so x = 4 is one past the field's own extent. */
  CHECK(zrcHeightfieldAddSpan(NULL, hf, 4, 0, 0, 2, ZRC_AREA_WALKABLE, 1) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* Two spans exist in the column; a buffer that holds one is too small, and
     the count reports the whole column regardless. */
  ZrcSpan short_column[1];
  int32_t column_count = 0;
  CHECK(zrcHeightfieldColumn(hf, 0, 0, short_column, 1, &column_count) ==
        ZRC_ERR_BUFFER_TOO_SMALL);
  CHECK(column_count == 2);

  ZrcCompactHeightfield* chf = NULL;
  CHECK(zrcCompactHeightfieldCreate(NULL, 10, 4, hf, &chf) == ZRC_OK);
  zrcHeightfieldDestroy(hf);
  if (chf == NULL) {
    ++failures;
    return;
  }

  /* Watershed regions read the distance field; this one was never built. */
  CHECK(zrcCompactHeightfieldBuildRegions(NULL, chf, ZRC_PARTITION_WATERSHED,
                                          0, 8, 20) == ZRC_ERR_INVALID_ARGUMENT);
  /* Neither does the distance array itself exist yet. */
  CHECK(zrcCompactHeightfieldDistances(chf, 0, 0, NULL) == ZRC_ERR_NOT_FOUND);
  /* A NULL build context, taken by a stage function, changes nothing about
     the result. */
  CHECK(zrcCompactHeightfieldMedianFilter(NULL, chf) == ZRC_OK);

  zrcCompactHeightfieldDestroy(chf);

  /* ZRC_MAX_TIMERS is the length of the table upstream indexes with a label,
     not a label itself. */
  CHECK(zrcBuildContextStartTimer(NULL, ZRC_MAX_TIMERS) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* Every direct build-context entry point also takes a NULL context and
     does nothing rather than faulting. */
  CHECK(zrcBuildContextLog(NULL, ZRC_LOG_PROGRESS, "no context installed") ==
        ZRC_OK);
  CHECK(zrcBuildContextResetLog(NULL) == ZRC_OK);
  CHECK(zrcBuildContextResetTimers(NULL) == ZRC_OK);
  CHECK(zrcBuildContextStopTimer(NULL, ZRC_TIMER_TOTAL) == ZRC_OK);
  int32_t elapsed = 7;
  CHECK(zrcBuildContextAccumulatedTime(NULL, ZRC_TIMER_TOTAL, &elapsed) ==
        ZRC_OK);
  CHECK(elapsed == -1);
}

//===----------------------------------------------------------------------===//
// The tile cache: cooking layers from the fixture, loading them, carving an
// obstacle into a baked mesh at runtime, and taking it back out again.
//===----------------------------------------------------------------------===//

static int tilecache_compress_calls = 0;
static int tilecache_decompress_calls = 0;

/// Stores each buffer unchanged. No codec ships with this package, so the
/// suite supplies the simplest one that satisfies the interface: every byte
/// a cook compresses has to come back out of the decompressor a rebuild
/// calls.
static int32_t tilecache_max_compressed_size(void* user, int32_t buffer_size) {
  (void)user;
  return buffer_size;
}

static ZrcResult tilecache_compress(void* user, const uint8_t* buffer,
                                    int32_t buffer_size, uint8_t* compressed,
                                    int32_t max_compressed_size,
                                    int32_t* out_compressed_size) {
  (void)user;
  if (buffer_size > max_compressed_size) return ZRC_ERR_BUFFER_TOO_SMALL;
  memcpy(compressed, buffer, (size_t)buffer_size);
  *out_compressed_size = buffer_size;
  ++tilecache_compress_calls;
  return ZRC_OK;
}

static ZrcResult tilecache_decompress(void* user, const uint8_t* compressed,
                                      int32_t compressed_size, uint8_t* buffer,
                                      int32_t max_buffer_size,
                                      int32_t* out_size) {
  (void)user;
  if (compressed_size > max_buffer_size) return ZRC_ERR_BUFFER_TOO_SMALL;
  memcpy(buffer, compressed, (size_t)compressed_size);
  *out_size = compressed_size;
  ++tilecache_decompress_calls;
  return ZRC_OK;
}

static ZrcTileCacheCompressor tilecache_store_compressor(void) {
  ZrcTileCacheCompressor compressor;
  compressor.user = NULL;
  compressor.max_compressed_size = tilecache_max_compressed_size;
  compressor.compress = tilecache_compress;
  compressor.decompress = tilecache_decompress;
  return compressor;
}

static int tilecache_mesh_process_calls = 0;

/// Gives every polygon of a rebuilt tile the walkable flag, unless its area
/// is the null area. Recast leaves the flags zero and Detour copies them
/// verbatim, so a tile cache with no callback produces tiles no nonzero
/// filter admits.
static ZrcResult tilecache_mesh_process(void* user,
                                        ZrcTileCacheBuildParams* params) {
  (void)user;
  ++tilecache_mesh_process_calls;
  for (int32_t i = 0; i < params->poly_count; ++i) {
    params->flags[i] =
        params->areas[i] == ZRC_AREA_NULL ? 0 : ZRC_POLY_FLAG_WALKABLE;
  }
  return ZRC_OK;
}

/// Cooks one tile of the grid into compressed layers and adds them to
/// `cache`: builds a heightfield over the tile's bounds plus the border,
/// rasterizes, filters, builds a compact heightfield, erodes, then cuts it
/// into layers.
///
/// Mirrors CachedWorld.cookTile in src/integration_test.zig; the arithmetic
/// here has to match it exactly, or the tiles do not line up and nothing
/// says so.
static int32_t tilecache_cook_tile(ZrcTileCache* cache,
                                   const ZrcBakeConfig* config,
                                   const ZrcBuildCells* cells,
                                   const ZrcTileGrid* grid,
                                   const ZrcTriMesh* mesh, int32_t tile_x,
                                   int32_t tile_y) {
  const float edge = grid->tile_world_size;
  const float border_world = (float)cells->border_size * config->cell_size;
  const float bmin[3] = {
      grid->origin[0] + (float)tile_x * edge - border_world,
      grid->origin[1],
      grid->origin[2] + (float)tile_y * edge - border_world,
  };
  const float bmax[3] = {
      grid->origin[0] + (float)(tile_x + 1) * edge + border_world,
      grid->extent_max[1],
      grid->origin[2] + (float)(tile_y + 1) * edge + border_world,
  };
  const int32_t side = config->tile_size + cells->border_size * 2;

  ZrcHeightfield* field = NULL;
  CHECK(zrcHeightfieldCreate(NULL, side, side, bmin, bmax, config->cell_size,
                             config->cell_height, &field) == ZRC_OK);
  if (field == NULL) {
    ++failures;
    return 0;
  }

  uint8_t* tri_areas = (uint8_t*)malloc((size_t)mesh->tri_count);
  CHECK(tri_areas != NULL);
  memset(tri_areas, ZRC_AREA_NULL, (size_t)mesh->tri_count);
  CHECK(zrcMarkWalkableTriangles(NULL, config->agent_max_slope, mesh,
                                 tri_areas) == ZRC_OK);
  CHECK(zrcHeightfieldRasterizeTriangles(NULL, field, mesh, tri_areas,
                                         cells->walkable_climb) == ZRC_OK);
  free(tri_areas);

  CHECK(zrcHeightfieldFilterLowHangingObstacles(NULL, field,
                                                cells->walkable_climb) ==
        ZRC_OK);
  CHECK(zrcHeightfieldFilterLedgeSpans(NULL, field, cells->walkable_height,
                                       cells->walkable_climb) == ZRC_OK);
  CHECK(zrcHeightfieldFilterWalkableLowHeightSpans(
            NULL, field, cells->walkable_height) == ZRC_OK);

  ZrcCompactHeightfield* compact = NULL;
  CHECK(zrcCompactHeightfieldCreate(NULL, cells->walkable_height,
                                    cells->walkable_climb, field,
                                    &compact) == ZRC_OK);
  zrcHeightfieldDestroy(field);
  if (compact == NULL) {
    ++failures;
    return 0;
  }

  if (cells->walkable_radius > 0) {
    CHECK(zrcCompactHeightfieldErode(NULL, compact, cells->walkable_radius) ==
          ZRC_OK);
  }

  ZrcHeightfieldLayerSet* layers = NULL;
  CHECK(zrcHeightfieldLayerSetCreate(NULL, compact, cells->border_size,
                                     cells->walkable_height, &layers) ==
        ZRC_OK);
  zrcCompactHeightfieldDestroy(compact);
  if (layers == NULL) {
    ++failures;
    return 0;
  }

  int32_t layer_count = 0;
  CHECK(zrcHeightfieldLayerSetCount(layers, &layer_count) == ZRC_OK);

  ZrcTileCacheCompressor compressor = tilecache_store_compressor();
  int32_t added = 0;
  for (int32_t i = 0; i < layer_count; ++i) {
    ZrcHeightfieldLayer layer;
    memset(&layer, 0, sizeof(layer));
    CHECK(zrcHeightfieldLayerAt(layers, i, &layer) == ZRC_OK);

    const size_t n = (size_t)layer.width * (size_t)layer.height;
    uint8_t* heights = (uint8_t*)malloc(n);
    uint8_t* areas = (uint8_t*)malloc(n);
    uint8_t* cons = (uint8_t*)malloc(n);
    CHECK(heights != NULL && areas != NULL && cons != NULL);
    CHECK(zrcHeightfieldLayerHeights(layers, i, 0, (int32_t)n, heights) ==
          ZRC_OK);
    CHECK(zrcHeightfieldLayerAreas(layers, i, 0, (int32_t)n, areas) ==
          ZRC_OK);
    CHECK(zrcHeightfieldLayerCons(layers, i, 0, (int32_t)n, cons) == ZRC_OK);

    ZrcTileCacheLayerHeader header;
    memset(&header, 0, sizeof(header));
    header.tile_x = tile_x;
    header.tile_y = tile_y;
    header.tile_layer = i;
    memcpy(header.bmin, layer.bmin, sizeof(header.bmin));
    memcpy(header.bmax, layer.bmax, sizeof(header.bmax));
    header.height_min = layer.height_min;
    header.height_max = layer.height_max;
    header.width = layer.width;
    header.height = layer.height;
    header.min_x = layer.min_x;
    header.max_x = layer.max_x;
    header.min_z = layer.min_z;
    header.max_z = layer.max_z;

    void* data = NULL;
    size_t size = 0;
    CHECK(zrcTileCacheLayerBuild(&compressor, &header, heights, areas, cons,
                                 &data, &size) == ZRC_OK);
    free(heights);
    free(areas);
    free(cons);

    ZrcCompressedTileRef ref = 0;
    CHECK(zrcTileCacheAddTile(cache, data, size, &ref) == ZRC_OK);
    zrcFree(data);
    ++added;
  }

  zrcHeightfieldLayerSetDestroy(layers);
  return added;
}

/// Drives zrcTileCacheUpdate to completion, the way a frame would spend a
/// budget on it. Bounded so a cache that never settles fails the test rather
/// than hanging it.
static void tilecache_settle(ZrcTileCache* cache, ZrcNavMesh* navmesh) {
  ZrcBool up_to_date = ZRC_FALSE;
  int spins = 0;
  while (spins < 512 && up_to_date == ZRC_FALSE) {
    CHECK(zrcTileCacheUpdate(cache, navmesh, &up_to_date) == ZRC_OK);
    ++spins;
  }
  CHECK(up_to_date == ZRC_TRUE);
}

/// Cooks the fixture into a tile cache, builds it into a tiled navmesh,
/// carves an obstacle into the gap in the fixture's wall, and takes it back
/// out again — the arc a game walks for a door, a destructible, or a bridge
/// that collapses.
static void test_tile_cache(void) {
  tilecache_compress_calls = 0;
  tilecache_decompress_calls = 0;
  tilecache_mesh_process_calls = 0;

  ZrcBakeConfig config;
  zrcBakeConfigDefault(&config);
  /* 32 voxels at the default 0.3 m cell size puts a tile edge at 9.6 m,
     which divides the fixture's 20 m ground plane into several tiles. */
  config.tile_size = 32;

  ZrcTriMesh mesh;
  zrcFixtureTriMesh(&mesh);

  ZrcTileGrid grid;
  memset(&grid, 0, sizeof(grid));
  CHECK(zrcTileGridCompute(&config, &mesh, &grid) == ZRC_OK);

  ZrcBuildCells cells;
  memset(&cells, 0, sizeof(cells));
  CHECK(zrcBakeConfigCells(&config, &cells) == ZRC_OK);

  const int32_t tiles_per_axis = grid.tile_count_x * grid.tile_count_z;

  ZrcTileCacheParams cache_params;
  memset(&cache_params, 0, sizeof(cache_params));
  memcpy(cache_params.origin, grid.origin, sizeof(cache_params.origin));
  cache_params.cell_size = config.cell_size;
  cache_params.cell_height = config.cell_height;
  cache_params.width = config.tile_size;
  cache_params.height = config.tile_size;
  cache_params.walkable_height =
      (float)cells.walkable_height * config.cell_height;
  cache_params.walkable_radius =
      (float)cells.walkable_radius * config.cell_size;
  cache_params.walkable_climb =
      (float)cells.walkable_climb * config.cell_height;
  cache_params.max_simplification_error = cells.max_simplification_error;
  cache_params.max_tiles = tiles_per_axis * 4;
  cache_params.max_obstacles = 64;

  ZrcTileCacheCompressor compressor = tilecache_store_compressor();
  ZrcTileCache* cache = NULL;
  CHECK(zrcTileCacheCreate(&cache_params, &compressor, NULL,
                           tilecache_mesh_process, NULL, &cache) == ZRC_OK);
  if (cache == NULL) {
    ++failures;
    return;
  }

  int32_t total_layers = 0;
  for (int32_t z = 0; z < grid.tile_count_z; ++z) {
    for (int32_t x = 0; x < grid.tile_count_x; ++x) {
      total_layers +=
          tilecache_cook_tile(cache, &config, &cells, &grid, &mesh, x, z);
    }
  }
  CHECK(total_layers > 0);

  ZrcNavMesh* navmesh = NULL;
  CHECK(zrcNavMeshCreateTiled(&grid, cache_params.max_tiles, 1 << 14,
                              &navmesh) == ZRC_OK);
  if (navmesh == NULL) {
    zrcTileCacheDestroy(cache);
    ++failures;
    return;
  }

  for (int32_t z = 0; z < grid.tile_count_z; ++z) {
    for (int32_t x = 0; x < grid.tile_count_x; ++x) {
      CHECK(zrcTileCacheBuildNavMeshTilesAt(cache, x, z, navmesh) == ZRC_OK);
    }
  }

  /* The cook and the first build both went through the host's codec, and
     every tile went through the host's mesh-process callback. Without the
     callback the tiles are well formed and invisible to every filter. */
  CHECK(tilecache_compress_calls > 0);
  CHECK(tilecache_decompress_calls > 0);
  CHECK(tilecache_mesh_process_calls > 0);

  ZrcNavMeshQuery* query = NULL;
  CHECK(zrcNavMeshQueryCreate(navmesh, 4096, &query) == ZRC_OK);
  if (query == NULL) {
    zrcNavMeshDestroy(navmesh);
    zrcTileCacheDestroy(cache);
    ++failures;
    return;
  }

  ZrcQueryFilter filter;
  zrcQueryFilterDefault(&filter);
  const float start[3] = {ZRC_FIXTURE_START_X, ZRC_FIXTURE_START_Y,
                          ZRC_FIXTURE_START_Z};
  const float extents[3] = {2.f, 4.f, 2.f};
  ZrcPolyRef start_ref = 0;
  float start_pt[3];
  /* The proof the cached mesh is a real navmesh, not merely a set of
     containers that filled in. */
  CHECK(zrcFindNearestPoly(query, start, extents, &filter, &start_ref,
                           start_pt, NULL) == ZRC_OK);
  CHECK(start_ref != 0);

  /* The fixture's wall runs from the -x edge to ZRC_FIXTURE_WALL_END_X,
     leaving a gap to plug. */
  const float gap_min[3] = {ZRC_FIXTURE_WALL_END_X - 1.0f, -1.0f, -1.5f};
  const float gap_max[3] = {ZRC_FIXTURE_GROUND_EXTENT + 1.0f, 3.0f, 1.5f};
  ZrcObstacleRef obstacle = 0;
  CHECK(zrcTileCacheAddBoxObstacle(cache, gap_min, gap_max, &obstacle) ==
        ZRC_OK);
  CHECK(obstacle != 0);

  /* Which slot the obstacle landed in, found by scanning rather than assumed
     from the allocator's own internal order. */
  int32_t obstacle_index = -1;
  for (int32_t i = 0; i < cache_params.max_obstacles; ++i) {
    ZrcObstacleRef candidate = 0;
    CHECK(zrcTileCacheObstacleRefAt(cache, i, &candidate) == ZRC_OK);
    if (candidate == obstacle) {
      obstacle_index = i;
      break;
    }
  }
  CHECK(obstacle_index >= 0);

  /* Queued, not applied: upstream fills touched_count only on the first
     update that processes the request, so an obstacle read back immediately
     reports the shape it was given and nothing about where it landed. */
  ZrcObstacleInfo queued;
  memset(&queued, 0xAB, sizeof(queued));
  CHECK(zrcTileCacheObstacleInfo(cache, obstacle, &queued) == ZRC_OK);
  CHECK(queued.shape == ZRC_OBSTACLE_BOX);
  CHECK(queued.state == ZRC_OBSTACLE_PROCESSING);
  CHECK(queued.touched_count == 0);

  tilecache_settle(cache, navmesh);

  ZrcObstacleInfo carved;
  memset(&carved, 0xAB, sizeof(carved));
  CHECK(zrcTileCacheObstacleInfo(cache, obstacle, &carved) == ZRC_OK);
  CHECK(carved.state == ZRC_OBSTACLE_PROCESSED);
  CHECK(carved.touched_count > 0);

  CHECK(zrcTileCacheRemoveObstacle(cache, obstacle) == ZRC_OK);
  tilecache_settle(cache, navmesh);

  /* The reference's salt turns over in the very update that empties the
     slot (DetourTileCache.cpp), so the old reference no longer names
     anything resident. */
  ZrcObstacleInfo gone;
  memset(&gone, 0xAB, sizeof(gone));
  CHECK(zrcTileCacheObstacleInfo(cache, obstacle, &gone) == ZRC_ERR_NOT_FOUND);

  /* The slot itself reads back as free: zrcTileCacheObstacleRefAt reports 0
     for a slot whose state is ZRC_OBSTACLE_EMPTY, and for no other slot. */
  ZrcObstacleRef ref_at_slot = 1;
  CHECK(zrcTileCacheObstacleRefAt(cache, obstacle_index, &ref_at_slot) ==
        ZRC_OK);
  CHECK(ref_at_slot == 0);

  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);
  zrcTileCacheDestroy(cache);
}

/// The error contracts a host driving the tile cache by hand depends on: a
/// layer buffer too short or wrongly tagged to be a layer, an index an
/// accessor does not bound on its own, a reference naming nothing resident,
/// and the parameter checks zrcTileCacheCreate applies before it allocates
/// anything.
static void test_tile_cache_errors(void) {
  ZrcTileCacheParams params;
  memset(&params, 0, sizeof(params));
  params.cell_size = 0.3f;
  params.cell_height = 0.2f;
  params.width = 32;
  params.height = 32;
  params.walkable_height = 2.0f;
  params.walkable_radius = 0.6f;
  params.walkable_climb = 0.9f;
  params.max_simplification_error = 1.3f;
  params.max_tiles = 16;
  params.max_obstacles = 16;

  ZrcTileCacheCompressor compressor = tilecache_store_compressor();

  /* A NULL compressor is refused: upstream calls into it while rebuilding
     every tile. The out-parameter must be cleared even on failure. */
  ZrcTileCache* rejected = (ZrcTileCache*)0x1;
  CHECK(zrcTileCacheCreate(&params, NULL, NULL, NULL, NULL, &rejected) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(rejected == NULL);

  /* A tile edge above 255 voxels is refused: a layer header stores each as
     a single byte. */
  rejected = (ZrcTileCache*)0x1;
  ZrcTileCacheParams too_wide = params;
  too_wide.width = 256;
  CHECK(zrcTileCacheCreate(&too_wide, &compressor, NULL, NULL, NULL,
                           &rejected) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(rejected == NULL);

  ZrcTileCache* cache = NULL;
  CHECK(zrcTileCacheCreate(&params, &compressor, NULL, NULL, NULL, &cache) ==
        ZRC_OK);
  if (cache == NULL) {
    ++failures;
    return;
  }

  /* A buffer shorter than a layer header: upstream reads the header off the
     buffer before comparing size against anything, so the header size is
     checked here first. Upstream's own dtTileCacheLayerHeader is well under
     128 bytes even aligned, so this is unambiguously short. */
  const uint8_t short_buffer[4] = {0, 0, 0, 0};
  ZrcCompressedTileRef ref = 1;
  CHECK(zrcTileCacheAddTile(cache, short_buffer, sizeof(short_buffer),
                            &ref) == ZRC_ERR_BAD_FORMAT);
  CHECK(ref == 0);

  /* A well-sized buffer whose magic does not name a layer at all. */
  uint8_t garbage[128];
  memset(garbage, 0, sizeof(garbage));
  ref = 1;
  CHECK(zrcTileCacheAddTile(cache, garbage, sizeof(garbage), &ref) ==
        ZRC_ERR_BAD_FORMAT);
  CHECK(ref == 0);

  /* Upstream's own accessors bound nothing at all; the index is checked
     here, in both directions. */
  ZrcCompressedTileRef tile_ref = 1;
  CHECK(zrcTileCacheTileRefAt(cache, -1, &tile_ref) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(tile_ref == 0);
  tile_ref = 1;
  CHECK(zrcTileCacheTileRefAt(cache, params.max_tiles, &tile_ref) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(tile_ref == 0);

  ZrcObstacleRef obstacle_ref = 1;
  CHECK(zrcTileCacheObstacleRefAt(cache, -1, &obstacle_ref) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(obstacle_ref == 0);
  obstacle_ref = 1;
  CHECK(zrcTileCacheObstacleRefAt(cache, params.max_obstacles,
                                  &obstacle_ref) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(obstacle_ref == 0);

  /* A reference of 0 names no obstacle at all, the same as one whose slot is
     free — ResidentObstacleByRef treats both alike. */
  ZrcObstacleInfo info;
  memset(&info, 0xAB, sizeof(info));
  CHECK(zrcTileCacheObstacleInfo(cache, 0, &info) == ZRC_ERR_NOT_FOUND);

  /* A length shorter than the header is refused rather than dereferenced. */
  uint8_t tiny[4] = {0, 0, 0, 0};
  CHECK(zrcTileCacheHeaderSwapEndian(tiny, sizeof(tiny)) ==
        ZRC_ERR_BAD_FORMAT);

  zrcTileCacheDestroy(cache);
}

/*===========================================================================*/
/* Steering: the proximity grid and the velocity sampler                     */
/*===========================================================================*/

static void test_proximity_grid(void) {
  ZrcProximityGrid* grid = NULL;
  int32_t count = 0;
  int32_t bounds[4];
  float cell = 0.f;
  uint16_t ids[8];

  /* A pool of zero rounds to zero hash buckets upstream and every bucket
     index computed after that is out of bounds. */
  CHECK(zrcProximityGridCreate(0, 1.0f, &grid) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(grid == NULL);
  CHECK(zrcProximityGridCreate(16, 0.0f, &grid) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcProximityGridCreate(16, -1.0f, &grid) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcProximityGridCreate(65535, 1.0f, &grid) == ZRC_ERR_INVALID_ARGUMENT);

  CHECK(zrcProximityGridCreate(64, 2.0f, &grid) == ZRC_OK);
  CHECK(grid != NULL);

  CHECK(zrcProximityGridCellSize(grid, &cell) == ZRC_OK);
  CHECK(cell == 2.0f);

  /* An empty grid reports upstream's inverted sentinel, so min > max is how
     a caller tells empty from one cell at the origin. */
  CHECK(zrcProximityGridBounds(grid, bounds) == ZRC_OK);
  CHECK(bounds[0] > bounds[2]);
  CHECK(bounds[1] > bounds[3]);

  /* Two items, one cell apart at this cell size. */
  CHECK(zrcProximityGridAddItem(grid, 7, 0.f, 0.f, 1.f, 1.f) == ZRC_OK);
  CHECK(zrcProximityGridAddItem(grid, 9, 4.f, 0.f, 5.f, 1.f) == ZRC_OK);

  CHECK(zrcProximityGridBounds(grid, bounds) == ZRC_OK);
  CHECK(bounds[0] <= bounds[2]);
  CHECK(bounds[1] <= bounds[3]);

  /* A box over the first item finds it and not the second. */
  count = -1;
  CHECK(zrcProximityGridQueryItems(grid, -0.5f, -0.5f, 1.5f, 1.5f, ids, 8,
                                   &count) == ZRC_OK);
  CHECK(count == 1);
  CHECK(ids[0] == 7);

  /* A box over both finds both. */
  CHECK(zrcProximityGridQueryItems(grid, -0.5f, -0.5f, 5.5f, 1.5f, ids, 8,
                                   &count) == ZRC_OK);
  CHECK(count == 2);

  /* And a buffer that cannot hold them says so rather than answering with
     half the neighbourhood, which is what upstream does. */
  CHECK(zrcProximityGridQueryItems(grid, -0.5f, -0.5f, 5.5f, 1.5f, ids, 1,
                                   &count) == ZRC_ERR_BUFFER_TOO_SMALL);
  CHECK(count == 1);

  CHECK(zrcProximityGridItemCountAt(grid, 0, 0, &count) == ZRC_OK);
  CHECK(count >= 1);
  /* Kept inside ZRC_PROXIMITY_GRID_SAFE_CELL. Past it upstream's cell hash
     multiplies the coordinate into signed overflow, which is undefined and
     which this package deliberately does not defend against: dtCrowd reaches
     the same grid from inside update() with positions no boundary sees, so
     refusing them here would remove a capability rather than bind one. See
     UPSTREAM.md. */
  CHECK(zrcProximityGridItemCountAt(grid, 25, 25, &count) == ZRC_OK);
  CHECK(count == 0);

  /* A bound Recast could not floor into a cell index is refused before the
     conversion that would be undefined. */
  CHECK(zrcProximityGridAddItem(grid, 1, (float)NAN, 0.f, 1.f, 1.f) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcProximityGridAddItem(grid, 1, 0.f, 0.f, 1e30f, 1.f) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcProximityGridAddItem(grid, 1, 1.f, 0.f, 0.f, 1.f) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcProximityGridQueryItems(grid, 0.f, 0.f, 1e30f, 1.f, ids, 8,
                                   &count) == ZRC_ERR_INVALID_ARGUMENT);

  CHECK(zrcProximityGridClear(grid) == ZRC_OK);
  CHECK(zrcProximityGridQueryItems(grid, -0.5f, -0.5f, 5.5f, 1.5f, ids, 8,
                                   &count) == ZRC_OK);
  CHECK(count == 0);

  zrcProximityGridDestroy(grid);
}

static void test_avoidance(void) {
  ZrcAvoidanceQuery* query = NULL;
  ZrcAvoidanceDebug* debug = NULL;
  ZrcAvoidanceParams params;
  ZrcAvoidanceCircle circle;
  ZrcAvoidanceSegment segment;
  ZrcAvoidanceSample sample;
  const float position[3] = {0.f, 0.f, 0.f};
  const float velocity[3] = {1.f, 0.f, 0.f};
  const float desired[3] = {1.f, 0.f, 0.f};
  const float blocker[3] = {2.f, 0.f, 0.f};
  const float head_on[3] = {-1.f, 0.f, 0.f};
  const float still[3] = {0.f, 0.f, 0.f};
  float chosen[3];
  int32_t samples = 0;
  int32_t recorded = 0;
  int32_t count = 0;

  zrcAvoidanceParamsDefault(&params);
  CHECK(params.grid_size >= 2);
  CHECK(params.adaptive_depth >= 1);
  CHECK(params.horiz_time > 0.f);

  CHECK(zrcAvoidanceQueryCreate(0, 8, &query) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(query == NULL);
  CHECK(zrcAvoidanceQueryCreate(6, 8, &query) == ZRC_OK);

  CHECK(zrcAvoidanceCircleCount(query, &count) == ZRC_OK);
  CHECK(count == 0);

  /* An obstacle coming straight at the agent, and a wall behind it. */
  CHECK(zrcAvoidanceAddCircle(query, blocker, 0.5f, head_on, head_on) ==
        ZRC_OK);
  CHECK(zrcAvoidanceAddSegment(query, blocker, position) == ZRC_OK);
  CHECK(zrcAvoidanceCircleCount(query, &count) == ZRC_OK);
  CHECK(count == 1);
  CHECK(zrcAvoidanceSegmentCount(query, &count) == ZRC_OK);
  CHECK(count == 1);

  CHECK(zrcAvoidanceCircleAt(query, 0, &circle) == ZRC_OK);
  CHECK(circle.radius == 0.5f);
  CHECK(circle.position[0] == blocker[0]);
  CHECK(zrcAvoidanceCircleAt(query, 1, &circle) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcAvoidanceCircleAt(query, -1, &circle) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcAvoidanceSegmentAt(query, 0, &segment) == ZRC_OK);
  CHECK(zrcAvoidanceSegmentAt(query, 1, &segment) == ZRC_ERR_INVALID_ARGUMENT);

  /* The obstacle has to change the answer. Comparing against the desired
     velocity would pass on the velocity bias alone, so the claim is made
     against the same query with nothing in it. */
  CHECK(zrcAvoidanceDebugCreate(256, &debug) == ZRC_OK);
  CHECK(zrcAvoidanceSampleAdaptive(query, position, 0.5f, 2.f, velocity,
                                   desired, &params, debug, chosen,
                                   &samples) == ZRC_OK);
  CHECK(samples > 0);
  {
    ZrcAvoidanceQuery* empty = NULL;
    float unobstructed[3];
    int32_t unobstructed_samples = 0;
    CHECK(zrcAvoidanceQueryCreate(6, 8, &empty) == ZRC_OK);
    CHECK(zrcAvoidanceSampleAdaptive(empty, position, 0.5f, 2.f, velocity,
                                     desired, &params, NULL, unobstructed,
                                     &unobstructed_samples) == ZRC_OK);
    CHECK(unobstructed_samples == samples);
    CHECK(chosen[0] != unobstructed[0] || chosen[1] != unobstructed[1] ||
          chosen[2] != unobstructed[2]);
    zrcAvoidanceQueryDestroy(empty);
  }
  /* The recorder saw candidates, at most as many as were scored: upstream
     resets it at the top of the sampler and adds one per candidate it
     actually evaluates, dropping any past the recorder's capacity. */
  CHECK(zrcAvoidanceDebugSampleCount(debug, &recorded) == ZRC_OK);
  CHECK(recorded > 0);
  CHECK(recorded <= samples);
  CHECK(zrcAvoidanceDebugSampleAt(debug, 0, &sample) == ZRC_OK);
  CHECK(zrcAvoidanceDebugSampleAt(debug, recorded, &sample) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcAvoidanceDebugSampleAt(debug, -1, &sample) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* Normalising leaves the penalties inside [0, 1] and the count alone. */
  CHECK(zrcAvoidanceDebugNormalize(debug) == ZRC_OK);
  CHECK(zrcAvoidanceDebugSampleAt(debug, 0, &sample) == ZRC_OK);
  CHECK(sample.penalty >= 0.f && sample.penalty <= 1.f);
  CHECK(zrcAvoidanceDebugSampleCount(debug, &count) == ZRC_OK);
  CHECK(count == recorded);

  /* A horizon of zero makes every candidate score NaN upstream, and the
     sampler then reports success having chosen nothing. */
  params.horiz_time = 0.f;
  CHECK(zrcAvoidanceSampleAdaptive(query, position, 0.5f, 2.f, velocity,
                                   desired, &params, debug, chosen, &samples) ==
        ZRC_ERR_INVALID_ARGUMENT);
  zrcAvoidanceParamsDefault(&params);

  /* One candidate per axis is a division by zero in the grid sampler. */
  params.grid_size = 1;
  CHECK(zrcAvoidanceSampleGrid(query, position, 0.5f, 2.f, velocity, desired,
                               &params, NULL, chosen, &samples) ==
        ZRC_ERR_INVALID_ARGUMENT);
  zrcAvoidanceParamsDefault(&params);

  /* No refinement pass at all hands back the desired velocity unavoided. */
  params.adaptive_depth = 0;
  CHECK(zrcAvoidanceSampleAdaptive(query, position, 0.5f, 2.f, velocity,
                                   desired, &params, NULL, chosen, &samples) ==
        ZRC_ERR_INVALID_ARGUMENT);
  zrcAvoidanceParamsDefault(&params);

  /* The grid sampler works when its own bound is respected. */
  CHECK(zrcAvoidanceSampleGrid(query, position, 0.5f, 2.f, velocity, desired,
                               &params, NULL, chosen, &samples) == ZRC_OK);
  CHECK(samples > 0);

  /* A circle exactly on the agent normalises a zero vector upstream, which
     poisons every candidate's penalty and leaves the answer at zero. */
  CHECK(zrcAvoidanceQueryReset(query) == ZRC_OK);
  CHECK(zrcAvoidanceAddCircle(query, still, 0.5f, still, still) == ZRC_OK);
  CHECK(zrcAvoidanceSampleAdaptive(query, position, 0.5f, 2.f, velocity,
                                   desired, &params, NULL, chosen, &samples) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* Reset empties it and the capacity is what it was. */
  CHECK(zrcAvoidanceQueryReset(query) == ZRC_OK);
  CHECK(zrcAvoidanceCircleCount(query, &count) == ZRC_OK);
  CHECK(count == 0);

  /* Six circles fit; the seventh is refused rather than dropped, because an
     obstacle the sampler never saw is an agent that walks through it. */
  {
    int i;
    for (i = 0; i < 6; ++i) {
      const float p[3] = {(float)(i + 2), 0.f, 1.f};
      CHECK(zrcAvoidanceAddCircle(query, p, 0.5f, still, still) == ZRC_OK);
    }
    CHECK(zrcAvoidanceAddCircle(query, blocker, 0.5f, still, still) ==
          ZRC_ERR_BUFFER_TOO_SMALL);
  }

  zrcAvoidanceDebugDestroy(debug);
  zrcAvoidanceQueryDestroy(query);
}

/*===========================================================================*/
/* Crowds                                                                    */
/*===========================================================================*/

/* Bakes the fixture into a navmesh a crowd can plan against. NULL on
   failure, already counted. */
static ZrcNavMesh* bake_fixture_navmesh(void) {
  ZrcBakeConfig config;
  ZrcTriMesh mesh;
  ZrcPolyMesh* poly = NULL;
  ZrcNavMesh* navmesh = NULL;

  zrcBakeConfigDefault(&config);
  zrcFixtureTriMesh(&mesh);
  if (zrcPolyMeshBake(&config, &mesh, NULL, NULL, &poly) != ZRC_OK) {
    ++failures;
    return NULL;
  }
  CHECK(zrcNavMeshCreate(poly, NULL, &navmesh) == ZRC_OK);
  zrcPolyMeshDestroy(poly);
  return navmesh;
}

static void default_agent_params(ZrcCrowdAgentParams* out, float radius) {
  memset(out, 0, sizeof(*out));
  out->radius = radius;
  out->height = 2.0f;
  out->max_acceleration = 8.0f;
  out->max_speed = 3.5f;
  out->collision_query_range = radius * 12.0f;
  out->path_optimization_range = radius * 30.0f;
  out->separation_weight = 2.0f;
  out->update_flags = (uint8_t)(ZRC_CROWD_ANTICIPATE_TURNS |
                                ZRC_CROWD_OBSTACLE_AVOIDANCE |
                                ZRC_CROWD_SEPARATION | ZRC_CROWD_OPTIMIZE_VIS |
                                ZRC_CROWD_OPTIMIZE_TOPO);
  out->obstacle_avoidance_type = 0;
  out->query_filter_type = 0;
  out->user_data = NULL;
}

static void test_crowd_arguments(void) {
  ZrcNavMesh* navmesh = bake_fixture_navmesh();
  ZrcCrowd* crowd = NULL;
  ZrcCrowdAgentParams params;
  ZrcAgentRef ref = 0;
  const float at[3] = {ZRC_FIXTURE_START_X, 0.f, ZRC_FIXTURE_START_Z};

  if (navmesh == NULL) return;

  CHECK(zrcCrowdCreate(NULL, 8, 0.6f, &crowd) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(crowd == NULL);
  CHECK(zrcCrowdCreate(navmesh, 0, 0.6f, &crowd) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcCrowdCreate(navmesh, 8, 0.f, &crowd) == ZRC_ERR_INVALID_ARGUMENT);
  /* Above this the proximity grid's unsigned short pool index truncates. */
  CHECK(zrcCrowdCreate(navmesh, ZRC_CROWD_MAX_AGENTS + 1, 0.6f, &crowd) ==
        ZRC_ERR_INVALID_ARGUMENT);

  CHECK(zrcCrowdCreate(navmesh, 8, 0.6f, &crowd) == ZRC_OK);

  default_agent_params(&params, 0.6f);

  /* The two fields upstream indexes its filter and avoidance tables with,
     unchecked, at fifteen call sites between them. */
  params.query_filter_type = ZRC_CROWD_MAX_FILTERS;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.query_filter_type = 255;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.query_filter_type = 0;

  params.obstacle_avoidance_type = ZRC_CROWD_MAX_AVOIDANCE_PARAMS;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.obstacle_avoidance_type = 0;

  /* Zero maximum speed gives an infinite off-mesh traversal budget. */
  params.max_speed = 0.f;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.max_speed = 3.5f;

  /* Zero collision range is a reciprocal upstream takes without a guard. */
  params.collision_query_range = 0.f;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.collision_query_range = 7.2f;

  /* A radius the crowd was not sized for, and one Recast could not floor
     into a grid cell. */
  params.radius = 5.0f;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.radius = (float)NAN;
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_ERR_INVALID_ARGUMENT);
  params.radius = 0.6f;

  CHECK(zrcCrowdAddAgent(crowd, at, &params, &ref) == ZRC_OK);
  CHECK(ref != 0);

  /* A frame is a positive number. Upstream multiplies acceleration by it and
     asserts nothing, so a NaN frame poisons every position from then on. */
  CHECK(zrcCrowdUpdate(crowd, 0.f, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcCrowdUpdate(crowd, -0.1f, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcCrowdUpdate(crowd, (float)NAN, NULL) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcCrowdUpdate(crowd, 1.f / 60.f, NULL) == ZRC_OK);

  /* Every shared table is bounded here, where upstream returns null. */
  {
    ZrcQueryFilter filter;
    ZrcAvoidanceParams avoidance;
    zrcQueryFilterDefault(&filter);
    zrcAvoidanceParamsDefault(&avoidance);
    CHECK(zrcCrowdSetFilter(crowd, ZRC_CROWD_MAX_FILTERS, &filter) ==
          ZRC_ERR_INVALID_ARGUMENT);
    CHECK(zrcCrowdSetFilter(crowd, -1, &filter) == ZRC_ERR_INVALID_ARGUMENT);
    CHECK(zrcCrowdSetFilter(crowd, 3, &filter) == ZRC_OK);
    CHECK(zrcCrowdSetAvoidanceParams(crowd, ZRC_CROWD_MAX_AVOIDANCE_PARAMS,
                                     &avoidance) == ZRC_ERR_INVALID_ARGUMENT);
    CHECK(zrcCrowdSetAvoidanceParams(crowd, 5, &avoidance) == ZRC_OK);
    avoidance.horiz_time = 0.f;
    CHECK(zrcCrowdSetAvoidanceParams(crowd, 5, &avoidance) ==
          ZRC_ERR_INVALID_ARGUMENT);
  }

  zrcCrowdDestroy(crowd);
  zrcNavMeshDestroy(navmesh);
}

static void test_crowd_agent_identity(void) {
  ZrcNavMesh* navmesh = bake_fixture_navmesh();
  ZrcCrowd* crowd = NULL;
  ZrcCrowdAgentParams params;
  ZrcCrowdAgent agent;
  ZrcAgentRef first = 0;
  ZrcAgentRef second = 0;
  int32_t count = 0;
  const float at[3] = {ZRC_FIXTURE_START_X, 0.f, ZRC_FIXTURE_START_Z};

  if (navmesh == NULL) return;
  CHECK(zrcCrowdCreate(navmesh, 4, 0.6f, &crowd) == ZRC_OK);
  default_agent_params(&params, 0.6f);

  CHECK(zrcCrowdAgentCapacity(crowd, &count) == ZRC_OK);
  CHECK(count == 4);
  CHECK(zrcCrowdActiveAgentCount(crowd, &count) == ZRC_OK);
  CHECK(count == 0);

  CHECK(zrcCrowdAddAgent(crowd, at, &params, &first) == ZRC_OK);
  CHECK(zrcCrowdActiveAgentCount(crowd, &count) == ZRC_OK);
  CHECK(count == 1);
  CHECK(zrcCrowdAgentInfo(crowd, first, &agent) == ZRC_OK);
  CHECK(agent.state == ZRC_CROWD_AGENT_WALKING);
  CHECK(agent.params.radius == 0.6f);

  /* The slot comes back, and the reference does not. Upstream's removeAgent
     only clears a flag, so a stale index drives whoever took the slot. */
  CHECK(zrcCrowdRemoveAgent(crowd, first) == ZRC_OK);
  CHECK(zrcCrowdAgentInfo(crowd, first, &agent) == ZRC_ERR_NOT_FOUND);
  CHECK(zrcCrowdRemoveAgent(crowd, first) == ZRC_ERR_NOT_FOUND);

  CHECK(zrcCrowdAddAgent(crowd, at, &params, &second) == ZRC_OK);
  CHECK(second != first);
  CHECK(zrcCrowdAgentInfo(crowd, second, &agent) == ZRC_OK);
  CHECK(zrcCrowdAgentInfo(crowd, first, &agent) == ZRC_ERR_NOT_FOUND);
  CHECK(zrcCrowdSetAgentParams(crowd, first, &params) == ZRC_ERR_NOT_FOUND);
  CHECK(zrcCrowdRequestMoveVelocity(crowd, first, at) == ZRC_ERR_NOT_FOUND);
  CHECK(zrcCrowdAgentRefAt(crowd, 0, &first) == ZRC_OK);
  CHECK(first == second);
  CHECK(zrcCrowdAgentRefAt(crowd, 1, &first) == ZRC_OK);
  CHECK(first == 0);
  CHECK(zrcCrowdAgentRefAt(crowd, 4, &first) == ZRC_ERR_INVALID_ARGUMENT);

  /* Re-initialising purges upstream and must retire every reference minted
     before it, including one whose slot is filled again afterwards. */
  CHECK(zrcCrowdInit(crowd, navmesh, 4, 0.6f) == ZRC_OK);
  CHECK(zrcCrowdActiveAgentCount(crowd, &count) == ZRC_OK);
  CHECK(count == 0);
  CHECK(zrcCrowdAgentInfo(crowd, second, &agent) == ZRC_ERR_NOT_FOUND);
  CHECK(zrcCrowdAddAgent(crowd, at, &params, &first) == ZRC_OK);
  CHECK(first != second);
  CHECK(zrcCrowdAgentInfo(crowd, second, &agent) == ZRC_ERR_NOT_FOUND);

  /* A full crowd says so rather than answering with a slot it does not have. */
  {
    ZrcAgentRef extra = 0;
    int i;
    for (i = 1; i < 4; ++i) {
      CHECK(zrcCrowdAddAgent(crowd, at, &params, &extra) == ZRC_OK);
    }
    CHECK(zrcCrowdAddAgent(crowd, at, &params, &extra) == ZRC_ERR_CROWD_FULL);
  }

  /* Active agents: the total is reported even when the buffer is short. */
  {
    ZrcAgentRef refs[4];
    CHECK(zrcCrowdActiveAgents(crowd, refs, 4, &count) == ZRC_OK);
    CHECK(count == 4);
    CHECK(zrcCrowdActiveAgents(crowd, refs, 2, &count) ==
          ZRC_ERR_BUFFER_TOO_SMALL);
    CHECK(count == 4);
    CHECK(zrcCrowdActiveAgents(crowd, NULL, 0, &count) ==
          ZRC_ERR_BUFFER_TOO_SMALL);
    CHECK(count == 4);
  }

  zrcCrowdDestroy(crowd);
  zrcNavMeshDestroy(navmesh);
}

/* Fails the suite if the step from `prev` to `now` crossed the wall.

   The fixture's wall lies along z = 0 from the -X edge to
   ZRC_FIXTURE_WALL_END_X. A step that changes the sign of z crossed the line
   z = 0 somewhere; the x at that crossing is what says whether it went round
   the wall's end or through the wall itself. */
static void check_no_tunnel(const float* prev, const float* now) {
  float dz;
  float t;
  float x;
  if ((prev[2] < 0.f) == (now[2] < 0.f)) return;
  dz = now[2] - prev[2];
  t = (dz == 0.f) ? 0.f : (0.f - prev[2]) / dz;
  x = prev[0] + (now[0] - prev[0]) * t;
  CHECK(x > ZRC_FIXTURE_WALL_END_X - 0.5f);
}

static void test_crowd_steering(void) {
  ZrcNavMesh* navmesh = bake_fixture_navmesh();
  ZrcNavMeshQuery* query = NULL;
  ZrcCrowd* crowd = NULL;
  ZrcCrowdAgentParams params;
  ZrcCrowdAgent a;
  ZrcCrowdAgent b;
  ZrcAgentRef first = 0;
  ZrcAgentRef second = 0;
  ZrcPolyRef goal_poly = 0;
  float goal_point[3];
  const float half_extents[3] = {2.f, 4.f, 2.f};
  const float start[3] = {ZRC_FIXTURE_START_X, 0.f, ZRC_FIXTURE_START_Z};
  const float goal[3] = {ZRC_FIXTURE_GOAL_X, 0.f, ZRC_FIXTURE_GOAL_Z};
  ZrcQueryFilter filter;
  int frame;
  float separation;
  float dx;
  float dz;
  float a_prev[3];
  float b_prev[3];

  if (navmesh == NULL) return;
  CHECK(zrcNavMeshQueryCreate(navmesh, 2048, &query) == ZRC_OK);
  zrcQueryFilterDefault(&filter);
  CHECK(zrcFindNearestPoly(query, goal, half_extents, &filter, &goal_poly,
                           goal_point, NULL) == ZRC_OK);
  CHECK(goal_poly != 0);

  CHECK(zrcCrowdCreate(navmesh, 8, 0.6f, &crowd) == ZRC_OK);
  default_agent_params(&params, 0.6f);

  /* Two agents added at the same point, which upstream places in the same
     polygon at the same position. */
  CHECK(zrcCrowdAddAgent(crowd, start, &params, &first) == ZRC_OK);
  CHECK(zrcCrowdAddAgent(crowd, start, &params, &second) == ZRC_OK);
  CHECK(zrcCrowdRequestMoveTarget(crowd, first, goal_poly, goal_point) ==
        ZRC_OK);
  CHECK(zrcCrowdRequestMoveTarget(crowd, second, goal_poly, goal_point) ==
        ZRC_OK);

  /* A target of no polygon is refused; clearing one is a separate call. */
  CHECK(zrcCrowdRequestMoveTarget(crowd, first, 0, goal_point) ==
        ZRC_ERR_INVALID_ARGUMENT);

  CHECK(zrcCrowdAgentInfo(crowd, first, &a) == ZRC_OK);
  CHECK(zrcCrowdAgentInfo(crowd, second, &b) == ZRC_OK);
  memcpy(a_prev, a.position, sizeof(a_prev));
  memcpy(b_prev, b.position, sizeof(b_prev));

  for (frame = 0; frame < 600; ++frame) {
    CHECK(zrcCrowdUpdate(crowd, 1.f / 60.f, NULL) == ZRC_OK);
    CHECK(zrcCrowdAgentInfo(crowd, first, &a) == ZRC_OK);
    CHECK(zrcCrowdAgentInfo(crowd, second, &b) == ZRC_OK);

    /* Neither agent may pass through the wall. Crossing z = 0 is how an
       agent reaches the goal, so the claim is about where it crosses: only
       past the wall's far end is legitimate. A snapshot cannot say that —
       an agent that has already rounded the wall is back at a small x on
       the far side — so the crossing point is interpolated from the step. */
    check_no_tunnel(a_prev, a.position);
    check_no_tunnel(b_prev, b.position);
    memcpy(a_prev, a.position, sizeof(a_prev));
    memcpy(b_prev, b.position, sizeof(b_prev));

    /* And neither may leave the ground plane. */
    CHECK(a.position[0] >= -ZRC_FIXTURE_GROUND_EXTENT - 1.f &&
          a.position[0] <= ZRC_FIXTURE_GROUND_EXTENT + 1.f);
    CHECK(a.position[2] >= -ZRC_FIXTURE_GROUND_EXTENT - 1.f &&
          a.position[2] <= ZRC_FIXTURE_GROUND_EXTENT + 1.f);
  }

  /* Started on top of each other, they end at least a radius apart. */
  dx = a.position[0] - b.position[0];
  dz = a.position[2] - b.position[2];
  separation = dx * dx + dz * dz;
  CHECK(separation > params.radius * params.radius);

  /* Both got somewhere: at least one reached the far side of the wall. */
  CHECK(a.position[2] > 0.f || b.position[2] > 0.f);

  /* Velocity sampling happened, which is what says avoidance ran at all. */
  {
    int32_t samples = 0;
    CHECK(zrcCrowdVelocitySampleCount(crowd, &samples) == ZRC_OK);
    CHECK(samples > 0);
  }

  /* Clearing the target stops the agent where it is. */
  CHECK(zrcCrowdResetMoveTarget(crowd, first) == ZRC_OK);
  CHECK(zrcCrowdAgentInfo(crowd, first, &a) == ZRC_OK);
  CHECK(a.target_state == ZRC_CROWD_TARGET_NONE);

  zrcCrowdDestroy(crowd);
  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);
}

/*===========================================================================*/
/* The path corridor, the local boundary and the path queue                  */
/*===========================================================================*/

static void test_path_corridor(void) {
  ZrcNavMesh* navmesh = bake_fixture_navmesh();
  ZrcNavMeshQuery* query = NULL;
  ZrcPathCorridor* corridor = NULL;
  ZrcQueryFilter filter;
  ZrcPathCorridorInfo info;
  ZrcCrowdCorner corners[8];
  ZrcPolyRef path[64];
  ZrcPolyRef read_back[64];
  ZrcPolyRef start_poly = 0;
  ZrcPolyRef goal_poly = 0;
  float start_point[3];
  float goal_point[3];
  const float half_extents[3] = {2.f, 4.f, 2.f};
  const float start[3] = {ZRC_FIXTURE_START_X, 0.f, ZRC_FIXTURE_START_Z};
  const float goal[3] = {ZRC_FIXTURE_GOAL_X, 0.f, ZRC_FIXTURE_GOAL_Z};
  int32_t path_count = 0;
  int32_t corner_count = 0;
  ZrcBool valid = ZRC_FALSE;
  ZrcBool moved = ZRC_FALSE;

  if (navmesh == NULL) return;
  CHECK(zrcNavMeshQueryCreate(navmesh, 2048, &query) == ZRC_OK);
  zrcQueryFilterDefault(&filter);
  CHECK(zrcFindNearestPoly(query, start, half_extents, &filter, &start_poly,
                           start_point, NULL) == ZRC_OK);
  CHECK(zrcFindNearestPoly(query, goal, half_extents, &filter, &goal_poly,
                           goal_point, NULL) == ZRC_OK);
  CHECK(start_poly != 0 && goal_poly != 0);

  /* Shorter than the minimum is refused. fixPathStart writes m_path[2] with
     no bound of any kind, and two of the merge helpers hand memmove a
     negative length once the corridor is shorter than what they merge. */
  CHECK(zrcPathCorridorCreate(1, &corridor) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(corridor == NULL);
  CHECK(zrcPathCorridorCreate(ZRC_PATH_CORRIDOR_MIN_PATH - 1, &corridor) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcPathCorridorCreate(64, &corridor) == ZRC_OK);

  /* A reset corridor holds the one polygon it was put in. */
  CHECK(zrcPathCorridorReset(corridor, start_poly, start_point) == ZRC_OK);
  CHECK(zrcPathCorridorInfo(corridor, &info) == ZRC_OK);
  CHECK(info.first_poly == start_poly);
  CHECK(info.last_poly == start_poly);
  CHECK(info.path_count == 1);

  /* Loading a real path makes the corridor span it end to end. */
  CHECK(zrcFindPath(query, start_poly, goal_poly, start_point, goal_point,
                    &filter, path, 64, &path_count, NULL) == ZRC_OK);
  CHECK(path_count > 1);
  CHECK(zrcPathCorridorSetCorridor(corridor, goal_point, path, path_count) ==
        ZRC_OK);
  CHECK(zrcPathCorridorInfo(corridor, &info) == ZRC_OK);
  CHECK(info.path_count == path_count);
  CHECK(info.first_poly == path[0]);
  CHECK(info.last_poly == path[path_count - 1]);

  /* Read back exactly what went in. */
  CHECK(zrcPathCorridorPath(corridor, 0, path_count, read_back) == ZRC_OK);
  CHECK(memcmp(path, read_back, sizeof(ZrcPolyRef) * (size_t)path_count) == 0);
  CHECK(zrcPathCorridorPath(corridor, 0, path_count + 1, read_back) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* An empty corridor and one longer than the buffer are both refused. */
  CHECK(zrcPathCorridorSetCorridor(corridor, goal_point, path, 0) ==
        ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcPathCorridorSetCorridor(corridor, goal_point, path, 65) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* One corner buffer is not enough: the string-pull reserves the last slot
     for the corridor's end point. */
  CHECK(zrcPathCorridorFindCorners(corridor, query, &filter, corners, 1,
                                   &corner_count) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcPathCorridorFindCorners(corridor, query, &filter, corners, 8,
                                   &corner_count) == ZRC_OK);
  CHECK(corner_count > 0);
  CHECK(corner_count <= 7);
  /* The corridor rounds the wall, so a corner sits past its far end. */
  {
    int i;
    int rounded = 0;
    int named = 0;
    for (i = 0; i < corner_count; ++i) {
      /* The last corner is the corridor's end point, which enters no polygon
         and so carries a reference of 0; the ones before it name the polygon
         being entered. */
      if (corners[i].poly != 0) named = 1;
      if (corners[i].position[0] > ZRC_FIXTURE_WALL_END_X - 1.f) rounded = 1;
    }
    CHECK(named);
    CHECK(rounded);
  }

  CHECK(zrcPathCorridorIsValid(corridor, path_count, query, &filter, &valid) ==
        ZRC_OK);
  CHECK(valid == ZRC_TRUE);

  /* Sliding the position forward drops the polygons left behind. */
  CHECK(zrcPathCorridorMovePosition(corridor, corners[0].position, query,
                                    &filter, &moved) == ZRC_OK);
  CHECK(moved == ZRC_TRUE);
  {
    ZrcPathCorridorInfo after;
    CHECK(zrcPathCorridorInfo(corridor, &after) == ZRC_OK);
    CHECK(after.path_count <= info.path_count);
    CHECK(after.last_poly == info.last_poly);
  }

  /* A non-finite position never reaches upstream, which would store it. */
  {
    const float poisoned[3] = {(float)NAN, 0.f, 0.f};
    CHECK(zrcPathCorridorReset(corridor, start_poly, poisoned) ==
          ZRC_ERR_INVALID_ARGUMENT);
    CHECK(zrcPathCorridorMovePosition(corridor, poisoned, query, &filter,
                                      &moved) == ZRC_ERR_INVALID_ARGUMENT);
  }
  /* And a zero optimisation range is a division by zero upstream. */
  CHECK(zrcPathCorridorOptimizeVisibility(corridor, goal_point, 0.f, query,
                                          &filter) ==
        ZRC_ERR_INVALID_ARGUMENT);

  /* The merge helpers: more visited polygons than the array can hold is what
     turns their signed length subtraction negative. */
  {
    ZrcPolyRef small[4];
    ZrcPolyRef visited[8];
    int32_t out_count = 0;
    int i;
    for (i = 0; i < 4; ++i) small[i] = path[i % path_count];
    for (i = 0; i < 8; ++i) visited[i] = path[i % path_count];
    CHECK(zrcMergeCorridorStartMoved(small, 4, 4, visited, 8, &out_count) ==
          ZRC_ERR_INVALID_ARGUMENT);
    CHECK(zrcMergeCorridorStartShortcut(small, 4, 4, visited, 8, &out_count) ==
          ZRC_ERR_INVALID_ARGUMENT);
    CHECK(zrcMergeCorridorEndMoved(small, 5, 4, visited, 2, &out_count) ==
          ZRC_ERR_INVALID_ARGUMENT);
    /* Within the bound it does the merge and reports the new length. */
    CHECK(zrcMergeCorridorStartMoved(small, 4, 4, visited, 2, &out_count) ==
          ZRC_OK);
    CHECK(out_count > 0 && out_count <= 4);
  }

  zrcPathCorridorDestroy(corridor);
  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);
}

static void test_local_boundary(void) {
  ZrcNavMesh* navmesh = bake_fixture_navmesh();
  ZrcNavMeshQuery* query = NULL;
  ZrcLocalBoundary* boundary = NULL;
  ZrcQueryFilter filter;
  ZrcPolyRef poly = 0;
  float point[3];
  float centre[3];
  float segments[8 * 6];
  const float half_extents[3] = {2.f, 4.f, 2.f};
  /* Just south of the wall, where there is a wall to find. */
  const float near_wall[3] = {0.f, 0.f, -1.f};
  int32_t count = 0;
  ZrcBool valid = ZRC_FALSE;

  if (navmesh == NULL) return;
  CHECK(zrcNavMeshQueryCreate(navmesh, 2048, &query) == ZRC_OK);
  zrcQueryFilterDefault(&filter);
  CHECK(zrcFindNearestPoly(query, near_wall, half_extents, &filter, &poly,
                           point, NULL) == ZRC_OK);
  CHECK(poly != 0);

  CHECK(zrcLocalBoundaryCreate(&boundary) == ZRC_OK);

  /* Never updated: upstream's own sentinel, not a zero. */
  CHECK(zrcLocalBoundaryCenter(boundary, centre) == ZRC_OK);
  CHECK(centre[0] > 1e30f);
  CHECK(zrcLocalBoundarySegmentCount(boundary, &count) == ZRC_OK);
  CHECK(count == 0);

  CHECK(zrcLocalBoundaryUpdate(boundary, poly, point, 4.f, query, &filter) ==
        ZRC_OK);
  CHECK(zrcLocalBoundaryCenter(boundary, centre) == ZRC_OK);
  CHECK(centre[0] == point[0]);
  CHECK(zrcLocalBoundarySegmentCount(boundary, &count) == ZRC_OK);
  CHECK(count > 0);
  CHECK(count <= 8);
  CHECK(zrcLocalBoundarySegments(boundary, 0, count, segments) == ZRC_OK);
  CHECK(zrcLocalBoundarySegments(boundary, 0, count + 1, segments) ==
        ZRC_ERR_INVALID_ARGUMENT);

  CHECK(zrcLocalBoundaryIsValid(boundary, query, &filter, &valid) == ZRC_OK);
  CHECK(valid == ZRC_TRUE);

  /* A polygon of 0 resets it rather than collecting anything, and a range of
     zero or a non-finite position never reaches upstream, which stores the
     position before it validates it. */
  CHECK(zrcLocalBoundaryUpdate(boundary, poly, point, 0.f, query, &filter) ==
        ZRC_ERR_INVALID_ARGUMENT);
  {
    const float poisoned[3] = {(float)NAN, 0.f, 0.f};
    CHECK(zrcLocalBoundaryUpdate(boundary, poly, poisoned, 4.f, query,
                                 &filter) == ZRC_ERR_INVALID_ARGUMENT);
  }
  CHECK(zrcLocalBoundaryUpdate(boundary, 0, point, 4.f, query, &filter) ==
        ZRC_OK);
  CHECK(zrcLocalBoundarySegmentCount(boundary, &count) == ZRC_OK);
  CHECK(count == 0);

  CHECK(zrcLocalBoundaryReset(boundary) == ZRC_OK);
  CHECK(zrcLocalBoundaryCenter(boundary, centre) == ZRC_OK);
  CHECK(centre[0] > 1e30f);

  zrcLocalBoundaryDestroy(boundary);
  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);
}

static void test_path_queue(void) {
  ZrcNavMesh* navmesh = bake_fixture_navmesh();
  ZrcNavMeshQuery* query = NULL;
  ZrcPathQueue* queue = NULL;
  ZrcQueryFilter filter;
  ZrcPathRequestRef ref = ZRC_PATH_REQUEST_NONE;
  ZrcPathRequestState state = ZRC_PATH_REQUEST_UNKNOWN;
  ZrcPolyRef start_poly = 0;
  ZrcPolyRef goal_poly = 0;
  ZrcPolyRef path[256];
  float start_point[3];
  float goal_point[3];
  const float half_extents[3] = {2.f, 4.f, 2.f};
  const float start[3] = {ZRC_FIXTURE_START_X, 0.f, ZRC_FIXTURE_START_Z};
  const float goal[3] = {ZRC_FIXTURE_GOAL_X, 0.f, ZRC_FIXTURE_GOAL_Z};
  int32_t count = 0;
  int spin;

  if (navmesh == NULL) return;
  CHECK(zrcNavMeshQueryCreate(navmesh, 2048, &query) == ZRC_OK);
  zrcQueryFilterDefault(&filter);
  CHECK(zrcFindNearestPoly(query, start, half_extents, &filter, &start_poly,
                           start_point, NULL) == ZRC_OK);
  CHECK(zrcFindNearestPoly(query, goal, half_extents, &filter, &goal_poly,
                           goal_point, NULL) == ZRC_OK);

  /* Fewer than four search nodes leaves upstream's hash table with zero
     buckets and every index it computes outside the allocation. */
  CHECK(zrcPathQueueCreate(navmesh, 256, 3, &queue) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(queue == NULL);
  CHECK(zrcPathQueueCreate(NULL, 256, 4096, &queue) == ZRC_ERR_INVALID_ARGUMENT);
  CHECK(zrcPathQueueCreate(navmesh, 256, 4096, &queue) == ZRC_OK);

  /* Nothing submitted yet. */
  CHECK(zrcPathQueueRequestStatus(queue, 1, &state) == ZRC_OK);
  CHECK(state == ZRC_PATH_REQUEST_UNKNOWN);
  CHECK(zrcPathQueueResult(queue, 1, path, 256, &count) == ZRC_ERR_NOT_FOUND);

  CHECK(zrcPathQueueRequest(queue, start_poly, goal_poly, start_point,
                            goal_point, &filter, &ref) == ZRC_OK);
  CHECK(ref != ZRC_PATH_REQUEST_NONE);

  /* Taking a result before the search finished frees the slot upstream and
     reports success with an empty path. Refused here, and the request lives. */
  CHECK(zrcPathQueueRequestStatus(queue, ref, &state) == ZRC_OK);
  CHECK(state == ZRC_PATH_REQUEST_RUNNING);
  CHECK(zrcPathQueueResult(queue, ref, path, 256, &count) ==
        ZRC_ERR_SEARCH_IN_PROGRESS);

  for (spin = 0; spin < 64; ++spin) {
    CHECK(zrcPathQueueUpdate(queue, 100) == ZRC_OK);
    CHECK(zrcPathQueueRequestStatus(queue, ref, &state) == ZRC_OK);
    if (state != ZRC_PATH_REQUEST_RUNNING) break;
  }
  CHECK(state == ZRC_PATH_REQUEST_READY);

  CHECK(zrcPathQueueResult(queue, ref, path, 256, &count) == ZRC_OK);
  CHECK(count > 1);
  CHECK(path[0] == start_poly);

  /* The slot is free again and the reference is spent. */
  CHECK(zrcPathQueueRequestStatus(queue, ref, &state) == ZRC_OK);
  CHECK(state == ZRC_PATH_REQUEST_UNKNOWN);

  /* Eight fit; the ninth is reported as no slot rather than as an error. */
  {
    ZrcPathRequestRef refs[9];
    int i;
    for (i = 0; i < 8; ++i) {
      CHECK(zrcPathQueueRequest(queue, start_poly, goal_poly, start_point,
                                goal_point, &filter, &refs[i]) == ZRC_OK);
      CHECK(refs[i] != ZRC_PATH_REQUEST_NONE);
    }
    CHECK(zrcPathQueueRequest(queue, start_poly, goal_poly, start_point,
                              goal_point, &filter, &refs[8]) == ZRC_OK);
    CHECK(refs[8] == ZRC_PATH_REQUEST_NONE);
  }

  /* The queue's own query object is the one it plans with. */
  {
    const ZrcNavMeshQuery* owned = NULL;
    CHECK(zrcPathQueueNavMeshQuery(queue, &owned) == ZRC_OK);
    CHECK(owned != NULL);
    CHECK(owned != query);
  }

  zrcPathQueueDestroy(queue);
  zrcNavMeshQueryDestroy(query);
  zrcNavMeshDestroy(navmesh);
}

int main(void) {
  Counters counters = {0, 0};
  ZrcAllocator allocator;
  allocator.allocate = count_allocate;
  allocator.deallocate = count_deallocate;
  allocator.user = &counters;

  test_version();
  test_result_names();
  test_abi_layout();
  test_allocator_rejects_incomplete();
  test_null_handles_are_safe();
  test_assert_handler();
  test_point_in_polygon();

  CHECK(zrcSetAllocator(&allocator) == ZRC_OK);

  test_null_arguments();
  test_bad_bake_input();
  test_bad_navmesh_input();
  test_allocator_swapping(&counters);
  test_alloc_free();
  test_area_authoring();
  test_pipeline();
  test_staged_pipeline();
  test_staged_pipeline_errors();
  test_tile_cache();
  test_tile_cache_errors();
  test_proximity_grid();
  test_avoidance();
  test_crowd_arguments();
  test_crowd_agent_identity();
  test_crowd_steering();
  test_path_corridor();
  test_local_boundary();
  test_path_queue();

  /* The seam must actually have been used, and everything taken must have been
     given back. */
  CHECK(counters.allocations > 0);
  if (counters.allocations != counters.frees) {
    fprintf(stderr, "leak: %zu allocations, %zu frees\n", counters.allocations,
            counters.frees);
    ++failures;
  }

  CHECK(zrcSetAllocator(NULL) == ZRC_OK);

  if (failures != 0) {
    fprintf(stderr, "zrecast c smoke: %d check(s) failed\n", failures);
    return 1;
  }
  printf("zrecast c smoke: ok (%zu allocations balanced)\n",
         counters.allocations);
  return 0;
}
