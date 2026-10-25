//! Translation between the C result enum and a Zig error set, and the frame
//! each refusal leaves on the caller's `Failure`.

const std = @import("std");
const aegis = @import("aegis");
const c = @import("c.zig");

pub const Error = error{
    /// A null handle, an out-of-range count, or a non-finite scalar.
    InvalidArgument,
    /// The installed allocator returned null.
    OutOfMemory,
    /// Serialised navmesh bytes that are not a navmesh, or whose header does
    /// not describe the buffer carrying it.
    BadFormat,
    /// Serialised navmesh bytes in a format version this build does not speak.
    UnsupportedVersion,
    /// The destination buffer was smaller than the result.
    BufferTooSmall,
    /// A Recast build stage failed. Pass a log buffer to find out which.
    BakeFailed,
    /// The bake succeeded but produced no walkable polygons.
    EmptyResult,
    /// Detour reported a failure with no more specific mapping.
    QueryFailed,
    /// The query's node pool was exhausted; raise `max_nodes`.
    OutOfNodes,
    /// A tile already occupies that grid position. Remove it first.
    TileOccupied,
    /// The navmesh has no free tile slot; it holds `max_tiles` already.
    NavMeshFull,
    /// A sliced path search is in flight; finalise or cancel it first.
    SearchInProgress,
    /// No sliced path search is in flight on this query.
    NoSearch,
    /// The thing asked for is not there, though the question was well formed.
    NotFound,
    /// A build stage was pointed at a container that already holds a result.
    AlreadyBuilt,
    /// The crowd has no free agent slot. It was created for `max_agents`,
    /// and that many are active.
    CrowdFull,
};

/// The call a refusal names: the C entry point that returned it, or the one
/// the binding refused to make because an argument could not reach it, in
/// snake case without the `zrc` prefix (`zrcFindPath` is `find_path`). A
/// refusal no C call stands behind names the Zig function
/// (`convex_volume`, `dir_for_offset`).
pub const Call = enum {
    alloc,
    avoidance_add_circle,
    avoidance_add_segment,
    avoidance_circle_at,
    avoidance_circle_count,
    avoidance_debug_add_sample,
    avoidance_debug_create,
    avoidance_debug_normalize,
    avoidance_debug_reset,
    avoidance_debug_sample_at,
    avoidance_debug_sample_count,
    avoidance_query_create,
    avoidance_query_reset,
    avoidance_segment_at,
    avoidance_segment_count,
    bake_config_cells,
    build_context_accumulated_time,
    build_context_log,
    build_context_reset_log,
    build_context_reset_timers,
    build_context_start_timer,
    build_context_stop_timer,
    calc_bounds,
    calc_grid_size,
    clear_unwalkable_triangles,
    closest_height_point_triangle,
    closest_point_on_poly,
    closest_point_on_poly_boundary,
    closest_point_on_triangle,
    compact_heightfield_areas,
    compact_heightfield_build_distance_field,
    compact_heightfield_build_regions,
    compact_heightfield_cells,
    compact_heightfield_create,
    compact_heightfield_distances,
    compact_heightfield_erode,
    compact_heightfield_info,
    compact_heightfield_mark_areas,
    compact_heightfield_median_filter,
    compact_heightfield_set_areas,
    compact_heightfield_set_spans,
    compact_heightfield_spans,
    contour_at,
    contour_raw_verts,
    contour_set_create,
    contour_set_info,
    contour_verts,
    convex_volume,
    crowd_active_agent_count,
    crowd_active_agents,
    crowd_add_agent,
    crowd_agent_boundary_center,
    crowd_agent_boundary_segment_count,
    crowd_agent_boundary_segments,
    crowd_agent_capacity,
    crowd_agent_corners,
    crowd_agent_corridor_info,
    crowd_agent_corridor_path,
    crowd_agent_info,
    crowd_agent_neighbours,
    crowd_agent_ref_at,
    crowd_avoidance_params,
    crowd_create,
    crowd_filter,
    crowd_grid,
    crowd_init,
    crowd_nav_mesh_query,
    crowd_path_queue,
    crowd_query_half_extents,
    crowd_remove_agent,
    crowd_request_move_target,
    crowd_request_move_velocity,
    crowd_reset_move_target,
    crowd_set_agent_params,
    crowd_set_avoidance_params,
    crowd_set_filter,
    crowd_update,
    crowd_velocity_sample_count,
    debug_append_arc,
    debug_append_arrow,
    debug_append_box,
    debug_append_box_points,
    debug_append_box_wire,
    debug_append_circle,
    debug_append_cross,
    debug_append_cylinder,
    debug_append_cylinder_wire,
    debug_draw_arc,
    debug_draw_arrow,
    debug_draw_box,
    debug_draw_box_wire,
    debug_draw_circle,
    debug_draw_compact_heightfield_distance,
    debug_draw_compact_heightfield_regions,
    debug_draw_compact_heightfield_solid,
    debug_draw_contours,
    debug_draw_cross,
    debug_draw_cylinder,
    debug_draw_cylinder_wire,
    debug_draw_grid_xz,
    debug_draw_heightfield_layer,
    debug_draw_heightfield_layers,
    debug_draw_heightfield_solid,
    debug_draw_heightfield_walkable,
    debug_draw_nav_mesh,
    debug_draw_nav_mesh_bv_tree,
    debug_draw_nav_mesh_nodes,
    debug_draw_nav_mesh_poly,
    debug_draw_nav_mesh_polys_with_flags,
    debug_draw_nav_mesh_portals,
    debug_draw_nav_mesh_with_closed_list,
    debug_draw_poly_mesh,
    debug_draw_poly_mesh_detail,
    debug_draw_raw_contours,
    debug_draw_region_connections,
    debug_draw_tile_cache_contours,
    debug_draw_tile_cache_layer_areas,
    debug_draw_tile_cache_layer_regions,
    debug_draw_tile_cache_poly_mesh,
    debug_draw_tri_mesh,
    debug_draw_tri_mesh_slope,
    decode_poly_ref,
    dir_for_offset,
    display_list_begin,
    display_list_clear,
    display_list_create,
    display_list_depth_mask,
    display_list_draw,
    display_list_end,
    display_list_vertex,
    display_list_vertex_count,
    display_list_vertex_xyz,
    distance_point_to_poly_edges,
    distance_point_to_segment_2d,
    dump_compact_heightfield,
    dump_contour_set,
    dump_poly_mesh_detail_to_obj,
    dump_poly_mesh_to_obj,
    encode_poly_ref,
    find_distance_to_wall,
    find_local_neighbourhood,
    find_nearest_poly,
    find_path,
    find_polys_around_circle,
    find_polys_around_shape,
    find_random_point,
    find_random_point_around_circle,
    find_straight_path,
    heightfield_add_span,
    heightfield_column,
    heightfield_create,
    heightfield_filter_ledge_spans,
    heightfield_filter_low_hanging_obstacles,
    heightfield_filter_walkable_low_height_spans,
    heightfield_info,
    heightfield_layer_areas,
    heightfield_layer_at,
    heightfield_layer_cons,
    heightfield_layer_heights,
    heightfield_layer_set_count,
    heightfield_layer_set_create,
    heightfield_rasterize_triangle,
    heightfield_rasterize_triangle_soup,
    heightfield_rasterize_triangles,
    heightfield_rasterize_triangles_u16,
    heightfield_span_count,
    heightfield_storage,
    intersect_seg_seg_2d,
    intersect_segment_poly_2d,
    is_in_closed_list,
    is_valid_poly_ref,
    local_boundary_center,
    local_boundary_create,
    local_boundary_is_valid,
    local_boundary_reset,
    local_boundary_segment_count,
    local_boundary_segments,
    local_boundary_update,
    log_build_times,
    mark_walkable_triangles,
    merge_corridor_end_moved,
    merge_corridor_start_moved,
    merge_corridor_start_shortcut,
    move_along_surface,
    nav_mesh_add_tile,
    nav_mesh_bounds,
    nav_mesh_calc_tile_loc,
    nav_mesh_create,
    nav_mesh_create_tiled,
    nav_mesh_deserialize,
    nav_mesh_get_poly_area,
    nav_mesh_get_poly_flags,
    nav_mesh_get_poly_type,
    nav_mesh_image_swap_endian,
    nav_mesh_off_mesh_connection,
    nav_mesh_off_mesh_connection_end_points,
    nav_mesh_params,
    nav_mesh_poly_info,
    nav_mesh_query_create,
    nav_mesh_remove_tile,
    nav_mesh_restore_tile_state,
    nav_mesh_serialize,
    nav_mesh_set_poly_area,
    nav_mesh_set_poly_flags,
    nav_mesh_store_tile_state,
    nav_mesh_tile_bounds,
    nav_mesh_tile_bv_node,
    nav_mesh_tile_detail_tris,
    nav_mesh_tile_detail_verts,
    nav_mesh_tile_info,
    nav_mesh_tile_link,
    nav_mesh_tile_poly_ref,
    nav_mesh_tile_ref_at,
    nav_mesh_tile_ref_at_index,
    nav_mesh_tile_refs_at,
    nav_mesh_tile_state_size,
    nav_mesh_tile_user_id,
    nav_mesh_tile_verts,
    nav_mesh_validate,
    offset_poly,
    opposite_tile_side,
    overlap_bounds,
    overlap_poly_poly_2d,
    overlap_quant_bounds,
    path_corridor_create,
    path_corridor_find_corners,
    path_corridor_fix_start,
    path_corridor_info,
    path_corridor_is_valid,
    path_corridor_move_over_offmesh_connection,
    path_corridor_move_position,
    path_corridor_move_target_position,
    path_corridor_optimize_topology,
    path_corridor_optimize_visibility,
    path_corridor_path,
    path_corridor_reset,
    path_corridor_set_corridor,
    path_corridor_trim_invalid,
    path_from_dijkstra_search,
    path_queue_create,
    path_queue_nav_mesh_query,
    path_queue_request,
    path_queue_request_status,
    path_queue_result,
    path_queue_update,
    point_in_polygon,
    poly_center,
    poly_height,
    poly_mesh_bake,
    poly_mesh_bake_tile,
    poly_mesh_build,
    poly_mesh_build_detail,
    poly_mesh_copy,
    poly_mesh_create,
    poly_mesh_detail_meshes,
    poly_mesh_detail_tris,
    poly_mesh_detail_verts,
    poly_mesh_info,
    poly_mesh_merge,
    poly_mesh_poly_areas,
    poly_mesh_poly_flags,
    poly_mesh_polys,
    poly_mesh_regions,
    poly_mesh_set_agent_dims,
    poly_mesh_set_poly_areas,
    poly_mesh_set_poly_flags,
    poly_mesh_verts,
    poly_wall_segments,
    proximity_grid_add_item,
    proximity_grid_bounds,
    proximity_grid_cell_size,
    proximity_grid_clear,
    proximity_grid_create,
    proximity_grid_item_count_at,
    proximity_grid_query_items,
    query_filter_create,
    query_filter_set_area_cost,
    query_find_node,
    query_find_nodes,
    query_nav_mesh,
    query_node_at,
    query_node_pool_info,
    query_polygons,
    query_polygons_batched,
    random_point_in_convex_poly,
    raycast,
    read_compact_heightfield,
    read_contour_set,
    sliced_find_path_finalize,
    sliced_find_path_finalize_partial,
    sliced_find_path_init,
    sliced_find_path_update,
    tile_cache_add_box_obstacle,
    tile_cache_add_cylinder_obstacle,
    tile_cache_add_oriented_box_obstacle,
    tile_cache_add_tile,
    tile_cache_build_nav_mesh_tile,
    tile_cache_build_nav_mesh_tiles_at,
    tile_cache_contour_at,
    tile_cache_contour_set_count,
    tile_cache_contour_set_create,
    tile_cache_contour_verts,
    tile_cache_create,
    tile_cache_header_swap_endian,
    tile_cache_layer_areas,
    tile_cache_layer_build,
    tile_cache_layer_build_regions,
    tile_cache_layer_cons,
    tile_cache_layer_decompress,
    tile_cache_layer_header_of,
    tile_cache_layer_heights,
    tile_cache_layer_mark_box,
    tile_cache_layer_mark_cylinder,
    tile_cache_layer_mark_oriented_box,
    tile_cache_layer_region_count,
    tile_cache_layer_regions,
    tile_cache_layer_set_areas,
    tile_cache_obstacle_info,
    tile_cache_obstacle_ref_at,
    tile_cache_params,
    tile_cache_poly_mesh_areas,
    tile_cache_poly_mesh_create,
    tile_cache_poly_mesh_flags,
    tile_cache_poly_mesh_info,
    tile_cache_poly_mesh_polys,
    tile_cache_poly_mesh_verts,
    tile_cache_query_tiles,
    tile_cache_remove_obstacle,
    tile_cache_remove_tile,
    tile_cache_tile_at,
    tile_cache_tile_info,
    tile_cache_tile_ref_at,
    tile_cache_tiles_at,
    tile_cache_update,
    tile_data_build,
    tile_grid_compute,
    tile_layout,
    tri_area_2d,
};

/// One refusal: the call, and the result it returned or was refused with.
pub const Frame = struct {
    pub const aegis_public_frame = true;
    call: Call,
    result: c.Result,
};

/// What refused, oldest first. Every call that can fail takes one and pushes
/// one frame where it refuses; a call that only passes a refusal on pushes
/// nothing. Four are kept, and a fifth sets `truncated`.
pub const Failure = aegis.err.Context(Frame, 4);

/// Turns a C result into a Zig error, or void on success, leaving no frame:
/// for a call whose refusal `aegis.assert.never` rules out.
///
/// The switch is exhaustive over `c.Result` with no `else` branch: adding a
/// result to the C enum without handling it here is a compile error, which is
/// the property `AbiLayout.result_count` also checks at runtime.
pub fn map(result: c.Result) Error!void {
    return switch (result) {
        .ok => {},
        .invalid_argument => Error.InvalidArgument,
        .out_of_memory => Error.OutOfMemory,
        .bad_format => Error.BadFormat,
        .unsupported_version => Error.UnsupportedVersion,
        .buffer_too_small => Error.BufferTooSmall,
        .bake_failed => Error.BakeFailed,
        .empty_result => Error.EmptyResult,
        .query_failed => Error.QueryFailed,
        .out_of_nodes => Error.OutOfNodes,
        .tile_occupied => Error.TileOccupied,
        .navmesh_full => Error.NavMeshFull,
        .search_in_progress => Error.SearchInProgress,
        .no_search => Error.NoSearch,
        .not_found => Error.NotFound,
        .already_built => Error.AlreadyBuilt,
        .crowd_full => Error.CrowdFull,
    };
}

/// The result an error stands for, `map` read backwards.
pub fn resultOf(e: Error) c.Result {
    return switch (e) {
        Error.InvalidArgument => .invalid_argument,
        Error.OutOfMemory => .out_of_memory,
        Error.BadFormat => .bad_format,
        Error.UnsupportedVersion => .unsupported_version,
        Error.BufferTooSmall => .buffer_too_small,
        Error.BakeFailed => .bake_failed,
        Error.EmptyResult => .empty_result,
        Error.QueryFailed => .query_failed,
        Error.OutOfNodes => .out_of_nodes,
        Error.TileOccupied => .tile_occupied,
        Error.NavMeshFull => .navmesh_full,
        Error.SearchInProgress => .search_in_progress,
        Error.NoSearch => .no_search,
        Error.NotFound => .not_found,
        Error.AlreadyBuilt => .already_built,
        Error.CrowdFull => .crowd_full,
    };
}

/// Nothing on success; otherwise a frame on `failure` naming `call` and
/// `result`, and the error `result` maps to.
pub fn check(failure: *Failure, call: Call, result: c.Result) Error!void {
    map(result) catch |e| {
        failure.push(.{ .call = call, .result = result });
        return e;
    };
}

/// A refusal the binding makes before C is called: a frame on `failure`
/// naming `call` and the result `e` stands for, and `e`.
pub fn refuse(failure: *Failure, call: Call, e: Error) Error {
    failure.push(.{ .call = call, .result = resultOf(e) });
    return e;
}

/// Borrowed, static description of a result, for logging.
pub fn name(result: c.Result) [:0]const u8 {
    return std.mem.span(c.zrcResultName(result));
}

test "ok is the only result that is not an error, and all of them are named" {
    try map(.ok);
    inline for (@typeInfo(c.Result).@"enum".field_values) |value| {
        const result: c.Result = @fromBackingInt(@intCast(value));
        try std.testing.expect(name(result).len > 0);
        if (result == .ok) continue;
        if (map(result)) |_| return error.ResultMappedToSuccess else |e| try std.testing.expectEqual(result, resultOf(e));
    }
}

test "a refusal pushes one frame naming its call and result, and success pushes none" {
    var failure: Failure = .{};
    try check(&failure, .find_path, .ok);
    try std.testing.expectEqual(0, failure.frames().len);
    try std.testing.expectError(Error.OutOfNodes, check(&failure, .find_path, .out_of_nodes));
    try std.testing.expectEqual(Error.InvalidArgument, refuse(&failure, .convex_volume, Error.InvalidArgument));
    try std.testing.expectEqualSlices(Frame, &.{
        .{ .call = .find_path, .result = .out_of_nodes },
        .{ .call = .convex_volume, .result = .invalid_argument },
    }, failure.frames());
}

/// `zrcFindPath` -> `find_path`, as `Call` spells it.
fn callName(c_name: []const u8, buf: []u8) []const u8 {
    const s = c_name["zrc".len..];
    var len: usize = 0;
    for (s, 0..) |ch, i| {
        const prev = if (i == 0) 0 else s[i - 1];
        const next = if (i + 1 < s.len) s[i + 1] else 0;
        const boundary = i != 0 and
            ((std.ascii.isLower(prev) and (std.ascii.isUpper(ch) or std.ascii.isDigit(ch))) or
                (std.ascii.isDigit(prev) and std.ascii.isUpper(ch) and std.ascii.isLower(next)) or
                (std.ascii.isUpper(prev) and std.ascii.isUpper(ch) and std.ascii.isLower(next)));
        if (boundary) {
            buf[len] = '_';
            len += 1;
        }
        buf[len] = std.ascii.toLower(ch);
        len += 1;
    }
    return buf[0..len];
}

/// Every module that refuses, read as it is compiled.
const sources = [_][]const u8{
    @embedFile("asserts.zig"),   @embedFile("bake.zig"),     @embedFile("crowd.zig"),
    @embedFile("debugdraw.zig"), @embedFile("geom.zig"),     @embedFile("memory.zig"),
    @embedFile("navmesh.zig"),   @embedFile("pipeline.zig"), @embedFile("query.zig"),
    @embedFile("tilecache.zig"), @embedFile("vec.zig"),      @embedFile("zrecast.zig"),
};

test "every frame a checked C call pushes names that call" {
    // `err.check(failure, .x, c.zrcY(` must name `.x` as `zrcY` spells it, so
    // a frame can never name a call other than the one that refused. The
    // compiler proves both exist.
    var checked: usize = 0;
    for (sources) |source| {
        var rest: []const u8 = source;
        const marker = "err.check(failure, .";
        while (std.mem.find(u8, rest, marker)) |at| {
            rest = rest[at + marker.len ..];
            const end = std.mem.findScalar(u8, rest, ',') orelse return error.TestUnexpectedResult;
            const named = rest[0..end];
            const after = std.mem.trimStart(u8, rest[end + 1 ..], " ");
            const prefix: usize = if (std.mem.startsWith(u8, after, "c.zrc")) "c.".len else if (std.mem.startsWith(u8, after, "zc.zrc")) "zc.".len else continue;
            const paren = std.mem.findScalar(u8, after, '(') orelse return error.TestUnexpectedResult;
            var buf: [128]u8 = undefined;
            try std.testing.expectEqualStrings(callName(after[prefix..paren], &buf), named);
            checked += 1;
        }
    }
    try std.testing.expect(checked > 300);
}

test "no module refuses without a frame" {
    // A refusal is made by `check` or `refuse`, which push the frame. One of
    // this set's errors returned any other way would reach the caller with
    // nothing on its `Failure`; and `map`, which pushes none, is only for a
    // result `aegis.assert.never` rules out.
    inline for (@typeInfo(Error).error_set.error_names.?) |error_name| {
        inline for (.{ "return ", "catch ", "orelse " }) |verb| {
            inline for (.{ "error.", "Error.", "err.Error." }) |spelling| {
                for (sources) |source| {
                    try std.testing.expect(std.mem.find(u8, source, verb ++ spelling ++ error_name) == null);
                }
            }
        }
    }
    for (sources) |source| {
        // Whether the innermost declaration above is a test block: a module
        // never makes a `Failure` of its own, whose frames no caller would see.
        var in_test = false;
        var lines = std.mem.splitScalar(u8, source, '\n');
        while (lines.next()) |line| {
            const code = std.mem.trimStart(u8, line, " ");
            if (std.mem.startsWith(u8, code, "test \"")) in_test = true;
            if (std.mem.startsWith(u8, code, "fn ") or std.mem.startsWith(u8, code, "pub fn ")) in_test = false;
            if (std.mem.find(u8, line, "err.map(") != null) {
                try std.testing.expect(std.mem.find(u8, line, "catch aegis.assert.never(") != null);
            }
            if (std.mem.find(u8, line, "err.Failure = .{}") != null) try std.testing.expect(in_test);
        }
    }
}
