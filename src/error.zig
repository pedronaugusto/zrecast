//! Translation between the C result enum and a Zig error set.

const std = @import("std");
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

/// Turns a C result into a Zig error, or void on success.
///
/// The switch is exhaustive over `c.Result` with no `else` branch: adding a
/// result to the C enum without handling it here is a compile error, which is
/// the property `AbiLayout.result_count` also checks at runtime.
pub fn check(result: c.Result) Error!void {
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

/// Borrowed, static description of a result, for logging.
pub fn name(result: c.Result) [:0]const u8 {
    return std.mem.span(c.zrcResultName(result));
}

test "ok is the only result that is not an error, and all of them are named" {
    try check(.ok);
    inline for (@typeInfo(c.Result).@"enum".fields) |field| {
        const result: c.Result = @enumFromInt(field.value);
        try std.testing.expect(name(result).len > 0);
        if (result == .ok) continue;
        if (check(result)) |_| return error.ResultMappedToSuccess else |_| {}
    }
}
