//! Production source layers, lowest first. Every production source has one
//! place; test code is in no layer.
const gantry = @import("gantry");

pub const layers: []const gantry.rules.Layer = &.{
    .{ .name = "C boundary", .patterns = &.{ "src/c.zig", "src/vec.zig" } },
    .{ .name = "errors and memory", .patterns = &.{ "src/error.zig", "src/memory.zig" } },
    .{ .name = "geometry and bake", .patterns = &.{ "src/asserts.zig", "src/bake.zig", "src/geom.zig" } },
    .{ .name = "navmesh and pipeline", .patterns = &.{ "src/navmesh.zig", "src/pipeline.zig" } },
    .{ .name = "query and tile cache", .patterns = &.{ "src/query.zig", "src/tilecache.zig" } },
    .{ .name = "crowd", .patterns = &.{"src/crowd.zig"} },
    .{ .name = "debug draw", .patterns = &.{"src/debugdraw.zig"} },
    .{ .name = "bindings", .patterns = &.{"src/zrecast.zig"} },
};

pub const entries: []const []const u8 = &.{"src/zrecast.zig"};

pub const modules: []const gantry.NamedModule = &.{};

pub const references: []const gantry.rules.ReferenceRule = &.{
    .{ .name = "named dependencies", .unresolved_only = true, .except_targets = &.{
        "builtin",
        "std",
        "zrecast_options",
    } },
    .{ .name = "source siblings", .suffix = ".zig", .relative = true, .except_targets = &.{"src/**"} },
};

pub const required = [_][]const u8{
    "src/c.zig",
    "src/zrecast.zig",
};

/// Tokens only their owners may spell: the C boundary is declared in
/// src/c.zig alone, and the test fixture's entry points in the test that
/// links it.
pub const owned: []const gantry.rules.TokenRule = &.{
    .{ .name = "the C boundary", .tokens = &.{"extern"}, .owners = &.{ "src/c.zig", "src/integration_test.zig" } },
};
