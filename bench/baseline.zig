//! The C++ twins of the bench rows, defined in bench/baseline.cpp.

pub extern fn zrecast_baseline_open(image: [*]const u8, size: usize, max_nodes: i32) ?*anyopaque;
pub extern fn zrecast_baseline_close(world: *anyopaque) void;
pub extern fn zrecast_baseline_load(image: [*]const u8, size: usize, times: u64) u64;
pub extern fn zrecast_baseline_load_validated(image: [*]const u8, size: usize, times: u64) u64;
pub extern fn zrecast_baseline_nearest(world: *anyopaque, points: [*]const [3]f32, count: usize, extents: *const [3]f32, times: u64) u64;
pub extern fn zrecast_baseline_route(world: *anyopaque, refs: [*]const u32, points: [*]const [3]f32, pairs: [*]const [2]u32, count: usize, times: u64) u64;
pub extern fn zrecast_baseline_raycast(world: *anyopaque, refs: [*]const u32, points: [*]const [3]f32, pairs: [*]const [2]u32, count: usize, times: u64) f32;
pub extern fn zrecast_baseline_crowd_open(world: *anyopaque, refs: [*]const u32, points: [*]const [3]f32, pairs: [*]const [2]u32, count: usize, radius: f32) ?*anyopaque;
pub extern fn zrecast_baseline_crowd_update(crowd: *anyopaque, dt: f32, times: u64) u64;
pub extern fn zrecast_baseline_crowd_close(crowd: *anyopaque) void;
