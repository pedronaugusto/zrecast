const std = @import("std");

/// Builds zrecast the way a real consumer does, which is not the way its own
/// test suite does.
///
/// The two are genuinely different code paths. An in-repo test reaches the
/// module and the library through the same `std.Build` graph that created
/// them; a consumer reaches them through `b.dependency`, which resolves
/// artifacts by scanning the dependency's *install step* and installed
/// headers by their spelling. Neither is exercised by anything in `src/` or
/// `tests/`, so both can break while the whole suite stays green.
pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Forwarded so this build can be run twice, once either way; running
    // with `-Denable_asserts=false` proves the option plumbing a consumer
    // would use.
    const enable_asserts = b.option(
        bool,
        "enable_asserts",
        "Keep the boundary's argument checks in release modes",
    ) orelse true;

    const zrecast = b.dependency("zrecast", .{
        .target = target,
        .optimize = optimize,
        .enable_asserts = enable_asserts,
    });

    // 1. The Zig module, driven the way the README's example drives it.
    const zig_consumer = b.addExecutable(.{
        .name = "zig-consumer",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zrecast", .module = zrecast.module("zrecast") },
            },
        }),
    });

    // 2. The C library as an artifact with its installed header —
    //    `#include <zrecast.h>` resolves only through installHeader, which
    //    nothing in-repo exercises (the in-repo C files reach the header via
    //    an include path into the source tree).
    const c_consumer = b.addExecutable(.{
        .name = "c-consumer",
        .root_module = b.createModule(.{ .target = target, .optimize = optimize }),
    });
    c_consumer.root_module.link_libc = true;
    c_consumer.root_module.addCSourceFile(.{
        .file = b.path("src/main.c"),
        .flags = &.{"-std=c11"},
    });
    c_consumer.root_module.linkLibrary(zrecast.artifact("zrecast"));

    const step = b.step("run", "Build and run both consumers");
    step.dependOn(&b.addRunArtifact(zig_consumer).step);
    step.dependOn(&b.addRunArtifact(c_consumer).step);
    b.getInstallStep().dependOn(step);
}
