const std = @import("std");

/// Recast — the navmesh baker.
///
/// This list is explicit rather than a directory glob for two reasons: a glob
/// would silently start compiling whatever a future re-vendor drops in, and the
/// vendored tree contains one further library (DebugUtils) that this package
/// deliberately does not build.
const recast_sources = [_][]const u8{
    "libs/recastnavigation/Recast/Source/Recast.cpp",
    "libs/recastnavigation/Recast/Source/RecastAlloc.cpp",
    "libs/recastnavigation/Recast/Source/RecastArea.cpp",
    "libs/recastnavigation/Recast/Source/RecastAssert.cpp",
    "libs/recastnavigation/Recast/Source/RecastContour.cpp",
    "libs/recastnavigation/Recast/Source/RecastFilter.cpp",
    "libs/recastnavigation/Recast/Source/RecastLayers.cpp",
    "libs/recastnavigation/Recast/Source/RecastMesh.cpp",
    "libs/recastnavigation/Recast/Source/RecastMeshDetail.cpp",
    "libs/recastnavigation/Recast/Source/RecastRasterization.cpp",
    "libs/recastnavigation/Recast/Source/RecastRegion.cpp",
};

/// Detour — the runtime query side. Depends on nothing from Recast; the two are
/// linked into one library here only because one package ships both.
const detour_sources = [_][]const u8{
    "libs/recastnavigation/Detour/Source/DetourAlloc.cpp",
    "libs/recastnavigation/Detour/Source/DetourAssert.cpp",
    "libs/recastnavigation/Detour/Source/DetourCommon.cpp",
    "libs/recastnavigation/Detour/Source/DetourNavMesh.cpp",
    "libs/recastnavigation/Detour/Source/DetourNavMeshBuilder.cpp",
    "libs/recastnavigation/Detour/Source/DetourNavMeshQuery.cpp",
    "libs/recastnavigation/Detour/Source/DetourNode.cpp",
};

/// DetourTileCache — dynamic obstacles carved into a baked mesh at runtime.
/// Depends on Detour's headers and on nothing from Recast.
const detour_tile_cache_sources = [_][]const u8{
    "libs/recastnavigation/DetourTileCache/Source/DetourTileCache.cpp",
    "libs/recastnavigation/DetourTileCache/Source/DetourTileCacheBuilder.cpp",
};

/// DetourCrowd — local steering and avoidance for many agents at once.
/// Depends on Detour's headers and on nothing from Recast.
const detour_crowd_sources = [_][]const u8{
    "libs/recastnavigation/DetourCrowd/Source/DetourCrowd.cpp",
    "libs/recastnavigation/DetourCrowd/Source/DetourLocalBoundary.cpp",
    "libs/recastnavigation/DetourCrowd/Source/DetourObstacleAvoidance.cpp",
    "libs/recastnavigation/DetourCrowd/Source/DetourPathCorridor.cpp",
    "libs/recastnavigation/DetourCrowd/Source/DetourPathQueue.cpp",
    "libs/recastnavigation/DetourCrowd/Source/DetourProximityGrid.cpp",
};

/// The zrecast C boundary. One translation unit per concern — deliberately not
/// a single monolithic binding file. The bake/navmesh/query split is the same
/// split the two lifecycles have: baking is a cook step, querying is a frame.
const zrecast_ffi_sources = [_][]const u8{
    "ffi/zrecast_core.cpp",
    "ffi/zrecast_bake.cpp",
    "ffi/zrecast_navmesh.cpp",
    "ffi/zrecast_query.cpp",
    "ffi/zrecast_geom.cpp",
    "ffi/zrecast_context.cpp",
    "ffi/zrecast_pipeline.cpp",
    "ffi/zrecast_stages.cpp",
    "ffi/zrecast_layers.cpp",
    "ffi/zrecast_tilecache.cpp",
    "ffi/zrecast_crowd.cpp",
    "ffi/zrecast_steering.cpp",
    "ffi/zrecast_corridor.cpp",
    "ffi/zrecast_abi.cpp",
};

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const options = .{
        .shared = b.option(
            bool,
            "shared",
            "Build the C library as a shared object",
        ) orelse false,
        .enable_asserts = b.option(
            bool,
            "enable_asserts",
            "Keep Recast's and Detour's internal asserts (defaults to on in Debug)",
        ) orelse (optimize == .Debug),
        .sanitize_c = b.option(
            bool,
            "sanitize_c",
            "Keep Zig's C undefined-behaviour sanitizer enabled",
        ) orelse (optimize == .Debug),
    };

    // Every ABI- or behaviour-affecting option is mirrored into a Zig module so
    // the wrapper can never disagree with how the C++ was compiled. The single
    // `options` struct above is the one source both sides read from.
    const options_step = b.addOptions();
    inline for (std.meta.fields(@TypeOf(options))) |field| {
        options_step.addOption(field.type, field.name, @field(options, field.name));
    }
    // The package version's one home is build.zig.zon; the ZRC_VERSION_*
    // macros in ffi/zrecast.h restate it for C consumers, and the version
    // test compares the two so they cannot drift.
    options_step.addOption([]const u8, "package_version", @import("build.zig.zon").version);
    const options_module = options_step.createModule();

    //=====================================================================
    // The C library: vendored Recast + Detour, plus the zrecast FFI layer.
    //=====================================================================

    const lib = b.addLibrary(.{
        .name = "zrecast",
        .linkage = if (options.shared) .dynamic else .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });

    lib.root_module.link_libc = true;
    if (target.result.abi != .msvc) lib.root_module.link_libcpp = true;

    lib.root_module.addIncludePath(b.path("libs/recastnavigation/Recast/Include"));
    lib.root_module.addIncludePath(b.path("libs/recastnavigation/Detour/Include"));
    lib.root_module.addIncludePath(b.path("libs/recastnavigation/DetourTileCache/Include"));
    lib.root_module.addIncludePath(b.path("libs/recastnavigation/DetourCrowd/Include"));
    lib.root_module.addIncludePath(b.path("ffi"));

    if (!options.enable_asserts) lib.root_module.addCMacro("NDEBUG", "");
    if (options.shared) {
        // Not just an MSVC concern: on ELF and Mach-O these switch ZRC_API to
        // an explicit default-visibility attribute, which is what keeps the
        // export list down to zrecast's own entry points once the sources are
        // compiled with -fvisibility=hidden below.
        lib.root_module.addCMacro("ZRECAST_SHARED", "");
        lib.root_module.addCMacro("ZRECAST_BUILD", "");
    }

    // Recast and Detour throw nothing and use no RTTI — there is not a single
    // `throw`, `try`, `dynamic_cast` or `typeid` in either tree, and Recast
    // even ships its own placement-new tag so it can avoid <new>'s operator —
    // so both features are disabled where doing so is reliable.
    //
    // Under the MSVC ABI they are not reliable: the Microsoft standard library
    // headers these sources pull in are written assuming exceptions are
    // available, and disabling them through Clang flags is a well-known source
    // of header errors. The saving is a little code size; the cost would be a
    // toolchain-specific build failure, so the MSVC ABI keeps the defaults.
    //
    // Note what is NOT here on any target: no -fno-access-control (the FFI
    // layer uses only public upstream API, so it has no reason to defeat access
    // checking) and no blanket -fno-sanitize=undefined (UBSan stays on in
    // Debug, controlled by the `sanitize_c` option, so that real undefined
    // behaviour surfaces instead of being suppressed).
    // -ffp-contract=off pins the one liberty clang keeps over IEEE-754
    // arithmetic: contracting a multiply and an add into a fused multiply-add
    // wherever the target has one, rounding once where the separate form
    // rounds twice. AArch64's base instruction set mandates FMA and x86-64's
    // baseline has none, so an unpinned build is licensed to cook different
    // floats per architecture. Measured 2026-09-02 on the test fixture
    // (x86-64 baseline vs x86-64-v3, Debug and ReleaseFast): the cooked
    // bytes happen not to move without the pin — the cook quantises its
    // floats into grid cells, and a last-bit difference has to land exactly
    // on a cell boundary to reach the output. The pin stays because the
    // cross-target byte-equality gate (ci/determinism.sh) must not lean on
    // that landing never happening for any input anyone ever cooks. On
    // tests/reference.cpp the same flag is load-bearing outright, and the
    // comment at its inclusion below says why.
    const msvc = target.result.abi == .msvc;
    const cxx_flags: []const []const u8 = if (msvc)
        &.{ "-std=c++17", "-ffp-contract=off" }
    else
        &.{
            "-std=c++17",
            "-ffp-contract=off",
            "-fno-exceptions",
            "-fno-rtti",
            "-fvisibility=hidden",
            "-fvisibility-inlines-hidden",
        };

    // Our own translation units additionally get -Wall -Wextra. The vendored
    // sources do not: they are not ours to clean up, and a wall of upstream
    // warnings is the fastest way to stop reading warnings altogether.
    const ffi_flags: []const []const u8 = if (msvc)
        &.{ "-std=c++17", "-ffp-contract=off", "-Wall", "-Wextra" }
    else
        &.{
            "-std=c++17",
            "-ffp-contract=off",
            "-fno-exceptions",
            "-fno-rtti",
            "-fvisibility=hidden",
            "-fvisibility-inlines-hidden",
            "-Wall",
            "-Wextra",
        };

    lib.root_module.addCSourceFiles(.{
        .files = &recast_sources,
        .flags = cxx_flags,
    });
    lib.root_module.addCSourceFiles(.{
        .files = &detour_sources,
        .flags = cxx_flags,
    });
    lib.root_module.addCSourceFiles(.{
        .files = &detour_tile_cache_sources,
        .flags = cxx_flags,
    });
    lib.root_module.addCSourceFiles(.{
        .files = &detour_crowd_sources,
        .flags = cxx_flags,
    });
    lib.root_module.addCSourceFiles(.{
        .files = &zrecast_ffi_sources,
        .flags = ffi_flags,
    });
    lib.root_module.sanitize_c = if (options.sanitize_c) .full else .off;

    // Consumers get the public header without reaching into the source tree.
    lib.installHeader(b.path("ffi/zrecast.h"), "zrecast.h");

    //=====================================================================
    // The Zig module.
    //=====================================================================

    const module = b.addModule("zrecast", .{
        .root_source_file = b.path("src/zrecast.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zrecast_options", .module = options_module },
        },
    });
    // No include path: the wrapper hand-writes its externs rather than
    // @cImport-ing the header, so nothing Zig-side compiles C.
    module.linkLibrary(lib);

    // Unconditional: `b.dependency(...).artifact("zrecast")` resolves by
    // scanning this install step, so a consumer linking the C library finds
    // nothing unless the artifact is registered here.
    b.installArtifact(lib);

    //=====================================================================
    // Examples.
    //=====================================================================

    // Built AND run: the README's Usage section is extracted from this file
    // by ci/readme_usage.sh, so the snippet a reader copies is a program the
    // suite has executed.
    const usage = b.addExecutable(.{
        .name = "zrecast-usage",
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/usage.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zrecast", .module = module },
            },
        }),
    });
    usage.root_module.sanitize_c = if (options.sanitize_c) .full else .off;
    const examples_step = b.step("examples", "Build and run the examples");
    examples_step.dependOn(&b.addRunArtifact(usage).step);

    //=====================================================================
    // Tests.
    //=====================================================================

    // Synthetic input geometry, generated programmatically. Keeps the suite
    // self-contained: no vendored third-party meshes, no asset provenance to
    // account for, and a navmesh baked at test time from geometry whose shape
    // the assertions know exactly.
    const fixture = b.addLibrary(.{
        .name = "zrecast-fixture",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    fixture.root_module.link_libc = true;
    if (target.result.abi != .msvc) fixture.root_module.link_libcpp = true;
    fixture.root_module.addIncludePath(b.path("ffi"));
    fixture.root_module.addIncludePath(b.path("tests"));
    // tests/reference.cpp shims upstream's inline vector and scalar math, so it
    // needs Recast's and Detour's own headers directly rather than only
    // zrecast's FFI layer.
    fixture.root_module.addIncludePath(b.path("libs/recastnavigation/Recast/Include"));
    fixture.root_module.addIncludePath(b.path("libs/recastnavigation/Detour/Include"));
    if (!options.enable_asserts) fixture.root_module.addCMacro("NDEBUG", "");
    // -ffp-contract=off (part of ffi_flags) is load-bearing on reference.cpp
    // specifically: contracting a multiply and an add into a fused
    // multiply-add would make the bit-identity test in src/vec.zig compare the
    // Zig arithmetic against something other than what the header specifies.
    fixture.root_module.addCSourceFiles(.{
        .files = &.{ "tests/fixture.cpp", "tests/reference.cpp" },
        .flags = ffi_flags,
    });
    fixture.root_module.sanitize_c = if (options.sanitize_c) .full else .off;
    fixture.root_module.linkLibrary(lib);

    const tests = b.addTest(.{
        .name = "zrecast-tests",
        // Run one check in isolation, which matters when proving that a
        // deliberately broken declaration is caught by the guard that is
        // supposed to catch it rather than by whichever test crashes first.
        .filters = b.option(
            []const []const u8,
            "test-filter",
            "Only run tests whose name contains this",
        ) orelse &.{},
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/zrecast.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zrecast_options", .module = options_module },
            },
        }),
    });
    // The Zig module compiles no C, but the sanitizer setting still has to be
    // mirrored onto it: the C libraries above are separate compilations, and it
    // is this final link that decides whether the UBSan runtime is pulled in.
    // Leaving it unset means every `-Dsanitize_c=true` build outside Debug ends
    // in undefined references to __ubsan_handle_*.
    tests.root_module.sanitize_c = if (options.sanitize_c) .full else .off;
    tests.root_module.linkLibrary(lib);
    tests.root_module.linkLibrary(fixture);

    const test_step = b.step("test", "Run zrecast tests");
    test_step.dependOn(&b.addRunArtifact(tests).step);
    // The examples are part of the suite: a README snippet that stops
    // compiling is a test failure, not a documentation chore.
    test_step.dependOn(examples_step);

    // Installs the test binary instead of running it. A target this host cannot
    // execute can still be built here and run somewhere that can, which is how
    // ci/determinism.sh reaches Linux from a macOS host and how cook digests
    // get compared across architectures this machine only cross-compiles for.
    const test_artifact_step = b.step(
        "test-artifact",
        "Install the test binary without running it",
    );
    test_artifact_step.dependOn(&b.addInstallArtifact(tests, .{}).step);

    // A C-only smoke test proves the boundary stands on its own, independent of
    // anything Zig-side — the header is a real C contract, not a private detail
    // of the wrapper.
    const c_smoke = b.addExecutable(.{
        .name = "zrecast-c-smoke",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    c_smoke.root_module.link_libc = true;
    c_smoke.root_module.addIncludePath(b.path("ffi"));
    c_smoke.root_module.addIncludePath(b.path("tests"));
    c_smoke.root_module.addCSourceFile(.{
        .file = b.path("tests/c_smoke.c"),
        .flags = &.{ "-std=c11", "-Wall", "-Wextra" },
    });
    c_smoke.root_module.sanitize_c = if (options.sanitize_c) .full else .off;
    c_smoke.root_module.linkLibrary(lib);
    c_smoke.root_module.linkLibrary(fixture);

    const c_test_step = b.step("test-c", "Run the C-level smoke test");
    c_test_step.dependOn(&b.addRunArtifact(c_smoke).step);
    test_step.dependOn(c_test_step);
}
