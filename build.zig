const std = @import("std");

/// Recast — the navmesh baker.
///
/// This list is explicit rather than a directory glob: a glob would silently
/// start compiling whatever a future re-vendor drops in.
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

/// DebugUtils — what a bake produced, drawn through a host's renderer, and
/// dumped to a host's bytes. Depends on Recast's, Detour's and the tile
/// cache's headers.
const debug_utils_sources = [_][]const u8{
    "libs/recastnavigation/DebugUtils/Source/DebugDraw.cpp",
    "libs/recastnavigation/DebugUtils/Source/DetourDebugDraw.cpp",
    "libs/recastnavigation/DebugUtils/Source/RecastDebugDraw.cpp",
    "libs/recastnavigation/DebugUtils/Source/RecastDump.cpp",
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
    "ffi/zrecast_debugdraw.cpp",
    "ffi/zrecast_dump.cpp",
    "ffi/zrecast_abi.cpp",
};

/// The upstream include directories the library compiles against.
const upstream_includes = [_][]const u8{
    "libs/recastnavigation/Recast/Include",
    "libs/recastnavigation/Detour/Include",
    "libs/recastnavigation/DetourTileCache/Include",
    "libs/recastnavigation/DetourCrowd/Include",
    "libs/recastnavigation/DebugUtils/Include",
};

/// How the C++ was built. Every ABI- or behaviour-affecting option is mirrored
/// into the `zrecast_options` module, so the wrapper can never disagree with
/// the library it links.
const Options = struct {
    shared: bool,
    enable_asserts: bool,
    sanitize_c: bool,
};

/// What a project that depends on zrecast builds: the `zrecast` C library
/// (vendored Recast and Detour plus the FFI layer in ffi/) and the `zrecast`
/// module over it.
pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const options: Options = .{
        .shared = b.option(bool, "shared", "Build the C library as a shared object") orelse false,
        .enable_asserts = b.option(bool, "enable_asserts", "Keep Recast's and Detour's internal asserts (defaults to on in Debug)") orelse (optimize == .debug),
        // Off by default, and deliberately NOT tied to `optimize`: Zig's C
        // sanitizer emits calls into a runtime linked only into a compilation
        // that is itself sanitized, so defaulting it on in Debug hands a
        // consumer who forgot to forward `optimize` a link failure naming a
        // __ubsan symbol.
        .sanitize_c = b.option(bool, "sanitize_c", "Compile the C++ with Zig's undefined-behaviour sanitizer") orelse false,
    };

    //=====================================================================
    // The C library: vendored Recast + Detour, plus the zrecast FFI layer.
    //=====================================================================

    const lib = b.addLibrary(.{
        .name = "zrecast",
        .linkage = if (options.shared) .dynamic else .static,
        .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }),
    });
    if (target.result.abi != .msvc) lib.root_module.link_libcpp = true;
    for (upstream_includes) |path| lib.root_module.addIncludePath(b.path(path));
    lib.root_module.addIncludePath(b.path("ffi"));

    if (options.shared) {
        // Not just an MSVC concern: on ELF and Mach-O these switch ZRC_API to
        // an explicit default-visibility attribute, which is what keeps the
        // export list down to zrecast's own entry points once the sources are
        // compiled with -fvisibility=hidden.
        lib.root_module.addCMacro("ZRECAST_SHARED", "");
        lib.root_module.addCMacro("ZRECAST_BUILD", "");
    }

    const upstream_flags = compileFlags(b, target, false, options.enable_asserts);
    for ([_][]const []const u8{ &recast_sources, &detour_sources, &detour_tile_cache_sources, &detour_crowd_sources, &debug_utils_sources }) |files| {
        lib.root_module.addCSourceFiles(.{ .files = files, .flags = upstream_flags });
    }
    lib.root_module.addCSourceFiles(.{ .files = &zrecast_ffi_sources, .flags = compileFlags(b, target, true, options.enable_asserts) });
    lib.root_module.sanitize_c = if (options.sanitize_c) .full else .off;

    // Consumers get the public header without reaching into the source tree.
    lib.installHeader(b.path("ffi/zrecast.h"), "zrecast.h");
    // Registered unconditionally: `std.Build.Dependency.artifact` scans the
    // dependency's install step, so an artifact not installed here is one
    // `dep.artifact("zrecast")` cannot find.
    b.installArtifact(lib);

    //=====================================================================
    // The Zig module. No include path: the wrapper hand-writes its externs
    // rather than translating the header, so nothing Zig-side compiles C.
    //=====================================================================

    const module = b.addModule("zrecast", .{
        .root_source_file = b.path("src/zrecast.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zrecast_options", .module = optionsModule(b, options) },
            .{ .name = "aegis", .module = b.dependency("aegis", .{ .target = target, .optimize = optimize }).module("aegis") },
        },
    });
    module.linkLibrary(lib);
}

/// `cxxFlags`, then NDEBUG exactly as `enable_asserts` says. Zig defines
/// NDEBUG for C by itself in ReleaseFast and ReleaseSmall, so keeping the
/// asserts there takes an explicit -UNDEBUG, which follows Zig's own flags.
fn compileFlags(b: *std.Build, target: std.Build.ResolvedTarget, own: bool, enable_asserts: bool) []const []const u8 {
    const asserts: []const []const u8 = &.{if (enable_asserts) "-UNDEBUG" else "-DNDEBUG"};
    return std.mem.concat(b.allocator, []const u8, &.{ cxxFlags(target, own), asserts }) catch @panic("out of memory");
}

/// The C++ flags, for upstream's translation units or for the package's own.
///
/// Recast and Detour throw nothing and use no RTTI — there is not a single
/// `throw`, `try`, `dynamic_cast` or `typeid` in either tree — so both
/// features are disabled where doing so is reliable. Under the MSVC ABI it is
/// not: the Microsoft standard library headers these sources pull in assume
/// exceptions, and disabling them through Clang flags is a known source of
/// header errors, so the MSVC ABI keeps the defaults.
///
/// No -fno-access-control (the FFI layer uses only public upstream API) and no
/// blanket -fno-sanitize=undefined (the `sanitize_c` option decides).
///
/// -ffp-contract=off pins the one liberty clang keeps over IEEE-754
/// arithmetic: contracting a multiply and an add into a fused multiply-add
/// wherever the target has one, rounding once where the separate form rounds
/// twice. AArch64's base instruction set mandates FMA and x86-64's baseline
/// has none, so an unpinned build is licensed to cook different floats per
/// architecture. On tests/reference.cpp the pin is load-bearing outright: the
/// bit-identity test in src/vec.zig compares the Zig arithmetic against what
/// the header specifies, not against a contracted form of it.
///
/// The package's own translation units additionally get -Wall -Wextra; the
/// vendored ones do not, being upstream's to clean up.
fn cxxFlags(target: std.Build.ResolvedTarget, own: bool) []const []const u8 {
    const msvc = target.result.abi == .msvc;
    if (msvc and own) return &.{ "-std=c++17", "-ffp-contract=off", "-Wall", "-Wextra" };
    if (msvc) return &.{ "-std=c++17", "-ffp-contract=off" };
    const common = .{ "-std=c++17", "-ffp-contract=off", "-fno-exceptions", "-fno-rtti", "-fvisibility=hidden", "-fvisibility-inlines-hidden" };
    if (own) return &(common ++ .{ "-Wall", "-Wextra" });
    return &common;
}

/// How the C++ was built, plus the package version, as the `zrecast_options`
/// module. The version's one home is build.zig.zon; the ZRC_VERSION_* macros
/// in ffi/zrecast.h restate it for C consumers, and the version test compares
/// the two so they cannot drift.
fn optionsModule(b: *std.Build, options: Options) *std.Build.Module {
    const step = b.addOptions();
    inline for (@typeInfo(Options).@"struct".field_names) |name| step.addOption(bool, name, @field(options, name));
    step.addOption([]const u8, "package_version", @import("build.zig.zon").version);
    return step.createModule();
}

/// zrecast's development: its tests, examples and checks, under preflight's
/// gate. Run through bay. The package is reached as a dependency here, the way
/// a consumer reaches it, so the artifact, its installed header and the
/// module are the ones a consumer gets.
pub fn dev(b: *std.Build, tools: type) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const preflight = tools.preflight;
    const package = b.dependency("zrecast", .{ .target = target, .optimize = optimize });
    const p = package.builder;

    const test_step = b.step("test", "Run the zrecast tests");
    const check_step = b.step("check", "Compile the zrecast tests, examples and build configurations without running them");
    const filters: []const []const u8 = if (b.option([]const u8, "test-filter", "Select tests by name")) |filter| b.allocator.dupe([]const u8, &.{filter}) catch @panic("OOM") else &.{};

    //=====================================================================
    // Tests: against the C++ under Zig's undefined-behaviour sanitizer, as a
    // consumer builds it by default, and with the asserts flipped from the
    // mode's default, so both sides of every NDEBUG seam run. The suite's C++
    // is the dependency's artifact, so the C++ a consumer links is the C++
    // tested.
    //=====================================================================

    const asserts_default = optimize == .debug;
    const arms = [_]struct { name: []const u8, options: Options }{
        .{ .name = "zrecast-tests-ubsan", .options = .{ .shared = false, .enable_asserts = asserts_default, .sanitize_c = true } },
        .{ .name = "zrecast-tests", .options = .{ .shared = false, .enable_asserts = asserts_default, .sanitize_c = false } },
        .{ .name = "zrecast-tests-asserts-flipped", .options = .{ .shared = false, .enable_asserts = !asserts_default, .sanitize_c = false } },
    };
    for (arms) |arm| {
        const tests = suite(b, p, target, optimize, arm.name, arm.options, filters);
        test_step.dependOn(&b.addRunArtifact(tests).step);
        check_step.dependOn(&tests.step);
    }

    // The C boundary on its own, with no Zig in the picture: plain C11 that
    // includes <zrecast.h> through the artifact's installed header, walks
    // bake, build, serialise, reload and query with a malloc allocator, and
    // checks every byte came back. Twice: as a consumer links it, and with
    // the program and the C++ under the sanitizer, since some of the
    // boundary's paths only C reaches. The sanitized arm takes the library
    // the sanitized suite arm builds, so the graph holds no further instance
    // of the package.
    const library = package.artifact("zrecast");
    for ([_]struct { name: []const u8, sanitize_c: bool }{
        .{ .name = "zrecast-c-smoke", .sanitize_c = false },
        .{ .name = "zrecast-c-smoke-ubsan", .sanitize_c = true },
    }) |arm| {
        const options: Options = .{ .shared = false, .enable_asserts = asserts_default, .sanitize_c = arm.sanitize_c };
        const linked = if (arm.sanitize_c) b.dependency("zrecast", .{
            .target = target,
            .optimize = optimize,
            .enable_asserts = options.enable_asserts,
            .sanitize_c = options.sanitize_c,
        }).artifact("zrecast") else library;
        const c_smoke = b.addExecutable(.{
            .name = arm.name,
            .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }),
        });
        c_smoke.root_module.sanitize_c = if (arm.sanitize_c) .full else .off;
        c_smoke.root_module.addIncludePath(p.path("tests"));
        c_smoke.root_module.addCSourceFile(.{ .file = p.path("tests/c_smoke.c"), .flags = &.{ "-std=c11", "-Wall", "-Wextra" } });
        c_smoke.root_module.linkLibrary(linked);
        c_smoke.root_module.linkLibrary(fixtureLibrary(b, p, target, optimize, options, linked));
        test_step.dependOn(&b.addRunArtifact(c_smoke).step);
        check_step.dependOn(&c_smoke.step);
    }

    // The configurations no arm above links: the shared library, which is
    // what proves the export list, and the asserts flipped under the
    // sanitizer.
    for ([_]Options{
        .{ .shared = true, .enable_asserts = asserts_default, .sanitize_c = false },
        .{ .shared = false, .enable_asserts = !asserts_default, .sanitize_c = true },
    }) |options| {
        const variant = b.dependency("zrecast", .{
            .target = target,
            .optimize = optimize,
            .shared = options.shared,
            .enable_asserts = options.enable_asserts,
            .sanitize_c = options.sanitize_c,
        }).artifact("zrecast");
        test_step.dependOn(&variant.step);
        check_step.dependOn(&variant.step);
    }

    //=====================================================================
    // Examples, built AND run against the module a consumer gets.
    // examples/usage.zig is also where README.md's Usage block comes from
    // (`bay docs -- usage`), so the snippet a reader copies is code the
    // tests execute.
    //=====================================================================

    const examples_step = b.step("examples", "Build and run the examples");
    for (example_sources) |source| {
        const example = b.addExecutable(.{
            .name = std.Io.Dir.path.stem(source),
            .root_module = b.createModule(.{
                .root_source_file = p.path(source),
                .target = target,
                .optimize = optimize,
                .imports = &.{.{ .name = "zrecast", .module = package.module("zrecast") }},
            }),
        });
        examples_step.dependOn(&b.addRunArtifact(example).step);
        check_step.dependOn(&example.step);
    }
    test_step.dependOn(examples_step);

    //=====================================================================
    // CI wiring
    //=====================================================================

    for (checks) |c| withAegis(p, preflight.addCheck(b, p, c.name, c.source));
    // README.md's numbers block, which the docs check regenerates through
    // this step (ci/preflight.json).
    _ = tool(b, p, test_step, "measurements", "ci/measurements.zig", "Print README.md's numbers block, recomputed from the tree");
    // The coverage record's other modes (`bay coverage -- --record`, `--names`,
    // `--summary AREA`, ...), through the program check-coverage runs.
    const coverage_check = preflight.addCheck(b, p, "check-coverage", "ci/coverage.zig");
    withAegis(p, coverage_check);
    const coverage = b.addRunArtifact(coverage_check);
    coverage.setCwd(p.path("."));
    coverage.addPassthruArgs();
    b.step("coverage", "Run ci/coverage.zig in the mode given after --").dependOn(&coverage.step);

    // libs/recastnavigation against upstream at the pinned commit, which the
    // manifest fetches as a dev dependency.
    const vendor = b.addExecutable(.{
        .name = "zrecast-vendor",
        .root_module = b.createModule(.{ .root_source_file = p.path("ci/vendor.zig"), .target = b.graph.host, .optimize = .debug }),
    });
    withAegis(p, vendor);
    const verify = b.addRunArtifact(vendor);
    verify.addDirectoryArg2(p.path("libs/recastnavigation"), .{});
    verify.addDirectoryArg2(b.dependency("recastnavigation", .{}).path("."), .{});
    verify.addFileArg(p.path("build.zig.zon"));
    verify.addFileArg(p.path("docs/design.md"));
    const vendor_tests = b.addTest(.{ .root_module = vendor.root_module });
    verify.step.dependOn(&b.addRunArtifact(vendor_tests).step);
    const vendor_step = b.step("check-vendor", "Prove libs/recastnavigation is upstream at the pinned commit");
    vendor_step.dependOn(&verify.step);

    // Outside the gate, each a rebuild per input: the mutation probes, and
    // the cook digests across targets. Their own tests run with the suite.
    const probe = tool(b, p, test_step, "probe", "ci/probes.zig", "Mutate each invariant in ci/probes/ and require the named failure (`bay probe [filter]`)");
    probe.addPassthruArgs();
    const determinism = tool(b, p, test_step, "determinism", "ci/determinism.zig", "Cook the fixture on every target this host can run, and compare the digests");
    for (determinism_targets) |entry| {
        const resolved = b.resolveTargetQuery(std.Target.Query.parse(.{ .arch_os_abi = entry.triple }) catch @panic("a determinism target does not parse"));
        determinism.addArgs(&.{ entry.triple, entry.group, entry.runner });
        determinism.addArtifactArg2(suite(b, p, resolved, .debug, b.fmt("zrecast-tests-{s}", .{entry.triple}), .{ .shared = false, .enable_asserts = true, .sanitize_c = false }, &.{}), .{});
    }

    // The hardened tier fuzzes the properties in src/integration_test.zig,
    // which live in the test step; zrecast runs no threads of its own.
    preflight.addCi(b, p, .{
        .tests = test_step,
        .hardened = .{},
        // Each binding against the same Detour work written in C++.
        .bench = .{
            .programs = &.{.{ .name = "zrecast-bench", .source = "bench/binding.zig" }},
            .imports = benchImports,
            .target = target,
            .optimize = optimize,
        },
    });
    // The build a consumer gets: the module, the library and aegis, fetching
    // nothing.
    preflight.addConsumerCheck(b, p, .{
        .package = "zrecast",
        .program = p.path("ci/consumer.zig"),
        .packages = &.{p.dependency("aegis", .{})},
    });
}

/// What the bench imports, in the mode it builds in: an imported module keeps
/// its own mode, so a ReleaseFast bench over a Debug zrecast would time the
/// Debug one. `baseline` is the bench's C++ twin, bench/baseline.cpp, compiled
/// against the vendored headers with the library's flags and linked to the
/// same library.
fn benchImports(b: *std.Build, target: std.Build.ResolvedTarget, optimize: std.lang.Optimize) []const std.Build.Module.Import {
    const package = b.dependency("zrecast", .{ .target = target, .optimize = optimize });
    const p = package.builder;
    const baseline = b.createModule(.{
        .root_source_file = p.path("bench/baseline.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    if (target.result.abi != .msvc) baseline.link_libcpp = true;
    for (upstream_includes) |path| baseline.addIncludePath(p.path(path));
    baseline.addCSourceFile(.{ .file = p.path("bench/baseline.cpp"), .flags = compileFlags(b, target, true, optimize == .debug) });
    baseline.linkLibrary(package.artifact("zrecast"));
    // Its polygon references are the binding's type, the same four bytes.
    baseline.addImport("zrecast", package.module("zrecast"));
    return b.allocator.dupe(std.Build.Module.Import, &.{
        .{ .name = "zrecast", .module = package.module("zrecast") },
        .{ .name = "baseline", .module = baseline },
    }) catch @panic("OOM");
}

/// The suite, against the package's library built with `options`.
fn suite(b: *std.Build, p: *std.Build, target: std.Build.ResolvedTarget, optimize: std.lang.Optimize, name: []const u8, options: Options, filters: []const []const u8) *std.Build.Step.Compile {
    const library = b.dependency("zrecast", .{
        .target = target,
        .optimize = optimize,
        .enable_asserts = options.enable_asserts,
        .sanitize_c = options.sanitize_c,
    }).artifact("zrecast");
    const tests = b.addTest(.{
        .name = name,
        .filters = filters,
        .root_module = b.createModule(.{
            .root_source_file = p.path("src/zrecast.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zrecast_options", .module = optionsModule(b, options) },
                .{ .name = "aegis", .module = p.dependency("aegis", .{ .target = target, .optimize = optimize }).module("aegis") },
                .{ .name = "shakedown", .module = b.dependency("shakedown", .{ .target = target, .optimize = optimize }).module("shakedown") },
            },
        }),
    });
    tests.root_module.linkLibrary(library);
    tests.root_module.linkLibrary(fixtureLibrary(b, p, target, optimize, options, library));
    // The surface check reads the header translated as `zrecast_h`, on the
    // test module alone, never on the shipped one.
    const header = b.addTranslateC(.{ .root_source_file = p.path("ffi/zrecast.h"), .target = target, .optimize = optimize, .link_libc = true });
    tests.root_module.addImport("zrecast_h", header.createModule());
    // The sanitized C++ calls into the sanitizer's runtime, which Zig
    // bundles by itself only into a Debug compilation.
    tests.bundle_ubsan_rt = options.sanitize_c;
    return tests;
}

/// The test fixture: synthetic input geometry generated in C++
/// (tests/fixture.cpp), and tests/reference.cpp, which restates upstream's
/// inline vector and scalar math so the Zig ports can be compared against it
/// bit for bit. Built as the arm's library is built.
fn fixtureLibrary(b: *std.Build, p: *std.Build, target: std.Build.ResolvedTarget, optimize: std.lang.Optimize, options: Options, library: *std.Build.Step.Compile) *std.Build.Step.Compile {
    const fixture = b.addLibrary(.{
        .name = "zrecast-fixture",
        .linkage = .static,
        .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }),
    });
    if (target.result.abi != .msvc) fixture.root_module.link_libcpp = true;
    fixture.root_module.addIncludePath(p.path("tests"));
    fixture.root_module.addIncludePath(p.path("libs/recastnavigation/Recast/Include"));
    fixture.root_module.addIncludePath(p.path("libs/recastnavigation/Detour/Include"));
    fixture.root_module.addCSourceFiles(.{ .root = p.path("."), .files = &.{ "tests/fixture.cpp", "tests/reference.cpp" }, .flags = compileFlags(b, target, true, options.enable_asserts) });
    fixture.root_module.sanitize_c = if (options.sanitize_c) .full else .off;
    fixture.root_module.linkLibrary(library);
    return fixture;
}

/// A development tool: a host program run from the package root, its tests
/// run with the suite.
fn tool(b: *std.Build, p: *std.Build, test_step: *std.Build.Step, name: []const u8, source: []const u8, description: []const u8) *std.Build.Step.Run {
    const exe = b.addExecutable(.{
        .name = b.fmt("zrecast-{s}", .{name}),
        .root_module = b.createModule(.{ .root_source_file = p.path(source), .target = b.graph.host, .optimize = .debug }),
    });
    withAegis(p, exe);
    const run = b.addRunArtifact(exe);
    run.setCwd(p.path("."));
    b.step(name, description).dependOn(&run.step);
    test_step.dependOn(&b.addRunArtifact(b.addTest(.{ .root_module = exe.root_module })).step);
    return run;
}

/// Gives a host program under ci/ aegis, whose `own.readFile` reads a file
/// whole with a limit (preflight's P006 holds every package to it).
fn withAegis(p: *std.Build, exe: *std.Build.Step.Compile) void {
    exe.root_module.addImport("aegis", p.dependency("aegis", .{ .target = exe.root_module.resolved_target.?, .optimize = .debug }).module("aegis"));
}

/// What `bay determinism` cooks on: each target, the C library whose qsort
/// it shares (targets in one group must agree to the byte, across groups the
/// difference is reported), and what runs it here.
const determinism_targets = [_]struct { triple: []const u8, group: []const u8, runner: []const u8 }{
    .{ .triple = "native", .group = "host", .runner = "native" },
    .{ .triple = "x86_64-linux-gnu", .group = "gnu", .runner = "docker:debian:bookworm-slim:linux/amd64" },
    .{ .triple = "aarch64-linux-gnu", .group = "gnu", .runner = "docker:debian:bookworm-slim:linux/arm64" },
    .{ .triple = "x86_64-linux-musl", .group = "musl", .runner = "docker:alpine:3.20:linux/amd64" },
    .{ .triple = "aarch64-linux-musl", .group = "musl", .runner = "docker:alpine:3.20:linux/arm64" },
};

/// Every example, listed rather than globbed: a build graph that scans a
/// directory is not reproducible from the manifest alone.
const example_sources = [_][]const u8{
    "examples/usage.zig",
};

/// The repository checks: each is built, its tests run, then it runs over the
/// tree from the repository root. ci/preflight.json runs each as an extra check.
const checks = [_]struct { name: []const u8, source: []const u8 }{
    .{ .name = "check-comments", .source = "ci/comments.zig" },
    .{ .name = "check-ignored", .source = "ci/ignored.zig" },
    .{ .name = "check-numbers", .source = "ci/numbers.zig" },
};
