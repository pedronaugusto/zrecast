# zrecast

[![CI](https://github.com/pedronaugusto/zrecast/actions/workflows/ci.yml/badge.svg)](https://github.com/pedronaugusto/zrecast/actions/workflows/ci.yml)

Zig bindings for [recastnavigation](https://github.com/recastnavigation/recastnavigation):
Recast bakes a navigation mesh out of level geometry, Detour paths over that
mesh at runtime. Upstream v1.6.0 is vendored in `libs/recastnavigation`,
verbatim, and `build.zig` compiles it.

## Usage

The block below is a region of [`examples/usage.zig`](examples/usage.zig),
which `zig build examples` builds and runs; `ci/readme_usage.sh` extracts it
and `ci/check-docs.sh` fails if this copy has drifted.

<!-- BEGIN GENERATED ci/readme_usage.sh -->
```zig
const zrecast = @import("zrecast");

try zrecast.setAllocator(gpa);
defer zrecast.resetAllocator();

// In the cook: geometry in, bytes out. `log` is optional, and the
// difference between a diagnosable failure and a bare error code.
var log: [1024]u8 = undefined;
const poly = try zrecast.PolyMesh.bake(zrecast.defaultConfig(), .{
    .verts = &level_verts, // 3 floats per vertex, right-handed, Y up
    .tris = &level_indices, // 3 indices per triangle
}, null, &log);
defer poly.deinit();

const baked = try zrecast.NavMesh.initFromPolyMesh(poly, null);
defer baked.deinit();
const image = try baked.serialize();
defer image.deinit();
// ...write `image.bytes` wherever the pipeline puts build artefacts.

// In the game: bytes in, answers out.
const mesh = try zrecast.NavMesh.initFromBytes(image.bytes);
defer mesh.deinit();
const query = try zrecast.NavMeshQuery.init(mesh, 2048);
defer query.deinit();

const filter = zrecast.defaultFilter();
const extents = [3]f32{ 2, 4, 2 };
const from = try query.findNearestPoly(agent_pos, extents, &filter);
const to = try query.findNearestPoly(target_pos, extents, &filter);
if (from.ref == null or to.ref == null) return error.OffMesh;

var corridor: [256]zrecast.PolyRef = undefined;
const path = try query.findPath(
    from.ref.?,
    to.ref.?,
    from.point,
    to.point,
    &filter,
    &corridor,
);

var corners: [64][3]f32 = undefined;
const walk = try query.findStraightPath(
    from.point,
    to.point,
    corridor[0..path.len],
    .{},
    &corners,
    null,
    null,
);
// Walk `corners[0..walk.len]`; `path.partial` says the goal was out of
// reach and the corridor stops at the closest polygon instead.
```
<!-- END GENERATED -->

## Install

```sh
zig fetch --save git+https://github.com/pedronaugusto/zrecast#v0.1.1
```

```zig
const zrecast_dep = b.dependency("zrecast", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("zrecast", zrecast_dep.module("zrecast"));

// A C or C++ host links the artifact instead and includes the installed zrecast.h.
exe.root_module.linkLibrary(zrecast_dep.artifact("zrecast"));
```

Zig 0.16.0 or newer. Nothing else is fetched: upstream is in the tree and the
package has no dependencies of its own.

| Build option | Default | Effect |
|---|---|---|
| `-Dshared` | off | Build the C library as a shared object |
| `-Denable_asserts` | on in Debug | Keep Recast's and Detour's internal asserts |
| `-Dsanitize_c` | on in Debug | Keep Zig's C undefined-behaviour sanitizer |

Each is declared once and mirrored into a Zig `options` module, so the wrapper
cannot disagree with how the C++ was compiled.

## The API

Everything is re-exported from the root module.

| File | What it holds |
|---|---|
| `bake.zig` | Recast: triangle soup in, polygon mesh out, tiled or untiled. |
| `navmesh.zig` | The seam — a baked mesh becomes bytes, bytes become a `dtNavMesh`, and a tile can be read back by value. |
| `query.zig` | Detour: the whole of `dtNavMeshQuery`, filters, polygon references, sliced search, the node pool. |
| `pipeline.zig` | The same bake taken apart, one stage per call, every intermediate a handle a host owns. |
| `tilecache.zig` | Dynamic obstacles: compressed layers rebuilt as obstacles appear and go away. |
| `crowd.zig` | Agents: corridor, local boundary, proximity grid, obstacle avoidance, path queue. |
| `debugdraw.zig` | DebugUtils: every container drawn through a renderer a Zig type implements, the display list, and the OBJ and binary dumps. |
| `geom.zig`, `vec.zig` | Detour's computational geometry, callable directly, and the `dtV`/`rcV` families in Zig. |
| `memory.zig`, `asserts.zig`, `error.zig` | The allocator seam, the assertion handler, the error set. |

Every public name in Recast, Detour, DetourCrowd, DetourTileCache and
DebugUtils at the vendored version carries one verdict, and
`ci/check-coverage.sh` re-derives the name list from the headers and checks
each verdict against the tree: bound through the C boundary, reimplemented in
Zig, or C++-only — upstream's own containers (`rcVectorBase`, `rcIntArray`,
`dtNodeQueue`), its libm wrappers and its `dtSwapEndian` overloads. One name is
declared upstream and defined nowhere, so nothing can call it; that has a
verdict of its own, rechecked against the vendored sources. Nothing is left
over. [docs/surface.md](docs/surface.md) lists the bound surface capability by
capability and [docs/coverage.md](docs/coverage.md) how it is measured.

## Design

### Bake time and run time are different programs

Recast voxelises a triangle soup, filters it for what an agent of a given size
can stand on, and emits a polygon mesh: slow, allocation-heavy, and its output
an artefact a cook step checks into a content pipeline. Detour loads that
artefact and answers questions about it — nearest polygon, route, what a
character hits walking that way — allocating once, at load. So a tool uses
`bake.zig` and `navmesh.zig`; a game uses `navmesh.zig` and `query.zig`.

Because the boundary is one C entry point per concern in its own translation
unit, a host that never calls the bake never links it: a ReleaseFast C program
that deserialises a navmesh and queries it contains none of
`rcCreateHeightfield`, `rcRasterizeTriangles`, `rcBuildContours`,
`rcErodeWalkableArea` or `rcBuildPolyMesh`, and comes out about 170 KB smaller
than the same program with a bake call added (measured 2026-09-02 with `zig
c++ -O2` on x86_64-windows).

### Partial is a result, not an error

`findPath` returns a corridor and a `partial` flag. Detour sets it when the
goal was unreachable, when the node pool ran out, or when the caller's buffer
filled, and in all three cases the corridor is a real, walkable best effort
towards the goal. Collapsing that into an error makes every caller treat "no
route" and "long route" identically, which is the bug that freezes agents at
the edge of a map. It is a field, and the caller decides.

### The bake assigns the polygon flags

Recast leaves `rcPolyMesh::flags` zeroed, and Detour's default filter admits a
polygon only if it shares a bit with `include_flags`. Bake, build, query — the
obvious sequence — therefore returns an empty result from every query, with
nothing reporting a problem. `zrcPolyMeshBake` assigns `ZRC_AREA_WALKABLE` /
`ZRC_POLY_FLAG_WALKABLE` to every walkable polygon, so the obvious thing works.
The suite checks both directions: a default filter finds the ground, and a
filter excluding exactly that flag finds nothing.

### A failed bake says which stage failed

Recast's eleven build stages each return a bare `bool` and send the reason to a
virtual log callback with no default implementation. Pass a buffer as
`ZrcBakeLog` and it is filled with those messages; without one, a failed bake
is undiagnosable by construction. `rcBuildPolyMesh` succeeding with zero
polygons is not an error upstream, but it produces a navmesh that answers every
query with "nowhere to go", so here it is `EmptyResult`, with an explanation of
the two things that usually cause it.

### Drawing is a renderer the host supplies

`DebugUtils` decides what a heightfield, a region, a contour set or a navmesh
looks like as points, lines, triangles and quads; what those become is the
host's. `DebugDraw` is that seam as a Zig struct, built from a renderer type's
own methods at compile time, so a renderer is a Zig struct rather than a table
of C callbacks — and the four hooks of `FileIO` are the same for bytes, so a
dump reaches whatever a host calls a file. Nothing in that half allocates,
mutates a container, or affects a navmesh a query will see. The two read
entry points are the exception and say so.

### Validation at the boundary

Recast and Detour assert in debug and read on regardless in release. Geometry
is range- and NaN-checked before the rasteriser sees it, and a serialised
navmesh is validated before it is trusted — every count, then every index,
coordinate and off-mesh connection the image can point at, because a single
byte in an otherwise valid image overruns a fixed stack array in four separate
Detour functions. That buys memory safety, not trustworthiness: an in-bounds
image can still describe a useless navmesh, and neither library is hardened
against arbitrary hostile input, so put a length-and-signature container around
anything loaded from an untrusted source.

The Zig side hand-writes `extern struct`s mirroring `ffi/zrecast.h`, and
nothing in either compiler checks the two still agree, so `zrcAbiLayout()`
reports what the C++ compiled to and a test asserts every size and offset
against the Zig declarations by name. `static_assert`s in `ffi/zrecast_abi.cpp`
fail the build if a re-vendored upstream moves a mirrored constant, a
serialised struct, the format version or the polygon reference type.
[docs/validation.md](docs/validation.md) lists each check and the upstream
behaviour it stands in front of.

### Allocators

`setAllocator` routes every Recast and Detour allocation through a
`std.mem.Allocator`. It is process-wide, because both upstream seams are:
`rcAllocSetCustom` and `dtAllocSetCustom` take a bare pair of function pointers
with nowhere to thread a host pointer through. Both libraries also free with
`free(ptr)`, no size, while Zig requires the size back, so `src/memory.zig`
stores a header ahead of each block. Neither `rcAlloc` nor `dtAlloc` has an
alignment parameter, so there is one compile-time alignment
(`ZRC_ALLOC_ALIGNMENT`) rather than a per-call one. The C API keeps upstream's
shape, so a plain C host can still pass `malloc`/`free` in two lines.

zrecast's own allocations are checked and rolled back on every path, tested by
inducing a failure at every allocation site in turn. Upstream's are not, and
that cannot be fixed from outside the library — the cases are recorded with
file and line in [UPSTREAM.md](UPSTREAM.md). Run the bake somewhere an
allocation failure is fatal anyway, which is where a cook already lives.

### A cook is deterministic; a frame's steering is not

One target cooking the same geometry twice produces the same bytes, asserted by
the suite; two targets sharing a C library produce the same bytes, gated by
`ci/determinism.sh`. Two different C libraries are compared and reported, not
promised: upstream sorts with `qsort` in five places, ties fall to the
implementation, and any order they land in is a valid navmesh. Ship cooks from
one platform, or key the asset cache by cook platform.

Holding the first two costs `-ffp-contract=off` on every target, a bit-identity
test between `src/vec.zig` and upstream's own `inline` definitions, and a
polynomial cosine in place of `cosf` for the slope limit;
[docs/determinism.md](docs/determinism.md) has the detail. The guarantee covers
baked assets and stops there — a frame's steering reaches `cosf` and `sinf` in
the adaptive velocity sampler, so a host that needs a reproducible replay
should record its inputs.

## Scope

- The tile cache carries no compressor and invents no container format. The
  codec is the host's, and so is the callback that decides each rebuilt
  polygon's flags.
- Serialisation is single-tile. A single-tile navmesh round-trips to bytes; a
  tiled one is cooked and loaded a tile at a time.
- A tile the tile cache rebuilds reaches the navmesh without the validation an
  added tile gets; the mesh-process callback is narrowed to the fields it is
  for.
- Upstream's own allocation failures are not recoverable from here.

## Platforms

| | Suite executed by CI | Compile-checked by CI |
|---|---|---|
| Linux | x86_64 glibc | aarch64 glibc, x86_64 and aarch64 musl |
| macOS | aarch64 | x86_64 |
| Windows | x86_64, gnu and MSVC ABIs | aarch64 gnu |

Every job in that matrix passed on run
[`34779510358`](https://github.com/pedronaugusto/zrecast/actions/runs/34779510358).
Zig's default ABI for a Windows host is `gnu`, not `msvc`, so a bare `zig build
test` on a Windows runner never touches the MSVC path; both are named
explicitly there and both run the suite.

## Testing

```sh
zig build test      # the Zig suite
zig build test-c    # the C-level smoke test, plain C11, no Zig in the picture
ci/run.sh           # the full matrix CI runs; --quick is native Debug only
ci/probe.sh         # mutate each invariant, report the ones the suite survives
ci/determinism.sh   # cook digests across architectures and C libraries
```

The suite generates its own geometry — a ground plane, a wall with a gap at one
end, a pillar, and an island connected to nothing — so no third-party meshes
are shipped. `ci/run.sh` is the same matrix as `.github/workflows/ci.yml`, and
`ci/check-mirror.sh` fails if the two stop agreeing. See
[docs/testing.md](docs/testing.md).

## By the numbers

<!-- BEGIN GENERATED ci/measurements.sh --markdown -->
| | |
|---:|---|
| **0.1.1** | version (one home: `build.zig.zon`) |
| **383** | C entry points (`ZRC_API` in `ffi/*.h`) |
| **383** | Zig externs (`pub extern fn` in `src/`) |
| **1186** | public Recast/Detour names, each carrying a verdict in `tools/` |
| **1041** | of them reachable through the C boundary (`BOUND`) |
| **91** | C++-only surface a C boundary cannot carry (`LANGUAGE`), each with the reason |
| **53** | reimplemented on the Zig side (`ZIG`), each naming its mirror |
| **1** | declared upstream and defined nowhere (`UNDEFINED`), so no host can call it |
| **166** | Zig tests `zig build test` executes |
| **1145** | assertions in the standalone C smoke test |
| **30** | vendored recastnavigation translation units `build.zig` compiles |
| **19911** | C boundary lines (`ffi/`) |
| **20432** | Zig source lines (`src/`) |
| **96** | invariants `ci/probe.sh` mutates, each with the test that must notice |
| **32** | steps `ci/run.sh` runs |
| **8** | further targets `ci/run.sh` cross-compiles |
<!-- END GENERATED -->

`ci/measurements.sh` recomputes those from the tree, and `ci/check-docs.sh`
fails the build if the committed block differs or if this file states any other
number `tools/doc_numbers.txt` has not accounted for.

## Contributing

Issues and pull requests are welcome: [docs/contributing.md](docs/contributing.md)
has the three rules that fail a build, and [BINDING.md](BINDING.md) is the
walkthrough for adding surface.

## Licence

MIT, see [LICENSE](LICENSE), which covers this package's own code. Vendored
recastnavigation is zlib-licensed, copyright Mikko Mononen and contributors;
its licence text ships with the package at `libs/recastnavigation/License.txt`.
