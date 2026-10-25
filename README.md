# zrecast

Zig bindings for [recastnavigation](https://github.com/recastnavigation/recastnavigation):
Recast bakes a navigation mesh out of level geometry, and Detour paths over
that mesh at runtime. Upstream v1.6.0 is vendored in `libs/recastnavigation`,
verbatim, and `build.zig` compiles it.

## Usage

The block below is a region of [`examples/usage.zig`](examples/usage.zig),
which the tests build and run; the gate fails if this copy has drifted from
it.

<!-- BEGIN GENERATED zig build docs -- usage -->
```zig
const zrecast = @import("zrecast");

zrecast.setAllocator(gpa);
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
const mesh = try zrecast.NavMesh.initFromBytes(.init(image.bytes));
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

Requires Zig 0.17.0 and libc:

```sh
zig fetch --save git+https://github.com/pedronaugusto/zrecast#main
```

Then in `build.zig`:

```zig
const zrecast_dep = b.dependency("zrecast", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("zrecast", zrecast_dep.module("zrecast"));
```

Forward `optimize` as shown, or a release executable links a Debug library.
A C or C++ host links the library and includes the header it installs:

```zig
exe.root_module.linkLibrary(zrecast_dep.artifact("zrecast"));  // then #include <zrecast.h>
```

zrecast's one dependency is [aegis](https://github.com/pedronaugusto/aegis);
upstream is in the tree.

| Build option | Default | Effect |
|---|---|---|
| `-Dshared` | off | Build the C library as a shared object |
| `-Denable_asserts` | on in Debug | Keep Recast's and Detour's internal asserts, in any mode |
| `-Dsanitize_c` | off | Compile the C++ with Zig's undefined-behaviour sanitizer |

## Features

- Every public name in Recast, Detour, DetourCrowd, DetourTileCache and
  DebugUtils at the vendored version carries a verdict: bound through the C
  boundary, reimplemented in Zig, C++-only with the reason, or declared
  upstream and defined nowhere. `bay check-coverage` re-derives the names from
  the headers and checks each verdict against the tree.
- The bake: the whole Recast pipeline in one call, tiled or untiled, or one
  stage per call with every intermediate a handle the host can read and edit.
  A failed bake fills a log with the stage that failed and why.
- The navmesh: a bake becomes bytes, bytes become a navmesh, and a tile reads
  back by value. Bytes from outside arrive as a `zrecast.Image`, aegis's
  untrusted-input type, and a serialised navmesh is validated before Detour
  reads it: every count, then every index, coordinate and off-mesh connection
  it can point at.
- The queries: the whole of `dtNavMeshQuery`, sliced pathfinding, polygon
  references and the node pool. A path that stops short of its goal is a
  result with `partial` set, not an error.
- Dynamic obstacles through a tile cache whose codec is the host's, and crowds:
  agents with corridors, local boundaries, obstacle avoidance and a path queue,
  named by references a reused slot cannot be driven through.
- Debug drawing through a renderer a Zig type implements, a display list, and
  OBJ and binary dumps through four I/O hooks.
- Allocation through any `std.mem.Allocator` with `setAllocator`. It is
  process-wide, because upstream's two allocator hooks are.
- A cook is the same bytes on every target sharing a C library:
  `-ffp-contract=off`, a polynomial cosine for the slope limit, and Zig vector
  math tested bit-identical to upstream's. A frame's crowd steering calls the C
  library's `cosf` and `sinf`, so it is not.

| File | What it holds |
|---|---|
| `bake.zig` | Recast: triangle soup in, polygon mesh out, tiled or untiled. |
| `navmesh.zig` | A baked mesh becomes bytes, bytes become a `dtNavMesh`, and a tile reads back by value. |
| `query.zig` | Detour: `dtNavMeshQuery`, filters, polygon references, sliced search, the node pool. |
| `pipeline.zig` | The bake one stage per call. |
| `tilecache.zig` | Dynamic obstacles. |
| `crowd.zig` | Agents: corridor, local boundary, proximity grid, obstacle avoidance, path queue. |
| `debugdraw.zig` | DebugUtils: drawing, the display list, the OBJ and binary dumps. |
| `geom.zig`, `vec.zig` | Detour's computational geometry, and the `dtV`/`rcV` families in Zig. |
| `memory.zig`, `asserts.zig`, `error.zig` | The allocator hook, the assertion handler, the error set. |

[docs/design.md](docs/design.md) has the layers, the bound surface capability
by capability, what upstream does that the boundary checks for it, the cook's
determinism, and how the vendored copy is checked and updated.

## Testing

```sh
bay test
```

runs the gate: formatting, structure, lint and documentation, the checks in
`ci/`, a consumer build, and the suite three times: against the C++ under
Zig's undefined-behaviour sanitizer, as a consumer builds it, and with the
asserts flipped from the mode's default. The suite generates its own geometry,
so no third-party meshes are shipped. `bay probe` breaks each invariant in
`ci/probes/` on purpose and requires the failure it names, and
`bay determinism` compares cook digests across the targets the host can run.
`bay dev bench` times each binding beside the same Detour work in C++.

## By the numbers

<!-- BEGIN GENERATED zig build measurements -->
| | |
|---:|---|
| **0.1.1** | version (one home: `build.zig.zon`) |
| **383** | C entry points (`ZRC_API` in `ffi/*.h`), each with its Zig extern |
| **1186** | public Recast/Detour names, each carrying a verdict in `ci/coverage/` |
| **1041** | of them reachable through the C boundary (`BOUND`) |
| **91** | C++-only surface a C boundary cannot carry (`LANGUAGE`), each with the reason |
| **53** | reimplemented on the Zig side (`ZIG`), each naming its mirror |
| **1** | declared upstream and defined nowhere (`UNDEFINED`), so no host can call it |
| **1145** | assertions in the standalone C smoke test |
| **30** | vendored recastnavigation translation units `build.zig` compiles |
| **98** | invariants `bay probe` mutates, each with the test that must notice |
<!-- END GENERATED -->

`bay measurements` recomputes those from the tree, and the gate fails if the
committed block differs, or if a document states a number by hand that
`ci/numbers.zig` does not account for.

## Licence

MIT, see [LICENSE](LICENSE), which covers this package's own code. Vendored
recastnavigation is zlib-licensed, copyright Mikko Mononen and contributors;
its licence text ships with the package at `libs/recastnavigation/License.txt`.
