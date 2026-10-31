# zrecast design

How the package is built, what each layer holds, what upstream does that the
boundary stands in front of, and how each claim the README makes is checked.

## Layers

Upstream is C++, and C++ does not cross a linking boundary intact, so the
package is three layers and a new capability lands in each.

1. **`libs/recastnavigation` is upstream, byte for byte.** Nothing is added or
   changed there; `bay check-vendor` compares it against the pinned commit,
   both ways, and fails on a local edit or a file upstream does not have.
   Behaviour upstream lacks belongs in the next layer.
2. **`ffi/` is the C boundary.** One translation unit per concern
   (`zrecast_bake.cpp`, `zrecast_query.cpp`, …), each entry point `ZRC_API` and
   `zrc`-prefixed, declared in the one public header `ffi/zrecast.h` with
   `Zrc`-prefixed types. A mirrored struct or constant gets a `static_assert`
   in `ffi/zrecast_abi.cpp` against upstream's definition, so a re-vendor that
   moves anything fails the build. Validation lives here too: an index, a
   length or a reference upstream would trust is checked before upstream sees
   it, and each refusal is recorded under
   [Known upstream behaviour](#known-upstream-behaviour).
3. **`src/` is the Zig layer.** The externs (`pub extern fn zrc…`) sit in the
   module that owns them (`bake.zig`, `navmesh.zig`, `query.zig`, `crowd.zig`,
   `tilecache.zig`, `pipeline.zig`), next to the wrapper that calls them.
   The shipped module translates no header; the test module does, and the
   surface check holds the externs to it (see [The ABI guard](#the-abi-guard)).

`ci/layers.zig` holds the order of the Zig modules: the C boundary, errors and
memory, geometry and bake, navmesh and pipeline, query and tile cache, crowd,
debug draw, and the root. A module imports only from the layers below it, and
only `src/c.zig` and the integration tests declare `extern` functions.

## Bake time and run time are different programs

Recast voxelises a triangle soup, filters it for what an agent of a given size
can stand on, and emits a polygon mesh: slow, allocation-heavy, and its output
an artefact a cook step checks into a content pipeline. Detour loads that
artefact and answers questions about it (nearest polygon, route, what a
character hits walking that way), allocating once, at load. So a tool uses
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
route" and "long route" identically, which freezes agents at the edge of a
map. It is a field, and the caller decides.

### The bake assigns the polygon flags

Recast leaves `rcPolyMesh::flags` zeroed, and Detour's default filter admits a
polygon only if it shares a bit with `include_flags`. Bake, build, query, the
obvious sequence, therefore returns an empty result from every query, with
nothing reporting a problem. `zrcPolyMeshBake` assigns `ZRC_AREA_WALKABLE` /
`ZRC_POLY_FLAG_WALKABLE` to every walkable polygon, so the obvious sequence
works. The suite checks both directions: a default filter finds the ground,
and a filter excluding exactly that flag finds nothing.

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
of C callbacks, and the four hooks of `FileIO` are the same for bytes, so a
dump reaches whatever a host calls a file. Nothing in that half allocates,
mutates a container, or affects a navmesh a query will see. The two read
entry points are the exception and say so.

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
that cannot be fixed from outside the library; the cases are recorded with file
and line under [Known upstream behaviour](#known-upstream-behaviour). Recast's
own vector placement-news into the result of a failed allocation, Detour's
BV-tree builder indexes an unchecked array, and its node pool asserts and then
writes through the null. One case, a `dtNavMesh` whose `init` ran out of memory
and which cannot then be destroyed, is worked around narrowly, and the bounded
cost of that workaround is stated there. Run the bake somewhere an allocation
failure is fatal anyway, which is where a cook already lives. The runtime half
behaves far better, and that is the half a game links.

A free of a block the bridge did not issue, a sub-slice of one or a block
from another allocator, never reaches the backing allocator: the header's tag
tells it apart. A Debug or ReleaseSafe build stops on it, naming it, since it
is the caller's bug; a ReleaseFast or ReleaseSmall build leaves the block
alone, so it leaks and the heap stays intact. `tests/foreign_free.zig` holds
both, out of process.

### Assertions

`setAssertHandler` routes Recast's and Detour's internal assertions to a
host's handler, and it is process-wide for the same reason `setAllocator` is:
`rcAssertFailSetCustom` and `dtAssertFailSetCustom` take a bare function
pointer. One install replaces the handler for every caller in the process,
and the handler runs on whichever thread fails the assertion. In a build that
compiled the asserts out it is recorded and never called
(`assertsEnabled`).

## Binding conventions

An entry point operating on an object is named object-first
(`zrcPolyMeshBake`, `zrcHeightfieldAddSpan`), so the header groups by the
thing operated on; a query or free helper keeps upstream's own verb
(`zrcFindNearestPoly`, `zrcCalcBounds`). The Zig layer turns a handle-taking
entry point into a method on the owning wrapper (`NavMeshQuery.findPath`) and
keeps upstream's terminology for concepts (polys, contours, regions,
corridors), so upstream's documentation stays usable as this package's.

- Slices in, typed results out. Counts and capacities stay inside the wrapper;
  a caller passes `[]PolyRef` and gets `.len` back.
- `ZrcResult` becomes the error set in `src/error.zig`; a partial outcome that
  upstream reports as success stays a success with a `.partial` field.
- Every call that can fail takes the caller's `Failure`
  (`aegis.err.Context`) last, and pushes one `Frame` where it refuses: the
  `Call`, which is the C entry point's name in snake case (`zrcFindPath` is
  `.find_path`), and the `ZrcResult` it returned. A refusal the wrapper makes
  before calling C, a slice too long for an `int`, names the call it was
  about to make, with the result its error stands for; the two with no C call
  behind them, `convexVolume` and `dirForOffset`, name themselves. A call
  that only passes a refusal on pushes nothing, so one refusal is one frame.
  Four are kept, and `truncated` says a fifth arrived. Tests in
  `src/error.zig` read the modules as compiled and fail if a checked call's
  frame names any other call, if an error of the set is returned any other
  way, or if a module makes a `Failure` of its own.
- A call whose refusal cannot happen from Zig, such as the assertion
  handler's or `QueryFilter.setIncludeFlags`, takes no `Failure` and ends in
  `aegis.assert.never`.
- Optionals mean what they say: a nullable pointer parameter is `?`, a
  reference that can be "none" is `?PolyRef`, never a magic zero at the
  surface.
- Each kind of reference is its own type, `aegis.id.NonZero` over the
  header's integer: `PolyRef`, `TileRef`, `CompressedTileRef`, `ObstacleRef`,
  `AgentRef` and `PathRequestRef`. A tile reference cannot be passed where a
  polygon one is wanted, no reference is 0, and each is the same bytes as the
  header's, which `src/abi_test.zig` holds. A buffer or field the boundary
  may fill with "none" (a straight path's end corner, a search's start
  parent, a corridor that is empty) holds `Maybe(PolyRef)`, the same bytes,
  read with `get()` as `?PolyRef`.
- Flags are named option structs (`StraightPathOptions`), never bare `u32`s.
- Bytes the package parses arrive as `Image`, `aegis.input.Untrusted`: a
  navmesh, a tile, a tile's stored state, a tile-cache layer.
- A query filter is a handle, `QueryFilter`, made from a `QueryFilterDesc`.
  Its costs are checked when it is made and when one is set, and every query
  hands Detour the filter the handle holds, so no call checks a cost or
  rebuilds a filter. A sliced search, a path queue request and a crowd copy
  the filter, so it need not outlive them.
- A handle another object owns is lent as a `Borrowed` type whose handle is
  `*const`, as upstream's getter is: `Crowd.grid`, `Crowd.pathQueue`,
  `Crowd.navMeshQuery`, `PathQueue.navMeshQuery` and
  `NavMeshQuery.attachedNavMesh`. It has no `deinit` and none of the writes,
  so the crowd's own grid cannot be freed or cleared through it.
- Narrowing into the boundary is `aegis.int.cast` with the caller's refusal;
  a count the boundary reports comes back through `c.reported`, which stops in
  every build if it does not fit, where `@intCast` would be undefined in
  ReleaseFast. Every capacity a caller sets is range-checked once, in `ffi/`,
  where a C host meets the same check.
- Memory routes through the process-wide allocator seam
  (`setAllocator`/`resetAllocator`), which `tests/c_smoke.c` proves is in use
  by counting.
- The version has one home, `build.zig.zon`. The `ZRC_VERSION_*` macros in
  `ffi/zrecast.h` restate it for C consumers, and the "version reporting is
  wired up" test compares what they compiled to against the `.zon` string.
- Build options are declared once and mirrored into a Zig `options` module,
  so the wrapper cannot disagree with how the C++ was compiled.

## The bound surface

`ci/coverage/unbound_*.txt` is the record and `bay check-coverage` is the
gate; this is the same answer in prose, grouped by what a host is trying to do.
[Coverage](#coverage) has how the record is derived.

- **Bake** — the full Recast pipeline over a triangle soup: heightfield,
  filters, compact heightfield, erosion, regions (all three partitioning
  strategies), contours, polygon mesh, detail mesh.
- **Tiles** — a tile grid computed from the geometry, a per-tile bake whose empty
  tiles are a success rather than an error, each tile cooked to bytes of its own,
  and add/remove/lookup/enumerate against a live navmesh. A path crosses tile
  boundaries through the portal edges a tiled bake emits.
- **Areas and flags** — convex, box and cylinder volumes applied between erosion
  and region building, colouring the surface with area ids a query filter charges
  for; a table mapping each area id to the polygon flags a filter admits or
  refuses; and `get`/`set` for both on a live navmesh, so a door opening or a
  zone flooding is a write rather than a re-cook.
- **Off-mesh connections** — point-to-point links across ground the surface does
  not join: a jump, a ladder, a door. One-way or two-way, carrying an area, flags
  and an opaque id, supplied when a tile is built and read back from a live
  navmesh. A path through one produces a straight-path corner flagged
  `ZRC_STRAIGHTPATH_OFFMESH_CONNECTION`, which is how a game knows to play an
  animation instead of walking.
- **Tile state** — one tile's polygon areas and flags as bytes, for a save file.
  The blob is only restorable onto the tile it came from, and only at exactly the
  length that tile reports.
- **Reading a navmesh back** — the grid parameters, a tile's header, its
  polygons, links, detail sub-meshes, vertices and bounding-volume nodes, all
  copied out **by value** so a read survives the tile's removal. Ranges are
  half-open and a range outside an array is an error, never a short read, since
  a short read is indistinguishable from an empty tile.
- **Navmesh** — a `dtNavMesh` from that bake, single-tile or tiled; a single-tile
  one serialises to and from bytes, with the image validated before it is
  trusted.
- **Queries** — the whole of `dtNavMeshQuery`: nearest polygon, path,
  string-pulled corners, surface movement, walkability raycast, the polygons in
  a box (into a buffer or through a callback), the three outward searches and
  the corridor one leaves behind, wall segments and distance to the nearest
  wall, closest point and height on a single polygon, and reproducible random
  placement. Every one that fills a caller's buffer reports whether it had to
  stop short.
- **Sliced pathfinding** — the same search driven a few iterations at a time so
  a frame can spend a fixed budget on it. The filter is copied for the slice's
  lifetime, because upstream keeps only a pointer to it; a second search that
  would clear the node pool underneath a slice is refused; and finalising twice
  is refused rather than answering with a one-element path holding the null
  reference.
- **What a reference means** — validity against a filter or on its own, whether
  the last search closed it, and the salt/tile/polygon fields packed into it,
  split and rebuilt with the widths that navmesh actually uses.
- **The search's own node pool** — how full it is and what the last search
  concluded about a given polygon, which is what makes `max_nodes` tunable
  rather than guessed.
- **Geometry** — the computational geometry Detour runs on, callable directly:
  point-in-polygon, closest point on a triangle, distance to a polygon's edges,
  segment/polygon and segment/segment intersection, the two overlap tests a
  BV-tree walk needs, polygon offsetting and a reproducible random point in a
  convex polygon. Each one checks the array bound upstream leaves to the caller.
- **Vector math** — the `dtV`/`rcV` families and the scalar helpers around them,
  in Zig rather than across the boundary, and asserted **bit-identical** to the
  C over a table that includes zeros, negative zeros and denormals. Six of
  them answer differently from Zig's obvious spelling at an edge, and the tests
  pin all six.
- **Seams** — the allocator, reachable for a host's own allocations, and one
  assertion handler installed into both halves of upstream, with a way to ask
  whether this build kept the assertions that would call it.
- **Tile images** — the byte offset and length of each of a tile's eight
  arrays, so a host can parse or patch a cooked image with the layout Detour
  itself derives rather than one it guessed.
- **The Recast pipeline, stage by stage** — the same bake taken apart. Each
  stage is its own call, each intermediate container is a handle a host owns,
  and every one can be read, edited or replaced between one stage and the next:
  spans added by hand, areas painted from data no volume shape can express,
  polygon flags written before the mesh becomes a tile. A staged mesh goes into
  `zrcTileDataBuild` exactly as a baked one does, and the suite asserts the two
  produce identical bytes from identical input.
- **A build context** — the log and timer hooks Recast calls during a build,
  with the two enable flags upstream consults before it reaches one, so a host
  measuring a bake measures the phases a C++ host measures.
- **Dynamic obstacles** — a tile cache: each tile's walkable surface kept in a
  compressed layer beside the navmesh, and rebuilt whenever an obstacle over it
  appears or goes away. Cylinders, boxes and boxes rotated about y, queued and
  carved a tile per update so a frame can spend a fixed budget on it. The codec
  is the host's — none is bundled and no container format is invented — and so
  is the callback that decides each rebuilt polygon's flags, narrowed to the
  fields it is for because the tile it produces reaches the navmesh without the
  validation an added tile gets. The suite carves an obstacle into the fixture's
  only gap, watches the route close, removes it, and watches it open.
- **Drawing** — every container a bake or a load produces, rendered through a
  renderer the host supplies: the input soup shaded by slope, the heightfield,
  the compact heightfield solid, by region and by distance, the layered
  heightfield, the traced and simplified contours, the polygon and detail
  meshes, a navmesh's polygons, bounding-volume tree, portals and off-mesh
  connections, a search's node pool and closed list, and a tile cache's layers,
  contours and rebuilt mesh. Plus the shapes upstream draws them out of —
  boxes, cylinders, arcs, arrows, circles, crosses and a grid — each in a form
  that opens its own run and a form that appends to one the caller opened. The
  renderer is a Zig struct whose methods are read at compile time; a display
  list records one run of primitives and replays it into any other renderer.
- **Dumping a build** — the polygon mesh and the detail mesh as Wavefront OBJ,
  and the contour set and compact heightfield as upstream's own binary form,
  written through four hooks a host fills rather than to a file this package
  opens. Both binary forms read back into a container this package creates, so
  a failed read destroys a half-built one rather than handing it over. The
  format is upstream's struct layout copied to the stream, so it is neither
  endian- nor padding-portable and carries no length to check a count against:
  feed a read only bytes this package wrote.
- **Crowds** — many agents steering around each other and the world: a local
  re-plan per agent per frame, neighbours from a proximity grid, walls from a
  cached local boundary, a velocity chosen by an obstacle-avoidance sampler, and
  a position integrated under acceleration and speed limits. An agent is named
  by a reference carrying a serial rather than by its pool slot, so a slot
  reused after a removal cannot be driven by a stale handle — upstream has no
  identity at all and every setter it offers takes a bare index. The two
  parameter fields that index the crowd's filter and avoidance tables are
  refused at the door; unchecked, they read tens of kilobytes past those tables
  every frame. Long searches go through a path queue whose filters this package
  owns for the request's lifetime, because upstream keeps only a pointer to the
  caller's. The corridor, the local boundary, the proximity grid and the
  avoidance sampler are each usable on their own, which is what a host steering
  one character wants.

## Coverage

The suite says the code works. It says nothing about how much of Recast and
Detour a host can get at. That is answered by a generated record, one line per
public upstream name, and by a gate that re-derives the name list from the
vendored headers so a name cannot go missing quietly.

### The verdicts

Every public name in `Recast/`, `Detour/`, `DetourCrowd/`, `DetourTileCache/`
and `DebugUtils/` carries exactly one:

- **`BOUND`**: reachable through the C boundary. The evidence names the `zrc`
  symbol that carries it, and the gate checks `ffi/zrecast.h` declares it.
- **`LANGUAGE`**: C++-only surface a C boundary cannot carry: `rcVectorBase`,
  `rcIntArray`, `rcTempVector`, `rcPermVector`, `rcScopedDelete`,
  `rcScopedTimer`, `dtNodeQueue`, the `dtMath*` wrappers over libm, the
  `dtSwapEndian` overloads, `rcSqrt`, `RC_PI`. The evidence says what a C or
  Zig host does instead.
- **`ZIG`**: reimplemented on the Zig side: the `dtV`/`rcV` vector families
  and the scalar helpers around them. The evidence names the mirror in
  `src/vec.zig`, and that file's bit-identity test against linkable shims over
  upstream's own `inline` definitions is what makes a mirror true.
- **`UNDEFINED`**: a header declares it and no vendored source defines it, so
  there is no symbol for any host to call. The evidence names the header and
  line, and the gate rechecks both halves against the tree: a re-vendor that
  supplies the definition fails until the name is bound. One name carries it,
  `duDebugDrawHeightfieldLayersRegions`, which would also have nothing to draw:
  `rcHeightfieldLayer` carries no region ids.

A name with none of these is a gap, and a gap fails the gate unless it names
the tranche of work that closes it. None is open.

### The commands

```sh
bay check-coverage                    # the gate
bay coverage -- --list                # the gate, and the open gaps
bay coverage -- --record              # rewrite the record from its inputs
bay coverage -- --check-record        # the record matches its inputs
bay coverage -- --names               # every public name upstream declares
bay coverage -- --summary [AREA]      # verdicts per area, and one area's names
bay coverage -- --audit               # every line the harvester did not understand
bay coverage -- --collapsed           # symbols covering more than one declaration
```

The record is generated by `ci/coverage.zig` from the harvest, from
`ci/coverage/bindings.tsv` (the names that are answered, the only half kept by
hand) and from the rule that says which tranche closes each of the rest, so a
name cannot carry one verdict in one place and another somewhere else, and a
gap's reason is a rule that is re-read rather than a sentence that rots. The
gate refuses any verdict it cannot check: a `BOUND` line has to name a symbol
`ffi/zrecast.h` declares, and a gap has to say which tranche closes it. Every
order in the record is byte order, so it is the same file on every host.

### What the harvester counts

The harvester enumerates functions, data members, enum constants, `static
const` values and type names; keeps overloads apart by arity and, where arity
is not enough (`dtSwapEndian`'s five one-argument forms), by parameter type;
and prints what it could not parse. Counting only an identifier followed by
`(` would miss `dtNavMeshCreateParams`' must-populate fields, the `dtStatus`
detail bits that are `static const unsigned int` rather than an enum, and the
constants of the enums whose names carry no `rc` or `dt` prefix.

`DetourNavMeshQuery` is its own area rather than part of `Detour`:
`dtNavMeshQuery` is the pathfinding engine, large enough to want its own line.
`DebugUtils` is one area covering its four headers. Preprocessor macros are not
names here (`rcLikely`, the assert macros), and operator overloads carry no name
a C ABI can spell; `--summary` prints both blind spots.

### The documented numbers

No number in README.md or this file is typed in by hand without a reason.
`bay measurements` (`ci/measurements.zig`) recomputes the README's block from
the tree, the docs check fails while the committed block differs, and
`bay check-numbers` (`ci/numbers.zig`) refuses any other number in prose
unless its allowlist says why it cannot go stale. An allowance nothing uses
fails too. Adding a claim means adding its measurement.

A count is a count. Matching entry-point counts prove presence, not
correctness; the behavioural tests, the C smoke test and the mutation probes
hold that, and the coverage gate holds each verdict one name at a time. The
number check has three blind spots: a number spelled as a word, a number inside
`code`, where it is an identifier or a citation rather than a claim, and a
sentence that is wrong without any number in it.

## Validation at the boundary

Recast and Detour assert in debug and read on regardless in release. `ffi/`
checks an index, a length or a reference before upstream sees it. This is what
is checked and why.

### Geometry

Recast indexes `verts[tris[i] * 3]` with no range test of its own. An index
past the end of the vertex array is an out-of-bounds read inside the rasteriser
in every build configuration. A NaN coordinate is worse: `rcCalcBounds`
propagates it into the grid size, and the heightfield allocation is then
computed from garbage. Both are rejected at the door.

### The spare bounding-volume node

A tile reserves two bounding-volume nodes per polygon and the tree fills
`2n - 1` of them. Detour's own traversal ends one node past the tree and reads
the spare, zeroed one as a leaf naming polygon 0. A box of all zeroes overlaps
every query clamped to the tile's minimum corner, so `findNearestPoly` at a
navmesh's own corner came back with a polygon nine metres outside the
half-extents it was given. Nothing faults; the answer is simply wrong. That
node is sealed as an internal node in every image this package cooks and in
every image it loads. See [Known upstream behaviour](#known-upstream-behaviour).

### Serialised navmeshes

`dtNavMesh::init` dereferences the buffer to read the magic and version
*before* it looks at the length, which it then never compares against anything.
`addTile` derives eight array pointers from counts in that header and never
bounds-checks them either — and it writes `links[maxLinkCount - 1]` before any
loop guard, so a zero there is an out-of-bounds *write*.

`zrcNavMeshValidate` closes that: it copies the header out (so a misaligned
caller buffer is not a struct read either), sanity-checks every count, and
requires the size implied by those counts to equal the buffer length exactly.

Header checks alone are not enough. A `dtPoly::vertCount` of `200` — one byte,
in an otherwise untouched image — passes every count check, loads without
complaint, and then overruns a fixed `float verts[DT_VERTS_PER_POLYGON*3]`
stack array in four separate Detour functions, with coordinates read from the
image. So the validator makes a second pass and bounds everything the image can
point at:

- **Polygon corner counts and corner indices**, which is the case above.
- **Neighbour indices.** `connectIntLinks` turns each into a link reference
  with no range test, and every traversal query resolves that reference with
  `getTileAndPolyByRefUnsafe` — unsafe as named. The safe accessor guards the
  *caller's* references; the unsafe one is used on references the tile made for
  itself, so this is the same overrun reached the long way round.
- **Vertex and detail-vertex coordinates**, which must be finite.
  `closestPointOnDetailEdges` keeps its nearest edge with `if (d < dmin)` and
  lerps through the result unconditionally, so one NaN — against which every
  comparison is false — leaves it dereferencing null.
- **Detail sub-mesh bases, extents and triangle corners**, plus a triangle
  count of at least one and at least one boundary edge flag per sub-mesh: both
  are further ways that same nearest-edge pointer stays null.
- **BV-tree polygon indices and escape jumps.**
- **Off-mesh connections.** `baseOffMeshLinks` indexes `tile->polys` by a
  connection's polygon field and writes through the vertex it finds there, with
  no bound on either. Each connection must own exactly the polygon the builder
  assigns it, name finite endpoints and a non-negative radius, and carry only
  the side codes and flags Detour itself emits.

None of that rejects anything a real bake produces; the round-trip test is the
control. What it buys is memory safety, not trustworthiness: an entirely
in-bounds image can still describe a degenerate or useless navmesh.

Every truncation of a valid navmesh image — every prefix of the test fixture's,
byte by byte — is verified to be rejected by both the validator and the loader,
while the whole image still loads. So is each targeted header and interior
corruption in the suite, every one aimed at a specific field Detour would
otherwise trust.

Most of the interior checks came from a mutation fuzzer: random bytes anywhere
in the image, then validate, deserialise and query. It found three holes that
reading the validator had not suggested, including one that had been reasoned
about and gotten wrong. That fuzzer is a shakedown property in the suite now,
one of four (see [Untrusted bytes](#untrusted-bytes)), asserting that a
meaningful share of its cases get past validation and into the queries, so it
cannot decay into an exerciser of the magic check. The development campaign
behind the checks was larger; the in-suite run is the regression net.

Neither library is hardened against arbitrary hostile input, and zrecast cannot
make them so from the outside. The fuzzer found three holes after the first
round of hand-written checks, so a fourth is possible. Load a navmesh from an
untrusted source behind a length-and-signature container, and validate that
first.

### The ABI guard

The Zig side hand-writes `src/c.zig` mirroring `zrecast.h`, and neither
compiler checks that the two still agree: a field reordered on one side and
not the other is silent corruption. Two checks close that.

`src/abi_test.zig` holds `src/c.zig` to the header at compile time. The test
module alone translates `ffi/zrecast.h` as `zrecast_h`, and the check pairs
every declaration by name: each function with the C function of the same name,
by parameter count and by each parameter's size, alignment and scalar kind;
each type `X` with `ZrcX`, a struct field by field and by name, a callback
signature-deep; each enumerator with its C constant; each mirrored constant
`x` with `ZRC_X`. A declaration it does not know how to compare is a compile
error, and the reverse sweep requires every function the header declares to be
bound. Each comparison is shown to name a planted drift. It also counts the two
caller shapes Zig 0.16.0 was measured to miscompile in zmeshopt, and the
boundary has neither.

`zrcAbiLayout()` reports what the C++ actually compiled to, and a test asserts
every size and offset against the Zig declarations.

Every offset, looked up **by name**. Sampling a few is not a guard:
`ZrcBakeConfig` is ten consecutive floats followed by seven more 4-byte fields,
so swapping any two leaves the size, the alignment and the whole *sequence* of
offsets identical — each side renumbers together — while every bake uses the
wrong agent dimensions. Asking where the field called X lives on each side is
the only comparison that sees it, and each of those swaps is verified to fail
the test.

In the other direction, `static_assert`s in `ffi/zrecast_abi.cpp` fail the
build if a re-vendored upstream changes:

- a constant this package mirrors into its own header (`DT_MAX_AREAS`,
  `DT_VERTS_PER_POLYGON`, `RC_WALKABLE_AREA`, the allocator hint enumerators);
- the size of any struct that appears in a serialised navmesh, because
  `zrc::ValidateNavMeshImage` reproduces Detour's own pointer arithmetic over
  exactly those six types in order to locate and bounds-check every array;
- the serialised format version, because every previously written image becomes
  unreadable;
- the polygon reference type, which the query layer passes through with no
  cast.

### Build hygiene

- Source lists are explicit, never globs — a re-vendor cannot silently change
  what compiles.
- No `-fno-access-control`. The FFI layer uses only public upstream API.
- The C undefined-behaviour sanitizer is not switched off to make anything
  pass. A package build leaves it off unless `-Dsanitize_c` asks for it, and
  the suite runs one of its three arms against the C++ compiled with it, in
  every optimize mode; see [Known upstream behaviour](#known-upstream-behaviour).
- `-fno-exceptions` / `-fno-rtti` everywhere except the MSVC ABI, where
  disabling them through Clang flags breaks the Microsoft standard library
  headers. Neither Recast nor Detour contains a single `throw`, `try`,
  `dynamic_cast` or `typeid`, so nothing is lost.
- One translation unit per concern on both sides of the boundary.

## Cook determinism

A navmesh is a shipped asset. If two machines cooking the same geometry produce
different bytes, the asset cannot be built once and trusted, and a bug that
reproduces on one of them may not reproduce on the other. What this package
guarantees is scoped to what it measures, in three rings.

- **One target, cooked twice, is the same bytes.** Asserted by the suite
  (untiled, tiled and area-authored cooks, hashed and compared) on every target
  that runs it.
- **Two targets sharing a C library are the same bytes.** `bay determinism`
  cooks on both architectures of musl and glibc and fails if a pair sharing a
  library disagrees. With contraction pinned and no libm in the bake,
  architecture is not allowed to move a byte. On Zig 0.17 this ring is NOT
  ESTABLISHED: the comparison last ran for 0.1.0 (2026-09-03), on Zig 0.16,
  and has not run since the move, since it needs Linux containers of both
  architectures.
- **Two different C libraries are compared, not promised.** Upstream sorts BV
  items, contour holes and diagonals with `qsort`, ties fall to the
  implementation, and any order the ties land in is a valid navmesh;
  [Known upstream behaviour](#known-upstream-behaviour) records the five sites.
  `bay determinism` prints the digests side by side, so a divergence is seen,
  named and attributed rather than asserted away. Ship cooks from one platform,
  or key the asset cache by cook platform.

### What the first two rings rest on

Each of these was a real gap rather than a precaution.

- **`-ffp-contract=off`, for every target.** Clang fuses a multiply and an add
  into a single instruction wherever the target has one, and the fused form
  rounds once where the separate form rounds twice. AArch64 mandates FMA; the
  x86-64 baseline has none. The same source line in Recast therefore produced
  different floats on the two architectures. With contraction off, every
  remaining operation in the bake path is one IEEE-754 specifies exactly: add,
  subtract, multiply, divide, `sqrtf`, `floorf`, `ceilf`, `fabsf`. The vector
  math's bit-identity test rests on it too: `tests/reference.cpp` is compiled
  with it, so the Zig arithmetic is compared against the header's own, not a
  contracted form of it.
- **`zrc::CosDegrees` instead of `cosf`.** The slope limit becomes a cosine
  through `rcMarkWalkableTriangles`, and that was the only transcendental the
  bake reached. The C standard does not require a correctly rounded cosine, so
  the threshold, which decides per triangle whether a surface is navigable,
  depended on the host's C library. It is now a polynomial in double over the
  bounded range a slope limit can occupy, and the triangles are marked with
  upstream's own vector helpers. No other transcendental occurs anywhere in
  `Recast/Source` or `Detour/Source`; `DetourMath.h`'s wrappers are query-path
  only. The staged pipeline reaches the same substitution:
  `zrcMarkWalkableTriangles` and `zrcClearUnwalkableTriangles` are the
  replacements rather than upstream's functions, so a mesh a host assembles a
  stage at a time is the same bytes on every platform a baked one is.
- **The Zig vector math is asserted bit-identical to the C.** `src/vec.zig`
  reimplements the `dtV`/`rcV` families rather than calling across the boundary
  to add three floats, which only holds if the two agree everywhere, so a test
  compares them bit for bit over a table of zeros, negative zeros and
  denormals, against linkable shims over upstream's own `inline` definitions.
  Six of the scalar helpers answer differently from Zig's obvious spelling:
  `dtMin(3, NaN)` and `dtMax(3, NaN)` are `NaN` where `@min` and `@max` give
  `3`, `dtAbs(-0.0)` keeps its sign bit, `dtClamp(NaN, 0, 1)` passes the NaN
  through, and `dtNextPow2(0)` and `dtIlog2(0)` are both `0`. Each is written
  out as upstream's own expression and pinned by a test at exactly that value.

### Running it

```sh
bay determinism            # every target this host can run
bay determinism -- --strict  # and a target this host could not run is a failure
```

The development build hands `ci/determinism.zig` the suite built for each
target, and it runs each where this host can: natively, or in a container
(`docker`, with the image's platform under emulation). It reports three
outcomes and never two: a target that ran, a target that failed, and a target
this host could not run at all. The last is counted and named in the summary
rather than dropped, and `--strict` makes it a failure. It is a development
step, not part of `bay test`: the gate holds the first ring on every target CI
runs, and pinning one C library's `qsort` tie order would be a check no
consumer of upstream could pass.

### Where it ends

The byte-for-byte guarantee covers baked assets. It does not extend to a
frame's steering, which reaches the host's `cosf` and `sinf` in the adaptive
velocity sampler (`DetourObstacleAvoidance.cpp:499-500`, `:535-536`). A host
that needs a reproducible replay should record its inputs. `ffi/zrecast.h` says
the same where the crowd is declared.

## The vendored upstream

`libs/recastnavigation` is a pinned copy of upstream **recastnavigation**,
verbatim — byte for byte, nothing applied on top. Behaviour this package needs
that upstream lacks lives in `ffi/`, on our side of the boundary.

| | |
|---|---|
| Source | <https://github.com/recastnavigation/recastnavigation> |
| Version | v1.6.0 |
| Commit | `6dc1667f580357e8a2154c28b7867bea7e8ad3a7` |
| Date | 2023-05-21 |
| License | zlib (`libs/recastnavigation/License.txt`) |

v1.6.0 was upstream's most recent tag when last checked, on 2026-10-25 with
`git ls-remote --tags`; [Re-vendoring](#re-vendoring) is what a newer one
calls for.

### What was taken, and what was left behind

Taken: `Recast/`, `Detour/`, `DetourCrowd/`, `DetourTileCache/`, `DebugUtils/`,
`License.txt`, `README.md`, `CHANGELOG.md`.

| Excluded | Size | Reason |
|---|---|---|
| `RecastDemo/` | 1.3 MB | The SDL2/OpenGL sample application. Carries third-party code of its own (`Contrib/fastlz`, `Contrib/stb_truetype.h`, and an imgui fork inlined into `Include/` and `Source/`) and sample meshes under `Bin/Meshes` whose provenance is not stated per file. The tests here generate their geometry instead — see `tests/fixture.cpp`. |
| `Tests/` | 855 KB | Upstream's own suite, plus the copy of Catch2 in `Tests/Contrib/catch2` that it needs. |
| `Docs/` | 584 KB | Doxygen inputs, a CSS theme and images. |
| `CMakeLists.txt` (all) | — | Superseded by `build.zig`. |

All five of the taken directories are compiled. Which translation units do so
is decided explicitly in `build.zig` (`recast_sources`, `detour_sources`,
`detour_tile_cache_sources`, `detour_crowd_sources`, `debug_utils_sources`),
never by a directory glob.

### Compile-time configuration

Upstream has two macros that change its ABI. Neither is defined here, and both
are guarded:

| Macro | State | Guard |
|---|---|---|
| `DT_POLYREF64` | not defined — `dtPolyRef` is 32-bit | `static_assert` in `ffi/zrecast_abi.cpp` that `ZrcPolyRef` and `dtPolyRef` are the same type. `ffi/zrecast_query.cpp` passes reference arrays straight through with no cast, so anything else is also a compile error at every call site. |
| `DT_VIRTUAL_QUERYFILTER` | not defined — `dtQueryFilter` has no vtable | zrecast never subclasses it; a `ZrcQueryFilter` holds one, built once from a `ZrcQueryFilterDesc`. |

### The serialised navmesh format

`zrcNavMeshSerialize` writes Detour's own tile image. At the pinned version its
format version (`DT_NAVMESH_VERSION`) is **7**, which `zrcNavMeshDataVersion`
reports and a `static_assert` pins — a re-vendor that bumps it fails the build,
which is the point, because every previously written image becomes unreadable.

The image is independent of pointer width (nothing in it is a pointer) but it is
**native-endian** and tied to the exact layout of `dtMeshHeader`, `dtPoly`,
`dtLink`, `dtPolyDetail`, `dtBVNode` and `dtOffMeshConnection`. All six sizes are
`static_assert`ed, because `zrc::ValidateNavMeshImage` reproduces Detour's own pointer
arithmetic over them in order to bounds-check an image before Detour walks it.

Treat an image as a build artefact keyed to this library's version, not as an
interchange format.


### Known upstream behaviour

Recorded so a future re-vendor can check whether any of it has been fixed, and
so the workarounds are not mistaken for arbitrary defensiveness. Nothing in
this section is patched: each is handled at the boundary, in `ffi/`, or is
documented rather than defended where there is no boundary to defend at.

| Upstream | What zrecast does |
|---|---|
| `qsort`, five comparators in `dtCreateNavMeshData`, `mergeRegionHoles` and the contour diagonal search: each returns 0 on a tie and `qsort` is not stable, so two C libraries may cook different, equally valid bytes from one input — BV items sorted per axis and serialised into the tile (`DetourNavMeshBuilder.cpp:145-155`, comparators `:40-72`), holes by leftmost vertex (`RecastContour.cpp:727`), diagonal candidates by squared distance (`:773`). | Documented, not defended: the sorts run inside upstream's build functions behind no seam, and rewriting them in the vendored tree is the local divergence `bay check-vendor` refuses; the determinism claim is scoped to match, below the table. |
| `dtCrowd::removeAgent` clears only the agent's `active` flag (`DetourCrowd.cpp:572-578`) while `dtCrowd::update` walks the parallel `m_agentAnims` by raw slot (`DetourCrowd.cpp:1411-1447`), so an agent removed mid off-mesh crossing leaves a live animation that overwrites the position of whoever takes the slot next, a silently wrong result. | Worked around: `zrcCrowdRemoveAgent` retires the reference at once but, for an agent in `DT_CROWDAGENT_STATE_OFFMESH`, calls `resetMoveTarget` and keeps the slot until `zrcCrowdUpdate` calls upstream's `removeAgent` after the crossing ends, a busy slot `zrcCrowdAddAgent` documents. |
| `dtProximityGrid`'s `hashPos2` multiplies each cell coordinate by a large prime in `int` (`DetourProximityGrid.cpp:43-46`), so a cell coordinate of magnitude above 29 is signed overflow, which a world wider than roughly a hundred metres reaches at `dtCrowd`'s cell size of `maxAgentRadius * 3`. | Documented, not defended: `dtCrowd::update` calls its private grid with positions the binding never sees, the standalone grid entry points do not bound it either, and `ZRC_PROXIMITY_GRID_SAFE_CELL` in `ffi/zrecast.h` names the limit. |
| `dtProximityGrid::addItem` floors each bound into an `int` unchecked and loops between them (`DetourProximityGrid.cpp:104-112`): a non-finite bound is an undefined conversion, a box spanning billions of cells a loop that does not end, and a cell coordinate is stored in a `short`, so an item outside its range is filed and never found. | Refused at the boundary: `zrcProximityGridAddItem` and `zrcProximityGridQueryItems` check the box (`zrc::CheckProximityBox`) and return `ZRC_ERR_INVALID_ARGUMENT`. |
| `dtMergeCorridorStartMoved` and `dtMergeCorridorStartShortcut` subtract a merge width from the capacity in a signed `int` and pass the difference to `memmove` as a length (`DetourPathCorridor.cpp:60-63`, `:146-149`), so a visited count wider than the path's capacity is a memory corruption; `dtMergeCorridorEndMoved` guards the same bound only with a `dtAssert` (`:103`) that `NDEBUG` compiles away. | Refused at the boundary: the three `zrcMergeCorridor*` calls require `path_count` and `visited_count` in `[0, max_path]` and return `ZRC_ERR_INVALID_ARGUMENT` otherwise. |
| `dtNavMesh::removeTile` reads `tile->header->x` (`DetourNavMesh.cpp:1253`) after `getTileByRef` has checked only slot index and salt, and a fresh navmesh gives every slot salt 1, so a reference to an unused slot is a null dereference. | Worked around: every `ZrcTileRef` resolves through a helper that requires a non-null header before `zrcNavMeshRemoveTile` or `zrcNavMeshTileBounds` proceeds. |
| `dtNavMesh::getNeighbourTilesAt` steps with a bare `nx++` / `nx--` / `ny++` / `ny--` (`DetourNavMesh.cpp:1078-1089`), so a tile header carrying `INT_MIN` or `INT_MAX` in a grid coordinate is signed overflow on the first neighbour lookup. | Refused at the boundary: tile-image validation bounds both coordinates to `ZRC_MAX_TILE_COORD`. |
| `dtNavMesh::init` dereferences `(dtMeshHeader*)data` for magic and version, and `addTile` derives eight array pointers from its counts, without either comparing `dataSize` to anything, so a short buffer or a header describing more data than it carries is an out-of-bounds read in every build configuration. | Refused at the boundary: `zrcNavMeshValidate` copies the header out (so a misaligned buffer is not a struct read), sanity-checks every count, and requires the size the counts imply to equal the buffer length exactly. |
| `dtNavMesh::addTile` with `maxLinkCount == 0` executes `tile->links[header->maxLinkCount-1].next = DT_NULL_LINK;` before any loop guard, an out-of-bounds write one element before the array. | Refused at the boundary by `zrcNavMeshValidate`. |
| `dtPoly::vertCount` is the count `getPolyHeight` (`DetourNavMesh.cpp:687`), `closestPointOnPoly`, `moveAlongSurface` and `raycast` (`DetourNavMeshQuery.cpp:549`, `:2091`, `:2490`) copy into a fixed `float verts[DT_VERTS_PER_POLYGON*3]` stack array, so a value of `200` is an out-of-bounds write over the saved return address. | Refused at the boundary: `zrcNavMeshValidate` checks it before Detour sees the image, and rejects nothing `dtCreateNavMeshData` produces. |
| `dtPoly::neis` becomes a link reference `base \| (neis[j] - 1)` in `connectIntLinks` with no range test (`DetourNavMesh.cpp:549`), which traversal resolves through `getTileAndPolyByRefUnsafe` into `tile->polys` unchecked, an out-of-bounds read. | Refused at the boundary: `zrcNavMeshValidate` checks it before Detour sees the image. |
| `closestPointOnDetailEdges` keeps its nearest edge with `if (d < dmin)` and calls `dtVlerp` on it unconditionally (`DetourNavMesh.cpp:673`), so one NaN vertex coordinate leaves the edge pointers null, a null dereference. | Refused at the boundary: `zrcNavMeshValidate` checks vertex coordinates before Detour sees the image. |
| `closestPointOnDetailEdges` reaches the same null dereference from detail edge flags and the triangle count: the `onlyBoundary` instantiation skips every triangle with no boundary bit, and a sub-mesh with no triangles never enters the loop. | Refused at the boundary: `zrcNavMeshValidate` checks both before Detour sees the image. |
| `dtBVNode::i` is used by `queryPolygonsInTile` to index `tile->polys` directly (`DetourNavMeshQuery.cpp:776`), an out-of-bounds read. | Refused at the boundary: `zrcNavMeshValidate` checks it before Detour sees the image. |
| `dtNavMeshQuery::init` sizes the `dtNodePool` hash with `dtNextPow2(maxNodes / 4)`, and `dtNextPow2(0)` wraps back to 0 (`DetourCommon.h:439`), so a pool below four nodes gets a zero-byte table that passes its own `dtAssert`, and the first search reads and writes out of bounds. | Refused at the boundary: `max_nodes >= 4` is required, the smallest value for which the table exists. |
| `rasterizeTri` converts `ceilf(spanMax * inverseCellHeight)` to `int` before `rcClamp(..., RC_SPAN_MAX_HEIGHT)` (`RecastRasterization.cpp:445-446`), so tall geometry over a fine `cell_height` is an out-of-range float-to-int conversion, and past that height Recast silently flattens everything into the topmost span. | Refused at the boundary: `ffi/zrecast_bake.cpp` bounds the vertical extent to `RC_SPAN_MAX_HEIGHT` voxels, all a 13-bit span extent can address. |
| `simplifyContour` compares `dx*dx + dz*dz` against `maxEdgeLen*maxEdgeLen` in `int` (`RecastContour.cpp:399`), so the largest defined value is 32767, since `2 * 32767^2` is 2147352578 and `2 * 32768^2` is signed overflow. | Refused at the boundary: `ffi/zrecast_bake.cpp` holds both the per-axis grid dimension and `maxEdgeLen` to 32767. |
| `rcPolyMeshDetail` has no destructor, unlike the other four Recast containers, and frees its three buffers only in `rcFreePolyMeshDetail`, so a generic `~T(); rcFree(p)` helper leaks three allocations per detail mesh, as the C smoke test's balanced-allocation assertion caught. | Worked around: `zrc::RcFree` is an overload set that defers to upstream's own free function for each type. |
| `rcNew` is `new (rcAlloc(...)) T()` with no check, so a failed allocation is a placement-new at address zero, a null dereference `rcAllocHeightfield` and friends inherit. | Worked around: a checked `zrc::RcNew` in `ffi/zrecast_internal.h` replaces it, which makes the `ZRC_ERR_OUT_OF_MEMORY` paths reachable; `dtAllocNavMesh` and `dtAllocNavMeshQuery` return null on failure and are used as they are. |
| `rcVectorBase::push_back` placement-news through the null `allocate_and_copy` returns when `rcAlloc` fails (`RecastAlloc.h:214`), so a bake under memory pressure is a null dereference, not an error. | Not worked around, and cannot be from outside; the bake is outside the allocation-failure sweep test, which says so where it skips it. |
| `createBVTree` allocates its item array and indexes it unchecked (`DetourNavMeshBuilder.cpp:175`), so `dtCreateNavMeshData` is a null dereference on allocation failure. | Not worked around, and cannot be from outside; it is part of the bake, outside the allocation-failure sweep. |
| `dtNodePool` and `dtNodeQueue` check their allocations with `dtAssert` and then `memset` them (`DetourNode.cpp:64`, `:163`), a null `memset` under `NDEBUG` and an abort with asserts on, so building a query object does not survive allocation failure. | Not worked around, and cannot be from outside; the query object is outside the allocation-failure sweep. |
| `~dtNavMesh` walks `m_tiles` for `m_maxTiles` entries (`DetourNavMesh.cpp:211`), which `dtNavMesh::init` sets before allocating the array and `memset`s only after a second allocation succeeds, so after an out-of-memory `init` the destructor dereferences null or frees garbage. | Worked around, narrowly: `FreeFailedNavMesh` in `ffi/zrecast_navmesh.cpp` releases the storage without running the destructor when `init` reports `DT_OUT_OF_MEMORY`, leaving at worst one leaked `dtMeshTile` array, the one leak the `a failure at any allocation zrecast owns` sweep permits. |
| `rcContext` reports a stage failure as a bare `false` and sends the reason to a virtual log callback with no recording default, so a caller without a subclass learns only that one of eleven stages failed. | Worked around: `ffi/zrecast_bake.cpp` installs a context that captures the messages into the caller-supplied `ZrcBakeLog` buffer. |
| `dtNavMesh::calcTileLoc` casts `floorf((pos[0] - m_orig[0]) / m_tileWidth)` and its z twin to `int` unchecked (`DetourNavMesh.cpp:1063-1067`), so a far position or a degenerate tile size is an out-of-range float-to-int conversion. | Refused at the boundary: the division is done in `double` and a position whose tile an `int` cannot name is refused, as are a non-finite position and a NULL `pos`. |
| `baseOffMeshLinks` indexes `tile->polys[con->poly]` with no comparison against `polyCount` and writes through `tile->verts[poly->verts[0] * 3]` (`DetourNavMesh.cpp:571`, `:584-585`), as does `connectExtOffMeshLinks` (`DetourNavMesh.cpp:468`, `:485-486`), so an `unsigned short` from tile bytes is an out-of-bounds write. | Refused at the boundary: image validation requires `con[i].poly == offMeshBase + i` exactly, as the builder writes them (`DetourNavMeshBuilder.cpp:660-662`), which covers the bound, the order, and two connections claiming one polygon. |
| `getOffMeshConnectionByRef` computes the unsigned `ip - tile->header->offMeshBase` (`DetourNavMesh.cpp:1521`) under only a `dtAssert` that `DetourAssert.h:25-29` compiles out under `NDEBUG`, so a polygon below `offMeshBase` typed `DT_POLYTYPE_OFFMESH_CONNECTION` yields `&tile->offMeshCons[huge]`, an out-of-bounds read, and `getOffMeshConnectionPolyEndPoints` reads `tile->verts[poly->verts[idx]*3]` unbounded (`:1495-1496`). | Refused at the boundary: image validation types a polygon off-mesh iff its index lies in `[offMeshBase, offMeshBase + offMeshConCount)`, accepts neither undefined type value, and bounds the two endpoint vertex indices. |
| `queryPolygonsInTile` skips off-mesh polygons only in its no-tree branch (`DetourNavMesh.cpp:869-871`) and returns any overlapping leaf in its BV branch (`:808-860`), so a doctored tree can land a connection resolved by `baseOffMeshLinks` or `connectExtOffMeshLinks` on another connection's polygon, a silently wrong result. | Worked around: a BV leaf is bounded by `offMeshBase` rather than `polyCount`, since `createBVTree` only emits ground leaves (`DetourNavMeshBuilder.cpp:200`). |
| `restoreTileState` sizes its `dtPolyState` array from the live tile's `polyCount`, which the `dtTileState` blob does not carry (`DetourNavMesh.cpp:1363-1368`), and accepts any buffer at least that large (`:1422-1450`), so a blob from a differently shaped earlier tile is silently accepted. | Refused at the boundary: the caller's length must equal `getTileStateSize` exactly; a short buffer mirrors upstream's asymmetry, `DT_BUFFER_TOO_SMALL` from store and `DT_INVALID_PARAM` from restore. |
| `dtPoly::setArea` masks with `& 0x3f` (`DetourNavMesh.h:178`), and `dtNavMesh::setPolyArea`, `rcSpan::area` (`Recast.h:298`) and `rcMarkBoxArea` with its two siblings take any `unsigned char` unchecked, so area `64` silently becomes area `0`, unwalkable. | Refused at the boundary: `ValidateAreaAuthoring` and `zrcNavMeshSetPolyArea` refuse an id outside `[0, ZRC_MAX_AREAS)`. |
| `dtCreateNavMeshData` reserves `polyCount * 2` BV nodes (`DetourNavMeshBuilder.cpp:507`) where `createBVTree` fills `2 * polyCount - 1`, and `queryPolygonsInTile` walks to `&bvTree[bvNodeCount]` (`DetourNavMeshQuery.cpp:744`), so the zeroed spare node is a leaf naming polygon 0 that matches any query clamped onto the tile's minimum corner (`:750-757`), a silently wrong result measured on the fixture as a polygon nine metres away. | Worked around: `zrc::SealBvSentinel` marks that node internal on every tile this package cooks and on the private copy taken when a tile is added or a navmesh deserialised, at one `int` per tile of the cooked image. |
| `dtAlign4` is `(x + 3) & ~3` on a signed `int` (`DetourCommon.h:463`), so any `x` above `INT_MAX - 3` is signed overflow, unreached inside Detour but reachable by a host calling the public function. | Worked around in the Zig port: `src/vec.zig` uses a wrapping add, defined over the whole range; the C is left as it is, and the parity test stops comparing against it at `INT_MAX - 3`. |
| `dtNodePool::getNodeAtIdx` tests only `if (!idx)` (`DetourNode.h:68-72`), so an index between the node count and the capacity returns memory no search has written. | Refused at the boundary: `zrcQueryNodeAt` bounds against the count the pool reports, so an index past it is `ZRC_ERR_NOT_FOUND`. |
| `findNearestPoly` writes `isOverPoly` only inside the branch that writes `nearestPt` (`DetourNavMeshQuery.cpp:722-725`), so a caller declining the point is told false whatever the answer. | Worked around: `zrcFindNearestPoly` passes scratch of its own when the caller declines the point. |
| `findRandomPointAroundCircle` bounds its search by `maxRadius` and then places the point anywhere in the polygon it settled on (`DetourNavMeshQuery.cpp:478-492`), so the point can lie outside the radius. | Documented, not defended: stated in the header. |
| `initSlicedFindPath` keeps the caller's filter as `m_query.filter = filter` (`DetourNavMeshQuery.cpp:1230-1233`) for every later `update` and `finalize`, so a filter destroyed after `init` leaves a dangling pointer. | Worked around: `ZrcNavMeshQuery` holds its own `dtQueryFilter`, copied from the caller's, for the life of the slice. |
| `dtNavMeshQuery`'s class comment promises const members do not affect a sliced search (`DetourNavMeshQuery.cpp:133-135`), yet `findPath` (`:1003`), `findPolysAroundCircle` (`:2742`), `findPolysAroundShape` (`:2919`), `findRandomPointAroundCircle` (`:340`) and `findDistanceToWall` (`:3487`) clear the shared node pool, which leaves node storage in place (`DetourNode.cpp:83-87`), so finalising walks another search's chain and returns plausible, wrong references. | Refused at the boundary: those five return `ZRC_ERR_SEARCH_IN_PROGRESS` while a slice is in flight; `raycast`, `moveAlongSurface`, `findLocalNeighbourhood` and `findRandomPoint` are not refused, since they do not touch that pool. |
| `finalizeSlicedFindPath` and its partial twin zero the query state on exit (`DetourNavMeshQuery.cpp:1579`) and guard re-entry with `dtStatusFailed(m_query.status)`, false for that zero, so a second call returns `DT_SUCCESS` with a one-element path holding the null reference (`:1516-1520`). | Refused at the boundary: with the in-flight flag cleared, a second finalise is `ZRC_ERR_NO_SEARCH`. |
| `findLocalNeighbourhood` writes `resultRef[n]` unguarded (`DetourNavMeshQuery.cpp:3135`, `:3252`) where its siblings test `if (resultRef)` (`:2784`, `:2964`), so a caller asking only for a count gets a null write. | Refused at the boundary: `zrcFindLocalNeighbourhood` requires the array. |
| `findDistanceToWall` with no wall in range keeps the caller's `hitPos`, builds the normal from it and returns `DT_SUCCESS` (`DetourNavMeshQuery.cpp:3648-3652`), and `dtVnormalize` has no zero-length guard, so an agent exactly on a wall gets a NaN normal. | Worked around: both outputs are seeded, nothing-in-range is reported through its own flag, and a non-finite normal is zeroed. |
| `dtNavMeshQuery::isValidPolyRef` alone does not null-check its filter (`:3663`), where `findPath` (`:991`), `findPolysAroundCircle` (`:2737`), `findPolysAroundShape` (`:2910`), `findLocalNeighbourhood` (`:3108`), `findDistanceToWall` (`:3482`) and `getPolyWallSegments` (`:3330`) all do, so a NULL filter is a null dereference. | Worked around: a NULL filter is answered by `dtNavMesh::isValidPolyRef` instead, whether the reference names a live polygon at all. |
| `dtRaycastHit::hitEdgeIndex` is written by `raycast` once per polygon step (`DetourNavMeshQuery.cpp:2534`) but never on the early return when the first polygon's test fails (`:2527-2532`), so the caller reads back an uninitialised field. | Worked around: seeded to -1 before the call. |
| `dtNavMesh::getTilesAt` appends only `if (n < maxTiles)` with no overflow signal (`DetourNavMesh.cpp:1133`) and `queryPolygons` passes `MAX_NEIS = 32` (`DetourNavMeshQuery.cpp:944, 951`), so a column more than 32 layers deep is silently dropped from every query that touches it. | Documented, not defended: the capacity is upstream's own and a layered navmesh that deep is outside what this version builds; stated in the header. |
| `encodePolyId` shifts and ors without checking salt, tile index or polygon index against the widths `dtNavMesh::init` computed (`DetourNavMesh.h`), so a field one bit too wide silently names something else. | Refused at the boundary: `zrcEncodePolyRef` recomputes the widths from the navmesh's own parameters and refuses. |
| `dtRandomPointInConvexPoly` with `npts == 1` leaves `tri` at 0 and reads `pts[(tri - 1) * 3]` (`DetourCommon.cpp:337-345`), three floats before the caller's array, an out-of-bounds read. | Refused at the boundary: at least three vertices are required, the smallest polygon the fan has a defined answer for. |
| `dtCalcPolyCenter` takes no vertex count, trusting every `idx` value as an offset into `verts`, and divides by `nidx` unchecked (`DetourCommon.cpp:186-202`), so a bad index is an out-of-bounds read and an empty polygon is NaN. | Refused at the boundary: `zrcPolyCenter` takes the vertex count as a parameter, bounds every index against it, and requires at least one index. |
| `dtOverlapPolyPoly2D`'s helper reads `&poly[0]` before any loop (`DetourCommon.cpp:275`), so `npolya == 0` with a non-empty second polygon reads three floats from an empty array. | Refused at the boundary: both counts are checked. |
| `rcOffsetPoly` bounds with `n + 2 >= maxOutVerts` and `n + 1 >= maxOutVerts` (`RecastArea.cpp:496, 510`), so the last slot is never written and a buffer sized exactly to the result returns 0, the failure value. | Documented, not defended: a fix would change which inputs succeed; stated in the header, and the zero return is named `ZRC_ERR_BUFFER_TOO_SMALL`. |
| `dtAssertFailFunc` and `rcAssertFailFunc`, with their `SetCustom` and `GetCustom` functions, exist only inside the `#else` of `#ifdef NDEBUG` in `DetourAssert.h` and `RecastAssert.h`, so a release build has no assertion hook and nothing says which build a host holds. | Worked around: `zrcSetAssertHandler` records the handler in either build, and `zrcAssertsEnabled` says which build it is. |
| `rcBuildPolyMesh` succeeds with `npolys == 0`, giving a navmesh that answers every query with nowhere to go. | Refused at the boundary: the bake returns `ZRC_ERR_EMPTY_RESULT` and writes an explanation to the bake log. |
| `rcBuildCompactHeightfield`, `rcBuildContours`, `rcBuildPolyMesh` and `rcBuildPolyMeshDetail` assign over their destination's members with no test, and `rcBuildHeightfieldLayers`, `rcCopyPolyMesh` and `rcMergePolyMeshes` only with an `rcAssert` (`RecastMesh.cpp:1491-1495`) that `RecastAssert.h:25` compiles out under `NDEBUG`, so a reused container leaks in a release build. | Refused at the boundary: every staged entry point tests the destination and returns `ZRC_ERR_ALREADY_BUILT` in every build configuration. |
| `rcErodeWalkableArea` takes its threshold as `(unsigned char)(radius*2)` (`RecastArea.cpp:210`), so radius 128 erodes nothing and 129 erodes as 1 would, and it saturates distances at 255 (`RecastArea.cpp:111`), so nothing above 127 is honoured. | Refused at the boundary: the radius is bounded to 127 in both the bake and the staged entry point. |
| `rcBuildRegions`' watershed reads `chf.dist` through `floodRegion`, `expandRegions` and `sortCellsByLevel` (`RecastRegion.cpp:339, 381, 494`) unchecked, and it is null until `rcBuildDistanceField` runs, a null dereference; `rcBuildRegionsMonotone` and `rcBuildLayerRegions` do not read it. | Refused at the boundary: the staged entry point refuses the watershed strategy on a field whose distance array has not been built. |
| `rcCreateHeightfield` stores `sizeX` and `sizeZ` unchecked and allocates `sizeof(rcSpan*) * width * height` (`Recast.cpp:306-325`), so two negative sizes allocate a wrong positive size and every later `spans[x + z * width]` is out of bounds, and two large positive sizes overflow the `int` product before the allocation is attempted. | Refused at the boundary: `zrcHeightfieldCreate` bounds both sizes and their product, and the vertical extent to what a span's 13-bit height can address. |
| `rcAddSpan`'s helper dereferences `hf.spans[x + z * hf.width]` with no test against the field's extent (`RecastRasterization.cpp:122-124`), an out-of-bounds access unreachable from a bake and reachable from a host adding spans by hand. | Refused at the boundary: both coordinates are checked. |
| `rcGetDirForOffset` indexes a five-entry table with no bound of its own, and a diagonal offset with each argument inside [-1, 1] still derives an index outside it, an out-of-bounds read. | Refused at the boundary: `dirForOffset` bounds both arguments and the derived index and returns `error.InvalidArgument`. |
| `rcMergePolyMeshes` takes `nvp` from `meshes[0]` and uses it as every input's stride (`RecastMesh.cpp:1424`), so an input packed at a smaller `nvp` is read past its end. | Refused at the boundary: every input must agree on `nvp`. |
| `rcMergePolyMeshDetails` lacks the `if (!nmeshes \|\| !meshes) return true;` its twin has (`RecastMesh.cpp:1312`) and reads `meshes[i]->nverts` at once (`RecastMeshDetail.cpp:1401-1402`), so a null array with a positive count is a null dereference. | Refused at the boundary: the array is checked. |
| `rcContext::log` reports a message of 512 bytes or more as a separate `RC_LOG_ERROR` "Log message was truncated" and then as the message cut to 511 bytes (`Recast.cpp:58-79`), two entries, one at a category the caller did not choose. | Documented, not defended: it is upstream's contract for the host's logging hook; stated in the header. |
| `dtTileCache::addTile` tests a layer header's magic and version with `dataSize` unchecked (`DetourTileCache.cpp:250-254`) and hands the codec `dataSize - headerSize`, negative for a short buffer (`:283-284`); `dtDecompressTileCacheLayer` repeats the subtraction (`DetourTileCacheBuilder.cpp:2202-2203`) and `dtTileCacheHeaderSwapEndian` discards its length and dereferences anyway (`DetourTileCacheBuilder.cpp:2223-2226`), out-of-bounds reads. | Refused at the boundary: all three lengths are checked before upstream sees the bytes. |
| `dtDecompressTileCacheLayer`, `dtTileCacheHeaderSwapEndian`, `storeTileState` and `restoreTileState` read or write a caller's bytes as `dtTileCacheLayerHeader`, `dtTileState` and `dtPolyState` structs (`DetourTileCacheBuilder.cpp:2177`, `DetourTileCacheBuilder.cpp:2226`, `DetourNavMesh.cpp:1397`, `DetourNavMesh.cpp:1429-1430`), a misaligned access at an odd address. | Worked around: upstream gets an aligned copy, a block from the allocator seam for the three that take a whole buffer and a local header for the swap. |
| `walkContour` follows a set `cons` bit through `getNeighbourReg` with no bounds test (`DetourTileCacheBuilder.cpp:452-456`) and reads that cell's height (`DetourTileCacheBuilder.cpp:509`), so a border cell claiming an off-grid neighbour walks out of the layer's buffers, an out-of-bounds read the compressed-layer fuzz property found; Recast never writes such a bit (`RecastLayers.cpp:638`). | Refused at the boundary: `buildTileCacheLayer` refuses such a `cons`, and `TileCacheLayer.initFromBytes` and `TileCache.addTile` decompress the bytes and refuse a layer whose header or connections fail the same checks, at one decompression per `addTile`. |
| `dtTileCache::buildNavMeshTile` bounds a tile index with `>` where the file uses `>=` (`DetourTileCache.cpp:662-663`), admitting one `dtCompressedTile` past the allocation whose garbage `data` and `dataSize` are read if its salt matches. | Not reachable through the binding: every reference resolves through upstream's own salt-checking `getTileByRef` first. |
| `dtTileCache::getTile(int)` (`DetourTileCache.h:112`) and `getObstacle(int)` (`:115`) return `&array[i]` with no test against the count, an out-of-bounds access. | Refused at the boundary: both indices are bounded. |
| `dtTileCache::queryTiles` writes only `if (n < maxResults)` and returns `DT_SUCCESS` (`DetourTileCache.cpp:511-520`), and `update` passes the eight-entry `DT_MAX_TOUCHED_TILES` array (`:548`), so an obstacle over more than eight tiles is silently missing from the rest. | Refused at the boundary: `zrcTileCacheQueryTiles` reports the true count, and an obstacle that would touch more tiles than upstream can track is refused. |
| `dtTileCache::addObstacle` returns `DT_FAILURE \| DT_OUT_OF_MEMORY` when the free list is empty (`DetourTileCache.cpp:371-372`), reporting a full table as an allocator failure. | Worked around: the boundary counts the occupied slots itself and reports `ZRC_ERR_NAVMESH_FULL`, as `zrcNavMeshAddTile` does. |
| `dtTileCache::init` assigns every member and allocates both arrays with no purge (`DetourTileCache.cpp:118-171`), so a second call leaks the first call's arrays. | Not reachable through the binding: a `ZrcTileCache` is created once and destroyed once. |
| `dtTileCacheMeshProcess::process` receives a mutable `dtNavMeshCreateParams*` (`DetourTileCache.h:95-99`) whose tile reaches the navmesh through `dtCreateNavMeshData` and `dtNavMesh::addTile` directly (`DetourTileCache.cpp:756-773`), skipping every check `zrcNavMeshAddTile` applies, so a callback changing `polyCount` or a buffer pointer produces a malformed tile. | Worked around: the callback sees `ZrcTileCacheBuildParams` (areas, flags, user id, off-mesh connections) only, and the binding validates what comes back and writes it into the params itself. |
| `dtTileCache::buildNavMeshTile` computes `(int)(m_params.walkableClimb / m_params.ch)` (`DetourTileCache.cpp:673`) with neither value bounded against the other, an unbounded float-to-int conversion. | Refused at the boundary: the ratio is checked when the cache is created. |
| `dtTileCache::buildNavMeshTile` sets `params.buildBvTree` false unconditionally (`DetourTileCache.cpp:750`), so tile-cache tiles carry no BV tree and queries scan the tile linearly; not a defect. | Nothing to do: `SealBvSentinel` is not called on this path, since such a tile has `bvNodeCount == 0` (`DetourNavMeshBuilder.cpp:507`) and no spare node. |
| `dtTileCache::getTilesAt` writes only while `n < maxTiles` and returns the count written (`DetourTileCache.cpp:173-193`), so a full answer cannot be told from a clipped one. | Worked around: the boundary asks for the cache's whole tile capacity, applies the caller's limit itself, and says when the result did not fit. |
| `dtTileCache::removeTile` reads `tile->header->tx` (`DetourTileCache.cpp:301-306`) after checking only slot index and salt, and every slot starts at salt 1, so a never-issued slot of a fresh cache is a null dereference. | Worked around: every reference resolves through a helper that requires an occupied slot first. |
| `getObstacleByRef` rejects on the salt (`DetourTileCache.cpp:228-240`) that `dtTileCache::update` bumps in the same block that sets `DT_OBSTACLE_EMPTY` (`:618-624`), so that state is never observed through a reference; not a defect. | Documented: the header says so. |
| `dtAllocTileCacheContourSet` and `dtAllocTileCachePolyMesh` `memset` the allocation with no null test (`DetourTileCacheBuilder.cpp:66-73, 86-93`), a write to address zero when the allocator fails. | Documented, not defended: they are the only way to obtain those containers; stated in the header, and reachable only through a host allocator that fails. |
| `dtFreeTileCacheLayer` has no null guard, unlike its two siblings, and passes `layer` straight to `alloc->free` (`DetourTileCacheBuilder.cpp:2156-2161`). | Worked around: guarded at the call site. |
| `rcBuildHeightfieldLayers` computes `lw` and `lh` as the field size minus `borderSize*2` (`RecastLayers.cpp:497-498`), so a border wider than half the field gives negative dimensions and a layer set that reports dimensions no geometry has, not memory-unsafe. | Refused at the boundary. |
| `duDebugDrawHeightfieldLayersRegions` is declared (`RecastDebugDraw.h:34`) and defined nowhere, so a call fails to link, and `rcHeightfieldLayer` holds no region ids to draw. | Not reachable through the binding: no entry point; the coverage record carries it as `UNDEFINED`, which `bay check-coverage` recomputes, so a re-vendor that defines it fails the gate until it is bound. |
| `duDisplayList` overrides five of `duDebugDraw`'s nine pure virtuals and leaves `texture` and the two textured `vertex` forms (`DebugDraw.h:195-222`), so the class is abstract and cannot be instantiated. | Worked around: `zrc::ConcreteDisplayList` supplies the three, recording nothing for a texture change and a textured vertex without its coordinates, as `duDisplayList::draw` replays it anyway. |
| `duDisplayList` allocates its arrays with `new float[]` and `new unsigned int[]` (`DebugDraw.cpp:547-562`), outside `rcAlloc`, so `zrcSetAllocator` never sees them and a failure terminates under `-fno-exceptions`. | Documented, not defended: the arrays are private to the class; the handle itself goes through the seam. |
| `RecastDump.h`'s six entry points `printf` a line and return false on a null or wrong-mode `duFileIO` (`RecastDump.cpp:45-53` and the same shape in the other five), and `duReadContourSet` and `duReadCompactHeightfield` print again on a bad magic or version. | Refused at the boundary for every argument case, so only the two magic/version lines stay reachable, from a read fed bytes that are not a dump. |
| `duReadContourSet` and `duReadCompactHeightfield` allocate from counts read out of the stream with no bound (`RecastDump.cpp:197-201`, `:340-393`), over a format that is upstream's struct layout with no length, and `ioprintf` writes through a fixed 256-byte line buffer ignoring what `write` returns (`RecastDump.cpp:32-40`). | Not defended, since the reads happen inside upstream: `zrcReadContourSet` and `zrcReadCompactHeightfield` create the container so a failed read destroys a half-built one, and the header says to feed them only bytes this package wrote. |
| `duLogBuildTimes` computes `100.0f / totalTimeUsec` on an `int` total (`RecastDump.cpp:437`), so zero logs an infinity as a percentage and a negative total logs percentages of the wrong sign. | Refused at the boundary: the total is bounded. |

What a host acts on:

- The determinism claim is scoped to the `qsort` ties: one target cooking twice gets the same bytes, asserted by the suite on every target that runs it; targets sharing a C library get the same bytes, checked by `bay determinism` across architectures (NOT ESTABLISHED on Zig 0.17; see [Cook determinism](#cook-determinism)); targets with different C libraries are compared and reported there, not gated. A host that ships cooked navmeshes should cook on one platform, or key its asset cache by cook platform.
- Run the bake where an allocation failure is fatal anyway — a cook, a tool, a loading screen with a budget; the runtime side (deserialise, query), the half a game runs, is far better behaved.
- A host building with UBSan should read a trap inside `hashPos2` as the `ZRC_PROXIMITY_GRID_SAFE_CELL` limit.

### Re-vendoring

`bay check-vendor` (`ci/vendor.zig`) fetches upstream by URL and hash at the
commit `build.zig.zon` pins, compares every taken path against `libs/` in both
directions, and requires the table at the top of
[The vendored upstream](#the-vendored-upstream) to name that commit, so the
claim about what this copy is gets checked rather than asserted. It runs in
`bay test`.

1. Point the `recastnavigation` development dependency in `build.zig.zon` at
   the new tag (`bay add` writes the URL and hash), and copy the taken
   directories over `libs/recastnavigation/`, keeping the exclusions.
2. Update the table and `zrcRecastVersion()` in `ffi/zrecast_core.cpp`.
3. `bay test`. The `static_assert`s in `ffi/zrecast_abi.cpp` fail the build if
   a mirrored constant, a serialised-image struct size, or the polygon
   reference type has changed; the ABI test fails if the Zig externs have
   drifted; the coverage gate fails on a public name that has no verdict.
4. If `DT_NAVMESH_VERSION` changed, say so here and expect every previously
   serialised navmesh to be rejected with `ZRC_ERR_UNSUPPORTED_VERSION`.
5. Re-read [Known upstream behaviour](#known-upstream-behaviour) and delete
   anything fixed upstream, along with its workaround. The allowances in
   `ci/numbers.zig` that cite the pinned commit are re-measured at the same
   time; each names the upstream file its value sits in.
6. Add any new source files to `build.zig` deliberately: the explicit lists
   exist so a re-vendor cannot silently change what gets compiled.

## Testing

```sh
bay test                    # the gate
bay test -Dtest-filter=…    # the suite, filtered
bay probe [-- name]         # the mutation probes, or the ones whose name matches
bay determinism             # cook digests across the targets this host can run
bay coverage -- --summary   # the coverage record, per area
bay measurements            # README's numbers, from the tree
bay dev bench               # each binding beside the same work in C++
```

`bay test` is preflight's gate over the package: formatting, the module
layers, lint, every documented snippet against its source, the checks in
`ci/` (comments, ignored files, the hand-written numbers, coverage, the vendor
comparison), a consumer build, and the suite. Every optimize mode runs the
same steps with `bay dev ci -Doptimize=<mode>`.

The suite runs three times: against the C++ under Zig's undefined-behaviour
sanitizer, as a consumer builds it, and with the asserts flipped from the
mode's default, so both sides of every `NDEBUG` seam run in every mode. The
gate also builds the library shared, and with the asserts flipped under the
sanitizer, runs the examples, and runs the bench once.

### Untrusted bytes

Every function that parses bytes a host hands it has a property in
`src/integration_test.zig`, run on [shakedown](https://github.com/pedronaugusto/shakedown):
the navmesh image (the validator, the loader, the layout, the byte-order swap
and the queries over whatever loads), a tile added to a tiled navmesh, a
tile's stored state, and a compressed layer (decompressed, swapped, added to a
tile cache and built into a tile). Each case is a seed the fixture cooks,
mutated, or bytes of its own, placed at an odd address because a host's
bytes have any alignment; that is how the misaligned reads under
[Known upstream behaviour](#known-upstream-behaviour) were found. A case
either is refused or holds the property, and the sanitizer and the leak check
see every one. Under `bay fuzz`, and the hardened tier's campaign, Zig's
fuzzer drives the same properties.

`readContourSet` and `readCompactHeightfield` are not among them: upstream
reads a dump field by field, with no length to check a count against, so they
take only bytes this package wrote, as recorded under
[Known upstream behaviour](#known-upstream-behaviour).

The allocation sweep in the same file is shakedown's `everyFault`: the
serialise, validate and load arc runs once clean, then once per allocation
with that one refused, and each run must report `OutOfMemory` and give every
byte back but the one upstream leak recorded below. zrecast makes no `std.Io`
call, so there is no I/O to fault.

### The fixture

The suite is self-contained. `tests/fixture.cpp` generates its input geometry
(a ground plane, a wall with a gap at one end, a pillar, and an island
connected to nothing) and the tests bake a navmesh from it at test time. No
third-party meshes are shipped, and no asset provenance has to be accounted
for.

The shape is chosen so the assertions can bite. A path across the wall must
round its end, so the string-pull has to produce real corners rather than a
two-point line. The island is reachable from nowhere, so `partial` has
something to be true about. A second fixture, a tent of 71-degree faces, has
real three-dimensional bounds and no surface an agent could stand on, which is
the `EmptyResult` path.

### The C smoke test

`tests/c_smoke.c` walks the same arc in plain C11 with a `malloc`-based
allocator, linked against the installed library and header, which proves the
header is a real C contract rather than a private detail of the Zig wrapper.
It also checks what only a C caller can get wrong: null handles and null
out-parameters on every entry point that takes them, a companion buffer
shorter than the point array it accompanies, a half-filled allocator being
refused without unseating the working one, and `zrcSetAllocator(NULL)` handing
allocation back to malloc. And it asserts that every allocation was returned;
that assertion caught a leak of three buffers per detail mesh during
development, recorded under [Known upstream behaviour](#known-upstream-behaviour).
It runs twice: as a consumer links it, and with the program and the C++
under Zig's undefined-behaviour sanitizer, since some of the boundary's
checks guard paths only C reaches.

`ci/consumer.zig` is the Zig consumer: preflight builds it against the package
as a dependency, the way a project that depends on zrecast reaches it.

### Mutation probes

A test that passes may be asserting something the code could not have violated
anyway. `ci/probes/` holds one patch per invariant that breaks it on purpose,
with the failure it must produce (`# expect:`) and the commands that must
produce it (`# run:`, `zig build test` unless stated). `bay probe` applies each
to a copy of the tree and reports one of three outcomes per probe: caught,
not caught, or caught by something other than the named failure. A patch that
no longer applies is refused as stale rather than counted. The runner matches
each hunk by content and tolerates moved lines, so a probe survives edits
elsewhere in its file.

Of the first eleven probes written, four went uncaught. Three were holes: a
tile window shifted by one cell sat inside a tolerance that had been set too
loosely, the reference-bit check was indistinguishable from Detour reaching the
same verdict a few allocations later, and nothing at all reached the
null-header path this package guards upstream against. The tests that close
those three were written because the probes said they were missing. The fourth
was the probe's own expectation, which named the wrong check as the one that
fires.

### What CI runs

`.github/workflows/ci.yml` is generated by `bay generate` from
`ci/workflow.json` and calls preflight's workflow: the source checks, the gate
on Linux, macOS and Windows, compile and link checks for every target in
`ci/workflow.json`, and, nightly and before a release, the gate in
ReleaseSafe and ReleaseFast as well, a ReleaseSmall compile, the arm64
runners, and riscv64 under QEMU. The probes and the
cross-target cook comparison run on a developer's machine, not in CI.

## Measured cost

`bay dev bench` times each binding beside the same Detour work written in C++
(`bench/baseline.cpp`), which loops inside C++ and sets Detour up as the
boundary does. The level is generated: open ground with a grid of box pillars,
baked into one tile, and a lattice of query points between the pillars, each
paired with one across the level. ReleaseFast, Windows laptop, 2026-10-25,
medians of three runs:

| row | Zig | C++ |
|---|---|---|
| load the level's image, per navmesh | 5.9–6.0 µs | 2.0–2.2 µs |
| the same through `setAllocator` | 5.3–5.6 µs | |
| `findNearestPoly`, per query | 106–107 ns | 98–99 ns |
| `findPath` then `findStraightPath`, per route | 2.14–2.21 µs | 2.15–2.20 µs |
| `raycast`, per ray | 68–70 ns | 63–65 ns |
| a crowd frame, an agent at every query point, first second after each is sent | 148–149 µs | 147–154 µs |

A route and a crowd frame are level with C++. A query and a load are slower,
and the table alone does not say why. Two rows separate the wrapper and the
crossing from the boundary's checks: `nearest_c_boundary` and
`raycast_c_boundary` make the same `zrcFindNearestPoly` and `zrcRaycast` calls
the Zig rows make, on the binding's own query and filter, from a C++ loop.
`load_c_validated` does the same for a load, calling `zrcNavMeshValidate`
before Detour reads the image. Those rows are in `bay dev bench` but not yet
in this table: each gap is measured, and its cause, the `raycast` gap's
included, is NOT ESTABLISHED.
Through the Zig allocator a load measured faster than through upstream's
`malloc`, on this machine.

## Contributing

Issues and pull requests are welcome. These are the rules that fail a build.

- **`libs/recastnavigation` is vendored and is not edited, at all.**
  `bay check-vendor` compares it against the pinned commit. If upstream needs
  fixing, fix it upstream; if zrecast needs to work around upstream, do it in
  `ffi/` and record the reason under
  [Known upstream behaviour](#known-upstream-behaviour).
- **`bay test` is green before a push**, in every optimize mode a change can
  reach.
- **A new binding adds a line to `ci/coverage/bindings.tsv`.** Every public
  Recast and Detour name is either answered there or is a gap, and the evidence
  names the symbol in `ffi/zrecast.h` that a host now calls. Then
  `bay coverage -- --record` rewrites `ci/coverage/unbound_*.txt` from that
  file, the upstream harvest and the tranche rules; the record is generated, and
  `bay check-coverage` fails if what is checked in differs. The gate checks that
  the symbol exists; it cannot check that the evidence is apt, so say what the
  mapping is when it is not obvious.
- **A new C entry point gets `CHECK`s in `tests/c_smoke.c`.**
- **A new invariant gets a probe in `ci/probes/`** naming the failure it must
  produce; `bay probe` reports a probe the suite survives.
- **A number the README states comes from `ci/measurements.zig`**, and the
  docs check fails while the committed block is stale.

New source files are added to the explicit lists in `build.zig` deliberately;
there are no globs, so nothing starts compiling by accident.
