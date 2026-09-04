# zrecast

[![CI](https://github.com/pedronaugusto/zrecast/actions/workflows/ci.yml/badge.svg)](https://github.com/pedronaugusto/zrecast/actions/workflows/ci.yml)

Zig bindings for [recastnavigation](https://github.com/recastnavigation/recastnavigation)
— bake a navigation mesh from level geometry, then path over it at runtime — in
a package with no renderer, no engine and no asset system attached.

- Vendored, pinned upstream recastnavigation (v1.6.0), verbatim. A script
  proves the tree is that commit byte for byte, and every upstream defect is
  worked around at the boundary rather than forked. See
  [UPSTREAM.md](UPSTREAM.md).
- A real C ABI (`ffi/zrecast.h`) that stands on its own — the Zig wrapper is one
  consumer of it, not its only reason to exist.
- Host allocator injection: every Recast and Detour allocation can go through
  your `std.mem.Allocator`.
- Layout drift between the C header and the Zig externs is a **test failure**,
  not a memory-corruption bug.

Status: **complete** — every public Recast and Detour name is answered, and the
coverage gate that measures it is green. Bakes tiled and untiled, the pipeline
stage by stage, off-mesh connections, area and flag authoring, the whole of
`dtNavMeshQuery` including sliced search, dynamic obstacles through a tile
cache, and crowds. See [Scope](#scope) for what that means one capability at a
time, and [Coverage](#coverage--what-of-recast-and-detour-is-reachable) for
how it is measured.

## Two lifecycles, and why the API is split down the middle

Recast and Detour are usually shipped as one library, which hides the most
important thing about them: **they run at different times, in different
programs.**

**Recast is a baker.** It voxelises a triangle soup, filters it for what an
agent of a given size can stand on, and emits a polygon mesh. It is slow, it
allocates freely, and its output is an artefact — the sort of thing a cook step
produces and checks into a content pipeline. Nothing about it belongs in a
frame.

**Detour is the runtime.** It loads that artefact and answers questions: where
is the nearest polygon, what is the route from here to there, what does this
character hit if it walks that way. It allocates once, at load, and its queries
are the only part a shipping game runs.

So the modules split the same way: `bake.zig` is Recast, `navmesh.zig` is the
seam where a baked mesh becomes bytes, and `query.zig` is Detour. A tool uses
the first two; a game uses the last two.

That is not just an organising principle. Because the boundary is one C entry
point per concern in its own translation unit, a host that never calls the bake
never links it: a ReleaseFast C program that deserialises a navmesh and queries
it contains none of `rcCreateHeightfield`, `rcRasterizeTriangles`,
`rcBuildContours`, `rcErodeWalkableArea` or `rcBuildPolyMesh`, and comes out
about 170 KB smaller than the same program with a bake call added (measured
2026-09-02 with `zig c++ -O2` on x86_64-windows).

## Usage

The block below is not written here: it is a region of
[`examples/usage.zig`](examples/usage.zig), which `zig build examples` builds
and RUNS, extracted by `ci/readme_usage.sh` and compared by CI. A snippet in a
README is a claim about how the library is used, and this one is a claim
something executes.

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

Add it as a dependency and link the module:

```zig
const zrecast_dep = b.dependency("zrecast", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("zrecast", zrecast_dep.module("zrecast"));
```

## Design

### Partial is a result, not an error

`findPath` returns a corridor and a `partial` flag. Detour sets it when the goal
was unreachable, when the node pool ran out, or when the caller's buffer filled
— and in all three cases the corridor it returns is a real, walkable best effort
towards the goal.

Collapsing that into an error would push every caller into treating "no route"
and "long route" identically, which is exactly the bug that makes agents freeze
at the edge of a map. So it is a field, and the caller decides.

### The polygon flags are assigned during the bake

Recast leaves `rcPolyMesh::flags` zeroed, and Detour's default filter admits a
polygon only if it shares a bit with `include_flags`. A caller who does the
obvious thing — bake, build, query — therefore gets an empty result from every
query, with nothing anywhere reporting a problem.

`zrcPolyMeshBake` assigns `ZRC_AREA_WALKABLE` / `ZRC_POLY_FLAG_WALKABLE` to
every polygon that came out walkable, so the obvious thing works. The test suite
checks both directions: that a default filter finds the ground, and that a
filter excluding exactly that flag finds nothing.

### Bake failures say which stage failed

Recast's eleven build stages each return a bare `bool` and send the reason to a
virtual log callback with no default implementation. Pass a buffer as
`ZrcBakeLog` and it is filled with those messages; without one, a failed bake is
undiagnosable by construction.

The empty-result case gets the same treatment. `rcBuildPolyMesh` succeeding with
zero polygons is not an error upstream, but it produces a navmesh that answers
every query with "nowhere to go", so it is `EmptyResult` here with an
explanation of the two things that usually cause it.

### Validation at the boundary

Recast and Detour assert in debug and read on regardless in release. Two of
those gaps are worth naming, because they are the difference between an error
and a fault:

- **Geometry.** Recast indexes `verts[tris[i] * 3]` with no range test of its
  own. An index past the end of the vertex array is an out-of-bounds read inside
  the rasteriser in every build configuration. A NaN coordinate is worse:
  `rcCalcBounds` propagates it into the grid size, and the heightfield
  allocation is then computed from garbage. Both are rejected at the door.

- **Query results, not only memory.** A tile reserves two bounding-volume nodes
  per polygon and the tree fills `2n - 1` of them; Detour's own traversal ends
  one node past the tree and reads the spare, zeroed one as a leaf naming
  polygon 0. A box of all zeroes overlaps every query clamped to the tile's
  minimum corner, so `findNearestPoly` at a navmesh's own corner came back with
  a polygon nine metres outside the half-extents it was given. Nothing faults;
  the answer is simply wrong. That node is now sealed as an internal node in
  every image this package cooks and in every image it loads. See
  [UPSTREAM.md](UPSTREAM.md).

- **Serialised navmeshes.** `dtNavMesh::init` dereferences the buffer to read
  the magic and version *before* it looks at the length, which it then never
  compares against anything. `addTile` derives eight array pointers from counts
  in that header and never bounds-checks them either — and it writes
  `links[maxLinkCount - 1]` before any loop guard, so a zero there is an
  out-of-bounds *write*. `zrcNavMeshValidate` closes all of that: it copies the
  header out (so a misaligned caller buffer is not a struct read either),
  sanity-checks every count, and requires the size implied by those counts to
  equal the buffer length exactly.

  Header checks alone are not enough, and the gap is not subtle. A
  `dtPoly::vertCount` of `200` — one byte, in an otherwise untouched image —
  passes every count check, loads without complaint, and then overruns a fixed
  `float verts[DT_VERTS_PER_POLYGON*3]` stack array in four separate Detour
  functions, with coordinates read from the image. So the validator makes a
  second pass and bounds everything the image can point at:

  - **Polygon corner counts and corner indices**, which is the case above.
  - **Neighbour indices.** `connectIntLinks` turns each into a link reference
    with no range test, and every traversal query resolves that reference with
    `getTileAndPolyByRefUnsafe` — unsafe as named. The safe accessor guards the
    *caller's* references; the unsafe one is used on references the tile made
    for itself, so this is the same overrun reached the long way round.
  - **Vertex and detail-vertex coordinates**, which must be finite.
    `closestPointOnDetailEdges` keeps its nearest edge with `if (d < dmin)` and
    lerps through the result unconditionally, so one NaN — against which every
    comparison is false — leaves it dereferencing null.
  - **Detail sub-mesh bases, extents and triangle corners**, plus a triangle
    count of at least one and at least one boundary edge flag per sub-mesh:
    both are further ways that same nearest-edge pointer stays null.
  - **BV-tree polygon indices and escape jumps.**

  - **Off-mesh connections.** `baseOffMeshLinks` indexes `tile->polys` by a
    connection's polygon field and writes through the vertex it finds there,
    with no bound on either. Each connection must own exactly the polygon the
    builder assigns it, name finite endpoints and a non-negative radius, and
    carry only the side codes and flags Detour itself emits.

  None of that rejects anything a real bake produces; the round-trip test is
  the control. What it buys is memory safety, not trustworthiness: an entirely
  in-bounds image can still describe a degenerate or useless navmesh.

Every truncation of a valid navmesh image — every prefix of the test
fixture's, byte by byte — is verified to be rejected by both the validator and
the loader, while the whole image still loads. So is each targeted header and
interior corruption in the suite, every one aimed at a specific field Detour
would otherwise trust.

Those were not all found by reading. Most of the interior checks came from a
mutation fuzzer: random bytes anywhere in the image, then validate,
deserialise and query. It found three holes that reading the validator had not
suggested, including one that had been reasoned about and gotten wrong. That
fuzzer is a test in the suite now — deterministic, sized so every CI leg can
afford it, and asserting that a meaningful share of its mutants get past
validation and into the queries, so it cannot quietly decay into an exerciser
of the magic check. The development campaign behind the checks was larger; the
in-suite run is the regression net that keeps them honest.

What this does **not** claim: neither library is hardened against arbitrary
hostile input, and zrecast cannot make them so from the outside. The fuzzer
found three holes after the first round of hand-written checks, which is the
honest evidence that a fourth is possible. If you ever load a navmesh from an
untrusted source, put a length-and-signature container around it and validate
that first.

### Out of memory, and what is honestly guaranteed

`zrecast`'s own allocations are checked, rolled back correctly on every path,
and tested by inducing a failure at every allocation site in turn.

**Upstream's are not.** Recast's own vector placement-news into the result of a
failed allocation; Detour's BV-tree builder indexes an unchecked array; and its
node pool asserts and then writes through the null. None of that is fixable from
outside the library, so it is documented in [UPSTREAM.md](UPSTREAM.md) with file
and line rather than papered over. One case — a `dtNavMesh` whose `init` ran out
of memory, which cannot then be destroyed — *is* worked around narrowly, and the
bounded cost of that workaround is stated.

The practical reading: run the bake somewhere an allocation failure is fatal
anyway, which is where a cook already lives. The runtime half behaves far
better, and that is the half a game links.

### The ABI guard

The Zig side hand-writes `extern struct`s mirroring `zrecast.h`. Nothing in
either compiler checks those two declarations still agree — a field reordered on
one side and not the other is silent corruption. `zrcAbiLayout()` reports what
the C++ actually compiled to, and a test asserts every size and offset against
the Zig declarations.

Every offset, and looked up **by name**. Sampling a few is not a guard:
`ZrcBakeConfig` is ten consecutive floats followed by seven more 4-byte fields,
so swapping any two leaves the size, the alignment, and the whole *sequence* of
offsets identical — each side renumbers together — while every bake silently
uses the wrong agent dimensions. Asking "where does the field called X live on
each side" is the only comparison that sees it, and each of those swaps is
verified to fail the test.

In the other direction, `static_assert`s in `ffi/zrecast_abi.cpp` fail the
**build** if a re-vendored upstream changes:

- a constant this package mirrors into its own header (`DT_MAX_AREAS`,
  `DT_VERTS_PER_POLYGON`, `RC_WALKABLE_AREA`, the allocator hint enumerators);
- the size of any struct that appears in a serialised navmesh, because
  `zrc::ValidateNavMeshImage` reproduces Detour's own pointer arithmetic over
  exactly those six types in order to locate and bounds-check every array;
- the serialised format version, because every previously written image becomes
  unreadable;
- the polygon reference type, which the query layer passes through with no cast.

This is deliberate: it is the check that comparable C++-to-Zig bindings tend to
skip, and the one whose absence is hardest to debug.

### Allocator injection, honestly scoped

`setAllocator` routes every Recast and Detour allocation through a
`std.mem.Allocator`. It is process-wide, because both upstream seams are —
`rcAllocSetCustom` and `dtAllocSetCustom` take a bare pair of function pointers
with nowhere to thread a host pointer through. That is surfaced rather than
hidden behind a per-object parameter that could not be honoured.

The seam has one wrinkle: both libraries free with `free(ptr)`, no size, while
Zig requires the size back. `src/memory.zig` bridges that with a header stored
ahead of each block. There is no *alignment* to record, unlike a bridge over a
library that asks for one per call — neither `rcAlloc` nor `dtAlloc` has an
alignment parameter, so there is a single compile-time alignment
(`ZRC_ALLOC_ALIGNMENT`) which the ABI test checks against the compiled library.

The C API keeps upstream's shape, so a plain C host can still pass
`malloc`/`free` in two lines.

### Build hygiene

- Source lists are explicit, never globs — a re-vendor cannot silently change
  what compiles.
- No `-fno-access-control`. The FFI layer uses only public upstream API, so it
  has no reason to defeat C++ access checking.
- UBSan is **not** blanket-disabled. It stays on in Debug (`-Dsanitize_c`), and
  the suite passes with it on in all four optimize modes — see
  [UPSTREAM.md](UPSTREAM.md) for what that does and does not prove.
- `-fno-exceptions` / `-fno-rtti` everywhere except the MSVC ABI, where
  disabling them through Clang flags breaks the Microsoft standard library
  headers. Neither Recast nor Detour contains a single `throw`, `try`,
  `dynamic_cast` or `typeid`, so nothing is lost.
- Build options are declared once and mirrored into a Zig `options` module, so
  the wrapper cannot disagree with how the C++ was compiled.
- One translation unit per concern on both sides of the boundary.

## Testing

```sh
zig build test
```

The suite is self-contained: `tests/fixture.cpp` generates its input geometry
programmatically — a ground plane, a wall with a gap at one end, a pillar, and
an island connected to nothing — and the tests bake a navmesh from it at test
time. No third-party meshes are shipped, and no asset provenance has to be
accounted for.

The shape is chosen so the assertions can bite. A path across the wall *must*
round its end, so the string-pull has to produce real corners rather than a
two-point line. The island is reachable from nowhere, so `partial` has something
to be true about. A second fixture — a tent of 71-degree faces — has real
three-dimensional bounds and no surface an agent could stand on, which is the
`EmptyResult` path.

`zig build test-c` runs the C-level smoke test on its own. It walks the same arc
in plain C11 with a `malloc`-based allocator, proving the header is a real C
contract rather than a private detail of the Zig wrapper. It also checks the
things only a C caller can get wrong: null handles and null out-parameters on
every entry point that takes them, a companion buffer shorter than the point
array it accompanies, a half-filled allocator being refused *without* unseating
the working one, and `zrcSetAllocator(NULL)` genuinely handing allocation back
to malloc.

And it asserts that every allocation was returned. That assertion is not
decoration: it is what caught a leak of three buffers per detail mesh during
development, described in [UPSTREAM.md](UPSTREAM.md).

### Coverage — what of Recast and Detour is reachable

`zig build test` says the code works. It says nothing about how much of the
library a host can actually get at, and that question used to be answered here
by a script that matched an identifier followed by `(`. It therefore counted
functions and nothing else — missing `dtNavMeshCreateParams`' 31 must-populate
fields, the 12 `dtStatus` detail bits that are `static const unsigned int`
rather than an enum, and the 24 constants in the six enums whose names carry no
`rc` or `dt` prefix. Its answer, "7 of 421 names", was a true sentence about
the wrong set.

What replaced it enumerates functions, data members, enum constants, `static
const` values and type names, keeps overloads apart by arity and — where an
arity is not enough, as for `dtSwapEndian`'s five one-argument forms — by
parameter type, and prints what it could not parse:

```sh
tools/coverage.sh              # the summary, per area
tools/coverage.sh --names      # every public name upstream declares
tools/coverage.sh --audit      # every line the harvester did not understand
tools/coverage.sh --collapsed  # symbols covering more than one declaration
tools/record.sh                # rewrite the record from its three inputs
ci/check-coverage.sh --list    # the gate, and the open gaps
```

The measured surface — every public Recast and Detour name in every area
`tools/coverage.sh` divides the headers into, against the C entry points that
answer them — is counted in [By the numbers](#by-the-numbers) below. Every one
of those names has a line in `tools/unbound_*.txt` giving it a verdict and
evidence.
Those files are generated — `tools/record.sh` writes them from the harvest,
from `tools/bindings.tsv` (the names that are answered, the only half kept by
hand) and from `tools/tranches.awk` (which tranche closes each of the rest), so
a name cannot carry one verdict in one place and another somewhere else, and a
gap's reason is a rule that is re-read rather than a sentence that rots.
`ci/check-coverage.sh` refuses any verdict it cannot check: a `BOUND` line has to
name a symbol `ffi/zrecast.h` really declares, and a gap has to say which tranche
of work closes it.

**That gate was red from the day it was written until the binding was
complete**, and it is part of `ci/run.sh` and of CI throughout — a coverage
check that only runs once the answer is comfortable is not a check. It is green
now, which means every one of those names carries a verdict the gate can
verify, and none of them is a gap.

### A cook is deterministic, and the claim says where it ends

A navmesh is a shipped asset. If two machines cooking the same geometry produce
different bytes, the asset cannot be built once and trusted, and a bug that
reproduces on one of them may not reproduce on the other. The claim this
package makes is scoped to what it measures, in three rings:

- **One target, cooked twice, is the same bytes.** Asserted by the suite —
  untiled, tiled and area-authored cooks, hashed and compared — on **every
  target that runs it**: this machine, a Windows box, a console toolchain,
  each leg of the CI matrix.
- **Two targets sharing a C library are the same bytes.** `ci/determinism.sh`
  cooks on both architectures of musl and glibc and fails if a pair sharing a
  library disagrees — with contraction pinned and no libm in the bake,
  architecture is not allowed to move a byte.
- **Two different C libraries are compared, not promised.** Upstream sorts BV
  items, contour holes and diagonals with `qsort`, ties fall to the
  implementation, and any order the ties land in is a valid navmesh —
  [UPSTREAM.md](UPSTREAM.md) records the five sites. The script prints the
  digests side by side, so a divergence is seen, named and attributed rather
  than asserted away. Ship cooks from one platform, or key the asset cache by
  cook platform — the position every engine consuming Recast from C++ is in.

Three things stand behind the first two rings, and each was a real gap rather
than a precaution:

- **`-ffp-contract=off`, for every target.** Clang fuses a multiply and an add
  into a single instruction wherever the target has one, and the fused form
  rounds once where the separate form rounds twice. AArch64 mandates FMA;
  the x86-64 baseline has none. The same source line in Recast therefore
  produced different floats on the two architectures.
- **The Zig vector math is asserted bit-identical to the C.** `src/vec.zig`
  reimplements the `dtV`/`rcV` families rather than calling across the boundary
  to add three floats, which is only honest if the two agree everywhere — so a
  test compares them bit for bit over a table of zeros, negative zeros and
  denormals, against linkable shims over upstream's own `inline` definitions.
  Six of the scalar helpers answer differently from Zig's obvious spelling:
  `dtMin(3, NaN)` and `dtMax(3, NaN)` are `NaN` where `@min` and `@max` give
  `3`, `dtAbs(-0.0)` keeps its sign bit, `dtClamp(NaN, 0, 1)` passes the NaN
  through, and `dtNextPow2(0)` and `dtIlog2(0)` are both `0`. Each is written
  out as upstream's own expression and pinned by a test at exactly that value.
- **`zrc::CosDegrees` instead of `cosf`.** The slope limit becomes a cosine, and
  that was the only transcendental the bake reached. The C standard does not
  require a correctly rounded cosine, so the threshold — which decides per
  triangle whether a surface is navigable — depended on the host's C library.
  It is now a polynomial over the bounded range a slope limit can occupy, using
  only operations IEEE-754 specifies exactly.

```sh
ci/determinism.sh            # two architectures and three C libraries, here
ci/determinism.sh --strict   # and a target that could not be run is a failure
```

That script builds the suite for each target and runs it wherever it can —
natively, under Rosetta, or in a container. It reports three outcomes and never
two: a target that ran, a target that **failed**, and a target this host
could not run at all. The last is counted and named in the summary rather than
dropped, so the result can never read as more coverage than it was, and
`--strict` makes it a failure.

### Mutation probes — is a passing test testing anything

A test that passes proves nothing on its own: it may be asserting something the
code could not have violated anyway. `ci/probes/` holds one patch per invariant
that breaks it on purpose and names the test that has to fail because of it,
and `ci/probe.sh` applies each to a copy of the tree and reports the ones the
suite survives.

```sh
ci/probe.sh              # every probe
ci/probe.sh tile-bounds  # only the ones whose name matches
```

This is not theory. Of the first eleven probes written, four went uncaught.
Three were holes: a tile window shifted by one cell sat inside a tolerance that
had been set too loosely, the reference-bit check was indistinguishable from
Detour reaching the same verdict a few allocations later, and nothing at all
reached the null-header path this package guards upstream against. The tests
that close those three were written because the probes said they were missing.
The fourth was the probe's own expectation, which named the wrong check as the
one that fires.

### Continuous integration

CI runs the whole suite on **Linux, macOS and Windows**, in four optimize modes
with the sanitizer both on and off, plus the standalone C test and the consumer
package; builds both Windows ABIs explicitly on the Windows runner;
cross-compiles eight further targets; and gives the mutation probes, the cook
determinism comparison, the documentation numbers and vendor integrity a job
each. See [`.github/workflows/ci.yml`](.github/workflows/ci.yml).

The same matrix runs locally, so a failure is reproducible on your machine
before it is a red mark on a pull request:

```sh
ci/run.sh            # the full matrix
ci/run.sh --quick    # native Debug only, for the inner loop
ci/install-hooks.sh  # run it automatically before every push
```

It reports every failure rather than stopping at the first.

### Platform coverage

| | Suite executed by CI | Compile-checked by CI |
|---|---|---|
| Linux | x86_64 (glibc) | + aarch64, musl |
| macOS | aarch64 | + x86_64 |
| Windows | x86_64, gnu and MSVC ABIs | + aarch64 |

Compiling proves the sources and build graph are portable; only an executed
configuration proves behaviour, which is why the two are separate jobs.

The two Windows ABIs are separate rows' worth of risk, and the distinction is
easy to get wrong: Zig's default ABI for a Windows host is `gnu`, not `msvc`, so
a bare `zig build test` on a Windows runner never touches the MSVC path. Both
are therefore named explicitly, and both **run the suite** rather than only
linking — a cross-compile that links is not evidence that anything behaves.

That table describes the matrix, not a promise: **the badge at the top of this
file is the authority on whether those runs have actually happened and passed.**

### By the numbers

<!-- BEGIN GENERATED ci/measurements.sh --markdown -->
| | |
|---:|---|
| **0.1.0** | version (one home: `build.zig.zon`) |
| **313** | C entry points (`ZRC_API` in `ffi/*.h`) |
| **313** | Zig externs (`pub extern fn` in `src/`) |
| **1089** | public Recast/Detour names, each carrying a verdict in `tools/` |
| **945** | of them reachable through the C boundary (`BOUND`) |
| **91** | C++-only surface a C boundary cannot carry (`LANGUAGE`), each with the reason |
| **53** | reimplemented on the Zig side (`ZIG`), each naming its mirror |
| **153** | Zig tests `zig build test` executes |
| **975** | assertions in the standalone C smoke test |
| **26** | vendored recastnavigation translation units `build.zig` compiles |
| **18165** | C boundary lines (`ffi/`) |
| **17744** | Zig source lines (`src/`) |
| **93** | invariants `ci/probe.sh` mutates, each with the test that must notice |
| **32** | steps `ci/run.sh` runs |
| **8** | further targets `ci/run.sh` cross-compiles |
<!-- END GENERATED -->

Not one of those is typed into this file. `ci/measurements.sh` recomputes them
from the tree, `ci/check-docs.sh` regenerates the block and fails the build if
what is committed differs, and the same gate refuses any other hand-written
number in these documents unless `tools/doc_numbers.txt` says why it cannot go
stale. Adding a claim means adding its measurement.

**What the numbers do not say.** A count is a count. Matching entry-point
counts prove presence, not correctness — the behavioural tests, the C smoke
test and the mutation probes hold that, and `ci/check-coverage.sh` holds each
verdict one name at a time. Source lines measure volume, not surface. And the
gate itself has three blind spots: a number spelled as a word, a number inside
`code` — where it is an identifier or a citation rather than a claim — and a
sentence that is wrong without any number in it.

## Scope

`ci/check-coverage.sh` is the authority here; this section is its summary in
prose. **Every public Recast and Detour name is reachable from a zrecast
host**, each with a line in the record saying how; [By the
numbers](#by-the-numbers) carries the counts.

Exposed:

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

**Steering is not a cook.** The byte-for-byte guarantee above covers baked
assets and does not extend to a frame's steering, which reaches `cosf` and
`sinf` in the adaptive velocity sampler. A host that needs a reproducible
replay should record its inputs. This is stated here, in `ffi/zrecast.h` and
in [UPSTREAM.md](UPSTREAM.md), because the package's other half makes the
opposite promise and the asymmetry would otherwise read as an oversight.

Deliberately out of scope: a steering behaviour library above the crowd, a level
format, a scene graph, or anything that decides *when* to path. Those are a
host's job, and keeping them out is what makes this package reusable.

## Contributing

Issues and pull requests are welcome. Three things to know before opening one:

- **`libs/recastnavigation` is vendored and must not be edited, at all.**
  `ci/verify-vendor.sh` diffs it against a fresh clone of the pinned commit, so
  any local edit fails the build. If upstream needs fixing, fix it upstream; if
  zrecast needs to work around upstream, do it in `ffi/` and record the reason
  in [UPSTREAM.md](UPSTREAM.md).
- **Run `ci/run.sh` before pushing** — or `ci/install-hooks.sh` once, and it runs
  itself. It is the same matrix CI runs, and every step of it must be green.
- **A new binding also adds a line to `tools/bindings.tsv`.** Every public
  Recast and Detour name is either answered there or is a gap, and the evidence
  has to name the symbol in `ffi/zrecast.h` that a host now calls. Then run
  `tools/record.sh`, which rewrites `tools/unbound_*.txt` from that file, the
  upstream harvest and the tranche rules — the record is generated, and
  `ci/check-coverage.sh` fails if what is checked in differs. The gate checks
  that the symbol exists; it cannot check that the evidence is apt, so say what
  the mapping is when it is not obvious.

New source files are added to the explicit lists in `build.zig` deliberately;
there are no globs, so nothing starts compiling by accident.

## Licence

MIT, see [LICENSE](LICENSE), which covers this package's own code. Vendored
recastnavigation is zlib-licensed, copyright Mikko Mononen and contributors;
its licence text ships with the package at `libs/recastnavigation/License.txt`.
