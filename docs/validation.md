# Validation at the boundary

Recast and Detour assert in debug and read on regardless in release. `ffi/`
checks an index, a length or a reference before upstream sees it. This file
lists what is checked and why; the summary is in the README.

## Geometry

Recast indexes `verts[tris[i] * 3]` with no range test of its own. An index
past the end of the vertex array is an out-of-bounds read inside the rasteriser
in every build configuration. A NaN coordinate is worse: `rcCalcBounds`
propagates it into the grid size, and the heightfield allocation is then
computed from garbage. Both are rejected at the door.

## The spare bounding-volume node

A tile reserves two bounding-volume nodes per polygon and the tree fills
`2n - 1` of them. Detour's own traversal ends one node past the tree and reads
the spare, zeroed one as a leaf naming polygon 0. A box of all zeroes overlaps
every query clamped to the tile's minimum corner, so `findNearestPoly` at a
navmesh's own corner came back with a polygon nine metres outside the
half-extents it was given. Nothing faults; the answer is simply wrong. That
node is sealed as an internal node in every image this package cooks and in
every image it loads. See [UPSTREAM.md](../UPSTREAM.md).

## Serialised navmeshes

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
about and gotten wrong. That fuzzer is a test in the suite now — deterministic,
sized so every CI leg can afford it, and asserting that a meaningful share of
its mutants get past validation and into the queries, so it cannot decay into
an exerciser of the magic check. The development campaign behind the checks was
larger; the in-suite run is the regression net.

Neither library is hardened against arbitrary hostile input, and zrecast cannot
make them so from the outside. The fuzzer found three holes after the first
round of hand-written checks, so a fourth is possible. Load a navmesh from an
untrusted source behind a length-and-signature container, and validate that
first.

## Out of memory

zrecast's own allocations are checked, rolled back correctly on every path, and
tested by inducing a failure at every allocation site in turn.

Upstream's are not. Recast's own vector placement-news into the result of a
failed allocation; Detour's BV-tree builder indexes an unchecked array; and its
node pool asserts and then writes through the null. None of that is fixable
from outside the library, so it is documented in [UPSTREAM.md](../UPSTREAM.md)
with file and line rather than papered over. One case — a `dtNavMesh` whose
`init` ran out of memory, which cannot then be destroyed — is worked around
narrowly, and the bounded cost of that workaround is stated there.

Run the bake somewhere an allocation failure is fatal anyway, which is where a
cook already lives. The runtime half behaves far better, and that is the half a
game links.

## The ABI guard

The Zig side hand-writes `extern struct`s mirroring `zrecast.h`. Nothing in
either compiler checks those two declarations still agree — a field reordered
on one side and not the other is silent corruption. `zrcAbiLayout()` reports
what the C++ actually compiled to, and a test asserts every size and offset
against the Zig declarations.

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

## Build hygiene

- Source lists are explicit, never globs — a re-vendor cannot silently change
  what compiles.
- No `-fno-access-control`. The FFI layer uses only public upstream API.
- UBSan is not blanket-disabled. It stays on in Debug (`-Dsanitize_c`), and the
  suite passes with it on in all four optimize modes — see
  [UPSTREAM.md](../UPSTREAM.md) for what that does and does not prove.
- `-fno-exceptions` / `-fno-rtti` everywhere except the MSVC ABI, where
  disabling them through Clang flags breaks the Microsoft standard library
  headers. Neither Recast nor Detour contains a single `throw`, `try`,
  `dynamic_cast` or `typeid`, so nothing is lost.
- Build options are declared once and mirrored into a Zig `options` module, so
  the wrapper cannot disagree with how the C++ was compiled.
- One translation unit per concern on both sides of the boundary.
