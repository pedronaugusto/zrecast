# Vendored upstream

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

v1.6.0 was upstream's most recent tag when last checked, on 2026-09-03 with
`git ls-remote --tags`; the re-vendoring procedure at the end of this file is
what a newer one calls for.

## What was taken, and what was left behind

Taken: `Recast/`, `Detour/`, `DetourCrowd/`, `DetourTileCache/`, `DebugUtils/`,
`License.txt`, `README.md`, `CHANGELOG.md`.

| Excluded | Size | Reason |
|---|---|---|
| `RecastDemo/` | 1.3 MB | The SDL2/OpenGL sample application. Carries third-party code of its own (`Contrib/fastlz`, `Contrib/stb_truetype.h`, and an imgui fork inlined into `Include/` and `Source/`) and sample meshes under `Bin/Meshes` whose provenance is not stated per file. The tests here generate their geometry instead — see `tests/fixture.cpp`. |
| `Tests/` | 855 KB | Upstream's own suite, plus the copy of Catch2 in `Tests/Contrib/catch2` that it needs. |
| `Docs/` | 584 KB | Doxygen inputs, a CSS theme and images. |
| `CMakeLists.txt` (all) | — | Superseded by `build.zig`. |

Of what was taken, `Recast/`, `Detour/`, `DetourTileCache/` and `DetourCrowd/`
are compiled. `DebugUtils/` is on disk and unbuilt: every function in it takes
a `duDebugDraw*` renderer callback, a drawing interface rather than a
navigation capability, and this ABI hosts none. It is kept because it is cheap
to keep — it depends on nothing but the Recast and Detour headers already here
— and `tools/coverage.sh` claims the directory as deliberately empty, so the
coverage gate sees it as accounted for rather than as a directory nobody looked
at.

Which translation units actually compile is decided explicitly in `build.zig`
(`recast_sources`, `detour_sources`, `detour_tile_cache_sources`,
`detour_crowd_sources`), never by a directory glob.

## What the cook's determinism rests on

Apart from the `qsort` tie order recorded under "Known upstream behaviour"
below, the cook is deterministic by construction once `-ffp-contract=off` is
set, which `build.zig` does for every target. Clang otherwise fuses a multiply
and an add wherever the target has an FMA instruction — mandatory on AArch64,
absent from the x86-64 baseline — and the fused form rounds once where the
separate form rounds twice.

With contraction off, every remaining operation in the bake path is one
IEEE-754 specifies exactly: add, subtract, multiply, divide, `sqrtf`, `floorf`,
`ceilf`, `fabsf`. The one exception was `cosf`, reached through
`rcMarkWalkableTriangles` to turn the slope limit into a threshold. The C
standard does not require a correctly rounded cosine, so zrecast computes that
threshold itself — `zrc::CosDegrees`, a polynomial in double over the bounded
range a slope limit can occupy — and marks the triangles with upstream's own
vector helpers. No other transcendental occurs anywhere in `Recast/Source` or
`Detour/Source`; `DetourMath.h`'s wrappers are query-path only.

The claim stops at the cook. Crowd steering reaches the host's `cosf` and
`sinf` in the adaptive velocity sampler
(`DetourObstacleAvoidance.cpp:499-500`, `:535-536`), so a frame's steering is
not byte-reproducible across C libraries and is not claimed to be —
`ffi/zrecast.h` says the same where the crowd is declared.

The staged pipeline reaches the same substitution. `zrcMarkWalkableTriangles`
and `zrcClearUnwalkableTriangles` are the replacements rather than upstream's
functions, so a mesh a host assembles a stage at a time is the same bytes on
every platform a baked one is. Every other stage the staged entry points call
is upstream's own, and none of them reaches a transcendental.

## Compile-time configuration

Upstream has two macros that change its ABI. Neither is defined here, and both
are guarded:

| Macro | State | Guard |
|---|---|---|
| `DT_POLYREF64` | not defined — `dtPolyRef` is 32-bit | `static_assert` in `ffi/zrecast_abi.cpp` that `ZrcPolyRef` and `dtPolyRef` are the same type. `ffi/zrecast_query.cpp` passes reference arrays straight through with no cast, so anything else is also a compile error at every call site. |
| `DT_VIRTUAL_QUERYFILTER` | not defined — `dtQueryFilter` has no vtable | zrecast never subclasses it; the POD `ZrcQueryFilter` is copied into one per call. |

## The serialised navmesh format

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

## Known upstream behaviour worked around here

Recorded so a future re-vendor can check whether any of it has been fixed, and
so the workarounds are not mistaken for arbitrary defensiveness. Nothing in
this section is patched: each is handled at the boundary, in `ffi/`, or is
documented rather than defended where there is no boundary to defend at.

**Five `qsort` comparators leave ties to the C library, so two C libraries may
cook different bytes from the same input.** `dtCreateNavMeshData` sorts the
tile's bounding-volume items one axis at a time
(`DetourNavMeshBuilder.cpp:145-155`) and serialises the sorted array straight
into the tile image; the comparators (`:40-72`) return 0 for two items whose
minimum on that axis is equal, and `qsort` is not required to be stable. The
contour stage has the same shape with more visible consequences:
`mergeRegionHoles` sorts a region's holes by their leftmost vertex
(`RecastContour.cpp:727`) and the order decides how the holes merge into the
outline, and the diagonal search sorts candidates by squared distance
(`:773`), where equal distances are ordinary on a regular grid and the winner
picks the triangulation.

**Documented, not defended — there is no boundary to defend at.** The sorts
run deep inside upstream's build functions, behind no seam this package can
reach, and rewriting them in the vendored tree would be exactly the local
divergence `ci/verify-vendor.sh` exists to refuse. Any order the ties land in
is a valid navmesh; what varies between C libraries is which valid one comes
out. So the determinism claim is scoped to match: one target cooking twice
gets the same bytes, asserted by the suite on every target that runs it;
targets sharing a C library get the same bytes, gated by `ci/determinism.sh`
across architectures; targets with different C libraries are compared and
reported there, not gated. A host that ships cooked navmeshes should cook on
one platform, or key its asset cache by cook platform — the same position
every engine consuming upstream from C++ is in.

**`dtCrowd::removeAgent` orphans an off-mesh traversal onto the next agent in
the slot.** It clears the agent's `active` flag and touches nothing else
(`DetourCrowd.cpp:572-578`). The traversal itself lives in `m_agentAnims`, a
parallel array `dtCrowd::update` walks by raw pool slot, consulting only that
array's own `active` flag and never the agent's state
(`DetourCrowd.cpp:1411-1447`). `addAgent` hands out the lowest free slot, so an
agent removed part way across a connection leaves a live animation for whoever
takes the slot next: that agent's position is overwritten every frame by a lerp
between coordinates it has never been near, until the dead agent's time budget
runs out. Neither array is reachable from outside `dtCrowd`, and `addAgent`
chooses the slot itself, so there is nothing to intercept.

Worked around by not handing the slot back yet. `zrcCrowdRemoveAgent` retires
the caller's reference immediately, and for an agent in
`DT_CROWDAGENT_STATE_OFFMESH` it calls `resetMoveTarget` and leaves the slot
occupied; `zrcCrowdUpdate` calls upstream's `removeAgent` once the crossing has
finished on its own. The cost is a slot that stays busy for the rest of that
crossing, which `zrcCrowdAddAgent` documents.

**`dtProximityGrid`'s cell hash overflows a signed int.** `hashPos2` multiplies
each cell coordinate by a large prime in `int` arithmetic
(`DetourProximityGrid.cpp:43-46`), and `73856093 * 30` is already past
`INT_MAX`, so any cell coordinate whose magnitude exceeds 29 is signed
overflow — undefined behaviour, not a wrapped value. The coordinate is an
agent's world position divided by the grid's cell size, and `dtCrowd` sizes that cell at
`maxAgentRadius * 3`, about 1.8 m for a half-metre agent, so a world wider than
roughly a hundred metres reaches it.

**Not worked around, and deliberately so.** `dtCrowd::update` calls its own
grid, with positions this package never sees, between two private members;
there is no seam. The alternatives would be to fork the hash or to refuse large
worlds at the door, and refusing them would remove a capability upstream has
rather than bind it — a C++ host is in exactly this position and gets exactly
this behaviour. `ZRC_PROXIMITY_GRID_SAFE_CELL` names the limit in
`ffi/zrecast.h` so a host building with UBSan knows what a trap inside
`hashPos2` is. The standalone grid entry points do not bound it either, for
the same reason: a C++ host passing the same coordinate gets the same
behaviour, and a package that refused it would be answering a different
question from the one upstream answers.

**`dtNavMesh::removeTile` dereferences a null header.** `getTileByRef` resolves
a reference by checking the slot index and the salt and stops there, so it
returns free slots as well as occupied ones — and a free slot's `header` is
null. `removeTile` then reads `tile->header->x` to unhash the tile
(`DetourNavMesh.cpp:1253`) with no test of its own. A freshly created navmesh
gives every slot salt 1, so a reference naming an unused slot of one passes both
of `removeTile`'s checks and faults on the third line. Worked around by
resolving every `ZrcTileRef` through a helper that requires a non-null header
before either `zrcNavMeshRemoveTile` or `zrcNavMeshTileBounds` proceeds.

**`dtNavMesh::getNeighbourTilesAt` walks off the end of the coordinate space.**
It reaches a neighbour with a bare `nx++` / `nx--` / `ny++` / `ny--`
(`DetourNavMesh.cpp:1078-1089`), so a tile whose header carries `INT_MIN` or
`INT_MAX` in either grid coordinate is signed overflow the moment a query looks
for its neighbours. Worked around by bounding both coordinates to
`ZRC_MAX_TILE_COORD` when a tile image is validated, which keeps every such step
far inside range.

**`dtNavMesh::init` reads the buffer before it checks the length.** Its first
two statements dereference `(dtMeshHeader*)data` to test the magic and version;
`dataSize` is never compared against anything. `addTile` then derives eight
array pointers from counts in that header — vertices, polygons, links, detail
meshes, detail vertices, detail triangles, BV nodes, off-mesh connections — and
again never compares them against the length. A buffer shorter than
`sizeof(dtMeshHeader)`, or one whose header describes more data than it carries,
is an out-of-bounds read in every build configuration. Worked around by
`zrcNavMeshValidate`, which copies the header out (so a misaligned caller buffer
is not a struct read either), sanity-checks every count, and requires the total
size implied by those counts to equal the buffer length exactly.

**`maxLinkCount == 0` is an out-of-bounds *write*.** While building the tile's
link free list, `addTile` executes `tile->links[header->maxLinkCount-1].next =
DT_NULL_LINK;` before any loop guard. With a zero count that writes one element
before the array. `zrcNavMeshValidate` rejects it.

**Every index inside a tile image is trusted, and several become memory
errors.** `dtNavMesh::addTile` bounds nothing it reads out of the buffer, and
neither does the query layer that walks the result. Four of these were found by
mutating the image at random and running the full query surface; three of the
four were not visible from reading the validator:

| Field | What Detour does with it |
|---|---|
| `dtPoly::vertCount` | Four functions copy that many vertices into a fixed `float verts[DT_VERTS_PER_POLYGON*3]` stack array — `getPolyHeight` (`DetourNavMesh.cpp:687`), `closestPointOnPoly`, `moveAlongSurface`, `raycast` (`DetourNavMeshQuery.cpp:549`, `:2091`, `:2490`). A value of `200` overwrites the saved return address with coordinates from the image. |
| `dtPoly::neis` | `connectIntLinks` turns it into a link reference, `base \| (neis[j] - 1)`, with no range test (`DetourNavMesh.cpp:549`). Traversal queries then resolve that reference with `getTileAndPolyByRefUnsafe`, which indexes `tile->polys` and checks nothing — the same overrun, one indirection later. The *safe* accessor is used only on the caller's own references. |
| Vertex coordinates | `closestPointOnDetailEdges` keeps its nearest edge with `if (d < dmin)` and calls `dtVlerp` on the result unconditionally (`DetourNavMesh.cpp:673`). Every comparison against a NaN distance is false, so one NaN coordinate leaves the pointers null. |
| Detail edge flags, triangle count | Same null dereference from two more directions: the `onlyBoundary` instantiation skips every triangle with no boundary bit, and a sub-mesh with no triangles never enters the loop at all. |
| BV-tree node `i` | `queryPolygonsInTile` indexes `tile->polys` by a leaf's `i` directly (`DetourNavMeshQuery.cpp:776`). |

All are checked by `zrcNavMeshValidate` before Detour sees the image, and none
of those checks rejects anything `dtCreateNavMeshData` produces.

**A node pool below four nodes has a zero-bucket hash table.**
`dtNavMeshQuery::init` builds `dtNodePool(maxNodes, dtNextPow2(maxNodes / 4))`,
and `dtNextPow2(0)` returns **0** — it decrements to `0xFFFFFFFF`, smears, and
increments back through the wrap (`DetourCommon.h:439`). The pool then allocates
a zero-byte hash table, and its own
`dtAssert(dtNextPow2(m_hashSize) == m_hashSize)` passes because `0 == 0`, so
even an asserts-on build is silent. Every subsequent bucket index is
`hash & (m_hashSize - 1)` — `hash & 0xFFFFFFFF` — so the first search reads and
then *writes* far outside the allocation. Worked around by requiring
`max_nodes >= 4`, which is the smallest value for which the table exists.

**Recast converts a span height to `int` before clamping it.** `rasterizeTri`
computes `(int)ceilf(spanMax * inverseCellHeight)` and only then passes the
result to `rcClamp(..., RC_SPAN_MAX_HEIGHT)` (`RecastRasterization.cpp:445-446`),
so tall geometry over a fine `cell_height` is an out-of-range float conversion
before the clamp ever runs — one vertex at y = 4e8 with the default
`cell_height` of 0.2 is enough. `ffi/zrecast_bake.cpp` bounds the vertical
extent to `RC_SPAN_MAX_HEIGHT` voxels, which is also all a 13-bit span extent
can address: past that Recast silently flattens everything into the topmost
span, so rejecting removes a quietly wrong mesh as well as the undefined
behaviour.

**Recast sums squared voxel deltas in `int`.** `simplifyContour` compares
`dx*dx + dz*dz` against `maxEdgeLen*maxEdgeLen` (`RecastContour.cpp:399`). The
bound that keeps this defined is therefore not the largest integer whose square
fits an `int` but the largest whose square fits *twice over* — 32767, since
`2 * 32767^2` is 2147352578 and `2 * 32768^2` is not representable. Both the
per-axis grid dimension and `maxEdgeLen` are held to it in
`ffi/zrecast_bake.cpp`.

**`rcPolyMeshDetail` does not free itself.** Four of Recast's five container
types — `rcHeightfield`, `rcCompactHeightfield`, `rcContourSet`, `rcPolyMesh` —
release their internal buffers in a destructor, and their `rcFreeX` functions
are one-liners calling `rcDelete`. `rcPolyMeshDetail` has no user-declared
destructor at all; its three buffers are released only inside
`rcFreePolyMeshDetail`. Nothing in the type system distinguishes it, so a
generic `~T(); rcFree(p)` helper leaks three allocations per detail mesh
silently. This is not hypothetical — it is exactly the bug the C smoke test's
balanced-allocation assertion caught during development. Worked around by
`zrc::RcFree`, an overload set that defers to upstream's own free function for
each type rather than re-deriving the rule.

**`rcNew` does not check its allocation.** It is
`new (rcAlloc(...)) T()`, so a failed allocation is a placement-new at address
zero, and `rcAllocHeightfield` and friends inherit that. Replaced by a checked
`zrc::RcNew` in `ffi/zrecast_internal.h`, which is what makes the
`ZRC_ERR_OUT_OF_MEMORY` paths reachable rather than decorative. Detour's
`dtAllocNavMesh` and `dtAllocNavMeshQuery` *do* return null on failure, so they
are used as they are.

**Neither library survives an allocation failure inside itself.** This is the
one limitation that could not be closed from outside, so it is recorded in full
rather than glossed. Four places, all found by sweeping an induced allocation
failure across every allocation site:

| Where | What happens |
|---|---|
| `rcVectorBase::push_back` (`RecastAlloc.h:214`) | `allocate_and_copy` returns null when `rcAlloc` fails, and `construct(data + m_size, value)` then placement-news through it. A bake under memory pressure is a null dereference, not an error. |
| `createBVTree` (`DetourNavMeshBuilder.cpp:175`) | Allocates its item array and indexes it with no check, so `dtCreateNavMeshData` faults the same way. |
| `dtNodePool`, `dtNodeQueue` (`DetourNode.cpp:64`, `:163`) | Check their allocations with `dtAssert` and then `memset` the result. Under `NDEBUG` that is a null `memset`; with asserts on it is an abort. Building a query object is therefore not recoverable either. |
| `~dtNavMesh` (`DetourNavMesh.cpp:211`) | Walks `m_tiles` for `m_maxTiles` entries. `dtNavMesh::init` sets `m_maxTiles` *before* allocating that array and `memset`s it only after a second allocation also succeeds, so after an out-of-memory failure the destructor either dereferences null or reads uninitialised tile flags and calls `dtFree` on the garbage behind them. |

The last one *is* worked around, narrowly: `FreeFailedNavMesh` in
`ffi/zrecast_navmesh.cpp` releases the object's storage without running its
destructor when `init` reports `DT_OUT_OF_MEMORY`, which turns a certain crash
into at worst one leaked `dtMeshTile` array on an already-fatal path. Detour
distinguishes the cases for us: its only other `init` failure happens after both
arrays are live, where the destructor is safe.

The other three are not worked around, because they cannot be. What zrecast
does instead is keep its own allocations correct and *prove* it: the
`a failure at any allocation zrecast owns` test sweeps every allocation site in
the serialise/validate/deserialise arc and requires an error, the documented
error, and balanced bytes — with exactly one index permitted to leak, that being
the `~dtNavMesh` residue above. The bake and the query object are outside the
sweep for the reasons in the table, and the test says so at the point where it
skips them.

The practical consequence for a host: **run the bake where an allocation failure
is not survivable anyway** — a cook, a tool, a loading screen with a budget —
and do not expect Recast to degrade gracefully under memory pressure. The
runtime side (deserialise, query) is far better behaved, and that is the half a
game runs.

**Recast reports failure as a bare `false`.** The reason goes to a virtual log
callback on `rcContext`, and there is no default that records it. A caller
without a context subclass therefore learns only that some stage of eleven
failed. Worked around by `ZrcBakeLog`: `ffi/zrecast_bake.cpp` installs a context
that captures the messages into a caller-supplied buffer.

**`dtNavMesh::calcTileLoc` casts an unbounded quotient to `int`.** It computes
`(int)floorf((pos[0] - m_orig[0]) / m_tileWidth)` and the same for z
(`DetourNavMesh.cpp:1063-1067`), with no check on either. A position far enough
from the origin — or a degenerate tile size, which makes the quotient infinite —
is a float-to-int conversion whose value does not fit, which is undefined
behaviour rather than a wrong answer. This is the same defect class as the grid
extent recorded above, in a function a host calls directly. Worked around by
doing the division in `double` first and refusing a position whose tile could
not be named by an `int`; a non-finite position and a NULL `pos` are refused at
the same point.

**Nothing bounds an off-mesh connection's polygon index.** `baseOffMeshLinks`
does `dtPoly* poly = &tile->polys[con->poly];` (`DetourNavMesh.cpp:571`) with no
comparison against `polyCount`, then writes through
`tile->verts[poly->verts[0] * 3]` (584-585); `connectExtOffMeshLinks` repeats
both (`DetourNavMesh.cpp:468`, `:485-486`). `con->poly` is an `unsigned short`
read straight out of the tile image, so this is an out-of-bounds **write**
primitive reachable from tile bytes, not merely a wrong answer. Worked around
by requiring `con[i].poly == offMeshBase + i` exactly when an image is validated — the
builder writes them from the same counter (`DetourNavMeshBuilder.cpp:660-662`),
so the equality covers the bound, the ordering, and two connections claiming one
polygon at once.

**`getOffMeshConnectionByRef` underflows an unsigned subtraction.** It computes
`const unsigned int idx = ip - tile->header->offMeshBase;`
(`DetourNavMesh.cpp:1521`) with no check that `ip >= offMeshBase`, guarded only
by a `dtAssert` that `DetourAssert.h:25-29` compiles to `(void)sizeof(x)` under
`NDEBUG` — nothing at all in a release build. A polygon below `offMeshBase`
tagged `DT_POLYTYPE_OFFMESH_CONNECTION` therefore returns
`&tile->offMeshCons[huge]`. `dtPoly::getType()` is a 2-bit field with two
undefined values, and nothing upstream constrains it. Worked around by a
two-sided rule when an image is validated: a polygon is typed off-mesh **iff**
its index lies in `[offMeshBase, offMeshBase + offMeshConCount)`, and neither
undefined type value is accepted. `getOffMeshConnectionPolyEndPoints` reads
`tile->verts[poly->verts[idx]*3]` (1495-1496) with no bound of its own either;
the same validation pass bounds those two vertex indices.

**The BV-tree query does not filter off-mesh polygons; the linear one does.**
`queryPolygonsInTile`'s no-tree branch skips them explicitly
(`DetourNavMesh.cpp:869-871`), and its BV branch returns any leaf that passes the
overlap test with no type test at all (808-860). Both `baseOffMeshLinks` and
`connectExtOffMeshLinks` resolve their endpoints through it, so a doctored tree
could make a connection land on another connection's polygon rather than on
ground. `createBVTree` only ever emits ground leaves
(`DetourNavMeshBuilder.cpp:200`), so the workaround is to bound a leaf by
`offMeshBase` rather than by `polyCount`.

**Tile state is restored against a polygon count the blob does not carry.**
`dtTileState` holds a magic, a version and a tile ref and nothing else
(`DetourNavMesh.cpp:1363-1368`), so `restoreTileState` sizes its `dtPolyState`
array from the **live tile's** `polyCount` and accepts any buffer at least that
large (1422-1450). A blob taken from a tile that has since been replaced by a
differently shaped one therefore passes every check upstream makes. Worked
around by requiring the caller's length to equal `getTileStateSize` exactly
rather than merely reach it, which is the only remaining signal that a blob and
a tile disagree. Note also the asymmetry, mirrored deliberately rather than by
accident: upstream reports a short buffer as `DT_BUFFER_TOO_SMALL` from `store`
and as `DT_INVALID_PARAM` from `restore`.

**An area id past the end of its field is truncated, not refused.** Both halves
of the library store an area in six bits — `rcSpan::area` on the Recast side
(`Recast.h:298`), `dtPoly::areaAndtype` on the Detour side — and neither
validates what it is given. `dtPoly::setArea` masks with `& 0x3f`
(`DetourNavMesh.h:178`), and `dtNavMesh::setPolyArea` hands it any
`unsigned char` unchecked, so area `64` silently becomes area `0`, the id that
means unwalkable. `rcMarkBoxArea` and its two siblings take the same
unchecked `unsigned char`. The limit is documented in both headers and enforced
in neither. Worked around by refusing an id outside `[0, ZRC_MAX_AREAS)` in
`ValidateAreaAuthoring` and in `zrcNavMeshSetPolyArea`, so a host learns it
asked for something impossible rather than silently getting a different area.

**The bounding-volume tree is walked one node past its end, and that node
names polygon 0.** `dtCreateNavMeshData` reserves `polyCount * 2` nodes
(`DetourNavMeshBuilder.cpp:507`) while `createBVTree` fills exactly
`2 * polyCount - 1` of them — `subdivide` emits one node per call and either
makes a leaf or splits into two children, so there are no unary nodes and the
count is fixed. The last reserved node keeps the zeroes the whole buffer was
memset with. `queryPolygonsInTile` then ends its traversal at
`&bvTree[bvNodeCount]` rather than at the tree
(`DetourNavMeshQuery.cpp:744`), so it reads that node: an `i` of 0 is a leaf
naming polygon 0, and bounds of `{0,0,0}..{0,0,0}` overlap any query box whose
quantised minimum is 0 on all three axes.

That is not an exotic query. Every box is clamped to the tile's bounds before
being quantised (`:750-757`), so *any* query at or below the tile's minimum
corner — including every query that misses the tile entirely and is clamped
onto it — matches. Measured on the test fixture: `findNearestPoly` at the
navmesh's own minimum corner with 0.05 m half-extents returned polygon 0, whose
nearest point is nine metres away. `zrcFindNearestPoly` had that behaviour from
the day it was bound.

Worked around by `zrc::SealBvSentinel`, which marks that node internal — the
traversal reports a polygon only for a node with a non-negative `i`, and steps
past this one either way. It runs on every tile this package cooks, so an image
handed straight to Detour is protected too, and again on the private copy taken
when a tile is added or a navmesh deserialised, so an image from another tool is
protected without being altered where it lies. It costs one `int` per tile of
the cooked image.

**`dtAlign4` overflows for the top three values of its range.** It is
`(x + 3) & ~3` on a signed `int` (`DetourCommon.h:463`), so any `x` above
`INT_MAX - 3` is signed overflow. Zig's undefined-behaviour sanitizer traps it,
which is how it was found rather than reasoned about. Nothing inside Detour
reaches it — every call site derives `x` from a tile count already bounded by
image validation — but the function is public, and a host aligning its own
sizes with it can. `src/vec.zig`'s port uses a wrapping add, so the Zig
spelling is defined over the whole range; the C is not, and the parity test
stops comparing against it at `INT_MAX - 3` for that reason.

**`dtNodePool::getNodeAtIdx` bounds only the zero.** It tests `if (!idx)` and
otherwise indexes straight into the pool's storage (`DetourNode.h:68-72`), so
an index between the node count and the pool's capacity returns a pointer to
memory no search has written. `zrcQueryNodeAt` bounds against the count the
pool itself reports rather than its capacity, so an index past what a search
populated is `ZRC_ERR_NOT_FOUND`.

**`findNearestPoly` writes `isOverPoly` only when it also writes
`nearestPt`.** Both are `[opt]` in the header, but the second is written inside
the branch guarded by the first (`DetourNavMeshQuery.cpp:722-725`) — so a
caller wanting to know whether the point is over the polygon, and not wanting
the point, is told false whatever the answer. `zrcFindNearestPoly` passes
scratch of its own when the caller declines the point, which is what makes the
two outputs independent.

**A random point near a position is not within the radius given.**
`findRandomPointAroundCircle` bounds its *search* by `maxRadius` and then
places the point anywhere inside whichever polygon it settled on
(`DetourNavMeshQuery.cpp:478-492`). A polygon reached at the edge of the circle
extends past it, so the point can too. Not worked around — the alternative
would be rejection sampling with a caller's own generator — but stated in the
header, because the parameter name invites the opposite reading.

**A sliced path search keeps the caller's filter as a raw pointer.**
`initSlicedFindPath` copies the start and end positions into its query state
and stores `m_query.filter = filter` (`DetourNavMeshQuery.cpp:1230-1233`);
every later `update` and `finalize` dereferences it. A host that passes a
stack filter — which is what every other query in the API invites — has a
dangling pointer the moment `init` returns. Worked around by giving
`ZrcNavMeshQuery` a `dtQueryFilter` of its own that lives as long as the slice,
so the boundary keeps the same copy-by-value contract everywhere.

**"No impact on an in-progress sliced path query" is not true.** The class
comment says const member functions are safe to call alongside a slice
(`DetourNavMeshQuery.cpp:133-135`). Five of them begin by clearing the shared
node pool: `findPath` (`:1003`), `findPolysAroundCircle` (`:2742`),
`findPolysAroundShape` (`:2919`), `findRandomPointAroundCircle` (`:340`) and
`findDistanceToWall` (`:3487`). `dtNodePool::clear` resets the bucket table and
the count and never touches the node storage (`DetourNode.cpp:83-87`), so the
slice's `lastBestNode` stays a valid pointer into nodes the next search hands
out and overwrites. Finalising then walks a chain belonging to a different
search and returns plausible, wrong references. The obvious game-loop shape —
one query object, iterate agents — walks straight into it. Worked around by
refusing those five while a slice is in flight, with
`ZRC_ERR_SEARCH_IN_PROGRESS`. Only those five: `raycast`, `moveAlongSurface`
and `findLocalNeighbourhood` use the separate tiny pool and `findRandomPoint`
runs no search at all, so refusing them would be a restriction the defect does
not justify.

**Finalising a sliced path twice succeeds and answers nothing.** Both finalise
functions `memset` their query state to zero on the way out
(`DetourNavMeshQuery.cpp:1579`) and guard re-entry with
`dtStatusFailed(m_query.status)` — which is false for the zero that memset just
wrote. A second call therefore reaches the `startRef == endRef` special case,
both being 0, and returns `DT_SUCCESS` with a one-element path holding the null
reference (`:1516-1520`). A real `init` can never leave `startRef` at 0, so the
state is reachable only this way. Closed by the in-flight flag: with it
cleared, a second finalise is `ZRC_ERR_NO_SEARCH` and upstream's zeroed path is
never entered.

**`findLocalNeighbourhood` writes through a null result pointer.** It writes
`resultRef[n]` with no test (`DetourNavMeshQuery.cpp:3135`, `:3252`) while both
of its siblings guard the identical write with `if (resultRef)` (`:2784`,
`:2964`). Its own validation block never checks the pointer either. The header
does not mark that parameter `[opt]`, so the omission may be deliberate — but a
caller asking only for a count gets a null write rather than a refusal.
`zrcFindLocalNeighbourhood` requires the array.

**`findDistanceToWall` reports "nothing found" as a hit.** `hitPos` is written
only inside the branch that finds a closer edge, so with no wall in range it
keeps whatever the caller passed in. The function then computes
`dtVsub(hitNormal, centerPos, hitPos)` and `dtVnormalize(hitNormal)` over it
regardless and returns `DT_SUCCESS` (`DetourNavMeshQuery.cpp:3648-3652`); the
only signal is the convention that `*hitDist` equals `maxRadius`. Separately,
`dtVnormalize` has no zero-length guard, so an agent standing exactly on a wall
normalises a zero vector and gets NaN in all three components. Worked around by
seeding both outputs, reporting nothing-in-range through its own flag, and
zeroing a normal that comes back non-finite.

**`isValidPolyRef` dereferences its filter with no null check.** Every other
filtered method in the file tests it — `findPath` (`:991`),
`findPolysAroundCircle` (`:2737`), `findPolysAroundShape` (`:2910`),
`findLocalNeighbourhood` (`:3108`), `findDistanceToWall` (`:3482`),
`getPolyWallSegments` (`:3330`). `isValidPolyRef` alone does not (`:3663`), so
a NULL filter is an immediate crash. Here a NULL filter is a different
question, answered by `dtNavMesh::isValidPolyRef`: whether the reference names
a live polygon at all.

**`dtRaycastHit::hitEdgeIndex` can come back uninitialised.** `raycast` writes
it once per polygon step (`DetourNavMeshQuery.cpp:2534`), but returns early
when the *first* polygon's intersection test fails (`:2527-2532`) — and
neither it nor its two-overload wrapper zeroes the field first. In that path
the caller reads back whatever its own struct held. Seeded to -1 before the
call.

**`getTilesAt` truncates silently at 32 tiles per column.** `queryPolygons`
declares `MAX_NEIS = 32` and passes it as the capacity
(`DetourNavMeshQuery.cpp:944, 951`); `dtNavMesh::getTilesAt` walks the whole
bucket chain but appends only `if (n < maxTiles)` (`DetourNavMesh.cpp:1133`),
with no overflow signal — its own comment says it "will simply fill the array
to capacity". A grid column stacked more than 32 layers deep therefore drops
the rest from every query that touches it. Not worked around, because the
capacity is upstream's own and a layered navmesh that deep is outside what this
version builds; stated in the header instead.

**`encodePolyId` truncates a field that does not fit.** It shifts and ors
without checking any of the three values against the bit widths
`dtNavMesh::init` computed (`DetourNavMesh.h`), so a salt or tile index one bit
too wide silently becomes a reference to something else. `zrcEncodePolyRef`
recomputes those widths from the navmesh's own parameters and refuses.

**A convex polygon with one vertex is read from before its start.**
`dtRandomPointInConvexPoly` fan-triangulates with `for (int i = 2; i < npts;
i++)` and initialises its chosen triangle to `npts - 1`
(`DetourCommon.cpp:337-345`). Given `npts == 1` neither loop body runs, `tri`
stays 0, and the function then evaluates `pts[(tri - 1) * 3]` — three floats
below the caller's array — before reading `pts[tri * 3]` on top of it. Worked
around by requiring at least three vertices at the boundary, which is also the
smallest polygon the fan has a defined answer for.

**`dtCalcPolyCenter` is handed an index array and no bound for it.** Its
signature is `(tc, idx, nidx, verts)` — there is no vertex count anywhere in it,
so every value in `idx` is trusted as an offset into `verts`
(`DetourCommon.cpp:186-202`). It then divides by `nidx` with no test, so an
empty polygon yields `1.0f / 0` multiplied by a zero accumulator, which is NaN
in all three components rather than an error. `zrcPolyCenter` takes the vertex
count as a real parameter, bounds every index against it, and requires at least
one index.

**An empty polygon is an out-of-bounds read in the overlap test.**
`dtOverlapPolyPoly2D`'s helper opens with
`rmin = rmax = dtVdot2D(axis, &poly[0]);` before any loop
(`DetourCommon.cpp:275`). The outer loops are
guarded by their own counts, but the second one projects the *first* polygon on
every iteration — so `npolya == 0` with a non-empty second polygon reads three
floats from an empty array. Both counts are checked at the boundary.

**`rcOffsetPoly` needs one more slot than it fills.** Its bounds checks are
`if (n + 2 >= maxOutVerts)` on the bevelled path and `if (n + 1 >= maxOutVerts)`
on the other (`RecastArea.cpp:496, 510`), where every comparable bound in the
tree is `>` rather than `>=`. The last slot of the caller's buffer is therefore
never written and a buffer sized exactly to the result returns 0, which is also
how the function reports failure. Not worked around — a fix would change which
inputs succeed — but stated in the header, and the zero return is given the name
`ZRC_ERR_BUFFER_TOO_SMALL` so it is at least distinguishable from a result.

**The assertion hook does not exist in a release build.** In both
`DetourAssert.h` and `RecastAssert.h` the whole family — the
`dtAssertFailFunc` / `rcAssertFailFunc` typedef, the `SetCustom` setter and the
`GetCustom` getter — sits inside the `#else` of `#ifdef NDEBUG`, and `dtAssert`
/ `rcAssert` expand to `do { (void)sizeof(x); } while(...)`. So a host cannot
route upstream's assertions anywhere in a build that compiled them out, and
nothing in the API says which build it is holding. `zrcSetAssertHandler` still
records the handler in either build, and `zrcAssertsEnabled` is the answer to
the question upstream leaves unanswerable.

**A bake that produces nothing is not an error.** `rcBuildPolyMesh` succeeds
with `npolys == 0`, and the resulting navmesh answers every query with "nowhere
to go" rather than failing. Worked around by returning `ZRC_ERR_EMPTY_RESULT`,
with an explanation written to the bake log.

**A build stage overwrites its output container without freeing it.**
`rcBuildCompactHeightfield`, `rcBuildContours`, `rcBuildPolyMesh` and
`rcBuildPolyMeshDetail` assign fresh allocations straight over their
destination's members with no emptiness test at all.
`rcBuildHeightfieldLayers`, `rcCopyPolyMesh` and `rcMergePolyMeshes` attempt the
test with `rcAssert` (`RecastMesh.cpp:1491-1495` for the copy), which
`RecastAssert.h:25` compiles to `(void)sizeof(x)` under `NDEBUG` — so in a
release build every one of them leaks the buffers the container already held.
Unreachable while the only way to run a stage was a whole bake, and reachable
the moment a host owns the containers, so every staged entry point tests the
destination itself and returns `ZRC_ERR_ALREADY_BUILT` in every build
configuration.

**`rcErodeWalkableArea` stops eroding at a radius of 128.** The threshold is
`const unsigned char thr = (unsigned char)(radius*2);` (`RecastArea.cpp:210`),
so radius 128 wraps to 0 and erodes nothing, and 129 erodes as 1 would. The
distances it compares are also saturated at 255
(`RecastArea.cpp:111`), so nothing above 127 could be honoured anyway. Bounded
to 127 at the boundary, in both the bake and the staged entry point.

**`rcBuildRegions` reads a distance field that may not exist.** The watershed
partitioner reaches `chf.dist` through `floodRegion`, `expandRegions` and
`sortCellsByLevel` (`RecastRegion.cpp:339, 381, 494`), and `chf.dist` is null
until `rcBuildDistanceField` has allocated it. Nothing in `rcBuildRegions`
checks. `rcBuildRegionsMonotone` and `rcBuildLayerRegions` do not read it and
are unaffected. The staged entry point refuses the watershed strategy on a field
whose distance array has not been built.

**`rcCreateHeightfield` accepts a negative size.** It stores `sizeX` and `sizeZ`
straight into the heightfield and allocates `sizeof(rcSpan*) * width * height`
(`Recast.cpp:306-325`). Two negative sizes multiply to a positive product, so
the allocation succeeds at the wrong size and every later `spans[x + z * width]`
indexes an array that does not have that shape. Bounded at the boundary.

**`rcAddSpan` indexes the column array unbounded.** Its helper computes
`columnIndex = x + z * hf.width` and dereferences `hf.spans[columnIndex]`
(`RecastRasterization.cpp:122-124`) with no test against the field's own extent.
Unreachable from a bake, which derives every coordinate itself, and reachable
from a host adding spans by hand. Both coordinates are checked at the boundary.

**`rcMergePolyMeshes` uses the first mesh's polygon stride for all of them.**
`mesh.nvp` is taken from `meshes[0]` and then used as the stride into every
input: `src = &pmesh->polys[j * 2 * mesh.nvp]` (`RecastMesh.cpp:1424`). An input
that packs its polygons at a smaller `nvp` is read past its end, one polygon at
a time. The boundary requires every input to agree on `nvp`.

**`rcMergePolyMeshDetails` lacks the guard its twin has.**
`rcMergePolyMeshes` opens with `if (!nmeshes || !meshes) return true;`
(`RecastMesh.cpp:1312`); the detail-mesh merge has no such line and goes
straight to `meshes[i]->nverts` (`RecastMeshDetail.cpp:1401-1402`). A null array with
a positive count is a null dereference. Checked at the boundary.

**`rcContext::log` can produce two entries from one call.** A formatted message
of 512 bytes or more is reported first as a separate `RC_LOG_ERROR` reading
"Log message was truncated" — at a category the caller did not choose — and then
as the message cut to 511 bytes (`Recast.cpp:58-79`). Not worked around: it is
upstream's contract for a host's own logging hook. Stated in the header, because
a host counting entries or trusting the category would otherwise be wrong.

**The tile cache reads a layer header before it checks the length.**
`dtTileCache::addTile` casts `data` to a `dtTileCacheLayerHeader*` and tests its
magic and version (`DetourTileCache.cpp:250-254`) with `dataSize` compared
against nothing, then stores `compressedSize = dataSize - headerSize`
(`:283-284`), which is negative for a short buffer and is handed to the host's
codec as an `int`. `dtDecompressTileCacheLayer` repeats the same subtraction
(`DetourTileCacheBuilder.cpp:2202-2203`), and
`dtTileCacheHeaderSwapEndian` takes a length, discards it with
`dtIgnoreUnused`, and dereferences the header anyway
(`DetourTileCacheBuilder.cpp:2223-2226`). All three lengths are checked at the
boundary before upstream sees the bytes.

**`buildNavMeshTile` bounds a tile index with `>` where the file uses `>=`
everywhere else.** `if (idx > (unsigned int)m_params.maxTiles)`
(`DetourTileCache.cpp:662-663`) admits one `dtCompressedTile` past the
allocation; if that slot's garbage salt happens to match the reference's, its
`data` and `dataSize` are read and fed onward. Every reference is resolved
through upstream's own salt-checking `getTileByRef` before anything can reach
it.

**Two index accessors bound nothing at all.** `getTile(int)`
(`DetourTileCache.h:112`) and `getObstacle(int)` (`:115`) each return
`&array[i]` with no test against the count the cache was created for. The
boundary bounds both.

**`queryTiles` drops what does not fit and reports success.** It writes only
`if (n < maxResults)` and returns `DT_SUCCESS` either way
(`DetourTileCache.cpp:511-520`). Worse, `update` calls it with the fixed
eight-entry `DT_MAX_TOUCHED_TILES` array (`:548`), so an obstacle overlapping
more than eight tiles is carved into the first eight and silently missing from
the rest — a hole in the navmesh nothing reports. The boundary reports the true
count from `zrcTileCacheQueryTiles`, and refuses an obstacle that would touch
more tiles than upstream can track.

**A full obstacle table reads as an allocator failure.**
`dtTileCache::addObstacle` returns `DT_FAILURE | DT_OUT_OF_MEMORY` when the
free list is empty (`DetourTileCache.cpp:371-372`), which is the cache being
full rather than the allocator failing. The boundary counts the occupied slots
itself and reports ZRC_ERR_NAVMESH_FULL, the same distinction
`zrcNavMeshAddTile` already draws.

**`dtTileCache::init` has no purge.** It assigns every member and allocates
both arrays (`DetourTileCache.cpp:118-171`) with no check that a previous call
left anything behind, so calling it twice leaks every array from the first. A
ZrcTileCache is created once and destroyed once.

**The mesh-process callback is handed the whole tile description.**
`dtTileCacheMeshProcess::process` receives a mutable `dtNavMeshCreateParams*`
(`DetourTileCache.h:95-99`), and the tile it produces goes into the navmesh
through `dtCreateNavMeshData` and `dtNavMesh::addTile` directly
(`DetourTileCache.cpp:756-773`) — the one path into a navmesh that skips every
check `zrcNavMeshAddTile` applies. A callback that changed `polyCount` or a
buffer pointer would produce exactly the malformed tile that validator exists
to catch. Worked around by narrowing the callback: `ZrcTileCacheBuildParams`
shows a host the areas, the flags, the user id and the off-mesh connections,
validates what comes back, and writes it into the params itself.

**`walkableClimb / ch` is an unbounded float-to-int conversion.**
`buildNavMeshTile` computes `(int)(m_params.walkableClimb / m_params.ch)`
(`DetourTileCache.cpp:673`) with neither value bounded against the other. The
ratio is checked when the cache is created.

**Tile-cache tiles carry no bounding-volume tree.** `params.buildBvTree` is
set false unconditionally (`DetourTileCache.cpp:750`), so every query against
one falls back to a linear scan of the tile. Not a defect — it is why tile size
matters — but it is also why `SealBvSentinel` is not called on this path and
does not need to be: `dtCreateNavMeshData` gives such a tile `bvNodeCount == 0`
(`DetourNavMeshBuilder.cpp:507`), and there is no spare node to seal.

**`getTilesAt` truncates the same way `queryTiles` does.** It writes only
while `n < maxTiles` and returns the count it wrote
(`DetourTileCache.cpp:173-193`), so a caller cannot tell a full answer from a
clipped one. The boundary asks for the cache's whole tile capacity, then
applies the caller's limit itself and says when it did not fit.

**`dtTileCache::removeTile` dereferences a header it has not checked.** Having
validated the slot index and the salt it goes straight to
`computeTileHash(tile->header->tx, tile->header->ty, ...)`
(`DetourTileCache.cpp:301-306`) — and every slot starts at salt 1, so a
reference naming a never-issued slot of a fresh cache passes both checks and
faults on the third line. Exactly the shape `dtNavMesh::removeTile` has, and
worked around the same way: every reference resolves through a helper that
requires an occupied slot first.

**An obstacle's slot is never observed empty through a reference.**
`dtTileCache::update` sets `ob->state = DT_OBSTACLE_EMPTY` and bumps
`ob->salt` in the same block (`DetourTileCache.cpp:618-624`), and
`getObstacleByRef` rejects on the salt before it ever looks at the state
(`:228-240`). So `DT_OBSTACLE_EMPTY` is an enumerator no reference-based
lookup can return. Not a defect, but it makes one of upstream's four states
unreachable through the accessor that reports the other three, which the header
says rather than leaving a host to discover.

**Two tile-cache allocators memset before checking the allocation.**
`dtAllocTileCacheContourSet` and `dtAllocTileCachePolyMesh` both write
`memset(p, 0, sizeof(*p))` on the line after `alloc->alloc(...)` with no null
test (`DetourTileCacheBuilder.cpp:66-73, 86-93`), so an allocator under
pressure is a write to address zero. Not worked around — the two are the only
way to obtain those containers, and reimplementing them would be a second copy
of a format upstream owns — but stated here and in the header, and reachable
only through a host allocator that fails.

**`dtFreeTileCacheLayer` has no null guard, unlike its two siblings.**
`dtFreeTileCacheContourSet` and `dtFreeTileCachePolyMesh` both open with
`if (!cset) return;`; the layer version goes straight to `alloc->free(layer)`
(`DetourTileCacheBuilder.cpp:2156-2161`). Guarded at the call site.

**`rcBuildHeightfieldLayers` computes negative grid dimensions.** It takes
`lw = chf.width - borderSize*2` and `lh = chf.height - borderSize*2`
(`RecastLayers.cpp:497-498`) with no relationship check between the two. A
border wider than half the field makes both negative, and the allocation size
`lw*lh` is then a positive, meaningless product while every write loop, bounded
by the same negative values, writes nothing. Not memory-unsafe, but a layer set
that reports dimensions no geometry has. Refused at the boundary.

### What the sanitizer found: nothing

Zig's C undefined-behaviour sanitizer is enabled in Debug and runs over the
whole vendored tree. Across all four optimize modes it reports no undefined
behaviour for the workload the test suite exercises. That is a statement about
this workload, not a clean bill of health for Recast and Detour — but it is why,
unlike some vendored C++, nothing here needs the sanitizer switched off to pass.

## Re-vendoring procedure

`ci/verify-vendor.sh` fetches the pinned commit and diffs it against `libs/`,
so the claim about what this copy is gets checked rather than asserted. It
runs as its own CI job. Run it after any step below.

1. Clone upstream at the new tag; copy the directories listed above over
   `libs/recastnavigation/`, re-applying the exclusions.
2. Update the table at the top of this file and `zrcRecastVersion()` in
   `ffi/zrecast_core.cpp`.
3. `zig build test`. The `static_assert`s in `ffi/zrecast_abi.cpp` fail the
   build if a mirrored constant, a serialised-image struct size, or the polygon
   reference type has changed; the ABI test fails if the Zig externs have
   drifted.
4. If `DT_NAVMESH_VERSION` changed, say so here and expect every previously
   serialised navmesh to be rejected with `ZRC_ERR_UNSUPPORTED_VERSION`.
5. Re-read the "Known upstream behaviour" list above and delete anything that
   has been fixed upstream, along with its workaround. The
   `tools/doc_numbers.txt` lines that cite the pinned commit are re-measured at
   the same time — each names the upstream file its value sits in.
6. Add any new source files to `build.zig` deliberately — the explicit lists
   exist so a re-vendor cannot silently change what gets compiled.
