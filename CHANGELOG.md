# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html); before
1.0 the minor is the breaking one. Each entry says what the old shape could
not express, so a port has the reason and not only the diff.

## [Unreleased]

### Breaking

- Every call that can fail takes a `*zrecast.Failure` as its last argument,
  and a refusal pushes a `zrecast.Frame` on it: the `zrecast.Call` that
  refused and the result. An error alone said what went wrong but not which
  call, and a bake, a load or a crowd update runs several. Declare one with
  `var failure: zrecast.Failure = .{};` and pass `&failure`. `TriMesh.toC` and
  `AreaAuthoring.toC` also take the call they prepare for.
- `setAssertHandler` returns `void` and `assertHandler` returns `?AssertHandler`:
  neither can be refused from Zig, so their error unions were never an
  answer.

- Builds on Zig 0.17.0, which `build.zig.zon` now requires. The wrapper walks
  0.17's reflection lists and `main` takes `std.process.Init`.
- `-Dsanitize_c` is off by default, no longer tied to `optimize`. Zig's C
  sanitizer emits calls into a runtime linked only into a compilation that is
  itself sanitized, so defaulting it on in Debug handed a consumer who forgot
  to forward `optimize` a link failure naming a `__ubsan` symbol.
- The functions that parse bytes take a `zrecast.Image`, aegis's
  untrusted-input type: `NavMesh.initFromBytes`, `NavMesh.addTile`,
  `NavMesh.restoreTileState`, `validate`, `tileLayout`, `swapImageEndian`,
  `TileCache.addTile` and `TileCacheLayer.initFromBytes`. Wrap the bytes with
  `.init(bytes)`.
- A handle another object owns is lent as a borrowed type:
  `Crowd.grid`, `Crowd.pathQueue` and `Crowd.navMeshQuery`,
  `PathQueue.navMeshQuery` and `NavMeshQuery.attachedNavMesh` return
  `BorrowedProximityGrid`, `BorrowedPathQueue`, `BorrowedNavMeshQuery` and
  `BorrowedNavMesh`. They returned the owning types, built by casting away the
  boundary's `const`, so a caller could free the crowd's own grid, a double
  free, or clear and fill it.
- A query filter is a handle: `QueryFilter.init(desc)` and `deinit`, with
  `describe`, `setAreaCost`, `setIncludeFlags` and `setExcludeFlags`, and in C
  `zrcQueryFilterCreate`, `zrcQueryFilterDestroy`, `zrcQueryFilterDescribe`
  and the three setters. Every query, the corridor, the local boundary, the
  path queue and `Crowd.setFilter` take one. The plain struct it is made from
  is `QueryFilterDesc` (`ZrcQueryFilterDesc`), which `defaultFilterDesc`
  (`zrcQueryFilterDescDefault`) fills and `Crowd.filter` returns. Every call
  checked every area cost and built Detour's filter from the struct again,
  which was most of what `findNearestPoly` and `raycast` cost over the same
  calls made from C++; the handle is checked once, where it is made, and
  handed to Detour as it is. A negative or non-finite cost is now refused by
  `QueryFilter.init` or `setAreaCost` rather than by the query.
- `setAllocator` and `NavMeshQuery.slicedFindPathCancel` return `void`. The
  boundary refuses only a bridge missing a function and a query never made,
  and the Zig side can pass neither.

### Added

- A compile-time surface check holds every declaration in `src/c.zig` to
  `ffi/zrecast.h`, translated for the tests alone: every function by
  signature, every struct field by name, every callback, enumerator and
  mirrored constant, and every header function bound.
- `bench/` measures each binding against the same Detour work written in C++
  (`bay dev bench`); `bay test` runs it once as a smoke test.
- `DebugUtils` is bound. Upstream's fifth directory is compiled and every
  public name in it carries a verdict, so the whole vendored tree is now
  accounted for rather than four fifths of it. A host can draw the heightfield,
  the compact heightfield, the layered heightfield, the contours, the polygon
  and detail meshes, a navmesh's polygons, bounding-volume tree, portals and
  off-mesh connections, a search's node pool and closed list, and a tile
  cache's layers, contours and rebuilt mesh — through `DebugDraw`, a renderer
  built from a Zig type's own methods at compile time. Before this there was no
  way to see what a cook produced: a bake that came out wrong could only be
  inspected by reading arrays back a range at a time and drawing them by hand.
- `DisplayList` records one run of primitives and replays it into any
  renderer, so a static overlay is built once instead of re-emitted per frame.
  Upstream's class is abstract as declared — it leaves three of `duDebugDraw`'s
  pure virtuals unoverridden — and the shim completes it.
- `RecastDump` is bound: the polygon mesh and the detail mesh dump to Wavefront
  OBJ, and the contour set and the compact heightfield to upstream's binary
  form and back, through `FileIO` — four hooks a host fills. No file is opened
  or named here, and the two reads create their container rather than filling a
  caller's, so a failed read destroys a half-built one instead of handing it
  over. `logBuildTimes` sends a build's twenty-five phase timings to a build
  context's log hook.
- One upstream name is declared and defined nowhere:
  `duDebugDrawHeightfieldLayersRegions`. The coverage record gained a verdict
  for exactly that, rechecked against the vendored sources, so a re-vendor that
  supplies the definition fails the gate until it is bound rather than passing
  with the name quietly excused.

### Changed

- zrecast depends on [aegis](https://github.com/pedronaugusto/aegis) 0.2.4
  for its untrusted-input type, its checked conversions and its contracts,
  and glint's aegis rules are part of the gate.
- The development gate is preflight's, run with `bay test`: formatting,
  structure layers, lint, documentation drift, the suite three times (under
  the C sanitizer, without it, and with the asserts flipped), the C smoke test
  against the installed header, with the sanitizer and without it, the
  examples, the shared and asserts-flipped
  library builds, and a consumer build. Every check that was a shell script is
  a Zig program in `ci/`: comments, ignored files, coverage, hand-written
  numbers and the vendor comparison, which now fetches upstream by URL and
  hash at the pinned commit.
- The mutation probes and the cross-target cook comparison are no longer CI
  jobs. They run on a developer's machine with `bay probe` and
  `bay determinism`, each reporting three outcomes, so a probe the suite
  survives and a target the host could not run are named rather than passed.
  README's numbers come from `bay measurements`, and the gate refuses a stale
  block.

- The tests run on [shakedown](https://github.com/pedronaugusto/shakedown):
  every parser of a host's bytes has a property, under the fuzzer too, and
  the allocation sweep is `everyFault`. The suite's random image mutations
  were a fixed-seed loop, and its failing allocator a hand-written double.
  The hardened tier is on.
- The documentation is two files. README.md says what the package is and
  what it does; `docs/design.md` holds the layers, the bound surface, the
  boundary's checks, the cook's determinism, the record of upstream's
  behaviour and how the package is tested, folding in BINDING.md, UPSTREAM.md
  and the six files under `docs/`.

### Removed

- `ci/*.sh`, `tools/` and `tests/consumer`. The consumer build is preflight's
  consumer check, and the C consumer is the smoke test linked against the
  installed artifact. The coverage record moves to `ci/coverage/`.

### Fixed

- A tile cache layer whose cells claim a connection off its grid is refused
  wherever it enters: `buildTileCacheLayer` returns `error.InvalidArgument`,
  and `TileCacheLayer.initFromBytes` and `TileCache.addTile` return
  `error.BadFormat`. Upstream's contour walk follows a connection with no
  bounds test, so building such a layer read past the layer's buffers. A
  layer from bytes is also held to the header checks `buildTileCacheLayer`
  makes, which `addTile` never applied; `addTile` now decompresses the layer
  once to check it.
- Tile state, a compressed layer and a layer header are read and written at
  any alignment. `storeTileState`, `restoreTileState`,
  `TileCacheLayer.initFromBytes` and `swapTileCacheHeaderEndian` handed the
  caller's pointer to upstream, which reads it as a struct, so bytes at an odd
  address were a misaligned read or write: undefined behaviour, and a fault on
  targets that trap on one. Each now goes through an aligned copy, as the
  navmesh image already did.
- A count the C boundary reports is checked in every build. It was
  `@intCast`, which is undefined behaviour in ReleaseFast and ReleaseSmall
  for a value that does not fit.
- `-Denable_asserts=true` keeps Recast's and Detour's asserts in ReleaseFast
  and ReleaseSmall. Zig defines `NDEBUG` for C by itself in those two modes, so
  the option had no effect there and `assertsEnabled()` reported the asserts
  gone. The C++ now gets `-UNDEBUG` when the option asks for asserts and
  `-DNDEBUG` when it does not, in every mode.
- The assertion handler and the allocator upstream's hooks route to are
  read and written under a lock. A thread installing one while another baked
  or failed an assertion read them unsynchronised, and could see half of the
  old value and half of the new.

## [0.1.1] - 2026-09-04

### Fixed

- The allocator bridge can now tell one of its own blocks from a pointer into
  the middle of one. Every private header carries a tag alongside the length,
  and a free whose tag does not match is refused instead of acted on. Without
  the tag a length read out of payload bytes was indistinguishable from a
  recorded one, so the bridge handed its backing allocator a base pointer and
  a size that allocator never issued, and the damage surfaced somewhere else
  entirely. A refused block leaks, and any allocator that tracks leaks says so.
- `free` no longer offers a serialised navmesh image as a legal argument. An
  image arrives as `Serialized`, which owns its slice and releases it in
  `deinit`; `free` takes exactly the slice `alloc` returned, never a sub-slice
  of it.

## [0.1.0] - 2026-09-03

### Added

First release. Bindings for recastnavigation v1.6.0 — the bake, the staged
pipeline, the navmesh and its serialised image, queries, the corridor and
steering layer, the crowd, and the tile cache — with:

- A real C boundary (`ffi/zrecast.h`, one translation unit per concern) that
  stands on its own: validation before upstream is trusted with an index, a
  length or a tile image, `static_assert`s pinning every mirrored layout, and
  a standalone C smoke test proving the header is a contract rather than a
  private detail of the wrapper.
- A verdict ledger covering every public Recast/Detour name — bound through
  the boundary, C++-only with the reason, or reimplemented in Zig with its
  mirror named — regenerated from the vendored headers by `ci/check-coverage.sh`
  so completeness is a gate, not a claim.
- An idiomatic Zig layer split the way the two libraries actually run: bake
  at build time, queries at runtime, serialisation as the seam between them.
  Slices in, typed results out; partial paths are results, not errors.
- A determinism claim scoped to what is measured: one target cooking twice is
  asserted identical by the suite, targets sharing a C library are gated
  identical across architectures by `ci/determinism.sh`, and cross-library
  divergence — upstream's `qsort` tie order — is documented in UPSTREAM.md
  and reported rather than promised away. `-ffp-contract=off` everywhere and
  a polynomial cosine keep architecture and libm out of the bytes.
- A mutation-probe suite, each probe breaking one invariant and naming the
  test that must notice; a consumer package (`tests/consumer/`) driving the
  module and the installed C artifact the way a downstream `b.dependency`
  does; examples that are built AND run; generated README numbers; and the
  family CI matrix.
