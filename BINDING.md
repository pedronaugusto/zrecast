# Adding surface to zrecast

What to do so a new piece of Recast or Detour arrives here with the same
guarantees everything else has. The order below is the order that works;
each step names the gate that holds it.

## The layered shape

Upstream is C++, and C++ does not cross a linking boundary intact — so the
package is three layers, and a new capability lands in each:

1. **`libs/recastnavigation` is upstream, byte for byte.** Nothing is ever
   added or changed there; `ci/verify-vendor.sh` diffs it against the pinned
   commit and fails on any local edit. Behaviour upstream lacks belongs in the
   next layer down.
2. **`ffi/` is the C boundary.** One translation unit per concern
   (`zrecast_bake.cpp`, `zrecast_query.cpp`, …), each entry point `ZRC_API`,
   `zrc`-prefixed, declared in the single public header `ffi/zrecast.h` with
   `Zrc`-prefixed types. A mirrored struct or constant gets a `static_assert`
   in `ffi/zrecast_abi.cpp` against upstream's definition, so a re-vendor that
   moves anything fails the build. Validation lives here too: an index, a
   length or a reference that upstream would trust is checked before upstream
   sees it, and the refusals are documented in UPSTREAM.md.
3. **`src/` is the Zig layer.** The externs (`pub extern fn zrc…`) sit in the
   module that owns them — `bake.zig`, `navmesh.zig`, `query.zig`, `crowd.zig`,
   `tilecache.zig`, `pipeline.zig` — next to the idiomatic wrapper that calls
   them. Nothing `@cImport`s the header; the ABI is held by the asserts layer
   and the smoke test instead.

## Naming, which is load-bearing

An entry point operating on an object is named object-first
(`zrcPolyMeshBake`, `zrcHeightfieldAddSpan`), so the header groups by the
thing operated on; a query or free helper keeps upstream's own verb
(`zrcFindNearestPoly`, `zrcCalcBounds`). The Zig layer turns a handle-taking
entry point into a method on the owning wrapper (`NavMeshQuery.findPath`) and
keeps upstream's terminology for concepts (polys, contours, regions,
corridors), so upstream's documentation stays usable as this package's.

## The verdict ledger — how "complete" stays checked

Every public name in the vendored headers carries one verdict, and
`ci/check-coverage.sh` regenerates the name list from the headers themselves,
so a name cannot be missing quietly:

- **`BOUND`** — reachable through the C boundary; the evidence names the
  `zrc` symbol that carries it, and the gate checks that symbol exists.
- **`LANGUAGE`** — C++-only surface a C boundary cannot carry (a class method
  whose object never crosses, an inline helper); the evidence says what a C
  host does instead.
- **`ZIG`** — reimplemented on the Zig side (vector math, small inline
  helpers); the evidence names the mirror, and `src/vec.zig`'s bit-identity
  test is what makes a mirror honest.

A new binding therefore edits `tools/bindings.tsv`, runs `tools/record.sh` to
regenerate `tools/unbound_*.txt`, and leaves `ci/check-coverage.sh` green.
A verdict with stale evidence — naming a symbol that no longer exists — fails
the same gate.

## The idiomatic layer's rules

- Slices in, typed results out. Counts and capacities stay inside the wrapper;
  a caller passes `[]PolyRef` and gets `.len` back.
- `ZrcResult` becomes the error set in `src/error.zig`; a partial outcome that
  upstream reports as success stays a success with a `.partial` field, because
  partial is a result, not an error.
- Optionals mean what they say: a nullable pointer parameter is `?`, a
  reference that can be "none" is `?PolyRef`, never a magic zero at the
  surface.
- Flags are named option structs (`StraightPathOptions`), never bare `u32`s.
- Memory routes through the process-global allocator seam
  (`setAllocator`/`resetAllocator`), which `tests/c_smoke.c` proves is
  actually in use by counting.

## The version has one home

`build.zig.zon` is the version. The `ZRC_VERSION_*` macros in `ffi/zrecast.h`
restate it for C consumers, and the "version reporting is wired up" test
compares what they compiled to against the `.zon` string, so the two cannot
drift.

## Before you call it done

- `ci/run.sh` green — the suite (which runs the examples and the C smoke
  test), every release mode, both sanitizer settings, the cross-targets, and
  the coverage gate.
- A new invariant gets a probe in `ci/probes/` naming the test that must fail
  when it is broken; `ci/probe.sh` refuses a probe the suite survives.
- A new C entry point gets `CHECK`s in `tests/c_smoke.c` — the boundary is a
  real C contract, not a private detail of the wrapper.
- If any number the README states moved, `ci/check-docs.sh --write`
  regenerates the blocks; the same script fails CI while they are stale.
