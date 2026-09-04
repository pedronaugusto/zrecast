# Changelog

Each entry says what the old shape could not express, so a port has the reason
and not only the diff. Versions follow [semantic versioning](https://semver.org);
before 1.0 the minor is the breaking one.

## Unreleased

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

## 0.1.0

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
