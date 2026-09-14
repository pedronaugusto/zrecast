# The test suite

```sh
zig build test        # the Zig suite
zig build test-c      # the C-level smoke test on its own
zig build examples    # build and run the examples
ci/run.sh             # the full matrix, the same one CI runs
ci/run.sh --quick     # native Debug only, for the inner loop
ci/install-hooks.sh   # run ci/run.sh automatically before every push
```

`ci/run.sh` reports every failure rather than stopping at the first, and
`ci/check-mirror.sh` fails if the set of build-option combinations it executes
stops matching `.github/workflows/ci.yml`.

## The fixture

The suite is self-contained. `tests/fixture.cpp` generates its input geometry
programmatically — a ground plane, a wall with a gap at one end, a pillar, and
an island connected to nothing — and the tests bake a navmesh from it at test
time. No third-party meshes are shipped, and no asset provenance has to be
accounted for.

The shape is chosen so the assertions can bite. A path across the wall must
round its end, so the string-pull has to produce real corners rather than a
two-point line. The island is reachable from nowhere, so `partial` has
something to be true about. A second fixture — a tent of 71-degree faces — has
real three-dimensional bounds and no surface an agent could stand on, which is
the `EmptyResult` path.

## The C smoke test

`zig build test-c` walks the same arc in plain C11 with a `malloc`-based
allocator, proving the header is a real C contract rather than a private detail
of the Zig wrapper. It also checks the things only a C caller can get wrong:
null handles and null out-parameters on every entry point that takes them, a
companion buffer shorter than the point array it accompanies, a half-filled
allocator being refused *without* unseating the working one, and
`zrcSetAllocator(NULL)` genuinely handing allocation back to malloc.

And it asserts that every allocation was returned. That assertion caught a leak
of three buffers per detail mesh during development, described in
[UPSTREAM.md](../UPSTREAM.md).

## Mutation probes

A test that passes may be asserting something the code could not have violated
anyway. `ci/probes/` holds one patch per invariant that breaks it on purpose
and names the test that has to fail because of it, and `ci/probe.sh` applies
each to a copy of the tree and reports the ones the suite survives.

```sh
ci/probe.sh              # every probe
ci/probe.sh tile-bounds  # only the ones whose name matches
```

Of the first eleven probes written, four went uncaught. Three were holes: a
tile window shifted by one cell sat inside a tolerance that had been set too
loosely, the reference-bit check was indistinguishable from Detour reaching the
same verdict a few allocations later, and nothing at all reached the
null-header path this package guards upstream against. The tests that close
those three were written because the probes said they were missing. The fourth
was the probe's own expectation, which named the wrong check as the one that
fires.

## What CI runs

`.github/workflows/ci.yml`, on push to `main` and on every pull request:

- `test` on ubuntu-latest, macos-latest and windows-latest: `zig fmt --check`,
  then the suite in all four optimize modes with `-Dsanitize_c` both on and
  off, then `zig build test-c`, then the consumer package
  (`tests/consumer/`) with asserts on and off. On the Windows runner it also
  runs the suite and the C smoke test against `x86_64-windows-msvc` and builds
  `x86_64-windows-gnu` explicitly, because Zig's default ABI on a Windows host
  is gnu and a bare `zig build test` never touches the MSVC path.
- `cross` — one compile-only job per target, eight of them.
- `build configurations` — `-Dshared=true`, `-Denable_asserts=false`, and
  ReleaseFast with asserts on.
- `hygiene` — the installed-header check, `ci/check-comments.sh`,
  `ci/check-executable.sh`, `ci/check-ignored.sh`, `ci/check-mirror.sh` and
  `ci/check-coverage.sh --list`.
- `mutation probes` — `ci/probe.sh`.
- `cook determinism` — `ci/determinism.sh --strict` under QEMU.
- `documentation numbers` — `ci/check-docs.sh`.
- `vendor integrity` — `ci/verify-vendor.sh`, which diffs
  `libs/recastnavigation` against a fresh clone of the pinned commit.

The consumer package is its own job's worth of value: it exercises the module
through `b.dependency` and the installed artifact through `linkLibrary` +
`installHeader`, and nothing in `src/` exercises either resolution path.
