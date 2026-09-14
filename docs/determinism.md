# Cook determinism

A navmesh is a shipped asset. If two machines cooking the same geometry produce
different bytes, the asset cannot be built once and trusted, and a bug that
reproduces on one of them may not reproduce on the other. What this package
guarantees is scoped to what it measures, in three rings.

- **One target, cooked twice, is the same bytes.** Asserted by the suite —
  untiled, tiled and area-authored cooks, hashed and compared — on every target
  that runs it.
- **Two targets sharing a C library are the same bytes.** `ci/determinism.sh`
  cooks on both architectures of musl and glibc and fails if a pair sharing a
  library disagrees. With contraction pinned and no libm in the bake,
  architecture is not allowed to move a byte.
- **Two different C libraries are compared, not promised.** Upstream sorts BV
  items, contour holes and diagonals with `qsort`, ties fall to the
  implementation, and any order the ties land in is a valid navmesh —
  [UPSTREAM.md](../UPSTREAM.md) records the five sites. The script prints the
  digests side by side, so a divergence is seen, named and attributed rather
  than asserted away. Ship cooks from one platform, or key the asset cache by
  cook platform.

## What the first two rings rest on

Each of these was a real gap rather than a precaution.

- **`-ffp-contract=off`, for every target.** Clang fuses a multiply and an add
  into a single instruction wherever the target has one, and the fused form
  rounds once where the separate form rounds twice. AArch64 mandates FMA; the
  x86-64 baseline has none. The same source line in Recast therefore produced
  different floats on the two architectures.
- **The Zig vector math is asserted bit-identical to the C.** `src/vec.zig`
  reimplements the `dtV`/`rcV` families rather than calling across the boundary
  to add three floats, which only holds if the two agree everywhere — so a test
  compares them bit for bit over a table of zeros, negative zeros and
  denormals, against linkable shims over upstream's own `inline` definitions.
  Six of the scalar helpers answer differently from Zig's obvious spelling:
  `dtMin(3, NaN)` and `dtMax(3, NaN)` are `NaN` where `@min` and `@max` give
  `3`, `dtAbs(-0.0)` keeps its sign bit, `dtClamp(NaN, 0, 1)` passes the NaN
  through, and `dtNextPow2(0)` and `dtIlog2(0)` are both `0`. Each is written
  out as upstream's own expression and pinned by a test at exactly that value.
- **`zrc::CosDegrees` instead of `cosf`.** The slope limit becomes a cosine,
  and that was the only transcendental the bake reached. The C standard does
  not require a correctly rounded cosine, so the threshold — which decides per
  triangle whether a surface is navigable — depended on the host's C library.
  It is now a polynomial over the bounded range a slope limit can occupy, using
  only operations IEEE-754 specifies exactly.

## Running it

```sh
ci/determinism.sh            # two architectures and three C libraries, here
ci/determinism.sh --strict   # and a target that could not be run is a failure
```

The script builds the suite for each target and runs it wherever it can —
natively, under Rosetta, or in a container. It reports three outcomes and never
two: a target that ran, a target that failed, and a target this host could not
run at all. The last is counted and named in the summary rather than dropped,
and `--strict` makes it a failure. CI runs it with `--strict`, with QEMU
supplying the aarch64 containers.

## Where it ends

The byte-for-byte guarantee covers baked assets. It does not extend to a
frame's steering, which reaches `cosf` and `sinf` in the adaptive velocity
sampler. A host that needs a reproducible replay should record its inputs.
This is stated here, in `ffi/zrecast.h` and in [UPSTREAM.md](../UPSTREAM.md),
because the package's other half makes the opposite promise and the asymmetry
would otherwise read as an oversight.
