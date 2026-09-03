//! Vector and scalar math Recast and Detour spell out `inline` in their C++
//! headers, reimplemented here so a host that lerps between straight-path
//! corners gets exactly the same answer Detour does — a call across the FFI
//! boundary to add three floats is not a binding worth having.
//!
//! Every function below is asserted bit-identical to the C, over a table
//! that includes zeros, negative zeros and denormals. Each one is a sum of
//! products, the shape a fused multiply-add contracts into a single
//! rounding instead of two, so the C side is compiled with
//! `-ffp-contract=off` and this side leans on Zig's default float mode,
//! `.strict`, being the same promise. Do not write `@setFloatMode` anywhere
//! in this file: strict is already the default, and naming it would suggest
//! otherwise.
//!
//! Three of the scalar helpers below answer differently from the obvious
//! Zig spelling at an edge, and two more do at a value the parity table
//! already exercises. Each is transcribed as upstream's own expression
//! rather than the natural Zig one, and its doc comment names the value
//! where the two diverge.

const std = @import("std");

pub const Vec3 = [3]f32;

pub inline fn add(a: Vec3, b: Vec3) Vec3 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

pub inline fn sub(a: Vec3, b: Vec3) Vec3 {
    return .{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

pub inline fn scale(v: Vec3, t: f32) Vec3 {
    return .{ v[0] * t, v[1] * t, v[2] * t };
}

/// `a + b * s`. Two operations, not `@mulAdd`: the fused form rounds once
/// where upstream's separate multiply-then-add rounds twice.
pub inline fn mad(a: Vec3, b: Vec3, s: f32) Vec3 {
    return .{ a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s };
}

pub inline fn lerp(a: Vec3, b: Vec3, t: f32) Vec3 {
    return .{
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    };
}

pub inline fn cross(a: Vec3, b: Vec3) Vec3 {
    return .{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

pub inline fn dot(a: Vec3, b: Vec3) f32 {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// Dot product on the xz-plane; y is ignored.
pub inline fn dot2D(a: Vec3, b: Vec3) f32 {
    return a[0] * b[0] + a[2] * b[2];
}

/// The xz-plane perp product, `a.z*b.x - a.x*b.z`.
pub inline fn perp2D(a: Vec3, b: Vec3) f32 {
    return a[2] * b[0] - a[0] * b[2];
}

/// Componentwise minimum. Upstream's `dtVmin(mn, v)` computes
/// `dtMin(mn[i], v[i])`, so `a` is the first argument to `scalar.min` and
/// wins a tie or a NaN the way `scalar.min` does; the operand order here
/// matches that.
pub inline fn min(a: Vec3, b: Vec3) Vec3 {
    return .{ scalar.min(a[0], b[0]), scalar.min(a[1], b[1]), scalar.min(a[2], b[2]) };
}

/// Componentwise maximum, same operand-order contract as `min`.
pub inline fn max(a: Vec3, b: Vec3) Vec3 {
    return .{ scalar.max(a[0], b[0]), scalar.max(a[1], b[1]), scalar.max(a[2], b[2]) };
}

pub inline fn len(v: Vec3) f32 {
    return @sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

pub inline fn lenSqr(v: Vec3) f32 {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

/// Distance between two points. The subtraction is `b - a`, upstream's own
/// order, not the reverse.
pub inline fn dist(a: Vec3, b: Vec3) f32 {
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    const dz = b[2] - a[2];
    return @sqrt(dx * dx + dy * dy + dz * dz);
}

pub inline fn distSqr(a: Vec3, b: Vec3) f32 {
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    const dz = b[2] - a[2];
    return dx * dx + dy * dy + dz * dz;
}

/// Distance on the xz-plane; y is ignored.
pub inline fn dist2D(a: Vec3, b: Vec3) f32 {
    const dx = b[0] - a[0];
    const dz = b[2] - a[2];
    return @sqrt(dx * dx + dz * dz);
}

pub inline fn dist2DSqr(a: Vec3, b: Vec3) f32 {
    const dx = b[0] - a[0];
    const dz = b[2] - a[2];
    return dx * dx + dz * dz;
}

/// Upstream computes a reciprocal once and multiplies each component by it,
/// rather than dividing each component by the length; a reciprocal followed
/// by three multiplies is not bit-identical to three divides, so the
/// reciprocal form is transcribed here. No zero-length guard, matching
/// upstream: a zero vector normalises to NaN, the same answer a C++ host
/// gets.
pub inline fn normalize(v: Vec3) Vec3 {
    const d = 1.0 / @sqrt(scalar.sqr(v[0]) + scalar.sqr(v[1]) + scalar.sqr(v[2]));
    return .{ v[0] * d, v[1] * d, v[2] * d };
}

/// A "sloppy" colocation check: true when the squared distance between `a`
/// and `b` is under `sqr(1/16384)`, exactly `0x1p-28`.
pub inline fn equal(a: Vec3, b: Vec3) bool {
    const thr = scalar.sqr(1.0 / 16384.0);
    return distSqr(a, b) < thr;
}

pub inline fn isFinite(v: Vec3) bool {
    return std.math.isFinite(v[0]) and std.math.isFinite(v[1]) and std.math.isFinite(v[2]);
}

/// Finiteness on the xz-plane; y is not examined.
pub inline fn isFinite2D(v: Vec3) bool {
    return std.math.isFinite(v[0]) and std.math.isFinite(v[2]);
}

/// Scalar helpers Recast and Detour template over their element type.
pub const scalar = struct {
    /// Upstream's `dtMin(3, NaN)` is `NaN`, since the template returns `b`
    /// whenever `a < b` is false and every NaN comparison is false; `@min`
    /// returns `3` instead.
    pub inline fn min(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
        return if (a < b) a else b;
    }

    /// The same template shape as `min`, so it carries the same asymmetry:
    /// `dtMax(3, NaN)` is `NaN` where `@max` gives `3`, while `dtMax(NaN, 3)`
    /// and `@max(NaN, 3)` agree at `3`.
    pub inline fn max(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
        return if (a > b) a else b;
    }

    /// `dtAbs(-0.0)` is `-0.0`, bit pattern `0x80000000`; `@abs` gives `+0.0`.
    pub inline fn abs(a: anytype) @TypeOf(a) {
        return if (a < 0) -a else a;
    }

    /// `a * a` is the obvious Zig spelling too, so unlike its neighbours this
    /// one never disagrees with it.
    pub inline fn sqr(a: anytype) @TypeOf(a) {
        return a * a;
    }

    /// `dtClamp(NaN, 0, 1)` is `NaN`, since both comparisons against NaN are
    /// false and the value falls through unchanged; `std.math.clamp` gives
    /// `1`.
    pub inline fn clamp(v: anytype, lo: @TypeOf(v), hi: @TypeOf(v)) @TypeOf(v) {
        return if (v < lo) lo else if (v > hi) hi else v;
    }

    /// `dtNextPow2(0)` is `0`. `std.math.ceilPowerOfTwo` refuses a zero input
    /// outright rather than answering for it, so it is not an alternative
    /// spelling of this at all, let alone an agreeing one.
    ///
    /// The decrement underflows to `0xFFFFFFFF` for an input of 0 and the
    /// increment wraps back to 0, both intentional, so `-%` and `+%` are used
    /// in place of upstream's plain `--` and `++`.
    pub inline fn nextPow2(v: u32) u32 {
        var x = v;
        x -%= 1;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        x +%= 1;
        return x;
    }

    /// `dtIlog2(0)` is `0`. The naive `31 - @clz(v)` spelling gives `-1` for
    /// the same input; upstream's shift sequence is transcribed instead of
    /// that spelling.
    pub inline fn ilog2(v: u32) u32 {
        var x = v;
        var r: u32 = @intFromBool(x > 0xffff);
        r <<= 4;
        x >>= @intCast(r);
        var shift: u32 = @intFromBool(x > 0xff);
        shift <<= 3;
        x >>= @intCast(shift);
        r |= shift;
        shift = @intFromBool(x > 0xf);
        shift <<= 2;
        x >>= @intCast(shift);
        r |= shift;
        shift = @intFromBool(x > 0x3);
        shift <<= 1;
        x >>= @intCast(shift);
        r |= shift;
        r |= (x >> 1);
        return r;
    }

    /// `(x + 3) & ~3`. The add overflows for `x` near `maxInt(i32)`, which is
    /// undefined behaviour in the C and a panic in a safety-checked Zig
    /// build, so `+%` replaces the plain `+`.
    pub inline fn align4(x: i32) i32 {
        return (x +% 3) & ~@as(i32, 3);
    }
};

test "min and max both fall back to NaN only when it is the second operand" {
    const nan = std.math.nan(f32);
    const three: f32 = 3.0;

    // dtMin(3, NaN) is NaN; @min(3, NaN) would be 3.
    try std.testing.expectEqual(
        @as(u32, @bitCast(nan)),
        @as(u32, @bitCast(scalar.min(three, nan))),
    );
    // dtMin(NaN, 3) is 3, agreeing with what @min would give here.
    try std.testing.expectEqual(@as(f32, 3.0), scalar.min(nan, three));

    // The same asymmetry, mirrored, for max.
    try std.testing.expectEqual(
        @as(u32, @bitCast(nan)),
        @as(u32, @bitCast(scalar.max(three, nan))),
    );
    try std.testing.expectEqual(@as(f32, 3.0), scalar.max(nan, three));
}

test "abs keeps the sign bit of negative zero" {
    const neg_zero: f32 = -0.0;
    try std.testing.expectEqual(
        @as(u32, 0x80000000),
        @as(u32, @bitCast(scalar.abs(neg_zero))),
    );
}

test "clamp propagates NaN instead of pinning it to the high bound" {
    const nan = std.math.nan(f32);
    const clamped = scalar.clamp(nan, @as(f32, 0), @as(f32, 1));
    try std.testing.expect(std.math.isNan(clamped));
}

test "nextPow2 and ilog2 both answer 0 for an input of 0" {
    try std.testing.expectEqual(@as(u32, 0), scalar.nextPow2(0));
    try std.testing.expectEqual(@as(u32, 0), scalar.ilog2(0));
}

test "align4 wraps instead of panicking near the top of i32" {
    try std.testing.expectEqual(
        @as(i32, std.math.minInt(i32)),
        scalar.align4(std.math.maxInt(i32)),
    );
}
