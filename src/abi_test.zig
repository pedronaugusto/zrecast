//! Comptime cross-check: the hand-written declarations in `src/c.zig`
//! against `ffi/zrecast.h`, which the build translates for the test module
//! alone as `zrecast_h` (the shipped module stays translate-c-free), compared
//! per declaration via reflection; a declaration nothing here knows how to
//! check is a compile error.
//!
//! The names pair by convention: a function carries the C name itself, a
//! type `BakeConfig` pairs with `ZrcBakeConfig`, a constant `max_areas` with
//! `ZRC_MAX_AREAS`, and an enum's enumerators with the prefix
//! `enum_prefixes` gives it plus the field name in capitals. The two names
//! that break a convention are rows of their own, each checked against the
//! header. The reverse sweep is the completeness gate: every function the
//! header declares must be bound. Pointees compare by size and alignment
//! only; function pointers compare signature-deep.

const std = @import("std");
const c = @import("c.zig");

const h = @import("zrecast_h");

/// What a comparison found: null where the two sides agree, else what
/// drifted, naming both sides. Every comparison returns one, so the drift
/// tests below can hold each to the message it gives; the sweep test makes
/// a drift a compile error.
const Drift = ?[]const u8;

fn num(comptime value: anytype) []const u8 {
    return std.fmt.comptimePrint("{d}", .{value});
}

//=============================================================================
// Name conventions
//=============================================================================

/// `max_areas` -> `MAX_AREAS`.
fn upper(comptime name: []const u8) []const u8 {
    comptime {
        var out: []const u8 = "";
        for (name) |ch| out = out ++ [_]u8{std.ascii.toUpper(ch)};
        return out;
    }
}

/// The prefix each enum's enumerators carry in the header. An enum missing
/// here is a drift.
const enum_prefixes = .{
    .{ "Result", "ZRC_ERR_" },
    .{ "AllocHint", "ZRC_ALLOC_" },
    .{ "Partition", "ZRC_PARTITION_" },
    .{ "VolumeShape", "ZRC_VOLUME_" },
    .{ "LogCategory", "ZRC_LOG_" },
    .{ "TimerLabel", "ZRC_TIMER_" },
    .{ "PolyType", "ZRC_POLY_" },
    .{ "ObstacleShape", "ZRC_OBSTACLE_" },
    .{ "ObstacleState", "ZRC_OBSTACLE_" },
    .{ "CrowdAgentState", "ZRC_CROWD_AGENT_" },
    .{ "CrowdTargetState", "ZRC_CROWD_TARGET_" },
    .{ "PathRequestState", "ZRC_PATH_REQUEST_" },
    .{ "DebugDrawPrimitive", "ZRC_DEBUG_DRAW_" },
};

/// The C names no convention derives, each checked against the header like
/// any other: an enumerator outside its enum's prefix, and the two `ZrcBool`
/// values, which Zig spells around its own keywords.
const named = .{
    .{ "Result.ok", "ZRC_OK" },
    .{ "TimerLabel.max_timers", "ZRC_MAX_TIMERS" },
    .{ "c_false", "ZRC_FALSE" },
    .{ "c_true", "ZRC_TRUE" },
};

/// What the Zig side declares that the header has no counterpart for, with
/// why. Each is checked to be absent from the header, so one cannot hide a
/// real counterpart.
const zig_only = .{
    .{ "Access", "who frees a handle, a Zig-side type parameter of the handle types" },
};

fn namedCName(comptime key: []const u8) ?[]const u8 {
    inline for (named) |row| {
        if (comptime std.mem.eql(u8, row[0], key)) return row[1];
    }
    return null;
}

fn enumPrefix(comptime name: []const u8) ?[]const u8 {
    inline for (enum_prefixes) |row| {
        if (comptime std.mem.eql(u8, row[0], name)) return row[1];
    }
    return null;
}

fn zigOnly(comptime name: []const u8) bool {
    inline for (zig_only) |row| {
        if (comptime std.mem.eql(u8, row[0], name)) return true;
    }
    return false;
}

//=============================================================================
// Comparison primitives
//=============================================================================

/// A drift when `Header` does not declare `name`, which `because` expects.
fn missing(comptime Header: type, comptime name: []const u8, comptime because: []const u8) Drift {
    if (@hasDecl(Header, name)) return null;
    return "`" ++ because ++ "` expects `" ++ name ++ "` from the C header, which does not declare it";
}

fn sameSizeAndAlign(comptime what: []const u8, comptime Ours: type, comptime Theirs: type) Drift {
    if (@sizeOf(Ours) != @sizeOf(Theirs)) {
        return what ++ " is " ++ num(@sizeOf(Ours)) ++ " bytes on the Zig side but " ++
            num(@sizeOf(Theirs)) ++ " in the C header";
    }
    if (@alignOf(Ours) != @alignOf(Theirs)) {
        return what ++ " has alignment " ++ num(@alignOf(Ours)) ++ " on the Zig side but " ++
            num(@alignOf(Theirs)) ++ " in the C header";
    }
    return null;
}

/// The scalar a type really is at the boundary, with the Zig-side wrapper
/// removed. translate-c renders every C enum as a plain integer; this side
/// keeps enums as `enum(...)`, so each is resolved to its backing integer
/// first.
fn ScalarIdentity(comptime T: type) type {
    return switch (@typeInfo(T)) {
        .@"enum" => |e| e.tag_type,
        .@"struct" => |st| st.backing_integer orelse T,
        else => T,
    };
}

/// Whether `T` stands for a C enum, whose underlying type C leaves to the
/// implementation. A reference (an aegis id, or the `Maybe` slot that may hold
/// none) is an enum on the Zig side only to keep it apart from other integers:
/// its header twin is a fixed-width typedef, and its signedness is compared.
fn isCEnum(comptime T: type) bool {
    return ScalarIdentity(T) != T and !(@typeInfo(T) == .@"enum" and @hasDecl(T, "Representation"));
}

/// A `*const fn`/`?*const fn` unwrapped to the fn type, or null when the
/// type is not a function pointer. translate-c makes every C function
/// pointer optional; both sides unwrap the same way.
fn fnPointee(comptime T: type) ?type {
    const Base = switch (@typeInfo(T)) {
        .optional => |o| o.child,
        else => T,
    };
    if (@typeInfo(Base) == .pointer and @typeInfo(@typeInfo(Base).pointer.child) == .@"fn") {
        return @typeInfo(Base).pointer.child;
    }
    return null;
}

/// Size and alignment, plus the scalar identity they do not carry: a
/// `size_t` declared `isize` or a `float` declared `u32` passes both and
/// silently reinterprets every value. Function pointers on both sides
/// compare signature-deep, which is where a callback's shape is held to the
/// header.
fn sameScalar(comptime what: []const u8, comptime Ours: type, comptime Theirs: type) Drift {
    if (sameSizeAndAlign(what, Ours, Theirs)) |d| return d;
    if (fnPointee(Ours)) |OursFn| {
        const TheirsFn = fnPointee(Theirs) orelse
            return what ++ " is a function pointer on the Zig side but not in the C header";
        return checkFnType(what ++ " (callback)", OursFn, TheirsFn);
    }
    if (fnPointee(Theirs) != null) return what ++ " is a function pointer in the C header but not on the Zig side";
    const oi = @typeInfo(ScalarIdentity(Ours));
    const ti = @typeInfo(ScalarIdentity(Theirs));
    // Signedness, EXCEPT across an enum: C leaves an enum's underlying type
    // to the implementation (clang and gcc pick unsigned when no enumerator
    // is negative, MSVC uses `int`). Skipping it is sound only while every
    // enumerator is non-negative, which `checkEnumValues` holds.
    const across_enum = isCEnum(Ours) or isCEnum(Theirs);
    if (!across_enum and oi == .int and ti == .int and oi.int.signedness != ti.int.signedness) {
        return what ++ " is " ++ @tagName(oi.int.signedness) ++ " on the Zig side but " ++
            @tagName(ti.int.signedness) ++ " in the C header";
    }
    if ((oi == .int) != (ti == .int) or (oi == .float) != (ti == .float)) {
        return what ++ " is a " ++ @tagName(oi) ++ " on the Zig side but a " ++
            @tagName(ti) ++ " in the C header";
    }
    return null;
}

/// Compares two function types by what translate-c preserves: how many
/// parameters there are, how each is passed, and whether it is variadic.
fn checkFnType(comptime what: []const u8, comptime Ours: type, comptime Theirs: type) Drift {
    const ours = @typeInfo(Ours).@"fn";
    const theirs = @typeInfo(Theirs).@"fn";
    if (ours.param_types.len != theirs.param_types.len) {
        return what ++ " takes " ++ num(ours.param_types.len) ++ " parameters on the Zig side but " ++
            num(theirs.param_types.len) ++ " in the C header";
    }
    if (ours.attrs.varargs != theirs.attrs.varargs) return what ++ " is variadic on one side of the boundary only";
    inline for (ours.param_types, theirs.param_types, 0..) |op, tp, i| {
        const OP = op orelse return what ++ " has an untyped parameter on the Zig side";
        const TP = tp orelse return what ++ " has an untyped parameter in the C header";
        if (sameScalar(what ++ " parameter " ++ num(i), OP, TP)) |d| return d;
    }
    const OR = ours.return_type orelse return what ++ " has no return type on the Zig side";
    const TR = theirs.return_type orelse return what ++ " has no return type in the C header";
    return sameScalar(what ++ " return value", OR, TR);
}

//=============================================================================
// Hazard shapes
//
// Two caller shapes were measured miscompiled by Zig 0.16.0 in the sibling
// package zmeshopt (its hosted CI, 2026-09-02): a float argument after more
// than 6 integer-class parameters, and a small all-float struct returned by
// value. zrecast's boundary has neither, so it needs no forwarding shim;
// both counts are pinned at 0 below so a change that introduces one reopens
// that decision explicitly.
//=============================================================================

const max_int_class_params_before_float = 6;

/// True when `Fn` passes a float after more than 6 integer-class parameters.
fn hasLateFloat(comptime Fn: type) bool {
    comptime {
        var int_class: usize = 0;
        for (@typeInfo(Fn).@"fn".param_types) |param| {
            const P = param orelse continue;
            switch (@typeInfo(ScalarIdentity(P))) {
                .float, .vector => if (int_class > max_int_class_params_before_float) return true,
                else => int_class += 1,
            }
        }
        return false;
    }
}

/// True when `Fn` returns a struct or union by value.
fn hasAggregateReturn(comptime Fn: type) bool {
    const R = @typeInfo(Fn).@"fn".return_type orelse return false;
    return switch (@typeInfo(ScalarIdentity(R))) {
        .@"struct", .@"union" => true,
        else => false,
    };
}

//=============================================================================
// Struct, enum and constant comparisons
//=============================================================================

/// Struct layout, compared field by NAME rather than by position: two
/// same-sized adjacent fields swapping places leave the sequence of offsets
/// identical, so a positional comparison passes a swap that silently
/// reinterprets both; pairing each name with its own offset catches it.
fn checkStructLayout(comptime what: []const u8, comptime Ours: type, comptime Theirs: type) Drift {
    if (sameSizeAndAlign(what, Ours, Theirs)) |d| return d;
    const ours = @typeInfo(Ours).@"struct";
    const theirs = switch (@typeInfo(Theirs)) {
        .@"struct" => |s| s,
        else => return what ++ " is a struct in src/c.zig but not in the C header",
    };
    if (ours.field_names.len != theirs.field_names.len) {
        return what ++ " has " ++ num(ours.field_names.len) ++ " fields in src/c.zig but " ++
            num(theirs.field_names.len) ++ " in the C header";
    }
    inline for (ours.field_names, ours.field_types) |name, Field| {
        if (!@hasField(Theirs, name)) return what ++ " has field `" ++ name ++ "` in src/c.zig, which the C header does not";
        if (@offsetOf(Ours, name) != @offsetOf(Theirs, name)) {
            return what ++ "." ++ name ++ " is at byte " ++ num(@offsetOf(Ours, name)) ++ " in src/c.zig but " ++
                num(@offsetOf(Theirs, name)) ++ " in the C header";
        }
        if (sameScalar(what ++ "." ++ name, Field, @FieldType(Theirs, name))) |d| return d;
    }
    return null;
}

/// Enumerator values, each paired with its C constant by `prefix` plus the
/// field name in capitals, or by its row in `named`. translate-c flattens a
/// C enum to an integer alias and loses which enumerators belong to it; the
/// prefix is what puts them back together.
fn checkEnumValues(comptime Header: type, comptime what: []const u8, comptime name: []const u8, comptime Ours: type, comptime prefix: []const u8) Drift {
    const info = @typeInfo(Ours).@"enum";
    inline for (info.field_names, info.field_values) |field, value| {
        const cname = namedCName(name ++ "." ++ field) orelse (prefix ++ upper(field));
        if (missing(Header, cname, what ++ "." ++ field)) |d| return d;
        // The precondition that lets sameScalar skip signedness across an
        // enum: the underlying type is unobservable only while every
        // enumerator is non-negative.
        if (value < 0) {
            return what ++ "." ++ field ++ " is negative, which makes the enum's underlying type " ++
                "observable; C leaves that to the implementation and MSVC and clang choose differently";
        }
        if (@as(i128, @field(Header, cname)) != @as(i128, value)) {
            return what ++ "." ++ field ++ " is " ++ num(value) ++ " in src/c.zig but " ++ cname ++ " is " ++
                num(@field(Header, cname)) ++ " in the C header";
        }
    }
    return null;
}

/// A constant src/c.zig mirrors, against the header's macro or enumerator
/// of the same name in capitals.
fn checkConstant(comptime Header: type, comptime name: []const u8, comptime value: anytype) Drift {
    const what = "constant " ++ name;
    const cname = namedCName(name) orelse ("ZRC_" ++ upper(name));
    if (missing(Header, cname, what)) |d| return d;
    const theirs = @field(Header, cname);
    const equal = switch (@typeInfo(@TypeOf(value))) {
        .float, .comptime_float => @as(f64, value) == @as(f64, theirs),
        else => @as(i128, value) == @as(i128, theirs),
    };
    if (!equal) return what ++ " is " ++ num(value) ++ " in src/c.zig but " ++ cname ++ " is " ++ num(theirs) ++ " in the C header";
    return null;
}

//=============================================================================
// The sweep
//=============================================================================

const Counts = struct {
    types: usize = 0,
    functions: usize = 0,
    fields: usize = 0,
    enumerators: usize = 0,
    constants: usize = 0,
    late_float_fns: usize = 0,
    aggregate_return_fns: usize = 0,
};

const Swept = struct { counts: Counts = .{}, drift: Drift = null };

/// Every public declaration in `module`, classified and compared against
/// `Header`. A declaration this does not know how to check is a drift: a
/// hole in the guard stops the build rather than counting as a pass.
fn sweepOurs(comptime module: type, comptime Header: type) Swept {
    comptime {
        var n: Counts = .{};
        for (@typeInfo(module).@"struct".decl_names) |name| {
            const Decl = @TypeOf(@field(module, name));
            if (Decl == type) {
                if (sweepType(Header, name, @field(module, name), &n)) |d| return .{ .drift = d };
            } else if (@typeInfo(Decl) == .@"fn") {
                if (sweepFn(Header, name, Decl, &n)) |d| return .{ .drift = d };
            } else switch (@typeInfo(Decl)) {
                .comptime_int, .comptime_float, .int, .float => {
                    n.constants += 1;
                    if (checkConstant(Header, name, @field(module, name))) |d| return .{ .drift = d };
                },
                else => return .{ .drift = "src/c.zig declares `" ++ name ++ "` as a " ++ @tagName(@typeInfo(Decl)) ++
                    ", which this check does not know how to compare; add a case rather than leaving it unchecked" },
            }
        }
        return .{ .counts = n };
    }
}

/// One type declaration of src/c.zig, against its header counterpart.
fn sweepType(comptime Header: type, comptime name: []const u8, comptime Ours: type, comptime n: *Counts) Drift {
    const what = "type " ++ name;
    const cname = "Zrc" ++ name;
    if (zigOnly(name)) {
        if (@hasDecl(Header, cname)) return what ++ " is listed as Zig-only, but the C header declares `" ++ cname ++ "`";
        return null;
    }
    n.types += 1;
    if (missing(Header, cname, what)) |d| return d;
    const Theirs = @field(Header, cname);
    switch (@typeInfo(Ours)) {
        .@"struct" => |s| switch (s.layout) {
            .@"extern" => {
                n.fields += s.field_names.len;
                return checkStructLayout(what, Ours, Theirs);
            },
            .@"packed" => return what ++ " is a packed struct, which nothing here knows how to pair; add a check before introducing it",
            .auto => return what ++ " has automatic layout, so it has no defined ABI; declare it extern",
        },
        // A handle: the header's is opaque too, and has no layout to compare.
        .@"opaque" => return if (@typeInfo(Theirs) == .@"opaque") null else what ++ " is opaque in src/c.zig but not in the C header",
        // A reference: the header's integer typedef, compared as a scalar.
        .@"enum" => |e| if (@hasDecl(Ours, "aegis_scalar")) return sameScalar(what, Ours, Theirs) else {
            const prefix = enumPrefix(name) orelse
                return what ++ " has no row in enum_prefixes, so its enumerators cannot be paired with the header's constants";
            n.enumerators += e.field_names.len;
            if (sameSizeAndAlign(what, Ours, Theirs)) |d| return d;
            return checkEnumValues(Header, what, name, Ours, prefix);
        },
        // A scalar typedef or a callback typedef.
        .int, .float, .optional, .pointer => return sameScalar(what, Ours, Theirs),
        else => return what ++ " is a " ++ @tagName(@typeInfo(Ours)) ++ ", which this check does not know how to compare against the header",
    }
}

/// One function declaration of src/c.zig. A Zig helper is allowed, but not
/// on a boundary name: the reverse sweep only checks that a name exists, so a
/// helper on an exported symbol's name would satisfy it while the extern it
/// displaced vanishes.
fn sweepFn(comptime Header: type, comptime name: []const u8, comptime Decl: type, comptime n: *Counts) Drift {
    if (@typeInfo(Decl).@"fn".attrs.@"callconv" == .auto) {
        if (!std.mem.startsWith(u8, name, "zrc")) return null;
        return "src/c.zig declares `" ++ name ++ "` as a Zig function, not an extern; the `zrc` prefix " ++
            "is reserved for the C boundary here, so rename the helper";
    }
    const what = "function " ++ name;
    if (missing(Header, name, what)) |d| return d;
    if (hasLateFloat(Decl)) n.late_float_fns += 1;
    if (hasAggregateReturn(Decl)) n.aggregate_return_fns += 1;
    n.functions += 1;
    return checkFnType(what, Decl, @TypeOf(@field(Header, name)));
}

const Bound = struct { functions: usize = 0, drift: Drift = null };

/// The other direction, the completeness gate: a function `Header` declares
/// that `module` never bound is invisible to the sweep above, which walks
/// only what src/c.zig has.
fn sweepTheirs(comptime module: type, comptime Header: type) Bound {
    comptime {
        var found: usize = 0;
        for (@typeInfo(Header).@"struct".decl_names) |name| {
            // Filter by name BEFORE touching the value: translate-c emits
            // `@compileError` declarations for system macros it cannot render.
            if (!std.mem.startsWith(u8, name, "zrc")) continue;
            if (@typeInfo(@TypeOf(@field(Header, name))) != .@"fn") continue;
            found += 1;
            if (!@hasDecl(module, name)) {
                return .{ .drift = "the C header declares `" ++ name ++ "` but src/c.zig does not bind it; " ++
                    "the binding is complete by contract" };
            }
            // The name must resolve to something that links: the forward
            // sweep refuses helpers on boundary names first, and this is the
            // backstop, which depends only on the header.
            const Ours = @TypeOf(@field(module, name));
            if (@typeInfo(Ours) != .@"fn") {
                return .{ .drift = "the C header declares `" ++ name ++ "` but src/c.zig declares that name as a " ++
                    @tagName(@typeInfo(Ours)) ++ " rather than a function" };
            }
            if (@typeInfo(Ours).@"fn".attrs.@"callconv" == .auto) {
                return .{ .drift = "the C header declares `" ++ name ++ "` but src/c.zig declares that name as a " ++
                    "Zig function rather than an extern, so nothing binds the symbol" };
            }
        }
        return .{ .functions = found };
    }
}

//=============================================================================
// The tests
//
// The first holds the real declarations to the real header; reaching its
// body means no comparison drifted, so what is left to assert is that the
// sweeps ran: one matching nothing would read the same as one matching
// everything. The rest give the guard one deliberate drift each, of a kind a
// review would miss, and require it to name it.
//=============================================================================

test "ABI: src/c.zig agrees with ffi/zrecast.h, and binds all of it" {
    @setEvalBranchQuota(20_000_000);
    const ours = comptime sweepOurs(c, h);
    const theirs = comptime sweepTheirs(c, h);
    if (ours.drift) |d| @compileError("zrecast ABI drift: " ++ d);
    if (theirs.drift) |d| @compileError("zrecast ABI drift: " ++ d);

    // The counted surface of zrecast 0.1.1. Floors, not equalities, for all
    // but functions: the reverse sweep fails on an unbound function, so
    // `functions == theirs` pins completeness exactly.
    try std.testing.expect(ours.counts.types >= 98);
    try std.testing.expect(ours.counts.fields >= 686);
    try std.testing.expect(ours.counts.enumerators >= 84);
    try std.testing.expect(ours.counts.constants >= 63);
    try std.testing.expect(theirs.functions >= 383);
    try std.testing.expectEqual(ours.counts.functions, theirs.functions);
    // The two hazard shapes. The boundary has neither, so the package ships
    // no forwarding shim; a change that adds one fails here.
    try std.testing.expectEqual(@as(usize, 0), ours.counts.late_float_fns);
    try std.testing.expectEqual(@as(usize, 0), ours.counts.aggregate_return_fns);
}

fn expectDrift(comptime drift: Drift, comptime says: []const u8) !void {
    const found = drift orelse return error.DriftNotCaught;
    if (std.mem.find(u8, found, says) == null) {
        std.log.err("expected a drift saying \"{s}\", got: {s}", .{ says, found });
        return error.WrongDrift;
    }
}

/// The position of the field called `name`.
fn fieldIndex(comptime names: []const [:0]const u8, comptime name: []const u8) usize {
    for (names, 0..) |field, k| {
        if (std.mem.eql(u8, field, name)) return k;
    }
    @compileError("no field " ++ name);
}

/// `T` with fields `a` and `b` exchanged in declaration order.
fn Swapped(comptime T: type, comptime a: []const u8, comptime b: []const u8) type {
    const s = @typeInfo(T).@"struct";
    var names = s.field_names[0..s.field_names.len].*;
    var types = s.field_types[0..s.field_types.len].*;
    const i = fieldIndex(&names, a);
    const j = fieldIndex(&names, b);
    std.mem.swap([:0]const u8, &names[i], &names[j]);
    std.mem.swap(type, &types[i], &types[j]);
    return @Struct(s.layout, null, &names, &types, s.field_attrs);
}

/// `T` with field `name` of type `F`.
fn Retyped(comptime T: type, comptime name: []const u8, comptime F: type) type {
    const s = @typeInfo(T).@"struct";
    var types = s.field_types[0..s.field_types.len].*;
    for (s.field_names, 0..) |field, k| {
        if (std.mem.eql(u8, field, name)) types[k] = F;
    }
    return @Struct(s.layout, null, s.field_names, &types, s.field_attrs);
}

/// Enum `E` over tag type `Tag`, with enumerator `name` (if given) set to `value`.
fn Reenumerated(comptime E: type, comptime Tag: type, comptime name: ?[]const u8, comptime value: comptime_int) type {
    const e = @typeInfo(E).@"enum";
    var values: [e.field_values.len]Tag = undefined;
    for (e.field_values, &values) |v, *slot| slot.* = v;
    if (name) |n| values[fieldIndex(e.field_names, n)] = value;
    return @Enum(Tag, e.mode, e.field_names, &values);
}

/// `T` with a field `name` of type `F` added first.
fn Prepended(comptime T: type, comptime name: [:0]const u8, comptime F: type) type {
    const s = @typeInfo(T).@"struct";
    const names = [_][:0]const u8{name} ++ s.field_names[0..s.field_names.len].*;
    const types = [_]type{F} ++ s.field_types[0..s.field_types.len].*;
    const attrs = [_]@TypeOf(s.field_attrs[0]){.{}} ++ s.field_attrs[0..s.field_attrs.len].*;
    return @Struct(s.layout, null, &names, &types, &attrs);
}

test "two same-sized fields swapped are caught by their offsets" {
    @setEvalBranchQuota(100_000);
    // cell_size and cell_height are both floats, so the offset sequence is unchanged.
    const Ours = Swapped(c.BakeConfig, "cell_size", "cell_height");
    try expectDrift(comptime checkStructLayout("type BakeConfig", Ours, h.ZrcBakeConfig), "is at byte");
}

test "a callback's parameter narrowed is caught through the function pointer" {
    @setEvalBranchQuota(100_000);
    const Narrowed = Retyped(c.Allocator, "allocate", ?*const fn (?*anyopaque, u32, c.AllocHint) callconv(.c) ?*anyopaque);
    try expectDrift(comptime checkStructLayout("type Allocator", Narrowed, h.ZrcAllocator), "allocate (callback) parameter 1 is 4 bytes");
}

test "a parameter dropped, or its signedness flipped, is caught" {
    const Dropped = fn (?*const anyopaque) callconv(.c) c.Result;
    try expectDrift(comptime checkFnType("function zrcNavMeshValidate", Dropped, @TypeOf(h.zrcNavMeshValidate)), "takes 1 parameters");
    const Flipped = fn (*const c.NavMesh, u32, *c.TileRef) callconv(.c) c.Result;
    try expectDrift(comptime checkFnType("function zrcNavMeshTileRefAtIndex", Flipped, @TypeOf(h.zrcNavMeshTileRefAtIndex)), "parameter 1 is unsigned");
}

test "an enumerator renumbered, a tag narrowed and a prefix misspelled are caught" {
    @setEvalBranchQuota(100_000);
    const partition = @typeInfo(c.Partition).@"enum";
    const Renumbered = Reenumerated(c.Partition, partition.tag_type, "monotone", 9);
    try expectDrift(comptime checkEnumValues(h, "type Partition", "Partition", Renumbered, "ZRC_PARTITION_"), "monotone is 9");
    const Narrowed = Reenumerated(c.Result, u8, null, 0);
    try expectDrift(comptime sameSizeAndAlign("type Result", Narrowed, h.ZrcResult), "is 1 bytes");
    try expectDrift(comptime checkEnumValues(h, "type Partition", "Partition", c.Partition, "ZRC_PARTITIONS_"), "expects `ZRC_PARTITIONS_WATERSHED`");
}

test "a mirrored constant changed is caught against its macro" {
    try expectDrift(comptime checkConstant(h, "max_areas", 63), "max_areas is 63");
    try expectDrift(comptime checkConstant(h, "raycast_limit_proportions", 49.5), "raycast_limit_proportions is 49.5");
    try expectDrift(comptime checkConstant(h, "max_area", 64), "expects `ZRC_MAX_AREA`");
}

test "header-side drift: a field added, retyped signed, or retyped float" {
    @setEvalBranchQuota(200_000);
    try expectDrift(comptime checkStructLayout("type TileGrid", c.TileGrid, Prepended(h.ZrcTileGrid, "intruder", c_int)), "bytes on the Zig side");
    try expectDrift(comptime checkStructLayout("type TriMesh", c.TriMesh, Retyped(h.ZrcTriMesh, "vert_count", u32)), "vert_count is signed on the Zig side but unsigned");
    try expectDrift(comptime checkStructLayout("type TriMesh", c.TriMesh, Retyped(h.ZrcTriMesh, "tri_count", f32)), "tri_count is a int on the Zig side but a float");
}

test "the reverse sweep refuses a header function src/c.zig does not bind" {
    @setEvalBranchQuota(100_000);
    const Header = struct {
        pub extern fn zrcFutureThing(mesh: *c.NavMesh) c.Result;
    };
    try expectDrift(comptime sweepTheirs(c, Header).drift, "declares `zrcFutureThing` but src/c.zig does not bind it");
}

test "a Zig helper wearing an exported symbol's name is refused both ways" {
    @setEvalBranchQuota(100_000);
    const Helper = struct {
        pub fn zrcNavMeshDataVersion() i32 {
            return 7;
        }
    };
    try expectDrift(comptime sweepOurs(Helper, h).drift, "as a Zig function, not an extern");
    const Header = struct {
        pub extern fn zrcNavMeshDataVersion() i32;
    };
    try expectDrift(comptime sweepTheirs(Helper, Header).drift, "rather than an extern");
}

test "a Zig-only type is refused when the header declares it" {
    const Header = struct {
        pub const ZrcAccess = c_int;
    };
    try expectDrift(comptime sweepType(Header, "Access", c.Access, @constCast(&Counts{})), "listed as Zig-only");
}

test "the hazard shapes are recognised" {
    try std.testing.expect(comptime hasLateFloat(fn (u8, u8, u8, u8, u8, u8, u8, f32) callconv(.c) void));
    try std.testing.expect(!comptime hasLateFloat(fn (u8, u8, u8, u8, u8, u8, f32) callconv(.c) void));
    try std.testing.expect(comptime hasAggregateReturn(fn () callconv(.c) extern struct { x: f32, y: f32 }));
    try std.testing.expect(!comptime hasAggregateReturn(@TypeOf(h.zrcNavMeshDataVersion)));
}
