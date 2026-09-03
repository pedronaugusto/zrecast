//! The seam that routes Recast's and Detour's internal assertions into a
//! host's own logger.
//!
//! Upstream compiles the whole hook family out under NDEBUG: in a release
//! build nothing is ever checked and the handler installed here is never
//! called. `assertsEnabled` is the only way to tell which build this is.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");

/// One assertion failure, as upstream reported it.
pub const Failure = struct {
    expression: []const u8,
    file: []const u8,
    line: i32,
};

pub const Handler = struct {
    /// Invoked on the failing thread, from inside upstream, before the state
    /// the assertion was guarding against is entered. Returning is allowed and
    /// execution continues into that state, so a handler that means to stop
    /// must not return.
    fail: *const fn (user: ?*anyopaque, failure: Failure) void,
    user: ?*anyopaque = null,
};

var installed: ?Handler = null;

/// Matches `c.AssertFailFunc`. The C-level `user` pointer is left null on
/// every install below: `Handler` already carries its own context, so
/// threading a second one through the C struct would answer the same
/// question twice.
fn trampoline(
    _: ?*anyopaque,
    expression: [*:0]const u8,
    file: [*:0]const u8,
    line: i32,
) callconv(.c) void {
    const h = installed orelse return;
    h.fail(h.user, .{
        .expression = std.mem.span(expression),
        .file = std.mem.span(file),
        .line = line,
    });
}

/// Whether this build kept Recast's and Detour's internal assertions.
///
/// False in a build compiled with NDEBUG, where `setHandler` still records a
/// handler and nothing will ever call it.
pub fn assertsEnabled() bool {
    return c.zrcAssertsEnabled() != c.c_false;
}

/// Installs a process-wide handler for Recast's and Detour's internal
/// assertions. `null` clears it and restores upstream's default, `assert()`.
pub fn setHandler(h: ?Handler) err.Error!void {
    installed = h;
    if (h == null) {
        try err.check(c.zrcSetAssertHandler(null));
        return;
    }
    const bridge = c.AssertHandler{ .fail = trampoline, .user = null };
    try err.check(c.zrcSetAssertHandler(&bridge));
}

/// The handler this library is actually routed to, or `null`.
///
/// Asked of the library rather than answered from the slot above: the C side
/// holds the trampoline, so comparing what it reports against `trampoline`
/// says whether the routing is still in place. A handler cleared or replaced
/// through the C entry point directly reports `null` here, which a mirror of
/// the last Zig call could not.
pub fn handler() err.Error!?Handler {
    var out: c.AssertHandler = undefined;
    try err.check(c.zrcAssertHandler(&out));
    if (out.fail != &trampoline) return null;
    return installed;
}

test "setHandler round-trips through handler, and null clears it" {
    const Static = struct {
        fn onFail(user: ?*anyopaque, failure: Failure) void {
            _ = user;
            _ = failure;
        }
    };

    try setHandler(.{ .fail = Static.onFail, .user = null });
    const got = (try handler()) orelse return error.TestUnexpectedResult;
    try std.testing.expect(got.fail == &Static.onFail);
    try std.testing.expect(got.user == null);

    try setHandler(null);
    try std.testing.expect((try handler()) == null);

    // Cleared through the C entry point behind this module's back: the slot
    // still holds the handler, and the answer is still `null`, because the
    // library is no longer routed to it.
    try setHandler(.{ .fail = Static.onFail, .user = null });
    try std.testing.expectEqual(c.Result.ok, c.zrcSetAssertHandler(null));
    try std.testing.expect((try handler()) == null);
    try setHandler(null);
}

test "assertsEnabled agrees with how the C library was actually compiled" {
    const options = @import("zrecast_options");
    try std.testing.expectEqual(options.enable_asserts, assertsEnabled());
}
