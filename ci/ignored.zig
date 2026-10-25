//! Nothing the repository ignores is tracked by it. `.gitignore` names what
//! does not belong in the history (build output, fetched packages, objects
//! carrying a machine's paths), but tracking overrides every rule in it, so a
//! path swept in by `git add -A` would sit in every clone with nothing to say
//! so. git is asked, so `.gitignore` stays the one home of the rules.
const std = @import("std");
const Io = std.Io;

/// The most output read from git: the tracked paths that match a rule.
const max_output = 1 << 20;

pub fn main(init: std.process.Init) !u8 {
    const io = init.io;
    const result = try std.process.run(init.gpa, io, .{
        .argv = &.{ "git", "ls-files", "--cached", "--ignored", "--exclude-standard" },
        .stdout_limit = .limited(max_output),
        .stderr_limit = .limited(max_output),
    });
    defer init.gpa.free(result.stdout);
    defer init.gpa.free(result.stderr);
    var buffer: [4096]u8 = undefined;
    var stderr = Io.File.stderr().writer(io, &buffer);
    const out = &stderr.interface;
    defer out.flush() catch {}; // glint-ignore: Z026 -- the exit code already reports the result
    if (result.term != .exited or result.term.exited != 0) {
        try out.print("git ls-files failed: {s}", .{result.stderr});
        return 1;
    }
    const tracked = std.mem.trim(u8, result.stdout, "\n");
    if (tracked.len == 0) return 0;
    try out.print("these tracked paths match an ignore rule; take each out of the index " ++
        "(git rm -r --cached <path>) or drop the rule:\n{s}\n", .{tracked});
    return 1;
}
