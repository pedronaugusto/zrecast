//! The Recast half: turning geometry into a polygon mesh.
//!
//! This is build-time work. It allocates heavily, takes as long as it takes,
//! and its output is meant to be serialised by `navmesh.zig` and shipped. A
//! runtime that only queries never needs anything in this file.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");

/// How the walkable surface is split into regions before contours are traced.
pub const Partition = c.Partition;

/// Bake parameters in world units. Start from `default()` rather than a zeroed
/// struct — a zero `cell_size` has no meaning and is rejected.
pub const Config = c.BakeConfig;

/// The parameters Recast's own solo-mesh sample ships with: a 0.6 m radius,
/// 2.0 m tall agent on 0.3 m voxels.
pub fn defaultConfig() Config {
    var config: Config = undefined;
    c.zrcBakeConfigDefault(&config);
    return config;
}

/// The tile grid a tiled bake and its navmesh share.
///
/// Computed once by `tileGrid` and passed to both halves, because a grid the
/// two disagree about produces tiles that do not line up.
pub const TileGrid = c.TileGrid;

/// Which fields of an `AreaVolume` are read.
pub const VolumeShape = c.VolumeShape;

/// One volume of the world, and the area id the surface inside it takes.
///
/// Build one with `convexVolume`, `boxVolume` or `cylinderVolume` rather than
/// by hand: the struct carries a pointer and a count that only mean anything
/// together, and the constructors are what keep them agreeing.
pub const AreaVolume = c.AreaVolume;

/// A convex footprint in the xz plane, extruded between `y_min` and `y_max`.
///
/// The y of each vertex is ignored — `y_min` and `y_max` carry the extent — so
/// a renderer's vertex array can be passed straight in. `InvalidArgument` for
/// fewer than three vertices or a length that is not a multiple of three.
pub fn convexVolume(
    area: i32,
    y_min: f32,
    y_max: f32,
    verts: []const f32,
) err.Error!AreaVolume {
    if (verts.len % 3 != 0) return err.Error.InvalidArgument;
    const vert_count = std.math.cast(i32, verts.len / 3) orelse
        return err.Error.InvalidArgument;
    if (vert_count < 3) return err.Error.InvalidArgument;
    return .{
        .shape = .convex,
        .area = area,
        .y_min = y_min,
        .y_max = y_max,
        .verts = verts.ptr,
        .vert_count = vert_count,
        .xz_min = .{ 0, 0 },
        .xz_max = .{ 0, 0 },
        .radius = 0,
    };
}

/// An axis-aligned box, `xz_min` and `xz_max` being its two corners in xz.
pub fn boxVolume(
    area: i32,
    y_min: f32,
    y_max: f32,
    xz_min: [2]f32,
    xz_max: [2]f32,
) AreaVolume {
    return .{
        .shape = .box,
        .area = area,
        .y_min = y_min,
        .y_max = y_max,
        .verts = null,
        .vert_count = 0,
        .xz_min = xz_min,
        .xz_max = xz_max,
        .radius = 0,
    };
}

/// A vertical cylinder standing on its base, centred at `xz_centre`.
pub fn cylinderVolume(
    area: i32,
    y_min: f32,
    y_max: f32,
    xz_centre: [2]f32,
    radius: f32,
) AreaVolume {
    return .{
        .shape = .cylinder,
        .area = area,
        .y_min = y_min,
        .y_max = y_max,
        .verts = null,
        .vert_count = 0,
        .xz_min = xz_centre,
        .xz_max = .{ 0, 0 },
        .radius = radius,
    };
}

/// What a bake writes into the mesh beyond the shape of the geometry.
///
/// Both fields are optional: the zero value is the uniform-space default, in
/// which every walkable polygon gets area `area_walkable` and flag
/// `poly_flag_walkable`.
pub const AreaAuthoring = struct {
    /// Applied in order, after the walkable surface has been eroded by the
    /// agent radius and before it is cut into regions. Later volumes overwrite
    /// earlier ones where they overlap, so the order is the layering.
    volumes: []const AreaVolume = &.{},

    /// The polygon flags each area id receives, indexed by area id.
    ///
    /// `null` keeps the default. Entry `area_null` must be 0: an unwalkable
    /// polygon that some filter still admits is a polygon a path can cross.
    area_flags: ?*const [c.max_areas]u16 = null,

    pub fn toC(self: AreaAuthoring) err.Error!c.AreaAuthoring {
        const volume_count = std.math.cast(i32, self.volumes.len) orelse
            return err.Error.InvalidArgument;
        return .{
            .volumes = if (self.volumes.len == 0) null else self.volumes.ptr,
            .volume_count = volume_count,
            .area_flags = self.area_flags,
        };
    }
};

/// Computes the tile grid `config` and `mesh` imply.
///
/// `InvalidArgument` when `config.tile_size` is 0: a single-tile bake has no
/// grid, and `PolyMesh.bake` is the entry point for one.
pub fn tileGrid(config: Config, mesh: TriMesh) err.Error!TileGrid {
    const c_mesh = try mesh.toC();
    var grid: TileGrid = undefined;
    try err.check(c.zrcTileGridCompute(&config, &c_mesh, &grid));
    return grid;
}

/// An indexed triangle soup, right-handed and **Y up**.
///
/// Held as slices so lengths travel with the pointers; the counts handed to C
/// are derived, never passed in separately.
pub const TriMesh = struct {
    /// `3 * n` floats, (x, y, z) interleaved.
    verts: []const f32,
    /// `3 * m` indices into the vertex array.
    tris: []const i32,

    pub fn toC(self: TriMesh) err.Error!c.TriMesh {
        // The C side re-checks all of this. Doing it here too is not
        // redundancy: it is what lets the division below be safe, and it turns
        // a slice whose length is not a multiple of three — something the C
        // struct cannot express — into an error rather than a silent truncation.
        if (self.verts.len == 0 or self.verts.len % 3 != 0) return err.Error.InvalidArgument;
        if (self.tris.len == 0 or self.tris.len % 3 != 0) return err.Error.InvalidArgument;
        const vert_count = std.math.cast(i32, self.verts.len / 3) orelse
            return err.Error.InvalidArgument;
        const tri_count = std.math.cast(i32, self.tris.len / 3) orelse
            return err.Error.InvalidArgument;
        return .{
            .verts = self.verts.ptr,
            .vert_count = vert_count,
            .tris = self.tris.ptr,
            .tri_count = tri_count,
        };
    }
};

/// Everything about a polygon mesh's shape and state, copied out in one
/// call: vertex and polygon counts, bounds, cell size, the detail half's
/// counts (zero until built), and the agent dimensions `PolyMesh.setAgentDims`
/// recorded.
pub const PolyMeshInfo = c.PolyMeshInfo;

/// A baked polygon mesh: the artefact a cook produces.
pub const PolyMesh = struct {
    handle: *c.PolyMesh,

    /// Allocates an empty polygon mesh, for a later `polyMeshBuild`,
    /// `polyMeshCopy` or `polyMeshMerge` stage to fill.
    pub fn initEmpty() err.Error!PolyMesh {
        var handle: *c.PolyMesh = undefined;
        try err.check(c.zrcPolyMeshCreate(&handle));
        return .{ .handle = handle };
    }

    /// Runs the whole Recast pipeline over `mesh`.
    ///
    /// `authoring` is optional; `null` gives every walkable polygon area
    /// `area_walkable` and flag `poly_flag_walkable`. `log` is an optional
    /// buffer for the messages Recast emits while building. Recast reports
    /// failure as a bare boolean and puts the reason in a log callback, so
    /// passing one is the difference between "the bake failed" and knowing
    /// which stage failed and why. On return it holds NUL-terminated text;
    /// `logText` extracts it.
    pub fn bake(
        config: Config,
        mesh: TriMesh,
        authoring: ?AreaAuthoring,
        log: ?[]u8,
    ) err.Error!PolyMesh {
        const c_mesh = try mesh.toC();
        var c_log = c.BakeLog{
            .buffer = if (log) |buffer| buffer.ptr else null,
            .capacity = if (log) |buffer| buffer.len else 0,
        };
        var authoring_storage: c.AreaAuthoring = undefined;
        if (authoring) |a| authoring_storage = try a.toC();
        const c_authoring: ?*const c.AreaAuthoring =
            if (authoring != null) &authoring_storage else null;
        var handle: *c.PolyMesh = undefined;
        try err.check(c.zrcPolyMeshBake(
            &config,
            &c_mesh,
            c_authoring,
            if (log != null) &c_log else null,
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Bakes one tile of a tiled navmesh.
    ///
    /// `mesh` is the whole world's geometry every time; rasterisation clips it
    /// to the tile. `authoring` is optional, same contract as `bake`. `null`
    /// means the tile holds no walkable surface, which is the common case in a
    /// real world and is a success rather than an error — unlike `bake`, where
    /// nothing walkable anywhere is a mistake.
    pub fn bakeTile(
        config: Config,
        mesh: TriMesh,
        grid: TileGrid,
        tile_x: i32,
        tile_z: i32,
        authoring: ?AreaAuthoring,
        log: ?[]u8,
    ) err.Error!?PolyMesh {
        const c_mesh = try mesh.toC();
        var c_log = c.BakeLog{
            .buffer = if (log) |buffer| buffer.ptr else null,
            .capacity = if (log) |buffer| buffer.len else 0,
        };
        var authoring_storage: c.AreaAuthoring = undefined;
        if (authoring) |a| authoring_storage = try a.toC();
        const c_authoring: ?*const c.AreaAuthoring =
            if (authoring != null) &authoring_storage else null;
        var handle: ?*c.PolyMesh = null;
        try err.check(c.zrcPolyMeshBakeTile(
            &config,
            &c_mesh,
            &grid,
            tile_x,
            tile_z,
            c_authoring,
            if (log != null) &c_log else null,
            &handle,
        ));
        return if (handle) |h| PolyMesh{ .handle = h } else null;
    }

    pub fn deinit(self: PolyMesh) void {
        c.zrcPolyMeshDestroy(self.handle);
    }

    /// Records the agent this mesh was built for, in world units.
    ///
    /// Recast does not carry these and Detour demands them, so a mesh
    /// assembled stage by stage has to be told. `NavMesh.initFromPolyMesh`
    /// and `buildTileData` refuse a mesh that has not been.
    pub fn setAgentDims(
        self: PolyMesh,
        walkable_height: f32,
        walkable_radius: f32,
        walkable_climb: f32,
    ) err.Error!void {
        try err.check(c.zrcPolyMeshSetAgentDims(
            self.handle,
            walkable_height,
            walkable_radius,
            walkable_climb,
        ));
    }

    pub fn info(self: PolyMesh) err.Error!PolyMeshInfo {
        var out: PolyMeshInfo = undefined;
        try err.check(c.zrcPolyMeshInfo(self.handle, &out));
        return out;
    }

    /// Copies vertices into `out`, starting at `first`, one `[3]u16` each, in
    /// cells above the mesh's minimum corner. A range outside the array is an
    /// error rather than a short read.
    pub fn verts(self: PolyMesh, first: u32, out: [][3]u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcPolyMeshVerts(self.handle, first_i, count, ptr));
    }

    /// Copies polygons into `out`, starting at `first`.
    ///
    /// A polygon occupies `2 * verts_per_poly` entries: the corner indices
    /// first, `mesh_null_idx` past the last corner, then the neighbour across
    /// each edge. `out.len` must be a whole number of polygons, and the
    /// polygon array is `max_polys` long rather than `poly_count` — that is
    /// what indexes it.
    pub fn polys(self: PolyMesh, first: u32, out: []u16) err.Error!void {
        const stride = 2 * @as(usize, @intCast((try self.info()).verts_per_poly));
        if (stride == 0 or out.len % stride != 0) return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len / stride) orelse
            return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcPolyMeshPolys(self.handle, first_i, count, ptr));
    }

    /// Copies region ids into `out`, one per polygon. `multiple_regs` marks a
    /// polygon merged from more than one region.
    pub fn regions(self: PolyMesh, first: u32, out: []u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcPolyMeshRegions(self.handle, first_i, count, ptr));
    }

    /// Copies area ids into `out`, one per polygon.
    pub fn polyAreas(self: PolyMesh, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcPolyMeshPolyAreas(self.handle, first_i, count, ptr));
    }

    /// Writes area ids back, starting at `first`. Each must be below
    /// `max_areas`; a batch with one that is not is refused whole.
    pub fn setPolyAreas(self: PolyMesh, first: u32, areas: []const u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, areas.len) orelse return err.Error.InvalidArgument;
        const ptr: [*]const u8 = if (areas.len == 0) undefined else areas.ptr;
        try err.check(c.zrcPolyMeshSetPolyAreas(self.handle, first_i, count, ptr));
    }

    /// Copies polygon flags into `out`, one per polygon.
    pub fn polyFlags(self: PolyMesh, first: u32, out: []u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcPolyMeshPolyFlags(self.handle, first_i, count, ptr));
    }

    /// Writes polygon flags back, starting at `first`.
    ///
    /// A mesh assembled a stage at a time comes out with every flag zero,
    /// which no nonzero query filter admits, so this is not optional for one.
    /// `bake` assigns them already.
    pub fn setPolyFlags(self: PolyMesh, first: u32, flags: []const u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, flags.len) orelse return err.Error.InvalidArgument;
        const ptr: [*]const u16 = if (flags.len == 0) undefined else flags.ptr;
        try err.check(c.zrcPolyMeshSetPolyFlags(self.handle, first_i, count, ptr));
    }

    /// Copies detail sub-meshes into `out`, one `[4]u32` each: the first
    /// vertex and vertex count, then the first triangle and triangle count, of
    /// the polygon at the same index.
    pub fn detailMeshes(self: PolyMesh, first: u32, out: [][4]u32) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcPolyMeshDetailMeshes(self.handle, first_i, count, ptr));
    }

    /// Copies detail vertices into `out`, one `[3]f32` each, in world units.
    pub fn detailVerts(self: PolyMesh, first: u32, out: [][3]f32) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcPolyMeshDetailVerts(self.handle, first_i, count, ptr));
    }

    /// Copies detail triangles into `out`, one `[4]u8` each: three corner
    /// indices into the owning sub-mesh's vertices, then the edge flags.
    pub fn detailTris(self: PolyMesh, first: u32, out: [][4]u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcPolyMeshDetailTris(self.handle, first_i, count, ptr));
    }
};

/// The voxel quantities a `Config` implies, which is what the staged pipeline
/// takes.
pub const BuildCells = c.BuildCells;

/// Converts `config` into the voxel quantities the staged entry points take.
///
/// The staged stages take voxels because Recast's own do; `Config` describes an
/// agent in world units. This is the conversion `PolyMesh.bake` performs
/// internally, so a host that drives the stages with these numbers reproduces a
/// bake exactly rather than approximately. Fails for exactly the configurations
/// `bake` rejects.
pub fn buildCells(config: Config) err.Error!BuildCells {
    var out: BuildCells = undefined;
    try err.check(c.zrcBakeConfigCells(&config, &out));
    return out;
}

/// The NUL-terminated prefix of a bake log buffer.
///
/// The C side fills the buffer as a C string; this is the slice up to the
/// terminator, or the whole buffer if the writer somehow left none.
pub fn logText(buffer: []const u8) []const u8 {
    const end = std.mem.indexOfScalar(u8, buffer, 0) orelse buffer.len;
    return buffer[0..end];
}

test "the default config describes a human-sized agent" {
    const config = defaultConfig();
    try std.testing.expect(config.cell_size > 0);
    try std.testing.expect(config.cell_height > 0);
    try std.testing.expect(config.agent_height > config.agent_radius);
    try std.testing.expectEqual(Partition.watershed, config.partition);
    try std.testing.expectEqual(@as(i32, c.verts_per_polygon), config.verts_per_poly);
}

test "a triangle soup with a length that is not a multiple of three is refused" {
    const verts = [_]f32{ 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    const tris = [_]i32{ 0, 1, 2 };

    try std.testing.expectError(
        err.Error.InvalidArgument,
        (TriMesh{ .verts = verts[0..8], .tris = &tris }).toC(),
    );
    try std.testing.expectError(
        err.Error.InvalidArgument,
        (TriMesh{ .verts = &verts, .tris = tris[0..2] }).toC(),
    );
    try std.testing.expectError(
        err.Error.InvalidArgument,
        (TriMesh{ .verts = &.{}, .tris = &tris }).toC(),
    );
}

test "logText stops at the terminator" {
    var buffer = [_]u8{ 'a', 'b', 0, 'x', 'y' };
    try std.testing.expectEqualStrings("ab", logText(&buffer));
    try std.testing.expectEqualStrings("abc", logText("abc"));
}
