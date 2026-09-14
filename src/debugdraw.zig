//! Drawing what a bake produced, and dumping it to a host's bytes.
//!
//! Upstream's `DebugUtils` decides what a heightfield, a region, a contour set
//! or a navmesh looks like as points, lines, triangles and quads, and hands
//! each primitive to a renderer the host supplies. `DebugDraw` is that
//! renderer: a table of hooks, either filled in by hand or derived from a Zig
//! type's methods by `DebugDraw.of`, so a renderer is a Zig struct rather than
//! a C callback table a caller has to spell out.
//!
//! Every draw call is a method on the renderer, because the renderer is what
//! upstream takes first and what the primitives are emitted into. None of them
//! allocates, and none changes the container it is drawing.
//!
//! `FileIO` is the same shape for bytes: four hooks, and no file is opened or
//! named here. `dumpPolyMeshToObj` and its siblings write through it;
//! `readContourSet` and `readCompactHeightfield` build a container back out of
//! what one of them wrote.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const Vec3 = @import("vec.zig").Vec3;
const PolyMesh = @import("bake.zig").PolyMesh;
const NavMesh = @import("navmesh.zig").NavMesh;
const NavMeshQuery = @import("query.zig").NavMeshQuery;
const PolyRef = @import("query.zig").PolyRef;
const BuildContext = @import("pipeline.zig").BuildContext;
const Heightfield = @import("pipeline.zig").Heightfield;
const CompactHeightfield = @import("pipeline.zig").CompactHeightfield;
const ContourSet = @import("pipeline.zig").ContourSet;
const HeightfieldLayerSet = @import("tilecache.zig").HeightfieldLayerSet;
const TileCacheLayer = @import("tilecache.zig").TileCacheLayer;
const TileCacheContourSet = @import("tilecache.zig").TileCacheContourSet;
const TileCachePolyMesh = @import("tilecache.zig").TileCachePolyMesh;

//=============================================================================
// Colours
//=============================================================================

/// Packs four 0-255 channels into the 0xAABBGGRR word every colour argument
/// here takes.
pub fn rgba(r: i32, g: i32, b: i32, a: i32) u32 {
    return c.zrcDebugRgba(r, g, b, a);
}

/// The same from four 0..1 floats, each multiplied by 255 and truncated.
pub fn rgbaFloat(r: f32, g: f32, b: f32, a: f32) u32 {
    return c.zrcDebugRgbaFloat(r, g, b, a);
}

/// A repeatable colour for an integer: six bits of `i` become three channels,
/// so the table repeats every 64 values.
pub fn intToCol(i: i32, a: i32) u32 {
    return c.zrcDebugIntToCol(i, a);
}

/// Upstream's other overload of the same name, as 0..1 floats. It is not the
/// float form of `intToCol`: it assigns the six bits to different channels
/// and inverts them, so the two produce different colours for the same `i`.
pub fn intToColFloat(i: i32) [3]f32 {
    var out: [3]f32 = undefined;
    // The only failure is a null pointer, which this cannot pass.
    err.check(c.zrcDebugIntToColFloat(i, &out)) catch unreachable;
    return out;
}

/// Scales every colour channel by `d`/256, leaving alpha alone.
pub fn multCol(col: u32, d: u32) u32 {
    return c.zrcDebugMultCol(col, d);
}

/// Halves every colour channel, leaving alpha alone.
pub fn darkenCol(col: u32) u32 {
    return c.zrcDebugDarkenCol(col);
}

/// Blends two colours, `u` running 0 (all `a`) to 255 (all `b`).
pub fn lerpCol(a: u32, b: u32, u: u32) u32 {
    return c.zrcDebugLerpCol(a, b, u);
}

/// Replaces a colour's alpha.
pub fn transCol(col: u32, a: u32) u32 {
    return c.zrcDebugTransCol(col, a);
}

/// The six face colours `DebugDraw.box` wants: the top colour, the side
/// colour, and the four darkened variants.
pub fn boxColors(top: u32, side: u32) [6]u32 {
    var out: [6]u32 = undefined;
    err.check(c.zrcDebugCalcBoxColors(&out, top, side)) catch unreachable;
    return out;
}

//=============================================================================
// The renderer
//=============================================================================

/// What a `begin`/`end` pair between them describes.
pub const Primitive = c.DebugDrawPrimitive;

/// What `DebugDraw.navMesh` should include beyond the polygons themselves.
pub const NavMeshDrawOptions = struct {
    /// The off-mesh connections, as arcs between their endpoints.
    off_mesh_connections: bool = false,
    /// Shade each polygon the last search closed. Only the
    /// `navMeshWithClosedList` form has a search to read.
    closed_list: bool = false,
    /// One colour per tile rather than one colour for the mesh.
    color_tiles: bool = false,

    fn toC(self: NavMeshDrawOptions) u8 {
        var bits: u8 = 0;
        if (self.off_mesh_connections) bits |= c.drawnavmesh_offmeshcons;
        if (self.closed_list) bits |= c.drawnavmesh_closedlist;
        if (self.color_tiles) bits |= c.drawnavmesh_color_tiles;
        return bits;
    }
};

/// The renderer every draw call emits into.
///
/// Every hook but `area_to_col` is required: upstream declares all of them
/// pure, and which one a draw call reaches depends on the shape it is drawing,
/// so a partly filled table would work on some containers and fail on others.
/// `area_to_col` left `null` selects upstream's own area colour table.
///
/// `of` is how a Zig renderer arrives: it reads the methods off a pointer's
/// type at compile time and fills the table, so nothing about C appears in the
/// renderer itself. Filling the fields by hand is still available for a host
/// that already has C hooks to point at.
pub const DebugDraw = struct {
    user: ?*anyopaque = null,
    depth_mask: ?*const fn (user: ?*anyopaque, state: c.Bool) callconv(.c) void = null,
    texture: ?*const fn (user: ?*anyopaque, state: c.Bool) callconv(.c) void = null,
    begin: ?*const fn (
        user: ?*anyopaque,
        prim: Primitive,
        size: f32,
    ) callconv(.c) void = null,
    vertex: ?*const fn (
        user: ?*anyopaque,
        pos: *const [3]f32,
        color: u32,
    ) callconv(.c) void = null,
    vertex_xyz: ?*const fn (
        user: ?*anyopaque,
        x: f32,
        y: f32,
        z: f32,
        color: u32,
    ) callconv(.c) void = null,
    vertex_uv: ?*const fn (
        user: ?*anyopaque,
        pos: *const [3]f32,
        color: u32,
        uv: *const [2]f32,
    ) callconv(.c) void = null,
    vertex_xyz_uv: ?*const fn (
        user: ?*anyopaque,
        x: f32,
        y: f32,
        z: f32,
        color: u32,
        u: f32,
        v: f32,
    ) callconv(.c) void = null,
    end: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
    area_to_col: ?*const fn (user: ?*anyopaque, area: u32) callconv(.c) u32 = null,

    pub fn toC(self: DebugDraw) c.DebugDraw {
        return .{
            .user = self.user,
            .depth_mask = self.depth_mask,
            .texture = self.texture,
            .begin = self.begin,
            .vertex = self.vertex,
            .vertex_xyz = self.vertex_xyz,
            .vertex_uv = self.vertex_uv,
            .vertex_xyz_uv = self.vertex_xyz_uv,
            .end = self.end,
            .area_to_col = self.area_to_col,
        };
    }

    /// Builds the table from `renderer`, a pointer to a Zig value whose type
    /// declares the drawing methods.
    ///
    /// Required: `depthMask(bool)`, `texture(bool)`, `begin(Primitive, f32)`,
    /// `vertex(Vec3, u32)` and `end()`. A type that omits one of these is a
    /// compile error naming it.
    ///
    /// Optional, each with a default that discards what the renderer does not
    /// ask for: `vertexXyz(f32, f32, f32, u32)` splits the position back out,
    /// `vertexUv(Vec3, u32, [2]f32)` and `vertexXyzUv(f32, f32, f32, u32,
    /// f32, f32)` carry texture coordinates, and `areaToCol(u32) u32` replaces
    /// upstream's area colours. Left undeclared, the three vertex forms fall
    /// through to `vertex` and the colours stay upstream's.
    ///
    /// `renderer` is borrowed: the table points at it, and both must outlive
    /// every draw call made through it.
    pub fn of(renderer: anytype) DebugDraw {
        const Ptr = @TypeOf(renderer);
        const info = @typeInfo(Ptr);
        if (info != .pointer or info.pointer.size != .one) {
            @compileError("DebugDraw.of takes a single-item pointer to the renderer");
        }
        const T = info.pointer.child;
        inline for (.{ "depthMask", "texture", "begin", "vertex", "end" }) |name| {
            if (!@hasDecl(T, name)) {
                @compileError(@typeName(T) ++ " has no " ++ name ++ ", which a renderer must declare");
            }
        }

        const adapt = struct {
            fn self(user: ?*anyopaque) Ptr {
                return @ptrCast(@alignCast(user.?));
            }
            fn depthMask(user: ?*anyopaque, state: c.Bool) callconv(.c) void {
                self(user).depthMask(state != c.c_false);
            }
            fn texture(user: ?*anyopaque, state: c.Bool) callconv(.c) void {
                self(user).texture(state != c.c_false);
            }
            fn begin(user: ?*anyopaque, prim: Primitive, size: f32) callconv(.c) void {
                self(user).begin(prim, size);
            }
            fn vertex(user: ?*anyopaque, pos: *const [3]f32, color: u32) callconv(.c) void {
                self(user).vertex(pos.*, color);
            }
            fn vertexXyz(user: ?*anyopaque, x: f32, y: f32, z: f32, color: u32) callconv(.c) void {
                if (@hasDecl(T, "vertexXyz")) {
                    self(user).vertexXyz(x, y, z, color);
                } else {
                    self(user).vertex(.{ x, y, z }, color);
                }
            }
            fn vertexUv(
                user: ?*anyopaque,
                pos: *const [3]f32,
                color: u32,
                uv: *const [2]f32,
            ) callconv(.c) void {
                if (@hasDecl(T, "vertexUv")) {
                    self(user).vertexUv(pos.*, color, uv.*);
                } else {
                    self(user).vertex(pos.*, color);
                }
            }
            fn vertexXyzUv(
                user: ?*anyopaque,
                x: f32,
                y: f32,
                z: f32,
                color: u32,
                u: f32,
                v: f32,
            ) callconv(.c) void {
                if (@hasDecl(T, "vertexXyzUv")) {
                    self(user).vertexXyzUv(x, y, z, color, u, v);
                } else if (@hasDecl(T, "vertexUv")) {
                    self(user).vertexUv(.{ x, y, z }, color, .{ u, v });
                } else {
                    self(user).vertex(.{ x, y, z }, color);
                }
            }
            fn end(user: ?*anyopaque) callconv(.c) void {
                self(user).end();
            }
            fn areaToCol(user: ?*anyopaque, area: u32) callconv(.c) u32 {
                return self(user).areaToCol(area);
            }
        };

        return .{
            .user = @constCast(@as(*const anyopaque, renderer)),
            .depth_mask = adapt.depthMask,
            .texture = adapt.texture,
            .begin = adapt.begin,
            .vertex = adapt.vertex,
            .vertex_xyz = adapt.vertexXyz,
            .vertex_uv = adapt.vertexUv,
            .vertex_xyz_uv = adapt.vertexXyzUv,
            .end = adapt.end,
            .area_to_col = if (@hasDecl(T, "areaToCol")) adapt.areaToCol else null,
        };
    }

    //-------------------------------------------------------------------------
    // Primitives. The `append*` forms emit vertices into a run the caller
    // opened, so several shapes can share one; the rest open and close their
    // own.
    //-------------------------------------------------------------------------

    pub fn boxWire(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32, line_width: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawBoxWire(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
            line_width,
        ));
    }

    pub fn cylinderWire(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32, line_width: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCylinderWire(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
            line_width,
        ));
    }

    /// An arc from `from` to `to`, bulging `height` times its own length
    /// upwards. `head_start` and `head_end` are arrowhead sizes, 0 for none.
    pub fn arc(
        self: DebugDraw,
        from: Vec3,
        to: Vec3,
        height: f32,
        head_start: f32,
        head_end: f32,
        col: u32,
        line_width: f32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawArc(
            &dd,
            from[0],
            from[1],
            from[2],
            to[0],
            to[1],
            to[2],
            height,
            head_start,
            head_end,
            col,
            line_width,
        ));
    }

    pub fn arrow(
        self: DebugDraw,
        from: Vec3,
        to: Vec3,
        head_start: f32,
        head_end: f32,
        col: u32,
        line_width: f32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawArrow(
            &dd,
            from[0],
            from[1],
            from[2],
            to[0],
            to[1],
            to[2],
            head_start,
            head_end,
            col,
            line_width,
        ));
    }

    /// A circle in the xz plane, centred on `center`.
    pub fn circle(self: DebugDraw, center: Vec3, radius: f32, col: u32, line_width: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCircle(
            &dd,
            center[0],
            center[1],
            center[2],
            radius,
            col,
            line_width,
        ));
    }

    /// Three axis-aligned segments through `center`, each `size` long each way.
    pub fn cross(self: DebugDraw, center: Vec3, size: f32, col: u32, line_width: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCross(
            &dd,
            center[0],
            center[1],
            center[2],
            size,
            col,
            line_width,
        ));
    }

    /// A solid box, one colour per face as `boxColors` writes them.
    pub fn box(self: DebugDraw, bmin: Vec3, bmax: Vec3, face_colors: *const [6]u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawBox(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            face_colors,
        ));
    }

    pub fn cylinder(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCylinder(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
        ));
    }

    /// A `width` by `height` grid of `size`-wide cells in the xz plane, its
    /// corner at `origin`.
    pub fn gridXZ(
        self: DebugDraw,
        origin: Vec3,
        width: u32,
        height: u32,
        size: f32,
        col: u32,
        line_width: f32,
    ) err.Error!void {
        const w = std.math.cast(i32, width) orelse return err.Error.InvalidArgument;
        const h = std.math.cast(i32, height) orelse return err.Error.InvalidArgument;
        const dd = self.toC();
        try err.check(c.zrcDebugDrawGridXZ(
            &dd,
            origin[0],
            origin[1],
            origin[2],
            w,
            h,
            size,
            col,
            line_width,
        ));
    }

    pub fn appendBoxWire(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendBoxWire(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
        ));
    }

    /// The top and bottom outlines of the box, four segments each — sixteen
    /// vertices, with every corner emitted twice.
    pub fn appendBoxPoints(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendBoxPoints(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
        ));
    }

    pub fn appendCylinderWire(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendCylinderWire(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
        ));
    }

    pub fn appendArc(
        self: DebugDraw,
        from: Vec3,
        to: Vec3,
        height: f32,
        head_start: f32,
        head_end: f32,
        col: u32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendArc(
            &dd,
            from[0],
            from[1],
            from[2],
            to[0],
            to[1],
            to[2],
            height,
            head_start,
            head_end,
            col,
        ));
    }

    pub fn appendArrow(
        self: DebugDraw,
        from: Vec3,
        to: Vec3,
        head_start: f32,
        head_end: f32,
        col: u32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendArrow(
            &dd,
            from[0],
            from[1],
            from[2],
            to[0],
            to[1],
            to[2],
            head_start,
            head_end,
            col,
        ));
    }

    pub fn appendCircle(self: DebugDraw, center: Vec3, radius: f32, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendCircle(
            &dd,
            center[0],
            center[1],
            center[2],
            radius,
            col,
        ));
    }

    pub fn appendCross(self: DebugDraw, center: Vec3, size: f32, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendCross(
            &dd,
            center[0],
            center[1],
            center[2],
            size,
            col,
        ));
    }

    pub fn appendBox(self: DebugDraw, bmin: Vec3, bmax: Vec3, face_colors: *const [6]u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendBox(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            face_colors,
        ));
    }

    pub fn appendCylinder(self: DebugDraw, bmin: Vec3, bmax: Vec3, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugAppendCylinder(
            &dd,
            bmin[0],
            bmin[1],
            bmin[2],
            bmax[0],
            bmax[1],
            bmax[2],
            col,
        ));
    }

    //-------------------------------------------------------------------------
    // What Recast built
    //-------------------------------------------------------------------------

    /// The input soup, shaded by face normal and by walkability.
    ///
    /// `normals` is three floats per triangle and is required: upstream draws
    /// nothing at all without it. `flags` is one byte per triangle, zero for
    /// one to be tinted unwalkable, or `null` to tint none.
    pub fn triMesh(
        self: DebugDraw,
        mesh: @import("bake.zig").TriMesh,
        normals: []const f32,
        flags: ?[]const u8,
        tex_scale: f32,
    ) err.Error!void {
        const tri_count = mesh.tris.len / 3;
        if (normals.len != tri_count * 3) return err.Error.InvalidArgument;
        if (flags) |f| {
            if (f.len != tri_count) return err.Error.InvalidArgument;
        }
        const raw = try mesh.toC();
        const dd = self.toC();
        try err.check(c.zrcDebugDrawTriMesh(
            &dd,
            &raw,
            normals.ptr,
            if (flags) |f| f.ptr else null,
            tex_scale,
        ));
    }

    /// The same, tinting by slope against `walkable_slope_angle` in degrees
    /// rather than by a caller's flags.
    pub fn triMeshSlope(
        self: DebugDraw,
        mesh: @import("bake.zig").TriMesh,
        normals: []const f32,
        walkable_slope_angle: f32,
        tex_scale: f32,
    ) err.Error!void {
        const tri_count = mesh.tris.len / 3;
        if (normals.len != tri_count * 3) return err.Error.InvalidArgument;
        const raw = try mesh.toC();
        const dd = self.toC();
        try err.check(c.zrcDebugDrawTriMeshSlope(
            &dd,
            &raw,
            normals.ptr,
            walkable_slope_angle,
            tex_scale,
        ));
    }

    /// Every span in the heightfield as a box.
    pub fn heightfieldSolid(self: DebugDraw, field: Heightfield) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawHeightfieldSolid(&dd, field.handle));
    }

    /// The same, colouring a span by the area id it carries.
    pub fn heightfieldWalkable(self: DebugDraw, field: Heightfield) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawHeightfieldWalkable(&dd, field.handle));
    }

    pub fn compactHeightfieldSolid(self: DebugDraw, field: CompactHeightfield) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCompactHeightfieldSolid(&dd, field.handle));
    }

    /// Each span coloured by its region id; spans in no region are grey.
    pub fn compactHeightfieldRegions(self: DebugDraw, field: CompactHeightfield) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCompactHeightfieldRegions(&dd, field.handle));
    }

    /// Each span coloured by its distance to the nearest border.
    /// `error.EmptyResult` before `buildDistanceField` has run.
    pub fn compactHeightfieldDistance(self: DebugDraw, field: CompactHeightfield) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawCompactHeightfieldDistance(&dd, field.handle));
    }

    /// One sheet of a layered heightfield, coloured by its index.
    pub fn heightfieldLayer(self: DebugDraw, layers: HeightfieldLayerSet, index: u32) err.Error!void {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const dd = self.toC();
        try err.check(c.zrcDebugDrawHeightfieldLayer(&dd, layers.handle, i));
    }

    /// Every sheet, each in its own colour.
    ///
    /// Upstream declares a third form, `duDebugDrawHeightfieldLayersRegions`,
    /// that the vendored tree never defines and that would have nothing to
    /// draw: `rcHeightfieldLayer` carries no region ids. There is no method
    /// for it here.
    pub fn heightfieldLayers(self: DebugDraw, layers: HeightfieldLayerSet) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawHeightfieldLayers(&dd, layers.handle));
    }

    /// Arcs joining each contour vertex to the region it borders, which is
    /// what makes a mis-merged region visible. [Limit: 0 <= alpha <= 1]
    pub fn regionConnections(self: DebugDraw, set: ContourSet, alpha: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawRegionConnections(&dd, set.handle, alpha));
    }

    /// The traced outlines, before simplification.
    pub fn rawContours(self: DebugDraw, set: ContourSet, alpha: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawRawContours(&dd, set.handle, alpha));
    }

    /// The simplified outlines the polygon mesh is built from.
    pub fn contours(self: DebugDraw, set: ContourSet, alpha: f32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawContours(&dd, set.handle, alpha));
    }

    /// Filled polygons, their internal and boundary edges, and their vertices.
    pub fn polyMesh(self: DebugDraw, mesh: PolyMesh) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawPolyMesh(&dd, mesh.handle));
    }

    /// The detail mesh that restores the height Recast quantised away. A
    /// mesh that has not been through `polyMeshBuildDetail` draws nothing.
    pub fn polyMeshDetail(self: DebugDraw, mesh: PolyMesh) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawPolyMeshDetail(&dd, mesh.handle));
    }

    //-------------------------------------------------------------------------
    // What Detour loaded
    //-------------------------------------------------------------------------

    /// Every tile's polygons, plus whatever `options` asks for.
    pub fn navMesh(self: DebugDraw, mesh: NavMesh, options: NavMeshDrawOptions) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMesh(&dd, mesh.handle, options.toC()));
    }

    /// The same, shading each polygon the last search on `query` closed.
    pub fn navMeshWithClosedList(
        self: DebugDraw,
        mesh: NavMesh,
        query: NavMeshQuery,
        options: NavMeshDrawOptions,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMeshWithClosedList(
            &dd,
            mesh.handle,
            query.handle,
            options.toC(),
        ));
    }

    /// The search's node pool: one point per node, and a line to its parent.
    pub fn navMeshNodes(self: DebugDraw, query: NavMeshQuery) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMeshNodes(&dd, query.handle));
    }

    /// Every tile's bounding-volume tree as nested boxes. A tile built
    /// without one contributes nothing.
    pub fn navMeshBvTree(self: DebugDraw, mesh: NavMesh) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMeshBVTree(&dd, mesh.handle));
    }

    /// The portal edges tiles join across.
    pub fn navMeshPortals(self: DebugDraw, mesh: NavMesh) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMeshPortals(&dd, mesh.handle));
    }

    /// Every polygon sharing a bit with `poly_flags`, in one colour. Zero
    /// matches nothing, the same rule a query filter follows.
    pub fn navMeshPolysWithFlags(
        self: DebugDraw,
        mesh: NavMesh,
        poly_flags: u16,
        col: u32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMeshPolysWithFlags(&dd, mesh.handle, poly_flags, col));
    }

    /// One polygon. A reference naming no resident polygon draws nothing and
    /// is not an error, upstream's own behaviour.
    pub fn navMeshPoly(self: DebugDraw, mesh: NavMesh, ref: PolyRef, col: u32) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawNavMeshPoly(&dd, mesh.handle, ref, col));
    }

    /// A decompressed tile-cache layer, each cell coloured by its area id.
    pub fn tileCacheLayerAreas(
        self: DebugDraw,
        layer: TileCacheLayer,
        cell_size: f32,
        cell_height: f32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawTileCacheLayerAreas(&dd, layer.handle, cell_size, cell_height));
    }

    /// The same, coloured by region id. A layer that has not been through
    /// `buildRegions` carries an all-zero region grid and draws as one
    /// region.
    pub fn tileCacheLayerRegions(
        self: DebugDraw,
        layer: TileCacheLayer,
        cell_size: f32,
        cell_height: f32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawTileCacheLayerRegions(&dd, layer.handle, cell_size, cell_height));
    }

    /// The outlines a tile-cache rebuild traced.
    pub fn tileCacheContours(
        self: DebugDraw,
        set: TileCacheContourSet,
        origin: Vec3,
        cell_size: f32,
        cell_height: f32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawTileCacheContours(
            &dd,
            set.handle,
            &origin,
            cell_size,
            cell_height,
        ));
    }

    /// The polygon mesh a tile-cache rebuild produced, before it becomes a
    /// tile.
    pub fn tileCachePolyMesh(
        self: DebugDraw,
        mesh: TileCachePolyMesh,
        origin: Vec3,
        cell_size: f32,
        cell_height: f32,
    ) err.Error!void {
        const dd = self.toC();
        try err.check(c.zrcDebugDrawTileCachePolyMesh(
            &dd,
            mesh.handle,
            &origin,
            cell_size,
            cell_height,
        ));
    }
};

//=============================================================================
// The display list
//=============================================================================

/// One recorded run of primitives, replayable into any renderer.
///
/// Upstream's `duDisplayList` records a single run: `begin` discards whatever
/// the list already held, so a list carries the last run submitted to it and
/// nothing earlier. Its vertex arrays come from C++'s `new[]` rather than from
/// the allocator `setAllocator` installs — upstream's own code, and not
/// reachable from here.
pub const DisplayList = struct {
    handle: *c.DisplayList,

    /// [Limit: 0 <= capacity <= 0x10000000; upstream raises anything under
    /// 8 to 8]
    pub fn init(capacity: u32) err.Error!DisplayList {
        const cap = std.math.cast(i32, capacity) orelse return err.Error.InvalidArgument;
        var handle: *c.DisplayList = undefined;
        try err.check(c.zrcDisplayListCreate(cap, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: DisplayList) void {
        c.zrcDisplayListDestroy(self.handle);
    }

    /// The list wearing the renderer interface, so a draw call records into
    /// it. The table borrows the list and dies with it.
    pub fn recorder(self: DisplayList) DebugDraw {
        var out: c.DebugDraw = undefined;
        err.check(c.zrcDisplayListRecorder(self.handle, &out)) catch unreachable;
        return .{
            .user = out.user,
            .depth_mask = out.depth_mask,
            .texture = out.texture,
            .begin = out.begin,
            .vertex = out.vertex,
            .vertex_xyz = out.vertex_xyz,
            .vertex_uv = out.vertex_uv,
            .vertex_xyz_uv = out.vertex_xyz_uv,
            .end = out.end,
            .area_to_col = out.area_to_col,
        };
    }

    pub fn setDepthMask(self: DisplayList, state: bool) err.Error!void {
        try err.check(c.zrcDisplayListDepthMask(
            self.handle,
            if (state) c.c_true else c.c_false,
        ));
    }

    /// Opens a run, discarding whatever the list already held.
    pub fn begin(self: DisplayList, prim: Primitive, size: f32) err.Error!void {
        try err.check(c.zrcDisplayListBegin(self.handle, prim, size));
    }

    pub fn vertex(self: DisplayList, pos: Vec3, color: u32) err.Error!void {
        try err.check(c.zrcDisplayListVertex(self.handle, &pos, color));
    }

    /// The same with the position spelled out, which is the form upstream's
    /// own replay reaches for.
    pub fn vertexXyz(self: DisplayList, x: f32, y: f32, z: f32, color: u32) err.Error!void {
        try err.check(c.zrcDisplayListVertexXYZ(self.handle, x, y, z, color));
    }

    pub fn end(self: DisplayList) err.Error!void {
        try err.check(c.zrcDisplayListEnd(self.handle));
    }

    /// Drops every recorded vertex, keeping the primitive type and size.
    pub fn clear(self: DisplayList) err.Error!void {
        try err.check(c.zrcDisplayListClear(self.handle));
    }

    /// Replays the recorded run into `dd`. An empty list draws nothing at
    /// all, not even the `begin`/`end` pair.
    pub fn draw(self: DisplayList, dd: DebugDraw) err.Error!void {
        const raw = dd.toC();
        try err.check(c.zrcDisplayListDraw(self.handle, &raw));
    }

    /// How many vertices the list holds. Upstream keeps this private and
    /// offers no accessor.
    pub fn count(self: DisplayList) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcDisplayListVertexCount(self.handle, &out));
        return @intCast(out);
    }
};

//=============================================================================
// Dumping a build to a host's bytes
//=============================================================================

/// The byte sink or source a dump reads and writes through. Every hook is
/// required, and one returning `false` aborts the dump that called it.
///
/// `of` derives the table from a Zig type's `isWriting`, `isReading`, `write`
/// and `read` methods, the same way `DebugDraw.of` does.
pub const FileIO = struct {
    user: ?*anyopaque = null,
    is_writing: ?*const fn (user: ?*anyopaque) callconv(.c) c.Bool = null,
    is_reading: ?*const fn (user: ?*anyopaque) callconv(.c) c.Bool = null,
    write: ?*const fn (
        user: ?*anyopaque,
        ptr: *const anyopaque,
        size: usize,
    ) callconv(.c) c.Bool = null,
    read: ?*const fn (
        user: ?*anyopaque,
        ptr: *anyopaque,
        size: usize,
    ) callconv(.c) c.Bool = null,

    pub fn toC(self: FileIO) c.FileIO {
        return .{
            .user = self.user,
            .is_writing = self.is_writing,
            .is_reading = self.is_reading,
            .write = self.write,
            .read = self.read,
        };
    }

    /// Builds the table from `stream`, a pointer to a Zig value whose type
    /// declares `isWriting() bool`, `isReading() bool`, `write([]const u8)
    /// bool` and `read([]u8) bool`. The stream is borrowed and must outlive
    /// every dump made through it.
    pub fn of(stream: anytype) FileIO {
        const Ptr = @TypeOf(stream);
        const info = @typeInfo(Ptr);
        if (info != .pointer or info.pointer.size != .one) {
            @compileError("FileIO.of takes a single-item pointer to the stream");
        }
        const T = info.pointer.child;
        inline for (.{ "isWriting", "isReading", "write", "read" }) |name| {
            if (!@hasDecl(T, name)) {
                @compileError(@typeName(T) ++ " has no " ++ name ++ ", which a stream must declare");
            }
        }

        const adapt = struct {
            fn self(user: ?*anyopaque) Ptr {
                return @ptrCast(@alignCast(user.?));
            }
            fn isWriting(user: ?*anyopaque) callconv(.c) c.Bool {
                return if (self(user).isWriting()) c.c_true else c.c_false;
            }
            fn isReading(user: ?*anyopaque) callconv(.c) c.Bool {
                return if (self(user).isReading()) c.c_true else c.c_false;
            }
            fn write(user: ?*anyopaque, ptr: *const anyopaque, size: usize) callconv(.c) c.Bool {
                const bytes: [*]const u8 = @ptrCast(ptr);
                return if (self(user).write(bytes[0..size])) c.c_true else c.c_false;
            }
            fn read(user: ?*anyopaque, ptr: *anyopaque, size: usize) callconv(.c) c.Bool {
                const bytes: [*]u8 = @ptrCast(ptr);
                return if (self(user).read(bytes[0..size])) c.c_true else c.c_false;
            }
        };

        return .{
            .user = @constCast(@as(*const anyopaque, stream)),
            .is_writing = adapt.isWriting,
            .is_reading = adapt.isReading,
            .write = adapt.write,
            .read = adapt.read,
        };
    }
};

/// Writes the polygon mesh as a Wavefront OBJ: one `v` line per vertex, one
/// `f` line per triangle of each polygon's fan.
pub fn dumpPolyMeshToObj(mesh: PolyMesh, io: FileIO) err.Error!void {
    const raw = io.toC();
    try err.check(c.zrcDumpPolyMeshToObj(mesh.handle, &raw));
}

/// The same for the detail mesh. One that has not been built writes the two
/// header lines and no geometry.
pub fn dumpPolyMeshDetailToObj(mesh: PolyMesh, io: FileIO) err.Error!void {
    const raw = io.toC();
    try err.check(c.zrcDumpPolyMeshDetailToObj(mesh.handle, &raw));
}

/// Writes a contour set in upstream's own binary form.
pub fn dumpContourSet(contours: ContourSet, io: FileIO) err.Error!void {
    const raw = io.toC();
    try err.check(c.zrcDumpContourSet(contours.handle, &raw));
}

/// Reads one back. The result is the caller's, released with `deinit`.
///
/// The format is upstream's own struct layout copied to the stream: neither
/// endian- nor padding-portable, and carrying no length a reader could check
/// a count against. Feed it only bytes `dumpContourSet` wrote, in this build,
/// on this target.
pub fn readContourSet(io: FileIO) err.Error!ContourSet {
    const raw = io.toC();
    var handle: *c.ContourSet = undefined;
    try err.check(c.zrcReadContourSet(&raw, &handle));
    return .{ .handle = handle };
}

/// Writes a compact heightfield in upstream's own binary form. Whichever of
/// the cell, span, distance and area arrays exist are written; the rest are
/// recorded as absent.
pub fn dumpCompactHeightfield(field: CompactHeightfield, io: FileIO) err.Error!void {
    const raw = io.toC();
    try err.check(c.zrcDumpCompactHeightfield(field.handle, &raw));
}

/// Reads one back, under the same caution as `readContourSet`.
pub fn readCompactHeightfield(io: FileIO) err.Error!CompactHeightfield {
    const raw = io.toC();
    var handle: *c.CompactHeightfield = undefined;
    try err.check(c.zrcReadCompactHeightfield(&raw, &handle));
    return .{ .handle = handle };
}

/// Logs each build phase's accumulated time and its share of `total_usec`
/// through the context's own log hook, as `.progress` lines.
///
/// The context needs logging enabled and a log hook, or upstream formats
/// twenty-six lines and discards every one. [Limit: 0 < total_usec]
pub fn logBuildTimes(context: ?*const BuildContext, total_usec: i32) err.Error!void {
    var storage: c.BuildContext = undefined;
    const ctx: ?*const c.BuildContext = if (context) |value| blk: {
        storage = value.toC();
        break :blk &storage;
    } else null;
    try err.check(c.zrcLogBuildTimes(ctx, total_usec));
}
