//! The seam between the two lifecycles: a baked polygon mesh becomes bytes on
//! one side of a build, and a queryable navmesh on the other.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const PolyMesh = @import("bake.zig").PolyMesh;
const TileGrid = @import("bake.zig").TileGrid;

/// A tile reference. Zero means "no tile".
///
/// Carries a salt that changes when its slot is reused, so a reference held
/// across a removal is detected rather than pointing at whatever now occupies
/// the slot.
pub const TileRef = c.TileRef;

/// Which kind of polygon a reference names.
pub const PolyType = c.PolyType;

/// One point-to-point link between two places the surface does not join: a
/// jump down a ledge, a ladder, a door, a teleporter. A two-vertex polygon of
/// its own, linked into the graph at both ends.
///
/// `end_side` is output only: `NavMesh.offMeshConnection` fills it in, a build
/// ignores it, and it carries a default so a literal need not mention it.
pub const OffMeshConnection = c.OffMeshConnection;

/// How a navmesh's tile grid was sized, as `NavMesh.initTiled` set it up.
pub const NavMeshParams = c.NavMeshParams;

/// Everything a tile's header says about it, copied out.
pub const TileInfo = c.TileInfo;

/// One polygon, copied out of the tile that holds it.
pub const PolyInfo = c.PolyInfo;

/// One link: an edge from a polygon to a neighbour it can be walked to.
pub const Link = c.Link;

/// One node of a tile's bounding-volume tree.
pub const BvNode = c.BvNode;

/// Which tile of the grid a world position falls in, from `NavMesh.calcTileLoc`.
pub const TileLoc = struct {
    x: i32,
    z: i32,
};

/// What a tile carries beyond the shape of its surface.
///
/// Both fields are optional: the zero value authors nothing, the same
/// uniform-space default `bake.AreaAuthoring`'s zero value gives a bake.
pub const TileAuthoring = struct {
    /// The connections this tile owns. A connection is stored by the tile
    /// that contains endpoint A; keep both endpoints inside one tile, or
    /// build a tiled navmesh, or the far end is left unwired. See
    /// `ZrcTileAuthoring` in zrecast.h for the full rule.
    connections: []const OffMeshConnection = &.{},

    /// Opaque to zrecast, stored in the tile header and readable back with
    /// `NavMesh.tileUserId`.
    user_id: u32 = 0,

    /// Omits the tile's bounding-volume tree. Without it Detour finds the
    /// polygons overlapping a box by scanning every one of them, which is a
    /// smaller tile and a slower query.
    skip_bv_tree: bool = false,

    fn toC(self: TileAuthoring) err.Error!c.TileAuthoring {
        const connection_count = std.math.cast(i32, self.connections.len) orelse
            return err.Error.InvalidArgument;
        return .{
            .connections = if (self.connections.len == 0) null else self.connections.ptr,
            .connection_count = connection_count,
            .user_id = self.user_id,
            .skip_bv_tree = if (self.skip_bv_tree) c.c_true else c.c_false,
        };
    }
};

/// A navigation mesh ready to be queried, holding one tile or a grid of them.
pub const NavMesh = struct {
    handle: *c.NavMesh,

    /// Converts a baked polygon mesh into a queryable navmesh. The navmesh
    /// takes its own copy, so `mesh` may be destroyed straight afterwards.
    ///
    /// `authoring` is optional and supplies the off-mesh connections and the
    /// opaque user id the single tile this creates is built with.
    pub fn initFromPolyMesh(mesh: PolyMesh, authoring: ?TileAuthoring) err.Error!NavMesh {
        var authoring_storage: c.TileAuthoring = undefined;
        if (authoring) |a| authoring_storage = try a.toC();
        const c_authoring: ?*const c.TileAuthoring =
            if (authoring != null) &authoring_storage else null;
        var handle: *c.NavMesh = undefined;
        try err.check(c.zrcNavMeshCreate(mesh.handle, c_authoring, &handle));
        return .{ .handle = handle };
    }

    /// Creates an empty navmesh sized for a tile grid.
    ///
    /// `max_tiles` bounds how many tiles may be resident at once and
    /// `max_polys_per_tile` how many polygons any one may hold. Both feed
    /// Detour's 32-bit polygon reference, which needs 10 bits of salt left
    /// over: rounded up to powers of two, the two together may claim at most
    /// 22 bits, and `InvalidArgument` says they did not.
    pub fn initTiled(
        grid: TileGrid,
        max_tiles: u32,
        max_polys_per_tile: u32,
    ) err.Error!NavMesh {
        const tiles = std.math.cast(i32, max_tiles) orelse
            return err.Error.InvalidArgument;
        const polys = std.math.cast(i32, max_polys_per_tile) orelse
            return err.Error.InvalidArgument;
        var handle: *c.NavMesh = undefined;
        try err.check(c.zrcNavMeshCreateTiled(&grid, tiles, polys, &handle));
        return .{ .handle = handle };
    }

    /// Adds a tile, checking the bytes in full first and then copying them, so
    /// `bytes` stays the caller's and may be freed immediately.
    pub fn addTile(self: NavMesh, bytes: []const u8) err.Error!TileRef {
        var ref: TileRef = 0;
        try err.check(c.zrcNavMeshAddTile(self.handle, bytes.ptr, bytes.len, &ref));
        return ref;
    }

    /// Removes a tile and releases the copy the navmesh made of it.
    pub fn removeTile(self: NavMesh, ref: TileRef) err.Error!void {
        try err.check(c.zrcNavMeshRemoveTile(self.handle, ref));
    }

    /// The tile at a grid position, or 0 when the slot is empty — a fact about
    /// the world rather than an error.
    pub fn tileRefAt(
        self: NavMesh,
        tile_x: i32,
        tile_z: i32,
        tile_layer: i32,
    ) err.Error!TileRef {
        var ref: TileRef = 0;
        try err.check(c.zrcNavMeshTileRefAt(
            self.handle,
            tile_x,
            tile_z,
            tile_layer,
            &ref,
        ));
        return ref;
    }

    /// Tiles currently resident.
    pub fn tileCount(self: NavMesh) u32 {
        return @intCast(c.zrcNavMeshTileCount(self.handle));
    }

    /// The tile in slot `index`, or 0 when the slot is empty.
    ///
    /// Slots are the navmesh's own storage, `maxTiles` of them; walking them is
    /// how a host enumerates what is resident without knowing where to look.
    /// Every tile stacked at a grid position, across all layers, lowest
    /// first, and how many are resident there in total.
    ///
    /// `out` receives however many fit; the total is always the return value,
    /// so a caller compares it against `out.len` to see whether a second,
    /// larger call is needed rather than treating a short buffer as failure.
    /// An empty `out` asks only for the count. A tile cache is what stacks
    /// layers at one position.
    pub fn tileRefsAt(
        self: NavMesh,
        tile_x: i32,
        tile_z: i32,
        out: []TileRef,
    ) err.Error!usize {
        const max_tiles = std.math.cast(i32, out.len) orelse
            return err.Error.InvalidArgument;
        const ptr: ?[*]TileRef = if (out.len == 0) null else out.ptr;
        var count: i32 = 0;
        const result = c.zrcNavMeshTileRefsAt(self.handle, tile_x, tile_z, ptr, max_tiles, &count);
        if (result == .buffer_too_small) return @intCast(count);
        try err.check(result);
        return @intCast(count);
    }

    pub fn tileRefAtIndex(self: NavMesh, index: u32) err.Error!TileRef {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var ref: TileRef = 0;
        try err.check(c.zrcNavMeshTileRefAtIndex(self.handle, i, &ref));
        return ref;
    }

    /// Tiles this navmesh was sized to hold.
    pub fn maxTiles(self: NavMesh) u32 {
        return @intCast(c.zrcNavMeshMaxTiles(self.handle));
    }

    /// World-space bounds of one resident tile, `.{ min, max }`.
    pub fn tileBounds(self: NavMesh, ref: TileRef) err.Error![2][3]f32 {
        var result: [2][3]f32 = undefined;
        try err.check(c.zrcNavMeshTileBounds(
            self.handle,
            ref,
            &result[0],
            &result[1],
        ));
        return result;
    }

    /// The area id of one polygon, 0 to `max_areas - 1`.
    ///
    /// The area is what a query filter charges to cross the polygon, through
    /// `Filter.area_cost`; the flags are what it admits or refuses outright.
    /// Both are per polygon and both are a write into the tile the reference
    /// names, so a world can change shape without being re-cooked.
    pub fn polyArea(self: NavMesh, ref: c.PolyRef) err.Error!i32 {
        var area: i32 = 0;
        try err.check(c.zrcNavMeshGetPolyArea(self.handle, ref, &area));
        return area;
    }

    /// Rewrites the area id of one polygon. `InvalidArgument` outside
    /// `0..max_areas`.
    pub fn setPolyArea(self: NavMesh, ref: c.PolyRef, area: i32) err.Error!void {
        try err.check(c.zrcNavMeshSetPolyArea(self.handle, ref, area));
    }

    /// The query flags of one polygon.
    pub fn polyFlags(self: NavMesh, ref: c.PolyRef) err.Error!u16 {
        var flags: u16 = 0;
        try err.check(c.zrcNavMeshGetPolyFlags(self.handle, ref, &flags));
        return flags;
    }

    /// Rewrites the query flags of one polygon. Zero makes it invisible to
    /// every query.
    pub fn setPolyFlags(self: NavMesh, ref: c.PolyRef, flags: u16) err.Error!void {
        try err.check(c.zrcNavMeshSetPolyFlags(self.handle, ref, flags));
    }

    /// Which kind of polygon a reference names.
    ///
    /// There is no setter: a polygon's type decides how every query treats it
    /// and which of a tile's arrays it may index, so a host chooses it by
    /// supplying connections at build time instead.
    pub fn polyType(self: NavMesh, ref: c.PolyRef) err.Error!PolyType {
        var out_type: i32 = 0;
        try err.check(c.zrcNavMeshGetPolyType(self.handle, ref, &out_type));
        return @enumFromInt(out_type);
    }

    /// The two endpoints of an off-mesh connection, `.{ start, end }`, in the
    /// direction of travel.
    ///
    /// `prev_ref` is the polygon the path arrives from, and decides which
    /// endpoint is reported first: pass the polygon immediately before the
    /// connection in the corridor. Anything else — including 0 — reports them
    /// reversed, because arriving from anywhere that is not endpoint A's
    /// landing polygon means arriving from endpoint B.
    pub fn offMeshConnectionEndPoints(
        self: NavMesh,
        prev_ref: c.PolyRef,
        poly_ref: c.PolyRef,
    ) err.Error![2][3]f32 {
        var result: [2][3]f32 = undefined;
        try err.check(c.zrcNavMeshOffMeshConnectionEndPoints(
            self.handle,
            prev_ref,
            poly_ref,
            &result[0],
            &result[1],
        ));
        return result;
    }

    /// The connection a polygon reference names, as it is stored.
    ///
    /// `.flags` and `.area` come from the connection's polygon, so a
    /// `setPolyFlags` since the tile was added is reflected here. The
    /// endpoints are the stored ones, which is not always where the
    /// connection attached: `offMeshConnectionEndPoints` reports where it
    /// landed.
    pub fn offMeshConnection(self: NavMesh, ref: c.PolyRef) err.Error!OffMeshConnection {
        var out: OffMeshConnection = undefined;
        try err.check(c.zrcNavMeshOffMeshConnection(self.handle, ref, &out));
        return out;
    }

    /// The tile's opaque user id, as given to `buildTileData` or
    /// `initFromPolyMesh`.
    pub fn tileUserId(self: NavMesh, ref: TileRef) err.Error!u32 {
        var out_user_id: u32 = 0;
        try err.check(c.zrcNavMeshTileUserId(self.handle, ref, &out_user_id));
        return out_user_id;
    }

    /// How this navmesh's tile grid was sized, as `initTiled` set it up.
    pub fn params(self: NavMesh) err.Error!NavMeshParams {
        var out: NavMeshParams = undefined;
        try err.check(c.zrcNavMeshParams(self.handle, &out));
        return out;
    }

    /// Which tile of the grid `pos` falls in.
    ///
    /// Answers for any position, including one outside the grid: the result is
    /// the coordinate the position would have, not a promise that a tile is
    /// there.
    pub fn calcTileLoc(self: NavMesh, pos: [3]f32) err.Error!TileLoc {
        var loc: TileLoc = undefined;
        try err.check(c.zrcNavMeshCalcTileLoc(self.handle, &pos, &loc.x, &loc.z));
        return loc;
    }

    /// Everything a tile's header says about it, copied out.
    pub fn tileInfo(self: NavMesh, ref: TileRef) err.Error!TileInfo {
        var out: TileInfo = undefined;
        try err.check(c.zrcNavMeshTileInfo(self.handle, ref, &out));
        return out;
    }

    /// One polygon, copied out of the tile that holds it.
    pub fn polyInfo(self: NavMesh, ref: c.PolyRef) err.Error!PolyInfo {
        var out: PolyInfo = undefined;
        try err.check(c.zrcNavMeshPolyInfo(self.handle, ref, &out));
        return out;
    }

    /// The reference naming one polygon of a tile, by index.
    ///
    /// Without this a polygon is reachable only through a reference a query
    /// already returned, so a tile's polygons could not be walked at all. The
    /// index runs over the whole tile: the first `TileInfo.ground_poly_count`
    /// are ground, the rest belong to off-mesh connections.
    pub fn tilePolyRef(self: NavMesh, ref: TileRef, index: u32) err.Error!c.PolyRef {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: c.PolyRef = 0;
        try err.check(c.zrcNavMeshTilePolyRef(self.handle, ref, i, &out));
        return out;
    }

    /// One of a tile's links by index.
    ///
    /// Indices come from `PolyInfo.first_link` and from `Link.next`; there is
    /// no reason to walk the array itself, and a link not on any polygon's
    /// chain is free storage rather than an edge.
    pub fn tileLink(self: NavMesh, ref: TileRef, index: u32) err.Error!Link {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: Link = undefined;
        try err.check(c.zrcNavMeshTileLink(self.handle, ref, i, &out));
        return out;
    }

    /// One node of a tile's bounding-volume tree by index.
    pub fn tileBvNode(self: NavMesh, ref: TileRef, index: u32) err.Error!BvNode {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: BvNode = undefined;
        try err.check(c.zrcNavMeshTileBvNode(self.handle, ref, i, &out));
        return out;
    }

    /// Copies vertices into `out`, starting at `first`, one `[3]f32` each.
    ///
    /// `out.len` is the count read; a range partly outside the tile's vertex
    /// array is `InvalidArgument` rather than a short read, since a silently
    /// short read is indistinguishable from an empty tile.
    pub fn tileVerts(self: NavMesh, ref: TileRef, first: u32, out: [][3]f32) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcNavMeshTileVerts(self.handle, ref, first_i, count, ptr));
    }

    /// Copies detail vertices into `out`, starting at `first`, one `[3]f32`
    /// each. Same range rule as `tileVerts`.
    pub fn tileDetailVerts(self: NavMesh, ref: TileRef, first: u32, out: [][3]f32) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcNavMeshTileDetailVerts(self.handle, ref, first_i, count, ptr));
    }

    /// Copies detail triangles into `out`, starting at `first`, one `[4]u8`
    /// each: three corner indices and an edge-flag byte. Same range rule as
    /// `tileVerts`.
    ///
    /// A corner below the owning polygon's `vert_count` names one of its own
    /// corners; at or above it, name `detail_vert_base + corner - vert_count`
    /// in the detail vertex array.
    pub fn tileDetailTris(self: NavMesh, ref: TileRef, first: u32, out: [][4]u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcNavMeshTileDetailTris(self.handle, ref, first_i, count, ptr));
    }

    /// Bytes `storeTileState` needs for this tile.
    pub fn tileStateSize(self: NavMesh, ref: TileRef) err.Error!usize {
        var out_size: usize = 0;
        try err.check(c.zrcNavMeshTileStateSize(self.handle, ref, &out_size));
        return out_size;
    }

    /// Writes the tile's polygon areas and flags into `data`.
    ///
    /// `data.len` must equal `tileStateSize`'s answer exactly, not merely be
    /// at least it: too small is `BufferTooSmall`, too large is
    /// `InvalidArgument`. See `ZrcTileAuthoring`'s tile-state section in
    /// zrecast.h for why the length has to be exact.
    pub fn storeTileState(self: NavMesh, ref: TileRef, data: []u8) err.Error!void {
        try err.check(c.zrcNavMeshStoreTileState(self.handle, ref, data.ptr, data.len));
    }

    /// Puts stored areas and flags back onto the tile they came from.
    ///
    /// `data.len` must equal `tileStateSize`'s answer exactly, the same rule
    /// `storeTileState` enforces. A tile removed and re-added gets a new
    /// salt, so its old state no longer applies to it.
    pub fn restoreTileState(self: NavMesh, ref: TileRef, data: []const u8) err.Error!void {
        try err.check(c.zrcNavMeshRestoreTileState(self.handle, ref, data.ptr, data.len));
    }

    /// Rebuilds a navmesh from bytes produced by `serialize`.
    ///
    /// The bytes are checked before Detour sees any of them, then copied, so
    /// the slice need not outlive the call. See `validate` for exactly what
    /// that check does and does not cover.
    pub fn initFromBytes(bytes: []const u8) err.Error!NavMesh {
        var handle: *c.NavMesh = undefined;
        try err.check(c.zrcNavMeshDeserialize(bytes.ptr, bytes.len, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: NavMesh) void {
        c.zrcNavMeshDestroy(self.handle);
    }

    /// Serialises a single-tile navmesh into a buffer owned by the caller.
    ///
    /// A tiled navmesh has no single image and this is `InvalidArgument` for
    /// one; cook each tile with `buildTileData` instead.
    pub fn serialize(self: NavMesh) err.Error!Serialized {
        var data: ?*anyopaque = null;
        var size: usize = 0;
        try err.check(c.zrcNavMeshSerialize(self.handle, &data, &size));
        const ptr: [*]u8 = @ptrCast(data orelse return err.Error.EmptyResult);
        return .{ .bytes = ptr[0..size] };
    }

    pub fn polyCount(self: NavMesh) u32 {
        return @intCast(c.zrcNavMeshPolyCount(self.handle));
    }

    /// World-space bounds, `.{ min, max }`.
    pub fn bounds(self: NavMesh) err.Error![2][3]f32 {
        var result: [2][3]f32 = undefined;
        try err.check(c.zrcNavMeshBounds(self.handle, &result[0], &result[1]));
        return result;
    }
};

/// A buffer the library allocated, and the right to free it.
///
/// A whole single-tile navmesh from `NavMesh.serialize`, or one tile from
/// `buildTileData`. Both are bytes from the installed allocator and both are
/// freed the same way.
///
/// This is a type rather than a bare `[]u8` plus a `freeSerialized(bytes)`
/// function for one reason: the buffer comes from the installed allocator
/// behind a private header, so only its original base pointer can be freed.
/// With a free function taking a slice, `freeSerialized(bytes[8..])` compiles,
/// reads a header that is not there, and corrupts the heap. Owning the slice
/// makes that spelling impossible instead of merely discouraged.
pub const Serialized = struct {
    /// The image. Borrow it freely; it lives until `deinit`.
    bytes: []u8,

    pub fn deinit(self: Serialized) void {
        c.zrcFree(self.bytes.ptr);
    }
};

/// Cooks a baked tile into the bytes a navmesh accepts.
///
/// This is the cook step of a tiled build: write the bytes to disk, and hand
/// them to `NavMesh.addTile` at runtime. The grid position is stored in the
/// image, so a tile knows where it belongs without being told again.
///
/// `authoring` is optional and supplies the tile's off-mesh connections and
/// opaque user id, same contract as `NavMesh.initFromPolyMesh`.
pub fn buildTileData(
    mesh: PolyMesh,
    tile_x: i32,
    tile_z: i32,
    tile_layer: i32,
    authoring: ?TileAuthoring,
) err.Error!Serialized {
    var authoring_storage: c.TileAuthoring = undefined;
    if (authoring) |a| authoring_storage = try a.toC();
    const c_authoring: ?*const c.TileAuthoring =
        if (authoring != null) &authoring_storage else null;
    var data: ?*anyopaque = null;
    var size: usize = 0;
    try err.check(c.zrcTileDataBuild(
        mesh.handle,
        tile_x,
        tile_z,
        tile_layer,
        c_authoring,
        &data,
        &size,
    ));
    const ptr: [*]u8 = @ptrCast(data orelse return err.Error.EmptyResult);
    return .{ .bytes = ptr[0..size] };
}

/// Checks that `bytes` is a tile image this build can load, without building
/// anything.
///
/// One rule, and it is about memory safety rather than about which navmesh the
/// tile belongs to: a tile of a grid passes here just as a lone tile does. The
/// extra conditions live where they matter — `NavMesh.initFromBytes` wants a
/// tile with no neighbours, `NavMesh.addTile` one that fits its grid.
///
/// Worth calling on its own before, say, admitting a file into a content
/// pipeline. Detour's own loader checks two header fields by dereferencing the
/// buffer before it looks at the length, then trusts every count in that header
/// to address the rest; this closes both gaps.
///
/// It then bounds every index inside the image that Detour dereferences —
/// polygon corner counts and indices, detail sub-mesh bases and extents, detail
/// triangle corners, BV-tree nodes. That pass is what makes a doctored image
/// safe: a polygon claiming 200 corners passes every header check and then
/// overruns a fixed stack array inside Detour.
///
/// It does not decide whether the mesh is *sensible*. An in-bounds image can
/// still describe a degenerate or useless navmesh. This is a guarantee about
/// memory safety, not about trustworthiness.
pub fn validate(bytes: []const u8) err.Error!void {
    try err.check(c.zrcNavMeshValidate(bytes.ptr, bytes.len));
}

/// A byte range inside a tile image.
pub const Range = struct {
    offset: usize,
    len: usize,

    /// The bytes this range names, inside the image it was computed from.
    pub fn of(self: Range, image: []const u8) []const u8 {
        return image[self.offset..][0..self.len];
    }
};

/// Where each of a tile image's eight arrays lives, and how many bytes it
/// occupies, both relative to the start of the image.
///
/// A host parsing an image itself — a tool that patches polygon flags in a
/// cooked asset, say — wants these numbers directly rather than through
/// `.of`, which borrows from the image; `.offset` and `.len` write into it
/// instead.
pub const TileLayout = struct {
    verts: Range,
    polys: Range,
    links: Range,
    detail_meshes: Range,
    detail_verts: Range,
    detail_tris: Range,
    bv_tree: Range,
    off_mesh_cons: Range,
    /// Total bytes the header's counts imply, which a well-formed image
    /// matches exactly.
    total_size: usize,
};

/// Reports the layout of a serialised tile image.
///
/// The image is validated first, so every range this returns is inside
/// `image` and every array it names is fully contained.
pub fn tileLayout(image: []const u8) err.Error!TileLayout {
    var out: c.TileLayout = undefined;
    try err.check(c.zrcTileLayout(image.ptr, image.len, &out));

    const cast = struct {
        fn toUsize(x: i64) err.Error!usize {
            return std.math.cast(usize, x) orelse return err.Error.BadFormat;
        }
        fn range(offset: i64, len: i64) err.Error!Range {
            return .{ .offset = try toUsize(offset), .len = try toUsize(len) };
        }
    };
    return .{
        .verts = try cast.range(out.verts_offset, out.verts_size),
        .polys = try cast.range(out.polys_offset, out.polys_size),
        .links = try cast.range(out.links_offset, out.links_size),
        .detail_meshes = try cast.range(out.detail_meshes_offset, out.detail_meshes_size),
        .detail_verts = try cast.range(out.detail_verts_offset, out.detail_verts_size),
        .detail_tris = try cast.range(out.detail_tris_offset, out.detail_tris_size),
        .bv_tree = try cast.range(out.bv_tree_offset, out.bv_tree_size),
        .off_mesh_cons = try cast.range(out.off_mesh_cons_offset, out.off_mesh_cons_size),
        .total_size = try cast.toUsize(out.total_size),
    };
}

/// Rewrites a cooked tile image into the opposite byte order, as a new buffer.
///
/// A host that cooks on one architecture and loads on another of the opposite
/// byte order converts on one side or the other; this is that conversion. The
/// input is never modified, and the caller owns the result.
///
/// `from_native` says which way round: true when `bytes` is an image this build
/// can load and the result is for the other machine, false when `bytes` came
/// from that machine and the result is for this one. Either way the result is
/// validated in full before it is handed back.
pub fn swapImageEndian(bytes: []const u8, from_native: bool) err.Error!Serialized {
    var data: ?*anyopaque = null;
    var size: usize = 0;
    try err.check(c.zrcNavMeshImageSwapEndian(
        bytes.ptr,
        bytes.len,
        if (from_native) c.c_true else c.c_false,
        &data,
        &size,
    ));
    const raw: [*]u8 = @ptrCast(data.?);
    return .{ .bytes = raw[0..size] };
}

/// The image format version this build writes and accepts.
pub fn dataVersion() i32 {
    return c.zrcNavMeshDataVersion();
}

/// The side code facing a given one, for the four values a portal edge
/// carries: a portal on side 0 of one tile meets side 4 of its neighbour.
/// `InvalidArgument` outside `0..8`.
///
/// A free function rather than a `NavMesh` method: it takes no navmesh.
pub fn oppositeTileSide(side: i32) err.Error!i32 {
    var out: i32 = 0;
    try err.check(c.zrcOppositeTileSide(side, &out));
    return out;
}

/// The detail-triangle edge-flag bit that marks an edge as lying on the
/// polygon's own boundary, mirroring `ZRC_DETAIL_EDGE_BOUNDARY`. Test
/// `detailTriEdgeFlags`'s result against it.
pub const detail_edge_boundary: u8 = 0x01;

/// Which edges of a detail triangle lie on the polygon's own boundary.
///
/// `tri_flags` is every fourth byte of what `tileDetailTris` reports;
/// `edge_index` is 0, 1 or 2. Test the result against `detail_edge_boundary`.
pub fn detailTriEdgeFlags(tri_flags: u8, edge_index: u2) u8 {
    const shift: u3 = @as(u3, edge_index) * 2;
    return (tri_flags >> shift) & 0x3;
}

test "obvious non-navmesh bytes are rejected" {
    const garbage = "this is not a navigation mesh, not even a little bit" ** 4;
    try std.testing.expectError(err.Error.BadFormat, validate(garbage));
    try std.testing.expectError(err.Error.BadFormat, validate(""));
    try std.testing.expectError(err.Error.BadFormat, NavMesh.initFromBytes(garbage));
}

test "the data version is a real number the C side agrees on" {
    try std.testing.expect(dataVersion() > 0);
}
