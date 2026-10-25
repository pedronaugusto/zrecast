//! The seam between the two lifecycles: a baked polygon mesh becomes bytes on
//! one side of a build, and a queryable navmesh on the other.

const std = @import("std");
const aegis = @import("aegis");
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

    fn toC(self: TileAuthoring, failure: *err.Failure, call: err.Call) err.Error!c.TileAuthoring {
        const connection_count = aegis.int.cast(i32, self.connections.len) catch
            return err.refuse(failure, call, error.InvalidArgument);
        return .{
            .connections = if (self.connections.len == 0) null else self.connections.ptr,
            .connection_count = connection_count,
            .user_id = self.user_id,
            .skip_bv_tree = if (self.skip_bv_tree) c.c_true else c.c_false,
        };
    }
};

/// A navigation mesh ready to be queried, holding one tile or a grid of them.
/// Bytes read as a serialised navmesh or tile image. They can come from a
/// file, the network or another build, so every function that takes them
/// checks them before upstream reads any of them.
pub const Image = aegis.input.Untrusted([]const u8);

pub const NavMesh = NavMeshOf(.owned);

/// A navmesh another object lent, read-only: what `NavMeshQuery.attachedNavMesh`
/// returns. It cannot be written or freed, and it lives as long as its owner.
pub const BorrowedNavMesh = NavMeshOf(.borrowed);

fn NavMeshOf(comptime access: c.Access) type {
    return struct {
        const Self = @This();

        handle: c.Handle(c.NavMesh, access),

        /// Converts a baked polygon mesh into a queryable navmesh. The navmesh
        /// takes its own copy, so `mesh` may be destroyed straight afterwards.
        ///
        /// `authoring` is optional and supplies the off-mesh connections and the
        /// opaque user id the single tile this creates is built with.
        pub fn initFromPolyMesh(mesh: PolyMesh, authoring: ?TileAuthoring, failure: *err.Failure) err.Error!NavMesh {
            comptime c.ownedOnly(access);
            var authoring_storage: c.TileAuthoring = undefined;
            if (authoring) |a| authoring_storage = try a.toC(failure, .nav_mesh_create);
            const c_authoring: ?*const c.TileAuthoring =
                if (authoring != null) &authoring_storage else null;
            var handle: *c.NavMesh = undefined;
            try err.check(failure, .nav_mesh_create, c.zrcNavMeshCreate(mesh.handle, c_authoring, &handle));
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
            failure: *err.Failure,
        ) err.Error!NavMesh {
            comptime c.ownedOnly(access);
            const tiles = aegis.int.cast(i32, max_tiles) catch
                return err.refuse(failure, .nav_mesh_create_tiled, error.InvalidArgument);
            const polys = aegis.int.cast(i32, max_polys_per_tile) catch
                return err.refuse(failure, .nav_mesh_create_tiled, error.InvalidArgument);
            var handle: *c.NavMesh = undefined;
            try err.check(failure, .nav_mesh_create_tiled, c.zrcNavMeshCreateTiled(&grid, tiles, polys, &handle));
            return .{ .handle = handle };
        }

        /// Adds a tile, checking the bytes in full first and then copying them, so
        /// `image` stays the caller's and may be freed immediately.
        pub fn addTile(self: Self, image: Image, failure: *err.Failure) err.Error!TileRef {
            const bytes = image.readForParse();
            var ref: TileRef = 0;
            try err.check(failure, .nav_mesh_add_tile, c.zrcNavMeshAddTile(self.handle, bytes.ptr, bytes.len, &ref));
            return ref;
        }

        /// Removes a tile and releases the copy the navmesh made of it.
        pub fn removeTile(self: Self, ref: TileRef, failure: *err.Failure) err.Error!void {
            try err.check(failure, .nav_mesh_remove_tile, c.zrcNavMeshRemoveTile(self.handle, ref));
        }

        /// The tile at a grid position, or 0 when the slot is empty — a fact about
        /// the world rather than an error.
        pub fn tileRefAt(
            self: Self,
            tile_x: i32,
            tile_z: i32,
            tile_layer: i32,
            failure: *err.Failure,
        ) err.Error!TileRef {
            var ref: TileRef = 0;
            try err.check(failure, .nav_mesh_tile_ref_at, c.zrcNavMeshTileRefAt(
                self.handle,
                tile_x,
                tile_z,
                tile_layer,
                &ref,
            ));
            return ref;
        }

        /// Tiles currently resident.
        pub fn tileCount(self: Self) u32 {
            return c.reported(u32, c.zrcNavMeshTileCount(self.handle));
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
            self: Self,
            tile_x: i32,
            tile_z: i32,
            out: []TileRef,
            failure: *err.Failure,
        ) err.Error!usize {
            const max_tiles = aegis.int.cast(i32, out.len) catch
                return err.refuse(failure, .nav_mesh_tile_refs_at, error.InvalidArgument);
            const ptr: ?[*]TileRef = if (out.len == 0) null else out.ptr;
            var count: i32 = 0;
            const result = c.zrcNavMeshTileRefsAt(self.handle, tile_x, tile_z, ptr, max_tiles, &count);
            if (result == .buffer_too_small) return c.reported(usize, count);
            try err.check(failure, .nav_mesh_tile_refs_at, result);
            return c.reported(usize, count);
        }

        pub fn tileRefAtIndex(self: Self, index: u32, failure: *err.Failure) err.Error!TileRef {
            const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .nav_mesh_tile_ref_at_index, error.InvalidArgument);
            var ref: TileRef = 0;
            try err.check(failure, .nav_mesh_tile_ref_at_index, c.zrcNavMeshTileRefAtIndex(self.handle, i, &ref));
            return ref;
        }

        /// Tiles this navmesh was sized to hold.
        pub fn maxTiles(self: Self) u32 {
            return c.reported(u32, c.zrcNavMeshMaxTiles(self.handle));
        }

        /// World-space bounds of one resident tile, `.{ min, max }`.
        pub fn tileBounds(self: Self, ref: TileRef, failure: *err.Failure) err.Error![2][3]f32 {
            var result: [2][3]f32 = undefined;
            try err.check(failure, .nav_mesh_tile_bounds, c.zrcNavMeshTileBounds(
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
        pub fn polyArea(self: Self, ref: c.PolyRef, failure: *err.Failure) err.Error!i32 {
            var area: i32 = 0;
            try err.check(failure, .nav_mesh_get_poly_area, c.zrcNavMeshGetPolyArea(self.handle, ref, &area));
            return area;
        }

        /// Rewrites the area id of one polygon. `InvalidArgument` outside
        /// `0..max_areas`.
        pub fn setPolyArea(self: Self, ref: c.PolyRef, area: i32, failure: *err.Failure) err.Error!void {
            try err.check(failure, .nav_mesh_set_poly_area, c.zrcNavMeshSetPolyArea(self.handle, ref, area));
        }

        /// The query flags of one polygon.
        pub fn polyFlags(self: Self, ref: c.PolyRef, failure: *err.Failure) err.Error!u16 {
            var flags: u16 = 0;
            try err.check(failure, .nav_mesh_get_poly_flags, c.zrcNavMeshGetPolyFlags(self.handle, ref, &flags));
            return flags;
        }

        /// Rewrites the query flags of one polygon. Zero makes it invisible to
        /// every query.
        pub fn setPolyFlags(self: Self, ref: c.PolyRef, flags: u16, failure: *err.Failure) err.Error!void {
            try err.check(failure, .nav_mesh_set_poly_flags, c.zrcNavMeshSetPolyFlags(self.handle, ref, flags));
        }

        /// Which kind of polygon a reference names.
        ///
        /// There is no setter: a polygon's type decides how every query treats it
        /// and which of a tile's arrays it may index, so a host chooses it by
        /// supplying connections at build time instead.
        pub fn polyType(self: Self, ref: c.PolyRef, failure: *err.Failure) err.Error!PolyType {
            var out_type: i32 = 0;
            try err.check(failure, .nav_mesh_get_poly_type, c.zrcNavMeshGetPolyType(self.handle, ref, &out_type));
            return std.enums.fromInt(PolyType, out_type) orelse
                aegis.assert.never("the C boundary reports only the polygon types Detour defines");
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
            self: Self,
            prev_ref: c.PolyRef,
            poly_ref: c.PolyRef,
            failure: *err.Failure,
        ) err.Error![2][3]f32 {
            var result: [2][3]f32 = undefined;
            try err.check(failure, .nav_mesh_off_mesh_connection_end_points, c.zrcNavMeshOffMeshConnectionEndPoints(
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
        pub fn offMeshConnection(self: Self, ref: c.PolyRef, failure: *err.Failure) err.Error!OffMeshConnection {
            var out: OffMeshConnection = undefined;
            try err.check(failure, .nav_mesh_off_mesh_connection, c.zrcNavMeshOffMeshConnection(self.handle, ref, &out));
            return out;
        }

        /// The tile's opaque user id, as given to `buildTileData` or
        /// `initFromPolyMesh`.
        pub fn tileUserId(self: Self, ref: TileRef, failure: *err.Failure) err.Error!u32 {
            var out_user_id: u32 = 0;
            try err.check(failure, .nav_mesh_tile_user_id, c.zrcNavMeshTileUserId(self.handle, ref, &out_user_id));
            return out_user_id;
        }

        /// How this navmesh's tile grid was sized, as `initTiled` set it up.
        pub fn params(self: Self, failure: *err.Failure) err.Error!NavMeshParams {
            var out: NavMeshParams = undefined;
            try err.check(failure, .nav_mesh_params, c.zrcNavMeshParams(self.handle, &out));
            return out;
        }

        /// Which tile of the grid `pos` falls in.
        ///
        /// Answers for any position, including one outside the grid: the result is
        /// the coordinate the position would have, not a promise that a tile is
        /// there.
        pub fn calcTileLoc(self: Self, pos: [3]f32, failure: *err.Failure) err.Error!TileLoc {
            var loc: TileLoc = undefined;
            try err.check(failure, .nav_mesh_calc_tile_loc, c.zrcNavMeshCalcTileLoc(self.handle, &pos, &loc.x, &loc.z));
            return loc;
        }

        /// Everything a tile's header says about it, copied out.
        pub fn tileInfo(self: Self, ref: TileRef, failure: *err.Failure) err.Error!TileInfo {
            var out: TileInfo = undefined;
            try err.check(failure, .nav_mesh_tile_info, c.zrcNavMeshTileInfo(self.handle, ref, &out));
            return out;
        }

        /// One polygon, copied out of the tile that holds it.
        pub fn polyInfo(self: Self, ref: c.PolyRef, failure: *err.Failure) err.Error!PolyInfo {
            var out: PolyInfo = undefined;
            try err.check(failure, .nav_mesh_poly_info, c.zrcNavMeshPolyInfo(self.handle, ref, &out));
            return out;
        }

        /// The reference naming one polygon of a tile, by index.
        ///
        /// Without this a polygon is reachable only through a reference a query
        /// already returned, so a tile's polygons could not be walked at all. The
        /// index runs over the whole tile: the first `TileInfo.ground_poly_count`
        /// are ground, the rest belong to off-mesh connections.
        pub fn tilePolyRef(self: Self, ref: TileRef, index: u32, failure: *err.Failure) err.Error!c.PolyRef {
            const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .nav_mesh_tile_poly_ref, error.InvalidArgument);
            var out: c.PolyRef = 0;
            try err.check(failure, .nav_mesh_tile_poly_ref, c.zrcNavMeshTilePolyRef(self.handle, ref, i, &out));
            return out;
        }

        /// One of a tile's links by index.
        ///
        /// Indices come from `PolyInfo.first_link` and from `Link.next`; there is
        /// no reason to walk the array itself, and a link not on any polygon's
        /// chain is free storage rather than an edge.
        pub fn tileLink(self: Self, ref: TileRef, index: u32, failure: *err.Failure) err.Error!Link {
            const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .nav_mesh_tile_link, error.InvalidArgument);
            var out: Link = undefined;
            try err.check(failure, .nav_mesh_tile_link, c.zrcNavMeshTileLink(self.handle, ref, i, &out));
            return out;
        }

        /// One node of a tile's bounding-volume tree by index.
        pub fn tileBvNode(self: Self, ref: TileRef, index: u32, failure: *err.Failure) err.Error!BvNode {
            const i = aegis.int.cast(i32, index) catch return err.refuse(failure, .nav_mesh_tile_bv_node, error.InvalidArgument);
            var out: BvNode = undefined;
            try err.check(failure, .nav_mesh_tile_bv_node, c.zrcNavMeshTileBvNode(self.handle, ref, i, &out));
            return out;
        }

        /// Copies vertices into `out`, starting at `first`, one `[3]f32` each.
        ///
        /// `out.len` is the count read; a range partly outside the tile's vertex
        /// array is `InvalidArgument` rather than a short read, since a silently
        /// short read is indistinguishable from an empty tile.
        pub fn tileVerts(self: Self, ref: TileRef, first: u32, out: [][3]f32, failure: *err.Failure) err.Error!void {
            const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .nav_mesh_tile_verts, error.InvalidArgument);
            const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .nav_mesh_tile_verts, error.InvalidArgument);
            const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr); // safe: each element of the slice is an array of the pointee type, so the slice is one contiguous run of them
            try err.check(failure, .nav_mesh_tile_verts, c.zrcNavMeshTileVerts(self.handle, ref, first_i, count, ptr));
        }

        /// Copies detail vertices into `out`, starting at `first`, one `[3]f32`
        /// each. Same range rule as `tileVerts`.
        pub fn tileDetailVerts(self: Self, ref: TileRef, first: u32, out: [][3]f32, failure: *err.Failure) err.Error!void {
            const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .nav_mesh_tile_detail_verts, error.InvalidArgument);
            const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .nav_mesh_tile_detail_verts, error.InvalidArgument);
            const ptr: ?[*]f32 = if (out.len == 0) null else @ptrCast(out.ptr); // safe: each element of the slice is an array of the pointee type, so the slice is one contiguous run of them
            try err.check(failure, .nav_mesh_tile_detail_verts, c.zrcNavMeshTileDetailVerts(self.handle, ref, first_i, count, ptr));
        }

        /// Copies detail triangles into `out`, starting at `first`, one `[4]u8`
        /// each: three corner indices and an edge-flag byte. Same range rule as
        /// `tileVerts`.
        ///
        /// A corner below the owning polygon's `vert_count` names one of its own
        /// corners; at or above it, name `detail_vert_base + corner - vert_count`
        /// in the detail vertex array.
        pub fn tileDetailTris(self: Self, ref: TileRef, first: u32, out: [][4]u8, failure: *err.Failure) err.Error!void {
            const first_i = aegis.int.cast(i32, first) catch return err.refuse(failure, .nav_mesh_tile_detail_tris, error.InvalidArgument);
            const count = aegis.int.cast(i32, out.len) catch return err.refuse(failure, .nav_mesh_tile_detail_tris, error.InvalidArgument);
            const ptr: ?[*]u8 = if (out.len == 0) null else @ptrCast(out.ptr); // safe: each element of the slice is an array of the pointee type, so the slice is one contiguous run of them
            try err.check(failure, .nav_mesh_tile_detail_tris, c.zrcNavMeshTileDetailTris(self.handle, ref, first_i, count, ptr));
        }

        /// Bytes `storeTileState` needs for this tile.
        pub fn tileStateSize(self: Self, ref: TileRef, failure: *err.Failure) err.Error!usize {
            var out_size: usize = 0;
            try err.check(failure, .nav_mesh_tile_state_size, c.zrcNavMeshTileStateSize(self.handle, ref, &out_size));
            return out_size;
        }

        /// Writes the tile's polygon areas and flags into `data`.
        ///
        /// `data.len` must equal `tileStateSize`'s answer exactly, not merely be
        /// at least it: too small is `BufferTooSmall`, too large is
        /// `InvalidArgument`. See `ZrcTileAuthoring`'s tile-state section in
        /// zrecast.h for why the length has to be exact.
        pub fn storeTileState(self: Self, ref: TileRef, data: []u8, failure: *err.Failure) err.Error!void {
            try err.check(failure, .nav_mesh_store_tile_state, c.zrcNavMeshStoreTileState(self.handle, ref, data.ptr, data.len));
        }

        /// Puts stored areas and flags back onto the tile they came from.
        ///
        /// `state`'s length must equal `tileStateSize`'s answer exactly, the same rule
        /// `storeTileState` enforces. A tile removed and re-added gets a new
        /// salt, so its old state no longer applies to it.
        pub fn restoreTileState(self: Self, ref: TileRef, state: Image, failure: *err.Failure) err.Error!void {
            const data = state.readForParse();
            try err.check(failure, .nav_mesh_restore_tile_state, c.zrcNavMeshRestoreTileState(self.handle, ref, data.ptr, data.len));
        }

        /// Rebuilds a navmesh from bytes produced by `serialize`.
        ///
        /// The bytes are checked before Detour sees any of them, then copied, so
        /// the slice need not outlive the call. See `validate` for exactly what
        /// that check does and does not cover.
        pub fn initFromBytes(image: Image, failure: *err.Failure) err.Error!NavMesh {
            comptime c.ownedOnly(access);
            const bytes = image.readForParse();
            var handle: *c.NavMesh = undefined;
            try err.check(failure, .nav_mesh_deserialize, c.zrcNavMeshDeserialize(bytes.ptr, bytes.len, &handle));
            return .{ .handle = handle };
        }

        pub fn deinit(self: Self) void {
            comptime c.ownedOnly(access);
            c.zrcNavMeshDestroy(self.handle);
        }

        /// Serialises a single-tile navmesh into a buffer owned by the caller.
        ///
        /// A tiled navmesh has no single image and this is `InvalidArgument` for
        /// one; cook each tile with `buildTileData` instead.
        pub fn serialize(self: Self, failure: *err.Failure) err.Error!Serialized {
            var data: ?*anyopaque = null;
            var size: usize = 0;
            try err.check(failure, .nav_mesh_serialize, c.zrcNavMeshSerialize(self.handle, &data, &size));
            const ptr: [*]u8 = @ptrCast(data orelse return err.refuse(failure, .nav_mesh_serialize, error.EmptyResult)); // safe: the C boundary allocates the image as bytes, through the allocator seam
            return .{ .bytes = ptr[0..size] };
        }

        pub fn polyCount(self: Self) u32 {
            return c.reported(u32, c.zrcNavMeshPolyCount(self.handle));
        }

        /// World-space bounds, `.{ min, max }`.
        pub fn bounds(self: Self, failure: *err.Failure) err.Error![2][3]f32 {
            var result: [2][3]f32 = undefined;
            try err.check(failure, .nav_mesh_bounds, c.zrcNavMeshBounds(self.handle, &result[0], &result[1]));
            return result;
        }
    };
}

/// A buffer the library allocated, and the right to free it.
///
/// A whole single-tile navmesh from `NavMesh.serialize`, or one tile from
/// `buildTileData`. Both are bytes from the installed allocator and both are
/// freed the same way.
///
/// This is a type rather than a bare `[]u8` plus a `freeSerialized(bytes)`
/// function for one reason: the buffer comes from the installed allocator
/// behind a private header, so only its original base pointer can be freed.
/// With a free function taking a slice, `freeSerialized(bytes[8..])` compiles
/// and hands the allocator a pointer it never issued. The Zig allocator
/// bridge refuses such a pointer where it can recognise one, and upstream's
/// malloc cannot; an image is also a thing callers index into, so the
/// spelling should not exist at this seam at all. Owning the slice makes it
/// impossible instead of merely discouraged.
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
    failure: *err.Failure,
) err.Error!Serialized {
    var authoring_storage: c.TileAuthoring = undefined;
    if (authoring) |a| authoring_storage = try a.toC(failure, .tile_data_build);
    const c_authoring: ?*const c.TileAuthoring =
        if (authoring != null) &authoring_storage else null;
    var data: ?*anyopaque = null;
    var size: usize = 0;
    try err.check(failure, .tile_data_build, c.zrcTileDataBuild(
        mesh.handle,
        tile_x,
        tile_z,
        tile_layer,
        c_authoring,
        &data,
        &size,
    ));
    const ptr: [*]u8 = @ptrCast(data orelse return err.refuse(failure, .tile_data_build, error.EmptyResult)); // safe: the C boundary allocates the image as bytes, through the allocator seam
    return .{ .bytes = ptr[0..size] };
}

/// Checks that `image` is a tile image this build can load, without building
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
pub fn validate(image: Image, failure: *err.Failure) err.Error!void {
    const bytes = image.readForParse();
    try err.check(failure, .nav_mesh_validate, c.zrcNavMeshValidate(bytes.ptr, bytes.len));
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
pub fn tileLayout(image: Image, failure: *err.Failure) err.Error!TileLayout {
    const bytes = image.readForParse();
    var out: c.TileLayout = undefined;
    try err.check(failure, .tile_layout, c.zrcTileLayout(bytes.ptr, bytes.len, &out));

    const cast = struct {
        fn toUsize(x: i64, into: *err.Failure) err.Error!usize {
            return aegis.int.cast(usize, x) catch return err.refuse(into, .tile_layout, error.BadFormat);
        }
        fn range(offset: i64, len: i64, into: *err.Failure) err.Error!Range {
            return .{ .offset = try toUsize(offset, into), .len = try toUsize(len, into) };
        }
    };
    return .{
        .verts = try cast.range(out.verts_offset, out.verts_size, failure),
        .polys = try cast.range(out.polys_offset, out.polys_size, failure),
        .links = try cast.range(out.links_offset, out.links_size, failure),
        .detail_meshes = try cast.range(out.detail_meshes_offset, out.detail_meshes_size, failure),
        .detail_verts = try cast.range(out.detail_verts_offset, out.detail_verts_size, failure),
        .detail_tris = try cast.range(out.detail_tris_offset, out.detail_tris_size, failure),
        .bv_tree = try cast.range(out.bv_tree_offset, out.bv_tree_size, failure),
        .off_mesh_cons = try cast.range(out.off_mesh_cons_offset, out.off_mesh_cons_size, failure),
        .total_size = try cast.toUsize(out.total_size, failure),
    };
}

/// Rewrites a cooked tile image into the opposite byte order, as a new buffer.
///
/// A host that cooks on one architecture and loads on another of the opposite
/// byte order converts on one side or the other; this is that conversion. The
/// input is never modified, and the caller owns the result.
///
/// `from_native` says which way round: true when `image` is one this build
/// can load and the result is for the other machine, false when `image` came
/// from that machine and the result is for this one. Either way the result is
/// validated in full before it is handed back.
pub fn swapImageEndian(image: Image, from_native: bool, failure: *err.Failure) err.Error!Serialized {
    const bytes = image.readForParse();
    var data: ?*anyopaque = null;
    var size: usize = 0;
    try err.check(failure, .nav_mesh_image_swap_endian, c.zrcNavMeshImageSwapEndian(
        bytes.ptr,
        bytes.len,
        if (from_native) c.c_true else c.c_false,
        &data,
        &size,
    ));
    const raw: [*]u8 = @ptrCast(data.?); // safe: the C boundary allocates the image as bytes, through the allocator seam
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
pub fn oppositeTileSide(side: i32, failure: *err.Failure) err.Error!i32 {
    var out: i32 = 0;
    try err.check(failure, .opposite_tile_side, c.zrcOppositeTileSide(side, &out));
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
    var failure: err.Failure = .{};
    const line = "this is not a navigation mesh, not even a little bit";
    const garbage = line ++ line ++ line ++ line;
    try std.testing.expectError(err.Error.BadFormat, validate(.init(garbage), &failure));
    try std.testing.expectError(err.Error.BadFormat, validate(.init(""), &failure));
    try std.testing.expectError(err.Error.BadFormat, NavMesh.initFromBytes(.init(garbage), &failure));
}

test "the data version is a real number the C side agrees on" {
    try std.testing.expect(dataVersion() > 0);
}
