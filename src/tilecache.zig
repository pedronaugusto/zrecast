//! The layered heightfield, the tile cache, and the layer builder that sits
//! between them.
//!
//! A compact heightfield built for a tiled bake can be cut into layers with
//! `HeightfieldLayerSet`, each a walkable sheet cheap enough to compress and
//! carve at runtime. `buildTileCacheLayer` compresses one layer through a
//! host-supplied codec into the bytes a `TileCache` stores; `TileCache` then
//! keeps one compressed layer per tile, rebuilding a tile through Detour
//! whenever an obstacle over it appears or goes away. `TileCacheLayer`,
//! `TileCacheContourSet` and `TileCachePolyMesh` are the same three stages a
//! tile cache drives internally, exposed for a tool that wants to see a
//! rebuild without running one.
//!
//! No compression codec ships with this package and none is assumed: a host
//! supplies one through `Compressor`, and the bytes a cook produces are
//! whatever that codec writes.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const BuildContext = @import("pipeline.zig").BuildContext;
const CompactHeightfield = @import("pipeline.zig").CompactHeightfield;
const NavMesh = @import("navmesh.zig").NavMesh;
const Serialized = @import("navmesh.zig").Serialized;

//===----------------------------------------------------------------------===//
// References
//===----------------------------------------------------------------------===//

/// Names one compressed layer in a tile cache. Salt-protected the same way a
/// `navmesh.TileRef` is: removing a tile invalidates every reference to it.
pub const CompressedTileRef = c.CompressedTileRef;

/// Names one obstacle. Salt-protected, and the salt turns over when the
/// deferred removal completes rather than when it is requested.
pub const ObstacleRef = c.ObstacleRef;

//===----------------------------------------------------------------------===//
// The layered heightfield
//===----------------------------------------------------------------------===//

/// Converts `context` for one C call, using `storage` to hold the widened
/// struct for the duration of that call. `null` in is `null` out.
///
/// A private copy of `pipeline.zig`'s own helper of the same name:
/// `BuildContext.toC` is public and safe to call from here, but the helper
/// wrapping it is private to that file, and this is the only entry point in
/// the tile-cache surface that takes a context.
fn toCContext(context: ?*const BuildContext, storage: *c.BuildContext) ?*const c.BuildContext {
    const ctx = context orelse return null;
    storage.* = ctx.toC();
    return storage;
}

/// One sheet's extent and where its usable data sits inside it. The three
/// per-cell arrays `HeightfieldLayerSet.heights` and its siblings read are
/// `width * height` long, row-major.
pub const HeightfieldLayer = c.HeightfieldLayer;

/// A set of walkable sheets cut out of a compact heightfield, each small
/// enough to compress and carve obstacles into at runtime.
pub const HeightfieldLayerSet = struct {
    handle: *c.HeightfieldLayerSet,

    /// Cuts `field` into layers, each an unobstructed sheet of walkable
    /// surface. `border_size` is the unnavigable ring in cells a tiled
    /// build carries, and `walkable_height` is in cells.
    ///
    /// Upstream stops at 63 overlapping platforms and fails the whole build
    /// when a field has more, naming the reason in the log — pass a build
    /// context and check it when this returns `error.BakeFailed` and the
    /// geometry has many stacked floors.
    /// [Limit: 0 <= border_size <= 255, 0 <= walkable_height <= ZRC_SPAN_MAX_HEIGHT]
    pub fn init(
        context: ?*const BuildContext,
        field: CompactHeightfield,
        border_size: i32,
        walkable_height: i32,
    ) err.Error!HeightfieldLayerSet {
        var context_storage: c.BuildContext = undefined;
        var handle: *c.HeightfieldLayerSet = undefined;
        try err.check(c.zrcHeightfieldLayerSetCreate(
            toCContext(context, &context_storage),
            field.handle,
            border_size,
            walkable_height,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: HeightfieldLayerSet) void {
        c.zrcHeightfieldLayerSetDestroy(self.handle);
    }

    /// How many layers the set holds. Zero is a legitimate answer for a
    /// field with no walkable surface.
    pub fn count(self: HeightfieldLayerSet) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcHeightfieldLayerSetCount(self.handle, &out));
        return @intCast(out);
    }

    pub fn at(self: HeightfieldLayerSet, index: u32) err.Error!HeightfieldLayer {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: HeightfieldLayer = undefined;
        try err.check(c.zrcHeightfieldLayerAt(self.handle, i, &out));
        return out;
    }

    /// Copies `out.len` height samples of layer `index` from `first`, one
    /// byte each.
    pub fn heights(self: HeightfieldLayerSet, index: u32, first: u32, out: []u8) err.Error!void {
        const idx = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count_i = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcHeightfieldLayerHeights(self.handle, idx, first_i, count_i, ptr));
    }

    /// The same for the area ids, one per sample.
    pub fn areas(self: HeightfieldLayerSet, index: u32, first: u32, out: []u8) err.Error!void {
        const idx = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count_i = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcHeightfieldLayerAreas(self.handle, idx, first_i, count_i, ptr));
    }

    /// The same for the packed neighbour connections. Decode with
    /// `layerCon` and `layerPortal` rather than by hand.
    pub fn cons(self: HeightfieldLayerSet, index: u32, first: u32, out: []u8) err.Error!void {
        const idx = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count_i = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcHeightfieldLayerCons(self.handle, idx, first_i, count_i, ptr));
    }
};

//===----------------------------------------------------------------------===//
// The tile cache — obstacles carved into a baked mesh at runtime
//===----------------------------------------------------------------------===//

pub const TileCacheParams = c.TileCacheParams;

/// The compression codec a host supplies for every compressed layer a tile
/// cache stores.
///
/// Upstream calls these while rebuilding a tile, inside `TileCache.update`
/// and `buildTileCacheLayer`; a hook that returns anything but `.ok` aborts
/// the operation that called it. No codec ships with this package: the
/// bytes a cook produces are whatever the installed codec writes.
pub const Compressor = struct {
    user: ?*anyopaque = null,
    /// An upper bound on what `compress` may write for a buffer this size.
    /// Must not be less than `buffer_size`, and must not overflow an i32.
    max_compressed_size: ?*const fn (user: ?*anyopaque, buffer_size: i32) callconv(.c) i32 = null,
    compress: ?*const fn (
        user: ?*anyopaque,
        buffer: [*]const u8,
        buffer_size: i32,
        compressed: [*]u8,
        max_compressed_size: i32,
        out_compressed_size: *i32,
    ) callconv(.c) c.Result = null,
    /// `compressed_size` arrives already checked to be positive: upstream
    /// computes it as a subtraction that goes negative for a short buffer,
    /// which would otherwise reach this hook as a raw int.
    decompress: ?*const fn (
        user: ?*anyopaque,
        compressed: [*]const u8,
        compressed_size: i32,
        buffer: [*]u8,
        max_buffer_size: i32,
        out_size: *i32,
    ) callconv(.c) c.Result = null,

    pub fn toC(self: Compressor) c.TileCacheCompressor {
        return .{
            .user = self.user,
            .max_compressed_size = self.max_compressed_size,
            .compress = self.compress,
            .decompress = self.decompress,
        };
    }
};

fn toCCompressor(compressor: *const Compressor, storage: *c.TileCacheCompressor) *const c.TileCacheCompressor {
    storage.* = compressor.toC();
    return storage;
}

/// Optional scratch allocator for a tile rebuild, reset once per tile.
///
/// `null` selects upstream's own default, which forwards to the allocator
/// `setAllocator` installs and whose reset does nothing. A host supplying
/// one gets a linear arena per tile instead.
pub const Allocator = struct {
    user: ?*anyopaque = null,
    /// Called once at the start of every tile rebuild, before any
    /// allocation.
    reset: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
    allocate: ?*const fn (user: ?*anyopaque, size: usize) callconv(.c) ?*anyopaque = null,
    deallocate: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void = null,

    pub fn toC(self: Allocator) c.TileCacheAllocator {
        return .{
            .user = self.user,
            .reset = self.reset,
            .allocate = self.allocate,
            .deallocate = self.deallocate,
        };
    }
};

fn toCAllocator(allocator: ?*const Allocator, storage: *c.TileCacheAllocator) ?*const c.TileCacheAllocator {
    const a = allocator orelse return null;
    storage.* = a.toC();
    return storage;
}

/// What a mesh-process callback may change about a tile before it is built:
/// the writable half of a tile's build parameters.
pub const TileCacheBuildParams = c.TileCacheBuildParams;

/// Called once per tile rebuild, after the polygons exist and before the
/// tile is built.
///
/// `process` left `null` leaves every polygon's flags at 0, which no
/// nonzero query filter admits — a navmesh that is silently unreachable
/// rather than one that fails outright. The areas a callback sees are not
/// zeroed: they carry the ids a cook wrote into the layer, so a callback
/// that only wants flags can read `params.areas` to decide them.
pub const MeshProcess = struct {
    user: ?*anyopaque = null,
    process: ?*const fn (user: ?*anyopaque, params: *TileCacheBuildParams) callconv(.c) c.Result = null,

    pub fn toC(self: MeshProcess) c.TileCacheMeshProcess {
        return self.process;
    }
};

/// The three obstacle shapes a tile cache carves.
pub const ObstacleShape = c.ObstacleShape;

/// Where an obstacle sits in the add/remove cycle.
pub const ObstacleState = c.ObstacleState;

pub const ObstacleInfo = c.ObstacleInfo;

pub const CompressedTileInfo = c.CompressedTileInfo;

/// A live tile cache: the compressed layers, the obstacles over them, and
/// the queue of tiles waiting to be rebuilt.
pub const TileCache = struct {
    handle: *c.TileCache,

    /// Creates a tile cache. `compressor` is required; `allocator` may be
    /// `null`, and `mesh_process` may be the zero value for no callback.
    ///
    /// A cache cannot be re-initialised: upstream's init has no purge and
    /// leaks every array if called twice, so there is one `init` and one
    /// `deinit`.
    pub fn init(
        create_params: TileCacheParams,
        compressor: *const Compressor,
        allocator: ?*const Allocator,
        mesh_process: MeshProcess,
    ) err.Error!TileCache {
        var compressor_storage: c.TileCacheCompressor = undefined;
        var allocator_storage: c.TileCacheAllocator = undefined;
        var handle: *c.TileCache = undefined;
        try err.check(c.zrcTileCacheCreate(
            &create_params,
            toCCompressor(compressor, &compressor_storage),
            toCAllocator(allocator, &allocator_storage),
            mesh_process.toC(),
            mesh_process.user,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: TileCache) void {
        c.zrcTileCacheDestroy(self.handle);
    }

    pub fn params(self: TileCache) err.Error!TileCacheParams {
        var out: TileCacheParams = undefined;
        try err.check(c.zrcTileCacheParams(self.handle, &out));
        return out;
    }

    /// Adds one compressed layer, produced by `buildTileCacheLayer`. The
    /// bytes are copied, so `data` stays the caller's and may be freed
    /// immediately.
    ///
    /// `error.TileOccupied` when a layer already sits at that position.
    pub fn addTile(self: TileCache, data: []const u8) err.Error!CompressedTileRef {
        var ref: CompressedTileRef = 0;
        try err.check(c.zrcTileCacheAddTile(self.handle, data.ptr, data.len, &ref));
        return ref;
    }

    /// Removes a layer, discarding its bytes.
    pub fn removeTile(self: TileCache, ref: CompressedTileRef) err.Error!void {
        try err.check(c.zrcTileCacheRemoveTile(self.handle, ref, null, null));
    }

    /// Removes a layer, returning a copy of its bytes the caller owns and
    /// frees with `Serialized.deinit`.
    pub fn removeTileTakingData(self: TileCache, ref: CompressedTileRef) err.Error!Serialized {
        var data: ?*anyopaque = null;
        var size: usize = 0;
        try err.check(c.zrcTileCacheRemoveTile(self.handle, ref, &data, &size));
        const ptr: [*]u8 = @ptrCast(data orelse return err.Error.EmptyResult);
        return .{ .bytes = ptr[0..size] };
    }

    pub fn tileInfo(self: TileCache, ref: CompressedTileRef) err.Error!CompressedTileInfo {
        var out: CompressedTileInfo = undefined;
        try err.check(c.zrcTileCacheTileInfo(self.handle, ref, &out));
        return out;
    }

    /// The layer at (`tile_x`, `tile_y`, `tile_layer`), or 0 in the result
    /// when that position is empty — a legitimate answer, not an error.
    pub fn tileAt(self: TileCache, tile_x: i32, tile_y: i32, tile_layer: i32) err.Error!CompressedTileRef {
        var ref: CompressedTileRef = 0;
        try err.check(c.zrcTileCacheTileAt(self.handle, tile_x, tile_y, tile_layer, &ref));
        return ref;
    }

    /// Every layer stacked at (`tile_x`, `tile_y`), lowest first, copied
    /// into `out`. Returns how many exist in total, whether or not every
    /// one fit; compare it against `out.len` to see whether a larger
    /// buffer is needed, the same rule `pipeline.Heightfield.column`
    /// follows — a short `out` is not an error.
    pub fn tilesAt(self: TileCache, tile_x: i32, tile_y: i32, out: []CompressedTileRef) err.Error!usize {
        const max_tiles = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        var found: i32 = 0;
        const result = c.zrcTileCacheTilesAt(self.handle, tile_x, tile_y, out.ptr, max_tiles, &found);
        if (result == .buffer_too_small) return @intCast(found);
        try err.check(result);
        return @intCast(found);
    }

    /// The reference of the slot at `index`, or 0 when it is free. Slots
    /// run from 0 to `params().max_tiles - 1`; upstream's own accessor
    /// bounds nothing, so the index is checked here.
    pub fn tileRefAt(self: TileCache, index: u32) err.Error!CompressedTileRef {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var ref: CompressedTileRef = 0;
        try err.check(c.zrcTileCacheTileRefAt(self.handle, i, &ref));
        return ref;
    }

    pub fn obstacleInfo(self: TileCache, ref: ObstacleRef) err.Error!ObstacleInfo {
        var out: ObstacleInfo = undefined;
        try err.check(c.zrcTileCacheObstacleInfo(self.handle, ref, &out));
        return out;
    }

    /// The reference of the obstacle slot at `index`, or 0 when it is
    /// free. Upstream's own accessor bounds nothing; the index is checked
    /// here.
    pub fn obstacleRefAt(self: TileCache, index: u32) err.Error!ObstacleRef {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var ref: ObstacleRef = 0;
        try err.check(c.zrcTileCacheObstacleRefAt(self.handle, i, &ref));
        return ref;
    }

    /// A vertical cylinder standing on its base. Queued, not applied: it
    /// appears in the navmesh once `update` has rebuilt every tile it
    /// touches.
    ///
    /// `error.BufferTooSmall` when the cylinder overlaps more than
    /// `c.max_touched_tiles` tiles — upstream carves such an obstacle into
    /// the first eight and leaves the rest untouched without saying so,
    /// a hole in the navmesh nothing else reports; splitting it into
    /// several obstacles is the fix. `error.NavMeshFull` when every
    /// obstacle slot is taken.
    pub fn addCylinderObstacle(
        self: TileCache,
        position: [3]f32,
        radius: f32,
        height: f32,
    ) err.Error!ObstacleRef {
        var ref: ObstacleRef = 0;
        try err.check(c.zrcTileCacheAddCylinderObstacle(self.handle, &position, radius, height, &ref));
        return ref;
    }

    /// An axis-aligned box. Same queueing and same limits as
    /// `addCylinderObstacle`.
    pub fn addBoxObstacle(self: TileCache, bmin: [3]f32, bmax: [3]f32) err.Error!ObstacleRef {
        var ref: ObstacleRef = 0;
        try err.check(c.zrcTileCacheAddBoxObstacle(self.handle, &bmin, &bmax, &ref));
        return ref;
    }

    /// A box rotated about the y axis. Same queueing and same limits.
    pub fn addOrientedBoxObstacle(
        self: TileCache,
        center: [3]f32,
        half_extents: [3]f32,
        y_radians: f32,
    ) err.Error!ObstacleRef {
        var ref: ObstacleRef = 0;
        try err.check(c.zrcTileCacheAddOrientedBoxObstacle(
            self.handle,
            &center,
            &half_extents,
            y_radians,
            &ref,
        ));
        return ref;
    }

    /// Queues an obstacle's removal. The reference stays resolvable until
    /// `update` has rebuilt every tile it touched, and only then does its
    /// salt turn over.
    pub fn removeObstacle(self: TileCache, ref: ObstacleRef) err.Error!void {
        try err.check(c.zrcTileCacheRemoveObstacle(self.handle, ref));
    }

    /// Every layer overlapping the box, copied into `out` — how a host
    /// sizes an obstacle against `c.max_touched_tiles` before adding one.
    /// Same short-buffer rule as `tilesAt`.
    pub fn queryTiles(self: TileCache, bmin: [3]f32, bmax: [3]f32, out: []CompressedTileRef) err.Error!usize {
        const max_tiles = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        var found: i32 = 0;
        const result = c.zrcTileCacheQueryTiles(self.handle, &bmin, &bmax, out.ptr, max_tiles, &found);
        if (result == .buffer_too_small) return @intCast(found);
        try err.check(result);
        return @intCast(found);
    }

    /// Processes queued obstacle requests and rebuilds one tile, returning
    /// whether the whole cache is now up to date.
    ///
    /// This is the throttle rather than a batch API: call it in a loop
    /// until it answers `true`, spending as much of a frame on it as
    /// affordable.
    pub fn update(self: TileCache, navmesh: NavMesh) err.Error!bool {
        var up_to_date: c.Bool = c.c_false;
        try err.check(c.zrcTileCacheUpdate(self.handle, navmesh.handle, &up_to_date));
        return up_to_date != c.c_false;
    }

    /// Rebuilds one named tile immediately, outside the update loop — how
    /// a host commits a layer it has just added without waiting for an
    /// obstacle.
    pub fn buildNavMeshTile(self: TileCache, ref: CompressedTileRef, navmesh: NavMesh) err.Error!void {
        try err.check(c.zrcTileCacheBuildNavMeshTile(self.handle, ref, navmesh.handle));
    }

    /// The same for every layer stacked at one grid position.
    pub fn buildNavMeshTilesAt(self: TileCache, tile_x: i32, tile_y: i32, navmesh: NavMesh) err.Error!void {
        try err.check(c.zrcTileCacheBuildNavMeshTilesAt(self.handle, tile_x, tile_y, navmesh.handle));
    }
};

//===----------------------------------------------------------------------===//
// Building a compressed layer, and taking one apart
//===----------------------------------------------------------------------===//

/// A layer's identity and extent, as it crosses into and out of
/// compression.
pub const TileCacheLayerHeader = c.TileCacheLayerHeader;

/// Compresses one layer into the bytes `TileCache.addTile` takes.
///
/// `heights`, `areas` and `cons` must each be exactly
/// `header.width * header.height` bytes — the three arrays
/// `HeightfieldLayerSet.heights` and its siblings report for one layer. The
/// C side is handed three bare pointers it cannot relate to the header's
/// dimensions, so the match is checked here; nothing else in this package
/// checks it.
pub fn buildTileCacheLayer(
    compressor: *const Compressor,
    header: TileCacheLayerHeader,
    heights: []const u8,
    areas: []const u8,
    cons: []const u8,
) err.Error!Serialized {
    const width = std.math.cast(usize, header.width) orelse return err.Error.InvalidArgument;
    const height = std.math.cast(usize, header.height) orelse return err.Error.InvalidArgument;
    const expected = std.math.mul(usize, width, height) catch return err.Error.InvalidArgument;
    if (heights.len != expected or areas.len != expected or cons.len != expected) {
        return err.Error.InvalidArgument;
    }
    var compressor_storage: c.TileCacheCompressor = undefined;
    var data: ?*anyopaque = null;
    var size: usize = 0;
    try err.check(c.zrcTileCacheLayerBuild(
        toCCompressor(compressor, &compressor_storage),
        &header,
        heights.ptr,
        areas.ptr,
        cons.ptr,
        &data,
        &size,
    ));
    const ptr: [*]u8 = @ptrCast(data orelse return err.Error.EmptyResult);
    return .{ .bytes = ptr[0..size] };
}

/// Swaps a compressed layer header between endiannesses, in place, for a
/// host reading an asset cooked on a machine of the other byte order.
/// `error.BadFormat` for bytes that are not a layer header in either order.
pub fn swapTileCacheHeaderEndian(data: []u8) err.Error!void {
    try err.check(c.zrcTileCacheHeaderSwapEndian(data.ptr, data.len));
}

/// One decompressed layer: the three grids, plus the regions once they are
/// built.
pub const TileCacheLayer = struct {
    handle: *c.TileCacheLayer,

    /// Decompresses a layer, through the same codec that produced it.
    pub fn initFromBytes(
        compressor: *const Compressor,
        allocator: ?*const Allocator,
        data: []const u8,
    ) err.Error!TileCacheLayer {
        var compressor_storage: c.TileCacheCompressor = undefined;
        var allocator_storage: c.TileCacheAllocator = undefined;
        var handle: *c.TileCacheLayer = undefined;
        try err.check(c.zrcTileCacheLayerDecompress(
            toCCompressor(compressor, &compressor_storage),
            toCAllocator(allocator, &allocator_storage),
            data.ptr,
            data.len,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: TileCacheLayer) void {
        c.zrcTileCacheLayerDestroy(self.handle);
    }

    pub fn header(self: TileCacheLayer) err.Error!TileCacheLayerHeader {
        var out: TileCacheLayerHeader = undefined;
        try err.check(c.zrcTileCacheLayerHeaderOf(self.handle, &out));
        return out;
    }

    /// How many regions `buildRegions` found, or 0 before it runs.
    pub fn regionCount(self: TileCacheLayer) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcTileCacheLayerRegionCount(self.handle, &out));
        return @intCast(out);
    }

    /// Copies `out.len` height samples from `first`. The array is
    /// `width * height` long, row-major.
    pub fn heights(self: TileCacheLayer, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCacheLayerHeights(self.handle, first_i, count, ptr));
    }

    /// The same for the area ids, one per sample.
    pub fn areas(self: TileCacheLayer, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCacheLayerAreas(self.handle, first_i, count, ptr));
    }

    /// Writes area ids back from `first`. Each must be below
    /// `c.max_areas`.
    pub fn setAreas(self: TileCacheLayer, first: u32, areas_in: []const u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, areas_in.len) orelse return err.Error.InvalidArgument;
        const ptr: [*]const u8 = if (areas_in.len == 0) undefined else areas_in.ptr;
        try err.check(c.zrcTileCacheLayerSetAreas(self.handle, first_i, count, ptr));
    }

    /// The same packing as `HeightfieldLayerSet.cons`: the low nibble is
    /// one bit per direction for a neighbour inside this layer, the high
    /// nibble one bit per direction for a portal into another. Decode with
    /// `layerCon` and `layerPortal`.
    pub fn cons(self: TileCacheLayer, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCacheLayerCons(self.handle, first_i, count, ptr));
    }

    /// The region id of each sample, once regions have been built.
    /// `error.NotFound` before `buildRegions` has run.
    pub fn regions(self: TileCacheLayer, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCacheLayerRegions(self.handle, first_i, count, ptr));
    }

    /// Carves a cylinder into the layer's area ids, the same way a tile
    /// cache obstacle does. `origin`, `cell_size` and `cell_height` are
    /// the layer's own, from the tile cache's parameters.
    pub fn markCylinder(
        self: TileCacheLayer,
        origin: [3]f32,
        cell_size: f32,
        cell_height: f32,
        position: [3]f32,
        radius: f32,
        height: f32,
        area: u8,
    ) err.Error!void {
        try err.check(c.zrcTileCacheLayerMarkCylinder(
            self.handle,
            &origin,
            cell_size,
            cell_height,
            &position,
            radius,
            height,
            area,
        ));
    }

    /// The same for an axis-aligned box.
    pub fn markBox(
        self: TileCacheLayer,
        origin: [3]f32,
        cell_size: f32,
        cell_height: f32,
        bmin: [3]f32,
        bmax: [3]f32,
        area: u8,
    ) err.Error!void {
        try err.check(c.zrcTileCacheLayerMarkBox(
            self.handle,
            &origin,
            cell_size,
            cell_height,
            &bmin,
            &bmax,
            area,
        ));
    }

    /// The same for a box rotated about the y axis.
    pub fn markOrientedBox(
        self: TileCacheLayer,
        origin: [3]f32,
        cell_size: f32,
        cell_height: f32,
        center: [3]f32,
        half_extents: [3]f32,
        y_radians: f32,
        area: u8,
    ) err.Error!void {
        try err.check(c.zrcTileCacheLayerMarkOrientedBox(
            self.handle,
            &origin,
            cell_size,
            cell_height,
            &center,
            &half_extents,
            y_radians,
            area,
        ));
    }

    /// Splits the layer's walkable surface into regions, in place.
    /// `walkable_climb` is in cells.
    pub fn buildRegions(self: TileCacheLayer, walkable_climb: i32) err.Error!void {
        try err.check(c.zrcTileCacheLayerBuildRegions(self.handle, walkable_climb));
    }
};

pub const TileCacheContourInfo = c.TileCacheContourInfo;

/// One contour vertex: x, y, z in cells, then the packed connection of the
/// edge that starts here.
pub const TileCacheContourVertex = [4]u8;

/// The traced outlines of a layer's regions.
pub const TileCacheContourSet = struct {
    handle: *c.TileCacheContourSet,

    /// Traces and simplifies the outline of every region in `layer`, which
    /// must have had its regions built. `max_error` is in cells.
    pub fn init(
        allocator: ?*const Allocator,
        layer: TileCacheLayer,
        walkable_climb: i32,
        max_error: f32,
    ) err.Error!TileCacheContourSet {
        var allocator_storage: c.TileCacheAllocator = undefined;
        var handle: *c.TileCacheContourSet = undefined;
        try err.check(c.zrcTileCacheContourSetCreate(
            toCAllocator(allocator, &allocator_storage),
            layer.handle,
            walkable_climb,
            max_error,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: TileCacheContourSet) void {
        c.zrcTileCacheContourSetDestroy(self.handle);
    }

    pub fn count(self: TileCacheContourSet) err.Error!u32 {
        var out: i32 = 0;
        try err.check(c.zrcTileCacheContourSetCount(self.handle, &out));
        return @intCast(out);
    }

    pub fn at(self: TileCacheContourSet, index: u32) err.Error!TileCacheContourInfo {
        const i = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        var out: TileCacheContourInfo = undefined;
        try err.check(c.zrcTileCacheContourAt(self.handle, i, &out));
        return out;
    }

    /// Copies vertices of contour `index` into `out`, starting at `first`.
    pub fn verts(
        self: TileCacheContourSet,
        index: u32,
        first: u32,
        out: []TileCacheContourVertex,
    ) err.Error!void {
        const idx = std.math.cast(i32, index) orelse return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const vert_count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcTileCacheContourVerts(self.handle, idx, first_i, vert_count, ptr));
    }
};

pub const TileCachePolyMeshInfo = c.TileCachePolyMeshInfo;

/// The polygons a layer's contours become — what a tile cache hands
/// Detour.
pub const TileCachePolyMesh = struct {
    handle: *c.TileCachePolyMesh,

    pub fn init(allocator: ?*const Allocator, contours: TileCacheContourSet) err.Error!TileCachePolyMesh {
        var allocator_storage: c.TileCacheAllocator = undefined;
        var handle: *c.TileCachePolyMesh = undefined;
        try err.check(c.zrcTileCachePolyMeshCreate(
            toCAllocator(allocator, &allocator_storage),
            contours.handle,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: TileCachePolyMesh) void {
        c.zrcTileCachePolyMeshDestroy(self.handle);
    }

    pub fn info(self: TileCachePolyMesh) err.Error!TileCachePolyMeshInfo {
        var out: TileCachePolyMeshInfo = undefined;
        try err.check(c.zrcTileCachePolyMeshInfo(self.handle, &out));
        return out;
    }

    /// Copies vertices into `out`, starting at `first`, one `[3]u16` each,
    /// in cells.
    pub fn verts(self: TileCachePolyMesh, first: u32, out: [][3]u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else @ptrCast(out.ptr);
        try err.check(c.zrcTileCachePolyMeshVerts(self.handle, first_i, count, ptr));
    }

    /// Copies polygons into `out`, starting at `first`, `2 * verts_per_poly`
    /// entries each: corner indices then the neighbour across each edge.
    /// `c.tilecache_null_idx` marks an absent corner or neighbour. `out.len`
    /// must be a whole number of polygons — the stride the C side cannot
    /// derive from a bare pointer.
    pub fn polys(self: TileCachePolyMesh, first: u32, out: []u16) err.Error!void {
        const stride = 2 * @as(usize, @intCast((try self.info()).verts_per_poly));
        if (stride == 0 or out.len % stride != 0) return err.Error.InvalidArgument;
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len / stride) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCachePolyMeshPolys(self.handle, first_i, count, ptr));
    }

    /// Copies area ids into `out`, one per polygon.
    pub fn areas(self: TileCachePolyMesh, first: u32, out: []u8) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u8 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCachePolyMeshAreas(self.handle, first_i, count, ptr));
    }

    /// Copies polygon flags into `out`, one per polygon. Every entry is 0
    /// unless a mesh-process callback has written it.
    pub fn flags(self: TileCachePolyMesh, first: u32, out: []u16) err.Error!void {
        const first_i = std.math.cast(i32, first) orelse return err.Error.InvalidArgument;
        const count = std.math.cast(i32, out.len) orelse return err.Error.InvalidArgument;
        const ptr: ?[*]u16 = if (out.len == 0) null else out.ptr;
        try err.check(c.zrcTileCachePolyMeshFlags(self.handle, first_i, count, ptr));
    }
};

//===----------------------------------------------------------------------===//
// Connection helpers
//
// Pure bit arithmetic on the packed connection byte `HeightfieldLayerSet.cons`
// and `TileCacheLayer.cons` report. Upstream packs it as
// `(portal << 4) | con` (RecastLayers.cpp), one bit per compass direction in
// each nibble — a different scheme from the six-bit-per-direction packing
// `pipeline.getCon` decodes from a compact heightfield span's `con` field.
//===----------------------------------------------------------------------===//

/// The low nibble of a layer connection byte: bit `1 << dir` set when the
/// neighbour cell in that direction sits inside the same layer.
///
/// Not `pipeline.getCon`'s packing — a compact heightfield span stores four
/// six-bit neighbour indices in a `u32`; a layer stores four one-bit flags
/// in a byte instead.
pub fn layerCon(byte: u8) u4 {
    return @truncate(byte);
}

/// The high nibble of a layer connection byte: bit `1 << dir` set when the
/// neighbour cell in that direction sits in a different layer, reached
/// through a portal rather than a direct step. Same packing note as
/// `layerCon`.
pub fn layerPortal(byte: u8) u4 {
    return @truncate(byte >> 4);
}

test "layerCon and layerPortal split the byte upstream packs as (portal << 4) | con" {
    const cases = [_]struct { byte: u8, con: u4, portal: u4 }{
        .{ .byte = 0x00, .con = 0x0, .portal = 0x0 },
        .{ .byte = 0x0f, .con = 0xf, .portal = 0x0 },
        .{ .byte = 0xf0, .con = 0x0, .portal = 0xf },
        .{ .byte = 0xff, .con = 0xf, .portal = 0xf },
        .{ .byte = 0x05, .con = 0x5, .portal = 0x0 },
        .{ .byte = 0x50, .con = 0x0, .portal = 0x5 },
        .{ .byte = 0xa3, .con = 0x3, .portal = 0xa },
        .{ .byte = 0x91, .con = 0x1, .portal = 0x9 },
    };
    for (cases) |case| {
        try std.testing.expectEqual(case.con, layerCon(case.byte));
        try std.testing.expectEqual(case.portal, layerPortal(case.byte));
    }
}

test "buildTileCacheLayer rejects an array shorter than header.width * header.height" {
    const header = TileCacheLayerHeader{
        .tile_x = 0,
        .tile_y = 0,
        .tile_layer = 0,
        .bmin = .{ 0, 0, 0 },
        .bmax = .{ 1, 1, 1 },
        .height_min = 0,
        .height_max = 1,
        .width = 4,
        .height = 4,
        .min_x = 0,
        .max_x = 3,
        .min_z = 0,
        .max_z = 3,
    };
    const heights = [_]u8{0} ** 16;
    const areas = [_]u8{0} ** 16;
    const short_cons = [_]u8{0} ** 15;
    const compressor = Compressor{};
    try std.testing.expectError(
        err.Error.InvalidArgument,
        buildTileCacheLayer(&compressor, header, &heights, &areas, &short_cons),
    );
}

test "buildTileCacheLayer rejects an array longer than header.width * header.height too" {
    const header = TileCacheLayerHeader{
        .tile_x = 0,
        .tile_y = 0,
        .tile_layer = 0,
        .bmin = .{ 0, 0, 0 },
        .bmax = .{ 1, 1, 1 },
        .height_min = 0,
        .height_max = 1,
        .width = 2,
        .height = 2,
        .min_x = 0,
        .max_x = 1,
        .min_z = 0,
        .max_z = 1,
    };
    const long_heights = [_]u8{0} ** 5;
    const areas = [_]u8{0} ** 4;
    const cons = [_]u8{0} ** 4;
    const compressor = Compressor{};
    try std.testing.expectError(
        err.Error.InvalidArgument,
        buildTileCacheLayer(&compressor, header, &long_heights, &areas, &cons),
    );
}
