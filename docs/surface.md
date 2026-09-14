# The bound surface, capability by capability

`tools/unbound_*.txt` is the record and `ci/check-coverage.sh` is the gate;
this is the same answer in prose, grouped by what a host is trying to do. See
[coverage.md](coverage.md) for how the record is derived and what the three
verdicts mean.

- **Bake** — the full Recast pipeline over a triangle soup: heightfield,
  filters, compact heightfield, erosion, regions (all three partitioning
  strategies), contours, polygon mesh, detail mesh.
- **Tiles** — a tile grid computed from the geometry, a per-tile bake whose empty
  tiles are a success rather than an error, each tile cooked to bytes of its own,
  and add/remove/lookup/enumerate against a live navmesh. A path crosses tile
  boundaries through the portal edges a tiled bake emits.
- **Areas and flags** — convex, box and cylinder volumes applied between erosion
  and region building, colouring the surface with area ids a query filter charges
  for; a table mapping each area id to the polygon flags a filter admits or
  refuses; and `get`/`set` for both on a live navmesh, so a door opening or a
  zone flooding is a write rather than a re-cook.
- **Off-mesh connections** — point-to-point links across ground the surface does
  not join: a jump, a ladder, a door. One-way or two-way, carrying an area, flags
  and an opaque id, supplied when a tile is built and read back from a live
  navmesh. A path through one produces a straight-path corner flagged
  `ZRC_STRAIGHTPATH_OFFMESH_CONNECTION`, which is how a game knows to play an
  animation instead of walking.
- **Tile state** — one tile's polygon areas and flags as bytes, for a save file.
  The blob is only restorable onto the tile it came from, and only at exactly the
  length that tile reports.
- **Reading a navmesh back** — the grid parameters, a tile's header, its
  polygons, links, detail sub-meshes, vertices and bounding-volume nodes, all
  copied out **by value** so a read survives the tile's removal. Ranges are
  half-open and a range outside an array is an error, never a short read, since
  a short read is indistinguishable from an empty tile.
- **Navmesh** — a `dtNavMesh` from that bake, single-tile or tiled; a single-tile
  one serialises to and from bytes, with the image validated before it is
  trusted.
- **Queries** — the whole of `dtNavMeshQuery`: nearest polygon, path,
  string-pulled corners, surface movement, walkability raycast, the polygons in
  a box (into a buffer or through a callback), the three outward searches and
  the corridor one leaves behind, wall segments and distance to the nearest
  wall, closest point and height on a single polygon, and reproducible random
  placement. Every one that fills a caller's buffer reports whether it had to
  stop short.
- **Sliced pathfinding** — the same search driven a few iterations at a time so
  a frame can spend a fixed budget on it. The filter is copied for the slice's
  lifetime, because upstream keeps only a pointer to it; a second search that
  would clear the node pool underneath a slice is refused; and finalising twice
  is refused rather than answering with a one-element path holding the null
  reference.
- **What a reference means** — validity against a filter or on its own, whether
  the last search closed it, and the salt/tile/polygon fields packed into it,
  split and rebuilt with the widths that navmesh actually uses.
- **The search's own node pool** — how full it is and what the last search
  concluded about a given polygon, which is what makes `max_nodes` tunable
  rather than guessed.
- **Geometry** — the computational geometry Detour runs on, callable directly:
  point-in-polygon, closest point on a triangle, distance to a polygon's edges,
  segment/polygon and segment/segment intersection, the two overlap tests a
  BV-tree walk needs, polygon offsetting and a reproducible random point in a
  convex polygon. Each one checks the array bound upstream leaves to the caller.
- **Vector math** — the `dtV`/`rcV` families and the scalar helpers around them,
  in Zig rather than across the boundary, and asserted **bit-identical** to the
  C over a table that includes zeros, negative zeros and denormals. Six of
  them answer differently from Zig's obvious spelling at an edge, and the tests
  pin all six.
- **Seams** — the allocator, reachable for a host's own allocations, and one
  assertion handler installed into both halves of upstream, with a way to ask
  whether this build kept the assertions that would call it.
- **Tile images** — the byte offset and length of each of a tile's eight
  arrays, so a host can parse or patch a cooked image with the layout Detour
  itself derives rather than one it guessed.
- **The Recast pipeline, stage by stage** — the same bake taken apart. Each
  stage is its own call, each intermediate container is a handle a host owns,
  and every one can be read, edited or replaced between one stage and the next:
  spans added by hand, areas painted from data no volume shape can express,
  polygon flags written before the mesh becomes a tile. A staged mesh goes into
  `zrcTileDataBuild` exactly as a baked one does, and the suite asserts the two
  produce identical bytes from identical input.
- **A build context** — the log and timer hooks Recast calls during a build,
  with the two enable flags upstream consults before it reaches one, so a host
  measuring a bake measures the phases a C++ host measures.
- **Dynamic obstacles** — a tile cache: each tile's walkable surface kept in a
  compressed layer beside the navmesh, and rebuilt whenever an obstacle over it
  appears or goes away. Cylinders, boxes and boxes rotated about y, queued and
  carved a tile per update so a frame can spend a fixed budget on it. The codec
  is the host's — none is bundled and no container format is invented — and so
  is the callback that decides each rebuilt polygon's flags, narrowed to the
  fields it is for because the tile it produces reaches the navmesh without the
  validation an added tile gets. The suite carves an obstacle into the fixture's
  only gap, watches the route close, removes it, and watches it open.
- **Drawing** — every container a bake or a load produces, rendered through a
  renderer the host supplies: the input soup shaded by slope, the heightfield,
  the compact heightfield solid, by region and by distance, the layered
  heightfield, the traced and simplified contours, the polygon and detail
  meshes, a navmesh's polygons, bounding-volume tree, portals and off-mesh
  connections, a search's node pool and closed list, and a tile cache's layers,
  contours and rebuilt mesh. Plus the shapes upstream draws them out of —
  boxes, cylinders, arcs, arrows, circles, crosses and a grid — each in a form
  that opens its own run and a form that appends to one the caller opened. The
  renderer is a Zig struct whose methods are read at compile time; a display
  list records one run of primitives and replays it into any other renderer.
- **Dumping a build** — the polygon mesh and the detail mesh as Wavefront OBJ,
  and the contour set and compact heightfield as upstream's own binary form,
  written through four hooks a host fills rather than to a file this package
  opens. Both binary forms read back into a container this package creates, so
  a failed read destroys a half-built one rather than handing it over. The
  format is upstream's struct layout copied to the stream, so it is neither
  endian- nor padding-portable and carries no length to check a count against:
  feed a read only bytes this package wrote.
- **Crowds** — many agents steering around each other and the world: a local
  re-plan per agent per frame, neighbours from a proximity grid, walls from a
  cached local boundary, a velocity chosen by an obstacle-avoidance sampler, and
  a position integrated under acceleration and speed limits. An agent is named
  by a reference carrying a serial rather than by its pool slot, so a slot
  reused after a removal cannot be driven by a stale handle — upstream has no
  identity at all and every setter it offers takes a bare index. The two
  parameter fields that index the crowd's filter and avoidance tables are
  refused at the door; unchecked, they read tens of kilobytes past those tables
  every frame. Long searches go through a path queue whose filters this package
  owns for the request's lifetime, because upstream keeps only a pointer to the
  caller's. The corridor, the local boundary, the proximity grid and the
  avoidance sampler are each usable on their own, which is what a host steering
  one character wants.
