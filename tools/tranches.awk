# zrecast — which tranche of work closes an unbound name.
#
# tools/record.sh feeds every public Recast/Detour name that tools/bindings.tsv
# does not already answer for through this file, and writes the answer into
# tools/unbound_*.txt as the gap's evidence. So the reasoning for a gap lives
# here, once per rule, instead of once per row.
#
# That matters more than it looks. The first version of the record carried a
# sentence of prose on every one of its 937 gaps, and a single tranche rotted
# dozens of them at once: rows saying a tile grid was not exposed survived the
# tranche that exposed it. A rule is re-read every time the record is generated.
#
# The rules are ordered and the first match wins, so a specific name can be
# lifted out of its owner's tranche by putting it above the owner's rule.
#
# Input:  AREA <TAB> SYMBOL, where SYMBOL is KIND:OWNER::NAME[/ARITY].
# Output: AREA <TAB> SYMBOL <TAB> LABEL.

function owner_of(sym,    rest, p) {
  rest = sym; sub(/^[a-z]+:/, "", rest)      # drop the kind
  p = index(rest, "::")
  return p ? substr(rest, 1, p - 1) : "-"
}

function name_of(sym,    rest, p) {
  rest = sym; sub(/\/[0-9]+(\(.*)?$/, "", rest)   # drop the arity and any types
  p = index(rest, "::")
  return p ? substr(rest, p + 2) : rest
}

# The labels. ci/check-coverage.sh requires a gap's evidence to start with one
# of these, so a gap can never be a gap nobody has scheduled.
BEGIN {
  T2  = "tranche 2 - deterministic cooks"
  T3A = "tranche 3a - authoring: areas, off-mesh links, poly flags"
  T3B = "tranche 3b - the staged Recast pipeline"
  T4  = "tranche 4 - query breadth"
  T5  = "tranche 5 - DetourTileCache"
  T6  = "tranche 6 - DetourCrowd"
  T7  = "tranche 7 - value math, geometry and containers"
  T8  = "tranche 8 - reading a loaded navmesh back"
  FS = "\t"; OFS = "\t"
}

{
  area = $1; sym = $2
  owner = owner_of(sym); name = name_of(sym)
  print area, sym, tranche(area, owner, name, sym)
}

function tranche(area, owner, name, sym) {

  #--- Named exceptions. These sit above the family rules that would otherwise
  #--- claim them, so the reason each one moves is visible at the point of the
  #--- move rather than buried in a wider pattern.

  # A bake input, the write side of dtMeshHeader::userId. It travels with the
  # rest of the authoring parameters, not with the accessors that read a tile.
  if (sym == "field:dtNavMeshCreateParams::userId") return T3A

  # Plain tile data. Whether a tile carries a BV tree, and whether it owns the
  # buffer it was added with, are read back beside the header's other fields.
  if (sym == "field:dtNavMeshCreateParams::buildBvTree") return T8
  if (owner == "dtTileFlags" || name == "dtTileFlags") return T8
  if (name == "DT_TILE_FREE_DATA") return T8

  # Computational geometry with a real algorithm inside. A second Zig
  # implementation of these would be a liability to keep in step with upstream,
  # so they cross the boundary as entry points.
  if (name ~ /^dt(ClosestPtPointTriangle|DistancePtPolyEdgesSqr|DistancePtSegSqr2D)$/) return T7
  if (name ~ /^dt(IntersectSegmentPoly2D|IntersectSegSeg2D|OverlapPolyPoly2D)$/) return T7
  if (name ~ /^dt(OverlapBounds|OverlapQuantBounds|PointInPolygon)$/) return T7
  if (name ~ /^dt(RandomPointInConvexPoly|CalcPolyCenter|TriArea2D)$/) return T7
  if (name == "rcOffsetPoly") return T7

  # Not value math: these decode rcCompactSpan::con, so they belong with the
  # compact heightfield they read.
  if (name ~ /^rc(GetCon|SetCon|GetDirOffsetX|GetDirOffsetY|GetDirForOffset)$/) return T3B

  # Area authoring. The marking functions are what a designer's volumes become,
  # and they run inside the bake rather than as separate pipeline stages.
  if (name ~ /^rc(MarkBoxArea|MarkConvexPolyArea|MarkCylinderArea)$/) return T3A

  # Value math, in both spellings. The rcV/dtV families and the scalar helpers
  # around them are the same decision.
  if (name ~ /^(rc|dt)V[A-Za-z0-9]*$/) return T7
  if (name ~ /^(rc|dt)(Min|Max|Abs|Sqr|Sqrt|Clamp|Swap|IgnoreUnused)$/) return T7
  # Math[A-Za-z0-9]+ rather than Math[A-Za-z]+: dtMathAtan2f carries a digit,
  # and without it the one member of that family with a number in its name
  # falls through every rule below to whichever tranche owns the area.
  if (name ~ /^dt(Ilog2|NextPow2|Align4|Alloc|Math[A-Za-z0-9]+)$/) return T7
  if (name ~ /^dtGetThenAdvanceBufferPointer$/) return T7

  # Routing upstream's asserts into a host's logger. Both halves, together: the
  # two hook families are separate seams upstream, and a binding that installed
  # one of them would give a host its own logger for half the library and a bare
  # abort for the other half.
  if (name ~ /^(dt|rc)AssertFail/) return T7

  # The C++ container family, and the scoped-delete guard beside it.
  if (owner ~ /^rc(VectorBase|TempVector|PermVector|IntArray|ScopedDelete)$/) return T7
  if (name ~ /^rc(VectorBase|TempVector|PermVector|IntArray|ScopedDelete|SizeType)$/) return T7

  # Byte-order swapping, and the magic that identifies the image being swapped.
  # A cook that is bit-identical everywhere is what makes an endian-portable
  # image worth having at all.
  if (name ~ /^dt(SwapByte|SwapEndian|NavMeshDataSwapEndian|NavMeshHeaderSwapEndian)$/) return T2
  if (name == "DT_NAVMESH_MAGIC") return T2

  #--- Authoring: off-mesh connections, per-polygon areas and flags, and the
  #--- per-tile state that saves and restores them.

  if (owner == "dtPolyTypes" || name == "dtPolyTypes") return T3A
  if (name ~ /^DT_(NAVMESH_STATE_MAGIC|NAVMESH_STATE_VERSION|OFFMESH_CON_BIDIR)$/) return T3A
  if (owner == "dtPoly" && name ~ /^(areaAndtype|flags|getArea|setArea|getType|setType)$/) return T3A
  if (owner == "dtMeshTile" && name == "offMeshCons") return T3A
  # dtOffMeshConnection::poly and ::side, and dtMeshHeader's off-mesh counts,
  # are the wiring rather than the capability: authoring never names them and
  # nothing a host calls reports them. They belong with the tranche that reads
  # a tile's own structures back.
  if (owner == "dtOffMeshConnection") return T8
  if (owner == "dtNavMeshCreateParams" && name ~ /^(offMeshCon|polyAreas|polyFlags)/) return T3A
  if (owner == "dtNavMesh" && name ~ /^(getOffMeshConnection|getPolyArea|getPolyFlags|setPolyArea|setPolyFlags)/) return T3A
  if (owner == "dtNavMesh" && name ~ /^(getTileStateSize|storeTileState|restoreTileState)$/) return T3A

  #--- Subsystems, whole.

  if (area == "DetourCrowd") return T6
  if (area == "DetourTileCache") return T5
  if (area == "DetourNavMeshQuery") return T4

  # The layered heightfield is what the tile cache compresses and rebuilds from,
  # so it arrives with the tile cache rather than with the rest of Recast.
  if (owner ~ /^rcHeightfieldLayer(Set)?$/) return T5
  if (name ~ /^rcHeightfieldLayer(Set)?$/) return T5
  if (name ~ /^rc(AllocHeightfieldLayerSet|BuildHeightfieldLayers|FreeHeightfieldLayerSet)$/) return T5
  if (owner == "dtNavMesh" && name == "getTilesAt") return T5

  #--- Query internals and the queries themselves.

  if (owner ~ /^dt(Node|NodePool|NodeQueue|NodeFlags)$/) return T4
  if (name ~ /^dt(Node|NodePool|NodeQueue|NodeFlags|NodeIndex)$/) return T4
  if (name ~ /^dt(Status|FindPathOptions|RaycastOptions|StraightPathOptions|StraightPathFlags|DetailTriEdgeFlags|PolyQuery)$/) return T4
  if (name ~ /^DT_(NULL_IDX|POLY_BITS|SALT_BITS|TILE_BITS|IN_PROGRESS|STATUS_DETAIL_MASK)$/) return T4
  if (name ~ /^DT_(MAX_STATES_PER_NODE|NODE_PARENT_BITS|NODE_STATE_BITS|RAY_CAST_LIMIT_PROPORTIONS)$/) return T4
  if (name ~ /^dt(StatusInProgress|GetDetailTriEdgeFlags|ClosestHeightPointTriangle)$/) return T4
  if (owner == "dtNavMesh" && name ~ /^(decodePolyId|encodePolyId|getTileAndPolyByRef|isValidPolyRef)/) return T4

  #--- Reading a loaded navmesh back: the tile's own arrays and header.

  if (owner ~ /^dt(BVNode|Link|MeshHeader|MeshTile|Poly|PolyDetail)$/) return T8
  if (name ~ /^dt(BVNode|Link|MeshHeader|MeshTile|Poly|PolyDetail)$/) return T8
  if (name ~ /^DT_(EXT_LINK|NULL_LINK)$/) return T8
  if (name == "dtOppositeTile") return T8
  if (owner == "dtNavMesh" && name ~ /^(calcTileLoc|getParams|getPolyRefBase)$/) return T8

  #--- Everything left in Recast is the pipeline: its stages, the containers
  #--- they fill, the build context that logs and times them, and the constants
  #--- those stages write into a mesh.

  if (area == "Recast") return T3B
  if (area == "Detour") return T4

  return "UNCLASSIFIED"
}
