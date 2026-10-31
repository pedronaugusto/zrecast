//===----------------------------------------------------------------------===//
// zrecast — synthetic test geometry, generated at test time; why this shape is
// in docs/design.md, The fixture. Test-only: not part of the library or ABI.
//
//        +Z          . . . . .
//         |          .       .   <- island, disconnected from everything
//         |          . . . . .
//    +----+--------------------+
//    |  G |        goal        |
//    |####|####################|  <- wall, with a gap at its +X end
//    |    |            [ ]     |  <- pillar
//    | S  |                    |
//    +----+--------------------+ --- +X
//===----------------------------------------------------------------------===//

#ifndef ZRECAST_TEST_FIXTURE_H_
#define ZRECAST_TEST_FIXTURE_H_

#include "zrecast.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Half-extent of the square ground plane, centred on the origin at y = 0.
#define ZRC_FIXTURE_GROUND_EXTENT 10.0f

/// The wall runs along z = 0 from the -X edge to here, leaving a gap between
/// this and +ZRC_FIXTURE_GROUND_EXTENT.
#define ZRC_FIXTURE_WALL_END_X 4.0f

/// Centre of the disconnected island, in the xz plane.
#define ZRC_FIXTURE_ISLAND_X 40.0f
#define ZRC_FIXTURE_ISLAND_Z 40.0f
#define ZRC_FIXTURE_ISLAND_EXTENT 3.0f

/// Exact vertex and triangle counts of zrcFixtureTriMesh, so a test can assert
/// it received the mesh it expected rather than an empty one.
#define ZRC_FIXTURE_VERT_COUNT 24
#define ZRC_FIXTURE_TRI_COUNT 28

/// Points the tests navigate between. Both sit on the ground, on opposite sides
/// of the wall and well clear of it.
#define ZRC_FIXTURE_START_X (-8.0f)
#define ZRC_FIXTURE_START_Y 0.0f
#define ZRC_FIXTURE_START_Z (-8.0f)
#define ZRC_FIXTURE_GOAL_X (-8.0f)
#define ZRC_FIXTURE_GOAL_Y 0.0f
#define ZRC_FIXTURE_GOAL_Z 8.0f

/// Ground, wall, pillar and island as one indexed triangle soup.
///
/// Points at static storage that lives for the whole process; there is nothing
/// to free, and no allocation happens, so a test asserting on the allocator's
/// counters is not disturbed by building geometry.
void zrcFixtureTriMesh(ZrcTriMesh* out);

/// A tent of two 71-degree faces: real three-dimensional bounds, but not one
/// surface an agent could stand on. Bakes to zero polygons, which is the
/// ZRC_ERR_EMPTY_RESULT path.
void zrcFixtureUnwalkableTriMesh(ZrcTriMesh* out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZRECAST_TEST_FIXTURE_H_
