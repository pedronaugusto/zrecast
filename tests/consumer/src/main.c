/* Proves the installed artifact: <zrecast.h> resolves through installHeader
 * alone, the static library links from a plain C program, and a bake runs. */

#include <stdio.h>
#include <stdlib.h>

#include <zrecast.h>

int main(void) {
  const float verts[] = {
      -10, 0, -10, /**/ 10, 0, -10, /**/ 10, 0, 10, /**/ -10, 0, 10,
  };
  const int tris[] = {0, 2, 1, 0, 3, 2};

  ZrcTriMesh mesh = {verts, 4, tris, 2};
  ZrcBakeConfig config;
  zrcBakeConfigDefault(&config);

  ZrcPolyMesh* poly = NULL;
  if (zrcPolyMeshBake(&config, &mesh, NULL, NULL, &poly) != ZRC_OK) {
    fprintf(stderr, "c consumer: bake failed\n");
    return 1;
  }

  ZrcNavMesh* navmesh = NULL;
  if (zrcNavMeshCreate(poly, NULL, &navmesh) != ZRC_OK) {
    fprintf(stderr, "c consumer: navmesh create failed\n");
    zrcPolyMeshDestroy(poly);
    return 1;
  }

  void* data = NULL;
  size_t size = 0;
  if (zrcNavMeshSerialize(navmesh, &data, &size) != ZRC_OK || size == 0) {
    fprintf(stderr, "c consumer: serialize failed\n");
    zrcNavMeshDestroy(navmesh);
    zrcPolyMeshDestroy(poly);
    return 1;
  }

  printf("c consumer: ok (%u-byte image, zrecast %u over recast %u)\n",
         (unsigned)size, zrcVersion(), zrcRecastVersion());

  zrcFree(data);
  zrcNavMeshDestroy(navmesh);
  zrcPolyMeshDestroy(poly);
  return 0;
}
