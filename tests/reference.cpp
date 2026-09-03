//===----------------------------------------------------------------------===//
// zrecast — single-line forwarding shims giving upstream's inline math a
// linkable name. See reference.h for why these exist.
//===----------------------------------------------------------------------===//

#include "reference.h"

#include <string.h>

#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourNavMesh.h"
#include "Recast.h"

extern "C" {

//===----------------------------------------------------------------------===//
// Detour vectors
//===----------------------------------------------------------------------===//

void zrcRefDtVcross(float* d, const float* a, const float* b) { dtVcross(d, a, b); }
float zrcRefDtVdot(const float* a, const float* b) { return dtVdot(a, b); }
void zrcRefDtVmad(float* d, const float* a, const float* b, float s) { dtVmad(d, a, b, s); }
void zrcRefDtVlerp(float* d, const float* a, const float* b, float t) { dtVlerp(d, a, b, t); }
void zrcRefDtVadd(float* d, const float* a, const float* b) { dtVadd(d, a, b); }
void zrcRefDtVsub(float* d, const float* a, const float* b) { dtVsub(d, a, b); }
void zrcRefDtVscale(float* d, const float* v, float t) { dtVscale(d, v, t); }
void zrcRefDtVmin(float* mn, const float* v) { dtVmin(mn, v); }
void zrcRefDtVmax(float* mx, const float* v) { dtVmax(mx, v); }
void zrcRefDtVset(float* d, float x, float y, float z) { dtVset(d, x, y, z); }
void zrcRefDtVcopy(float* d, const float* a) { dtVcopy(d, a); }
float zrcRefDtVlen(const float* v) { return dtVlen(v); }
float zrcRefDtVlenSqr(const float* v) { return dtVlenSqr(v); }
float zrcRefDtVdist(const float* a, const float* b) { return dtVdist(a, b); }
float zrcRefDtVdistSqr(const float* a, const float* b) { return dtVdistSqr(a, b); }
float zrcRefDtVdist2D(const float* a, const float* b) { return dtVdist2D(a, b); }
float zrcRefDtVdist2DSqr(const float* a, const float* b) { return dtVdist2DSqr(a, b); }
void zrcRefDtVnormalize(float* v) { dtVnormalize(v); }
int zrcRefDtVequal(const float* a, const float* b) { return dtVequal(a, b) ? 1 : 0; }
int zrcRefDtVisfinite(const float* v) { return dtVisfinite(v) ? 1 : 0; }
int zrcRefDtVisfinite2D(const float* v) { return dtVisfinite2D(v) ? 1 : 0; }
float zrcRefDtVdot2D(const float* a, const float* b) { return dtVdot2D(a, b); }
float zrcRefDtVperp2D(const float* a, const float* b) { return dtVperp2D(a, b); }

//===----------------------------------------------------------------------===//
// Recast vectors
//===----------------------------------------------------------------------===//

void zrcRefRcVcross(float* d, const float* a, const float* b) { rcVcross(d, a, b); }
float zrcRefRcVdot(const float* a, const float* b) { return rcVdot(a, b); }
void zrcRefRcVmad(float* d, const float* a, const float* b, float s) { rcVmad(d, a, b, s); }
void zrcRefRcVadd(float* d, const float* a, const float* b) { rcVadd(d, a, b); }
void zrcRefRcVsub(float* d, const float* a, const float* b) { rcVsub(d, a, b); }
void zrcRefRcVmin(float* mn, const float* v) { rcVmin(mn, v); }
void zrcRefRcVmax(float* mx, const float* v) { rcVmax(mx, v); }
void zrcRefRcVcopy(float* d, const float* v) { rcVcopy(d, v); }
float zrcRefRcVdist(const float* a, const float* b) { return rcVdist(a, b); }
float zrcRefRcVdistSqr(const float* a, const float* b) { return rcVdistSqr(a, b); }
void zrcRefRcVnormalize(float* v) { rcVnormalize(v); }

//===----------------------------------------------------------------------===//
// Scalars, instantiated at float explicitly.
//===----------------------------------------------------------------------===//

uint32_t zrcRefDtNextPow2(uint32_t v) { return dtNextPow2(v); }
uint32_t zrcRefDtIlog2(uint32_t v) { return dtIlog2(v); }
int32_t zrcRefDtAlign4(int32_t x) { return dtAlign4(x); }
float zrcRefDtMinF(float a, float b) { return dtMin<float>(a, b); }
float zrcRefDtMaxF(float a, float b) { return dtMax<float>(a, b); }
float zrcRefDtAbsF(float a) { return dtAbs<float>(a); }
float zrcRefDtSqrF(float a) { return dtSqr<float>(a); }
float zrcRefDtClampF(float v, float mn, float mx) { return dtClamp<float>(v, mn, mx); }
float zrcRefRcMinF(float a, float b) { return rcMin<float>(a, b); }
float zrcRefRcMaxF(float a, float b) { return rcMax<float>(a, b); }
float zrcRefRcAbsF(float a) { return rcAbs<float>(a); }
float zrcRefRcSqrF(float a) { return rcSqr<float>(a); }
float zrcRefRcClampF(float v, float mn, float mx) { return rcClamp<float>(v, mn, mx); }
float zrcRefRcSqrt(float x) { return rcSqrt(x); }
float zrcRefDtMathSqrtf(float x) { return dtMathSqrtf(x); }
float zrcRefDtMathFabsf(float x) { return dtMathFabsf(x); }
float zrcRefDtMathFloorf(float x) { return dtMathFloorf(x); }
float zrcRefDtMathCeilf(float x) { return dtMathCeilf(x); }
float zrcRefDtMathCosf(float x) { return dtMathCosf(x); }
float zrcRefDtMathSinf(float x) { return dtMathSinf(x); }
float zrcRefDtMathAtan2f(float y, float x) { return dtMathAtan2f(y, x); }
int zrcRefDtMathIsfinite(float x) { return dtMathIsfinite(x) ? 1 : 0; }

//===----------------------------------------------------------------------===//
// Tile layout oracle
//===----------------------------------------------------------------------===//

void zrcRefTileArrayOffsets(const void* data, int64_t* out_offsets) {
  dtMeshHeader header;
  memcpy(&header, data, sizeof(header));

  // Mirrors dtNavMesh::addTile (Detour/Source/DetourNavMesh.cpp) literally:
  // the same eight sizes, in the same order, walked with the same primitive.
  const int headerSize = dtAlign4(sizeof(dtMeshHeader));
  const int vertsSize = dtAlign4(sizeof(float) * 3 * header.vertCount);
  const int polysSize = dtAlign4(sizeof(dtPoly) * header.polyCount);
  const int linksSize = dtAlign4(sizeof(dtLink) * header.maxLinkCount);
  const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * header.detailMeshCount);
  const int detailVertsSize = dtAlign4(sizeof(float) * 3 * header.detailVertCount);
  const int detailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * header.detailTriCount);
  const int bvtreeSize = dtAlign4(sizeof(dtBVNode) * header.bvNodeCount);
  const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection) * header.offMeshConCount);

  // The const-buffer overload of dtGetThenAdvanceBufferPointer reinterpret_casts
  // to exactly the template argument given, so a const buffer needs a const
  // argument explicitly — the same way DetourNavMesh.cpp's own const walk of a
  // tile-state blob instantiates it as `<const dtTileState>` rather than
  // `<dtTileState>`.
  const unsigned char* const base = static_cast<const unsigned char*>(data);
  const unsigned char* d = base + headerSize;

  out_offsets[0] = d - base;
  dtGetThenAdvanceBufferPointer<const float>(d, static_cast<size_t>(vertsSize));
  out_offsets[1] = d - base;
  dtGetThenAdvanceBufferPointer<const dtPoly>(d, static_cast<size_t>(polysSize));
  out_offsets[2] = d - base;
  dtGetThenAdvanceBufferPointer<const dtLink>(d, static_cast<size_t>(linksSize));
  out_offsets[3] = d - base;
  dtGetThenAdvanceBufferPointer<const dtPolyDetail>(
      d, static_cast<size_t>(detailMeshesSize));
  out_offsets[4] = d - base;
  dtGetThenAdvanceBufferPointer<const float>(d, static_cast<size_t>(detailVertsSize));
  out_offsets[5] = d - base;
  dtGetThenAdvanceBufferPointer<const unsigned char>(
      d, static_cast<size_t>(detailTrisSize));
  out_offsets[6] = d - base;
  dtGetThenAdvanceBufferPointer<const dtBVNode>(d, static_cast<size_t>(bvtreeSize));
  out_offsets[7] = d - base;
  dtGetThenAdvanceBufferPointer<const dtOffMeshConnection>(
      d, static_cast<size_t>(offMeshLinksSize));
}

}  // extern "C"
