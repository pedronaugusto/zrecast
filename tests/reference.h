//===----------------------------------------------------------------------===//
// zrecast — linkage shims over upstream's inline vector and scalar math.
//
// src/vec.zig re-implements this arithmetic in Zig, and a test has to prove it
// bit-identical to the C. The upstream functions are `inline`, declared in a
// header with no matching .cpp, so they carry no linkable symbol of their own;
// these shims give each one a name a test can call. Test-only: not part of the
// zrecast library, not part of the ABI, not installed.
//===----------------------------------------------------------------------===//

#ifndef ZRECAST_TEST_REFERENCE_H_
#define ZRECAST_TEST_REFERENCE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Detour vectors (dtV*, Detour/Include/DetourCommon.h)
//===----------------------------------------------------------------------===//

void zrcRefDtVcross(float* d, const float* a, const float* b);
float zrcRefDtVdot(const float* a, const float* b);
void zrcRefDtVmad(float* d, const float* a, const float* b, float s);
void zrcRefDtVlerp(float* d, const float* a, const float* b, float t);
void zrcRefDtVadd(float* d, const float* a, const float* b);
void zrcRefDtVsub(float* d, const float* a, const float* b);
void zrcRefDtVscale(float* d, const float* v, float t);
void zrcRefDtVmin(float* mn, const float* v);
void zrcRefDtVmax(float* mx, const float* v);
void zrcRefDtVset(float* d, float x, float y, float z);
void zrcRefDtVcopy(float* d, const float* a);
float zrcRefDtVlen(const float* v);
float zrcRefDtVlenSqr(const float* v);
float zrcRefDtVdist(const float* a, const float* b);
float zrcRefDtVdistSqr(const float* a, const float* b);
float zrcRefDtVdist2D(const float* a, const float* b);
float zrcRefDtVdist2DSqr(const float* a, const float* b);
void zrcRefDtVnormalize(float* v);
int zrcRefDtVequal(const float* a, const float* b);
int zrcRefDtVisfinite(const float* v);
int zrcRefDtVisfinite2D(const float* v);
float zrcRefDtVdot2D(const float* a, const float* b);
float zrcRefDtVperp2D(const float* a, const float* b);

//===----------------------------------------------------------------------===//
// Recast vectors (rcV*, Recast/Include/Recast.h)
//===----------------------------------------------------------------------===//

void zrcRefRcVcross(float* d, const float* a, const float* b);
float zrcRefRcVdot(const float* a, const float* b);
void zrcRefRcVmad(float* d, const float* a, const float* b, float s);
void zrcRefRcVadd(float* d, const float* a, const float* b);
void zrcRefRcVsub(float* d, const float* a, const float* b);
void zrcRefRcVmin(float* mn, const float* v);
void zrcRefRcVmax(float* mx, const float* v);
void zrcRefRcVcopy(float* d, const float* v);
float zrcRefRcVdist(const float* a, const float* b);
float zrcRefRcVdistSqr(const float* a, const float* b);
void zrcRefRcVnormalize(float* v);

//===----------------------------------------------------------------------===//
// Scalars: dtMin/dtMax/dtAbs/dtSqr/dtClamp and the rc equivalents,
// instantiated at float; plus the free-standing math helpers each side layers
// over <cmath>.
//===----------------------------------------------------------------------===//

uint32_t zrcRefDtNextPow2(uint32_t v);
uint32_t zrcRefDtIlog2(uint32_t v);
int32_t zrcRefDtAlign4(int32_t x);
float zrcRefDtMinF(float a, float b);
float zrcRefDtMaxF(float a, float b);
float zrcRefDtAbsF(float a);
float zrcRefDtSqrF(float a);
float zrcRefDtClampF(float v, float mn, float mx);
float zrcRefRcMinF(float a, float b);
float zrcRefRcMaxF(float a, float b);
float zrcRefRcAbsF(float a);
float zrcRefRcSqrF(float a);
float zrcRefRcClampF(float v, float mn, float mx);
float zrcRefRcSqrt(float x);
float zrcRefDtMathSqrtf(float x);
float zrcRefDtMathFabsf(float x);
float zrcRefDtMathFloorf(float x);
float zrcRefDtMathCeilf(float x);
float zrcRefDtMathCosf(float x);
float zrcRefDtMathSinf(float x);
float zrcRefDtMathAtan2f(float y, float x);
int zrcRefDtMathIsfinite(float x);

/// Byte offset of each of a tile image's eight arrays, derived the way
/// dtNavMesh::addTile derives them: by walking the buffer with
/// dtGetThenAdvanceBufferPointer in its fixed order. Eight entries, in the
/// order verts, polys, links, detail meshes, detail verts, detail tris,
/// BV tree, off-mesh connections. The caller guarantees a valid image.
void zrcRefTileArrayOffsets(const void* data, int64_t* out_offsets);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZRECAST_TEST_REFERENCE_H_
