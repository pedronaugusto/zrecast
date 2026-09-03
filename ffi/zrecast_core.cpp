//===----------------------------------------------------------------------===//
// zrecast — version reporting, result naming, and the allocator seam.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

//===----------------------------------------------------------------------===//
// Allocator seam
//
// Recast and Detour each take a bare pair of function pointers with no user
// context (rcAllocSetCustom, dtAllocSetCustom). There is nowhere to thread a
// host pointer through, so the installed ZrcAllocator lives in a file-scope
// slot and the thunks below read it. That is upstream's design surfaced, not a
// shortcut: the seam is global on their side, so it is global here.
//===----------------------------------------------------------------------===//

/// Function-local static rather than a namespace-scope object so there is no
/// static-initialisation-order dependency against upstream's own defaults.
ZrcAllocator& Host() {
  static ZrcAllocator instance = {nullptr, nullptr, nullptr};
  return instance;
}

void* RecastAllocThunk(size_t size, rcAllocHint hint) {
  const ZrcAllocator& host = Host();
  if (host.allocate == nullptr) return nullptr;
  // rcAllocHint and dtAllocHint are separate enums with identical meaning and
  // identical enumerator values; ZrcAllocHint is the one both map onto. The
  // static_asserts in zrecast_abi.cpp fail the build if that stops holding.
  return host.allocate(host.user, size,
                       hint == RC_ALLOC_TEMP ? ZRC_ALLOC_TEMP : ZRC_ALLOC_PERM);
}

void RecastFreeThunk(void* block) {
  const ZrcAllocator& host = Host();
  if (host.deallocate == nullptr) return;
  host.deallocate(host.user, block);
}

void* DetourAllocThunk(size_t size, dtAllocHint hint) {
  const ZrcAllocator& host = Host();
  if (host.allocate == nullptr) return nullptr;
  return host.allocate(host.user, size,
                       hint == DT_ALLOC_TEMP ? ZRC_ALLOC_TEMP : ZRC_ALLOC_PERM);
}

void DetourFreeThunk(void* block) {
  const ZrcAllocator& host = Host();
  if (host.deallocate == nullptr) return;
  host.deallocate(host.user, block);
}

//===----------------------------------------------------------------------===//
// Assertion seam
//
// Both halves of upstream's internal-assertion mechanism — Recast's
// rcAssertFailSetCustom and Detour's dtAssertFailSetCustom — are installed
// from one ZrcAssertHandler, the same shape as the allocator seam above. The
// hook family they belong to does not exist at all under NDEBUG: in both
// DetourAssert.h and RecastAssert.h the typedef, the setter and the getter
// sit inside `#else NDEBUG`, and rcAssert/dtAssert compile down to
// `(void)sizeof(x)`. The slot below still records a handler in that build —
// zrcAssertHandler reads it back regardless — but nothing ever calls it.
//===----------------------------------------------------------------------===//

/// Function-local static for the same reason Host() above is: no
/// static-initialisation-order dependency against upstream's own defaults.
ZrcAssertHandler& AssertHost() {
  static ZrcAssertHandler instance = {nullptr, nullptr};
  return instance;
}

#ifndef NDEBUG

/// Matches rcAssertFailFunc's signature exactly: no user pointer, because
/// upstream's hook takes none. The host pointer comes from AssertHost().
void RecastAssertThunk(const char* expression, const char* file, int line) {
  const ZrcAssertHandler& host = AssertHost();
  if (host.fail == nullptr) return;
  host.fail(host.user, expression, file, static_cast<int32_t>(line));
}

/// Identical in body to RecastAssertThunk, but a separate function: dtAssert's
/// hook and rcAssert's hook are distinct types upstream, even though both
/// happen to share this signature.
void DetourAssertThunk(const char* expression, const char* file, int line) {
  const ZrcAssertHandler& host = AssertHost();
  if (host.fail == nullptr) return;
  host.fail(host.user, expression, file, static_cast<int32_t>(line));
}

#endif  // NDEBUG

}  // namespace

extern "C" {

uint32_t zrcVersion(void) {
  return (static_cast<uint32_t>(ZRC_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(ZRC_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(ZRC_VERSION_PATCH);
}

uint32_t zrcRecastVersion(void) {
  // Pinned in UPSTREAM.md; bump both together when re-vendoring.
  return (1u << 16) | (6u << 8) | 0u;
}

int32_t zrcNavMeshDataVersion(void) {
  return static_cast<int32_t>(DT_NAVMESH_VERSION);
}

const char* zrcResultName(ZrcResult result) {
  switch (result) {
    case ZRC_OK:
      return "ok";
    case ZRC_ERR_INVALID_ARGUMENT:
      return "invalid argument";
    case ZRC_ERR_OUT_OF_MEMORY:
      return "out of memory";
    case ZRC_ERR_BAD_FORMAT:
      return "bad format";
    case ZRC_ERR_UNSUPPORTED_VERSION:
      return "unsupported navmesh data version";
    case ZRC_ERR_BUFFER_TOO_SMALL:
      return "buffer too small";
    case ZRC_ERR_BAKE_FAILED:
      return "bake failed";
    case ZRC_ERR_EMPTY_RESULT:
      return "bake produced no polygons";
    case ZRC_ERR_QUERY_FAILED:
      return "query failed";
    case ZRC_ERR_OUT_OF_NODES:
      return "query ran out of nodes";
    case ZRC_ERR_TILE_OCCUPIED:
      return "a tile already occupies that grid position";
    case ZRC_ERR_NAVMESH_FULL:
      return "the navmesh has no free tile slot";
    case ZRC_ERR_SEARCH_IN_PROGRESS:
      return "a sliced path search is in flight on this query";
    case ZRC_ERR_NO_SEARCH:
      return "no sliced path search is in flight on this query";
    case ZRC_ERR_NOT_FOUND:
      return "not found";
    case ZRC_ERR_ALREADY_BUILT:
      return "the destination container already holds a result";
    case ZRC_ERR_CROWD_FULL:
      return "the crowd has no free agent slot";
  }
  return "unknown result";
}

ZrcResult zrcSetAllocator(const ZrcAllocator* alloc) {
  if (alloc == nullptr) {
    // Upstream restores its own malloc/free defaults when handed nulls, so
    // there is nothing to remember and nothing to guess at.
    rcAllocSetCustom(nullptr, nullptr);
    dtAllocSetCustom(nullptr, nullptr);
    Host() = ZrcAllocator{nullptr, nullptr, nullptr};
    return ZRC_OK;
  }

  if (alloc->allocate == nullptr || alloc->deallocate == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }

  Host() = *alloc;
  rcAllocSetCustom(RecastAllocThunk, RecastFreeThunk);
  dtAllocSetCustom(DetourAllocThunk, DetourFreeThunk);
  return ZRC_OK;
}

void* zrcAlloc(size_t size, ZrcAllocHint hint) {
  return zrc::Alloc(size, hint == ZRC_ALLOC_TEMP ? DT_ALLOC_TEMP : DT_ALLOC_PERM);
}

void zrcFree(void* block) { zrc::Free(block); }

ZrcBool zrcAssertsEnabled(void) {
#ifdef NDEBUG
  return ZRC_FALSE;
#else
  return ZRC_TRUE;
#endif
}

ZrcResult zrcSetAssertHandler(const ZrcAssertHandler* handler) {
  if (handler == nullptr) {
    AssertHost() = ZrcAssertHandler{nullptr, nullptr};
#ifndef NDEBUG
    rcAssertFailSetCustom(nullptr);
    dtAssertFailSetCustom(nullptr);
#endif
    return ZRC_OK;
  }

  if (handler->fail == nullptr) return ZRC_ERR_INVALID_ARGUMENT;

  AssertHost() = *handler;
#ifndef NDEBUG
  rcAssertFailSetCustom(RecastAssertThunk);
  dtAssertFailSetCustom(DetourAssertThunk);
#endif
  return ZRC_OK;
}

ZrcResult zrcAssertHandler(ZrcAssertHandler* out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = AssertHost();
  return ZRC_OK;
}

}  // extern "C"

//===----------------------------------------------------------------------===//
// Shared helpers declared in zrecast_internal.h
//===----------------------------------------------------------------------===//

namespace zrc {

ZrcResult ResultFromStatus(dtStatus status) {
  if (dtStatusSucceed(status)) return ZRC_OK;
  // Order matters only in that the most actionable cause should win when
  // several detail bits are set at once.
  if (dtStatusDetail(status, DT_OUT_OF_NODES)) return ZRC_ERR_OUT_OF_NODES;
  if (dtStatusDetail(status, DT_OUT_OF_MEMORY)) return ZRC_ERR_OUT_OF_MEMORY;
  if (dtStatusDetail(status, DT_INVALID_PARAM)) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL)) {
    return ZRC_ERR_BUFFER_TOO_SMALL;
  }
  if (dtStatusDetail(status, DT_WRONG_VERSION)) {
    return ZRC_ERR_UNSUPPORTED_VERSION;
  }
  if (dtStatusDetail(status, DT_WRONG_MAGIC)) return ZRC_ERR_BAD_FORMAT;
  return ZRC_ERR_QUERY_FAILED;
}

}  // namespace zrc
