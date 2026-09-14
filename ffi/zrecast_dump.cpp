//===----------------------------------------------------------------------===//
// zrecast — RecastDump: a build written to a host's bytes, and read back.
//
// Upstream's four dump functions and two read functions take a duFileIO, which
// is four hooks and no file. HostFileIO wears that interface over the POD the
// C ABI carries.
//
// Upstream diagnoses a null or wrong-mode stream by printing to stdout and
// returning false, which is a library writing to a host's console. Every such
// case is refused here first, so that printf is unreachable from this package.
//
// The two read entry points build their container rather than filling a
// caller's. Upstream's read the counts straight out of the stream and allocate
// from them, so a half-read container is a real possibility; creating it here
// means a failure destroys it rather than handing back something partly
// filled. What the stream says is still trusted — see UPSTREAM.md.
//===----------------------------------------------------------------------===//

#include "zrecast_internal.h"

namespace {

/// All four hooks are required. A stream that answered `is_writing` but had no
/// `write` would fail partway through a dump, with whatever was already
/// transferred left behind.
ZrcResult ValidateFileIO(const ZrcFileIO* io) {
  if (io == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  if (io->is_writing == nullptr || io->is_reading == nullptr ||
      io->write == nullptr || io->read == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  return ZRC_OK;
}

}  // namespace

namespace zrc {

HostFileIO::HostFileIO(const ZrcFileIO& hooks) : hooks_(hooks) {}

bool HostFileIO::isWriting() const {
  return hooks_.is_writing(hooks_.user) != ZRC_FALSE;
}

bool HostFileIO::isReading() const {
  return hooks_.is_reading(hooks_.user) != ZRC_FALSE;
}

bool HostFileIO::write(const void* ptr, const size_t size) {
  return hooks_.write(hooks_.user, ptr, size) != ZRC_FALSE;
}

bool HostFileIO::read(void* ptr, const size_t size) {
  return hooks_.read(hooks_.user, ptr, size) != ZRC_FALSE;
}

}  // namespace zrc

ZRC_API ZrcResult zrcDumpPolyMeshToObj(const ZrcPolyMesh* mesh,
                                       const ZrcFileIO* io) {
  const ZrcResult check = ValidateFileIO(io);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->poly == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostFileIO stream(*io);
  if (!stream.isWriting()) return ZRC_ERR_INVALID_ARGUMENT;
  // Upstream takes the mesh by non-const reference and writes nothing
  // through it; the handle is const, the mesh it points at is not.
  return duDumpPolyMeshToObj(*mesh->poly, &stream) ? ZRC_OK
                                                   : ZRC_ERR_QUERY_FAILED;
}

ZRC_API ZrcResult zrcDumpPolyMeshDetailToObj(const ZrcPolyMesh* mesh,
                                             const ZrcFileIO* io) {
  const ZrcResult check = ValidateFileIO(io);
  if (check != ZRC_OK) return check;
  if (mesh == nullptr || mesh->detail == nullptr) {
    return ZRC_ERR_INVALID_ARGUMENT;
  }
  zrc::HostFileIO stream(*io);
  if (!stream.isWriting()) return ZRC_ERR_INVALID_ARGUMENT;
  return duDumpPolyMeshDetailToObj(*mesh->detail, &stream)
             ? ZRC_OK
             : ZRC_ERR_QUERY_FAILED;
}

ZRC_API ZrcResult zrcDumpContourSet(const ZrcContourSet* cset,
                                    const ZrcFileIO* io) {
  const ZrcResult check = ValidateFileIO(io);
  if (check != ZRC_OK) return check;
  if (cset == nullptr || cset->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostFileIO stream(*io);
  if (!stream.isWriting()) return ZRC_ERR_INVALID_ARGUMENT;
  return duDumpContourSet(*cset->impl, &stream) ? ZRC_OK : ZRC_ERR_QUERY_FAILED;
}

ZRC_API ZrcResult zrcReadContourSet(const ZrcFileIO* io, ZrcContourSet** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult check = ValidateFileIO(io);
  if (check != ZRC_OK) return check;
  zrc::HostFileIO stream(*io);
  if (!stream.isReading()) return ZRC_ERR_INVALID_ARGUMENT;

  ZrcContourSet* handle = zrc::New<ZrcContourSet>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = rcAllocContourSet();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  if (!duReadContourSet(*handle->impl, &stream)) {
    // rcFreeContourSet walks nconts and releases whichever vertex arrays were
    // allocated before the failure; rcAllocContourSet zeroed the rest.
    rcFreeContourSet(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_BAD_FORMAT;
  }
  *out = handle;
  return ZRC_OK;
}

ZRC_API ZrcResult zrcDumpCompactHeightfield(const ZrcCompactHeightfield* chf,
                                            const ZrcFileIO* io) {
  const ZrcResult check = ValidateFileIO(io);
  if (check != ZRC_OK) return check;
  if (chf == nullptr || chf->impl == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostFileIO stream(*io);
  if (!stream.isWriting()) return ZRC_ERR_INVALID_ARGUMENT;
  return duDumpCompactHeightfield(*chf->impl, &stream) ? ZRC_OK
                                                       : ZRC_ERR_QUERY_FAILED;
}

ZRC_API ZrcResult zrcReadCompactHeightfield(const ZrcFileIO* io,
                                            ZrcCompactHeightfield** out) {
  if (out == nullptr) return ZRC_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  const ZrcResult check = ValidateFileIO(io);
  if (check != ZRC_OK) return check;
  zrc::HostFileIO stream(*io);
  if (!stream.isReading()) return ZRC_ERR_INVALID_ARGUMENT;

  ZrcCompactHeightfield* handle = zrc::New<ZrcCompactHeightfield>();
  if (handle == nullptr) return ZRC_ERR_OUT_OF_MEMORY;
  handle->impl = rcAllocCompactHeightfield();
  if (handle->impl == nullptr) {
    zrc::Delete(handle);
    return ZRC_ERR_OUT_OF_MEMORY;
  }
  if (!duReadCompactHeightfield(*handle->impl, &stream)) {
    rcFreeCompactHeightfield(handle->impl);
    zrc::Delete(handle);
    return ZRC_ERR_BAD_FORMAT;
  }
  *out = handle;
  return ZRC_OK;
}

ZRC_API ZrcResult zrcLogBuildTimes(const ZrcBuildContext* context,
                                   int32_t total_usec) {
  // Upstream divides 100.0f by this to turn each phase into a percentage, so
  // zero produces an infinity that reaches the host's log as a formatted
  // number, and a negative total produces percentages of the wrong sign.
  if (total_usec <= 0) return ZRC_ERR_INVALID_ARGUMENT;
  zrc::HostContext ctx(context);
  duLogBuildTimes(ctx, total_usec);
  return ZRC_OK;
}
