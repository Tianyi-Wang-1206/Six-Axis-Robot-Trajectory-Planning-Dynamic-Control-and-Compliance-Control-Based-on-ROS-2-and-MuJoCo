#include "lite6_interfaces/shm_registers.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>

namespace lite6_interfaces::shm
{
namespace
{

std::string sanitize_namespace(const char * raw)
{
  if (!raw || raw[0] == '\0') {
    return "default";
  }
  std::string out;
  for (size_t i = 0; raw[i] != '\0' && out.size() < 48; ++i) {
    const unsigned char ch = static_cast<unsigned char>(raw[i]);
    if (std::isalnum(ch) || ch == '_' || ch == '-') {
      out.push_back(static_cast<char>(ch));
    }
  }
  return out.empty() ? std::string("default") : out;
}

std::string build_region_name(const char * suffix, uint32_t version)
{
  const std::string ns = sanitize_namespace(std::getenv("LITE6_SHM_NAMESPACE"));
  return "/lite6_shm_" + ns + "_" + std::string(suffix ? suffix : "seg") + "_v" +
         std::to_string(version) + "_" +
         std::to_string(static_cast<unsigned long>(::geteuid()));
}

bool header_is_ready(const ShmHeader * hdr, uint32_t version, size_t expected_size) noexcept
{
  return hdr->magic == kShmMagic &&
         hdr->version == version &&
         hdr->size_bytes == expected_size &&
         hdr->init_state.load(std::memory_order_acquire) == kInitReady;
}

bool header_is_clean_uninit(const ShmHeader * hdr) noexcept
{
  return hdr->magic == 0U &&
         hdr->version == 0U &&
         hdr->size_bytes == 0U &&
         hdr->init_state.load(std::memory_order_acquire) == kInitUninit;
}

}  // namespace

namespace detail
{

ShmOpenStatus open_or_create_region(
  const char * suffix,
  uint32_t version,
  size_t expected_size,
  bool writable,
  void ** out_addr)
{
  if (!out_addr || expected_size < sizeof(ShmHeader)) {
    return ShmOpenStatus::OS_ERROR;
  }
  *out_addr = nullptr;

  const std::string name = build_region_name(suffix, version);

  if (!writable) {
    // Read-only consumers never create, truncate, or modify the backing region.
    const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
    if (fd < 0) {
      return (errno == ENOENT) ? ShmOpenStatus::NOT_FOUND : ShmOpenStatus::OS_ERROR;
    }
    if (::flock(fd, LOCK_SH) != 0) {
      ::close(fd);
      return ShmOpenStatus::OS_ERROR;
    }

    struct stat st{};
    if (::fstat(fd, &st) != 0) {
      ::flock(fd, LOCK_UN);
      ::close(fd);
      return ShmOpenStatus::OS_ERROR;
    }
    if (st.st_size == 0) {
      ::flock(fd, LOCK_UN);
      ::close(fd);
      return ShmOpenStatus::NOT_FOUND;
    }
    if (static_cast<size_t>(st.st_size) != expected_size) {
      ::flock(fd, LOCK_UN);
      ::close(fd);
      return ShmOpenStatus::INCOMPATIBLE;
    }

    void * map = ::mmap(nullptr, expected_size, PROT_READ, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
      ::flock(fd, LOCK_UN);
      ::close(fd);
      return ShmOpenStatus::OS_ERROR;
    }

    const auto * hdr = static_cast<const ShmHeader *>(map);
    ShmOpenStatus status = ShmOpenStatus::OK;
    if (header_is_ready(hdr, version, expected_size)) {
      *out_addr = map;
    } else if (header_is_clean_uninit(hdr)) {
      ::munmap(map, expected_size);
      status = ShmOpenStatus::NOT_FOUND;
    } else {
      ::munmap(map, expected_size);
      status = ShmOpenStatus::INCOMPATIBLE;
    }

    ::flock(fd, LOCK_UN);
    ::close(fd);
    return status;
  }

  // Writable producer path: create if absent, initialize only if brand-new/uninitialized,
  // and reject size/header mismatches instead of truncating or zeroing live memory.
  const int fd = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  if (fd < 0) {
    return ShmOpenStatus::OS_ERROR;
  }
  if (::flock(fd, LOCK_EX) != 0) {
    ::close(fd);
    return ShmOpenStatus::OS_ERROR;
  }

  struct stat st{};
  if (::fstat(fd, &st) != 0) {
    ::flock(fd, LOCK_UN);
    ::close(fd);
    return ShmOpenStatus::OS_ERROR;
  }

  bool brand_new = false;
  if (st.st_size == 0) {
    if (::ftruncate(fd, static_cast<off_t>(expected_size)) != 0) {
      ::flock(fd, LOCK_UN);
      ::close(fd);
      return ShmOpenStatus::OS_ERROR;
    }
    brand_new = true;
  } else if (static_cast<size_t>(st.st_size) != expected_size) {
    ::flock(fd, LOCK_UN);
    ::close(fd);
    return ShmOpenStatus::INCOMPATIBLE;
  }

  void * map = ::mmap(nullptr, expected_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (map == MAP_FAILED) {
    ::flock(fd, LOCK_UN);
    ::close(fd);
    return ShmOpenStatus::OS_ERROR;
  }

  auto * hdr = static_cast<ShmHeader *>(map);
  ShmOpenStatus status = ShmOpenStatus::OK;
  if (brand_new || header_is_clean_uninit(hdr)) {
    std::memset(map, 0, expected_size);
    hdr->magic = kShmMagic;
    hdr->version = version;
    hdr->size_bytes = expected_size;
    hdr->init_state.store(kInitReady, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    *out_addr = map;
  } else if (header_is_ready(hdr, version, expected_size)) {
    *out_addr = map;
  } else {
    ::munmap(map, expected_size);
    status = ShmOpenStatus::INCOMPATIBLE;
  }

  ::flock(fd, LOCK_UN);
  ::close(fd);
  return status;
}

void close_region(void * addr, size_t size_bytes)
{
  if (addr != nullptr && size_bytes > 0) {
    ::munmap(addr, size_bytes);
  }
}

}  // namespace detail
}  // namespace lite6_interfaces::shm

extern "C" {

int64_t lite6_shm_steady_now_ns(void)
{
  return lite6_interfaces::shm::steady_now_ns();
}

int lite6_shm_is_stamp_fresh(int64_t stamp_ns, int64_t now_ns, int64_t max_age_ns)
{
  return lite6_interfaces::shm::is_stamp_fresh(stamp_ns, now_ns, max_age_ns) ? 1 : 0;
}

size_t lite6_shm_joint_pod_size(void)
{
  return sizeof(lite6_interfaces::shm::JointStatePod);
}

size_t lite6_shm_sysid_pod_size(void)
{
  return sizeof(lite6_interfaces::shm::SysIdSamplePod);
}

void * lite6_shm_state_open_ex(int writable, int * out_status)
{
  auto * handle = new lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment>();
  const auto status = handle->open_status(writable != 0);
  if (out_status) {
    *out_status = static_cast<int>(status);
  }
  if (status != lite6_interfaces::shm::ShmOpenStatus::OK) {
    delete handle;
    return nullptr;
  }
  return handle;
}

void * lite6_shm_state_open(int writable)
{
  return lite6_shm_state_open_ex(writable, nullptr);
}

void lite6_shm_state_close(void * handle)
{
  delete static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> *>(
    handle);
}

int lite6_shm_read_joint_state(
  void * handle,
  lite6_interfaces::shm::JointStatePod * out,
  uint64_t * out_seq)
{
  if (!handle || !out) {
    return 0;
  }
  auto * h =
    static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> *>(handle);
  if (!h->valid()) {
    return 0;
  }
  uint64_t seq = 0;
  if (!(*h)->joint_state.read(*out, seq)) {
    return 0;
  }
  if (out_seq) {
    *out_seq = seq;
  }
  return out->valid != 0 ? 1 : 0;
}

void * lite6_shm_sysid_open_ex(int writable, int * out_status)
{
  auto * handle = new lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::SysIdShmSegment>();
  const auto status = handle->open_status(writable != 0);
  if (out_status) {
    *out_status = static_cast<int>(status);
  }
  if (status != lite6_interfaces::shm::ShmOpenStatus::OK) {
    delete handle;
    return nullptr;
  }
  return handle;
}

void * lite6_shm_sysid_open(int writable)
{
  return lite6_shm_sysid_open_ex(writable, nullptr);
}

void lite6_shm_sysid_close(void * handle)
{
  delete static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::SysIdShmSegment> *>(
    handle);
}

void lite6_shm_sysid_set_active(void * handle, int active, int64_t heartbeat_ns)
{
  if (!handle) {
    return;
  }
  auto * h =
    static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::SysIdShmSegment> *>(handle);
  if (!h->valid()) {
    return;
  }
  if (active != 0) {
    (*h)->consumer_heartbeat_ns.store(heartbeat_ns, std::memory_order_release);
    (*h)->consumer_active.store(1U, std::memory_order_release);
  } else {
    (*h)->consumer_active.store(0U, std::memory_order_release);
  }
}

void lite6_shm_sysid_discard_all(void * handle)
{
  if (!handle) {
    return;
  }
  auto * h =
    static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::SysIdShmSegment> *>(handle);
  if (h->valid()) {
    (*h)->ring.discard_all();
  }
}

size_t lite6_shm_sysid_pop_batch(
  void * handle,
  lite6_interfaces::shm::SysIdSamplePod * out,
  size_t max_count)
{
  if (!handle || !out || max_count == 0) {
    return 0;
  }
  auto * h =
    static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::SysIdShmSegment> *>(handle);
  if (!h->valid()) {
    return 0;
  }
  return (*h)->ring.pop_batch(out, max_count);
}

uint64_t lite6_shm_sysid_dropped(void * handle)
{
  if (!handle) {
    return 0;
  }
  auto * h =
    static_cast<lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::SysIdShmSegment> *>(handle);
  if (!h->valid()) {
    return 0;
  }
  return (*h)->ring.dropped.load(std::memory_order_relaxed);
}

int lite6_shm_unlink_region(const char * suffix)
{
  if (!suffix) {
    return -1;
  }
  const std::string name =
    lite6_interfaces::shm::build_region_name(suffix, lite6_interfaces::shm::kShmVersion);
  return ::shm_unlink(name.c_str());
}

}  // extern "C"
