#ifndef LITE6_INTERFACES__SHM_REGISTERS_HPP_
#define LITE6_INTERFACES__SHM_REGISTERS_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace lite6_interfaces::shm
{

inline int64_t steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline bool is_stamp_fresh(int64_t stamp_ns, int64_t now_ns, int64_t max_age_ns) noexcept
{
  return stamp_ns > 0 && max_age_ns >= 0 && now_ns >= stamp_ns &&
         (now_ns - stamp_ns) <= max_age_ns;
}

inline void cpu_relax() noexcept
{
#if defined(__x86_64__) || defined(__i386__)
  __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
  asm volatile("yield" ::: "memory");
#else
  std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}

constexpr uint32_t kShmMagic = 0x4C365348U;  // "L6SH"
constexpr uint32_t kShmVersion = 1U;
constexpr uint32_t kInitUninit = 0U;
constexpr uint32_t kInitReady = 2U;

struct alignas(64) ShmHeader
{
  uint32_t magic = 0;
  uint32_t version = 0;
  uint64_t size_bytes = 0;
  std::atomic<uint32_t> init_state{kInitUninit};
  uint32_t reserved = 0;
};

static_assert(std::atomic<uint32_t>::is_always_lock_free);
static_assert(std::atomic<uint64_t>::is_always_lock_free);
static_assert(std::atomic<int64_t>::is_always_lock_free);
static_assert(std::atomic<uint8_t>::is_always_lock_free);

/**
 * Lock-free atomic-word SeqLock register for wait-free single-writer,
 * multi-reader POD snapshots without concurrent non-atomic accesses.
 */
template<typename T>
struct alignas(64) ShmSeqRegister
{
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(sizeof(T) % sizeof(uint64_t) == 0);
  static constexpr size_t kWords = sizeof(T) / sizeof(uint64_t);

  alignas(64) std::atomic<uint64_t> seq{0};
  alignas(64) std::array<std::atomic<uint64_t>, kWords> words{};

  void write(const T & value) noexcept
  {
    uint64_t buf[kWords];
    std::memcpy(buf, &value, sizeof(T));
    uint64_t s = seq.load(std::memory_order_relaxed);
    if ((s & 1ULL) == 0ULL) {
      s += 1ULL;
    }
    seq.store(s, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acq_rel);
    for (size_t i = 0; i < kWords; ++i) {
      words[i].store(buf[i], std::memory_order_relaxed);
    }
    seq.store(s + 1ULL, std::memory_order_release);
  }

  void clear() noexcept
  {
    write(T{});
  }

  bool read(T & out, uint64_t & out_seq, uint32_t max_retries = 64) const noexcept
  {
    uint64_t buf[kWords];
    for (uint32_t attempt = 0; attempt < max_retries; ++attempt) {
      const uint64_t s1 = seq.load(std::memory_order_acquire);
      if (s1 == 0ULL) {
        return false;
      }
      if ((s1 & 1ULL) != 0ULL) {
        cpu_relax();
        continue;
      }
      for (size_t i = 0; i < kWords; ++i) {
        buf[i] = words[i].load(std::memory_order_relaxed);
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      const uint64_t s2 = seq.load(std::memory_order_relaxed);
      if (s1 == s2) {
        std::memcpy(&out, buf, sizeof(T));
        out_seq = s1;
        return true;
      }
      cpu_relax();
    }
    return false;
  }

  bool read_if_updated(T & out, uint64_t & inout_seq, uint32_t max_retries = 64) const noexcept
  {
    const uint64_t cur = seq.load(std::memory_order_acquire);
    if (cur == 0ULL || cur == inout_seq) {
      return false;
    }
    uint64_t new_seq = 0;
    if (!read(out, new_seq, max_retries) || new_seq == inout_seq) {
      return false;
    }
    inout_seq = new_seq;
    return true;
  }
};

/**
 * Lock-free single-producer, single-consumer ring buffer for lossless
 * cross-process time-series streaming.
 */
template<typename T, size_t Capacity>
struct alignas(64) ShmSpscRing
{
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
  static constexpr uint64_t kMask = Capacity - 1;

  alignas(64) std::atomic<uint64_t> head{0};
  alignas(64) std::atomic<uint64_t> tail{0};
  alignas(64) std::atomic<uint64_t> dropped{0};
  alignas(64) std::array<T, Capacity> entries{};

  bool push(const T & item) noexcept
  {
    const uint64_t h = head.load(std::memory_order_relaxed);
    const uint64_t t = tail.load(std::memory_order_acquire);
    if (h - t >= Capacity) {
      dropped.fetch_add(1ULL, std::memory_order_relaxed);
      return false;
    }
    entries[h & kMask] = item;
    head.store(h + 1ULL, std::memory_order_release);
    return true;
  }

  size_t pop_batch(T * out, size_t max_count) noexcept
  {
    if (!out || max_count == 0) {
      return 0;
    }
    const uint64_t t = tail.load(std::memory_order_relaxed);
    const uint64_t h = head.load(std::memory_order_acquire);
    const uint64_t avail = h - t;
    if (avail == 0ULL) {
      return 0;
    }
    const size_t n = static_cast<size_t>(std::min<uint64_t>(avail, max_count));
    for (size_t i = 0; i < n; ++i) {
      out[i] = entries[(t + i) & kMask];
    }
    tail.store(t + n, std::memory_order_release);
    return n;
  }

  void discard_all() noexcept
  {
    const uint64_t h = head.load(std::memory_order_acquire);
    tail.store(h, std::memory_order_release);
  }
};

struct alignas(8) JointStatePod
{
  double position[6]{};
  double velocity[6]{};
  double effort[6]{};
  double gripper_position[2]{};
  double gripper_velocity[2]{};
  int64_t ros_stamp_ns = 0;
  int64_t steady_stamp_ns = 0;
  uint64_t sequence = 0;
  uint8_t valid = 0;
  uint8_t reserved[7]{};
};

struct alignas(8) VisualStatePod
{
  double position[6]{};
  double velocity[6]{};
  double gripper_position[2]{};
  double gripper_velocity[2]{};
  double object_qpos[14]{};
  double simulation_time = 0.0;
  int64_t published_steady_ns = 0;
  uint64_t steps = 0;
  uint8_t active = 0;
  uint8_t valid = 0;
  uint8_t reserved[6]{};
};

struct alignas(8) ShadowStatePod
{
  double position[6]{};
  int64_t stamp_ns = 0;
  uint64_t controller_epoch = 0;
  uint8_t valid = 0;
  uint8_t reserved[7]{};
};

struct alignas(8) MousePerturbationPod
{
  int64_t issued_ns = 0;
  double refpos[3]{};
  double refselpos[3]{};
  double refquat[4]{};
  double localpos[3]{};
  double localmass = 0.0;
  int32_t body_id = 0;
  int32_t active = 0;
};

struct alignas(8) SysIdSamplePod
{
  int64_t stamp_ns = 0;
  int64_t read_stamp_ns = 0;
  double simulation_time = 0.0;
  uint64_t step = 0;
  uint64_t command_sequence = 0;
  double control_period = 0.0;
  double physics_lag = 0.0;
  double command_age = 0.0;
  double position[6]{};
  double velocity[6]{};
  double effort[6]{};
  double gripper_position[2]{};
  double gripper_velocity[2]{};
  double gripper_acceleration[2]{};
};

/**
 * Memory-mapped state segment written by ros2_control_node (hardware + controller)
 * and mapped read-only (PROT_READ) by viewer, planners, and HMIs.
 */
struct alignas(64) StateShmSegment
{
  ShmHeader header;
  alignas(64) std::atomic<int64_t> visual_alive_ns{0};
  ShmSeqRegister<JointStatePod> joint_state;
  ShmSeqRegister<VisualStatePod> visual_state;
  ShmSeqRegister<ShadowStatePod> shadow_state;
};

/**
 * Memory-mapped perturbation segment written by mujoco_viewer and mapped
 * read-only (PROT_READ) by Lite6MujocoSystem.
 */
struct alignas(64) PerturbShmSegment
{
  ShmHeader header;
  ShmSeqRegister<MousePerturbationPod> mouse_perturbation;
};

/**
 * Memory-mapped SysID sample ring shared between Lite6MujocoSystem (producer)
 * and lite6_hmi (consumer).
 */
struct alignas(64) SysIdShmSegment
{
  static constexpr size_t kRingCapacity = 16384;

  ShmHeader header;
  alignas(64) std::atomic<uint32_t> consumer_active{0};
  alignas(64) std::atomic<int64_t> consumer_heartbeat_ns{0};
  ShmSpscRing<SysIdSamplePod, kRingCapacity> ring;
};

namespace detail
{
bool open_or_create_region(
  const char * suffix,
  uint32_t version,
  size_t expected_size,
  bool writable,
  void ** out_addr);

void close_region(void * addr, size_t size_bytes);
}  // namespace detail

template<typename SegmentT>
class ShmHandle
{
public:
  ShmHandle() = default;
  ~ShmHandle() { close(); }

  ShmHandle(const ShmHandle &) = delete;
  ShmHandle & operator=(const ShmHandle &) = delete;

  ShmHandle(ShmHandle && other) noexcept : ptr_(other.ptr_)
  {
    other.ptr_ = nullptr;
  }

  ShmHandle & operator=(ShmHandle && other) noexcept
  {
    if (this != &other) {
      close();
      ptr_ = other.ptr_;
      other.ptr_ = nullptr;
    }
    return *this;
  }

  bool open(bool writable);

  void close() noexcept
  {
    if (ptr_ != nullptr) {
      detail::close_region(ptr_, sizeof(SegmentT));
      ptr_ = nullptr;
    }
  }

  bool valid() const noexcept { return ptr_ != nullptr; }
  SegmentT * get() noexcept { return ptr_; }
  const SegmentT * get() const noexcept { return ptr_; }
  SegmentT * operator->() noexcept { return ptr_; }
  const SegmentT * operator->() const noexcept { return ptr_; }

private:
  SegmentT * ptr_ = nullptr;
};

template<>
inline bool ShmHandle<StateShmSegment>::open(bool writable)
{
  close();
  void * addr = nullptr;
  if (!detail::open_or_create_region("state", kShmVersion, sizeof(StateShmSegment), writable, &addr)) {
    return false;
  }
  ptr_ = static_cast<StateShmSegment *>(addr);
  return true;
}

template<>
inline bool ShmHandle<PerturbShmSegment>::open(bool writable)
{
  close();
  void * addr = nullptr;
  if (!detail::open_or_create_region(
        "perturb", kShmVersion, sizeof(PerturbShmSegment), writable, &addr))
  {
    return false;
  }
  ptr_ = static_cast<PerturbShmSegment *>(addr);
  return true;
}

template<>
inline bool ShmHandle<SysIdShmSegment>::open(bool writable)
{
  close();
  void * addr = nullptr;
  if (!detail::open_or_create_region("sysid", kShmVersion, sizeof(SysIdShmSegment), writable, &addr)) {
    return false;
  }
  ptr_ = static_cast<SysIdShmSegment *>(addr);
  return true;
}

}  // namespace lite6_interfaces::shm

#endif  // LITE6_INTERFACES__SHM_REGISTERS_HPP_
