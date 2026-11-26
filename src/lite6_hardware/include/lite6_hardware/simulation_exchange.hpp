#ifndef LITE6_HARDWARE__SIMULATION_EXCHANGE_HPP_
#define LITE6_HARDWARE__SIMULATION_EXCHANGE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace lite6_hardware::detail
{

// One producer and one consumer. A full queue is reported to the caller;
// overwriting old commands would destroy the history needed during catch-up.
template<typename T, size_t Capacity>
class SpscQueue
{
public:
  bool push(const T & value)
  {
    const uint64_t head = head_.load(std::memory_order_relaxed);
    const uint64_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= Capacity) {return false;}
    entries_[head & kMask] = value;
    head_.store(head + 1ULL, std::memory_order_release);
    return true;
  }

  bool pop(T & value)
  {
    const uint64_t tail = tail_.load(std::memory_order_relaxed);
    const uint64_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {return false;}
    value = entries_[tail & kMask];
    tail_.store(tail + 1ULL, std::memory_order_release);
    return true;
  }

  bool pop_due(int64_t boundary_ns, T & value)
  {
    const uint64_t tail = tail_.load(std::memory_order_relaxed);
    const uint64_t head = head_.load(std::memory_order_acquire);
    if (tail == head || entries_[tail & kMask].issued_ns > boundary_ns) {
      return false;
    }
    value = entries_[tail & kMask];
    tail_.store(tail + 1ULL, std::memory_order_release);
    return true;
  }

  // Lifecycle only: neither producer nor consumer may be running.
  void reset()
  {
    head_.store(0ULL, std::memory_order_relaxed);
    tail_.store(0ULL, std::memory_order_relaxed);
  }

private:
  static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");
  static_assert(std::atomic<uint64_t>::is_always_lock_free);
  static constexpr uint64_t kMask = Capacity - 1;
  alignas(64) std::atomic<uint64_t> head_{0};
  alignas(64) std::atomic<uint64_t> tail_{0};
  alignas(64) std::array<T, Capacity> entries_{};
};

// Single-producer/single-consumer triple buffer with exclusive slot ownership.
// Use separate instances for additional readers.
template<typename T>
class LatestSnapshot
{
public:
  void publish(const T & value)
  {
    slots_[back_].value = value;
    back_ = middle_.exchange(back_ | kDirty, std::memory_order_acq_rel) & kIndex;
  }

  T read()
  {
    if (middle_.load(std::memory_order_acquire) & kDirty) {
      front_ = middle_.exchange(front_, std::memory_order_acq_rel) & kIndex;
    }
    return slots_[front_].value;
  }

private:
  struct alignas(64) Slot
  {
    T value{};
  };
  static_assert(std::atomic<unsigned>::is_always_lock_free);
  static constexpr unsigned kDirty = 4;
  static constexpr unsigned kIndex = 3;
  alignas(64) std::array<Slot, 3> slots_{};
  alignas(64) unsigned front_ = 0;  // Consumer only.
  alignas(64) unsigned back_ = 2;   // Producer only.
  alignas(64) std::atomic<unsigned> middle_{1};
};

}  // namespace lite6_hardware::detail

#endif  // LITE6_HARDWARE__SIMULATION_EXCHANGE_HPP_
