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
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % (Capacity + 1);
    if (next == tail_.load(std::memory_order_acquire)) {return false;}
    entries_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T & value)
  {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {return false;}
    value = entries_[tail];
    tail_.store((tail + 1) % (Capacity + 1), std::memory_order_release);
    return true;
  }

  bool pop_due(int64_t boundary_ns, T & value)
  {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire) ||
      entries_[tail].issued_ns > boundary_ns)
    {
      return false;
    }
    value = entries_[tail];
    tail_.store((tail + 1) % (Capacity + 1), std::memory_order_release);
    return true;
  }

  // Lifecycle only: neither producer nor consumer may be running.
  void reset()
  {
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
  }

private:
  static_assert(Capacity > 0);
  static_assert(std::atomic<size_t>::is_always_lock_free);
  std::array<T, Capacity + 1> entries_{};
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
};

// Latest complete snapshot, with independent front/back ownership. Exchanging
// the middle slot transfers ownership; neither side touches the other's slot.
// Unlike a seqlock over plain doubles, this has no concurrent non-atomic access.
// Exactly one producer and one consumer; use separate instances for readers.
template<typename T>
class LatestSnapshot
{
public:
  void publish(const T & value)
  {
    slots_[back_] = value;
    back_ = middle_.exchange(back_ | kDirty, std::memory_order_acq_rel) & kIndex;
  }

  T read()
  {
    if (middle_.load(std::memory_order_acquire) & kDirty) {
      front_ = middle_.exchange(front_, std::memory_order_acq_rel) & kIndex;
    }
    return slots_[front_];
  }

private:
  static_assert(std::atomic<unsigned>::is_always_lock_free);
  static constexpr unsigned kDirty = 4;
  static constexpr unsigned kIndex = 3;
  std::array<T, 3> slots_{};
  unsigned front_ = 0;  // Consumer only.
  unsigned back_ = 2;   // Producer only.
  std::atomic<unsigned> middle_{1};
};

}  // namespace lite6_hardware::detail

#endif  // LITE6_HARDWARE__SIMULATION_EXCHANGE_HPP_
