#ifndef ROOMBABOT_FIRMWARE__SEQLOCK_HPP
#define ROOMBABOT_FIRMWARE__SEQLOCK_HPP

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace roombabot_firmware
{

// Single-writer / single-reader handoff for a small POD, with no mutex on either side.
//
// This is the piece that keeps serial latency out of the 100 Hz control loop. The
// serial reader thread blocks on poll()/read() and drops each decoded frame in here;
// ros2_control's read() picks up whatever the latest one is and returns immediately.
// No syscall, no allocation, and -- unlike a mutex -- no chance of the control loop
// being descheduled while holding a lock the reader wants, or vice versa.
//
// The sequence counter is odd while a write is in progress, so a reader that observes
// an odd count, or a different count before and after its copy, knows it saw a torn
// value and retries. Reads are bounded rather than unbounded-spin: after a few
// attempts the reader gives up and the caller reuses the previous sample. Missing one
// 10 ms update is harmless; stalling the control loop is not.
template<typename T>
class SeqlockSlot
{
  static_assert(std::is_trivially_copyable<T>::value, "SeqlockSlot requires a trivially copyable type");

public:
  // Called only from the reader thread.
  void store(const T & value) noexcept
  {
    const uint32_t seq = sequence_.load(std::memory_order_relaxed);
    sequence_.store(seq + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    value_ = value;

    std::atomic_thread_fence(std::memory_order_release);
    sequence_.store(seq + 2, std::memory_order_relaxed);
  }

  // Called only from the control thread. Returns false if nothing has ever been
  // stored, or if the writer was mid-update for every attempt -- in both cases `out`
  // is left untouched.
  bool load(T & out) const noexcept
  {
    for (int attempt = 0; attempt < kMaxReadAttempts; ++attempt)
    {
      const uint32_t before = sequence_.load(std::memory_order_acquire);
      if (before == 0 || (before & 1u) != 0u)
      {
        continue;  // never written, or a write is in flight
      }

      T candidate = value_;

      std::atomic_thread_fence(std::memory_order_acquire);
      if (sequence_.load(std::memory_order_relaxed) == before)
      {
        out = candidate;
        return true;
      }
    }
    return false;
  }

  // Monotonic count of completed writes -- handy for spotting a link that has gone
  // quiet without having to compare payload contents.
  uint32_t writeCount() const noexcept
  {
    return sequence_.load(std::memory_order_acquire) / 2u;
  }

private:
  static constexpr int kMaxReadAttempts = 4;

  std::atomic<uint32_t> sequence_{0};
  T value_{};
};

}  // namespace roombabot_firmware

#endif  // ROOMBABOT_FIRMWARE__SEQLOCK_HPP
