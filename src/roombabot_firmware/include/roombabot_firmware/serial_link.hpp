#ifndef ROOMBABOT_FIRMWARE__SERIAL_LINK_HPP
#define ROOMBABOT_FIRMWARE__SERIAL_LINK_HPP

#include "roombabot_firmware/esp32_protocol.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace roombabot_firmware
{

// One framed, binary serial link to an ESP32.
//
// Deliberately raw POSIX termios rather than libserial: libserial's ReadLine reads a
// byte at a time through a C++ stream and is built around a blocking, line-oriented
// model, which is exactly the wrong shape for a fixed-rate control loop. Here a
// dedicated thread owns the fd and does the blocking, the frame callback fires on
// that thread, and the control loop never touches the fd except to write.
//
// Writes are non-blocking by design. If the kernel's TX buffer is full the frame is
// dropped and counted rather than stalling the caller -- a velocity setpoint that is
// 10 ms stale is worth less than the next one, which is already on its way.
class SerialLink
{
public:
  // type, payload pointer, payload length. Invoked on the reader thread, once per
  // CRC-valid frame. Keep it short; anything slow belongs off this thread.
  using FrameHandler = std::function<void (uint8_t, const uint8_t *, size_t)>;

  SerialLink() = default;
  ~SerialLink();

  SerialLink(const SerialLink &) = delete;
  SerialLink & operator=(const SerialLink &) = delete;

  // Opens and configures the port in raw mode. `device` is a udev symlink such as
  // /dev/esp32_motor. Returns false and fills `error` on failure.
  bool open(const std::string & device, unsigned int baud_rate, std::string & error);

  // Spawns the reader thread. Safe to call only once per open().
  void start(FrameHandler handler);

  // Joins the reader thread and closes the fd. Idempotent.
  void close();

  bool isOpen() const { return fd_ >= 0; }

  // Frames a payload and pushes it out in a single write() syscall. Returns false if
  // the port is closed or the frame was dropped because the TX buffer was full.
  bool writeFrame(protocol::MsgType type, const void * payload, size_t length);

  // Diagnostics. All monotonic, all safe to read from any thread.
  uint64_t bytesReceived() const { return bytes_received_.load(std::memory_order_relaxed); }
  uint64_t framesReceived() const { return frames_received_.load(std::memory_order_relaxed); }
  uint64_t crcErrors() const { return crc_errors_.load(std::memory_order_relaxed); }
  uint64_t resyncs() const { return resyncs_.load(std::memory_order_relaxed); }
  uint64_t writeDrops() const { return write_drops_.load(std::memory_order_relaxed); }
  uint64_t readErrors() const { return read_errors_.load(std::memory_order_relaxed); }

  const std::string & device() const { return device_; }

private:
  void readerLoop();
  void consume(const uint8_t * data, size_t length);

  // Byte-wise frame state machine. Fixed-length payloads mean there is no length
  // field to trust, so the worst a corrupt byte can do is cost one frame.
  enum class ParseState : uint8_t
  {
    Sync0,
    Sync1,
    Type,
    Payload,
    Crc,
  };

  int fd_{-1};
  std::string device_;
  std::thread reader_;
  std::atomic<bool> running_{false};
  FrameHandler handler_;

  ParseState state_{ParseState::Sync0};
  uint8_t frame_type_{0};
  size_t expected_payload_{0};
  size_t payload_index_{0};
  uint8_t payload_[protocol::kMaxPayloadSize]{};

  std::atomic<uint64_t> bytes_received_{0};
  std::atomic<uint64_t> frames_received_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> resyncs_{0};
  std::atomic<uint64_t> write_drops_{0};
  std::atomic<uint64_t> read_errors_{0};
};

}  // namespace roombabot_firmware

#endif  // ROOMBABOT_FIRMWARE__SERIAL_LINK_HPP
