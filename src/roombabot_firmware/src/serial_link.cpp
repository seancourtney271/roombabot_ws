// cfmakeraw() and the ASYNC_LOW_LATENCY ioctl live behind glibc's __USE_MISC, which
// -std=c++17 (as opposed to -std=gnu++17) switches off. Define this before any system
// header so the build does not depend on which standard flag ament happens to pass.
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "roombabot_firmware/serial_link.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace roombabot_firmware
{

namespace
{

// termios speed_t constant for a plain integer baud rate. Over USB CDC the ESP32
// ignores this entirely (the "baud rate" is a fiction on a USB ACM endpoint), but it
// still has to be a value termios accepts, and it matters if the link is ever moved
// onto a real UART.
speed_t toSpeed(unsigned int baud_rate)
{
  switch (baud_rate)
  {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return B0;
  }
}

// FTDI and some CP210x adapters default to a 16 ms read latency timer, which would
// add up to 16 ms of jitter to a 10 ms control cycle. Ask the kernel to drop it to
// 1 ms. Native-USB ESP32s (S2/S3/C3 CDC) and most CH340s do not expose this, so a
// failure here is expected and ignored.
void tryLowLatency(int fd)
{
  struct serial_struct serial_info;
  if (ioctl(fd, TIOCGSERIAL, &serial_info) != 0)
  {
    return;
  }
  serial_info.flags |= ASYNC_LOW_LATENCY;
  (void)ioctl(fd, TIOCSSERIAL, &serial_info);
}

}  // namespace


SerialLink::~SerialLink()
{
  close();
}


bool SerialLink::open(const std::string & device, unsigned int baud_rate, std::string & error)
{
  if (fd_ >= 0)
  {
    error = "port " + device_ + " is already open";
    return false;
  }

  const speed_t speed = toSpeed(baud_rate);
  if (speed == B0)
  {
    error = "unsupported baud rate " + std::to_string(baud_rate);
    return false;
  }

  // O_NONBLOCK on open so a port with no carrier can't hang startup; the reader
  // thread uses poll() and the writer relies on the non-blocking behaviour, so it
  // stays set afterwards. O_NOCTTY keeps this process from adopting the port as its
  // controlling terminal.
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0)
  {
    error = "could not open " + device + ": " + std::strerror(errno);
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0)
  {
    error = "tcgetattr failed on " + device + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }

  // Fully raw: no canonical mode, no echo, no signal chars, no software flow control,
  // no CR/LF translation, no parity. Anything less and the kernel would silently eat
  // or rewrite bytes of a binary frame -- 0x11/0x13 (XON/XOFF) and 0x0D in particular.
  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem lines, enable the receiver
  tty.c_cflag &= ~CRTSCTS;           // no hardware flow control
  tty.c_cflag &= ~CSTOPB;            // one stop bit
  tty.c_cflag &= ~PARENB;            // no parity
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

  // VMIN=0/VTIME=0 -> read() returns whatever is buffered and never blocks. The
  // reader thread does its waiting in poll() instead, which also lets it notice a
  // stop request promptly.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    error = "tcsetattr failed on " + device + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }

  tryLowLatency(fd);

  // Drop anything the ESP32 sent before we were listening -- it is almost certainly a
  // partial frame or boot-loader chatter, and starting mid-frame just costs a resync.
  (void)tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  device_ = device;
  state_ = ParseState::Sync0;
  return true;
}


void SerialLink::start(FrameHandler handler)
{
  if (fd_ < 0 || running_.load(std::memory_order_acquire))
  {
    return;
  }

  handler_ = std::move(handler);
  running_.store(true, std::memory_order_release);
  reader_ = std::thread(&SerialLink::readerLoop, this);
}


void SerialLink::close()
{
  running_.store(false, std::memory_order_release);
  if (reader_.joinable())
  {
    reader_.join();
  }
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}


void SerialLink::readerLoop()
{
  // 512 bytes is ~19 of the largest frames; at 100 Hz the link produces well under
  // one frame per poll wakeup, so this only ever fills if the thread is starved.
  uint8_t buffer[512];

  while (running_.load(std::memory_order_acquire))
  {
    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    // 100 ms bounds how long close() can wait on this thread; it has no effect on
    // frame latency, since POLLIN fires as soon as bytes land.
    const int ready = ::poll(&pfd, 1, 100);
    if (ready < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      read_errors_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    if (ready == 0)
    {
      continue;  // timeout, just re-check running_
    }

    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
    {
      // The ESP32 was unplugged or reset. Count it and keep polling -- the owning
      // node decides what to do about a stale link via its own timeout.
      read_errors_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
    if (count > 0)
    {
      consume(buffer, static_cast<size_t>(count));
    }
    else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
      read_errors_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}


void SerialLink::consume(const uint8_t * data, size_t length)
{
  for (size_t i = 0; i < length; ++i)
  {
    const uint8_t byte = data[i];

    switch (state_)
    {
      case ParseState::Sync0:
        if (byte == protocol::kSync0)
        {
          state_ = ParseState::Sync1;
        }
        break;

      case ParseState::Sync1:
        if (byte == protocol::kSync1)
        {
          state_ = ParseState::Type;
        }
        else if (byte == protocol::kSync0)
        {
          // 0xA5 0xA5 -- the second one could still start a real frame.
          state_ = ParseState::Sync1;
        }
        else
        {
          state_ = ParseState::Sync0;
        }
        break;

      case ParseState::Type:
      {
        expected_payload_ = protocol::payloadSize(byte);
        if (expected_payload_ == 0)
        {
          // Unknown type: the sync pattern was a coincidence inside a payload.
          resyncs_.fetch_add(1, std::memory_order_relaxed);
          state_ = ParseState::Sync0;
          break;
        }
        frame_type_ = byte;
        payload_index_ = 0;
        state_ = ParseState::Payload;
        break;
      }

      case ParseState::Payload:
        payload_[payload_index_++] = byte;
        if (payload_index_ >= expected_payload_)
        {
          state_ = ParseState::Crc;
        }
        break;

      case ParseState::Crc:
      {
        const uint8_t expected = protocol::crc8(payload_, expected_payload_,
                                                protocol::crc8(&frame_type_, 1));
        if (byte == expected)
        {
          frames_received_.fetch_add(1, std::memory_order_relaxed);
          if (handler_)
          {
            handler_(frame_type_, payload_, expected_payload_);
          }
        }
        else
        {
          crc_errors_.fetch_add(1, std::memory_order_relaxed);
        }
        state_ = ParseState::Sync0;
        break;
      }
    }
  }
}


bool SerialLink::writeFrame(protocol::MsgType type, const void * payload, size_t length)
{
  if (fd_ < 0 || length > protocol::kMaxPayloadSize)
  {
    return false;
  }

  // Stack buffer, built and flushed in one syscall -- no heap traffic on the control
  // thread, and the ESP32 never sees a frame split across two USB transfers.
  uint8_t frame[protocol::kMaxFrameSize];
  const uint8_t type_byte = static_cast<uint8_t>(type);

  frame[0] = protocol::kSync0;
  frame[1] = protocol::kSync1;
  frame[2] = type_byte;
  std::memcpy(&frame[3], payload, length);
  frame[3 + length] = protocol::crc8(static_cast<const uint8_t *>(payload), length,
                                     protocol::crc8(&type_byte, 1));

  const size_t total = length + protocol::kFrameOverhead;
  const ssize_t written = ::write(fd_, frame, total);

  if (written == static_cast<ssize_t>(total))
  {
    return true;
  }

  // EAGAIN means the TX buffer is backed up. Dropping is the right call: the next
  // cycle's setpoint is 10 ms away and strictly more correct than this one. A short
  // write is treated the same way -- the frame is lost, but the parser on the other
  // end resyncs on the next sync pattern rather than acting on half a command.
  write_drops_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

}  // namespace roombabot_firmware
