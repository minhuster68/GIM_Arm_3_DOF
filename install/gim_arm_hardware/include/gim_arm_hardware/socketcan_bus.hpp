#pragma once
// Minimal non-blocking SocketCAN wrapper for a single CAN interface (e.g. "can0").
// Uses raw AF_CAN / SOCK_RAW sockets -- no external library needed, just the
// Linux kernel headers below (always available, no find_package()/linking needed).
// Đã compile + test edge case (bind fail, send/receive trên bus chưa mở) trong
// sandbox -- xem tin nhắn kèm theo để biết chi tiết đã test gì.

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace gim_arm_hardware
{

class SocketCanBus
{
public:
  SocketCanBus() = default;
  ~SocketCanBus() { close_bus(); }

  SocketCanBus(const SocketCanBus &) = delete;
  SocketCanBus & operator=(const SocketCanBus &) = delete;

  // Mở và bind 1 raw CAN socket vào `ifname` (vd "can0"). Trả false nếu lỗi.
  bool open_bus(const std::string & ifname)
  {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      close_bus();
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      close_bus();
      return false;
    }

    // Non-blocking: read() trong vòng lặp read() của ros2_control không được
    // phép chờ CAN -- phải trả về ngay dù chưa có frame nào tới.
    const int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
  }

  void close_bus()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const { return fd_ >= 0; }

  // Gửi 1 frame CAN chuẩn (11-bit ID). Trả false nếu lỗi (chưa mở, buffer đầy, ...).
  bool send(uint32_t can_id, const uint8_t * data, uint8_t dlc)
  {
    if (fd_ < 0) {
      return false;
    }
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id & CAN_SFF_MASK;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    const ssize_t n = write(fd_, &frame, sizeof(frame));
    return n == static_cast<ssize_t>(sizeof(frame));
  }

  // Rút hết các frame đang có sẵn vào `out`. Non-blocking: dừng ngay khi hết
  // frame để đọc (EAGAIN) -- an toàn để gọi ở mỗi chu kỳ read().
  void receive_all(std::vector<struct can_frame> & out)
  {
    if (fd_ < 0) {
      return;
    }
    struct can_frame frame;
    while (true) {
      const ssize_t n = read(fd_, &frame, sizeof(frame));
      if (n != static_cast<ssize_t>(sizeof(frame))) {
        break;
      }
      out.push_back(frame);
    }
  }

private:
  int fd_ = -1;
};

}  // namespace gim_arm_hardware