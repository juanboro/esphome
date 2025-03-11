#pragma once
#include "esphome/core/defines.h"
#ifdef USE_NETWORK
#include "esphome/core/component.h"
#if defined(USE_SOCKET_IMPL_BSD_SOCKETS) || defined(USE_SOCKET_IMPL_LWIP_SOCKETS)
#include "esphome/components/socket/socket.h"
#else
#include "WiFiUdp.h"
#endif

namespace esphome {
namespace udp_broadcast {

class UDPBroadcastComponent : public Component {
 public:
  void add_address(const char *addr) { this->addresses_.emplace_back(addr); }
  void set_port(uint16_t port) { port_ = port; }
  void set_ttl(uint8_t ttl) { ttl_ = ttl; }
  void send_data(const char *data, size_t len);
  void dump_config() override;
  void setup() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

 protected:
#if defined(USE_SOCKET_IMPL_BSD_SOCKETS) || defined(USE_SOCKET_IMPL_LWIP_SOCKETS)
  std::unique_ptr<socket::Socket> broadcast_socket_{};
#else
  WiFiUDP udp_client_{};
#endif
  uint16_t port_{5007};
  uint8_t ttl_{64};
  std::vector<std::string> addresses_{};
};

}  // namespace udp_broadcast
}  // namespace esphome
#endif
