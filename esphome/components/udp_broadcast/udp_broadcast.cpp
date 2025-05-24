#include "udp_broadcast.h"
#ifdef USE_NETWORK
#include "esphome/core/log.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace udp_broadcast {

static const char *const TAG = "udp_broadcast";

void UDPBroadcastComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  for (const auto &address : this->addresses_)
    ESP_LOGCONFIG(TAG, "  Address: %s", address.c_str());
}

void UDPBroadcastComponent::send_data(const char *data, size_t len) {
  if (!network::is_connected()) {
    ESP_LOGW(TAG, "Network not connected");
    return;
  }
  // ESP_LOGI(TAG, "Sending multicast Packet...");
#if defined(USE_SOCKET_IMPL_BSD_SOCKETS) || defined(USE_SOCKET_IMPL_LWIP_SOCKETS)
  struct sockaddr_storage saddr {};
  for (auto addr : this->addresses_) {
    auto addr_len = socket::set_sockaddr(reinterpret_cast<sockaddr *>(&saddr), sizeof(saddr), addr, this->port_);
    if (this->broadcast_socket_->sendto(data, len, 0, reinterpret_cast<const sockaddr *>(&saddr), addr_len) <= 0)
      ESP_LOGW(TAG, "sendto() error %d", errno);
  }
#else
  IPAddress broadcast = IPAddress();
  for (auto ip : esphome::network::get_ip_addresses()) {
    if (ip.is_ip4()) {
      for (auto addr : this->addresses_) {
        broadcast.fromString(addr);
        // see also: https://gist.github.com/mqu/f8d4f6877703bb39676d68733801871d
        if (this->udp_client_.beginPacketMulticast(broadcast, this->port_, ip, this->ttl_) != 0) {
          this->udp_client_.write(data, len);
          if (this->udp_client_.endPacket() != 0)
            continue;
          ESP_LOGW(TAG, "send broadcast failed");
        }
      }
    }
  }
#endif
}

void UDPBroadcastComponent::setup() {
#if defined(USE_SOCKET_IMPL_BSD_SOCKETS) || defined(USE_SOCKET_IMPL_LWIP_SOCKETS)
  // where is ttl set?  if necessary dig into:
  // https://stackoverflow.com/questions/31066061/setting-ttl-on-outgoing-udp-packets
  this->broadcast_socket_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (this->broadcast_socket_ == nullptr) {
    this->mark_failed();
    this->status_set_error("Could not create socket");
    return;
  }
  int enable = 1;
  auto err = this->broadcast_socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  if (err != 0) {
    this->status_set_warning("Socket unable to set reuseaddr");
    // we can still continue
  }
  err = this->broadcast_socket_->setsockopt(SOL_SOCKET, SO_BROADCAST, &enable, sizeof(int));
  if (err != 0) {
    this->status_set_warning("Socket unable to set broadcast");
  }
#endif
}

}  // namespace udp_broadcast
}  // namespace esphome
#endif
