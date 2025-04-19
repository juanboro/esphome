#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/tcp_server/tcp_server.h"

namespace esphome {
namespace girs {

static const char *CRLF = "\r\n";

class GirsComponent : public Component, public remote_base::RemoteReceiverListener {
 public:
  GirsComponent() {}

  void setup() override;
  void loop() override;
  void dump_config() override;

  void stream_in(const std::string &data);
  void stream_out(const std::string &data);
  void println(const std::string &data) {
    stream_out(data);
    stream_out(CRLF);
  }

  void do_command(const std::string &cmd);

  void set_tcp_server(tcp_server::TCPServerComponent *tcp) {
    tcp_ = tcp;
    tcp_->register_onread_callback(
        [this](std::string client_id, std::string data) { this->tcpreadcb(client_id, data); });
  }

  void set_can_rx(bool can_rx) { can_rx_ = can_rx; }
  void set_can_tx(bool can_tx) { can_tx_ = can_tx; }
  void set_transmitter(remote_base::RemoteTransmitterBase *transmitter) { this->transmitter_ = transmitter; }
  void set_beginTimeout(unsigned long beginTimeout) { beginTimeout_ = beginTimeout; }

  bool on_receive(remote_base::RemoteReceiveData data);

 protected:
  void tcpreadcb(std::string client_id, std::string data) { stream_in(data); }
  std::string cmd_buffer_;

  unsigned long beginTimeout_ = 10000;  // Time to wait for signal to start: milliseconds
  unsigned long nextTimeout_ = 500;     // Time to wait for next signal to start: milliseconds

  tcp_server::TCPServerComponent *tcp_{nullptr};
  remote_base::RemoteTransmitterBase *transmitter_{nullptr};

  bool can_rx_ = false;
  bool can_tx_ = false;

  uint32_t rx_start_ = 0;   // non-zero indicated millis() start of rx
  uint32_t rx_inprog_ = 0;  // non-zero indicated millis() for inprog rx between receives

 private:
  size_t parseNextValue_(const std::string &input, const size_t start, uint32_t &output);
  size_t transmit_data_(const std::string &cmd, const size_t start, const uint32_t frequency, const uint16_t Length,
                        const uint16_t noSends);
};
}  // namespace girs
}  // namespace esphome