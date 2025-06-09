/*

Partially based on: https://github.com/oxan/esphome-stream-server , Copyright (C) 2020-2022 Oxan van Leeuwen, License:
GPL-V3 and: https://github.com/tube0013/esphome-stream-server-v2
*/

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/uart/uart.h"

#include <string>
#include <vector>

#ifdef USE_ESP_IDF
#include "esphome/components/socket/socket.h"
#else
// https://registry.platformio.org/libraries/esphome/ESPAsyncTCP-esphome/examples/ClientServer/Server/Server.ino
#ifdef ESP8266
#include <ESPAsyncTCP.h>
#else
#include <AsyncTCP.h>
#endif
#endif

namespace esphome {
namespace tcp_server {

class TCPServerTrigger : public Trigger<const char *, const char *, size_t> {};
class TCPServerOnConnectTrigger : public Trigger<const char *> {};
class TCPServerOnDisconnectTrigger : public Trigger<const char *> {};
// mechanism for receiving messages directly to components...
using TCPServerReadCallBack = std::function<void(const char *, const char *, size_t)>;

class TCPServerBaseComponent : public Component {
 public:
  TCPServerBaseComponent() {}

  void setup();
  void loop();
  void dump_config();
  virtual void on_shutdown() {}

  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

  virtual void write(const std::string &data) {}
  virtual void write(const std::string &data, const char *client_id) {}
  virtual void write(const char *data, size_t size) {}
  virtual void write(const char *data, size_t size, const char *client_id) {}

  virtual void disconnect() {}
  virtual void disconnect(const char *client_id) {}

  void register_onmessage_trigger(TCPServerTrigger *trig) { this->triggers_onmsg_.push_back(trig); }
  void register_onconnect_trigger(TCPServerOnConnectTrigger *trig) { this->triggers_on_connect_.push_back(trig); }
  void register_ondisconnect_trigger(TCPServerOnDisconnectTrigger *trig) {
    this->triggers_on_disconnect_.push_back(trig);
  }
  void register_onread_callback(TCPServerReadCallBack readcb) { this->on_read_callbacks_.push_back(readcb); }

  void set_port(uint16_t port) { this->port_ = port; }
  virtual int get_client_count() { return 0; }
  void set_count_sensor(binary_sensor::BinarySensor *sensor);
  void set_uart_parent(uart::UARTComponent *uart) { this->uart_ = uart; }

 protected:
  uint16_t port_{9000};

  std::vector<TCPServerTrigger *> triggers_onmsg_;
  std::vector<TCPServerOnConnectTrigger *> triggers_on_connect_;
  std::vector<TCPServerOnDisconnectTrigger *> triggers_on_disconnect_;
  std::vector<TCPServerReadCallBack> on_read_callbacks_;

  binary_sensor::BinarySensor *count_sensor_ = nullptr;
  uart::UARTComponent *uart_ = nullptr;

  char buf_[128];
};

#ifdef USE_ESP_IDF
class TCPServerComponent : public TCPServerBaseComponent {
 public:
  TCPServerComponent() {}

  void setup() override;
  void loop() override;
  void on_shutdown() override;

  void write(const std::string &data);
  void write(const std::string &data, const char *client_id);
  void write(const char *data, size_t size);
  void write(const char *data, size_t size, const char *client_id);

  void disconnect();
  void disconnect(const char *client_id);

  int get_client_count() override { return this->clients_.size(); }

 protected:
  void accept();
  void cleanup();
  void read();

  struct Client {
    Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier);
    std::unique_ptr<esphome::socket::Socket> socket{nullptr};
    std::string identifier{};
    bool disconnected{false};
  };

  std::vector<Client> clients_{};
  std::unique_ptr<esphome::socket::Socket> socket_{};
};
#else
class TCPServerComponent : public TCPServerBaseComponent {
 public:
  TCPServerComponent() {}

  void setup() override;
  void loop() override;
  void on_shutdown() override;

  void write(const std::string &data);
  void write(const std::string &data, const char *client_id);
  void write(const char *data, size_t size);
  void write(const char *data, size_t size, const char *client_id);

  int get_client_count() override { return this->clients_.size(); }

  void disconnect();
  void disconnect(const char *client_id);

 protected:
  void cleanup();

  struct Client {
    Client(AsyncClient *client, std::string identifier);
    AsyncClient *client{nullptr};
    std::string identifier{};
    bool disconnected{false};
  };

  std::vector<Client> clients_{};

 private:
  void handleNewClient(AsyncClient *client);
  void handleError(AsyncClient *client, int8_t error);
  void handleData(AsyncClient *client, void *data, size_t len);
  void handleDisconnect(AsyncClient *client);
  void handleTimeout(AsyncClient *client, uint32_t time);

  AsyncServer *server_{};
};
#endif

}  // namespace tcp_server
}  // namespace esphome
