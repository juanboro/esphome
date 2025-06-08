/*

Heavily based on: https://github.com/oxan/esphome-stream-server , Copyright (C) 2020-2022 Oxan van Leeuwen, License:
GPL-V3 and: https://github.com/tube0013/esphome-stream-server-v2
*/

#include "tcp_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace tcp_server {

static const char *TAG = "tcpserver";

void TCPServerBaseComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up tcp server...");
  if (this->uart_ != nullptr) {
    // connect uart if requested
    this->register_onread_callback([this](std::string client_id, std::string data) {
      this->uart_->write_array((const uint8_t *) data.c_str(), data.size());
    });
  }
}

void TCPServerBaseComponent::loop() {
  if (this->uart_ != nullptr) {
    size_t available = this->uart_->available();
    if (available > 0) {
      size_t len = std::min<size_t>(available, sizeof(this->buf_));
      this->uart_->read_array((uint8_t *) this->buf_, len);
      this->write(this->buf_, len);
    }
  }
}

void TCPServerBaseComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TCP Server:");
  std::string ip_str = "";
  for (auto &ip : network::get_ip_addresses()) {
    if (ip.is_set())
      ip_str += " " + ip.str();
  }
  ESP_LOGCONFIG(TAG, "  Address:%s", ip_str.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
}

void TCPServerBaseComponent::set_count_sensor(binary_sensor::BinarySensor *sensor) { this->count_sensor_ = sensor; }

#ifdef USE_ESP_IDF
void TCPServerComponent::setup() {
  TCPServerBaseComponent::setup();

  this->socket_ = socket::socket_ip_loop_monitored(SOCK_STREAM, 0);  // monitored for incoming connections
  if (this->socket_ == nullptr) {
    ESP_LOGW(TAG, "Could not create socket");
    this->mark_failed();
    return;
  }
  int enable = 1;
  int err = this->socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  if (err != 0) {
    ESP_LOGW(TAG, "Socket unable to set reuseaddr: errno %d", err);
    // we can still continue
  }
  err = this->socket_->setblocking(false);
  if (err != 0) {
    ESP_LOGW(TAG, "Socket unable to set nonblocking mode: errno %d", err);
    this->mark_failed();
    return;
  }

  struct sockaddr_storage server;

  socklen_t sl = socket::set_sockaddr_any((struct sockaddr *) &server, sizeof(server), this->port_);
  if (sl == 0) {
    ESP_LOGW(TAG, "Socket unable to set sockaddr: errno %d", errno);
    this->mark_failed();
    return;
  }

  err = this->socket_->bind((struct sockaddr *) &server, sl);
  if (err != 0) {
    ESP_LOGW(TAG, "Socket unable to bind: errno %d", errno);
    this->mark_failed();
    return;
  }

  err = this->socket_->listen(4);
  if (err != 0) {
    ESP_LOGW(TAG, "Socket unable to listen: errno %d", errno);
    this->mark_failed();
    return;
  }
}

void TCPServerComponent::read() {
  ssize_t len;
  for (Client &client : this->clients_) {
    while ((len = client.socket->read(&(this->buf_), sizeof(this->buf_))) > 0) {
      for (auto *trigger : this->triggers_onmsg_) {
        trigger->trigger(client.identifier, std::string(this->buf_, len));
      }
      for (auto tcpreadcb : this->on_read_callbacks_) {
        tcpreadcb(client.identifier, std::string(this->buf_, len));
      }
    }
    if (len == 0) {
      ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
      client.disconnected = true;

      for (auto *trigger : this->triggers_on_disconnect_) {
        trigger->trigger(client.identifier);
      }

      continue;
    }
  }
}

void TCPServerComponent::accept() {
  if (this->socket_->ready()) {
    while (true) {
      struct sockaddr_storage source_addr;
      socklen_t addr_len = sizeof(source_addr);
      auto socket = this->socket_->accept_loop_monitored((struct sockaddr *) &source_addr, &addr_len);
      if (!socket)
        break;

      socket->setblocking(false);

      std::string identifier = socket->getpeername();
      this->clients_.emplace_back(std::move(socket), identifier);
      ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());

      for (auto *trigger : this->triggers_on_connect_) {
        trigger->trigger(identifier);
      }
    }
  }
}

void TCPServerComponent::cleanup() {
  auto discriminator = [](const Client &client) { return !client.disconnected; };
  auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
  this->clients_.erase(last_client, this->clients_.end());
}

void TCPServerComponent::loop() {
  TCPServerBaseComponent::loop();

  this->accept();
  this->read();
  this->cleanup();

  if (this->count_sensor_ != nullptr) {
    this->count_sensor_->publish_state(get_client_count() > 0);
  }
}

void TCPServerComponent::write(const std::string &data) {
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;
    client.socket->write(data.c_str(), data.size());
  }
}

void TCPServerComponent::write(const char *data, size_t size) {
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;
    client.socket->write(data, size);
  }
}

void TCPServerComponent::write(const std::string &data, const std::string &client_id) {
  for (Client &client : this->clients_) {
    if ((client.disconnected) || (client.identifier.compare(client_id) != 0))
      continue;
    client.socket->write(data.c_str(), data.size());
  }
}
void TCPServerComponent::write(const char *data, size_t size, const std::string &client_id) {
  for (Client &client : this->clients_) {
    if ((client.disconnected) || (client.identifier.compare(client_id) != 0))
      continue;
    client.socket->write(data, size);
  }
}

void TCPServerComponent::disconnect(const std::string &client_id) {
  for (Client &client : this->clients_) {
    if ((client.disconnected) || (client.identifier.compare(client_id) != 0))
      continue;
    client.socket->shutdown(SHUT_RDWR);
    client.disconnected = true;
  }
}

void TCPServerComponent::disconnect() {
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;
    client.socket->shutdown(SHUT_RDWR);
    client.disconnected = true;
  }
}

void TCPServerComponent::on_shutdown() {
  for (const Client &client : this->clients_)
    client.socket->shutdown(SHUT_RDWR);
}

TCPServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier)
    : socket(std::move(socket)), identifier{identifier} {}

#else
void TCPServerComponent::setup() {
  TCPServerBaseComponent::setup();

  this->server_ = new AsyncServer(this->port_);
  this->server_->onClient([this](void *arg, AsyncClient *client) { this->handleNewClient(client); }, this->server_);
  this->server_->begin();
}

void TCPServerComponent::handleNewClient(AsyncClient *client) {
  std::string identifier = std::string(client->remoteIP().toString().c_str());
  ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());

  this->clients_.emplace_back(client, identifier);

  client->onData(
      [this](void *arg, AsyncClient *client, void *data, size_t len) { this->handleData(client, data, len); });
  client->onError([this](void *arg, AsyncClient *client, int8_t error) { this->handleError(client, error); });
  client->onDisconnect([this](void *arg, AsyncClient *client) { this->handleDisconnect(client); });
  client->onTimeout([this](void *arg, AsyncClient *client, uint32_t time) { this->handleTimeout(client, time); });

  for (auto *trigger : this->triggers_on_connect_) {
    trigger->trigger(identifier);
  }
}

void TCPServerComponent::handleData(AsyncClient *client, void *data, size_t len) {
  std::string received_data(static_cast<char *>(data), len);
  std::string identifier = std::string(client->remoteIP().toString().c_str());
  ESP_LOGD(TAG, "Received data from client %s: %s", identifier.c_str(), received_data.c_str());
  for (auto *trigger : this->triggers_onmsg_) {
    trigger->trigger(identifier, received_data);
  }
  for (auto tcpreadcb : this->on_read_callbacks_) {
    tcpreadcb(identifier, received_data);
  }
}

void TCPServerComponent::handleError(AsyncClient *client, int8_t error) {
  ESP_LOGW(TAG, "Client %s encountered error: %d", client->remoteIP().toString().c_str(), error);
  client->close();
}

void TCPServerComponent::handleDisconnect(AsyncClient *client) {
  auto it = std::find_if(this->clients_.begin(), this->clients_.end(),
                         [client](const Client &c) { return c.client == client; });
  if (it != this->clients_.end()) {
    it->disconnected = true;
    ESP_LOGD(TAG, "Client %s disconnected", it->identifier.c_str());
    for (auto *trigger : this->triggers_on_disconnect_) {
      trigger->trigger(it->identifier);
    }
  }

  client->close();
}

void TCPServerComponent::handleTimeout(AsyncClient *client, uint32_t time) {
  ESP_LOGW(TAG, "Client %s timed out after %u ms", client->remoteIP().toString().c_str(), time);
  client->close();
}

void TCPServerComponent::loop() {
  TCPServerBaseComponent::loop();

  if (this->count_sensor_ != nullptr) {
    this->count_sensor_->publish_state(get_client_count() > 0);
  }

  cleanup();  // cleanup disconnected clients
}

void TCPServerComponent::write(const std::string &data) {
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;
    client.client->write(data.c_str(), data.size());
  }
}

void TCPServerComponent::write(const char *data, size_t size) {
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;
    client.client->write(data, size);
  }
}

void TCPServerComponent::write(const std::string &data, const std::string &client_id) {
  for (Client &client : this->clients_) {
    if ((client.disconnected) || (client.identifier.compare(client_id) != 0))
      continue;
    client.client->write(data.c_str(), data.size());
  }
}

void TCPServerComponent::write(const char *data, size_t size, const std::string &client_id) {
  for (Client &client : this->clients_) {
    if ((client.disconnected) || (client.identifier.compare(client_id) != 0))
      continue;
    client.client->write(data, size);
  }
}

void TCPServerComponent::disconnect(const std::string &client_id) {
  for (Client &client : this->clients_) {
    if ((client.disconnected) || (client.identifier.compare(client_id) != 0))
      continue;
    client.client->stop();
    client.disconnected = true;
  }
}

void TCPServerComponent::disconnect() {
  for (Client &client : this->clients_) {
    if (client.disconnected)
      continue;
    client.client->stop();
    client.disconnected = true;
  }
}

void TCPServerComponent::cleanup() {
  auto discriminator = [](const Client &client) { return !client.disconnected; };
  auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
  this->clients_.erase(last_client, this->clients_.end());
}

void TCPServerComponent::on_shutdown() {
  for (const Client &client : this->clients_)
    client.client->stop();
}

TCPServerComponent::Client::Client(AsyncClient *client, std::string identifier)
    : client(std::move(client)), identifier{identifier} {}

#endif  // everything but esp-idf gets async tcp
}  // namespace tcp_server
}  // namespace esphome
