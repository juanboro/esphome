#include "girs.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

#include "esphome/components/network/util.h"
#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace girs {

static const char *TAG = "girs";

static const char *okString = "OK";
static const char *errorString = "ERROR";
static const char *timeoutString = ".";

void GirsComponent::setup() { ESP_LOGCONFIG(TAG, "Setting up girs server..."); }
void GirsComponent::loop() {
  std::size_t start = 0;
  std::size_t end;

  // check on rx in progress
  if (rx_start_ > 0) {
    if ((millis() - rx_start_) > beginTimeout_) {
      // rx timeout
      rx_start_ = 0;
      println(timeoutString);
    }
  } else if (rx_inprog_ > 0) {
    if ((millis() - rx_inprog_) > nextTimeout_) {
      rx_inprog_ = 0;
      println(timeoutString);
    }
  }

  // see if uart has anything for us
  if (this->uart_ != nullptr) {
    size_t available = this->uart_->available();
    if (available > 0) {
      size_t buf_pos = this->cmd_buffer_.size();
      this->cmd_buffer_.resize(buf_pos + available);
      this->uart_->read_array((uint8_t *) &this->cmd_buffer_[buf_pos], available);
    }
  }

  // Process any pending commands
  while ((end = this->cmd_buffer_.find_first_of(CRLF, start)) != std::string::npos) {
    this->do_command(this->cmd_buffer_.substr(start, end - start));
    start = end + 1;  // Move past the delimiter
  }
  this->cmd_buffer_ = this->cmd_buffer_.substr(start);
}

void GirsComponent::stream_out(const std::string &data) {
  ESP_LOGVV(TAG, "Girs stream_out (length=%d): %s", data.length(), data.c_str());
  if (tcp_ != nullptr) {
    tcp_->write(data);
  }
  if (uart_ != nullptr) {
    uart_->write_array((const uint8_t *) data.data(), data.size());
  }
}

void GirsComponent::stream_in(const std::string &data) {
  ESP_LOGD(TAG, "Girs stream_in (length=%d): %s", data.length(), data.c_str());
  this->cmd_buffer_ += data;
}

size_t GirsComponent::parseNextValue_(const std::string &input, const size_t start, uint32_t &output) {
  size_t end = start;
  output = 0;
  end = start;

  while ((end < input.size()) && (input[end] == ' '))
    ++end;

  while (end < input.size() && std::isdigit(input[end])) {
    output = output * 10 + (input[end] - '0');
    ++end;
  }

  return end;
}

size_t GirsComponent::transmit_data_(const std::string &cmd, const size_t start, const uint32_t frequency,
                                     const uint16_t Length, const uint16_t noSends) {
  uint32_t mark, space;
  size_t newstart = start;

  if (Length > 0) {
    auto call = this->transmitter_->transmit();
    call.set_send_times(noSends);
    auto *data = call.get_data();
    data->set_carrier_frequency(frequency);

    for (uint16_t i = 0; i < Length; i++) {
      newstart = this->parseNextValue_(cmd, newstart, mark);
      newstart = this->parseNextValue_(cmd, newstart, space);
      data->item(mark, space);
    }
    call.perform();
  }
  return newstart;
}

void GirsComponent::do_command(const std::string &cmd) {
  ESP_LOGD(TAG, "Girs command: %s", cmd.c_str());

  if (cmd.length() < 1) {
    // empty command, return ok
    println(okString);
    return;
  }

  char subcommand[16] = {0};
  if (sscanf(cmd.c_str(), "%15s", subcommand) != 1) {
    println(errorString);
    return;
  }

  if (subcommand[0] == 'm') {  // modules
    stream_out("base");
    if (can_rx_)
      stream_out(" receive");
    if (can_tx_)
      stream_out(" transmit");
    stream_out("\r\n");
  } else if (subcommand[0] == 'v') {  // version
    println("ESPHOME GIRS COMPONENT: 0.0.1");
  } else if ((can_tx_) && (subcommand[0] == 's')) {  // send (transmit)
    // no_sends, frequency, intro_length, repeat_length, end_length
    uint32_t noSends;
    uint32_t frequency;
    uint32_t introLength;
    uint32_t repeatLength;
    uint32_t endingLength;

    size_t start = cmd.find(' ') + 1;  // start after send subcommand

    start = this->parseNextValue_(cmd, start, noSends);
    start = this->parseNextValue_(cmd, start, frequency);
    start = this->parseNextValue_(cmd, start, introLength);
    start = this->parseNextValue_(cmd, start, repeatLength);
    start = this->parseNextValue_(cmd, start, endingLength);

    // uses this convention:
    // https://github.com/bengtmartensson/Infrared4Arduino/blob/1cc95fef1e4955c12805e19b548730fbf3a2413b/src/IrSender.cpp#L30
    start = this->transmit_data_(cmd, start, frequency, introLength, 1);         // transmit intro
    start = this->transmit_data_(cmd, start, frequency, repeatLength, noSends);  // transmit repeat
    start = this->transmit_data_(cmd, start, frequency, endingLength, 1);        // transmit ending

    println(okString);
  } else if ((can_rx_) && (strncmp(subcommand, "receive", 7) == 0)) {  // receive
    rx_start_ = millis();  // send received data if within the timeout window
  } else {
    println(errorString);
  }
}

bool GirsComponent::on_receive(remote_base::RemoteReceiveData data) {
  if (data.size() < 4)
    return false;  // reject anything less than 2 mark/space as glitchy garbage

  if ((rx_start_ > 0) || (rx_inprog_ > 0)) {
    if (rx_start_ > 0)
      rx_start_ = 0;
    rx_inprog_ = millis();

    char buffer[148];
    char *bufp = nullptr;
    int32_t last = -1;

    // we are receiving - so send the raw data
    for (auto dp : data.get_raw_data()) {
      if (bufp == nullptr) {
        // first
        bufp = buffer;
      } else {
        *bufp = ' ';
        ++bufp;
      }
      if ((last * dp) > 0) {
        // should be alternating mark and space - so insert opposite of last
        if (last < 0) {
          *bufp = '+';
          ++bufp;
        }
        bufp += std::sprintf(bufp, "%d ", -last);
      }
      last = dp;

      if (dp > 0) {
        *bufp = '+';
        ++bufp;
      }
      bufp += std::sprintf(bufp, "%d", dp);
      if ((bufp - buffer) > 128) {
        stream_out(std::string(buffer, bufp - buffer));
        bufp = buffer;
      }
    }

    if (last > 0)
      bufp += std::sprintf(bufp, " %d", -last);

    if ((bufp - buffer) > 0)
      stream_out(std::string(buffer, bufp - buffer));

    stream_out(CRLF);
    return true;
  }

  return false;
}

void GirsComponent::dump_config() { ESP_LOGCONFIG(TAG, "Girs Server:"); }

}  // namespace girs
}  // namespace esphome