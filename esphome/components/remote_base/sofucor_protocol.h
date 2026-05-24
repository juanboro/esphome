#pragma once

#include "esphome/core/component.h"
#include "remote_base.h"

namespace esphome {
namespace remote_base {

struct SofucorData {
  uint16_t address;
  uint8_t command;

  bool operator==(const SofucorData &rhs) const { return address == rhs.address && command == rhs.command; }
};

class SofucorProtocol : public RemoteProtocol<SofucorData> {
 public:
  void encode(RemoteTransmitData *dst, const SofucorData &data) override;
  optional<SofucorData> decode(RemoteReceiveData src) override;
  void dump(const SofucorData &data) override;
};

DECLARE_REMOTE_PROTOCOL(Sofucor)

template<typename... Ts> class SofucorAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint16_t, address)
  TEMPLATABLE_VALUE(uint8_t, command)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    SofucorData data{};
    data.address = this->address_.value(x...);
    data.command = this->command_.value(x...);
    SofucorProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome
