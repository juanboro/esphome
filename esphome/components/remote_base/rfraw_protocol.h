#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <algorithm>
#include <forward_list>
#include "remote_base.h"

using RFRawData = std::string;

#define PD_MAX_PULSES 1200  // Maximum number of pulses before forcing End Of Package

#define MAX_HIST_BINS 16

// Helper macros, collides with MSVC's stdlib.h unless NOMINMAX is used
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

namespace esphome {
namespace remote_base {

void rfraw_encode(RemoteTransmitData *dst, const RFRawData &data);

class RFRAWProtocol : public RemoteProtocol<RFRawData> {
 public:
  void encode(RemoteTransmitData *dst, const RFRawData &data) override;
  optional<RFRawData> decode(RemoteReceiveData src) override;
  void dump(const RFRawData &data) override;
};

DECLARE_REMOTE_PROTOCOL(RFRAW)

template<typename... Ts> class RFRawAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(std::string, data)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    RFRAWProtocol().encode(dst, x...);
    dst->set_carrier_frequency(this->carrier_frequency_.value(x...));
  }
};

}  // namespace remote_base
}  // namespace esphome
