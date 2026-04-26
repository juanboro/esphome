#include "sofucor_protocol.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.sofucor";

// My Sofucor Fan remote - pretty much as easy as it gets:
// rtl_433 -r sofucorfan.ook -X 'n=SofucorFan,m=OOK_PPM,s=564,l=1684,reset=4516'
// 32 bit data returned.  First 2 bytes appear to be address
// Second 2 bytes appear to be a command - the last byte being the inverse of the previous.

static const uint32_t HEADER_HIGH_US = 4500;
static const uint32_t HEADER_LOW_US = 4500;
static const uint32_t BIT_HIGH_US = 560;
static const uint32_t BIT_ONE_LOW_US = 1680;
static const uint32_t BIT_ZERO_LOW_US = 560;

void SofucorProtocol::encode(RemoteTransmitData *dst, const SofucorData &data) {
  dst->reserve(4 + 32 * 2u);

  dst->item(HEADER_HIGH_US, HEADER_LOW_US);

  uint32_t sdata = (data.address << 16) | (data.command << 8) | (0xff ^ data.command);
  for (uint8_t bit = 32; bit > 0; bit--) {
    if ((sdata >> (bit - 1)) & 1) {
      dst->item(BIT_HIGH_US, BIT_ONE_LOW_US);
    } else {
      dst->item(BIT_HIGH_US, BIT_ZERO_LOW_US);
    }
  }
  dst->item(BIT_HIGH_US, HEADER_LOW_US * 2);
}
optional<SofucorData> SofucorProtocol::decode(RemoteReceiveData src) {
  if (src.size() < 66)
    return {};

  SofucorData out{
      .address = 0,
      .command = 0,
  };
  uint32_t data;

  if (src.peek() > 0)
    src.advance();

  bool retry = true;
  while (retry) {
    data = 0;
    retry = false;
    if (!src.expect_space(HEADER_LOW_US))
      return {};

    ESP_LOGD(TAG, "decode got space");

    for (uint8_t nbits = 0; nbits < 32; nbits++) {
      if (src.expect_item(BIT_HIGH_US, BIT_ONE_LOW_US)) {
        data = (data << 1) | 1;
      } else if (src.expect_item(BIT_HIGH_US, BIT_ZERO_LOW_US)) {
        data = (data << 1) | 0;
      } else {
        if (src.peek() > 0) {
          src.advance();
          retry = true;
        } else {
          ESP_LOGD(TAG, "failed on bit %d - value: %d", nbits, src.peek());
          return {};
        }
      }
    }
  }

  if (((data >> 8) ^ data) & 0xff == 0xff) {
    out.address = data >> 16;
    out.command = (data >> 8) & 0xff;
    return out;
  }

  ESP_LOGD(TAG, "data match failed: %x vs %x", (data >> 8) & 0xff, data & 0xff);

  return {};
}
void SofucorProtocol::dump(const SofucorData &data) {
  ESP_LOGI(TAG, "Received SofucorFan: address=0x%" PRIX16 ", command=0x%" PRIX8, data.address, data.command);
}

}  // namespace remote_base
}  // namespace esphome
