#include "rfraw_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.rfraw";

/// simple histogram class adapted from rtl_433 c histogram
/// Histogram data for single bin
typedef struct {
  unsigned count;
  int sum;
  int mean;
  int min;
  int max;
} hist_bin_t;

// Do not use the remote_receiver tolerance here -- that is applied during matching
#define TOLERANCE (0.2f)  // 20% tolerance should still discern between the pulse widths: 0.33, 0.66, 1.0

class rfraw_histogram {
 public:
  rfraw_histogram(float tolerance) : tolerance(tolerance) {}

  void add_data(int data) {
    bool found = false;
    int _size = 0;
    for (auto &bin : bins) {
      ++_size;
      int bn = data;
      int bm = bin.mean;
      if (abs(bn - bm) < (tolerance * MAX(bn, bm))) {
        bin.count++;
        bin.sum += data;
        bin.mean = bin.sum / bin.count;
        bin.min = MIN(data, bin.min);
        bin.max = MAX(data, bin.max);
        found = true;
        break;  // Match found! Data added to existing bin
      }
    }
    // No match found? Add new bin
    if (!found && _size <= MAX_HIST_BINS) {
      size = _size + 1;
      bins.emplace_front(hist_bin_t{1, data, data, data, data});
    }
  }
  void sort_by_mean() {
    bins.sort([](const hist_bin_t &m, const hist_bin_t &n) { return m.mean < n.mean; });
  }

  int find_bin_index(int width) {
    int n = 0;
    for (auto &bin : bins) {
      if (bin.min <= width && width <= bin.max) {
        return n;
      }
      ++n;
    }
    return -1;
  }

  /// Fuse histogram bins with means within tolerance
  void fuse_bins(float tolerance = TOLERANCE) {
    std::forward_list<hist_bin_t>::iterator prev;

    for (std::forward_list<hist_bin_t>::iterator n = bins.begin(); n != bins.end(); ++n) {
      prev = n;
      // Compare all bins
      for (std::forward_list<hist_bin_t>::iterator m = std::next(n); m != bins.end();) {
        auto &bn = *n;
        auto &bm = *m;
        // if within tolerance
        if (abs(bn.mean - bm.mean) < (tolerance * MAX(bn.mean, bm.mean))) {
          // Fuse data for bin[n] and bin[m]
          bn.count += bm.count;
          bn.sum += bm.sum;
          bn.mean = bn.sum / bn.count;
          bn.min = MIN(bn.min, bm.min);
          bn.max = MAX(bn.max, bm.max);
          // Delete bin[m]
          bins.erase_after(prev);
          m = std::next(prev);
          size--;
        } else {
          prev = m;
          ++m;
        }
      }
    }
  }

  std::forward_list<hist_bin_t> bins;
  float tolerance;
  int size = 0;
};

#define HEXSTR_BUILDER_SIZE 1024
#define HEXSTR_MAX_COUNT 32

/// Hex string builder
using hexstr_t = std::vector<uint8_t>;

static inline void hexstr_push_word(hexstr_t &h, uint16_t v) {
  h.push_back(v >> 8);
  h.push_back(v & 0xff);
}

// simple iterating through raw data as cleaned up pulses (be more tolerant of bad input data)
class raw_in {
 public:
  raw_in(RawTimings &rawdata) : rawdata(rawdata) {}
  int begin() {
    lastgap = false;
    it = rawdata.begin();
    return next_pulse();
  }
  int next_pulse() {
    data = 0;
    while ((it != rawdata.end()) && (*it >= 0)) {
      data += *it;
      ++it;
    }
    return data;
  }
  int next_gap() {
    data = 0;
    while ((it != rawdata.end()) && (*it <= 0)) {
      if (it == rawdata.end() - 1)
        lastgap = true;  // mark last gap (end)
      data += -*it;
      ++it;
    }
    return data;
  }

  RawTimings &rawdata;
  RawTimings::iterator it;
  int data;
  bool lastgap;
};

/// Analyze the statistics of a pulse data structure and print result
optional<RFRAWData> RFRAWProtocol::decode(RemoteReceiveData src) {
  auto rawdata = src.get_raw_data();

  if (rawdata.size() == 0) {
    return {};
  }

  RFRAWData rfraw;

  rfraw.reserve(HEXSTR_BUILDER_SIZE);
  rfraw_histogram hist_gaps(TOLERANCE);
  rfraw_histogram hist_timings(TOLERANCE);

  // Generate statistics
  raw_in rawpulses(rawdata);

  int data = rawpulses.begin();
  while (data > 0) {
    hist_timings.add_data(data);
    data = rawpulses.next_gap();
    if (data > 0) {
      if (!rawpulses.lastgap)
        hist_gaps.add_data(data);  // Leave out last gap (end)
      hist_timings.add_data(data);
    }
    data = rawpulses.next_pulse();
  }

  // Fuse overlapping bins
  hist_gaps.fuse_bins();
  hist_timings.fuse_bins();

  hist_gaps.sort_by_mean();
  hist_timings.sort_by_mean();

  // Output RfRaw line (if possible)
  hexstr_t hexstr;

  if (hist_timings.size <= 8) {
    // if there is no 3rd gap length output one long B1 code
    if (hist_gaps.size <= 2) {
      hexstr.reserve(HEXSTR_BUILDER_SIZE);
      // ESP_LOGI("rfraw","begin b1 code");

      hexstr.push_back(0xaa);
      hexstr.push_back(0xb1);
      hexstr.push_back(hist_timings.size);

      for (auto &bin : hist_timings.bins)
        hexstr_push_word(hexstr, bin.mean < USHRT_MAX ? bin.mean : USHRT_MAX);

      int data = rawpulses.begin();
      while (data > 0) {
        int p = hist_timings.find_bin_index(data);
        data = rawpulses.next_gap();
        if (data <= 0)
          break;
        int g = hist_timings.find_bin_index(data);
        assert(!(p < 0 || g < 0));  // this can't happen
        hexstr.push_back(0x80 | (p << 4) | g);
        data = rawpulses.next_pulse();
      }
      hexstr.push_back(0x55);
      rfraw += format_hex(hexstr);
    }
    // otherwise try to group as B0 codes
    else {
      hexstr.reserve(HEXSTR_BUILDER_SIZE / HEXSTR_MAX_COUNT);
      // ESP_LOGI("rfraw","begin b0 code");
      //  pick last gap length but a most the 4th
      auto it = hist_gaps.bins.begin();
      std::advance(it, MIN(3, hist_gaps.size - 1));
      int limit = (*it).min;
      std::vector<hexstr_t> hexstrs;
      hexstrs.reserve(HEXSTR_MAX_COUNT);
      int data = rawpulses.begin();

      while (data > 0 && hexstrs.size() < HEXSTR_MAX_COUNT) {
        hexstr.push_back(0xaa);
        hexstr.push_back(0xb0);
        hexstr.push_back(0);  // len
        hexstr.push_back(hist_timings.size);
        hexstr.push_back(1);  // repeats
        for (auto &bin : hist_timings.bins)
          hexstr_push_word(hexstr, bin.mean < USHRT_MAX ? bin.mean : USHRT_MAX);

        while (data > 0) {
          int p = hist_timings.find_bin_index(data);
          data = rawpulses.next_gap();
          if (data <= 0)
            break;
          int g = hist_timings.find_bin_index(data);
          assert(!(p < 0 || g < 0));  // this can't happen
          hexstr.push_back(0x80 | (p << 4) | g);
          data = rawpulses.next_pulse();
          if (data > limit)
            break;
        }
        hexstr.push_back(0x55);
        hexstr[2] = hexstr.size() - 4 <= 255 ? hexstr.size() - 4 : 0;  // len
        if (hexstrs.size() > 0 && hexstrs.back().size() == hexstr.size() &&
            !memcmp(hexstrs.back().data() + 5, hexstr.data() + 5, hexstr.size() - 5)) {
          hexstrs.back()[4] += 1;  // repeats
        } else {
          hexstrs.push_back(hexstr);
        }
        hexstr.clear();
      }

      int j = 0;
      for (auto &hexstr : hexstrs) {
        if (j > 0)
          rfraw.push_back('+');
        ++j;
        rfraw += format_hex(hexstr);
      }
      if (j >= HEXSTR_MAX_COUNT) {
        fprintf(stderr, "Too many pulse groups (%u pulses missed in rfraw)\n", (rawdata.end() - rawpulses.it) / 2);
      }
    }
  }

  return rfraw;
}

static int hexstr_get_nibble(char const **p) {
  if (!p || !*p || !**p)
    return -1;
  while (**p == ' ' || **p == '\t' || **p == '-' || **p == ':')
    ++*p;

  int c = **p;
  if (c >= '0' && c <= '9') {
    ++*p;
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    ++*p;
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    ++*p;
    return c - 'a' + 10;
  }

  return -1;
}

static int hexstr_get_byte(char const **p) {
  int h = hexstr_get_nibble(p);
  int l = hexstr_get_nibble(p);
  if (h >= 0 && l >= 0)
    return (h << 4) | l;
  return -1;
}

static int hexstr_get_word(char const **p) {
  int h = hexstr_get_byte(p);
  int l = hexstr_get_byte(p);
  if (h >= 0 && l >= 0)
    return (h << 8) | l;
  return -1;
}

static int hexstr_peek_byte(char const *p) {
  int h = hexstr_get_nibble(&p);
  int l = hexstr_get_nibble(&p);
  if (h >= 0 && l >= 0)
    return (h << 4) | l;
  return -1;
}

static bool parse_rfraw(RemoteTransmitData *dst, char const **p) {
  if (!p || !*p || !**p)
    return false;

  int hdr = hexstr_get_byte(p);
  if (hdr != 0xaa)
    return false;

  int fmt = hexstr_get_byte(p);
  if (fmt != 0xb0 && fmt != 0xb1)
    return false;

  if (fmt == 0xb0) {
    hexstr_get_byte(p);  // ignore len
  }

  int bins_len = hexstr_get_byte(p);
  if (bins_len > 8)
    return false;

  int repeats = 1;
  if (fmt == 0xb0) {
    repeats = hexstr_get_byte(p);
  }

  int bins[8] = {0};
  for (int i = 0; i < bins_len; ++i) {
    bins[i] = hexstr_get_word(p);
  }

  // check if this is the old or new format
  bool oldfmt = true;
  char const *t = *p;
  while (*t) {
    int b = hexstr_get_byte(&t);
    if (b < 0 || b == 0x55) {
      break;
    }
    if (b & 0x88) {
      oldfmt = false;
      break;
    }
  }

  bool pulse_needed = true;
  bool aligned = true;
  while (*p) {
    if (aligned && hexstr_peek_byte(*p) == 0x55) {
      hexstr_get_byte(p);  // consume 0x55
      break;
    }

    int w = hexstr_get_nibble(p);
    aligned = !aligned;
    if (w < 0)
      return false;
    if (w >= 8 || (oldfmt && !aligned)) {  // pulse
      if (!pulse_needed) {
        dst->mark(0);
      }
      dst->mark(bins[w & 7]);
      pulse_needed = false;
    } else {  // gap
      if (pulse_needed) {
        dst->space(0);
      }
      dst->space(bins[w]);
      pulse_needed = true;
    }
  }
  return true;
}

void RFRAWProtocol::encode(RemoteTransmitData *dst, const RFRAWData &data) {
  const char *p = data.data();

  while (*p) {
    // skip whitespace and separators
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == '+' || *p == '-')
      ++p;

    if (!parse_rfraw(dst, &p))
      break;
  }
}

void RFRAWProtocol::dump(const RFRAWData &data) {
  std::string rest;

  rest = data;
  ESP_LOGI(TAG, "Received RFRAW: data=");
  while (true) {
    ESP_LOGI(TAG, "%s", rest.substr(0, 230).c_str());
    if (rest.size() > 230) {
      rest = rest.substr(230);
    } else {
      break;
    }
  }
}

}  // namespace remote_base
}  // namespace esphome
