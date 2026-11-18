#include "sdt/comms/codec.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstring>

namespace sdt::proto {
namespace {

constexpr std::array<std::uint32_t, 256> makeCrcTable() {
  std::array<std::uint32_t, 256> t{};
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint32_t c = i;
    for (int k = 0; k < 8; ++k) c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    t[i] = c;
  }
  return t;
}
constexpr auto kCrcTable = makeCrcTable();

class Writer {
 public:
  Writer(std::uint8_t* p, std::size_t cap) noexcept : p_(p), cap_(cap) {}
  void u8(std::uint8_t v) noexcept { put(&v, 1); }
  void u16(std::uint16_t v) noexcept {
    const std::uint8_t b[2] = {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8)};
    put(b, 2);
  }
  void u32(std::uint32_t v) noexcept {
    const std::uint8_t b[4] = {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8),
                               static_cast<std::uint8_t>(v >> 16), static_cast<std::uint8_t>(v >> 24)};
    put(b, 4);
  }
  void u64(std::uint64_t v) noexcept {
    u32(static_cast<std::uint32_t>(v));
    u32(static_cast<std::uint32_t>(v >> 32));
  }
  void f32(float v) noexcept { u32(std::bit_cast<std::uint32_t>(v)); }
  void f32(double v) noexcept { f32(static_cast<float>(v)); }
  bool ok() const noexcept { return ok_; }
  std::size_t size() const noexcept { return n_; }

 private:
  void put(const std::uint8_t* b, std::size_t k) noexcept {
    if (n_ + k > cap_) {
      ok_ = false;
      return;
    }
    std::memcpy(p_ + n_, b, k);
    n_ += k;
  }
  std::uint8_t* p_;
  std::size_t cap_;
  std::size_t n_{0};
  bool ok_{true};
};

class Reader {
 public:
  Reader(const std::uint8_t* p, std::size_t n) noexcept : p_(p), n_(n) {}
  std::uint8_t u8() noexcept { return take(1) ? p_[i_ - 1] : std::uint8_t{0}; }
  std::uint16_t u16() noexcept {
    if (!take(2)) return 0;
    return static_cast<std::uint16_t>(p_[i_ - 2] | (p_[i_ - 1] << 8));
  }
  std::uint32_t u32() noexcept {
    if (!take(4)) return 0;
    return static_cast<std::uint32_t>(p_[i_ - 4]) | (static_cast<std::uint32_t>(p_[i_ - 3]) << 8) |
           (static_cast<std::uint32_t>(p_[i_ - 2]) << 16) | (static_cast<std::uint32_t>(p_[i_ - 1]) << 24);
  }
  std::uint64_t u64() noexcept {
    const std::uint64_t lo = u32();
    return lo | (static_cast<std::uint64_t>(u32()) << 32);
  }
  float f32() noexcept {
    const float v = std::bit_cast<float>(u32());
    if (!std::isfinite(v)) ok_ = false;  // NaN / Inf never enter the system
    return v;
  }
  bool ok() const noexcept { return ok_; }
  bool done() const noexcept { return i_ == n_; }

 private:
  bool take(std::size_t k) noexcept {
    if (i_ + k > n_) {
      ok_ = false;
      return false;
    }
    i_ += k;
    return true;
  }
  const std::uint8_t* p_;
  std::size_t n_;
  std::size_t i_{0};
  bool ok_{true};
};

std::size_t frameCommand(MsgType t, std::uint16_t seq, const std::uint8_t* p, std::size_t n,
                         std::uint8_t* out, std::size_t cap) noexcept {
  return encodeFrame(t, seq, p, n, out, cap);
}

}  // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t n) noexcept {
  std::uint32_t c = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < n; ++i) c = kCrcTable[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

std::size_t encodeFrame(MsgType type, std::uint16_t seq, const std::uint8_t* payload,
                        std::size_t len, std::uint8_t* out, std::size_t cap) noexcept {
  if (len > kMaxPayload || cap < kHeaderSize + len + kCrcSize) return 0;
  Writer w(out, cap);
  w.u8(kMagic0);
  w.u8(kMagic1);
  w.u8(kVersion);
  w.u8(static_cast<std::uint8_t>(type));
  w.u16(seq);
  w.u16(static_cast<std::uint16_t>(len));
  if (len > 0) std::memcpy(out + kHeaderSize, payload, len);
  const std::uint32_t crc = crc32(out + 2, 6 + len);
  Writer c(out + kHeaderSize + len, kCrcSize);
  c.u32(crc);
  return kHeaderSize + len + kCrcSize;
}

std::size_t encodeCommand(const Command& cmd, std::uint16_t seq, std::uint8_t* out,
                          std::size_t cap) noexcept {
  static thread_local std::uint8_t pl[kMaxPayload];
  Writer w(pl, sizeof pl);
  MsgType type = MsgType::Heartbeat;
  switch (cmd.index()) {
    case 0: type = MsgType::Heartbeat; break;
    case 1:
      type = MsgType::Arm;
      w.u8(std::get<CmdArm>(cmd).arm ? 1 : 0);
      break;
    case 2:
      type = MsgType::SetMode;
      w.u8(static_cast<std::uint8_t>(std::get<CmdSetMode>(cmd).mode));
      break;
    case 3: {
      type = MsgType::ManualControl;
      const auto& m = std::get<CmdManual>(cmd);
      w.f32(m.forward);
      w.f32(m.lateral);
      w.f32(m.vertical);
      w.f32(m.yaw);
      break;
    }
    case 4: {
      type = MsgType::MissionUpload;
      const Mission& m = std::get<CmdMissionUpload>(cmd).mission;
      w.u32(m.id);
      w.u8(m.loop ? 1 : 0);
      w.u16(m.count);
      for (std::size_t i = 0; i < m.count && i < kMaxWaypoints; ++i) {
        const Waypoint& wp = m.waypoints[i];
        w.f32(wp.position.x);
        w.f32(wp.position.y);
        w.f32(wp.position.z);
        w.f32(wp.speed_mps);
        w.f32(wp.acceptance_m);
        w.f32(wp.hold_s);
      }
      break;
    }
    case 5:
      type = MsgType::MissionControl;
      w.u8(static_cast<std::uint8_t>(std::get<CmdMissionControl>(cmd).action));
      break;
    default: return 0;
  }
  if (!w.ok()) return 0;
  return frameCommand(type, seq, pl, w.size(), out, cap);
}

bool decodeCommand(MsgType type, const std::uint8_t* p, std::size_t len, Command& out) noexcept {
  Reader r(p, len);
  switch (type) {
    case MsgType::Heartbeat:
      if (len != 0) return false;
      out = CmdHeartbeat{};
      return true;
    case MsgType::Arm: {
      const std::uint8_t v = r.u8();
      if (!r.ok() || !r.done() || v > 1) return false;
      out = CmdArm{v == 1};
      return true;
    }
    case MsgType::SetMode: {
      const std::uint8_t v = r.u8();
      if (!r.ok() || !r.done() || v > static_cast<std::uint8_t>(ControlMode::ReturnHome)) return false;
      out = CmdSetMode{static_cast<ControlMode>(v)};
      return true;
    }
    case MsgType::ManualControl: {
      CmdManual m;
      m.forward = r.f32();
      m.lateral = r.f32();
      m.vertical = r.f32();
      m.yaw = r.f32();
      if (!r.ok() || !r.done()) return false;
      auto in_range = [](float v) { return v >= -1.0f && v <= 1.0f; };
      if (!in_range(m.forward) || !in_range(m.lateral) || !in_range(m.vertical) || !in_range(m.yaw)) return false;
      out = m;
      return true;
    }
    case MsgType::MissionUpload: {
      CmdMissionUpload up;
      Mission& m = up.mission;
      m.id = r.u32();
      const std::uint8_t loop = r.u8();
      m.count = r.u16();
      if (!r.ok() || loop > 1 || m.count == 0 || m.count > kMaxWaypoints) return false;
      m.loop = loop == 1;
      for (std::size_t i = 0; i < m.count; ++i) {
        Waypoint& wp = m.waypoints[i];
        wp.position.x = static_cast<double>(r.f32());
        wp.position.y = static_cast<double>(r.f32());
        wp.position.z = static_cast<double>(r.f32());
        wp.speed_mps = r.f32();
        wp.acceptance_m = r.f32();
        wp.hold_s = r.f32();
      }
      if (!r.ok() || !r.done()) return false;
      out = up;
      return true;
    }
    case MsgType::MissionControl: {
      const std::uint8_t v = r.u8();
      if (!r.ok() || !r.done() || v < 1 || v > 4) return false;
      out = CmdMissionControl{static_cast<MissionAction>(v)};
      return true;
    }
    default: return false;
  }
}

std::size_t encodeTelemetry(const Telemetry& t, std::uint16_t seq, std::uint8_t* out,
                            std::size_t cap) noexcept {
  std::uint8_t pl[64];
  Writer w(pl, sizeof pl);
  w.u64(static_cast<std::uint64_t>(t.stamp));
  w.f32(t.position.x);
  w.f32(t.position.y);
  w.f32(t.position.z);
  w.f32(t.velocity.x);
  w.f32(t.velocity.y);
  w.f32(t.velocity.z);
  w.f32(t.yaw);
  w.u8(t.armed ? 1 : 0);
  w.u8(static_cast<std::uint8_t>(t.mode));
  w.u8(static_cast<std::uint8_t>(t.safety));
  w.u8(static_cast<std::uint8_t>(t.mission_state));
  w.u16(t.mission_index);
  w.u32(t.faults);
  w.u16(t.tracks);
  w.u32(t.rejected_commands);
  if (!w.ok()) return 0;
  return encodeFrame(MsgType::Telemetry, seq, pl, w.size(), out, cap);
}

bool decodeTelemetry(const std::uint8_t* p, std::size_t len, Telemetry& t) noexcept {
  Reader r(p, len);
  t.stamp = static_cast<TimeNs>(r.u64());
  t.position = {static_cast<double>(r.f32()), static_cast<double>(r.f32()), static_cast<double>(r.f32())};
  t.velocity = {static_cast<double>(r.f32()), static_cast<double>(r.f32()), static_cast<double>(r.f32())};
  t.yaw = static_cast<double>(r.f32());
  t.armed = r.u8() == 1;
  t.mode = static_cast<ControlMode>(r.u8());
  t.safety = static_cast<SafetyState>(r.u8());
  t.mission_state = static_cast<MissionState>(r.u8());
  t.mission_index = r.u16();
  t.faults = r.u32();
  t.tracks = r.u16();
  t.rejected_commands = r.u32();
  return r.ok() && r.done();
}

void FrameDecoder::resetFrame() noexcept {
  st_ = St::Magic0;
  hdr_n_ = 0;
  got_ = 0;
  crc_n_ = 0;
}

bool FrameDecoder::push(std::uint8_t b, Frame& out) noexcept {
  switch (st_) {
    case St::Magic0:
      if (b == kMagic0) st_ = St::Magic1;
      return false;
    case St::Magic1:
      if (b == kMagic1) {
        st_ = St::Header;
        hdr_n_ = 0;
      } else {
        st_ = (b == kMagic0) ? St::Magic1 : St::Magic0;
      }
      return false;
    case St::Header:
      hdr_[hdr_n_++] = b;
      if (hdr_n_ < sizeof hdr_) return false;
      len_ = static_cast<std::size_t>(hdr_[4] | (hdr_[5] << 8));
      if (hdr_[0] != kVersion) {
        ++stats_.bad_version;
        resetFrame();
      } else if (len_ > kMaxPayload) {
        ++stats_.oversize;
        resetFrame();
      } else {
        got_ = 0;
        st_ = (len_ == 0) ? St::Crc : St::Payload;
        crc_n_ = 0;
      }
      return false;
    case St::Payload:
      buf_[got_++] = b;
      if (got_ == len_) {
        st_ = St::Crc;
        crc_n_ = 0;
      }
      return false;
    case St::Crc: {
      crc_[crc_n_++] = b;
      if (crc_n_ < kCrcSize) return false;
      std::uint8_t tmp[sizeof hdr_ + kMaxPayload];
      std::memcpy(tmp, hdr_, sizeof hdr_);
      if (len_ > 0) std::memcpy(tmp + sizeof hdr_, buf_, len_);
      const std::uint32_t want = crc32(tmp, sizeof hdr_ + len_);
      const std::uint32_t have = static_cast<std::uint32_t>(crc_[0]) | (static_cast<std::uint32_t>(crc_[1]) << 8) |
                                 (static_cast<std::uint32_t>(crc_[2]) << 16) | (static_cast<std::uint32_t>(crc_[3]) << 24);
      const bool ok = want == have;
      if (ok) {
        out.type = static_cast<MsgType>(hdr_[1]);
        out.seq = static_cast<std::uint16_t>(hdr_[2] | (hdr_[3] << 8));
        out.payload = buf_;
        out.length = len_;
        ++stats_.frames;
      } else {
        ++stats_.crc_errors;
      }
      resetFrame();
      return ok;
    }
  }
  return false;
}

}  // namespace sdt::proto
