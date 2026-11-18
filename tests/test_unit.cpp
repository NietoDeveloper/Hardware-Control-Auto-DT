#include <cstring>
#include <numbers>
#include <thread>

#include "sdt/common/spsc_queue.hpp"
#include "sdt/comms/codec.hpp"
#include "sdt/avoidance/collision_avoidance.hpp"
#include "sdt/control/pid.hpp"
#include "sdt/media/media_router.hpp"
#include "sdt/nav/mission_executor.hpp"
#include "sdt/perception/radar_tracker.hpp"
#include "sdt/safety/command_guard.hpp"
#include "sdt/safety/safety_supervisor.hpp"
#include "test_framework.hpp"

using namespace sdt;

// ---- lock-free queue ------------------------------------------------------------------------
SDT_TEST(spsc_queue_orders_and_bounds) {
  SpscQueue<int, 4> q;
  for (int i = 0; i < 4; ++i) CHECK(q.push(i));
  CHECK(!q.push(99));  // full
  int v = -1;
  for (int i = 0; i < 4; ++i) {
    CHECK(q.pop(v));
    CHECK(v == i);
  }
  CHECK(!q.pop(v));
}

SDT_TEST(spsc_queue_two_threads_no_loss) {
  SpscQueue<int, 64> q;
  constexpr int kN = 200000;
  long long sum = 0;
  std::thread prod([&] {
    for (int i = 1; i <= kN; ++i) {
      while (!q.push(i)) std::this_thread::yield();
    }
  });
  int got = 0;
  int v = 0;
  while (got < kN) {
    if (q.pop(v)) {
      sum += v;
      ++got;
    }
  }
  prod.join();
  CHECK(sum == static_cast<long long>(kN) * (kN + 1) / 2);
}

// ---- wire protocol --------------------------------------------------------------------------
SDT_TEST(codec_roundtrip_all_commands) {
  std::uint8_t buf[proto::kMaxFrame];
  proto::FrameDecoder dec;
  Mission m;
  m.id = 7;
  m.count = 2;
  m.loop = true;
  m.waypoints[0] = {{1, 2, 3}, 2.5f, 1.0f, 0.5f};
  m.waypoints[1] = {{-4, 5, 6}, 3.5f, 2.0f, 0.0f};
  const Command cmds[] = {CmdHeartbeat{}, CmdArm{true}, CmdSetMode{ControlMode::Mission},
                          CmdManual{0.5f, -0.25f, 0.f, 1.f}, CmdMissionUpload{m},
                          CmdMissionControl{MissionAction::Pause}};
  int seen = 0;
  for (const Command& c : cmds) {
    const std::size_t n = proto::encodeCommand(c, 1, buf, sizeof buf);
    CHECK(n > 0);
    dec.feed(buf, n, [&](const proto::Frame& f) {
      Command out;
      CHECK(proto::decodeCommand(f.type, f.payload, f.length, out));
      CHECK(out.index() == c.index());
      ++seen;
    });
  }
  CHECK(seen == 6);
}

SDT_TEST(codec_rejects_corruption_and_resyncs) {
  std::uint8_t buf[proto::kMaxFrame];
  const std::size_t n = proto::encodeCommand(CmdArm{true}, 1, buf, sizeof buf);
  proto::FrameDecoder dec;
  int frames = 0;
  auto count = [&](const proto::Frame&) { ++frames; };

  std::uint8_t bad[proto::kMaxFrame];
  std::memcpy(bad, buf, n);
  bad[proto::kHeaderSize] ^= 0x01;  // flip a payload bit
  dec.feed(bad, n, count);
  CHECK(frames == 0);
  CHECK(dec.stats().crc_errors == 1);

  const std::uint8_t garbage[] = {0xD7, 0xD7, 0x00, 0xFF, 0x7A, 0xD7};
  dec.feed(garbage, sizeof garbage, count);
  dec.feed(buf, n, count);  // good frame after garbage must still parse
  CHECK(frames == 1);
}

SDT_TEST(codec_rejects_nan_and_out_of_range) {
  std::uint8_t pl[16];
  const float nan = std::nanf("");
  std::memcpy(pl, &nan, 4);
  std::memset(pl + 4, 0, 12);
  Command out;
  CHECK(!proto::decodeCommand(proto::MsgType::ManualControl, pl, 16, out));
  const float big = 5.0f;
  std::memcpy(pl, &big, 4);
  CHECK(!proto::decodeCommand(proto::MsgType::ManualControl, pl, 16, out));
  const std::uint8_t bad_mode = 9;
  CHECK(!proto::decodeCommand(proto::MsgType::SetMode, &bad_mode, 1, out));
}

SDT_TEST(codec_telemetry_roundtrip) {
  Telemetry t;
  t.stamp = 123456789;
  t.position = {1.5, -2.5, 3.0};
  t.armed = true;
  t.mode = ControlMode::Mission;
  t.faults = 5;
  t.tracks = 3;
  std::uint8_t buf[proto::kMaxFrame];
  const std::size_t n = proto::encodeTelemetry(t, 9, buf, sizeof buf);
  proto::FrameDecoder dec;
  bool got = false;
  dec.feed(buf, n, [&](const proto::Frame& f) {
    Telemetry r;
    CHECK(proto::decodeTelemetry(f.payload, f.length, r));
    CHECK(r.stamp == t.stamp && r.armed && r.faults == 5 && r.tracks == 3);
    CHECK_NEAR(r.position.y, -2.5, 1e-6);
    got = true;
  });
  CHECK(got);
}

// ---- control primitives ---------------------------------------------------------------------
SDT_TEST(pid_saturates_and_does_not_wind_up) {
  Pid pid({1.0, 5.0, 0.0, -1.0, 1.0, 10.0});
  for (int i = 0; i < 1000; ++i) CHECK_NEAR(pid.update(10.0, 0.01), 1.0, 1e-9);
  // After a long saturation the integrator must not keep the output pinned once error flips.
  double out = 0;
  for (int i = 0; i < 30; ++i) out = pid.update(-1.0, 0.01);
  CHECK(out < 0.5);
}

SDT_TEST(command_guard_limits_speed_and_slew) {
  CommandGuard g({10.0, 3.0, 3.0, 5.0, 1.5, 2.0});
  VehicleState s;
  VelocityCommand in;
  in.velocity = {100.0, 0.0, 50.0};
  in.yaw_rate = 9.0;
  auto out = g.filter(in, s, 5.0, nullptr, 0.1);
  CHECK(out.velocity.norm() <= 0.3 + 1e-9);  // 3 m/s^2 * 0.1 s from rest
  CHECK_NEAR(out.yaw_rate, 1.5, 1e-9);
  for (int i = 0; i < 200; ++i) out = g.filter(in, s, 5.0, nullptr, 0.1);
  CHECK_NEAR(out.velocity.norm2d(), 5.0, 1e-6);  // speed limit wins
  CHECK_NEAR(out.velocity.z, 3.0, 1e-6);         // vertical clamp
}

SDT_TEST(geofence_margin_and_barrier) {
  Geofence f;
  f.enabled = true;
  f.vertex_count = 4;
  f.vertices[0] = {0, 0, 0};
  f.vertices[1] = {100, 0, 0};
  f.vertices[2] = {100, 100, 0};
  f.vertices[3] = {0, 100, 0};
  CHECK_NEAR(f.margin({50, 50, 0}), 50.0, 1e-9);
  CHECK(f.margin({-5, 50, 0}) < 0.0);
  CommandGuard g;
  VehicleState s;
  s.position = {97, 50, 0};  // braking point (5 m/s -> ~4.5 m) lies outside the fence
  s.velocity = {5, 0, 0};
  VelocityCommand in;
  in.velocity = {5, 0, 0};
  VelocityCommand out;
  for (int i = 0; i < 50; ++i) out = g.filter(in, s, 8.0, &f, 0.1);
  CHECK(out.velocity.norm() < 1e-6);  // refuses to drive out of the fence
  in.velocity = {-5, 0, 0};           // driving away from the edge is fine
  for (int i = 0; i < 50; ++i) out = g.filter(in, s, 8.0, &f, 0.1);
  CHECK(out.velocity.x < -4.0);
}

// ---- perception & avoidance -----------------------------------------------------------------
namespace {
RadarScan scanOfPoint(double range, double az, TimeNs stamp) {
  RadarScan s;
  s.stamp = stamp;
  s.count = 1;
  s.detections[0] = {static_cast<float>(range), static_cast<float>(az), 0.f, 0.f, 20.f};
  return s;
}
}  // namespace

SDT_TEST(tracker_confirms_and_estimates_velocity) {
  RadarTracker tr;
  VehicleState own;
  for (int i = 0; i < 60; ++i) {  // target at x = 30 + 1.0*t (receding), scans at 20 Hz
    const double t = i * 0.05;
    tr.update(scanOfPoint(30.0 + 1.0 * t, 0.0, fromSeconds(t)), own);
  }
  ObstacleList list;
  CHECK(tr.confirmed(list) == 1);
  CHECK_NEAR(list[0].velocity.x, 1.0, 0.15);
  CHECK_NEAR(list[0].position.x, 33.0, 0.3);
}

SDT_TEST(tracker_drops_stale_tracks_and_rejects_nan) {
  RadarTracker tr;
  VehicleState own;
  for (int i = 0; i < 5; ++i) tr.update(scanOfPoint(20.0, 0.0, millis(50 * i)), own);
  RadarScan empty;
  for (int i = 5; i < 20; ++i) {
    empty.stamp = millis(50 * i);
    tr.update(empty, own);
  }
  CHECK(tr.activeCount() == 0);
  RadarScan nan_scan = scanOfPoint(std::nan(""), 0.0, millis(2000));
  tr.update(nan_scan, own);
  CHECK(tr.activeCount() == 0);
}

SDT_TEST(avoidance_passes_through_when_clear_and_deflects_when_blocked) {
  CollisionAvoidance ca;
  VehicleState own;
  Obstacle o;
  o.position = {15.0, 0.0, 0.0};
  o.radius_m = 1.0;
  const Vec3 desired{4.0, 0.0, 0.0};

  auto clear = ca.resolve(own, desired, nullptr, 0, 8.0);
  CHECK(!clear.modified);

  auto r = ca.resolve(own, desired, &o, 1, 8.0);
  CHECK(r.modified);
  CHECK(!r.no_safe_option);
  CHECK(std::fabs(r.velocity.y) > 0.3);  // steers around
  CHECK(r.min_ttc_s >= 1.5);
}

SDT_TEST(avoidance_flags_no_safe_option_when_boxed_in) {
  CollisionAvoidance ca;
  VehicleState own;
  Obstacle ring[8];
  for (int i = 0; i < 8; ++i) {
    const double a = i * std::numbers::pi / 4.0;
    ring[i].position = {2.2 * std::cos(a), 2.2 * std::sin(a), 0.0};
    ring[i].velocity = {-1.5 * std::cos(a), -1.5 * std::sin(a), 0.0};  // all closing in
    ring[i].radius_m = 1.0;
  }
  auto r = ca.resolve(own, {3.0, 0.0, 0.0}, ring, 8, 8.0);
  CHECK(r.no_safe_option);
}

// ---- mission & safety -----------------------------------------------------------------------
SDT_TEST(mission_executor_validates_and_advances) {
  MissionExecutor ex;
  Mission bad;
  bad.count = 1;
  bad.waypoints[0] = {{1, 1, 0}, -1.0f, 1.0f, 0.0f};  // negative speed
  CHECK(!ex.load(bad));

  Mission m;
  m.count = 2;
  m.waypoints[0] = {{10, 0, 0}, 2.0f, 1.0f, 0.0f};
  m.waypoints[1] = {{10, 10, 0}, 2.0f, 1.0f, 0.0f};
  CHECK(ex.load(m));
  CHECK(ex.start({0, 0, 0}, 0));
  VehicleState s;
  Vec3 v = ex.step(s, 0);
  CHECK(v.x > 1.0);
  s.position = {10, 0, 0};
  v = ex.step(s, millis(10));
  CHECK(ex.currentIndex() == 1);
  CHECK(v.y > 0.2);
  s.position = {10, 10, 0};
  ex.step(s, millis(20));
  CHECK(ex.state() == MissionState::Completed);
  CHECK(!ex.load(bad));
}

SDT_TEST(supervisor_state_machine) {
  SafetySupervisor sup;
  SafetyInputs in;
  in.armed = true;
  in.now = millis(10000);
  in.last_link = in.last_state = in.last_radar = in.now;
  CHECK(sup.evaluate(in).state == SafetyState::Nominal);

  in.now += millis(2500);  // link silent for 2.5 s, other sensors fresh
  in.last_state = in.last_radar = in.now;
  auto v = sup.evaluate(in);
  CHECK(v.action == FailsafeAction::Hold && (v.faults & fault::kLinkLost) != 0u);

  in.now += millis(8000);  // 10.5 s of silence
  in.last_state = in.last_radar = in.now;
  CHECK(sup.evaluate(in).action == FailsafeAction::ReturnHome);

  in.last_link = in.now;  // link back: failsafe holds through the recovery window...
  CHECK(sup.evaluate(in).action == FailsafeAction::ReturnHome);
  in.now += millis(1200);
  in.last_link = in.last_state = in.last_radar = in.now;
  CHECK(sup.evaluate(in).action == FailsafeAction::None);  // ...then clears

  in.now += millis(300);  // estimator silent -> latched emergency stop
  in.last_link = in.last_radar = in.now;
  CHECK(sup.evaluate(in).action == FailsafeAction::EmergencyStop);
  in.last_state = in.now;
  CHECK(sup.evaluate(in).action == FailsafeAction::EmergencyStop);  // stays latched
  sup.reset();
  CHECK(sup.evaluate(in).action == FailsafeAction::None);
}

// ---- media ----------------------------------------------------------------------------------
namespace {
struct FakeCam final : IMediaSource {
  int n = 0;
  int keyframe_requests = 0;
  bool poll(MediaFrame& f) override {
    if (n >= 6) return false;
    f.stream_id = 1;
    f.kind = MediaKind::Video;
    f.keyframe = (n % 3 == 0);
    f.payload.assign(8, 0);
    ++n;
    return true;
  }
  std::uint8_t streamId() const noexcept override { return 1; }
  void requestKeyframe() noexcept override { ++keyframe_requests; }
};
struct FlakySink final : IMediaSink {
  int accepted = 0;
  int calls = 0;
  bool publish(MediaFrame&&) override { return ++calls != 1 ? (++accepted, true) : false; }  // first frame rejected
};
}  // namespace

SDT_TEST(media_router_resyncs_on_keyframe_after_drop) {
  FakeCam cam;
  FlakySink sink;
  MediaRouter r;
  r.addSource(cam);
  r.setSink(sink);
  r.pump();
  r.pump();
  // frame0 (key) rejected -> frames 1,2 (deltas) dropped; frame3 (key) onwards accepted.
  CHECK(r.stats().keyframe_requests == 1);
  CHECK(sink.accepted == 3);
  CHECK(cam.keyframe_requests == 1);
}
