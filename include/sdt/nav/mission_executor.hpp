#pragma once
#include <cstdint>

#include "sdt/nav/mission.hpp"
#include "sdt/nav/path_follower.hpp"

namespace sdt {

enum class MissionState : std::uint8_t { Idle = 0, Running = 1, Paused = 2, Completed = 3, Aborted = 4 };

/// Deterministic mission state machine. No I/O, no allocation, no clock access.
class MissionExecutor {
 public:
  explicit MissionExecutor(const FollowerConfig& cfg = {}) noexcept : follower_(cfg) {}

  /// Validates and stores a mission. Rejected while Running/Paused.
  bool load(const Mission& m) noexcept;
  bool start(const Vec3& position, TimeNs now) noexcept;
  bool pause() noexcept;
  bool resume() noexcept;
  void abort() noexcept;

  /// Desired world-frame velocity for this cycle (zero unless Running).
  Vec3 step(const VehicleState& s, TimeNs now) noexcept;

  MissionState state() const noexcept { return state_; }
  std::uint16_t currentIndex() const noexcept { return index_; }
