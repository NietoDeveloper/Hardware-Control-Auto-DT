#include "sdt/perception/radar_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace sdt {
namespace {
constexpr double kMaxTargetSpeedMps = 40.0;

struct Cluster {
  Vec3 sum;
  int n{0};
  Vec3 centroid() const noexcept { return sum * (1.0 / static_cast<double>(n)); }
};
}  // namespace

RadarTracker::RadarTracker(const TrackerConfig& cfg) noexcept : cfg_(cfg) {}

void RadarTracker::reset() noexcept {
  tracks_ = {};
  next_id_ = 1;
}

std::size_t RadarTracker::activeCount() const noexcept {
  return static_cast<std::size_t>(
      std::count_if(tracks_.begin(), tracks_.end(), [](const Track& t) { return t.active; }));
}

std::size_t RadarTracker::confirmed(ObstacleList& out) const noexcept {
  std::size_t n = 0;
  for (const Track& t : tracks_) {
    if (t.active && t.obs.hits >= cfg_.confirm_hits && t.obs.misses <= cfg_.max_misses) {
      out[n++] = t.obs;
    }
  }
  return n;
}

void RadarTracker::update(const RadarScan& scan, const VehicleState& own) noexcept {
  // 1) Sensor frame -> world frame, clustered into point measurements.
  std::array<Cluster, kMaxTracks> clusters{};
  std::size_t nc = 0;
  const double cy = std::cos(own.yaw);
  const double sy = std::sin(own.yaw);
  const std::size_t n_det = std::min<std::size_t>(scan.count, kMaxRadarDetections);

  for (std::size_t i = 0; i < n_det; ++i) {
    const RadarDetection& d = scan.detections[i];
    const double r = static_cast<double>(d.range_m);
    const double az = static_cast<double>(d.azimuth_rad);
    const double el = static_cast<double>(d.elevation_rad);
    if (!(r >= cfg_.min_range_m && r <= cfg_.max_range_m) || !std::isfinite(az) ||
        !std::isfinite(el) || static_cast<double>(d.snr_db) < cfg_.min_snr_db) {
      continue;  // also rejects NaN
    }
    const double xb = r * std::cos(el) * std::cos(az);
    const double yb = r * std::cos(el) * std::sin(az);
    const double zb = r * std::sin(el);
    const Vec3 w{own.position.x + xb * cy - yb * sy, own.position.y + xb * sy + yb * cy,
                 own.position.z + zb};

    bool merged = false;
    for (std::size_t c = 0; c < nc; ++c) {
      if ((clusters[c].centroid() - w).norm() <= cfg_.cluster_m) {
        clusters[c].sum += w;
        ++clusters[c].n;
        merged = true;
        break;
      }
    }
    if (!merged && nc < kMaxTracks) clusters[nc++] = Cluster{w, 1};
  }

  // 2) Predict existing tracks to the scan time.
  for (Track& t : tracks_) {
    t.matched = false;
    if (!t.active) continue;
    t.dt = std::max(0.0, toSeconds(scan.stamp - t.obs.last_update));
    t.pred = t.obs.position + t.obs.velocity * t.dt;
  }

  // 3) Greedy nearest-neighbour association inside the gate.
  std::array<bool, kMaxTracks> used{};
  for (std::size_t c = 0; c < nc; ++c) {
    const Vec3 z = clusters[c].centroid();
    std::size_t best = kMaxTracks;
    double best_d = cfg_.gate_m;
    for (std::size_t k = 0; k < kMaxTracks; ++k) {
      const Track& t = tracks_[k];
      if (!t.active || t.matched) continue;
      const double d = (z - t.pred).norm();
      if (d < best_d) {
        best_d = d;
        best = k;
      }
    }
    if (best == kMaxTracks) continue;

    Track& t = tracks_[best];
    const Vec3 res = z - t.pred;
    t.obs.position = t.pred + res * cfg_.alpha;
    if (t.dt > 1e-3) {
      Vec3 v = t.obs.velocity + res * (cfg_.beta / t.dt);
      const double sp = v.norm();
      if (sp > kMaxTargetSpeedMps) v = v * (kMaxTargetSpeedMps / sp);
      t.obs.velocity = v;
    }
    if (t.obs.hits < UINT16_MAX) ++t.obs.hits;
    t.obs.misses = 0;
    t.obs.last_update = scan.stamp;
    t.matched = true;
    used[c] = true;
  }

  // 4) Coast or drop unmatched tracks.
  for (Track& t : tracks_) {
    if (!t.active || t.matched) continue;
    t.obs.position = t.pred;
    t.obs.last_update = scan.stamp;
    if (++t.obs.misses > cfg_.max_misses) t.active = false;
  }

  // 5) Spawn tentative tracks from unused clusters.
  for (std::size_t c = 0; c < nc; ++c) {
    if (used[c]) continue;
    for (Track& t : tracks_) {
      if (t.active) continue;
      t = Track{};
      t.active = true;
      t.obs.id = next_id_++;
      t.obs.position = clusters[c].centroid();
      t.obs.radius_m = cfg_.default_radius_m;
      t.obs.hits = 1;
      t.obs.last_update = scan.stamp;
      break;
    }
  }
}

}  // namespace sdt
