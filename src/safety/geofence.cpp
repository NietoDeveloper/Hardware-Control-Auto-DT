#include "sdt/safety/geofence.hpp"

#include <algorithm>
#include <cmath>

namespace sdt {
namespace {
double distPointSegment(double px, double py, const Vec3& a, const Vec3& b) noexcept {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double len2 = abx * abx + aby * aby;
  double t = len2 > 1e-12 ? ((px - a.x) * abx + (py - a.y) * aby) / len2 : 0.0;
  t = clampd(t, 0.0, 1.0);
  return std::hypot(px - (a.x + t * abx), py - (a.y + t * aby));
}
}  // namespace

double Geofence::margin(const Vec3& p) const noexcept {
  if (!enabled || vertex_count < 3) return kInf;
  const std::size_t n = std::min<std::size_t>(vertex_count, kMaxFenceVertices);

  bool inside = false;
  double min_d = kInf;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Vec3& a = vertices[i];
    const Vec3& b = vertices[j];
    if (((a.y > p.y) != (b.y > p.y)) &&
        (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
      inside = !inside;
    }
    min_d = std::min(min_d, distPointSegment(p.x, p.y, a, b));
  }
  const double horizontal = inside ? min_d : -min_d;
  const double vertical = std::min(p.z - min_z, max_z - p.z);
  return std::min(horizontal, vertical);
}

}  // namespace sdt
