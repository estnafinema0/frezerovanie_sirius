#include "stock.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace milling {
namespace {
void need(bool b, const char *s) {
  if (!b)
    throw std::runtime_error(s);
}
double pointSegment(const Eigen::Vector2d &p, const Eigen::Vector2d &a,
                    const Eigen::Vector2d &b) {
  auto d = (b - a).eval();
  double t = d.squaredNorm() > 1e-20
                 ? std::clamp((p - a).dot(d) / d.squaredNorm(), 0., 1.)
                 : 0;
  return (p - a - t * d).norm();
}
std::vector<V> clip(std::vector<V> p, int axis, double bound, bool lower) {
  std::vector<V> out;
  for (size_t k = 0; k < p.size(); ++k) {
    V a = p[k], b = p[(k + 1) % p.size()];
    double da = (lower ? 1 : -1) * (a[axis] - bound),
           db = (lower ? 1 : -1) * (b[axis] - bound);
    if (da >= -1e-10)
      out.push_back(a);
    if ((da >= 0) != (db >= 0))
      out.push_back(a + (b - a) * da / (da - db));
  }
  return out;
}
} // namespace
void StockSettings::validate(const Options &o) const {
  o.validate();
  need(low.allFinite() && high.allFinite() &&
           (high.array() > low.array()).all(),
       "Stock requires explicit finite min < max on XYZ");
  need(low.cwiseAbs().maxCoeff() <= 100000 &&
           high.cwiseAbs().maxCoeff() <= 100000 &&
           (high - low).maxCoeff() <= 200,
       "Stock exceeds 200 mm prototype limit");
  for (double v : {cell, stepDown, stepOver, roughFeed})
    need(std::isfinite(v) && v > 0, "Invalid roughing parameter");
  need(cell >= .1 && cell <= 2 && cell <= o.diameter / 3,
       "Stock cell must be 0.1..2 mm and <= diameter/3");
  need(stepDown >= cell && stepDown <= 5 && stepOver >= cell &&
           stepOver <= o.diameter * .8,
       "Rough depth must be cell..5 mm; stepover cell..0.8 diameter");
  need(std::isfinite(allowance) && allowance >= 0 && allowance <= 5 &&
           roughFeed <= 50,
       "Invalid allowance or rough feed");
  need(high.z() - low.z() <= 30 && high.z() - low.z() <= o.length,
       "Stock height must be <=30 mm and within confirmed cutter length");
}
StockGrid::StockGrid(const Scene &s, const StockSettings &cfg, const Options &o)
    : settings(cfg) {
  cfg.validate(o);
  nx = int(std::ceil((cfg.high.x() - cfg.low.x()) / cfg.cell));
  ny = int(std::ceil((cfg.high.y() - cfg.low.y()) / cfg.cell));
  need(size_t(nx) * ny <= 40000,
       "Stock grid exceeds 40000 columns; increase cell size");
  dx = (cfg.high.x() - cfg.low.x()) / nx;
  dy = (cfg.high.y() - cfg.low.y()) / ny;
  target.assign(size_t(nx) * ny, cfg.low.z());
  remaining.assign(target.size(), cfg.high.z());
  initialVolume = (cfg.high - cfg.low).prod();
  size_t work = 0;
  for (const auto &t : s.mesh) {
    for (const auto &v : {t.a, t.b, t.c})
      need((v.array() >= cfg.low.array() - 1e-6).all() &&
               (v.array() <= cfg.high.array() + 1e-6).all(),
           "Target is outside stock; enter a containing block in part "
           "coordinates");
    V lo = t.a.cwiseMin(t.b).cwiseMin(t.c),
      hi = t.a.cwiseMax(t.b).cwiseMax(t.c);
    int x0 =
            std::clamp(int(std::floor((lo.x() - cfg.low.x()) / dx)), 0, nx - 1),
        x1 =
            std::clamp(int(std::floor((hi.x() - cfg.low.x()) / dx)), 0, nx - 1);
    int y0 =
            std::clamp(int(std::floor((lo.y() - cfg.low.y()) / dy)), 0, ny - 1),
        y1 =
            std::clamp(int(std::floor((hi.y() - cfg.low.y()) / dy)), 0, ny - 1);
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        need(++work <= 5000000, "Target rasterization budget exceeded");
        auto p = clip({t.a, t.b, t.c}, 0, cfg.low.x() + x * dx, true);
        p = clip(p, 0, cfg.low.x() + (x + 1) * dx, false);
        p = clip(p, 1, cfg.low.y() + y * dy, true);
        p = clip(p, 1, cfg.low.y() + (y + 1) * dy, false);
        for (const auto &v : p)
          target[y * nx + x] = std::max(target[y * nx + x], v.z());
      }
  }
}
V StockGrid::center(int x, int y) const {
  return V(settings.low.x() + (x + .5) * dx, settings.low.y() + (y + .5) * dy,
           0);
}
template <class F>
void StockGrid::visit(const V &a, const V &b, double radius, F fn) const {
  const double half = std::hypot(dx, dy) / 2;
  V lo = a.cwiseMin(b), hi = a.cwiseMax(b);
  int x0 = std::clamp(
          int(std::floor((lo.x() - radius - half - settings.low.x()) / dx)), 0,
          nx - 1),
      x1 = std::clamp(
          int(std::floor((hi.x() + radius + half - settings.low.x()) / dx)), 0,
          nx - 1);
  int y0 = std::clamp(
          int(std::floor((lo.y() - radius - half - settings.low.y()) / dy)), 0,
          ny - 1),
      y1 = std::clamp(
          int(std::floor((hi.y() + radius + half - settings.low.y()) / dy)), 0,
          ny - 1);
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) {
      need(++work <= 50000000, "Stock query work budget exceeded");
      double d = pointSegment(center(x, y).head<2>(), a.head<2>(), b.head<2>());
      if (d <= radius + half)
        fn(y * nx + x, d, half);
    }
}
bool StockGrid::targetClear(const V &a, const V &b, double radius,
                            double z) const {
  bool clear = true;
  // Half diagonal inflates the footprint to include every touched grid cell.
  // 0.03 mm mesh reserve is explicit; it is not a certified CAD error bound.
  visit(a, b, radius + settings.allowance, [&](int i, double, double) {
    if (target[i] > settings.low.z() + 1e-8 &&
        z < target[i] + settings.allowance + .03 - 1e-8)
      clear = false;
    if (z < settings.low.z() - 1e-8)
      clear = false;
  });
  return clear;
}
double StockGrid::engagement(const V &a, const V &b, double radius,
                             double z) const {
  double depth = 0;
  visit(a, b, radius, [&](int i, double d, double) {
    if (d <= radius)
      depth = std::max(depth, remaining[i] - z);
  });
  return depth;
}
double StockGrid::remove(const V &a, const V &b, double radius, double z) {
  double volume = 0;
  visit(a, b, radius, [&](int i, double d, double) {
    if (d <= radius && remaining[i] > z) {
      volume += (remaining[i] - z) * dx * dy;
      remaining[i] = z;
    }
  });
  removedVolume += volume;
  return volume;
}
double StockGrid::excessVolume(double allowance) const {
  double v = 0;
  for (size_t i = 0; i < target.size(); ++i)
    v += std::max(0.,
                  remaining[i] -
                      std::min(settings.high.z(),
                               target[i] + (target[i] > settings.low.z() + 1e-8
                                                ? allowance
                                                : 0))) *
         dx * dy;
  return v;
}
} // namespace milling
