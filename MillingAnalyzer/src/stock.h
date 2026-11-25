#pragma once
#include "geometry.h"
#include <cstdint>
namespace milling {
struct StockSettings {
  V low = V(-2, -2, 0), high = V(22, 22, 6);
  double cell = .5, stepDown = 1, stepOver = 1, allowance = .3, roughFeed = 10;
  bool plungeConfirmed = false;
  V topCorner() const { return {low.x(), low.y(), high.z()}; }
  void validate(const Options &) const;
};
// Bounded dexel grid: an upper envelope of the triangulated target, not a
// replacement for B-rep surface evaluation used by finishing.
class StockGrid {
public:
  StockSettings settings;
  int nx = 0, ny = 0;
  double dx = 0, dy = 0, initialVolume = 0, removedVolume = 0;
  std::vector<double> target, remaining;
  StockGrid(const Scene &, const StockSettings &, const Options &);
  V center(int ix, int iy) const;
  bool targetClear(const V &a, const V &b, double radius, double z) const;
  double engagement(const V &a, const V &b, double radius, double z) const;
  double remove(const V &a, const V &b, double radius, double z);
  double excessVolume(double allowance) const;

private:
  mutable size_t work = 0;
  template <class F> void visit(const V &, const V &, double, F) const;
};
} // namespace milling
