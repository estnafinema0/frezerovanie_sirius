#include "bezier_cam.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>

namespace freza::bezier {
namespace {
constexpr double guard =
    1e-8; // millimetres; numerical allowance, not formal interval arithmetic
void Require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
bool Finite(Point3 p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
Point3 Sub(Point3 a, Point3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double Length(Point3 p) { return std::hypot(std::hypot(p.x, p.y), p.z); }
double Curve(std::array<double, 4> c, double t, int n = 3) {
  for (int k = n; k > 0; --k)
    for (int i = 0; i < k; ++i)
      c[i] = (1 - t) * c[i] + t * c[i + 1];
  return c[0];
}
double Height(const Patch &p, double u, double v) {
  std::array<double, 4> c{};
  for (int i = 0; i < 4; ++i)
    c[i] =
        Curve({p.z[4 * i], p.z[4 * i + 1], p.z[4 * i + 2], p.z[4 * i + 3]}, v);
  return Curve(c, u);
}
void Settings(const Options &o) {
  Require(std::isfinite(o.diameter) && o.diameter > 0 && o.diameter <= 1000,
          "Cutter diameter must be in (0,1000] mm.");
  Require(std::isfinite(o.contactTolerance) && o.contactTolerance >= 1e-6 &&
              o.contactTolerance <= 1,
          "Contact tolerance must be in [1e-6,1] mm.");
  Require(std::isfinite(o.pathTolerance) &&
              o.pathTolerance >= 4 * o.contactTolerance &&
              o.pathTolerance <= 10,
          "Path tolerance must be >= 4*contactTolerance and <=10 mm.");
  Require(std::isfinite(o.maxStep) && o.maxStep >= 1e-4 && o.maxStep <= 100,
          "CAM step must be in [1e-4,100] mm.");
  Require(o.maxPoints >= 5 && o.maxPoints <= 200000 &&
              o.maxNodesPerQuery >= 4 && o.maxNodesPerQuery <= 1000000 &&
              o.maxDepth > 0 && o.maxDepth <= 30 && o.maxTotalNodes >= 4 &&
              o.maxTotalNodes <= 50000000,
          "Invalid resource limits.");
}
// Squared distance of point to segment, and closest point on the segment.
double Distance(Point3 p, Point3 a, Point3 b, Point3 *nearest = nullptr) {
  double dx = b.x - a.x, dy = b.y - a.y, den = dx * dx + dy * dy;
  double t =
      den > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / den, 0., 1.)
              : 0;
  Point3 q{a.x + t * dx, a.y + t * dy, 0};
  if (nearest)
    *nearest = q;
  return (p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y);
}
// Closest rectangle point to segment. Slab intersection, then
// endpoints/corners.
Point3 Closest(const Patch &p, Point3 a, Point3 b) {
  double lo = 0, hi = 1;
  const double starts[] = {a.x, a.y}, delta[] = {b.x - a.x, b.y - a.y},
               mins[] = {p.x, p.y}, maxs[] = {p.x + p.width, p.y + p.height};
  bool hit = true;
  for (int axis = 0; axis < 2; ++axis) {
    if (delta[axis] == 0) {
      if (starts[axis] < mins[axis] || starts[axis] > maxs[axis])
        hit = false;
    } else {
      double t0 = (mins[axis] - starts[axis]) / delta[axis],
             t1 = (maxs[axis] - starts[axis]) / delta[axis];
      if (t0 > t1)
        std::swap(t0, t1);
      lo = std::max(lo, t0);
      hi = std::min(hi, t1);
    }
  }
  if (hit && lo <= hi)
    return {std::clamp(a.x + lo * (b.x - a.x), p.x, p.x + p.width),
            std::clamp(a.y + lo * (b.y - a.y), p.y, p.y + p.height), 0};
  Point3 best{};
  double distance = std::numeric_limits<double>::infinity();
  auto consider = [&](Point3 q) {
    double d = Distance(q, a, b);
    if (d < distance) {
      best = q;
      distance = d;
    }
  };
  for (auto e : {a, b})
    consider({std::clamp(e.x, p.x, p.x + p.width),
              std::clamp(e.y, p.y, p.y + p.height), 0});
  for (double x : {p.x, p.x + p.width})
    for (double y : {p.y, p.y + p.height})
      consider({x, y, 0});
  return best;
}
std::pair<std::array<double, 4>, std::array<double, 4>>
Split(std::array<double, 4> c) {
  std::array<double, 4> l{}, r{};
  l[0] = c[0];
  r[3] = c[3];
  for (int k = 3; k > 0; --k) {
    for (int i = 0; i < k; ++i)
      c[i] = (c[i] + c[i + 1]) * 0.5;
    l[4 - k] = c[0];
    r[k - 1] = c[k - 1];
  }
  return {l, r};
}
std::array<Patch, 4> Children(const Patch &p) {
  Patch left = p, right = p;
  left.width = right.width = p.width / 2;
  right.x += left.width;
  for (int j = 0; j < 4; ++j) {
    auto s = Split({p.z[j], p.z[4 + j], p.z[8 + j], p.z[12 + j]});
    for (int i = 0; i < 4; ++i) {
      left.z[4 * i + j] = s.first[i];
      right.z[4 * i + j] = s.second[i];
    }
  }
  std::array<Patch, 4> out{left, left, right, right};
  for (int k = 0; k < 4; k += 2) {
    out[k].height = out[k + 1].height = p.height / 2;
    out[k + 1].y += p.height / 2;
    for (int i = 0; i < 4; ++i) {
      auto s = Split({out[k].z[4 * i], out[k].z[4 * i + 1], out[k].z[4 * i + 2],
                      out[k].z[4 * i + 3]});
      for (int j = 0; j < 4; ++j) {
        out[k].z[4 * i + j] = s.first[j];
        out[k + 1].z[4 * i + j] = s.second[j];
      }
    }
  }
  return out;
}
double Lipschitz(const Patch &p) {
  double dx = 0, dy = 0;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      if (i < 3)
        dx = std::max(dx, 3 * std::abs(p.z[4 * (i + 1) + j] - p.z[4 * i + j]) /
                              p.width);
      if (j < 3)
        dy = std::max(dy, 3 * std::abs(p.z[4 * i + j + 1] - p.z[4 * i + j]) /
                              p.height);
    }
  return std::hypot(dx, dy);
}
void InDomain(const Patch &p, Point3 q) {
  Require(Finite(q) && q.x >= p.x - guard && q.x <= p.x + p.width + guard &&
              q.y >= p.y - guard && q.y <= p.y + p.height + guard,
          "Tool centre outside the patch domain (unsupported "
          "approach/transition).");
}
} // namespace
void Validate(const Patch &p) {
  Require(std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) <= 100000 &&
              std::abs(p.y) <= 100000,
          "Invalid patch origin.");
  Require(std::isfinite(p.width) && std::isfinite(p.height) &&
              p.width >= 0.001 && p.height >= 0.001 && p.width <= 10000 &&
              p.height <= 10000,
          "Patch width/height must be in [0.001,10000] mm.");
  for (double z : p.z)
    Require(std::isfinite(z) && std::abs(z) <= 100000,
            "Nonfinite/out-of-range control height.");
}
Patch Load(const std::filesystem::path &path) {
  Require(std::filesystem::file_size(path) <= 8192,
          "Patch file exceeds 8 KiB.");
  std::ifstream file(path, std::ios::binary);
  Require(bool(file), "Cannot open patch file.");
  std::string data(8193, '\0');
  file.read(data.data(), 8193);
  data.resize(static_cast<std::size_t>(file.gcount()));
  Require(data.size() <= 8192, "Patch file exceeds 8 KiB.");
  std::istringstream f(data);
  f.imbue(std::locale::classic());
  auto tag = [&](const char *expected) {
    std::string t;
    Require(bool(f >> t) && t == expected, "Invalid patch format/tag.");
  };
  tag("FREZA_BEZIER_HEIGHTFIELD");
  int version = 0;
  Require(bool(f >> version) && version == 1, "Unsupported patch version.");
  tag("units");
  std::string units;
  f >> units;
  Require(units == "mm" || units == "inch",
          "Explicit units must be mm or inch.");
  double scale = units == "mm" ? 1 : 25.4;
  Patch p;
  tag("origin");
  Require(bool(f >> p.x >> p.y), "Invalid origin.");
  tag("size");
  Require(bool(f >> p.width >> p.height), "Invalid size.");
  tag("heights");
  for (auto &z : p.z)
    Require(bool(f >> z), "Expected 16 finite control heights.");
  std::string extra;
  Require(!(f >> extra), "Unexpected trailing data.");
  p.x *= scale;
  p.y *= scale;
  p.width *= scale;
  p.height *= scale;
  for (auto &z : p.z)
    z *= scale;
  Validate(p);
  return p;
}
Sample Evaluate(const Patch &p, double u, double v) {
  Validate(p);
  Require(std::isfinite(u) && std::isfinite(v) && u >= 0 && u <= 1 && v >= 0 &&
              v <= 1,
          "UV must lie in [0,1].");
  std::array<double, 4> cu{}, cv{};
  for (int i = 0; i < 3; ++i) {
    std::array<double, 4> row{};
    for (int j = 0; j < 4; ++j)
      row[j] = 3 * (p.z[4 * (i + 1) + j] - p.z[4 * i + j]);
    cu[i] = Curve(row, v);
  }
  for (int i = 0; i < 4; ++i) {
    std::array<double, 4> row{};
    for (int j = 0; j < 3; ++j)
      row[j] = 3 * (p.z[4 * i + j + 1] - p.z[4 * i + j]);
    cv[i] = Curve(row, v, 2);
  }
  double zu = Curve(cu, u, 2), zv = Curve(cv, u);
  Point3 n{-zu / p.width, -zv / p.height, 1};
  double length = Length(n);
  n = {n.x / length, n.y / length, n.z / length};
  return {{p.x + u * p.width, p.y + v * p.height, Height(p, u, v)},
          {p.width, 0, zu},
          {0, p.height, zv},
          n};
}
Bound ContactBound(const Patch &p, Point3 a, Point3 b, const Options &o) {
  Validate(p);
  Settings(o);
  InDomain(p, a);
  InDomain(p, b);
  struct Node {
    Patch p;
    unsigned depth;
    double upper;
    bool operator<(const Node &n) const { return upper < n.upper; }
  };
  std::priority_queue<Node> queue;
  double lower = -std::numeric_limits<double>::infinity();
  std::size_t nodes = 0;
  const double radius = o.diameter / 2;
  auto add = [&](const Patch &box, unsigned depth) {
    Require(++nodes <= o.maxNodesPerQuery,
            "Contact subdivision resource limit; loosen tolerance or simplify "
            "patch.");
    Point3 closest = Closest(box, a, b);
    if (Distance(closest, a, b) > (radius + guard) * (radius + guard))
      return;
    // Only actual footprint samples contribute lower bounds; enlarged test is
    // upper-bound-only.
    auto sample = [&](Point3 q) {
      if (Distance(q, a, b) <= radius * radius) {
        double z = Height(box, std::clamp((q.x - box.x) / box.width, 0., 1.),
                          std::clamp((q.y - box.y) / box.height, 0., 1.));
        lower = std::max(lower, z - guard);
      }
    };
    sample(closest);
    sample({box.x + box.width / 2, box.y + box.height / 2, 0});
    for (double x : {box.x, box.x + box.width})
      for (double y : {box.y, box.y + box.height})
        sample({x, y, 0});
    double upper = *std::max_element(box.z.begin(), box.z.end()) + guard;
    if (upper > lower)
      queue.push({box, depth, upper});
  };
  add(p, 0);
  while (!queue.empty() && queue.top().upper - lower > o.contactTolerance) {
    Node n = queue.top();
    queue.pop();
    Require(n.depth < o.maxDepth,
            "Contact depth limit; no uncertified result returned.");
    for (auto child : Children(n.p))
      add(child, n.depth + 1);
  }
  Require(std::isfinite(lower), "No surface contact in footprint.");
  return {lower,
          queue.empty() ? lower + 2 * guard
                        : std::max(lower + 2 * guard, queue.top().upper),
          nodes};
}
Result Perimeter(const Patch &p, const Options &o) {
  Validate(p);
  Settings(o);
  Result result;
  std::vector<Point3> xy{{p.x, p.y, 0},
                         {p.x + p.width, p.y, 0},
                         {p.x + p.width, p.y + p.height, 0},
                         {p.x, p.y + p.height, 0},
                         {p.x, p.y, 0}};
  const double lip = Lipschitz(p);
  std::size_t total = 0;
  auto query = [&](Point3 a, Point3 b) {
    auto bound = ContactBound(p, a, b, o);
    total += bound.nodes;
    Require(total <= o.maxTotalNodes, "Total contact work limit exceeded.");
    ++result.queries;
    return bound;
  };
  for (unsigned iteration = 0; iteration <= o.maxDepth; ++iteration) {
    Require(xy.size() <= o.maxPoints,
            "CAM point limit; loosen path tolerance/step.");
    std::vector<Bound> segment, endpoint;
    for (auto q : xy) {
      endpoint.push_back(query(q, q));
    }
    for (std::size_t i = 1; i < xy.size(); ++i) {
      segment.push_back(query(xy[i - 1], xy[i]));
    }
    auto points = xy;
    // Heights can be negative; start from the computed envelope, never
    // implicitly clamp to zero.
    for (std::size_t i = 0; i < points.size(); ++i) {
      double prev = segment[i == 0 ? segment.size() - 1 : i - 1].upper;
      double next = segment[i == segment.size() ? 0 : i].upper;
      points[i].z = std::max(prev, next) + 4 * guard;
    }
    std::vector<Point3> refined{xy.front()};
    bool done = true;
    double clearance = 0;
    for (std::size_t i = 0; i < segment.size(); ++i) {
      double length = Length(Sub(xy[i + 1], xy[i]));
      // Envelope is Lipschitz on rectangular XY domain (translate then clamp
      // footprint).
      double floor =
          std::min(endpoint[i].lower, endpoint[i + 1].lower) - lip * length / 2;
      double excess = std::max(points[i].z, points[i + 1].z) - floor;
      clearance = std::max(clearance, excess);
      if (excess > o.pathTolerance ||
          Length(Sub(points[i + 1], points[i])) > o.maxStep) {
        done = false;
        refined.push_back(
            {(xy[i].x + xy[i + 1].x) / 2, (xy[i].y + xy[i + 1].y) / 2, 0});
      }
      refined.push_back(xy[i + 1]);
    }
    if (done) {
      result.points = std::move(points);
      result.certifiedClearance = clearance;
      ValidatePath(p, result.points, o);
      return result;
    }
    xy = std::move(refined);
  }
  throw std::runtime_error("Path depth limit; no partial path returned.");
}
void ValidatePath(const Patch &p, const std::vector<Point3> &points,
                  const Options &o) {
  Validate(p);
  Settings(o);
  Require(points.size() >= 2 && points.size() <= 200000, "Invalid path size.");
  for (auto q : points)
    InDomain(p, q);
  std::size_t total = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    auto bound = ContactBound(p, points[i - 1], points[i], o);
    total += bound.nodes;
    Require(total <= o.maxTotalNodes, "Total validation work limit exceeded.");
    Require(std::min(points[i - 1].z, points[i].z) + guard >= bound.upper,
            "Tool penetration cannot be excluded on a continuous segment.");
  }
}
std::vector<Point3> ToBase(const Patch &p, const Result &result, Point3 t,
                           const Options &o, double step, double feed) {
  Require(Finite(t) && std::abs(t.x) <= 100000 && std::abs(t.y) <= 100000 &&
              std::abs(t.z) <= 100000,
          "Invalid translation.");
  Require(std::isfinite(step) && step >= 0.001 && step <= 100 &&
              std::isfinite(feed) && feed > 0 && feed <= 1000,
          "Invalid interpolation settings.");
  ValidatePath(p, result.points, o);
  std::vector<Point3> base;
  // Only remove numerically collinear forward points; re-certify the longer
  // segments.
  for (auto q : result.points) {
    while (base.size() >= 2) {
      auto a = base[base.size() - 2], b = base.back();
      auto ab = Sub(b, a), bc = Sub(q, b);
      Point3 cross{ab.y * bc.z - ab.z * bc.y, ab.z * bc.x - ab.x * bc.z,
                   ab.x * bc.y - ab.y * bc.x};
      if (Length(cross) > 1e-10 * std::max(1., Length(ab) + Length(bc)) ||
          ab.x * bc.x + ab.y * bc.y + ab.z * bc.z < 0)
        break;
      base.pop_back();
    }
    base.push_back(q);
  }
  ValidatePath(p, base, o);
  double estimated = 1;
  for (std::size_t i = 1; i < base.size(); ++i)
    estimated += std::ceil(Length(Sub(base[i], base[i - 1])) / step) + 1;
  Require(estimated <= 200000, "Interpolation resource limit.");
  for (auto &q : base) {
    // Existing CSV stores 6 decimals. Round XY explicitly and round Z UP, then
    // recheck.
    q.x = std::round((q.x + t.x) * 1e6) / 1e6;
    q.y = std::round((q.y + t.y) * 1e6) / 1e6;
    q.z = std::ceil((q.z + t.z) * 1e6 + 1) / 1e6;
  }
  auto quantized = base;
  for (auto &q : quantized) {
    q.x -= t.x;
    q.y -= t.y;
    q.z -= t.z;
  }
  ValidatePath(p, quantized, o);
  std::vector<Point3> interpolated;
  std::vector<int> dwell;
  std::vector<unsigned char> corner;
  InterpolatePath(base, step, feed, interpolated, dwell, corner);
  for (auto &q : interpolated) {
    q.x -= t.x;
    q.y -= t.y;
    q.z -= t.z;
  }
  ValidatePath(p, interpolated, o);
  return base;
}
} // namespace freza::bezier
