#include "operation_plan.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <algorithm>
#include <cmath>
#include <functional>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <set>
#include <sstream>
namespace milling {
std::vector<Operation> buildRoughOperations(StockGrid &, const Options &,
                                            const StockSettings &);
namespace {
constexpr double pi = 3.141592653589793;
void need(bool b, const char *s) {
  if (!b)
    throw std::runtime_error(s);
}
Pose at(V p) {
  Pose v;
  v.p = p;
  v.n = V::UnitZ();
  v.q = Q::Identity();
  return v;
}
void append(Pass &path, Pose end, int motion, double feed, const Options &o) {
  end.motion = motion;
  if (path.poses.empty()) {
    end.t = 0;
    path.poses.push_back(end);
    return;
  }
  auto a = path.poses.back();
  end.q = frame(end.q * V::UnitZ(), end.q * V::UnitX(), &a.q);
  double d = (end.p - a.p).norm(), angle = a.q.angularDistance(end.q);
  if (d < 1e-9 && angle < 1e-10)
    return;
  double duration = std::max(d / feed, angle / (30 * pi / 180));
  int n = std::max(
      1, int(std::ceil(std::max(d / o.step, angle / (o.angleDeg * pi / 180)))));
  need(path.poses.size() + n <= 100000, "Program pose budget exceeded");
  for (int k = 1; k <= n; ++k) {
    double t = double(k) / n;
    Pose p = end;
    p.p = a.p + t * (end.p - a.p);
    p.q = a.q.slerp(t, end.q).normalized();
    p.n = p.q * V::UnitZ();
    p.t = a.t + t * duration;
    path.poses.push_back(p);
  }
}
std::vector<TopoDS_Face> faces(const Model &m) {
  std::vector<TopoDS_Face> result;
  for (TopExp_Explorer e(m.shape, TopAbs_FACE); e.More(); e.Next())
    result.push_back(TopoDS::Face(e.Current()));
  return result;
}
std::vector<TopoDS_Wire> wires(const TopoDS_Face &f) {
  std::vector<TopoDS_Wire> result;
  auto outer = BRepTools::OuterWire(f);
  if (!outer.IsNull())
    result.push_back(outer);
  for (TopExp_Explorer e(f, TopAbs_WIRE); e.More(); e.Next())
    if (!e.Current().IsSame(outer))
      result.push_back(TopoDS::Wire(e.Current()));
  return result;
}
V vec(const gp_Pnt &p) { return V(p.X(), p.Y(), p.Z()); }
std::vector<V> sampleWire(const TopoDS_Wire &wire, const Options &o) {
  std::vector<V> points;
  for (BRepTools_WireExplorer e(wire); e.More(); e.Next()) {
    BRepAdaptor_Curve c(e.Current());
    double first = c.FirstParameter(), last = c.LastParameter();
    if (e.Current().Orientation() == TopAbs_REVERSED)
      std::swap(first, last);
    auto point = [&](double t) { return vec(c.Value(t)); };
    V a = point(first), b = point(last);
    if (points.empty())
      points.push_back(a);
    need((points.back() - a).norm() < 1e-5, "Contour edges are disconnected");
    std::function<void(double, double, V, V, int)> split;
    split = [&](double u, double v, V x, V y, int depth) {
      double mid = (u + v) / 2;
      V m = point(mid);
      if ((y - x).norm() > o.step || (m - (x + y) / 2).norm() > o.chord / 4) {
        need(depth < 22, "Contour subdivision budget exceeded");
        split(u, mid, x, m, depth + 1);
        split(mid, v, m, y, depth + 1);
      } else {
        need(points.size() < 20000, "Contour point budget exceeded");
        points.push_back(y);
      }
    };
    split(first, last, a, b, 0);
  }
  need(points.size() > 3 && (points.front() - points.back()).norm() < 1e-5,
       "Select a closed contour");
  return points;
}
double area(const std::vector<V> &p) {
  double a = 0;
  for (size_t i = 1; i < p.size(); ++i)
    a += p[i - 1].x() * p[i].y() - p[i].x() * p[i - 1].y();
  return std::abs(a) / 2;
}
bool horizontal(const TopoDS_Face &f) {
  BRepAdaptor_Surface s(f);
  return s.GetType() == GeomAbs_Plane &&
         std::abs(s.Plane().Axis().Direction().Z()) > 1 - 1e-8;
}
Pass surfacePath(const TopoDS_Face &f, const Options &o,
                 const StockSettings &stock) {
  auto passes = raster(f, o);
  need(!passes.empty(), "No usable passes for this tool and surface");
  V average = V::Zero();
  for (auto &pass : passes)
    for (auto &p : pass.poses)
      average += p.n;
  need(average.norm() > 1e-8,
       "Surface requires subdivision into accessible patches");
  average.normalize();
  for (auto &pass : passes)
    for (auto &p : pass.poses)
      need(p.n.z() >= -1e-6 && p.n.dot(average) > .2,
           "Surface requires another setup or subdivision");
  Q rotation = Q::FromTwoVectors(average, V::UnitZ());
  auto matrix = rotation.toRotationMatrix();
  gp_Trsf tr;
  tr.SetValues(matrix(0, 0), matrix(0, 1), matrix(0, 2), 0, matrix(1, 0),
               matrix(1, 1), matrix(1, 2), 0, matrix(2, 0), matrix(2, 1),
               matrix(2, 2), 0);
  auto local = TopoDS::Face(BRepBuilderAPI_Transform(f, tr, true).Shape());
  for (auto &pass : passes)
    for (auto &p : pass.poses) {
      p.p = rotation * p.p;
      p.n = rotation * p.n;
      p.q = rotation * p.q;
    }
  double top = -1e100;
  for (double x : {stock.low.x(), stock.high.x()})
    for (double y : {stock.low.y(), stock.high.y()})
      for (double z : {stock.low.z(), stock.high.z()})
        top = std::max(top, (rotation * V(x, y, z)).z());
  auto path = connectPasses(passes, top, o, &local);
  for (auto &p : path.poses) {
    p.p = rotation.conjugate() * p.p;
    p.n = rotation.conjugate() * p.n;
    p.q = rotation.conjugate() * p.q;
    if (p.motion == 3)
      p.motion = 4;
  }
  Pass result;
  Pose start = path.poses.front();
  start.p.z() = std::max(start.p.z(), stock.high.z() +
                                          std::hypot(o.length, o.diameter / 2) +
                                          o.clearance);
  append(result, start, 2, o.feed, o);
  for (auto &p : path.poses)
    append(result, p, p.motion, o.feed, o);
  Pose end = result.poses.back();
  end.p.z() = start.p.z();
  append(result, end, 1, o.feed, o);
  return result;
}
// Orthographic silhouette of the actual part, not its bounding rectangle.
// Positive-Z triangles cover the projection of a closed solid, including
// terraces/overhangs. Mesh approximation is the same 0.02 mm as validation.
TopoDS_Face projectedPart(const Scene &scene) {
  TopTools_ListOfShape arguments, tools;
  int count = 0;
  for (const auto &t : scene.mesh) {
    if ((t.b - t.a).cross(t.c - t.a).z() <= 1e-10)
      continue;
    need(
        ++count <= 2000,
        "Part silhouette exceeds 2000 projected triangles; simplify the model");
    BRepBuilderAPI_MakePolygon polygon;
    for (auto p : {t.a, t.b, t.c})
      polygon.Add(gp_Pnt(p.x(), p.y(), 0));
    polygon.Close();
    auto face = BRepBuilderAPI_MakeFace(polygon.Wire()).Face();
    if (arguments.IsEmpty())
      arguments.Append(face);
    else
      tools.Append(face);
  }
  need(!arguments.IsEmpty(), "Part has no usable XY silhouette");
  TopoDS_Shape shape = arguments.First();
  if (!tools.IsEmpty()) {
    BRepAlgoAPI_Fuse fuse;
    fuse.SetArguments(arguments);
    fuse.SetTools(tools);
    fuse.SetRunParallel(false);
    fuse.Build();
    need(fuse.IsDone(), "Cannot join part silhouette");
    fuse.SimplifyResult(true, true);
    shape = fuse.Shape();
  }
  std::vector<TopoDS_Face> result;
  for (TopExp_Explorer e(shape, TopAbs_FACE); e.More(); e.Next())
    result.push_back(TopoDS::Face(e.Current()));
  need(result.size() == 1,
       "Part projection has multiple outlines; select a separate part");
  return result.front();
}
bool projectedInside(const Triangle &t, const V &p) {
  auto side = [](const V &a, const V &b, const V &p) {
    return (b.x() - a.x()) * (p.y() - a.y()) -
           (b.y() - a.y()) * (p.x() - a.x());
  };
  double a = side(t.a, t.b, p), b = side(t.b, t.c, p), c = side(t.c, t.a, p);
  return (a >= -1e-9 && b >= -1e-9 && c >= -1e-9) ||
         (a <= 1e-9 && b <= 1e-9 && c <= 1e-9);
}
double roughSeconds(const Pass &p, double feed) {
  double total = 0;
  for (size_t i = 1; i < p.poses.size(); ++i)
    total += (p.poses[i].p - p.poses[i - 1].p).norm() /
             (p.poses[i].motion == 4 ? std::min(2., feed) : feed);
  return total;
}
// Preserve every cutting polyline from the proven raster. Only reorder/reverse
// these polylines and replace their links after checking the full swept XY
// footprint against the target and the evolving stock. No shortcut through
// cuts.
Pass optimizeRoughLinks(const std::vector<Operation> &layers, StockGrid &grid,
                        const Options &o, const StockSettings &s) {
  Pass result;
  const double r = o.diameter / 2, safe = s.high.z() + o.clearance;
  size_t work = 0;
  auto clear = [&](const V &a, const V &b, double z) {
    return grid.targetClear(a, b, r, z) &&
           grid.engagement(a, b, r, z) <= s.stepDown + 1e-8;
  };
  for (const auto &layer : layers) {
    std::vector<std::vector<V>> runs;
    bool active = false;
    for (size_t k = 1; k < layer.path.poses.size(); ++k) {
      const auto &p = layer.path.poses[k];
      if (p.motion != 0) {
        active = false;
        continue;
      }
      if (!active) {
        runs.push_back({layer.path.poses[k - 1].p});
        active = true;
      }
      runs.back().push_back(p.p);
    }
    std::vector<bool> done(runs.size(), false);
    for (size_t remaining = runs.size(); remaining; --remaining) {
      size_t best = 0;
      bool reverse = false;
      double cost = 1e100;
      for (size_t i = 0; i < runs.size(); ++i)
        if (!done[i])
          for (bool backwards : {false, true}) {
            need(++work <= 1000000, "Rough link search budget exceeded");
            V start = backwards ? runs[i].back() : runs[i].front();
            double candidate = 0;
            if (!result.poses.empty()) {
              V p = result.poses.back().p, down = p;
              down.z() = start.z();
              bool linked = clear(p, start, start.z());
              candidate = linked
                              ? (p - down).norm() / std::min(2., s.roughFeed) +
                                    (down - start).norm() / s.roughFeed
                              : (2 * safe - p.z() - start.z() +
                                 (p - start).head<2>().norm()) /
                                    s.roughFeed;
            }
            if (candidate < cost) {
              cost = candidate;
              best = i;
              reverse = backwards;
            }
          }
      done[best] = true;
      auto &run = runs[best];
      if (reverse)
        std::reverse(run.begin(), run.end());
      V start = run.front();
      bool linked = !result.poses.empty() &&
                    clear(result.poses.back().p, start, start.z());
      if (linked) {
        V p = result.poses.back().p, down = p;
        down.z() = start.z();
        append(result, at(down), 4, std::min(2., s.roughFeed), o);
        grid.remove(p, down, r, start.z());
        append(result, at(start), 0, s.roughFeed, o);
        grid.remove(down, start, r, start.z());
      } else {
        if (!result.poses.empty()) {
          V p = result.poses.back().p;
          p.z() = safe;
          append(result, at(p), 1, s.roughFeed, o);
        }
        append(result, at(V(start.x(), start.y(), safe)), 2, s.roughFeed, o);
        double top = start.z() + grid.engagement(start, start, r, start.z());
        append(result, at(V(start.x(), start.y(), top + .05)), 3, s.roughFeed,
               o);
        need(clear(start, start, start.z()),
             "Rough entry exceeds protected footprint/depth");
        append(result, at(start), 4, std::min(2., s.roughFeed), o);
        grid.remove(start, start, r, start.z());
      }
      for (size_t k = 1; k < run.size(); ++k) {
        need(clear(run[k - 1], run[k], start.z()),
             "Reordered rough cut exceeds protected footprint/depth");
        append(result, at(run[k]), 0, s.roughFeed, o);
        grid.remove(run[k - 1], run[k], r, start.z());
      }
    }
  }
  if (!result.poses.empty()) {
    V p = result.poses.back().p;
    p.z() = safe;
    append(result, at(p), 1, s.roughFeed, o);
  }
  return result;
}
Pass roughFace(const Scene &scene, int id, const Options &o,
               const StockSettings &stock, const OperationSpec &spec,
               std::string &report) {
  V normal = V::Zero();
  for (const auto &t : scene.mesh)
    if (t.face == id)
      normal += (t.b - t.a).cross(t.c - t.a);
  need(normal.norm() > 1e-8,
       "Selected face needs subdivision for directional roughing");
  normal.normalize();
  need(normal.z() >= -1e-6, "Downward surface requires another setup");
  Q rotation = Q::FromTwoVectors(normal, V::UnitZ());
  Scene local = scene;
  std::vector<Triangle> region;
  for (auto &t : local.mesh) {
    t.a = rotation * t.a;
    t.b = rotation * t.b;
    t.c = rotation * t.c;
    if (t.face == id) {
      V n = (t.b - t.a).cross(t.c - t.a);
      need(n.normalized().z() > .2,
           "Selected face needs subdivision for directional roughing");
      region.push_back(t);
    }
  }
  auto settings = stock;
  settings.low = V::Constant(1e100);
  settings.high = V::Constant(-1e100);
  settings.stepDown = spec.depth;
  settings.allowance = spec.allowance;
  for (double x : {stock.low.x(), stock.high.x()})
    for (double y : {stock.low.y(), stock.high.y()})
      for (double z : {stock.low.z(), stock.high.z()}) {
        V p = rotation * V(x, y, z);
        settings.low = settings.low.cwiseMin(p);
        settings.high = settings.high.cwiseMax(p);
      }
  settings.validate(o);
  StockGrid grid(local, settings, o);
  size_t work = 0;
  size_t selected = 0;
  for (int y = 0; y < grid.ny; ++y)
    for (int x = 0; x < grid.nx; ++x) {
      V p = grid.center(x, y);
      bool inside = false;
      for (auto &t : region) {
        need(++work < 10000000, "Selected face footprint budget exceeded");
        if (projectedInside(t, p)) {
          inside = true;
          break;
        }
      }
      if (!inside)
        grid.target[size_t(y) * grid.nx + x] = settings.high.z();
      else
        ++selected;
    }
  need(selected > 0, "Selected face has no roughable area for this setup");
  auto initial = grid;
  auto layers = buildRoughOperations(grid, o, settings);
  Pass baseline;
  for (const auto &layer : layers)
    for (const auto &p : layer.path.poses)
      append(baseline, p, p.motion,
             p.motion == 4 ? std::min(2., stock.roughFeed) : stock.roughFeed,
             o);
  Pass localPath = baseline;
  std::string reason;
  if (spec.optimize) {
    try {
      auto candidate = optimizeRoughLinks(layers, initial, o, settings);
      need(initial.remaining.size() == grid.remaining.size(),
           "Rough stock size mismatch");
      for (size_t k = 0; k < grid.remaining.size(); ++k)
        need(initial.remaining[k] <= grid.remaining[k] + 1e-7,
             "Optimized path would leave extra stock");
      if (roughSeconds(candidate, stock.roughFeed) <
          roughSeconds(baseline, stock.roughFeed))
        localPath = std::move(candidate);
    } catch (const std::runtime_error &e) {
      reason = e.what();
    }
  }
  auto wrap = [&](const Pass &localPath) {
    Pass path;
    for (auto p : localPath.poses) {
      p.p = rotation.conjugate() * p.p;
      p.n = rotation.conjugate() * p.n;
      p.q = rotation.conjugate() * p.q;
      append(path, p, p.motion, stock.roughFeed, o);
    }
    need(!path.poses.empty(),
         "Selected face has no roughable area for this setup");
    Pass result;
    Pose start = path.poses.front();
    start.p.z() = std::max(
        start.p.z(),
        stock.high.z() + std::hypot(o.length, o.diameter / 2) + o.clearance);
    append(result, start, 2, stock.roughFeed, o);
    for (const auto &p : path.poses)
      append(result, p, p.motion == 2 ? 3 : p.motion, stock.roughFeed, o);
    Pose end = result.poses.back();
    end.p.z() = std::max(end.p.z(), start.p.z());
    append(result, end, 1, stock.roughFeed, o);
    return result;
  };
  auto original = wrap(baseline), result = wrap(localPath);
  bool improved = roughSeconds(result, stock.roughFeed) + 1e-8 <
                  roughSeconds(original, stock.roughFeed);
  if (!improved)
    result = original;
  auto air = [](const Pass &p) {
    double length = 0;
    for (size_t k = 1; k < p.poses.size(); ++k)
      if (p.poses[k].motion != 0 && p.poses[k].motion != 4)
        length += (p.poses[k].p - p.poses[k - 1].p).norm();
    return length;
  };
  std::ostringstream summary;
  summary << "Черновая грани " << id << ": "
          << (improved        ? "переходы сокращены"
              : spec.optimize ? "сохранён исходный маршрут"
                              : "оптимизация отключена")
          << "; время " << roughSeconds(original, stock.roughFeed) << " → "
          << roughSeconds(result, stock.roughFeed) << " с; холостой путь "
          << air(original) << " → " << air(result)
          << " мм. Оценка без ускорений и межоперационных переходов, не время "
             "робота.";
  if (!reason.empty())
    summary << " Причина сохранения: " << reason;
  report = summary.str();
  return result;
}
Pass contourPath(const TopoDS_Face &f, const Options &o,
                 const StockSettings &stock, const OperationSpec &s) {
  need(std::abs(s.bottom - stock.low.z()) < 1e-6,
       "Contour bottom must equal stock Z zero (stock minimum Z)");
  Operation op;
  double r = o.diameter / 2, safe = stock.high.z() + o.clearance;

  need(horizontal(f), "Contour requires a horizontal planar face");
  auto ws = wires(f);
  need(s.wire == 1 && !ws.empty(),
       "External contour uses the outer boundary (contour 1)");
  auto original = sampleWire(ws[0], o);
  std::vector<V> boundary;
  for (double sign : {-1., 1.}) {
    BRepOffsetAPI_MakeOffset offset(ws[0], GeomAbs_Arc);
    offset.Perform(sign * (r + s.allowance + (s.face == 0 ? .03 : .01)));
    if (!offset.IsDone())
      continue;
    std::vector<TopoDS_Wire> candidates;
    for (TopExp_Explorer e(offset.Shape(), TopAbs_WIRE); e.More(); e.Next())
      candidates.push_back(TopoDS::Wire(e.Current()));
    if (candidates.size() != 1)
      continue;
    auto candidate = sampleWire(candidates[0], o);
    if (area(candidate) > area(original) + 1e-6)
      boundary = std::move(candidate);
  }
  need(!boundary.empty(), "Cannot construct one outward offset "
                          "contour; subdivide geometry");
  int layers = std::max(1, int(std::ceil((s.top - s.bottom) / s.depth)));
  need(size_t(layers) * boundary.size() < 80000,
       "Contour layer budget exceeded");
  double total = 0;
  std::vector<double> distance{0};
  for (size_t k = 1; k < boundary.size(); ++k) {
    total += (boundary[k] - boundary[k - 1]).norm();
    distance.push_back(total);
  }
  need(total > 1e-6, "Empty contour");
  V p = boundary.front();
  p.z() = safe;
  append(op.path, at(p), 2, o.feed, o);
  p.z() = s.top;
  append(op.path, at(p), 4, std::min(2., o.feed), o);
  for (int layer = 1; layer <= layers; ++layer) {
    double upper = std::max(s.bottom, s.top - (layer - 1) * s.depth),
           lower = std::max(s.bottom, s.top - layer * s.depth);
    if (!s.spiral) {
      p = boundary.front();
      p.z() = lower;
      append(op.path, at(p), 4, std::min(2., o.feed), o);
    }
    for (size_t k = 0; k < boundary.size(); ++k) {
      p = boundary[k];
      p.z() = s.spiral ? upper - (upper - lower) * distance[k] / total : lower;
      append(op.path, at(p), 0, o.feed, o);
    }
  }
  if (s.spiral)
    for (auto q : boundary) {
      q.z() = s.bottom;
      append(op.path, at(q), 0, o.feed, o);
    }
  p = op.path.poses.back().p;
  p.z() = safe;
  append(op.path, at(p), 1, o.feed, o);

  return op.path;
}
} // namespace
std::string withOperationCatalog(std::string json, const Model &m) {
  auto fs = faces(m);
  std::ostringstream s;
  json.pop_back();
  s << json << ",\"operationCatalog\":[";
  for (size_t i = 0; i < fs.size(); ++i) {
    if (i)
      s << ',';
    BRepAdaptor_Surface surface(fs[i]);
    auto ws = wires(fs[i]);
    double u1, u2, v1, v2;
    BRepTools::UVBounds(fs[i], u1, u2, v1, v2);
    auto point = surface.Value((u1 + u2) / 2, (v1 + v2) / 2);
    s << "{\"face\":" << i + 1
      << ",\"horizontal\":" << (horizontal(fs[i]) ? "true" : "false")
      << ",\"z\":" << point.Z() << ",\"wires\":" << ws.size()
      << ",\"verticalCylinder\":";
    bool cylinder =
        surface.GetType() == GeomAbs_Cylinder &&
        std::abs(surface.Cylinder().Axis().Direction().Z()) > 1 - 1e-8;
    s << (cylinder ? "true" : "false");
    if (cylinder) {
      auto p = evaluate(fs[i], (u1 + u2) / 2, (v1 + v2) / 2, Options{});
      V center = vec(surface.Cylinder().Location());
      double a = surface.Value(u1, v1).Z(), b = surface.Value(u1, v2).Z();
      s << ",\"hole\":"
        << (p.n.dot(V(p.p.x() - center.x(), p.p.y() - center.y(), 0)) < 0
                ? "true"
                : "false")
        << ",\"bottom\":" << std::min(a, b) << ",\"top\":" << std::max(a, b);
    }
    s << '}';
  }
  s << "]}";
  return s.str();
}
std::string operationPlanJson(const Scene &scene,
                              const MachiningProgram &program,
                              const std::vector<OperationSpec> &specs) {
  std::string base = programJson(scene, program);
  base.pop_back();
  std::ostringstream out;
  out.precision(15);
  out << base << ",\"operationTree\":true,\"operationSpecs\":[";
  for (size_t i = 0; i < specs.size(); ++i) {
    if (i)
      out << ',';
    auto &s = specs[i];
    out << "{\"kind\":\"" << s.kind << "\",\"face\":" << s.face
        << ",\"wire\":" << s.wire << ",\"top\":" << s.top
        << ",\"bottom\":" << s.bottom << ",\"depth\":" << s.depth
        << ",\"allowance\":" << s.allowance
        << ",\"optimize\":" << (s.optimize ? "true" : "false")
        << ",\"confirmed\":true,\"spiral\":" << (s.spiral ? "true" : "false")
        << '}';
  }
  out << "]}";
  return out.str();
}
MachiningProgram planOperations(const Model &m, Scene &scene, const Options &o,
                                const StockSettings &stock,
                                const std::vector<OperationSpec> &specs) {
  o.validate();
  stock.validate(o);
  need(o.normalSign == 1, "Operation tree uses outward normals");
  need(stock.plungeConfirmed,
       "Confirm a centre-cutting tool and permitted axial entry");
  need(!specs.empty() && specs.size() <= 32,
       "Confirm between 1 and 32 operations");
  int solids = 0;
  for (TopExp_Explorer e(m.shape, TopAbs_SOLID); e.More(); e.Next())
    ++solids;
  need(solids == 1, "Operations require one closed STEP solid");
  auto fs = faces(m);
  StockGrid grid(scene, stock, o);
  ProgramSettings cfg;
  cfg.stock = stock;
  MachiningProgram out;
  out.settings = cfg;
  out.nx = grid.nx;
  out.ny = grid.ny;
  out.dx = grid.dx;
  out.dy = grid.dy;
  out.target = grid.target;
  out.roughRemaining = grid.remaining;
  out.excess = grid.excessVolume(0);
  const double r = o.diameter / 2, depth = stock.high.z() - stock.low.z(),
               safe = stock.high.z() + std::hypot(o.length, r) + o.clearance;
  bool roughDone = false;
  std::set<int> roughFaces;
  for (size_t index = 0; index < specs.size(); ++index) {
    const auto &s = specs[index];
    try {
      need(s.confirmed, "Select geometry and confirm this operation");
      need(std::isfinite(s.top) && std::isfinite(s.bottom) &&
               std::isfinite(s.depth) && std::isfinite(s.allowance),
           "Non-finite operation parameter");
      need(s.depth >= .1 && s.depth <= 5 && s.allowance >= 0 &&
               s.allowance <= 5,
           "Invalid depth / allowance");
      need(s.top >= s.bottom && s.top <= stock.high.z() + 1e-6 &&
               s.bottom >= stock.low.z() - 1e-6,
           "Operation heights must lie within stock");
      Operation op;
      op.name = std::to_string(index + 1) + ". " + s.kind;
      op.face = s.face;
      op.stage = (s.kind == "rough" || s.kind == "face_rough") ? 1 : 2;
      if (s.kind == "rough") {
        need(s.face == 0, "Bulk roughing geometry must be the whole solid");
        auto settings = stock;
        settings.stepDown = s.depth;
        settings.allowance = s.allowance;
        settings.validate(o);
        grid.settings = settings;
        auto layers = buildRoughOperations(grid, o, settings);
        for (auto &layer : layers)
          for (auto &p : layer.path.poses)
            append(op.path, p, p.motion, settings.roughFeed, o);
        roughDone = true;
        out.roughRemaining = grid.remaining;
        out.excess = grid.excessVolume(0);
      } else if (s.kind == "contour") {
        need(s.face >= 0 && s.face <= int(fs.size()),
             "Select a valid face from the current model");
        auto outline = s.face ? fs[size_t(s.face - 1)] : projectedPart(scene);
        op.path = contourPath(outline, o, stock, s);
        if (s.face)
          out.includedFaces.push_back(s.face);
      } else {
        need(s.face > 0 && s.face <= int(fs.size()),
             "Select a valid face from the current model");
        auto f = fs[size_t(s.face - 1)];
        BRepAdaptor_Surface surface(f);
        std::string strategy = s.kind;
        bool hole = false;
        if (surface.GetType() == GeomAbs_Cylinder &&
            std::abs(surface.Cylinder().Axis().Direction().Z()) > 1 - 1e-8) {
          double a, b, c, d;
          BRepTools::UVBounds(f, a, b, c, d);
          auto p = evaluate(f, (a + b) / 2, (c + d) / 2, o);
          V center = vec(surface.Cylinder().Location());
          hole = p.n.dot(V(p.p.x() - center.x(), p.p.y() - center.y(), 0)) < 0;
        }
        if (strategy == "face_finish")
          strategy = hole ? "bore" : horizontal(f) ? "face" : "surface";
        if (strategy == "face_rough" && hole)
          strategy = "bore";
        if (strategy == "face_rough") {
          std::string report;
          op.path = roughFace(scene, s.face, o, stock, s, report);
          out.notices.push_back(report);
          roughFaces.insert(s.face);
        } else if (strategy == "surface") {
          need(roughDone || roughFaces.count(s.face),
               "Normal finishing requires roughing this face or the whole "
               "solid earlier in this tree");
          need(s.allowance == 0,
               "Normal finishing currently requires zero allowance");
          op.path = surfacePath(f, o, stock);
        } else if (strategy == "face" || strategy == "pocket") {
          need(horizontal(f),
               "Face / pocket operation requires a horizontal planar floor");
          Options local = o;
          local.diameter += 2 * s.allowance;
          local.spacing = std::min(o.spacing, stock.stepOver);
          auto rows = raster(f, local);
          need(!rows.empty(), "Tool does not fit selected floor");
          double floor = rows.front().poses.front().p.z() + s.allowance;
          need(std::abs(s.bottom - floor) < 1e-5,
               "Bottom must equal selected floor Z plus allowance");
          for (auto &row : rows)
            for (auto &p : row.poses)
              need(p.n.z() > .999, "Select an upward floor face");
          int layers =
              std::max(1, int(std::ceil((s.top - s.bottom) / s.depth)));
          need(layers <= 300, "Layer budget exceeded");
          for (int layer = 1; layer <= layers; ++layer)
            for (auto &row : rows) {
              double z = std::max(s.bottom, s.top - layer * s.depth);
              V a = row.poses.front().p;
              a.z() = safe;
              append(op.path, at(a), 2, o.feed, o);
              a.z() = z;
              append(op.path, at(a), 4, std::min(2., o.feed), o);
              for (auto &p : row.poses) {
                V q = p.p;
                q.z() = z;
                append(op.path, at(q), 0, o.feed, o);
              }
              a = op.path.poses.back().p;
              a.z() = safe;
              append(op.path, at(a), 1, o.feed, o);
            }
        } else if (strategy == "bore") {
          need(surface.GetType() == GeomAbs_Cylinder,
               "Hole operation requires a cylindrical wall");
          auto cylinder = surface.Cylinder();
          need(std::abs(cylinder.Axis().Direction().Z()) > 1 - 1e-8,
               "Hole axis must be perpendicular to base");
          double u1, u2, v1, v2;
          BRepTools::UVBounds(f, u1, u2, v1, v2);
          need(std::abs(u2 - u1 - 2 * pi) < 1e-5,
               "Select a complete cylindrical hole wall");
          auto test = evaluate(f, (u1 + u2) / 2, (v1 + v2) / 2, o);
          V center = vec(cylinder.Location());
          need(test.n.dot(
                   V(test.p.x() - center.x(), test.p.y() - center.y(), 0)) < 0,
               "Selected cylinder is a boss, not an internal hole");
          double z1 = surface.Value(u1, v1).Z(), z2 = surface.Value(u1, v2).Z();
          need(s.bottom >= std::min(z1, z2) - 1e-5 &&
                   s.top <= std::max(z1, z2) + 1e-5,
               "Hole heights exceed cylindrical wall");
          double radius = cylinder.Radius() - r - s.allowance - .03;
          need(radius > .05, "Tool is too large for hole");
          int rings = std::max(
                  1, int(std::ceil(radius / std::min(stock.stepOver, r)))),
              layers =
                  std::max(1, int(std::ceil((s.top - s.bottom) / s.depth)));
          int count = std::max(
              24, int(std::ceil(
                      2 * pi * radius /
                      std::min(o.step, std::sqrt(8 * radius *
                                                 std::min(o.chord, .01))))));
          need(size_t(rings) * layers * count <= 80000,
               "Hole path budget exceeded");
          // Clear the core and concentric rings at each level before wall
          // helix.
          V p(center.x(), center.y(), safe);
          append(op.path, at(p), 2, o.feed, o);
          for (int layer = 1; layer <= layers; ++layer) {
            double z = std::max(s.bottom, s.top - layer * s.depth);
            p = V(center.x(), center.y(), z);
            append(op.path, at(p), 4, std::min(2., o.feed), o);
            for (int ring = 1; ring <= rings; ++ring)
              for (int k = 0; k <= count; ++k) {
                double a = 2 * pi * k / count, rr = radius * ring / rings;
                append(op.path,
                       at(V(center.x() + rr * std::cos(a),
                            center.y() + rr * std::sin(a), z)),
                       0, o.feed, o);
              }
            append(op.path, at(V(center.x(), center.y(), z)), 0, o.feed, o);
          }
          p = op.path.poses.back().p;
          p.z() = safe;
          append(op.path, at(p), 1, o.feed, o);
        } else
          throw std::runtime_error("Unknown operation type");
        if (s.kind == "face_rough")
          roughFaces.insert(s.face);
        out.includedFaces.push_back(s.face);
      }
      need(!op.path.poses.empty(),
           "No toolpath for this operation / no removable stock");
      // Height-field planning state is updated only for vertical cutting moves.
      // The independent voxel replay below starts from the original stock.
      for (size_t k = 1; k < op.path.poses.size(); ++k) {
        auto &p = op.path.poses[k];
        auto &a = op.path.poses[k - 1];
        if ((p.motion == 0 || p.motion == 4) && p.n.z() > .999 &&
            a.n.z() > .999)
          grid.remove(a.p, p.p, r, std::max(a.p.z(), p.p.z()));
      }
      if (op.stage == 1) {
        out.roughRemaining = grid.remaining;
        out.excess = grid.excessVolume(0);
      }
      out.operations.push_back(std::move(op));
    } catch (const std::exception &e) {
      throw std::runtime_error("Operation " + std::to_string(index + 1) + " (" +
                               s.kind + "): " + e.what());
    }
  }
  // Never sort operations: explicit user order determines all linking moves.
  for (size_t i = 0; i < out.operations.size(); ++i) {
    auto &op = out.operations[i];
    size_t old = out.trajectory.poses.size();
    if (!out.trajectory.poses.empty()) {
      Pose p = out.trajectory.poses.back();
      p.p.z() = std::max(safe, p.p.z());
      append(out.trajectory, p, 1, o.feed, o);
      p.q = Q::Identity();
      p.n = V::UnitZ();
      append(out.trajectory, p, 2, o.feed, o);
      Pose q = op.path.poses.front();
      q.p.z() = std::max(safe, q.p.z());
      append(out.trajectory, q, 2, o.feed, o);
    }
    const double operationFeed = op.stage == 1 ? stock.roughFeed : o.feed;
    for (auto &p : op.path.poses)
      append(out.trajectory, p, p.motion,
             p.motion == 4 ? std::min(2., operationFeed) : operationFeed, o);
    while (old++ < out.trajectory.poses.size()) {
      out.stage.push_back(op.stage);
      out.operation.push_back(int(i));
    }
  }
  grid.remaining.assign(grid.remaining.size(), stock.high.z());
  out.notices.push_back(
      "Hole radial mesh reserve: 0.03 mm; selected-face contour reserve: 0.01 "
      "mm; projected part contour reserve: 0.03 mm. "
      "These are explicit unremoved allowances, not hidden material removal.");
  out.notices.push_back(
      "User operation order. Stock replay starts from the original block; no "
      "hidden roughing or face selection. RoughRemaining is the planning "
      "height envelope after selected operations (voxel replay for local face "
      "roughing).");
  // Replay every operation in user order against the original stock. Voxels
  // are centre samples (bounded resolution), with CAD solid classification as
  // an immutable target mask. Air moves never silently remove material.
  const int nz = int(std::ceil(depth / cfg.stock.cell));
  need(size_t(grid.nx) * grid.ny * nz <= 1000000,
       "Stock replay exceeds one million voxels");
  const double dz = depth / nz, voxelVolume = grid.dx * grid.dy * dz;
  std::vector<unsigned char> material(size_t(grid.nx) * grid.ny * nz, 0),
      targetMask(material.size(), 0);
  for (int z = 0; z < nz; ++z)
    for (size_t c = 0; c < grid.remaining.size(); ++c)
      material[size_t(z) * grid.remaining.size() + c] =
          cfg.stock.low.z() + (z + .5) * dz < grid.remaining[c] - 1e-8;
  BRepClass3d_SolidClassifier classifier(
      TopExp_Explorer(m.shape, TopAbs_SOLID).Current());
  size_t replayWork = 0;
  int replayStage = 2;
  auto replay = [&](const Pose &p, bool cutting) {
    V top = p.p + p.q * V(0, 0, o.length),
      lo = p.p.cwiseMin(top) - V::Constant(r),
      hi = p.p.cwiseMax(top) + V::Constant(r);
    int x0 = std::clamp(int(std::floor((lo.x() - cfg.stock.low.x()) / grid.dx)),
                        0, grid.nx - 1),
        x1 = std::clamp(int(std::floor((hi.x() - cfg.stock.low.x()) / grid.dx)),
                        0, grid.nx - 1);
    int y0 = std::clamp(int(std::floor((lo.y() - cfg.stock.low.y()) / grid.dy)),
                        0, grid.ny - 1),
        y1 = std::clamp(int(std::floor((hi.y() - cfg.stock.low.y()) / grid.dy)),
                        0, grid.ny - 1);
    int z0 = std::clamp(int(std::floor((lo.z() - cfg.stock.low.z()) / dz)), 0,
                        nz - 1),
        z1 = std::clamp(int(std::floor((hi.z() - cfg.stock.low.z()) / dz)), 0,
                        nz - 1);
    bool hit = false;
    for (int z = z0; z <= z1; ++z)
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
          need(++replayWork <= 30000000, "Stock replay work budget exceeded");
          size_t col = size_t(y) * grid.nx + x,
                 index = size_t(z) * grid.remaining.size() + col;
          if (!material[index])
            continue;
          V v = grid.center(x, y);
          v.z() = cfg.stock.low.z() + (z + .5) * dz;
          V local = p.q.conjugate() * (v - p.p);
          if (local.z() <= .001 || local.z() >= o.length - .001 ||
              local.head<2>().norm() >= r - .001)
            continue;
          if (!cutting) {
            hit = true;
            continue;
          }
          if (!targetMask[index]) {
            classifier.Perform(gp_Pnt(v.x(), v.y(), v.z()), 1e-7);
            targetMask[index] = (classifier.State() == TopAbs_IN ||
                                 classifier.State() == TopAbs_ON)
                                    ? 2
                                    : 1;
          }
          if (targetMask[index] == 2) {
            hit = true;
            continue;
          }
          material[index] = 0;
          if (replayStage == 1)
            out.removed += voxelVolume;
          else
            out.finishRemoved += voxelVolume;
        }
    return hit;
  };
  const bool selectedRoughing =
      std::any_of(specs.begin(), specs.end(), [](const OperationSpec &s) {
        return s.kind == "face_rough";
      });
  for (size_t k = 0; k < out.trajectory.poses.size(); ++k) {
    replayStage = out.stage[k];
    auto &p = out.trajectory.poses[k];
    auto a = k ? out.trajectory.poses[k - 1] : p;
    int samples = std::max(
        1, int(std::ceil(std::max((p.p - a.p).norm() /
                                      std::min(cfg.stock.cell / 2, r / 4),
                                  a.q.angularDistance(p.q) / (pi / 180)))));
    for (int j = 1; j <= samples; ++j) {
      double t = double(j) / samples;
      Pose v = p;
      v.p = a.p + t * (p.p - a.p);
      v.q = a.q.slerp(t, p.q);
      bool hit = replay(v, p.motion == 0 || p.motion == 4);
      p.sweepCollision |= hit;
      out.stockCollisions |= hit;
    }
    // Local face roughing may tilt the tool: a vertical height-field planning
    // update cannot represent that removal. Snapshot the actual replay instead.
    if (selectedRoughing && replayStage == 1 &&
        (k + 1 == out.stage.size() || out.stage[k + 1] != 1)) {
      out.roughRemaining.assign(grid.remaining.size(), cfg.stock.low.z());
      out.excess = 0;
      for (int z = 0; z < nz; ++z)
        for (size_t c = 0; c < grid.remaining.size(); ++c)
          if (material[size_t(z) * grid.remaining.size() + c]) {
            out.roughRemaining[c] = std::max(
                out.roughRemaining[c],
                std::min(cfg.stock.high.z(), cfg.stock.low.z() + (z + 1) * dz));
            if (cfg.stock.low.z() + (z + .5) * dz > grid.target[c] + .03)
              out.excess += voxelVolume;
          }
    }
  }
  out.finalRemaining.assign(grid.remaining.size(), cfg.stock.low.z());
  for (int z = 0; z < nz; ++z)
    for (size_t c = 0; c < grid.remaining.size(); ++c)
      if (material[size_t(z) * grid.remaining.size() + c]) {
        out.finalRemaining[c] = std::max(
            out.finalRemaining[c],
            std::min(grid.remaining[c], cfg.stock.low.z() + (z + 1) * dz));
        if (cfg.stock.low.z() + (z + .5) * dz > grid.target[c] + .03)
          out.finalExcess += voxelVolume;
      }
  out.notices.push_back(
      "Operation stock replay samples voxel centres; removed volume and "
      "remaining envelope are approximate at the chosen cell size.");
  // Independent validation of the composed path, including entries, transfers,
  // interpolation and finishing, against every triangle of the actual model.
  size_t work = 0;
  auto collides = [&](const Pose &p) {
    ++scene.collisionQueries;
    double reach = std::hypot(o.length, r);
    for (const auto &t : scene.mesh) {
      need(++work <= 150000000, "Program collision budget exceeded");
      V lo = t.a.cwiseMin(t.b).cwiseMin(t.c),
        hi = t.a.cwiseMax(t.b).cwiseMax(t.c);
      if ((p.p.cwiseMax(lo).cwiseMin(hi) - p.p).norm() > reach)
        continue;
      if (triangleCollision(t, p, r, o.length))
        return true;
    }
    return false;
  };
  for (size_t k = 0; k < out.trajectory.poses.size(); ++k) {
    auto &p = out.trajectory.poses[k];
    p.collision = collides(p);
    if (k) {
      const auto &a = out.trajectory.poses[k - 1];
      int n = std::max(
          2,
          int(std::ceil(std::max((p.p - a.p).norm() / std::min(.25, r / 2),
                                 a.q.angularDistance(p.q) / (2 * pi / 180)))));
      for (int j = 1; j < n; ++j) {
        double t = double(j) / n;
        Pose v = p;
        v.p = a.p + t * (p.p - a.p);
        v.q = a.q.slerp(t, p.q);
        p.sweepCollision |= collides(v);
      }
    }
    out.collisions |= p.collision || p.sweepCollision;
  }
  out.trajectory.segmentCollision = out.collisions;
  // Operation previews are slices of the final composed program, so quaternion,
  // interpolation, timing and collision flags cannot diverge between views.
  size_t begin = 0;
  for (size_t i = 0; i < out.operations.size(); ++i) {
    size_t end = begin;
    while (end < out.operation.size() && out.operation[end] == int(i))
      ++end;
    Pass slice;
    const size_t first = begin ? begin - 1 : begin;
    const double startTime = out.trajectory.poses[first].t;
    for (size_t k = first; k < end; ++k) {
      Pose p = out.trajectory.poses[k];
      p.t -= startTime;
      slice.segmentCollision |= p.collision || p.sweepCollision;
      slice.poses.push_back(p);
    }
    out.operations[i].path = std::move(slice);
    if (out.operations[i].face > 0) {
      auto &face = scene.faces[size_t(out.operations[i].face - 1)];
      face.trajectory = out.operations[i].path;
      face.passes = {out.operations[i].path};
      face.error.clear();
      face.connectionError.clear();
    }
    begin = end;
  }
  out.notices.push_back(
      "Offline diagnostic program: no robot IK, holder/fixture validation, "
      "force sensor or ABB motion authorization.");
  return out;
}
} // namespace milling
