#include "program.h"
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace milling {
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
std::string quoted(const std::string &s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    if (c >= 32 && c != '<' && c != '>')
      out += char(c);
  }
  return out + '"';
}
void vector(std::ostream &o, const V &p) {
  o << '[' << p.x() << ',' << p.y() << ',' << p.z() << ']';
}
void poses(std::ostream &o, const Pass &p) {
  o << "{\"sweepCollision\":" << (p.segmentCollision ? "true" : "false")
    << ",\"poses\":[";
  bool first = true;
  for (const auto &v : p.poses) {
    if (!first)
      o << ',';
    first = false;
    o << "{\"p\":";
    vector(o, v.p);
    o << ",\"n\":";
    vector(o, v.n);
    o << ",\"q\":[" << v.q.w() << ',' << v.q.x() << ',' << v.q.y() << ','
      << v.q.z() << "],\"t\":" << v.t << ",\"motion\":" << v.motion
      << ",\"collision\":" << (v.collision ? "true" : "false")
      << ",\"sweepCollision\":" << (v.sweepCollision ? "true" : "false") << '}';
  }
  o << "]}";
}
template <class T> void array(std::ostream &o, const std::vector<T> &v) {
  o << '[';
  for (size_t i = 0; i < v.size(); ++i) {
    if (i)
      o << ',';
    o << v[i];
  }
  o << ']';
}
} // namespace
Model programDemo() {
  auto base = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 20, 20, 2).Shape();
  auto step = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 2), 10, 20, 2).Shape();
  BRepAlgoAPI_Fuse fuse(base, step);
  fuse.Build();
  need(fuse.IsDone(), "Demo solid fuse failed");
  return {fuse.Shape(), "mm (synthetic stepped solid)"};
}
std::vector<Operation> buildRoughOperations(StockGrid &grid, const Options &o,
                                            const StockSettings &stock) {
  std::vector<Operation> operations;
  // Roughing never tilts: its cylinder extends upward, so TCP clearance over
  // the block is enough. The sphere reserve is only needed for tilted
  // transfers.
  const double r = o.diameter / 2, safeZ = stock.high.z() + o.clearance;
  const double depth = stock.high.z() - stock.low.z();
  int layers = int(std::ceil(depth / stock.stepDown));
  need(layers <= 300, "Layer budget exceeded");
  int rows =
      int(std::ceil((stock.high.y() - stock.low.y() + 2 * r) / stock.stepOver));
  int columns = int(std::ceil((stock.high.x() - stock.low.x() + 2 * r) /
                              std::min(o.step, stock.cell)));
  need(size_t(layers) * (rows + 1) * columns <= 500000,
       "Rough scan budget exceeded; increase grid/step/depth");
  for (int layer = 1; layer <= layers; ++layer) {
    double z = std::max(stock.low.z(), stock.high.z() - layer * stock.stepDown);
    Operation op;
    op.stage = 1;
    op.level = z;
    op.name = "Rough layer " + std::to_string(layer);
    double before = grid.removedVolume;
    auto run = [&](V a, V b) {
      bool linked = false;
      if (!op.path.poses.empty()) {
        V p = op.path.poses.back().p;
        linked = grid.targetClear(p, a, r, z) &&
                 grid.engagement(p, a, r, z) <= stock.stepDown + 1e-8;
        if (linked) {
          append(op.path, at(a), 0, stock.roughFeed, o);
          grid.remove(p, a, r, z);
        } else
          append(op.path, at(V(p.x(), p.y(), safeZ)), 1, stock.roughFeed, o);
      }
      if (!linked) {
        append(op.path, at(V(a.x(), a.y(), safeZ)), 2, stock.roughFeed, o);
        double top = z + grid.engagement(a, a, r, z);
        append(op.path, at(V(a.x(), a.y(), top + .05)), 3, stock.roughFeed, o);
        append(op.path, at(a), 4, std::min(stock.roughFeed, 2.), o);
        grid.remove(a, a, r, z);
      }
      append(op.path, at(b), 0, stock.roughFeed, o);
      grid.remove(a, b, r, z);
    };
    for (int row = 0; row <= rows; ++row) {
      double y = stock.low.y() - r +
                 (stock.high.y() - stock.low.y() + 2 * r) * row / rows;
      auto point = [&](int col) {
        double fraction = double(col) / columns;
        if (row % 2)
          fraction = 1 - fraction;
        return V(stock.low.x() - r +
                     (stock.high.x() - stock.low.x() + 2 * r) * fraction,
                 y, z);
      };
      bool active = false;
      V start = point(0), end = start;
      for (int col = 0; col < columns; ++col) {
        V a = point(col), b = point(col + 1);
        bool allowed = grid.targetClear(a, b, r, z) &&
                       grid.engagement(a, b, r, z) > 1e-8 &&
                       grid.engagement(a, b, r, z) <= stock.stepDown + 1e-8;
        if (allowed) {
          if (!active)
            start = a;
          active = true;
          end = b;
        } else if (active) {
          run(start, end);
          active = false;
        }
      }
      if (active)
        run(start, end);
    }
    if (!op.path.poses.empty()) {
      V p = op.path.poses.back().p;
      append(op.path, at(V(p.x(), p.y(), safeZ)), 1, stock.roughFeed, o);
      op.removed = grid.removedVolume - before;
      operations.push_back(std::move(op));
    }
  }
  return operations;
}
MachiningProgram planProgram(const Model &m, Scene &scene, const Options &o,
                             const ProgramSettings &cfg) {
  cfg.stock.validate(o);
  need(o.normalSign == 1,
       "Stock program requires outward +normal in this setup");
  need(cfg.stock.plungeConfirmed, "Confirm a centre-cutting tool and permitted "
                                  "axial entry before roughing");
  need(cfg.mode != ProgramSettings::Finish,
       "Use face finishing for a pre-machined part; stock program supports "
       "rough or both");
  int solids = 0;
  for (TopExp_Explorer e(m.shape, TopAbs_SOLID); e.More(); e.Next())
    ++solids;
  need(solids == 1, "Stock program requires one closed STEP solid, not an open "
                    "surface or assembly");
  need(scene.faces.size() <= (cfg.wholeModel ? 128 : 32),
       "Program face budget exceeded; simplify the CAD model");
  StockGrid grid(scene, cfg.stock, o);
  MachiningProgram out;
  out.settings = cfg;
  out.nx = grid.nx;
  out.ny = grid.ny;
  out.dx = grid.dx;
  out.dy = grid.dy;
  out.target = grid.target;
  const double r = o.diameter / 2,
               depth = cfg.stock.high.z() - cfg.stock.low.z();
  out.operations = buildRoughOperations(grid, o, cfg.stock);
  out.roughRemaining = grid.remaining;
  out.removed = grid.removedVolume;
  out.excess = grid.excessVolume(cfg.stock.allowance + .03);
  out.notices.push_back("One setup from +Z. Stock volumes are grid estimates, "
                        "not certified swept-volume results.");
  out.notices.push_back(
      "Allowance includes radial/vertical protection plus a 0.03 mm mesh "
      "reserve. Remaining stock is reported, never assumed removed.");
  if (cfg.mode == ProgramSettings::Both) {
    std::vector<TopoDS_Face> cadFaces;
    for (TopExp_Explorer e(m.shape, TopAbs_FACE); e.More(); e.Next())
      cadFaces.push_back(TopoDS::Face(e.Current()));
    auto upward = [&](int id) {
      bool has = false;
      for (const auto &t : scene.mesh)
        if (t.face == id) {
          V n = (t.b - t.a).cross(t.c - t.a);
          if (n.norm() > 1e-12) {
            has = true;
            if (n.normalized().z() <= .2)
              return false;
          }
        }
      return has;
    };
    size_t candidateWork = 0;
    auto clearPath = [&](const Pass &path) {
      auto clearPose = [&](const Pose &p) {
        ++scene.collisionQueries;
        for (const auto &t : scene.mesh) {
          need(++candidateWork <= 150000000,
               "Automatic access check budget exceeded");
          if (triangleCollision(t, p, r, o.length))
            return false;
        }
        return true;
      };
      for (size_t k = 0; k < path.poses.size(); ++k) {
        const auto &p = path.poses[k];
        if (!clearPose(p))
          return false;
        if (!k)
          continue;
        const auto &a = path.poses[k - 1];
        int n = std::max(2, int(std::ceil(std::max(
                                (p.p - a.p).norm() / std::min(.25, r / 2),
                                a.q.angularDistance(p.q) / (2 * pi / 180)))));
        for (int j = 1; j < n; ++j) {
          double t = double(j) / n;
          Pose v = p;
          v.p = a.p + t * (p.p - a.p);
          v.q = a.q.slerp(t, p.q);
          if (!clearPose(v))
            return false;
        }
      }
      return true;
    };
    auto exclude = [&](int id, const std::string &reason) {
      out.excludedFaces.push_back(id);
      out.excludedReasons.push_back(reason);
      scene.faces[size_t(id - 1)].error = reason;
    };
    std::vector<int> chosen = cfg.finishFaces;
    if (cfg.wholeModel) {
      chosen.clear();
      for (const auto &f : scene.faces)
        chosen.push_back(f.id);
    } else if (chosen.empty()) {
      for (const auto &f : scene.faces)
        if (upward(f.id))
          chosen.push_back(f.id);
    }
    for (const auto &f : scene.faces)
      if (std::find(chosen.begin(), chosen.end(), f.id) == chosen.end())
        exclude(f.id, "Outside selected top setup");
    if (!cfg.wholeModel)
      need(!chosen.empty(), "No upward finishing faces in this setup");
    std::sort(chosen.begin(), chosen.end());
    need(std::adjacent_find(chosen.begin(), chosen.end()) == chosen.end(),
         "Duplicate finish face");
    for (int id : chosen) {
      need(id >= 1 && id <= int(scene.faces.size()),
           "Invalid finishing face ID");
      try {
        if (!cfg.wholeModel)
          need(upward(id),
               "Selected finishing face is not upward in this setup");
        V average = V::Zero();
        if (cfg.wholeModel) {
          for (const auto &t : scene.mesh)
            if (t.face == id) {
              V n = (t.b - t.a).cross(t.c - t.a);
              if (n.norm() <= 1e-12)
                continue;
              need(n.normalized().z() >= -1e-6,
                   "Downward surface requires another setup");
              average += n;
            }
          need(average.norm() > 1e-10,
               "Surface normals require subdivision into accessible patches");
          average.normalize();
          for (const auto &t : scene.mesh)
            if (t.face == id) {
              V n = (t.b - t.a).cross(t.c - t.a);
              if (n.norm() > 1e-12)
                need(n.normalized().dot(average) > .2,
                     "Surface normals require subdivision into accessible "
                     "patches");
            }
        }
        Options one = o;
        one.face = id;
        Face f;
        if (cfg.wholeModel) {
          // Evaluate the actual B-rep directly, avoiding re-meshing the whole
          // part for every surface. The composed program is validated below.
          f = scene.faces[size_t(id - 1)];
          f.error.clear();
          f.connectionError.clear();
          f.passes = raster(cadFaces[size_t(id - 1)], one);
        } else {
          auto calculated = analyze(m, one);
          f = calculated.faces[size_t(id - 1)];
          need(f.error.empty() && f.connectionError.empty(),
               "A requested finish face cannot be calculated in this setup");
        }
        need(!f.passes.empty(), "No usable passes for this tool and surface");
        if (cfg.wholeModel)
          for (const auto &pass : f.passes)
            for (const auto &pose : pass.poses) {
              V axis = pose.q * V::UnitZ();
              need(axis.z() >= -1e-6,
                   "Downward surface requires another setup");
              need(axis.dot(average) > .2,
                   "Surface normals require subdivision into accessible "
                   "patches");
            }
        Operation op;
        op.stage = 2;
        op.face = id;
        op.name = "Finish face " + std::to_string(id);
        if (!cfg.wholeModel || upward(id)) {
          op.path = connectPasses(f.passes, cfg.stock.high.z(), o,
                                  &cadFaces[size_t(id - 1)]);
        } else {
          // Work in a rigid local frame whose +Z is the average outward normal.
          // Its safe plane lies beyond ALL stock corners plus the entire tool
          // radius bound. UV and normal geometry are preserved by this
          // rotation.
          Q rotation = Q::FromTwoVectors(average, V::UnitZ());
          auto matrix = rotation.toRotationMatrix();
          gp_Trsf transform;
          transform.SetValues(matrix(0, 0), matrix(0, 1), matrix(0, 2), 0,
                              matrix(1, 0), matrix(1, 1), matrix(1, 2), 0,
                              matrix(2, 0), matrix(2, 1), matrix(2, 2), 0);
          auto surface =
              TopoDS::Face(BRepBuilderAPI_Transform(cadFaces[size_t(id - 1)],
                                                    transform, true)
                               .Shape());
          auto passes = f.passes;
          for (auto &pass : passes)
            for (auto &pose : pass.poses) {
              need((pose.q * V::UnitZ()).z() >= -1e-6,
                   "Downward surface requires another setup");
              pose.p = rotation * pose.p;
              pose.n = rotation * pose.n;
              pose.q = rotation * pose.q;
            }
          double top = -1e100;
          for (double x : {cfg.stock.low.x(), cfg.stock.high.x()})
            for (double y : {cfg.stock.low.y(), cfg.stock.high.y()})
              for (double z : {cfg.stock.low.z(), cfg.stock.high.z()})
                top = std::max(top, (rotation * V(x, y, z)).z());
          auto path = connectPasses(passes, top, o, &surface);
          for (auto &pose : path.poses) {
            pose.p = rotation.conjugate() * pose.p;
            pose.n = rotation.conjugate() * pose.n;
            pose.q = rotation.conjugate() * pose.q;
          }
          double safeZ =
              cfg.stock.high.z() + std::hypot(o.length, r) + o.clearance;
          Pose start = path.poses.front();
          start.p.z() = std::max(start.p.z(), safeZ);
          append(op.path, start, 2, o.feed, o);
          for (size_t k = 0; k < path.poses.size(); ++k)
            append(op.path, path.poses[k], k ? path.poses[k].motion : 3, o.feed,
                   o);
          Pose end = op.path.poses.back();
          end.p.z() = std::max(end.p.z(), safeZ);
          append(op.path, end, 1, o.feed, o);
        }
        for (auto &p : op.path.poses)
          if (p.motion == 3)
            p.motion = 4;
        if (cfg.wholeModel)
          need(clearPath(op.path),
               "Tool or access path intersects the target model");
        f.trajectory = op.path;
        scene.faces[size_t(id - 1)] = std::move(f);
        out.includedFaces.push_back(id);
        out.operations.push_back(std::move(op));
      } catch (const Standard_Failure &e) {
        if (!cfg.wholeModel)
          throw;
        exclude(id, e.GetMessageString() ? e.GetMessageString() : "CAD error");
      } catch (const std::runtime_error &e) {
        if (!cfg.wholeModel)
          throw;
        std::string reason = e.what();
        // Resource exhaustion is a failure of the job, never hidden as a skip.
        if (reason.find("budget") != std::string::npos)
          throw;
        exclude(id, reason);
      }
    }
    out.notices.push_back(
        "Automatic coverage is reported per surface. Downward faces, narrow "
        "regions and inaccessible patches may require another setup or tool.");
  }
  need(!out.operations.empty(),
       "No removable stock / no operations for these settings");
  for (size_t i = 0; i < out.operations.size(); ++i) {
    auto &op = out.operations[i];
    for (size_t k = 0; k < op.path.poses.size(); ++k) {
      size_t old = out.trajectory.poses.size();
      if (!k && !out.trajectory.poses.empty() && op.stage == 2) {
        Pose raised = out.trajectory.poses.back();
        raised.p.z() = std::max(raised.p.z(), op.path.poses.front().p.z());
        append(out.trajectory, raised, 1, o.feed, o);
        // Opposite side normals have no unique minimum rotation. Route the
        // orientation through +Z above the entire stock, never at contact.
        Pose neutral = out.trajectory.poses.back();
        neutral.q = Q::Identity();
        neutral.n = V::UnitZ();
        append(out.trajectory, neutral, 2, o.feed, o);
      }
      append(out.trajectory, op.path.poses[k], k ? op.path.poses[k].motion : 2,
             op.path.poses[k].motion == 4
                 ? std::min(2., op.stage == 1 ? cfg.stock.roughFeed : o.feed)
                 : (op.stage == 1 ? cfg.stock.roughFeed : o.feed),
             o);
      for (size_t j = old; j < out.trajectory.poses.size(); ++j) {
        out.stage.push_back(op.stage);
        out.operation.push_back(int(i));
      }
    }
  }
  // Replay finishing against the material actually left by roughing. Voxels
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
          out.finishRemoved += voxelVolume;
        }
    return hit;
  };
  for (size_t k = 0; k < out.trajectory.poses.size(); ++k) {
    if (out.stage[k] != 2)
      continue;
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
      "Finishing stock replay samples voxel centres; removed volume and "
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
    if (out.operations[i].stage == 2)
      scene.faces[size_t(out.operations[i].face - 1)].trajectory =
          out.operations[i].path;
    begin = end;
  }
  out.notices.push_back(
      "Offline diagnostic program: no robot IK, holder/fixture validation, "
      "force sensor or ABB motion authorization.");
  return out;
}
std::string programJson(const Scene &s, const MachiningProgram &p) {
  std::ostringstream o;
  o << std::setprecision(15);
  auto base = json(s);
  base.pop_back();
  o << base
    << ",\"program\":{\"version\":1,\"robotReady\":false,\"collisions\":"
    << (p.collisions ? "true" : "false")
    << ",\"wholeModel\":" << (p.settings.wholeModel ? "true" : "false")
    << ",\"stockCollisions\":" << (p.stockCollisions ? "true" : "false")
    << ",\"stock\":{\"min\":";
  vector(o, p.settings.stock.low);
  o << ",\"max\":";
  vector(o, p.settings.stock.high);
  o << ",\"cell\":" << p.settings.stock.cell
    << ",\"stepDown\":" << p.settings.stock.stepDown
    << ",\"stepOver\":" << p.settings.stock.stepOver
    << ",\"allowance\":" << p.settings.stock.allowance
    << ",\"roughFeed\":" << p.settings.stock.roughFeed << ",\"nx\":" << p.nx
    << ",\"ny\":" << p.ny << ",\"dx\":" << p.dx << ",\"dy\":" << p.dy
    << ",\"removed_mm3\":" << p.removed
    << ",\"finishRemoved_mm3\":" << p.finishRemoved
    << ",\"finalExcess_mm3\":" << p.finalExcess
    << ",\"excessAfterRough_mm3\":" << p.excess << ",\"target\":";
  array(o, p.target);
  o << ",\"roughRemaining\":";
  array(o, p.roughRemaining);
  o << ",\"finalRemaining\":";
  array(o, p.finalRemaining);
  o << "},\"includedFaces\":";
  array(o, p.includedFaces);
  o << ",\"excludedFaces\":";
  array(o, p.excludedFaces);
  o << ",\"excludedReasons\":[";
  for (size_t i = 0; i < p.excludedReasons.size(); ++i) {
    if (i)
      o << ',';
    o << quoted(p.excludedReasons[i]);
  }
  o << ']';
  o << ",\"notices\":[";
  for (size_t i = 0; i < p.notices.size(); ++i) {
    if (i)
      o << ',';
    o << quoted(p.notices[i]);
  }
  o << "],\"operations\":[";
  for (size_t i = 0; i < p.operations.size(); ++i) {
    if (i)
      o << ',';
    const auto &op = p.operations[i];
    o << "{\"name\":" << quoted(op.name) << ",\"stage\":" << op.stage
      << ",\"face\":" << op.face << ",\"level\":" << op.level
      << ",\"removed_mm3\":" << op.removed << ",\"path\":";
    poses(o, op.path);
    o << '}';
  }
  o << "],\"stage\":";
  array(o, p.stage);
  o << ",\"operation\":";
  array(o, p.operation);
  o << ",\"trajectory\":";
  poses(o, p.trajectory);
  o << "}}";
  return o.str();
}
std::string programCsv(const MachiningProgram &p) {
  std::ostringstream o;
  o << std::setprecision(15)
    << "stage,operation,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,"
       "time_s,motion,collision,sweep_collision\n";
  for (size_t k = 0; k < p.trajectory.poses.size(); ++k) {
    const auto &v = p.trajectory.poses[k];
    o << p.stage[k] << ',' << p.operation[k] + 1 << ',' << k << ',' << v.p.x()
      << ',' << v.p.y() << ',' << v.p.z() << ',' << v.q.w() << ',' << v.q.x()
      << ',' << v.q.y() << ',' << v.q.z() << ',' << v.t << ',' << v.motion
      << ',' << v.collision << ',' << v.sweepCollision << '\n';
  }
  return o.str();
}
} // namespace milling
