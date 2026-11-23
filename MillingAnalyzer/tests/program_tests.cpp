#include "program.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <STEPControl_Writer.hxx>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace milling;
void check(bool ok, const char *s) {
  if (!ok)
    throw std::runtime_error(s);
}
template <class F> void rejects(F f, const char *s) {
  bool failed = false;
  try {
    f();
  } catch (...) {
    failed = true;
  }
  check(failed, s);
}
int main(int argc, char **argv) {
  try {
    auto model = programDemo();
    if (argc == 3 && std::string(argv[1]) == "--fixture") {
      STEPControl_Writer writer;
      check(writer.Transfer(model.shape, STEPControl_AsIs) == IFSelect_RetDone,
            "STEP transfer");
      check(writer.Write(argv[2]) == IFSelect_RetDone, "STEP write");
      return 0;
    }
    Options o;
    Options preview = o;
    preview.previewOnly = true;
    auto scene = analyze(model, preview);
    ProgramSettings cfg;
    rejects([&] { planProgram(model, scene, o, cfg); },
            "unconfirmed axial entry rejected");
    cfg.stock.plungeConfirmed = true;
    StockGrid stock(scene, cfg.stock, o);
    // Exact upper envelope for the two planar levels, not the overall bbox.
    int lowCount = 0, highCount = 0;
    for (double h : stock.target) {
      lowCount += std::abs(h - 2) < 1e-8;
      highCount += std::abs(h - 4) < 1e-8;
    }
    check(lowCount > 100 && highCount > 100,
          "two separate CAD heights resolved");
    check(!stock.targetClear(V(5, 5, 3), V(5, 15, 3), o.diameter / 2, 3),
          "cutter cannot cut high step");
    check(stock.targetClear(V(14, 4, 3), V(14, 16, 3), o.diameter / 2, 3),
          "lower terrace is removable");
    auto initial = stock.remaining;
    double removed =
        stock.remove(V(-1, -1, 5), V(21, -1, 5), o.diameter / 2, 5);
    check(removed > 0, "material removed");
    check(stock.remove(V(-1, -1, 5), V(21, -1, 5), o.diameter / 2, 5) == 0,
          "repeat stroke removes no additional stock");
    double remainingVolume = 0;
    for (double h : stock.remaining)
      remainingVolume += (h - cfg.stock.low.z()) * stock.dx * stock.dy;
    check(std::abs(remainingVolume + stock.removedVolume -
                   stock.initialVolume) < 1e-6,
          "stock volume conservation");
    auto job = planProgram(model, scene, o, cfg);
    check(job.operations.size() == 8, "six rough layers plus two finish faces");
    check(job.includedFaces.size() == 2 && !job.excludedFaces.empty(),
          "setup coverage explicit");
    check(job.removed > 0 && job.finishRemoved > 0,
          "both stages remove material in stock simulation");
    check(!job.collisions && !job.stockCollisions,
          "demo program target and residual-stock checks");
    check(job.finalExcess > 0, "unremoved material honestly reported");
    bool finishing = false;
    double lastLayer = cfg.stock.high.z();
    for (const auto &op : job.operations) {
      if (op.stage == 2)
        finishing = true;
      if (op.stage == 1) {
        check(!finishing, "all rough operations precede finishing");
        check(lastLayer - op.level <= cfg.stock.stepDown + 1e-8 &&
                  op.level < lastLayer,
              "bounded descending layers");
        lastLayer = op.level;
        for (const auto &p : op.path.poses)
          check(p.q.angularDistance(Q::Identity()) < 1e-10,
                "rough tool stays vertical without spin");
      }
    }
    StockGrid replay(scene, cfg.stock, o);
    int rowReversals = 0;
    double previousX = 0;
    for (size_t i = 1; i < job.trajectory.poses.size(); ++i) {
      auto &a = job.trajectory.poses[i - 1];
      auto &b = job.trajectory.poses[i];
      check(b.t > a.t, "program time strictly increases");
      check(std::abs(b.q.norm() - 1) < 1e-10, "program normalized quaternion");
      check((b.p - a.p).norm() <= o.step + 1e-7,
            "composed interpolation step bounded");
      if (job.stage[i] == 1 && (b.motion == 0 || b.motion == 4)) {
        check(replay.targetClear(a.p, b.p, o.diameter / 2,
                                 std::min(a.p.z(), b.p.z())),
              "entire rough stroke protects target plus allowance");
        check(replay.engagement(a.p, b.p, o.diameter / 2, b.p.z()) <=
                  cfg.stock.stepDown + 1e-7,
              "rough engagement bounded at material samples");
        replay.remove(a.p, b.p, o.diameter / 2, b.p.z());
        if (b.motion == 0) {
          double dx = b.p.x() - a.p.x();
          if (std::abs(dx) > .001) {
            rowReversals += dx * previousX < 0;
            previousX = dx;
          }
        }
      }
    }
    check(rowReversals > 10, "rough snake reverses feed");
    for (size_t i = 0; i < job.roughRemaining.size(); ++i) {
      check(job.roughRemaining[i] >= job.target[i] - 1e-8,
            "rough stock never below protected target");
      check(job.finalRemaining[i] <= job.roughRemaining[i] + 1e-8,
            "finishing never adds material");
    }
    check(programCsv(job).find("stage,operation,sample_id") == 0,
          "separate program CSV schema");
    check(programJson(scene, job).find("\"robotReady\":false") !=
              std::string::npos,
          "offline export explicit");
    auto bad = cfg;
    bad.stock.high.x() = 1;
    rejects([&] { StockGrid x(scene, bad.stock, o); },
            "undersized stock rejected");
    bad = cfg;
    bad.stock.stepDown = 0;
    rejects([&] { bad.stock.validate(o); }, "zero layer rejected");
    bad = cfg;
    bad.stock.allowance = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { bad.stock.validate(o); }, "nonfinite allowance rejected");
    bad = cfg;
    bad.stock.low.z() = -25;
    rejects([&] { bad.stock.validate(o); },
            "stock taller than cutter rejected");
    bad = cfg;
    bad.finishFaces = {1, 1};
    rejects([&] { planProgram(model, scene, o, bad); },
            "duplicate face rejected");
    bad = cfg;
    bad.finishFaces = {job.excludedFaces.front()};
    rejects([&] { planProgram(model, scene, o, bad); },
            "explicit side/bottom face rejected in top setup");
    bad = cfg;
    bad.finishFaces = {job.includedFaces.front()};
    auto subset = planProgram(model, scene, o, bad);
    check(subset.includedFaces.size() == 1 &&
              subset.excludedFaces.size() + 1 == scene.faces.size(),
          "explicit subset reports every excluded face");
    auto open = demo();
    auto openScene = analyze(open, preview);
    rejects([&] { planProgram(open, openScene, o, cfg); },
            "open Bezier surface is not stock solid");
    // Narrow obstacle between candidate endpoints: validation must inspect the
    // whole footprint/capsule, not only the end coordinates.
    Scene tiny;
    tiny.mesh = {{V(0, -.1, 2), V(.1, .1, 2), V(-.1, .1, 2), 1}};
    StockSettings ts = cfg.stock;
    ts.low = V(-3, -3, 0);
    ts.high = V(3, 3, 4);
    StockGrid obstacle(tiny, ts, o);
    check(!obstacle.targetClear(V(-2, 0, 1), V(2, 0, 1), .1, 1),
          "thin obstacle inside segment is protected");
    std::cout << "Rough/finish program, stock conservation, layers, swept "
                 "footprints, residuals and input limits: passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
