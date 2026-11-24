#include "operation_plan.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <fstream>
#include <iostream>
using namespace milling;
void check(bool yes, const char *text) {
  if (!yes)
    throw std::runtime_error(text);
}
template <class F> void rejects(F f, const char *text) {
  bool failed = false;
  try {
    f();
  } catch (const std::exception &) {
    failed = true;
  }
  check(failed, text);
}
int horizontalFace(const Model &m, double z) {
  int id = 0;
  for (TopExp_Explorer e(m.shape, TopAbs_FACE); e.More(); e.Next()) {
    ++id;
    auto f = TopoDS::Face(e.Current());
    BRepAdaptor_Surface s(f);
    double a, b, c, d;
    BRepTools::UVBounds(f, a, b, c, d);
    auto p = evaluate(f, (a + b) / 2, (c + d) / 2, Options{});
    if (s.GetType() == GeomAbs_Plane && p.n.z() > .999 &&
        std::abs(p.p.z() - z) < 1e-6)
      return id;
  }
  throw std::runtime_error("test floor missing");
}
int cylinderFace(const Model &m) {
  int id = 0;
  for (TopExp_Explorer e(m.shape, TopAbs_FACE); e.More(); e.Next()) {
    ++id;
    if (BRepAdaptor_Surface(TopoDS::Face(e.Current())).GetType() ==
        GeomAbs_Cylinder)
      return id;
  }
  throw std::runtime_error("test cylinder missing");
}
int main(int argc, char **argv) {
  try {
    Options o;
    Options preview = o;
    preview.previewOnly = true;
    Model box{BRepPrimAPI_MakeBox(20, 20, 4).Shape(), "mm"};
    StockSettings stock;
    stock.high = V(22, 22, 6);
    stock.plungeConfirmed = true;
    OperationSpec rough;
    rough.kind = "rough";
    rough.confirmed = true;
    rough.top = 6;
    rough.bottom = 0;
    rough.allowance = .3;
    OperationSpec face;
    face.kind = "face";
    face.face = horizontalFace(box, 4);
    face.confirmed = true;
    face.top = 6;
    face.bottom = 4;
    auto scene = analyze(box, preview);
    auto result = planOperations(box, scene, o, stock, {face, rough});
    check(result.operations.size() == 2 &&
              result.operations[0].name == "1. face" &&
              result.operations[1].name == "2. rough",
          "user order retained");
    check(!result.collisions && !result.stockCollisions,
          "face then rough: all composed segments clear");
    if (argc == 3 && std::string(argv[1]) == "--example") {
      auto dir = std::filesystem::u8path(argv[2]);
      std::filesystem::create_directories(dir);
      std::ofstream(dir / "operations.json")
          << operationPlanJson(scene, result, {face, rough});
      std::ofstream(dir / "operations.poses.csv") << programCsv(result);
    }
    for (size_t i = 1; i < result.trajectory.poses.size(); ++i)
      check(result.trajectory.poses[i].t >= result.trajectory.poses[i - 1].t,
            "monotonic time");
    auto slowStock = stock;
    slowStock.roughFeed = 3;
    auto slow = planOperations(box, scene, o, slowStock, {rough});
    for (size_t k = 1; k < slow.trajectory.poses.size(); ++k) {
      auto &a = slow.trajectory.poses[k - 1];
      auto &b = slow.trajectory.poses[k];
      check((b.p - a.p).norm() <= 3 * (b.t - a.t) + 1e-7,
            "composed roughing preserves separately selected feed");
    }
    OperationSpec contour = face;
    contour.kind = "contour";
    contour.bottom = 0;
    contour.allowance = .1;
    auto layers = planOperations(box, scene, o, stock, {contour});
    check(!layers.collisions && !layers.stockCollisions,
          "offset contour layers clear");
    contour.spiral = true;
    auto spiral = planOperations(box, scene, o, stock, {contour});
    check(!spiral.collisions && !spiral.stockCollisions,
          "offset contour spiral clear");
    bool fractional = false;
    double prior = 6;
    for (auto &p : spiral.trajectory.poses)
      if (p.motion == 0) {
        check(p.p.z() <= prior + 1e-6, "spiral descends");
        prior = p.p.z();
        fractional |= std::abs(p.p.z() - std::round(p.p.z())) > .1;
      }
    check(fractional && prior == 0,
          "spiral has continuous pitch and full bottom lap");
    // A whole-part contour must retain the lower terrace, not use the top face.
    Model terrace{
        BRepAlgoAPI_Fuse(
            box.shape, BRepPrimAPI_MakeBox(gp_Pnt(5, 5, 4), 10, 10, 2).Shape())
            .Shape(),
        "mm"};
    auto ts = analyze(terrace, preview);
    auto wholeContour = contour;
    wholeContour.face = 0;
    auto exterior = planOperations(terrace, ts, o, stock, {wholeContour});
    check(!exterior.collisions && !exterior.stockCollisions,
          "whole-part silhouette protects lower terrace");
    if (argc == 3 && std::string(argv[1]) == "--example") {
      auto dir = std::filesystem::u8path(argv[2]);
      std::ofstream(dir / "external-helix.json") << withOperationCatalog(
          operationPlanJson(ts, exterior, {wholeContour}), terrace);
      std::ofstream(dir / "external-helix.poses.csv") << programCsv(exterior);
      STEPControl_Writer writer;
      check(writer.Transfer(terrace.shape, STEPControl_AsIs) ==
                IFSelect_RetDone,
            "contour fixture transfer");
      check(writer.Write((dir / "terraced-part.step").string().c_str()) ==
                IFSelect_RetDone,
            "contour fixture write");
    }
    double lastZ = wholeContour.top;
    for (auto &p : exterior.operations.front().path.poses) {
      check((p.n - V::UnitZ()).norm() < 1e-9 &&
                p.q.angularDistance(Q::Identity()) < 1e-9,
            "contour keeps tool Z parallel with no axial rotation");
      if (p.motion != 0)
        continue;
      double dx = std::max({-p.p.x(), p.p.x() - 20, 0.});
      double dy = std::max({-p.p.y(), p.p.y() - 20, 0.});
      check(std::hypot(dx, dy) >=
                o.diameter / 2 + wholeContour.allowance + .019,
            "every contour sample outside real part by tool radius");
      check(p.p.z() <= lastZ + 1e-8, "whole-part helix always descends");
      lastZ = p.p.z();
    }
    check(std::abs(lastZ - stock.low.z()) < 1e-9, "helix reaches stock base");
    auto wrongBottom = wholeContour;
    wrongBottom.bottom = 4;
    rejects([&] { planOperations(terrace, ts, o, stock, {wrongBottom}); },
            "selected upper face cannot silently stop contour above base");
    auto shiftedStock = stock;
    shiftedStock.low.z() = -1;
    wholeContour.bottom = -1;
    auto shifted = planOperations(box, scene, o, shiftedStock, {wholeContour});
    double minZ = 1e9;
    for (const auto &p : shifted.trajectory.poses)
      minZ = std::min(minZ, p.p.z());
    check(std::abs(minZ + 1) < 1e-8,
          "stock zero is explicit, not assumed world zero");
    OperationSpec selectedRough = rough;
    selectedRough.kind = "face_rough";
    selectedRough.face = horizontalFace(box, 4);
    OperationSpec selectedFinish = face;
    selectedFinish.kind = "face_finish";
    auto selected =
        planOperations(box, scene, o, stock, {selectedRough, selectedFinish});
    check(selected.operations.size() == 2 &&
              selected.operations[0].stage == 1 &&
              selected.operations[1].stage == 2 && !selected.collisions &&
              !selected.stockCollisions,
          "selected planar face rough then automatic finish");
    for (const auto &p : selected.operations[0].path.poses)
      if (p.motion == 0)
        check(p.p.x() > 0 && p.p.x() < 20 && p.p.y() > 0 && p.p.y() < 20,
              "face roughing restricted to selected footprint");
    // Compare identical stock/tool/face against the retained legacy route.
    Model strip{BRepPrimAPI_MakeBox(20, 8, 2).Shape(), "mm"};
    auto stripScene = analyze(strip, preview);
    auto stripStock = stock;
    stripStock.high = V(22, 10, 10);
    auto sr = selectedRough;
    sr.face = horizontalFace(strip, 2);
    sr.top = 10;
    sr.optimize = false;
    auto baselineRough = planOperations(strip, stripScene, o, stripStock, {sr});
    sr.optimize = true;
    auto optimizedRough =
        planOperations(strip, stripScene, o, stripStock, {sr});
    check(!optimizedRough.collisions && !optimizedRough.stockCollisions,
          "optimized interlayer links checked against target and stock");
    check(optimizedRough.trajectory.poses.back().t <
              .9 * baselineRough.trajectory.poses.back().t,
          "optimized strip is at least ten percent faster at same feed");
    for (size_t i = 0; i < baselineRough.finalRemaining.size(); ++i)
      check(optimizedRough.finalRemaining[i] <=
                baselineRough.finalRemaining[i] + 1e-8,
            "optimization leaves no additional stock column");
    auto distance = [](const Pass &p, bool air) {
      double d = 0;
      for (size_t k = 1; k < p.poses.size(); ++k)
        if (!air || (p.poses[k].motion != 0 && p.poses[k].motion != 4))
          d += (p.poses[k].p - p.poses[k - 1].p).norm();
      return d;
    };
    std::cout << "ROUGH benchmark old/new: seconds "
              << baselineRough.trajectory.poses.back().t << " / "
              << optimizedRough.trajectory.poses.back().t << "; length "
              << distance(baselineRough.trajectory, false) << " / "
              << distance(optimizedRough.trajectory, false) << "; air "
              << distance(baselineRough.trajectory, true) << " / "
              << distance(optimizedRough.trajectory, true) << "\n";
    for (const auto &n : optimizedRough.notices)
      if (n.find("Черновая грани") != std::string::npos)
        std::cout << n << "\n";
    if (argc == 3 && std::string(argv[1]) == "--example") {
      auto dir = std::filesystem::u8path(argv[2]);
      std::ofstream(dir / "rough-before.poses.csv")
          << programCsv(baselineRough);
      std::ofstream(dir / "rough-optimized.poses.csv")
          << programCsv(optimizedRough);
      std::ofstream(dir / "rough-optimized.json") << withOperationCatalog(
          operationPlanJson(stripScene, optimizedRough, {sr}), strip);
      STEPControl_Writer writer;
      check(writer.Transfer(strip.shape, STEPControl_AsIs) == IFSelect_RetDone,
            "rough fixture transfer");
      check(writer.Write((dir / "rough-strip.step").string().c_str()) ==
                IFSelect_RetDone,
            "rough fixture write");
    }
    auto islandStock = stock;
    islandStock.high.z() = 10;
    auto islandRough = selectedRough;
    islandRough.face = horizontalFace(terrace, 4);
    islandRough.top = 10;
    islandRough.optimize = false;
    auto islandBefore =
        planOperations(terrace, ts, o, islandStock, {islandRough});
    islandRough.optimize = true;
    auto islandAfter =
        planOperations(terrace, ts, o, islandStock, {islandRough});
    check(!islandAfter.collisions && !islandAfter.stockCollisions,
          "optimized links cannot cross a protected island");
    check(islandAfter.trajectory.poses.back().t <=
              islandBefore.trajectory.poses.back().t + 1e-7,
          "island optimization cannot regress time");
    for (size_t k = 0; k < islandBefore.finalRemaining.size(); ++k)
      check(islandAfter.finalRemaining[k] <=
                islandBefore.finalRemaining[k] + 1e-7,
            "island stock coverage retained");
    BRepBuilderAPI_MakePolygon polygon;
    for (auto xy : std::vector<V>{V(0, 0, 0), V(20, 0, 0), V(20, 8, 0),
                                  V(8, 8, 0), V(8, 20, 0), V(0, 20, 0)})
      polygon.Add(gp_Pnt(xy.x(), xy.y(), xy.z()));
    polygon.Close();
    Model lpart{
        BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),
                              gp_Vec(0, 0, 4))
            .Shape(),
        "mm"};
    auto ls = analyze(lpart, preview);
    auto lc = contour;
    lc.face = horizontalFace(lpart, 4);
    auto lp = planOperations(lpart, ls, o, stock, {lc});
    check(!lp.collisions && !lp.stockCollisions,
          "concave contour offset clear");
    bool followsNotch = false;
    for (auto &p : lp.trajectory.poses)
      if (p.motion == 0 && p.p.x() > 8 && p.p.x() < 15 && p.p.y() > 8 &&
          p.p.y() < 15)
        followsNotch = true;
    check(followsNotch,
          "real L contour follows recess instead of bounding rectangle");
    lc.face = 0;
    auto wholeL = planOperations(lpart, ls, o, stock, {lc});
    check(!wholeL.collisions && !wholeL.stockCollisions,
          "projected concave whole-part contour clear");
    followsNotch = false;
    for (auto &p : wholeL.trajectory.poses)
      if (p.motion == 0 && p.p.x() > 8 && p.p.x() < 15 && p.p.y() > 8 &&
          p.p.y() < 15)
        followsNotch = true;
    check(followsNotch, "whole-part silhouette retains concavity");
    OperationSpec surface = face;
    surface.kind = "surface";
    surface.allowance = 0;
    auto finished = planOperations(box, scene, o, stock, {rough, surface});
    check(!finished.collisions && !finished.stockCollisions,
          "rough then normal finishing");
    rejects([&] { planOperations(box, scene, o, stock, {surface, rough}); },
            "normal finishing cannot secretly assume roughing");
    face.confirmed = false;
    rejects([&] { planOperations(box, scene, o, stock, {face}); },
            "unconfirmed operation rejected");
    face.confirmed = true;
    face.face = 999;
    rejects([&] { planOperations(box, scene, o, stock, {face}); },
            "stale geometry rejected");
    face.face = horizontalFace(box, 4);
    face.depth = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { planOperations(box, scene, o, stock, {face}); },
            "NaN rejected");
    face.depth = 1;
    face.kind = "pocket";
    face.bottom = 3;
    rejects([&] { planOperations(box, scene, o, stock, {face}); },
            "floor penetration rejected");
    auto blank = BRepPrimAPI_MakeBox(20, 20, 6).Shape();
    auto cavity = BRepPrimAPI_MakeBox(gp_Pnt(4, 4, 2), 12, 12, 5).Shape();
    Model pocket{BRepAlgoAPI_Cut(blank, cavity).Shape(), "mm"};
    auto pocketScene = analyze(pocket, preview);
    OperationSpec pocketOp;
    pocketOp.kind = "pocket";
    pocketOp.face = horizontalFace(pocket, 2);
    pocketOp.top = 6;
    pocketOp.bottom = 2;
    pocketOp.confirmed = true;
    auto pocketResult =
        planOperations(pocket, pocketScene, o, stock, {pocketOp});
    check(!pocketResult.collisions && !pocketResult.stockCollisions,
          "blind pocket clearing");
    check(pocketResult.finishRemoved > 100, "pocket removes core material");
    auto boreSolid = [&](double bottom) {
      return Model{
          BRepAlgoAPI_Cut(
              blank, BRepPrimAPI_MakeCylinder(
                         gp_Ax2(gp_Pnt(10, 10, bottom), gp_Dir(0, 0, 1)), 4, 8)
                         .Shape())
              .Shape(),
          "mm"};
    };
    for (double bottom : {0., 2.}) {
      auto hole = boreSolid(bottom);
      auto hs = analyze(hole, preview);
      OperationSpec holeOp;
      holeOp.kind = "bore";
      holeOp.face = cylinderFace(hole);
      holeOp.confirmed = true;
      holeOp.top = 6;
      holeOp.bottom = bottom;
      auto h = planOperations(hole, hs, o, stock, {holeOp});
      if (h.collisions || h.stockCollisions) {
        std::cerr << "hole bottom " << bottom << " mesh " << h.collisions
                  << " stock " << h.stockCollisions << "\n";
        for (auto &p : h.trajectory.poses)
          if (p.collision) {
            std::cerr << p.p.transpose() << "\n";
            break;
          }
      }
      check(!h.collisions && !h.stockCollisions,
            "through / blind hole clearing");
      check(h.finishRemoved > 100, "hole core removed");
      auto autoHole = holeOp;
      autoHole.kind = "face_rough";
      auto holeRough = planOperations(hole, hs, o, stock, {autoHole});
      check(!holeRough.collisions && !holeRough.stockCollisions &&
                holeRough.operations.front().stage == 1,
            "selected cylinder chooses hole roughing");
      autoHole.kind = "face_finish";
      auto holeFinish = planOperations(hole, hs, o, stock, {autoHole});
      check(!holeFinish.collisions && !holeFinish.stockCollisions,
            "selected cylinder chooses hole finishing");
      if (argc == 3 && std::string(argv[1]) == "--fixture" && bottom == 2) {
        STEPControl_Writer writer;
        check(writer.Transfer(hole.shape, STEPControl_AsIs) == IFSelect_RetDone,
              "fixture transfer");
        check(writer.Write(argv[2]) == IFSelect_RetDone, "fixture write");
      }
    }
    Model boss{BRepPrimAPI_MakeCylinder(4, 6).Shape(), "mm"};
    auto bs = analyze(boss, preview);
    OperationSpec bad;
    bad.kind = "bore";
    bad.face = cylinderFace(boss);
    bad.top = 6;
    bad.confirmed = true;
    rejects([&] { planOperations(boss, bs, o, stock, {bad}); },
            "external cylinder cannot be a hole");
    Model curved{BRepPrimAPI_MakePrism(demo().shape, gp_Vec(0, 0, -4)).Shape(),
                 "mm"};
    auto curvedScene = analyze(curved, preview);
    StockSettings curvedStock = stock;
    curvedStock.low = V(-2, -2, -5);
    curvedStock.high = V(22, 22, 4);
    int curvedFace = 0;
    for (auto &f : curvedScene.faces)
      if (f.type == "Bezier") {
        bool upward = true;
        for (auto &t : curvedScene.mesh)
          if (t.face == f.id && ((t.b - t.a).cross(t.c - t.a)).z() < 0)
            upward = false;
        if (upward) {
          curvedFace = f.id;
          break;
        }
      }
    check(curvedFace > 0, "closed Bezier fixture top located");
    auto cr = rough;
    cr.top = 4;
    cr.bottom = -5;
    auto cf = surface;
    cf.face = curvedFace;
    cf.top = 4;
    cf.bottom = -5;
    auto curvedProgram =
        planOperations(curved, curvedScene, o, curvedStock, {cr, cf});
    check(!curvedProgram.collisions && !curvedProgram.stockCollisions,
          "curved normal finishing clear");
    bool tilted = false;
    for (size_t k = 0; k < curvedProgram.trajectory.poses.size(); ++k)
      if (curvedProgram.stage[k] == 2) {
        auto &p = curvedProgram.trajectory.poses[k];
        check((p.q * V::UnitZ() - p.n).norm() < 1e-8,
              "quaternion follows normal");
        tilted |= p.n.z() < .99;
      }
    check(tilted, "curved surface varies tool orientation");
    cr.kind = "face_rough";
    cr.face = curvedFace;
    cf.kind = "face_finish";
    auto selectedCurved =
        planOperations(curved, curvedScene, o, curvedStock, {cr, cf});
    check(!selectedCurved.collisions && !selectedCurved.stockCollisions,
          "selected Bezier roughing and normal finishing");
    cr.optimize = false;
    auto curvedBefore =
        planOperations(curved, curvedScene, o, curvedStock, {cr});
    cr.optimize = true;
    auto curvedAfter =
        planOperations(curved, curvedScene, o, curvedStock, {cr});
    check(!curvedAfter.collisions && !curvedAfter.stockCollisions &&
              curvedAfter.trajectory.poses.back().t <=
                  curvedBefore.trajectory.poses.back().t + 1e-7,
          "curved rough optimization does not regress time or validation");
    BRepBuilderAPI_MakePolygon slope;
    for (auto p :
         std::vector<V>{V(0, 0, 0), V(20, 0, 0), V(20, 0, 4), V(0, 0, 8)})
      slope.Add(gp_Pnt(p.x(), p.y(), p.z()));
    slope.Close();
    Model wedge{
        BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(slope.Wire()).Face(),
                              gp_Vec(0, 20, 0))
            .Shape(),
        "mm"};
    auto wedgeScene = analyze(wedge, preview);
    auto wedgeStock = stock;
    wedgeStock.high.z() = 10;
    auto wr = selectedRough;
    for (const auto &t : wedgeScene.mesh) {
      V n = (t.b - t.a).cross(t.c - t.a).normalized();
      if (n.z() > .5 && n.z() < .999)
        wr.face = t.face;
    }
    wr.top = 10;
    auto wf = wr;
    wf.kind = "face_finish";
    wf.allowance = 0;
    auto wedgeProgram =
        planOperations(wedge, wedgeScene, o, wedgeStock, {wr, wf});
    check(!wedgeProgram.collisions && !wedgeProgram.stockCollisions,
          "inclined face local roughing then normal finish");
    check(*std::min_element(wedgeProgram.roughRemaining.begin(),
                            wedgeProgram.roughRemaining.end()) < 9.5,
          "tilted roughing snapshot comes from removed voxels");
    int underside = 0;
    for (const auto &t : wedgeScene.mesh)
      if ((t.b - t.a).cross(t.c - t.a).normalized().z() < -.999)
        underside = t.face;
    wr.face = underside;
    rejects([&] { planOperations(wedge, wedgeScene, o, wedgeStock, {wr}); },
            "inaccessible lower face requires another setup");
    check(withOperationCatalog(json(scene), box).find("operationCatalog") !=
              std::string::npos,
          "catalog produced");
    std::cout << "Operation order, confirmation, stock replay, floor, pocket, "
                 "through/blind holes, offsets and helix passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
