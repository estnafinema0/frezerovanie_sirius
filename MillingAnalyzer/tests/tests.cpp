#include "geometry.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace milling;
void check(bool b, const char *why) {
  if (!b)
    throw std::runtime_error(why);
}
int main(int argc, char **argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--fixture") {
      STEPControl_Writer writer;
      check(writer.Transfer(BRepPrimAPI_MakeBox(20, 10, 5).Shape(),
                            STEPControl_AsIs) == IFSelect_RetDone,
            "fixture transfer");
      check(writer.Write(argv[2]) == IFSelect_RetDone, "fixture write");
      return 0;
    }
    Options o;
    o.validate();
    int n = 0;
    for (V normal : {V(0, 0, 1), V(1, 0, 0), V(.2, .3, 1).normalized()}) {
      Q q = frame(normal, V(0, 0, 1));
      check((q * V::UnitZ() - normal).norm() < 1e-9, "normal alignment");
      check(std::abs(q.norm() - 1) < 1e-12, "unit quaternion");
      check(std::abs(q.toRotationMatrix().determinant() - 1) < 1e-12,
            "right handed");
      auto next = frame(normal, V::Zero(), &q);
      check(q.dot(next) > .999, "fallback continuity");
      auto reversedFeed = frame(normal, -(q * V::UnitX()), &q);
      check(q.angularDistance(reversedFeed) < 1e-10,
            "feed reversal must not spin tool about its axis");
      ++n;
    }
    {
      Q q = Q::Identity();
      for (int k = 0; k <= 200; ++k) {
        V normal = V(.3 * std::sin(k * .03), .2 * std::cos(k * .03), 1).normalized();
        Q next = frame(normal, V(k % 2 ? -1 : 1, 0, 0), &q);
        Q delta = next * q.conjugate();
        check(std::abs(delta.vec().dot(q * V::UnitZ())) < 1e-10,
              "transport angular increment has no tool-axis spin");
        check((next * V::UnitZ() - normal).norm() < 1e-10,
              "transport tracks curved normal");
        q = next;
      }
      bool rejected = false;
      try { frame(-(q * V::UnitZ()), V::UnitX(), &q); }
      catch (...) { rejected = true; }
      check(rejected, "ambiguous antipodal normal rejected");
    }
    auto d = demo();
    auto face = TopoDS::Face(TopExp_Explorer(d.shape, TopAbs_FACE).Current());
    auto p = evaluate(face, .3, .7, o);
    check(std::abs(p.p.z() - (8 * .3 * .7 + 4 * .7 * .3)) < 1e-9,
          "Bezier value");
    auto rev = TopoDS::Face(face.Reversed());
    check((p.n + evaluate(rev, .3, .7, o).n).norm() < 1e-9, "reversed normal");
    auto passes = raster(face, o);
    check(passes.size() >= 8 && passes.size() <= 11, "physical row spacing");
    check(!passes.empty(), "raster exists");
    for (auto &pass : passes)
      for (size_t i = 1; i < pass.poses.size(); ++i) {
        auto &a = pass.poses[i - 1];
        auto &b = pass.poses[i];
        check((a.p - b.p).norm() <= o.step + 1e-8, "3D step");
        check(a.q.dot(b.q) >= 0, "quaternion hemisphere");
        check((b.q * V::UnitZ() - b.n).norm() < 1e-9, "tool normal");
        check(a.q.angularDistance(b.q) <=
                  o.angleDeg * 3.141592653589793 / 180 + 1e-8,
              "orientation step");
        check(b.t >= a.t, "pass time");
        ++n;
      }
    BRepBuilderAPI_MakePolygon outer;
    outer.Add(gp_Pnt(0, 0, 0));
    outer.Add(gp_Pnt(20, 0, 0));
    outer.Add(gp_Pnt(20, 20, 0));
    outer.Add(gp_Pnt(0, 20, 0));
    outer.Close();
    auto plane = BRepBuilderAPI_MakeFace(outer.Wire()).Face();
    auto planePasses = raster(plane, o);
    auto planePath = connectPasses(planePasses, 0, o, &plane);
    check(!planePath.poses.empty(), "plane snake generated");
    int planeApproaches = 0, planeRetracts = 0;
    for (size_t i = 1; i < planePath.poses.size(); ++i) {
      auto &a = planePath.poses[i - 1];
      auto &b = planePath.poses[i];
      check(planePath.poses.front().q.angularDistance(b.q) < 1e-10,
            "plane orientation constant through all snake reversals");
      check(b.motion != 2, "plane rows connected on surface without transfers");
      planeApproaches += b.motion == 3 && a.motion != 3;
      planeRetracts += b.motion == 1 && a.motion != 1;
    }
    check(planeApproaches == 1 && planeRetracts == 1,
          "one approach and one retract for connected plane");
    for (size_t i = 1; i < planePasses.size(); ++i)
      check((planePasses[i].poses.back().p - planePasses[i].poses.front().p)
                .dot(planePasses[i - 1].poses.back().p - planePasses[i - 1].poses.front().p) < 0,
            "snake feed directions alternate independently of orientation");
    BRepBuilderAPI_MakeFace ring(outer.Wire());
    BRepBuilderAPI_MakePolygon inner;
    inner.Add(gp_Pnt(8, 8, 0));
    inner.Add(gp_Pnt(8, 12, 0));
    inner.Add(gp_Pnt(12, 12, 0));
    inner.Add(gp_Pnt(12, 8, 0));
    inner.Close();
    ring.Add(inner.Wire());
    auto trimmed = raster(ring.Face(), o);
    for (auto &pass : trimmed)
      for (auto &v : pass.poses)
        check(!(v.p.x() > 8 + 1e-6 && v.p.x() < 12 - 1e-6 &&
                v.p.y() > 8 + 1e-6 && v.p.y() < 12 - 1e-6),
              "hole excluded");
    Pose origin;
    origin.p = V::Zero();
    origin.q = Q::Identity();
    Triangle tangent{V(-2, -2, 0), V(2, -2, 0), V(0, 2, 0), 1},
        hit{V(-2, -2, 1), V(2, -2, 1), V(0, 2, 1), 1};
    check(!triangleCollision(tangent, origin, 1, 20),
          "tangency is not penetration");
    check(triangleCollision(hit, origin, 1, 20),
          "interior triangle intersects cylinder");

    double u0, u1, v0, v1;
    BRepTools::UVBounds(ring.Face(), u0, u1, v0, v1);
    auto spans = intervals(ring.Face(), (v0 + v1) / 2);
    check(spans.size() == 2, "hole splits raster into independent passes");
    auto temp = std::filesystem::temp_directory_path() /
                std::filesystem::u8path(u8"freza STEP проверка");
    std::filesystem::create_directories(temp);
    auto stp = temp / "box.step";
    STEPControl_Writer writer;
    check(writer.Transfer(BRepPrimAPI_MakeBox(20, 10, 5).Shape(),
                          STEPControl_AsIs) == IFSelect_RetDone,
          "STEP writer transfer");
    check(writer.Write(stp.u8string().c_str()) == IFSelect_RetDone,
          "STEP writer Unicode");
    auto box = analyze(loadStep(stp), o);
    check(box.faces.size() == 6, "STEP read Unicode / solid faces");
    auto patch = temp / "curve.bpatch";
    {
      std::ofstream f(patch);
      f << "FREZA_BEZIER_HEIGHTFIELD 1 units inch origin 0 0 size 1 1 heights ";
      for (int k = 0; k < 16; ++k)
        f << "0.5 ";
    }
    auto loaded = loadPatch(patch);
    auto fp =
        TopoDS::Face(TopExp_Explorer(loaded.shape, TopAbs_FACE).Current());
    auto sample = evaluate(fp, .5, .5, o);
    check((sample.p - V(12.7, 12.7, 12.7)).norm() < 1e-8, "Bezier inch units");
    {
      std::ofstream f(patch);
      f << "FREZA_BEZIER_HEIGHTFIELD 1 units mm origin 0 0 size 1 1 heights "
           "nan";
    }
    bool rejected = false;
    try {
      loadPatch(patch);
    } catch (...) {
      rejected = true;
    }
    check(rejected, "nonfinite patch rejected");
    std::filesystem::remove(patch);
    std::filesystem::remove(stp);
    std::filesystem::remove(temp);

    auto cyl = BRepPrimAPI_MakeCylinder(5, 10).Shape();
    for (TopExp_Explorer ex(cyl, TopAbs_FACE); ex.More(); ex.Next()) {
      auto f = TopoDS::Face(ex.Current());
      BRepAdaptor_Surface surf(f);
      if (surf.GetType() == GeomAbs_Cylinder) {
        double a, b, c, dv;
        BRepTools::UVBounds(f, a, b, c, dv);
        auto cp = evaluate(f, (a + b) / 2, (c + dv) / 2, o);
        check((cp.n - V(cp.p.x() / 5, cp.p.y() / 5, 0)).norm() < 1e-8,
              "cylinder normal");
      }
    }
    gp_Trsf transform;
    transform.SetTranslation(gp_Vec(100, 200, 300));
    auto shifted = TopoDS::Face(face.Moved(TopLoc_Location(transform)));
    check((evaluate(shifted, .3, .7, o).p - p.p - V(100, 200, 300)).norm() <
              1e-8,
          "CAD location applied once");
    auto scene = analyze(d, o);
    check(!scene.faces[0].passes.empty(), "scene paths");
    check(!json(scene).empty(), "scene JSON");
    const auto &continuous = scene.faces[0].trajectory;
    check(!continuous.poses.empty(), "whole-face trajectory exists");
    int kinds[4] = {};
    for (size_t k = 1; k < continuous.poses.size(); ++k) {
      auto &a = continuous.poses[k - 1];
      auto &b = continuous.poses[k];
      ++kinds[b.motion];
      check(b.t > a.t, "global trajectory time strictly increases");
      check((b.p - a.p).norm() <= o.step + 1e-8, "connected XYZ step bounded");
      check(a.q.angularDistance(b.q) <=
                o.angleDeg * 3.141592653589793 / 180 + 1e-8,
            "connected orientation step bounded");
      check(std::abs((b.q * a.q.conjugate()).vec().dot(a.q * V::UnitZ())) < 1e-9,
            "complete trajectory has no added axial spin");
      if (a.motion == 0 && b.motion == 0) {
        // Demo graph is z = 8u(1-u) + 4v(1-v), x=20u,y=20v.
        // Test internal XYZ interpolation, including the links between rows.
        for (double f : {.25, .5, .75}) {
          V p = (1-f)*a.p + f*b.p;
          double u=p.x()/20, v=p.y()/20;
          check(std::abs(p.z()-(8*u*(1-u)+4*v*(1-v))) <= o.chord,
                "working chord including row link follows Bezier surface");
        }
      }
      if (b.motion == 2)
        check(std::min(a.p.z(), b.p.z()) > 3 + o.length,
              "transfer above entire model");
    }
    check(kinds[0] > 0 && kinds[1] > 0 && kinds[3] > 0 && kinds[2] == 0,
          "Bezier snake joins all rows on surface with only initial/final lift");
    check(!continuous.segmentCollision, "Bezier snake sweep has no detected gouge");
    for (auto &pass : scene.faces[0].passes) {
      for (auto &endpoint : {pass.poses.front(), pass.poses.back()}) {
        bool included = false;
        for (auto &point : continuous.poses)
          included |= (point.p - endpoint.p).norm() < 1e-9;
        check(included, "whole-face path includes every pass endpoint");
      }
    }
    auto ringFace = ring.Face();
    auto holePath = connectPasses(trimmed, 0, o, &ringFace);
    check(!holePath.poses.empty(), "trimmed face joined");
    bool holeTransfer = false;
    for (size_t k = 1; k < holePath.poses.size(); ++k) {
      holeTransfer |= holePath.poses[k].motion == 2;
      if (holePath.poses[k].motion != 0)
        continue;
      V mid = (holePath.poses[k - 1].p + holePath.poses[k].p) / 2;
      check(!(mid.x() > 8 && mid.x() < 12 && mid.y() > 8 && mid.y() < 12),
            "no cutting across hole");
    }
    check(holeTransfer, "hole requires elevated transfer");
    auto sideways = trimmed;
    for (auto &pass : sideways)
      for (auto &pose : pass.poses)
        pose.q = Q(Eigen::AngleAxisd(3.141592653589793 / 2, V::UnitY()));
    bool connectionRejected = false;
    try {
      connectPasses(sideways, 0, o);
    } catch (...) {
      connectionRejected = true;
    }
    check(connectionRejected, "sideways tool unsupported connector rejected");
    Options preview;
    preview.previewOnly = true;
    auto meshOnly = analyze(d, preview);
    check(!meshOnly.mesh.empty() && meshOnly.faces[0].passes.empty(),
          "preview does not calculate toolpath");
    Options coarse = o;
    coarse.diameter = 4;
    auto coarselySampled = analyze(d, coarse);
    bool chordHit = false;
    for (const auto &pass : coarselySampled.faces[0].passes)
      chordHit |= pass.segmentCollision;
    check(chordHit, "intermediate chord collision remains detectable");
    auto concave = analyze(demo(true), o);
    bool detected = false;
    for (auto &pass : concave.faces[0].passes)
      for (auto &v : pass.poses)
        detected |= v.collision;
    check(detected, "concave flat cutter gouge detected");
    bool threw = false;
    o.step = 0;
    try {
      o.validate();
    } catch (...) {
      threw = true;
    }
    check(threw, "bad parameters rejected");
    std::cout << "Geometry / orientation / trim / collision checks passed; "
                 "pose checks="
              << n << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
