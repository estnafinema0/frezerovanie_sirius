#include "geometry.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
namespace milling {
namespace {
constexpr double pi = 3.141592653589793;
void require(bool b, const char *text) {
  if (!b)
    throw std::runtime_error(text);
}
V vec(const gp_Pnt &p) { return {p.X(), p.Y(), p.Z()}; }
V vec(const gp_Vec &p) { return {p.X(), p.Y(), p.Z()}; }
std::string quote(const std::string &s) {
  std::ostringstream o;
  o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      o << '\\' << c;
    else if (c < 32 || c == '<' || c == '>')
      o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c)
        << std::dec;
    else
      o << c;
  }
  o << '"';
  return o.str();
}
std::string vectorJson(const V &p) {
  std::ostringstream o;
  o << std::setprecision(15) << '[' << p.x() << ',' << p.y() << ',' << p.z()
    << ']';
  return o.str();
}
bool inside(const TopoDS_Face &f, double u, double v) {
  BRepClass_FaceClassifier c(f, gp_Pnt2d(u, v), 1e-8);
  return c.State() == TopAbs_IN || c.State() == TopAbs_ON;
}
double angle(const V &a, const V &b) {
  return std::acos(std::clamp(a.dot(b), -1., 1.));
}
} // namespace
void Options::validate() const {
  for (double x : {spacing, step, chord, angleDeg, diameter, length, feed})
    require(std::isfinite(x) && x > 0,
            "Parameters must be finite and positive");
  require(spacing >= .1 && spacing <= 100 && step >= .05 && step <= 100 &&
              chord >= 1e-5 && chord <= 1 && angleDeg >= .1 && angleDeg <= 30,
          "Sampling parameters outside supported range");
  require(diameter <= 100 && length <= 200 && feed <= 1000 &&
              (normalSign == 1 || normalSign == -1) && face >= 0,
          "Invalid tool, feed, face or normal sign");
}
Model loadStep(const std::filesystem::path &path) {
  require(std::filesystem::is_regular_file(path) &&
              std::filesystem::file_size(path) > 0 &&
              std::filesystem::file_size(path) <= 50 * 1024 * 1024,
          "STEP must be a nonempty regular file <=50 MiB");
  STEPControl_Reader r;
  // OCCT 7.9 accepts UTF-8 file names on Windows via its OSD layer.
  require(r.ReadFile(path.u8string().c_str()) == IFSelect_RetDone,
          "STEP read failed");
  TColStd_SequenceOfAsciiString len, a, s;
  r.FileUnits(len, a, s);
  require(len.Length() > 0, "STEP has no declared length units");
  std::string units;
  for (int i = 1; i <= len.Length(); ++i) {
    if (i > 1)
      units += ", ";
    units += len.Value(i).ToCString();
  }
  r.SetSystemLengthUnit(1.0);
  require(r.TransferRoots() > 0, "STEP has no transferable roots");
  auto shape = r.OneShape();
  require(!shape.IsNull() && BRepCheck_Analyzer(shape).IsValid(),
          "Invalid STEP B-rep");
  return {shape, units};
}
Model demo(bool concave) {
  TColgp_Array2OfPnt poles(1, 4, 1, 4);
  double h[] = {0, 1. / 3, 1. / 3, 0};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      poles.SetValue(i + 1, j + 1,
                     gp_Pnt(20 * i / 3., 20 * j / 3.,
                            (concave ? -1 : 1) * (8 * h[i] + 4 * h[j])));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(poles);
  return {BRepBuilderAPI_MakeFace(surf, 1e-7).Face(),
          "mm (analytic Bezier demo)"};
}
Model loadPatch(const std::filesystem::path &path) {
  require(std::filesystem::is_regular_file(path) &&
              std::filesystem::file_size(path) <= 8192,
          "Patch must be a regular file <=8 KiB");
  std::ifstream file(path);
  file.imbue(std::locale::classic());
  std::string tag, units;
  int version;
  double x, y, w, h;
  require(bool(file >> tag >> version) && tag == "FREZA_BEZIER_HEIGHTFIELD" &&
              version == 1,
          "Unsupported patch format");
  require(bool(file >> tag >> units) && tag == "units" &&
              (units == "mm" || units == "inch"),
          "Explicit mm/inch units required");
  require(bool(file >> tag >> x >> y) && tag == "origin",
          "Missing patch origin");
  require(bool(file >> tag >> w >> h) && tag == "size", "Missing patch size");
  require(bool(file >> tag) && tag == "heights", "Missing patch heights");
  double scale = units == "inch" ? 25.4 : 1;
  x *= scale;
  y *= scale;
  w *= scale;
  h *= scale;
  require(std::isfinite(x) && std::isfinite(y) && std::abs(x) <= 100000 &&
              std::abs(y) <= 100000 && std::isfinite(w) && std::isfinite(h) &&
              w >= .001 && h >= .001 && w <= 10000 && h <= 10000,
          "Invalid patch dimensions");
  TColgp_Array2OfPnt poles(1, 4, 1, 4);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double z;
      require(bool(file >> z) && std::isfinite(z) &&
                  std::abs(z * scale) <= 100000,
              "Invalid control height");
      poles.SetValue(i + 1, j + 1,
                     gp_Pnt(x + w * i / 3., y + h * j / 3., z * scale));
    }
  require(!(file >> tag), "Unexpected trailing patch data");
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(poles);
  return {BRepBuilderAPI_MakeFace(surf, 1e-7).Face(),
          units + " (Bezier control net)"};
}
Q frame(const V &normal, const V &tangent, const Q *previous) {
  require(normal.allFinite() && tangent.allFinite() && normal.norm() > 1e-10,
          "Degenerate tool frame");
  V z = normal.normalized(), x = tangent - z * tangent.dot(z);
  if (x.norm() < 1e-9 && previous) {
    x = (*previous) * V::UnitX();
    x -= z * x.dot(z);
  }
  if (x.norm() < 1e-9) {
    x = std::abs(z.x()) < .8 ? V::UnitX() : V::UnitY();
    x -= z * x.dot(z);
  }
  x.normalize();
  V y = z.cross(x).normalized();
  x = y.cross(z).normalized();
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  require((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-8 &&
              R.determinant() > .999999,
          "Invalid tool rotation");
  Q q(R);
  q.normalize();
  if (previous && q.dot(*previous) < 0)
    q.coeffs() *= -1;
  return q;
}
Pose evaluate(const TopoDS_Face &face, double u, double v, const Options &o) {
  BRepAdaptor_Surface surf(face, true);
  gp_Pnt p;
  gp_Vec du, dv;
  surf.D1(u, v, p, du, dv);
  V x = vec(du), n = x.cross(vec(dv));
  require(n.norm() > 1e-10 && n.allFinite(), "Degenerate surface normal");
  if (face.Orientation() == TopAbs_REVERSED)
    n = -n;
  n.normalize();
  Pose pose;
  pose.p = vec(p);
  pose.n = n;
  pose.q = frame(o.normalSign * n, x);
  require(pose.p.allFinite(), "Nonfinite surface point");
  return pose;
}
std::vector<std::pair<double, double>> intervals(const TopoDS_Face &face,
                                                 double v) {
  double u0, u1, v0, v1;
  BRepTools::UVBounds(face, u0, u1, v0, v1);
  Handle(Geom2d_Curve) line = new Geom2d_TrimmedCurve(
      new Geom2d_Line(gp_Pnt2d(0, v), gp_Dir2d(1, 0)), u0, u1);
  std::vector<double> cuts{u0, u1};
  for (TopExp_Explorer e(face, TopAbs_EDGE); e.More(); e.Next()) {
    double a, b;
    auto c = BRep_Tool::CurveOnSurface(TopoDS::Edge(e.Current()), face, a, b);
    require(!c.IsNull() && std::isfinite(a) && std::isfinite(b),
            "Missing trim curve");
    Handle(Geom2d_Curve) edge = new Geom2d_TrimmedCurve(c, a, b);
    Geom2dAPI_InterCurveCurve inter(line, edge, 1e-9);
    for (int i = 1; i <= inter.NbPoints(); ++i)
      cuts.push_back(std::clamp(inter.Point(i).X(), u0, u1));
    // A raster coincident with a boundary is deliberately omitted.
    if (inter.NbSegments())
      return {};
  }
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(
      std::unique(cuts.begin(), cuts.end(),
                  [](double a, double b) { return std::abs(a - b) < 1e-9; }),
      cuts.end());
  std::vector<std::pair<double, double>> out;
  for (size_t i = 1; i < cuts.size(); ++i)
    if (cuts[i] - cuts[i - 1] > 1e-8 &&
        inside(face, (cuts[i] + cuts[i - 1]) / 2, v))
      out.push_back({cuts[i - 1], cuts[i]});
  return out;
}
std::vector<Pass> raster(const TopoDS_Face &f, const Options &o) {
  o.validate();
  double u0, u1, v0, v1;
  BRepTools::UVBounds(f, u0, u1, v0, v1);
  require(std::isfinite(u0) && std::isfinite(u1) && std::isfinite(v0) &&
              std::isfinite(v1) && u1 > u0 && v1 > v0 && u1 - u0 < 1e6 &&
              v1 - v0 < 1e6,
          "Invalid UV domain");
  // Physical row-length estimate. Chord/normal adaptation is along each row.
  double vLength = 0;
  for (int j = 0; j < 5; ++j) {
    double d = 0, u = u0 + (u1 - u0) * j / 4;
    for (int k = 1; k <= 32; ++k)
      d += (evaluate(f, u, v0 + (v1 - v0) * k / 32, o).p -
            evaluate(f, u, v0 + (v1 - v0) * (k - 1) / 32, o).p)
               .norm();
    vLength = std::max(vLength, d);
  }
  int rows = std::max(1, int(std::ceil(vLength / o.spacing)));
  require(rows <= 1000, "Raster row budget exceeded");
  std::vector<Pass> result;
  size_t count = 0;
  for (int row = 0; row < rows; ++row) {
    double v = v0 + (v1 - v0) * (row + .5) / rows;
    auto spans = intervals(f, v);
    if (row % 2)
      std::reverse(spans.begin(), spans.end());
    for (auto span : spans) {
      double begin = row % 2 ? span.second : span.first,
             end = row % 2 ? span.first : span.second;
      Pass pass;
      pass.poses.push_back(evaluate(f, begin, v, o));
      std::function<void(double, double, const Pose &, const Pose &, int)>
          split;
      split = [&](double a, double b, const Pose &pa, const Pose &pb,
                  int depth) {
        require(++count <= 100000, "Raster sampling budget exceeded");
        bool ok = (pb.p - pa.p).norm() <= o.step &&
                  angle(pa.n, pb.n) <= o.angleDeg * pi / 180 &&
                  pa.q.angularDistance(pb.q) <= o.angleDeg * pi / 180;
        Pose mid = evaluate(f, (a + b) / 2, v, o);
        for (double t : {.25, .5, .75}) {
          auto q = evaluate(f, a + (b - a) * t, v, o);
          require(inside(f, a + (b - a) * t, v),
                  "Trim classification failed along pass");
          ok &= (q.p - ((1 - t) * pa.p + t * pb.p)).norm() <= o.chord &&
                angle(pa.n, q.n) <= o.angleDeg * pi / 180 &&
                angle(q.n, pb.n) <= o.angleDeg * pi / 180;
        }
        if (ok) {
          pass.poses.push_back(pb);
          return;
        }
        require(depth < 20, "Adaptive path depth exceeded");
        split(a, (a + b) / 2, pa, mid, depth + 1);
        split((a + b) / 2, b, mid, pb, depth + 1);
      };
      auto first = pass.poses.front(), last = evaluate(f, end, v, o);
      split(begin, end, first, last, 0);
      for (size_t i = 0; i < pass.poses.size(); ++i) {
        auto &p = pass.poses[i];
        V tangent = (end > begin ? 1. : -1.) * (p.q * V::UnitX());
        p.q = frame(o.normalSign * p.n, tangent,
                    i ? &pass.poses[i - 1].q : nullptr);
        if (i)
          p.t =
              pass.poses[i - 1].t + (p.p - pass.poses[i - 1].p).norm() / o.feed;
      }
      result.push_back(std::move(pass));
      require(result.size() <= 4000, "Pass budget exceeded");
    }
  }
  return result;
}
bool triangleCollision(const Triangle &tri, const Pose &pose, double radius,
                       double length) {
  std::vector<V> polygon;
  auto inverse = pose.q.conjugate();
  for (auto p : {tri.a, tri.b, tri.c})
    polygon.push_back(inverse * (p - pose.p));
  // Ignore tangency within mesh/contact tolerance, test the cylinder interior.
  constexpr double eps = .001;
  for (int side = 0; side < 2; ++side) {
    std::vector<V> out;
    for (size_t i = 0; i < polygon.size(); ++i) {
      V a = polygon[i], b = polygon[(i + 1) % polygon.size()];
      double da = side ? length - eps - a.z() : a.z() - eps,
             db = side ? length - eps - b.z() : b.z() - eps;
      if (da >= 0)
        out.push_back(a);
      if ((da >= 0) != (db >= 0))
        out.push_back(a + (b - a) * da / (da - db));
    }
    polygon = std::move(out);
    if (polygon.empty())
      return false;
  }
  double minDist = 1e100;
  bool positive = false, negative = false;
  for (size_t i = 0; i < polygon.size(); ++i) {
    Eigen::Vector2d a = polygon[i].head<2>(),
                    b = polygon[(i + 1) % polygon.size()].head<2>(), d = b - a;
    double t = d.squaredNorm() > 0
                   ? std::clamp(-a.dot(d) / d.squaredNorm(), 0., 1.)
                   : 0.;
    minDist = std::min(minDist, (a + t * d).norm());
    double cross = a.x() * b.y() - a.y() * b.x();
    positive |= cross > 1e-12;
    negative |= cross < -1e-12;
  }
  bool enclosed = polygon.size() >= 3 && (positive != negative);
  return enclosed || minDist < radius - eps;
}
Scene analyze(const Model &model, const Options &o) {
  o.validate();
  require(!model.shape.IsNull(), "Empty model");
  size_t faceCount = 0;
  for (TopExp_Explorer ex(model.shape, TopAbs_FACE); ex.More(); ex.Next())
    require(++faceCount <= 1000, "Face budget exceeded before meshing");
  Bnd_Box box;
  BRepBndLib::AddOptimal(model.shape, box, false, false);
  require(!box.IsVoid() && !box.IsOpen(), "Unbounded model");
  double x0, y0, z0, x1, y1, z1;
  box.Get(x0, y0, z0, x1, y1, z1);
  for (double x : {x0, y0, z0, x1, y1, z1})
    require(std::isfinite(x) && std::abs(x) <= 100000,
            "Model coordinates exceed range");
  require(
      std::max({x1 - x0, y1 - y0, z1 - z0}) <= 1000,
      "Preview supports models up to 1000 mm; choose appropriate model units");
  Scene scene;
  scene.options = o;
  scene.sourceUnits = model.units;
  BRepMesh_IncrementalMesh mesher(model.shape, .02, false, .15, false);
  require(mesher.IsDone(), "Tessellation failed");
  std::vector<TopoDS_Face> faces;
  for (TopExp_Explorer e(model.shape, TopAbs_FACE); e.More(); e.Next()) {
    auto f = TopoDS::Face(e.Current());
    faces.push_back(f);
    require(faces.size() <= 1000, "Face budget exceeded");
    TopLoc_Location loc;
    auto mesh = BRep_Tool::Triangulation(f, loc);
    require(!mesh.IsNull(), "Face has no display mesh");
    for (int i = 1; i <= mesh->NbTriangles(); ++i) {
      int a, b, c;
      mesh->Triangle(i).Get(a, b, c);
      if (f.Orientation() == TopAbs_REVERSED)
        std::swap(b, c);
      scene.mesh.push_back(
          {vec(mesh->Node(a).Transformed(loc.Transformation())),
           vec(mesh->Node(b).Transformed(loc.Transformation())),
           vec(mesh->Node(c).Transformed(loc.Transformation())),
           int(faces.size())});
      require(scene.mesh.size() <= 100000, "Mesh budget exceeded");
    }
  }
  require(!faces.empty() && o.face <= int(faces.size()), "No such face");
  std::vector<std::unique_ptr<BRepClass3d_SolidClassifier>> solids;
  for (TopExp_Explorer ex(model.shape, TopAbs_SOLID); ex.More(); ex.Next())
    solids.push_back(
        std::make_unique<BRepClass3d_SolidClassifier>(ex.Current()));
  size_t work = 0;
  for (size_t i = 0; i < faces.size(); ++i) {
    Face face;
    face.id = int(i + 1);
    BRepAdaptor_Surface s(faces[i]);
    const char *names[] = {"Plane",     "Cylinder", "Cone",    "Sphere",
                           "Torus",     "Bezier",   "BSpline", "Revolution",
                           "Extrusion", "Offset",   "Other"};
    face.type = names[std::min(10, int(s.GetType()))];
    if (s.GetType() == GeomAbs_BezierSurface) {
      auto b = s.Bezier();
      face.nu = b->NbUPoles();
      face.nv = b->NbVPoles();
      for (int u = 1; u <= face.nu; ++u)
        for (int v = 1; v <= face.nv; ++v) {
          face.controls.push_back(vec(b->Pole(u, v)));
          face.weights.push_back(b->Weight(u, v));
        }
    }
    if (o.previewOnly || (o.face && o.face != face.id) ||
        (!o.face && faces.size() > 32)) {
      face.error =
          "Not calculated: select this face and Calculate (CLI: --face ID)";
      scene.faces.push_back(face);
      continue;
    }
    try {
      face.passes = raster(faces[i], o);
      auto collides = [&](const Pose &p) {
        ++scene.collisionQueries;
        V middle = p.p + p.q * V(0, 0, o.length / 2);
        for (auto &solid : solids) {
          solid->Perform(gp_Pnt(middle.x(), middle.y(), middle.z()), 1e-7);
          if (solid->State() == TopAbs_IN)
            return true;
        }
        double r = std::hypot(o.diameter / 2, o.length);
        for (auto &t : scene.mesh) {
          require(++work <= 100000000,
                  "Collision work budget exceeded; choose one face");
          V low = t.a.cwiseMin(t.b).cwiseMin(t.c),
            high = t.a.cwiseMax(t.b).cwiseMax(t.c);
          if ((p.p.cwiseMax(low).cwiseMin(high) - p.p).norm() > r)
            continue;
          if (triangleCollision(t, p, o.diameter / 2, o.length))
            return true;
        }
        return false;
      };
      for (auto &pass : face.passes) {
        for (size_t k = 0; k < pass.poses.size(); ++k) {
          pass.poses[k].collision = collides(pass.poses[k]);
          if (k) {
            auto &a = pass.poses[k - 1];
            auto &b = pass.poses[k];
            int n = std::max(
                2, int(std::ceil(std::max(
                       (a.p - b.p).norm() / std::min(.5, o.diameter / 4),
                       a.q.angularDistance(b.q) / (2 * pi / 180)))));
            require(n <= 10000, "Pose sweep budget exceeded");
            for (int j = 1; j < n; ++j) {
              double t = double(j) / n;
              Pose p;
              p.p = (1 - t) * a.p + t * b.p;
              p.q = a.q.slerp(t, b.q);
              pass.segmentCollision |= collides(p);
            }
          }
        }
      }
    } catch (const Standard_Failure &e) {
      face.error = e.GetMessageString() ? e.GetMessageString() : "OCCT error";
      face.passes.clear();
    } catch (const std::exception &e) {
      face.error = e.what();
      face.passes.clear();
    }
    scene.faces.push_back(std::move(face));
  }
  return scene;
}
std::string json(const Scene &s) {
  std::ostringstream o;
  o << std::setprecision(15)
    << "{\"version\":1,\"units\":\"mm\",\"sourceUnits\":"
    << quote(s.sourceUnits) << ",\"diameter\":" << s.options.diameter
    << ",\"length\":" << s.options.length
    << ",\"normalSign\":" << s.options.normalSign
    << ",\"spacing\":" << s.options.spacing << ",\"step\":" << s.options.step
    << ",\"chord\":" << s.options.chord
    << ",\"angleDeg\":" << s.options.angleDeg << ",\"feed\":" << s.options.feed
    << ",\"collisionQueries\":" << s.collisionQueries << ",\"mesh\":[";
  bool sep = false;
  for (auto &t : s.mesh) {
    if (sep)
      o << ',';
    sep = true;
    o << '[' << t.face << ',' << vectorJson(t.a) << ',' << vectorJson(t.b)
      << ',' << vectorJson(t.c) << ']';
  }
  o << "],\"faces\":[";
  sep = false;
  for (auto &f : s.faces) {
    if (sep)
      o << ',';
    sep = true;
    o << "{\"id\":" << f.id << ",\"type\":" << quote(f.type)
      << ",\"error\":" << quote(f.error) << ",\"nu\":" << f.nu
      << ",\"nv\":" << f.nv << ",\"controls\":[";
    for (size_t i = 0; i < f.controls.size(); ++i) {
      if (i)
        o << ',';
      o << vectorJson(f.controls[i]);
    }
    o << "],\"weights\":[";
    for (size_t i = 0; i < f.weights.size(); ++i) {
      if (i)
        o << ',';
      o << f.weights[i];
    }
    o << "],\"passes\":[";
    bool ps = false;
    for (auto &p : f.passes) {
      if (ps)
        o << ',';
      ps = true;
      o << "{\"sweepCollision\":" << (p.segmentCollision ? "true" : "false")
        << ",\"poses\":[";
      bool pt = false;
      for (auto &v : p.poses) {
        if (pt)
          o << ',';
        pt = true;
        o << "{\"p\":" << vectorJson(v.p) << ",\"n\":" << vectorJson(v.n)
          << ",\"q\":[" << v.q.w() << ',' << v.q.x() << ',' << v.q.y() << ','
          << v.q.z() << "],\"t\":" << v.t
          << ",\"collision\":" << (v.collision ? "true" : "false") << '}';
      }
      o << "]}";
    }
    o << "]}";
  }
  o << "]}";
  return o.str();
}
} // namespace milling
