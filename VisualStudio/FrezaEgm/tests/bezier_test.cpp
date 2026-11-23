#include "bezier_cam.h"
#include "csv_io.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#ifdef FREZA_BEZIER_OCL_ORACLE
#include <opencamlib/cylcutter.hpp>
#include <opencamlib/line.hpp>
#include <opencamlib/pathdropcutter.hpp>
#include <opencamlib/stlsurf.hpp>
#endif

using namespace freza;
using namespace freza::bezier;
int checks = 0;
void Check(bool b, const char *label) {
  ++checks;
  if (!b)
    throw std::runtime_error(label);
}
void Reject(std::function<void()> f, const char *label) {
  bool threw = false;
  try {
    f();
  } catch (const std::exception &) {
    threw = true;
  }
  Check(threw, label);
}
bool Near(double a, double b, double eps = 1e-7) {
  return std::abs(a - b) <= eps;
}
Patch Plane(double a = 0, double b = 0, double c = .5) {
  Patch p;
  p.width = p.height = 20;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      p.z[4 * i + j] = c + a * 20 * i / 3 + b * 20 * j / 3;
  return p;
}
int main() {
  try {
    Options o;
    auto p = Plane();
    auto s = Evaluate(p, .3, .7);
    Check(Near(s.position.x, 6) && Near(s.position.y, 14) &&
              Near(s.position.z, .5),
          "plane Bernstein evaluation");
    Check(Near(s.du.x, 20) && Near(s.dv.y, 20) && Near(s.normal.z, 1),
          "plane derivatives/normal");
    auto tilted = Plane(.2, -.1, 2);
    s = Evaluate(tilted, .4, .6);
    Check(Near(s.position.z, 2 + .2 * 8 - .1 * 12) && Near(s.du.z, 4) &&
              Near(s.dv.z, -2),
          "affine precision");
    Check(Near(s.normal.x, -.2 / std::sqrt(1.05)) &&
              Near(s.normal.y, .1 / std::sqrt(1.05)),
          "analytic normal");
    auto bound = ContactBound(tilted, {10, 10, 0}, {10, 10, 0}, o);
    double exact = 3 + 2 * std::sqrt(.05);
    Check(bound.lower <= exact && bound.upper >= exact &&
              bound.upper - bound.lower <= o.contactTolerance + 1e-7,
          "inclined plane disk maximum");
    auto segment = ContactBound(tilted, {8, 10, 0}, {12, 10, 0}, o);
    exact = 3.4 + 2 * std::sqrt(.05);
    Check(segment.lower <= exact && segment.upper >= exact,
          "inclined capsule maximum");
    auto result = Perimeter(p, o);
    double length = 0;
    for (size_t i = 1; i < result.points.size(); ++i) {
      auto a = result.points[i - 1], b = result.points[i];
      length += std::hypot(a.x - b.x, a.y - b.y);
      Check(Near(b.z, .5), "plane contact height");
    }
    Check(Near(length, 80) &&
              Near(result.points.front().x, result.points.back().x) &&
              Near(result.points.front().z, result.points.back().z),
          "closed perimeter length");
    auto base = ToBase(p, result, {750, 0, 599.5}, o, 2, 50);
    Check(base.size() == 5 && Near(base[0].z, 600, 3e-6) &&
              Near(base[1].x, 770),
          "explicit base translation and collinear reduction");
    Check(result.certifiedClearance <= o.pathTolerance,
          "certified path tolerance");
    // Quadratic crown represented exactly in a bicubic net: z=8u(1-u)+4v(1-v).
    Patch crown = Plane(0, 0, 0);
    double h[] = {0, 1. / 3, 1. / 3, 0};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        crown.z[4 * i + j] = 8 * h[i] + 4 * h[j];
    s = Evaluate(crown, .31, .64);
    double e = 1e-5;
    Check(Near(s.position.z, 8 * .31 * .69 + 4 * .64 * .36),
          "crown analytic value");
    Check(Near(s.du.z, (Evaluate(crown, .31 + e, .64).position.z -
                        Evaluate(crown, .31 - e, .64).position.z) /
                           (2 * e)),
          "finite difference du");
    Check(Near(s.dv.z, (Evaluate(crown, .31, .64 + e).position.z -
                        Evaluate(crown, .31, .64 - e).position.z) /
                           (2 * e)),
          "finite difference dv");
    Check(Near(std::hypot(std::hypot(s.normal.x, s.normal.y), s.normal.z), 1),
          "unit normal");
    bound = ContactBound(crown, {10, 10, 0}, {10, 10, 0}, o);
    Check(bound.lower <= 3 && bound.upper >= 3, "crown interior maximum");
    // Both endpoint disks are low; the continuous segment crosses the peak.
    auto end = ContactBound(crown, {0, 10, 0}, {0, 10, 0}, o);
    Reject(
        [&] {
          ValidatePath(crown,
                       {{0, 10, end.upper + 1e-6}, {20, 10, end.upper + 1e-6}},
                       o);
        },
        "hidden inter-vertex gouge rejected");
    o.pathTolerance = .2;
    o.contactTolerance = .01;
#ifdef FREZA_BEZIER_OCL_ORACLE
    // Independent triangulated approximation. Concave quadratic mesh lies below
    // analytic crown.
    ocl::STLSurf mesh;
    auto vertex = [&](int i, int j) {
      double u = i / 100., v = j / 100.;
      return ocl::Point(20 * u, 20 * v, 8 * u * (1 - u) + 4 * v * (1 - v));
    };
    for (int i = 0; i < 100; ++i)
      for (int j = 0; j < 100; ++j) {
        mesh.addTriangle(ocl::Triangle(vertex(i, j), vertex(i + 1, j),
                                       vertex(i + 1, j + 1)));
        mesh.addTriangle(ocl::Triangle(vertex(i, j), vertex(i + 1, j + 1),
                                       vertex(i, j + 1)));
      }
    ocl::CylCutter cutter(4, 20);
    ocl::Path oclPath;
    oclPath.append(ocl::Line(ocl::Point(0, 10, 0), ocl::Point(20, 10, 0)));
    ocl::PathDropCutter dc;
    dc.setSTL(mesh);
    dc.setCutter(&cutter);
    dc.setSampling(.5);
    dc.setZ(-1);
    dc.setPath(&oclPath);
    dc.run();
    bool match = true;
    auto contacts = dc.getCLPoints();
    Check(contacts.size() > 10, "OCL comparison samples");
    for (auto cp : contacts) {
      auto oracle = ContactBound(crown, {cp.x, cp.y, 0}, {cp.x, cp.y, 0}, o);
      match &= cp.getCC().type != ocl::NONE && cp.z <= oracle.upper + 1e-7 &&
               cp.z >= oracle.lower - .002;
    }
    Check(match, "OCL mesh vs analytic envelope (mesh error <=0.002 mm)");
#endif
    auto curved = Perimeter(crown, o);
    ValidatePath(crown, curved.points, o);
    Check(curved.certifiedClearance <= o.pathTolerance &&
              curved.points.size() > 5,
          "curved adaptive contact path");
    auto curvedBase = ToBase(crown, curved, {750, 0, 600}, o, .5, 50);
    Check(curvedBase.size() > 5, "curved path not flattened");
    // Independent dense sampling of the ENTIRE swept disk for every segment
    // midpoint.
    bool clear = true;
    for (size_t k = 1; k < curved.points.size(); ++k) {
      auto a = curved.points[k - 1], b = curved.points[k];
      for (int t = 0; t <= 4; ++t) {
        double f = t / 4., x = a.x + (b.x - a.x) * f, y = a.y + (b.y - a.y) * f,
               z = a.z + (b.z - a.z) * f;
        for (int i = -4; i <= 4; ++i)
          for (int j = -4; j <= 4; ++j) {
            double dx = i * .5, dy = j * .5;
            if (dx * dx + dy * dy > 4 || x + dx < 0 || x + dx > 20 ||
                y + dy < 0 || y + dy > 20)
              continue;
            double u = (x + dx) / 20, v = (y + dy) / 20;
            clear &= z + 1e-7 >= 8 * u * (1 - u) + 4 * v * (1 - v);
          }
      }
    }
    Check(clear, "independent dense swept-footprint oracle");
    for (auto bad : {0., -1., std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
      auto invalid = o;
      invalid.diameter = bad;
      Reject([&] { Perimeter(p, invalid); }, "invalid diameter");
    }
    // Fixed-seed, independent Bernstein/dense oracle on nonmonotone control
    // nets.
    std::minstd_rand rng(7219);
    for (int trial = 0; trial < 24; ++trial) {
      auto randomPatch = Plane(0, 0, 0);
      for (auto &z : randomPatch.z)
        z = (int(rng() % 2001) - 1000) / 250.;
      Point3 a{double(rng() % 2001) / 100, double(rng() % 2001) / 100, 0};
      Point3 b{double(rng() % 2001) / 100, double(rng() % 2001) / 100, 0};
      auto cert = ContactBound(randomPatch, a, b, o);
      bool safe = true;
      auto basis = [](double t) {
        return std::array<double, 4>{std::pow(1 - t, 3),
                                     3 * t * (1 - t) * (1 - t),
                                     3 * t * t * (1 - t), t * t * t};
      };
      for (int i = 0; i <= 80; ++i)
        for (int j = 0; j <= 80; ++j) {
          double x = i / 4., y = j / 4., dx = b.x - a.x, dy = b.y - a.y;
          double t = std::clamp(
              ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy), 0., 1.);
          if (std::hypot(x - a.x - t * dx, y - a.y - t * dy) > o.diameter / 2)
            continue;
          auto bu = basis(x / 20), bv = basis(y / 20);
          double z = 0;
          for (int u = 0; u < 4; ++u)
            for (int v = 0; v < 4; ++v)
              z += bu[u] * bv[v] * randomPatch.z[4 * u + v];
          safe &= z <= cert.upper;
        }
      Check(safe && cert.upper - cert.lower <= o.contactTolerance + 1e-7,
            "random capsule Bernstein oracle");
    }
    auto invalid = p;
    invalid.z[5] = std::numeric_limits<double>::quiet_NaN();
    Reject([&] { Evaluate(invalid, 0, 0); }, "NaN patch");
    invalid = p;
    invalid.width = 0;
    Reject([&] { Evaluate(invalid, 0, 0); }, "degenerate patch");
    Reject([&] { Evaluate(p, -.01, 0); }, "invalid UV");
    Reject([&] { ContactBound(p, {-1, 0, 0}, {0, 0, 0}, o); },
           "outside domain");
    auto budget = o;
    budget.maxTotalNodes = 4;
    Reject([&] { Perimeter(crown, budget); }, "total work budget");
    auto limit = o;
    limit.maxNodesPerQuery = 4;
    Reject([&] { ContactBound(tilted, {10, 10, 0}, {10, 10, 0}, limit); },
           "query budget");
    limit = o;
    limit.maxPoints = 5;
    Reject([&] { Perimeter(crown, limit); }, "path budget");
    limit = o;
    limit.maxDepth = 1;
    Reject([&] { ContactBound(tilted, {10, 10, 0}, {10, 10, 0}, limit); },
           "depth budget");
    Reject([&] { ToBase(p, result, {NAN, 0, 0}, o, 1, 50); },
           "nonfinite placement");
    Reject([&] { ToBase(p, result, {0, 0, 0}, o, 0, 50); },
           "invalid interpolation");
    auto negative = Plane(0, 0, -5);
    Check(Perimeter(negative, o).points.front().z < 0,
          "negative heights preserved");
    // Round-trip the existing six-decimal CSV and recertify the continuous
    // path.
    auto csv =
        std::filesystem::temp_directory_path() / "freza_bezier_roundtrip.csv";
    std::wstring error;
    Check(WritePointsCsv(csv.wstring(), curvedBase, error), "CSV export");
    std::vector<Point3> readback;
    Check(ReadPointsCsv(csv.wstring(), readback, error), "CSV readback");
    for (auto &q : readback) {
      q.x -= 750;
      q.z -= 600;
    }
    ValidatePath(crown, readback, o);
    Check(true, "rounded CSV segment safety");
    std::filesystem::remove(csv);
    // Strict file format, explicit units, Unicode paths through filesystem.
    auto path = std::filesystem::temp_directory_path() /
                std::filesystem::u8path(u8"freza Безье test.bpatch");
    auto write = [&](const std::string &body) {
      std::ofstream f(path);
      f << body;
    };
    std::string heights;
    for (int i = 0; i < 16; ++i)
      heights += " 1";
    write("FREZA_BEZIER_HEIGHTFIELD 1\nunits inch\norigin 0 0\nsize 1 "
          "2\nheights" +
          heights);
    auto loaded = Load(path);
    Check(Near(loaded.width, 25.4) && Near(loaded.height, 50.8) &&
              Near(loaded.z[0], 25.4),
          "inch conversion Unicode path");
    for (auto body :
         {std::string(""), std::string("FREZA_BEZIER_HEIGHTFIELD 2"),
          std::string("FREZA_BEZIER_HEIGHTFIELD 1 units guessed"),
          std::string("FREZA_BEZIER_HEIGHTFIELD 1 units mm origin 0 0 size 1 1 "
                      "heights nan"),
          std::string("FREZA_BEZIER_HEIGHTFIELD 1 units mm origin 0 0 size 1 1 "
                      "heights") +
              heights + " extra",
          std::string(8193, 'x')}) {
      write(body);
      Reject([&] { Load(path); }, "invalid input file");
    }
    std::filesystem::remove(path);
    std::cout << "Bezier checks passed: " << checks
              << "; curved points=" << curved.points.size()
              << "; clearance=" << curved.certifiedClearance << "\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL after " << checks << ": " << e.what() << "\n";
    return 1;
  }
}
