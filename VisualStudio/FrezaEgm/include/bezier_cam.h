#pragma once
#include "trajectory.h"
#include <array>
#include <filesystem>
#include <vector>

// Independent, fixed-axis flat-cutter experiment. No EGM, orientation or stock
// model.
namespace freza::bezier {
struct Patch {
  double x = 0, y = 0, width = 0, height = 0;
  std::array<double, 16>
      z{}; // i*4+j: u index first, v index second; millimetres
};
struct Sample {
  Point3 position, du, dv, normal;
};
struct Options {
  double diameter = 4;
  double contactTolerance =
      0.005; // uncertainty of maximum over cutter footprint
  double pathTolerance =
      0.1; // maximum conservative clearance above that envelope
  double maxStep =
      2; // CAM segment length, independent of existing interpolation
  std::size_t maxPoints = 20000, maxNodesPerQuery = 100000;
  std::size_t maxTotalNodes = 10000000; // per generation/validation call
  unsigned maxDepth = 24;
};
struct Bound {
  double lower, upper;
  std::size_t nodes;
};
struct Result {
  std::vector<Point3>
      points; // part-frame tool-tip centres, NOT surface samples
  double certifiedClearance = 0;
  std::size_t queries = 0;
};
void Validate(const Patch &p);
Patch Load(const std::filesystem::path
               &path); // strict versioned format with explicit units
Sample Evaluate(const Patch &p, double u, double v);
// Maximum surface height in the XY capsule swept by a radius-diameter/2 cutter.
Bound ContactBound(const Patch &p, Point3 a, Point3 b, const Options &o);
Result Perimeter(const Patch &p, const Options &o);
// Checks EVERY continuous segment, in part frame, against the entire cutter
// footprint.
void ValidatePath(const Patch &p, const std::vector<Point3> &points,
                  const Options &o);
// Translation only; no implicit TCP offset. Checks both original and
// interpolated path.
std::vector<Point3> ToBase(const Patch &p, const Result &result,
                           Point3 translation, const Options &o,
                           double interpolationStep, double feed);
} // namespace freza::bezier
