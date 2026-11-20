#pragma once
#include "trajectory.h"
#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace freza::cam {
enum class Units { Unspecified, Millimetres, Inches };
struct Options {
    Units units = Units::Unspecified; // STL has no units: caller must choose.
    double scale = 1.0;
    double diameterMm = 4.0;
    double cutterLengthMm = 20.0; // geometric cutter length, NOT a TCP offset
    double samplingMm = 0.5; // independent of EGM interpolation
    double toleranceMm = 1e-6;
    Point3 translationMm{}; // base = part_mm + translation; rotation unsupported
};
struct Bounds { Point3 min, max; };
struct Mesh {
    std::vector<std::array<Point3, 3>> triangles;
    Bounds bounds;
};
struct Result {
    Options options;
    Bounds modelMm, baseBounds;
    size_t triangleCount = 0, contactCount = 0;
    std::vector<Point3> partPoints, basePoints;
};
bool Available();
// All functions are transactional: output changes only on success.
bool LoadStl(const std::filesystem::path&, const Options&, Mesh&, std::string& error);
bool Calculate(const std::filesystem::path&, const Options&, Result&, std::string& error);
// Analytic whole-segment validation for the proven planar rectangular strategy.
bool ValidatePath(const Result&, const std::vector<Point3>& basePath, std::string& error);
bool ValidateInterpolation(const Result&, double step, double feed, std::string& error);
std::string Describe(const Result&);
} // namespace freza::cam
