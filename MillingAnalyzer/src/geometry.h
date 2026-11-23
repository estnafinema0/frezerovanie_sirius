#pragma once
#include <Eigen/Geometry>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <filesystem>
#include <string>
#include <vector>
namespace milling {
using V = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
struct Options {
  bool previewOnly = false;
  double spacing = 2, step = 1, chord = .02, angleDeg = 5, diameter = 2.5,
         length = 20, feed = 10, clearance = 5;
  int normalSign = 1,
      face = 0; // 0: all faces (at most 32); otherwise 1-based ID
  void validate() const;
};
struct Pose {
  V p, n;
  Q q;
  double t = 0;
  bool collision = false, sweepCollision = false;
  double u = 0, v = 0;
  int motion =
      0; // segment ending here: 0=cut, 1=retract, 2=transfer, 3=approach
};
struct Pass {
  std::vector<Pose> poses;
  bool segmentCollision = false;
};
struct Triangle {
  V a, b, c;
  int face;
};
struct Face {
  int id;
  std::string type, error;
  std::vector<Pass> passes;
  Pass trajectory;
  std::string connectionError;
  int nu = 0, nv = 0;
  std::vector<V> controls;
  std::vector<double> weights;
};
struct Model {
  TopoDS_Shape shape;
  std::string units;
};
struct Scene {
  std::vector<Triangle> mesh;
  std::vector<Face> faces;
  Options options;
  std::string sourceUnits;
  std::size_t collisionQueries = 0;
};
Model loadStep(const std::filesystem::path &);
Model demo(bool concave = false);
Model loadPatch(const std::filesystem::path &);
Q frame(const V &normal, const V &tangent, const Q *previous = nullptr);
Pose evaluate(const TopoDS_Face &, double u, double v, const Options &);
std::vector<std::pair<double, double>> intervals(const TopoDS_Face &, double v);
std::vector<Pass> raster(const TopoDS_Face &, const Options &);
Pass connectPasses(const std::vector<Pass> &, double modelTop, const Options &,
                   const TopoDS_Face *surface = nullptr);
bool triangleCollision(const Triangle &, const Pose &, double radius,
                       double length);
Scene analyze(const Model &, const Options &);
std::string json(const Scene &);
} // namespace milling
