#pragma once
#include "stock.h"
namespace milling {
struct ProgramSettings {
  StockSettings stock;
  enum Mode { Rough, Finish, Both } mode = Both;
  std::vector<int> finishFaces; // empty = all upward faces in this setup
};
struct Operation {
  std::string name;
  int stage = 0, face = 0; // 1 roughing, 2 finishing
  double level = 0, removed = 0;
  Pass path;
};
struct MachiningProgram {
  ProgramSettings settings;
  std::vector<Operation> operations;
  Pass trajectory;
  std::vector<int> stage, operation;
  std::vector<std::string> notices;
  std::vector<int> includedFaces, excludedFaces;
  std::vector<double> target, roughRemaining, finalRemaining;
  int nx = 0, ny = 0;
  double dx = 0, dy = 0, removed = 0, excess = 0, finishRemoved = 0,
         finalExcess = 0;
  bool stockCollisions = false;
  bool collisions = false;
};
Model programDemo();
MachiningProgram planProgram(const Model &, Scene &, const Options &,
                             const ProgramSettings &);
std::string programJson(const Scene &, const MachiningProgram &);
std::string programCsv(const MachiningProgram &);
} // namespace milling
