#pragma once
#include "program.h"
namespace milling {
// Separate offline planner. IDs refer to the current imported B-rep only.
struct OperationSpec {
  std::string kind; // rough, face, pocket, bore, surface, contour
  int face = 0, wire = 1;
  bool confirmed = false, spiral = false;
  double top = 0, bottom = 0, depth = 1, allowance = 0;
};
std::string operationPlanJson(const Scene &, const MachiningProgram &,
                              const std::vector<OperationSpec> &);
std::string withOperationCatalog(std::string, const Model &);
MachiningProgram planOperations(const Model &, Scene &, const Options &,
                                const StockSettings &,
                                const std::vector<OperationSpec> &);
} // namespace milling
