#include "bezier_cam.h"
#include "csv_io.h"
#include <filesystem>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>

double Number(const std::wstring &s) {
  std::wistringstream f(s);
  f.imbue(std::locale::classic());
  double x;
  wchar_t tail;
  if (!(f >> x) || (f >> tail))
    throw std::runtime_error("Invalid numeric argument.");
  return x;
}
int Run(int argc, wchar_t **argv) {
  if (argc != 9) {
    std::cerr << "OFFLINE ONLY. Usage: FrezaEgm_bezier_cli patch.bpatch "
                 "output.csv diameter_mm path_tolerance_mm tx_mm ty_mm tz_mm "
                 "interpolation_step_mm\n";
    return 2;
  }
  try {
    auto patch = freza::bezier::Load(std::filesystem::path(argv[1]));
    freza::bezier::Options o;
    o.diameter = Number(argv[3]);
    o.pathTolerance = Number(argv[4]);
    freza::Point3 t{Number(argv[5]), Number(argv[6]), Number(argv[7])};
    auto result = freza::bezier::Perimeter(patch, o);
    auto base = freza::bezier::ToBase(patch, result, t, o, Number(argv[8]), 50);
    // Only write after every geometric check succeeds. Refuse accidental
    // source/output overwrite.
    if (std::filesystem::exists(std::filesystem::path(argv[2])))
      throw std::runtime_error("Output exists; choose a new filename.");
    std::wstring error;
    if (!freza::WritePointsCsv(argv[2], base, error))
      throw std::runtime_error("CSV write failed.");
    std::cout << "OFFLINE: fixed-axis flat cutter; no stock/holder/robot "
                 "collision clearance.\n"
              << "Units=mm, patch=" << patch.width << " x " << patch.height
              << ", diameter=" << o.diameter
              << ", CAM points=" << result.points.size()
              << ", exported=" << base.size()
              << ", contact queries=" << result.queries
              << ", clearance bound=" << result.certifiedClearance << " mm\n"
              << "Translation=" << t.x << "," << t.y << "," << t.z
              << "; first TCP=" << base.front().x << "," << base.front().y
              << "," << base.front().z << "\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
#ifdef _WIN32
int wmain(int argc, wchar_t **argv) { return Run(argc, argv); }
#else
int main(int argc, char **argv) {
  std::vector<std::wstring> args;
  std::vector<wchar_t *> ptrs;
  for (int i = 0; i < argc; ++i)
    args.push_back(std::filesystem::path(argv[i]).wstring());
  for (auto &s : args)
    ptrs.push_back(s.data());
  return Run(argc, ptrs.data());
}
#endif
