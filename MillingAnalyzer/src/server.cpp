// Windows API must precede OCCT: Standard_Macro.hxx defines NOUSER/NOKERNEL.
// clang-format off
#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#include <shellapi.h>
#endif
// clang-format on
#include "server.h"
#include "geometry.h"
#include "httplib.h"
#include "operation_plan.h"
#include "program.h"
#include "viewer_resource.h"
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>

namespace {
std::string errorJson(const std::string &text) {
  std::string out = "{\"error\":\"";
  for (unsigned char c : text) {
    if (c == '"' || c == '\\')
      out += '\\';
    if (c >= 32)
      out += char(c);
  }
  return out + "\"}";
}
double parameter(const httplib::Request &r, const char *name, double fallback) {
  if (!r.has_param(name))
    return fallback;
  auto text = r.get_param_value(name);
  size_t n = 0;
  double x = std::stod(text, &n);
  if (n != text.size() || !std::isfinite(x))
    throw std::runtime_error("Invalid numeric parameter");
  return x;
}
milling::Options options(const httplib::Request &r, bool whole = false) {
  milling::Options o;
  o.spacing = parameter(r, "spacing", 2);
  o.step = parameter(r, "step", 1);
  o.chord = parameter(r, "chord", .02);
  o.angleDeg = parameter(r, "angle", 5);
  o.diameter = parameter(r, "diameter", 2.5);
  o.length = parameter(r, "length", 20);
  o.feed = parameter(r, "feed", 10);
  o.clearance = parameter(r, "clearance", 5);
  double face = whole ? 1 : parameter(r, "face", 1),
         sign = whole ? 1 : parameter(r, "sign", 1);
  if (face < 1 || face > 1000 || face != std::floor(face) ||
      (sign != 1 && sign != -1))
    throw std::runtime_error("Invalid face or normal sign");
  o.face = int(face);
  o.normalSign = int(sign);
  o.validate();
  return o;
}
} // namespace
int serve(bool openBrowser) {
  try {
    std::random_device random;
    std::ostringstream token;
    for (int i = 0; i < 4; ++i)
      token << std::hex << std::setw(8) << std::setfill('0') << random();
    std::string root = "/" + token.str();
    httplib::Server server;
    server.set_payload_max_length(50 * 1024 * 1024);
    server.set_read_timeout(30);
    server.set_write_timeout(120);
    std::mutex mutex;
    auto model = milling::demo();
    auto initial = milling::withOperationCatalog(
        milling::json(milling::analyze(model, milling::Options{})), model);
    unsigned modelRevision = 1;
    auto revisionJson = [](std::string text, unsigned revision) {
      text.pop_back();
      return text + ",\"modelRevision\":" + std::to_string(revision) + "}";
    };
    initial = revisionJson(std::move(initial), modelRevision);
    int port = server.bind_to_any_port("127.0.0.1");
    if (port < 0)
      throw std::runtime_error("Cannot bind local viewer");
    std::string origin = "http://127.0.0.1:" + std::to_string(port);
    auto authorize = [&](const httplib::Request &r, httplib::Response &res) {
      if (r.has_header("Origin") && r.get_header_value("Origin") != origin) {
        res.status = 403;
        res.set_content(errorJson("Origin rejected"), "application/json");
        return false;
      }
      return true;
    };
    server.Get(root + "/",
               [&](const httplib::Request &, httplib::Response &res) {
                 std::lock_guard<std::mutex> lock(mutex);
                 std::string page = viewerTemplate;
                 auto p = page.find("@@SCENE@@");
                 page.replace(p, 9, initial);
                 p = page.find("@@API@@");
                 page.replace(p, 7, "\"" + root + "\"");
                 res.set_header("Cache-Control", "no-store");
                 res.set_header("Referrer-Policy", "no-referrer");
                 res.set_content(page, "text/html; charset=utf-8");
               });
    auto api = [&](const httplib::Request &r, httplib::Response &res) {
      if (!authorize(r, res))
        return;
      std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
      if (!lock) {
        res.status = 409;
        res.set_content(errorJson("Calculation already running"),
                        "application/json");
        return;
      }
      try {
        milling::Scene result;
        auto nextModel = model;
        std::string data;
        if (r.path == root + "/load") {
          auto format = r.get_param_value("format");
          if (format != "step" && format != "bpatch")
            throw std::runtime_error("Supported files: STEP, STP, bpatch");
          if (r.body.empty())
            throw std::runtime_error("Empty model");
          auto folder = std::filesystem::temp_directory_path() /
                        std::filesystem::u8path("milling-" + token.str());
          std::filesystem::create_directories(folder);
          struct Cleanup {
            std::filesystem::path file;
            ~Cleanup() {
              std::error_code ec;
              std::filesystem::remove(file, ec);
              std::filesystem::remove(file.parent_path(), ec);
            }
          } cleanup{folder / ("input." + format)};
          {
            std::ofstream f(cleanup.file, std::ios::binary);
            f.write(r.body.data(), r.body.size());
            f.flush();
            if (!f)
              throw std::runtime_error("Cannot stage local model");
          }
          auto candidate = format == "step" ? milling::loadStep(cleanup.file)
                                            : milling::loadPatch(cleanup.file);
          milling::Options o;
          o.previewOnly = true;
          result = milling::analyze(candidate, o);
          nextModel = std::move(candidate);
        } else if (r.path == root + "/operations" ||
                   r.path == root + "/program" ||
                   r.path == root + "/program-demo" ||
                   r.path == root + "/auto-program") {
          bool demoProgram = r.path == root + "/program-demo";
          if (demoProgram)
            nextModel = milling::programDemo();
          auto o = options(r, r.path == root + "/auto-program" ||
                                  r.path == root + "/operations");
          o.face = 0; // program scope never depends on the displayed face
          milling::ProgramSettings cfg;
          cfg.wholeModel = r.path == root + "/auto-program";
          if (r.get_param_value("mode") == "rough")
            cfg.mode = milling::ProgramSettings::Rough;
          else if (!r.get_param_value("mode").empty() &&
                   r.get_param_value("mode") != "both")
            throw std::runtime_error("Program mode must be rough or both");
          const char *names[] = {"xmin", "ymin", "zmin",
                                 "xmax", "ymax", "zmax"};
          for (int k = 0; k < 6; ++k) {
            if (!demoProgram && !r.has_param(names[k]))
              throw std::runtime_error("Enter all six stock bounds explicitly");
            double &v = k < 3 ? cfg.stock.low[k] : cfg.stock.high[k - 3];
            v = parameter(r, names[k], v);
          }
          cfg.stock.cell = parameter(r, "cell", .5);
          cfg.stock.stepDown = parameter(r, "depth", 1);
          cfg.stock.stepOver = parameter(r, "roughSpacing", 1);
          cfg.stock.allowance = parameter(r, "allowance", .3);
          cfg.stock.roughFeed = parameter(r, "roughFeed", 10);
          cfg.stock.plungeConfirmed =
              demoProgram || r.get_param_value("plunge") == "1";
          auto faces = r.get_param_value("finishFaces");
          if (!cfg.wholeModel && !faces.empty() && faces != "auto") {
            std::istringstream list(faces);
            std::string item;
            while (std::getline(list, item, ',')) {
              size_t n = 0;
              int id = std::stoi(item, &n);
              if (n != item.size() || id < 1 || id > 1000)
                throw std::runtime_error("Invalid finish face list");
              cfg.finishFaces.push_back(id);
            }
          }
          auto preview = o;
          preview.previewOnly = true;
          result = milling::analyze(nextModel, preview);
          milling::MachiningProgram program;
          std::vector<milling::OperationSpec> specs;
          if (r.path == root + "/operations") {
            if (parameter(r, "revision", 0) != modelRevision)
              throw std::runtime_error("Model changed in another window; "
                                       "reload and rebuild operations");
            double count = parameter(r, "count", 0);
            if (count < 1 || count > 32 || count != std::floor(count))
              throw std::runtime_error("Confirm between 1 and 32 operations");

            for (int i = 0; i < int(count); ++i) {
              std::string prefix = "op" + std::to_string(i) + ".";
              auto number = [&](const char *key, double fallback) {
                return parameter(r, (prefix + key).c_str(), fallback);
              };
              milling::OperationSpec op;
              op.kind = r.get_param_value(prefix + "kind");
              double face = number("face", 0), wire = number("wire", 1);
              if (face < 0 || face > 1000 || face != std::floor(face) ||
                  wire < 1 || wire > 1000 || wire != std::floor(wire))
                throw std::runtime_error("Invalid operation geometry ID");
              op.face = int(face);
              op.wire = int(wire);
              op.confirmed = r.get_param_value(prefix + "confirmed") == "1";
              op.spiral = r.get_param_value(prefix + "spiral") == "1";
              op.top = number("top", cfg.stock.high.z());
              op.bottom = number("bottom", cfg.stock.low.z());
              op.depth = number("depth", cfg.stock.stepDown);
              op.allowance = number("allowance", 0);
              specs.push_back(op);
            }
            program =
                milling::planOperations(nextModel, result, o, cfg.stock, specs);
          } else
            program = milling::planProgram(nextModel, result, o, cfg);
          data = specs.empty()
                     ? milling::programJson(result, program)
                     : milling::operationPlanJson(result, program, specs);
        } else if (r.path == root + "/demo") {
          nextModel = milling::demo(r.get_param_value("kind") == "bowl");
          result = milling::analyze(nextModel, milling::Options{});
        } else
          result = milling::analyze(model, options(r));
        if (data.empty())
          data = milling::json(result);
        data = milling::withOperationCatalog(std::move(data), nextModel);
        if (r.path == root + "/load" || r.path == root + "/demo" ||
            r.path == root + "/program-demo")
          ++modelRevision;
        data = revisionJson(std::move(data), modelRevision);
        model = std::move(nextModel);
        initial = data;
        res.set_header("Cache-Control", "no-store");
        res.set_content(data, "application/json");
      } catch (const Standard_Failure &e) {
        res.status = 400;
        res.set_content(errorJson(e.GetMessageString() ? e.GetMessageString()
                                                       : "CAD error"),
                        "application/json");
      } catch (const std::exception &e) {
        res.status = 400;
        res.set_content(errorJson(e.what()), "application/json");
      }
    };
    server.Post(root + "/load", api);
    server.Post(root + "/calculate", api);
    server.Post(root + "/demo", api);
    server.Post(root + "/operations", api);
    server.Post(root + "/program", api);
    server.Post(root + "/auto-program", api);
    server.Post(root + "/program-demo", api);
    std::string url = origin + root + "/";
    std::cout << "Milling Analyzer local UI: " << url
              << "\nKeep this console open. Ctrl+C stops the viewer service. "
                 "No EGM connection.\n"
              << std::flush;
#ifdef _WIN32
    if (openBrowser)
      ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr,
                    SW_SHOWNORMAL);
#else
    (void)openBrowser;
#endif
    server.listen_after_bind();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
