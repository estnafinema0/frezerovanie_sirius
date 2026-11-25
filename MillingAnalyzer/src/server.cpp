// Windows API must precede OCCT: Standard_Macro.hxx defines NOUSER/NOKERNEL.
// clang-format off
#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#include <shellapi.h>
#endif
// clang-format on
#include "server.h"
#include "egm_session.h"
#include "geometry.h"
#include "httplib.h"
#include "operation_plan.h"
#include "program.h"
#include "viewer_resource.h"
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
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
struct ExportPath {
  milling::Pass path;
  std::string csv, error;
  bool programInterference = false;
};
std::map<int, ExportPath> faceExports(const milling::Scene &scene, const milling::V &corner) {
  std::map<int, ExportPath> out;
  for (const auto &face : scene.faces)
    out[face.id] = {face.trajectory, "",
                    face.connectionError.empty() ? face.error
                                                 : face.connectionError};
  for (auto &item : out)
    for (auto &pose : item.second.path.poses) {
      pose.p -= corner;
      pose.q = milling::tcpOrientation(pose.q);
    }
  return out;
}
std::string exportCsv(int face, const ExportPath &entry, bool virtualReview) {
  if (!entry.error.empty())
    throw std::runtime_error(entry.error);
  if (entry.programInterference && !virtualReview)
    throw std::runtime_error("Program collision or remaining-stock interference; EGM preparation refused");
  if (entry.path.segmentCollision && !virtualReview)
    throw std::runtime_error("Swept-path collision; EGM preparation refused");
  if (!entry.csv.empty())
    return entry.csv;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17);
  out << "face_id,pass_id,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,"
         "nx,ny,nz,time_s,collision,sweep_collision,motion\n";
  unsigned i = 0;
  for (const auto &p : entry.path.poses)
    out << face << ",0," << i++ << ',' << p.p.x() << ',' << p.p.y() << ','
        << p.p.z() << ',' << p.q.w() << ',' << p.q.x() << ',' << p.q.y() << ','
        << p.q.z() << ',' << p.n.x() << ',' << p.n.y() << ',' << p.n.z() << ','
        << p.t << ',' << int(p.collision) << ',' << int(p.sweepCollision) << ','
        << p.motion << '\n';
  return out.str();
}
unsigned identifier(const httplib::Request &r, const char *name) {
  double x = parameter(r, name, -1);
  if (x < 0 || x > 4294967295. || x != std::floor(x))
    throw std::runtime_error("Invalid revision or job ID");
  return unsigned(x);
}
} // namespace
int serve(bool openBrowser, const std::filesystem::path &initialFile) {
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
    auto model = initialFile.empty() ? milling::demo() : milling::loadStep(initialFile);
    milling::Options initialOptions;
    initialOptions.previewOnly = !initialFile.empty();
    auto firstScene = milling::analyze(model, initialOptions);
    auto exports = faceExports(firstScene, modelTopCorner(firstScene));
    auto initial =
        milling::withOperationCatalog(milling::json(firstScene), model);
    milling::egm::Session egm;
    unsigned modelRevision = 1, trajectoryRevision = 1;
    auto revisionJson = [&](std::string text, unsigned revision) {
      text.pop_back();
      return text + ",\"modelRevision\":" + std::to_string(revision) +
             ",\"trajectoryRevision\":" + std::to_string(trajectoryRevision) +
             "}";
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
        if (egm.active())
          throw std::runtime_error("EGM session active; end it in RobotStudio "
                                   "before changing the trajectory");
        milling::Scene result;
        std::string programExport;
        milling::V stockCorner = milling::V::Zero();
        bool programInterference = false;
        auto nextModel = model;
        std::string data;
        if (r.path == root + "/reset-trajectory") {
          if (parameter(r, "revision", 0) != modelRevision)
            throw std::runtime_error("Model changed in another window; reload "
                                     "and rebuild operations");
          auto o = options(r, true);
          o.previewOnly = true;
          result = milling::analyze(model, o);
        } else if (r.path == root + "/load") {
          auto format = r.get_param_value("format");
          if (format != "step" && format != "bpatch")
            throw std::runtime_error("Supported files: STEP, STP, bpatch");
          if (r.body.empty())
            throw std::runtime_error("Empty model");
          milling::Model candidate;
          if (format == "step") {
            std::istringstream stream(r.body);
            candidate = milling::loadUploadedStep(stream);
          } else {
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
            candidate = milling::loadPatch(cleanup.file);
          }
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
              op.optimize = r.get_param_value(prefix + "optimize") != "0";
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
          stockCorner = program.settings.stock.topCorner();
          programExport = milling::programCsv(program);
          programInterference = program.collisions || program.stockCollisions ||
                                program.trajectory.segmentCollision;
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
        auto nextExports = faceExports(result, programExport.empty() ? modelTopCorner(result) : stockCorner);
        if (!programExport.empty()) {
          // A face view must not hide a program-level stock/collision
          // rejection.
          for (auto &entry : nextExports)
            entry.second.programInterference = programInterference;
          nextExports[0] = {{}, std::move(programExport), "", programInterference};
        }
        egm.invalidate();
        ++trajectoryRevision;
        data = revisionJson(std::move(data), modelRevision);
        exports = std::move(nextExports);
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
    server.Post(root + "/reset-trajectory", api);
    server.Post(root + "/operations", api);
    server.Post(root + "/program", api);
    server.Post(root + "/auto-program", api);
    server.Post(root + "/program-demo", api);
    auto egmApi = [&](const httplib::Request &r, httplib::Response &res) {
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
        if (r.path == root + "/egm/prepare") {
          if (identifier(r, "revision") != trajectoryRevision)
            throw std::runtime_error("Trajectory changed in another window; "
                                     "reload and prepare again");
          auto found = exports.find(int(identifier(r, "face")));
          if (found == exports.end())
            throw std::runtime_error(
                "No calculated trajectory for this face/program");
          const bool virtualReview = r.get_param_value("virtualReview") == "1";
          egm.prepare(exportCsv(found->first, found->second, virtualReview), virtualReview);
        } else if (r.path == root + "/egm/start") {
          milling::egm::Settings cfg;
          cfg.bindIp = r.get_param_value("bind");
          cfg.peerIp = r.get_param_value("peer");
          unsigned port = identifier(r, "port");
          if (port > 65535)
            throw std::runtime_error("UDP port must be 1024..65535");
          cfg.port = int(port);
          cfg.virtualConfirmed = r.get_param_value("virtual") == "1";
          cfg.anchorConfirmed = r.get_param_value("anchor") == "1";
          cfg.workObject = r.get_param_value("workObject") == "1";
          cfg.calibrated = r.get_param_value("calibrated") == "1";
          if (cfg.calibrated) {
            auto required = [&](const char *name) {
              if (!r.has_param(name) || r.get_param_value(name).empty())
                throw std::runtime_error(std::string("Missing calibration field: ") + name);
              return parameter(r, name, 0);
            };
            cfg.partOrigin = milling::egm::V(required("tx"), required("ty"), required("tz"));
            cfg.partRotation = milling::egm::Q(required("qw"), required("qx"),
                                                required("qy"), required("qz"));
          }
          egm.start(identifier(r, "job"), cfg);
        } else if (r.path == root + "/egm/stop")
          egm.stop();
        else if (r.path == root + "/egm/rapid-start") {
          res.set_header("Content-Disposition", "attachment; filename=CamStart.mod");
          res.set_header("Cache-Control", "no-store");
          res.set_content(egm.rapidStart(identifier(r, "job")), "application/octet-stream");
          return;
        } else if (r.path == root + "/egm/download") {
          res.set_header("Content-Disposition",
                         "attachment; filename=prepared.poses.csv");
          res.set_content(egm.csv(identifier(r, "job")),
                          "text/csv; charset=utf-8");
          return;
        }
        res.set_header("Cache-Control", "no-store");
        res.set_content(egm.status(), "application/json");
      } catch (const std::exception &e) {
        res.status = 400;
        res.set_content(errorJson(e.what()), "application/json");
      }
    };
    server.Post(root + "/egm/prepare", egmApi);
    server.Post(root + "/egm/start", egmApi);
    server.Post(root + "/egm/stop", egmApi);
    server.Get(root + "/egm/status", egmApi);
    server.Get(root + "/egm/download", egmApi);
    server.Get(root + "/egm/rapid-start", egmApi);
    std::string url = origin + root + "/";
    std::cout << "Milling Analyzer local UI: " << url
              << "\nKeep this console open. Ctrl+C stops the viewer service. "
                 "EGM opens only after explicit UI start.\n"
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
