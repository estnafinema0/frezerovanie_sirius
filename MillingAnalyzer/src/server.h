#pragma once
#include <filesystem>
int serve(bool openBrowser, const std::filesystem::path &initialFile,
          const std::filesystem::path &ethernetFile, bool desktop = false);
