# CMake generated Testfile for 
# Source directory: D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm
# Build directory: D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/build-vs
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[freza_selftest]=] "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/build-vs/Debug/FrezaEgm_selftest.exe")
  set_tests_properties([=[freza_selftest]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;61;add_test;D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[freza_selftest]=] "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/build-vs/Release/FrezaEgm_selftest.exe")
  set_tests_properties([=[freza_selftest]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;61;add_test;D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[freza_selftest]=] "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/build-vs/MinSizeRel/FrezaEgm_selftest.exe")
  set_tests_properties([=[freza_selftest]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;61;add_test;D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[freza_selftest]=] "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/build-vs/RelWithDebInfo/FrezaEgm_selftest.exe")
  set_tests_properties([=[freza_selftest]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;61;add_test;D:/developments/DSH_workspaces/freza_sirius/VisualStudio/FrezaEgm/CMakeLists.txt;0;")
else()
  add_test([=[freza_selftest]=] NOT_AVAILABLE)
endif()
