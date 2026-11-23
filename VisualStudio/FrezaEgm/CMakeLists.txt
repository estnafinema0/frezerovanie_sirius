cmake_minimum_required(VERSION 3.20)

# OpenCAMLib's installed package calls find_dependency(Boost), which still relies
# on the FindBoost module. Keep it quiet on CMake >= 3.30 (CMP0167 introduced).
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

project(FrezaEgm CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

set(NANOPB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanopb")
set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
set(DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

include(cmake/Cam.cmake)
option(FREZA_OFFLINE_TESTS_ONLY "Build portable offline tests, never EGM/GUI" OFF)
if(FREZA_OFFLINE_TESTS_ONLY)
    enable_testing()
    add_subdirectory(tests)
    return()
endif()

# --- nanopb + сгенерированный код EGM ---------------------------------------
add_library(nanopb STATIC
    "${NANOPB_DIR}/pb_common.c"
    "${NANOPB_DIR}/pb_encode.c"
    "${NANOPB_DIR}/pb_decode.c"
    "${GENERATED_DIR}/egm3.pb.c")

target_include_directories(nanopb PUBLIC "${NANOPB_DIR}" "${GENERATED_DIR}")

# --- общая логика приложения ------------------------------------------------
add_library(freza_core STATIC
    src/text_io.cpp
    src/trajectory.cpp
    src/csv_io.cpp
    src/egm_client.cpp)

target_include_directories(freza_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(freza_core PUBLIC nanopb ws2_32)

# --- GUI-приложение ---------------------------------------------------------
add_executable(FrezaEgm WIN32
    src/main.cpp
    src/app_window.cpp)

target_link_libraries(FrezaEgm PRIVATE freza_core freza_cam comctl32 comdlg32 shell32)
freza_copy_cam_runtime(FrezaEgm)
target_compile_definitions(FrezaEgm PRIVATE
    UNICODE
    _UNICODE
    FREZA_DATA_DIR="${DATA_DIR}")

# --- самотест (консоль, без окна) -------------------------------------------
add_executable(FrezaEgm_selftest src/selftest.cpp)
target_link_libraries(FrezaEgm_selftest PRIVATE freza_core)
target_compile_definitions(FrezaEgm_selftest PRIVATE FREZA_DATA_DIR="${DATA_DIR}")

# --- настройки MSVC ---------------------------------------------------------
if(MSVC)
    set(MSVC_COMMON /utf-8 /W4 /permissive- /Zc:__cplusplus)
    set(MSVC_DEFS _CRT_SECURE_NO_WARNINGS)

    target_compile_definitions(freza_core PUBLIC ${MSVC_DEFS})
    target_compile_options(freza_core PRIVATE ${MSVC_COMMON})
    target_compile_options(FrezaEgm PRIVATE ${MSVC_COMMON})
    target_compile_options(FrezaEgm_selftest PRIVATE ${MSVC_COMMON})
endif()

# --- тесты ------------------------------------------------------------------
enable_testing()
add_test(NAME freza_selftest COMMAND FrezaEgm_selftest)

add_subdirectory(tests)
