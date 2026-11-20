cmake_minimum_required(VERSION 3.20)

project(FrezaEgm CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

set(NANOPB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanopb")
set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
set(DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

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

target_link_libraries(FrezaEgm PRIVATE freza_core comctl32)
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
