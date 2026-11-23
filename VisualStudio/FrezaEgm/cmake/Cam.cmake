# STL/CAM is optional. When the pinned dependency has been built into
# out/deps (tools/build_opencamlib.ps1), it is used automatically; on a machine
# without it CAM stays off and the coordinate/CSV/EGM modes keep working.
set(FREZA_CAM_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/out/deps")
set(FREZA_OCL_PREFIX "${FREZA_CAM_DEPS_DIR}/ocl-Release")
set(FREZA_BOOST_ROOT "${FREZA_CAM_DEPS_DIR}/boost_1_87_0")
set(FREZA_OCL_CONFIG "${FREZA_OCL_PREFIX}/share/cmake/OpenCAMLib/OpenCAMLibConfig.cmake")

set(_freza_cam_default OFF)
if(EXISTS "${FREZA_OCL_CONFIG}")
    set(_freza_cam_default ON)
endif()

option(FREZA_ENABLE_CAM "Enable STL contact calculation with OpenCAMLib" ${_freza_cam_default})

add_library(freza_cam STATIC src/cam.cpp)
target_include_directories(freza_cam PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
if(FREZA_ENABLE_CAM)
    if(EXISTS "${FREZA_OCL_CONFIG}")
        list(APPEND CMAKE_PREFIX_PATH "${FREZA_OCL_PREFIX}")
        if(NOT BOOST_ROOT AND EXISTS "${FREZA_BOOST_ROOT}/boost/version.hpp")
            set(BOOST_ROOT "${FREZA_BOOST_ROOT}")
        endif()
    endif()
    find_package(OpenCAMLib CONFIG QUIET)
    if(NOT OpenCAMLib_FOUND)
        message(FATAL_ERROR "FREZA_ENABLE_CAM=ON requires OpenCAMLib. See docs/13-stl-cam.md and tools/build_opencamlib.ps1, or use -DFREZA_ENABLE_CAM=OFF for the original application.")
    endif()
    target_link_libraries(freza_cam PUBLIC OpenCAMLib::ocl)
    target_compile_definitions(freza_cam PRIVATE FREZA_HAS_CAM=1)
    message(STATUS "STL CAM enabled (OpenCAMLib CylCutter / PathDropCutter): ${OpenCAMLib_DIR}")
else()
    message(STATUS "STL CAM disabled; coordinate/CSV/EGM mode remains available")
endif()
if(MSVC)
    target_compile_options(freza_cam PRIVATE /utf-8 /W4 /permissive- /Zc:__cplusplus)
endif()
# An installed MSVC OpenCAMLib DLL must accompany every executable using CAM.
function(freza_copy_cam_runtime target)
    if(WIN32 AND FREZA_ENABLE_CAM)
        add_custom_command(TARGET ${target} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:OpenCAMLib::ocl>" "$<TARGET_FILE_DIR:${target}>" VERBATIM)
    endif()
endfunction()
