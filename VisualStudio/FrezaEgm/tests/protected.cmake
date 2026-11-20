find_program(GIT_EXECUTABLE git REQUIRED)
get_filename_component(repo "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
execute_process(COMMAND "${GIT_EXECUTABLE}" diff --exit-code 2f259357c2a276c98c353471207110a2ddcae1fb --
    VisualStudio/FrezaEgm/src/egm_client.cpp VisualStudio/FrezaEgm/include/egm_client.h
    Egm/egm3.proto Egm/egm3.options VisualStudio/FrezaEgm/generated
    VisualStudio/FrezaEgm/third_party/nanopb RobotStudio
    VisualStudio/FrezaEgm/src/trajectory.cpp VisualStudio/FrezaEgm/include/trajectory.h
    VisualStudio/FrezaEgm/src/csv_io.cpp VisualStudio/FrezaEgm/include/csv_io.h
    VisualStudio/FrezaEgm/src/text_io.cpp VisualStudio/FrezaEgm/src/selftest.cpp
    VisualStudio/FrezaEgm/data/trajectory.gcode VisualStudio/FrezaEgm/data/trajectory.xyz
    VisualStudio/FrezaEgm/data/points.csv VisualStudio/FrezaEgm/data/points_selftest.csv
    WORKING_DIRECTORY "${repo}" RESULT_VARIABLE status OUTPUT_VARIABLE diff ERROR_VARIABLE error)
if(NOT status EQUAL 0)
    message(FATAL_ERROR "Protected baseline differs or baseline commit unavailable: ${diff}${error}")
endif()
message(STATUS "Protected sources, station/controller, parsers/interpolation and original data match 2f259357 exactly (Git-normalized line endings).")
