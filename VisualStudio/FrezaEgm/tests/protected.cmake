find_program(GIT_EXECUTABLE git REQUIRED)
get_filename_component(repo "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
# Проверка возможна только в git-копии проекта: без .git базовый коммит 2f259357
# недоступен. В перенесённой копии без репозитория тест не должен падать —
# он сообщает об этом и завершается успешно.
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --is-inside-work-tree
    WORKING_DIRECTORY "${repo}" RESULT_VARIABLE git_status OUTPUT_VARIABLE git_output ERROR_QUIET)
string(STRIP "${git_output}" git_output)
if(NOT git_status EQUAL 0 OR NOT git_output STREQUAL "true")
    message(STATUS "Protected baseline skipped: ${repo} is not a git checkout (no .git). Restore the repository to compare against 2f259357.")
    return()
endif()
# RobotStudio перезаписывает и удаляет свой автосейв при каждом открытии станции;
# это не исходник и не конфигурация, поэтому из проверки он исключён.
execute_process(COMMAND "${GIT_EXECUTABLE}" diff --exit-code 2f259357c2a276c98c353471207110a2ddcae1fb --
    VisualStudio/FrezaEgm/src/egm_client.cpp VisualStudio/FrezaEgm/include/egm_client.h
    Egm/egm3.proto Egm/egm3.options VisualStudio/FrezaEgm/generated
    VisualStudio/FrezaEgm/third_party/nanopb RobotStudio
    ":(exclude)RobotStudio/FrezaEGM/Stations/*.rsautosave"
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
