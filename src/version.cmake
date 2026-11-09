# version.cmake - generate version.h from the latest git tag
#
# Run in script mode on every build:
#   cmake -DVERSION_H=<output file> -DSRC_DIR=<source dir> -P version.cmake
#
# Falls back to FW_VERSION_DEFAULT if git is unavailable or the source
# is not a git repository. The file is only rewritten if its content
# changed, so unchanged versions don't trigger a rebuild.

set(FW_VERSION_DEFAULT "<unknown>")

if(NOT VERSION_H)
  message(FATAL_ERROR "version.cmake: VERSION_H not set")
endif()
if(NOT SRC_DIR)
  get_filename_component(SRC_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
endif()

set(FW_VERSION "${FW_VERSION_DEFAULT}")

find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty
    WORKING_DIRECTORY "${SRC_DIR}"
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_RESULT)
  if(GIT_RESULT EQUAL 0 AND GIT_VERSION)
    set(FW_VERSION "${GIT_VERSION}")
  endif()
endif()

set(CONTENT "#ifndef VERSION_H
#define VERSION_H
#define FW_VERSION \"${FW_VERSION}\"
#endif
")

if(EXISTS "${VERSION_H}")
  file(READ "${VERSION_H}" OLD_CONTENT)
else()
  set(OLD_CONTENT "")
endif()

if(NOT "${CONTENT}" STREQUAL "${OLD_CONTENT}")
  file(WRITE "${VERSION_H}" "${CONTENT}")
  message(STATUS "version.h: FW_VERSION=${FW_VERSION}")
endif()
