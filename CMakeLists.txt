cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME OpenDoorRL)
set(PROJECT_DESCRIPTION "OpenDoorRL")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(mc_mujoco REQUIRED)
execute_process(COMMAND python3 -c "import sys; print(\"{}.{}\".format(sys.version_info.major, sys.version_info.minor));" OUTPUT_VARIABLE PYTHON_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(Python3 ${PYTHON_VERSION} EXACT COMPONENTS Interpreter Development REQUIRED)
add_subdirectory(ext/pybind11 EXCLUDE_FROM_ALL)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/OpenDoorRL.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/OpenDoorRL.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/OpenDoorRL.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

