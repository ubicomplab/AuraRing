# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("mag-inverse")
set(CMAKE_MODULE_PATH ${mag-inverse_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Ceres CONFIG REQUIRED PATHS C:/lib/Ceres)
include_directories(${CERES_INCLUDE_DIRS})

find_package(gflags REQUIRED)
include_directories(.)

# Include sub-projects.
#add_subdirectory ("calibration")
add_subdirectory ("mag-runtime")
add_subdirectory ("mag-inverse")
add_subdirectory ("global_model")
