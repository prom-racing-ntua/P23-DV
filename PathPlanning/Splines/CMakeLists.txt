cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cubic_spline.o
    src/cubic_spline_main.cpp
    src/cubic_spline.cpp
)

add_executable(b_spline.o
    src/b-spline_main.cpp
    src/b-spline.cpp
)

add_executable(arc_length_spline.o
    src/arc_length_spline_main.cpp
    src/cubic_spline.cpp
)