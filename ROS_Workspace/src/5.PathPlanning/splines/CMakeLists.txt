cmake_minimum_required(VERSION 3.5)
project(splines)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  custom_msgs
)


# ----
# Create a spline library and export it so other packages can use it
add_library(${PROJECT_NAME}_lib SHARED
  src/b-spline.cpp 
  src/cubic_spline.cpp
)
# This installs the library to the package and setting its parameters
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}_lib    # Setting the include dir which must be installed
)
# This exports the library so other packages can see it
ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
# The include dir of the library is installed to the package here
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}_lib)


# ----
# Create the spline recorder node executable
add_executable(spline_recorder src/spline_recorder.cpp)
target_link_libraries(spline_recorder ${PROJECT_NAME}_lib)
ament_target_dependencies(spline_recorder rclcpp custom_msgs ament_index_cpp)

install(TARGETS
  spline_recorder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
