cmake_minimum_required(VERSION 3.8)
project(node_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(include)
include_directories(include)

# set(HEADER_FILES include/node_logger/node_logger.hpp)

ament_python_install_package(${PROJECT_NAME})
# Create a spline library and export it so other packages can use it

add_library(${PROJECT_NAME}_lib SHARED
  src/node_logger.cpp 
  node_logger/node_logger.py
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
