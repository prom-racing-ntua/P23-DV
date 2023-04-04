cmake_minimum_required(VERSION 3.5)
project(path_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-parameter)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

find_package(custom_msgs REQUIRED)

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

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

include_directories(
  include
  custom_msgs
)

add_executable(path_planner src/path_planner.cpp src/triangulation_cgal.cpp)
ament_target_dependencies(path_planner rclcpp std_msgs CGAL custom_msgs)

#add_to_cached_list( CGAL_EXECUTABLE_TARGETS path_planner )

#target_link_libraries(talker PRIVATE CGAL::CGAL )

#SET(TARGET_SRC
#  src/triangulation_cgal.cpp  
#)

install(TARGETS
  path_planner
  DESTINATION lib/path_planner)

install(DIRECTORY
  config launch
  DESTINATION share/path_planner)


ament_package()
