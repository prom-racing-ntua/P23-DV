cmake_minimum_required(VERSION 3.5)
project(path_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-parameter -std=c++17)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(node_logger REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# CGAL and its components
find_package(CGAL REQUIRED)
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

include_directories(
  include
  custom_msgs
)

add_executable(${PROJECT_NAME} src/path_planner.cpp src/triangulation_cgal.cpp)
add_executable(l_pth src/lifecycle/lifecycle_pathplanning_node.cpp src/triangulation_cgal.cpp src/lifecycle/lifecycle_states.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs CGAL custom_msgs)
ament_target_dependencies(l_pth rclcpp node_logger std_msgs CGAL custom_msgs rclcpp_lifecycle)

#add_to_cached_list( CGAL_EXECUTABLE_TARGETS path_planner )

#target_link_libraries(talker PRIVATE CGAL::CGAL )

#SET(TARGET_SRC
#  src/triangulation_cgal.cpp  
#)

install(TARGETS
  ${PROJECT_NAME} l_pth
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
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