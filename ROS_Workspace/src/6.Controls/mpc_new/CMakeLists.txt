cmake_minimum_required(VERSION 3.8)
project(mpc_new)
#alvania
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

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(node_logger REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

include_directories(
  include
  include/lifecycle
  custom_msgs
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/mpc_new_node.cpp 
src/mpc_new.cpp
src/cubic_spline.cpp
src/FORCESNLPsolver/FORCESNLPsolver_model.c
src/FORCESNLPsolver/FORCESNLPsolver_model.h
src/FORCESNLPsolver/FORCESNLPsolver_interface.c
src/FORCESNLPsolver/include/FORCESNLPsolver_memory.h
src/FORCESNLPsolver/include/FORCESNLPsolver.h
)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs custom_msgs)
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/FORCESNLPsolver/lib/libFORCESNLPsolver.a)

add_executable(l_mpc_new src/lifecycle/l_mpc_new.cpp src/lifecycle/lifecycle_states.cpp
src/mpc_new.cpp
src/cubic_spline.cpp
src/FORCESNLPsolver/FORCESNLPsolver_model.c
src/FORCESNLPsolver/FORCESNLPsolver_model.h
src/FORCESNLPsolver/FORCESNLPsolver_interface.c
src/FORCESNLPsolver/include/FORCESNLPsolver_memory.h
src/FORCESNLPsolver/include/FORCESNLPsolver.h
)
ament_target_dependencies(l_mpc_new node_logger rclcpp rclcpp_lifecycle custom_msgs)
target_link_libraries(l_mpc_new ${CMAKE_SOURCE_DIR}/src/FORCESNLPsolver/lib/libFORCESNLPsolver.a)

install(TARGETS
  ${PROJECT_NAME}
  l_mpc_new
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
