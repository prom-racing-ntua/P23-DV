cmake_minimum_required(VERSION 3.5)
project(hesai_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options( -Wpedantic)
endif()

## Find dependencies
find_package( Boost REQUIRED  COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_c REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS io)

ament_export_dependencies(rosidl_default_runtime)
# install(FILES DESTINATION share/${PROJECT_NAME})
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(rosidl_default_runtime)

set(CMAKE_CXX_FLAGS "-fPIC")

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
# find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
# find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})
rosidl_generate_interfaces(${PROJECT_NAME}
  	"msg/PandarPacket.msg"
  	"msg/PandarScan.msg"
  	DEPENDENCIES builtin_interfaces 
)

include_directories(
    src/HesaiLidar_General_SDK/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## PandarGeneralSDK
add_subdirectory(src/HesaiLidar_General_SDK)

## Hesai Lidar Node
add_executable(hesai_lidar_node
    src/main_ros2.cc
)
ament_target_dependencies(hesai_lidar_node
  	ament_index_cpp
  	pcl_conversions
 	rcl_interfaces
  	rclcpp
  	rcutils
  	sensor_msgs
  	std_msgs
  	tf2_geometry_msgs
)
target_link_libraries(hesai_lidar_node 
    ${PCL_IO_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    PandarGeneralSDK
)

rosidl_target_interfaces(hesai_lidar_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS hesai_lidar_node
  	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config pcapFiles
  	DESTINATION share/${PROJECT_NAME}
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
