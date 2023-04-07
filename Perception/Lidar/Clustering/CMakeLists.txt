cmake_minimum_required(VERSION 3.0.0)
project(lidar_cluster_cpp VERSION 0.1.0)

include(CTest)
enable_testing()
find_package(PCL 1.13)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_cluster_cpp main.cpp
fast_euclidean_clustering.h
)
target_link_libraries (lidar_cluster_cpp ${PCL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
