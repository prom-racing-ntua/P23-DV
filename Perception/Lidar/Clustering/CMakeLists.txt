cmake_minimum_required(VERSION 3.0.0)
project(lidar_cluster_cpp VERSION 0.1.0)

include(CTest)
enable_testing()
find_package(PCL 1.13)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(External/Json/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${YAMLCPP_INCLUDE_DIRS})

add_executable(lidar_cluster_cpp src/clustering.cpp
src/clustering.h
src/fast_euclidean_clustering.h
)

target_link_libraries (lidar_cluster_cpp ${PCL_LIBRARIES} ${YAMLCPP_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
