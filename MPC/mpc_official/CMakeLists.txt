cmake_minimum_required(VERSION 3.0.0)
project(mpc_cpp VERSION 0.1.0)

include(CTest)
include_directories(.)
# include_directories(FORCESNLPsolver/blasfeo/lib/include)
enable_testing()

add_executable(mpc_cpp interface.cpp
FORCESNLPsolver/FORCESNLPsolver_model.c
FORCESNLPsolver/FORCESNLPsolver_model.h
FORCESNLPsolver/FORCESNLPsolver_interface.c
FORCESNLPsolver/include/FORCESNLPsolver_memory.h
FORCESNLPsolver/include/FORCESNLPsolver.h)

target_link_libraries(mpc_cpp ${CMAKE_SOURCE_DIR}/FORCESNLPsolver/lib/libFORCESNLPsolver.a)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
