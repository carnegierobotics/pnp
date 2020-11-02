cmake_minimum_required(VERSION 3.0)
project(lambdatwist-pnp CXX) # defines ${CMAKE_PROJECT_NAME}(top level) PROJECT_NAME(most recent)

include(common.cmake)
message("================================================================================")



add_definitions("-std=c++11")
set(CMAKE_CXX_FLAGS "-fPIC")
include_directories(".")
include_directories("lambdatwist-p3p/utils")

find_package(Ceres REQUIRED )

add_library(pnp utils/mlibtime.cpp pnp_ransac.cpp p4p.cpp)
target_include_directories(pnp PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(pnp ${CERES_LIBRARIES})
#target_compile_features(pnp PUBLIC cxx_std_11)

add_executable(main test_pnp.cpp)
target_link_libraries(main pnp)

install(TARGETS pnp
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

install(DIRECTORY .
    DESTINATION include)

option(WITH_PYBIND11 "with pybind11 bindings" OFF)
if(WITH_PYBIND11)
    message("requires that submodules are inited! git submodule init && git submodule update")
    add_subdirectory(pybind11)
    pybind11_add_module(pnp_python_binding pnp_python_binding.cpp)
    target_link_libraries(pnp_python_binding PUBLIC pnp ${CERES_LIBRARIES})
    # OBS! Pybind11 require that you specify python exectuable to build for, e.g. cmake .. -DPYTHON_EXECUTABLE=python3
endif()

message("================================================================================")
