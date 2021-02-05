cmake_minimum_required(VERSION 3.0)
project(pnp CXX) # defines ${CMAKE_PROJECT_NAME}(top level) PROJECT_NAME(most recent)

include(common.cmake)
message("================================================================================")



add_definitions("-std=c++14")
set(CMAKE_CXX_FLAGS "-fPIC")
include_directories(".")
include_directories("lambdatwist-p3p/utils")

find_package(Ceres REQUIRED )

install(DIRECTORY "${CMAKE_SOURCE_DIR}/"
        DESTINATION "include"
        FILES_MATCHING
        PATTERN "*.h")

add_library(${PROJECT_NAME} utils/mlibtime.cpp pnp_ransac.cpp p4p.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
#target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

add_executable(main test_pnp.cpp)
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

option(WITH_PYBIND11 "with pybind11 bindings" OFF)
if(WITH_PYBIND11)
    message("requires that submodules are inited! git submodule init && git submodule update")
    add_subdirectory(pybind11)
    pybind11_add_module(pnp_python_binding pnp_python_binding.cpp)
    target_link_libraries(pnp_python_binding PUBLIC ${PROJECT_NAME} ${CERES_LIBRARIES})
    # OBS! Pybind11 require that you specify python exectuable to build for, e.g. cmake .. -DPYTHON_EXECUTABLE=python3
endif()

message("================================================================================")
