CMAKE_MINIMUM_REQUIRED ( VERSION 3.10 )

PROJECT(ray-tracer)

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ext/glog/cmake")
  message(FATAL_ERROR "The glog submodule directory is missing! "
    "You probably did not clone the project with --recursive, or you first checked out "
    "ray-tracer before it was added. It is possible to recover by running "
    "\"git submodule update --init --recursive\"")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ext/CLI11/cmake")
  message(FATAL_ERROR "The CLI11 submodule directory is missing! "
    "You probably did not clone the project with --recursive, or you first checked out "
    "ray-tracer before it was added. It is possible to recover by running "
    "\"git submodule update --init --recursive\"")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ext/assimp/cmake")
  message(FATAL_ERROR "The assimp submodule directory is missing! "
    "You probably did not clone the project with --recursive, or you first checked out "
    "ray-tracer before it was added. It is possible to recover by running "
    "\"git submodule update --init --recursive\"")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion-null")

###########################################################################
# glog

SET(WITH_GFLAGS OFF CACHE BOOL "Use gflags")
SET(BUILD_SHARED_LIBS OFF CACHE BOOL " " FORCE)
ADD_SUBDIRECTORY(ext/glog)
INCLUDE_DIRECTORIES (
  ext/glog/src
  ${CMAKE_BINARY_DIR}/glog
)

###########################################################################
# CLI11

INCLUDE_DIRECTORIES (
    ext/CLI11/include
)

###########################################################################
# assimp

ADD_SUBDIRECTORY(ext/assimp)
INCLUDE_DIRECTORIES(
    ext/assimp/include
    ${CMAKE_BINARY_DIR}/assimp
)

###########################################################################
ADD_EXECUTABLE(ray-tracer main.cpp)
TARGET_LINK_LIBRARIES(ray-tracer assimp glog)