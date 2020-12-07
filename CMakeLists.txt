CMAKE_MINIMUM_REQUIRED ( VERSION 3.10 )

PROJECT(ray-tracer)

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

SET(CMAKE_CXX_STANDARD 17)

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
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
)

ADD_EXECUTABLE(ray-tracer main.cpp scene.cpp intersect.cpp ray_tracer.cpp bvh.cpp)
set_target_properties(ray-tracer PROPERTIES CXX_FLAGS "-fopenmp")
TARGET_LINK_LIBRARIES(ray-tracer assimp)