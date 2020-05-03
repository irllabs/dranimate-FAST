cmake_minimum_required(VERSION 3.1)
project(dranimate-fast)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libigl/cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 --bind -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_ES6=1 -s FILESYSTEM=0 -s ENVIRONMENT='web' -s USE_ES6_IMPORT_META=0 -s LINKABLE=1 -s ASSERTIONS=1")

option(LIBIGL_WITH_TETGEN "Use Tetgen" OFF)

find_package(LIBIGL REQUIRED QUIET)

file(GLOB SRCFILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} igl::core)
