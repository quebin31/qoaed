cmake_minimum_required(VERSION 3.7)
project(QOAED CXX)

set(CMAKE_CXX_STANDARD 17)
set(TEST_SRC_DIR ${PROJECT_SOURCE_DIR}/test)
set(TEST_BIN_DIR ${PROJECT_BINARY_DIR}/bin)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Search for pkg-config
find_package(PkgConfig REQUIRED)
pkg_search_module(X11 REQUIRED x11)

include_directories(include)

add_executable(point_quadtree ${TEST_SRC_DIR}/PointQuadtreeTest.cpp)

#test targets using CImg should link with x11 libs
set_target_properties(
  point_quadtree

	PROPERTIES

	RUNTIME_OUTPUT_DIRECTORY "${TEST_BIN_DIR}"
  #LINK_LIBRARIES           "${X11_LIBRARIES}"
)

#test targets using CImg should link with pthread library
#target_link_libraries(test_target pthread)
