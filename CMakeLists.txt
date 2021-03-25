cmake_minimum_required(VERSION 3.10.2)
project(graph_algorithms)

set(CMAKE_CXX_STANDARD_REQUIRED 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(GTest REQUIRED)

enable_testing()

include_directories(include ${GTEST_INCLUDE_DIRS})

add_executable(dfs src/dfs.cpp)
add_executable(bfs src/bfs.cpp)
add_executable(connected_components src/connected_components.cpp)


add_executable(GraphManipulationTest src/test/unittest_GraphManipulation.cpp)
target_link_libraries(GraphManipulationTest  ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

add_test(GraphManipulation-Test GraphManipulationTest)