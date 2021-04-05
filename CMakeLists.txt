cmake_minimum_required(VERSION 3.10.2)
project(graph_algorithms)

set(CMAKE_CXX_STANDARD_REQUIRED 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(GTest REQUIRED)

enable_testing()

include_directories(include ${GTEST_INCLUDE_DIRS})

# add_executable(dfs src/dfs.cpp)
# target_link_libraries(dfs yaml-cpp)

add_executable(bfs 
  src/bfs.cpp 
  src/graph/graph.cpp
)
target_link_libraries(bfs yaml-cpp)

add_executable(connected_components 
  src/connected_components.cpp 
  src/graph/graph.cpp
)
target_link_libraries(connected_components yaml-cpp)

add_executable(top_sort 
  src/top_sort.cpp 
  src/graph/directed_acyclic_graph.cpp 
  src/graph/graph.cpp
)
target_link_libraries(top_sort yaml-cpp)

add_executable(dag_shortest_path 
  src/dag_shortest_path.cpp 
  src/graph/directed_acyclic_graph.cpp 
  src/graph/graph.cpp
)
target_link_libraries(dag_shortest_path yaml-cpp)

add_executable(dijkstra_shortest_path 
  src/dijkstra_shortest_path.cpp 
  src/graph/graph.cpp
  src/graph/directed_acyclic_graph.cpp
  src/algorithms/dijkstra.cpp
)
target_link_libraries(dijkstra_shortest_path yaml-cpp)

add_executable(bellman_ford_shortest_path 
  src/bellman_ford_shortest_path.cpp 
  src/graph/graph.cpp 
  src/algorithms/bellman_ford.cpp
)
target_link_libraries(bellman_ford_shortest_path yaml-cpp)

add_executable(floyd_warshall_shortest_path 
  src/floyd_warshall_shortest_path.cpp 
  src/graph/graph.cpp 
  src/algorithms/floyd_warshall.cpp
)
target_link_libraries(floyd_warshall_shortest_path yaml-cpp)

add_executable(GraphManipulationTest 
  src/test/unittest_GraphManipulation.cpp 
  src/graph/graph.cpp
)
target_link_libraries(GraphManipulationTest  ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

add_test(GraphManipulation-Test GraphManipulationTest)