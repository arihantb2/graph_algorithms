#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <yaml_interface.h>

data_types::EdgeId data_types::Edge::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: <dfs-executable> <yaml-config-file-path>");

    graph::Graph graph = yaml_loader::loadGraphFromFile(std::string(argv[1]));
    std::cout << graph << std::endl;

    graph::Graph::DFSResult result = graph.dfsTraversal(graph.vertices().begin()->first);

    std::cout << "DFS Traversal Order with start vertex [" << graph.vertices().begin()->first << "]: [\n";
    for (const auto vertexId : result.traversalOrder_)
        std::cout << "\t" << vertexId << std::endl;
    std::cout << "]\n";

    return 0;
}