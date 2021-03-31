#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <yaml_interface.h>

template <class T>
data_types::EdgeId data_types::Edge<T>::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: <dfs-executable> <yaml-config-file-path>");

    graph::Graph<double> graph = yaml_loader::loadGraphFromFile<double>(std::string(argv[1]));
    std::cout << graph << std::endl;

    data_types::VertexIdList ordering = graph.topologicalSort();

    std::cout << "Topological sort ordering: [\n";
    for (const auto vertexId : ordering)
        std::cout << "\t" << vertexId << std::endl;
    std::cout << "]\n";

    return 0;
}