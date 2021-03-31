#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <yaml_interface.h>

template <class T>
data_types::EdgeId data_types::Edge<T>::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage:<bfs-executable> <yaml-config-file-path>");

    graph::Graph<double> graph = yaml_loader::loadGraphFromFile<double>(std::string(argv[1]));
    std::cout << graph << std::endl;

    graph::Graph<double>::BFSResult result = graph.bfsTraversal(graph.vertices().begin()->first);

    std::cout << "BFS Traversal Order with start vertex [" << graph.vertices().begin()->first << "]: [\n";
    for (const auto vertexId : result.traversalOrder_)
    {
        std::cout << "\t" << vertexId;
        if ((result.previousVertexMap_.find(vertexId) != result.previousVertexMap_.end()) && result.previousVertexMap_[vertexId])
            std::cout << " <-- " << *result.previousVertexMap_[vertexId] << std::endl;
        else
            std::cout << " <-- nullptr" << std::endl;
    }
    std::cout << "]\n";

    return 0;
}