#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <yaml_interface.h>

data_types::EdgeId data_types::Edge::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;
int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage:<bfs-executable> <yaml-config-file-path>");

    graph::Graph graph = yaml_loader::loadGraphFromFile(std::string(argv[1]));
    std::cout << graph << std::endl;

    graph::Graph::ConnectedComponents connectedComponents = graph.findConnectedComponents();
    std::cout << "Count of connected components: [" << connectedComponents.count_ << "]\nComponents:\n";

    for (const auto component : connectedComponents.components_)
    {
        std::cout << "\t[";
        for (auto vertexId : component)
            std::cout << vertexId << ", ";
        std::cout << "]\n";
    }

    return 0;
}