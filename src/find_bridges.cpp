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

    std::deque<data_types::VertexPair> bridges = graph.findBridges();
    std::cout << "Bridges found in the graph: [" << graph.vertices().begin()->first << "]: [\n";
    for (const auto vertexPair : bridges)
        std::cout << "\t(" << vertexPair.first << ", " << vertexPair.second << ")\n";
    std::cout << "]\n";

    data_types::VertexIdList artPoints = graph.findArticulationPoints();
    std::cout << "Articulation points found in the graph: [" << graph.vertices().begin()->first << "]: [\n";
    for (const auto vertexId : artPoints)
        std::cout << "\t" << vertexId << "\n";
    std::cout << "]\n";

    return 0;
}