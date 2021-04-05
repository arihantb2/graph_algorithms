#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <directed_acyclic_graph.h>
#include <yaml_interface.h>

data_types::EdgeId data_types::Edge::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;


int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: <dag-shortest-path-executable> <yaml-config-file-path>");

    graph::DAG graph = yaml_loader::loadDAGFromFile(std::string(argv[1]));
    std::cout << graph << std::endl;

    data_types::VertexIdMap<data_types::Weight> distances = graph.shortestPathFrom(graph.vertices().begin()->first);

    std::cout << "Distances from start vertex: [" << graph.vertices().begin()->first << "] is: [\n";
    for (const auto distancePair : distances)
        std::cout << "\t 0 --> " << distancePair.first << ": " << distancePair.second << "\n";
    std::cout << "]\n";

    return 0;
}
