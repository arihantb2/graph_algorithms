
#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <floyd_warshall.h>
#include <yaml_interface.h>

data_types::EdgeId data_types::Edge::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: <floyd-warshall-executable> <yaml-config-file-path>");

    graph::Graph graph = yaml_loader::loadGraphFromFile(std::string(argv[1]));
    std::cout << graph << std::endl;

    const data_types::VertexId startId = graph.vertices().cbegin()->first;
    const data_types::VertexId endId = graph.vertices().crbegin()->first;

    algorithms::FloydWarshall solver(graph);
    solver.solve();

    graph::Graph::ShortestPathResult result = solver.reconstructPath(startId, endId);
    if (result.pathFound_)
    {
        std::cout << "Path found!!\n";
        std::cout << "Distance from " << startId << " --> " << endId << ": " << result.distance_ << "\n";
        std::cout << "Path: [\n";
        for (const auto id : result.path_)
            std::cout << "\t" << id << "\n";
        std::cout << "]\n";
    }
    else
        std::cout << "Path not found between " << startId << " --> " << endId << "\n";

    return 0;
}