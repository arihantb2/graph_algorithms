#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <bfs.h>
#include <yaml_interface.h>

data_types::EdgeId data_types::Edge::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: <dfs-executable> <yaml-config-file-path>");

    graph::Graph graph = yaml_loader::loadGraphFromFile(std::string(argv[1]));
    std::cout << graph << std::endl;

    data_types::VertexMap vertexMap = graph.vertices();
    const data_types::VertexId startId = vertexMap.cbegin()->first;

    algorithms::BFS solver(graph, startId);
    solver.solve();

    for (auto idIterator = vertexMap.begin(); idIterator != vertexMap.end(); idIterator++)
    {
        data_types::ShortestPathResult result = solver.reconstructPath(idIterator->first);
        if (result.pathFound_)
        {
            std::cout << "Path found!!\n";
            std::cout << "Distance from " << startId << " --> " << idIterator->first << ": " << result.distance_ << "\n";
            std::cout << "Path: [\n";
            for (const auto id : result.path_)
                std::cout << "\t" << id << "\n";
            std::cout << "]\n";
        }
        else
            std::cout << "Path not found between " << startId << " --> " << idIterator->first << "\n";
    }

    return 0;
}