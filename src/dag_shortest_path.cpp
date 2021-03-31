#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>
#include <yaml_interface.h>

template <class T>
data_types::EdgeId data_types::Edge<T>::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

template <class T>
std::map<data_types::VertexId, T> dagShortestPath(const graph::Graph<T> &, const data_types::VertexId &);

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: <dfs-executable> <yaml-config-file-path>");

    graph::Graph<double> graph = yaml_loader::loadGraphFromFile<double>(std::string(argv[1]));
    std::cout << graph << std::endl;

    auto distances = dagShortestPath(graph, graph.vertices().begin()->first);

    std::cout << "Distances from start vertex: [" << graph.vertices().begin()->first << "] is: [\n";
    for (const auto distancePair : distances)
        std::cout << "\t 0 --> " << distancePair.first << ": " << distancePair.second << "\n";
    std::cout << "]\n";

    return 0;
}

template <class T>
std::map<data_types::VertexId, T> dagShortestPath(const graph::Graph<T> &graph, const data_types::VertexId &startId)
{
    std::map<data_types::VertexId, T> distanceFromStart;
    for (const auto vertexPair : graph.vertices())
        distanceFromStart.emplace(vertexPair.first, std::numeric_limits<T>::max());

    if (distanceFromStart.find(startId) == distanceFromStart.end())
    {
        std::cout << "Start vertex: [] does not exist in graph. Cannot compute shortest path\n";
        return distanceFromStart;
    }

    distanceFromStart[startId] = T(0.0);

    data_types::VertexIdList ordering = graph.topologicalSort();
    for (const auto vertexId : ordering)
    {
        data_types::VertexPtr vertexPtr = graph.vertex(vertexId);

        if (!vertexPtr)
            continue;

        data_types::EdgeIdList edgeList = vertexPtr->adjList();
        for (const auto edgeId : edgeList)
        {
            data_types::EdgePtr<T> edgePtr = graph.edge(edgeId);
            if (!edgePtr)
                continue;

            std::pair<data_types::VertexId, bool> neighbor = edgePtr->getNeighbor(vertexId);
            if (!neighbor.second)
                continue;

            distanceFromStart[neighbor.first] = std::min(distanceFromStart[neighbor.first], distanceFromStart[vertexId] + edgePtr->weight());
        }
    }

    return distanceFromStart;
}
