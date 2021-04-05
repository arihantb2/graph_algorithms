#include <directed_acyclic_graph.h>

namespace graph
{

    VertexIdList DirectedAcyclicGraph::topologicalSort()
    {
        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        VertexIdList ordering;
        VertexIdList postTraversal;

        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
            {
                postTraversal.clear();
                __dfsRecursive(vertexPair.first, visited, postTraversal, true);

                for (const auto vertexId : postTraversal)
                    ordering.push_front(vertexId);
            }
        }

        return ordering;
    }

    VertexIdList DirectedAcyclicGraph::topologicalSort() const
    {
        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        VertexIdList ordering;
        VertexIdList postTraversal;

        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
            {
                postTraversal.clear();
                __dfsRecursive(vertexPair.first, visited, postTraversal, true);

                for (const auto vertexId : postTraversal)
                    ordering.push_front(vertexId);
            }
        }

        return ordering;
    }

    VertexIdMap<Weight> DirectedAcyclicGraph::shortestPathFrom(const data_types::VertexId &startId)
    {
        VertexIdMap<Weight> distanceFromStart;
        for (const auto vertexPair : vertices())
            distanceFromStart.emplace(vertexPair.first, std::numeric_limits<Weight>::infinity());

        if (distanceFromStart.find(startId) == distanceFromStart.end())
        {
            std::cout << "Start vertex: [] does not exist in graph. Cannot compute shortest path\n";
            return distanceFromStart;
        }

        distanceFromStart[startId] = 0.0;

        data_types::VertexIdList ordering = topologicalSort();
        for (const auto vertexId : ordering)
        {
            data_types::VertexPtr vertexPtr = vertex(vertexId);

            if (!vertexPtr)
                continue;

            data_types::EdgeIdList edgeList = vertexPtr->adjList();
            for (const auto edgeId : edgeList)
            {
                data_types::EdgePtr edgePtr = edge(edgeId);
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

    VertexIdMap<Weight> DirectedAcyclicGraph::shortestPathFrom(const data_types::VertexId &startId) const
    {
        VertexIdMap<Weight> distanceFromStart;
        for (const auto vertexPair : vertices())
            distanceFromStart.emplace(vertexPair.first, std::numeric_limits<Weight>::infinity());

        if (distanceFromStart.find(startId) == distanceFromStart.end())
        {
            std::cout << "Start vertex: [] does not exist in graph. Cannot compute shortest path\n";
            return distanceFromStart;
        }

        distanceFromStart[startId] = 0.0;

        data_types::VertexIdList ordering = topologicalSort();
        for (const auto vertexId : ordering)
        {
            data_types::VertexPtr vertexPtr = vertex(vertexId);

            if (!vertexPtr)
                continue;

            data_types::EdgeIdList edgeList = vertexPtr->adjList();
            for (const auto edgeId : edgeList)
            {
                data_types::EdgePtr edgePtr = edge(edgeId);
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
}