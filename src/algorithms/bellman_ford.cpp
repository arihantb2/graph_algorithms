#include <bellman_ford.h>

namespace algorithms
{
    BellmanFord::BellmanFord(const Graph &graph, const VertexId &startId) : startId_(startId), solved_(false)
    {
        constGraphPtr_ = std::make_shared<const Graph>(graph);
    }

    BellmanFord::BellmanFord(const DirectedAcyclicGraph &graph, const VertexId &startId) : startId_(startId), solved_(false)
    {
        constGraphPtr_ = std::make_shared<const Graph>(graph);
    }

    bool BellmanFord::solve()
    {
        std::cout << "\033[1;31m[WARNING]\033[0m: Bellman Ford Shortest Path treats all edges in graph as directed edges\n";

        for (const auto vertexPair : constGraphPtr_->vertices())
        {
            distanceMap_.insert(std::make_pair(vertexPair.first, std::numeric_limits<double>::infinity()));
            previous_.insert(std::make_pair(vertexPair.first, nullptr));
        }

        distanceMap_[startId_] = 0.0;

        if (constGraphPtr_->vertices().find(startId_) == constGraphPtr_->vertices().end())
            return false;

        EdgeId edgeCount = constGraphPtr_->numEdges();
        for (size_t i = 0; i < edgeCount - 1; i++)
        {
            for (const auto edge : constGraphPtr_->edges())
            {
                VertexPair vertexPair = edge.second.getVertexIDs();
                if (distanceMap_[vertexPair.first] + edge.second.weight() < distanceMap_[vertexPair.second])
                {
                    distanceMap_[vertexPair.second] = distanceMap_[vertexPair.first] + edge.second.weight();
                    previous_[vertexPair.second] = std::make_shared<VertexId>(vertexPair.first);
                }
            }
        }

        for (size_t i = 0; i < edgeCount - 1; i++)
        {
            for (const auto edge : constGraphPtr_->edges())
            {
                VertexPair vertexPair = edge.second.getVertexIDs();
                if (distanceMap_[vertexPair.first] + edge.second.weight() < distanceMap_[vertexPair.second])
                {
                    distanceMap_[vertexPair.second] = -std::numeric_limits<double>::infinity();
                    previous_[vertexPair.second] = nullptr;
                }
            }
        }

        return true;
    }

    Graph::ShortestPathResult BellmanFord::reconstructPath(const VertexId &endId)
    {
        Graph::ShortestPathResult result;
        result.pathFound_ = false;
        result.distance_ = distanceMap_.at(endId);

        result.path_.push_front(endId);

        std::shared_ptr<VertexId> at = previous_.at(endId);
        while (at)
        {
            result.path_.push_front(*at);
            if ((previous_[*at]) && (*(previous_[*at]) == startId_))
            {
                result.pathFound_ = true;
                result.path_.push_front(startId_);
                break;
            }
            at = previous_[*at];
        }

        return result;
    }
}