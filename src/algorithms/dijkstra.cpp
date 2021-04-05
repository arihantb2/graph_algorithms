#include <dijkstra.h>

namespace algorithms
{
    Dijkstra::Dijkstra(const Graph &graph, const VertexId &startId) : startId_(startId), solved_(false)
    {
        constGraphPtr_ = std::make_shared<const Graph>(graph);
    }

    Dijkstra::Dijkstra(const DirectedAcyclicGraph &graph, const VertexId &startId) : startId_(startId), solved_(false)
    {
        constGraphPtr_ = std::make_shared<const Graph>(graph);
    }

    bool Dijkstra::solve()
    {
        if (constGraphPtr_->vertices().find(startId_) == constGraphPtr_->vertices().end())
            return false;

        for (const auto vertexPair : constGraphPtr_->vertices())
        {
            visited_.insert(std::make_pair(vertexPair.first, false));
            distanceMap_.insert(std::make_pair(vertexPair.first, std::numeric_limits<double>::infinity()));
            previous_.insert(std::make_pair(vertexPair.first, nullptr));
        }

        distanceMap_[startId_] = 0.0;

        struct Comparator
        {
            bool operator()(const std::pair<VertexId, double> &lhs, const std::pair<VertexId, double> &rhs)
            {
                return (lhs.second > rhs.second);
            }
        };
        std::priority_queue<std::pair<VertexId, double>, std::vector<std::pair<VertexId, double>>, Comparator> pQueue;
        pQueue.push(std::make_pair(startId_, 0.0));

        while (!pQueue.empty())
        {
            auto vertexPair = pQueue.top();
            pQueue.pop();

            VertexPtr vertexPtr = constGraphPtr_->vertex(vertexPair.first);
            if (!vertexPtr)
                continue;

            visited_[vertexPair.first] = true;
            if (distanceMap_[vertexPair.first] < vertexPair.second)
                continue;

            for (const auto edgeId : vertexPtr->adjList())
            {
                EdgePtr edgePtr = constGraphPtr_->edge(edgeId);
                if (!edgePtr)
                    continue;

                std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(vertexPair.first);
                if (!nextVertex.second)
                    continue;

                if (visited_[nextVertex.first])
                    continue;

                double newDistance = distanceMap_[vertexPair.first] + edgePtr->weight();
                if (newDistance < distanceMap_[nextVertex.first])
                {
                    // Inserting duplicate key-value pair, O(log(n)), is faster than searching for existing key, O(n), and updating its value
                    pQueue.push(std::make_pair(nextVertex.first, newDistance));
                    distanceMap_[nextVertex.first] = newDistance;
                    previous_[nextVertex.first] = std::make_shared<VertexId>(vertexPair.first);
                }
            }
        }

        solved_ = true;
        return true;
    }

    Graph::ShortestPathResult Dijkstra::reconstructPath(const VertexId &endId)
    {
        if (!solved_)
            solve();

        Graph::ShortestPathResult result;
        result.pathFound_ = false;
        result.distance_ = distanceMap_.at(endId);

        result.path_.push_front(endId);
        if (endId == startId_)
        {
            result.pathFound_ = true;
            result.distance_ = 0;
            return result;
        }

        std::shared_ptr<VertexId> at = previous_.at(endId);
        if (at && *at == startId_)
        {
            result.pathFound_ = true;
            result.path_.push_front(startId_);
            return result;
        }

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