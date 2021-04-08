#include <bfs.h>

namespace algorithms
{
    BFS::BFS(const Graph &graph, const VertexId &startId) : startId_(startId), solved_(false)
    {
        constGraphPtr_ = std::make_shared<const Graph>(graph);
    }

    bool BFS::solve()
    {
        if (constGraphPtr_->vertices().find(startId_) == constGraphPtr_->vertices().end())
            return false;

        VertexIdList vertexQueue;

        visited_.clear();
        previous_.clear();
        for (const auto vertexPair : constGraphPtr_->vertices())
        {
            visited_[vertexPair.first] = false;
            previous_[vertexPair.first] = nullptr;
        }

        visited_[startId_] = true;

        vertexQueue.push_back(startId_);
        while (!vertexQueue.empty())
        {
            VertexId vertexId = vertexQueue.front();
            vertexQueue.pop_front();

            VertexPtr vertexPtr = constGraphPtr_->vertex(vertexId);
            if (!vertexPtr)
                continue;

            for (const auto edgeId : vertexPtr->adjList())
            {
                EdgePtr edgePtr = constGraphPtr_->edge(edgeId);
                if (!edgePtr)
                    continue;

                std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(vertexId);
                if (nextVertex.second == false)
                    continue;

                if (visited_[nextVertex.first])
                    continue;

                vertexQueue.push_back(nextVertex.first);
                visited_[nextVertex.first] = true;
                previous_[nextVertex.first] = std::make_shared<VertexId>(vertexId);
            }
        }

        solved_ = true;
        return true;
    }

    ShortestPathResult BFS::reconstructPath(const VertexId &endId)
    {
        ShortestPathResult result;
        result.pathFound_ = false;

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
            result.distance_ = 1;
            return result;
        }

        while (at)
        {
            result.path_.push_front(*at);
            if ((previous_[*at]) && (*(previous_[*at]) == startId_))
            {
                result.pathFound_ = true;
                result.path_.push_front(startId_);
                result.distance_ = result.path_.size() - 1;
                break;
            }
            at = previous_[*at];
        }

        return result;
    }
}