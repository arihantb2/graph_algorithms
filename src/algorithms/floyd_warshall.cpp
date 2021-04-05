#include <floyd_warshall.h>

namespace algorithms
{
    FloydWarshall::FloydWarshall(const Graph &graph)
    {
        adjMatrix_.clear();
        next_.clear();
        vertexList_.clear();

        for (const auto vertexPair : graph.vertices())
        {
            adjMatrix_.insert(std::make_pair(vertexPair.first, VertexIdMap<Weight>{}));
            next_.insert(std::make_pair(vertexPair.first, VertexIdMap<std::shared_ptr<VertexId>>{}));
            vertexList_.push_back(vertexPair.first);
            for (const auto vertexPair2 : graph.vertices())
            {
                next_.at(vertexPair.first).insert(std::make_pair(vertexPair2.first, nullptr));
                if (vertexPair.first == vertexPair2.first)
                    adjMatrix_.at(vertexPair.first).insert(std::make_pair(vertexPair2.first, 0.0));
                else
                    adjMatrix_.at(vertexPair.first).insert(std::make_pair(vertexPair2.first, std::numeric_limits<double>::infinity()));
            }
        }

        for (const auto edge : graph.edges())
        {
            VertexPair vertexPair = edge.second.getVertexIDs();
            adjMatrix_.at(vertexPair.first).at(vertexPair.second) = edge.second.weight();
            next_.at(vertexPair.first).at(vertexPair.second) = std::make_shared<VertexId>(vertexPair.second);
            if (!edge.second.directed())
            {
                adjMatrix_.at(vertexPair.second).at(vertexPair.first) = edge.second.weight();
                next_.at(vertexPair.second).at(vertexPair.first) = std::make_shared<VertexId>(vertexPair.first);
            }
        }

        solved_ = false;
    }

    FloydWarshall::FloydWarshall(const DirectedAcyclicGraph &graph)
    {
        adjMatrix_.clear();
        next_.clear();
        vertexList_.clear();

        for (const auto vertexPair : graph.vertices())
        {
            adjMatrix_.insert(std::make_pair(vertexPair.first, VertexIdMap<Weight>{}));
            next_.insert(std::make_pair(vertexPair.first, VertexIdMap<std::shared_ptr<VertexId>>{}));
            vertexList_.push_back(vertexPair.first);
            for (const auto vertexPair2 : graph.vertices())
            {
                next_.at(vertexPair.first).insert(std::make_pair(vertexPair2.first, nullptr));
                if (vertexPair.first == vertexPair2.first)
                    adjMatrix_.at(vertexPair.first).insert(std::make_pair(vertexPair2.first, 0.0));
                else
                    adjMatrix_.at(vertexPair.first).insert(std::make_pair(vertexPair2.first, std::numeric_limits<double>::infinity()));
            }
        }

        for (const auto edge : graph.edges())
        {
            VertexPair vertexPair = edge.second.getVertexIDs();
            adjMatrix_.at(vertexPair.first).at(vertexPair.second) = edge.second.weight();
            next_.at(vertexPair.first).at(vertexPair.second) = std::make_shared<VertexId>(vertexPair.second);
            if (!edge.second.directed())
            {
                adjMatrix_.at(vertexPair.second).at(vertexPair.first) = edge.second.weight();
                next_.at(vertexPair.second).at(vertexPair.first) = std::make_shared<VertexId>(vertexPair.first);
            }
        }

        solved_ = false;
    }

    void FloydWarshall::solve()
    {
        // Space: O(V^3) solution using a vector of 2D matrices (length = num of vertices)
        // std::vector<AdjMatrix_> dp;
        // dp.push_back(adjMatrix);
        // dp[k][i][j] = min(dp[k-1][i][j], dp[k-1][i][k] + dp[k-1][k][j]) for k = 0...V-1

        // Space: O(V^2) solution using a single 2D matrix
        // dp[i][j] = min(dp[i][j], dp[i][k] + dp[k][j])

        dp_ = adjMatrix_;
        for (const auto vertexK : vertexList_)
        {
            for (const auto vertexI : vertexList_)
            {
                for (const auto vertexJ : vertexList_)
                {
                    if (dp_[vertexI][vertexK] + dp_[vertexK][vertexJ] < dp_[vertexI][vertexJ])
                    {
                        dp_[vertexI][vertexJ] = dp_[vertexI][vertexK] + dp_[vertexK][vertexJ];
                        next_[vertexI][vertexJ] = next_[vertexI][vertexK];
                    }
                }
            }
        }

        for (const auto vertexK : vertexList_)
        {
            for (const auto vertexI : vertexList_)
            {
                for (const auto vertexJ : vertexList_)
                {
                    if (dp_[vertexI][vertexK] + dp_[vertexK][vertexJ] < dp_[vertexI][vertexJ])
                    {
                        dp_[vertexI][vertexJ] = -std::numeric_limits<double>::infinity();
                        next_[vertexI][vertexJ] = nullptr;
                    }
                }
            }
        }

        solved_ = true;
    }

    Graph::ShortestPathResult FloydWarshall::reconstructPath(const VertexId &startId, const VertexId &endId)
    {
        if (!solved_)
            solve();

        Graph::ShortestPathResult result;
        result.path_.clear();
        result.distance_ = std::numeric_limits<double>::infinity();
        result.pathFound_ = false;

        if (std::find(vertexList_.begin(), vertexList_.end(), startId) == vertexList_.end())
            return result;

        if (std::find(vertexList_.begin(), vertexList_.end(), endId) == vertexList_.end())
            return result;

        if (dp_[startId][endId] == std::numeric_limits<double>::infinity())
            return result;

        VertexIdList path;
        std::shared_ptr<VertexId> at = std::make_shared<VertexId>(startId);
        for (; (!at || *at != endId); at = next_[*at][endId])
        {
            if (at == nullptr)
                return result;

            path.push_back(*at);
        }

        path.push_back(endId);
        result.path_ = path;
        result.pathFound_ = true;
        result.distance_ = dp_[startId][endId];

        return result;
    }
}