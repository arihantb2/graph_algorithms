#include <graph.h>

namespace graph
{

    Graph::Graph(const EdgeList &edgeList)
    {
        for (const auto edge : edgeList)
            addEdge(edge);
    }

    Graph::Graph(const EdgeMap &edgeMap)
    {
        for (auto edgePair : edgeMap)
            addEdge(edgePair.second);
    }

    Graph::Graph(const Graph &graph)
    {
        for (auto vertexPair : graph.vertices())
            addVertex(vertexPair.second);

        for (auto edgePair : graph.edges())
            addEdge(edgePair.second);
    }

    Graph Graph::__clone()
    {
        Graph clonedGraph;

        for (auto vertexPair : vertices())
            clonedGraph.addVertex(vertexPair.second);

        for (auto edgePair : edges())
            clonedGraph.addEdge(edgePair.second);

        return clonedGraph;
    }

    Graph Graph::__clone() const
    {
        Graph clonedGraph;

        for (auto vertexPair : vertices())
            clonedGraph.addVertex(vertexPair.second);

        for (auto edgePair : edges())
            clonedGraph.addEdge(edgePair.second);

        return clonedGraph;
    }

    void Graph::__clear()
    {
        edgeMap_.clear();
        vertexMap_.clear();
    }

    std::ostream &operator<<(std::ostream &output, const Graph &graph)
    {
        output << "Graph:\n";
        output << "\tNum vertices: " << graph.vertices().size() << "\n";
        output << "\tNum edges   : " << graph.edges().size() << "\n";

        return output;
    }

    std::shared_ptr<Vertex> Graph::vertex(const VertexId &id)
    {
        if (vertexMap_.find(id) != vertexMap_.end())
            return std::make_shared<Vertex>(vertexMap_.at(id));

        return nullptr;
    }

    std::shared_ptr<Vertex> Graph::vertex(const VertexId &id) const
    {
        if (vertexMap_.find(id) != vertexMap_.end())
            return std::make_shared<Vertex>(vertexMap_.at(id));

        return nullptr;
    }

    std::shared_ptr<Edge> Graph::edge(const EdgeId &id)
    {
        if (edgeMap_.find(id) != edgeMap_.end())
            return std::make_shared<Edge>(edgeMap_.at(id));

        return nullptr;
    }

    std::shared_ptr<Edge> Graph::edge(const EdgeId &id) const
    {
        if (edgeMap_.find(id) != edgeMap_.end())
            return std::make_shared<Edge>(edgeMap_.at(id));

        return nullptr;
    }

    std::pair<VertexMap::iterator, bool> Graph::__addVertex(const VertexId &id)
    {
        return __addVertex(Vertex(id));
    }

    std::pair<VertexMap::iterator, bool> Graph::__addVertex(const Vertex &vertex)
    {
        std::pair<VertexMap::iterator, bool> result = vertexMap_.emplace(vertex.id(), vertex);

        return result;
    }

    std::pair<EdgeId, bool> Graph::__addEdge(const Edge &edge)
    {
        if (edgeMap_.find(edge.id()) == edgeMap_.end())
        {
            edgeMap_.emplace(edge.id(), edge);

            VertexPair vertexPair = edge.getVertexIDs();
            VertexMap::iterator srcVertexIt = __addVertex(vertexPair.first).first;
            VertexMap::iterator dstVertexIt = __addVertex(vertexPair.second).first;

            if (!srcVertexIt->second.addEdgeId(edge.id()))
                std::cout << "Could not add edge [" << edge.id() << "] to vertex [" << srcVertexIt->first << "]\n";

            if (edge.directed())
                ;
            else if (!dstVertexIt->second.addEdgeId(edge.id()))
                std::cout << "Could not add edge [" << edge.id() << "] to vertex [" << dstVertexIt->first << "]\n";

            return std::make_pair(edge.id(), true);
        }

        return std::make_pair(edge.id(), false);
    }

    bool Graph::__removeVertex(const VertexId &id)
    {
        std::shared_ptr<Vertex> vertexToRemove = vertex(id);
        if (!vertexToRemove)
            return false;

        for (const auto edgeId : vertexToRemove->adjList())
            __removeEdge(edgeId);

        vertexMap_.erase(vertexToRemove->id());
        return true;
    }

    bool Graph::__removeEdge(const EdgeId &id)
    {
        std::shared_ptr<Edge> edgeToRemove = edge(id);
        if (!edgeToRemove)
            return false;

        edgeMap_.erase(id);

        VertexPair vertexPair = edgeToRemove->getVertexIDs();
        std::shared_ptr<Vertex> srcVertex = vertex(vertexPair.first);
        std::shared_ptr<Vertex> destVertex = vertex(vertexPair.second);

        if (srcVertex)
            srcVertex->removeEdge(edgeToRemove->id());

        if (destVertex)
            destVertex->removeEdge(edgeToRemove->id());

        return true;
    }

    typename Graph::DFSResult Graph::dfsTraversal(const VertexId &startId)
    {
        DFSResult result;
        result.traversalOrder_.clear();

        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsRecursive(startId, visited, result.traversalOrder_);

        return result;
    }

    typename Graph::DFSResult Graph::dfsTraversal(const VertexId &startId) const
    {
        DFSResult result;
        result.traversalOrder_.clear();

        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsRecursive(startId, visited, result.traversalOrder_);

        return result;
    }

    typename Graph::BFSResult Graph::bfsTraversal(const VertexId &startId)
    {
        BFSResult result;
        result.traversalOrder_.clear();

        VertexIdList vertexQueue;

        VertexIdMap<bool> visited;
        visited.clear();
        result.previousVertexMap_.clear();
        for (const auto vertexPair : vertices())
        {
            visited[vertexPair.first] = false;
            result.previousVertexMap_[vertexPair.first] = nullptr;
        }

        visited[startId] = true;

        vertexQueue.push_back(startId);
        while (!vertexQueue.empty())
        {
            VertexId vertexId = vertexQueue.front();
            vertexQueue.pop_front();

            VertexPtr vertexPtr = vertex(vertexId);
            if (!vertexPtr)
                continue;

            result.traversalOrder_.push_back(vertexId);

            for (const auto edgeId : vertexPtr->adjList())
            {
                EdgePtr edgePtr = edge(edgeId);
                if (!edgePtr)
                    continue;

                std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(vertexId);
                if (nextVertex.second == false)
                    continue;

                if (visited[nextVertex.first])
                    continue;

                vertexQueue.push_back(nextVertex.first);
                visited[nextVertex.first] = true;
                result.previousVertexMap_[nextVertex.first] = std::make_shared<VertexId>(vertexId);
            }
        }

        return result;
    }

    typename Graph::BFSResult Graph::bfsTraversal(const VertexId &startId) const
    {
        BFSResult result;
        result.traversalOrder_.clear();

        VertexIdList vertexQueue;

        VertexIdMap<bool> visited;
        visited.clear();
        result.previousVertexMap_.clear();
        for (const auto vertexPair : vertices())
        {
            visited[vertexPair.first] = false;
            result.previousVertexMap_[vertexPair.first] = nullptr;
        }

        visited[startId] = true;

        vertexQueue.push_back(startId);
        while (!vertexQueue.empty())
        {
            VertexId vertexId = vertexQueue.front();
            vertexQueue.pop_front();

            VertexPtr vertexPtr = vertex(vertexId);
            if (!vertexPtr)
                continue;

            result.traversalOrder_.push_back(vertexId);

            for (const auto edgeId : vertexPtr->adjList())
            {
                EdgePtr edgePtr = edge(edgeId);
                if (!edgePtr)
                    continue;

                std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(vertexId);
                if (nextVertex.second == false)
                    continue;

                if (visited[nextVertex.first])
                    continue;

                vertexQueue.push_back(nextVertex.first);
                visited[nextVertex.first] = true;
                result.previousVertexMap_[nextVertex.first] = std::make_shared<VertexId>(vertexId);
            }
        }

        return result;
    }

    typename Graph::ConnectedComponents Graph::findConnectedComponents()
    {
        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        VertexIdList traversalOrder;

        ConnectedComponents connectedComponents;
        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
            {
                traversalOrder.clear();
                __dfsRecursive(vertexPair.first, visited, traversalOrder);

                connectedComponents.count_++;
                connectedComponents.components_.push_back(traversalOrder);
            }
        }

        return connectedComponents;
    }

    typename Graph::ConnectedComponents Graph::findConnectedComponents() const
    {
        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        VertexIdList traversalOrder;

        ConnectedComponents connectedComponents;
        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
            {
                traversalOrder.clear();
                __dfsRecursive(vertexPair.first, visited, traversalOrder);

                connectedComponents.count_++;
                connectedComponents.components_.push_back(traversalOrder);
            }
        }

        return connectedComponents;
    }

    Graph::ShortestPathResult Graph::dijkstraShortestPath(const VertexId &startId, const VertexId &endId)
    {
        Graph::ShortestPathResult result;
        result.distance_ = std::numeric_limits<double>::infinity();
        result.pathFound_ = false;

        if (vertexMap_.find(startId) == vertexMap_.end())
            return result;

        if (vertexMap_.find(endId) == vertexMap_.end())
            return result;

        VertexIdMap<bool> visited;
        VertexIdMap<Weight> distance;
        VertexIdMap<std::shared_ptr<VertexId>> previous;
        for (const auto vertexPair : vertexMap_)
        {
            visited.insert(std::make_pair(vertexPair.first, false));
            distance.insert(std::make_pair(vertexPair.first, std::numeric_limits<double>::infinity()));
            previous.insert(std::make_pair(vertexPair.first, nullptr));
        }

        distance[startId] = 0.0;

        struct Comparator
        {
            bool operator()(const std::pair<VertexId, double> &lhs, const std::pair<VertexId, double> &rhs)
            {
                return (lhs.second > rhs.second);
            }
        };
        std::priority_queue<std::pair<VertexId, double>, std::vector<std::pair<VertexId, double>>, Comparator> pQueue;
        pQueue.push(std::make_pair(startId, 0.0));

        while (!pQueue.empty())
        {
            auto vertexPair = pQueue.top();
            pQueue.pop();

            VertexPtr vertexPtr = vertex(vertexPair.first);
            if (!vertexPtr)
                continue;

            visited[vertexPair.first] = true;
            if (distance[vertexPair.first] < vertexPair.second)
                continue;

            for (const auto edgeId : vertexPtr->adjList())
            {
                EdgePtr edgePtr = edge(edgeId);
                if (!edgePtr)
                    continue;

                std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(vertexPair.first);
                if (!nextVertex.second)
                    continue;

                if (visited[nextVertex.first])
                    continue;

                double newDistance = distance[vertexPair.first] + edgePtr->weight();
                if (newDistance < distance[nextVertex.first])
                {
                    // Inserting duplicate key-value pair, O(log(n)), is faster than searching for existing key, O(n), and updating its value
                    pQueue.push(std::make_pair(nextVertex.first, newDistance));
                    distance[nextVertex.first] = newDistance;
                    previous[nextVertex.first] = std::make_shared<VertexId>(vertexPair.first);
                }

                if (nextVertex.first == endId)
                {
                    result.pathFound_ = true;
                    break;
                }
            }
        }

        if (result.pathFound_)
        {
            std::shared_ptr<VertexId> idPtr = previous[endId];
            result.path_.push_front(endId);
            while (idPtr)
            {
                result.path_.push_front(*idPtr);
                idPtr = previous[*idPtr];
            }

            result.distance_ = distance[endId];
        }

        return result;
    }

    Graph::ShortestPathResult Graph::dijkstraShortestPath(const VertexId &startId, const VertexId &endId) const
    {
        Graph::ShortestPathResult result;
        result.distance_ = std::numeric_limits<double>::infinity();
        result.pathFound_ = false;

        if (vertexMap_.find(startId) == vertexMap_.end())
            return result;

        if (vertexMap_.find(endId) == vertexMap_.end())
            return result;

        VertexIdMap<bool> visited;
        VertexIdMap<Weight> distance;
        VertexIdMap<std::shared_ptr<VertexId>> previous;
        for (const auto vertexPair : vertexMap_)
        {
            visited.insert(std::make_pair(vertexPair.first, false));
            distance.insert(std::make_pair(vertexPair.first, std::numeric_limits<double>::infinity()));
            previous.insert(std::make_pair(vertexPair.first, nullptr));
        }

        distance[startId] = 0.0;

        struct Comparator
        {
            bool operator()(const std::pair<VertexId, double> &lhs, const std::pair<VertexId, double> &rhs)
            {
                return (lhs.second > rhs.second);
            }
        };
        std::priority_queue<std::pair<VertexId, double>, std::vector<std::pair<VertexId, double>>, Comparator> pQueue;
        pQueue.push(std::make_pair(startId, 0.0));

        while (!pQueue.empty())
        {
            auto vertexPair = pQueue.top();
            pQueue.pop();

            VertexPtr vertexPtr = vertex(vertexPair.first);
            if (!vertexPtr)
                continue;

            visited[vertexPair.first] = true;
            if (distance[vertexPair.first] < vertexPair.second)
                continue;

            for (const auto edgeId : vertexPtr->adjList())
            {
                EdgePtr edgePtr = edge(edgeId);
                if (!edgePtr)
                    continue;

                std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(vertexPair.first);
                if (!nextVertex.second)
                    continue;

                if (visited[nextVertex.first])
                    continue;

                double newDistance = distance[vertexPair.first] + edgePtr->weight();
                if (newDistance < distance[nextVertex.first])
                {
                    // Inserting duplicate key-value pair, O(log(n)), is faster than searching for existing key, O(n), and updating its value
                    pQueue.push(std::make_pair(nextVertex.first, newDistance));
                    distance[nextVertex.first] = newDistance;
                    previous[nextVertex.first] = std::make_shared<VertexId>(vertexPair.first);
                }

                if (nextVertex.first == endId)
                {
                    result.pathFound_ = true;
                    break;
                }
            }
        }

        if (result.pathFound_)
        {
            std::shared_ptr<VertexId> idPtr = previous[endId];
            result.path_.push_front(endId);
            while (idPtr)
            {
                result.path_.push_front(*idPtr);
                idPtr = previous[*idPtr];
            }

            result.distance_ = distance[endId];
        }

        return result;
    }

    void Graph::__dfsRecursive(const VertexId &id, VertexIdMap<bool> &visited, VertexIdList &traversalOrder, bool postOrder)
    {
        if (visited[id])
            return;

        VertexPtr vertexPtr = vertex(id);

        if (!vertexPtr)
        {
            std::cout << "Vertex [" << id << "] cannot be read from graph\n";
            return;
        }

        visited[id] = true;

        if (!postOrder)
            traversalOrder.push_back(id);

        for (auto edgeId : vertexPtr->adjList())
        {
            EdgePtr edgePtr = edge(edgeId);
            if (!edgePtr)
            {
                std::cout << "Edge [" << edgeId << "] cannot be read from graph\n";
                continue;
            }

            std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(id);
            if (nextVertex.second == false)
                continue;

            __dfsRecursive(nextVertex.first, visited, traversalOrder, postOrder);
        }

        if (postOrder)
            traversalOrder.push_back(id);
    }

    void Graph::__dfsRecursive(const VertexId &id, VertexIdMap<bool> &visited, VertexIdList &traversalOrder, bool postOrder) const
    {
        if (visited[id])
            return;

        VertexPtr vertexPtr = vertex(id);

        if (!vertexPtr)
        {
            std::cout << "Vertex [" << id << "] cannot be read from graph\n";
            return;
        }

        visited[id] = true;

        if (!postOrder)
            traversalOrder.push_back(id);

        for (auto edgeId : vertexPtr->adjList())
        {
            EdgePtr edgePtr = edge(edgeId);
            if (!edgePtr)
            {
                std::cout << "Edge [" << edgeId << "] cannot be read from graph\n";
                continue;
            }

            std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(id);
            if (nextVertex.second == false)
                continue;

            __dfsRecursive(nextVertex.first, visited, traversalOrder, postOrder);
        }

        if (postOrder)
            traversalOrder.push_back(id);
    }
}