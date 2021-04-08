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

    VertexIdList Graph::dfsTraversal(const VertexId &startId)
    {
        VertexIdList result;
        result.clear();

        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsRecursive(startId, visited, result);

        return result;
    }

    VertexIdList Graph::dfsTraversal(const VertexId &startId) const
    {
        VertexIdList result;
        result.clear();

        VertexIdMap<bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsRecursive(startId, visited, result);

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

    std::deque<VertexPair> Graph::findBridges()
    {
        FindBridgesHelper helper;
        VertexIdMap<bool> visited;

        helper.idIndex_ = 0;
        for (const auto vertexPair : vertices())
        {
            helper.idIndexMap_.insert(std::make_pair(vertexPair.first, 0));
            helper.lowLinkIdMap_.insert(std::make_pair(vertexPair.first, 0));
            visited.insert(std::make_pair(vertexPair.first, false));
        }

        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
                __dfsRecursive(vertexPair.first, -1, helper, visited);
        }

        return helper.bridges_;
    }

    std::deque<VertexPair> Graph::findBridges() const
    {
        FindBridgesHelper helper;
        VertexIdMap<bool> visited;

        helper.idIndex_ = 0;
        for (const auto vertexPair : vertices())
        {
            helper.idIndexMap_.insert(std::make_pair(vertexPair.first, 0));
            helper.lowLinkIdMap_.insert(std::make_pair(vertexPair.first, 0));
            visited.insert(std::make_pair(vertexPair.first, false));
        }

        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
                __dfsRecursive(vertexPair.first, -1, helper, visited);
        }

        return helper.bridges_;
    }

    VertexIdList Graph::findArticulationPoints()
    {
        FindArtPointsHelper helper;
        VertexIdMap<bool> visited;

        helper.idIndex_ = 0;
        helper.outEdgeCount_ = 0;
        for (const auto vertexPair : vertices())
        {
            helper.idIndexMap_.insert(std::make_pair(vertexPair.first, 0));
            helper.lowLinkIdMap_.insert(std::make_pair(vertexPair.first, 0));
            visited.insert(std::make_pair(vertexPair.first, false));
        }

        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
            {
                helper.outEdgeCount_ = 0;
                __dfsRecursive(vertexPair.first, vertexPair.first, -1, helper, visited);

                if (helper.outEdgeCount_ > 1)
                    helper.addArtPoint(vertexPair.first);
            }
        }

        return helper.artPoints_;
    }

    VertexIdList Graph::findArticulationPoints() const
    {
        FindArtPointsHelper helper;
        VertexIdMap<bool> visited;

        helper.idIndex_ = 0;
        helper.outEdgeCount_ = 0;
        for (const auto vertexPair : vertices())
        {
            helper.idIndexMap_.insert(std::make_pair(vertexPair.first, 0));
            helper.lowLinkIdMap_.insert(std::make_pair(vertexPair.first, 0));
            visited.insert(std::make_pair(vertexPair.first, false));
        }

        for (const auto vertexPair : vertices())
        {
            if (!visited[vertexPair.first])
            {
                helper.outEdgeCount_ = 0;
                __dfsRecursive(vertexPair.first, vertexPair.first, -1, helper, visited);

                if (helper.outEdgeCount_ > 1)
                    helper.addArtPoint(vertexPair.first);
            }
        }

        return helper.artPoints_;
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

    void Graph::__dfsRecursive(const VertexId &id, const VertexId &parentId, FindBridgesHelper &helper, VertexIdMap<bool> &visited)
    {
        visited[id] = true;
        helper.idIndex_ = helper.idIndex_ + 1;
        helper.lowLinkIdMap_[id] = helper.idIndex_;
        helper.idIndexMap_[id] = helper.idIndex_;

        VertexPtr vertexPtr = vertex(id);
        if (!vertexPtr)
            return;

        for (const auto edgeId : vertexPtr->adjList())
        {
            EdgePtr edgePtr = edge(edgeId);

            if (!edgePtr)
                continue;

            auto vertexPair = edgePtr->getNeighbor(id);

            if (!vertexPair.second)
                continue;

            if (vertexPair.first == parentId)
                continue;

            if (!visited[vertexPair.first])
            {
                __dfsRecursive(vertexPair.first, id, helper, visited);
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.lowLinkIdMap_[vertexPair.first]);
                if (helper.idIndexMap_[id] < helper.lowLinkIdMap_[vertexPair.first])
                    helper.bridges_.push_back(std::make_pair(id, vertexPair.first));
            }

            else
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.idIndexMap_[vertexPair.first]);
        }
    }

    void Graph::__dfsRecursive(const VertexId &id, const VertexId &parentId, FindBridgesHelper &helper, VertexIdMap<bool> &visited) const
    {
        visited[id] = true;
        helper.idIndex_ = helper.idIndex_ + 1;
        helper.lowLinkIdMap_[id] = helper.idIndex_;
        helper.idIndexMap_[id] = helper.idIndex_;

        VertexPtr vertexPtr = vertex(id);
        if (!vertexPtr)
            return;

        for (const auto edgeId : vertexPtr->adjList())
        {
            EdgePtr edgePtr = edge(edgeId);

            if (!edgePtr)
                continue;

            auto vertexPair = edgePtr->getNeighbor(id);

            if (!vertexPair.second)
                continue;

            if (vertexPair.first == parentId)
                continue;

            if (!visited[vertexPair.first])
            {
                __dfsRecursive(vertexPair.first, id, helper, visited);
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.lowLinkIdMap_[vertexPair.first]);
                if (helper.idIndexMap_[id] < helper.lowLinkIdMap_[vertexPair.first])
                    helper.bridges_.push_back(std::make_pair(id, vertexPair.first));
            }

            else
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.idIndexMap_[vertexPair.first]);
        }
    }

    void Graph::__dfsRecursive(const VertexId &rootId, const VertexId &id, const VertexId &parentId, FindArtPointsHelper &helper, VertexIdMap<bool> &visited)
    {
        if (rootId == parentId)
            helper.outEdgeCount_++;

        visited[id] = true;
        helper.idIndex_ = helper.idIndex_ + 1;
        helper.lowLinkIdMap_[id] = helper.idIndex_;
        helper.idIndexMap_[id] = helper.idIndex_;

        VertexPtr vertexPtr = vertex(id);
        if (!vertexPtr)
            return;

        for (const auto edgeId : vertexPtr->adjList())
        {
            EdgePtr edgePtr = edge(edgeId);

            if (!edgePtr)
                continue;

            auto vertexPair = edgePtr->getNeighbor(id);

            if (!vertexPair.second)
                continue;

            if (vertexPair.first == parentId)
                continue;

            if (!visited[vertexPair.first])
            {
                __dfsRecursive(rootId, vertexPair.first, id, helper, visited);
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.lowLinkIdMap_[vertexPair.first]);

                // Add articulation point because it is part of a bridge (less than case) or a cycle (equal to case)
                if (helper.idIndexMap_[id] <= helper.lowLinkIdMap_[vertexPair.first])
                    helper.addArtPoint(id);
            }

            else
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.idIndexMap_[vertexPair.first]);
        }
    }

    void Graph::__dfsRecursive(const VertexId &rootId, const VertexId &id, const VertexId &parentId, FindArtPointsHelper &helper, VertexIdMap<bool> &visited) const
    {
        if (rootId == parentId)
            helper.outEdgeCount_++;

        visited[id] = true;
        helper.idIndex_ = helper.idIndex_ + 1;
        helper.lowLinkIdMap_[id] = helper.idIndex_;
        helper.idIndexMap_[id] = helper.idIndex_;

        VertexPtr vertexPtr = vertex(id);
        if (!vertexPtr)
            return;

        for (const auto edgeId : vertexPtr->adjList())
        {
            EdgePtr edgePtr = edge(edgeId);

            if (!edgePtr)
                continue;

            auto vertexPair = edgePtr->getNeighbor(id);

            if (!vertexPair.second)
                continue;

            if (vertexPair.first == parentId)
                continue;

            if (!visited[vertexPair.first])
            {
                __dfsRecursive(rootId, vertexPair.first, id, helper, visited);
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.lowLinkIdMap_[vertexPair.first]);

                // Add articulation point because it is part of a bridge (less than case) or a cycle (equal to case)
                if (helper.idIndexMap_[id] <= helper.lowLinkIdMap_[vertexPair.first])
                    helper.addArtPoint(id);
            }

            else
                helper.lowLinkIdMap_[id] = std::min(helper.lowLinkIdMap_[id], helper.idIndexMap_[vertexPair.first]);
        }
    }
}