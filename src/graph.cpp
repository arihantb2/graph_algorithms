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

        std::map<VertexId, bool> visited;
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

        std::map<VertexId, bool> visited;
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

        std::map<VertexId, bool> visited;
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

        std::map<VertexId, bool> visited;
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
        std::map<VertexId, bool> visited;
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
        std::map<VertexId, bool> visited;
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

    void Graph::__dfsRecursive(const VertexId &id, std::map<VertexId, bool> &visited, VertexIdList &traversalOrder, bool postOrder)
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

    void Graph::__dfsRecursive(const VertexId &id, std::map<VertexId, bool> &visited, VertexIdList &traversalOrder, bool postOrder) const
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