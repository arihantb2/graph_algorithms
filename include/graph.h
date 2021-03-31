#pragma once

#include <data_types.h>

namespace graph
{
    using namespace data_types;

    template <class T = int>
    class Graph
    {
    public:
        Graph() {}
        Graph(const EdgeList<T> &);
        Graph(const EdgeMap<T> &);
        ~Graph() {}

        Graph clone();
        Graph clone() const;

        void clear();

        template <class U>
        friend std::ostream &operator<<(std::ostream &, const Graph<U> &);

        std::shared_ptr<Vertex> vertex(const VertexId &);
        std::shared_ptr<Vertex> vertex(const VertexId &) const;

        std::shared_ptr<Edge<T>> edge(const EdgeId &);
        std::shared_ptr<Edge<T>> edge(const EdgeId &) const;

        VertexMap vertices() { return vertexMap_; }
        VertexMap vertices() const { return vertexMap_; }

        EdgeMap<T> edges() { return edgeMap_; }
        EdgeMap<T> edges() const { return edgeMap_; }

        size_t numVertices() { return vertexMap_.size(); }
        size_t numEdges() { return edgeMap_.size(); }

        std::pair<VertexMap::iterator, bool> addVertex(const Vertex &);
        std::pair<VertexMap::iterator, bool> addVertex(const VertexId &);
        std::pair<EdgeId, bool> addEdge(const Edge<T> &);

        bool removeVertex(const VertexId &);
        bool removeEdge(const EdgeId &);

        class DFSResult
        {
        public:
            VertexIdList traversalOrder_;
        };

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V+E)
         * Space: O(V)
         * DFS algorithm can be used as is or augmented to,
         * * Compute MST
         * * Detect and find cycles
         * * Check if graph is bipartite
         * * Find strongly connected components
         * * Topologically sort nodes of graph
         * * Find bridges and articulation points
         * * Find augmenting paths in a flow network
         * * Generate mazes
        */
        DFSResult dfsTraversal(const VertexId &);

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V+E)
         * Space: O(V)
         * DFS algorithm can be used as is or augmented to,
         * * Compute MST
         * * Detect and find cycles
         * * Check if graph is bipartite
         * * Find strongly connected components
         * * Topologically sort nodes of graph
         * * Find bridges and articulation points
         * * Find augmenting paths in a flow network
         * * Generate mazes
        */
        DFSResult dfsTraversal(const VertexId &) const;

        class BFSResult
        {
        public:
            VertexIdList traversalOrder_;
            std::map<VertexId, std::shared_ptr<VertexId>> previousVertexMap_;
        };

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V+E)
         * Space: O(V)
         * BFS algorithm is particularly useful for finding shortest path on unweighted graphs
        */
        BFSResult bfsTraversal(const VertexId &);

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V+E)
         * Space: O(V)
         * BFS algorithm is particularly useful for finding shortest path on unweighted graphs
        */
        BFSResult bfsTraversal(const VertexId &) const;

        class ConnectedComponents
        {
        public:
            ConnectedComponents() : count_(0) { components_.clear(); }
            ConnectedComponents(const ConnectedComponents &other) : count_(other.count_), components_(other.components_) {}

            ~ConnectedComponents() {}

            size_t count_;
            std::vector<VertexIdList> components_;
        };

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O()
         * Space: O()
        */
        ConnectedComponents findConnectedComponents();
        ConnectedComponents findConnectedComponents() const;

    private:
        void __dfsTraversalRecursive(const VertexId &, std::map<VertexId, bool> &, VertexIdList &);
        void __dfsTraversalRecursive(const VertexId &, std::map<VertexId, bool> &, VertexIdList &) const;

        VertexMap vertexMap_;
        EdgeMap<T> edgeMap_;
    };

    template <class T>
    Graph<T>::Graph(const EdgeList<T> &edgeList)
    {
        for (const auto edge : edgeList)
            addEdge(edge);
    }

    template <class T>
    Graph<T>::Graph(const EdgeMap<T> &edgeMap)
    {
        edgeMap_ = edgeMap;
    }

    template <class T>
    Graph<T> Graph<T>::clone()
    {
        Graph<T> clonedGraph;

        for (auto vertexPair : vertices())
            clonedGraph.addVertex(vertexPair.second);

        for (auto edgePair : edges())
            clonedGraph.addEdge(edgePair.second);

        return clonedGraph;
    }

    template <class T>
    Graph<T> Graph<T>::clone() const
    {
        Graph<T> clonedGraph;

        for (auto vertexPair : vertices())
            clonedGraph.addVertex(vertexPair.second);

        for (auto edgePair : edges())
            clonedGraph.addEdge(edgePair.second);

        return clonedGraph;
    }

    template <class T>
    void Graph<T>::clear()
    {
        edgeMap_.clear();
        vertexMap_.clear();
    }

    template <class U>
    std::ostream &operator<<(std::ostream &output, const Graph<U> &graph)
    {
        output << "Graph:\n";
        output << "\tNum vertices: " << graph.vertices().size() << "\n";
        output << "\tNum edges   : " << graph.edges().size() << "\n";

        return output;
    }

    template <class T>
    std::shared_ptr<Vertex> Graph<T>::vertex(const VertexId &id)
    {
        if (vertexMap_.find(id) != vertexMap_.end())
            return std::make_shared<Vertex>(vertexMap_.at(id));

        return nullptr;
    }

    template <class T>
    std::shared_ptr<Vertex> Graph<T>::vertex(const VertexId &id) const
    {
        if (vertexMap_.find(id) != vertexMap_.end())
            return std::make_shared<Vertex>(vertexMap_.at(id));

        return nullptr;
    }

    template <class T>
    std::shared_ptr<Edge<T>> Graph<T>::edge(const EdgeId &id)
    {
        if (edgeMap_.find(id) != edgeMap_.end())
            return std::make_shared<Edge<T>>(edgeMap_.at(id));

        return nullptr;
    }

    template <class T>
    std::shared_ptr<Edge<T>> Graph<T>::edge(const EdgeId &id) const
    {
        if (edgeMap_.find(id) != edgeMap_.end())
            return std::make_shared<Edge<T>>(edgeMap_.at(id));

        return nullptr;
    }

    template <class T>
    std::pair<VertexMap::iterator, bool> Graph<T>::addVertex(const VertexId &id)
    {
        return addVertex(Vertex(id));
    }

    template <class T>
    std::pair<VertexMap::iterator, bool> Graph<T>::addVertex(const Vertex &vertex)
    {
        std::pair<VertexMap::iterator, bool> result = vertexMap_.emplace(vertex.id(), vertex);

        return result;
    }

    template <class T>
    std::pair<EdgeId, bool> Graph<T>::addEdge(const Edge<T> &edge)
    {
        if (edgeMap_.find(edge.id()) == edgeMap_.end())
        {
            edgeMap_.emplace(edge.id(), edge);

            VertexPair vertexPair = edge.getVertexIDs();
            VertexMap::iterator srcVertexIt = addVertex(vertexPair.first).first;
            VertexMap::iterator dstVertexIt = addVertex(vertexPair.second).first;

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

    template <class T>
    bool Graph<T>::removeVertex(const VertexId &id)
    {
        std::shared_ptr<Vertex> vertexToRemove = vertex(id);
        if (!vertexToRemove)
            return false;

        for (const auto edgeId : vertexToRemove->adjList())
            removeEdge(edgeId);

        vertexMap_.erase(vertexToRemove->id());
        return true;
    }

    template <class T>
    bool Graph<T>::removeEdge(const EdgeId &id)
    {
        std::shared_ptr<Edge<T>> edgeToRemove = edge(id);
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

    template <class T>
    typename Graph<T>::DFSResult Graph<T>::dfsTraversal(const VertexId &startId)
    {
        DFSResult result;
        result.traversalOrder_.clear();

        std::map<VertexId, bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsTraversalRecursive(startId, visited, result.traversalOrder_);

        return result;
    }

    template <class T>
    typename Graph<T>::DFSResult Graph<T>::dfsTraversal(const VertexId &startId) const
    {
        DFSResult result;
        result.traversalOrder_.clear();

        std::map<VertexId, bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsTraversalRecursive(startId, visited, result.traversalOrder_);

        return result;
    }

    template <class T>
    typename Graph<T>::BFSResult Graph<T>::bfsTraversal(const VertexId &startId)
    {
        BFSResult result;
        result.traversalOrder_.clear();

        std::deque<VertexId> vertexQueue;

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
                EdgePtr<T> edgePtr = edge(edgeId);
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

    template <class T>
    typename Graph<T>::BFSResult Graph<T>::bfsTraversal(const VertexId &startId) const
    {
        BFSResult result;
        result.traversalOrder_.clear();

        std::deque<VertexId> vertexQueue;

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
                EdgePtr<T> edgePtr = edge(edgeId);
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

    template <class T>
    typename Graph<T>::ConnectedComponents Graph<T>::findConnectedComponents()
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
                __dfsTraversalRecursive(vertexPair.first, visited, traversalOrder);

                connectedComponents.count_++;
                connectedComponents.components_.push_back(traversalOrder);
            }
        }

        return connectedComponents;
    }

    template <class T>
    void Graph<T>::__dfsTraversalRecursive(const VertexId &id, std::map<VertexId, bool> &visited, VertexIdList &traversalOrder)
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
        traversalOrder.push_back(id);

        for (auto edgeId : vertexPtr->adjList())
        {
            EdgePtr<T> edgePtr = edge(edgeId);
            if (!edgePtr)
            {
                std::cout << "Edge [" << edgeId << "] cannot be read from graph\n";
                continue;
            }

            std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(id);
            if (nextVertex.second == false)
                continue;

            __dfsTraversalRecursive(nextVertex.first, visited, traversalOrder);
        }
    }

    template <class T>
    void Graph<T>::__dfsTraversalRecursive(const VertexId &id, std::map<VertexId, bool> &visited, VertexIdList &traversalOrder) const
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
        traversalOrder.push_back(id);

        for (auto edgeId : vertexPtr->adjList())
        {
            EdgePtr<T> edgePtr = edge(edgeId);
            if (!edgePtr)
            {
                std::cout << "Edge [" << edgeId << "] cannot be read from graph\n";
                continue;
            }

            std::pair<VertexId, bool> nextVertex = edgePtr->getNeighbor(id);
            if (nextVertex.second == false)
                continue;

            __dfsTraversalRecursive(nextVertex.first, visited, traversalOrder);
        }
    }

    template class Graph<double>;
    template class Graph<int>;
}