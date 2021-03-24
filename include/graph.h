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
        bool addEdge(const Edge<T> &);

        bool removeVertex(const VertexId &);
        bool removeEdge(const EdgeId &);

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
        VertexIdList dfsTraversal(const VertexId &);
        VertexIdList dfsTraversal(const VertexId &) const;

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
    bool Graph<T>::addEdge(const Edge<T> &edge)
    {
        if (edgeMap_.find(edge.id()) == edgeMap_.end())
        {
            edgeMap_.emplace(edge.id(), edge);

            VertexMap::iterator srcVertexIt = addVertex(Vertex(edge.srcVertexId())).first;
            VertexMap::iterator dstVertexIt = addVertex(Vertex(edge.destVertexId())).first;

            if (!srcVertexIt->second.addEdge(edge.id()))
                std::cout << "Could not add edge [" << edge.id() << "] to vertex [" << srcVertexIt->first << "]\n";

            if (!dstVertexIt->second.addEdge(edge.id()))
                std::cout << "Could not add edge [" << edge.id() << "] to vertex [" << dstVertexIt->first << "]\n";

            return true;
        }

        return false;
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
        std::shared_ptr<Vertex> srcVertex = vertex(edgeToRemove->srcVertexId());
        std::shared_ptr<Vertex> destVertex = vertex(edgeToRemove->destVertexId());

        if (srcVertex)
            srcVertex->removeEdge(edgeToRemove->id());

        if (destVertex)
            destVertex->removeEdge(edgeToRemove->id());

        return true;
    }

    template <class T>
    VertexIdList Graph<T>::dfsTraversal(const VertexId &startId)
    {
        VertexIdList traversalOrder;
        traversalOrder.clear();

        std::map<VertexId, bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsTraversalRecursive(startId, visited, traversalOrder);

        return traversalOrder;
    }

    template <class T>
    VertexIdList Graph<T>::dfsTraversal(const VertexId &startId) const
    {
        VertexIdList traversalOrder;
        traversalOrder.clear();

        std::map<VertexId, bool> visited;
        visited.clear();
        for (auto vertex : vertices())
            visited[vertex.first] = false;

        __dfsTraversalRecursive(startId, visited, traversalOrder);

        return traversalOrder;
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

            if (vertexPtr->id() == edgePtr->srcVertexId())
                __dfsTraversalRecursive(edgePtr->destVertexId(), visited, traversalOrder);
            else
                __dfsTraversalRecursive(edgePtr->srcVertexId(), visited, traversalOrder);
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

            if (vertexPtr->id() == edgePtr->srcVertexId())
                __dfsTraversalRecursive(edgePtr->destVertexId(), visited, traversalOrder);
            else
                __dfsTraversalRecursive(edgePtr->srcVertexId(), visited, traversalOrder);
        }
    }

    template class Graph<double>;
    template class Graph<int>;
}