#pragma once

#include <data_types.h>

namespace graph
{
    using namespace data_types;

    class Graph
    {
    public:
        Graph() {}
        Graph(const EdgeList &);
        Graph(const EdgeMap &);
        Graph(const Graph &);
        virtual ~Graph() {}

        virtual Graph clone() { return __clone(); }
        virtual Graph clone() const { return __clone(); }

        virtual void clear() { return __clear(); }

        friend std::ostream &operator<<(std::ostream &, const Graph &);

        std::shared_ptr<Vertex> vertex(const VertexId &);
        std::shared_ptr<Vertex> vertex(const VertexId &) const;

        std::shared_ptr<Edge> edge(const EdgeId &);
        std::shared_ptr<Edge> edge(const EdgeId &) const;

        VertexMap vertices() { return vertexMap_; }
        VertexMap vertices() const { return vertexMap_; }

        EdgeMap edges() { return edgeMap_; }
        EdgeMap edges() const { return edgeMap_; }

        size_t numVertices() { return vertexMap_.size(); }
        size_t numVertices() const { return vertexMap_.size(); }

        size_t numEdges() { return edgeMap_.size(); }
        size_t numEdges() const { return edgeMap_.size(); }

        virtual std::pair<VertexMap::iterator, bool> addVertex(const VertexId &id) { return __addVertex(id); }
        virtual std::pair<VertexMap::iterator, bool> addVertex(const Vertex &vertex) { return __addVertex(vertex); }
        virtual std::pair<EdgeId, bool> addEdge(const Edge &edge) { return __addEdge(edge); }

        virtual bool removeVertex(const VertexId &id) { return __removeVertex(id); }
        virtual bool removeEdge(const EdgeId &id) { return __removeEdge(id); }

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
            VertexIdMap<std::shared_ptr<VertexId>> previousVertexMap_;
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
         * Time : O(V+E)
         * Space: O()
         * Finds connected components in the graph and returns the count and list of components
         */
        ConnectedComponents findConnectedComponents();
        ConnectedComponents findConnectedComponents() const;

        class ShortestPathResult
        {
        public:
            Weight distance_;
            VertexIdList path_;
            bool pathFound_;
        };

    protected:
        void __clear();

        Graph __clone();
        Graph __clone() const;

        std::pair<VertexMap::iterator, bool> __addVertex(const VertexId &);
        std::pair<VertexMap::iterator, bool> __addVertex(const Vertex &);
        std::pair<EdgeId, bool> __addEdge(const Edge &);

        bool __removeVertex(const VertexId &);
        bool __removeEdge(const EdgeId &);

        void __dfsRecursive(const VertexId &, VertexIdMap<bool> &, VertexIdList &, bool postOrder = false);
        void __dfsRecursive(const VertexId &, VertexIdMap<bool> &, VertexIdList &, bool postOrder = false) const;

    private:
        VertexMap vertexMap_;
        EdgeMap edgeMap_;
    };
}