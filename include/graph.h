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
         * Time : O(V+E)
         * Space: O()
         * Finds connected components in the graph and returns the count and list of components
        */
        ConnectedComponents findConnectedComponents();
        ConnectedComponents findConnectedComponents() const;

        class ShortestPathResult
        {
        public:
            double distance_;
            VertexIdList path_;
            bool pathFound_;
        };

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(E * log(V))
         * Space: O()
         * This computes the shortest path between the two input vertex IDs
         * Note: This is a single source shortest path algorithm (SSSP)
         * This is an implementation of lazy dijkstra which inserts duplicate key-value pairs in the priority queue instead of updating existing key-value pairs
         * The lazy implementation can lead to high space complexity with dense graphs
         * All edges in the graph need to have a non-negative weight which allows this algorithm to act in a greedy manner
        */
        ShortestPathResult dijkstraShortestPath(const VertexId &, const VertexId &);
        ShortestPathResult dijkstraShortestPath(const VertexId &, const VertexId &) const;

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(EV)
         * Space: O()
         * This computes the shortest path between the two input vertex IDs
         * Note: This is a single source shortest path algorithm (SSSP)
         * Bellman-Ford algorithm is useful when graph has negative edge weights and it detects negative cycles
         * However, it has a worse time complexity than Dijkstra's algorithm, O(EV) as compared to O(E * log(V))
         * This algorithm treats each edge as a directed edge
        */
        ShortestPathResult bellmanFordShortestPath(const VertexId &, const VertexId &);
        ShortestPathResult bellmanFordShortestPath(const VertexId &, const VertexId &) const;

    protected:
        void __clear();

        Graph __clone();
        Graph __clone() const;

        std::pair<VertexMap::iterator, bool> __addVertex(const VertexId &);
        std::pair<VertexMap::iterator, bool> __addVertex(const Vertex &);
        std::pair<EdgeId, bool> __addEdge(const Edge &);

        bool __removeVertex(const VertexId &);
        bool __removeEdge(const EdgeId &);

        void __dfsRecursive(const VertexId &, std::map<VertexId, bool> &, VertexIdList &, bool postOrder = false);
        void __dfsRecursive(const VertexId &, std::map<VertexId, bool> &, VertexIdList &, bool postOrder = false) const;

    private:
        VertexMap vertexMap_;
        EdgeMap edgeMap_;
    };
}