#pragma once

#include <graph.h>
#include <directed_acyclic_graph.h>

namespace algorithms
{
    using namespace graph;
    using namespace data_types;

    class FloydWarshall
    {
    public:
        FloydWarshall() = delete;
        FloydWarshall(const Graph &);
        FloydWarshall(const DirectedAcyclicGraph &);

        ~FloydWarshall() {}

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V^3)
         * Space: O(V^2)
         * This computes the shortest path between all pairs of vertices in the graph
         * Note: This is an all-pairs shortest path algorithm (APSP)
         * This algorithm is ideal for small graphs with no more than a couple of hundred vertices and it detects negative cycles
         */
        void solve();

        Graph::ShortestPathResult reconstructPath(const VertexId &, const VertexId &);

    private:
        AdjMatrix<Weight> adjMatrix_;
        AdjMatrix<std::shared_ptr<VertexId>> next_;
        AdjMatrix<Weight> dp_;
        VertexIdList vertexList_;
        bool solved_;
    };
}