#pragma once

#include <graph.h>

namespace graph
{
    class DirectedAcyclicGraph : public Graph
    {
    public:
        DirectedAcyclicGraph() : Graph() {}
        DirectedAcyclicGraph(const EdgeList &);
        DirectedAcyclicGraph(const EdgeMap &);
        DirectedAcyclicGraph(const DirectedAcyclicGraph &);

        ~DirectedAcyclicGraph() {}

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V+E)
         * Space: O()
         * Topological sort is used to compute program/build dependencies that are represented with DAGs
         * Topological sort is also useful for solving the single source shortest path (SSSP) problem
        */
        VertexIdList topologicalSort();
        VertexIdList topologicalSort() const;

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O()
         * Space: O()
         * Computes the shortest path from given vertex to all other reachable vertices in the DAG
         * Note: This is a single source shortest path algorithm (SSSP)
        */
        std::map<data_types::VertexId, double> shortestPathFrom(const data_types::VertexId &startId);
        std::map<data_types::VertexId, double> shortestPathFrom(const data_types::VertexId &startId) const;
    };

    using DAG = DirectedAcyclicGraph;
}