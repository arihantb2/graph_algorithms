#pragma once

#include <graph.h>
#include <directed_acyclic_graph.h>
#include <data_types.h>

namespace algorithms
{
    using namespace graph;
    using namespace data_types;

    class Dijkstra
    {
    public:
        Dijkstra() = delete;
        Dijkstra(const Graph &, const VertexId &);
        Dijkstra(const DirectedAcyclicGraph &, const VertexId &);

        ~Dijkstra() {}

        VertexIdMap<Weight> distanceMap() { return distanceMap_; }

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
        bool solve();

        Graph::ShortestPathResult reconstructPath(const VertexId &);

    private:
        std::shared_ptr<const Graph> constGraphPtr_;
        VertexIdMap<bool> visited_;
        VertexIdMap<Weight> distanceMap_;
        VertexIdMap<std::shared_ptr<VertexId>> previous_;
        bool solved_;
        VertexId startId_;
    };
}