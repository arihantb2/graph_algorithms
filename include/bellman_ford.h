#pragma once

#include <graph.h>
#include <directed_acyclic_graph.h>
#include <data_types.h>

namespace algorithms
{
    using namespace graph;
    using namespace data_types;

    class BellmanFord
    {
    public:
        BellmanFord() = delete;
        BellmanFord(const Graph &, const VertexId &);
        BellmanFord(const DirectedAcyclicGraph &, const VertexId &);

        ~BellmanFord() {}

        VertexIdMap<Weight> distanceMap() { return distanceMap_; }

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(EV)
         * Space: O()
         * This computes the shortest path from a given vertex ID to all other vertices in the graph
         * Note: This is a single source shortest path algorithm (SSSP)
         * Bellman-Ford algorithm is useful when graph has negative edge weights and it detects negative cycles
         * However, it has a worse time complexity than Dijkstra's algorithm, O(EV) as compared to O(E * log(V))
         * This algorithm treats each edge as a directed edge
         */
        bool solve();

        Graph::ShortestPathResult reconstructPath(const VertexId &);

    private:
        std::shared_ptr<const Graph> constGraphPtr_;
        VertexIdMap<Weight> distanceMap_;
        VertexIdMap<std::shared_ptr<VertexId>> previous_;
        bool solved_;
        VertexId startId_;
    };
}