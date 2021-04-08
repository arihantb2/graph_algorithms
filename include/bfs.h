#pragma once

#include <graph.h>
#include <directed_acyclic_graph.h>
#include <data_types.h>

namespace algorithms
{
    using namespace graph;
    using namespace data_types;

    class BFS
    {
    public:
        BFS() = delete;
        BFS(const Graph &, const VertexId &);

        ~BFS() {}

        /*
         * V: len(vertices) in graph
         * E: len(edges) in graph
         * Time : O(V+E)
         * Space: O(V)
         * BFS algorithm is particularly useful for finding shortest path on unweighted graphs
         */
        bool solve();

        ShortestPathResult reconstructPath(const VertexId &);

    private:
        std::shared_ptr<const Graph> constGraphPtr_;
        VertexIdMap<bool> visited_;
        VertexIdMap<std::shared_ptr<VertexId>> previous_;
        bool solved_;
        VertexId startId_;
    };
}