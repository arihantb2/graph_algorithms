#pragma once

#include <bits/stdc++.h>

namespace data_types
{
    using VertexId = size_t;
    using EdgeId = size_t;

    class Vertex;

    template <class T>
    class Edge;

    using VertexPtr = std::shared_ptr<Vertex>;

    template <class T>
    using EdgePtr = std::shared_ptr<Edge<T>>;

    using VertexIdList = std::vector<VertexId>;
    using EdgeIdList = std::vector<EdgeId>;

    using VertexList = std::vector<Vertex>;
    using VertexMap = std::map<VertexId, Vertex>;

    template <class T>
    using EdgeList = std::vector<Edge<T>>;

    template <class T>
    using EdgeMap = std::map<EdgeId, Edge<T>>;

    class Vertex
    {
    public:
        Vertex() : id_(idCounter_++) { adjList_.clear(); }
        Vertex(VertexId id) : id_(id) { adjList_.clear(); }
        Vertex(const Vertex &other) : id_(other.id_), adjList_(other.adjList()) {}

        ~Vertex() {}

        VertexId id() { return id_; }
        VertexId id() const { return id_; }

        EdgeIdList adjList() { return adjList_; }
        EdgeIdList adjList() const { return adjList_; }

        bool addEdge(const EdgeId &id)
        {
            auto edgeIt = std::find(adjList_.begin(), adjList_.end(), id);
            if (edgeIt == adjList_.end())
            {
                adjList_.push_back(id);
                return true;
            }

            return false;
        }

        bool removeEdge(const EdgeId &id)
        {
            auto edgeIt = std::find(adjList_.begin(), adjList_.end(), id);
            if (edgeIt != adjList_.end())
            {
                adjList_.erase(edgeIt);
                return true;
            }

            return false;
        }

    private:
        static VertexId idCounter_;
        VertexId id_;
        EdgeIdList adjList_;
    };

    template <class T = int>
    class Edge
    {
    public:
        Edge() = delete;
        Edge(VertexId srcId, VertexId destId, T wt = T(1)) : id_(idCounter_++),
                                                             srcId_(srcId),
                                                             destId_(destId),
                                                             weight_(wt) {}
        Edge(const Vertex &src, const Vertex &dest, T wt = T(1)) : id_(idCounter_++),
                                                                   srcId_(src.id_),
                                                                   destId_(dest.id_),
                                                                   weight_(wt) {}
        Edge(const Edge &other) : id_(other.id_),
                                  srcId_(other.srcId_),
                                  destId_(other.destId_),
                                  weight_(other.weight_) {}

        ~Edge() {}

        static EdgeId idCounter_;
        EdgeId id_;
        VertexId srcId_;
        VertexId destId_;
        T weight_;
    };
}