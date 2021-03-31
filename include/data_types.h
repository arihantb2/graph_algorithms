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

    using VertexIdList = std::deque<VertexId>;
    using EdgeIdList = std::deque<EdgeId>;

    using VertexPair = std::pair<VertexId, VertexId>;
    using VertexList = std::deque<Vertex>;
    using VertexMap = std::map<VertexId, Vertex>;

    template <class T>
    using EdgeList = std::deque<Edge<T>>;

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

        virtual bool addEdgeId(const EdgeId &id)
        {
            auto edgeIt = std::find(adjList_.begin(), adjList_.end(), id);
            if (edgeIt == adjList_.end())
            {
                adjList_.push_back(id);
                return true;
            }

            return false;
        }

        virtual bool removeEdge(const EdgeId &id)
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
        Edge(VertexId srcId, VertexId dstId, T wt = T(1), bool directed = false) : id_(idCounter_++),
                                                                                   srcId_(srcId),
                                                                                   dstId_(dstId),
                                                                                   weight_(wt),
                                                                                   directed_(directed) {}
        Edge(const Vertex &src, const Vertex &dest, T wt = T(1), bool directed = false) : id_(idCounter_++),
                                                                                          srcId_(src.id_),
                                                                                          dstId_(dest.id_),
                                                                                          weight_(wt),
                                                                                          directed_(directed) {}
        Edge(const Edge &other) : id_(other.id_),
                                  srcId_(other.srcId_),
                                  dstId_(other.dstId_),
                                  weight_(other.weight_),
                                  directed_(other.directed()) {}

        ~Edge() {}

        EdgeId id() { return id_; }
        EdgeId id() const { return id_; }

        bool directed() { return directed_; }
        bool directed() const { return directed_; }

        T weight() { return weight_; }
        T weight() const { return weight_; }

        VertexPair getVertexIDs() { return std::make_pair(srcId_, dstId_); }
        VertexPair getVertexIDs() const { return std::make_pair(srcId_, dstId_); }

        std::pair<VertexId, bool> getNeighbor(const VertexId &id)
        {
            if (id == srcId_)
                return std::make_pair(dstId_, true);

            else if ((id == dstId_) && (directed_ == false))
                return std::make_pair(srcId_, true);

            return std::make_pair(id, false);
        }

        std::pair<VertexId, bool> getNeighbor(const VertexId &id) const
        {
            if (id == srcId_)
                return std::make_pair(dstId_, true);

            else if ((id == dstId_) && (directed_ == false))
                return std::make_pair(srcId_, true);

            return std::make_pair(id, false);
        }

    protected:
        static EdgeId idCounter_;
        EdgeId id_;
        VertexId srcId_;
        VertexId dstId_;
        T weight_;
        bool directed_;
    };
}