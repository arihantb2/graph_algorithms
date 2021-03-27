#include <bits/stdc++.h>

#include <data_types.h>
#include <graph.h>

template <class T>
data_types::EdgeId data_types::Edge<T>::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

int main(int argc, char const *argv[])
{
    graph::Graph<double> graph;

    graph.addVertex(0);
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);
    graph.addVertex(5);
    graph.addVertex(6);
    graph.addVertex(7);
    graph.addVertex(8);
    graph.addVertex(9);
    graph.addVertex(10);
    graph.addVertex(11);
    graph.addVertex(12);

    graph.addEdge(data_types::Edge<double>(0, 7));
    graph.addEdge(data_types::Edge<double>(0, 9));
    graph.addEdge(data_types::Edge<double>(0, 11));
    graph.addEdge(data_types::Edge<double>(1, 8));
    graph.addEdge(data_types::Edge<double>(1, 10));
    graph.addEdge(data_types::Edge<double>(2, 3));
    graph.addEdge(data_types::Edge<double>(2, 12));
    graph.addEdge(data_types::Edge<double>(3, 4));
    graph.addEdge(data_types::Edge<double>(3, 7));
    graph.addEdge(data_types::Edge<double>(5, 6));
    graph.addEdge(data_types::Edge<double>(6, 7));
    graph.addEdge(data_types::Edge<double>(7, 11));
    graph.addEdge(data_types::Edge<double>(8, 9));
    graph.addEdge(data_types::Edge<double>(8, 12));
    graph.addEdge(data_types::Edge<double>(9, 10));

    std::cout << graph << std::endl;

    std::map<data_types::VertexId, std::shared_ptr<data_types::VertexId>> previousVertex;
    data_types::VertexIdList traversalOrder = graph.bfsTraversal(graph.vertices().begin()->first, previousVertex);

    std::cout << "BFS Traversal Order with start vertex [" << graph.vertices().begin()->first << "]: [\n";
    for (const auto vertexId : traversalOrder)
    {
        std::cout << "\t" << vertexId;
        if ((previousVertex.find(vertexId) != previousVertex.end()) && previousVertex[vertexId])
            std::cout << " <-- " << *previousVertex[vertexId] << std::endl;
        else
            std::cout << " <-- nullptr" << std::endl;
    }
    std::cout << "]\n";

    return 0;
}