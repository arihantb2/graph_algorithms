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
    graph.addVertex(13);
    graph.addVertex(14);
    graph.addVertex(15);
    graph.addVertex(16);
    graph.addVertex(17);

    graph.addEdge(data_types::Edge<double>(0, 4));
    graph.addEdge(data_types::Edge<double>(0, 8));
    graph.addEdge(data_types::Edge<double>(0, 14));
    graph.addEdge(data_types::Edge<double>(0, 13));
    graph.addEdge(data_types::Edge<double>(1, 5));
    graph.addEdge(data_types::Edge<double>(2, 9));
    graph.addEdge(data_types::Edge<double>(2, 15));
    graph.addEdge(data_types::Edge<double>(3, 9));
    graph.addEdge(data_types::Edge<double>(4, 8));
    graph.addEdge(data_types::Edge<double>(5, 16));
    graph.addEdge(data_types::Edge<double>(5, 17));
    graph.addEdge(data_types::Edge<double>(6, 7));
    graph.addEdge(data_types::Edge<double>(6, 11));
    graph.addEdge(data_types::Edge<double>(7, 11));
    graph.addEdge(data_types::Edge<double>(8, 14));
    graph.addEdge(data_types::Edge<double>(9, 15));
    graph.addEdge(data_types::Edge<double>(10, 15));
    graph.addEdge(data_types::Edge<double>(13, 14));

    std::cout << graph << std::endl;

    graph::Graph<double>::ConnectedComponents connectedComponents = graph.findConnectedComponents();
    std::cout << "Count of connected components: [" << connectedComponents.count_ << "]\nComponents:\n";

    for (const auto component : connectedComponents.components_)
    {
        std::cout << "\t[";
        for (auto vertexId : component)
            std::cout << vertexId << ", ";
        std::cout << "]\n";
    }

    return 0;
}