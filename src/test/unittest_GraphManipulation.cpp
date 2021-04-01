#include <gtest/gtest.h>
#include <graph.h>

data_types::EdgeId data_types::Edge::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

class GraphManipulationTest : public ::testing::Test
{
protected:
    void SetUp() {}
    void TearDown() {}

public:
    graph::Graph graph;
};

TEST_F(GraphManipulationTest, VertexInsertion)
{
    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);

    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);

    ASSERT_EQ(graph.vertices().size(), 4);
    ASSERT_EQ(graph.edges().size(), 0);

    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);
}

TEST_F(GraphManipulationTest, VertexDeletion)
{
    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);

    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);

    ASSERT_EQ(graph.vertices().size(), 4);
    ASSERT_EQ(graph.edges().size(), 0);

    graph.removeVertex(1);
    graph.removeVertex(2);
    graph.removeVertex(3);
    graph.removeVertex(4);

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);

    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);
}

TEST_F(GraphManipulationTest, EdgeInsertion)
{
    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);

    auto edge1Result = graph.addEdge(data_types::Edge(1, 2));
    auto edge2Result = graph.addEdge(data_types::Edge(1, 3));
    auto edge3Result = graph.addEdge(data_types::Edge(1, 4));

    ASSERT_EQ(graph.vertices().size(), 4);
    ASSERT_EQ(graph.edges().size(), 3);

    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);
}

TEST_F(GraphManipulationTest, EdgeDeletion)
{
    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);

    auto edge1Result = graph.addEdge(data_types::Edge(1, 2));
    auto edge2Result = graph.addEdge(data_types::Edge(1, 3));
    auto edge3Result = graph.addEdge(data_types::Edge(1, 4));

    ASSERT_EQ(graph.vertices().size(), 4);
    ASSERT_EQ(graph.edges().size(), 3);

    ASSERT_TRUE(graph.removeEdge(edge1Result.first));
    ASSERT_TRUE(graph.removeEdge(edge2Result.first));
    ASSERT_TRUE(graph.removeEdge(edge3Result.first));

    ASSERT_EQ(graph.vertices().size(), 4);
    ASSERT_EQ(graph.edges().size(), 0);

    graph.clear();

    ASSERT_EQ(graph.vertices().size(), 0);
    ASSERT_EQ(graph.edges().size(), 0);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
