#include <gtest/gtest.h>
#include <graph.h>

template <class T>
data_types::EdgeId data_types::Edge<T>::idCounter_ = 0;
data_types::VertexId data_types::Vertex::idCounter_ = 0;

class GraphManipulationTest : public ::testing::Test
{
protected:
    void SetUp() {}
    void TearDown() {}

public:
    graph::Graph<double> graphd;
};

TEST_F(GraphManipulationTest, VertexInsertion)
{
    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);

    graphd.addVertex(1);
    graphd.addVertex(2);
    graphd.addVertex(3);
    graphd.addVertex(4);

    ASSERT_EQ(graphd.vertices().size(), 4);
    ASSERT_EQ(graphd.edges().size(), 0);

    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);
}

TEST_F(GraphManipulationTest, VertexDeletion)
{
    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);

    graphd.addVertex(1);
    graphd.addVertex(2);
    graphd.addVertex(3);
    graphd.addVertex(4);

    ASSERT_EQ(graphd.vertices().size(), 4);
    ASSERT_EQ(graphd.edges().size(), 0);

    graphd.removeVertex(1);
    graphd.removeVertex(2);
    graphd.removeVertex(3);
    graphd.removeVertex(4);

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);

    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);
}

TEST_F(GraphManipulationTest, EdgeInsertion)
{
    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);

    auto edge1Result = graphd.addEdge(data_types::Edge<double>(1, 2));
    auto edge2Result = graphd.addEdge(data_types::Edge<double>(1, 3));
    auto edge3Result = graphd.addEdge(data_types::Edge<double>(1, 4));

    ASSERT_EQ(graphd.vertices().size(), 4);
    ASSERT_EQ(graphd.edges().size(), 3);

    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);
}

TEST_F(GraphManipulationTest, EdgeDeletion)
{
    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);

    auto edge1Result = graphd.addEdge(data_types::Edge<double>(1, 2));
    auto edge2Result = graphd.addEdge(data_types::Edge<double>(1, 3));
    auto edge3Result = graphd.addEdge(data_types::Edge<double>(1, 4));

    ASSERT_EQ(graphd.vertices().size(), 4);
    ASSERT_EQ(graphd.edges().size(), 3);

    ASSERT_TRUE(graphd.removeEdge(edge1Result.first));
    ASSERT_TRUE(graphd.removeEdge(edge2Result.first));
    ASSERT_TRUE(graphd.removeEdge(edge3Result.first));

    ASSERT_EQ(graphd.vertices().size(), 4);
    ASSERT_EQ(graphd.edges().size(), 0);

    graphd.clear();

    ASSERT_EQ(graphd.vertices().size(), 0);
    ASSERT_EQ(graphd.edges().size(), 0);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
