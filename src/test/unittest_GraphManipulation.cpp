#include <gtest/gtest.h>
#include <graph.h>

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

    ASSERT_TRUE(graphd.vertices().size() == 0);
    ASSERT_TRUE(graphd.edges().size() == 0);

    graphd.addVertex(1);
    graphd.addVertex(2);
    graphd.addVertex(3);
    graphd.addVertex(4);

    ASSERT_TRUE(graphd.vertices().size() == 4);
    ASSERT_TRUE(graphd.edges().size() == 0);

    graphd.clear();

    ASSERT_TRUE(graphd.vertices().size() == 0);
    ASSERT_TRUE(graphd.edges().size() == 0);
}

TEST_F(GraphManipulationTest, VertexDeletion)
{
    ASSERT_TRUE(graphd.vertices().size() == 0);
    ASSERT_TRUE(graphd.edges().size() == 0);

    graphd.addVertex(1);
    graphd.addVertex(2);
    graphd.addVertex(3);
    graphd.addVertex(4);

    ASSERT_TRUE(graphd.vertices().size() == 4);
    ASSERT_TRUE(graphd.edges().size() == 0);

    graphd.removeVertex(1);
    graphd.removeVertex(2);
    graphd.removeVertex(3);
    graphd.removeVertex(4);

    ASSERT_TRUE(graphd.vertices().size() == 0);
    ASSERT_TRUE(graphd.edges().size() == 0);

    graphd.clear();

    ASSERT_TRUE(graphd.vertices().size() == 0);
    ASSERT_TRUE(graphd.edges().size() == 0);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
