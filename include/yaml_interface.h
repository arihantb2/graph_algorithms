#include <yaml-cpp/yaml.h>
#include <graph.h>
#include <directed_acyclic_graph.h>

namespace yaml_loader
{
    static graph::Graph loadGraphFromFile(std::string filename)
    {
        using namespace YAML;
        Node config = LoadFile(filename);
        assert(config.IsDefined() && config.IsMap());

        graph::Graph graph;
        try
        {
            assert(config["graph"].IsMap());
            Node graphConfig = config["graph"];

            assert(graphConfig["vertices"].IsSequence());
            std::vector<int> vertices = graphConfig["vertices"].as<std::vector<int>>();
            for (const auto vertex : vertices)
                graph.addVertex(vertex);

            assert(graphConfig["edges"].IsSequence());
            Node edges = graphConfig["edges"];
            for (const auto edge : edges)
            {
                assert(edge.IsMap());
                graph.addEdge(data_types::Edge(edge["src"].as<int>(),
                                                  edge["dst"].as<int>(),
                                                  edge["wt"].as<double>(),
                                                  edge["dir"].as<bool>()));
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in reading graph config. Error: " << e.what() << std::endl;
        }

        return graph;
    }

    static graph::DAG loadDAGFromFile(std::string filename)
    {
        using namespace YAML;
        Node config = LoadFile(filename);
        assert(config.IsDefined() && config.IsMap());

        graph::DAG graph;
        try
        {
            assert(config["graph"].IsMap());
            Node graphConfig = config["graph"];

            assert(graphConfig["vertices"].IsSequence());
            std::vector<int> vertices = graphConfig["vertices"].as<std::vector<int>>();
            for (const auto vertex : vertices)
                graph.addVertex(vertex);

            assert(graphConfig["edges"].IsSequence());
            Node edges = graphConfig["edges"];
            for (const auto edge : edges)
            {
                assert(edge.IsMap());
                graph.addEdge(data_types::Edge(edge["src"].as<int>(),
                                                  edge["dst"].as<int>(),
                                                  edge["wt"].as<double>(),
                                                  true));
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in reading graph config. Error: " << e.what() << std::endl;
        }

        return graph;
    }

}