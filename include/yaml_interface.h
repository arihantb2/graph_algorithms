#include <yaml-cpp/yaml.h>
#include <graph.h>

namespace yaml_loader
{
    template <class T>
    static graph::Graph<T> loadGraphFromFile(std::string filename)
    {
        using namespace YAML;
        Node config = LoadFile(filename);
        assert(config.IsDefined() && config.IsMap());

        graph::Graph<T> graph;
        try
        {
            assert(config["graph"].IsMap());
            Node graphConfig = config["graph"];

            assert(graphConfig["vertices"].IsSequence());
            std::vector<T> vertices = graphConfig["vertices"].as<std::vector<T>>();
            for (const auto vertex : vertices)
                graph.addVertex(vertex);

            assert(graphConfig["edges"].IsSequence());
            Node edges = graphConfig["edges"];
            for (const auto edge : edges)
            {
                assert(edge.IsMap());
                graph.addEdge(data_types::Edge<T>(edge["src"].as<int>(),
                                                  edge["dst"].as<int>(),
                                                  edge["wt"].as<T>(),
                                                  edge["dir"].as<bool>()));
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in reading graph config. Error: " << e.what() << std::endl;
        }

        return graph;
    }

}