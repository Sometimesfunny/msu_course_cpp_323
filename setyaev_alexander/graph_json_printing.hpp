#pragma once
#include <string>
#include "graph.hpp"
namespace printing {
namespace json {
std::string color_to_string(const Edge::Color& color);
std::string graph_to_string(const Graph& graph);
std::string vertex_to_string(const Vertex& vertex,
                             Graph::Depth depth,
                             const std::vector<EdgeId>& connected_edges);
std::string edge_to_string(const Edge& edge);
}  // namespace json
}  // namespace printing
