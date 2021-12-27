#include <cassert>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <unordered_map>
#include <vector>

class Graph {
 public:
  using VertexId = int;
  using EdgeId = int;
  using Depth = int;

  static constexpr int INVALID_ID = -1;

  struct Vertex {
    explicit Vertex(const VertexId& _id) : id_(_id) {}

    VertexId id() const { return id_; }
    Depth depth() const { return depth_; }
    Depth set_depth(const Depth depth) {
      depth_ = depth;
      return depth_;
    }

   private:
    const VertexId id_ = INVALID_ID;
    Depth depth_ = 0;
  };

  struct Edge {
    enum class Color { Grey, Green, Yellow, Red };

    Edge(const EdgeId& _id,
         const Color& _color,
         const VertexId& _from_vertex_id,
         const VertexId& _to_vertex_id)
        : id_(_id),
          color_(_color),
          from_vertex_id_(_from_vertex_id),
          to_vertex_id_(_to_vertex_id) {}
    EdgeId id() const { return id_; }
    VertexId from_vertex_id() const { return from_vertex_id_; }
    VertexId to_vertex_id() const { return to_vertex_id_; }
    Color color() const { return color_; }

   private:
    const Color color_ = Color::Grey;
    const EdgeId id_ = INVALID_ID;
    const VertexId from_vertex_id_ = INVALID_ID;
    const VertexId to_vertex_id_ = INVALID_ID;
  };

  // Graph functions
  const VertexId get_vertex_id() { return current_vertex_id++; }
  const EdgeId get_edge_id() { return current_edge_id++; }
  const EdgeId & current_id() const { return current_edge_id; }
  Edge::Color get_color(const EdgeId edge_id) const {
    assert(edge_id >= 0);
    assert(edge_id < current_edge_id);
    return edges_[edge_id].color();
  }

  Vertex& add_vertex() {
    const VertexId new_vertex_id = get_vertex_id();
    adjacency_list_[new_vertex_id] = {};
    return vertices_.emplace_back(new_vertex_id);
  }

  Edge& add_edge(const VertexId& from_vertex_id, const VertexId& to_vertex_id) {
    assert(from_vertex_id < current_vertex_id);
    assert(from_vertex_id >= 0);
    assert(to_vertex_id < current_vertex_id);
    assert(to_vertex_id >= 0);
    const EdgeId new_edge_id = get_edge_id();
    Edge::Color edge_color = Edge::Color::Grey;
    if (from_vertex_id == to_vertex_id)
      edge_color = Edge::Color::Green;
    else if (adjacency_list_.at(to_vertex_id).empty())
      edge_color = Edge::Color::Grey;
    else if (!adjacency_list_.at(to_vertex_id).empty() &&
            (abs(get_vertex_depth(to_vertex_id) - get_vertex_depth(from_vertex_id)) == 1))
      edge_color = Edge::Color::Yellow;
    else if (abs(get_vertex_depth(to_vertex_id) - get_vertex_depth(from_vertex_id)) == 2)
      edge_color = Edge::Color::Red;
    else
      throw std::runtime_error("Failed to determine color");
    adjacency_list_[to_vertex_id].emplace(new_edge_id);
    adjacency_list_[from_vertex_id].emplace(new_edge_id);
    return edges_.emplace_back(new_edge_id, edge_color , from_vertex_id, to_vertex_id);
  }

  const std::set<EdgeId>& get_edges_ids(const VertexId vertex_id) const {
    return adjacency_list_.at(vertex_id);
  }

  Depth get_vertex_depth(const VertexId vertex_id) {
    assert(vertex_id < current_vertex_id);
    assert(vertex_id >= 0);
    return vertices_[vertex_id].depth();
  }

  Depth set_vertex_depth(const VertexId to_vertex_id,
                         const Depth vertex_depth) {
    assert(to_vertex_id < current_vertex_id);
    assert(to_vertex_id >= 0);
    Depth depth = vertices_[to_vertex_id].set_depth(vertex_depth);
    return depth;
  }

  bool is_connected(const VertexId from_vertex_id,
                    const VertexId to_vertex_id) const {
    for (const auto& edge_id : adjacency_list_.at(from_vertex_id)) {
      if (edges_[edge_id].to_vertex_id() == to_vertex_id &&
              edges_[edge_id].from_vertex_id() == from_vertex_id ||
          edges_[edge_id].to_vertex_id() == from_vertex_id &&
              edges_[edge_id].from_vertex_id() == to_vertex_id)
        return true;
    }
    return false;
  }

  std::vector<VertexId> vertices_on_depth(Depth depth) const {
    std::vector<VertexId> valid_vertices_ids;
    Depth prev_depth = 0;
    for (const auto& vertex : vertices_) {
      if (vertex.depth() > depth)
        return valid_vertices_ids;
      if (vertex.depth() == depth)
        valid_vertices_ids.push_back(vertex.id());
    }
    return valid_vertices_ids;
  }

  const std::vector<Vertex>& vertices() const { return vertices_; }
  const std::vector<Edge>& edges() const { return edges_; }

 private:
  std::unordered_map<VertexId, std::set<EdgeId>> adjacency_list_;
  std::vector<Vertex> vertices_;
  std::vector<Edge> edges_;
  EdgeId current_edge_id = 0;
  VertexId current_vertex_id = 0;
};

class GraphGenerator {
 public:
  struct Params {
   public:
    explicit Params(Graph::Depth depth = 0, int new_vertices_count = 0)
        : depth_(depth), new_vertices_count_(new_vertices_count) {}

    Graph::Depth depth() const { return depth_; }
    int new_vertices_count() const { return new_vertices_count_; }

   private:
    Graph::Depth depth_ = 0;
    int new_vertices_count_ = 0;
  };

  explicit GraphGenerator(const Params& params = Params()) : params_(params) {}

  int generate_grey_edges(Graph& graph) const {
    std::random_device random_device;
    std::mt19937 generator(random_device());
    Graph::Depth depth = params_.depth();
    int new_vertices_count = params_.new_vertices_count();
    auto vertices_on_current_depth = graph.vertices();
    std::vector<Graph::Vertex> vertices_on_next_depth;
    for (Graph::Depth current_depth = 0; current_depth < depth; current_depth++) {
      std::bernoulli_distribution distribution((double)(depth - current_depth) /
                                               depth);
      vertices_on_next_depth.clear();
      for (const auto& vertex : vertices_on_current_depth)
        for (int vertex_n = 0; vertex_n < new_vertices_count; vertex_n++) {
          if (distribution(generator)) {
            auto& new_vertex = graph.add_vertex();
            graph.set_vertex_depth(new_vertex.id(), vertex.depth() + 1);
            vertices_on_next_depth.push_back(new_vertex);
            graph.add_edge(vertex.id(), new_vertex.id());
          }
        }
      vertices_on_current_depth.swap(vertices_on_next_depth);
    }
    return 0;
  }

  int generate_green_edges(Graph& graph) const {
    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::bernoulli_distribution distribution(green_edge_probability);
    for (const auto& vertex : graph.vertices()) {
      if (distribution(generator))
        graph.add_edge(vertex.id(), vertex.id());
    }
    return 0;
  }

  int generate_yellow_edges(Graph& graph) const {
    std::random_device random_device;
    std::mt19937 generator(random_device());
    Graph::Depth depth = params_.depth();
    for (Graph::Depth current_depth = 0; current_depth < depth - 1;
         current_depth++) {
      auto vertices_ids = graph.vertices_on_depth(current_depth);
      std::bernoulli_distribution distribution(double(1) -
                                               (double)current_depth / depth);
      for (auto& vertex_id : vertices_ids) {
        auto vertices_ids_next = graph.vertices_on_depth(current_depth + 1);
        for (auto& vertex_id_next : vertices_ids_next) {
          if (!graph.is_connected(vertex_id, vertex_id_next) &&
              distribution(generator)) {
            graph.add_edge(vertex_id, vertex_id_next);
          }
        }
      }
    }
    return 0;
  }

  int generate_red_edges(Graph& graph) const {
    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::bernoulli_distribution distribution(red_edge_probability);
    Graph::Depth depth = params_.depth();
    for (Graph::Depth current_depth = 0; current_depth < depth - 2;
         current_depth++) {
      auto vertices_ids = graph.vertices_on_depth(current_depth);
      for (auto& vertex_id : vertices_ids) {
        auto vertices_ids_next = graph.vertices_on_depth(current_depth + 2);
        for (auto& vertex_id_next : vertices_ids_next) {
          if (distribution(generator)) {
            graph.add_edge(vertex_id, vertex_id_next);
          }
        }
      }
    }
    return 0;
  }

  Graph generate() const {
    auto graph = Graph();
    graph.add_vertex();
    generate_grey_edges(graph);
    generate_green_edges(graph);
    generate_yellow_edges(graph);
    generate_red_edges(graph);
    return graph;
  }

 private:
  static constexpr double red_edge_probability = 1.0/3.0;
  static constexpr double green_edge_probability = 0.1;
  const Params params_ = Params();
};

namespace printing {
namespace json {
std::string print_vertex(const Graph::Vertex& vertex, const Graph& graph) {
  std::string string;
  string += "{ \"id\": ";
  string += std::to_string(vertex.id());
  string += ", \"edge_ids\": [";
  auto edges_ids = graph.get_edges_ids(vertex.id());
  for (auto edge_id_it = edges_ids.begin(); edge_id_it != edges_ids.end();
       edge_id_it++) {
    if (edge_id_it != edges_ids.begin())
      string += ", ";
    string += std::to_string(*edge_id_it);
  }
  string += "],";
  string += "\"depth\" : " + std::to_string(vertex.depth());
  string += " }";
  return string;
}

std::string print_edge_color(const Graph::Edge::Color&);

std::string print_edge(const Graph::Edge& edge) {
  std::string string;
  string += "{ \"id\": ";
  string += std::to_string(edge.id());
  string += ", \"vertex_ids\": [";
  string += std::to_string(edge.from_vertex_id());
  string += ", ";
  string += std::to_string(edge.to_vertex_id());
  string += "],";
  string += "\"color\" : " + print_edge_color(edge.color());
  string += " }";
  return string;
}

std::string print_graph(const Graph& graph) {
  auto vertices = graph.vertices();
  assert(vertices.size() != 0);
  auto edges = graph.edges();
  assert(edges.size() != 0);
  std::string string;
  string += "{\n  \"vertices\": [\n";
  for (const auto& vertex : vertices) {
    string += "    " + print_vertex(vertex, graph) + ",\n";
  }
  if (!vertices.empty()) {
    string.pop_back();
    string.pop_back();
  }
  string += "\n  ],\n  \"edges\": [\n";
  for (const auto& edge : edges) {
    string += "    " + print_edge(edge) + ",\n";
  }
  if (!edges.empty()) {
    string.pop_back();
    string.pop_back();
  }
  string += "\n  ]\n}\n";
  return string;
}

std::string print_edge_color(const Graph::Edge::Color& color) {
  switch (color) {
    case Graph::Edge::Color::Grey:
      return "\"grey\"";
      break;
    case Graph::Edge::Color::Green:
      return "\"green\"";
      break;
    case Graph::Edge::Color::Yellow:
      return "\"yellow\"";
      break;
    case Graph::Edge::Color::Red:
      return "\"red\"";
      break;
    default:
      throw std::runtime_error("No such color");
      break;
  }
}
}  // namespace json
}  // namespace printing

void write_file(const std::string& string, const std::string& name) {
  std::ofstream file;
  file.open(name, std::ios::out);
  file << string;
}


int handle_depth_input() {
  int depth = 0;
  std::cout << "Enter graph depth please: ";
  std::cin >> depth;
  std::cout << std::endl;
  return depth;
}

int handle_new_vertices_count_input() {
  int n_vertices = 0;
  std::cout << "Enter vertices number please: ";
  std::cin >> n_vertices;
  std::cout << std::endl;
  return n_vertices;
}

int main(void) {
  const int depth = handle_depth_input();
  const int new_vertices_count = handle_new_vertices_count_input();

  const auto params = GraphGenerator::Params(depth, new_vertices_count);
  const auto generator = GraphGenerator(params);
  std::cout << "Start generation" << std::endl;
  const auto graph = generator.generate();
  std::cout << "Generation finished" << std::endl;

  const auto graph_json = printing::json::print_graph(graph);
  std::cout << graph_json << std::endl;
  write_file(graph_json, "graph.json");
  return 0;
}
