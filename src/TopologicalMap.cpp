//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "topological_navigation/TopologicalMap.h"

#include <fstream>
#include <queue>

unsigned int TopologicalMap::num_vertices() const { return graph.size(); }
float TopologicalMap::threshold() const { return threshold_; }

bool TopologicalMap::get_coord_by_id(const unsigned int point_id,
                                     geometry_msgs::Point *coord_ptr) const {
  auto it = id_to_coords.find(point_id);
  if (it == id_to_coords.end()) {
    return false;
  }
  coord_ptr->x = it->second.x;
  coord_ptr->y = it->second.y;
  coord_ptr->z = it->second.z;
  return true;
}
int TopologicalMap::get_id_by_coord(
    const geometry_msgs::Point &coord_in) const {
  int point_id = -1;
  float min_dist = 1.f / 0.f;

  // Todo: make this faster by B+ tree or sth
  for (const auto &[id, coord] : id_to_coords) {
    auto dist = getDist(coord_in, coord);
    if (dist < min_dist) {
      min_dist = dist;
      point_id = id;
    }
  }

  return min_dist < threshold_ ? point_id : -1;
}

// add a new vertice into graph by coord
int TopologicalMap::add_vertice(const geometry_msgs::Point &coord) {
  int id = get_id_by_coord(coord);
  if (-1 != id) {
    ROS_INFO("adjacent coord overlaps, no need to add vertice");
    return id;
  }
  id = id_to_coords.size();
  id_to_coords[id] = coord;
  graph.emplace_back(std::vector<unsigned int>{});
  ROS_ASSERT(id_to_coords.size() == graph.size());
  return id;
}

// add a new undirected edge into graph by point id
bool TopologicalMap::add_edge_undirected(const unsigned int point1_id,
                                         const unsigned int point2_id) {
  return add_edge_directed(point1_id, point2_id) &&
         add_edge_directed(point2_id, point1_id);
}

bool TopologicalMap::add_edge_directed(const unsigned int src_id,
                                       const unsigned int end_id) {
  if (src_id >= graph.size() || end_id >= graph.size()) {
    ROS_WARN("invalid point id");
    return false;
  }
  for (auto nb : graph[src_id]) {
    if (nb == end_id) {
      ROS_WARN("edge already exists");
      return false;
    }
  }
  graph[src_id].emplace_back(end_id);
  return true;
}

// calculate the shortest path from src point to target point, returns a path
std::list<unsigned int>
TopologicalMap::get_path(const unsigned int src_id,
                         const unsigned int end_id) const {
  // Dijkstra with priority queue
  std::priority_queue<std::pair<float, unsigned int>> q; // dist at pair.first
  std::vector<float> min_dist(graph.size(), 1.f / 0.f);
  std::vector<bool> finish(graph.size(), false);
  std::vector<std::list<unsigned int>> paths(graph.size(),
                                             std::list<unsigned int>{src_id});
  q.push({0.f, src_id});
  for (int i = 0; i < graph.size(); i++) {
    auto [dist, id] = q.top();
    q.pop();
    finish[id] = true;
    geometry_msgs::Point coord1, coord2;
    get_coord_by_id(id, &coord1);
    for (auto nb : graph[id]) {
      if (finish[nb]) {
        continue;
      }
      get_coord_by_id(nb, &coord2);
      auto d = getDist(coord1, coord2);
      if (dist + d < min_dist[nb]) {
        min_dist[nb] = dist + d;
        q.push({dist + d, nb});
        paths[nb] = paths[id]; // update path src->nb
        paths[nb].emplace_back(nb);
      }
    }
  }

  return paths[end_id];
}

bool TopologicalMap::save_to_file(const std::string filename) const {
  ROS_ASSERT(id_to_coords.size() == graph.size());
  ROS_INFO("writing graph to %s", filename.c_str());
  if (graph.empty()) {
    ROS_WARN("cannot output an empty graph.");
    return false;
  }
  std::ofstream fout;
  fout.open(filename, std::ios::out);
  if (!fout.is_open()) {
    ROS_ERROR("unable to open file: %s", filename.c_str());
    return false;
  }
  fout << graph.size() << '\n'; // number of vertices
  fout << resolution_ << '\n';  // resolution
  // coords for vertices: id x y z
  for (auto &[id, coord] : id_to_coords) {
    fout << id << ' ' << coord.x << ' ' << coord.y << ' ' << coord.z << '\n';
  }
  // adjList for vertices: id num_of_nbs nb1 nb2 ... nbn
  for (int i = 0; i < graph.size(); i++) {
    fout << i << ' ' << graph[i].size();
    for (auto nb : graph[i]) {
      fout << ' ' << nb;
    }
    fout << '\n';
  }

  fout.close();

  return true;
}

bool TopologicalMap::load_from_file(const std::string filename) {
  std::ifstream fin;
  fin.open(filename, std::ios::in);
  if (!fin.is_open()) {
    ROS_ERROR("unable to open file: %s", filename.c_str());
    return false;
  }

  int num_vertices, id;
  fin >> num_vertices; // read number of vertices.
  fin >> resolution_;  // read resolution.
  // read coords
  id_to_coords.clear();
  for (int i = 0; i < num_vertices; i++) {
    geometry_msgs::Point pt;
    fin >> id >> pt.x >> pt.y >> pt.z;
    id_to_coords.emplace(std::make_pair(id, pt));
  }
  // read adjList
  graph.resize(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    int num_nbs;
    fin >> id >> num_nbs;
    graph[id].resize(num_nbs);
    for (int j = 0; j < num_nbs; j++) {
      fin >> graph[id][j];
    }
  }

  fin.close();

  return true;
}