//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "topological_navigation/TopologicalMap.h"

#include <cmath>
#include <queue>

float getDistSquare(const geometry_msgs::Point &pt1,
                    const geometry_msgs::Point &pt2) {
  auto dx = pt1.x - pt2.x;
  auto dy = pt1.y - pt2.y;
  auto dz = pt1.z - pt2.z;
  return dx * dx + dy * dy + dz * dz;
}

float getDist(const geometry_msgs::Point &pt1,
              const geometry_msgs::Point &pt2) {
  return std::sqrt(getDistSquare(pt1, pt2));
}

bool TopologicalMap::get_coord_by_id(const unsigned int point_id,
                                     geometry_msgs::Point *coord_ptr) const {
  auto it = id_to_coords.find(point_id);
  if (it == id_to_coords.end()) {
    return false;
  }
  coord_ptr->x = it->second.x;
  coord_ptr->y = it->second.y;
  // coord_ptr->z = it->second.z;
  return true;
}
int TopologicalMap::get_id_by_coord(const geometry_msgs::Point coord_in) const {
  const float dist_threshold = 20 / resolution_;
  int point_id = -1;
  float min_dist = 1.0 / 0.0;

  // Todo: make this faster by B+ tree or sth
  for (const auto &[id, coord] : id_to_coords) {
    auto dist = getDist(coord_in, coord);
    if (dist < min_dist) {
      min_dist = dist;
      point_id = id;
    }
  }

  return min_dist < dist_threshold ? point_id : -1;
}

// add a new vertice into graph by coord
int TopologicalMap::add_vertice(const geometry_msgs::Point coord) {
  int id = get_id_by_coord(coord);
  if (-1 != id) {
    ROS_INFO("adjacent coord overlaps, no need to add vertice");
    return id;
  }
  id = id_to_coords.size();
  id_to_coords[id] = coord;
  graph.emplace_back(std::vector<unsigned int>{});
  ROS_ASSERT(id_to_coords.size() == graph.size());
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
  // Todo: check whether edge already exists
  graph[src_id].emplace_back(end_id);
  return true;
}

// calculate the shortest path from src point to target point, returns a path
std::vector<unsigned int>
TopologicalMap::get_path(const unsigned int src_id,
                         const unsigned int end_id) const {
  // Dijkstra with priority queue
  std::priority_queue<std::pair<unsigned int, float>> q;
  std::vector<float> min_dist(graph.size(), 1.f / 0.f);
  std::vector<bool> finish(graph.size(), false);
  std::vector<std::vector<unsigned int>> paths(
      graph.size(), std::vector<unsigned int>{src_id});
  q.push({src_id, 0.f});
  for (int i = 0; i < graph.size(); i++) {
    auto [id, dist] = q.top();
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
        q.push({nb, dist + d});
        paths[nb] = paths[id]; // update path src->nb
        paths[nb].emplace_back(nb);
      }
    }
  }

  return paths[end_id];
}