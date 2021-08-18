//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#ifndef TOPOLOGICAL_NAVIGATION_TOPOLOGICALMAP_H
#define TOPOLOGICAL_NAVIGATION_TOPOLOGICALMAP_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <vector>

// 1 = 5cm
#define RESOLUTION 5

class TopologicalMap {
private:
  float resolution_; // 1 : x(cm)
  std::unordered_map<unsigned int, geometry_msgs::Point> id_to_coords;
  std::vector<std::vector<unsigned int>> graph;

public:
  TopologicalMap(float resolution = RESOLUTION) : resolution_(resolution) {}
  unsigned int num_vertices() const;
  bool get_coord_by_id(const unsigned int point_id,
                       geometry_msgs::Point *coord_ptr) const;
  int get_id_by_coord(const geometry_msgs::Point) const;

  // add a new vertice into graph by coord, returns its id
  int add_vertice(const geometry_msgs::Point coord);

  // add a new undirected edge into graph
  bool add_edge_undirected(const unsigned int point1_id,
                           const unsigned int point2_id);
  // add a new directed edge into graph
  bool add_edge_directed(const unsigned int src_id, const unsigned int end_id);

  // calculate the shortest path from src point to target point, returns a path
  std::vector<unsigned int> get_path(const unsigned int src_id,
                                     const unsigned int end_id) const;

  // to .txt file and from .txt file
  bool save_to_file(const std::string filename) const;
  bool load_from_file(const std::string filename);
};

#endif // TOPOLOGICAL_NAVIGATION_TOPOLOGICALMAP_H
