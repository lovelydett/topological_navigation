//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//
#include "icecream.hpp"
#include "topological_navigation/TopologicalMap.h"

#include <cstdio>

geometry_msgs::Point create_point(float x, float y, float z = 0.f) {
  geometry_msgs::Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  return pt;
}

int main() {
  auto m = TopologicalMap(1.f); // 1 : 1cm

  // add points
  m.add_vertice(create_point(0., 0.));
  m.add_vertice(create_point(30., 0.));
  m.add_vertice(create_point(60., 0.));
  m.add_vertice(create_point(90., 0.));
  m.add_vertice(create_point(90., 30.));
  m.add_vertice(create_point(120., 30.));
  m.add_vertice(create_point(120., 60.));
  m.add_vertice(create_point(90., 60.));
  m.add_vertice(create_point(30., 60.));

  // add edges
  m.add_edge_undirected(0, 1);
  m.add_edge_undirected(8, 1);
  m.add_edge_undirected(2, 1);
  m.add_edge_undirected(3, 4);
  m.add_edge_undirected(4, 5);
  m.add_edge_undirected(7, 5);
  m.add_edge_undirected(7, 8);
  m.add_edge_undirected(7, 6);

  // get the shortest path
  auto path = m.get_path(3, 2);
  for (auto id : path) {
    printf("%d -> ", id);
  }
  printf("\n");

  // test range localization
  geometry_msgs::Point pt;
  pt.x = 115.;
  pt.y = 55.;
  IC(m.get_id_by_coord(pt));

  return 0;
}