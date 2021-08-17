//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//
#include "topological_navigation/TopologicalMap.h"

#include <cstdio>
#include <vector>

geometry_msgs::Point create_point(float x, float y, float z = 0.f) {
  geometry_msgs::Point pt;
  pt.x = x;
  pt.y = y;
  pt, z = z;
  return pt;
}

int main() {
  auto m = TopologicalMap(1.f); // 1 : 1cm
  std::vector<>
}