//
// Created by Yuting.Xie on 2021/8/18.
// email: xyt@bupt.cn
//
#include <cstdio>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "topological_build_map_test");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Point>("pos", 10);
  ros::Rate loop_rate(1);
  int count = 0;
  geometry_msgs::Point msg;
  while (ros::ok()) {
    if (++count % 2 == 0) {
      msg.x += 30.f;
    } else {
      msg.y += 30.f;
    }
    pub.publish(msg);
    ROS_INFO("%d point msgs published, current pos: %.2f, %.2f", count, msg.x,
             msg.y);
    loop_rate.sleep();
  }

  return 0;
}