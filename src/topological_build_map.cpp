//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "topological_navigation/TopologicalMap.h"

#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <thread>

class TopologicalMapBuilder {
private:
  static TopologicalMapBuilder *instance_;
  TopologicalMap m;
  geometry_msgs::Point current_pos;
  int last_pos_id_;
  tf::TransformListener tf_listener_;
  TopologicalMapBuilder() : last_pos_id_(-1) {
    std::thread listen([&]() {
      ROS_INFO("tf_listener thread starts");
      tf::StampedTransform transform;
      while (ros::ok()) {
        try {
          // wait time > 3 secs -> throws exception
          tf_listener_.waitForTransform("/map", "/base_laser_link",
                                        ros::Time(0), ros::Duration(3.f));
          tf_listener_.lookupTransform("/map", "/base_laser_link", ros::Time(0),
                                       transform);
          current_pos.x = transform.getOrigin().x();
          current_pos.y = transform.getOrigin().y();
          // current_pos.z = transform.getOrigin().z();
          current_pos.z = 0.f;
          ROS_INFO("current pos updated: %.2f, %.2f", current_pos.x,
                   current_pos.y);
          ros::Duration(0.1f).sleep();
        } catch (tf::TransformException exception) {
          ROS_WARN("time-out for tf msg: %s", exception.what());
          ros::Duration(2.f).sleep();
        }
      }
    });
    listen.detach();
  }
  TopologicalMapBuilder(const TopologicalMapBuilder &other_instance) = delete;

public:
  static TopologicalMapBuilder &Instance() {
    if (!instance_) {
      instance_ = new TopologicalMapBuilder;
    }
    return *instance_;
  }

  // record current pos as a new point and add a new edge (cur_pos, last_pos)
  bool add_current_pos() {
    ROS_INFO("adding current pos");
    // first judge whether current pos is covered by previous pos(s)
    int id = m.get_id_by_coord(current_pos);
    if (-1 != id) {
      geometry_msgs::Point coord;
      m.get_coord_by_id(id, &coord);
      ROS_INFO("current pos(%.2f, %.2f) too close to known pos(%.2f, %.2f), no "
               "need to add",
               current_pos.x, current_pos.y, coord.x, coord.y);
      return false;
    }
    id = m.add_vertice(current_pos); // add cur as a new point
    if (id == -1) {
      ROS_ERROR("failed to add current pos");
      return false;
    }
    if (last_pos_id_ == -1) {
      last_pos_id_ = id;
      return true;
    }
    // add an edge between cur and last
    if (!m.add_edge_undirected(id, last_pos_id_)) {
      ROS_ERROR("failed to add new edge");
      return false;
    }
    last_pos_id_ = id; // update last id
    return true;
  }

  bool load_topological_map(
      std::string filename = "/home/tt/Desktop/topological_map.txt") {
    if (!m.load_from_file(filename)) {
      return false;
    }
    last_pos_id_ = m.num_vertices() - 1; // set last!
  }
  bool save_topological_map(
      std::string filename = "/home/tt/Desktop/topological_map.txt") {
    return m.save_to_file(filename);
  }

  ~TopologicalMapBuilder() {
    // before shutting down, save the map
    save_topological_map();
  }
};

TopologicalMapBuilder *TopologicalMapBuilder::instance_ = nullptr;

int main(int argc, char **argv) {
  // init ros node
  ros::init(argc, argv, "topological_build_map");
  ros::NodeHandle n;
  ROS_INFO("topological_build_map node started");

  // create the builder, must after ros::init
  TopologicalMapBuilder &builder = TopologicalMapBuilder::Instance();

  // keyboard control thread
  std::thread keyboard_thread([&]() {
    ROS_INFO("Press 'a' to add current pos, 's' to save current map, 'l' to "
             "load map, 'e' to exit");
    while (true) {
      auto ch = getchar();
      if (ch == 'a') {
        builder.add_current_pos();
      } else if (ch == 's') {
        builder.save_topological_map();
      } else if (ch == 'l') {
        std::cout << "input topological filename:";
        std::string filename;
        std::cin >> filename;
        builder.load_topological_map(filename);
      } else if (ch == 'e') {
        break;
      }
    }
  });
  keyboard_thread.detach();

  ros::spin();
}