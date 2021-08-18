//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "topological_navigation/TopologicalMap.h"

#include <iostream>
#include <termio.h>
#include <thread>

class TopologicalMapBuilder {
private:
  static TopologicalMapBuilder *instance_;
  TopologicalMap m;
  geometry_msgs::Point current_pos;
  int last_pos_id_;
  ros::Subscriber pos_sub;
  TopologicalMapBuilder() : last_pos_id_(-1) {
    // init subscriber
    // Todo: determine the topic name for this
    ros::NodeHandle n;
    const std::string pos_topic_name = "pos";
    pos_sub = n.subscribe<geometry_msgs::Point>(
        pos_topic_name, 1, &TopologicalMapBuilder::position_callback, this);
  }

  // subscribe location msg
  void position_callback(const geometry_msgs::Point::ConstPtr &msg) {
    // Todo: lock protect
    current_pos.x = msg->x;
    current_pos.y = msg->y;
    current_pos.z = msg->z;
  }

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
    if (-1 != m.get_id_by_coord(current_pos)) {
      ROS_INFO("current pos too close to known pos, no need to add");
      return false;
    }
    int id = m.add_vertice(current_pos); // add cur as a new point
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

  bool
  load_topological_map(std::string filename = "../map/topological_map.txt") {
    if (!m.load_from_file(filename)) {
      return false;
    }
    last_pos_id_ = m.num_vertices() - 1; // set last!
  }
  bool
  save_topological_map(std::string filename = "../map/topological_map.txt") {
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
  auto builder = TopologicalMapBuilder::Instance();

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