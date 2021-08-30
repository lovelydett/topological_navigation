//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "topological_navigation/TopologicalMap.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <mutex>
#include <thread>

class TopologicalMapBuilder {
private:
  static TopologicalMapBuilder *instance_;
  TopologicalMap *m;
  geometry_msgs::Point current_pos_;
  std::mutex pos_lock_;
  int last_pos_id_;
  tf::TransformListener tf_listener_;
  std::string map_file_path_;

  TopologicalMapBuilder() : last_pos_id_(-1) {
    // resolve params from launch file
    ros::NodeHandle n;
    float threshold_cm, resolution;
    n.param<float>("threshold_cm", threshold_cm, 10.f);
    n.param<float>("resolution", resolution, 100.f);
    n.param<std::string>("map_file_path", map_file_path_,
                         "./topological_map.txt");

    // create a topological map
    m = new TopologicalMap(resolution, threshold_cm);

    // start listener thread
    std::thread([&]() {
      ROS_INFO("tf_listener thread starts");
      tf::StampedTransform transform;
      while (ros::ok()) {
        try {
          // wait time > 3 secs -> throws exception
          tf_listener_.waitForTransform("/map", "/base_laser_link",
                                        ros::Time(0), ros::Duration(3.f));
          tf_listener_.lookupTransform("/map", "/base_laser_link", ros::Time(0),
                                       transform);
          {
            std::unique_lock<std::mutex> _(pos_lock_);
            current_pos_.x = transform.getOrigin().x();
            current_pos_.y = transform.getOrigin().y();
            // current_pos_.z = transform->getOrigin().z();
            current_pos_.z = 0.f;
            // ROS_INFO("current pos updated: %.2f, %.2f", current_pos_.x,
            // current_pos_.y);
          }
          ros::Duration(0.1f).sleep();
        } catch (tf::TransformException exception) {
          ROS_WARN("time-out for tf msg: %s", exception.what());
          ros::Duration(2.f).sleep();
        }
      }
    }).detach();
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
    std::unique_lock<std::mutex> _(pos_lock_);
    ROS_INFO("adding current pos");
    // first judge whether current pos is covered by previous pos(s), if so,
    // update last pos!
    int id = m->get_id_by_coord(current_pos_);
    if (-1 != id) {
      geometry_msgs::Point coord;
      m->get_coord_by_id(id, &coord);
      ROS_INFO("current pos(%.2f, %.2f) too close to known pos(%.2f, %.2f), no "
               "need to add, just update last pos to %d",
               current_pos_.x, current_pos_.y, coord.x, coord.y, id);
      // should also add a path <last, cur> to avoid breaking the graph
      m->add_edge_undirected(last_pos_id_, id);
      last_pos_id_ = id;
      return false;
    }
    id = m->add_vertice(current_pos_); // add cur as a new point
    if (id == -1) {
      ROS_ERROR("failed to add current pos");
      return false;
    }
    if (last_pos_id_ == -1) {
      last_pos_id_ = id;
      return true;
    }
    // add an edge between cur and last
    if (!m->add_edge_undirected(id, last_pos_id_)) {
      ROS_ERROR("failed to add new edge");
      return false;
    }
    last_pos_id_ = id; // update last id
    return true;
  }

  bool load_topological_map() {
    if (!m->load_from_file(map_file_path_)) {
      ROS_ERROR("unable to load topological map, shutting down node");
      ros::shutdown();
      return false;
    }
    last_pos_id_ = m->num_vertices() - 1; // set last!
    return true;
  }
  bool save_topological_map() {
    if (!m->save_to_file(map_file_path_)) {
      ROS_ERROR("unable to save topological map, check the parameter "
                "<map_file_path>, shutting down node");
      ros::shutdown();
      return false;
    }
    return true;
  }

  ~TopologicalMapBuilder() {
    // before shutting down, save the map
    save_topological_map();
    if (m) {
      delete m;
    }
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
        builder.load_topological_map();
      } else if (ch == 'e') {
        break;
      }
    }
  });
  keyboard_thread.detach();
  ros::spin();
}