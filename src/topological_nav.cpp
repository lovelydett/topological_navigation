//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "../include/topological_navigation/TopologicalMap.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <list>

class TopologicalNavigator {
private:
  static TopologicalNavigator *instance_;
  geometry_msgs::Point current_pos_;
  TopologicalMap m;
  ros::Subscriber pos_sub_;
  std::list<unsigned int> path; // last point in path list is destination
  float distance_to_next_goal_;

  TopologicalNavigator(
      std::string filename = "/home/tt/Desktop/topological_map.txt")
      : distance_to_next_goal_(1.f / 0.f) {
    // load topological map
    if (-1 == m.load_from_file(filename)) {
      ROS_ERROR("unable to load topological map: %s", filename.c_str());
    }

    // subscribe real-time Pose msg from amcl
    const std::string pos_topic_name = "/amcl_pose";
    ros::NodeHandle n;
    pos_sub_ = n.subscribe(pos_topic_name, 1,
                           &TopologicalNavigator::current_pos_callback, this);
  }
  TopologicalNavigator(const TopologicalNavigator &other) = delete;
  TopologicalNavigator(const TopologicalNavigator &&other) = delete;

  void current_pos_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // Todo: lock cur pos
    current_pos_ = msg->pose.pose.position;

    // validate that we are getting closer if we are in navigation
    // Todo: lock path
    geometry_msgs::Point next_goal_coord;
    if (!path.empty()) {
      ROS_ASSERT(m.get_coord_by_id(path.front(), &next_goal_coord));
      float dist = getDist(current_pos_, next_goal_coord);
      if (dist < distance_to_next_goal_) {
        ROS_WARN("getting farther from next goal");
      }
    }

    // judge if we enter next goal point
    if (is_close_to(current_pos_, next_goal_coord)) {
      // arriving, set next goal
      path.erase(path.begin());
      if (!path.empty()) {
        // Todo: publish next goal msg
      }
    }
  }

public:
  static TopologicalNavigator &Instance() {
    if (!instance_) {
      instance_ = new TopologicalNavigator();
    }
    return *instance_;
  }

  bool update_goal(const geometry_msgs::Point &goal_point) {
    // see where we are right now
    int src_id = m.get_id_by_coord(current_pos_);
    if (-1 == src_id) {
      ROS_WARN("current pos out of topological map, failed to update goal");
      return false;
    }

    // find target point id
    int end_id = m.get_id_by_coord(goal_point);
    if (-1 == end_id) {
      ROS_WARN("destination pos out of topological map, failed to update goal");
      return false;
    }

    // get the path from src to end
    path = m.get_path(src_id, end_id);
  }
};

TopologicalNavigator *TopologicalNavigator::instance_ = nullptr;

int main(int argc, char **argv) {
  // init ros node
  ros::init(argc, argv, "topological_nav");
  ros::NodeHandle n;
  ROS_INFO("topological_nav node started");

  // create navigator
  auto &navigator = TopologicalNavigator::Instance();

  return 0;
}
