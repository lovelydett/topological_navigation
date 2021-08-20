//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "../include/topological_navigation/TopologicalMap.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <list>
#include <thread>

class TopologicalNavigator {
private:
  static TopologicalNavigator *instance_;
  geometry_msgs::Point current_pos_;
  TopologicalMap m;
  ros::Subscriber pos_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher goal_pub_;
  std::list<unsigned int> path; // last point in path list is destination
  float distance_to_next_goal_;
  const std::string goal_topic_name_;
  const std::string topological_goal_topic_name_;
  const std::string localization_topic_name;

  TopologicalNavigator(
      std::string filename = "/home/tt/Desktop/topological_map.txt")
      : distance_to_next_goal_(1.f / 0.f), goal_topic_name_("/move_base/goal"),
        topological_goal_topic_name_("/topological_nav/goal"),
        localization_topic_name("/amcl_pose") {
    // load topological map
    if (-1 == m.load_from_file(filename)) {
      ROS_ERROR("unable to load topological map: %s", filename.c_str());
    } else {
      ROS_INFO("topological map: %s loaded, %d points in total",
               filename.c_str(), m.num_vertices());
    }
    m.get_coord_by_id(m.num_vertices() - 1, &current_pos_);

    // subscribe real-time Pose msg from amcl
    ros::NodeHandle n;
    pos_sub_ = n.subscribe(localization_topic_name, 1,
                           &TopologicalNavigator::current_pos_callback, this);
    // subscribe topological
    goal_sub_ =
        n.subscribe(topological_goal_topic_name_, 1,
                    &TopologicalNavigator::topological_goal_callback, this);
    // publish navigation goal to move_base
    goal_pub_ = n.advertise<geometry_msgs::PoseStamped>(goal_topic_name_, 1);
  }
  TopologicalNavigator(const TopologicalNavigator &other) = delete;
  TopologicalNavigator(const TopologicalNavigator &&other) = delete;

  void publish_current_goal() {
    geometry_msgs::Point goal;
    m.get_coord_by_id(path.front(), &goal);
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.setNow(ros::Time::now());
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position = goal;
    goal_pub_.publish(goal);
    distance_to_next_goal_ = getDist(current_pos_, goal);
  }

  void current_pos_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // Todo: lock cur pos
    current_pos_ = msg->pose.pose.position;

    // validate that we are getting closer if we are in navigation
    // Todo: lock path
    if (path.empty()) {
      ROS_INFO("idle, goal not set");
      return;
    }
    ROS_INFO("%d points left to reach final goal", path.size());
    geometry_msgs::Point cur_goal;
    ROS_ASSERT(m.get_coord_by_id(path.front(), &cur_goal));
    float dist = getDist(current_pos_, cur_goal);
    if (dist < distance_to_next_goal_) {
      ROS_WARN("seems we are getting farther from current goal");
    }
    distance_to_next_goal_ = dist;

    // judge if entering cur goal point
    if (is_close_to(current_pos_, cur_goal)) {
      // arriving at cur goal, publish next goal
      path.erase(path.begin());
      if (!path.empty()) {
        publish_current_goal();
      } else {
        ROS_INFO("arrived at final goal, navigation finished");
      }
    }
  }

  void topological_goal_callback(const geometry_msgs::Point::ConstPtr msg) {
    ROS_INFO("new goal received, updating path");
    if (is_close_to(current_pos_, *msg)) {
      ROS_INFO("new goal to close to cur pos, no need to update path");
      return;
    }
    update_goal(*msg);
    // immediately publish new goal
    publish_current_goal();
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
    ROS_INFO("got the path to new goal: %d mid points", path.size());
    return true;
  }

public:
  static TopologicalNavigator &Instance() {
    if (!instance_) {
      instance_ = new TopologicalNavigator();
    }
    return *instance_;
  }

  void mock_goal() {
    if (m.num_vertices() == 0) {
      return;
    }
    geometry_msgs::Point goal;
    m.get_coord_by_id(m.num_vertices() - 1, &goal);
    update_goal(goal);
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

  // mock
  // navigator.mock_goal();

  ros::spin();

  return 0;
}
