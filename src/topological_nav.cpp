//
// Created by Yuting.Xie on 2021/8/17.
// email: xyt@bupt.cn
//

#include "../include/topological_navigation/TopologicalMap.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cstdio>
#include <list>
#include <mutex>
#include <thread>

class TopologicalNavigator {
private:
  static TopologicalNavigator *instance_;
  geometry_msgs::Point current_pos_;
  TopologicalMap *m;
  ros::Subscriber pos_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher goal_pub_;
  tf::TransformListener tf_listener_;
  std::list<unsigned int> path; // last point in path list is destination
  float distance_to_next_goal_;

  std::string hdmap_goal_topic_;
  std::string topological_goal_topic_;

  std::recursive_mutex lock_path_;

  TopologicalNavigator() {
    distance_to_next_goal_ = 1.f / 0.f;

    // resolve launch file params
    ros::NodeHandle n;
    float threshold_cm, resolution;
    std::string map_file_path;
    n.param<float>("threshold_cm", threshold_cm, 10.f);
    n.param<float>("resolution", resolution, 100.f);
    n.param<std::string>("map_file_path", map_file_path,
                         "./topological_map.txt");
    ROS_INFO("map_file_path set to: %s", map_file_path.c_str());
    n.param<std::string>("hdmap_goal_topic", hdmap_goal_topic_,
                         "/move_base_simple/goal");
    n.param<std::string>("topological_goal_topic", topological_goal_topic_,
                         "/topological_nav/goal");

    // create topological map, resolution is actually useless
    m = new TopologicalMap(resolution, threshold_cm);

    // load topological map
    if (!m->load_from_file(map_file_path)) {
      ROS_ERROR("unable to load topological map: %s, shutting down",
                map_file_path.c_str());
      ros::shutdown();
      exit(-1);
    } else {
      ROS_INFO("topological map: %s loaded, %d points in total",
               map_file_path.c_str(), m->num_vertices());
    }
    // init current pos to avoid weird situations
    m->get_coord_by_id(0, &current_pos_);

    // subscribe topological
    goal_sub_ =
        n.subscribe(topological_goal_topic_, 1,
                    &TopologicalNavigator::topological_goal_callback, this);
    // publish navigation goal to move_base
    goal_pub_ = n.advertise<geometry_msgs::PoseStamped>(hdmap_goal_topic_, 1);

    // listener thread for realtime pos
    std::thread([&]() {
      ROS_INFO("tf listener thread starts");
      tf::StampedTransform transform;
      ros::Rate rate(5);
      while (ros::ok()) {
        try {
          // wait time > 3 secs -> throws exception
          tf_listener_.waitForTransform("/map", "/base_laser_link",
                                        ros::Time(0), ros::Duration(3.f));
          tf_listener_.lookupTransform("/map", "/base_laser_link", ros::Time(0),
                                       transform);
          current_pos_.x = transform.getOrigin().x();
          current_pos_.y = transform.getOrigin().y();
          // current_pos_.z = transform.getOrigin().z();
          current_pos_.z = 0.f;
          // ROS_INFO("current pos updated: %.2f, %.2f", current_pos_.x,
          //         current_pos_.y);
          current_pos_callback();
        } catch (const tf::TransformException &exception) {
          ROS_WARN("time-out for tf msg: %s", exception.what());
        }
        rate.sleep(); // sleep until next interval
      }
      ROS_INFO("tf listener thread quited");
    }).detach();

    // publisher thread for current middle goal point
    std::thread([&]() {
      ROS_INFO("nav goal publisher thread starts");
      ros::Rate rate(2);
      geometry_msgs::Point goal;
      int last_goal_id = -1;
      geometry_msgs::PoseStamped pose_msg;
      while (ros::ok()) {
        std::unique_lock<std::recursive_mutex> ulock_path(lock_path_,
                                                          std::try_to_lock);
        if (!ulock_path.owns_lock()) {
          ROS_INFO("unable to lock path");
          goto SLEEP;
        }
        if (path.empty() || last_goal_id == path.front()) {
          goto SLEEP;
        }
        m->get_coord_by_id(path.front(), &goal);
        // this line makes ros crash!!!
        // pose_msg.header.stamp.setNow(ros::Time::now());
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position = goal;
        // Quaternion in pose must be set!!!
        pose_msg.pose.orientation.w = 1.f;
        pose_msg.pose.orientation.x = 0.f;
        pose_msg.pose.orientation.y = 0.f;
        pose_msg.pose.orientation.z = 0.f;
        goal_pub_.publish(pose_msg);
        distance_to_next_goal_ = getDist(current_pos_, goal);
        last_goal_id = (int)path.front();
        ROS_INFO("a new middle goal %d (%.2f, %.2f) published", path.front(),
                 goal.x, goal.y);

      SLEEP:
        if (ulock_path.owns_lock()) {
          ulock_path.unlock();
        }
        rate.sleep();
      }
    }).detach();
  }
  TopologicalNavigator(const TopologicalNavigator &other) = delete;
  TopologicalNavigator(const TopologicalNavigator &&other) = delete;

  void current_pos_callback() {
    // pre-store cur pos to avoid inconsistency
    geometry_msgs::Point cur_pos = current_pos_;
    std::unique_lock<std::recursive_mutex> ulock_path(lock_path_,
                                                      std::try_to_lock);
    if (!ulock_path.owns_lock() || path.empty()) {
      return;
    }
    geometry_msgs::Point cur_goal;
    ROS_ASSERT(m->get_coord_by_id(path.front(), &cur_goal));
    // validate that we are getting closer if we are in navigation
    float dist = getDist(cur_pos, cur_goal);
    if (dist < distance_to_next_goal_) {
      ROS_WARN("seems we are getting farther from current goal");
    }
    distance_to_next_goal_ = dist;

    // judge if entering cur goal point
    if (is_close_to(cur_pos, cur_goal, m->threshold())) {
      // arriving at cur goal, publish next goal
      path.erase(path.begin());
      if (!path.empty()) {
        ROS_INFO("arrived at middle point (%.2f, %.2f), %ld left on path",
                 cur_goal.x, cur_goal.y, path.size());
      } else {
        ROS_INFO("arrived at final goal, navigation finished");
      }
    }
  }

  void topological_goal_callback(const geometry_msgs::Point::ConstPtr msg) {
    ROS_INFO("new topological goal msg(%.2f, %.2f) received", msg->x, msg->y);
    if (is_close_to(current_pos_, *msg, m->threshold())) {
      ROS_INFO(
          "new topological goal to close to cur pos, no need to update path");
      return;
    }
    update_path(*msg);
  }

  bool update_path(const geometry_msgs::Point &goal_point) {
    ROS_INFO("we have a new final goal(%.2f, %.2f), re-calculating new path",
             goal_point.x, goal_point.y);
    // see where we are right now
    int src_id = m->get_id_by_coord(current_pos_);
    if (-1 == src_id) {
      ROS_WARN("current pos out of topological map, failed to update goal");
      return false;
    }

    // find target point id
    int end_id = m->get_id_by_coord(goal_point);
    if (-1 == end_id) {
      ROS_WARN("destination pos out of topological map, failed to update goal");
      return false;
    }

    // get the path from src to end
    std::unique_lock<std::recursive_mutex> ulock_path(lock_path_,
                                                      std::try_to_lock);
    while (!ulock_path.owns_lock()) {
      ulock_path.try_lock();
      ROS_INFO("waiting for path lock to update it");
    }
    path = m->get_path(src_id, end_id);
    ROS_INFO("update path to new goal: %ld mid points", path.size());
    return true;
  }

public:
  static TopologicalNavigator &Instance() {
    if (!instance_) {
      instance_ = new TopologicalNavigator();
    }
    return *instance_;
  }
};

TopologicalNavigator *TopologicalNavigator::instance_ = nullptr;

int main(int argc, char **argv) {
  // init ros node
  ros::init(argc, argv, "topological_nav");
  ROS_INFO("topological_nav node started");

  // create navigator
  auto &navigator = TopologicalNavigator::Instance();

  // user interact thread
  std::thread user_thread([&]() {
    ros::NodeHandle n;
    ros::Publisher pub =
        n.advertise<geometry_msgs::Point>("/topological_nav/goal", 1);
    geometry_msgs::Point goal_msg;
    while (ros::ok()) {
      printf("input a goal (x, y):\n");
      scanf("%lf, %lf", &(goal_msg.x), &(goal_msg.y));
      pub.publish(goal_msg);
      ROS_INFO("new final goal published: %.2f, %.2f", goal_msg.x, goal_msg.y);
    }
  });
  user_thread.detach();

  ros::spin();

  return 0;
}
