//
// Created by naivehobo on 5/7/19.
//

#ifndef FAKE_PLANNER_FAKEPLANNER_H
#define FAKE_PLANNER_FAKEPLANNER_H

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <fake_planner/SetMaxVel.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

class FakePlanner {

 public:
  FakePlanner();

  void run();

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);
  bool setMaxVelocity(fake_planner::SetMaxVel::Request  &req,
                      fake_planner::SetMaxVel::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher goal_reached_pub_;

  ros::Subscriber goal_sub_;

  ros::ServiceServer max_vel_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double max_angular_speed_;
  double max_linear_speed_;
  double linear_tolerance_;
  double angular_tolerance_;
  double time_to_x_;
  double time_to_angle_;
  bool is_goal_set_;

  std::string goal_topic_;
  std::string cmd_topic_;
  std::string goal_reached_topic_;
  std::string max_vel_service_name_;

  geometry_msgs::PoseStamped goal_;

  double euclideanDistance(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
  }

};

#endif //FAKE_PLANNER_FAKEPLANNER_H
