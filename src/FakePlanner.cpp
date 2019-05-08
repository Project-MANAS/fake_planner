//
// Created by naivehobo on 7/7/19.
//

#include "fake_planner/FakePlanner.h"

#include <ros/ros.h>

#include <fake_planner/SetMaxVel.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

FakePlanner::FakePlanner() : private_nh_("~"), tf_listener_(tf_buffer_) {
  private_nh_.param<std::string>("goal_topic", goal_topic_, "goal");
  private_nh_.param<std::string>("cmd_vel_topic", cmd_topic_, "cmd_vel");
  private_nh_.param<std::string>("max_velocity_service", max_vel_service_name_, "set_max_velocity");
  private_nh_.param("max_angular_speed", max_angular_speed_, 0.25);
  private_nh_.param("max_linear_speed", max_linear_speed_, 0.5);
  private_nh_.param("linear_threshold", linear_tolerance_, 0.1);
  private_nh_.param("angular_threshold", angular_tolerance_, 0.01);
  private_nh_.param("time_to_x", time_to_x_, 1.0);
  private_nh_.param("time_to_angle", time_to_angle_, 1.0);

  is_goal_set_ = false;

  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

  max_vel_server_ = private_nh_.advertiseService(max_vel_service_name_, &FakePlanner::setMaxVelocity, this);

  goal_sub_ = nh_.subscribe(goal_topic_, 1, &FakePlanner::goalCallback, this);
}

bool FakePlanner::setMaxVelocity(fake_planner::SetMaxVel::Request &req, fake_planner::SetMaxVel::Response &res) {
  max_angular_speed_ = req.max_vel.data;
  max_linear_speed_ = req.max_vel.data;
}

void FakePlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
  is_goal_set_ = true;
  goal_ = *goal;
  ROS_INFO("Goal was set to (%lf, %lf)",
           goal_.pose.position.x,
           goal_.pose.position.y);
}

void FakePlanner::run() {

  geometry_msgs::PoseStamped goal_tf;
  geometry_msgs::TransformStamped transform;

  ros::Rate rate(20);

  while (ros::ok()) {

    geometry_msgs::Twist move_cmd;

    if (is_goal_set_) {

      transform = tf_buffer_.lookupTransform("base_link", goal_.header.frame_id, ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(goal_, goal_tf, transform);

      tf_buffer_.transform(goal_, goal_tf, "base_link");

      auto displacement = euclideanDistance(goal_tf.pose.position.x,
                                            goal_tf.pose.position.y);

      auto theta = atan2(goal_tf.pose.position.y,
                         goal_tf.pose.position.x);

      ROS_INFO("Displacement: %lf\tTheta: %lf", displacement, theta);

      if (abs(theta) > angular_tolerance_) {

        auto angular_speed = theta / time_to_angle_;

        angular_speed = copysign(fmax(0.0, fmin(max_angular_speed_, abs(angular_speed))), angular_speed);

        move_cmd.angular.z = angular_speed;

      } else if (displacement > linear_tolerance_) {

        auto linear_speed = displacement / time_to_x_;

        auto angular_speed = theta / time_to_angle_;

        linear_speed = copysign(fmax(0.0, fmin(max_linear_speed_, abs(linear_speed))), linear_speed);
        angular_speed = copysign(fmax(0.0, fmin(max_angular_speed_, abs(angular_speed))), angular_speed);

        move_cmd.linear.x = linear_speed;
        move_cmd.angular.z = angular_speed;

      } else {
        ROS_INFO("Goal reached!");
        is_goal_set_ = false;
      }
    }

    cmd_pub_.publish(move_cmd);

    ros::spinOnce();

    rate.sleep();
  }
}
