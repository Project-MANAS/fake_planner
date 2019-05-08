//
// Created by naivehobo on 5/7/19.
//

#include <ros/ros.h>

#include <fake_planner/FakePlanner.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_planner");

  auto planner = FakePlanner();

  planner.run();

  ros::spin();

  return 0;
}