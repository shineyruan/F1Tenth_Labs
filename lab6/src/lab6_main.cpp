#include <ros/ros.h>

#include <lab6/pure_pursuit.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit_planner", ros::InitOption::AnonymousName);

  lab6::PurePursuitPlanner planner;

  ros::spin();
  return 0;
}
