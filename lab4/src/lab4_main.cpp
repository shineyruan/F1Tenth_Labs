#include <ros/ros.h>

#include <lab4/planning.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lab4_planner", ros::InitOption::AnonymousName);
  lab4::FollowGapPlanner planner_followGap;

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
