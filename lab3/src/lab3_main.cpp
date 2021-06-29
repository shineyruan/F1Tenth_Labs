#include <ros/ros.h>

#include <lab3/wall_follower.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_follower");
  lab3::WallFollower wall_follower;

  ros::Rate loopRate(30);

  while (ros::ok()) {
    loopRate.sleep();
    ros::spinOnce();
  }

  return 0;
}
