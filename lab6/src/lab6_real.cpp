#include <ros/package.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <lab6/pure_pursuit.hpp>

namespace fs = boost::filesystem;

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit_planner", ros::InitOption::AnonymousName);

  fs::path package_path = ros::package::getPath("lab6");

  lab6::PurePursuitPlanner planner(
      (package_path / fs::path("csv/levine_real.csv")).string(),
      "/pf/pose/odom", "/vesc/high_level/ackermann_cmd_mux/input/nav_1");

  ros::spin();
  return 0;
}
