#include <ros/package.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <lab6/pure_pursuit.hpp>

namespace fs = boost::filesystem;

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit_planner", ros::InitOption::AnonymousName);

  fs::path package_path = ros::package::getPath("lab6");

  lab6::PurePursuitPlanner planner(
      (package_path / fs::path("csv/levine_pf.csv")).string());

  ros::spin();
  return 0;
}
