#include <ros/ros.h>

#include <filesystem>
#include <lab6/pure_pursuit.hpp>

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_planner", ros::InitOption::AnonymousName);
    lab6::PurePursuitPlanner planner(fs::current_path() / fs::path("src/F1Tenth_Labs/lab6/csv/levine_pf.csv"));
    ros::spin();
    return 0;
}
