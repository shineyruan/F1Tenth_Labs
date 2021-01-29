#include <lab6/pure_pursuit.hpp>

namespace lab6 {

PurePursuitPlanner::PurePursuitPlanner(const std::string& csv_path)
    : _position(),
      _waypoints() {
    _n = ros::NodeHandle();
    _poseSubscriber = _n.subscribe("/odom", 1, &PurePursuitPlanner::pose_callback, this);
    _steerPublisher = _n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
    readWaypoints(csv_path);
}

void PurePursuitPlanner::pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    updatePosition(odom_msg->pose.pose.position);
}

void PurePursuitPlanner::updatePosition(const geometry_msgs::Point& new_pos) {
    _position = new_pos;
}

#include <fast_csv_parser/csv.h>
void PurePursuitPlanner::readWaypoints(const std::string& csv_path) {
    ROS_INFO_STREAM("Reading waypoints from CSV...");

    io::CSVReader<4> file_in(csv_path);
    double x = 0.0, y = 0.0, orientation = 0.0, speed = 0.0;
    while (file_in.read_row(x, y, orientation, speed)) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        _waypoints.push_back(p);
    }

    ROS_INFO_STREAM("Read complete. Total size: " << _waypoints.size());
}

}  // namespace lab6
