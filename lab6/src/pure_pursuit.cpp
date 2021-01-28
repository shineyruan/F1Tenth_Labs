#include <lab6/pure_pursuit.hpp>

namespace lab6 {

PurePursuitPlanner::PurePursuitPlanner() {
    _n = ros::NodeHandle();
    _poseSubscriber = _n.subscribe("/odom", 1, &PurePursuitPlanner::pose_callback, this);
    _steerPublisher = _n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
}

void PurePursuitPlanner::pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    updatePosition(odom_msg->pose.pose.position);
}

void PurePursuitPlanner::updatePosition(const geometry_msgs::Point& new_pos) {
    _position = new_pos;
}

}  // namespace lab6
