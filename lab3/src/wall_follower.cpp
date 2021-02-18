#include <lab3/wall_follower.hpp>

namespace lab3 {

WallFollower::WallFollower()
    : _desired_distance(1.0),     // 0.78m for f1tenth simulator,
      _look_ahead_distance(0.9),  // since car is driving fast,
                                  // lookahead distance accounts for the delay
                                  // between signal & action
      _prevError_Left(0.0),
      _integral_Left(0.0),
      _error_Left(0.0),
      K_P_Left(1.5),
      K_D_Left(0.05),
      K_I_Left(0.25),
      _currentTime(ros::Time::now().toSec()),
      _prevTime(0.0) {
    _n = ros::NodeHandle();
    _scanSubscriber =
        _n.subscribe("/scan", 1, &WallFollower::scanCallback, this);
    _steerPublisher =
        _n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
}

void WallFollower::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    _prevTime    = _currentTime;
    _currentTime = scan_msg->header.stamp.toSec();

    double distanceToLeftWall = updateErrorLeft(scan_msg);
    double steering_angle     = -1 * updateControlLeft();

    // publish message
    ackermann_msgs::AckermannDriveStamped control_msg;
    control_msg.drive.steering_angle = steering_angle;
    control_msg.header.stamp         = ros::Time().now();
    control_msg.header.frame_id      = "laser";

    if (std::abs(toDegree(steering_angle)) >= 0.0 &&
        std::abs(toDegree(steering_angle)) <= 10)
        control_msg.drive.speed = 1.5;
    else if (std::abs(toDegree(steering_angle)) > 10.0 &&
             std::abs(toDegree(steering_angle)) <= 20.0)
        control_msg.drive.speed = 1.0;
    else
        control_msg.drive.speed = 0.5;

    _steerPublisher.publish(control_msg);
}

double WallFollower::updateErrorLeft(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Calculate the distance to left wall
    //  take the laser angle 45 degrees, 90 degrees w.r.t local frame
    int range_idx_1 = static_cast<int>((toRadians(45) - scan_msg->angle_min) /
                                       scan_msg->angle_increment);
    double range_1  = scan_msg->ranges[range_idx_1];
    double angle_1 =
        scan_msg->angle_min + range_idx_1 * scan_msg->angle_increment;

    int range_idx_2 = static_cast<int>((toRadians(90) - scan_msg->angle_min) /
                                       scan_msg->angle_increment);
    double range_2  = scan_msg->ranges[range_idx_2];
    double angle_2 =
        scan_msg->angle_min + range_idx_2 * scan_msg->angle_increment;

    double theta = std::abs(angle_1 - angle_2);

    // alpha = atan((a*cos(theta) - b) / (a*sin(theta)))
    double alpha    = std::atan((range_1 * std::cos(theta) - range_2) /
                             (range_1 * std::sin(theta)));
    double distance = range_2 * std::cos(alpha);

    double distance_overhead =
        distance + _look_ahead_distance * std::sin(alpha);

    _prevError_Left = _error_Left;
    _error_Left     = _desired_distance - distance_overhead;

    return distance_overhead;
}

double WallFollower::updateControlLeft() {
    double dt = _currentTime - _prevTime;

    // integral term
    _integral_Left += _error_Left * dt;

    // derivative term
    double derivative = (_error_Left - _prevError_Left) / dt;

    // calculate PID control
    return K_P_Left * _error_Left + K_I_Left * _integral_Left +
           K_D_Left * derivative;
}

double toDegree(double r) { return r * 180 / M_PI; }

double toRadians(double d) { return d * M_PI / 180; }

}  // namespace lab3
