#ifndef __WALL_FOLLOWER_HPP__
#define __WALL_FOLLOWER_HPP__

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace lab3 {

class WallFollower {
public:
    /**
     * @brief Construct a new Wall Follower object
     *
     */
    WallFollower();

    /**
     * @brief Laser scan ROS message call back function
     *
     * @param scan_msg:     new laser scan message
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    /**
     * @brief Calculates error to desired distance to left wall from laser scan
     * message
     *
     * @param scan_msg:         LiDAR LaserScan message
     * @return double:          new estimated error
     */
    double updateErrorLeft(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    /**
     * @brief Calculates desired steering angle from PID control to left wall
     *
     * @return double
     */
    double updateControlLeft();

private:
    ros::NodeHandle _n;
    ros::Subscriber _scanSubscriber;
    ros::Publisher _steerPublisher;

    const double _desired_distance;
    const double _look_ahead_distance;

    double _prevError_Left;
    double _integral_Left;
    double _error_Left;
    const double K_P_Left;
    const double K_D_Left;
    const double K_I_Left;

    double _currentTime;
    double _prevTime;
};

/**
 * @brief Converts angle in radians to degree
 *
 * @param r:        angle in radians
 * @return double
 */
double toDegree(double r);

/**
 * @brief Converts angle in degree to radians
 *
 * @param d:        angle in degree
 * @return double
 */
double toRadians(double d);

}  // namespace lab3

#endif /* __WALL_FOLLOWER_HPP__ */
