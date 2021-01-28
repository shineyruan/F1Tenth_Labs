#ifndef __PLANNING_HPP__
#define __PLANNING_HPP__

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

namespace lab4 {

class FollowGapPlanner {
public:
    /**
     * @brief Construct a new Follow Gap Planner object
     * 
     */
    FollowGapPlanner();

    /**
     * @brief Callback function for LaserScan LiDAR message
     * 
     * @param scan_msg 
     */
    void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    /**
     * @brief Truncates the laser scan endpoints according to FOV (Field of Vision).
     *      Keep only those whose angles are between -fov/2 and fov/2
     * 
     * @param scan_msg:             Incoming laser scan message
     * @param fov_min:              empty variable for holding min angle value for truncated scan data
     * @param fov_max:              empty variable for holding max angle value for truncated scan data
     * @param angle_increment:      empty variable for holding angle increment for truncated scan data
     * @return std::vector<float> 
     */
    std::vector<float> truncateFOV(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                   float* fov_min, float* fov_max, float* angle_increment);

    /**
     * @brief Finds the max chunk of free space & the farthest endpoint in the chunk
     * 
     * @param ranges:           processed laser scan data
     * @return unsigned int:    index of farthest endpoint in the widest free space
     */
    unsigned int findMaxFreeSpace(const std::vector<float>& ranges);

    /**
     * @brief Clips the scan data between range_min and range_max & averages data using a sliding window
     * 
     * @param ranges:       raw laser scan data
     * @param range_min:    upper bound of scan data
     * @param range_max:    lower bound of scan data
     */
    void processScan(std::vector<float>& ranges, const double range_min, const double range_max);

    /**
     * @brief Imposes the "safety angle" concept on scan data. 
     *      Makes endpoints close to obstacle as 0 as well
     * 
     * @param ranges:               laser scan data
     * @param safe_angle_radius:    radius of safety angle in index (safety_angle / angle_increment)
     */
    void imposeSafetyAngle(std::vector<float>& ranges, const int safe_angle_radius);

private:
    ros::NodeHandle _n;
    ros::Subscriber _scanSubscriber;
    ros::Publisher _steerPublisher;

    // safety bubble radius (in meters)
    const int _bubbleRadius;
    // Field of View angle
    const double _fov;
    // Length of sliding window in running averaging laser range data
    const unsigned int _slidingWindowLength;
    // Safety angle
    const double _safety_angle;
};

/**
 * @brief Finds the distance between two laser scan endpoints given the ranges & angle 
 * 
 *      This function calculates the length of opposite side of a triangle using the Law of Cosines
 *      c = sqrt(a^2 + b^2 - 2*a*b*cos(angle))
 * 
 * @param sideA:            adjacent side A
 * @param sideB:            adjacent side B
 * @param included_angle:   included angle of the adjacent sides
 * @return double:          length of opposite side of the angle
 */
double findRadius(double sideA, double sideB, double included_angle);

/**
 * @brief Radius to degree converter
 * 
 * @param r:        radius value
 * @return double:  degree value
 */
double toDegrees(double r);

/**
 * @brief degree to radius converter
 * 
 * @param d:        degree value
 * @return double:  radius value
 */
double toRadius(double d);

}  // namespace lab4

#endif /* __PLANNING_HPP__ */
