#ifndef __PURE_PURSUIT_HPP__
#define __PURE_PURSUIT_HPP__

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fast_csv_parser/csv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lab6/point.hpp>

namespace lab6 {

class PurePursuitPlanner {
public:
    PurePursuitPlanner(const std::string& csv_path);

    void readWaypoints(const std::string& csv_path);

    void pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void updatePose(const geometry_msgs::Point& new_pos, const geometry_msgs::Quaternion& new_orientation);

private:
    ros::NodeHandle _n;
    ros::Subscriber _poseSubscriber;
    ros::Publisher _steerPublisher;

    Point _position;
    double _orientation;

    std::vector<Point> _waypoints;
};

}  // namespace lab6

#endif /* __PURE_PURSUIT_HPP__ */
