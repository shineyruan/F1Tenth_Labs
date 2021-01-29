#ifndef __PURE_PURSUIT_HPP__
#define __PURE_PURSUIT_HPP__

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fast_csv_parser/csv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace lab6 {

class PurePursuitPlanner {
public:
    PurePursuitPlanner(const std::string& csv_path);

    void readWaypoints(const std::string& csv_path);

    void pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void updatePosition(const geometry_msgs::Point& new_pos);

private:
    ros::NodeHandle _n;
    ros::Subscriber _poseSubscriber;
    ros::Publisher _steerPublisher;

    geometry_msgs::Point _position;

    std::vector<geometry_msgs::Point> _waypoints;
};

}  // namespace lab6

#endif /* __PURE_PURSUIT_HPP__ */
