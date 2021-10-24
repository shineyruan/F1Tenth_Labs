#ifndef __PURE_PURSUIT_HPP__
#define __PURE_PURSUIT_HPP__

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fast_csv_parser/csv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <lab6/point.hpp>
#include <limits>

namespace lab6 {

class PurePursuitPlanner {
public:
  PurePursuitPlanner(const std::string& csv_path);
  PurePursuitPlanner(const std::string& csv_path, const std::string& pose_topic,
                     const std::string& nav_topic);

  void initMarkers();

  void readWaypoints(const std::string& csv_path);

  void pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

  void updatePose(const geometry_msgs::Point& new_pos,
                  const geometry_msgs::Quaternion& new_orientation);

  double findWaypoint(const Point& pos);

  double findGoal(const Point& pos);

private:
  ros::NodeHandle _n;
  ros::Subscriber _poseSubscriber;
  ros::Publisher _steerPublisher;
  ros::Publisher _pathVisualizer;
  ros::Publisher _posVisualizer;
  ros::Publisher _goalVisualizer;
  visualization_msgs::Marker _pathMarker;
  visualization_msgs::Marker _posMarker;
  visualization_msgs::Marker _goalMarker;

  Point _position;      // vehicle position
  double _orientation;  // vehicle orientation

  std::vector<Point> _waypoints;  // array of all waypoints
  int _currIdx;                   // idx of current waypoint

  double _lookahead;  // look ahead distance
  const double K_p;
};

}  // namespace lab6

#endif /* __PURE_PURSUIT_HPP__ */
