#include <fast_csv_parser/csv.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <lab6/pure_pursuit.hpp>

namespace fs = boost::filesystem;

namespace lab6 {

PurePursuitPlanner::PurePursuitPlanner()
    : _position(), _waypoints(), _currIdx(-1) {
  _n = ros::NodeHandle();

  std::string csv_path;
  _n.getParam("csv_waypoint_path", csv_path);
  _n.getParam("pose_topic", _pose_topic);
  _n.getParam("nav_topic", _nav_topic);
  _n.getParam("nav_msg_frame_id", _nav_msg_frame_id);
  _n.getParam("path_visualize_topic", _path_viz_topic);
  _n.getParam("pose_visualize_topic", _pose_viz_topic);
  _n.getParam("goal_visualize_topic", _goal_viz_topic);
  _n.getParam("path_visualize_frame_id", _path_viz_frame_id);
  _n.getParam("pose_visualize_frame_id", _pose_viz_frame_id);
  _n.getParam("goal_visualize_frame_id", _goal_viz_frame_id);
  _n.getParam("nav_msg_speed", _speed);
  _n.getParam("lookahead_distance", _lookahead);
  _n.getParam("steering_Kp", K_p);

  _poseSubscriber =
      _n.subscribe(_pose_topic, 1, &PurePursuitPlanner::pose_callback, this);
  _steerPublisher =
      _n.advertise<ackermann_msgs::AckermannDriveStamped>(_nav_topic, 1);
  _pathVisualizer =
      _n.advertise<visualization_msgs::Marker>(_path_viz_topic, 1);
  _posVisualizer = _n.advertise<visualization_msgs::Marker>(_pose_viz_topic, 1);
  _goalVisualizer =
      _n.advertise<visualization_msgs::Marker>(_goal_viz_topic, 1);

  initMarkers();
  readWaypoints(csv_path);
}

void PurePursuitPlanner::initMarkers() {
  _pathMarker.header.frame_id    = _path_viz_frame_id;
  _pathMarker.header.stamp       = ros::Time::now();
  _pathMarker.id                 = 0;
  _pathMarker.type               = visualization_msgs::Marker::POINTS;
  _pathMarker.action             = visualization_msgs::Marker::ADD;
  _pathMarker.pose.orientation.x = 0.0;
  _pathMarker.pose.orientation.y = 0.0;
  _pathMarker.pose.orientation.z = 0.0;
  _pathMarker.pose.orientation.w = 1.0;
  _pathMarker.scale.x            = 0.1;
  _pathMarker.scale.y            = 0.1;
  _pathMarker.color.a            = 1.0;  // Don't forget to set the alpha!
  _pathMarker.color.r            = 0.0;
  _pathMarker.color.g            = 1.0;
  _pathMarker.color.b            = 0.0;

  _posMarker.header.frame_id    = _pose_viz_frame_id;
  _posMarker.header.stamp       = ros::Time::now();
  _posMarker.id                 = 1;
  _posMarker.type               = visualization_msgs::Marker::POINTS;
  _posMarker.action             = visualization_msgs::Marker::ADD;
  _posMarker.pose.orientation.x = 0.0;
  _posMarker.pose.orientation.y = 0.0;
  _posMarker.pose.orientation.z = 0.0;
  _posMarker.pose.orientation.w = 1.0;
  _posMarker.scale.x            = 0.25;
  _posMarker.scale.y            = 0.25;
  _posMarker.color.a            = 1.0;  // Don't forget to set the alpha!
  _posMarker.color.r            = 0.0;
  _posMarker.color.g            = 0.0;
  _posMarker.color.b            = 1.0;

  _goalMarker.header.frame_id    = _goal_viz_frame_id;
  _goalMarker.header.stamp       = ros::Time::now();
  _goalMarker.id                 = 2;
  _goalMarker.type               = visualization_msgs::Marker::POINTS;
  _goalMarker.action             = visualization_msgs::Marker::ADD;
  _goalMarker.pose.orientation.x = 0.0;
  _goalMarker.pose.orientation.y = 0.0;
  _goalMarker.pose.orientation.z = 0.0;
  _goalMarker.pose.orientation.w = 1.0;
  _goalMarker.scale.x            = 0.25;
  _goalMarker.scale.y            = 0.25;
  _goalMarker.color.a            = 1.0;  // Don't forget to set the alpha!
  _goalMarker.color.r            = 1.0;
  _goalMarker.color.g            = 0.0;
  _goalMarker.color.b            = 0.0;
}

void PurePursuitPlanner::pose_callback(
    const nav_msgs::Odometry::ConstPtr& odom_msg) {
  updatePose(odom_msg->pose.pose.position, odom_msg->pose.pose.orientation);

  if (_currIdx < 0) findWaypoint(_position);

  findGoal(_position);

  Point goal_local =
      Point::toLocalFrame2D(_position, _waypoints[_currIdx], _orientation);
  double curvature = 2 * (goal_local.y) / (goal_local.distance2(Point()));

  double steering_angle = K_p * curvature;

  ROS_INFO_STREAM(_currIdx << " " << _position.y << " "
                           << _waypoints[_currIdx].y << " " << curvature);

  ackermann_msgs::AckermannDriveStamped nav_msg;
  nav_msg.header.frame_id      = _nav_msg_frame_id;
  nav_msg.header.stamp         = ros::Time::now();
  nav_msg.drive.steering_angle = steering_angle;
  nav_msg.drive.speed          = _speed;

  _steerPublisher.publish(nav_msg);
  _pathVisualizer.publish(_pathMarker);
  _posVisualizer.publish(_posMarker);
  _goalVisualizer.publish(_goalMarker);
}

void PurePursuitPlanner::updatePose(
    const geometry_msgs::Point& new_pos,
    const geometry_msgs::Quaternion& new_orientation) {
  _position = Point::fromMsg(new_pos);

  tf2::Quaternion quat;
  tf2::fromMsg(new_orientation, quat);
  tf2::Matrix3x3 mat(quat);

  double roll = 0, pitch = 0, yaw = 0;
  mat.getRPY(roll, pitch, yaw);
  _orientation = yaw;

  if (_posMarker.points.empty())
    _posMarker.points.push_back(new_pos);
  else
    _posMarker.points.back() = new_pos;
}

double PurePursuitPlanner::findWaypoint(const Point& p) {
  double min_distance = std::numeric_limits<double>::infinity();

  for (unsigned int i = 0; i < _waypoints.size(); ++i) {
    if (_position.distance(_waypoints[i]) < min_distance) {
      min_distance = _position.distance(_waypoints[i]);
      _currIdx     = i;
    }
  }

  return min_distance;
}

double PurePursuitPlanner::findGoal(const Point& pos) {
  double distance = pos.distance(_waypoints[_currIdx]);
  if (distance > _lookahead) return distance;

  while (pos.distance(_waypoints[_currIdx]) < _lookahead) {
    ++_currIdx;
    if (_currIdx >= static_cast<int>(_waypoints.size()))
      _currIdx -= _waypoints.size();
  }

  geometry_msgs::Point goal;
  goal.x = _waypoints[_currIdx].x;
  goal.y = _waypoints[_currIdx].y;

  if (_goalMarker.points.empty())
    _goalMarker.points.push_back(goal);
  else
    _goalMarker.points.back() = goal;

  return pos.distance(_waypoints[_currIdx]);
}

void PurePursuitPlanner::readWaypoints(const std::string& csv_path) {
  fs::path package_path     = ros::package::getPath("lab6");
  std::string full_csv_path = (package_path / csv_path).string();
  ROS_INFO_STREAM("Reading waypoints from CSV...");
  ROS_INFO_STREAM("File path: " << full_csv_path);

  io::CSVReader<4> file_in(full_csv_path);
  double x = 0.0, y = 0.0, orientation = 0.0, speed = 0.0;
  while (file_in.read_row(x, y, orientation, speed)) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    _waypoints.emplace_back(p);
    _pathMarker.points.push_back(p);
  }

  ROS_INFO_STREAM("Read complete. Total size: " << _waypoints.size());
}

}  // namespace lab6
