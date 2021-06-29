#include <lab6/point.hpp>

namespace lab6 {

Point::Point() : x(0.0), y(0.0), z(0.0) {}

Point::Point(double in_x, double in_y, double in_z)
    : x(in_x), y(in_y), z(in_z) {}

Point::Point(const Point& other) {
  x = other.x;
  y = other.y;
  z = other.z;
}

Point::Point(const geometry_msgs::Point& other) {
  x = other.x;
  y = other.y;
  z = other.z;
}

Point& Point::operator=(const Point& other) {
  x = other.x;
  y = other.y;
  z = other.z;

  return *this;
}

Point Point::operator-(const Point& other) const {
  Point ret;
  ret.x = this->x - other.x;
  ret.y = this->y - other.y;
  ret.z = this->z - other.z;

  return ret;
}

double Point::distance(const Point& other) const {
  return std::sqrt((other.x - x) * (other.x - x) +
                   (other.y - y) * (other.y - y) +
                   (other.z - z) * (other.z - z));
}

double Point::distance2(const Point& other) const {
  return (other.x - x) * (other.x - x) + (other.y - y) * (other.y - y) +
         (other.z - z) * (other.z - z);
}

Point Point::fromMsg(const geometry_msgs::Point& point_msg) {
  Point p(point_msg);
  return p;
}

Point Point::toLocalFrame2D(const Point& base, const Point& target,
                            double orientation) {
  Point p = target - base;
  Point ret;
  ret.x = p.x * std::cos(orientation) + p.y * std::sin(orientation);
  ret.y = -p.x * std::sin(orientation) + p.y * std::cos(orientation);

  return ret;
}

std::ostream& operator<<(std::ostream& o, const Point& p) {
  o << "[" << p.x << ", " << p.y << ", " << p.z << "]";
  return o;
}

}  // namespace lab6
