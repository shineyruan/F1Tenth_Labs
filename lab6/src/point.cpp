#include <lab6/point.hpp>

namespace lab6 {

Point::Point() : x(0.0), y(0.0), z(0.0) {}

Point::Point(double in_x, double in_y, double in_z) : x(in_x), y(in_y), z(in_z) {}

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

double Point::distance(const Point& other) const {
    return std::sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y) + (other.z - z) * (other.z - z));
}

Point Point::fromMsg(const geometry_msgs::Point& point_msg) {
    Point p(point_msg);
    return p;
}

std::ostream& operator<<(std::ostream& o, const Point& p) {
    o << "[" << p.x << ", " << p.y << ", " << p.z << "]";
    return o;
}

}  // namespace lab6
