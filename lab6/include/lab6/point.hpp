#ifndef __LAB6_POINT_HPP__
#define __LAB6_POINT_HPP__

#include <geometry_msgs/Point.h>
#include <math.h>

#include <iostream>

namespace lab6 {

class Point {
public:
    double x, y, z;

    Point();
    Point(double in_x, double in_y, double in_z);
    Point(const Point& other);
    Point(const geometry_msgs::Point& other);

    Point& operator=(const Point& other);
    double distance(const Point& other) const;

    static Point fromMsg(const geometry_msgs::Point& point_msg);
};

std::ostream& operator<<(std::ostream& o, const Point& p);

}  // namespace lab6

#endif /* __LAB6_POINT_HPP__ */
