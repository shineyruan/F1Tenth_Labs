#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

class LidarProcessor {
public:
    // Read-only public member variable of closest/farthest range
    //  within a LaserScan
    const std_msgs::Float64& range_closest;
    const std_msgs::Float64& range_farthest;

    // constructor
    LidarProcessor() : range_closest(_closest), range_farthest(_farthest) {}

    void handleLaserScan(const sensor_msgs::LaserScanConstPtr& new_scan) {
        std::vector<float> ranges = new_scan->ranges;

        float range_closest = std::numeric_limits<float>::infinity();
        float range_farthest = 0.f;
        for (const float& r : ranges) {
            if (std::isnan(r) || std::isinf(r)) continue;
            if (r < range_closest) range_closest = r;
            if (r > range_farthest) range_farthest = r;
        }

        _closest.data = range_closest;
        _farthest.data = range_farthest;

        ROS_INFO("LaserScan message received: closest %f, farthest %f", _closest.data, _farthest.data);
    }

private:
    std_msgs::Float64 _closest;
    std_msgs::Float64 _farthest;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    LidarProcessor processor;

    ros::Subscriber laserScan_subscriber = nh.subscribe("/scan", 1, &LidarProcessor::handleLaserScan, &processor);
    ros::Publisher closestPoint_publisher = nh.advertise<std_msgs::Float64>("/closest_point", 1);
    ros::Publisher farthestPoint_publisher = nh.advertise<std_msgs::Float64>("/farthest_point", 1);

    while (ros::ok()) {
        closestPoint_publisher.publish(processor.range_closest);
        farthestPoint_publisher.publish(processor.range_farthest);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
