#include <lab1/scan_range.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LidarProcessor {
public:
  // Read-only public member variable of closest/farthest range
  //  within a LaserScan
  const lab1::scan_range& scan_range_out;

  // constructor
  LidarProcessor() : scan_range_out(_scan_range_msg) {}

  // callback function for LaserScan subscriber
  void handleLaserScan(const sensor_msgs::LaserScanConstPtr& new_scan) {
    std::vector<float> ranges = new_scan->ranges;

    // find the min/max range from linear search
    float range_closest  = std::numeric_limits<float>::infinity();
    float range_farthest = 0.f;
    for (const float& r : ranges) {
      if (std::isnan(r) || std::isinf(r)) continue;
      if (r < range_closest) range_closest = r;
      if (r > range_farthest) range_farthest = r;
    }

    // clip the data if it exceeds the required min/max range
    _scan_range_msg.range_min.data = (range_closest < new_scan->range_min)
                                         ? new_scan->range_min
                                         : range_closest;
    _scan_range_msg.range_max.data = (range_farthest > new_scan->range_max)
                                         ? new_scan->range_max
                                         : range_farthest;

    ROS_INFO("LaserScan message received: closest %f, farthest %f",
             _scan_range_msg.range_min.data, _scan_range_msg.range_max.data);
  }

private:
  lab1::scan_range _scan_range_msg;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_subscriber_custom_msg",
            ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  LidarProcessor processor;

  ros::Subscriber laserScan_subscriber =
      nh.subscribe("/scan", 1, &LidarProcessor::handleLaserScan, &processor);
  ros::Publisher publisher = nh.advertise<lab1::scan_range>("/scan_range", 1);

  while (ros::ok()) {
    publisher.publish(processor.scan_range_out);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
