#include <lab4/planning.hpp>

namespace lab4 {

FollowGapPlanner::FollowGapPlanner()
    : _bubbleRadius(1),
      _fov(180.f),
      _slidingWindowLength(5),
      _safety_angle(20.0)  // 20 degrees for Levine Hall
{
    _n = ros::NodeHandle();
    _scanSubscriber =
        _n.subscribe("/scan", 1, &FollowGapPlanner::laserScan_callback, this);
    _steerPublisher =
        _n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
}

void FollowGapPlanner::laserScan_callback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // 0. Save the current laser scan
    //      Truncate the field of view angle
    float fov_min = 0.f, fov_max = 0.f, angle_increment = 0.f;
    std::vector<float> scan_ranges =
        truncateFOV(scan_msg, &fov_min, &fov_max, &angle_increment);
    processScan(scan_ranges, scan_msg->range_min, scan_msg->range_max);
    int num_scan = static_cast<int>(scan_ranges.size());

    // 1. Find the closest laser point (and its index)
    auto min_ptr = std::min_element(scan_ranges.begin(), scan_ranges.end());
    const double min_range = *min_ptr;
    int min_idx            = min_ptr - scan_ranges.begin();

    // 2. Set the ranges to 0 using radius 2 around that closest point
    for (int i = 0; i < num_scan; ++i) {
        if (findRadius(scan_ranges[i], min_range,
                       angle_increment * std::abs(min_idx - i)) <=
            _bubbleRadius)
            scan_ranges[i] = 0;
    }
    // imposes safety angle on the scan data
    imposeSafetyAngle(scan_ranges, static_cast<int>(toRadius(_safety_angle) /
                                                    angle_increment));

    // 3. Find maximum length sequence of consecutive non-zeros among "free
    // space" points
    //          In this case we only have 2 chunks: left chunk of the
    //          bubble/right chunk of the bubble
    unsigned int target_idx = findMaxFreeSpace(scan_ranges);

    // 4. Choose the furthest point in free space, set the steering angle
    // towards it
    //
    double steering_angle = fov_min + target_idx * angle_increment;

    // 5. Publish control message
    ackermann_msgs::AckermannDriveStamped control_msg;
    control_msg.header.stamp         = ros::Time::now();
    control_msg.header.frame_id      = "laser";
    control_msg.drive.steering_angle = steering_angle;
    control_msg.drive.speed          = 2.0;

    _steerPublisher.publish(control_msg);
}

void FollowGapPlanner::imposeSafetyAngle(std::vector<float>& ranges,
                                         const int safety_angle_radius) {
    // find start idx & end idx of region where entries are all 0
    std::vector<std::pair<int, int>> obstacle_regions;
    int i = 0;
    while (i < static_cast<int>(ranges.size())) {
        if (ranges[i] == 0) {
            int region_start = i;
            int j            = i;
            while (j < ranges.size() && ranges[j] == 0) ++j;
            int region_end = j;
            obstacle_regions.emplace_back(region_start, region_end);
            i = region_end;
        }
        ++i;
    }

    // expand the "0"-region by safety_angle_radius
    for (const auto& p : obstacle_regions) {
        for (int j = p.first; j >= std::max(0, p.first - safety_angle_radius);
             --j)
            ranges[j] = 0;
        for (int j = p.second; j < std::min(static_cast<int>(ranges.size()),
                                            p.second + safety_angle_radius);
             ++j)
            ranges[j] = 0;
    }
}

void FollowGapPlanner::processScan(std::vector<float>& ranges,
                                   const double range_min,
                                   const double range_max) {
    // clip noisy data
    for (float& r : ranges) {
        if (r < range_min)
            r = range_min;
        else if (r > range_max)
            r = range_max;
    }

    // impose sliding window running average on scan data
    std::vector<float> temp(ranges.size(), 0);
    std::deque<float> running_avg;

    for (unsigned int i = 0; i < ranges.size(); ++i) {
        running_avg.push_back(ranges[i]);
        if (running_avg.size() > _slidingWindowLength) running_avg.pop_front();

        double sum = std::accumulate(running_avg.begin(), running_avg.end(), 0);
        temp[i]    = sum / running_avg.size();
    }

    ranges = temp;
}

unsigned int FollowGapPlanner::findMaxFreeSpace(
    const std::vector<float>& ranges) {
    // one-pass algorithm
    //  finds the widest non-zero free space
    unsigned int count = 0, max_count = 0, max_end = 0;
    for (unsigned int i = 0; i <= ranges.size(); ++i) {
        if (i < ranges.size() && ranges[i] > 0.f)
            ++count;
        else {
            if (count > max_count) {
                max_count = count;
                max_end   = i;
            }
            count = 0;
        }
    }

    // finds the max element from the chosen non-zero free space
    auto max_range_ptr = std::max_element(
        ranges.begin() + (max_end - max_count), ranges.begin() + max_end);
    return (max_range_ptr - ranges.begin());
}

std::vector<float> FollowGapPlanner::truncateFOV(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg, float* fov_min,
    float* fov_max, float* angle_increment) {
    // only keeps data between -_fov/2 to _fov/2
    double fov_2   = toRadius(_fov / 2);
    int fov_min_id = static_cast<int>(
        ((-fov_2 - scan_msg->angle_min) / scan_msg->angle_increment));
    int fov_max_id = static_cast<int>((fov_2 - scan_msg->angle_min) /
                                      scan_msg->angle_increment);

    *angle_increment = scan_msg->angle_increment;
    *fov_min = scan_msg->angle_min + fov_min_id * scan_msg->angle_increment;
    *fov_max = scan_msg->angle_min + fov_max_id * scan_msg->angle_increment;

    return std::vector<float>(scan_msg->ranges.begin() + fov_min_id,
                              scan_msg->ranges.begin() + fov_max_id);
}

double findRadius(double sideA, double sideB, double included_angle) {
    // the Law of Cosines
    return std::sqrt(sideA * sideA + sideB * sideB -
                     2 * sideA * sideB * std::cos(included_angle));
}

double toDegree(double r) { return r * 180 / M_PI; }

double toRadius(double d) { return d * M_PI / 180; }

}  // namespace lab4
