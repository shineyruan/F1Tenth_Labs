#include <lab2/utils.hpp>

namespace lab2_utils {

double vecLength(const geometry_msgs::Vector3 &v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

}  // namespace lab2_utils
