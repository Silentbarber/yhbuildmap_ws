#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "pointcloud_preprocess.h"

using faster_lio::PointCloudPreprocess;

livox_ros_driver::CustomMsg::Ptr fixture() {
    livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg());
    msg->points.resize(7);
    msg->point_num = msg->points.size();
    for (size_t i = 0; i < msg->points.size(); ++i) {
        auto &p = msg->points[i];
        p.x = 2.0f;
        p.y = 0.5f;
        p.z = 0.2f;
        p.line = i % 4;
        p.tag = 0x10;
        p.reflectivity = 20;
        p.offset_time = i * 1000000;
    }
    msg->points[1].x = 0.1f;
    msg->points[1].y = msg->points[1].z = 0;
    msg->points[2].x = msg->points[2].y = msg->points[2].z = 0;
    msg->points[3].x = 3.0f;
    msg->points[4].x = std::numeric_limits<float>::quiet_NaN();
    msg->points[5].tag = 0x20;
    msg->points[6].line = 4;
    return msg;
}

int main() {
    PointCloudPreprocess preprocess;
    preprocess.Set(faster_lio::LidarType::AVIA, 0.35, 1);
    preprocess.NumScans() = 4;
    PointCloudType::Ptr cloud(new PointCloudType());
    size_t near_points = 0, nonfinite_points = 0;
    for (int trial = 0; trial < 100; ++trial) {
        preprocess.Process(fixture(), cloud);
        for (const auto &p : *cloud) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) ++nonfinite_points;
            else if (p.x * p.x + p.y * p.y + p.z * p.z <= 0.35 * 0.35) ++near_points;
        }
    }
    std::cout << "{\"trials\":100,\"near_points_leaked\":" << near_points
              << ",\"nonfinite_points_leaked\":" << nonfinite_points << "}" << std::endl;
    if (near_points || nonfinite_points) return 1;
    if (cloud->size() != 1 || std::abs(cloud->points[0].x - 3.0f) > 1e-6 ||
        std::abs(cloud->points[0].curvature - 3.0f) > 1e-6) return 2;
    auto empty = fixture();
    empty->points.clear();
    empty->point_num = 0;
    preprocess.Process(empty, cloud);
    if (!cloud->empty()) return 3;
    auto single = fixture();
    single->points.resize(1);
    single->point_num = 1;
    preprocess.Process(single, cloud);
    if (!cloud->empty()) return 4;
    std::cout << "{\"status\":\"passed\",\"scope\":\"native MID360 range/finite/tag/line/time and empty scan regression\"}" << std::endl;
    return 0;
}
