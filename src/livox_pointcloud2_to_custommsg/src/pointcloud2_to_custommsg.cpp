#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <livox_ros_driver/CustomMsg.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

class Mid360Bridge {
 public:
  Mid360Bridge() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    pnh_.param<std::string>("output_topic", output_topic_, "/livox/lidar_custom");
    pnh_.param<std::string>("frame_id", frame_id_, "livox_frame");
    pub_ = nh_.advertise<livox_ros_driver::CustomMsg>(output_topic_, 1000);
    sub_ = nh_.subscribe(input_topic_, 1000, &Mid360Bridge::cloudCallback, this);
  }

 private:
  static bool hasField(const sensor_msgs::PointCloud2& msg, const std::string& name) {
    return std::any_of(msg.fields.begin(), msg.fields.end(),
                       [&](const sensor_msgs::PointField& f) { return f.name == name; });
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!hasField(*msg, "x") || !hasField(*msg, "y") || !hasField(*msg, "z") ||
        !hasField(*msg, "intensity") || !hasField(*msg, "tag") ||
        !hasField(*msg, "line") || !hasField(*msg, "timestamp")) {
      ROS_ERROR_THROTTLE(5.0, "MID360 PointCloud2 is missing one of x/y/z/intensity/tag/line/timestamp");
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> tag(*msg, "tag");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> line(*msg, "line");
    sensor_msgs::PointCloud2ConstIterator<double> timestamp(*msg, "timestamp");

    const std::size_t count = static_cast<std::size_t>(msg->width) * msg->height;
    if (count == 0) return;

    const double first_ns = *timestamp;
    if (!std::isfinite(first_ns) || first_ns < 0.0 || first_ns > 2.0e19) {
      ROS_ERROR_THROTTLE(5.0, "Invalid MID360 point timestamp");
      return;
    }

    livox_ros_driver::CustomMsg out;
    out.header = msg->header;
    out.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;
    out.timebase = static_cast<uint64_t>(std::llround(first_ns));
    out.lidar_id = 0;
    out.point_num = static_cast<uint32_t>(std::min<std::size_t>(count, std::numeric_limits<uint32_t>::max()));
    out.points.reserve(out.point_num);

    for (std::size_t i = 0; i < out.point_num; ++i, ++x, ++y, ++z, ++intensity, ++tag, ++line, ++timestamp) {
      const double point_ns = *timestamp;
      const double delta_ns = point_ns - first_ns;
      if (!std::isfinite(delta_ns) || delta_ns < 0.0 ||
          delta_ns > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
        ROS_ERROR_THROTTLE(5.0, "Invalid MID360 point offset; rejecting scan instead of changing measurement time");
        return;
      }
      livox_ros_driver::CustomPoint p;
      p.x = *x;
      p.y = *y;
      p.z = *z;
      p.reflectivity = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, *intensity)));
      p.tag = *tag;
      p.line = *line;
      p.offset_time = static_cast<uint32_t>(std::llround(delta_ns));
      out.points.push_back(p);
    }
    // Packet order is not measurement order. Preserve times while satisfying
    // estimators that use the last point to determine scan end time.
    std::stable_sort(out.points.begin(), out.points.end(),
                     [](const livox_ros_driver::CustomPoint& a,
                        const livox_ros_driver::CustomPoint& b) {
                       return a.offset_time < b.offset_time;
                     });
    pub_.publish(out);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_, output_topic_, frame_id_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud2_to_custommsg");
  Mid360Bridge bridge;
  ros::spin();
  return 0;
}
