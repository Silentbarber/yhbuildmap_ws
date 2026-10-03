#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuUnits {
 public:
  ImuUnits() : private_nh_("~") {
    std::string input, output;
    private_nh_.param<std::string>("input_topic", input, "/livox/imu");
    private_nh_.param<std::string>("output_topic", output, "/mid360/imu_si");
    private_nh_.param<double>("acceleration_scale", scale_, 9.80665);
    pub_ = nh_.advertise<sensor_msgs::Imu>(output, 2000);
    sub_ = nh_.subscribe(input, 20000, &ImuUnits::callback, this);
    ROS_INFO("IMU acceleration scale: %.8f, %s -> %s", scale_, input.c_str(), output.c_str());
  }

 private:
  void callback(const sensor_msgs::Imu::ConstPtr& input) {
    sensor_msgs::Imu output = *input;
    output.linear_acceleration.x *= scale_;
    output.linear_acceleration.y *= scale_;
    output.linear_acceleration.z *= scale_;
    if (output.linear_acceleration_covariance[0] >= 0) {
      for (double& entry : output.linear_acceleration_covariance) entry *= scale_ * scale_;
    }
    pub_.publish(output);
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double scale_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_g_to_si");
  ImuUnits units;
  ros::spin();
  return 0;
}
