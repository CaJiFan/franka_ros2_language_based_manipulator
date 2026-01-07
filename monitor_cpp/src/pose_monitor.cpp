#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip> // For nice formatting

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class PoseMonitor : public rclcpp::Node
{
public:
  PoseMonitor() : Node("pose_monitor")
  {
    // 1. Initialize the TF (Transform) Listener
    // This allows the node to listen to the robot's geometry
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. Create a timer to run the loop at 10Hz
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PoseMonitor::print_pose, this));

    RCLCPP_INFO(this->get_logger(), "Pose Monitor Started. Move the robot!");
  }

private:
  void print_pose()
  {
    // CONFIGURATION: Check these frame names!
    // For FR3, it is usually 'fr3_link0' and 'fr3_hand_tcp'
    // If your lab uses the Panda description, it might be 'panda_link0'
    std::string from_frame = "fr3_link0"; 
    std::string to_frame = "fr3_hand_tcp";
    // std::string to_frame = "fr3_link8";

    try {
      // Ask TF for the latest transform
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        from_frame, to_frame, tf2::TimePointZero);

      // --- Translation ---
      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;

      // --- Rotation (Quaternion to Euler) ---
      tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // Convert radians to degrees for readability
      double r_deg = roll * 180.0 / M_PI;
      double p_deg = pitch * 180.0 / M_PI;
      double y_deg = yaw * 180.0 / M_PI;

      // --- Print to Screen ---
      // \r returns the cursor to the start of the line (animation effect)
      std::cout << "\rPOS: x=" << std::fixed << std::setprecision(3) << x 
                << " y=" << y << " z=" << z 
                << " | RPY: " << std::setprecision(1) << r_deg 
                << " " << p_deg << " " << y_deg << "     " << std::flush;

    } catch (const tf2::TransformException & ex) {
      // It is normal to fail for the first few seconds while connecting
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseMonitor>());
  rclcpp::shutdown();
  return 0;
}