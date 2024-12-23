#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticArucoFramePublisher : public rclcpp::Node
{
public:
  StaticArucoFramePublisher()
  : Node("static_aruco_tf2_broadcaster"), has_published_(false)
  {
    // Initialize the static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Subscribe to ArUco marker pose
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_single/pose", 10,
      std::bind(&StaticArucoFramePublisher::marker_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "Waiting for ArUco marker detection...");
  }

private:
  void marker_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Publish only once when marker is first detected
    if (!has_published_) {
      geometry_msgs::msg::TransformStamped t;
      
      // Read message content and assign it to transform
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "map";  // Parent frame
      t.child_frame_id = "aruco_marker_frame";  // Child frame
      
      // Copy translation
      t.transform.translation.x = msg->pose.position.x;
      t.transform.translation.y = msg->pose.position.y;
      t.transform.translation.z = msg->pose.position.z;
      
      // Convert quaternion to RPY
      tf2::Quaternion q_orig(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
      );
      
      // Get Euler angles
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
      
      // Apply corrections
      yaw += M_PI;  // Add pi to yaw
      pitch += M_PI/2;  // Add pi/2 to pitch
      
      // Create new quaternion with corrected angles
      tf2::Quaternion q_corrected;
      q_corrected.setRPY(roll, pitch, yaw);
      q_corrected.normalize();
      
      // Set the corrected rotation
      t.transform.rotation.x = q_corrected.x();
      t.transform.rotation.y = q_corrected.y();
      t.transform.rotation.z = q_corrected.z();
      t.transform.rotation.w = q_corrected.w();
      
      // Send the transformation
      tf_static_broadcaster_->sendTransform(t);
      
      // Log the detection with RPY angles
      RCLCPP_INFO(
        this->get_logger(),
        "ArUco marker detected! Publishing static transform: map -> aruco_marker_frame"
        "\nTranslation: [%.2f, %.2f, %.2f]"
        "\nRotation (RPY deg): [%.2f, %.2f, %.2f]",
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z,
        roll * 180/M_PI,
        pitch * 180/M_PI,
        yaw * 180/M_PI);
        
      // Set flag to true so we don't publish again
      has_published_ = true;
      
      // Optionally unsubscribe since we don't need more updates
      subscription_.reset();
    }
  }
  
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  bool has_published_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticArucoFramePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}