// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class CameraOffsetPublisher : public rclcpp::Node
{
public:
  CameraOffsetPublisher()
  : Node("camera_offset_publisher"),
    rng_(std::random_device{}()),
    uniform_dist_(0.0, 1.0)
  {
    constexpr double kDefaultDetectionMinDistance = 0.05;
    constexpr double kDefaultDetectionMaxDistance = 2.0;
    constexpr double kDefaultHorizontalFovDeg = 70.0;
    constexpr double kDefaultDetectionProbability = 1.0;
    constexpr double kDefaultMeasurementNoise = 0.0;

    // Declare parameters
    this->declare_parameter("goal_x", 1.5);
    this->declare_parameter("goal_y", 1.0);
    this->declare_parameter("publish_rate", 100.0);  // Hz
    this->declare_parameter("detection_min_distance", kDefaultDetectionMinDistance);
    this->declare_parameter("detection_max_distance", kDefaultDetectionMaxDistance);
    this->declare_parameter("horizontal_fov_deg", kDefaultHorizontalFovDeg);
    this->declare_parameter("detection_probability", kDefaultDetectionProbability);
    this->declare_parameter("measurement_noise_stddev_x", kDefaultMeasurementNoise);
    this->declare_parameter("measurement_noise_stddev_y", kDefaultMeasurementNoise);
    
    // Get parameters
    this->get_parameter("goal_x", goal_x_);
    this->get_parameter("goal_y", goal_y_);
    double publish_rate;
    this->get_parameter("publish_rate", publish_rate);
    this->get_parameter("detection_min_distance", detection_min_distance_);
    this->get_parameter("detection_max_distance", detection_max_distance_);
    this->get_parameter("horizontal_fov_deg", horizontal_fov_deg_);
    this->get_parameter("detection_probability", detection_probability_);
    this->get_parameter("measurement_noise_stddev_x", measurement_noise_stddev_x_);
    this->get_parameter("measurement_noise_stddev_y", measurement_noise_stddev_y_);

    detection_probability_ = std::clamp(detection_probability_, 0.0, 1.0);
    horizontal_fov_deg_ = std::clamp(horizontal_fov_deg_, 0.0, 180.0);
    if (detection_max_distance_ < detection_min_distance_) {
      std::swap(detection_max_distance_, detection_min_distance_);
    }
    
    // Initialize current robot pose
    current_robot_x_ = 0.0;
    current_robot_y_ = 0.0;
    current_robot_yaw_ = 0.0;
    pose_received_ = false;
    
    // Create publisher for detected dock pose
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_dock_pose", 
      rclcpp::QoS(10).durability_volatile()
    );
    
    // Create publisher for absolute velocity
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/absolute_velocity",
      rclcpp::QoS(10)
    );
    
    // Camera detection feedback: true when object is detected in view
    detection_feedback_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/camera/object_detected",
      rclcpp::QoS(10).reliable().transient_local()
    );
    
    // Subscribe to robot pose
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose/global",
      rclcpp::QoS(10),
      std::bind(&CameraOffsetPublisher::pose_callback, this, std::placeholders::_1)
    );
    
    // Subscribe to cmd_vel
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::QoS(10),
      std::bind(&CameraOffsetPublisher::cmd_vel_callback, this, std::placeholders::_1)
    );
    
    // Create timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CameraOffsetPublisher::timer_callback, this)
    );
    
    // Initialize feedback state
    publish_detection_feedback(false);
    
    RCLCPP_INFO(this->get_logger(), "Camera Offset Publisher started");
    RCLCPP_INFO(this->get_logger(), "Goal position: (%.3f, %.3f)", goal_x_, goal_y_);
    RCLCPP_INFO(this->get_logger(), "Publishing at %.1f Hz", publish_rate);
    RCLCPP_INFO(
      this->get_logger(),
      "Camera simulation: range=[%.2f, %.2f] m, FOV=%.1f deg, probability=%.2f, noise=(%.3f, %.3f)",
      detection_min_distance_, detection_max_distance_,
      horizontal_fov_deg_, detection_probability_,
      measurement_noise_stddev_x_, measurement_noise_stddev_y_);
  }

private:
  void publish_detection_feedback(const bool detected)
  {
    auto feedback = std_msgs::msg::Bool();
    feedback.data = detected;
    detection_feedback_publisher_->publish(feedback);
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_robot_x_ = msg->pose.pose.position.x;
    current_robot_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    auto q = msg->pose.pose.orientation;
    current_robot_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    
    pose_received_ = true;
  }
  
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Calculate absolute velocity (magnitude of linear velocity vector)
    double abs_velocity = std::sqrt(
      std::pow(msg->linear.x, 2) + 
      std::pow(msg->linear.y, 2) + 
      std::pow(msg->linear.z, 2)
    );
    
    // Publish absolute velocity
    auto velocity_msg = std_msgs::msg::Float64();
    velocity_msg.data = abs_velocity;
    velocity_publisher_->publish(velocity_msg);
  }

  void timer_callback()
  {
    constexpr double kPi = 3.14159265358979323846;
    const double half_fov_rad = (horizontal_fov_deg_ * kPi / 180.0) * 0.5;
    
    if (!pose_received_) {
      // No pose received yet, report not detected.
      publish_detection_feedback(false);
      return;
    }

    // Calculate relative position in robot frame.
    const double dx_global = goal_x_ - current_robot_x_;
    const double dy_global = goal_y_ - current_robot_y_;
    const double cos_yaw = std::cos(current_robot_yaw_);
    const double sin_yaw = std::sin(current_robot_yaw_);
    const double local_x = dx_global * cos_yaw + dy_global * sin_yaw;
    const double local_y = -dx_global * sin_yaw + dy_global * cos_yaw;

    const double distance = std::hypot(local_x, local_y);
    const double bearing = std::atan2(local_y, local_x);
    const bool in_front = local_x > 0.0;
    const bool in_range =
      distance >= detection_min_distance_ && distance <= detection_max_distance_;
    const bool in_fov = std::fabs(bearing) <= half_fov_rad;
    const bool pass_probability = uniform_dist_(rng_) <= detection_probability_;
    const bool detected = in_front && in_range && in_fov && pass_probability;

    publish_detection_feedback(detected);
    if (!detected) {
      if ((count_ % 100) == 0U) {
        RCLCPP_INFO(
          this->get_logger(),
          "No detection | distance: %.3f m | bearing: %.1f deg",
          distance, bearing * 180.0 / kPi);
      }
      ++count_;
      return;
    }

    auto message = geometry_msgs::msg::PoseStamped();
    message.header.frame_id = "base_link";
    message.header.stamp = this->now();

    std::normal_distribution<double> noise_x_dist(0.0, measurement_noise_stddev_x_);
    std::normal_distribution<double> noise_y_dist(0.0, measurement_noise_stddev_y_);
    message.pose.position.x = local_x + noise_x_dist(rng_);
    message.pose.position.y = local_y + noise_y_dist(rng_);
    message.pose.position.z = 0.0;

    // Keep a fixed orientation in base_link for docking pipeline.
    message.pose.orientation.w = 0.70710678;
    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.70710678;
    
    publisher_->publish(message);

    if ((count_ % 100) == 0U) {
      RCLCPP_INFO(
        this->get_logger(),
        "Detected | Robot: (%.3f, %.3f) | Goal: (%.3f, %.3f) | Offset: (%.3f, %.3f) | Distance: %.3f",
        current_robot_x_, current_robot_y_,
        goal_x_, goal_y_,
        message.pose.position.x, message.pose.position.y,
        distance);
    }
    ++count_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_feedback_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  double goal_x_;
  double goal_y_;
  double detection_min_distance_;
  double detection_max_distance_;
  double horizontal_fov_deg_;
  double detection_probability_;
  double measurement_noise_stddev_x_;
  double measurement_noise_stddev_y_;
  double current_robot_x_;
  double current_robot_y_;
  double current_robot_yaw_;
  bool pose_received_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraOffsetPublisher>());
  rclcpp::shutdown();
  return 0;
}
