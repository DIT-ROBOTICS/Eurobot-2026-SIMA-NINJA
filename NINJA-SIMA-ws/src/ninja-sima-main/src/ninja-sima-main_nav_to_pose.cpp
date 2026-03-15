#include "ninja-sima-main/ninja-sima-main_nav_to_pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

NinjaSimaMainNavToPose::NinjaSimaMainNavToPose(rclcpp::Node* node)
    : node_(node), is_navigating_(false) {
    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_,
        "navigate_to_pose");
}

void NinjaSimaMainNavToPose::set_goal_reached_callback(std::function<void(bool)> callback) {
    goal_reached_callback_ = callback;
}

void NinjaSimaMainNavToPose::move_to_pose(double x, double y, double theta) {
    if (!nav_to_pose_client_->action_server_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        is_navigating_ = false;
        if (goal_reached_callback_) goal_reached_callback_(false);
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(node_->get_logger(), "Sending goal: x=%f, y=%f, theta=%f", x, y, theta);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NinjaSimaMainNavToPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NinjaSimaMainNavToPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NinjaSimaMainNavToPose::result_callback, this, std::placeholders::_1);

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    is_navigating_ = true;
}

void NinjaSimaMainNavToPose::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        is_navigating_ = false; // Reset to allow retrying or fallback
        if (goal_reached_callback_) goal_reached_callback_(false);
    } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void NinjaSimaMainNavToPose::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(node_->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
}

void NinjaSimaMainNavToPose::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
    bool success = false;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            break;
    }
    is_navigating_ = false; // Allow sending the next goal or retrying
    if (goal_reached_callback_) {
        goal_reached_callback_(success);
    }
}
