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

void NinjaSimaMainNavToPose::move_to_pose(const std::string& task_name, double x, double y, double theta) {
    active_goal_name_ = task_name;
    RCLCPP_INFO(node_->get_logger(), "NavToPose preparing goal %s", active_goal_name_.c_str());

    if (!nav_to_pose_client_->action_server_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available for goal %s", active_goal_name_.c_str());
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

    RCLCPP_INFO(node_->get_logger(), "Sending goal %s: x=%f, y=%f, theta=%f", active_goal_name_.c_str(), x, y, theta);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NinjaSimaMainNavToPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NinjaSimaMainNavToPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NinjaSimaMainNavToPose::result_callback, this, std::placeholders::_1);

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    is_navigating_ = true;
    RCLCPP_INFO(node_->get_logger(), "NavigateToPose goal %s dispatched; is_navigating=true", active_goal_name_.c_str());
}

void NinjaSimaMainNavToPose::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose goal %s was rejected by server", active_goal_name_.c_str());
        is_navigating_ = false; // Reset to allow retrying or fallback
        if (goal_reached_callback_) goal_reached_callback_(false);
    } else {
        RCLCPP_INFO(node_->get_logger(), "NavigateToPose goal %s accepted by server, waiting for result", active_goal_name_.c_str());
    }
}

void NinjaSimaMainNavToPose::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "NavigateToPose feedback %s: distance_remaining=%.3f, recoveries=%d, current=(%.3f, %.3f)",
        active_goal_name_.c_str(),
        feedback->distance_remaining,
        feedback->number_of_recoveries,
        feedback->current_pose.pose.position.x,
        feedback->current_pose.pose.position.y);
}

void NinjaSimaMainNavToPose::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
    bool success = false;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "NavigateToPose goal %s succeeded!", active_goal_name_.c_str());
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "NavigateToPose goal %s was aborted", active_goal_name_.c_str());
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "NavigateToPose goal %s was canceled", active_goal_name_.c_str());
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "NavigateToPose goal %s returned unknown result code %d",
                active_goal_name_.c_str(), static_cast<int>(result.code));
            break;
    }
    is_navigating_ = false; // Allow sending the next goal or retrying
    RCLCPP_INFO(node_->get_logger(), "NavigateToPose result callback %s: success=%s, is_navigating=false",
        active_goal_name_.c_str(), success ? "true" : "false");
    if (goal_reached_callback_) {
        goal_reached_callback_(success);
    }
}
