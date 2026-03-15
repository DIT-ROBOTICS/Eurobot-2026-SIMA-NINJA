#ifndef NINJA_SIMA_MAIN_NAV_TO_POSE_HPP
#define NINJA_SIMA_MAIN_NAV_TO_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <functional>
#include <memory>

class NinjaSimaMainNavToPose {
public:
    NinjaSimaMainNavToPose(rclcpp::Node* node);
    ~NinjaSimaMainNavToPose() = default;

    void move_to_pose(double x, double y, double theta);
    bool is_navigating() const { return is_navigating_; }

    void set_goal_reached_callback(std::function<void(bool)> callback);

private:
    rclcpp::Node* node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    bool is_navigating_;
    std::function<void(bool)> goal_reached_callback_;

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle);
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
};

#endif // NINJA_SIMA_MAIN_NAV_TO_POSE_HPP
