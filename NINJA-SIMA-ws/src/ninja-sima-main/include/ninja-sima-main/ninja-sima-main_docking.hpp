#ifndef NINJA_SIMA_MAIN_DOCKING_HPP
#define NINJA_SIMA_MAIN_DOCKING_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include <functional>
#include <memory>
#include <string>

class NinjaSimaMainDocking {
public:
    explicit NinjaSimaMainDocking(rclcpp::Node* node);
    ~NinjaSimaMainDocking() = default;

    void start_docking(const std::string& task_name, double x, double y, double yaw);
    bool is_docking() const { return is_docking_; }
    std::string debug_status() const;

    void set_result_callback(std::function<void(bool)> callback);

private:
    void dock_goal_response_callback(
        const rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::SharedPtr & goal_handle);
    void dock_feedback_callback(
        rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::SharedPtr,
        const std::shared_ptr<const opennav_docking_msgs::action::DockRobot::Feedback> feedback);
    void dock_result_callback(
        const rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::WrappedResult & result);

    rclcpp::Node* node_;
    rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SharedPtr dock_robot_client_;
    bool is_docking_;
    std::string active_docking_name_;
    rclcpp::Time docking_start_time_;
    rclcpp::Time last_feedback_time_;
    size_t feedback_count_;
    uint16_t last_feedback_state_;
    uint16_t last_feedback_retries_;
    std::function<void(bool)> result_callback_;
};

#endif // NINJA_SIMA_MAIN_DOCKING_HPP
