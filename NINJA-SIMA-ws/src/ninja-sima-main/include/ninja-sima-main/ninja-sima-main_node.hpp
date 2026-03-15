#ifndef NINJA_SIMA_MAIN_HPP
#define NINJA_SIMA_MAIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "ninja-sima-main/ninja-sima-main_nav_to_pose.hpp"
#include <vector>

struct Pose2D {
    double x;
    double y;
    double theta;
};

enum class NinjaSimaMainState{
    INIT,
    READY,
    START,
    STOP
};

class NinjaSimaMain : public rclcpp::Node
{
public:
    NinjaSimaMain();
    ~NinjaSimaMain();

private:
    void timer_callback();
    void init_param();

    void state_transmit();
    void ninja_INIT();  // init all the param state. If all the states are ready, then change the state into READY
    void ninja_READY(); // check if the start signal is recieved. If the start signal is recieved, then change to START. Otherwise, stay in READY
    void ninja_START(); // all the processes is here, and if the stop signal is recieved then change to STOP
    void ninja_STOP();  // force stop all the process

    void ReadyCheckSub_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void StartSub_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void StopSub_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void on_goal_reached(bool success);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ReadyCheck_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Stop_sub_;
    rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr StartUp_client_;
    
    std::shared_ptr<NinjaSimaMainNavToPose> nav_to_pose_;

    std::vector<Pose2D> test_waypoints_;
    size_t current_waypoint_index_;

    NinjaSimaMainState state_;
    bool team_;
    int server_ping_fail_cnt_;
};

#endif  // NINJA_SIMA_MAIN_HPP