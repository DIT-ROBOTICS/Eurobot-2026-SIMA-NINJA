#ifndef NINJA_SIMA_MAIN_HPP
#define NINJA_SIMA_MAIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "ninja-sima-main/ninja-sima-main_nav_to_pose.hpp"
#include <vector>
#include <queue>
#include <string>

struct WaypointTask {
    std::string name;
    double x;
    double y;
    double yaw;
    std::string task_type;
    int32_t task_num;
};

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
    void load_waypoints(const std::string& team_name);
    void execute_task(const int32_t& task_num); // execute task

    void state_transmit();
    void ninja_INIT();  // init all the param state. If all the states are ready, then change the state into READY
    void ninja_READY(); // check if the start signal is recieved. If the start signal is recieved, then change to START. Otherwise, stay in READY
    void ninja_START(); // all the processes is here, and if the stop signal is recieved then change to STOP
    void ninja_STOP();  // force stop all the process

    void ReadyCheckSub_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void StartSub_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void StopSub_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void MissionStatus_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void on_goal_reached(bool success);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ReadyCheck_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr Start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Stop_sub_;
    rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr StartUp_client_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr MissionType_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr MissionStatus_sub_;
    
    std::shared_ptr<NinjaSimaMainNavToPose> nav_to_pose_;

    std::queue<WaypointTask> task_queue_;
    bool is_task_running_;
    int32_t current_running_task_num_;
    int32_t prev_task_;
    int32_t mission_type_now_;
    int32_t mission_status_;

    std::vector<Pose2D> test_waypoints_;
    size_t current_waypoint_index_;

    NinjaSimaMainState state_;
    bool team_;
    int server_ping_fail_cnt_;
};

#endif  // NINJA_SIMA_MAIN_HPP