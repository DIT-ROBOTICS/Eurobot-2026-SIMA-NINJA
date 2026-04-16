#include "ninja-sima-main/ninja-sima-main_node.hpp"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace std::chrono_literals;

NinjaSimaMain::NinjaSimaMain() : Node("ninja_sima_main_node"), is_task_running_(false), current_running_task_num_(-1){
    state_ = NinjaSimaMainState::START;
    current_waypoint_index_ = 0;

    init_param();
    ReadyCheck_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/startup/are_you_ready", 10,
        std::bind(&NinjaSimaMain::ReadyCheckSub_callback, this, std::placeholders::_1));
    Start_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "robot/startup/ninja/start", 10,
        std::bind(&NinjaSimaMain::StartSub_callback, this, std::placeholders::_1));
    Stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/stop", 10,
        std::bind(&NinjaSimaMain::StopSub_callback, this, std::placeholders::_1));
    StartUp_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal");
    MissionType_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_type", 10);
    MissionStatus_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/mission_status", 10,
        std::bind(&NinjaSimaMain::MissionStatus_callback, this, std::placeholders::_1));
    
    nav_to_pose_ = std::make_shared<NinjaSimaMainNavToPose>(this);
    nav_to_pose_->set_goal_reached_callback(
        std::bind(&NinjaSimaMain::on_goal_reached, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
        20ms, std::bind(&NinjaSimaMain::timer_callback, this));
}

NinjaSimaMain::~NinjaSimaMain() {
    RCLCPP_INFO(this->get_logger(), "NinjaSimaMain node shutting down");
}

void NinjaSimaMain::timer_callback() {
    state_transmit();
}

void NinjaSimaMain::init_param() {    
    // 從 ROS 2 Parameter Server 讀取 team 參數 (預設為 "blue")
    this->declare_parameter<std::string>("team", "blue");
    std::string team_str;
    this->get_parameter("team", team_str);

    RCLCPP_INFO(this->get_logger(), "Team selected: %s", team_str.c_str());

    load_waypoints(team_str);
    
    RCLCPP_INFO(this->get_logger(), "NinjaSimaMain initialized");
}

void NinjaSimaMain::load_waypoints(const std::string& team_name) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("ninja-sima-main");
        std::string yaml_file_name = team_name + "_param.yaml";
        std::string yaml_path = package_share_directory + "/params/" + yaml_file_name;

        YAML::Node config = YAML::LoadFile(yaml_path);
        
        if (config["ninja_sima_main_node"] && config["ninja_sima_main_node"]["ros__parameters"]) {
            auto params = config["ninja_sima_main_node"]["ros__parameters"];
            
            if (params["waypoints"]) {
                for (const auto& wp_node : params["waypoints"]) {
                    std::string wp_name = wp_node.as<std::string>();
                    
                    if (params[wp_name]) {
                        WaypointTask task;
                        task.name = wp_name;
                        task.x = params[wp_name]["x"].as<double>();
                        task.y = params[wp_name]["y"].as<double>();
                        task.yaw = params[wp_name]["yaw"].as<double>();
                        task.task_type = params[wp_name]["task_type"].as<std::string>();
                        task.task_num = -1; // Default
                        
                        if (params[wp_name]["task_num"]) {
                            task.task_num = params[wp_name]["task_num"].as<std::int32_t>();
                        }
                        
                        task_queue_.push(task);
                        RCLCPP_INFO(this->get_logger(), "Loaded Task %s: [%.2f, %.2f, %.2f] Type: %s, Num: %d", 
                                    task.name.c_str(), task.x, task.y, task.yaw, task.task_type.c_str(), task.task_num);
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception parsing YAML: %s", e.what());
    }
}

void NinjaSimaMain::execute_task(const std::int32_t& task_num) {
    if (task_num <= 0) { // Assume invalid/dummy task
        is_task_running_ = false;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), ">>> Executing Task Number: %d <<<", task_num);
    
    current_running_task_num_ = task_num;
    is_task_running_ = true;
    
    // Publish task code to firmware
    std_msgs::msg::Int32 msg;
    msg.data = task_num;
    MissionType_pub_->publish(msg);
}

void NinjaSimaMain::state_transmit() {
    switch (state_) {
        case NinjaSimaMainState::INIT:
            ninja_INIT();
            break;
        case NinjaSimaMainState::READY:
            ninja_READY();
            break;
        case NinjaSimaMainState::START:
            ninja_START();
            break;
        case NinjaSimaMainState::STOP:
            ninja_STOP();
            break;
    }
}

void NinjaSimaMain::ninja_INIT() {
    RCLCPP_INFO(this->get_logger(), "STATE: %d", static_cast<int>(state_));
    /* Get the plan code and get plan file */
}

void NinjaSimaMain::ninja_READY() {
    RCLCPP_INFO(this->get_logger(), "STATE: %d", static_cast<int>(state_));
    /* Wait for the start signal */
}

void NinjaSimaMain::ninja_START() {
    /* Do all the mission until all things being done */
    if (!nav_to_pose_->is_navigating() && !is_task_running_) {
        if (!task_queue_.empty()) {
            WaypointTask wp = task_queue_.front();
            RCLCPP_INFO(this->get_logger(), "Sending waypoint %s: x=%.2f, y=%.2f, theta=%.2f",
                        wp.name.c_str(), wp.x, wp.y, wp.yaw);
            
            nav_to_pose_->move_to_pose(wp.x, wp.y, wp.yaw);
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "All tasks have been completed.");
        }
    }
}

void NinjaSimaMain::ninja_STOP() {
    RCLCPP_INFO(this->get_logger(), "STATE: %d", static_cast<int>(state_));
    /* Do the final actuator motion and stop all the moving motion */
}

void NinjaSimaMain::ReadyCheckSub_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data){
        auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
        request->group = 6; // group number = 6 => Ninja SIMA
        request->state = 1; // ready = 1
        auto captured_group = request->group;
        auto result = StartUp_client_->async_send_request(
            request,
            [this, captured_group](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "ReadySignal SUCCESS: group=%d", response->group);
                    state_ = NinjaSimaMainState::READY;
                } 
            });
    }
}

void NinjaSimaMain::StartSub_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    if (msg->data) {
        state_ = NinjaSimaMainState::START;
    } else {
        state_ = NinjaSimaMainState::READY;
    }
}

void NinjaSimaMain::StopSub_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        state_ = NinjaSimaMainState::STOP;
    }
}

void NinjaSimaMain::MissionStatus_callback(const std_msgs::msg::Int32::SharedPtr msg){
    mission_type_now_ = int32_t(msg->data/10);
    mission_status_ = msg->data%10;

    if (is_task_running_ && mission_type_now_ == current_running_task_num_) {
        // Status 1 means completed
        if (mission_status_ == 1) {
            RCLCPP_INFO(this->get_logger(), "Task %d completed by firmware.", current_running_task_num_);
            is_task_running_ = false;
            current_running_task_num_ = -1;
        }
    }
}

void NinjaSimaMain::on_goal_reached(bool success) {
    if (success && !task_queue_.empty()) {
        WaypointTask completed_task = task_queue_.front();
        task_queue_.pop(); // 導航成功，從 queue 移除
        
        if (completed_task.task_type == "docking") {
            // 如果 type 是 docking，執行 task_num
            RCLCPP_INFO(this->get_logger(), "Goal reached. Task type is 'docking'. Preparing to execute task %d.", completed_task.task_num);
            is_task_running_ = true;
            execute_task(completed_task.task_num);
        } else if (completed_task.task_type == "goto") {
            // 如果是 goto 就單純忽略 task_num
            RCLCPP_INFO(this->get_logger(), "Goal reached. Task type is 'goto'. Moving to next task.");
        }
    } else if (!success) {
        RCLCPP_WARN(this->get_logger(), "Navigation Failed! Handling error needed.");
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NinjaSimaMain>());
  rclcpp::shutdown();
  return 0;
}