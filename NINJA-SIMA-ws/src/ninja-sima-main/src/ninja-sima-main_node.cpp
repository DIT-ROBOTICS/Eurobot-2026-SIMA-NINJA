#include "ninja-sima-main/ninja-sima-main_node.hpp"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace std::chrono_literals;

NinjaSimaMain::NinjaSimaMain() : Node("ninja_sima_main_node"), is_task_running_(false), is_docking_(false), current_running_task_num_(-1){
    state_ = NinjaSimaMainState::INIT;
    current_waypoint_index_ = 0;

    init_param();
    ReadyCheck_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/startup/are_you_ready", 10,
        std::bind(&NinjaSimaMain::ReadyCheckSub_callback, this, std::placeholders::_1));
    Start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/start_signal", 10,
        std::bind(&NinjaSimaMain::StartSub_callback, this, std::placeholders::_1));
    Stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/stop", 10,
        std::bind(&NinjaSimaMain::StopSub_callback, this, std::placeholders::_1));
    StartUp_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal");
    MissionType_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_type", 10);
    MissionStatus_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/mission_status", 10,
        std::bind(&NinjaSimaMain::MissionStatus_callback, this, std::placeholders::_1));
    dock_robot_client_ = rclcpp_action::create_client<opennav_docking_msgs::action::DockRobot>(
        this,
        "/dock_robot");
    
    nav_to_pose_ = std::make_shared<NinjaSimaMainNavToPose>(this);
    nav_to_pose_->set_goal_reached_callback(
        std::bind(&NinjaSimaMain::on_goal_reached, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
        20ms, std::bind(&NinjaSimaMain::timer_callback, this));
    state_ = NinjaSimaMainState::READY;
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
    if (!nav_to_pose_->is_navigating() && !is_task_running_ && !is_docking_) {
        if (!task_queue_.empty()) {
            WaypointTask wp = task_queue_.front();

            if (wp.task_type == "docking") {
                RCLCPP_INFO(this->get_logger(), "Sending docking goal %s: x=%.2f, y=%.2f, theta=%.2f",
                            wp.name.c_str(), wp.x, wp.y, wp.yaw);
                start_docking(wp);
            } else {
                RCLCPP_INFO(this->get_logger(), "Sending waypoint %s: x=%.2f, y=%.2f, theta=%.2f",
                            wp.name.c_str(), wp.x, wp.y, wp.yaw);
                nav_to_pose_->move_to_pose(wp.x, wp.y, wp.yaw);
            }
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

void NinjaSimaMain::StartSub_callback(const std_msgs::msg::Bool::SharedPtr msg) {
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
        task_queue_.pop();

        if (completed_task.task_type == "goto") {
            RCLCPP_INFO(this->get_logger(), "Goal reached. Task type is 'goto'. Moving to next task.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Navigation callback reached for non-goto task type '%s'.", completed_task.task_type.c_str());
        }
    } else if (!success) {
        RCLCPP_WARN(this->get_logger(), "Navigation Failed! Handling error needed.");
    }
}

void NinjaSimaMain::start_docking(const WaypointTask& dock_task) {
    if (!dock_robot_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "DockRobot action server '/dock_robot' is not ready.");
        is_docking_ = false;
        return;
    }

    auto goal_msg = opennav_docking_msgs::action::DockRobot::Goal();
    goal_msg.use_dock_id = false;
    goal_msg.dock_type = "dock";
    goal_msg.navigate_to_staging_pose = true;

    goal_msg.dock_pose.header.frame_id = "map";
    goal_msg.dock_pose.header.stamp = this->now();
    goal_msg.dock_pose.pose.position.x = dock_task.x;
    goal_msg.dock_pose.pose.position.y = dock_task.y;
    goal_msg.dock_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dock_task.yaw);
    goal_msg.dock_pose.pose.orientation.x = q.x();
    goal_msg.dock_pose.pose.orientation.y = q.y();
    goal_msg.dock_pose.pose.orientation.z = q.z();
    goal_msg.dock_pose.pose.orientation.w = q.w();

    auto send_goal_options = rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NinjaSimaMain::dock_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NinjaSimaMain::dock_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NinjaSimaMain::dock_result_callback, this, std::placeholders::_1);

    dock_robot_client_->async_send_goal(goal_msg, send_goal_options);
    is_docking_ = true;

    RCLCPP_INFO(this->get_logger(), "DockRobot goal sent: type=%s, use_dock_id=%s, navigate_to_staging_pose=%s",
                goal_msg.dock_type.c_str(),
                goal_msg.use_dock_id ? "true" : "false",
                goal_msg.navigate_to_staging_pose ? "true" : "false");
}

void NinjaSimaMain::dock_goal_response_callback(
    const rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "DockRobot goal was rejected by server.");
        is_docking_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "DockRobot goal accepted by server.");
    }
}

void NinjaSimaMain::dock_feedback_callback(
    rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::SharedPtr,
    const std::shared_ptr<const opennav_docking_msgs::action::DockRobot::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Docking feedback: state=%u, retries=%u",
                feedback->state, feedback->num_retries);
}

void NinjaSimaMain::dock_result_callback(
    const rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::WrappedResult & result) {
    is_docking_ = false;

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        const uint16_t error_code = result.result ? result.result->error_code : 999;
        RCLCPP_ERROR(this->get_logger(), "DockRobot failed. result_code=%d, error_code=%u",
                     static_cast<int>(result.code), error_code);
        return;
    }

    if (task_queue_.empty()) {
        RCLCPP_WARN(this->get_logger(), "DockRobot succeeded but task queue is empty.");
        return;
    }

    WaypointTask completed_task = task_queue_.front();
    if (completed_task.task_type != "docking") {
        RCLCPP_WARN(this->get_logger(), "DockRobot succeeded but front task type is '%s'.", completed_task.task_type.c_str());
        return;
    }

    task_queue_.pop();
    RCLCPP_INFO(this->get_logger(), "DockRobot succeeded. Executing mission task %d.", completed_task.task_num);
    execute_task(completed_task.task_num);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NinjaSimaMain>());
  rclcpp::shutdown();
  return 0;
}