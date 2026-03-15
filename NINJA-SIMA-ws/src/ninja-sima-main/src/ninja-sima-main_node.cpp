#include "ninja-sima-main/ninja-sima-main_node.hpp"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

NinjaSimaMain::NinjaSimaMain() : Node("ninja_sima_main_node"){
    state_ = NinjaSimaMainState::START;
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
    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose");

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
    // Initialize team parameter (default to false)
    this->declare_parameter<bool>("team", false); // false: blue ;true: yellow

    this->get_parameter<bool>("team", team_);
    
    RCLCPP_INFO(this->get_logger(), "NinjaSimaMain initialized");
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
    RCLCPP_INFO(this->get_logger(), "STATE: %d", static_cast<int>(state_));
    /* Do all the mission until all things being done */
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

void NinjaSimaMain::move_to_pose(double x, double y, double theta) {
    if (!nav_to_pose_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%f, y=%f, theta=%f", x, y, theta);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NinjaSimaMain::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NinjaSimaMain::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NinjaSimaMain::result_callback, this, std::placeholders::_1);

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void NinjaSimaMain::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void NinjaSimaMain::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
}

void NinjaSimaMain::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NinjaSimaMain>());
  rclcpp::shutdown();
  return 0;
}