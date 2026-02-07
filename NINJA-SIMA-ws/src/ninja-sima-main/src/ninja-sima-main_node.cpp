#include "ninja-sima-main/ninja-sima-main_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

NinjaSimaMain::NinjaSimaMain() : Node("ninja_sima_main_node"){
    state_ = NinjaSimaMainState::INIT;
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NinjaSimaMain>());
  rclcpp::shutdown();
  return 0;
}