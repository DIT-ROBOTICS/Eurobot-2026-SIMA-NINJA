#include "ninja-sima-main/ninja-sima-main_docking.hpp"

#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
constexpr float kMaxStagingTimeSeconds = 20.0F;
}  // namespace

NinjaSimaMainDocking::NinjaSimaMainDocking(rclcpp::Node* node)
	: node_(node),
	  is_docking_(false),
	  feedback_count_(0),
	  last_feedback_state_(0),
	  last_feedback_retries_(0) {
	dock_robot_client_ = rclcpp_action::create_client<opennav_docking_msgs::action::DockRobot>(
		node_,
		"/dock_robot");
}

void NinjaSimaMainDocking::set_result_callback(std::function<void(bool)> callback) {
	result_callback_ = callback;
}

void NinjaSimaMainDocking::start_docking(const std::string& task_name, double x, double y, double yaw) {
	active_docking_name_ = task_name;
	feedback_count_ = 0;
	last_feedback_state_ = 0;
	last_feedback_retries_ = 0;
	docking_start_time_ = node_->now();
	last_feedback_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
	RCLCPP_INFO(node_->get_logger(), "DockRobot preparing goal %s", active_docking_name_.c_str());

	if (!dock_robot_client_->action_server_is_ready()) {
		RCLCPP_ERROR(node_->get_logger(), "DockRobot action server '/dock_robot' is not ready for goal %s.", active_docking_name_.c_str());
		is_docking_ = false;
		if (result_callback_) {
			result_callback_(false);
		}
		return;
	}

	auto goal_msg = opennav_docking_msgs::action::DockRobot::Goal();
	goal_msg.use_dock_id = false;
	goal_msg.dock_type = "dock";
	goal_msg.navigate_to_staging_pose = true;
	goal_msg.max_staging_time = kMaxStagingTimeSeconds;

	goal_msg.dock_pose.header.frame_id = "map";
	goal_msg.dock_pose.header.stamp = node_->now();
	goal_msg.dock_pose.pose.position.x = x;
	goal_msg.dock_pose.pose.position.y = y;
	goal_msg.dock_pose.pose.position.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, yaw);
	goal_msg.dock_pose.pose.orientation.x = q.x();
	goal_msg.dock_pose.pose.orientation.y = q.y();
	goal_msg.dock_pose.pose.orientation.z = q.z();
	goal_msg.dock_pose.pose.orientation.w = q.w();

	auto send_goal_options = rclcpp_action::Client<opennav_docking_msgs::action::DockRobot>::SendGoalOptions();
	send_goal_options.goal_response_callback =
		std::bind(&NinjaSimaMainDocking::dock_goal_response_callback, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&NinjaSimaMainDocking::dock_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&NinjaSimaMainDocking::dock_result_callback, this, std::placeholders::_1);

	dock_robot_client_->async_send_goal(goal_msg, send_goal_options);
	is_docking_ = true;

	RCLCPP_INFO(node_->get_logger(), "Sending docking goal %s: x=%.2f, y=%.2f, theta=%.2f",
				active_docking_name_.c_str(), x, y, yaw);
	RCLCPP_INFO(node_->get_logger(), "DockRobot goal sent: type=%s, use_dock_id=%s, navigate_to_staging_pose=%s, max_staging_time=%.1fs",
				goal_msg.dock_type.c_str(),
				goal_msg.use_dock_id ? "true" : "false",
				goal_msg.navigate_to_staging_pose ? "true" : "false",
				goal_msg.max_staging_time);
	RCLCPP_INFO(node_->get_logger(), "DockRobot goal %s dispatched; is_docking=true", active_docking_name_.c_str());
}

void NinjaSimaMainDocking::dock_goal_response_callback(
	const rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::SharedPtr & goal_handle) {
	if (!goal_handle) {
		RCLCPP_ERROR(node_->get_logger(), "DockRobot goal %s was rejected by server.", active_docking_name_.c_str());
		is_docking_ = false;
		if (result_callback_) {
			result_callback_(false);
		}
	} else {
		RCLCPP_INFO(node_->get_logger(), "DockRobot goal %s accepted by server.", active_docking_name_.c_str());
	}
}

void NinjaSimaMainDocking::dock_feedback_callback(
	rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::SharedPtr,
	const std::shared_ptr<const opennav_docking_msgs::action::DockRobot::Feedback> feedback) {
	feedback_count_++;
	last_feedback_time_ = node_->now();
	last_feedback_state_ = feedback->state;
	last_feedback_retries_ = feedback->num_retries;
	RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
		"DockRobot feedback %s: state=%u, retries=%u",
		active_docking_name_.c_str(), feedback->state, feedback->num_retries);
}

void NinjaSimaMainDocking::dock_result_callback(
	const rclcpp_action::ClientGoalHandle<opennav_docking_msgs::action::DockRobot>::WrappedResult & result) {
	is_docking_ = false;

	const bool success =
		result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success;

	if (!success) {
		const uint16_t error_code = result.result ? result.result->error_code : 999;
		RCLCPP_ERROR(node_->get_logger(), "DockRobot goal %s failed. result_code=%d, error_code=%u",
					 active_docking_name_.c_str(), static_cast<int>(result.code), error_code);
	} else {
		RCLCPP_INFO(node_->get_logger(), "DockRobot goal %s succeeded.", active_docking_name_.c_str());
	}
	RCLCPP_INFO(node_->get_logger(), "DockRobot result callback %s: success=%s, is_docking=false",
		active_docking_name_.c_str(), success ? "true" : "false");

	if (result_callback_) {
		result_callback_(success);
	}
}

std::string NinjaSimaMainDocking::debug_status() const {
	std::ostringstream stream;
	const double elapsed = is_docking_ ? (node_->now() - docking_start_time_).seconds() : 0.0;
	stream << "active=" << active_docking_name_
		   << ", elapsed=" << elapsed << "s"
		   << ", feedback_count=" << feedback_count_;

	if (feedback_count_ > 0) {
		const double feedback_age = (node_->now() - last_feedback_time_).seconds();
		stream << ", last_feedback_age=" << feedback_age << "s"
			   << ", state=" << last_feedback_state_
			   << ", retries=" << last_feedback_retries_;
	} else {
		stream << ", no_feedback_yet";
	}

	return stream.str();
}
