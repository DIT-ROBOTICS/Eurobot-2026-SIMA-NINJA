#include <limits>

#include "nav2_behaviors/plugins/shrink.hpp"

namespace nav2_behaviors
{
    Shrink::Shrink() : TimedBehavior<ShrinkAction>(){
    }

    Shrink::~Shrink(){
        setToOriginal();
    }

    void Shrink::onConfigure()
    {
        times = 0;
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error("Failed to lock node");
        }

        radius_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            node, 
            "/global_costmap/global_costmap"
        );

        sub_costmap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 
            rclcpp::QoS(10), 
            std::bind(&Shrink::costmapCallback, this, std::placeholders::_1)
        );

        goal_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 
            rclcpp::SystemDefaultsQoS(), 
            std::bind(&Shrink::goalPoseCallback, this, std::placeholders::_1)
        );
        shrinkBack = false;
        has_inflation_layer_param = false;
        has_costmap = false;
        has_goal_pose = false;
        node->get_parameter("costmap_tolerance", costmap_tolerance);
        RCLCPP_INFO(logger_,
            "Shrink configured: costmap_tolerance=%.3f, global_frame=%s, robot_base_frame=%s",
            costmap_tolerance, global_frame_.c_str(), robot_base_frame_.c_str());

        shrinkCheck_srv = node->create_service<std_srvs::srv::SetBool>(
            "/shrink/doneShrink",
            std::bind(&Shrink::handleShrinkCheck, this, std::placeholders::_1, std::placeholders::_2)
        );

        shrinkback_pub = node->create_publisher<std_msgs::msg::Bool>(
            "/shrink/shrinkback", 
            rclcpp::SystemDefaultsQoS()
        );

        setMode_rival_client = node->create_client<std_srvs::srv::SetBool>(
            "/rival_layer/set_mode", 
            rmw_qos_profile_services_default
        );

        setMode_object_client = node->create_client<std_srvs::srv::SetBool>(
            "/object_layer/set_mode", 
            rmw_qos_profile_services_default
        );
    }

    Status Shrink::onRun(const std::shared_ptr<const ShrinkAction::Goal> command){
        unused_shrink = command->shrink_to;

        // Get the current robot pose (already done)
        const bool got_pose = nav2_util::getCurrentPose(
            robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);

        RCLCPP_INFO(logger_,
            "Shrink onRun: shrink_to=%d, got_robot_pose=%s, robot=(%.3f, %.3f), has_costmap=%s, has_goal_pose=%s, goal=(%.3f, %.3f)",
            unused_shrink,
            got_pose ? "true" : "false",
            robotPose.pose.position.x,
            robotPose.pose.position.y,
            has_costmap ? "true" : "false",
            has_goal_pose ? "true" : "false",
            goalPose.pose.position.x,
            goalPose.pose.position.y);

        return Status::SUCCEEDED;
    }

    Status Shrink::onCycleUpdate(){
        times++;
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
            "Shrink cycle: times=%d, shrinkBack=%s, has_costmap=%s, costmap_size=%ux%u, has_goal_pose=%s, robot=(%.3f, %.3f), goal=(%.3f, %.3f)",
            times,
            shrinkBack ? "true" : "false",
            has_costmap ? "true" : "false",
            costmap.info.width,
            costmap.info.height,
            has_goal_pose ? "true" : "false",
            robotPose.pose.position.x,
            robotPose.pose.position.y,
            goalPose.pose.position.x,
            goalPose.pose.position.y);
        if(times == 1){
            getOriginalParam();
        }
        else if(times == 3){
            setToShrink();
            shrinkBack = false;
            std_msgs::msg::Bool msg;
            msg.data = shrinkBack;
            shrinkback_pub->publish(msg);
        }

        const bool robot_clear = noCostInMiddle();
        const bool goal_clear = noCostAtGoal();

        if(robot_clear && goal_clear && times > 20){
            times = 0;
            shrinkBack = true;
            RCLCPP_INFO(logger_, "Shrink succeeded: robot and goal costs are below tolerance");
            tellStopToShrinkBack();
            return Status::SUCCEEDED;
        }
        else if(times > 20){
            int robot_cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
            int goal_cost = getOneGridCost(goalPose.pose.position.x, goalPose.pose.position.y);
            RCLCPP_ERROR(logger_, "shrink the inflation radius is not working");
            RCLCPP_INFO(logger_, "obstacle detected at the center of the robot: the center %f, %f; the cost: %d", robotPose.pose.position.x, robotPose.pose.position.y, robot_cost);
            RCLCPP_INFO(logger_, "obstacle detected at the center of the goal: the center %f, %f; the cost: %d", goalPose.pose.position.x, goalPose.pose.position.y, goal_cost);
            times = 0;
            shrinkBack = true;
            tellStopToShrinkBack();
            return Status::FAILED;
        }
        else return Status::RUNNING;
    }

    void Shrink::tellStopToShrinkBack(){
        std_msgs::msg::Bool msg;
        msg.data = shrinkBack;
        shrinkback_pub->publish(msg);
        RCLCPP_INFO(logger_, "tell stop to shrink back");
    }

    void Shrink::handleShrinkCheck(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if(request->data){
            RCLCPP_INFO(logger_, "Shrink check service received doneShrink=true; restoring layers");
            response->success = true;
            response->message = "getting message from the service";
            setToOriginal();
        } else {
            RCLCPP_INFO(logger_, "Shrink check service received doneShrink=false");
            response->success = true;
            response->message = "doneShrink false received";
        }
    }

    void Shrink::getOriginalParam(){
        RCLCPP_INFO(logger_, "Shrink getting original inflation parameter; service_ready=%s",
            radius_param_client->service_is_ready() ? "true" : "false");
        if(radius_param_client->service_is_ready()){
            radius_param_client->get_parameters({"inflation_layer.inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                if (result.empty() || result[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
                    has_inflation_layer_param = false;
                    RCLCPP_WARN(logger_, "inflation_layer.inflation_radius is not available; skipping inflation shrink");
                    return;
                }
                has_inflation_layer_param = true;
                original_inflation_radius = result[0].as_double();
                RCLCPP_INFO(logger_, "Original inflation radius captured: %.3f", original_inflation_radius);
            });
        } else {
            RCLCPP_ERROR(logger_, "Parameter service is not ready for /global_costmap/global_costmap");
        }
    }

    void Shrink::setToOriginal(){
        RCLCPP_INFO(logger_, "set the inflation radius to original");
        changeInflationLayer(false);
        changeRivalLayer(false);
        changeObjectLayer(false);
    }

    void Shrink::setToShrink(){
        RCLCPP_INFO(logger_, "shrink the inflation radius");
        changeInflationLayer(true);
        changeRivalLayer(true);
        changeObjectLayer(true);
    }

    void Shrink::worldToMap(double wx, double wy, int & mx, int & my){
        mx = (int)((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        my = (int)((wy - costmap.info.origin.position.y) / costmap.info.resolution);
    }

    double Shrink::getOneGridCost(double x, double y){
        int map_x, map_y;
        if (costmap.data.empty() || costmap.info.width == 0 || costmap.info.height == 0) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                "Cannot read costmap cost at (%.3f, %.3f): costmap is empty", x, y);
            return std::numeric_limits<double>::max();
        }
        worldToMap(x, y, map_x, map_y);
        if (map_x < 0 || map_y < 0 ||
            map_x >= static_cast<int>(costmap.info.width) ||
            map_y >= static_cast<int>(costmap.info.height))
        {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                "Cost query outside map: world=(%.3f, %.3f), map=(%d, %d), size=%ux%u, origin=(%.3f, %.3f), resolution=%.3f",
                x, y, map_x, map_y, costmap.info.width, costmap.info.height,
                costmap.info.origin.position.x, costmap.info.origin.position.y,
                costmap.info.resolution);
            return std::numeric_limits<double>::max();
        }
        return costmap.data[map_y * costmap.info.width + map_x];
    }

    void Shrink::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        costmap = msg;
        has_costmap = true;
        RCLCPP_INFO_ONCE(logger_,
            "Shrink received first costmap: size=%ux%u, resolution=%.3f, origin=(%.3f, %.3f), data_size=%zu",
            costmap.info.width,
            costmap.info.height,
            costmap.info.resolution,
            costmap.info.origin.position.x,
            costmap.info.origin.position.y,
            costmap.data.size());
    }

    void Shrink::goalPoseCallback(const geometry_msgs::msg::PoseStamped& msg){
        goalPose = msg;
        has_goal_pose = true;
        RCLCPP_INFO(logger_, "Shrink received goal pose from /move_base_simple/goal: frame=%s, goal=(%.3f, %.3f)",
            goalPose.header.frame_id.c_str(),
            goalPose.pose.position.x,
            goalPose.pose.position.y);
    }

    void Shrink::changeInflationLayer(bool doShrink) {
        if (!has_inflation_layer_param) {
            RCLCPP_WARN(logger_, "Skip inflation layer update because inflation radius parameter is unavailable");
            return;
        }
        double radius = doShrink ? 0.15 : original_inflation_radius;
        if (radius_param_client->service_is_ready()) {
            radius_param_client->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", radius)},
                [this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                    // Add timeout check
                    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
                        RCLCPP_ERROR(logger_, "Timeout setting inflation_radius to %f", radius);
                        return;
                    }
                    
                    // Add try-catch
                    try {
                        auto result = future.get();
                        if (result.empty()) {
                            RCLCPP_ERROR(logger_, "No result returned while setting inflation_radius to %f", radius);
                            return;
                        }
                        if (result[0].successful) {
                            RCLCPP_INFO(logger_, "Set inflation_radius successfully to %f", radius);
                        } else {
                            RCLCPP_ERROR(logger_, "Failed to set inflation_radius to %f: %s", 
                                        radius, result[0].reason.c_str());
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(logger_, "Exception setting inflation_radius: %s", e.what());
                    }
                });
        } else {
            RCLCPP_ERROR(logger_, "Service is not ready for inflation layer");
        }
        RCLCPP_INFO(logger_, "Requested inflation layer update");
    }

    void Shrink::changeRivalLayer(bool doShrink) {
        if (!setMode_rival_client->service_is_ready()) {
            RCLCPP_ERROR(logger_, "Service is not ready for rival layer");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;
        
        // Use a callback-based approach instead of spin_until_future_complete
        setMode_rival_client->async_send_request(
            request,
            [this, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger_, "Rival layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to set rival layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Exception in rival layer callback: %s", e.what());
                }
            });
        
        RCLCPP_INFO(logger_, "Requested rival layer update asynchronously");
    }

    void Shrink::changeObjectLayer(bool doShrink) {
        if (!setMode_object_client->service_is_ready()) {
            RCLCPP_ERROR(logger_, "Service is not ready for object layer");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;
        
        // Use a callback-based approach instead of spin_until_future_complete
        setMode_object_client->async_send_request(
            request,
            [this, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger_, "Object layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to set object layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Exception in object layer callback: %s", e.what());
                }
            });
        
        RCLCPP_INFO(logger_, "Requested object layer update asynchronously");
    }

    bool Shrink::noCostInMiddle(){
        int cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
        if(cost > costmap_tolerance){
            return false;
        }
        else{
            return true;
        }
    }

    bool Shrink::noCostAtGoal(){
        int cost = getOneGridCost(goalPose.pose.position.x, goalPose.pose.position.y);
        if(cost > costmap_tolerance){
            return false;
        }
        else{
            return true;
        }
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Shrink, nav2_core::Behavior)
