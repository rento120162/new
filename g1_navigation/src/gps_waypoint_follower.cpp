// gps_waypoint_follower.cpp
//
// Humble対応:
// GPS(lat/lon) waypoint → map座標変換 → NavigateToPose送信
//
// 追加機能:
// - waypointごとに attribute を設定可能
// - 到達時に topic publish
// - yaw不要（進行方向はNav2任せ）
//
// YAML例:
//
// waypoints:
//   - latitude: 38.4239
//     longitude: 110.7852
//     attribute: "spray_on"
//
//   - latitude: 38.4241
//     longitude: 110.7855
//     attribute: "take_photo"
//
// publish topic:
//   /waypoint_attribute   std_msgs/String
//
// --------------------------------------------

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class GPSWaypointFollower : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    struct Waypoint
    {
        double latitude;
        double longitude;
        std::string attribute;
    };

    GPSWaypointFollower(const std::string & yaml_path) : Node("gps_waypoint_follower")
    {
        loadWaypoints(yaml_path);

        attribute_pub_ = this->create_publisher<std_msgs::msg::String>("/waypoint_attribute", 10);
        fromll_client_ = this->create_client<robot_localization::srv::FromLL>("/fromLL");
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        waitForServices();
        current_index_ = 0;
        sendNextWaypoint();
    }

private:

    std::vector<Waypoint> waypoints_;

    size_t current_index_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr attribute_pub_;

    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr
        fromll_client_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    void waitForServices()
    {
        while (!fromll_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(get_logger(), "Waiting for /fromLL service...");
        }

        while (!nav_client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO( get_logger(), "Waiting for navigate_to_pose...");
        }
    }

    void loadWaypoints(const std::string & yaml_path)
    {
        YAML::Node config = YAML::LoadFile(yaml_path);

        auto wps = config["waypoints"];
        for (auto wp : wps)
        {
            Waypoint w;
            w.latitude = wp["latitude"].as<double>();
            w.longitude = wp["longitude"].as<double>();
            if (wp["attribute"])
                w.attribute = wp["attribute"].as<std::string>();
            else
                w.attribute = "";

            waypoints_.push_back(w);
        }

        RCLCPP_INFO( get_logger(), "Loaded %ld waypoints", waypoints_.size());
    }

    void sendNextWaypoint()
    {
        if (current_index_ >= waypoints_.size())
        {
            RCLCPP_INFO( get_logger(), "All waypoints completed");
            return;
        }

        auto & wp = waypoints_[current_index_];

        RCLCPP_INFO( get_logger(), "Converting GPS waypoint %ld", current_index_);

        auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
        request->ll_point.latitude = wp.latitude;
        request->ll_point.longitude = wp.longitude;
        request->ll_point.altitude = 0.0;

        auto future = fromll_client_->async_send_request(request, [this](rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture result)
                {
                    handleFromLL(result);
                });
    }

    void handleFromLL(rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future)
    {
        auto response = future.get();
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = now();
        goal_pose.pose.position.x = -(response->map_point.x);
        goal_pose.pose.position.y = -(response->map_point.y);
        goal_pose.pose.position.z = 0.0;
        // yaw不要
        goal_pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(get_logger(), "Sending goal x=%.2f y=%.2f", goal_pose.pose.position.x, goal_pose.pose.position.y);

        NavigateToPose::Goal goal_msg;
        goal_msg.pose = goal_pose;
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.result_callback = std::bind(&GPSWaypointFollower::resultCallback, this, std::placeholders::_1);
        nav_client_->async_send_goal(goal_msg, options);
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            auto & wp = waypoints_[current_index_];
            RCLCPP_INFO(get_logger(), "Waypoint %ld reached", current_index_);

            if (!wp.attribute.empty())
            {
                std_msgs::msg::String msg;
                msg.data = wp.attribute;
                attribute_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "Published attribute: %s", wp.attribute.c_str());
            }

            current_index_++;
            rclcpp::sleep_for(1s);
            sendNextWaypoint();
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Navigation failed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2)
    {
        std::cout
            << "Usage:\n"
            << "gps_waypoint_follower waypoints.yaml\n";

        return 1;
    }

    auto node = std::make_shared<GPSWaypointFollower>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}