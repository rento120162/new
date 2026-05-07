// gps_waypoint_follower.cpp
//
// GPS waypoint -> ENU(fromLL) -> Fast-LIO odom frame
//
// 前提:
// - map == odom (static transform)
// - Fast-LIO は odom -> base_link を publish
// - fromLL は ENU座標を返す
//
// 起動時に:
// - GPS原点
// - Fast-LIO yaw
// を保存してENU→odom変換を行う

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "robot_localization/srv/from_ll.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class GPSWaypointFollower : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    using GoalHandleNavigateToPose =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;

    struct Waypoint
    {
        double latitude;
        double longitude;
        std::string attribute;
    };

    GPSWaypointFollower(const std::string & yaml_path)
        : Node("gps_waypoint_follower")
    {
        loadWaypoints(yaml_path);

        declare_parameter<double>("origin_latitude", 0.0);
        declare_parameter<double>("origin_longitude", 0.0);

        origin_lat_ = get_parameter("origin_latitude").as_double();
        origin_lon_ = get_parameter("origin_longitude").as_double();

        attribute_pub_ =
            create_publisher<std_msgs::msg::String>(
                "/waypoint_attribute",
                10);

        fromll_client_ =
            create_client<robot_localization::srv::FromLL>(
                "/fromLL");

        nav_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "navigate_to_pose");

        odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/Odometry",
                10,
                std::bind(
                    &GPSWaypointFollower::odomCallback,
                    this,
                    std::placeholders::_1));

        waitForServices();

        init_timer_ =
            create_wall_timer(1s, std::bind(&GPSWaypointFollower::initializeOrigin, this));

        initializeOrigin();
    }

private:

    std::vector<Waypoint> waypoints_;

    size_t current_index_ = 0;

    bool origin_initialized_ = false;
    bool yaw_initialized_ = false;

    double origin_lat_;
    double origin_lon_;

    double origin_enu_x_;
    double origin_enu_y_;

    double origin_yaw_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        attribute_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr
        fromll_client_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr
        nav_client_;

    rclcpp::TimerBase::SharedPtr init_timer_;

    void waitForServices()
    {
        while (!fromll_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(
                get_logger(),
                "Waiting for /fromLL service...");
        }

        while (!nav_client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(
                get_logger(),
                "Waiting for navigate_to_pose...");
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
                w.attribute =
                    wp["attribute"].as<std::string>();

            waypoints_.push_back(w);
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %ld waypoints",
            waypoints_.size());
    }

    void initializeOrigin()
    {
        static bool requested = false;

        if (requested)
            return;

        requested = true;

        auto request =
            std::make_shared<
                robot_localization::srv::FromLL::Request>();

        request->ll_point.latitude = origin_lat_;
        request->ll_point.longitude = origin_lon_;
        request->ll_point.altitude = 0.0;

        fromll_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    robot_localization::srv::FromLL>::SharedFuture future)
            {
                auto result = future.get();

                origin_enu_x_ =
                    result->map_point.x;

                origin_enu_y_ =
                    result->map_point.y;

                RCLCPP_INFO(
                    get_logger(),
                    "Origin initialized x=%.2f y=%.2f",
                    origin_enu_x_,
                    origin_enu_y_);

                origin_initialized_ = true;

                if (yaw_initialized_)
                {
                    sendNextWaypoint();
                }

                init_timer_->cancel();
            });
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {   
        if (yaw_initialized_)
            return;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll, pitch, yaw;

        tf2::Matrix3x3(q).getRPY(
            roll,
            pitch,
            yaw);

        origin_yaw_ = yaw;

        yaw_initialized_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Initial yaw = %.3f",
            yaw);

        if (origin_initialized_)
        {
            sendNextWaypoint();
        }
    }


    void sendNextWaypoint()
    {
        if (!origin_initialized_ || !yaw_initialized_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Waiting initialization...");
            return;
        }
        
        if (current_index_ >= waypoints_.size())
        {
            RCLCPP_INFO(
                get_logger(),
                "All waypoints completed");

            return;
        }

        auto & wp = waypoints_[current_index_];

        auto request =
            std::make_shared<
                robot_localization::srv::FromLL::Request>();

        request->ll_point.latitude = wp.latitude;
        request->ll_point.longitude = wp.longitude;
        request->ll_point.altitude = 0.0;

        fromll_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    robot_localization::srv::FromLL>::SharedFuture future)
            {
                handleFromLL(future);
            });
    }

    void handleFromLL(
        rclcpp::Client<
            robot_localization::srv::FromLL>::SharedFuture future)
    {
        auto response = future.get();

        double enu_x = response->map_point.x;
        double enu_y = response->map_point.y;

        double dx = enu_x - origin_enu_x_;
        double dy = enu_y - origin_enu_y_;

        double c = cos(origin_yaw_);
        double s = sin(origin_yaw_);

        double odom_x =
            c * dx + s * dy;

        double odom_y =
            -s * dx + c * dy;

        geometry_msgs::msg::PoseStamped goal_pose;

        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = now();

        goal_pose.pose.position.x = odom_x;
        goal_pose.pose.position.y = odom_y;
        goal_pose.pose.position.z = 0.0;

        goal_pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(
            get_logger(),
            "Goal odom x=%.2f y=%.2f",
            odom_x,
            odom_y);

        NavigateToPose::Goal goal_msg;
        goal_msg.pose = goal_pose;

        auto options =
            rclcpp_action::Client<
                NavigateToPose>::SendGoalOptions();

        options.result_callback =
            std::bind(
                &GPSWaypointFollower::resultCallback,
                this,
                std::placeholders::_1);

        nav_client_->async_send_goal(
            goal_msg,
            options);
    }

    void resultCallback(
        const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code ==
            rclcpp_action::ResultCode::SUCCEEDED)
        {
            auto & wp = waypoints_[current_index_];

            RCLCPP_INFO(
                get_logger(),
                "Waypoint %ld reached",
                current_index_);

            if (!wp.attribute.empty())
            {
                std_msgs::msg::String msg;

                msg.data = wp.attribute;

                attribute_pub_->publish(msg);
            }

            current_index_++;

            rclcpp::sleep_for(1s);

            sendNextWaypoint();
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "Navigation failed");
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

    auto node =
        std::make_shared<GPSWaypointFollower>(
            argv[1]);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}