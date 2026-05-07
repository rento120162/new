#include <chrono>
#include <g1/g1_loco_client.hpp>
#include <iostream>
#include <map>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common/ut_errror.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "std_msgs/msg/bool.hpp"


using namespace std::placeholders;

class CmdVelToSportRequest : public rclcpp::Node
{
public:
    CmdVelToSportRequest() : Node("cmd_vel_to_sport_request"), client_(this)
    {
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToSportRequest::cmdVelCallback, this, _1));

        emergency_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_cmd", 10, std::bind(&CmdVelToSportRequest::emergencyCallback, this, _1));
        
        //start_mode
        std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        RCLCPP_INFO(this->get_logger(), "Trying Start...");
        int ret = client_.Start();

        if (ret == 0) {
          RCLCPP_INFO(this->get_logger(), "Start SUCCESS");

          client_.StandUp();
          RCLCPP_INFO(this->get_logger(), "StandUp sent");

          started_ = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Start FAILED");
        }
        }).detach();
    }

private:
    bool started_ = false;
    bool emergency_flag_ = false;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
    {
        if (!emergency_flag_) {
            
            //RCLCPP_INFO(this->get_logger(), "Received cmd_vel: LinearX=%f, LinearY=%f, AngularZ=%f",
            //            cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);

            client_.Move(cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
        } else {
            client_.BalanceStand();
        }
    }

    void emergencyCallback(const std_msgs::msg::Bool::SharedPtr emergency_msg)
    {
        emergency_flag_ = emergency_msg->data;
    }

    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_cmd_subscriber_;
    unitree::robot::g1::LocoClient client_;
    

    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToSportRequest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
