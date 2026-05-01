#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OdomFrameFixer : public rclcpp::Node
{
public:
    OdomFrameFixer()
    : Node("odom_frame_fixer")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry",10,std::bind(&OdomFrameFixer::odomcallback, this, std::placeholders::_1));
        global_pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/Laser_map",10,std::bind(&OdomFrameFixer::globalpc2callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lio_odom_fixed",10);
        global_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map_fix",10);

        RCLCPP_INFO(this->get_logger(), "odom_frame_fixer started");
    }

private:
    void odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto odom = *msg;

        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom_pub_->publish(odom);
    }

    void globalpc2callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto pc2 = *msg;

        pc2.header.frame_id = "odom";

        global_pc2_pub_->publish(pc2);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc2_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc2_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdomFrameFixer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}