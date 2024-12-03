#ifndef SIMPLE_LOCAL_PLANNER
#define SIMPLE_LOCAL_PLANNER

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace simple_local_planner
{
    class SimpleLocalPlanner : public rclcpp::Node
    {
        public:
        explicit SimpleLocalPlanner(const rclcpp::NodeOptions&option = rclcpp::NodeOptions());

        void topic_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub;
        nav_msgs::msg::Path::SharedPtr path_;
        geometry_msgs::msg::PoseStamped::SharedPtr t;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    };
}

#endif