#ifndef SIMPLE_LOCAL_PLANNER
#define SIMPLE_LOCAL_PLANNER

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

        private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        nav_msgs::msg::Path::SharedPtr path_;
        uint8_t path_pose_id_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}

#endif