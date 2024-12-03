#include "simple_local_planner/simple_local_planner.hpp"

namespace simple_local_planner
{
    SimpleLocalPlanner::SimpleLocalPlanner(const rclcpp::NodeOptions& option) : Node("SimpleLocalPlanner", option)
    {
        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/path",
            0,
            std::bind(&SimpleLocalPlanner::topic_callback, this, _1)
        );

        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 0);

        current_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current",
            0,
            std::bind(&SimpleLocalPlanner::pose_callback, this, _1)
        );

        t = nullptr;
    }

    void SimpleLocalPlanner::topic_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg;

        if(t != nullptr)
        {
            geometry_msgs::msg::PoseStamped target_pose = path_->poses.front();

            tf2::Vector3 cu_pos, tar_pos;
            cu_pos.setW(t->pose.orientation.w);
            cu_pos.setX(t->pose.orientation.x);
            cu_pos.setY(t->pose.orientation.y);
            cu_pos.setZ(t->pose.orientation.z);
            tar_pos.setW(target_pose.pose.orientation.w);
            tar_pos.setX(target_pose.pose.orientation.x);
            tar_pos.setY(target_pose.pose.orientation.y);
            tar_pos.setZ(target_pose.pose.orientation.z);

            RCLCPP_INFO(this->get_logger(), "current:%.1lf, target:%.1lf", t->pose.position.x, target_pose.pose.position.x);

            auto x_vec = target_pose.pose.position.x - t->pose.position.x;
            auto y_vec = target_pose.pose.position.y - t->pose.position.y;
            auto delta_degree = tar_pos.getZ() - cu_pos.getZ();
            auto rotation_vec = delta_degree * (M_PI / 180.0);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = x_vec;
            cmd.linear.y = y_vec;
            cmd.angular.z = rotation_vec;

            cmd_pub->publish(cmd);
        }
    }

    void SimpleLocalPlanner::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        t = msg;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_local_planner::SimpleLocalPlanner)