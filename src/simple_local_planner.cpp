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

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        path_pose_id_ = 0;
    }

    void SimpleLocalPlanner::topic_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg;

        try
        {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped target_pose = path_->poses.front();

            tf2::Vector3 cu_pos, tar_pos;
            cu_pos.setW(t.transform.rotation.w);
            cu_pos.setX(t.transform.rotation.x);
            cu_pos.setY(t.transform.rotation.y);
            cu_pos.setZ(t.transform.rotation.z);
            tar_pos.setW(target_pose.pose.orientation.w);
            tar_pos.setX(target_pose.pose.orientation.x);
            tar_pos.setY(target_pose.pose.orientation.y);
            tar_pos.setZ(target_pose.pose.orientation.z);

            auto x_vec = target_pose.pose.position.x - t.transform.translation.x;
            auto y_vec = target_pose.pose.position.y - t.transform.translation.y;
            auto delta_degree = tar_pos.getZ() - cu_pos.getZ();
            auto rotation_vec = delta_degree * (M_PI / 180.0);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = x_vec;
            cmd.linear.y = y_vec;
            cmd.angular.z = rotation_vec;

            cmd_pub->publish(cmd);
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform : %s", ex.what());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_local_planner::SimpleLocalPlanner)