#include <odom_updater.h>
#include <rclcpp/rclcpp.hpp>

OdomBroadcaster::OdomBroadcaster(const std::string &node_name)
    : Node(node_name)
{
    // Setting up the tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Setting up the subscriber
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("robot1/odom", 10, std::bind(&OdomBroadcaster::odom_callback, this, std::placeholders::_1));
}

void OdomBroadcaster::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // store present message
    geometry_msgs::msg::TransformStamped current_msg; 

    current_msg.header.stamp = this->get_clock()->now();
    current_msg.header.frame_id = "robot1/odom";
    current_msg.child_frame_id = "robot1/base_footprint";

    // Storing the pose
    current_msg.transform.translation.x = msg->pose.pose.position.x;
    current_msg.transform.translation.y = msg->pose.pose.position.y;
    current_msg.transform.translation.z = msg->pose.pose.position.z;

    // Storing the orientation
    current_msg.transform.rotation.w = msg->pose.pose.orientation.w;
    current_msg.transform.rotation.x = msg->pose.pose.orientation.x;
    current_msg.transform.rotation.y = msg->pose.pose.orientation.y;
    current_msg.transform.rotation.z = msg->pose.pose.orientation.z;

    // Sending the transform
    tf_data(current_msg);
}

void OdomBroadcaster::tf_data(geometry_msgs::msg::TransformStamped t_msg)
{
    tf_broadcaster_->sendTransform(t_msg);
}
