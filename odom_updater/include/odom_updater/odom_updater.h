/**
 * @author Sai Surya Sriramoju
 * @author Dhruv Sharma
 * @author Satish Vennapu
*/
#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <functional>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>



class OdomBroadcaster : public rclcpp::Node
{
    public:
    /**
     * @brief Broadcaster for connecting the /robot1/base_footprint as a child of /robot1/odom
    */
        OdomBroadcaster (const std::string &node_name = "odom_updater");
    
    /**
     * @brief method for retrieving the pose of the bot
    */
        void odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief sending the transforms
    */
        void tf_data (geometry_msgs::msg::TransformStamped t_msg);

    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //!< The frame broadcaster.
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; //!< The subscriber to the robot1/odom topic.

}; //OdomBroadcaster
