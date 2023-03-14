/**
 * @author Sai Surya Sriramoju
 * @author Dhruv Sharma
 * @author Satish Vennapu
*/

#pragma once

#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <std_msgs/msg/bool.hpp>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"


// timer
class TargetReacher : public rclcpp::Node
{
public:
/**
 * @brief default constructor
*/
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;
        this->declare_parameter<std::string>("final_destination.frame_id");

        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");

        auto aruco_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_y = this->declare_parameter<double>("aruco_target.y");

        m_bot_controller->set_goal(aruco_x, aruco_y);
        goal_sub = this->create_subscription<std_msgs::msg::Bool>("/goal_reached",10,std::bind(&TargetReacher::goal_callback, this, std::placeholders::_1));
        aruco_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));
        rotation_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel",10);

        final_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
        final_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
        
    }
    /**
     * @brief /goal_reached is true if the bot reaches the goal
    */
    void goal_callback(std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief method for retrieving the pose of the bot according to the marker ID's
    */
    void aruco_callback(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief method to rotate the bot as soon as it reaches near the marker
    */
    void rotate();

    /**
     * @brief To stop the bot
    */
    void stop();

    /**
     * @brief method to send the final_destination (origin) frame
    */
    void timer_callback();

    /**
     * @brief method to specify the goal coordinates according to the marker ID used
    */
    void go_to_goal();

private:
    // attributes

    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotation_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_sub;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub;

    double goal_x;
    double goal_y;
    std::string end_origin;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> final_broadcaster;
    
    std::shared_ptr<tf2_ros::TransformListener> final_listener;
    
    std::unique_ptr<tf2_ros::Buffer> buffer;

    

    


};