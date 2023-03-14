#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include <rclcpp/logging.hpp>

//to rotate the bot once it is near the aruco marker
void TargetReacher::rotate()
{
    geometry_msgs::msg::Twist T;
    T.linear.x = 0;
    T.linear.y = 0;
    T.linear.z = 0;
    T.angular.x = 0;
    T.angular.y = 0;
    T.angular.z = 0.2;
    rotation_pub->publish(T);
}

//to stop the bot
void TargetReacher::stop()
{
    geometry_msgs::msg::Twist T;
    T.linear.x = 0;
    T.linear.y = 0;
    T.linear.z = 0;
    T.angular.x = 0;
    T.angular.y = 0;
    T.angular.z = 0.0;
    rotation_pub->publish(T);
}

//True if the robot reaches the goal
void TargetReacher::goal_callback(std_msgs::msg::Bool::SharedPtr msg)
{
   if(msg->data){
        rotate();
   }
}

//Updating the goal according to the marker Id
void TargetReacher::aruco_callback(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    switch(msg->marker_ids[0])
    {
        case 0:
        {
        // stop();
        rclcpp::Parameter x_0 = this->get_parameter("final_destination.aruco_0.x");
        rclcpp::Parameter y_0 = this->get_parameter("final_destination.aruco_0.y");
        goal_x = x_0.as_double();
        goal_y = y_0.as_double();
        }
        break;

        case 1:
        {
        // stop();
        rclcpp::Parameter x_1 = this->get_parameter("final_destination.aruco_1.x");
        rclcpp::Parameter y_1 = this->get_parameter("final_destination.aruco_1.y");
        goal_x = x_1.as_double();
        goal_y = y_1.as_double();
        }
        break;

        case 2:
        {
        // stop();
        rclcpp::Parameter x_2 = this->get_parameter("final_destination.aruco_2.x");
        rclcpp::Parameter y_2 = this->get_parameter("final_destination.aruco_2.y");
        goal_x = x_2.as_double();
        goal_y = y_2.as_double();
        }
        break;

        case 3:
        {
        // stop();
        rclcpp::Parameter x_3 = this->get_parameter("final_destination.aruco_3.x");
        rclcpp::Parameter y_3 = this->get_parameter("final_destination.aruco_3.y");
        goal_x = x_3.as_double();
        goal_y = y_3.as_double();
        }
        break; 

    }

    for(int i=0;i<500;i++)
    {
        timer_callback();
    }
    go_to_goal();
}

//To update the frames for the final_destination
void TargetReacher::timer_callback()
{
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    
    rclcpp::Parameter end_goal = this->get_parameter("final_destination.frame_id");
    end_origin = end_goal.as_string();
    
    tf.header.frame_id = end_origin;
    tf.child_frame_id = "final_destination";
    
    tf.transform.translation.x = goal_x;
    tf.transform.translation.y = goal_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    final_broadcaster->sendTransform(tf);
    
}

//for specifying the goal coordiantes
void TargetReacher::go_to_goal()
{
    geometry_msgs::msg::TransformStamped tf;
    
    tf = buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
    
    m_bot_controller->set_goal(tf.transform.translation.x, tf.transform.translation.y);
}