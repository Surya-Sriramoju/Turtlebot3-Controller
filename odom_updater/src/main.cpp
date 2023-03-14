#include <memory>

#include <odom_updater.h>

int main(int argc, char** argv)
{
    // Initializing rclcpp.
    rclcpp::init(argc, argv);

    // Starting the Node.
    rclcpp::spin(std::make_shared<OdomBroadcaster>());

    // Shutting down the node.
    rclcpp::shutdown();

}