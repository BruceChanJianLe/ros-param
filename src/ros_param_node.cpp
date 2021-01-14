#include "ros-param/ros_param.hpp"

const std::string ROSNodeName {"ros_param_node"};

int main(int argc, char ** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, ROSNodeName);

    // Instantiate ros_param:: loader
    ros_param::loader node;

    // Run feature
    node.run();

    // Exit successfully
    return 0;
}