#ifndef ros_param_H__
#define ros_param_H__

#include <ros/ros.h>

// Tools for retrieving ROS Param (mostly for catching errors)
#include <xmlrpcpp/XmlRpc.h>

#include <string>


namespace ros_param
{
    class loader
    {
    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle global_nh_;

    public:
        loader();
        ~loader();

        void run();
    };
    
} // namespace ros_param

#endif