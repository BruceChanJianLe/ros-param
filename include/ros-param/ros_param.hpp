#ifndef ros_param_H__
#define ros_param_H__

#include <ros/ros.h>

// Tools for retrieving ROS Param (mostly for catching errors)
#include <xmlrpcpp/XmlRpc.h>

#include <string>
#include <map>
#include <vector>


namespace ros_param
{
    class loader
    {
    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle global_nh_;
        ros::NodeHandle param_nh_;

        // Local variables
        bool bool_var_;
        int int_var_;
        double double_var_;
        std::string string_var_;
        std::map<std::string, std::string> dict_var_;

        // List variables
        std::vector<double> list_of_double_;
        std::vector<std::vector<double>> list_of_list_of_double_;

        // Another list variable
        std::vector<double> another_list_of_double_;
        std::vector<std::vector<double>> another_list_of_list_of_double_;

        // List of dictionary
        std::vector<std::map<std::string, std::string>> list_of_dict_;

        template <typename T>
        bool singleParamLoad(
            ros::NodeHandle &,
            const std::string,
            T & local_var
        );

        bool paramSearch(
            ros::NodeHandle &,
            const std::string,
            std::string &
        );

        template <typename T>
        bool paramLoad(
            ros::NodeHandle &,
            const std::string &,
            T & local_var
        );

        template <typename T1, typename T2>
        bool singleDictParamLoad(
            ros::NodeHandle &,
            const std::string,
            const std::vector<std::string> &,
            std::map<T1, T2> &
            );

        template <typename T>
        bool listParamLoad(
            ros::NodeHandle &,
            const std::string,
            std::vector<T> &
        );

    public:
        loader();
        ~loader();

        void run();
    };
    
} // namespace ros_param

#endif