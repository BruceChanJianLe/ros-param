#include "ros-param/ros_param.hpp"


namespace ros_param
{
    loader::loader()
    :   private_nh_(ros::NodeHandle("~")),
        global_nh_(ros::NodeHandle()),
        bool_var_(false),
        int_var_(0),
        double_var_(0.0),
        string_var_("")
    {
        // Initializing all local params from ROS server
        singleParamLoad(global_nh_, "int_var", int_var_);
    }


    loader::~loader()
    {
    }


    /**
     * Search is the parameter exist and return the full namespace
     * @param global_nh reference to global namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param full_path full path of variable if found
     * @return true if successful, false otherwise
     */
    bool loader::paramSearch(
        ros::NodeHandle & global_nh,
        const std::string search_var,
        std::string & full_path
    )
    {
        bool isok = false;
        if(global_nh.searchParam(search_var, full_path))
        {
            isok = true;
        }
        else
        {
            ROS_INFO_STREAM(full_path);
            return isok;
        }
        return isok;
    }


    /**
     * Load param to local variable
     * @param global_nh reference to global namespace node handle
     * @param full_path full path to obtain the variable in ROS server
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    template <typename T>
    bool loader::paramLoad(
        ros::NodeHandle & global_nh,
        std::string & full_path,
        T & local_var
    )
    {
        bool isok = false;
        XmlRpc::XmlRpcValue value;
        if(global_nh.getParam(full_path, local_var))
        {
            isok = true;
        }
        else
        {
            return isok;
        }
        return isok;
    }


    /**
     * 
     * 
     */
    template <typename T>
    bool loader::singleParamLoad(
            ros::NodeHandle & global_nh,
            const std::string search_var,
            T & local_var
    )
    {
        bool isok = false;
        // Searching for single param
        std::string full_path_tmp;
        if(paramSearch(global_nh, search_var, full_path_tmp))
        {
            // Load param to local variable
            if(paramLoad<T>(global_nh, full_path_tmp, local_var))
            {
                isok = true;
                ROS_INFO_STREAM(ros::this_node::getName() << " successfully loaded " << search_var << ": " << local_var);
            }
            else
            {
                ROS_WARN_STREAM(ros::this_node::getName() << "failed to load " <<  search_var << " from ROS param server.");
                return isok;
            }
            
        }
        else
        {
            ROS_WARN_STREAM(ros::this_node::getName() << " failed to find " << search_var << " from ROS param server.");
            return isok;
        }
        return isok;
    }


    void loader::run()
    {
        ;
    }
} // namespace ros_param
