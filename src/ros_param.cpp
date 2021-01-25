#include "ros-param/ros_param.hpp"


namespace ros_param
{
    loader::loader()
    :   private_nh_(ros::NodeHandle("~")),
        global_nh_(ros::NodeHandle()),
        param_nh_(ros::NodeHandle("main_namespace/sub_namespace")),
        bool_var_(false),
        int_var_(0),
        double_var_(0.0),
        string_var_(""),
        list_of_double_({}),
        list_of_list_of_double_({}),
        another_list_of_double_({}),
        another_list_of_list_of_double_({}),
        list_of_dict_({})
    {
        // Initializing all local params from ROS server

        // Obstain single param
        singleParamLoad(param_nh_, "bool_var", bool_var_);
        singleParamLoad(param_nh_, "int_var", int_var_);
        singleParamLoad(param_nh_, "double_var", double_var_);
        singleParamLoad(param_nh_, "string_var", string_var_);

        // Obtain dict (singleDictParamLoad is able to printout std::map)
        std::vector<std::string> dict_name {"name", "type"};
        singleDictParamLoad(param_nh_, "dict_var", dict_name, dict_var_);

        // Obtain list of double
        listParamLoad(param_nh_, "list_of_double", list_of_double_);
        listParamLoad(param_nh_, "another_list_of_double", another_list_of_double_);

        // Obtain list of list of double
        listOfListParamLoad(param_nh_, "list_of_list_of_double", list_of_list_of_double_);
        listOfListParamLoad(param_nh_, "another_list_of_list_of_double", another_list_of_list_of_double_);

        // Obtain list of dict
        listOfDictParamLoad(param_nh_, "list_of_dict", dict_name, list_of_dict_);
    }


    loader::~loader()
    {
    }


    /**
     * Search is the parameter exist and return the full namespace
     * @param param_nh reference to param namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param full_path full path of variable if found
     * @return true if successful, false otherwise
     */
    bool loader::paramSearch(
        ros::NodeHandle & param_nh,
        const std::string search_var,
        std::string & full_path
    )
    {
        bool isok = false;
        if(param_nh.searchParam(search_var, full_path))
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
     * @param param_nh reference to param namespace node handle
     * @param full_path full path to obtain the variable in ROS server
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    template <typename T>
    bool loader::paramLoad(
        ros::NodeHandle & param_nh,
        const std::string & full_path,
        T & local_var
    )
    {
        bool isok = false;
        if(param_nh.getParam(full_path, local_var))
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
     * Load single ros param to local variable
     * @param param_nh reference to param namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    template <typename T>
    bool loader::singleParamLoad(
            ros::NodeHandle & param_nh,
            const std::string search_var,
            T & local_var
    )
    {
        bool isok = false;
        // Searching for single param
        std::string full_path_tmp;
        if(paramSearch(param_nh, search_var, full_path_tmp))
        {
            // Load param to local variable
            if(paramLoad<T>(param_nh, full_path_tmp, local_var))
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


    /**
     * Load single ros param to local variable
     * @param param_nh reference to param namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param dict_name a vector of string to loop through to obtain the set of dictionary (map)
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    template <typename T1, typename T2>
    bool loader::singleDictParamLoad(
        ros::NodeHandle & param_nh,
        const std::string search_var,
        const std::vector<std::string> & dict_name,
        std::map<T1, T2> & local_var
    )
    {
        bool isok = false;
        // Searching for dict param
        std::string full_path_tmp;
        if(paramSearch(param_nh, search_var, full_path_tmp))
        {
            // Load param to local variable
            if(paramLoad(param_nh, full_path_tmp, local_var))
            {
                isok = true;
                ROS_INFO_STREAM(ros::this_node::getName() << " successfully loaded " << search_var);
                for(auto single_dict_name : dict_name)
                {
                    ROS_INFO_STREAM(ros::this_node::getName() << " map with " << single_dict_name << " has " << local_var[single_dict_name]);
                }
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


    /**
     * Loading a list of ROS param
     * @param param_nh reference to param namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    template <typename T>
    bool loader::listParamLoad(
        ros::NodeHandle & param_nh,
        const std::string search_var,
        T & local_var
    )
    {
        bool isok = false;
        // Variable full path
        std::string full_path_tmp;
        if(param_nh.searchParam(search_var, full_path_tmp))
        {
            if(param_nh.getParam(full_path_tmp, local_var))
            {
                isok = true;
                ROS_INFO_STREAM(ros::this_node::getName() << " successfully loaded " << search_var);
                for(auto single_local_var : local_var)
                {
                    ROS_INFO_STREAM(ros::this_node::getName() << " vector element: " << single_local_var);
                }
            }
            else
            {
                ROS_WARN_STREAM(ros::this_node::getName() << " failed to load " << search_var << " from ROS param server.");
            }
        }
        else
        {
            ROS_WARN_STREAM(ros::this_node::getName() << " failed to find " << search_var << " from ROS param server.");
        }
        return isok;
    }


    /**
     * Loading a list of ROS param
     * @param param_nh reference to param namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    template <typename T>
    bool loader::listOfListParamLoad(
        ros::NodeHandle & param_nh,
        const std::string search_var,
        std::vector<std::vector<T>> & local_var
    )
    {
        bool isok = false;
        std::string full_path_tmp;
        XmlRpc::XmlRpcValue xmlrpc_value;
        if(param_nh.searchParam(search_var, full_path_tmp))
        {
            if(param_nh.getParam(full_path_tmp, xmlrpc_value))
            {
                // Validate if it is TypeArray
                if(xmlrpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    // Clear vector of vector
                    local_var.clear();
                    for(int i = 0; i < xmlrpc_value.size(); ++i)
                    {
                        if(xmlrpc_value[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
                        {
                            std::vector<T> vec_tmp;
                            for(int j = 0; j < xmlrpc_value[i].size(); ++j)
                            {
                                vec_tmp.emplace_back((T)xmlrpc_value[i][j]);
                            }
                            local_var.emplace_back(vec_tmp);
                        }
                        else
                        {
                            ROS_WARN_STREAM(ros::this_node::getName() << " expected to be TypeArray (second level).");
                        }
                    }
                    isok = true;

                    ROS_INFO_STREAM(ros::this_node::getName() << " successfully loaded " << search_var);
                    for(auto vec : local_var)
                    {
                        for(auto element : vec)
                        {
                            ROS_INFO_STREAM(ros::this_node::getName() << " vector of vector element: " << element);
                        }
                        ROS_INFO_STREAM(ros::this_node::getName() << " NEXT");
                    }
                }
                else
                {
                    ROS_WARN_STREAM(ros::this_node::getName() << " expected to be TypeArray (first level).");
                }
            }
            else
            {
                ROS_WARN_STREAM(ros::this_node::getName() << " failed to load " << search_var << " from ROS param server.");
            }
        }
        else
        {
            ROS_WARN_STREAM(ros::this_node::getName() << " failed to find " << search_var << " from ROS param sever.");
        }
        return isok;
    }


    /**
     * Loading a list of dict from ROS param (most used for loading plugins)
     * @param param_nh reference to param namespace node handle
     * @param search_var variable to be searched in the ROS server
     * @param local_var reference to local variable to be passed to
     * @return true if successful, false otherwise
     */
    bool loader::listOfDictParamLoad(
        ros::NodeHandle & param_nh,
        const std::string search_var,
        const std::vector<std::string> & dict_name,
        std::map<std::string, std::string> & local_var
    )
    {
        bool isok = false;
        std::string full_path_tmp;
        XmlRpc::XmlRpcValue xmlrpc_value;
        if(param_nh.searchParam(search_var, full_path_tmp))
        {
            if(param_nh.getParam(full_path_tmp, xmlrpc_value))
            {
                // Validate if it is TypeArray
                if(xmlrpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    // Clear local variable
                    local_var.clear();
                    for(int i = 0; i < xmlrpc_value.size(); ++i)
                    {
                        if(xmlrpc_value[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                        {
                            // Insert into local_var
                            local_var.insert(std::make_pair(xmlrpc_value[i][dict_name.at(0)], xmlrpc_value[i][dict_name.at(1)]));
                        }
                        else
                        {
                            ROS_WARN_STREAM(ros::this_node::getName() << " expected to be TypeStruct (second level).");
                        }
                    }
                    isok = true;

                    ROS_INFO_STREAM(ros::this_node::getName() << " successfully loaded " << search_var);
                    for(auto itr = local_var.begin(); itr != local_var.end(); ++itr)
                    {
                        ROS_INFO_STREAM(ros::this_node::getName() << " list of map element: key: " << itr->first << ", value: " << itr->second);
                    }
                }
                else
                {
                    ROS_WARN_STREAM(ros::this_node::getName() << " expected to be TypeArray (fisrt level).");
                }
            }
            else
            {
                ROS_WARN_STREAM(ros::this_node::getName() << " failed to load " << search_var << " from ROS param server.");
            }
        }
        else
        {
            ROS_WARN_STREAM(ros::this_node::getName() << " failed to find " << search_var << " from ROS param server.");
        }
    }


    void loader::run()
    {
        ;
    }
} // namespace ros_param
