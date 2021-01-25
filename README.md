# ROS Param

This repository demonstrates the use of ROS param setting and retrieving from the server.

Note that the Parameter Server uses XMLRPC data types for parameter values, which include:

- 32-bit integers
- booleans
- strings
- doubles
- iso8601 dates
- lists
- base64-encoded binary data 

You can also store dictionaries (i.e. structs) on the Parameter Server, though they have special meaning. The Parameter Server represents ROS namespaces as dictionaries. For example, imagine you set the following three parameters: 

```
/gains/P = 10.0
/gains/I = 1.0
/gains/D = 0.1
```

You can either read them back separately, i.e. retrieving /gains/P would return 10.0, or you can retrieving /gains, which would return a dictionary: 

```
{ 'P': 10.0, 'I': 1.0, 'D' : 0.1 }
```

**Note**  
It is important to note that if you wish to use `ros::NodeHandle` to search for params in sever, you will need to know the namespace in advance. If not you the `searchParam` will always return false, unlike what it says it does. Therefore, you will notice that there is a `param_nh_` private member to retrieve params from server.

To load a single ROS param you may follow the following function to load it into your node. The example given below loads an boolean variable.

```cpp
// Searching for single param
bool local_var;
std::string search_var = "bool_var";
std::string full_path;
if(param_nh.searchParam(search_var, full_path))
{
    // Load param to local variable
    if(param_nh.getParam(full_path, local_var)))
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " successfully loaded " << search_var << ": " << local_var);
    }
    else
    {
        ROS_WARN_STREAM(ros::this_node::getName() << "failed to load " <<  search_var << " from ROS param server.");
    }
}
else
{
    ROS_WARN_STREAM(ros::this_node::getName() << " failed to find " << search_var << " from ROS param server.");
}
```

To load a list of ROS param, for example, a vector of double, you may follow the instructions below to load it into your node.
```cpp
std::vector<double> local_var
std::string search_var = "list_of_double";
std::string full_path_tmp;
if(param_nh.searchParam(search_var, full_path_tmp))
{
    if(param_nh.getParam(full_path_tmp, local_var))
    {
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
```

To load a list of list of ROS param, for example, a vector of vector of double. This is usually something useful for loading in robot footprint in `move_base`. Please follow the instructions below to load it into your node.
```cpp
std::vector<std::vector<double>> local_var;
std::string search_var = "list_of_list_of_double";
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
```

To load a list of dict of ROS param, for example, a std::map. This is usually something useful for loading in plugins (pluginlib). Please follow the instructions below to load it into your node.
```cpp
std::map<std::string, std::string> local_var;
std::string search_var list_of_dict = "list_of_dict";
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
```

## Reference

- Good reference from ROS Answer [link](https://answers.ros.org/question/266012/getparam-a-nested-stdmap/)
- A repository which has method of retrieving param [link](https://github.com/PickNikRobotics/rosparam_shortcuts)
- ROS wiki list of list [link](https://answers.ros.org/question/318544/retrieve-list-of-lists-from-yaml-file-parameter-server/)
