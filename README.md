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

CURRENTLY THE EXAMPLE CODE IS NOT READY. WILL BE UPDATED IN A LATER TIME.  
- @todo add loading single rosparam  
- @todo add loading list of rosparam  

## Reference

- Good reference from ROS Answer [link](https://answers.ros.org/question/266012/getparam-a-nested-stdmap/)
- A repository which has method of retrieving param [link](https://github.com/PickNikRobotics/rosparam_shortcuts)
