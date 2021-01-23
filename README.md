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

CURRENTLY THE EXAMPLE CODE IS NOT READY. WILL BE UPDATED IN A LATER TIME.  
- @todo add loading single rosparam  
- @todo add loading list of rosparam  

**Note**  
It is important to note that if you wish to use `ros::NodeHandle` to search for params in sever, you will need to know the namespace in advance. If not you the `searchParam` will always return false, unlike what it says it does. Therefore, you will notice that there is a `param_nh_` private member to retrieve params from server.

## Reference

- Good reference from ROS Answer [link](https://answers.ros.org/question/266012/getparam-a-nested-stdmap/)
- A repository which has method of retrieving param [link](https://github.com/PickNikRobotics/rosparam_shortcuts)
