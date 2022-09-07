# ROS Bridge Message Forwarder

A client ROS node for the [ROS Bridge](http://wiki.ros.org/rosbridge_suite).

Use this node whenever you need ROS nodes located in separated ROS clusters to communicate with each other and don't feel to alter the code of your application.

Make sure one ROS cluster has a ROS Bridge instance.
Run an instance of this node in the other cluster.

## Input

This node takes as input a YAML configuration file.

``` YAML
---
remote_topics:
  -
    topic: '/a'
    type: 'std_msgs/String'

local_topics:
  -
    topic: '/b'
    type: 'std_msgs/String'
```

Each message publish on a declared _remote topic_ will be republished locally on a ROS topic with the same name.

Each message published on a declared _local topic_ will be republished in the remote ROS cluster on a ROS topic with the same name.

## Launch

The node can be configured using the following arguments:

- ```rosbridge_host```: the IP address of the remote ROS Bridge server. Default value is ```127.0.0.1```;

- ```rosbridge_port```: the network port to connect. Default value is ```9090```;

- ```topics_file```: path for the YAML configuration file (REQUIRED).

To launch this node execute the following command:

```bash
roslaunch msg_forwarder run.launch rosbridge_host:=<REMOTE_ROSBRIDGE_IP_ADDR> rosbridge_port:=<REMOTE_ROSBRIDGE_IP_PORT> topics_file:=<CONFIG_FILE_PATH>
```