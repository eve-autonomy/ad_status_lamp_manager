# Status lamp manager for autonomous driving

## Overview
This node changes the lighting pattern of the lamp for the operation manager to check the system status.

Lighting patterns are defined as follows;

|system status|lighting pattern|
|:------------|:---------------|
|System starting up|Blink once every second|
|Preparing for autonomous driving|Repeat the following;<br>Blinks twice at 0.2 second intervals, and the second light off for 1.5 seconds.|
|Autonomous driving is available|Light up|
|System shutting down|Blink once every second|
|Power off|Off|

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine/)
    - `/autoware_state_machine/state` \[[autoware_state_machine_msgs/msg/StateMachine](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateMachine.msg)\]:<br>State of the system.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver/)
    - `/dio/dout0` \[[dio_ros_driver/msg/DIOPort](https://github.com/tier4/dio_ros_driver/blob/develop/ros2/msg/DIOPort.msg)\]:<br>GPIO output topic. (this topic is remapped from `/ad_status_lamp_out`)

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/ad_status_lamp_manager/main/docs/node_graph.pu)

## Parameter description
This node has no parameters.
