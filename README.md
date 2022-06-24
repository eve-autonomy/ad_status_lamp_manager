# Status lamp manager for autonomous driving

## Overview
This node changes the lighting pattern of the lamp for the operation manager to check the system status.

Lighting patterns are defined as follows;
<table>
  <thead>
    <tr>
      <th scope="col">system status</th>
	    <th scope="col">lighting pattern</th>
	  </tr>
  </thead>
  <tbody>
    <tr>
      <td>System starting up</td>
      <td>Blink once every second</td>
    </tr>
    <tr>
      <td>Preparing for autonomous driving</td>
      <td>Repeat the following;<br>Blinks twice at 0.2 second intervals, and the second light off for 1.5 seconds.</td>
    </tr>
    <tr>
      <td>Autonomous driving is available</td>
      <td>Light up</td>
    </tr>
    <tr>
      <td>System shutting down</td>
      <td>Blink once every second</td>
    </tr>
    <tr>
      <td>Power off</td>
      <td>Off</td>
    </tr>
  </tbody>
</table>

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine/)
    - `/autoware_state_machine/state` : State of the system.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver/)
    - `/dio/dout0` : GPIO output topic. (this topic is remapped from `/ad_status_lamp_out`)

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/ad_status_lamp_manager/main/docs/node_graph.pu)

## Parameter description
This node has no parameters.
