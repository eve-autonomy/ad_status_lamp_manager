# Status lamp manager for autonomous driving

## Overview
By switching the lighting pattern of the lamp, this node informs the transport manager of the system status.

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
      <td>During system startup</td>
      <td>Blink once every second</td>
    </tr>
    <tr>
      <td>Preparing for autonomous driving</td>
      <td>Flashes twice quickly on a regular basis</td>
    </tr>
    <tr>
      <td>Autonomous driving is available</td>
      <td>Keeps lighting</td>
    </tr>
    <tr>
      <td>During system shutdown</td>
      <td>Blinks every second</td>
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
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/ad_status_lamp_manager/docs/node_graph.pu)

## Parameter description
This node has no parameters.
