# ad_status_lamp_manager

## Overview
By switching the lighting pattern of the lamp, this node informs the transport manager of the system status.

The lighting pattern is defined as follows.
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
      <td>Blinks every second</td>
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
      <td>Completes system shutdown</td>
      <td>Keeps going off</td>
    </tr>
  </tbody>
</table>

## Input and Output
- input
  - from eve oss
    - `/autoware_state_machine/state` : State of the system.
- output
  - to tier iv oss (not autoware)
    - `/dio/dout0` : GPIO output topic. (this topic is remapping from /ad_status_lamp_out)

## Node Graph
![image](https://user-images.githubusercontent.com/33311630/172419849-07abb3c0-a1a3-45df-83a9-70a8198b13fd.png)

<details>

  <summary> plantuml </summary>

```

@startuml

rectangle "eve oss" {
  usecase "/autoware_state_machine"
  usecase "/ad_status_lamp_manager"
}

rectangle "tier iv oss" {
  usecase "/dio_ros_driver"
}
(/autoware_state_machine) -> (/ad_status_lamp_manager) : /autoware_state_machine/state

(/ad_status_lamp_manager) -> (/dio_ros_driver) : /dio/dout0

@enduml

```

</details>

## Parameter discription
This node has no parameters.
