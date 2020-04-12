rostopic pub /cmd_vel_mux/input/navi geometry_msgs/Twist "linear:
  x: 10.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" &
  
  sleep 1
  
  rostopic pub /cmd_vel_mux/input/navi geometry_msgs/Twist "linear:
  x: 0.0
  y: 10.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" &
  
  sleep 1
  
  rostopic pub /cmd_vel_mux/input/navi geometry_msgs/Twist "linear:
  x: 10.0
  y: 10.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" &
  
  sleep 1