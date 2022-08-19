This repo is for sending position or attitude setpoints through mavros offboard mode.  

**Key subscribed topics:**  
/desired_position (posctl)  
/desired_atti (attctl)  
/mission_status (both)  

**Key published topics:**  
/mavros/setpoint_position/local (posctl)  
/mavros/setpoint_raw/attitude (attctl)  


/desired_position will allow for varying 3D position.  

/desired_att will allow for varying 3D attitude and thrust.  

/mission_status will act as an emergency feature, once activated, will send the UAV to LOITER then RTL/LAND after 3 secs. Safety feature for simulation and real-life experiments.  

TO USE:  
terminal 1: $**roslaunch px4 posix_sitl.launch or px4_sitl.launch**  
terminal 2: $**roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"** (for gazebo simulation)  
terminal 3: (Optional) ./QGroundControl.AppImage (if you need to rtl or change settings)  
terminal 4: $**python mavrosPosCtrl.py or mavrosAttCtrl.py** (make sure to be in the right directory)  
terminal 5:  
$**rostopic pub /desired_pos geometry_msgs/PoseStamped**
"header:  
    seq: 0  
    stamp: {secs: 0, nsecs: 0}  
    frame_id: ''  
pose:  
    position:  
      x: 0.0  
      y: 0.0  
      z: **-/+x.x**  
    orientation:  
      x: 0.0  
      y: 0.0  
      z: 0.0  
      w: 0.0"  
  
or  
  
$**rostopic pub /desired_atti mavros_msgs/AttitudeTarget**  
"header:  
    seq: 0  
    stamp: {secs: 0, nsecs: 0}  
    frame_id: ''  
type_mask: 0  
orientation: {x: 0.0, y: **-/+x.xx**, z: 0.0, w: 0.0}  
body_rate: {x: 0.0, y: 0.0, z: 0.0}  
thrust: **x.x**"   

terminal 6: $**rostopic pub /mission_status std_msgs/Bool True**  




