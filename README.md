Terminal 1:
cd cd ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14540

Terminak 2:
gz sim -v4 -r iris_runway.sdf

Terminal 3:
ros2 launch mavros apm.launch fcu_url:=udp://:14540@127.0.0.1:14557

Terminal 4:(any one)
ros2 run drone_control takeoff_node (using entire code from this cpp file)
ros2 run drone_control takeoff_function (by calling functions from hpp file and executing them)
This node sets the vechile to guided mode and arms the motors

Terminal 5:(Debug)
ros2 topic echo /mavros/state

Should get this
header:
  stamp:
    sec: 1750686173
    nanosec: 836661130
  frame_id: ''
connected: true
armed: true
guided: true
manual_input: true
mode: GUIDED
system_status: 4
___________________________________________________________________________________________________________________________________________________

to make the drone follow square path by the code 
change the terminal 4 command to 
 ros2 run drone_control square 

everything is same