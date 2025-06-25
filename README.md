Terminal 1:
cd ardupilot/ArduCopter/
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

position 30 0 0
param set WPNAV_SPEED 1000
param show WPNAV_SPEED
O/P>>GUIDED> WPNAV_SPEED      1000.0


OTHER COMMANDS 
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ardu_params_and_commands.md

_______________________________________________________________________________________________________________________________________________

to make the drone follow square path by the code 
change the terminal 4 command to 
ros2 run drone_control square 

everything is same
___________________________________________________________________________________________________________________________________________________

to launch drone in the specific world

gz sim -v4 -r /home/nagateja/ws_drone/src/drone_control/drone_launch/empty_drone.sdf

___________________________________________________________________________________________________________________________________________________
Using Launch Files

Terminal 1:
cd ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14540

Terminal 2:
ros2 launch drone_control launch_drone.py

Terminal 3:
any run file 

___________________________________________________________________________________________________________________________________________________

To source the drones to gazebo we need to run the below command
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/nagateja/ws_drone/src/drone_control/drones

for drone 1: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

for drone 2: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I1 

change I1 to anything like I2 I3 to launch different drones

to launch 2 drones: gz sim -v4 -r ~/ws_drone/src/drone_control/worlds/multi_drone_empty.sdf

to launch 9 drons of 3x3 matrix : gz sim -v4 -r ~/ws_drone/src/drone_control/worlds/matrix_drone.sdf

___________________________________________________________________________________________________________________________________________________

to add new drones 
copy drone1 and in sdf file change the name add same name to the folder.
add plus 10 to these numbers and change the below lines 
      <fdm_port_in>9052</fdm_port_in>
      <fdm_port_out>9053</fdm_port_out>
change the name tag inside config file
___________________________________________________________________________________________________________________________________________________