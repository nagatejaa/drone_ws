#Drone Simulation & Control with ArduPilot, MAVROS, and Gazebo Harmonic

## To launch drones and connect MAVROS follow this

### Terminal 1:
```bash
cd ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14540
```
### Terminal 2:
```bash
gz sim -v4 -r iris_runway.sdf
```
### Terminal 3:
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

### Terminal 4:(any one)
#### Using entire code from this cpp file
```bash
ros2 run drone_control takeoff_node 
```

#### by calling functions from hpp file and executing them
```bash
ros2 run drone_control takeoff_function
```
This node sets the vechile to guided mode and arms the motors

### Terminal 5:(Debug)
```bash
ros2 topic echo /mavros/state
```

Should get this
```bash
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
```

## Commands to control the drone
```bash
position 30 0 0
param set WPNAV_SPEED 1000
param show WPNAV_SPEED
O/P>>GUIDED> WPNAV_SPEED      1000.0
```

If you want to try more commands, follow this [link](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ardu_params_and_commands.md).

_______________________________________________________________________________________________________________________________________________

### To make the drone follow square path by the code 
#### Change the terminal 4 command to

```bash
ros2 run drone_control square 
```
___________________________________________________________________________________________________________________________________________________

## To launch drone in the specific world
```bash
gz sim -v4 -r /home/nagateja/ws_drone/src/drone_control/drone_launch/empty_drone.sdf
```
___________________________________________________________________________________________________________________________________________________
## To launch using Launch Files

### Terminal 1:
```bash
cd ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14540
```
### Terminal 2:
```bash
ros2 launch drone_control launch_mavros_drone.py
```
### Terminal 3:
```bash
any run file 
```
___________________________________________________________________________________________________________________________________________________

## To source the drones to gazebo we need to run the below command
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/nagateja/ws_drone/src/drone_control/drones
```
### for drone 1: 
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0
```
### for drone 2:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I1 
```
change I1 to anything like I2 I3 to launch different drones

### To launch 2 drones: 
```bash
gz sim -v4 -r ~/ws_drone/src/drone_control/worlds/multi_drone_empty.sdf
```
### To launch 9 drons of 3x3 matrix : 
```bash
gz sim -v4 -r ~/ws_drone/src/drone_control/worlds/matrix_drone.sdf
```
___________________________________________________________________________________________________________________________________________________

## To add new drones 
Copy drone1 and in sdf file change the name add same name to the folder.
add plus 10 to these numbers and change the below lines 
```bash
      <fdm_port_in>9052</fdm_port_in>
      <fdm_port_out>9053</fdm_port_out>
```
change the name tag inside config file
___________________________________________________________________________________________________________________________________________________
## To launch multiple drone in different terminals 

### Terminal-1
```bash
gz sim -v4 -r ~/ws_drone/src/drone_control/worlds/impact_site.sdf
```
### Terminal-2
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/nagateja/ws_drone/src/drone_control/models
cd ws_drone/src/drone_control/scripts/
python3 launch_drones.py 
```
___________________________________________________________________________________________________________________________________________________

## To make the export change permanent follow the commands

```bash
nano ~/.bashrc
```
add this code at the bottom

```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export GAZEBO_MODEL_PATH=$HOME/ws_drone/src/drone_control/models
export GAZEBO_RESOURCE_PATH=$HOME/ws_drone/src/drone_control/worlds
```
