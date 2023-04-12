# Senior-Design-2023
FAMU-FSU COE Senior Design Team 504 Code Workspace

## Start up instructions

### 1. Power on the Jetson

### 2. Log in
```
Username: nvidia
Password: nvidia
```
  
### 3. Connect to WiFi *(Optional)*
You'll need to connect to WiFi to download updates. The school WiFi is too hard to connect to, so instead, you'll need to set up a mobile hotspot on your laptop and connect to it on the Jetson. Once connected, you might want to check that the date on the Jetson is correct by going into Settings > Date & Time and disable, then reenable the automatic date setting.
  
### 4. Run the project
To run the project, open Terminal (Alt+Shift+T) and do the following command:
```
./run_project.sh
```
This will start the docker container with the ROS2 environment pre-configured.

***Note:** Make sure the RealSense camera and Teensy are connected prior to starting the container, otherwise their ports won't be mapped and will be inaccessible.*

### 5. Test the Camera and Teensy
To see if the Realsense camera is connected properly, run the following command:
```
realsense-viewer
```
This will run a the RealSense application. You should see 'RealSense device found' on the startup, and then be able to enable the depth/RGB video feeds in the application. Also, make sure it says connected by USB 3.2 on the top of the left panel.

To see if the Teensy is connected properly, run the following command:
```
ls /dev | grep tty
```
This will show you the `tty` devices connected to the Jetson that are ported into the ROS2 container. Specifically, look for `ttyAMC0` which is the name for the Teensy device. 

### 6. Launch project ROS2 nodes
To run the project nodes, you'll first have to source the ROS2 environment with the following command:
```
source install/setup.bash
```
Then, to launch the project run:
```
ros2 launch sd504 sd504.launch.py
```
This should start up all the ROS nodes and open up a RViz GUI for visualization. The camera might take a couple seconds to start up, as well as the map.

Once the map appears on RViz, the planner should start creating paths and sending the commands to the motor controller, which will send those commands to the Teensy.

### 7. Restarting the nodes
Press Ctrl+C in the terminal to end all ROS2 nodes. This should also send a command to the Teensy to set the steering angle to 0 and stop the drive motors. 

Then, to restart, just run `ros2 launch sd504 sd504.launch.py` again.

### Debugging and Additional Info
- If the camera or Teensy is not connecting, make sure the cables are plugged in correctly. You might want to try wiggling them around a little bit.
- If you make changes to the source code in any packages, you'll have to recompile the code by running `colcon build --packages-select sd504_nav_planner sd504_motor_controller --cmake-args -DCMAKE_BUILD_TYPE=Release`
- If the drive motors are stuck in reverse even after the nodes are closed, re-run the nodes, let them send a few motor commands, and then close them again. This should fix that issue.
