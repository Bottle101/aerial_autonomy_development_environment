# Aerial Autonomy Development Environment

The environment is meant for leveraging system development and robot deployment for autonomous ground and aerial navigation. Integrating Gazebo and Unity simulators and containing autonomous navigation modules such as collision avoidance, waypoint following, and assistive teleoperation, users can develop autonomous navigation systems and later on deploy to real flying robots with minor sim-to-real gap.

# 7-Minutes Quick Start

The repository has been tested in Ubuntu 20.04 with [ROS Noetic](http://wiki.ros.org/noetic/Installation).  Install dependencies with command below.

```bash
sudo apt install libusb-dev python-yaml python-is-python3 # may need 'sudo apt update' first
git clone https://github.com/Bottle101/aerial_autonomy_development_environment.git

# in a terminal, go to the folder and compile.
cd aerial_autonomy_development_environment
unzip ./src/local_planner/paths/path_files.zip -d ./src/local_planner/paths
catkin_make
```

Download `'urban_city'` of our [Unity environment models](https://drive.google.com/drive/folders/1bmxdT6Oxzt0_0tohye2br7gqTnkMaq20?usp=share_link) (have not uploaded) and unzip the files to the `'src/vehicle_simulator/mesh'` folder. 

```bash
# launch
./system_bring_up.sh
```

Now, you can send a waypoint by clicking the 'Waypoint' button in RVIZ and then clicking a point to set the waypoint. Otherwise you can conduct assistive teleoperation by moving the virtual joystick on the right, hold the left-button of the mouse to control the direction, scroll up or down the mouse wheel to adjust the altitude. The drone will automatically avoid obstacles during navigation.

**(Optional)** If you want to use **Gazebo**, please do the following steps:

```bash
# run a script to download environment models for Gazebo
./src/vehicle_simulator/mesh/download_environments.sh

# launch
roslaunch vehicle_simulator system_gazebo.launch
```

## Change Environments

### Unity:

Change the **Line 4** in `system_unity.launch`

```bash
  <arg name="map_name" default="SCENE_TO_USE"/>
```

### Gazebo:

Change the **Line 3** in `system_gazebo.launch`

```bash
  <arg name="map_name" default="SCENE_TO_USE"/>
```