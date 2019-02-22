# Jackal Exploration
**Michael Wiznitzer**

Northwestern University Winter Project


## Introduction
####  Objective
The goal of this project is for [Jackal](https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) (a Clearpath UGV) to autonomously navigate and map an unknown area using SLAM and a custom made frontier exploration algorithm based on laser data from a [Velodyne VLP-16 LIDAR](http://velodynelidar.com/vlp-16.html).

#### How it works
- The  [NavFN](http://wiki.ros.org/navfn?distro=indigo) planner is used for global path planning
- The [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) is used for local path planning
- A custom made exploration planner is used for determining the next goal position to go to. My algorithm is based off a frontier exploration algorithm described in this [paper](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf)
- The [gmapping](http://wiki.ros.org/gmapping?distro=indigo) package is used for Simultaneous Localization and Mapping (SLAM)

## Installation
#### Prerequisits
- Linux running on Ubuntu 16.04 with ROS Kinetic
- Jackal running Linux on Ubuntu 14.04 with ROS Indigo (Not required if you just want to run the simulation without the real robot)

#### Dependencies
To install this package, open up a terminal and perform the following steps:
```bash
mkdir -p jackal_ws/src; cd jackal_ws/src; catkin_init_workspace
git clone git@github.com:mechwiz/jackal_exploration.git
cd jackal_ws; rosdep install --from-paths . --ignore-src --rosdistro=kinetic
cd jackal_ws; catkin_make; source devel/setup.bash
```
Note, replace the third line with the following when installing this repo on the actual robot.
```bash
cd jackal_ws; rosdep install --from-paths . --ignore-src --rosdistro=indigo
```
All of the required dependencies should be included (Gazebo, Rviz, relevant Jackal packages, Velodyne, etc...). This should hopefully just be a plug and play package.

## Implementation
Note, wait until each of the following launch files have finished starting before running the next one.
#### Simulation
- Launch the Gazebo world: `roslaunch jackal_exploration jackal_world.launch`
- Launch the Keyboard Teleop node if desired: `roslaunch jackal_exploration keyboard_teleop.launch`
- Launch the Jackal Setup configs: `roslaunch jackal_exploration jackal_setup.launch`. These include running:
    - gmapping to start SLAM
    - pointcloud filtering (cropbox and voxel-grid downsampling)
    - pointcloud to laserscan conversion to make gmapping happy
    - twist_mux to allow for various cmd_vel inputs based on priority including:
        - PS3 controller (`cmd_vel\joy`) - highest priority
        - Keyboard (`cmd_vel\keyboard`) - second priority
        - Interactive Markers (`cmd_vel\marker`) - third priority
        - move_base navigation (`cmd_vel\nav`) - lowest priority
- Launch Rviz: `roslaunch jackal_exploration view_robot.launch`
- Launch the Jackal Exploration node: `roslaunch jackal_exploration jackal_exploration.launch`

#### Actual Robot
In order for this to work, you will need to be connected to Jackal over a local network. Clearpath has provided some [documentation](https://www.clearpathrobotics.com/assets/guides/jackal/network.html) to help you with this.

##### Initial Setup
SSH into Jackal:
- Navigate to the `/etc/ros/indigo/ros.d` directory.
- Modify the `base.launch` startup file so that the following lines:
```
<include file="$(find jackal_description)/launch/description.launch" />
<include file="$(find jackal_control)/launch/control.launch" />
<include file="$(find jackal_control)/launch/teleop.launch">
```
are changed to
```
<include file="$(find jackal_exploration)/launch/description.launch" />
<include file="$(find jackal_exploration)/launch/control.launch" />
<include file="$(find jackal_exploration)/launch/teleop.launch">
```
Note that the [jackal.urdf.xacro](urdf/jackal.urdf.xacro) file (which is loaded in the [description.launch](launch/description.launch) file) has been modified to include the Velodyne Lidar and mounting plate as they are setup on the Northwestern Jackal robot.
- Add the [velodyne_startup.launch](launch/velodyne_startup.launch) file to the directory
- Navigate to the `/etc/ros/setup.bash` file and change the line `source /opt/ros/indigo/setup.bash` to `source /home/administrator/jackal_ws/devel/setup.bash`
- Reboot Jackal

##### Running the launch files
- Launch the Jackal Setup configs: `roslaunch jackal_exploration jackal_setup.launch`

On your computer:
- Export the following ROS environment variables similar to how it's done in [remote-jackal.sh](remote-jackal.sh).
```bash
export ROS_MASTER_URI=http://CPR-J100-0076.local:11311  # Jackal's hostname and port
export ROS_HOSTNAME=yourhostname.local # your computer's hostname
```
- Launch Rviz: `roslaunch jackal_exploration view_robot.launch`
- Launch the Keyboard Teleop node if desired: `roslaunch jackal_exploration keyboard_teleop.launch`

On Jackal:
- Launch the Jackal Exploration node: `roslaunch jackal_exploration jackal_exploration.launch`


#### Nodes
##### Exploration Node
[`exlore.py`](src/explore.py)

This node is the brain that computes what the next goal position should be that Jackal should go to. See my portfolio post that goes into detail about this [here](https://mechwiz.github.io/Portfolio/).

Subscribed Topics:
- `/map` for updating the current map
- `/move_base/global_costmap/costmap` for updating the current global costmap
- `/move_base/global_costmap/costmap_updates` for updating the current global costmap
- `move_base/result` for determining when Jackal has reached its goal position
- `move_base/feedback` for determining where Jackal is in the map at any given point in time

Published Topic: `/move_base_simple/goal`

#### Robot Localization
In the development process, several uncharacteristic behaviors were noticed:
- z-drift between the `odom` and `base_link` frames
- Pitch and Roll drift

The culprit seemed to be due to the default [robot_localization.yaml](https://github.com/jackal/jackal/blob/kinetic-devel/jackal_control/config/robot_localization.yaml) file which sets the parameters for sensor fusion of the odometry and IMU data. Roll, pitch, and yaw data were being considered from the IMU but were not set to be _relative_ (i.e. calibrated so that the first data point is the new zero). This explained the uncharacteristic pitch and roll behavior. Due to the fact that this project is only meant for a flat surface anyway, the roll and pitch parameters were changed such that the robot-ekf does not even consider them. Similarly, the parameter file by default considers velocity along the z-axis from the robot odometry which probably led to the z-drift problem, especially since the _relative_ parameter was initially set to false. As this project is only meant for a flat surface, the z-axis velocity is not considered. These changes can be seen in the updated [robot_localization.yaml](params/robot_localization.yaml). This fixed the uncharacterisitc behavior that was seen.

## Demo & Future Improvements
#### Video
- A video of Jackal exploring a simulated environment in Gazebo can be found [here](https://www.youtube.com/watch?v=x4oIJKmgQMc).
- A video of Jackal exploring an actual hallway in Northwestern can be found [here](https://www.youtube.com/watch?v=8slMv4ZIi4U).
#### Further Improvements
- Making the exploration algorithm more efficient and robust.
