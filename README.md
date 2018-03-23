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
- Launch the Jackal Setup configs: `roslaunch jackal_exploration jackal_setup.launch`. These include running:
    - gmapping
    - pointcloud filtering
    - pointcloud to laserscan conversion to make gmapping happy
    - z-correction
- Launch Rviz: `roslaunch jackal_exploration view_robot.launch`
- Launch the Jackal Exploration node: `roslaunch jackal_exploration view_robot.launch`

#### Actual Robot
In order for this to work, you will need to be connected to Jackal over a local network. Clearpath has provided some [documentation](https://www.clearpathrobotics.com/assets/guides/jackal/network.html) to help you with this.

SSH into Jackal and run:
- Launch the Jackal Setup configs: `roslaunch jackal_exploration jackal_setup.launch`

On your computer:
- Export the following ROS environment variables similar to how it's done in [remote-jackal.sh](remote-jackal.sh).
```bash
export ROS_MASTER_URI=http://CPR-J100-0076.local:11311  # Jackal's hostname and port
export ROS_HOSTNAME=yourhostname.local # your computer's hostname
```
- Launch Rviz: `roslaunch jackal_exploration view_robot.launch`

On Jackal:
- Launch the Jackal Setup configs: `roslaunch jackal_exploration jackal_setup.launch`
- Launch the Jackal Exploration node: `roslaunch jackal_exploration view_robot.launch`


#### Nodes
##### Z-Correction Node
[`z_correction.py`](src/z_correction.py)

This node corrects for any z-drift between the `odom` and `base_link` frames that tends to occur as the Jackal moves around. The corrected fram is outputted in the `odom_corrected` frame.

Subscribed Topic: `/odometry/filtered`

Published Topic: `/odometry/corrected`


##### Exploration Node
[`gridcalc.py`](src/gridcalc.py)

This node is the brain that computes what the next goal position should be that Jackal should go to. See my portfolio post that goes into detail about this [here](https://mechwiz.github.io/Portfolio/).

Subscribed Topics:
- `/map` for updating the current map
- `/move_base/global_costmap/costmap` for updating the current global costmap
- `/move_base/global_costmap/costmap_updates` for updating the current global costmap
- `move_base/result` for determining when Jackal has reached its goal position
- `move_base/feedback` for determining where Jackal is in the map at any given point in time

Published Topic: `/move_base_simple/goal`


## Demo & Future Improvements
#### Video
A video of Jackal exploring a simulated environment in Gazebo can be found [here](https://www.youtube.com/watch?v=x4oIJKmgQMc).

#### Further Improvements
- Further tuning still needs to be done to have Jackal create less noisy maps in a real environment.
- Making the exploration algorithm to be more robost to take into account more edge cases. See my portfolio post for examples.
