# gazebo_ros_pkgs

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kpr__gazebo_ros_pkgs__ubuntu_xenial_amd64)](http://build.ros.org/job/Kpr__gazebo_ros_pkgs__ubuntu_xenial_amd64)

Wrappers, tools and additional API's for using ROS with the Gazebo simulator. Formally simulator_gazebo stack, gazebo_pkgs is a meta package. Now Catkinized and works with the standalone Gazebo debian.

### Installation
```
source /opt/ros/dashing/setup.bash
mkdir -p ~/gazebo_ros_ws/src/ros-simulation
cd ~/gazebo_ros_ws/src/ros-simulation
git clone -b dashing https://github.com/kkonen/gazebo_ros_pkgs.git
cd ~/gazebo_ros_ws
rosdep install --from-paths src --ignore-src -r -y
RMW_IMPLEMENTATION=rmw_opensplice_cpp colcon build --symlink-install
```

Ignore RMW_IMPLEMENTATION=rmw_opensplice_cpp if you are not installing this for the phantomx version of ros2learn, or don't.. I'm just a readme.


Be sure to source this workspace's install setup for every new terminal you open:
```
source ~/gazebo_ros_ws/install/setup.bash
```

### Documentation and Tutorials
[On gazebosim.org](http://gazebosim.org/tutorials?cat=connect_ros)

### Develop and Contribute

See [Contribute](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/hydro-devel/CONTRIBUTING.md) page.


