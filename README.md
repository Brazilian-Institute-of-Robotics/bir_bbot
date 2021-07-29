# Bbot - Balancing Robot

**Authors:** Lucas Lins Souza and Matheus Henrique Nunes França  
**Advisor:** Marco Antonio dos Reis

## Dependencies

Download the following dependencies using the command below

```
 sudo apt install ros-noetic-joint-trajectory-controller ros-noetic-pid ros-noetic-twist-mux*
```

Clone the following repository to the `src` folder of your workspace.

```
$ git clone https://github.com/AprilRobotics/apriltag.git

$ git clone https://github.com/AprilRobotics/apriltag_ros.git
```

Return to the root of the catkin workspace and run:

`rosdep install --from-paths src --ignore-src -r -y `

Build the workspace with:

```
catkin build
# OR
catkin_make_isolated
```

**List of dependencies:**

* [Trajectory controller](http://wiki.ros.org/joint_trajectory_controller);
* [PID node: link](http://wiki.ros.org/pid);
* [Twist_mux](http://wiki.ros.org/twist_mux).
* [AprilTAG](https://github.com/AprilRobotics/apriltag.git)
* [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros.git)


## Simulation 

```
 roslaunch bbot_gazebo bbot_gazebo.launch

 roslaunch bbot_control pid_control.launch

 roslaunch bbot_perception tag_detection.launch
```