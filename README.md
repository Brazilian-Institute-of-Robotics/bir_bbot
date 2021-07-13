# Bbot - Balancing Robot

**Authors:** Lucas Lins Souza and Matheus Henrique Nunes França  
**Advisor:** Marco Antonio dos Reis

## Dependencies

Download the following dependencies using the command below

```
 sudo apt install ros-noetic-joint-trajectory-controller ros-noetic-pid ros-noetic-twist-mux*
```

**List of dependencies:**

* [Trajectory controller](http://wiki.ros.org/joint_trajectory_controller);
* [PID node: link](http://wiki.ros.org/pid);
* [Twist_mux](http://wiki.ros.org/twist_mux).


## Simulation 

```
 roslaunch bbot_gazebo bbot_gazebo.launch

 roslaunch bbot_control pid_control.launch
```