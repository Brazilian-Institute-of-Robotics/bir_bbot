# bir_bbot - BIR Balancing Robot

The **bir_bbot**, provides the real robot bringup for the _Bbot_ balancing robot with _ROS_.

**Keywords:** Balancing robot, ROS

**Author: [Matheus França](https://github.com/MatheusFranca-dev) and [Lucas Souza](https://github.com/lucaslins0035)<br />
Advisor: [Marco Reis](https://github.com/mhar-vell)<br />
Affiliation: [BIR - Brazilian Institute of Robotics](https://github.com/Brazilian-Institute-of-Robotics)<br />**

# **Purpose of the Project**

Bbot, or _Balancing Robot_, is a self-balancing autonomous robot project. Our goal is to build a mobile robot operated via **ROS Noetic** capable of balancing and moving on two wheels. Furthermore, he must be able to read a TAG (fiducial framework). The TAG will send the robot a destination position to which it must autonomously navigate. To carry out navigation, this robot must be able to create a map of where it is and locate itself in it, allowing it to update its position throughout the mission and avoid obstacles while navigating to its objective. For more about this project see our web page, containing the steps for build this project, [LINK](https://mhar-vell.github.io/rasc/project-bbot/).

### Supported Versions

- **Noetic**: Built and tested under [ROS] Noetic and Ubuntu 20.04

### Dependencies 
- [ROS] : An open-source robot framework. (**Version == Noetic**)
- [Trajectory controller](http://wiki.ros.org/joint_trajectory_controller) : Controller for executing joint-space trajectories.
- [Twist mux](http://wiki.ros.org/twist_mux) : Twist multiplexer, which multiplex several velocity commands (topics) and allows to priorize or disable them (locks).
- [Teleop twist joy](http://wiki.ros.org/teleop_twist_joy) : Generic joystick teleop for twist robots.

# **Table of Contents**
- [**bir_bbot**](#bir_bbot)
- [**Purpose of the Project**](#purpose-of-the-project)
    - [Supported Versions](#supported-versions)
    - [Dependencies](#dependencies)
- [**File System**](#file-system)
- [**Installation**](#installation)
	- [Building from Source:](#building-from-source)
	- [Example of Usage](#example-of-usage)
	    - [Real](#real)
	    - [Simulation](#simulation)
- [**License**](#license)
- [**Bugs & Feature Requests**](#bugs--feature-requests)

# **File System**

- [doc_resources](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot/tree/feature/final/doc_resources) : Support files, including Jupyter about the control systems of Bbot and images to support the readme.
- [lqr_controller](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot/tree/feature/final/lqr_controller) : The support package for _bbot_control_. Contains our ROS LQR controller.
- [bbot_control](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot/tree/feature/final/bbot_control) : Contains the controllers parameters to the robot.
- [bbot_description](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot/tree/feature/final/bbot_description) : Defines the Bbot URDF, Rviz and meshes.
- [bbot_bringup](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot/tree/feature/final/bbot_bringup) : Is the real robot bringup package.

# **Installation**

##  Building from Source:

Attention, if you haven't installed [ROS] yet, please check [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.  

**Building:**

First, lets create a catkin workspace.

    $ mkdir -p ~/catkin_ws/src

Then, clone **bir_bbot** inside your workspace source. For bringup, use the branch 'feature/final'

	$ git clone git@github.com:Brazilian-Institute-of-Robotics/bir_bbot.git -b feature/final

Now, just build your catkin workspace.

    $ cd ~/catkin_ws
    $ catkin_make

Don't forget to source your workspace before using it.
    
    $ source devel/setup.bash

## Example of Usage

### Real

Just Run

	$ roslaunch bbot_bringup bbot_bringup.launch

The teleop launch is configured for a Xbox joystick. Just plug it in Bbot or another machine connected to the Bbot ROS Master and call the following launch from the same machine.

    $ roslaunch bbot_bringup bbot_teleop.launch

![](/doc_resources/bbot.gif)
* _Real model of Bbot._
	
### Simulation

For simulation, we have another repository, see the [bir_bbot-simulation](https://github.com/Brazilian-Institute-of-Robotics/bir_bbot-simulation), and have fun!!

# **License**

<!-- Bir Bbot source code is released under a [MIT License](/LICENSE). -->

# **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker].

<!-- Hyperlinks -->
[ROS]: https://www.ros.org
[Issue Tracker]: https://github.com/Brazilian-Institute-of-Robotics/bir_bbot/issues