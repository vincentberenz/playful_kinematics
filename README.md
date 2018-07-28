# PLAYFUL KINEMATICS


## What it is

* Playful kinematics is a tool for computing inverse kinematics in python (using a c++ backend).
* It is designed to favor convenience and reactivity over precision. It works similarly on under-actuated and over-actuated robots.
* It has no scientific novelty and no originality. It uses coordinate descent. 
* It works on Ubuntu
* It uses KDL in the background
* It uses caktin for compilation and requires some ROS package

## See it in action

The following two videos shows playful kinematics in action:

* https://www.youtube.com/watch?v=71gciiVrOsY
* https://www.youtube.com/watch?v=V05v9Z_5K6E

## Supported robots

* For the moment, only Softbank robotics pepper is supported.

Adding a new robot is simple if 1) this robot has between 5 to 8 dofs effectors (including) and 2) you have an urdf of the robot

If your	robot has less or more dofs in its effectors, source code will need to be modified in the source code is various manageable but	annoying ways.


## Playful

Playful kinematics (this repository) has no dependencies on Playful (the scripting language), and can be used independantly (but many playful libraries use playful kinematics).
For more information on Playful : playful.is.tuebingen.mpg.de

## Prerequisites

* The following ros packages needs to be installed: kdl_parser, urdf. Playful kinematics has been tested exclusively on indigo and kinetics, but is likely to work with other versions. Visit: http://wiki.ros.org/ROS/Installation. kdl_parser and urdf can be installed via aptitude:

```bash
# You may need to replace 'kinetic' with the ros distro you use
sudo apt-get install ros-kinetic-kdl-parser ros-kinetic-urdf
```
## Installation and compilation

* Create a catkin workspace (if you do not already have one)

```bash
mkdir workspace
cd workspace
mkdir src
```

* Clone playful kinematics (this repo) 

```bash
cd src
git clone https://git-amd.tuebingen.mpg.de/amd-clmc/playful-kinematics.git
```

* Compilation

```bash
source /opt/ros/kinetic/setup.bash # replace kinetic by the distro you use
catkin_make install
```

* Unit tests

```bash
source ./devel/setup.bash 
catkin_make run_tests
```


## Usage

* In short:

```python

from playful_kinematics.playful_kinematics import IK
import math

# setting ik for pepper robot
ik = IK("pepper")	

# setting a reference posture for the left arm
reference_posture={'LShoulderRoll': 0.78535,
                   'LShoulderPitch': 0.0,
		   'LElbowYaw': 0.0,
		   'LWristYaw': 0.0,
		   'KneePitch': 0.0,
		   'HipRoll': 0.0,
		   'LElbowRoll': -0.78535,
		   'HipPitch': 0.0}

# we do not want to move the knee and hip
blocked_joints = {"KneePitch":0.0,
                  "HipPitch":0.0,
                  "HipRoll":0.0}

# asking the IK to find a target posture minimizing
# from this reference
ik.set_reference_posture(True,reference_posture) 

# where we want to end effector to be in cartesian space
target_xyz = [0.2, 0.24, 1.0]

# desired orientation of the end-effector. Here
# we require the yaw to be at , and pitch and tilt are ignored.
target_orientation = [-math.pi/2.0,None,None]

# True means we find a posture for the left arm
# success is a boolean, True if the IK found a precise solution
# score: the lowest the value, the best the solution
# posture : dictionary {joint_name:angle}
success,score,posture = ik.get_posture(True,
                                       target_xyz,
                                       target_orientation,
				       blocked_joints=blocked_joints)

# note : if no solution was found, the posture returned is still
#        meaningfull, i.e. the best the robot could do to get close
#        to the target



```

* For a running example which cover the full API on Pepper, check the examples folder
* For doxygen html documentation, see the doc folder

## Adding a new robot

* copy the urdf file of the robot in the urdf folder
* edit CMakeLists.txt. Take example on already existing robots
* add a python file named 'set_ik_for_name.py' (replace name by your robot name) in scripts/playful_kinematics . Take inspiration of files setting ik for already supported robots
* add example files in scripts/playful_kinematics/examples . Take inspiration from existing examples.

If your robot end effector has less than 5 dofs or more than 8 dofs, you will also need to edit the source files. See ik.cpp and ik.py. 


## Support

playful-support A T tuebingen mpg de

## Author

vincentberenz.is.tuebingen.mpg.de


