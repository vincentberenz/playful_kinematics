from playful_kinematics_configuration import _Configuration
import os

# This sets the configurations (one for the left arm and one for the right arm)
# for Softbank robotics Pepper robot.

# Can be used as example to set configuration for other robots


_ROBOT_NAME = "pepper"

_JOINTS = [ "KneePitch" ,
            "HipPitch","HipRoll",
            "ShoulderPitch" , "ShoulderRoll",
            "ElbowYaw", "ElbowRoll",
            "WristYaw"]

# knee and hip also parts of left arm 
_LEFT_JOINTS = _JOINTS[:3] + ["L"+joint for joint in _JOINTS[3:]]

# knee and hip also parts of right arm
_RIGHT_JOINTS = _JOINTS[:3] + ["R"+joint for joint in _JOINTS[3:]]

# as provided in Pepper's documantation
_JOINTS_LIMITS = {
    "HipRoll":(-0.5149,0.5149),
    "KneePitch":(-1.0385,1.0385),
    "HipPitch":(-0.5149,0.5149),
    "LShoulderPitch":(-2.0857,2.0857),
    "LShoulderRoll":(0.0087,1.562),
    "LElbowYaw":(-2.0857,2.0857),
    "LElbowRoll":(-1.562,-0.0087),
    "LWristYaw":(-1.8239,1.8239),
    "RShoulderPitch":(-2.0857,2.0857),
    "RShoulderRoll":(-1.562,-0.0087),
    "RElbowYaw":(-2.0857,2.0857),
    "RElbowRoll":(0.0087,1.562),
    "RWristYaw":(-1.8239,1.8239)
}

# minimization priority, i.e which joint should move in
# priority to achieve the desired cartesian position.
# Here, we use the hip and knee only if really necessary
# (priority of 1 for all joints, except of hip and knee which have priority of 2)
_MINIMIZATION_PRIORITY = {joint:1 for joint in _JOINTS_LIMITS.keys()}
_MINIMIZATION_PRIORITY["HipRoll"]=2
_MINIMIZATION_PRIORITY["HipPitch"]=2
_MINIMIZATION_PRIORITY["KneePitch"]=2


def initialize():

    left_config = _Configuration(_ROBOT_NAME, # robot name
                                True, # configuration is for left side (True means left)
                                _LEFT_JOINTS, # list of joints
                                None, # default reference posture
                                {joint:_JOINTS_LIMITS[joint] for joint in _LEFT_JOINTS}, # joint limits, {joint name: (min,max)}
                                minimization_priority = _MINIMIZATION_PRIORITY)  # minimization priority, {joint name: priority}


    right_config = _Configuration(_ROBOT_NAME,False,
                                 _RIGHT_JOINTS,
                                 None,
                                 {joint:_JOINTS_LIMITS[joint] for joint in _RIGHT_JOINTS},
                                 minimization_priority = _MINIMIZATION_PRIORITY)

    # saving configuration 
    _Configuration.configurations[_ROBOT_NAME]={}
    _Configuration.configurations[_ROBOT_NAME][True]=left_config
    _Configuration.configurations[_ROBOT_NAME][False]=right_config



