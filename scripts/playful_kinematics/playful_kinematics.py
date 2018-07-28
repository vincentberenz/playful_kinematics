# Copyright  (C)  2018 Max Planck Gesellschaft
# Author : Vincent Berenz

# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.

# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


import ctypes,importlib,copy,os,math
from playful_kinematics_cpp import _Playful_ik


# End user interface !

##
# Class for performing inverse kinematics jobs
class IK:

    ##
    # robot_name only "pepper" supported for the moment
    # @param reference_posture dictionary {joint name:joint position} setting kinematic chain posture
    #                          from which minimization will be performed.
    #                          If None, default posture (for the specified robot) is used
    # @param joint_limits dictionary {joint name: (min,max)}
    #        if none, default limits used
    # @param minimization_priority dictionaty {joint name:integer score} the higher the priority
    #        (the lower the score) for a
    #        a joint, the higher priority it will have in the mimimization process
    #        if none, default is used (e.g. for Pepper, moving hips and knee is lower priority and
    #        the robot will avoid moving them, if possible)
    def __init__(self,robot_name,reference_posture=None,joint_limits=None,minimization_priority=None):

        # configuring all via code in set_ik_for_<robot_name>.py
        config_module_name = "playful_kinematics.set_ik_for_"+robot_name
        config_function = importlib.import_module(config_module_name)
        config_function.initialize()

        # code that will interface with c++
        self._playful_ik = _Playful_ik(robot_name,
                                       reference_posture=reference_posture,
                                       joint_limits=joint_limits,
                                       minimization_priority=minimization_priority)

    ##
    # overwrites the current reference posture for the specified end-effector
    # @param left if true, changing configuration for left effector, otherwise for right effector
    # @param reference_posture dictionary {joint name:joint position}
    def set_reference_posture(self,left,reference_posture):

        self._playful_ik.set_reference_posture(left,reference_posture)
        
    ##   
    # performs forward kinematics for the specified posture
    # @param left if true, left end-effector, otherwise right end-effector
    # @param posture dictionary {joint name:value}
    # @return tuple success(bool),[x,y,z],[yaw,pitch,roll]
    def forward_kinematics(self,left,posture):
        return self._playful_ik._fk(left,posture)

    ##
    # @param left if true, left end-effector, otherwise right end-effector
    # @return tuple [joint names],{joint name: (min limit,max_limit)}
    def get_params(self,left):

        if left :
            joints = copy.deepcopy(self._playful_ik.left_config.joints)
            limits = copy.deepcopy(self._playful_ik.left_config.joints_limits)

        else:
            joints = copy.deepcopy(self._playful_ik.right_config.joints)
            limits = copy.deepcopy(self._playful_ik.right_config.joints_limits)
        
        return joints,limits

    ##
    # @param left if true, left end-effector, otherwise right end-effector
    # @return {joint name: value} corresponding to the currently used reference posture
    def get_reference_posture(self,left):

        return self._playful_ik.get_reference_posture(left)
    


    ##
    # Inverse kinematics job, using current configuration
    # @param left if true, left end-effector, otherwise right end-effector
    # @param desired_xyz target cartesian position [x,y,z]. Use "None" for the dimensions
    #        that should not be taken into account
    # @param desired_ypr target cartesian orientation [yaw,pitch,roll]. Use "None" for the
    #        dimensions that should not be taken into account.
    # @param blocked_joints {joint_name:joint value} the posture found will have the specified joints at the
    #        specified posture, and the job will not use these joints for minimization.
    #        (not blocked joints if None)
    # @return tuple success (bool), score (the lower, the closer the end effector to desired target),
    #         posture (dict {joint name:joint position}) 
    def get_posture(self,left,desired_xyz, desired_ypr,blocked_joints=None):

        # getting joint names and limits
        joints,limits = self.get_params(left)

        # blocking some joints at desired fixed value
        if blocked_joints:
            self._playful_ik.block_joints(left,blocked_joints)

        # minimizing
        success,score,posture = self._playful_ik.ik(left,target_xyz=desired_xyz,target_abg=desired_ypr)
        posture = {joint:value for joint,value in zip(joints,posture)}

        # unblocking joints
        self._playful_ik.unblock_joints(left)

        return success, score, posture

