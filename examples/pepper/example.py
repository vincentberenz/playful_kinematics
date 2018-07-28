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



try :
    from playful_kinematics.playful_kinematics import IK
except :
    raise Exception("failed to import playful kinematics. Did you source devel ?")

import math,time


# in this example, we have pepper moving its left hand to trace circles


# ---------------- configuration ---------------- # 

# WARNING: some configurations (e.g, requiring the robot
# to trace very large circle while moving arms, knee and hips)
# may result in the robot loosing balance. Proceed with
# caution. We decline any responsability for any damage or
# injury.

# Motion plane. The robot will make circle with its hand
# on the xy , the yz or the xz plan.
# Select the one you want to test
xy = [1,2,0]
yz = [0,1,2]
xz = [1,0,2]
motion_plane = xy

# circle center shift
# changing the values below will
# move the center of the circle in the
# cartesian space
shift = [-0.03,-0.05,0.08]

# Motion amplitude, i.e radius of the motion
# Note: bigger radius may require the robot
# to use knee and hip (if not blocked)
amplitude = 0.06

# choose the arm to move
left = False # move the right arm
#left = True # move the left arm

# uncomment to block knee and hip at 0
# or to just block the knee
# or to not block anything
# The robot will not use blocked joints
# note: if you look at the file set_ik_for_pepper.py
# around line 45, you may notice that knee and hip have
# lower priority, i.e. even if not blocked, the robot
# will attempt not to use them.
blocked_joints = {"KneePitch":0.0,
                  "HipPitch":0.0,
                  "HipRoll":0.0}
#blocked_joints = {"KneePitch":0.0}
#blocked_joints = {"KneePitch":0.0,
#                  "HipPitch":0.0}
#blocked_joints = {}

# None: we do not care about orientation
# of the end effector
# (yaw, pitch, tilt)
hand_orientation = [None,None,None]
# uncomment to force palm directing right
hand_orientation = [-math.pi/2.0,None,None]
# uncomment to force palm up
#hand_orientation = [-math.pi/1.1,None,None]

# for how many seconds the program should run ?
duration = 15

# set to false if you prefer the robot not to move
# note : robot control assumes the robot is
# stiff and in start posture
robot_control = False

IP = "127.0.0.1" # change to your robot IP
PORT = 9559 # typical port for real robot


# ----------------------------------------------- #


def round_print(values):
    values_ = ["%.3f" % v for v in values]
    print values_


# returns current joint position of the robot,
# using NaoQi
def get_current_posture(joints,motion_proxy):
    values = motion_proxy.getAngles(joints,True)
    return {joint:value for joint,value in zip(joints,values)}
    
    
# preparing robot control
if robot_control:
    try:
        import naoqi
        motion = naoqi.ALProxy("ALMotion", IP, PORT)
        posture = naoqi.ALProxy("ALRobotPosture",IP,PORT)
        robot_control = True
    except :
        print "naoqi not found, robot control disabled. You may download py naoqi from Softbank robotics website."
        robot_control = False

    

# instantating playful kinematics
# (note: set_ik_for_pepper.py called automatically) 
ik = IK("pepper")


# we use the joints mid range as reference posture
joint_zero = {}
joints,limits = ik.get_params(left)
reference_posture = {}
zeros = []
for joint in joints :
    min_,max_ = limits[joint]
    zero = ( min_+max_ ) / 2.0
    reference_posture[joint]=zero
    zeros.append(zero)
ik.set_reference_posture(left,reference_posture) 

    
# moving the robot to initial posture
if robot_control:
    motion.setStiffnesses("Body", 1.0)
    posture.goToPosture("Stand",0.6)
    motion.angleInterpolation(joints, zeros, 2.0, True)

# forward kinematics : position of hand in cartesian space
# when at reference posture
success,reference_xyz,_ = ik.forward_kinematics(left,reference_posture)
for index,shift_ in enumerate(shift):
    reference_xyz[index]+=shift_



# starting job
time_start = time.time()
update = 0
increment = 0.06
time_wait = 0.005
previous_motion_id = None
while time.time()-time_start < duration :

    # target cartesian position for this iteration
    update += increment
    target_xyz = [r for r in reference_xyz]
    for index,(target,plane) in enumerate(zip(target_xyz,motion_plane)):
        if plane == 1:
            target+=amplitude*math.cos(update)
        elif plane == 2:
            target+=amplitude*math.sin(update)
        target_xyz[index]=target
        
    # inverse kinematics
    success,score,posture = ik.get_posture(left,
                                           target_xyz,
                                           hand_orientation,
                                           blocked_joints=blocked_joints)
        
    # evaluating ik: does the new position correspond to the target position ?
    success,xyz,ryp_ = ik.forward_kinematics(left,posture)
    result = [abs(target-result) for target,result in zip(target_xyz,xyz)]
    print "target position"
    round_print(target_xyz)
    print "end effector position"
    round_print(xyz)
    print "difference"
    round_print(result)
    print

    # updating robot motion using NaoQi
    if robot_control:

        joints = []
        values = []
        durations = []
        for joint,value in posture.iteritems():
            joints.append(joint)
            values.append(value)
            durations.append(1.0)
            
        motion_id = motion.post.angleInterpolation(joints, values, durations,True)
        try :
            motion.stop(previous_motion_id)
        except :
            pass
        previous_motion_id = motion_id

    time.sleep(time_wait)
        

