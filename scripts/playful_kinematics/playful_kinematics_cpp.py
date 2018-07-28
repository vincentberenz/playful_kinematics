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
from playful_kinematics_configuration import _Configuration


# goes up directories until finding devel, then finding
# the c++ libraries from there.
# assumes things have been compiled using "catkin_make"

def _find_library(robot_name):

    def find_devel_folder(path):
        
        content = os.listdir(path)

        for c in content:

            if c=="devel" and os.path.isdir(path+os.sep+c):

                return path+os.sep+c

        try :
            
            upper_path = os.path.abspath(path+os.sep+'..')
            return find_devel_folder(upper_path)

        except :

            raise Exception("failed to find 'devel' directory, "+
                            "did you compile the workspace using catkin ?")
            
    path = os.path.abspath(__file__)    
    filename = os.path.basename(path)
    path = path[:-len(filename)]

    devel = find_devel_folder(path)

    lib = devel+"/lib/lib"+robot_name+"_kinematics.so"

    if not os.path.isfile(lib):
        raise Exception("failed to find the library "+lib)

    return lib
    


# bridges to the cpp and performs FK and IK

class _Playful_ik:

    
    def __init__(self,robot_name,reference_posture=None,joint_limits=None,minimization_priority=None,path_to_cpp_lib=None):

        # loading config related to robot
        try :
            self.left_config = _Configuration.configurations[robot_name][True]
            self.right_config = _Configuration.configurations[robot_name][False]
        except :
            error_message = "could not find configuration for robot",robot_name+"did you call set_playful_kinematics_for_"+str(robot_name)+".py ?"
            raise Exception(error_message)

        # if path to library not explicitely given, trying to find it ourself
        if path_to_cpp_lib is None:
            path_to_cpp_lib = _find_library(robot_name)

        # bridging python and cpp
        try :
            self.left_config.set_cpp_bridge(path_to_cpp_lib)
            self.right_config.set_cpp_bridge(path_to_cpp_lib)
        except Exception as e:
            raise Exception("failed to import library: "+str(path_to_cpp_lib)+":\n"+str(e))
            
        # updating config with reference posture if requested 
        if reference_posture:
            self.left_config.update_reference(reference_posture)
            self.right_config.update_reference(reference_posture)

        # updating joint limits if requested
        if joint_limits:
            self.left_config.update_limits(joint_limits)
            self.right_config.update_limits(joint_limits)
            
        # updating minimization order if requested
        if minimization_priority:
            self.left_config.update_minimization_limits(minimization_priority)
            self.right_config.update_minimization_limits(minimization_priority)


    def set_reference_posture(self,left,reference_posture):

        if left :
            self.left_config.update_reference(reference_posture)
        else:
            self.right_config.update_reference(reference_posture)

            
    def get_reference_posture(self,left):

        if left:
            config = self.left_config
        else :
            config = self.right_config

        return copy.deepcopy(config.reference_posture)

            
    def _fk(self,left,posture):

        if left:
            config = self.left_config
        else :
            config = self.right_config
            
        posture = [posture[joint] for joint in config.joints]

        ctype_joints = ctypes.c_double * len(posture)
        
        x = ctypes.c_double(0)
        y = ctypes.c_double(0)
        z = ctypes.c_double(0)

        alpha = ctypes.c_double(0)
        beta = ctypes.c_double(0)
        gamma = ctypes.c_double(0)

        success = config.kinematics_lib.forward_kinematics(ctypes.c_bool(left),
                                                           ctype_joints(*posture),
                                                           ctypes.byref(x),
                                                           ctypes.byref(y),
                                                           ctypes.byref(z),
                                                           ctypes.byref(alpha),
                                                           ctypes.byref(beta),
                                                           ctypes.byref(gamma))

        return success,[a.value for a in [x,y,z]],[a.value for a in [alpha,beta,gamma]]

    
    def get_joint_names(self,left):

        if left:
            return self.left_config.joints

        return self.right_config.joints

    
    def fk(self,left,current_posture):

        return self._fk(left,current_posture)


    def block_joints(self,left,joints_values):
        
        if left:
            config = self.left_config
        else :
            config = self.right_config

        config.block_joints(joints_values)


    def unblock_joints(self,left):

        if left:
            config = self.left_config
        else :
            config = self.right_config

        config.unblock_joints()

    
    def ik(self,left,
           target_xyz=[None,None,None],
           target_abg=[None,None,None]):

        if left:
            config = self.left_config
        else:
            config = self.right_config

        reference_posture = config.reference_posture

        mask = [False if v is None else True for v in target_xyz] + [False if v is None else True for v in target_abg]  

        config.prepare_ik(mask)
        
        left = ctypes.c_bool(left)
        x,y,z = [0 if v is None else ctypes.c_float(v) for v in target_xyz]
        alpha,beta,gamma = [0 if v is None else ctypes.c_float(v) for v in target_abg]

        joints = [ctypes.c_float(reference_posture[joint]) for joint in config.joints]
        byrefs = [ctypes.byref(joint) for joint in joints]
        score = ctypes.c_float(0)
        score_ = ctypes.byref(score)
        byrefs.append(score_)

        success = config.kinematics_function(left,
                                             x,y,z,
                                             alpha,beta,gamma,
                                             *byrefs)

        
        return success,score.value,[joint.value for joint in joints]

