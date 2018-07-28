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


import ctypes


class _Configuration:

    # dict {robot_name:{True:Configuration,False:Configuration}}
    # True referes to left arm, False to right arm
    configurations = {}
    
    def __init__(self,robot_name,left,
                 joints,
                 reference_posture,
                 joints_limits,minimization_priority={}):


        self.left = left
        self.robot = robot_name
        self.joints = joints
        self.reference_posture = reference_posture
        self.joints_limits = joints_limits
        self.nb_dofs = len(joints)
        self.__class__.configurations[robot_name]=self
        self.blocked_joints = {}
        self.minimization_priority = {}
        try :
            max_priority = max(minimization_priority.values())
        except :
            max_priority = 0
        for joint in joints:
            try :
                self.minimization_priority[joint] = minimization_priority[joint]
            except:
                self.minimization_priority[joint] = max_priority+1

        self.reference_posture = None
        if reference_posture is not None:
            self.reference_posture = reference_posture

            
    # bridge to cpp library
    def set_cpp_bridge(self,kinematics_lib_path):
        
        self.kinematics_lib = ctypes.cdll.LoadLibrary(kinematics_lib_path)
        
        self.kinematics_lib.forward_kinematics.argtypes = (ctypes.c_bool,
                                                           ctypes.POINTER(ctypes.c_double),
                                                           ctypes.POINTER(ctypes.c_double),
                                                           ctypes.POINTER(ctypes.c_double),
                                                           ctypes.POINTER(ctypes.c_double),
                                                           ctypes.POINTER(ctypes.c_double),
                                                           ctypes.POINTER(ctypes.c_double),
                                                           ctypes.POINTER(ctypes.c_double))
    

        # bridge to cpp function
        kinematics_function = None

        self.kinematics_lib.get_nb_joints.argtypes = (ctypes.c_bool,)

        self.kinematics_lib.set_mask.argtypes = (ctypes.c_bool,
                                                ctypes.c_bool,
                                                ctypes.c_bool,
                                                ctypes.c_bool,
                                                ctypes.c_bool,
                                                ctypes.c_bool)

        c_floats = [ctypes.POINTER(ctypes.c_float)]*(self.nb_dofs+1)

        
        if self.nb_dofs==5 : self.kinematics_function = self.kinematics_lib.ik_5dofs
        elif self.nb_dofs==6 : self.kinematics_function = self.kinematics_lib.ik_6dofs
        elif self.nb_dofs==7 : self.kinematics_function = self.kinematics_lib.ik_7dofs
        elif self.nb_dofs==8 : self.kinematics_function = self.kinematics_lib.ik_8dofs
        else :
            raise Exception("playful kinematics supports only 5 to 8 dofs")

        self.kinematics_function.argtypes = (ctypes.c_bool,
                                             ctypes.c_float,ctypes.c_float,ctypes.c_float,
                                             ctypes.c_float,ctypes.c_float,ctypes.c_float) + tuple(c_floats)


        self.kinematics_lib.set_kinematics_joint_limit.argtypes = (ctypes.c_int,ctypes.c_float,ctypes.c_float)

        self.kinematics_lib.set_minimization_priority.argtypes = (ctypes.c_int,ctypes.c_int)
        
        

    # will set the min and max values of 
    # specified joint to specified same value,
    # so the joint will not move
    def block_joints(self,joints_values):

        for joint,value in joints_values.iteritems():
            self.blocked_joints[joint]=value

        self._update()
            

    def unblock_joints(self):

        self.blocked_joints = {}
        self._update()

        
    def update_minimization_order(self,minimization_order):

        self.minimization_priority = minimization_priority
        self._update()
        

    def update_reference(self,reference_posture):

        if self.reference_posture is None:
            self.reference_posture = {}
        
        for joint,value in reference_posture.iteritems():
            if joint in self.joints:
                self.reference_posture[joint]=value

        
    def update_limits(self,joint_min_max):

        for joint,values in joint_min_max.iteritems():
            if joint in self.joints_min.keys():
                self.joints_min[joint] = values[0]
                self.joints_max[joint] = values[1]
        self._update()

        
    def set_ik_mask(self,mask):

        mask_  = [ctypes.c_bool(m) for m in mask]
        self.kinematics_lib.set_mask(*mask_)


    def _update(self):

        for index,joint in enumerate(self.joints):

            index_ = ctypes.c_int(index)
            priority = ctypes.c_int(self.minimization_priority[joint])

            self.kinematics_lib.set_minimization_priority(index_,priority)

            min_,max_ = self.joints_limits[joint]
            try :
                value = self.blocked_joints[joint]
                min_ = value
                max_ = value
            except:
                pass

            min_ = ctypes.c_float(min_)
            max_ = ctypes.c_float(max_)
            self.kinematics_lib.set_kinematics_joint_limit(index_,min_,max_)


    def prepare_ik(self,mask):
        self.set_ik_mask(mask)
        self._update()
        
