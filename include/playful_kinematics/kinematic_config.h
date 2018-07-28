// Copyright  (C)  2018 Max Planck Gesellschaft
// Author : Vincent Berenz

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>

namespace playful_kinematics {

  
  /*! set for the next inverse kinematics jobs the posture from 
      which minimization will be performed
   */
  void set_kinematics_joints(std::vector<float> reference_ik_joints);

  
  /*! set for the next inverse kinematics jobs the side (left or right
      end effector)
   */
  void set_kinematics_side(bool left);

  
  /*! the mask is used to determine which dimensions in the cartesian
      space the inverse kinematics should take into account.
      for example [true,false,false,true,false,false] will
      only attempt to bring the end effector at the desired 
      x position with the desired yaw.
   */
  void set_kinematics_mask(std::vector<bool> mask);

  
  /*! set the limits of the joint at the specified index  
   */
  void set_kinematics_joint_limit(int index, float min, float max);

  
  /*! minimization toward the desired cartesian position will be done
      by moving first joints with higher priority (1 is higher priority that 2).
      thus, in case of overactuated robot, this has an impact on which solution
      will be found.
   */
  void set_minimization_priority(std::vector<int> minimization_priority);

  
  /*! returns the joint position as last set by "set_reference_posture" 
   */
  void get_kinematics_joints(std::vector<float> &get);

  
  /*! returns the side, left (true) or right (false) as last set by
      "set_kinematics_side"
   */
  bool get_kinematics_side();

  
  /*! returns the minimization priority as last set by 
      "set_minimization_priority"
   */
  std::vector<int> get_minimization_priority(int size);

  
  /*! returns mask as last set by "set_kinematics_mask"
   */
  std::vector<bool>& get_kinematics_mask();

  
  /*! returns min joint limits as last set by 
    "set_kinematics_joint_limit" */
  std::map<int,float>& get_kinematics_joint_min_limit();

  
  /*! returns max joint limits as last set by 
    "set_kinematics_joint_limit" */
  std::map<int,float>& get_kinematics_joint_max_limit();

  
  /*! returns the number of joints used for inverse and 
      forward kinematics. This is based on the urdf and 
      chain specified in the CMakeLists.txt 
   */
  int get_kinematics_nb_joints();

}
