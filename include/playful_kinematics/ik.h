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
#include "playful_kinematics/soma.h"
#include "playful_kinematics/score_functions.h"


namespace playful_kinematics {

  /**
   * performs inverse kinematics for the specified end effector 
   * to reach (x,y,z) cartesian position. Orientation of the end-effector is
   * not taken into account.
   * @param left left end effector if true, right end effector otherwise
   * @param target_x x position the end effector should reach
   * @param target_y y position the end effector should reach
   * @param target_z z position the end effector should reach
   * @param get_posture joint positions corresponding of the end-effector reaching the desired cartesian position
   * @param get_score how close the end effector is to the desired position. The lower the score the better.
   * @see kinematics_config.h see how to configure the inverse kinematics job
   * @see score_functions.h see the scoring function used during minimization
   * @see soma.h see the minimization algorithm
   */
  bool ik(bool left, float target_x, float target_y, float target_z, 
	  std::vector<float> &get_posture,float &get_score);

   /**
   * performs inverse kinematics for the specified end effector 
   * to reach (x,y,z) cartesian position and (yaw,pitch,roll)
   * orientation 
   * @param left left end effector if true, right end effector otherwise
   * @param target_x x position the end effector should reach
   * @param target_y y position the end effector should reach
   * @param target_z z position the end effector should reach
   * @param target_yaw yaw orientation the end effector should reach
   * @param target_pitch pitch orientation the end effector should reach
   * @param target_roll roll orientation the end effector should reach
   * @param get_posture joint positions corresponding of the end-effector reaching the desired cartesian position
   * @param get_score how close the end effector is to the desired position. The lower the score the better.
   * @see kinematics_config.h see how to configure the inverse kinematics job
   * @see score_functions.h see the scoring function used during minimization
   * @see soma.h see the minimization algorithm
   */
  bool ik(bool left, float target_x, float target_y, float target_z, 
	  float target_yaw, float target_pitch, float target_roll, 
	  std::vector<float> &get_posture,float &get_score);

}
