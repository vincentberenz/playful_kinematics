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

#include <math.h>
#include <limits>
#include "playful_kinematics/fk.h"
#include "playful_kinematics/kinematic_config.h"
#include <cmath>

namespace playful_kinematics {

  /*! set the desired target cartesian position, 
      to be used before calling at_desired_cartesian_position */
  void set_target_cartesian_position(float x, float y, float z, float alpha, float beta, float gamma);

  /*! set configuration: use of left arm or right arm, position mask (x,y,z,alpha,beta,gamma)
    e.g. [true,true,true,false,false,false] if orientation if irrelevant */
  void set_configuration(bool left,std::vector<bool> mask);

  /*! score function, call set_target_cartesian_position and set_configuration first */
  float at_desired_cartesian_position(std::vector<float> &posture);

}
