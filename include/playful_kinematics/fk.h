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

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <kdl_parser/kdl_parser.hpp>
#include <cstdlib>

namespace playful_kinematics {



  /**
   * performs forward kinematics for the specified end effector
   * @param left left end effector if true, right end effector otherwise
   * @param posture list of current joint position of the kinematic chain (as defined in the urdf and specified in the CMakeLists.txt)
   * @param get_position cartesian position of the end-effector as determined by forward kinematics
   * @param get_orientation yaw, pitch and roll of the end-effector as determined by forward kinematics
   */
  bool forward_kinematics(bool left,
			  std::vector<float> &posture, std::vector<float> &get_position,
			  std::vector<float> &get_orientation);

  bool forward_kinematics(bool left, double *q,
			  double *x, double *y, double *z,
			  double *alpha, double *beta, double *gamma);

}
