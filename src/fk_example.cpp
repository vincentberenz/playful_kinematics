// Copyright  (C)  2018 Max Planck Gesellschaft
// Autor : Vincent Berenz

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


#include "playful_kinematics/fk.h"

int main( int argc, char** argv ){

  double q[NB_JOINTS];
  double xyz[3];
  double euler[3];
  for(int i=0;i<NB_JOINTS;i++) q[i]=0.1;

  bool success = playful_kinematics::forward_kinematics(true, q, &xyz[0], &xyz[1], &xyz[2],&euler[0], &euler[1], &euler[2]);
  for (int i=0;i<3;i++) std::cout << xyz[i] << " ";
  std::cout << success << " " << std::endl;   

}
