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



#include "playful_kinematics/score_functions.h"


#define V_PI 3.14159265358979323846 
#define V_2_PI 2.0*V_PI


namespace playful_kinematics {


  static inline float cartesian_diff(float p1, float p2){
    return fabs(p1-p2);
  }

  
  static inline float rotation_diff(float a1, float a2){
    return fabs( V_PI - std::fabs(std::fmod(std::fabs(a1 - a2), V_2_PI) - V_PI) );
  }

  
  // ! the first 3 indexes are x, y, z and use cartesian diff
  // ! the last 3 indexes are alpha, beta, gamma and use rotation_diff
  static float _distance(float *cartesian_p1, float *cartesian_p2, std::vector<bool> &mask){

    float distance = 0;
    float diff;

    for(int i=0;i<3;i++){
      if (mask[i]){
	diff = cartesian_diff(cartesian_p1[i],cartesian_p2[i]);
	distance += (diff*diff);
      }
    }
    
    for(int i=3;i<6;i++){
      if (mask[i]){
	diff = rotation_diff(cartesian_p1[i],cartesian_p2[i]);
	distance += (diff*diff);
      }
    }

    distance = sqrt(distance);
    return distance;

  }


  static void _get_position_array(float *cartesian,float x, float y, float z,float alpha, float beta, float gamma){

    cartesian[0]=(float)x; cartesian[1]=(float)y; cartesian[2]=(float)z;
    cartesian[3]=(float)alpha; cartesian[4]=(float)beta; cartesian[5]=(float)gamma;

  }

  
  class target_cartesian_position {
  public:
    float x,y,z,alpha,beta,gamma;
    void set(float x, float y, float z, float alpha, float beta, float gamma){
      this->x = x;
      this->y = y;
      this->z = z;
      this->alpha = alpha;
      this->beta = beta;
      this->gamma = gamma;
    }
    
  };


  static boost::shared_ptr<target_cartesian_position> tcp;


  void set_target_cartesian_position(float x, float y, float z,
				     float alpha, float beta, float gamma){
    if(!tcp) tcp.reset(new target_cartesian_position());
    tcp->set(x,y,z,alpha,beta,gamma);
  }

  
  static void print_posture(std::vector<float> posture){
    for(int i=0;i<posture.size();i++) std::cout << posture[i] << "\t";
  }

  
  float at_desired_cartesian_position(std::vector<float> &posture){

    double q[NB_JOINTS];
    double x,y,z,alpha,beta,gamma;

    for(int i=0;i<posture.size();i++) q[i]=posture[i];
    bool success = playful_kinematics::forward_kinematics(playful_kinematics::get_kinematics_side(),q,
							  &x,&y,&z,
							  &alpha,&beta,&gamma);
    if (!success) {
      return std::numeric_limits<float>::max();
    }

    float cartesian[6];
    float cartesian_target[6];

    _get_position_array(cartesian,x,y,z,alpha,beta,gamma);
    _get_position_array(cartesian_target,tcp->x,tcp->y,tcp->z,tcp->alpha,tcp->beta,tcp->gamma);

    float distance = _distance(cartesian,cartesian_target,playful_kinematics::get_kinematics_mask());

    return distance;

  }

}
