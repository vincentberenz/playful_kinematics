
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



#include "playful_kinematics/ik.h"

namespace playful_kinematics {

  
  static boost::shared_ptr< std::vector<bool> > applied_mask;

  
  static void print_posture(std::vector<float> posture){

    for(int i=0;i<posture.size();i++) std::cout << posture[i] << "\t";

    std::cout << std::endl;

  }

  void _init_masks(){

    applied_mask.reset(new std::vector<bool>);

    for(int i=0;i<6;i++) {
      playful_kinematics::applied_mask->push_back(true);
    }
    
  }


  bool _ik(boost::shared_ptr< std::vector<bool> > mask, bool left, 
	   float target_x, float target_y, float target_z, 
	   float target_alpha, float target_gamma, float target_beta,
	   std::vector<float> &get_posture, float &get_score){

    playful_kinematics::set_target_cartesian_position(target_x,target_y,target_z,
						      target_alpha,target_beta,target_gamma);
    playful_kinematics::set_kinematics_side(left);
    playful_kinematics::set_kinematics_mask(*mask);
    playful_kinematics::get_kinematics_joints(get_posture);

    std::vector<int> minimization_priority = playful_kinematics::get_minimization_priority(get_posture.size());
    
    bool success = playful_kinematics::minimize(get_posture,
						minimization_priority,
						playful_kinematics::get_kinematics_joint_min_limit(),
						playful_kinematics::get_kinematics_joint_max_limit(),
						0.001,0.1,0.001,15,&(playful_kinematics::at_desired_cartesian_position),get_score);

    return success;

  }

  
  bool ik(bool left, float target_x, float target_y, float target_z, 
	  float target_alpha, float target_gamma, float target_beta, 
	  std::vector<float> &get_posture, float &get_score){

    if(!playful_kinematics::applied_mask) _init_masks();

    return _ik(applied_mask,left,
	       target_x,target_y,target_z,
	       target_alpha,target_beta,target_gamma,get_posture,
	       get_score);

  }


}


static void print_posture(std::vector<float> posture){
  for(int i=0;i<posture.size();i++) std::cout << posture[i] << "\t";
  std::cout << std::endl;
}


/* INTERFACE FOR PYTHON WRAPPER */

// note: implementation sucks, but could not manage to find ways to return float array using ctypes.
//       any cleaner update of this would be sincerely welcomed


extern "C" {

  void set_mask(bool x, bool y, bool z, bool alpha, bool beta, bool gamma){

    if(!playful_kinematics::applied_mask) {
      playful_kinematics::_init_masks();
    }

    playful_kinematics::applied_mask->at(0)=x;
    playful_kinematics::applied_mask->at(1)=y;
    playful_kinematics::applied_mask->at(2)=z;
    playful_kinematics::applied_mask->at(3)=alpha;
    playful_kinematics::applied_mask->at(4)=beta;
    playful_kinematics::applied_mask->at(5)=gamma;

  }


  bool ik_8dofs(bool left, float target_x, float target_y, float target_z, 
		float target_alpha, float target_gamma, float target_beta, 
		float *arm_1_joint, float *arm_2_joint, float *arm_3_joint, 
		float *arm_4_joint, float *arm_5_joint, float *arm_6_joint,  
		float *arm_7_joint,  float *arm_8_joint,
		float *get_score){

    std::vector<float> ik_joints;

    ik_joints.push_back(*arm_1_joint);
    ik_joints.push_back(*arm_2_joint);
    ik_joints.push_back(*arm_3_joint);
    ik_joints.push_back(*arm_4_joint);
    ik_joints.push_back(*arm_5_joint);
    ik_joints.push_back(*arm_6_joint);
    ik_joints.push_back(*arm_7_joint);
    ik_joints.push_back(*arm_8_joint);

    playful_kinematics::set_kinematics_joints(ik_joints);
    std::vector<float> get_posture;
    bool success = playful_kinematics::ik(left,target_x,target_y,target_z,
					  target_alpha,target_gamma,target_beta,
					  get_posture,*get_score);

    *arm_1_joint = get_posture[0];
    *arm_2_joint = get_posture[1];
    *arm_3_joint = get_posture[2];
    *arm_4_joint = get_posture[3];
    *arm_5_joint = get_posture[4];
    *arm_6_joint = get_posture[5];
    *arm_7_joint = get_posture[6];
    *arm_8_joint = get_posture[7];

    return success;
  }		


  bool ik_7dofs(bool left, float target_x, float target_y, float target_z, 
		float target_alpha, float target_gamma, float target_beta, 
		float *arm_1_joint, float *arm_2_joint, float *arm_3_joint, 
		float *arm_4_joint, float *arm_5_joint, float *arm_6_joint,  
		float *arm_7_joint,
		float *get_score){

    std::vector<float> ik_joints;

    ik_joints.push_back(*arm_1_joint);
    ik_joints.push_back(*arm_2_joint);
    ik_joints.push_back(*arm_3_joint);
    ik_joints.push_back(*arm_4_joint);
    ik_joints.push_back(*arm_5_joint);
    ik_joints.push_back(*arm_6_joint);
    ik_joints.push_back(*arm_7_joint);

    playful_kinematics::set_kinematics_joints(ik_joints);
    std::vector<float> get_posture;
    bool success = playful_kinematics::ik(left,target_x,target_y,target_z,
					  target_alpha,target_gamma,target_beta,
					  get_posture,*get_score);

    *arm_1_joint = get_posture[0];
    *arm_2_joint = get_posture[1];
    *arm_3_joint = get_posture[2];
    *arm_4_joint = get_posture[3];
    *arm_5_joint = get_posture[4];
    *arm_6_joint = get_posture[5];
    *arm_7_joint = get_posture[6];

    return success;

  }		


  bool ik_6dofs(bool left, float target_x, float target_y, float target_z, 
		float target_alpha, float target_gamma, float target_beta, 
		float *arm_1_joint, float *arm_2_joint, float *arm_3_joint, 
		float *arm_4_joint, float *arm_5_joint, float *arm_6_joint,
		float *get_score){

    std::vector<float> ik_joints;

    ik_joints.push_back(*arm_1_joint);
    ik_joints.push_back(*arm_2_joint);
    ik_joints.push_back(*arm_3_joint);
    ik_joints.push_back(*arm_4_joint);
    ik_joints.push_back(*arm_5_joint);
    ik_joints.push_back(*arm_6_joint);

    playful_kinematics::set_kinematics_joints(ik_joints);
    std::vector<float> get_posture;
    bool success = playful_kinematics::ik(left,target_x,target_y,target_z,
					  target_alpha,target_gamma,target_beta,
					  get_posture,*get_score);

    *arm_1_joint = get_posture[0];
    *arm_2_joint = get_posture[1];
    *arm_3_joint = get_posture[2];
    *arm_4_joint = get_posture[3];
    *arm_5_joint = get_posture[4];
    *arm_6_joint = get_posture[5];

    return success;

  }		


  bool ik_5dofs(bool left, float target_x, float target_y, float target_z, 
		float target_alpha, float target_gamma, float target_beta, 
		float *arm_1_joint, float *arm_2_joint, float *arm_3_joint, 
		float *arm_4_joint, float *arm_5_joint,
		float *get_score){
    
    std::vector<float> ik_joints;

    ik_joints.push_back(*arm_1_joint);
    ik_joints.push_back(*arm_2_joint);
    ik_joints.push_back(*arm_3_joint);
    ik_joints.push_back(*arm_4_joint);
    ik_joints.push_back(*arm_5_joint);

    playful_kinematics::set_kinematics_joints(ik_joints);
    std::vector<float> get_posture;
    bool success = playful_kinematics::ik(left,target_x,target_y,target_z,
					  target_alpha,target_gamma,target_beta,
					  get_posture,*get_score);

    *arm_1_joint = get_posture[0];
    *arm_2_joint = get_posture[1];
    *arm_3_joint = get_posture[2];
    *arm_4_joint = get_posture[3];
    *arm_5_joint = get_posture[4];

    return success;

  }		


}

