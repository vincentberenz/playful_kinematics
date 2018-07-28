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



#include "playful_kinematics/kinematic_config.h"

namespace playful_kinematics {


  class kinematics_configuration {

  public :

    bool left;

    std::vector<float> reference_ik_joints;
    std::vector<bool> mask;
    std::map<int,int> minimization_priority;
    std::map<int,float> min;
    std::map<int,float> max;
    
    int nb_joints;

    void set_side(bool left); 
    void set_mask(std::vector<bool> max);
    void set_min_max(int index,float min,float max);
    void set_kinematics_joints(std::vector<float> reference_ik_joints);
    void set_minimization_priority(int index, int priority);
    
  };

  
  void kinematics_configuration::set_side(bool left){ 
    this->left=left;
  }

  
  void kinematics_configuration::set_mask(std::vector<bool> mask){ 
    this->mask=mask;
  }

  
  void kinematics_configuration::set_min_max(int index,float min,float max){
    this->min[index]=min;
    this->max[index]=max;
  }

  
  void kinematics_configuration::set_kinematics_joints(std::vector<float> reference_ik_joints){
    this->nb_joints = reference_ik_joints.size();
    this->reference_ik_joints = reference_ik_joints;
  }


  void kinematics_configuration::set_minimization_priority(int index, int priority){
    this->minimization_priority[index] = priority;
  }

  static boost::shared_ptr<kinematics_configuration> playful_kinematics_config;

  
  void set_kinematics_joints(std::vector<float> reference_ik_joints){

    if(!playful_kinematics_config) {
      playful_kinematics_config.reset(new kinematics_configuration());
    }
    playful_kinematics_config->set_kinematics_joints(reference_ik_joints);

  }

  
  void set_kinematics_side(bool left){

    if(!playful_kinematics_config) {
      playful_kinematics_config.reset(new kinematics_configuration());
    }
    
    playful_kinematics_config->set_side(left);

  }


  void set_kinematics_mask(std::vector<bool> mask){

    if(!playful_kinematics_config) {
      playful_kinematics_config.reset(new kinematics_configuration());
    }

    playful_kinematics_config->set_mask(mask);

  }


  void set_kinematics_joint_limit(int index, float min, float max){

    if(!playful_kinematics_config){
      playful_kinematics_config.reset(new kinematics_configuration());
    }
    
    playful_kinematics_config->set_min_max(index,min,max);

  }


  void set_minimization_priority(int index, int priority){

    if(!playful_kinematics_config){
      playful_kinematics_config.reset(new kinematics_configuration());
    }
    
    playful_kinematics_config->set_minimization_priority(index,priority);
    
  }


  bool get_kinematics_side(){

    return playful_kinematics_config->left;

  }

  
  void get_kinematics_joints(std::vector<float> &get){

    for(int i=0;i<playful_kinematics_config->nb_joints;i++){
      get.push_back(playful_kinematics_config->reference_ik_joints[i]);
    }

  }


  std::vector<int> get_minimization_priority(int size){

    std::vector<int> r;

    for(int i=0;i<size;i++){
      if ( playful_kinematics_config->minimization_priority.find(i) == playful_kinematics_config->minimization_priority.end() ) {
	r.push_back(1);
      } else {
	r.push_back(playful_kinematics_config->minimization_priority[i]);
      }
    }

    return r;
    
  }

  
  std::vector<bool>& get_kinematics_mask(){
    return playful_kinematics_config->mask;
  }

  
  std::map<int,float>& get_kinematics_joint_min_limit(){
    return playful_kinematics_config->min;
  }

  
  std::map<int,float>& get_kinematics_joint_max_limit(){
    return playful_kinematics_config->max;
  }

  
  int get_kinematics_nb_joints(){
    return playful_kinematics_config->nb_joints;
  }


}


/* INTERFACE FOR PYTHON WRAPPER */


extern "C" {


  void set_minimization_priority(int index, int priority){

    playful_kinematics::set_minimization_priority(index,priority);
    
  }

  
  void set_kinematics_side(bool left){

    playful_kinematics::set_kinematics_side(left);

  }

  
  void set_kinematics_mask(bool *mask){

    std::vector<bool> bmask;
    for(int i=0;i<6;i++) bmask[i] = mask[i];
    playful_kinematics::set_kinematics_mask(bmask);

  }

  
  void set_kinematics_joint_limit(int index, float min, float max){

    playful_kinematics::set_kinematics_joint_limit(index,min,max);

  }

  
  void set_kinematics_joints(int nb_joints,
			     float * reference_ik_joints){

    std::vector<float> ik;
    for(int i=0;i<nb_joints;i++) ik.push_back(reference_ik_joints[i]);
    playful_kinematics::set_kinematics_joints(ik);

  }

}

