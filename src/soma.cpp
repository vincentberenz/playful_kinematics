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



#include "playful_kinematics/soma.h"

namespace playful_kinematics {

  
  static void print_posture(std::vector<float> &posture, float score){

    for(int i=0;i<posture.size();i++) std::cout << posture[i] << "\t";
    std::cout << "\t| "<< score << std::endl;

  }

  
  static float _get_score(std::vector<float> posture,int index, float step,
			  float(*score)(std::vector<float>&)){

    posture[index]+=step;
    float s = score(posture);
    return s;

  }

  
  static bool _minimize(std::vector<float> &posture, int index,
			float min, float max, float step,
			float target_score, float(*score)(std::vector<float>&)){

    float current_score = score(posture);
    float new_score = current_score;

    while(true){

      posture[index]+=step;

      if(posture[index]>max){ 
	posture[index]-=step; 
	return false;
      }

      if(posture[index]<min) {
	posture[index]-=step;
	return false;
      }

      new_score = score(posture);
      
      if (new_score>current_score){
	posture[index]-=step;
	return false;
      }
      
      current_score = new_score;

      if (current_score<=target_score) {
	return true;
      }

    }

  }


  static bool _select_best(std::vector<float> posture,
			   std::vector<int> &indexes,
			   std::map<int,float> &min,
			   std::map<int,float> &max,
			   float step,
			   float target_score,
			   float(*score)(std::vector<float>&),
			   int &get_index,
			   float &get_sign){

    float current_score = score(posture);
    float start_score = current_score;
    float best_score = current_score;
    get_index = 0;
    float score_plus,score_minus;

    for(int i=0;i<indexes.size();i++){

      int index = indexes[i];
      
      score_plus = std::numeric_limits<float>::max();
      score_minus = std::numeric_limits<float>::max();

      if ( (posture[index]+step) < max[index] ){
	score_plus = _get_score(posture,index,+step,score);
      }

      if ( (posture[index]-step) > min[index] ) {
	score_minus = _get_score(posture,index,-step,score);
      }

      if (score_plus<best_score) {
	best_score = score_plus;
	get_index = index;
	get_sign = +1.0;
      }

      if(score_minus<best_score && score_minus<score_plus){
	best_score=score_minus;
	get_index=index;
	get_sign = -1.0;
      }
      
    }

    if (std::abs(best_score-start_score)>(target_score/10.0)) {
      return true;
    }
    
    return false;

  }


  bool _minimize_step(std::vector<float> &posture,
		      std::vector< std::vector<int> > &minimization_order,
		      std::map<int,float> &min,
		      std::map<int,float> &max, 
		      float target_score,
		      float step,
		      float min_step,
		      int max_iterations, 
		      float(*score)(std::vector<float>&),
		      float &final_score ){

    float current_score = score(posture);
    float new_score;
    int iteration = 0;
    float sign;
    int index;
    bool success;
    bool found_better=false;
    int minimization_index = 0;
    int starting_minimization_index = minimization_index;

    while (true) {

      while (!found_better){

	found_better = _select_best(posture, minimization_order[minimization_index],
				  min, max,
				  step, target_score, score, index, sign);

	if(!found_better){
	  minimization_index++;
	  if(minimization_index>=minimization_order.size()){
	    minimization_index=0;
	  }
	  if(minimization_index==starting_minimization_index){
	    return false;
	  }
	}

      }

      found_better = false;
      minimization_index=0;
      starting_minimization_index = minimization_index;
      
      success = _minimize(posture,index,min[index],max[index],sign*step,target_score,score);
      if (success) {
	final_score = score(posture);
	return true;
      }
      
      iteration++;

      if (iteration>=(max_iterations*posture.size())) {
	return false;
      }

      new_score = score(posture);
      current_score=new_score;
      final_score = current_score;

    }

  }

  
  static std::vector< std::vector<int> > _minimization_order(std::vector<int> minimization_priority){

    std::map< int , std::vector<int> > priority_indexes;
    
    for (int i=0;i<minimization_priority.size();i++){
      priority_indexes[minimization_priority[i]].push_back(i);
    }

    std::vector<int> priorities;
    for(std::map< int, std::vector<int> >::iterator it = priority_indexes.begin(); it != priority_indexes.end(); ++it) {
      priorities.push_back(it->first);
    }

    std::sort(priorities.begin(), priorities.end());  

    std::vector< std::vector<int> > minimization_order;
    
    for(int i=0;i<priorities.size();i++){
      minimization_order.push_back(priority_indexes[priorities[i]]);
    }

    return minimization_order;
    
  }

  
  bool minimize(std::vector<float> &posture,
		std::vector<int> minimization_priority,
		std::map<int,float> min,
		std::map<int,float> max,
		float target_score,
		float max_step,
		float min_step,
		int max_iterations, float(*score)(std::vector<float>&),
		float &final_score){

    std::vector< std::vector<int> > minimization_order = _minimization_order(minimization_priority);

    bool success;
    float step = max_step;

    while (step>=(min_step/2.0)){

      success = _minimize_step(posture, minimization_order,
			       min, max,
			       target_score,
			       step, min_step,
			       max_iterations, score, final_score);

      if (success) {
	return true;
      }

      step = step/10.0;

    }

    return false;

  }


  bool minimize(std::vector<float> &posture,
		std::map<int,float> min,
		std::map<int,float> max,
		float target_score,
		float max_step,
		float min_step,
		int max_iterations, float(*score)(std::vector<float>&),
		float &final_score){

    std::vector<int> minimization_priority;

    for(int i=0;i<posture.size();i++){
      minimization_priority.push_back(1);
    }

    minimize(posture,
	     minimization_priority,
	     min,max,
	     target_score,
	     max_step,min_step,
	     max_iterations,
	     score,
	     final_score);

  }

  

}
