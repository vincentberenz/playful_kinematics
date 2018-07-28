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


#include <limits>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>


namespace playful_kinematics {


  /**
   * Minimize the input posture such as minimizing the scoring function
   * using gradient descent.
   * @param posture posture to be mimized
   * @param min min acceptable for the minimized posture
   * @param max max acceptable for the minimized posture
   * @param target_score the algorithm will exit once the score below this value
   * @param max_step starting 'big' step of the gradient descent
   * @param min_step the step of the gradient descent will decrease to this value
   * @param max_iteration the algorithm will exit early if the max number of iteration is reached
   * @param score scoring function
   * @param final_score score reached when the alorithm exits
   */
  bool minimize(std::vector<float> &posture, 
		std::map<int,float> min,
		std::map<int,float> max, 
		float target_score, 
		float max_step, 
		float min_step, 
		int max_iteration,  
		float(*score)(std::vector<float>&),
		float &final_score
		);


   /**
   * Minimize the input posture such as minimizing the scoring function
   * using gradient descent. The "minimization_priority" parameter allows
   * to specify which dimension of the posture should be minimized first,
   * which has an impact in which solution will be found
   * @param posture posture to be mimized
   * @param mimimization_priority, e.g [1,2,2,2] will favor changing the value of the first dimension
   * @param min min acceptable for the minimized posture
   * @param max max acceptable for the minimized posture
   * @param target_score the algorithm will exit once the score below this value
   * @param max_step starting 'big' step of the gradient descent
   * @param min_step the step of the gradient descent will decrease to this value
   * @param max_iteration the algorithm will exit early if the max number of iteration is reached
   * @param score scoring function
   * @param final_score score reached when the alorithm exits
   */
  bool minimize(std::vector<float> &posture, 
		std::vector<int> minimization_priority,
		std::map<int,float> min,
		std::map<int,float> max, 
		float target_score, 
		float max_step, 
		float min_step, 
		int max_iteration,  
		float(*score)(std::vector<float>&),
		float &final_score
		);

}
