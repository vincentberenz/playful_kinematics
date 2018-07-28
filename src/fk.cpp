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

 
using namespace KDL;

static std::string first_left_link = FIRST_LEFT_LINK;
static std::string last_left_link = LAST_LEFT_LINK;

static std::string first_right_link = FIRST_RIGHT_LINK;
static std::string last_right_link = LAST_RIGHT_LINK;


namespace playful_kinematics{

  
  /* BACK END FUNCTIONS AND CLASSES */
  
  
  class RobotChain {

  public:
    RobotChain(std::string first_link, std::string last_link, KDL::Tree &tree);
    ~RobotChain();
    KDL::Chain arm;
    ChainFkSolverPos_recursive *fksolver;
  };

  RobotChain::RobotChain(std::string first_link, std::string last_link,KDL::Tree &tree){
    tree.getChain(first_link,last_link,this->arm);
    this->fksolver = new ChainFkSolverPos_recursive(this->arm);
  }

  RobotChain::~RobotChain(){
    delete this->fksolver;
  }


  class robot_kinematics {

  private:

    static const std::string urdf;
    RobotChain *left_arm;
    RobotChain *right_arm;
    KDL::Tree tree;

  public:

    robot_kinematics();
    ~robot_kinematics();
    int get_nb_joints(const bool left);
    std::string get_joint_name(bool left, int index);
    void print_segments();
    void print_segments(bool left);
    bool run_forward_kinematics(const bool left,
				const double *joints,
				double *translation,
				double *euler_rotation);
    bool run_forward_kinematics(const bool left,
				const double *joints,
				double *x, double *y, double *z,
				double *alpha, double *beta, double *gamma);

  };


  const std::string robot_kinematics::urdf = URDF_PATH;

  
  robot_kinematics::robot_kinematics(){

    kdl_parser::treeFromFile(this->urdf,this->tree);
    this->left_arm = new RobotChain(std::string(first_left_link),
				    std::string(last_left_link),this->tree);
    this->right_arm = new RobotChain(std::string(first_right_link),
				     std::string(last_right_link),this->tree);

  }

  
  robot_kinematics::~robot_kinematics(){

    delete this->left_arm;
    delete this->right_arm;

  }

  
  int robot_kinematics::get_nb_joints(const bool left) {

    if(left) return this->left_arm->arm.getNrOfJoints();
    return this->right_arm->arm.getNrOfJoints();

  }


  void robot_kinematics::print_segments() {

    SegmentMap sm = this->tree.getSegments();

    for(std::map<std::string,TreeElement>::iterator it = sm.begin(); it != sm.end(); ++it) {
      std::cout << std::endl;
      std::cout << it->first << std::endl;
      std::cout << std::endl;
    }
    
  }

  
  void robot_kinematics::print_segments(bool left){
    
    RobotChain *chain;
    if(left) chain = this->left_arm;
    else chain = this->right_arm;

    int nb = chain->arm.getNrOfSegments();
    std::cout << "number of segments: " << nb << std::endl;
    std::cout << "number of joints: " << this->get_nb_joints(left) << std::endl;
    for(int i=0;i<nb;i++){
      Segment segment = chain->arm.getSegment(i);
      std::string segment_name = segment.getName();
      Joint joint  = segment.getJoint();
      std::string joint_name = joint.getName();
      std::cout << segment_name << "\t" << joint_name << std::endl;
    }
    
  }

  
  std::string robot_kinematics::get_joint_name(bool left,int index) {

    if (index>=this->get_nb_joints(left)) return "";

    SegmentMap sm = this->tree.getSegments();

    int count=0;
    for(std::map<std::string,TreeElement>::iterator it = sm.begin(); it != sm.end(); ++it) {
      if (count == index) return it->first;
      count++;
    }

    return "";

  }

  
  bool robot_kinematics::run_forward_kinematics(const bool left, const double *joints, double *translation, double *euler_rotation){

    static KDL::Frame cartesian;    

    int nb = this->get_nb_joints(left);
    JntArray q(nb);
    for(unsigned int i=0;i<nb;i++) q(i)=joints[i];
    
    int success;

    if (left) success = this->left_arm->fksolver->JntToCart(q,cartesian);
    else success = this->right_arm->fksolver->JntToCart(q,cartesian);

    if (success>=0){

      for(int i=0;i<3;i++) translation[i]=cartesian.p[i];      

      cartesian.M.GetRPY(euler_rotation[0],euler_rotation[1],euler_rotation[2]);
      
      return true;

    }

    return false;

  }


  bool robot_kinematics::run_forward_kinematics(const bool left,
						const double *joints,
						double *x, double *y, double *z,
						double *alpha, double *beta, double *gamma){

    static double xyz[3];
    static double euler[3];

    xyz[0]=*x;
    xyz[1]=*y;
    xyz[2]=*z;
    euler[0]=*alpha;
    euler[1]=*beta;
    euler[2]=*gamma;

    bool success = this->run_forward_kinematics(left,joints,xyz,euler);

    if (success) {
      *x=xyz[0];
      *y=xyz[1];
      *z=xyz[2];
      *alpha = euler[0];
      *beta = euler[1];
      *gamma = euler[2];
    }
    
    return success;

  }

  /* END OF BACK END FUNCTIONS */
  

  /* FRONT END FUNCTIONS */

  
  bool forward_kinematics(bool left,
			  std::vector<float> &posture,
			  std::vector<float> &get_position,
			  std::vector<float> &get_orientation){

    static robot_kinematics robot; 
    static double q[NB_JOINTS];
    
    double x,y,z,alpha,beta,gamma;

    for(int i=0;i<NB_JOINTS;i++) q[i]=(double)posture[i];

    bool success = robot.run_forward_kinematics(left,q,&x,&y,&z,&alpha,&beta,&gamma);

    get_position.push_back((float)x);
    get_position.push_back((float)y);
    get_position.push_back((float)z);
    get_orientation.push_back((float)alpha);
    get_orientation.push_back((float)beta);
    get_orientation.push_back((float)gamma);

    return success;

  }

  
  bool forward_kinematics(bool left,
			  double *q,
			  double *x, double *y, double *z,
			  double *alpha, double *beta, double *gamma){

    static robot_kinematics robot;
    return robot.run_forward_kinematics(left,q,x,y,z,alpha,beta,gamma);

  }


  /* END OF FRONT END FUNCTIONS */

}


/* INTERFACE FOR PYTHON WRAPPER */


extern "C" {


  int get_nb_joints(bool left){

    playful_kinematics:: robot_kinematics robot;
    return robot.get_nb_joints(left);

  }

  bool forward_kinematics(bool left, double *q,
			  double *x, double *y, double *z,
			  double *alpha, double *beta, double *gamma){

    static playful_kinematics::robot_kinematics robot; 
    return robot.run_forward_kinematics(left,q,x,y,z,alpha,beta,gamma);

  }

  
}



