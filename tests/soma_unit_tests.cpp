#include "playful_kinematics/soma.h"
#include "gtest/gtest.h"


class SOMA_tests : public ::testing::Test {

protected:
  void SetUp() {}
  void TearDown() {}
};


static float abs_score_function(std::vector<float> &posture){

  float sum = 0;

  for(int i=0;i<posture.size();i++){
    sum += std::abs(posture[i]);
  }
  
  return sum;

}

static float targeting_one_score_function(std::vector<float> &posture){

  float min_val = std::numeric_limits<float>::max();
  
  for(int i=0;i<posture.size();i++){
    float v = std::abs(posture[i]-1);
    if(v<min_val){
      min_val = v;
    }
  }

  return min_val;
}


static float targeting_sum_sixteen_score_function(std::vector<float> &posture){

  float sum = 0;

  for(int i=0;i<posture.size();i++){
    sum += std::abs(posture[i]);
  }
  
  return std::abs(sum-16.0);

}



TEST_F(SOMA_tests, basic_test){

  std::vector<float> posture;
  posture.push_back(1.0);
  posture.push_back(-10.0);
  posture.push_back(3.1);

  std::map<int,float> min;
  min[0]=-10;
  min[1]=-10;
  min[2]=1; // !! so [0,0,1] expected as output

  std::map<int,float> max;
  max[0]=10;
  max[1]=10;
  max[2]=10;

  float target_score = 0.001;
  float max_step = 0.1;
  float min_step = target_score;
  int max_iterations = 10;
  float final_score;

  bool success = playful_kinematics::minimize(posture,min,max,
				target_score,max_step,min_step,
				max_iterations,&abs_score_function,final_score);

  ASSERT_NEAR(posture[0],0.0,target_score);
  ASSERT_NEAR(posture[1],0.0,target_score);
  ASSERT_NEAR(posture[2],1.0,target_score);  

}


TEST_F(SOMA_tests, minimization_order_test){

  std::vector<float> posture;
  posture.push_back(8.0);
  posture.push_back(8.0);
  posture.push_back(8.0);

  std::map<int,float> min;
  min[0]=-10;
  min[1]=-10;
  min[2]=-10;

  std::map<int,float> max;
  max[0]=10;
  max[1]=10;
  max[2]=10;

  std::vector<int> minimization_priority;
  minimization_priority.push_back(2);
  minimization_priority.push_back(2);
  minimization_priority.push_back(1);
  
  float target_score = 0.001;
  float max_step = 0.1;
  float min_step = target_score;
  int max_iterations = 10;
  float final_score;

  bool success = playful_kinematics::minimize(posture,minimization_priority,
					      min,max,
					      target_score,max_step,min_step,
					      max_iterations,&targeting_one_score_function,final_score);

  ASSERT_NEAR(posture[0],8.0,target_score);
  ASSERT_NEAR(posture[1],8.0,target_score);
  ASSERT_NEAR(posture[2],1.0,target_score);  


}

TEST_F(SOMA_tests, minimization_order_test_2){

  std::vector<float> posture;
  posture.push_back(8.0);
  posture.push_back(8.0);
  posture.push_back(8.0);
  posture.push_back(8.0);

  std::map<int,float> min;
  min[0]=0;
  min[1]=0;
  min[2]=0;
  min[3]=0;

  std::map<int,float> max;
  max[0]=10;
  max[1]=10;
  max[2]=10;
  max[3]=10;

  std::vector<int> minimization_priority;
  minimization_priority.push_back(1);
  minimization_priority.push_back(2);
  minimization_priority.push_back(1);
  minimization_priority.push_back(2);
  
  float target_score = 0.001;
  float max_step = 0.1;
  float min_step = target_score;
  int max_iterations = 10;
  float final_score;

  bool success = playful_kinematics::minimize(posture,minimization_priority,
					      min,max,
					      target_score,max_step,min_step,
					      max_iterations,&targeting_sum_sixteen_score_function,final_score);

  ASSERT_NEAR(posture[0],0.0,target_score);
  ASSERT_NEAR(posture[1],8.0,target_score);
  ASSERT_NEAR(posture[2],0.0,target_score);
  ASSERT_NEAR(posture[3],8.0,target_score);  


}


TEST_F(SOMA_tests, minimization_order_test_3){

  std::vector<float> posture;
  posture.push_back(8.0);
  posture.push_back(8.0);
  posture.push_back(8.0);
  posture.push_back(8.0);

  std::map<int,float> min;
  min[0]=2;
  min[1]=0;
  min[2]=0;
  min[3]=0;

  std::map<int,float> max;
  max[0]=10;
  max[1]=10;
  max[2]=10;
  max[3]=10;

  std::vector<int> minimization_priority;
  minimization_priority.push_back(1);
  minimization_priority.push_back(2);
  minimization_priority.push_back(1);
  minimization_priority.push_back(3);
  
  float target_score = 0.001;
  float max_step = 0.1;
  float min_step = target_score;
  int max_iterations = 10;
  float final_score;

  bool success = playful_kinematics::minimize(posture,minimization_priority,
					      min,max,
					      target_score,max_step,min_step,
					      max_iterations,&targeting_sum_sixteen_score_function,final_score);

  ASSERT_NEAR(posture[0],2.0,target_score);
  ASSERT_NEAR(posture[1],6.0,target_score);
  ASSERT_NEAR(posture[2],0.0,target_score);
  ASSERT_NEAR(posture[3],8.0,target_score);  


}

