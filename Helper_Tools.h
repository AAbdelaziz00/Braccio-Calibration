#ifndef CALIBRATE_AND_HELPER_TOOLS_H_
#define CALIBRATE_AND_HELPER_TOOLS_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <math.h>
#include <limits>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/ 
/* Joint angle values that get the robotic arm in its upright position with a closed gripper
 and the base joint at its zero value*/
static float const HOME_POSITION[6] =     {157.5, 157.5, 157.5, 157.5, 157.5, 0.0};
/* The (geometrically) expected angle values that get the robotic arm in its upright position 
 with a closed gripper and the base joint at its zero value*/
static float const EXPECTED_HOME_POSITION[6] = {157.5, 180.0, 180.0, 180.0, 180.0, 0.0};


/* Number of possible alpha (radians) angle values to use when performing inverse kinematics
  alpha is the orientation of the gripper produced by a rotation about the (intrinsic) X-axis */
int const ALPHA_NUM = 10;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

/* degrees to radians*/
float degrees_to_radians(float const degree_val){
  return degree_val * M_PI / 180.0;
}

/* radians to degrees*/
float radians_to_degrees(float const radian_val){
  return radian_val * 180.0 / M_PI;
}

/* convert angles from Braccio to expected angle values 
  i.e. HOME_POSITION of shoulder is expected to be 180.0 degrees and not 157.5 degrees*/
float* joint_angles_to_thetas(const float* joint_angles){
  float* thetas = new float[6];
  for(int i = 0; i < 6; i++){
    thetas[i] = joint_angles[i] + (EXPECTED_HOME_POSITION[i] - HOME_POSITION[i]);
  }
  return thetas;
}

/* convert expected angle values to Braccio angle values 
  i.e. HOME_POSITION of shoulder has an expected angle value of 180.0 degrees 
  and a Braccio joint angle value of 157.5 degrees*/
float* thetas_to_joint_angles(const float thetas[6]){
  float* joint_angles = new float[6];
  for(int i = 0; i < 6; i++){
    joint_angles[i] = thetas[i] + (HOME_POSITION[i] - EXPECTED_HOME_POSITION[i]);
  }
  return joint_angles;
} 
/* returns an array of possible alpha (radians) angle values to use for inverse kinematics
  alpha is the orientation of the gripper produced by a rotation about the (extrinsic) X-axis */
float* get_alpha_angles(){
  float* alphas = new float[ALPHA_NUM];
  float step = 90.0/ALPHA_NUM;
  for(int i = 0; i < ALPHA_NUM; i++){
    alphas[i] = degrees_to_radians(90.0 + step*(i+1));    
  }
  return alphas;
} 

#endif /* CALIBRATE_AND_HELPER_TOOLS_H_ */