/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <Braccio++.h>
#include "Helper_Tools.h"
#include "IK_FK.h" 
#include <math.h>
/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/
/* Length of links in centimeters: can be obtained from measureing, CAD model, or URDF 
 * file of robot */
const float L7 =  0.0; // Ground to base (M6)
const float L6 =  7.2; // base (M6) to shoulder (M5) 
const float L5 = 12.5; // shoulder (M5) to elbow (M4)
const float L4 = 12.5; // elbow (M4) to wrist pitch (M3)
const float L3 =  6.0; // wrist pitch (M3) to wrist roll (M2)
const float L2 = 13.5; // wrist roll (M2) to tip of gripper // 13.0
const float L23 = L2 + L3; // wrist pitch (M3) to tip of gripper 

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/ 

// forward_kinematics function from the previous week
/* returns a pointer to an array of the XYZ position of the gripper for a given array of joint angles*/
float* forward_kinematics(const float* joint_angles){
  // creates a pointer of an array of the XYZ position of the gripper 
  float* pos = new float[3];
  /* converts joint angles given in Braccio's angle ranges (with a mid value of 157.5 degrees) to 
  * the expected angle ranges (with a mid value of 180.0 degrees)*/
  float* thetas = joint_angles_to_thetas(joint_angles);

  /* YOUR CODE GOES HERE:  START */ 
  /* REMEMBER: convert angle values from degrees to radians eg. */
  float theta6 = degrees_to_radians(thetas[5]); // Angle of base motor (M6)
  float beta3, beta4, beta5;
  /* update beta values */
  beta5 = degrees_to_radians(180.0 - thetas[4]);
  beta4 = beta5 + degrees_to_radians(180.0 - thetas[3]);
  beta3 = beta4 + degrees_to_radians(180.0 - thetas[2]);
  /* update position values: */
  pos[0] = L5*sin(beta5)*cos(theta6) + L4*sin(beta4)*cos(theta6) + L23*sin(beta3)*cos(theta6);
  pos[1] = L5*sin(beta5)*sin(theta6) + L4*sin(beta4)*sin(theta6) + L23*sin(beta3)*sin(theta6);
  pos[2] = L7 + L6 + L5*cos(beta5) + L4*cos(beta4) + L23*cos(beta3);  
    
  /* YOUR CODE GOES HERE:  END */
  return pos;
} 
