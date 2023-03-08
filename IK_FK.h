#ifndef CALIBRATE_AND_IK_FK_H_
#define CALIBRATE_AND_IK_FK_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <math.h>
#include <vector>
#include <array>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/ 

/* Orientation angles for the X-axis extrinsic rotation*/
static float const ALPHA_120 = 120.0  * M_PI / 180.0;
static float const ALPHA_150 = 150.0  * M_PI / 180.0;
static float const ALPHA_180 = 180.0  * M_PI / 180.0; 

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

/* Joint offsets set during calibration */
static float JOINT_OFFSETS[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} 

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/ 

/* get the XYZ position of the tip of the gripper for a given set of joint angles*/
float* forward_kinematics(const float* joint_angles);  

#endif /* CALIBRATE_AND_IK_FK_H_ */