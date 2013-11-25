/*===========================================================================*
  *
  *  user_sf_IO.h
  *	
  *  Project:	CoMan_LegsCad_23Dof_V4_FB
  * 
  *  Generation date: 22-Jun-2012 19:07:47
  * 
  *  (c) Universite catholique de Louvain
  *      Département de Mécanique 
  *      Unité de Production Mécanique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/
 
#ifdef ACCELRED 
#define S_FUNCTION_NAME  mbs_sf_accelred_CoMan_LegsCad_23Dof_V4_FB 
#elif defined DIRDYNARED 
#define S_FUNCTION_NAME  mbs_sf_dirdynared_CoMan_LegsCad_23Dof_V4_FB 
#elif defined INVDYNARED 
#define S_FUNCTION_NAME  mbs_sf_invdynared_CoMan_LegsCad_23Dof_V4_FB 
#elif defined SENSORKIN 
#define S_FUNCTION_NAME  mbs_sf_sensorkin_CoMan_LegsCad_23Dof_V4_FB 
#endif 
 
#define SF_N_USER_INPUT 1 
#define SF_N_USER_OUTPUT 7 

#include "userDef.h"
 
typedef struct UserIOStruct 
{

    double Control[23+1];
    double MotorPosition[23+1];
    double MotorVelocity[23+1];
    double JointTorque[23+1];
    double Ground_Reaction_Right_FT[6+1];
    double Ground_Reaction_Right_Pos[3+1];
    double Ground_Reaction_Left_FT[6+1];
    double Ground_Reaction_Left_Pos[3+1];

} UserIOStruct;

/*--------------------*/
#endif
