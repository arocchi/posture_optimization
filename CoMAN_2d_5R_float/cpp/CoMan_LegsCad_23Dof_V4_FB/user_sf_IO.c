/*===========================================================================*
  *
  *  user_sf_IO.c
  *	
  *  Project:	CoMan_LegsCad_23Dof_V4_FB
  * 
  *  Generation date: 22-Jun-2012 19:07:38
  * 
  *  (c) Universite catholique de Louvain
  *      Département de Mécanique 
  *      Unité de Production Mécanique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#include "MBSfun.h" 
#include "user_sf_IO.h" 
#include "sfdef.h" 
#include "userDef.h"



UserIOStruct * initUserIO(MBSdataStruct *s)
{
	UserIOStruct *uvs;
	int i=0;
	//
	uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));
	

	// Control //
	for (i=1;i<=23;i++)
	{
		uvs->Control[i] = 0.0;
	}

	// MotorPosition //
	for (i=1;i<=23;i++)
	{
		uvs->MotorPosition[i] = 0.0;
	}

	// MotorVelocity //
	for (i=1;i<=23;i++)
	{
		uvs->MotorVelocity[i] = 0.0;
	}

	// JointTorque //
	for (i=1;i<=23;i++)
	{
		uvs->JointTorque[i] = 0.0;
	}

	// Ground_Reaction_Right_FT //
	for (i=1;i<=6;i++)
	{
		uvs->Ground_Reaction_Right_FT[i] = 0.0;
	}

	// Ground_Reaction_Right_Pos //
	for (i=1;i<=3;i++)
	{
		uvs->Ground_Reaction_Right_Pos[i] = 0.0;
	}

	// Ground_Reaction_Left_FT //
	for (i=1;i<=6;i++)
	{
		uvs->Ground_Reaction_Left_FT[i] = 0.0;
	}

	// Ground_Reaction_Left_Pos //
	for (i=1;i<=3;i++)
	{
		uvs->Ground_Reaction_Left_Pos[i] = 0.0;
	}


	return uvs;
}

void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{



	free(uvs);

}

#ifndef CMEX
void sf_set_user_input_sizes(SimStruct *S, MBSdataStruct *MBSdata, int sf_ninput) 
{ 
	if (SF_N_USER_INPUT > 0) { // warning: index starts at sf_ninput 
        // example: ssSetInputPortWidth(S,sf_ninput,10); 
 
		/* User input port0 : Control */ 
		ssSetInputPortWidth(S, sf_ninput, 23); 
		ssSetInputPortDirectFeedThrough(S, sf_ninput, 1); 
	} 
} 

void sf_set_user_output_sizes(SimStruct *S, MBSdataStruct *MBSdata) 
        // example: ssSetOutputPortWidth(S, SF_NOUTPUT, 10); 
{ 
	if (SF_N_USER_OUTPUT > 0) { // warning: index starts at SF_NOUTPUT 

		/* User output port0 : MotorPosition */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT, 23); 

		/* User output port1 : MotorVelocity */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+1, 23); 

		/* User output port2 : JointTorque */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+2, 23); 

		/* User output port3 : Ground_Reaction_Right_FT */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+3, 6); 

		/* User output port4 : Ground_Reaction_Right_Pos */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+4, 3); 

		/* User output port5 : Ground_Reaction_Left_FT */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+5, 6); 

		/* User output port6 : Ground_Reaction_Left_Pos */ 
		ssSetOutputPortWidth(S, SF_NOUTPUT+6, 3); 
	} 
} 

void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput) 
{ 
    // warning: index starts at sf_ninput
    // example: InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);
    //          MBSdata->user_IO->var1 = *uPtrs0[0];
    int i;
	InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput); 

	/* User input port0 : Control */ 
   if (ssGetInputPortConnected(S,sf_ninput)) 
      for (i=1;i<=23;i++)
          MBSdata->user_IO->Control[i] = *uPtrs0[i-1]; 
} 

void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
    // warning: index starts at SF_NOUTPUT  
    // example: real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
    //          *y0 = MBSdata->user_IO->var1;  
    int i;
	real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
	real_T *y1 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+1); 
	real_T *y2 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+2); 
	real_T *y3 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+3); 
	real_T *y4 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+4); 
	real_T *y5 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+5); 
	real_T *y6 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+6); 

	/* User output port0 : MotorPosition */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT)) 
      for (i=1;i<=23;i++)
          y0[i-1] = MBSdata->user_IO->MotorPosition[i]; 

	/* User output port1 : MotorVelocity */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+1)) 
      for (i=1;i<=23;i++)
          y1[i-1] = MBSdata->user_IO->MotorVelocity[i]; 

	/* User output port2 : JointTorque */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+2)) 
      for (i=1;i<=23;i++)
          y2[i-1] = MBSdata->user_IO->JointTorque[i]; 

	/* User output port3 : Ground_Reaction_Right_FT */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+3)) 
      for (i=1;i<=6;i++)
          y3[i-1] = MBSdata->user_IO->Ground_Reaction_Right_FT[i]; 

	/* User output port4 : Ground_Reaction_Right_Pos */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+4)) 
      for (i=1;i<=3;i++)
          y4[i-1] = MBSdata->user_IO->Ground_Reaction_Right_Pos[i]; 

	/* User output port5 : Ground_Reaction_Left_FT */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+5)) 
      for (i=1;i<=6;i++)
          y5[i-1] = MBSdata->user_IO->Ground_Reaction_Left_FT[i]; 

	/* User output port6 : Ground_Reaction_Left_Pos */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+6)) 
      for (i=1;i<=3;i++)
          y6[i-1] = MBSdata->user_IO->Ground_Reaction_Left_Pos[i]; 
} 
#endif
