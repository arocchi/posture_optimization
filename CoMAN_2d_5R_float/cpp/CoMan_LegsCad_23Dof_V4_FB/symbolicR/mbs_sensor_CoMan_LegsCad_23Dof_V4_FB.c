//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universit� catholique de Louvain 
//	D�partement de M�canique 
//	Unit� de Production M�canique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Tue Jun 12 14:29:34 2012
//
//	==> Project name : CoMan_LegsCad_23Dof_V4_FB
//	==> using XML input file 
//
//	==> Number of joints : 29
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 41949
//
//	==> All Parameter Symbols included
//	==> Generation Time :  0.660 seconds
//	==> Post-Processing :  1.100 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void  sensor(MBSsensorStruct *sens, 
              MBSdataStruct *s,
              int isens)
{ 
 
#include "mbs_sensor_CoMan_LegsCad_23Dof_V4_FB.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

C4 = cos(q[4]);
S4 = sin(q[4]);
C5 = cos(q[5]);
S5 = sin(q[5]);
C6 = cos(q[6]);
S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

C7 = cos(q[7]);
S7 = sin(q[7]);
C8 = cos(q[8]);
S8 = sin(q[8]);
C9 = cos(q[9]);
S9 = sin(q[9]);
C10 = cos(q[10]);
S10 = sin(q[10]);
C11 = cos(q[11]);
S11 = sin(q[11]);
C12 = cos(q[12]);
S12 = sin(q[12]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

C13 = cos(q[13]);
S13 = sin(q[13]);
C14 = cos(q[14]);
S14 = sin(q[14]);
C15 = cos(q[15]);
S15 = sin(q[15]);
C16 = cos(q[16]);
S16 = sin(q[16]);
C17 = cos(q[17]);
S17 = sin(q[17]);
C18 = cos(q[18]);
S18 = sin(q[18]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

C19 = cos(q[19]);
S19 = sin(q[19]);
C20 = cos(q[20]);
S20 = sin(q[20]);
C21 = cos(q[21]);
S21 = sin(q[21]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

C22 = cos(q[22]);
S22 = sin(q[22]);
C23 = cos(q[23]);
S23 = sin(q[23]);
C24 = cos(q[24]);
S24 = sin(q[24]);
C25 = cos(q[25]);
S25 = sin(q[25]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

C26 = cos(q[26]);
S26 = sin(q[26]);
C27 = cos(q[27]);
S27 = sin(q[27]);
C28 = cos(q[28]);
S28 = sin(q[28]);
C29 = cos(q[29]);
S29 = sin(q[29]);

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


ROcp0_25 = S4*S5;
ROcp0_35 = -C4*S5;
ROcp0_85 = -S4*C5;
ROcp0_95 = C4*C5;
ROcp0_16 = C5*C6;
ROcp0_26 = ROcp0_25*C6+C4*S6;
ROcp0_36 = ROcp0_35*C6+S4*S6;
ROcp0_46 = -C5*S6;
ROcp0_56 = -(ROcp0_25*S6-C4*C6);
ROcp0_66 = -(ROcp0_35*S6-S4*C6);
OMcp0_25 = qd[5]*C4;
OMcp0_35 = qd[5]*S4;
OMcp0_16 = qd[4]+qd[6]*S5;
OMcp0_26 = OMcp0_25+ROcp0_85*qd[6];
OMcp0_36 = OMcp0_35+ROcp0_95*qd[6];
OPcp0_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp0_26 = ROcp0_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp0_35*S5-ROcp0_95*qd[4]);
OPcp0_36 = ROcp0_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp0_25*S5-ROcp0_85*qd[4]);
RLcp0_130 = s->dpt[1][1]*ROcp0_16+s->dpt[3][1]*S5+ROcp0_46*s->dpt[2][1];
RLcp0_230 = s->dpt[1][1]*ROcp0_26+s->dpt[3][1]*ROcp0_85+ROcp0_56*s->dpt[2][1];
RLcp0_330 = s->dpt[1][1]*ROcp0_36+s->dpt[3][1]*ROcp0_95+ROcp0_66*s->dpt[2][1];
POcp0_130 = RLcp0_130+q[1];
POcp0_230 = RLcp0_230+q[2];
POcp0_330 = RLcp0_330+q[3];
JTcp0_130_5 = -(RLcp0_230*S4-RLcp0_330*C4);
JTcp0_230_5 = RLcp0_130*S4;
JTcp0_330_5 = -RLcp0_130*C4;
JTcp0_130_6 = -(RLcp0_230*ROcp0_95-RLcp0_330*ROcp0_85);
JTcp0_230_6 = RLcp0_130*ROcp0_95-RLcp0_330*S5;
JTcp0_330_6 = -(RLcp0_130*ROcp0_85-RLcp0_230*S5);
ORcp0_130 = OMcp0_26*RLcp0_330-OMcp0_36*RLcp0_230;
ORcp0_230 = -(OMcp0_16*RLcp0_330-OMcp0_36*RLcp0_130);
ORcp0_330 = OMcp0_16*RLcp0_230-OMcp0_26*RLcp0_130;
VIcp0_130 = ORcp0_130+qd[1];
VIcp0_230 = ORcp0_230+qd[2];
VIcp0_330 = ORcp0_330+qd[3];
ACcp0_130 = qdd[1]+OMcp0_26*ORcp0_330-OMcp0_36*ORcp0_230+OPcp0_26*RLcp0_330-OPcp0_36*RLcp0_230;
ACcp0_230 = qdd[2]-OMcp0_16*ORcp0_330+OMcp0_36*ORcp0_130-OPcp0_16*RLcp0_330+OPcp0_36*RLcp0_130;
ACcp0_330 = qdd[3]+OMcp0_16*ORcp0_230-OMcp0_26*ORcp0_130+OPcp0_16*RLcp0_230-OPcp0_26*RLcp0_130;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp0_130;
sens->P[2] = POcp0_230;
sens->P[3] = POcp0_330;
sens->R[1][1] = ROcp0_16;
sens->R[1][2] = ROcp0_26;
sens->R[1][3] = ROcp0_36;
sens->R[2][1] = ROcp0_46;
sens->R[2][2] = ROcp0_56;
sens->R[2][3] = ROcp0_66;
sens->R[3][1] = S5;
sens->R[3][2] = ROcp0_85;
sens->R[3][3] = ROcp0_95;
sens->V[1] = VIcp0_130;
sens->V[2] = VIcp0_230;
sens->V[3] = VIcp0_330;
sens->OM[1] = OMcp0_16;
sens->OM[2] = OMcp0_26;
sens->OM[3] = OMcp0_36;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp0_130_5;
sens->J[1][6] = JTcp0_130_6;
sens->J[2][2] = (1.0);
sens->J[2][4] = -RLcp0_330;
sens->J[2][5] = JTcp0_230_5;
sens->J[2][6] = JTcp0_230_6;
sens->J[3][3] = (1.0);
sens->J[3][4] = RLcp0_230;
sens->J[3][5] = JTcp0_330_5;
sens->J[3][6] = JTcp0_330_6;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp0_85;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp0_95;
sens->A[1] = ACcp0_130;
sens->A[2] = ACcp0_230;
sens->A[3] = ACcp0_330;
sens->OMP[1] = OPcp0_16;
sens->OMP[2] = OPcp0_26;
sens->OMP[3] = OPcp0_36;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


ROcp1_25 = S4*S5;
ROcp1_35 = -C4*S5;
ROcp1_85 = -S4*C5;
ROcp1_95 = C4*C5;
ROcp1_16 = C5*C6;
ROcp1_26 = ROcp1_25*C6+C4*S6;
ROcp1_36 = ROcp1_35*C6+S4*S6;
ROcp1_46 = -C5*S6;
ROcp1_56 = -(ROcp1_25*S6-C4*C6);
ROcp1_66 = -(ROcp1_35*S6-S4*C6);
OMcp1_25 = qd[5]*C4;
OMcp1_35 = qd[5]*S4;
OMcp1_16 = qd[4]+qd[6]*S5;
OMcp1_26 = OMcp1_25+ROcp1_85*qd[6];
OMcp1_36 = OMcp1_35+ROcp1_95*qd[6];
OPcp1_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp1_26 = ROcp1_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp1_35*S5-ROcp1_95*qd[4]);
OPcp1_36 = ROcp1_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp1_25*S5-ROcp1_85*qd[4]);
RLcp1_131 = s->dpt[1][2]*ROcp1_16+s->dpt[3][2]*S5+ROcp1_46*s->dpt[2][2];
RLcp1_231 = s->dpt[1][2]*ROcp1_26+s->dpt[3][2]*ROcp1_85+ROcp1_56*s->dpt[2][2];
RLcp1_331 = s->dpt[1][2]*ROcp1_36+s->dpt[3][2]*ROcp1_95+ROcp1_66*s->dpt[2][2];
POcp1_131 = RLcp1_131+q[1];
POcp1_231 = RLcp1_231+q[2];
POcp1_331 = RLcp1_331+q[3];
JTcp1_131_5 = -(RLcp1_231*S4-RLcp1_331*C4);
JTcp1_231_5 = RLcp1_131*S4;
JTcp1_331_5 = -RLcp1_131*C4;
JTcp1_131_6 = -(RLcp1_231*ROcp1_95-RLcp1_331*ROcp1_85);
JTcp1_231_6 = RLcp1_131*ROcp1_95-RLcp1_331*S5;
JTcp1_331_6 = -(RLcp1_131*ROcp1_85-RLcp1_231*S5);
ORcp1_131 = OMcp1_26*RLcp1_331-OMcp1_36*RLcp1_231;
ORcp1_231 = -(OMcp1_16*RLcp1_331-OMcp1_36*RLcp1_131);
ORcp1_331 = OMcp1_16*RLcp1_231-OMcp1_26*RLcp1_131;
VIcp1_131 = ORcp1_131+qd[1];
VIcp1_231 = ORcp1_231+qd[2];
VIcp1_331 = ORcp1_331+qd[3];
ACcp1_131 = qdd[1]+OMcp1_26*ORcp1_331-OMcp1_36*ORcp1_231+OPcp1_26*RLcp1_331-OPcp1_36*RLcp1_231;
ACcp1_231 = qdd[2]-OMcp1_16*ORcp1_331+OMcp1_36*ORcp1_131-OPcp1_16*RLcp1_331+OPcp1_36*RLcp1_131;
ACcp1_331 = qdd[3]+OMcp1_16*ORcp1_231-OMcp1_26*ORcp1_131+OPcp1_16*RLcp1_231-OPcp1_26*RLcp1_131;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp1_131;
sens->P[2] = POcp1_231;
sens->P[3] = POcp1_331;
sens->R[1][1] = ROcp1_16;
sens->R[1][2] = ROcp1_26;
sens->R[1][3] = ROcp1_36;
sens->R[2][1] = ROcp1_46;
sens->R[2][2] = ROcp1_56;
sens->R[2][3] = ROcp1_66;
sens->R[3][1] = S5;
sens->R[3][2] = ROcp1_85;
sens->R[3][3] = ROcp1_95;
sens->V[1] = VIcp1_131;
sens->V[2] = VIcp1_231;
sens->V[3] = VIcp1_331;
sens->OM[1] = OMcp1_16;
sens->OM[2] = OMcp1_26;
sens->OM[3] = OMcp1_36;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp1_131_5;
sens->J[1][6] = JTcp1_131_6;
sens->J[2][2] = (1.0);
sens->J[2][4] = -RLcp1_331;
sens->J[2][5] = JTcp1_231_5;
sens->J[2][6] = JTcp1_231_6;
sens->J[3][3] = (1.0);
sens->J[3][4] = RLcp1_231;
sens->J[3][5] = JTcp1_331_5;
sens->J[3][6] = JTcp1_331_6;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp1_85;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp1_95;
sens->A[1] = ACcp1_131;
sens->A[2] = ACcp1_231;
sens->A[3] = ACcp1_331;
sens->OMP[1] = OPcp1_16;
sens->OMP[2] = OPcp1_26;
sens->OMP[3] = OPcp1_36;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


ROcp2_25 = S4*S5;
ROcp2_35 = -C4*S5;
ROcp2_85 = -S4*C5;
ROcp2_95 = C4*C5;
ROcp2_16 = C5*C6;
ROcp2_26 = ROcp2_25*C6+C4*S6;
ROcp2_36 = ROcp2_35*C6+S4*S6;
ROcp2_46 = -C5*S6;
ROcp2_56 = -(ROcp2_25*S6-C4*C6);
ROcp2_66 = -(ROcp2_35*S6-S4*C6);
OMcp2_25 = qd[5]*C4;
OMcp2_35 = qd[5]*S4;
OMcp2_16 = qd[4]+qd[6]*S5;
OMcp2_26 = OMcp2_25+ROcp2_85*qd[6];
OMcp2_36 = OMcp2_35+ROcp2_95*qd[6];
OPcp2_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp2_26 = ROcp2_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp2_35*S5-ROcp2_95*qd[4]);
OPcp2_36 = ROcp2_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp2_25*S5-ROcp2_85*qd[4]);
RLcp2_132 = s->dpt[2][3]*ROcp2_46+ROcp2_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp2_232 = s->dpt[2][3]*ROcp2_56+ROcp2_26*s->dpt[1][3]+ROcp2_85*s->dpt[3][3];
RLcp2_332 = s->dpt[2][3]*ROcp2_66+ROcp2_36*s->dpt[1][3]+ROcp2_95*s->dpt[3][3];
POcp2_132 = RLcp2_132+q[1];
POcp2_232 = RLcp2_232+q[2];
POcp2_332 = RLcp2_332+q[3];
JTcp2_132_5 = -(RLcp2_232*S4-RLcp2_332*C4);
JTcp2_232_5 = RLcp2_132*S4;
JTcp2_332_5 = -RLcp2_132*C4;
JTcp2_132_6 = -(RLcp2_232*ROcp2_95-RLcp2_332*ROcp2_85);
JTcp2_232_6 = RLcp2_132*ROcp2_95-RLcp2_332*S5;
JTcp2_332_6 = -(RLcp2_132*ROcp2_85-RLcp2_232*S5);
ORcp2_132 = OMcp2_26*RLcp2_332-OMcp2_36*RLcp2_232;
ORcp2_232 = -(OMcp2_16*RLcp2_332-OMcp2_36*RLcp2_132);
ORcp2_332 = OMcp2_16*RLcp2_232-OMcp2_26*RLcp2_132;
VIcp2_132 = ORcp2_132+qd[1];
VIcp2_232 = ORcp2_232+qd[2];
VIcp2_332 = ORcp2_332+qd[3];
ACcp2_132 = qdd[1]+OMcp2_26*ORcp2_332-OMcp2_36*ORcp2_232+OPcp2_26*RLcp2_332-OPcp2_36*RLcp2_232;
ACcp2_232 = qdd[2]-OMcp2_16*ORcp2_332+OMcp2_36*ORcp2_132-OPcp2_16*RLcp2_332+OPcp2_36*RLcp2_132;
ACcp2_332 = qdd[3]+OMcp2_16*ORcp2_232-OMcp2_26*ORcp2_132+OPcp2_16*RLcp2_232-OPcp2_26*RLcp2_132;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp2_132;
sens->P[2] = POcp2_232;
sens->P[3] = POcp2_332;
sens->R[1][1] = ROcp2_16;
sens->R[1][2] = ROcp2_26;
sens->R[1][3] = ROcp2_36;
sens->R[2][1] = ROcp2_46;
sens->R[2][2] = ROcp2_56;
sens->R[2][3] = ROcp2_66;
sens->R[3][1] = S5;
sens->R[3][2] = ROcp2_85;
sens->R[3][3] = ROcp2_95;
sens->V[1] = VIcp2_132;
sens->V[2] = VIcp2_232;
sens->V[3] = VIcp2_332;
sens->OM[1] = OMcp2_16;
sens->OM[2] = OMcp2_26;
sens->OM[3] = OMcp2_36;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp2_132_5;
sens->J[1][6] = JTcp2_132_6;
sens->J[2][2] = (1.0);
sens->J[2][4] = -RLcp2_332;
sens->J[2][5] = JTcp2_232_5;
sens->J[2][6] = JTcp2_232_6;
sens->J[3][3] = (1.0);
sens->J[3][4] = RLcp2_232;
sens->J[3][5] = JTcp2_332_5;
sens->J[3][6] = JTcp2_332_6;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp2_85;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp2_95;
sens->A[1] = ACcp2_132;
sens->A[2] = ACcp2_232;
sens->A[3] = ACcp2_332;
sens->OMP[1] = OPcp2_16;
sens->OMP[2] = OPcp2_26;
sens->OMP[3] = OPcp2_36;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


ROcp3_25 = S4*S5;
ROcp3_35 = -C4*S5;
ROcp3_85 = -S4*C5;
ROcp3_95 = C4*C5;
ROcp3_16 = C5*C6;
ROcp3_26 = ROcp3_25*C6+C4*S6;
ROcp3_36 = ROcp3_35*C6+S4*S6;
ROcp3_46 = -C5*S6;
ROcp3_56 = -(ROcp3_25*S6-C4*C6);
ROcp3_66 = -(ROcp3_35*S6-S4*C6);
OMcp3_25 = qd[5]*C4;
OMcp3_35 = qd[5]*S4;
OMcp3_16 = qd[4]+qd[6]*S5;
OMcp3_26 = OMcp3_25+ROcp3_85*qd[6];
OMcp3_36 = OMcp3_35+ROcp3_95*qd[6];
OPcp3_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp3_26 = ROcp3_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp3_35*S5-ROcp3_95*qd[4]);
OPcp3_36 = ROcp3_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp3_25*S5-ROcp3_85*qd[4]);
RLcp3_133 = ROcp3_16*s->dpt[1][4]+ROcp3_46*s->dpt[2][4]+s->dpt[3][4]*S5;
RLcp3_233 = ROcp3_26*s->dpt[1][4]+ROcp3_56*s->dpt[2][4]+ROcp3_85*s->dpt[3][4];
RLcp3_333 = ROcp3_36*s->dpt[1][4]+ROcp3_66*s->dpt[2][4]+ROcp3_95*s->dpt[3][4];
POcp3_133 = RLcp3_133+q[1];
POcp3_233 = RLcp3_233+q[2];
POcp3_333 = RLcp3_333+q[3];
JTcp3_133_5 = -(RLcp3_233*S4-RLcp3_333*C4);
JTcp3_233_5 = RLcp3_133*S4;
JTcp3_333_5 = -RLcp3_133*C4;
JTcp3_133_6 = -(RLcp3_233*ROcp3_95-RLcp3_333*ROcp3_85);
JTcp3_233_6 = RLcp3_133*ROcp3_95-RLcp3_333*S5;
JTcp3_333_6 = -(RLcp3_133*ROcp3_85-RLcp3_233*S5);
ORcp3_133 = OMcp3_26*RLcp3_333-OMcp3_36*RLcp3_233;
ORcp3_233 = -(OMcp3_16*RLcp3_333-OMcp3_36*RLcp3_133);
ORcp3_333 = OMcp3_16*RLcp3_233-OMcp3_26*RLcp3_133;
VIcp3_133 = ORcp3_133+qd[1];
VIcp3_233 = ORcp3_233+qd[2];
VIcp3_333 = ORcp3_333+qd[3];
ACcp3_133 = qdd[1]+OMcp3_26*ORcp3_333-OMcp3_36*ORcp3_233+OPcp3_26*RLcp3_333-OPcp3_36*RLcp3_233;
ACcp3_233 = qdd[2]-OMcp3_16*ORcp3_333+OMcp3_36*ORcp3_133-OPcp3_16*RLcp3_333+OPcp3_36*RLcp3_133;
ACcp3_333 = qdd[3]+OMcp3_16*ORcp3_233-OMcp3_26*ORcp3_133+OPcp3_16*RLcp3_233-OPcp3_26*RLcp3_133;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp3_133;
sens->P[2] = POcp3_233;
sens->P[3] = POcp3_333;
sens->R[1][1] = ROcp3_16;
sens->R[1][2] = ROcp3_26;
sens->R[1][3] = ROcp3_36;
sens->R[2][1] = ROcp3_46;
sens->R[2][2] = ROcp3_56;
sens->R[2][3] = ROcp3_66;
sens->R[3][1] = S5;
sens->R[3][2] = ROcp3_85;
sens->R[3][3] = ROcp3_95;
sens->V[1] = VIcp3_133;
sens->V[2] = VIcp3_233;
sens->V[3] = VIcp3_333;
sens->OM[1] = OMcp3_16;
sens->OM[2] = OMcp3_26;
sens->OM[3] = OMcp3_36;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp3_133_5;
sens->J[1][6] = JTcp3_133_6;
sens->J[2][2] = (1.0);
sens->J[2][4] = -RLcp3_333;
sens->J[2][5] = JTcp3_233_5;
sens->J[2][6] = JTcp3_233_6;
sens->J[3][3] = (1.0);
sens->J[3][4] = RLcp3_233;
sens->J[3][5] = JTcp3_333_5;
sens->J[3][6] = JTcp3_333_6;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp3_85;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp3_95;
sens->A[1] = ACcp3_133;
sens->A[2] = ACcp3_233;
sens->A[3] = ACcp3_333;
sens->OMP[1] = OPcp3_16;
sens->OMP[2] = OPcp3_26;
sens->OMP[3] = OPcp3_36;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


ROcp4_25 = S4*S5;
ROcp4_35 = -C4*S5;
ROcp4_85 = -S4*C5;
ROcp4_95 = C4*C5;
ROcp4_16 = C5*C6;
ROcp4_26 = ROcp4_25*C6+C4*S6;
ROcp4_36 = ROcp4_35*C6+S4*S6;
ROcp4_46 = -C5*S6;
ROcp4_56 = -(ROcp4_25*S6-C4*C6);
ROcp4_66 = -(ROcp4_35*S6-S4*C6);
OMcp4_25 = qd[5]*C4;
OMcp4_35 = qd[5]*S4;
OMcp4_16 = qd[4]+qd[6]*S5;
OMcp4_26 = OMcp4_25+ROcp4_85*qd[6];
OMcp4_36 = OMcp4_35+ROcp4_95*qd[6];
OPcp4_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp4_26 = ROcp4_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp4_35*S5-ROcp4_95*qd[4]);
OPcp4_36 = ROcp4_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp4_25*S5-ROcp4_85*qd[4]);
RLcp4_134 = s->dpt[1][5]*ROcp4_16+s->dpt[2][5]*ROcp4_46+s->dpt[3][5]*S5;
RLcp4_234 = s->dpt[1][5]*ROcp4_26+s->dpt[2][5]*ROcp4_56+s->dpt[3][5]*ROcp4_85;
RLcp4_334 = s->dpt[1][5]*ROcp4_36+s->dpt[2][5]*ROcp4_66+s->dpt[3][5]*ROcp4_95;
POcp4_134 = RLcp4_134+q[1];
POcp4_234 = RLcp4_234+q[2];
POcp4_334 = RLcp4_334+q[3];
JTcp4_134_5 = -(RLcp4_234*S4-RLcp4_334*C4);
JTcp4_234_5 = RLcp4_134*S4;
JTcp4_334_5 = -RLcp4_134*C4;
JTcp4_134_6 = -(RLcp4_234*ROcp4_95-RLcp4_334*ROcp4_85);
JTcp4_234_6 = RLcp4_134*ROcp4_95-RLcp4_334*S5;
JTcp4_334_6 = -(RLcp4_134*ROcp4_85-RLcp4_234*S5);
ORcp4_134 = OMcp4_26*RLcp4_334-OMcp4_36*RLcp4_234;
ORcp4_234 = -(OMcp4_16*RLcp4_334-OMcp4_36*RLcp4_134);
ORcp4_334 = OMcp4_16*RLcp4_234-OMcp4_26*RLcp4_134;
VIcp4_134 = ORcp4_134+qd[1];
VIcp4_234 = ORcp4_234+qd[2];
VIcp4_334 = ORcp4_334+qd[3];
ACcp4_134 = qdd[1]+OMcp4_26*ORcp4_334-OMcp4_36*ORcp4_234+OPcp4_26*RLcp4_334-OPcp4_36*RLcp4_234;
ACcp4_234 = qdd[2]-OMcp4_16*ORcp4_334+OMcp4_36*ORcp4_134-OPcp4_16*RLcp4_334+OPcp4_36*RLcp4_134;
ACcp4_334 = qdd[3]+OMcp4_16*ORcp4_234-OMcp4_26*ORcp4_134+OPcp4_16*RLcp4_234-OPcp4_26*RLcp4_134;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp4_134;
sens->P[2] = POcp4_234;
sens->P[3] = POcp4_334;
sens->R[1][1] = ROcp4_16;
sens->R[1][2] = ROcp4_26;
sens->R[1][3] = ROcp4_36;
sens->R[2][1] = ROcp4_46;
sens->R[2][2] = ROcp4_56;
sens->R[2][3] = ROcp4_66;
sens->R[3][1] = S5;
sens->R[3][2] = ROcp4_85;
sens->R[3][3] = ROcp4_95;
sens->V[1] = VIcp4_134;
sens->V[2] = VIcp4_234;
sens->V[3] = VIcp4_334;
sens->OM[1] = OMcp4_16;
sens->OM[2] = OMcp4_26;
sens->OM[3] = OMcp4_36;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp4_134_5;
sens->J[1][6] = JTcp4_134_6;
sens->J[2][2] = (1.0);
sens->J[2][4] = -RLcp4_334;
sens->J[2][5] = JTcp4_234_5;
sens->J[2][6] = JTcp4_234_6;
sens->J[3][3] = (1.0);
sens->J[3][4] = RLcp4_234;
sens->J[3][5] = JTcp4_334_5;
sens->J[3][6] = JTcp4_334_6;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp4_85;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp4_95;
sens->A[1] = ACcp4_134;
sens->A[2] = ACcp4_234;
sens->A[3] = ACcp4_334;
sens->OMP[1] = OPcp4_16;
sens->OMP[2] = OPcp4_26;
sens->OMP[3] = OPcp4_36;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


ROcp5_25 = S4*S5;
ROcp5_35 = -C4*S5;
ROcp5_85 = -S4*C5;
ROcp5_95 = C4*C5;
ROcp5_16 = C5*C6;
ROcp5_26 = ROcp5_25*C6+C4*S6;
ROcp5_36 = ROcp5_35*C6+S4*S6;
ROcp5_46 = -C5*S6;
ROcp5_56 = -(ROcp5_25*S6-C4*C6);
ROcp5_66 = -(ROcp5_35*S6-S4*C6);
OMcp5_25 = qd[5]*C4;
OMcp5_35 = qd[5]*S4;
OMcp5_16 = qd[4]+qd[6]*S5;
OMcp5_26 = OMcp5_25+ROcp5_85*qd[6];
OMcp5_36 = OMcp5_35+ROcp5_95*qd[6];
OPcp5_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp5_26 = ROcp5_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_35*S5-ROcp5_95*qd[4]);
OPcp5_36 = ROcp5_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_25*S5-ROcp5_85*qd[4]);

// = = Block_1_0_0_6_0_2 = = 
 
// Sensor Kinematics 


ROcp5_17 = ROcp5_16*C7-S5*S7;
ROcp5_27 = ROcp5_26*C7-ROcp5_85*S7;
ROcp5_37 = ROcp5_36*C7-ROcp5_95*S7;
ROcp5_77 = ROcp5_16*S7+S5*C7;
ROcp5_87 = ROcp5_26*S7+ROcp5_85*C7;
ROcp5_97 = ROcp5_36*S7+ROcp5_95*C7;
RLcp5_17 = s->dpt[1][1]*ROcp5_16+s->dpt[3][1]*S5+ROcp5_46*s->dpt[2][1];
RLcp5_27 = s->dpt[1][1]*ROcp5_26+s->dpt[3][1]*ROcp5_85+ROcp5_56*s->dpt[2][1];
RLcp5_37 = s->dpt[1][1]*ROcp5_36+s->dpt[3][1]*ROcp5_95+ROcp5_66*s->dpt[2][1];
OMcp5_17 = OMcp5_16+ROcp5_46*qd[7];
OMcp5_27 = OMcp5_26+ROcp5_56*qd[7];
OMcp5_37 = OMcp5_36+ROcp5_66*qd[7];
ORcp5_17 = OMcp5_26*RLcp5_37-OMcp5_36*RLcp5_27;
ORcp5_27 = -(OMcp5_16*RLcp5_37-OMcp5_36*RLcp5_17);
ORcp5_37 = OMcp5_16*RLcp5_27-OMcp5_26*RLcp5_17;
OPcp5_17 = OPcp5_16+ROcp5_46*qdd[7]+qd[7]*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56);
OPcp5_27 = OPcp5_26+ROcp5_56*qdd[7]-qd[7]*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46);
OPcp5_37 = OPcp5_36+ROcp5_66*qdd[7]+qd[7]*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46);
RLcp5_135 = s->dpt[1][6]*ROcp5_17+s->dpt[3][6]*ROcp5_77+ROcp5_46*s->dpt[2][6];
RLcp5_235 = s->dpt[1][6]*ROcp5_27+s->dpt[3][6]*ROcp5_87+ROcp5_56*s->dpt[2][6];
RLcp5_335 = s->dpt[1][6]*ROcp5_37+s->dpt[3][6]*ROcp5_97+ROcp5_66*s->dpt[2][6];
POcp5_135 = RLcp5_135+RLcp5_17+q[1];
POcp5_235 = RLcp5_235+RLcp5_27+q[2];
POcp5_335 = RLcp5_335+RLcp5_37+q[3];
JTcp5_235_4 = -(RLcp5_335+RLcp5_37);
JTcp5_335_4 = RLcp5_235+RLcp5_27;
JTcp5_135_5 = C4*(RLcp5_335+RLcp5_37)-S4*(RLcp5_235+RLcp5_27);
JTcp5_235_5 = S4*(RLcp5_135+RLcp5_17);
JTcp5_335_5 = -C4*(RLcp5_135+RLcp5_17);
JTcp5_135_6 = ROcp5_85*(RLcp5_335+RLcp5_37)-ROcp5_95*(RLcp5_235+RLcp5_27);
JTcp5_235_6 = ROcp5_95*(RLcp5_135+RLcp5_17)-S5*(RLcp5_335+RLcp5_37);
JTcp5_335_6 = -(ROcp5_85*(RLcp5_135+RLcp5_17)-S5*(RLcp5_235+RLcp5_27));
JTcp5_135_7 = -(RLcp5_235*ROcp5_66-RLcp5_335*ROcp5_56);
JTcp5_235_7 = RLcp5_135*ROcp5_66-RLcp5_335*ROcp5_46;
JTcp5_335_7 = -(RLcp5_135*ROcp5_56-RLcp5_235*ROcp5_46);
ORcp5_135 = OMcp5_27*RLcp5_335-OMcp5_37*RLcp5_235;
ORcp5_235 = -(OMcp5_17*RLcp5_335-OMcp5_37*RLcp5_135);
ORcp5_335 = OMcp5_17*RLcp5_235-OMcp5_27*RLcp5_135;
VIcp5_135 = ORcp5_135+ORcp5_17+qd[1];
VIcp5_235 = ORcp5_235+ORcp5_27+qd[2];
VIcp5_335 = ORcp5_335+ORcp5_37+qd[3];
ACcp5_135 = qdd[1]+OMcp5_26*ORcp5_37+OMcp5_27*ORcp5_335-OMcp5_36*ORcp5_27-OMcp5_37*ORcp5_235+OPcp5_26*RLcp5_37+
 OPcp5_27*RLcp5_335-OPcp5_36*RLcp5_27-OPcp5_37*RLcp5_235;
ACcp5_235 = qdd[2]-OMcp5_16*ORcp5_37-OMcp5_17*ORcp5_335+OMcp5_36*ORcp5_17+OMcp5_37*ORcp5_135-OPcp5_16*RLcp5_37-
 OPcp5_17*RLcp5_335+OPcp5_36*RLcp5_17+OPcp5_37*RLcp5_135;
ACcp5_335 = qdd[3]+OMcp5_16*ORcp5_27+OMcp5_17*ORcp5_235-OMcp5_26*ORcp5_17-OMcp5_27*ORcp5_135+OPcp5_16*RLcp5_27+
 OPcp5_17*RLcp5_235-OPcp5_26*RLcp5_17-OPcp5_27*RLcp5_135;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp5_135;
sens->P[2] = POcp5_235;
sens->P[3] = POcp5_335;
sens->R[1][1] = ROcp5_17;
sens->R[1][2] = ROcp5_27;
sens->R[1][3] = ROcp5_37;
sens->R[2][1] = ROcp5_46;
sens->R[2][2] = ROcp5_56;
sens->R[2][3] = ROcp5_66;
sens->R[3][1] = ROcp5_77;
sens->R[3][2] = ROcp5_87;
sens->R[3][3] = ROcp5_97;
sens->V[1] = VIcp5_135;
sens->V[2] = VIcp5_235;
sens->V[3] = VIcp5_335;
sens->OM[1] = OMcp5_17;
sens->OM[2] = OMcp5_27;
sens->OM[3] = OMcp5_37;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp5_135_5;
sens->J[1][6] = JTcp5_135_6;
sens->J[1][7] = JTcp5_135_7;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp5_235_4;
sens->J[2][5] = JTcp5_235_5;
sens->J[2][6] = JTcp5_235_6;
sens->J[2][7] = JTcp5_235_7;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp5_335_4;
sens->J[3][5] = JTcp5_335_5;
sens->J[3][6] = JTcp5_335_6;
sens->J[3][7] = JTcp5_335_7;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp5_46;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp5_85;
sens->J[5][7] = ROcp5_56;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp5_95;
sens->J[6][7] = ROcp5_66;
sens->A[1] = ACcp5_135;
sens->A[2] = ACcp5_235;
sens->A[3] = ACcp5_335;
sens->OMP[1] = OPcp5_17;
sens->OMP[2] = OPcp5_27;
sens->OMP[3] = OPcp5_37;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


ROcp6_25 = S4*S5;
ROcp6_35 = -C4*S5;
ROcp6_85 = -S4*C5;
ROcp6_95 = C4*C5;
ROcp6_16 = C5*C6;
ROcp6_26 = ROcp6_25*C6+C4*S6;
ROcp6_36 = ROcp6_35*C6+S4*S6;
ROcp6_46 = -C5*S6;
ROcp6_56 = -(ROcp6_25*S6-C4*C6);
ROcp6_66 = -(ROcp6_35*S6-S4*C6);
OMcp6_25 = qd[5]*C4;
OMcp6_35 = qd[5]*S4;
OMcp6_16 = qd[4]+qd[6]*S5;
OMcp6_26 = OMcp6_25+ROcp6_85*qd[6];
OMcp6_36 = OMcp6_35+ROcp6_95*qd[6];
OPcp6_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp6_26 = ROcp6_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_35*S5-ROcp6_95*qd[4]);
OPcp6_36 = ROcp6_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_25*S5-ROcp6_85*qd[4]);

// = = Block_1_0_0_7_0_2 = = 
 
// Sensor Kinematics 


ROcp6_17 = ROcp6_16*C7-S5*S7;
ROcp6_27 = ROcp6_26*C7-ROcp6_85*S7;
ROcp6_37 = ROcp6_36*C7-ROcp6_95*S7;
ROcp6_77 = ROcp6_16*S7+S5*C7;
ROcp6_87 = ROcp6_26*S7+ROcp6_85*C7;
ROcp6_97 = ROcp6_36*S7+ROcp6_95*C7;
RLcp6_17 = s->dpt[1][1]*ROcp6_16+s->dpt[3][1]*S5+ROcp6_46*s->dpt[2][1];
RLcp6_27 = s->dpt[1][1]*ROcp6_26+s->dpt[3][1]*ROcp6_85+ROcp6_56*s->dpt[2][1];
RLcp6_37 = s->dpt[1][1]*ROcp6_36+s->dpt[3][1]*ROcp6_95+ROcp6_66*s->dpt[2][1];
OMcp6_17 = OMcp6_16+ROcp6_46*qd[7];
OMcp6_27 = OMcp6_26+ROcp6_56*qd[7];
OMcp6_37 = OMcp6_36+ROcp6_66*qd[7];
ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
OPcp6_17 = OPcp6_16+ROcp6_46*qdd[7]+qd[7]*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56);
OPcp6_27 = OPcp6_26+ROcp6_56*qdd[7]-qd[7]*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46);
OPcp6_37 = OPcp6_36+ROcp6_66*qdd[7]+qd[7]*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46);
RLcp6_136 = ROcp6_17*s->dpt[1][7]+ROcp6_46*s->dpt[2][7]+ROcp6_77*s->dpt[3][7];
RLcp6_236 = ROcp6_27*s->dpt[1][7]+ROcp6_56*s->dpt[2][7]+ROcp6_87*s->dpt[3][7];
RLcp6_336 = ROcp6_37*s->dpt[1][7]+ROcp6_66*s->dpt[2][7]+ROcp6_97*s->dpt[3][7];
POcp6_136 = RLcp6_136+RLcp6_17+q[1];
POcp6_236 = RLcp6_236+RLcp6_27+q[2];
POcp6_336 = RLcp6_336+RLcp6_37+q[3];
JTcp6_236_4 = -(RLcp6_336+RLcp6_37);
JTcp6_336_4 = RLcp6_236+RLcp6_27;
JTcp6_136_5 = C4*(RLcp6_336+RLcp6_37)-S4*(RLcp6_236+RLcp6_27);
JTcp6_236_5 = S4*(RLcp6_136+RLcp6_17);
JTcp6_336_5 = -C4*(RLcp6_136+RLcp6_17);
JTcp6_136_6 = ROcp6_85*(RLcp6_336+RLcp6_37)-ROcp6_95*(RLcp6_236+RLcp6_27);
JTcp6_236_6 = ROcp6_95*(RLcp6_136+RLcp6_17)-S5*(RLcp6_336+RLcp6_37);
JTcp6_336_6 = -(ROcp6_85*(RLcp6_136+RLcp6_17)-S5*(RLcp6_236+RLcp6_27));
JTcp6_136_7 = -(RLcp6_236*ROcp6_66-RLcp6_336*ROcp6_56);
JTcp6_236_7 = RLcp6_136*ROcp6_66-RLcp6_336*ROcp6_46;
JTcp6_336_7 = -(RLcp6_136*ROcp6_56-RLcp6_236*ROcp6_46);
ORcp6_136 = OMcp6_27*RLcp6_336-OMcp6_37*RLcp6_236;
ORcp6_236 = -(OMcp6_17*RLcp6_336-OMcp6_37*RLcp6_136);
ORcp6_336 = OMcp6_17*RLcp6_236-OMcp6_27*RLcp6_136;
VIcp6_136 = ORcp6_136+ORcp6_17+qd[1];
VIcp6_236 = ORcp6_236+ORcp6_27+qd[2];
VIcp6_336 = ORcp6_336+ORcp6_37+qd[3];
ACcp6_136 = qdd[1]+OMcp6_26*ORcp6_37+OMcp6_27*ORcp6_336-OMcp6_36*ORcp6_27-OMcp6_37*ORcp6_236+OPcp6_26*RLcp6_37+
 OPcp6_27*RLcp6_336-OPcp6_36*RLcp6_27-OPcp6_37*RLcp6_236;
ACcp6_236 = qdd[2]-OMcp6_16*ORcp6_37-OMcp6_17*ORcp6_336+OMcp6_36*ORcp6_17+OMcp6_37*ORcp6_136-OPcp6_16*RLcp6_37-
 OPcp6_17*RLcp6_336+OPcp6_36*RLcp6_17+OPcp6_37*RLcp6_136;
ACcp6_336 = qdd[3]+OMcp6_16*ORcp6_27+OMcp6_17*ORcp6_236-OMcp6_26*ORcp6_17-OMcp6_27*ORcp6_136+OPcp6_16*RLcp6_27+
 OPcp6_17*RLcp6_236-OPcp6_26*RLcp6_17-OPcp6_27*RLcp6_136;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp6_136;
sens->P[2] = POcp6_236;
sens->P[3] = POcp6_336;
sens->R[1][1] = ROcp6_17;
sens->R[1][2] = ROcp6_27;
sens->R[1][3] = ROcp6_37;
sens->R[2][1] = ROcp6_46;
sens->R[2][2] = ROcp6_56;
sens->R[2][3] = ROcp6_66;
sens->R[3][1] = ROcp6_77;
sens->R[3][2] = ROcp6_87;
sens->R[3][3] = ROcp6_97;
sens->V[1] = VIcp6_136;
sens->V[2] = VIcp6_236;
sens->V[3] = VIcp6_336;
sens->OM[1] = OMcp6_17;
sens->OM[2] = OMcp6_27;
sens->OM[3] = OMcp6_37;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp6_136_5;
sens->J[1][6] = JTcp6_136_6;
sens->J[1][7] = JTcp6_136_7;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp6_236_4;
sens->J[2][5] = JTcp6_236_5;
sens->J[2][6] = JTcp6_236_6;
sens->J[2][7] = JTcp6_236_7;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp6_336_4;
sens->J[3][5] = JTcp6_336_5;
sens->J[3][6] = JTcp6_336_6;
sens->J[3][7] = JTcp6_336_7;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp6_46;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp6_85;
sens->J[5][7] = ROcp6_56;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp6_95;
sens->J[6][7] = ROcp6_66;
sens->A[1] = ACcp6_136;
sens->A[2] = ACcp6_236;
sens->A[3] = ACcp6_336;
sens->OMP[1] = OPcp6_17;
sens->OMP[2] = OPcp6_27;
sens->OMP[3] = OPcp6_37;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


ROcp7_25 = S4*S5;
ROcp7_35 = -C4*S5;
ROcp7_85 = -S4*C5;
ROcp7_95 = C4*C5;
ROcp7_16 = C5*C6;
ROcp7_26 = ROcp7_25*C6+C4*S6;
ROcp7_36 = ROcp7_35*C6+S4*S6;
ROcp7_46 = -C5*S6;
ROcp7_56 = -(ROcp7_25*S6-C4*C6);
ROcp7_66 = -(ROcp7_35*S6-S4*C6);
OMcp7_25 = qd[5]*C4;
OMcp7_35 = qd[5]*S4;
OMcp7_16 = qd[4]+qd[6]*S5;
OMcp7_26 = OMcp7_25+ROcp7_85*qd[6];
OMcp7_36 = OMcp7_35+ROcp7_95*qd[6];
OPcp7_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp7_26 = ROcp7_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_35*S5-ROcp7_95*qd[4]);
OPcp7_36 = ROcp7_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_25*S5-ROcp7_85*qd[4]);

// = = Block_1_0_0_8_0_2 = = 
 
// Sensor Kinematics 


ROcp7_17 = ROcp7_16*C7-S5*S7;
ROcp7_27 = ROcp7_26*C7-ROcp7_85*S7;
ROcp7_37 = ROcp7_36*C7-ROcp7_95*S7;
ROcp7_77 = ROcp7_16*S7+S5*C7;
ROcp7_87 = ROcp7_26*S7+ROcp7_85*C7;
ROcp7_97 = ROcp7_36*S7+ROcp7_95*C7;
ROcp7_48 = ROcp7_46*C8+ROcp7_77*S8;
ROcp7_58 = ROcp7_56*C8+ROcp7_87*S8;
ROcp7_68 = ROcp7_66*C8+ROcp7_97*S8;
ROcp7_78 = -(ROcp7_46*S8-ROcp7_77*C8);
ROcp7_88 = -(ROcp7_56*S8-ROcp7_87*C8);
ROcp7_98 = -(ROcp7_66*S8-ROcp7_97*C8);
RLcp7_17 = s->dpt[1][1]*ROcp7_16+s->dpt[3][1]*S5+ROcp7_46*s->dpt[2][1];
RLcp7_27 = s->dpt[1][1]*ROcp7_26+s->dpt[3][1]*ROcp7_85+ROcp7_56*s->dpt[2][1];
RLcp7_37 = s->dpt[1][1]*ROcp7_36+s->dpt[3][1]*ROcp7_95+ROcp7_66*s->dpt[2][1];
OMcp7_17 = OMcp7_16+ROcp7_46*qd[7];
OMcp7_27 = OMcp7_26+ROcp7_56*qd[7];
OMcp7_37 = OMcp7_36+ROcp7_66*qd[7];
ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
OPcp7_17 = OPcp7_16+ROcp7_46*qdd[7]+qd[7]*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
OPcp7_27 = OPcp7_26+ROcp7_56*qdd[7]-qd[7]*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
OPcp7_37 = OPcp7_36+ROcp7_66*qdd[7]+qd[7]*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
RLcp7_18 = s->dpt[1][6]*ROcp7_17+s->dpt[3][6]*ROcp7_77+ROcp7_46*s->dpt[2][6];
RLcp7_28 = s->dpt[1][6]*ROcp7_27+s->dpt[3][6]*ROcp7_87+ROcp7_56*s->dpt[2][6];
RLcp7_38 = s->dpt[1][6]*ROcp7_37+s->dpt[3][6]*ROcp7_97+ROcp7_66*s->dpt[2][6];
OMcp7_18 = OMcp7_17+ROcp7_17*qd[8];
OMcp7_28 = OMcp7_27+ROcp7_27*qd[8];
OMcp7_38 = OMcp7_37+ROcp7_37*qd[8];
ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
OPcp7_18 = OPcp7_17+ROcp7_17*qdd[8]+qd[8]*(OMcp7_27*ROcp7_37-OMcp7_37*ROcp7_27);
OPcp7_28 = OPcp7_27+ROcp7_27*qdd[8]-qd[8]*(OMcp7_17*ROcp7_37-OMcp7_37*ROcp7_17);
OPcp7_38 = OPcp7_37+ROcp7_37*qdd[8]+qd[8]*(OMcp7_17*ROcp7_27-OMcp7_27*ROcp7_17);
RLcp7_137 = s->dpt[1][8]*ROcp7_17+s->dpt[2][8]*ROcp7_48+ROcp7_78*s->dpt[3][8];
RLcp7_237 = s->dpt[1][8]*ROcp7_27+s->dpt[2][8]*ROcp7_58+ROcp7_88*s->dpt[3][8];
RLcp7_337 = s->dpt[1][8]*ROcp7_37+s->dpt[2][8]*ROcp7_68+ROcp7_98*s->dpt[3][8];
POcp7_137 = RLcp7_137+RLcp7_17+RLcp7_18+q[1];
POcp7_237 = RLcp7_237+RLcp7_27+RLcp7_28+q[2];
POcp7_337 = RLcp7_337+RLcp7_37+RLcp7_38+q[3];
JTcp7_237_4 = -(RLcp7_337+RLcp7_37+RLcp7_38);
JTcp7_337_4 = RLcp7_237+RLcp7_27+RLcp7_28;
JTcp7_137_5 = C4*(RLcp7_37+RLcp7_38)-S4*(RLcp7_27+RLcp7_28)-RLcp7_237*S4+RLcp7_337*C4;
JTcp7_237_5 = S4*(RLcp7_137+RLcp7_17+RLcp7_18);
JTcp7_337_5 = -C4*(RLcp7_137+RLcp7_17+RLcp7_18);
JTcp7_137_6 = ROcp7_85*(RLcp7_37+RLcp7_38)-ROcp7_95*(RLcp7_27+RLcp7_28)-RLcp7_237*ROcp7_95+RLcp7_337*ROcp7_85;
JTcp7_237_6 = -(RLcp7_337*S5-ROcp7_95*(RLcp7_137+RLcp7_17+RLcp7_18)+S5*(RLcp7_37+RLcp7_38));
JTcp7_337_6 = RLcp7_237*S5-ROcp7_85*(RLcp7_137+RLcp7_17+RLcp7_18)+S5*(RLcp7_27+RLcp7_28);
JTcp7_137_7 = ROcp7_56*(RLcp7_337+RLcp7_38)-ROcp7_66*(RLcp7_237+RLcp7_28);
JTcp7_237_7 = -(ROcp7_46*(RLcp7_337+RLcp7_38)-ROcp7_66*(RLcp7_137+RLcp7_18));
JTcp7_337_7 = ROcp7_46*(RLcp7_237+RLcp7_28)-ROcp7_56*(RLcp7_137+RLcp7_18);
JTcp7_137_8 = -(RLcp7_237*ROcp7_37-RLcp7_337*ROcp7_27);
JTcp7_237_8 = RLcp7_137*ROcp7_37-RLcp7_337*ROcp7_17;
JTcp7_337_8 = -(RLcp7_137*ROcp7_27-RLcp7_237*ROcp7_17);
ORcp7_137 = OMcp7_28*RLcp7_337-OMcp7_38*RLcp7_237;
ORcp7_237 = -(OMcp7_18*RLcp7_337-OMcp7_38*RLcp7_137);
ORcp7_337 = OMcp7_18*RLcp7_237-OMcp7_28*RLcp7_137;
VIcp7_137 = ORcp7_137+ORcp7_17+ORcp7_18+qd[1];
VIcp7_237 = ORcp7_237+ORcp7_27+ORcp7_28+qd[2];
VIcp7_337 = ORcp7_337+ORcp7_37+ORcp7_38+qd[3];
ACcp7_137 = qdd[1]+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38+OMcp7_28*ORcp7_337-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28-OMcp7_38
 *ORcp7_237+OPcp7_26*RLcp7_37+OPcp7_27*RLcp7_38+OPcp7_28*RLcp7_337-OPcp7_36*RLcp7_27-OPcp7_37*RLcp7_28-OPcp7_38*RLcp7_237;
ACcp7_237 = qdd[2]-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38-OMcp7_18*ORcp7_337+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18+OMcp7_38
 *ORcp7_137-OPcp7_16*RLcp7_37-OPcp7_17*RLcp7_38-OPcp7_18*RLcp7_337+OPcp7_36*RLcp7_17+OPcp7_37*RLcp7_18+OPcp7_38*RLcp7_137;
ACcp7_337 = qdd[3]+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28+OMcp7_18*ORcp7_237-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18-OMcp7_28
 *ORcp7_137+OPcp7_16*RLcp7_27+OPcp7_17*RLcp7_28+OPcp7_18*RLcp7_237-OPcp7_26*RLcp7_17-OPcp7_27*RLcp7_18-OPcp7_28*RLcp7_137;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp7_137;
sens->P[2] = POcp7_237;
sens->P[3] = POcp7_337;
sens->R[1][1] = ROcp7_17;
sens->R[1][2] = ROcp7_27;
sens->R[1][3] = ROcp7_37;
sens->R[2][1] = ROcp7_48;
sens->R[2][2] = ROcp7_58;
sens->R[2][3] = ROcp7_68;
sens->R[3][1] = ROcp7_78;
sens->R[3][2] = ROcp7_88;
sens->R[3][3] = ROcp7_98;
sens->V[1] = VIcp7_137;
sens->V[2] = VIcp7_237;
sens->V[3] = VIcp7_337;
sens->OM[1] = OMcp7_18;
sens->OM[2] = OMcp7_28;
sens->OM[3] = OMcp7_38;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp7_137_5;
sens->J[1][6] = JTcp7_137_6;
sens->J[1][7] = JTcp7_137_7;
sens->J[1][8] = JTcp7_137_8;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp7_237_4;
sens->J[2][5] = JTcp7_237_5;
sens->J[2][6] = JTcp7_237_6;
sens->J[2][7] = JTcp7_237_7;
sens->J[2][8] = JTcp7_237_8;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp7_337_4;
sens->J[3][5] = JTcp7_337_5;
sens->J[3][6] = JTcp7_337_6;
sens->J[3][7] = JTcp7_337_7;
sens->J[3][8] = JTcp7_337_8;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp7_46;
sens->J[4][8] = ROcp7_17;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp7_85;
sens->J[5][7] = ROcp7_56;
sens->J[5][8] = ROcp7_27;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp7_95;
sens->J[6][7] = ROcp7_66;
sens->J[6][8] = ROcp7_37;
sens->A[1] = ACcp7_137;
sens->A[2] = ACcp7_237;
sens->A[3] = ACcp7_337;
sens->OMP[1] = OPcp7_18;
sens->OMP[2] = OPcp7_28;
sens->OMP[3] = OPcp7_38;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


ROcp8_25 = S4*S5;
ROcp8_35 = -C4*S5;
ROcp8_85 = -S4*C5;
ROcp8_95 = C4*C5;
ROcp8_16 = C5*C6;
ROcp8_26 = ROcp8_25*C6+C4*S6;
ROcp8_36 = ROcp8_35*C6+S4*S6;
ROcp8_46 = -C5*S6;
ROcp8_56 = -(ROcp8_25*S6-C4*C6);
ROcp8_66 = -(ROcp8_35*S6-S4*C6);
OMcp8_25 = qd[5]*C4;
OMcp8_35 = qd[5]*S4;
OMcp8_16 = qd[4]+qd[6]*S5;
OMcp8_26 = OMcp8_25+ROcp8_85*qd[6];
OMcp8_36 = OMcp8_35+ROcp8_95*qd[6];
OPcp8_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp8_26 = ROcp8_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp8_35*S5-ROcp8_95*qd[4]);
OPcp8_36 = ROcp8_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp8_25*S5-ROcp8_85*qd[4]);

// = = Block_1_0_0_9_0_2 = = 
 
// Sensor Kinematics 


ROcp8_17 = ROcp8_16*C7-S5*S7;
ROcp8_27 = ROcp8_26*C7-ROcp8_85*S7;
ROcp8_37 = ROcp8_36*C7-ROcp8_95*S7;
ROcp8_77 = ROcp8_16*S7+S5*C7;
ROcp8_87 = ROcp8_26*S7+ROcp8_85*C7;
ROcp8_97 = ROcp8_36*S7+ROcp8_95*C7;
ROcp8_48 = ROcp8_46*C8+ROcp8_77*S8;
ROcp8_58 = ROcp8_56*C8+ROcp8_87*S8;
ROcp8_68 = ROcp8_66*C8+ROcp8_97*S8;
ROcp8_78 = -(ROcp8_46*S8-ROcp8_77*C8);
ROcp8_88 = -(ROcp8_56*S8-ROcp8_87*C8);
ROcp8_98 = -(ROcp8_66*S8-ROcp8_97*C8);
RLcp8_17 = s->dpt[1][1]*ROcp8_16+s->dpt[3][1]*S5+ROcp8_46*s->dpt[2][1];
RLcp8_27 = s->dpt[1][1]*ROcp8_26+s->dpt[3][1]*ROcp8_85+ROcp8_56*s->dpt[2][1];
RLcp8_37 = s->dpt[1][1]*ROcp8_36+s->dpt[3][1]*ROcp8_95+ROcp8_66*s->dpt[2][1];
OMcp8_17 = OMcp8_16+ROcp8_46*qd[7];
OMcp8_27 = OMcp8_26+ROcp8_56*qd[7];
OMcp8_37 = OMcp8_36+ROcp8_66*qd[7];
ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
OPcp8_17 = OPcp8_16+ROcp8_46*qdd[7]+qd[7]*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56);
OPcp8_27 = OPcp8_26+ROcp8_56*qdd[7]-qd[7]*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46);
OPcp8_37 = OPcp8_36+ROcp8_66*qdd[7]+qd[7]*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46);
RLcp8_18 = s->dpt[1][6]*ROcp8_17+s->dpt[3][6]*ROcp8_77+ROcp8_46*s->dpt[2][6];
RLcp8_28 = s->dpt[1][6]*ROcp8_27+s->dpt[3][6]*ROcp8_87+ROcp8_56*s->dpt[2][6];
RLcp8_38 = s->dpt[1][6]*ROcp8_37+s->dpt[3][6]*ROcp8_97+ROcp8_66*s->dpt[2][6];
OMcp8_18 = OMcp8_17+ROcp8_17*qd[8];
OMcp8_28 = OMcp8_27+ROcp8_27*qd[8];
OMcp8_38 = OMcp8_37+ROcp8_37*qd[8];
ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
OPcp8_18 = OPcp8_17+ROcp8_17*qdd[8]+qd[8]*(OMcp8_27*ROcp8_37-OMcp8_37*ROcp8_27);
OPcp8_28 = OPcp8_27+ROcp8_27*qdd[8]-qd[8]*(OMcp8_17*ROcp8_37-OMcp8_37*ROcp8_17);
OPcp8_38 = OPcp8_37+ROcp8_37*qdd[8]+qd[8]*(OMcp8_17*ROcp8_27-OMcp8_27*ROcp8_17);
RLcp8_138 = ROcp8_17*s->dpt[1][9]+ROcp8_48*s->dpt[2][9]+ROcp8_78*s->dpt[3][9];
RLcp8_238 = ROcp8_27*s->dpt[1][9]+ROcp8_58*s->dpt[2][9]+ROcp8_88*s->dpt[3][9];
RLcp8_338 = ROcp8_37*s->dpt[1][9]+ROcp8_68*s->dpt[2][9]+ROcp8_98*s->dpt[3][9];
POcp8_138 = RLcp8_138+RLcp8_17+RLcp8_18+q[1];
POcp8_238 = RLcp8_238+RLcp8_27+RLcp8_28+q[2];
POcp8_338 = RLcp8_338+RLcp8_37+RLcp8_38+q[3];
JTcp8_238_4 = -(RLcp8_338+RLcp8_37+RLcp8_38);
JTcp8_338_4 = RLcp8_238+RLcp8_27+RLcp8_28;
JTcp8_138_5 = C4*(RLcp8_37+RLcp8_38)-S4*(RLcp8_27+RLcp8_28)-RLcp8_238*S4+RLcp8_338*C4;
JTcp8_238_5 = S4*(RLcp8_138+RLcp8_17+RLcp8_18);
JTcp8_338_5 = -C4*(RLcp8_138+RLcp8_17+RLcp8_18);
JTcp8_138_6 = ROcp8_85*(RLcp8_37+RLcp8_38)-ROcp8_95*(RLcp8_27+RLcp8_28)-RLcp8_238*ROcp8_95+RLcp8_338*ROcp8_85;
JTcp8_238_6 = -(RLcp8_338*S5-ROcp8_95*(RLcp8_138+RLcp8_17+RLcp8_18)+S5*(RLcp8_37+RLcp8_38));
JTcp8_338_6 = RLcp8_238*S5-ROcp8_85*(RLcp8_138+RLcp8_17+RLcp8_18)+S5*(RLcp8_27+RLcp8_28);
JTcp8_138_7 = ROcp8_56*(RLcp8_338+RLcp8_38)-ROcp8_66*(RLcp8_238+RLcp8_28);
JTcp8_238_7 = -(ROcp8_46*(RLcp8_338+RLcp8_38)-ROcp8_66*(RLcp8_138+RLcp8_18));
JTcp8_338_7 = ROcp8_46*(RLcp8_238+RLcp8_28)-ROcp8_56*(RLcp8_138+RLcp8_18);
JTcp8_138_8 = -(RLcp8_238*ROcp8_37-RLcp8_338*ROcp8_27);
JTcp8_238_8 = RLcp8_138*ROcp8_37-RLcp8_338*ROcp8_17;
JTcp8_338_8 = -(RLcp8_138*ROcp8_27-RLcp8_238*ROcp8_17);
ORcp8_138 = OMcp8_28*RLcp8_338-OMcp8_38*RLcp8_238;
ORcp8_238 = -(OMcp8_18*RLcp8_338-OMcp8_38*RLcp8_138);
ORcp8_338 = OMcp8_18*RLcp8_238-OMcp8_28*RLcp8_138;
VIcp8_138 = ORcp8_138+ORcp8_17+ORcp8_18+qd[1];
VIcp8_238 = ORcp8_238+ORcp8_27+ORcp8_28+qd[2];
VIcp8_338 = ORcp8_338+ORcp8_37+ORcp8_38+qd[3];
ACcp8_138 = qdd[1]+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38+OMcp8_28*ORcp8_338-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28-OMcp8_38
 *ORcp8_238+OPcp8_26*RLcp8_37+OPcp8_27*RLcp8_38+OPcp8_28*RLcp8_338-OPcp8_36*RLcp8_27-OPcp8_37*RLcp8_28-OPcp8_38*RLcp8_238;
ACcp8_238 = qdd[2]-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38-OMcp8_18*ORcp8_338+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18+OMcp8_38
 *ORcp8_138-OPcp8_16*RLcp8_37-OPcp8_17*RLcp8_38-OPcp8_18*RLcp8_338+OPcp8_36*RLcp8_17+OPcp8_37*RLcp8_18+OPcp8_38*RLcp8_138;
ACcp8_338 = qdd[3]+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_238-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18-OMcp8_28
 *ORcp8_138+OPcp8_16*RLcp8_27+OPcp8_17*RLcp8_28+OPcp8_18*RLcp8_238-OPcp8_26*RLcp8_17-OPcp8_27*RLcp8_18-OPcp8_28*RLcp8_138;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp8_138;
sens->P[2] = POcp8_238;
sens->P[3] = POcp8_338;
sens->R[1][1] = ROcp8_17;
sens->R[1][2] = ROcp8_27;
sens->R[1][3] = ROcp8_37;
sens->R[2][1] = ROcp8_48;
sens->R[2][2] = ROcp8_58;
sens->R[2][3] = ROcp8_68;
sens->R[3][1] = ROcp8_78;
sens->R[3][2] = ROcp8_88;
sens->R[3][3] = ROcp8_98;
sens->V[1] = VIcp8_138;
sens->V[2] = VIcp8_238;
sens->V[3] = VIcp8_338;
sens->OM[1] = OMcp8_18;
sens->OM[2] = OMcp8_28;
sens->OM[3] = OMcp8_38;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp8_138_5;
sens->J[1][6] = JTcp8_138_6;
sens->J[1][7] = JTcp8_138_7;
sens->J[1][8] = JTcp8_138_8;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp8_238_4;
sens->J[2][5] = JTcp8_238_5;
sens->J[2][6] = JTcp8_238_6;
sens->J[2][7] = JTcp8_238_7;
sens->J[2][8] = JTcp8_238_8;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp8_338_4;
sens->J[3][5] = JTcp8_338_5;
sens->J[3][6] = JTcp8_338_6;
sens->J[3][7] = JTcp8_338_7;
sens->J[3][8] = JTcp8_338_8;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp8_46;
sens->J[4][8] = ROcp8_17;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp8_85;
sens->J[5][7] = ROcp8_56;
sens->J[5][8] = ROcp8_27;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp8_95;
sens->J[6][7] = ROcp8_66;
sens->J[6][8] = ROcp8_37;
sens->A[1] = ACcp8_138;
sens->A[2] = ACcp8_238;
sens->A[3] = ACcp8_338;
sens->OMP[1] = OPcp8_18;
sens->OMP[2] = OPcp8_28;
sens->OMP[3] = OPcp8_38;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


ROcp9_25 = S4*S5;
ROcp9_35 = -C4*S5;
ROcp9_85 = -S4*C5;
ROcp9_95 = C4*C5;
ROcp9_16 = C5*C6;
ROcp9_26 = ROcp9_25*C6+C4*S6;
ROcp9_36 = ROcp9_35*C6+S4*S6;
ROcp9_46 = -C5*S6;
ROcp9_56 = -(ROcp9_25*S6-C4*C6);
ROcp9_66 = -(ROcp9_35*S6-S4*C6);
OMcp9_25 = qd[5]*C4;
OMcp9_35 = qd[5]*S4;
OMcp9_16 = qd[4]+qd[6]*S5;
OMcp9_26 = OMcp9_25+ROcp9_85*qd[6];
OMcp9_36 = OMcp9_35+ROcp9_95*qd[6];
OPcp9_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp9_26 = ROcp9_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp9_35*S5-ROcp9_95*qd[4]);
OPcp9_36 = ROcp9_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp9_25*S5-ROcp9_85*qd[4]);

// = = Block_1_0_0_10_0_2 = = 
 
// Sensor Kinematics 


ROcp9_17 = ROcp9_16*C7-S5*S7;
ROcp9_27 = ROcp9_26*C7-ROcp9_85*S7;
ROcp9_37 = ROcp9_36*C7-ROcp9_95*S7;
ROcp9_77 = ROcp9_16*S7+S5*C7;
ROcp9_87 = ROcp9_26*S7+ROcp9_85*C7;
ROcp9_97 = ROcp9_36*S7+ROcp9_95*C7;
ROcp9_48 = ROcp9_46*C8+ROcp9_77*S8;
ROcp9_58 = ROcp9_56*C8+ROcp9_87*S8;
ROcp9_68 = ROcp9_66*C8+ROcp9_97*S8;
ROcp9_78 = -(ROcp9_46*S8-ROcp9_77*C8);
ROcp9_88 = -(ROcp9_56*S8-ROcp9_87*C8);
ROcp9_98 = -(ROcp9_66*S8-ROcp9_97*C8);
ROcp9_19 = ROcp9_17*C9+ROcp9_48*S9;
ROcp9_29 = ROcp9_27*C9+ROcp9_58*S9;
ROcp9_39 = ROcp9_37*C9+ROcp9_68*S9;
ROcp9_49 = -(ROcp9_17*S9-ROcp9_48*C9);
ROcp9_59 = -(ROcp9_27*S9-ROcp9_58*C9);
ROcp9_69 = -(ROcp9_37*S9-ROcp9_68*C9);
RLcp9_17 = s->dpt[1][1]*ROcp9_16+s->dpt[3][1]*S5+ROcp9_46*s->dpt[2][1];
RLcp9_27 = s->dpt[1][1]*ROcp9_26+s->dpt[3][1]*ROcp9_85+ROcp9_56*s->dpt[2][1];
RLcp9_37 = s->dpt[1][1]*ROcp9_36+s->dpt[3][1]*ROcp9_95+ROcp9_66*s->dpt[2][1];
OMcp9_17 = OMcp9_16+ROcp9_46*qd[7];
OMcp9_27 = OMcp9_26+ROcp9_56*qd[7];
OMcp9_37 = OMcp9_36+ROcp9_66*qd[7];
ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
OPcp9_17 = OPcp9_16+ROcp9_46*qdd[7]+qd[7]*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56);
OPcp9_27 = OPcp9_26+ROcp9_56*qdd[7]-qd[7]*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46);
OPcp9_37 = OPcp9_36+ROcp9_66*qdd[7]+qd[7]*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46);
RLcp9_18 = s->dpt[1][6]*ROcp9_17+s->dpt[3][6]*ROcp9_77+ROcp9_46*s->dpt[2][6];
RLcp9_28 = s->dpt[1][6]*ROcp9_27+s->dpt[3][6]*ROcp9_87+ROcp9_56*s->dpt[2][6];
RLcp9_38 = s->dpt[1][6]*ROcp9_37+s->dpt[3][6]*ROcp9_97+ROcp9_66*s->dpt[2][6];
OMcp9_18 = OMcp9_17+ROcp9_17*qd[8];
OMcp9_28 = OMcp9_27+ROcp9_27*qd[8];
OMcp9_38 = OMcp9_37+ROcp9_37*qd[8];
ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
OPcp9_18 = OPcp9_17+ROcp9_17*qdd[8]+qd[8]*(OMcp9_27*ROcp9_37-OMcp9_37*ROcp9_27);
OPcp9_28 = OPcp9_27+ROcp9_27*qdd[8]-qd[8]*(OMcp9_17*ROcp9_37-OMcp9_37*ROcp9_17);
OPcp9_38 = OPcp9_37+ROcp9_37*qdd[8]+qd[8]*(OMcp9_17*ROcp9_27-OMcp9_27*ROcp9_17);
RLcp9_19 = s->dpt[1][8]*ROcp9_17+s->dpt[2][8]*ROcp9_48+ROcp9_78*s->dpt[3][8];
RLcp9_29 = s->dpt[1][8]*ROcp9_27+s->dpt[2][8]*ROcp9_58+ROcp9_88*s->dpt[3][8];
RLcp9_39 = s->dpt[1][8]*ROcp9_37+s->dpt[2][8]*ROcp9_68+ROcp9_98*s->dpt[3][8];
OMcp9_19 = OMcp9_18+ROcp9_78*qd[9];
OMcp9_29 = OMcp9_28+ROcp9_88*qd[9];
OMcp9_39 = OMcp9_38+ROcp9_98*qd[9];
ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
OPcp9_19 = OPcp9_18+ROcp9_78*qdd[9]+qd[9]*(OMcp9_28*ROcp9_98-OMcp9_38*ROcp9_88);
OPcp9_29 = OPcp9_28+ROcp9_88*qdd[9]-qd[9]*(OMcp9_18*ROcp9_98-OMcp9_38*ROcp9_78);
OPcp9_39 = OPcp9_38+ROcp9_98*qdd[9]+qd[9]*(OMcp9_18*ROcp9_88-OMcp9_28*ROcp9_78);
RLcp9_139 = s->dpt[1][10]*ROcp9_19+s->dpt[2][10]*ROcp9_49+ROcp9_78*s->dpt[3][10];
RLcp9_239 = s->dpt[1][10]*ROcp9_29+s->dpt[2][10]*ROcp9_59+ROcp9_88*s->dpt[3][10];
RLcp9_339 = s->dpt[1][10]*ROcp9_39+s->dpt[2][10]*ROcp9_69+ROcp9_98*s->dpt[3][10];
POcp9_139 = RLcp9_139+RLcp9_17+RLcp9_18+RLcp9_19+q[1];
POcp9_239 = RLcp9_239+RLcp9_27+RLcp9_28+RLcp9_29+q[2];
POcp9_339 = RLcp9_339+RLcp9_37+RLcp9_38+RLcp9_39+q[3];
JTcp9_239_4 = -(RLcp9_339+RLcp9_37+RLcp9_38+RLcp9_39);
JTcp9_339_4 = RLcp9_239+RLcp9_27+RLcp9_28+RLcp9_29;
JTcp9_139_5 = C4*(RLcp9_339+RLcp9_37+RLcp9_38+RLcp9_39)-S4*(RLcp9_239+RLcp9_29)-S4*(RLcp9_27+RLcp9_28);
JTcp9_239_5 = S4*(RLcp9_139+RLcp9_17+RLcp9_18+RLcp9_19);
JTcp9_339_5 = -C4*(RLcp9_139+RLcp9_17+RLcp9_18+RLcp9_19);
JTcp9_139_6 = ROcp9_85*(RLcp9_339+RLcp9_37+RLcp9_38+RLcp9_39)-ROcp9_95*(RLcp9_239+RLcp9_29)-ROcp9_95*(RLcp9_27+
 RLcp9_28);
JTcp9_239_6 = RLcp9_139*ROcp9_95-RLcp9_339*S5-RLcp9_39*S5+ROcp9_95*(RLcp9_17+RLcp9_18+RLcp9_19)-S5*(RLcp9_37+RLcp9_38);
JTcp9_339_6 = RLcp9_29*S5-ROcp9_85*(RLcp9_17+RLcp9_18+RLcp9_19)+S5*(RLcp9_27+RLcp9_28)-RLcp9_139*ROcp9_85+RLcp9_239*S5;
JTcp9_139_7 = ROcp9_56*(RLcp9_38+RLcp9_39)-ROcp9_66*(RLcp9_28+RLcp9_29)-RLcp9_239*ROcp9_66+RLcp9_339*ROcp9_56;
JTcp9_239_7 = RLcp9_139*ROcp9_66-RLcp9_339*ROcp9_46-ROcp9_46*(RLcp9_38+RLcp9_39)+ROcp9_66*(RLcp9_18+RLcp9_19);
JTcp9_339_7 = ROcp9_46*(RLcp9_28+RLcp9_29)-ROcp9_56*(RLcp9_18+RLcp9_19)-RLcp9_139*ROcp9_56+RLcp9_239*ROcp9_46;
JTcp9_139_8 = ROcp9_27*(RLcp9_339+RLcp9_39)-ROcp9_37*(RLcp9_239+RLcp9_29);
JTcp9_239_8 = -(ROcp9_17*(RLcp9_339+RLcp9_39)-ROcp9_37*(RLcp9_139+RLcp9_19));
JTcp9_339_8 = ROcp9_17*(RLcp9_239+RLcp9_29)-ROcp9_27*(RLcp9_139+RLcp9_19);
JTcp9_139_9 = -(RLcp9_239*ROcp9_98-RLcp9_339*ROcp9_88);
JTcp9_239_9 = RLcp9_139*ROcp9_98-RLcp9_339*ROcp9_78;
JTcp9_339_9 = -(RLcp9_139*ROcp9_88-RLcp9_239*ROcp9_78);
ORcp9_139 = OMcp9_29*RLcp9_339-OMcp9_39*RLcp9_239;
ORcp9_239 = -(OMcp9_19*RLcp9_339-OMcp9_39*RLcp9_139);
ORcp9_339 = OMcp9_19*RLcp9_239-OMcp9_29*RLcp9_139;
VIcp9_139 = ORcp9_139+ORcp9_17+ORcp9_18+ORcp9_19+qd[1];
VIcp9_239 = ORcp9_239+ORcp9_27+ORcp9_28+ORcp9_29+qd[2];
VIcp9_339 = ORcp9_339+ORcp9_37+ORcp9_38+ORcp9_39+qd[3];
ACcp9_139 = qdd[1]+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38+OMcp9_28*ORcp9_39+OMcp9_29*ORcp9_339-OMcp9_36*ORcp9_27-OMcp9_37
 *ORcp9_28-OMcp9_38*ORcp9_29-OMcp9_39*ORcp9_239+OPcp9_26*RLcp9_37+OPcp9_27*RLcp9_38+OPcp9_28*RLcp9_39+OPcp9_29*RLcp9_339-
 OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28-OPcp9_38*RLcp9_29-OPcp9_39*RLcp9_239;
ACcp9_239 = qdd[2]-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38-OMcp9_18*ORcp9_39-OMcp9_19*ORcp9_339+OMcp9_36*ORcp9_17+OMcp9_37
 *ORcp9_18+OMcp9_38*ORcp9_19+OMcp9_39*ORcp9_139-OPcp9_16*RLcp9_37-OPcp9_17*RLcp9_38-OPcp9_18*RLcp9_39-OPcp9_19*RLcp9_339+
 OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18+OPcp9_38*RLcp9_19+OPcp9_39*RLcp9_139;
ACcp9_339 = qdd[3]+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28+OMcp9_18*ORcp9_29+OMcp9_19*ORcp9_239-OMcp9_26*ORcp9_17-OMcp9_27
 *ORcp9_18-OMcp9_28*ORcp9_19-OMcp9_29*ORcp9_139+OPcp9_16*RLcp9_27+OPcp9_17*RLcp9_28+OPcp9_18*RLcp9_29+OPcp9_19*RLcp9_239-
 OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18-OPcp9_28*RLcp9_19-OPcp9_29*RLcp9_139;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp9_139;
sens->P[2] = POcp9_239;
sens->P[3] = POcp9_339;
sens->R[1][1] = ROcp9_19;
sens->R[1][2] = ROcp9_29;
sens->R[1][3] = ROcp9_39;
sens->R[2][1] = ROcp9_49;
sens->R[2][2] = ROcp9_59;
sens->R[2][3] = ROcp9_69;
sens->R[3][1] = ROcp9_78;
sens->R[3][2] = ROcp9_88;
sens->R[3][3] = ROcp9_98;
sens->V[1] = VIcp9_139;
sens->V[2] = VIcp9_239;
sens->V[3] = VIcp9_339;
sens->OM[1] = OMcp9_19;
sens->OM[2] = OMcp9_29;
sens->OM[3] = OMcp9_39;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp9_139_5;
sens->J[1][6] = JTcp9_139_6;
sens->J[1][7] = JTcp9_139_7;
sens->J[1][8] = JTcp9_139_8;
sens->J[1][9] = JTcp9_139_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp9_239_4;
sens->J[2][5] = JTcp9_239_5;
sens->J[2][6] = JTcp9_239_6;
sens->J[2][7] = JTcp9_239_7;
sens->J[2][8] = JTcp9_239_8;
sens->J[2][9] = JTcp9_239_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp9_339_4;
sens->J[3][5] = JTcp9_339_5;
sens->J[3][6] = JTcp9_339_6;
sens->J[3][7] = JTcp9_339_7;
sens->J[3][8] = JTcp9_339_8;
sens->J[3][9] = JTcp9_339_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp9_46;
sens->J[4][8] = ROcp9_17;
sens->J[4][9] = ROcp9_78;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp9_85;
sens->J[5][7] = ROcp9_56;
sens->J[5][8] = ROcp9_27;
sens->J[5][9] = ROcp9_88;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp9_95;
sens->J[6][7] = ROcp9_66;
sens->J[6][8] = ROcp9_37;
sens->J[6][9] = ROcp9_98;
sens->A[1] = ACcp9_139;
sens->A[2] = ACcp9_239;
sens->A[3] = ACcp9_339;
sens->OMP[1] = OPcp9_19;
sens->OMP[2] = OPcp9_29;
sens->OMP[3] = OPcp9_39;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
// Sensor Kinematics 


ROcp10_25 = S4*S5;
ROcp10_35 = -C4*S5;
ROcp10_85 = -S4*C5;
ROcp10_95 = C4*C5;
ROcp10_16 = C5*C6;
ROcp10_26 = ROcp10_25*C6+C4*S6;
ROcp10_36 = ROcp10_35*C6+S4*S6;
ROcp10_46 = -C5*S6;
ROcp10_56 = -(ROcp10_25*S6-C4*C6);
ROcp10_66 = -(ROcp10_35*S6-S4*C6);
OMcp10_25 = qd[5]*C4;
OMcp10_35 = qd[5]*S4;
OMcp10_16 = qd[4]+qd[6]*S5;
OMcp10_26 = OMcp10_25+ROcp10_85*qd[6];
OMcp10_36 = OMcp10_35+ROcp10_95*qd[6];
OPcp10_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp10_26 = ROcp10_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp10_35*S5-ROcp10_95*qd[4]);
OPcp10_36 = ROcp10_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp10_25*S5-ROcp10_85*qd[4]);

// = = Block_1_0_0_11_0_2 = = 
 
// Sensor Kinematics 


ROcp10_17 = ROcp10_16*C7-S5*S7;
ROcp10_27 = ROcp10_26*C7-ROcp10_85*S7;
ROcp10_37 = ROcp10_36*C7-ROcp10_95*S7;
ROcp10_77 = ROcp10_16*S7+S5*C7;
ROcp10_87 = ROcp10_26*S7+ROcp10_85*C7;
ROcp10_97 = ROcp10_36*S7+ROcp10_95*C7;
ROcp10_48 = ROcp10_46*C8+ROcp10_77*S8;
ROcp10_58 = ROcp10_56*C8+ROcp10_87*S8;
ROcp10_68 = ROcp10_66*C8+ROcp10_97*S8;
ROcp10_78 = -(ROcp10_46*S8-ROcp10_77*C8);
ROcp10_88 = -(ROcp10_56*S8-ROcp10_87*C8);
ROcp10_98 = -(ROcp10_66*S8-ROcp10_97*C8);
ROcp10_19 = ROcp10_17*C9+ROcp10_48*S9;
ROcp10_29 = ROcp10_27*C9+ROcp10_58*S9;
ROcp10_39 = ROcp10_37*C9+ROcp10_68*S9;
ROcp10_49 = -(ROcp10_17*S9-ROcp10_48*C9);
ROcp10_59 = -(ROcp10_27*S9-ROcp10_58*C9);
ROcp10_69 = -(ROcp10_37*S9-ROcp10_68*C9);
RLcp10_17 = s->dpt[1][1]*ROcp10_16+s->dpt[3][1]*S5+ROcp10_46*s->dpt[2][1];
RLcp10_27 = s->dpt[1][1]*ROcp10_26+s->dpt[3][1]*ROcp10_85+ROcp10_56*s->dpt[2][1];
RLcp10_37 = s->dpt[1][1]*ROcp10_36+s->dpt[3][1]*ROcp10_95+ROcp10_66*s->dpt[2][1];
OMcp10_17 = OMcp10_16+ROcp10_46*qd[7];
OMcp10_27 = OMcp10_26+ROcp10_56*qd[7];
OMcp10_37 = OMcp10_36+ROcp10_66*qd[7];
ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
OPcp10_17 = OPcp10_16+ROcp10_46*qdd[7]+qd[7]*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56);
OPcp10_27 = OPcp10_26+ROcp10_56*qdd[7]-qd[7]*(OMcp10_16*ROcp10_66-OMcp10_36*ROcp10_46);
OPcp10_37 = OPcp10_36+ROcp10_66*qdd[7]+qd[7]*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46);
RLcp10_18 = s->dpt[1][6]*ROcp10_17+s->dpt[3][6]*ROcp10_77+ROcp10_46*s->dpt[2][6];
RLcp10_28 = s->dpt[1][6]*ROcp10_27+s->dpt[3][6]*ROcp10_87+ROcp10_56*s->dpt[2][6];
RLcp10_38 = s->dpt[1][6]*ROcp10_37+s->dpt[3][6]*ROcp10_97+ROcp10_66*s->dpt[2][6];
OMcp10_18 = OMcp10_17+ROcp10_17*qd[8];
OMcp10_28 = OMcp10_27+ROcp10_27*qd[8];
OMcp10_38 = OMcp10_37+ROcp10_37*qd[8];
ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
OPcp10_18 = OPcp10_17+ROcp10_17*qdd[8]+qd[8]*(OMcp10_27*ROcp10_37-OMcp10_37*ROcp10_27);
OPcp10_28 = OPcp10_27+ROcp10_27*qdd[8]-qd[8]*(OMcp10_17*ROcp10_37-OMcp10_37*ROcp10_17);
OPcp10_38 = OPcp10_37+ROcp10_37*qdd[8]+qd[8]*(OMcp10_17*ROcp10_27-OMcp10_27*ROcp10_17);
RLcp10_19 = s->dpt[1][8]*ROcp10_17+s->dpt[2][8]*ROcp10_48+ROcp10_78*s->dpt[3][8];
RLcp10_29 = s->dpt[1][8]*ROcp10_27+s->dpt[2][8]*ROcp10_58+ROcp10_88*s->dpt[3][8];
RLcp10_39 = s->dpt[1][8]*ROcp10_37+s->dpt[2][8]*ROcp10_68+ROcp10_98*s->dpt[3][8];
OMcp10_19 = OMcp10_18+ROcp10_78*qd[9];
OMcp10_29 = OMcp10_28+ROcp10_88*qd[9];
OMcp10_39 = OMcp10_38+ROcp10_98*qd[9];
ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
OPcp10_19 = OPcp10_18+ROcp10_78*qdd[9]+qd[9]*(OMcp10_28*ROcp10_98-OMcp10_38*ROcp10_88);
OPcp10_29 = OPcp10_28+ROcp10_88*qdd[9]-qd[9]*(OMcp10_18*ROcp10_98-OMcp10_38*ROcp10_78);
OPcp10_39 = OPcp10_38+ROcp10_98*qdd[9]+qd[9]*(OMcp10_18*ROcp10_88-OMcp10_28*ROcp10_78);
RLcp10_140 = ROcp10_19*s->dpt[1][11]+ROcp10_49*s->dpt[2][11]+ROcp10_78*s->dpt[3][11];
RLcp10_240 = ROcp10_29*s->dpt[1][11]+ROcp10_59*s->dpt[2][11]+ROcp10_88*s->dpt[3][11];
RLcp10_340 = ROcp10_39*s->dpt[1][11]+ROcp10_69*s->dpt[2][11]+ROcp10_98*s->dpt[3][11];
POcp10_140 = RLcp10_140+RLcp10_17+RLcp10_18+RLcp10_19+q[1];
POcp10_240 = RLcp10_240+RLcp10_27+RLcp10_28+RLcp10_29+q[2];
POcp10_340 = RLcp10_340+RLcp10_37+RLcp10_38+RLcp10_39+q[3];
JTcp10_240_4 = -(RLcp10_340+RLcp10_37+RLcp10_38+RLcp10_39);
JTcp10_340_4 = RLcp10_240+RLcp10_27+RLcp10_28+RLcp10_29;
JTcp10_140_5 = C4*(RLcp10_340+RLcp10_37+RLcp10_38+RLcp10_39)-S4*(RLcp10_240+RLcp10_29)-S4*(RLcp10_27+RLcp10_28);
JTcp10_240_5 = S4*(RLcp10_140+RLcp10_17+RLcp10_18+RLcp10_19);
JTcp10_340_5 = -C4*(RLcp10_140+RLcp10_17+RLcp10_18+RLcp10_19);
JTcp10_140_6 = ROcp10_85*(RLcp10_340+RLcp10_37+RLcp10_38+RLcp10_39)-ROcp10_95*(RLcp10_240+RLcp10_29)-ROcp10_95*(
 RLcp10_27+RLcp10_28);
JTcp10_240_6 = RLcp10_140*ROcp10_95-RLcp10_340*S5-RLcp10_39*S5+ROcp10_95*(RLcp10_17+RLcp10_18+RLcp10_19)-S5*(RLcp10_37
 +RLcp10_38);
JTcp10_340_6 = RLcp10_29*S5-ROcp10_85*(RLcp10_17+RLcp10_18+RLcp10_19)+S5*(RLcp10_27+RLcp10_28)-RLcp10_140*ROcp10_85+
 RLcp10_240*S5;
JTcp10_140_7 = ROcp10_56*(RLcp10_38+RLcp10_39)-ROcp10_66*(RLcp10_28+RLcp10_29)-RLcp10_240*ROcp10_66+RLcp10_340*
 ROcp10_56;
JTcp10_240_7 = RLcp10_140*ROcp10_66-RLcp10_340*ROcp10_46-ROcp10_46*(RLcp10_38+RLcp10_39)+ROcp10_66*(RLcp10_18+
 RLcp10_19);
JTcp10_340_7 = ROcp10_46*(RLcp10_28+RLcp10_29)-ROcp10_56*(RLcp10_18+RLcp10_19)-RLcp10_140*ROcp10_56+RLcp10_240*
 ROcp10_46;
JTcp10_140_8 = ROcp10_27*(RLcp10_340+RLcp10_39)-ROcp10_37*(RLcp10_240+RLcp10_29);
JTcp10_240_8 = -(ROcp10_17*(RLcp10_340+RLcp10_39)-ROcp10_37*(RLcp10_140+RLcp10_19));
JTcp10_340_8 = ROcp10_17*(RLcp10_240+RLcp10_29)-ROcp10_27*(RLcp10_140+RLcp10_19);
JTcp10_140_9 = -(RLcp10_240*ROcp10_98-RLcp10_340*ROcp10_88);
JTcp10_240_9 = RLcp10_140*ROcp10_98-RLcp10_340*ROcp10_78;
JTcp10_340_9 = -(RLcp10_140*ROcp10_88-RLcp10_240*ROcp10_78);
ORcp10_140 = OMcp10_29*RLcp10_340-OMcp10_39*RLcp10_240;
ORcp10_240 = -(OMcp10_19*RLcp10_340-OMcp10_39*RLcp10_140);
ORcp10_340 = OMcp10_19*RLcp10_240-OMcp10_29*RLcp10_140;
VIcp10_140 = ORcp10_140+ORcp10_17+ORcp10_18+ORcp10_19+qd[1];
VIcp10_240 = ORcp10_240+ORcp10_27+ORcp10_28+ORcp10_29+qd[2];
VIcp10_340 = ORcp10_340+ORcp10_37+ORcp10_38+ORcp10_39+qd[3];
ACcp10_140 = qdd[1]+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38+OMcp10_28*ORcp10_39+OMcp10_29*ORcp10_340-OMcp10_36*
 ORcp10_27-OMcp10_37*ORcp10_28-OMcp10_38*ORcp10_29-OMcp10_39*ORcp10_240+OPcp10_26*RLcp10_37+OPcp10_27*RLcp10_38+OPcp10_28*
 RLcp10_39+OPcp10_29*RLcp10_340-OPcp10_36*RLcp10_27-OPcp10_37*RLcp10_28-OPcp10_38*RLcp10_29-OPcp10_39*RLcp10_240;
ACcp10_240 = qdd[2]-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38-OMcp10_18*ORcp10_39-OMcp10_19*ORcp10_340+OMcp10_36*
 ORcp10_17+OMcp10_37*ORcp10_18+OMcp10_38*ORcp10_19+OMcp10_39*ORcp10_140-OPcp10_16*RLcp10_37-OPcp10_17*RLcp10_38-OPcp10_18*
 RLcp10_39-OPcp10_19*RLcp10_340+OPcp10_36*RLcp10_17+OPcp10_37*RLcp10_18+OPcp10_38*RLcp10_19+OPcp10_39*RLcp10_140;
ACcp10_340 = qdd[3]+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28+OMcp10_18*ORcp10_29+OMcp10_19*ORcp10_240-OMcp10_26*
 ORcp10_17-OMcp10_27*ORcp10_18-OMcp10_28*ORcp10_19-OMcp10_29*ORcp10_140+OPcp10_16*RLcp10_27+OPcp10_17*RLcp10_28+OPcp10_18*
 RLcp10_29+OPcp10_19*RLcp10_240-OPcp10_26*RLcp10_17-OPcp10_27*RLcp10_18-OPcp10_28*RLcp10_19-OPcp10_29*RLcp10_140;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp10_140;
sens->P[2] = POcp10_240;
sens->P[3] = POcp10_340;
sens->R[1][1] = ROcp10_19;
sens->R[1][2] = ROcp10_29;
sens->R[1][3] = ROcp10_39;
sens->R[2][1] = ROcp10_49;
sens->R[2][2] = ROcp10_59;
sens->R[2][3] = ROcp10_69;
sens->R[3][1] = ROcp10_78;
sens->R[3][2] = ROcp10_88;
sens->R[3][3] = ROcp10_98;
sens->V[1] = VIcp10_140;
sens->V[2] = VIcp10_240;
sens->V[3] = VIcp10_340;
sens->OM[1] = OMcp10_19;
sens->OM[2] = OMcp10_29;
sens->OM[3] = OMcp10_39;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp10_140_5;
sens->J[1][6] = JTcp10_140_6;
sens->J[1][7] = JTcp10_140_7;
sens->J[1][8] = JTcp10_140_8;
sens->J[1][9] = JTcp10_140_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp10_240_4;
sens->J[2][5] = JTcp10_240_5;
sens->J[2][6] = JTcp10_240_6;
sens->J[2][7] = JTcp10_240_7;
sens->J[2][8] = JTcp10_240_8;
sens->J[2][9] = JTcp10_240_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp10_340_4;
sens->J[3][5] = JTcp10_340_5;
sens->J[3][6] = JTcp10_340_6;
sens->J[3][7] = JTcp10_340_7;
sens->J[3][8] = JTcp10_340_8;
sens->J[3][9] = JTcp10_340_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp10_46;
sens->J[4][8] = ROcp10_17;
sens->J[4][9] = ROcp10_78;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp10_85;
sens->J[5][7] = ROcp10_56;
sens->J[5][8] = ROcp10_27;
sens->J[5][9] = ROcp10_88;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp10_95;
sens->J[6][7] = ROcp10_66;
sens->J[6][8] = ROcp10_37;
sens->J[6][9] = ROcp10_98;
sens->A[1] = ACcp10_140;
sens->A[2] = ACcp10_240;
sens->A[3] = ACcp10_340;
sens->OMP[1] = OPcp10_19;
sens->OMP[2] = OPcp10_29;
sens->OMP[3] = OPcp10_39;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
// Sensor Kinematics 


ROcp11_25 = S4*S5;
ROcp11_35 = -C4*S5;
ROcp11_85 = -S4*C5;
ROcp11_95 = C4*C5;
ROcp11_16 = C5*C6;
ROcp11_26 = ROcp11_25*C6+C4*S6;
ROcp11_36 = ROcp11_35*C6+S4*S6;
ROcp11_46 = -C5*S6;
ROcp11_56 = -(ROcp11_25*S6-C4*C6);
ROcp11_66 = -(ROcp11_35*S6-S4*C6);
OMcp11_25 = qd[5]*C4;
OMcp11_35 = qd[5]*S4;
OMcp11_16 = qd[4]+qd[6]*S5;
OMcp11_26 = OMcp11_25+ROcp11_85*qd[6];
OMcp11_36 = OMcp11_35+ROcp11_95*qd[6];
OPcp11_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp11_26 = ROcp11_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp11_35*S5-ROcp11_95*qd[4]);
OPcp11_36 = ROcp11_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp11_25*S5-ROcp11_85*qd[4]);

// = = Block_1_0_0_12_0_2 = = 
 
// Sensor Kinematics 


ROcp11_17 = ROcp11_16*C7-S5*S7;
ROcp11_27 = ROcp11_26*C7-ROcp11_85*S7;
ROcp11_37 = ROcp11_36*C7-ROcp11_95*S7;
ROcp11_77 = ROcp11_16*S7+S5*C7;
ROcp11_87 = ROcp11_26*S7+ROcp11_85*C7;
ROcp11_97 = ROcp11_36*S7+ROcp11_95*C7;
ROcp11_48 = ROcp11_46*C8+ROcp11_77*S8;
ROcp11_58 = ROcp11_56*C8+ROcp11_87*S8;
ROcp11_68 = ROcp11_66*C8+ROcp11_97*S8;
ROcp11_78 = -(ROcp11_46*S8-ROcp11_77*C8);
ROcp11_88 = -(ROcp11_56*S8-ROcp11_87*C8);
ROcp11_98 = -(ROcp11_66*S8-ROcp11_97*C8);
ROcp11_19 = ROcp11_17*C9+ROcp11_48*S9;
ROcp11_29 = ROcp11_27*C9+ROcp11_58*S9;
ROcp11_39 = ROcp11_37*C9+ROcp11_68*S9;
ROcp11_49 = -(ROcp11_17*S9-ROcp11_48*C9);
ROcp11_59 = -(ROcp11_27*S9-ROcp11_58*C9);
ROcp11_69 = -(ROcp11_37*S9-ROcp11_68*C9);
ROcp11_110 = ROcp11_19*C10-ROcp11_78*S10;
ROcp11_210 = ROcp11_29*C10-ROcp11_88*S10;
ROcp11_310 = ROcp11_39*C10-ROcp11_98*S10;
ROcp11_710 = ROcp11_19*S10+ROcp11_78*C10;
ROcp11_810 = ROcp11_29*S10+ROcp11_88*C10;
ROcp11_910 = ROcp11_39*S10+ROcp11_98*C10;
RLcp11_17 = s->dpt[1][1]*ROcp11_16+s->dpt[3][1]*S5+ROcp11_46*s->dpt[2][1];
RLcp11_27 = s->dpt[1][1]*ROcp11_26+s->dpt[3][1]*ROcp11_85+ROcp11_56*s->dpt[2][1];
RLcp11_37 = s->dpt[1][1]*ROcp11_36+s->dpt[3][1]*ROcp11_95+ROcp11_66*s->dpt[2][1];
OMcp11_17 = OMcp11_16+ROcp11_46*qd[7];
OMcp11_27 = OMcp11_26+ROcp11_56*qd[7];
OMcp11_37 = OMcp11_36+ROcp11_66*qd[7];
ORcp11_17 = OMcp11_26*RLcp11_37-OMcp11_36*RLcp11_27;
ORcp11_27 = -(OMcp11_16*RLcp11_37-OMcp11_36*RLcp11_17);
ORcp11_37 = OMcp11_16*RLcp11_27-OMcp11_26*RLcp11_17;
OPcp11_17 = OPcp11_16+ROcp11_46*qdd[7]+qd[7]*(OMcp11_26*ROcp11_66-OMcp11_36*ROcp11_56);
OPcp11_27 = OPcp11_26+ROcp11_56*qdd[7]-qd[7]*(OMcp11_16*ROcp11_66-OMcp11_36*ROcp11_46);
OPcp11_37 = OPcp11_36+ROcp11_66*qdd[7]+qd[7]*(OMcp11_16*ROcp11_56-OMcp11_26*ROcp11_46);
RLcp11_18 = s->dpt[1][6]*ROcp11_17+s->dpt[3][6]*ROcp11_77+ROcp11_46*s->dpt[2][6];
RLcp11_28 = s->dpt[1][6]*ROcp11_27+s->dpt[3][6]*ROcp11_87+ROcp11_56*s->dpt[2][6];
RLcp11_38 = s->dpt[1][6]*ROcp11_37+s->dpt[3][6]*ROcp11_97+ROcp11_66*s->dpt[2][6];
OMcp11_18 = OMcp11_17+ROcp11_17*qd[8];
OMcp11_28 = OMcp11_27+ROcp11_27*qd[8];
OMcp11_38 = OMcp11_37+ROcp11_37*qd[8];
ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
OPcp11_18 = OPcp11_17+ROcp11_17*qdd[8]+qd[8]*(OMcp11_27*ROcp11_37-OMcp11_37*ROcp11_27);
OPcp11_28 = OPcp11_27+ROcp11_27*qdd[8]-qd[8]*(OMcp11_17*ROcp11_37-OMcp11_37*ROcp11_17);
OPcp11_38 = OPcp11_37+ROcp11_37*qdd[8]+qd[8]*(OMcp11_17*ROcp11_27-OMcp11_27*ROcp11_17);
RLcp11_19 = s->dpt[1][8]*ROcp11_17+s->dpt[2][8]*ROcp11_48+ROcp11_78*s->dpt[3][8];
RLcp11_29 = s->dpt[1][8]*ROcp11_27+s->dpt[2][8]*ROcp11_58+ROcp11_88*s->dpt[3][8];
RLcp11_39 = s->dpt[1][8]*ROcp11_37+s->dpt[2][8]*ROcp11_68+ROcp11_98*s->dpt[3][8];
OMcp11_19 = OMcp11_18+ROcp11_78*qd[9];
OMcp11_29 = OMcp11_28+ROcp11_88*qd[9];
OMcp11_39 = OMcp11_38+ROcp11_98*qd[9];
ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
OPcp11_19 = OPcp11_18+ROcp11_78*qdd[9]+qd[9]*(OMcp11_28*ROcp11_98-OMcp11_38*ROcp11_88);
OPcp11_29 = OPcp11_28+ROcp11_88*qdd[9]-qd[9]*(OMcp11_18*ROcp11_98-OMcp11_38*ROcp11_78);
OPcp11_39 = OPcp11_38+ROcp11_98*qdd[9]+qd[9]*(OMcp11_18*ROcp11_88-OMcp11_28*ROcp11_78);
RLcp11_110 = s->dpt[1][10]*ROcp11_19+s->dpt[2][10]*ROcp11_49+ROcp11_78*s->dpt[3][10];
RLcp11_210 = s->dpt[1][10]*ROcp11_29+s->dpt[2][10]*ROcp11_59+ROcp11_88*s->dpt[3][10];
RLcp11_310 = s->dpt[1][10]*ROcp11_39+s->dpt[2][10]*ROcp11_69+ROcp11_98*s->dpt[3][10];
OMcp11_110 = OMcp11_19+ROcp11_49*qd[10];
OMcp11_210 = OMcp11_29+ROcp11_59*qd[10];
OMcp11_310 = OMcp11_39+ROcp11_69*qd[10];
ORcp11_110 = OMcp11_29*RLcp11_310-OMcp11_39*RLcp11_210;
ORcp11_210 = -(OMcp11_19*RLcp11_310-OMcp11_39*RLcp11_110);
ORcp11_310 = OMcp11_19*RLcp11_210-OMcp11_29*RLcp11_110;
OPcp11_110 = OPcp11_19+ROcp11_49*qdd[10]+qd[10]*(OMcp11_29*ROcp11_69-OMcp11_39*ROcp11_59);
OPcp11_210 = OPcp11_29+ROcp11_59*qdd[10]-qd[10]*(OMcp11_19*ROcp11_69-OMcp11_39*ROcp11_49);
OPcp11_310 = OPcp11_39+ROcp11_69*qdd[10]+qd[10]*(OMcp11_19*ROcp11_59-OMcp11_29*ROcp11_49);
RLcp11_141 = s->dpt[1][12]*ROcp11_110+s->dpt[2][12]*ROcp11_49+ROcp11_710*s->dpt[3][12];
RLcp11_241 = s->dpt[1][12]*ROcp11_210+s->dpt[2][12]*ROcp11_59+ROcp11_810*s->dpt[3][12];
RLcp11_341 = s->dpt[1][12]*ROcp11_310+s->dpt[2][12]*ROcp11_69+ROcp11_910*s->dpt[3][12];
POcp11_141 = RLcp11_110+RLcp11_141+RLcp11_17+RLcp11_18+RLcp11_19+q[1];
POcp11_241 = RLcp11_210+RLcp11_241+RLcp11_27+RLcp11_28+RLcp11_29+q[2];
POcp11_341 = RLcp11_310+RLcp11_341+RLcp11_37+RLcp11_38+RLcp11_39+q[3];
JTcp11_241_4 = -(RLcp11_310+RLcp11_341+RLcp11_37+RLcp11_38+RLcp11_39);
JTcp11_341_4 = RLcp11_210+RLcp11_241+RLcp11_27+RLcp11_28+RLcp11_29;
JTcp11_141_5 = C4*(RLcp11_310+RLcp11_37+RLcp11_38+RLcp11_39)-S4*(RLcp11_210+RLcp11_29)-S4*(RLcp11_27+RLcp11_28)-
 RLcp11_241*S4+RLcp11_341*C4;
JTcp11_241_5 = S4*(RLcp11_110+RLcp11_141+RLcp11_17+RLcp11_18+RLcp11_19);
JTcp11_341_5 = -C4*(RLcp11_110+RLcp11_141+RLcp11_17+RLcp11_18+RLcp11_19);
JTcp11_141_6 = ROcp11_85*(RLcp11_310+RLcp11_37+RLcp11_38+RLcp11_39)-ROcp11_95*(RLcp11_210+RLcp11_29)-ROcp11_95*(
 RLcp11_27+RLcp11_28)-RLcp11_241*ROcp11_95+RLcp11_341*ROcp11_85;
JTcp11_241_6 = -(RLcp11_341*S5-ROcp11_95*(RLcp11_110+RLcp11_141+RLcp11_17+RLcp11_18+RLcp11_19)+S5*(RLcp11_310+
 RLcp11_39)+S5*(RLcp11_37+RLcp11_38));
JTcp11_341_6 = RLcp11_241*S5-ROcp11_85*(RLcp11_110+RLcp11_141+RLcp11_17+RLcp11_18+RLcp11_19)+S5*(RLcp11_210+RLcp11_29)
 +S5*(RLcp11_27+RLcp11_28);
JTcp11_141_7 = ROcp11_56*(RLcp11_310+RLcp11_341+RLcp11_38+RLcp11_39)-ROcp11_66*(RLcp11_210+RLcp11_241)-ROcp11_66*(
 RLcp11_28+RLcp11_29);
JTcp11_241_7 = -(ROcp11_46*(RLcp11_310+RLcp11_341+RLcp11_38+RLcp11_39)-ROcp11_66*(RLcp11_110+RLcp11_141)-ROcp11_66*(
 RLcp11_18+RLcp11_19));
JTcp11_341_7 = ROcp11_46*(RLcp11_210+RLcp11_241+RLcp11_28+RLcp11_29)-ROcp11_56*(RLcp11_110+RLcp11_141)-ROcp11_56*(
 RLcp11_18+RLcp11_19);
JTcp11_141_8 = ROcp11_27*(RLcp11_310+RLcp11_39)-ROcp11_37*(RLcp11_210+RLcp11_29)-RLcp11_241*ROcp11_37+RLcp11_341*
 ROcp11_27;
JTcp11_241_8 = RLcp11_141*ROcp11_37-RLcp11_341*ROcp11_17-ROcp11_17*(RLcp11_310+RLcp11_39)+ROcp11_37*(RLcp11_110+
 RLcp11_19);
JTcp11_341_8 = ROcp11_17*(RLcp11_210+RLcp11_29)-ROcp11_27*(RLcp11_110+RLcp11_19)-RLcp11_141*ROcp11_27+RLcp11_241*
 ROcp11_17;
JTcp11_141_9 = ROcp11_88*(RLcp11_310+RLcp11_341)-ROcp11_98*(RLcp11_210+RLcp11_241);
JTcp11_241_9 = -(ROcp11_78*(RLcp11_310+RLcp11_341)-ROcp11_98*(RLcp11_110+RLcp11_141));
JTcp11_341_9 = ROcp11_78*(RLcp11_210+RLcp11_241)-ROcp11_88*(RLcp11_110+RLcp11_141);
JTcp11_141_10 = -(RLcp11_241*ROcp11_69-RLcp11_341*ROcp11_59);
JTcp11_241_10 = RLcp11_141*ROcp11_69-RLcp11_341*ROcp11_49;
JTcp11_341_10 = -(RLcp11_141*ROcp11_59-RLcp11_241*ROcp11_49);
ORcp11_141 = OMcp11_210*RLcp11_341-OMcp11_310*RLcp11_241;
ORcp11_241 = -(OMcp11_110*RLcp11_341-OMcp11_310*RLcp11_141);
ORcp11_341 = OMcp11_110*RLcp11_241-OMcp11_210*RLcp11_141;
VIcp11_141 = ORcp11_110+ORcp11_141+ORcp11_17+ORcp11_18+ORcp11_19+qd[1];
VIcp11_241 = ORcp11_210+ORcp11_241+ORcp11_27+ORcp11_28+ORcp11_29+qd[2];
VIcp11_341 = ORcp11_310+ORcp11_341+ORcp11_37+ORcp11_38+ORcp11_39+qd[3];
ACcp11_141 = qdd[1]+OMcp11_210*ORcp11_341+OMcp11_26*ORcp11_37+OMcp11_27*ORcp11_38+OMcp11_28*ORcp11_39+OMcp11_29*
 ORcp11_310-OMcp11_310*ORcp11_241-OMcp11_36*ORcp11_27-OMcp11_37*ORcp11_28-OMcp11_38*ORcp11_29-OMcp11_39*ORcp11_210+OPcp11_210
 *RLcp11_341+OPcp11_26*RLcp11_37+OPcp11_27*RLcp11_38+OPcp11_28*RLcp11_39+OPcp11_29*RLcp11_310-OPcp11_310*RLcp11_241-OPcp11_36
 *RLcp11_27-OPcp11_37*RLcp11_28-OPcp11_38*RLcp11_29-OPcp11_39*RLcp11_210;
ACcp11_241 = qdd[2]-OMcp11_110*ORcp11_341-OMcp11_16*ORcp11_37-OMcp11_17*ORcp11_38-OMcp11_18*ORcp11_39-OMcp11_19*
 ORcp11_310+OMcp11_310*ORcp11_141+OMcp11_36*ORcp11_17+OMcp11_37*ORcp11_18+OMcp11_38*ORcp11_19+OMcp11_39*ORcp11_110-OPcp11_110
 *RLcp11_341-OPcp11_16*RLcp11_37-OPcp11_17*RLcp11_38-OPcp11_18*RLcp11_39-OPcp11_19*RLcp11_310+OPcp11_310*RLcp11_141+OPcp11_36
 *RLcp11_17+OPcp11_37*RLcp11_18+OPcp11_38*RLcp11_19+OPcp11_39*RLcp11_110;
ACcp11_341 = qdd[3]+OMcp11_110*ORcp11_241+OMcp11_16*ORcp11_27+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29+OMcp11_19*
 ORcp11_210-OMcp11_210*ORcp11_141-OMcp11_26*ORcp11_17-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19-OMcp11_29*ORcp11_110+OPcp11_110
 *RLcp11_241+OPcp11_16*RLcp11_27+OPcp11_17*RLcp11_28+OPcp11_18*RLcp11_29+OPcp11_19*RLcp11_210-OPcp11_210*RLcp11_141-OPcp11_26
 *RLcp11_17-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19-OPcp11_29*RLcp11_110;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp11_141;
sens->P[2] = POcp11_241;
sens->P[3] = POcp11_341;
sens->R[1][1] = ROcp11_110;
sens->R[1][2] = ROcp11_210;
sens->R[1][3] = ROcp11_310;
sens->R[2][1] = ROcp11_49;
sens->R[2][2] = ROcp11_59;
sens->R[2][3] = ROcp11_69;
sens->R[3][1] = ROcp11_710;
sens->R[3][2] = ROcp11_810;
sens->R[3][3] = ROcp11_910;
sens->V[1] = VIcp11_141;
sens->V[2] = VIcp11_241;
sens->V[3] = VIcp11_341;
sens->OM[1] = OMcp11_110;
sens->OM[2] = OMcp11_210;
sens->OM[3] = OMcp11_310;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp11_141_5;
sens->J[1][6] = JTcp11_141_6;
sens->J[1][7] = JTcp11_141_7;
sens->J[1][8] = JTcp11_141_8;
sens->J[1][9] = JTcp11_141_9;
sens->J[1][10] = JTcp11_141_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp11_241_4;
sens->J[2][5] = JTcp11_241_5;
sens->J[2][6] = JTcp11_241_6;
sens->J[2][7] = JTcp11_241_7;
sens->J[2][8] = JTcp11_241_8;
sens->J[2][9] = JTcp11_241_9;
sens->J[2][10] = JTcp11_241_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp11_341_4;
sens->J[3][5] = JTcp11_341_5;
sens->J[3][6] = JTcp11_341_6;
sens->J[3][7] = JTcp11_341_7;
sens->J[3][8] = JTcp11_341_8;
sens->J[3][9] = JTcp11_341_9;
sens->J[3][10] = JTcp11_341_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp11_46;
sens->J[4][8] = ROcp11_17;
sens->J[4][9] = ROcp11_78;
sens->J[4][10] = ROcp11_49;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp11_85;
sens->J[5][7] = ROcp11_56;
sens->J[5][8] = ROcp11_27;
sens->J[5][9] = ROcp11_88;
sens->J[5][10] = ROcp11_59;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp11_95;
sens->J[6][7] = ROcp11_66;
sens->J[6][8] = ROcp11_37;
sens->J[6][9] = ROcp11_98;
sens->J[6][10] = ROcp11_69;
sens->A[1] = ACcp11_141;
sens->A[2] = ACcp11_241;
sens->A[3] = ACcp11_341;
sens->OMP[1] = OPcp11_110;
sens->OMP[2] = OPcp11_210;
sens->OMP[3] = OPcp11_310;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
// Sensor Kinematics 


ROcp12_25 = S4*S5;
ROcp12_35 = -C4*S5;
ROcp12_85 = -S4*C5;
ROcp12_95 = C4*C5;
ROcp12_16 = C5*C6;
ROcp12_26 = ROcp12_25*C6+C4*S6;
ROcp12_36 = ROcp12_35*C6+S4*S6;
ROcp12_46 = -C5*S6;
ROcp12_56 = -(ROcp12_25*S6-C4*C6);
ROcp12_66 = -(ROcp12_35*S6-S4*C6);
OMcp12_25 = qd[5]*C4;
OMcp12_35 = qd[5]*S4;
OMcp12_16 = qd[4]+qd[6]*S5;
OMcp12_26 = OMcp12_25+ROcp12_85*qd[6];
OMcp12_36 = OMcp12_35+ROcp12_95*qd[6];
OPcp12_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp12_26 = ROcp12_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp12_35*S5-ROcp12_95*qd[4]);
OPcp12_36 = ROcp12_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp12_25*S5-ROcp12_85*qd[4]);

// = = Block_1_0_0_13_0_2 = = 
 
// Sensor Kinematics 


ROcp12_17 = ROcp12_16*C7-S5*S7;
ROcp12_27 = ROcp12_26*C7-ROcp12_85*S7;
ROcp12_37 = ROcp12_36*C7-ROcp12_95*S7;
ROcp12_77 = ROcp12_16*S7+S5*C7;
ROcp12_87 = ROcp12_26*S7+ROcp12_85*C7;
ROcp12_97 = ROcp12_36*S7+ROcp12_95*C7;
ROcp12_48 = ROcp12_46*C8+ROcp12_77*S8;
ROcp12_58 = ROcp12_56*C8+ROcp12_87*S8;
ROcp12_68 = ROcp12_66*C8+ROcp12_97*S8;
ROcp12_78 = -(ROcp12_46*S8-ROcp12_77*C8);
ROcp12_88 = -(ROcp12_56*S8-ROcp12_87*C8);
ROcp12_98 = -(ROcp12_66*S8-ROcp12_97*C8);
ROcp12_19 = ROcp12_17*C9+ROcp12_48*S9;
ROcp12_29 = ROcp12_27*C9+ROcp12_58*S9;
ROcp12_39 = ROcp12_37*C9+ROcp12_68*S9;
ROcp12_49 = -(ROcp12_17*S9-ROcp12_48*C9);
ROcp12_59 = -(ROcp12_27*S9-ROcp12_58*C9);
ROcp12_69 = -(ROcp12_37*S9-ROcp12_68*C9);
ROcp12_110 = ROcp12_19*C10-ROcp12_78*S10;
ROcp12_210 = ROcp12_29*C10-ROcp12_88*S10;
ROcp12_310 = ROcp12_39*C10-ROcp12_98*S10;
ROcp12_710 = ROcp12_19*S10+ROcp12_78*C10;
ROcp12_810 = ROcp12_29*S10+ROcp12_88*C10;
ROcp12_910 = ROcp12_39*S10+ROcp12_98*C10;
RLcp12_17 = s->dpt[1][1]*ROcp12_16+s->dpt[3][1]*S5+ROcp12_46*s->dpt[2][1];
RLcp12_27 = s->dpt[1][1]*ROcp12_26+s->dpt[3][1]*ROcp12_85+ROcp12_56*s->dpt[2][1];
RLcp12_37 = s->dpt[1][1]*ROcp12_36+s->dpt[3][1]*ROcp12_95+ROcp12_66*s->dpt[2][1];
OMcp12_17 = OMcp12_16+ROcp12_46*qd[7];
OMcp12_27 = OMcp12_26+ROcp12_56*qd[7];
OMcp12_37 = OMcp12_36+ROcp12_66*qd[7];
ORcp12_17 = OMcp12_26*RLcp12_37-OMcp12_36*RLcp12_27;
ORcp12_27 = -(OMcp12_16*RLcp12_37-OMcp12_36*RLcp12_17);
ORcp12_37 = OMcp12_16*RLcp12_27-OMcp12_26*RLcp12_17;
OPcp12_17 = OPcp12_16+ROcp12_46*qdd[7]+qd[7]*(OMcp12_26*ROcp12_66-OMcp12_36*ROcp12_56);
OPcp12_27 = OPcp12_26+ROcp12_56*qdd[7]-qd[7]*(OMcp12_16*ROcp12_66-OMcp12_36*ROcp12_46);
OPcp12_37 = OPcp12_36+ROcp12_66*qdd[7]+qd[7]*(OMcp12_16*ROcp12_56-OMcp12_26*ROcp12_46);
RLcp12_18 = s->dpt[1][6]*ROcp12_17+s->dpt[3][6]*ROcp12_77+ROcp12_46*s->dpt[2][6];
RLcp12_28 = s->dpt[1][6]*ROcp12_27+s->dpt[3][6]*ROcp12_87+ROcp12_56*s->dpt[2][6];
RLcp12_38 = s->dpt[1][6]*ROcp12_37+s->dpt[3][6]*ROcp12_97+ROcp12_66*s->dpt[2][6];
OMcp12_18 = OMcp12_17+ROcp12_17*qd[8];
OMcp12_28 = OMcp12_27+ROcp12_27*qd[8];
OMcp12_38 = OMcp12_37+ROcp12_37*qd[8];
ORcp12_18 = OMcp12_27*RLcp12_38-OMcp12_37*RLcp12_28;
ORcp12_28 = -(OMcp12_17*RLcp12_38-OMcp12_37*RLcp12_18);
ORcp12_38 = OMcp12_17*RLcp12_28-OMcp12_27*RLcp12_18;
OPcp12_18 = OPcp12_17+ROcp12_17*qdd[8]+qd[8]*(OMcp12_27*ROcp12_37-OMcp12_37*ROcp12_27);
OPcp12_28 = OPcp12_27+ROcp12_27*qdd[8]-qd[8]*(OMcp12_17*ROcp12_37-OMcp12_37*ROcp12_17);
OPcp12_38 = OPcp12_37+ROcp12_37*qdd[8]+qd[8]*(OMcp12_17*ROcp12_27-OMcp12_27*ROcp12_17);
RLcp12_19 = s->dpt[1][8]*ROcp12_17+s->dpt[2][8]*ROcp12_48+ROcp12_78*s->dpt[3][8];
RLcp12_29 = s->dpt[1][8]*ROcp12_27+s->dpt[2][8]*ROcp12_58+ROcp12_88*s->dpt[3][8];
RLcp12_39 = s->dpt[1][8]*ROcp12_37+s->dpt[2][8]*ROcp12_68+ROcp12_98*s->dpt[3][8];
OMcp12_19 = OMcp12_18+ROcp12_78*qd[9];
OMcp12_29 = OMcp12_28+ROcp12_88*qd[9];
OMcp12_39 = OMcp12_38+ROcp12_98*qd[9];
ORcp12_19 = OMcp12_28*RLcp12_39-OMcp12_38*RLcp12_29;
ORcp12_29 = -(OMcp12_18*RLcp12_39-OMcp12_38*RLcp12_19);
ORcp12_39 = OMcp12_18*RLcp12_29-OMcp12_28*RLcp12_19;
OPcp12_19 = OPcp12_18+ROcp12_78*qdd[9]+qd[9]*(OMcp12_28*ROcp12_98-OMcp12_38*ROcp12_88);
OPcp12_29 = OPcp12_28+ROcp12_88*qdd[9]-qd[9]*(OMcp12_18*ROcp12_98-OMcp12_38*ROcp12_78);
OPcp12_39 = OPcp12_38+ROcp12_98*qdd[9]+qd[9]*(OMcp12_18*ROcp12_88-OMcp12_28*ROcp12_78);
RLcp12_110 = s->dpt[1][10]*ROcp12_19+s->dpt[2][10]*ROcp12_49+ROcp12_78*s->dpt[3][10];
RLcp12_210 = s->dpt[1][10]*ROcp12_29+s->dpt[2][10]*ROcp12_59+ROcp12_88*s->dpt[3][10];
RLcp12_310 = s->dpt[1][10]*ROcp12_39+s->dpt[2][10]*ROcp12_69+ROcp12_98*s->dpt[3][10];
OMcp12_110 = OMcp12_19+ROcp12_49*qd[10];
OMcp12_210 = OMcp12_29+ROcp12_59*qd[10];
OMcp12_310 = OMcp12_39+ROcp12_69*qd[10];
ORcp12_110 = OMcp12_29*RLcp12_310-OMcp12_39*RLcp12_210;
ORcp12_210 = -(OMcp12_19*RLcp12_310-OMcp12_39*RLcp12_110);
ORcp12_310 = OMcp12_19*RLcp12_210-OMcp12_29*RLcp12_110;
OPcp12_110 = OPcp12_19+ROcp12_49*qdd[10]+qd[10]*(OMcp12_29*ROcp12_69-OMcp12_39*ROcp12_59);
OPcp12_210 = OPcp12_29+ROcp12_59*qdd[10]-qd[10]*(OMcp12_19*ROcp12_69-OMcp12_39*ROcp12_49);
OPcp12_310 = OPcp12_39+ROcp12_69*qdd[10]+qd[10]*(OMcp12_19*ROcp12_59-OMcp12_29*ROcp12_49);
RLcp12_142 = ROcp12_110*s->dpt[1][13]+ROcp12_49*s->dpt[2][13]+ROcp12_710*s->dpt[3][13];
RLcp12_242 = ROcp12_210*s->dpt[1][13]+ROcp12_59*s->dpt[2][13]+ROcp12_810*s->dpt[3][13];
RLcp12_342 = ROcp12_310*s->dpt[1][13]+ROcp12_69*s->dpt[2][13]+ROcp12_910*s->dpt[3][13];
POcp12_142 = RLcp12_110+RLcp12_142+RLcp12_17+RLcp12_18+RLcp12_19+q[1];
POcp12_242 = RLcp12_210+RLcp12_242+RLcp12_27+RLcp12_28+RLcp12_29+q[2];
POcp12_342 = RLcp12_310+RLcp12_342+RLcp12_37+RLcp12_38+RLcp12_39+q[3];
JTcp12_242_4 = -(RLcp12_310+RLcp12_342+RLcp12_37+RLcp12_38+RLcp12_39);
JTcp12_342_4 = RLcp12_210+RLcp12_242+RLcp12_27+RLcp12_28+RLcp12_29;
JTcp12_142_5 = C4*(RLcp12_310+RLcp12_37+RLcp12_38+RLcp12_39)-S4*(RLcp12_210+RLcp12_29)-S4*(RLcp12_27+RLcp12_28)-
 RLcp12_242*S4+RLcp12_342*C4;
JTcp12_242_5 = S4*(RLcp12_110+RLcp12_142+RLcp12_17+RLcp12_18+RLcp12_19);
JTcp12_342_5 = -C4*(RLcp12_110+RLcp12_142+RLcp12_17+RLcp12_18+RLcp12_19);
JTcp12_142_6 = ROcp12_85*(RLcp12_310+RLcp12_37+RLcp12_38+RLcp12_39)-ROcp12_95*(RLcp12_210+RLcp12_29)-ROcp12_95*(
 RLcp12_27+RLcp12_28)-RLcp12_242*ROcp12_95+RLcp12_342*ROcp12_85;
JTcp12_242_6 = -(RLcp12_342*S5-ROcp12_95*(RLcp12_110+RLcp12_142+RLcp12_17+RLcp12_18+RLcp12_19)+S5*(RLcp12_310+
 RLcp12_39)+S5*(RLcp12_37+RLcp12_38));
JTcp12_342_6 = RLcp12_242*S5-ROcp12_85*(RLcp12_110+RLcp12_142+RLcp12_17+RLcp12_18+RLcp12_19)+S5*(RLcp12_210+RLcp12_29)
 +S5*(RLcp12_27+RLcp12_28);
JTcp12_142_7 = ROcp12_56*(RLcp12_310+RLcp12_342+RLcp12_38+RLcp12_39)-ROcp12_66*(RLcp12_210+RLcp12_242)-ROcp12_66*(
 RLcp12_28+RLcp12_29);
JTcp12_242_7 = -(ROcp12_46*(RLcp12_310+RLcp12_342+RLcp12_38+RLcp12_39)-ROcp12_66*(RLcp12_110+RLcp12_142)-ROcp12_66*(
 RLcp12_18+RLcp12_19));
JTcp12_342_7 = ROcp12_46*(RLcp12_210+RLcp12_242+RLcp12_28+RLcp12_29)-ROcp12_56*(RLcp12_110+RLcp12_142)-ROcp12_56*(
 RLcp12_18+RLcp12_19);
JTcp12_142_8 = ROcp12_27*(RLcp12_310+RLcp12_39)-ROcp12_37*(RLcp12_210+RLcp12_29)-RLcp12_242*ROcp12_37+RLcp12_342*
 ROcp12_27;
JTcp12_242_8 = RLcp12_142*ROcp12_37-RLcp12_342*ROcp12_17-ROcp12_17*(RLcp12_310+RLcp12_39)+ROcp12_37*(RLcp12_110+
 RLcp12_19);
JTcp12_342_8 = ROcp12_17*(RLcp12_210+RLcp12_29)-ROcp12_27*(RLcp12_110+RLcp12_19)-RLcp12_142*ROcp12_27+RLcp12_242*
 ROcp12_17;
JTcp12_142_9 = ROcp12_88*(RLcp12_310+RLcp12_342)-ROcp12_98*(RLcp12_210+RLcp12_242);
JTcp12_242_9 = -(ROcp12_78*(RLcp12_310+RLcp12_342)-ROcp12_98*(RLcp12_110+RLcp12_142));
JTcp12_342_9 = ROcp12_78*(RLcp12_210+RLcp12_242)-ROcp12_88*(RLcp12_110+RLcp12_142);
JTcp12_142_10 = -(RLcp12_242*ROcp12_69-RLcp12_342*ROcp12_59);
JTcp12_242_10 = RLcp12_142*ROcp12_69-RLcp12_342*ROcp12_49;
JTcp12_342_10 = -(RLcp12_142*ROcp12_59-RLcp12_242*ROcp12_49);
ORcp12_142 = OMcp12_210*RLcp12_342-OMcp12_310*RLcp12_242;
ORcp12_242 = -(OMcp12_110*RLcp12_342-OMcp12_310*RLcp12_142);
ORcp12_342 = OMcp12_110*RLcp12_242-OMcp12_210*RLcp12_142;
VIcp12_142 = ORcp12_110+ORcp12_142+ORcp12_17+ORcp12_18+ORcp12_19+qd[1];
VIcp12_242 = ORcp12_210+ORcp12_242+ORcp12_27+ORcp12_28+ORcp12_29+qd[2];
VIcp12_342 = ORcp12_310+ORcp12_342+ORcp12_37+ORcp12_38+ORcp12_39+qd[3];
ACcp12_142 = qdd[1]+OMcp12_210*ORcp12_342+OMcp12_26*ORcp12_37+OMcp12_27*ORcp12_38+OMcp12_28*ORcp12_39+OMcp12_29*
 ORcp12_310-OMcp12_310*ORcp12_242-OMcp12_36*ORcp12_27-OMcp12_37*ORcp12_28-OMcp12_38*ORcp12_29-OMcp12_39*ORcp12_210+OPcp12_210
 *RLcp12_342+OPcp12_26*RLcp12_37+OPcp12_27*RLcp12_38+OPcp12_28*RLcp12_39+OPcp12_29*RLcp12_310-OPcp12_310*RLcp12_242-OPcp12_36
 *RLcp12_27-OPcp12_37*RLcp12_28-OPcp12_38*RLcp12_29-OPcp12_39*RLcp12_210;
ACcp12_242 = qdd[2]-OMcp12_110*ORcp12_342-OMcp12_16*ORcp12_37-OMcp12_17*ORcp12_38-OMcp12_18*ORcp12_39-OMcp12_19*
 ORcp12_310+OMcp12_310*ORcp12_142+OMcp12_36*ORcp12_17+OMcp12_37*ORcp12_18+OMcp12_38*ORcp12_19+OMcp12_39*ORcp12_110-OPcp12_110
 *RLcp12_342-OPcp12_16*RLcp12_37-OPcp12_17*RLcp12_38-OPcp12_18*RLcp12_39-OPcp12_19*RLcp12_310+OPcp12_310*RLcp12_142+OPcp12_36
 *RLcp12_17+OPcp12_37*RLcp12_18+OPcp12_38*RLcp12_19+OPcp12_39*RLcp12_110;
ACcp12_342 = qdd[3]+OMcp12_110*ORcp12_242+OMcp12_16*ORcp12_27+OMcp12_17*ORcp12_28+OMcp12_18*ORcp12_29+OMcp12_19*
 ORcp12_210-OMcp12_210*ORcp12_142-OMcp12_26*ORcp12_17-OMcp12_27*ORcp12_18-OMcp12_28*ORcp12_19-OMcp12_29*ORcp12_110+OPcp12_110
 *RLcp12_242+OPcp12_16*RLcp12_27+OPcp12_17*RLcp12_28+OPcp12_18*RLcp12_29+OPcp12_19*RLcp12_210-OPcp12_210*RLcp12_142-OPcp12_26
 *RLcp12_17-OPcp12_27*RLcp12_18-OPcp12_28*RLcp12_19-OPcp12_29*RLcp12_110;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp12_142;
sens->P[2] = POcp12_242;
sens->P[3] = POcp12_342;
sens->R[1][1] = ROcp12_110;
sens->R[1][2] = ROcp12_210;
sens->R[1][3] = ROcp12_310;
sens->R[2][1] = ROcp12_49;
sens->R[2][2] = ROcp12_59;
sens->R[2][3] = ROcp12_69;
sens->R[3][1] = ROcp12_710;
sens->R[3][2] = ROcp12_810;
sens->R[3][3] = ROcp12_910;
sens->V[1] = VIcp12_142;
sens->V[2] = VIcp12_242;
sens->V[3] = VIcp12_342;
sens->OM[1] = OMcp12_110;
sens->OM[2] = OMcp12_210;
sens->OM[3] = OMcp12_310;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp12_142_5;
sens->J[1][6] = JTcp12_142_6;
sens->J[1][7] = JTcp12_142_7;
sens->J[1][8] = JTcp12_142_8;
sens->J[1][9] = JTcp12_142_9;
sens->J[1][10] = JTcp12_142_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp12_242_4;
sens->J[2][5] = JTcp12_242_5;
sens->J[2][6] = JTcp12_242_6;
sens->J[2][7] = JTcp12_242_7;
sens->J[2][8] = JTcp12_242_8;
sens->J[2][9] = JTcp12_242_9;
sens->J[2][10] = JTcp12_242_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp12_342_4;
sens->J[3][5] = JTcp12_342_5;
sens->J[3][6] = JTcp12_342_6;
sens->J[3][7] = JTcp12_342_7;
sens->J[3][8] = JTcp12_342_8;
sens->J[3][9] = JTcp12_342_9;
sens->J[3][10] = JTcp12_342_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp12_46;
sens->J[4][8] = ROcp12_17;
sens->J[4][9] = ROcp12_78;
sens->J[4][10] = ROcp12_49;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp12_85;
sens->J[5][7] = ROcp12_56;
sens->J[5][8] = ROcp12_27;
sens->J[5][9] = ROcp12_88;
sens->J[5][10] = ROcp12_59;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp12_95;
sens->J[6][7] = ROcp12_66;
sens->J[6][8] = ROcp12_37;
sens->J[6][9] = ROcp12_98;
sens->J[6][10] = ROcp12_69;
sens->A[1] = ACcp12_142;
sens->A[2] = ACcp12_242;
sens->A[3] = ACcp12_342;
sens->OMP[1] = OPcp12_110;
sens->OMP[2] = OPcp12_210;
sens->OMP[3] = OPcp12_310;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
// Sensor Kinematics 


ROcp13_25 = S4*S5;
ROcp13_35 = -C4*S5;
ROcp13_85 = -S4*C5;
ROcp13_95 = C4*C5;
ROcp13_16 = C5*C6;
ROcp13_26 = ROcp13_25*C6+C4*S6;
ROcp13_36 = ROcp13_35*C6+S4*S6;
ROcp13_46 = -C5*S6;
ROcp13_56 = -(ROcp13_25*S6-C4*C6);
ROcp13_66 = -(ROcp13_35*S6-S4*C6);
OMcp13_25 = qd[5]*C4;
OMcp13_35 = qd[5]*S4;
OMcp13_16 = qd[4]+qd[6]*S5;
OMcp13_26 = OMcp13_25+ROcp13_85*qd[6];
OMcp13_36 = OMcp13_35+ROcp13_95*qd[6];
OPcp13_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp13_26 = ROcp13_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp13_35*S5-ROcp13_95*qd[4]);
OPcp13_36 = ROcp13_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp13_25*S5-ROcp13_85*qd[4]);

// = = Block_1_0_0_14_0_2 = = 
 
// Sensor Kinematics 


ROcp13_17 = ROcp13_16*C7-S5*S7;
ROcp13_27 = ROcp13_26*C7-ROcp13_85*S7;
ROcp13_37 = ROcp13_36*C7-ROcp13_95*S7;
ROcp13_77 = ROcp13_16*S7+S5*C7;
ROcp13_87 = ROcp13_26*S7+ROcp13_85*C7;
ROcp13_97 = ROcp13_36*S7+ROcp13_95*C7;
ROcp13_48 = ROcp13_46*C8+ROcp13_77*S8;
ROcp13_58 = ROcp13_56*C8+ROcp13_87*S8;
ROcp13_68 = ROcp13_66*C8+ROcp13_97*S8;
ROcp13_78 = -(ROcp13_46*S8-ROcp13_77*C8);
ROcp13_88 = -(ROcp13_56*S8-ROcp13_87*C8);
ROcp13_98 = -(ROcp13_66*S8-ROcp13_97*C8);
ROcp13_19 = ROcp13_17*C9+ROcp13_48*S9;
ROcp13_29 = ROcp13_27*C9+ROcp13_58*S9;
ROcp13_39 = ROcp13_37*C9+ROcp13_68*S9;
ROcp13_49 = -(ROcp13_17*S9-ROcp13_48*C9);
ROcp13_59 = -(ROcp13_27*S9-ROcp13_58*C9);
ROcp13_69 = -(ROcp13_37*S9-ROcp13_68*C9);
ROcp13_110 = ROcp13_19*C10-ROcp13_78*S10;
ROcp13_210 = ROcp13_29*C10-ROcp13_88*S10;
ROcp13_310 = ROcp13_39*C10-ROcp13_98*S10;
ROcp13_710 = ROcp13_19*S10+ROcp13_78*C10;
ROcp13_810 = ROcp13_29*S10+ROcp13_88*C10;
ROcp13_910 = ROcp13_39*S10+ROcp13_98*C10;
ROcp13_411 = ROcp13_49*C11+ROcp13_710*S11;
ROcp13_511 = ROcp13_59*C11+ROcp13_810*S11;
ROcp13_611 = ROcp13_69*C11+ROcp13_910*S11;
ROcp13_711 = -(ROcp13_49*S11-ROcp13_710*C11);
ROcp13_811 = -(ROcp13_59*S11-ROcp13_810*C11);
ROcp13_911 = -(ROcp13_69*S11-ROcp13_910*C11);
RLcp13_17 = s->dpt[1][1]*ROcp13_16+s->dpt[3][1]*S5+ROcp13_46*s->dpt[2][1];
RLcp13_27 = s->dpt[1][1]*ROcp13_26+s->dpt[3][1]*ROcp13_85+ROcp13_56*s->dpt[2][1];
RLcp13_37 = s->dpt[1][1]*ROcp13_36+s->dpt[3][1]*ROcp13_95+ROcp13_66*s->dpt[2][1];
OMcp13_17 = OMcp13_16+ROcp13_46*qd[7];
OMcp13_27 = OMcp13_26+ROcp13_56*qd[7];
OMcp13_37 = OMcp13_36+ROcp13_66*qd[7];
ORcp13_17 = OMcp13_26*RLcp13_37-OMcp13_36*RLcp13_27;
ORcp13_27 = -(OMcp13_16*RLcp13_37-OMcp13_36*RLcp13_17);
ORcp13_37 = OMcp13_16*RLcp13_27-OMcp13_26*RLcp13_17;
OPcp13_17 = OPcp13_16+ROcp13_46*qdd[7]+qd[7]*(OMcp13_26*ROcp13_66-OMcp13_36*ROcp13_56);
OPcp13_27 = OPcp13_26+ROcp13_56*qdd[7]-qd[7]*(OMcp13_16*ROcp13_66-OMcp13_36*ROcp13_46);
OPcp13_37 = OPcp13_36+ROcp13_66*qdd[7]+qd[7]*(OMcp13_16*ROcp13_56-OMcp13_26*ROcp13_46);
RLcp13_18 = s->dpt[1][6]*ROcp13_17+s->dpt[3][6]*ROcp13_77+ROcp13_46*s->dpt[2][6];
RLcp13_28 = s->dpt[1][6]*ROcp13_27+s->dpt[3][6]*ROcp13_87+ROcp13_56*s->dpt[2][6];
RLcp13_38 = s->dpt[1][6]*ROcp13_37+s->dpt[3][6]*ROcp13_97+ROcp13_66*s->dpt[2][6];
OMcp13_18 = OMcp13_17+ROcp13_17*qd[8];
OMcp13_28 = OMcp13_27+ROcp13_27*qd[8];
OMcp13_38 = OMcp13_37+ROcp13_37*qd[8];
ORcp13_18 = OMcp13_27*RLcp13_38-OMcp13_37*RLcp13_28;
ORcp13_28 = -(OMcp13_17*RLcp13_38-OMcp13_37*RLcp13_18);
ORcp13_38 = OMcp13_17*RLcp13_28-OMcp13_27*RLcp13_18;
OPcp13_18 = OPcp13_17+ROcp13_17*qdd[8]+qd[8]*(OMcp13_27*ROcp13_37-OMcp13_37*ROcp13_27);
OPcp13_28 = OPcp13_27+ROcp13_27*qdd[8]-qd[8]*(OMcp13_17*ROcp13_37-OMcp13_37*ROcp13_17);
OPcp13_38 = OPcp13_37+ROcp13_37*qdd[8]+qd[8]*(OMcp13_17*ROcp13_27-OMcp13_27*ROcp13_17);
RLcp13_19 = s->dpt[1][8]*ROcp13_17+s->dpt[2][8]*ROcp13_48+ROcp13_78*s->dpt[3][8];
RLcp13_29 = s->dpt[1][8]*ROcp13_27+s->dpt[2][8]*ROcp13_58+ROcp13_88*s->dpt[3][8];
RLcp13_39 = s->dpt[1][8]*ROcp13_37+s->dpt[2][8]*ROcp13_68+ROcp13_98*s->dpt[3][8];
OMcp13_19 = OMcp13_18+ROcp13_78*qd[9];
OMcp13_29 = OMcp13_28+ROcp13_88*qd[9];
OMcp13_39 = OMcp13_38+ROcp13_98*qd[9];
ORcp13_19 = OMcp13_28*RLcp13_39-OMcp13_38*RLcp13_29;
ORcp13_29 = -(OMcp13_18*RLcp13_39-OMcp13_38*RLcp13_19);
ORcp13_39 = OMcp13_18*RLcp13_29-OMcp13_28*RLcp13_19;
OPcp13_19 = OPcp13_18+ROcp13_78*qdd[9]+qd[9]*(OMcp13_28*ROcp13_98-OMcp13_38*ROcp13_88);
OPcp13_29 = OPcp13_28+ROcp13_88*qdd[9]-qd[9]*(OMcp13_18*ROcp13_98-OMcp13_38*ROcp13_78);
OPcp13_39 = OPcp13_38+ROcp13_98*qdd[9]+qd[9]*(OMcp13_18*ROcp13_88-OMcp13_28*ROcp13_78);
RLcp13_110 = s->dpt[1][10]*ROcp13_19+s->dpt[2][10]*ROcp13_49+ROcp13_78*s->dpt[3][10];
RLcp13_210 = s->dpt[1][10]*ROcp13_29+s->dpt[2][10]*ROcp13_59+ROcp13_88*s->dpt[3][10];
RLcp13_310 = s->dpt[1][10]*ROcp13_39+s->dpt[2][10]*ROcp13_69+ROcp13_98*s->dpt[3][10];
OMcp13_110 = OMcp13_19+ROcp13_49*qd[10];
OMcp13_210 = OMcp13_29+ROcp13_59*qd[10];
OMcp13_310 = OMcp13_39+ROcp13_69*qd[10];
ORcp13_110 = OMcp13_29*RLcp13_310-OMcp13_39*RLcp13_210;
ORcp13_210 = -(OMcp13_19*RLcp13_310-OMcp13_39*RLcp13_110);
ORcp13_310 = OMcp13_19*RLcp13_210-OMcp13_29*RLcp13_110;
OPcp13_110 = OPcp13_19+ROcp13_49*qdd[10]+qd[10]*(OMcp13_29*ROcp13_69-OMcp13_39*ROcp13_59);
OPcp13_210 = OPcp13_29+ROcp13_59*qdd[10]-qd[10]*(OMcp13_19*ROcp13_69-OMcp13_39*ROcp13_49);
OPcp13_310 = OPcp13_39+ROcp13_69*qdd[10]+qd[10]*(OMcp13_19*ROcp13_59-OMcp13_29*ROcp13_49);
RLcp13_111 = s->dpt[1][12]*ROcp13_110+s->dpt[2][12]*ROcp13_49+ROcp13_710*s->dpt[3][12];
RLcp13_211 = s->dpt[1][12]*ROcp13_210+s->dpt[2][12]*ROcp13_59+ROcp13_810*s->dpt[3][12];
RLcp13_311 = s->dpt[1][12]*ROcp13_310+s->dpt[2][12]*ROcp13_69+ROcp13_910*s->dpt[3][12];
OMcp13_111 = OMcp13_110+ROcp13_110*qd[11];
OMcp13_211 = OMcp13_210+ROcp13_210*qd[11];
OMcp13_311 = OMcp13_310+ROcp13_310*qd[11];
ORcp13_111 = OMcp13_210*RLcp13_311-OMcp13_310*RLcp13_211;
ORcp13_211 = -(OMcp13_110*RLcp13_311-OMcp13_310*RLcp13_111);
ORcp13_311 = OMcp13_110*RLcp13_211-OMcp13_210*RLcp13_111;
OPcp13_111 = OPcp13_110+ROcp13_110*qdd[11]+qd[11]*(OMcp13_210*ROcp13_310-OMcp13_310*ROcp13_210);
OPcp13_211 = OPcp13_210+ROcp13_210*qdd[11]-qd[11]*(OMcp13_110*ROcp13_310-OMcp13_310*ROcp13_110);
OPcp13_311 = OPcp13_310+ROcp13_310*qdd[11]+qd[11]*(OMcp13_110*ROcp13_210-OMcp13_210*ROcp13_110);
RLcp13_143 = ROcp13_110*s->dpt[1][15]+ROcp13_411*s->dpt[2][15]+ROcp13_711*s->dpt[3][15];
RLcp13_243 = ROcp13_210*s->dpt[1][15]+ROcp13_511*s->dpt[2][15]+ROcp13_811*s->dpt[3][15];
RLcp13_343 = ROcp13_310*s->dpt[1][15]+ROcp13_611*s->dpt[2][15]+ROcp13_911*s->dpt[3][15];
POcp13_143 = RLcp13_110+RLcp13_111+RLcp13_143+RLcp13_17+RLcp13_18+RLcp13_19+q[1];
POcp13_243 = RLcp13_210+RLcp13_211+RLcp13_243+RLcp13_27+RLcp13_28+RLcp13_29+q[2];
POcp13_343 = RLcp13_310+RLcp13_311+RLcp13_343+RLcp13_37+RLcp13_38+RLcp13_39+q[3];
JTcp13_243_4 = -(RLcp13_310+RLcp13_311+RLcp13_343+RLcp13_37+RLcp13_38+RLcp13_39);
JTcp13_343_4 = RLcp13_210+RLcp13_211+RLcp13_243+RLcp13_27+RLcp13_28+RLcp13_29;
JTcp13_143_5 = C4*(RLcp13_310+RLcp13_311+RLcp13_343+RLcp13_37+RLcp13_38+RLcp13_39)-S4*(RLcp13_210+RLcp13_29)-S4*(
 RLcp13_211+RLcp13_243)-S4*(RLcp13_27+RLcp13_28);
JTcp13_243_5 = S4*(RLcp13_110+RLcp13_111+RLcp13_143+RLcp13_17+RLcp13_18+RLcp13_19);
JTcp13_343_5 = -C4*(RLcp13_110+RLcp13_111+RLcp13_143+RLcp13_17+RLcp13_18+RLcp13_19);
JTcp13_143_6 = ROcp13_85*(RLcp13_310+RLcp13_311+RLcp13_343+RLcp13_37+RLcp13_38+RLcp13_39)-ROcp13_95*(RLcp13_210+
 RLcp13_29)-ROcp13_95*(RLcp13_211+RLcp13_243)-ROcp13_95*(RLcp13_27+RLcp13_28);
JTcp13_243_6 = RLcp13_143*ROcp13_95-RLcp13_311*S5-RLcp13_343*S5+ROcp13_95*(RLcp13_110+RLcp13_111+RLcp13_17+RLcp13_18+
 RLcp13_19)-S5*(RLcp13_310+RLcp13_39)-S5*(RLcp13_37+RLcp13_38);
JTcp13_343_6 = RLcp13_211*S5-ROcp13_85*(RLcp13_110+RLcp13_111+RLcp13_17+RLcp13_18+RLcp13_19)+S5*(RLcp13_210+RLcp13_29)
 +S5*(RLcp13_27+RLcp13_28)-RLcp13_143*ROcp13_85+RLcp13_243*S5;
JTcp13_143_7 = ROcp13_56*(RLcp13_310+RLcp13_311+RLcp13_38+RLcp13_39)-ROcp13_66*(RLcp13_210+RLcp13_211)-ROcp13_66*(
 RLcp13_28+RLcp13_29)-RLcp13_243*ROcp13_66+RLcp13_343*ROcp13_56;
JTcp13_243_7 = RLcp13_143*ROcp13_66-RLcp13_343*ROcp13_46-ROcp13_46*(RLcp13_310+RLcp13_311+RLcp13_38+RLcp13_39)+
 ROcp13_66*(RLcp13_110+RLcp13_111)+ROcp13_66*(RLcp13_18+RLcp13_19);
JTcp13_343_7 = ROcp13_46*(RLcp13_210+RLcp13_211+RLcp13_28+RLcp13_29)-ROcp13_56*(RLcp13_110+RLcp13_111)-ROcp13_56*(
 RLcp13_18+RLcp13_19)-RLcp13_143*ROcp13_56+RLcp13_243*ROcp13_46;
JTcp13_143_8 = ROcp13_27*(RLcp13_310+RLcp13_311+RLcp13_343+RLcp13_39)-ROcp13_37*(RLcp13_210+RLcp13_29)-ROcp13_37*(
 RLcp13_211+RLcp13_243);
JTcp13_243_8 = -(ROcp13_17*(RLcp13_310+RLcp13_311+RLcp13_343+RLcp13_39)-ROcp13_37*(RLcp13_110+RLcp13_19)-ROcp13_37*(
 RLcp13_111+RLcp13_143));
JTcp13_343_8 = ROcp13_17*(RLcp13_210+RLcp13_211+RLcp13_243+RLcp13_29)-ROcp13_27*(RLcp13_110+RLcp13_19)-ROcp13_27*(
 RLcp13_111+RLcp13_143);
JTcp13_143_9 = ROcp13_88*(RLcp13_310+RLcp13_311)-ROcp13_98*(RLcp13_210+RLcp13_211)-RLcp13_243*ROcp13_98+RLcp13_343*
 ROcp13_88;
JTcp13_243_9 = RLcp13_143*ROcp13_98-RLcp13_343*ROcp13_78-ROcp13_78*(RLcp13_310+RLcp13_311)+ROcp13_98*(RLcp13_110+
 RLcp13_111);
JTcp13_343_9 = ROcp13_78*(RLcp13_210+RLcp13_211)-ROcp13_88*(RLcp13_110+RLcp13_111)-RLcp13_143*ROcp13_88+RLcp13_243*
 ROcp13_78;
JTcp13_143_10 = ROcp13_59*(RLcp13_311+RLcp13_343)-ROcp13_69*(RLcp13_211+RLcp13_243);
JTcp13_243_10 = -(ROcp13_49*(RLcp13_311+RLcp13_343)-ROcp13_69*(RLcp13_111+RLcp13_143));
JTcp13_343_10 = ROcp13_49*(RLcp13_211+RLcp13_243)-ROcp13_59*(RLcp13_111+RLcp13_143);
JTcp13_143_11 = -(RLcp13_243*ROcp13_310-RLcp13_343*ROcp13_210);
JTcp13_243_11 = RLcp13_143*ROcp13_310-RLcp13_343*ROcp13_110;
JTcp13_343_11 = -(RLcp13_143*ROcp13_210-RLcp13_243*ROcp13_110);
ORcp13_143 = OMcp13_211*RLcp13_343-OMcp13_311*RLcp13_243;
ORcp13_243 = -(OMcp13_111*RLcp13_343-OMcp13_311*RLcp13_143);
ORcp13_343 = OMcp13_111*RLcp13_243-OMcp13_211*RLcp13_143;
VIcp13_143 = ORcp13_110+ORcp13_111+ORcp13_143+ORcp13_17+ORcp13_18+ORcp13_19+qd[1];
VIcp13_243 = ORcp13_210+ORcp13_211+ORcp13_243+ORcp13_27+ORcp13_28+ORcp13_29+qd[2];
VIcp13_343 = ORcp13_310+ORcp13_311+ORcp13_343+ORcp13_37+ORcp13_38+ORcp13_39+qd[3];
ACcp13_143 = qdd[1]+OMcp13_210*ORcp13_311+OMcp13_211*ORcp13_343+OMcp13_26*ORcp13_37+OMcp13_27*ORcp13_38+OMcp13_28*
 ORcp13_39+OMcp13_29*ORcp13_310-OMcp13_310*ORcp13_211-OMcp13_311*ORcp13_243-OMcp13_36*ORcp13_27-OMcp13_37*ORcp13_28-OMcp13_38
 *ORcp13_29-OMcp13_39*ORcp13_210+OPcp13_210*RLcp13_311+OPcp13_211*RLcp13_343+OPcp13_26*RLcp13_37+OPcp13_27*RLcp13_38+
 OPcp13_28*RLcp13_39+OPcp13_29*RLcp13_310-OPcp13_310*RLcp13_211-OPcp13_311*RLcp13_243-OPcp13_36*RLcp13_27-OPcp13_37*RLcp13_28
 -OPcp13_38*RLcp13_29-OPcp13_39*RLcp13_210;
ACcp13_243 = qdd[2]-OMcp13_110*ORcp13_311-OMcp13_111*ORcp13_343-OMcp13_16*ORcp13_37-OMcp13_17*ORcp13_38-OMcp13_18*
 ORcp13_39-OMcp13_19*ORcp13_310+OMcp13_310*ORcp13_111+OMcp13_311*ORcp13_143+OMcp13_36*ORcp13_17+OMcp13_37*ORcp13_18+OMcp13_38
 *ORcp13_19+OMcp13_39*ORcp13_110-OPcp13_110*RLcp13_311-OPcp13_111*RLcp13_343-OPcp13_16*RLcp13_37-OPcp13_17*RLcp13_38-
 OPcp13_18*RLcp13_39-OPcp13_19*RLcp13_310+OPcp13_310*RLcp13_111+OPcp13_311*RLcp13_143+OPcp13_36*RLcp13_17+OPcp13_37*RLcp13_18
 +OPcp13_38*RLcp13_19+OPcp13_39*RLcp13_110;
ACcp13_343 = qdd[3]+OMcp13_110*ORcp13_211+OMcp13_111*ORcp13_243+OMcp13_16*ORcp13_27+OMcp13_17*ORcp13_28+OMcp13_18*
 ORcp13_29+OMcp13_19*ORcp13_210-OMcp13_210*ORcp13_111-OMcp13_211*ORcp13_143-OMcp13_26*ORcp13_17-OMcp13_27*ORcp13_18-OMcp13_28
 *ORcp13_19-OMcp13_29*ORcp13_110+OPcp13_110*RLcp13_211+OPcp13_111*RLcp13_243+OPcp13_16*RLcp13_27+OPcp13_17*RLcp13_28+
 OPcp13_18*RLcp13_29+OPcp13_19*RLcp13_210-OPcp13_210*RLcp13_111-OPcp13_211*RLcp13_143-OPcp13_26*RLcp13_17-OPcp13_27*RLcp13_18
 -OPcp13_28*RLcp13_19-OPcp13_29*RLcp13_110;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp13_143;
sens->P[2] = POcp13_243;
sens->P[3] = POcp13_343;
sens->R[1][1] = ROcp13_110;
sens->R[1][2] = ROcp13_210;
sens->R[1][3] = ROcp13_310;
sens->R[2][1] = ROcp13_411;
sens->R[2][2] = ROcp13_511;
sens->R[2][3] = ROcp13_611;
sens->R[3][1] = ROcp13_711;
sens->R[3][2] = ROcp13_811;
sens->R[3][3] = ROcp13_911;
sens->V[1] = VIcp13_143;
sens->V[2] = VIcp13_243;
sens->V[3] = VIcp13_343;
sens->OM[1] = OMcp13_111;
sens->OM[2] = OMcp13_211;
sens->OM[3] = OMcp13_311;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp13_143_5;
sens->J[1][6] = JTcp13_143_6;
sens->J[1][7] = JTcp13_143_7;
sens->J[1][8] = JTcp13_143_8;
sens->J[1][9] = JTcp13_143_9;
sens->J[1][10] = JTcp13_143_10;
sens->J[1][11] = JTcp13_143_11;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp13_243_4;
sens->J[2][5] = JTcp13_243_5;
sens->J[2][6] = JTcp13_243_6;
sens->J[2][7] = JTcp13_243_7;
sens->J[2][8] = JTcp13_243_8;
sens->J[2][9] = JTcp13_243_9;
sens->J[2][10] = JTcp13_243_10;
sens->J[2][11] = JTcp13_243_11;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp13_343_4;
sens->J[3][5] = JTcp13_343_5;
sens->J[3][6] = JTcp13_343_6;
sens->J[3][7] = JTcp13_343_7;
sens->J[3][8] = JTcp13_343_8;
sens->J[3][9] = JTcp13_343_9;
sens->J[3][10] = JTcp13_343_10;
sens->J[3][11] = JTcp13_343_11;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp13_46;
sens->J[4][8] = ROcp13_17;
sens->J[4][9] = ROcp13_78;
sens->J[4][10] = ROcp13_49;
sens->J[4][11] = ROcp13_110;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp13_85;
sens->J[5][7] = ROcp13_56;
sens->J[5][8] = ROcp13_27;
sens->J[5][9] = ROcp13_88;
sens->J[5][10] = ROcp13_59;
sens->J[5][11] = ROcp13_210;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp13_95;
sens->J[6][7] = ROcp13_66;
sens->J[6][8] = ROcp13_37;
sens->J[6][9] = ROcp13_98;
sens->J[6][10] = ROcp13_69;
sens->J[6][11] = ROcp13_310;
sens->A[1] = ACcp13_143;
sens->A[2] = ACcp13_243;
sens->A[3] = ACcp13_343;
sens->OMP[1] = OPcp13_111;
sens->OMP[2] = OPcp13_211;
sens->OMP[3] = OPcp13_311;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
// Sensor Kinematics 


ROcp14_25 = S4*S5;
ROcp14_35 = -C4*S5;
ROcp14_85 = -S4*C5;
ROcp14_95 = C4*C5;
ROcp14_16 = C5*C6;
ROcp14_26 = ROcp14_25*C6+C4*S6;
ROcp14_36 = ROcp14_35*C6+S4*S6;
ROcp14_46 = -C5*S6;
ROcp14_56 = -(ROcp14_25*S6-C4*C6);
ROcp14_66 = -(ROcp14_35*S6-S4*C6);
OMcp14_25 = qd[5]*C4;
OMcp14_35 = qd[5]*S4;
OMcp14_16 = qd[4]+qd[6]*S5;
OMcp14_26 = OMcp14_25+ROcp14_85*qd[6];
OMcp14_36 = OMcp14_35+ROcp14_95*qd[6];
OPcp14_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp14_26 = ROcp14_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp14_35*S5-ROcp14_95*qd[4]);
OPcp14_36 = ROcp14_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp14_25*S5-ROcp14_85*qd[4]);

// = = Block_1_0_0_15_0_2 = = 
 
// Sensor Kinematics 


ROcp14_17 = ROcp14_16*C7-S5*S7;
ROcp14_27 = ROcp14_26*C7-ROcp14_85*S7;
ROcp14_37 = ROcp14_36*C7-ROcp14_95*S7;
ROcp14_77 = ROcp14_16*S7+S5*C7;
ROcp14_87 = ROcp14_26*S7+ROcp14_85*C7;
ROcp14_97 = ROcp14_36*S7+ROcp14_95*C7;
ROcp14_48 = ROcp14_46*C8+ROcp14_77*S8;
ROcp14_58 = ROcp14_56*C8+ROcp14_87*S8;
ROcp14_68 = ROcp14_66*C8+ROcp14_97*S8;
ROcp14_78 = -(ROcp14_46*S8-ROcp14_77*C8);
ROcp14_88 = -(ROcp14_56*S8-ROcp14_87*C8);
ROcp14_98 = -(ROcp14_66*S8-ROcp14_97*C8);
ROcp14_19 = ROcp14_17*C9+ROcp14_48*S9;
ROcp14_29 = ROcp14_27*C9+ROcp14_58*S9;
ROcp14_39 = ROcp14_37*C9+ROcp14_68*S9;
ROcp14_49 = -(ROcp14_17*S9-ROcp14_48*C9);
ROcp14_59 = -(ROcp14_27*S9-ROcp14_58*C9);
ROcp14_69 = -(ROcp14_37*S9-ROcp14_68*C9);
ROcp14_110 = ROcp14_19*C10-ROcp14_78*S10;
ROcp14_210 = ROcp14_29*C10-ROcp14_88*S10;
ROcp14_310 = ROcp14_39*C10-ROcp14_98*S10;
ROcp14_710 = ROcp14_19*S10+ROcp14_78*C10;
ROcp14_810 = ROcp14_29*S10+ROcp14_88*C10;
ROcp14_910 = ROcp14_39*S10+ROcp14_98*C10;
ROcp14_411 = ROcp14_49*C11+ROcp14_710*S11;
ROcp14_511 = ROcp14_59*C11+ROcp14_810*S11;
ROcp14_611 = ROcp14_69*C11+ROcp14_910*S11;
ROcp14_711 = -(ROcp14_49*S11-ROcp14_710*C11);
ROcp14_811 = -(ROcp14_59*S11-ROcp14_810*C11);
ROcp14_911 = -(ROcp14_69*S11-ROcp14_910*C11);
ROcp14_112 = ROcp14_110*C12-ROcp14_711*S12;
ROcp14_212 = ROcp14_210*C12-ROcp14_811*S12;
ROcp14_312 = ROcp14_310*C12-ROcp14_911*S12;
ROcp14_712 = ROcp14_110*S12+ROcp14_711*C12;
ROcp14_812 = ROcp14_210*S12+ROcp14_811*C12;
ROcp14_912 = ROcp14_310*S12+ROcp14_911*C12;
RLcp14_17 = s->dpt[1][1]*ROcp14_16+s->dpt[3][1]*S5+ROcp14_46*s->dpt[2][1];
RLcp14_27 = s->dpt[1][1]*ROcp14_26+s->dpt[3][1]*ROcp14_85+ROcp14_56*s->dpt[2][1];
RLcp14_37 = s->dpt[1][1]*ROcp14_36+s->dpt[3][1]*ROcp14_95+ROcp14_66*s->dpt[2][1];
OMcp14_17 = OMcp14_16+ROcp14_46*qd[7];
OMcp14_27 = OMcp14_26+ROcp14_56*qd[7];
OMcp14_37 = OMcp14_36+ROcp14_66*qd[7];
ORcp14_17 = OMcp14_26*RLcp14_37-OMcp14_36*RLcp14_27;
ORcp14_27 = -(OMcp14_16*RLcp14_37-OMcp14_36*RLcp14_17);
ORcp14_37 = OMcp14_16*RLcp14_27-OMcp14_26*RLcp14_17;
OPcp14_17 = OPcp14_16+ROcp14_46*qdd[7]+qd[7]*(OMcp14_26*ROcp14_66-OMcp14_36*ROcp14_56);
OPcp14_27 = OPcp14_26+ROcp14_56*qdd[7]-qd[7]*(OMcp14_16*ROcp14_66-OMcp14_36*ROcp14_46);
OPcp14_37 = OPcp14_36+ROcp14_66*qdd[7]+qd[7]*(OMcp14_16*ROcp14_56-OMcp14_26*ROcp14_46);
RLcp14_18 = s->dpt[1][6]*ROcp14_17+s->dpt[3][6]*ROcp14_77+ROcp14_46*s->dpt[2][6];
RLcp14_28 = s->dpt[1][6]*ROcp14_27+s->dpt[3][6]*ROcp14_87+ROcp14_56*s->dpt[2][6];
RLcp14_38 = s->dpt[1][6]*ROcp14_37+s->dpt[3][6]*ROcp14_97+ROcp14_66*s->dpt[2][6];
OMcp14_18 = OMcp14_17+ROcp14_17*qd[8];
OMcp14_28 = OMcp14_27+ROcp14_27*qd[8];
OMcp14_38 = OMcp14_37+ROcp14_37*qd[8];
ORcp14_18 = OMcp14_27*RLcp14_38-OMcp14_37*RLcp14_28;
ORcp14_28 = -(OMcp14_17*RLcp14_38-OMcp14_37*RLcp14_18);
ORcp14_38 = OMcp14_17*RLcp14_28-OMcp14_27*RLcp14_18;
OPcp14_18 = OPcp14_17+ROcp14_17*qdd[8]+qd[8]*(OMcp14_27*ROcp14_37-OMcp14_37*ROcp14_27);
OPcp14_28 = OPcp14_27+ROcp14_27*qdd[8]-qd[8]*(OMcp14_17*ROcp14_37-OMcp14_37*ROcp14_17);
OPcp14_38 = OPcp14_37+ROcp14_37*qdd[8]+qd[8]*(OMcp14_17*ROcp14_27-OMcp14_27*ROcp14_17);
RLcp14_19 = s->dpt[1][8]*ROcp14_17+s->dpt[2][8]*ROcp14_48+ROcp14_78*s->dpt[3][8];
RLcp14_29 = s->dpt[1][8]*ROcp14_27+s->dpt[2][8]*ROcp14_58+ROcp14_88*s->dpt[3][8];
RLcp14_39 = s->dpt[1][8]*ROcp14_37+s->dpt[2][8]*ROcp14_68+ROcp14_98*s->dpt[3][8];
OMcp14_19 = OMcp14_18+ROcp14_78*qd[9];
OMcp14_29 = OMcp14_28+ROcp14_88*qd[9];
OMcp14_39 = OMcp14_38+ROcp14_98*qd[9];
ORcp14_19 = OMcp14_28*RLcp14_39-OMcp14_38*RLcp14_29;
ORcp14_29 = -(OMcp14_18*RLcp14_39-OMcp14_38*RLcp14_19);
ORcp14_39 = OMcp14_18*RLcp14_29-OMcp14_28*RLcp14_19;
OPcp14_19 = OPcp14_18+ROcp14_78*qdd[9]+qd[9]*(OMcp14_28*ROcp14_98-OMcp14_38*ROcp14_88);
OPcp14_29 = OPcp14_28+ROcp14_88*qdd[9]-qd[9]*(OMcp14_18*ROcp14_98-OMcp14_38*ROcp14_78);
OPcp14_39 = OPcp14_38+ROcp14_98*qdd[9]+qd[9]*(OMcp14_18*ROcp14_88-OMcp14_28*ROcp14_78);
RLcp14_110 = s->dpt[1][10]*ROcp14_19+s->dpt[2][10]*ROcp14_49+ROcp14_78*s->dpt[3][10];
RLcp14_210 = s->dpt[1][10]*ROcp14_29+s->dpt[2][10]*ROcp14_59+ROcp14_88*s->dpt[3][10];
RLcp14_310 = s->dpt[1][10]*ROcp14_39+s->dpt[2][10]*ROcp14_69+ROcp14_98*s->dpt[3][10];
OMcp14_110 = OMcp14_19+ROcp14_49*qd[10];
OMcp14_210 = OMcp14_29+ROcp14_59*qd[10];
OMcp14_310 = OMcp14_39+ROcp14_69*qd[10];
ORcp14_110 = OMcp14_29*RLcp14_310-OMcp14_39*RLcp14_210;
ORcp14_210 = -(OMcp14_19*RLcp14_310-OMcp14_39*RLcp14_110);
ORcp14_310 = OMcp14_19*RLcp14_210-OMcp14_29*RLcp14_110;
OPcp14_110 = OPcp14_19+ROcp14_49*qdd[10]+qd[10]*(OMcp14_29*ROcp14_69-OMcp14_39*ROcp14_59);
OPcp14_210 = OPcp14_29+ROcp14_59*qdd[10]-qd[10]*(OMcp14_19*ROcp14_69-OMcp14_39*ROcp14_49);
OPcp14_310 = OPcp14_39+ROcp14_69*qdd[10]+qd[10]*(OMcp14_19*ROcp14_59-OMcp14_29*ROcp14_49);
RLcp14_111 = s->dpt[1][12]*ROcp14_110+s->dpt[2][12]*ROcp14_49+ROcp14_710*s->dpt[3][12];
RLcp14_211 = s->dpt[1][12]*ROcp14_210+s->dpt[2][12]*ROcp14_59+ROcp14_810*s->dpt[3][12];
RLcp14_311 = s->dpt[1][12]*ROcp14_310+s->dpt[2][12]*ROcp14_69+ROcp14_910*s->dpt[3][12];
OMcp14_111 = OMcp14_110+ROcp14_110*qd[11];
OMcp14_211 = OMcp14_210+ROcp14_210*qd[11];
OMcp14_311 = OMcp14_310+ROcp14_310*qd[11];
ORcp14_111 = OMcp14_210*RLcp14_311-OMcp14_310*RLcp14_211;
ORcp14_211 = -(OMcp14_110*RLcp14_311-OMcp14_310*RLcp14_111);
ORcp14_311 = OMcp14_110*RLcp14_211-OMcp14_210*RLcp14_111;
OPcp14_111 = OPcp14_110+ROcp14_110*qdd[11]+qd[11]*(OMcp14_210*ROcp14_310-OMcp14_310*ROcp14_210);
OPcp14_211 = OPcp14_210+ROcp14_210*qdd[11]-qd[11]*(OMcp14_110*ROcp14_310-OMcp14_310*ROcp14_110);
OPcp14_311 = OPcp14_310+ROcp14_310*qdd[11]+qd[11]*(OMcp14_110*ROcp14_210-OMcp14_210*ROcp14_110);
RLcp14_112 = s->dpt[1][14]*ROcp14_110+s->dpt[2][14]*ROcp14_411+s->dpt[3][14]*ROcp14_711;
RLcp14_212 = s->dpt[1][14]*ROcp14_210+s->dpt[2][14]*ROcp14_511+s->dpt[3][14]*ROcp14_811;
RLcp14_312 = s->dpt[1][14]*ROcp14_310+s->dpt[2][14]*ROcp14_611+s->dpt[3][14]*ROcp14_911;
OMcp14_112 = OMcp14_111+ROcp14_411*qd[12];
OMcp14_212 = OMcp14_211+ROcp14_511*qd[12];
OMcp14_312 = OMcp14_311+ROcp14_611*qd[12];
ORcp14_112 = OMcp14_211*RLcp14_312-OMcp14_311*RLcp14_212;
ORcp14_212 = -(OMcp14_111*RLcp14_312-OMcp14_311*RLcp14_112);
ORcp14_312 = OMcp14_111*RLcp14_212-OMcp14_211*RLcp14_112;
OPcp14_112 = OPcp14_111+ROcp14_411*qdd[12]+qd[12]*(OMcp14_211*ROcp14_611-OMcp14_311*ROcp14_511);
OPcp14_212 = OPcp14_211+ROcp14_511*qdd[12]-qd[12]*(OMcp14_111*ROcp14_611-OMcp14_311*ROcp14_411);
OPcp14_312 = OPcp14_311+ROcp14_611*qdd[12]+qd[12]*(OMcp14_111*ROcp14_511-OMcp14_211*ROcp14_411);
RLcp14_144 = s->dpt[1][16]*ROcp14_112+s->dpt[2][16]*ROcp14_411+ROcp14_712*s->dpt[3][16];
RLcp14_244 = s->dpt[1][16]*ROcp14_212+s->dpt[2][16]*ROcp14_511+ROcp14_812*s->dpt[3][16];
RLcp14_344 = s->dpt[1][16]*ROcp14_312+s->dpt[2][16]*ROcp14_611+ROcp14_912*s->dpt[3][16];
POcp14_144 = RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_144+RLcp14_17+RLcp14_18+RLcp14_19+q[1];
POcp14_244 = RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_244+RLcp14_27+RLcp14_28+RLcp14_29+q[2];
POcp14_344 = RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_344+RLcp14_37+RLcp14_38+RLcp14_39+q[3];
JTcp14_244_4 = -(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_344+RLcp14_37+RLcp14_38+RLcp14_39);
JTcp14_344_4 = RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_244+RLcp14_27+RLcp14_28+RLcp14_29;
JTcp14_144_5 = C4*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_37+RLcp14_38+RLcp14_39)-S4*(RLcp14_210+RLcp14_29)-S4*(
 RLcp14_211+RLcp14_212)-S4*(RLcp14_27+RLcp14_28)-RLcp14_244*S4+RLcp14_344*C4;
JTcp14_244_5 = S4*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_144+RLcp14_17+RLcp14_18+RLcp14_19);
JTcp14_344_5 = -C4*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_144+RLcp14_17+RLcp14_18+RLcp14_19);
JTcp14_144_6 = ROcp14_85*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_37+RLcp14_38+RLcp14_39)-ROcp14_95*(RLcp14_210+
 RLcp14_29)-ROcp14_95*(RLcp14_211+RLcp14_212)-ROcp14_95*(RLcp14_27+RLcp14_28)-RLcp14_244*ROcp14_95+RLcp14_344*ROcp14_85;
JTcp14_244_6 = -(RLcp14_344*S5-ROcp14_95*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_144+RLcp14_17+RLcp14_18+RLcp14_19)+
 S5*(RLcp14_310+RLcp14_39)+S5*(RLcp14_311+RLcp14_312)+S5*(RLcp14_37+RLcp14_38));
JTcp14_344_6 = RLcp14_244*S5-ROcp14_85*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_144+RLcp14_17+RLcp14_18+RLcp14_19)+S5*
 (RLcp14_210+RLcp14_29)+S5*(RLcp14_211+RLcp14_212)+S5*(RLcp14_27+RLcp14_28);
JTcp14_144_7 = ROcp14_56*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_344+RLcp14_38+RLcp14_39)-ROcp14_66*(RLcp14_210+
 RLcp14_211)-ROcp14_66*(RLcp14_212+RLcp14_244)-ROcp14_66*(RLcp14_28+RLcp14_29);
JTcp14_244_7 = -(ROcp14_46*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_344+RLcp14_38+RLcp14_39)-ROcp14_66*(RLcp14_110+
 RLcp14_111)-ROcp14_66*(RLcp14_112+RLcp14_144)-ROcp14_66*(RLcp14_18+RLcp14_19));
JTcp14_344_7 = ROcp14_46*(RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_244+RLcp14_28+RLcp14_29)-ROcp14_56*(RLcp14_110+
 RLcp14_111)-ROcp14_56*(RLcp14_112+RLcp14_144)-ROcp14_56*(RLcp14_18+RLcp14_19);
JTcp14_144_8 = ROcp14_27*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_39)-ROcp14_37*(RLcp14_210+RLcp14_29)-ROcp14_37*(
 RLcp14_211+RLcp14_212)-RLcp14_244*ROcp14_37+RLcp14_344*ROcp14_27;
JTcp14_244_8 = RLcp14_144*ROcp14_37-RLcp14_344*ROcp14_17-ROcp14_17*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_39)+
 ROcp14_37*(RLcp14_110+RLcp14_19)+ROcp14_37*(RLcp14_111+RLcp14_112);
JTcp14_344_8 = ROcp14_17*(RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_29)-ROcp14_27*(RLcp14_110+RLcp14_19)-ROcp14_27*(
 RLcp14_111+RLcp14_112)-RLcp14_144*ROcp14_27+RLcp14_244*ROcp14_17;
JTcp14_144_9 = ROcp14_88*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_344)-ROcp14_98*(RLcp14_210+RLcp14_211)-ROcp14_98*(
 RLcp14_212+RLcp14_244);
JTcp14_244_9 = -(ROcp14_78*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_344)-ROcp14_98*(RLcp14_110+RLcp14_111)-ROcp14_98*(
 RLcp14_112+RLcp14_144));
JTcp14_344_9 = ROcp14_78*(RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_244)-ROcp14_88*(RLcp14_110+RLcp14_111)-ROcp14_88*(
 RLcp14_112+RLcp14_144);
JTcp14_144_10 = ROcp14_59*(RLcp14_311+RLcp14_312)-ROcp14_69*(RLcp14_211+RLcp14_212)-RLcp14_244*ROcp14_69+RLcp14_344*
 ROcp14_59;
JTcp14_244_10 = RLcp14_144*ROcp14_69-RLcp14_344*ROcp14_49-ROcp14_49*(RLcp14_311+RLcp14_312)+ROcp14_69*(RLcp14_111+
 RLcp14_112);
JTcp14_344_10 = ROcp14_49*(RLcp14_211+RLcp14_212)-ROcp14_59*(RLcp14_111+RLcp14_112)-RLcp14_144*ROcp14_59+RLcp14_244*
 ROcp14_49;
JTcp14_144_11 = ROcp14_210*(RLcp14_312+RLcp14_344)-ROcp14_310*(RLcp14_212+RLcp14_244);
JTcp14_244_11 = -(ROcp14_110*(RLcp14_312+RLcp14_344)-ROcp14_310*(RLcp14_112+RLcp14_144));
JTcp14_344_11 = ROcp14_110*(RLcp14_212+RLcp14_244)-ROcp14_210*(RLcp14_112+RLcp14_144);
JTcp14_144_12 = -(RLcp14_244*ROcp14_611-RLcp14_344*ROcp14_511);
JTcp14_244_12 = RLcp14_144*ROcp14_611-RLcp14_344*ROcp14_411;
JTcp14_344_12 = -(RLcp14_144*ROcp14_511-RLcp14_244*ROcp14_411);
ORcp14_144 = OMcp14_212*RLcp14_344-OMcp14_312*RLcp14_244;
ORcp14_244 = -(OMcp14_112*RLcp14_344-OMcp14_312*RLcp14_144);
ORcp14_344 = OMcp14_112*RLcp14_244-OMcp14_212*RLcp14_144;
VIcp14_144 = ORcp14_110+ORcp14_111+ORcp14_112+ORcp14_144+ORcp14_17+ORcp14_18+ORcp14_19+qd[1];
VIcp14_244 = ORcp14_210+ORcp14_211+ORcp14_212+ORcp14_244+ORcp14_27+ORcp14_28+ORcp14_29+qd[2];
VIcp14_344 = ORcp14_310+ORcp14_311+ORcp14_312+ORcp14_344+ORcp14_37+ORcp14_38+ORcp14_39+qd[3];
ACcp14_144 = qdd[1]+OMcp14_210*ORcp14_311+OMcp14_211*ORcp14_312+OMcp14_212*ORcp14_344+OMcp14_26*ORcp14_37+OMcp14_27*
 ORcp14_38+OMcp14_28*ORcp14_39+OMcp14_29*ORcp14_310-OMcp14_310*ORcp14_211-OMcp14_311*ORcp14_212-OMcp14_312*ORcp14_244-
 OMcp14_36*ORcp14_27-OMcp14_37*ORcp14_28-OMcp14_38*ORcp14_29-OMcp14_39*ORcp14_210+OPcp14_210*RLcp14_311+OPcp14_211*RLcp14_312
 +OPcp14_212*RLcp14_344+OPcp14_26*RLcp14_37+OPcp14_27*RLcp14_38+OPcp14_28*RLcp14_39+OPcp14_29*RLcp14_310-OPcp14_310*
 RLcp14_211-OPcp14_311*RLcp14_212-OPcp14_312*RLcp14_244-OPcp14_36*RLcp14_27-OPcp14_37*RLcp14_28-OPcp14_38*RLcp14_29-OPcp14_39
 *RLcp14_210;
ACcp14_244 = qdd[2]-OMcp14_110*ORcp14_311-OMcp14_111*ORcp14_312-OMcp14_112*ORcp14_344-OMcp14_16*ORcp14_37-OMcp14_17*
 ORcp14_38-OMcp14_18*ORcp14_39-OMcp14_19*ORcp14_310+OMcp14_310*ORcp14_111+OMcp14_311*ORcp14_112+OMcp14_312*ORcp14_144+
 OMcp14_36*ORcp14_17+OMcp14_37*ORcp14_18+OMcp14_38*ORcp14_19+OMcp14_39*ORcp14_110-OPcp14_110*RLcp14_311-OPcp14_111*RLcp14_312
 -OPcp14_112*RLcp14_344-OPcp14_16*RLcp14_37-OPcp14_17*RLcp14_38-OPcp14_18*RLcp14_39-OPcp14_19*RLcp14_310+OPcp14_310*
 RLcp14_111+OPcp14_311*RLcp14_112+OPcp14_312*RLcp14_144+OPcp14_36*RLcp14_17+OPcp14_37*RLcp14_18+OPcp14_38*RLcp14_19+OPcp14_39
 *RLcp14_110;
ACcp14_344 = qdd[3]+OMcp14_110*ORcp14_211+OMcp14_111*ORcp14_212+OMcp14_112*ORcp14_244+OMcp14_16*ORcp14_27+OMcp14_17*
 ORcp14_28+OMcp14_18*ORcp14_29+OMcp14_19*ORcp14_210-OMcp14_210*ORcp14_111-OMcp14_211*ORcp14_112-OMcp14_212*ORcp14_144-
 OMcp14_26*ORcp14_17-OMcp14_27*ORcp14_18-OMcp14_28*ORcp14_19-OMcp14_29*ORcp14_110+OPcp14_110*RLcp14_211+OPcp14_111*RLcp14_212
 +OPcp14_112*RLcp14_244+OPcp14_16*RLcp14_27+OPcp14_17*RLcp14_28+OPcp14_18*RLcp14_29+OPcp14_19*RLcp14_210-OPcp14_210*
 RLcp14_111-OPcp14_211*RLcp14_112-OPcp14_212*RLcp14_144-OPcp14_26*RLcp14_17-OPcp14_27*RLcp14_18-OPcp14_28*RLcp14_19-OPcp14_29
 *RLcp14_110;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp14_144;
sens->P[2] = POcp14_244;
sens->P[3] = POcp14_344;
sens->R[1][1] = ROcp14_112;
sens->R[1][2] = ROcp14_212;
sens->R[1][3] = ROcp14_312;
sens->R[2][1] = ROcp14_411;
sens->R[2][2] = ROcp14_511;
sens->R[2][3] = ROcp14_611;
sens->R[3][1] = ROcp14_712;
sens->R[3][2] = ROcp14_812;
sens->R[3][3] = ROcp14_912;
sens->V[1] = VIcp14_144;
sens->V[2] = VIcp14_244;
sens->V[3] = VIcp14_344;
sens->OM[1] = OMcp14_112;
sens->OM[2] = OMcp14_212;
sens->OM[3] = OMcp14_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp14_144_5;
sens->J[1][6] = JTcp14_144_6;
sens->J[1][7] = JTcp14_144_7;
sens->J[1][8] = JTcp14_144_8;
sens->J[1][9] = JTcp14_144_9;
sens->J[1][10] = JTcp14_144_10;
sens->J[1][11] = JTcp14_144_11;
sens->J[1][12] = JTcp14_144_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp14_244_4;
sens->J[2][5] = JTcp14_244_5;
sens->J[2][6] = JTcp14_244_6;
sens->J[2][7] = JTcp14_244_7;
sens->J[2][8] = JTcp14_244_8;
sens->J[2][9] = JTcp14_244_9;
sens->J[2][10] = JTcp14_244_10;
sens->J[2][11] = JTcp14_244_11;
sens->J[2][12] = JTcp14_244_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp14_344_4;
sens->J[3][5] = JTcp14_344_5;
sens->J[3][6] = JTcp14_344_6;
sens->J[3][7] = JTcp14_344_7;
sens->J[3][8] = JTcp14_344_8;
sens->J[3][9] = JTcp14_344_9;
sens->J[3][10] = JTcp14_344_10;
sens->J[3][11] = JTcp14_344_11;
sens->J[3][12] = JTcp14_344_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp14_46;
sens->J[4][8] = ROcp14_17;
sens->J[4][9] = ROcp14_78;
sens->J[4][10] = ROcp14_49;
sens->J[4][11] = ROcp14_110;
sens->J[4][12] = ROcp14_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp14_85;
sens->J[5][7] = ROcp14_56;
sens->J[5][8] = ROcp14_27;
sens->J[5][9] = ROcp14_88;
sens->J[5][10] = ROcp14_59;
sens->J[5][11] = ROcp14_210;
sens->J[5][12] = ROcp14_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp14_95;
sens->J[6][7] = ROcp14_66;
sens->J[6][8] = ROcp14_37;
sens->J[6][9] = ROcp14_98;
sens->J[6][10] = ROcp14_69;
sens->J[6][11] = ROcp14_310;
sens->J[6][12] = ROcp14_611;
sens->A[1] = ACcp14_144;
sens->A[2] = ACcp14_244;
sens->A[3] = ACcp14_344;
sens->OMP[1] = OPcp14_112;
sens->OMP[2] = OPcp14_212;
sens->OMP[3] = OPcp14_312;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


ROcp15_25 = S4*S5;
ROcp15_35 = -C4*S5;
ROcp15_85 = -S4*C5;
ROcp15_95 = C4*C5;
ROcp15_16 = C5*C6;
ROcp15_26 = ROcp15_25*C6+C4*S6;
ROcp15_36 = ROcp15_35*C6+S4*S6;
ROcp15_46 = -C5*S6;
ROcp15_56 = -(ROcp15_25*S6-C4*C6);
ROcp15_66 = -(ROcp15_35*S6-S4*C6);
OMcp15_25 = qd[5]*C4;
OMcp15_35 = qd[5]*S4;
OMcp15_16 = qd[4]+qd[6]*S5;
OMcp15_26 = OMcp15_25+ROcp15_85*qd[6];
OMcp15_36 = OMcp15_35+ROcp15_95*qd[6];
OPcp15_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp15_26 = ROcp15_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp15_35*S5-ROcp15_95*qd[4]);
OPcp15_36 = ROcp15_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp15_25*S5-ROcp15_85*qd[4]);

// = = Block_1_0_0_16_0_2 = = 
 
// Sensor Kinematics 


ROcp15_17 = ROcp15_16*C7-S5*S7;
ROcp15_27 = ROcp15_26*C7-ROcp15_85*S7;
ROcp15_37 = ROcp15_36*C7-ROcp15_95*S7;
ROcp15_77 = ROcp15_16*S7+S5*C7;
ROcp15_87 = ROcp15_26*S7+ROcp15_85*C7;
ROcp15_97 = ROcp15_36*S7+ROcp15_95*C7;
ROcp15_48 = ROcp15_46*C8+ROcp15_77*S8;
ROcp15_58 = ROcp15_56*C8+ROcp15_87*S8;
ROcp15_68 = ROcp15_66*C8+ROcp15_97*S8;
ROcp15_78 = -(ROcp15_46*S8-ROcp15_77*C8);
ROcp15_88 = -(ROcp15_56*S8-ROcp15_87*C8);
ROcp15_98 = -(ROcp15_66*S8-ROcp15_97*C8);
ROcp15_19 = ROcp15_17*C9+ROcp15_48*S9;
ROcp15_29 = ROcp15_27*C9+ROcp15_58*S9;
ROcp15_39 = ROcp15_37*C9+ROcp15_68*S9;
ROcp15_49 = -(ROcp15_17*S9-ROcp15_48*C9);
ROcp15_59 = -(ROcp15_27*S9-ROcp15_58*C9);
ROcp15_69 = -(ROcp15_37*S9-ROcp15_68*C9);
ROcp15_110 = ROcp15_19*C10-ROcp15_78*S10;
ROcp15_210 = ROcp15_29*C10-ROcp15_88*S10;
ROcp15_310 = ROcp15_39*C10-ROcp15_98*S10;
ROcp15_710 = ROcp15_19*S10+ROcp15_78*C10;
ROcp15_810 = ROcp15_29*S10+ROcp15_88*C10;
ROcp15_910 = ROcp15_39*S10+ROcp15_98*C10;
ROcp15_411 = ROcp15_49*C11+ROcp15_710*S11;
ROcp15_511 = ROcp15_59*C11+ROcp15_810*S11;
ROcp15_611 = ROcp15_69*C11+ROcp15_910*S11;
ROcp15_711 = -(ROcp15_49*S11-ROcp15_710*C11);
ROcp15_811 = -(ROcp15_59*S11-ROcp15_810*C11);
ROcp15_911 = -(ROcp15_69*S11-ROcp15_910*C11);
ROcp15_112 = ROcp15_110*C12-ROcp15_711*S12;
ROcp15_212 = ROcp15_210*C12-ROcp15_811*S12;
ROcp15_312 = ROcp15_310*C12-ROcp15_911*S12;
ROcp15_712 = ROcp15_110*S12+ROcp15_711*C12;
ROcp15_812 = ROcp15_210*S12+ROcp15_811*C12;
ROcp15_912 = ROcp15_310*S12+ROcp15_911*C12;
RLcp15_17 = s->dpt[1][1]*ROcp15_16+s->dpt[3][1]*S5+ROcp15_46*s->dpt[2][1];
RLcp15_27 = s->dpt[1][1]*ROcp15_26+s->dpt[3][1]*ROcp15_85+ROcp15_56*s->dpt[2][1];
RLcp15_37 = s->dpt[1][1]*ROcp15_36+s->dpt[3][1]*ROcp15_95+ROcp15_66*s->dpt[2][1];
OMcp15_17 = OMcp15_16+ROcp15_46*qd[7];
OMcp15_27 = OMcp15_26+ROcp15_56*qd[7];
OMcp15_37 = OMcp15_36+ROcp15_66*qd[7];
ORcp15_17 = OMcp15_26*RLcp15_37-OMcp15_36*RLcp15_27;
ORcp15_27 = -(OMcp15_16*RLcp15_37-OMcp15_36*RLcp15_17);
ORcp15_37 = OMcp15_16*RLcp15_27-OMcp15_26*RLcp15_17;
OPcp15_17 = OPcp15_16+ROcp15_46*qdd[7]+qd[7]*(OMcp15_26*ROcp15_66-OMcp15_36*ROcp15_56);
OPcp15_27 = OPcp15_26+ROcp15_56*qdd[7]-qd[7]*(OMcp15_16*ROcp15_66-OMcp15_36*ROcp15_46);
OPcp15_37 = OPcp15_36+ROcp15_66*qdd[7]+qd[7]*(OMcp15_16*ROcp15_56-OMcp15_26*ROcp15_46);
RLcp15_18 = s->dpt[1][6]*ROcp15_17+s->dpt[3][6]*ROcp15_77+ROcp15_46*s->dpt[2][6];
RLcp15_28 = s->dpt[1][6]*ROcp15_27+s->dpt[3][6]*ROcp15_87+ROcp15_56*s->dpt[2][6];
RLcp15_38 = s->dpt[1][6]*ROcp15_37+s->dpt[3][6]*ROcp15_97+ROcp15_66*s->dpt[2][6];
OMcp15_18 = OMcp15_17+ROcp15_17*qd[8];
OMcp15_28 = OMcp15_27+ROcp15_27*qd[8];
OMcp15_38 = OMcp15_37+ROcp15_37*qd[8];
ORcp15_18 = OMcp15_27*RLcp15_38-OMcp15_37*RLcp15_28;
ORcp15_28 = -(OMcp15_17*RLcp15_38-OMcp15_37*RLcp15_18);
ORcp15_38 = OMcp15_17*RLcp15_28-OMcp15_27*RLcp15_18;
OPcp15_18 = OPcp15_17+ROcp15_17*qdd[8]+qd[8]*(OMcp15_27*ROcp15_37-OMcp15_37*ROcp15_27);
OPcp15_28 = OPcp15_27+ROcp15_27*qdd[8]-qd[8]*(OMcp15_17*ROcp15_37-OMcp15_37*ROcp15_17);
OPcp15_38 = OPcp15_37+ROcp15_37*qdd[8]+qd[8]*(OMcp15_17*ROcp15_27-OMcp15_27*ROcp15_17);
RLcp15_19 = s->dpt[1][8]*ROcp15_17+s->dpt[2][8]*ROcp15_48+ROcp15_78*s->dpt[3][8];
RLcp15_29 = s->dpt[1][8]*ROcp15_27+s->dpt[2][8]*ROcp15_58+ROcp15_88*s->dpt[3][8];
RLcp15_39 = s->dpt[1][8]*ROcp15_37+s->dpt[2][8]*ROcp15_68+ROcp15_98*s->dpt[3][8];
OMcp15_19 = OMcp15_18+ROcp15_78*qd[9];
OMcp15_29 = OMcp15_28+ROcp15_88*qd[9];
OMcp15_39 = OMcp15_38+ROcp15_98*qd[9];
ORcp15_19 = OMcp15_28*RLcp15_39-OMcp15_38*RLcp15_29;
ORcp15_29 = -(OMcp15_18*RLcp15_39-OMcp15_38*RLcp15_19);
ORcp15_39 = OMcp15_18*RLcp15_29-OMcp15_28*RLcp15_19;
OPcp15_19 = OPcp15_18+ROcp15_78*qdd[9]+qd[9]*(OMcp15_28*ROcp15_98-OMcp15_38*ROcp15_88);
OPcp15_29 = OPcp15_28+ROcp15_88*qdd[9]-qd[9]*(OMcp15_18*ROcp15_98-OMcp15_38*ROcp15_78);
OPcp15_39 = OPcp15_38+ROcp15_98*qdd[9]+qd[9]*(OMcp15_18*ROcp15_88-OMcp15_28*ROcp15_78);
RLcp15_110 = s->dpt[1][10]*ROcp15_19+s->dpt[2][10]*ROcp15_49+ROcp15_78*s->dpt[3][10];
RLcp15_210 = s->dpt[1][10]*ROcp15_29+s->dpt[2][10]*ROcp15_59+ROcp15_88*s->dpt[3][10];
RLcp15_310 = s->dpt[1][10]*ROcp15_39+s->dpt[2][10]*ROcp15_69+ROcp15_98*s->dpt[3][10];
OMcp15_110 = OMcp15_19+ROcp15_49*qd[10];
OMcp15_210 = OMcp15_29+ROcp15_59*qd[10];
OMcp15_310 = OMcp15_39+ROcp15_69*qd[10];
ORcp15_110 = OMcp15_29*RLcp15_310-OMcp15_39*RLcp15_210;
ORcp15_210 = -(OMcp15_19*RLcp15_310-OMcp15_39*RLcp15_110);
ORcp15_310 = OMcp15_19*RLcp15_210-OMcp15_29*RLcp15_110;
OPcp15_110 = OPcp15_19+ROcp15_49*qdd[10]+qd[10]*(OMcp15_29*ROcp15_69-OMcp15_39*ROcp15_59);
OPcp15_210 = OPcp15_29+ROcp15_59*qdd[10]-qd[10]*(OMcp15_19*ROcp15_69-OMcp15_39*ROcp15_49);
OPcp15_310 = OPcp15_39+ROcp15_69*qdd[10]+qd[10]*(OMcp15_19*ROcp15_59-OMcp15_29*ROcp15_49);
RLcp15_111 = s->dpt[1][12]*ROcp15_110+s->dpt[2][12]*ROcp15_49+ROcp15_710*s->dpt[3][12];
RLcp15_211 = s->dpt[1][12]*ROcp15_210+s->dpt[2][12]*ROcp15_59+ROcp15_810*s->dpt[3][12];
RLcp15_311 = s->dpt[1][12]*ROcp15_310+s->dpt[2][12]*ROcp15_69+ROcp15_910*s->dpt[3][12];
OMcp15_111 = OMcp15_110+ROcp15_110*qd[11];
OMcp15_211 = OMcp15_210+ROcp15_210*qd[11];
OMcp15_311 = OMcp15_310+ROcp15_310*qd[11];
ORcp15_111 = OMcp15_210*RLcp15_311-OMcp15_310*RLcp15_211;
ORcp15_211 = -(OMcp15_110*RLcp15_311-OMcp15_310*RLcp15_111);
ORcp15_311 = OMcp15_110*RLcp15_211-OMcp15_210*RLcp15_111;
OPcp15_111 = OPcp15_110+ROcp15_110*qdd[11]+qd[11]*(OMcp15_210*ROcp15_310-OMcp15_310*ROcp15_210);
OPcp15_211 = OPcp15_210+ROcp15_210*qdd[11]-qd[11]*(OMcp15_110*ROcp15_310-OMcp15_310*ROcp15_110);
OPcp15_311 = OPcp15_310+ROcp15_310*qdd[11]+qd[11]*(OMcp15_110*ROcp15_210-OMcp15_210*ROcp15_110);
RLcp15_112 = s->dpt[1][14]*ROcp15_110+s->dpt[2][14]*ROcp15_411+s->dpt[3][14]*ROcp15_711;
RLcp15_212 = s->dpt[1][14]*ROcp15_210+s->dpt[2][14]*ROcp15_511+s->dpt[3][14]*ROcp15_811;
RLcp15_312 = s->dpt[1][14]*ROcp15_310+s->dpt[2][14]*ROcp15_611+s->dpt[3][14]*ROcp15_911;
OMcp15_112 = OMcp15_111+ROcp15_411*qd[12];
OMcp15_212 = OMcp15_211+ROcp15_511*qd[12];
OMcp15_312 = OMcp15_311+ROcp15_611*qd[12];
ORcp15_112 = OMcp15_211*RLcp15_312-OMcp15_311*RLcp15_212;
ORcp15_212 = -(OMcp15_111*RLcp15_312-OMcp15_311*RLcp15_112);
ORcp15_312 = OMcp15_111*RLcp15_212-OMcp15_211*RLcp15_112;
OPcp15_112 = OPcp15_111+ROcp15_411*qdd[12]+qd[12]*(OMcp15_211*ROcp15_611-OMcp15_311*ROcp15_511);
OPcp15_212 = OPcp15_211+ROcp15_511*qdd[12]-qd[12]*(OMcp15_111*ROcp15_611-OMcp15_311*ROcp15_411);
OPcp15_312 = OPcp15_311+ROcp15_611*qdd[12]+qd[12]*(OMcp15_111*ROcp15_511-OMcp15_211*ROcp15_411);
RLcp15_145 = ROcp15_112*s->dpt[1][17]+ROcp15_411*s->dpt[2][17]+ROcp15_712*s->dpt[3][17];
RLcp15_245 = ROcp15_212*s->dpt[1][17]+ROcp15_511*s->dpt[2][17]+ROcp15_812*s->dpt[3][17];
RLcp15_345 = ROcp15_312*s->dpt[1][17]+ROcp15_611*s->dpt[2][17]+ROcp15_912*s->dpt[3][17];
POcp15_145 = RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_145+RLcp15_17+RLcp15_18+RLcp15_19+q[1];
POcp15_245 = RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_245+RLcp15_27+RLcp15_28+RLcp15_29+q[2];
POcp15_345 = RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_345+RLcp15_37+RLcp15_38+RLcp15_39+q[3];
JTcp15_245_4 = -(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_345+RLcp15_37+RLcp15_38+RLcp15_39);
JTcp15_345_4 = RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_245+RLcp15_27+RLcp15_28+RLcp15_29;
JTcp15_145_5 = C4*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_37+RLcp15_38+RLcp15_39)-S4*(RLcp15_210+RLcp15_29)-S4*(
 RLcp15_211+RLcp15_212)-S4*(RLcp15_27+RLcp15_28)-RLcp15_245*S4+RLcp15_345*C4;
JTcp15_245_5 = S4*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_145+RLcp15_17+RLcp15_18+RLcp15_19);
JTcp15_345_5 = -C4*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_145+RLcp15_17+RLcp15_18+RLcp15_19);
JTcp15_145_6 = ROcp15_85*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_37+RLcp15_38+RLcp15_39)-ROcp15_95*(RLcp15_210+
 RLcp15_29)-ROcp15_95*(RLcp15_211+RLcp15_212)-ROcp15_95*(RLcp15_27+RLcp15_28)-RLcp15_245*ROcp15_95+RLcp15_345*ROcp15_85;
JTcp15_245_6 = -(RLcp15_345*S5-ROcp15_95*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_145+RLcp15_17+RLcp15_18+RLcp15_19)+
 S5*(RLcp15_310+RLcp15_39)+S5*(RLcp15_311+RLcp15_312)+S5*(RLcp15_37+RLcp15_38));
JTcp15_345_6 = RLcp15_245*S5-ROcp15_85*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_145+RLcp15_17+RLcp15_18+RLcp15_19)+S5*
 (RLcp15_210+RLcp15_29)+S5*(RLcp15_211+RLcp15_212)+S5*(RLcp15_27+RLcp15_28);
JTcp15_145_7 = ROcp15_56*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_345+RLcp15_38+RLcp15_39)-ROcp15_66*(RLcp15_210+
 RLcp15_211)-ROcp15_66*(RLcp15_212+RLcp15_245)-ROcp15_66*(RLcp15_28+RLcp15_29);
JTcp15_245_7 = -(ROcp15_46*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_345+RLcp15_38+RLcp15_39)-ROcp15_66*(RLcp15_110+
 RLcp15_111)-ROcp15_66*(RLcp15_112+RLcp15_145)-ROcp15_66*(RLcp15_18+RLcp15_19));
JTcp15_345_7 = ROcp15_46*(RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_245+RLcp15_28+RLcp15_29)-ROcp15_56*(RLcp15_110+
 RLcp15_111)-ROcp15_56*(RLcp15_112+RLcp15_145)-ROcp15_56*(RLcp15_18+RLcp15_19);
JTcp15_145_8 = ROcp15_27*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_39)-ROcp15_37*(RLcp15_210+RLcp15_29)-ROcp15_37*(
 RLcp15_211+RLcp15_212)-RLcp15_245*ROcp15_37+RLcp15_345*ROcp15_27;
JTcp15_245_8 = RLcp15_145*ROcp15_37-RLcp15_345*ROcp15_17-ROcp15_17*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_39)+
 ROcp15_37*(RLcp15_110+RLcp15_19)+ROcp15_37*(RLcp15_111+RLcp15_112);
JTcp15_345_8 = ROcp15_17*(RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_29)-ROcp15_27*(RLcp15_110+RLcp15_19)-ROcp15_27*(
 RLcp15_111+RLcp15_112)-RLcp15_145*ROcp15_27+RLcp15_245*ROcp15_17;
JTcp15_145_9 = ROcp15_88*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_345)-ROcp15_98*(RLcp15_210+RLcp15_211)-ROcp15_98*(
 RLcp15_212+RLcp15_245);
JTcp15_245_9 = -(ROcp15_78*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_345)-ROcp15_98*(RLcp15_110+RLcp15_111)-ROcp15_98*(
 RLcp15_112+RLcp15_145));
JTcp15_345_9 = ROcp15_78*(RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_245)-ROcp15_88*(RLcp15_110+RLcp15_111)-ROcp15_88*(
 RLcp15_112+RLcp15_145);
JTcp15_145_10 = ROcp15_59*(RLcp15_311+RLcp15_312)-ROcp15_69*(RLcp15_211+RLcp15_212)-RLcp15_245*ROcp15_69+RLcp15_345*
 ROcp15_59;
JTcp15_245_10 = RLcp15_145*ROcp15_69-RLcp15_345*ROcp15_49-ROcp15_49*(RLcp15_311+RLcp15_312)+ROcp15_69*(RLcp15_111+
 RLcp15_112);
JTcp15_345_10 = ROcp15_49*(RLcp15_211+RLcp15_212)-ROcp15_59*(RLcp15_111+RLcp15_112)-RLcp15_145*ROcp15_59+RLcp15_245*
 ROcp15_49;
JTcp15_145_11 = ROcp15_210*(RLcp15_312+RLcp15_345)-ROcp15_310*(RLcp15_212+RLcp15_245);
JTcp15_245_11 = -(ROcp15_110*(RLcp15_312+RLcp15_345)-ROcp15_310*(RLcp15_112+RLcp15_145));
JTcp15_345_11 = ROcp15_110*(RLcp15_212+RLcp15_245)-ROcp15_210*(RLcp15_112+RLcp15_145);
JTcp15_145_12 = -(RLcp15_245*ROcp15_611-RLcp15_345*ROcp15_511);
JTcp15_245_12 = RLcp15_145*ROcp15_611-RLcp15_345*ROcp15_411;
JTcp15_345_12 = -(RLcp15_145*ROcp15_511-RLcp15_245*ROcp15_411);
ORcp15_145 = OMcp15_212*RLcp15_345-OMcp15_312*RLcp15_245;
ORcp15_245 = -(OMcp15_112*RLcp15_345-OMcp15_312*RLcp15_145);
ORcp15_345 = OMcp15_112*RLcp15_245-OMcp15_212*RLcp15_145;
VIcp15_145 = ORcp15_110+ORcp15_111+ORcp15_112+ORcp15_145+ORcp15_17+ORcp15_18+ORcp15_19+qd[1];
VIcp15_245 = ORcp15_210+ORcp15_211+ORcp15_212+ORcp15_245+ORcp15_27+ORcp15_28+ORcp15_29+qd[2];
VIcp15_345 = ORcp15_310+ORcp15_311+ORcp15_312+ORcp15_345+ORcp15_37+ORcp15_38+ORcp15_39+qd[3];
ACcp15_145 = qdd[1]+OMcp15_210*ORcp15_311+OMcp15_211*ORcp15_312+OMcp15_212*ORcp15_345+OMcp15_26*ORcp15_37+OMcp15_27*
 ORcp15_38+OMcp15_28*ORcp15_39+OMcp15_29*ORcp15_310-OMcp15_310*ORcp15_211-OMcp15_311*ORcp15_212-OMcp15_312*ORcp15_245-
 OMcp15_36*ORcp15_27-OMcp15_37*ORcp15_28-OMcp15_38*ORcp15_29-OMcp15_39*ORcp15_210+OPcp15_210*RLcp15_311+OPcp15_211*RLcp15_312
 +OPcp15_212*RLcp15_345+OPcp15_26*RLcp15_37+OPcp15_27*RLcp15_38+OPcp15_28*RLcp15_39+OPcp15_29*RLcp15_310-OPcp15_310*
 RLcp15_211-OPcp15_311*RLcp15_212-OPcp15_312*RLcp15_245-OPcp15_36*RLcp15_27-OPcp15_37*RLcp15_28-OPcp15_38*RLcp15_29-OPcp15_39
 *RLcp15_210;
ACcp15_245 = qdd[2]-OMcp15_110*ORcp15_311-OMcp15_111*ORcp15_312-OMcp15_112*ORcp15_345-OMcp15_16*ORcp15_37-OMcp15_17*
 ORcp15_38-OMcp15_18*ORcp15_39-OMcp15_19*ORcp15_310+OMcp15_310*ORcp15_111+OMcp15_311*ORcp15_112+OMcp15_312*ORcp15_145+
 OMcp15_36*ORcp15_17+OMcp15_37*ORcp15_18+OMcp15_38*ORcp15_19+OMcp15_39*ORcp15_110-OPcp15_110*RLcp15_311-OPcp15_111*RLcp15_312
 -OPcp15_112*RLcp15_345-OPcp15_16*RLcp15_37-OPcp15_17*RLcp15_38-OPcp15_18*RLcp15_39-OPcp15_19*RLcp15_310+OPcp15_310*
 RLcp15_111+OPcp15_311*RLcp15_112+OPcp15_312*RLcp15_145+OPcp15_36*RLcp15_17+OPcp15_37*RLcp15_18+OPcp15_38*RLcp15_19+OPcp15_39
 *RLcp15_110;
ACcp15_345 = qdd[3]+OMcp15_110*ORcp15_211+OMcp15_111*ORcp15_212+OMcp15_112*ORcp15_245+OMcp15_16*ORcp15_27+OMcp15_17*
 ORcp15_28+OMcp15_18*ORcp15_29+OMcp15_19*ORcp15_210-OMcp15_210*ORcp15_111-OMcp15_211*ORcp15_112-OMcp15_212*ORcp15_145-
 OMcp15_26*ORcp15_17-OMcp15_27*ORcp15_18-OMcp15_28*ORcp15_19-OMcp15_29*ORcp15_110+OPcp15_110*RLcp15_211+OPcp15_111*RLcp15_212
 +OPcp15_112*RLcp15_245+OPcp15_16*RLcp15_27+OPcp15_17*RLcp15_28+OPcp15_18*RLcp15_29+OPcp15_19*RLcp15_210-OPcp15_210*
 RLcp15_111-OPcp15_211*RLcp15_112-OPcp15_212*RLcp15_145-OPcp15_26*RLcp15_17-OPcp15_27*RLcp15_18-OPcp15_28*RLcp15_19-OPcp15_29
 *RLcp15_110;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp15_145;
sens->P[2] = POcp15_245;
sens->P[3] = POcp15_345;
sens->R[1][1] = ROcp15_112;
sens->R[1][2] = ROcp15_212;
sens->R[1][3] = ROcp15_312;
sens->R[2][1] = ROcp15_411;
sens->R[2][2] = ROcp15_511;
sens->R[2][3] = ROcp15_611;
sens->R[3][1] = ROcp15_712;
sens->R[3][2] = ROcp15_812;
sens->R[3][3] = ROcp15_912;
sens->V[1] = VIcp15_145;
sens->V[2] = VIcp15_245;
sens->V[3] = VIcp15_345;
sens->OM[1] = OMcp15_112;
sens->OM[2] = OMcp15_212;
sens->OM[3] = OMcp15_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp15_145_5;
sens->J[1][6] = JTcp15_145_6;
sens->J[1][7] = JTcp15_145_7;
sens->J[1][8] = JTcp15_145_8;
sens->J[1][9] = JTcp15_145_9;
sens->J[1][10] = JTcp15_145_10;
sens->J[1][11] = JTcp15_145_11;
sens->J[1][12] = JTcp15_145_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp15_245_4;
sens->J[2][5] = JTcp15_245_5;
sens->J[2][6] = JTcp15_245_6;
sens->J[2][7] = JTcp15_245_7;
sens->J[2][8] = JTcp15_245_8;
sens->J[2][9] = JTcp15_245_9;
sens->J[2][10] = JTcp15_245_10;
sens->J[2][11] = JTcp15_245_11;
sens->J[2][12] = JTcp15_245_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp15_345_4;
sens->J[3][5] = JTcp15_345_5;
sens->J[3][6] = JTcp15_345_6;
sens->J[3][7] = JTcp15_345_7;
sens->J[3][8] = JTcp15_345_8;
sens->J[3][9] = JTcp15_345_9;
sens->J[3][10] = JTcp15_345_10;
sens->J[3][11] = JTcp15_345_11;
sens->J[3][12] = JTcp15_345_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp15_46;
sens->J[4][8] = ROcp15_17;
sens->J[4][9] = ROcp15_78;
sens->J[4][10] = ROcp15_49;
sens->J[4][11] = ROcp15_110;
sens->J[4][12] = ROcp15_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp15_85;
sens->J[5][7] = ROcp15_56;
sens->J[5][8] = ROcp15_27;
sens->J[5][9] = ROcp15_88;
sens->J[5][10] = ROcp15_59;
sens->J[5][11] = ROcp15_210;
sens->J[5][12] = ROcp15_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp15_95;
sens->J[6][7] = ROcp15_66;
sens->J[6][8] = ROcp15_37;
sens->J[6][9] = ROcp15_98;
sens->J[6][10] = ROcp15_69;
sens->J[6][11] = ROcp15_310;
sens->J[6][12] = ROcp15_611;
sens->A[1] = ACcp15_145;
sens->A[2] = ACcp15_245;
sens->A[3] = ACcp15_345;
sens->OMP[1] = OPcp15_112;
sens->OMP[2] = OPcp15_212;
sens->OMP[3] = OPcp15_312;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


ROcp16_25 = S4*S5;
ROcp16_35 = -C4*S5;
ROcp16_85 = -S4*C5;
ROcp16_95 = C4*C5;
ROcp16_16 = C5*C6;
ROcp16_26 = ROcp16_25*C6+C4*S6;
ROcp16_36 = ROcp16_35*C6+S4*S6;
ROcp16_46 = -C5*S6;
ROcp16_56 = -(ROcp16_25*S6-C4*C6);
ROcp16_66 = -(ROcp16_35*S6-S4*C6);
OMcp16_25 = qd[5]*C4;
OMcp16_35 = qd[5]*S4;
OMcp16_16 = qd[4]+qd[6]*S5;
OMcp16_26 = OMcp16_25+ROcp16_85*qd[6];
OMcp16_36 = OMcp16_35+ROcp16_95*qd[6];
OPcp16_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp16_26 = ROcp16_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp16_35*S5-ROcp16_95*qd[4]);
OPcp16_36 = ROcp16_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp16_25*S5-ROcp16_85*qd[4]);

// = = Block_1_0_0_17_0_3 = = 
 
// Sensor Kinematics 


ROcp16_113 = ROcp16_16*C13-S13*S5;
ROcp16_213 = ROcp16_26*C13-ROcp16_85*S13;
ROcp16_313 = ROcp16_36*C13-ROcp16_95*S13;
ROcp16_713 = ROcp16_16*S13+C13*S5;
ROcp16_813 = ROcp16_26*S13+ROcp16_85*C13;
ROcp16_913 = ROcp16_36*S13+ROcp16_95*C13;
RLcp16_113 = s->dpt[1][2]*ROcp16_16+s->dpt[3][2]*S5+ROcp16_46*s->dpt[2][2];
RLcp16_213 = s->dpt[1][2]*ROcp16_26+s->dpt[3][2]*ROcp16_85+ROcp16_56*s->dpt[2][2];
RLcp16_313 = s->dpt[1][2]*ROcp16_36+s->dpt[3][2]*ROcp16_95+ROcp16_66*s->dpt[2][2];
OMcp16_113 = OMcp16_16+ROcp16_46*qd[13];
OMcp16_213 = OMcp16_26+ROcp16_56*qd[13];
OMcp16_313 = OMcp16_36+ROcp16_66*qd[13];
ORcp16_113 = OMcp16_26*RLcp16_313-OMcp16_36*RLcp16_213;
ORcp16_213 = -(OMcp16_16*RLcp16_313-OMcp16_36*RLcp16_113);
ORcp16_313 = OMcp16_16*RLcp16_213-OMcp16_26*RLcp16_113;
OPcp16_113 = OPcp16_16+ROcp16_46*qdd[13]+qd[13]*(OMcp16_26*ROcp16_66-OMcp16_36*ROcp16_56);
OPcp16_213 = OPcp16_26+ROcp16_56*qdd[13]-qd[13]*(OMcp16_16*ROcp16_66-OMcp16_36*ROcp16_46);
OPcp16_313 = OPcp16_36+ROcp16_66*qdd[13]+qd[13]*(OMcp16_16*ROcp16_56-OMcp16_26*ROcp16_46);
RLcp16_146 = s->dpt[1][22]*ROcp16_113+s->dpt[3][22]*ROcp16_713+ROcp16_46*s->dpt[2][22];
RLcp16_246 = s->dpt[1][22]*ROcp16_213+s->dpt[3][22]*ROcp16_813+ROcp16_56*s->dpt[2][22];
RLcp16_346 = s->dpt[1][22]*ROcp16_313+s->dpt[3][22]*ROcp16_913+ROcp16_66*s->dpt[2][22];
POcp16_146 = RLcp16_113+RLcp16_146+q[1];
POcp16_246 = RLcp16_213+RLcp16_246+q[2];
POcp16_346 = RLcp16_313+RLcp16_346+q[3];
JTcp16_246_4 = -(RLcp16_313+RLcp16_346);
JTcp16_346_4 = RLcp16_213+RLcp16_246;
JTcp16_146_5 = C4*(RLcp16_313+RLcp16_346)-S4*(RLcp16_213+RLcp16_246);
JTcp16_246_5 = S4*(RLcp16_113+RLcp16_146);
JTcp16_346_5 = -C4*(RLcp16_113+RLcp16_146);
JTcp16_146_6 = ROcp16_85*(RLcp16_313+RLcp16_346)-ROcp16_95*(RLcp16_213+RLcp16_246);
JTcp16_246_6 = ROcp16_95*(RLcp16_113+RLcp16_146)-S5*(RLcp16_313+RLcp16_346);
JTcp16_346_6 = -(ROcp16_85*(RLcp16_113+RLcp16_146)-S5*(RLcp16_213+RLcp16_246));
JTcp16_146_7 = -(RLcp16_246*ROcp16_66-RLcp16_346*ROcp16_56);
JTcp16_246_7 = RLcp16_146*ROcp16_66-RLcp16_346*ROcp16_46;
JTcp16_346_7 = -(RLcp16_146*ROcp16_56-RLcp16_246*ROcp16_46);
ORcp16_146 = OMcp16_213*RLcp16_346-OMcp16_313*RLcp16_246;
ORcp16_246 = -(OMcp16_113*RLcp16_346-OMcp16_313*RLcp16_146);
ORcp16_346 = OMcp16_113*RLcp16_246-OMcp16_213*RLcp16_146;
VIcp16_146 = ORcp16_113+ORcp16_146+qd[1];
VIcp16_246 = ORcp16_213+ORcp16_246+qd[2];
VIcp16_346 = ORcp16_313+ORcp16_346+qd[3];
ACcp16_146 = qdd[1]+OMcp16_213*ORcp16_346+OMcp16_26*ORcp16_313-OMcp16_313*ORcp16_246-OMcp16_36*ORcp16_213+OPcp16_213*
 RLcp16_346+OPcp16_26*RLcp16_313-OPcp16_313*RLcp16_246-OPcp16_36*RLcp16_213;
ACcp16_246 = qdd[2]-OMcp16_113*ORcp16_346-OMcp16_16*ORcp16_313+OMcp16_313*ORcp16_146+OMcp16_36*ORcp16_113-OPcp16_113*
 RLcp16_346-OPcp16_16*RLcp16_313+OPcp16_313*RLcp16_146+OPcp16_36*RLcp16_113;
ACcp16_346 = qdd[3]+OMcp16_113*ORcp16_246+OMcp16_16*ORcp16_213-OMcp16_213*ORcp16_146-OMcp16_26*ORcp16_113+OPcp16_113*
 RLcp16_246+OPcp16_16*RLcp16_213-OPcp16_213*RLcp16_146-OPcp16_26*RLcp16_113;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp16_146;
sens->P[2] = POcp16_246;
sens->P[3] = POcp16_346;
sens->R[1][1] = ROcp16_113;
sens->R[1][2] = ROcp16_213;
sens->R[1][3] = ROcp16_313;
sens->R[2][1] = ROcp16_46;
sens->R[2][2] = ROcp16_56;
sens->R[2][3] = ROcp16_66;
sens->R[3][1] = ROcp16_713;
sens->R[3][2] = ROcp16_813;
sens->R[3][3] = ROcp16_913;
sens->V[1] = VIcp16_146;
sens->V[2] = VIcp16_246;
sens->V[3] = VIcp16_346;
sens->OM[1] = OMcp16_113;
sens->OM[2] = OMcp16_213;
sens->OM[3] = OMcp16_313;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp16_146_5;
sens->J[1][6] = JTcp16_146_6;
sens->J[1][13] = JTcp16_146_7;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp16_246_4;
sens->J[2][5] = JTcp16_246_5;
sens->J[2][6] = JTcp16_246_6;
sens->J[2][13] = JTcp16_246_7;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp16_346_4;
sens->J[3][5] = JTcp16_346_5;
sens->J[3][6] = JTcp16_346_6;
sens->J[3][13] = JTcp16_346_7;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp16_46;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp16_85;
sens->J[5][13] = ROcp16_56;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp16_95;
sens->J[6][13] = ROcp16_66;
sens->A[1] = ACcp16_146;
sens->A[2] = ACcp16_246;
sens->A[3] = ACcp16_346;
sens->OMP[1] = OPcp16_113;
sens->OMP[2] = OPcp16_213;
sens->OMP[3] = OPcp16_313;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


ROcp17_25 = S4*S5;
ROcp17_35 = -C4*S5;
ROcp17_85 = -S4*C5;
ROcp17_95 = C4*C5;
ROcp17_16 = C5*C6;
ROcp17_26 = ROcp17_25*C6+C4*S6;
ROcp17_36 = ROcp17_35*C6+S4*S6;
ROcp17_46 = -C5*S6;
ROcp17_56 = -(ROcp17_25*S6-C4*C6);
ROcp17_66 = -(ROcp17_35*S6-S4*C6);
OMcp17_25 = qd[5]*C4;
OMcp17_35 = qd[5]*S4;
OMcp17_16 = qd[4]+qd[6]*S5;
OMcp17_26 = OMcp17_25+ROcp17_85*qd[6];
OMcp17_36 = OMcp17_35+ROcp17_95*qd[6];
OPcp17_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp17_26 = ROcp17_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp17_35*S5-ROcp17_95*qd[4]);
OPcp17_36 = ROcp17_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp17_25*S5-ROcp17_85*qd[4]);

// = = Block_1_0_0_18_0_3 = = 
 
// Sensor Kinematics 


ROcp17_113 = ROcp17_16*C13-S13*S5;
ROcp17_213 = ROcp17_26*C13-ROcp17_85*S13;
ROcp17_313 = ROcp17_36*C13-ROcp17_95*S13;
ROcp17_713 = ROcp17_16*S13+C13*S5;
ROcp17_813 = ROcp17_26*S13+ROcp17_85*C13;
ROcp17_913 = ROcp17_36*S13+ROcp17_95*C13;
RLcp17_113 = s->dpt[1][2]*ROcp17_16+s->dpt[3][2]*S5+ROcp17_46*s->dpt[2][2];
RLcp17_213 = s->dpt[1][2]*ROcp17_26+s->dpt[3][2]*ROcp17_85+ROcp17_56*s->dpt[2][2];
RLcp17_313 = s->dpt[1][2]*ROcp17_36+s->dpt[3][2]*ROcp17_95+ROcp17_66*s->dpt[2][2];
OMcp17_113 = OMcp17_16+ROcp17_46*qd[13];
OMcp17_213 = OMcp17_26+ROcp17_56*qd[13];
OMcp17_313 = OMcp17_36+ROcp17_66*qd[13];
ORcp17_113 = OMcp17_26*RLcp17_313-OMcp17_36*RLcp17_213;
ORcp17_213 = -(OMcp17_16*RLcp17_313-OMcp17_36*RLcp17_113);
ORcp17_313 = OMcp17_16*RLcp17_213-OMcp17_26*RLcp17_113;
OPcp17_113 = OPcp17_16+ROcp17_46*qdd[13]+qd[13]*(OMcp17_26*ROcp17_66-OMcp17_36*ROcp17_56);
OPcp17_213 = OPcp17_26+ROcp17_56*qdd[13]-qd[13]*(OMcp17_16*ROcp17_66-OMcp17_36*ROcp17_46);
OPcp17_313 = OPcp17_36+ROcp17_66*qdd[13]+qd[13]*(OMcp17_16*ROcp17_56-OMcp17_26*ROcp17_46);
RLcp17_147 = ROcp17_113*s->dpt[1][23]+ROcp17_46*s->dpt[2][23]+ROcp17_713*s->dpt[3][23];
RLcp17_247 = ROcp17_213*s->dpt[1][23]+ROcp17_56*s->dpt[2][23]+ROcp17_813*s->dpt[3][23];
RLcp17_347 = ROcp17_313*s->dpt[1][23]+ROcp17_66*s->dpt[2][23]+ROcp17_913*s->dpt[3][23];
POcp17_147 = RLcp17_113+RLcp17_147+q[1];
POcp17_247 = RLcp17_213+RLcp17_247+q[2];
POcp17_347 = RLcp17_313+RLcp17_347+q[3];
JTcp17_247_4 = -(RLcp17_313+RLcp17_347);
JTcp17_347_4 = RLcp17_213+RLcp17_247;
JTcp17_147_5 = C4*(RLcp17_313+RLcp17_347)-S4*(RLcp17_213+RLcp17_247);
JTcp17_247_5 = S4*(RLcp17_113+RLcp17_147);
JTcp17_347_5 = -C4*(RLcp17_113+RLcp17_147);
JTcp17_147_6 = ROcp17_85*(RLcp17_313+RLcp17_347)-ROcp17_95*(RLcp17_213+RLcp17_247);
JTcp17_247_6 = ROcp17_95*(RLcp17_113+RLcp17_147)-S5*(RLcp17_313+RLcp17_347);
JTcp17_347_6 = -(ROcp17_85*(RLcp17_113+RLcp17_147)-S5*(RLcp17_213+RLcp17_247));
JTcp17_147_7 = -(RLcp17_247*ROcp17_66-RLcp17_347*ROcp17_56);
JTcp17_247_7 = RLcp17_147*ROcp17_66-RLcp17_347*ROcp17_46;
JTcp17_347_7 = -(RLcp17_147*ROcp17_56-RLcp17_247*ROcp17_46);
ORcp17_147 = OMcp17_213*RLcp17_347-OMcp17_313*RLcp17_247;
ORcp17_247 = -(OMcp17_113*RLcp17_347-OMcp17_313*RLcp17_147);
ORcp17_347 = OMcp17_113*RLcp17_247-OMcp17_213*RLcp17_147;
VIcp17_147 = ORcp17_113+ORcp17_147+qd[1];
VIcp17_247 = ORcp17_213+ORcp17_247+qd[2];
VIcp17_347 = ORcp17_313+ORcp17_347+qd[3];
ACcp17_147 = qdd[1]+OMcp17_213*ORcp17_347+OMcp17_26*ORcp17_313-OMcp17_313*ORcp17_247-OMcp17_36*ORcp17_213+OPcp17_213*
 RLcp17_347+OPcp17_26*RLcp17_313-OPcp17_313*RLcp17_247-OPcp17_36*RLcp17_213;
ACcp17_247 = qdd[2]-OMcp17_113*ORcp17_347-OMcp17_16*ORcp17_313+OMcp17_313*ORcp17_147+OMcp17_36*ORcp17_113-OPcp17_113*
 RLcp17_347-OPcp17_16*RLcp17_313+OPcp17_313*RLcp17_147+OPcp17_36*RLcp17_113;
ACcp17_347 = qdd[3]+OMcp17_113*ORcp17_247+OMcp17_16*ORcp17_213-OMcp17_213*ORcp17_147-OMcp17_26*ORcp17_113+OPcp17_113*
 RLcp17_247+OPcp17_16*RLcp17_213-OPcp17_213*RLcp17_147-OPcp17_26*RLcp17_113;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp17_147;
sens->P[2] = POcp17_247;
sens->P[3] = POcp17_347;
sens->R[1][1] = ROcp17_113;
sens->R[1][2] = ROcp17_213;
sens->R[1][3] = ROcp17_313;
sens->R[2][1] = ROcp17_46;
sens->R[2][2] = ROcp17_56;
sens->R[2][3] = ROcp17_66;
sens->R[3][1] = ROcp17_713;
sens->R[3][2] = ROcp17_813;
sens->R[3][3] = ROcp17_913;
sens->V[1] = VIcp17_147;
sens->V[2] = VIcp17_247;
sens->V[3] = VIcp17_347;
sens->OM[1] = OMcp17_113;
sens->OM[2] = OMcp17_213;
sens->OM[3] = OMcp17_313;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp17_147_5;
sens->J[1][6] = JTcp17_147_6;
sens->J[1][13] = JTcp17_147_7;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp17_247_4;
sens->J[2][5] = JTcp17_247_5;
sens->J[2][6] = JTcp17_247_6;
sens->J[2][13] = JTcp17_247_7;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp17_347_4;
sens->J[3][5] = JTcp17_347_5;
sens->J[3][6] = JTcp17_347_6;
sens->J[3][13] = JTcp17_347_7;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp17_46;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp17_85;
sens->J[5][13] = ROcp17_56;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp17_95;
sens->J[6][13] = ROcp17_66;
sens->A[1] = ACcp17_147;
sens->A[2] = ACcp17_247;
sens->A[3] = ACcp17_347;
sens->OMP[1] = OPcp17_113;
sens->OMP[2] = OPcp17_213;
sens->OMP[3] = OPcp17_313;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


ROcp18_25 = S4*S5;
ROcp18_35 = -C4*S5;
ROcp18_85 = -S4*C5;
ROcp18_95 = C4*C5;
ROcp18_16 = C5*C6;
ROcp18_26 = ROcp18_25*C6+C4*S6;
ROcp18_36 = ROcp18_35*C6+S4*S6;
ROcp18_46 = -C5*S6;
ROcp18_56 = -(ROcp18_25*S6-C4*C6);
ROcp18_66 = -(ROcp18_35*S6-S4*C6);
OMcp18_25 = qd[5]*C4;
OMcp18_35 = qd[5]*S4;
OMcp18_16 = qd[4]+qd[6]*S5;
OMcp18_26 = OMcp18_25+ROcp18_85*qd[6];
OMcp18_36 = OMcp18_35+ROcp18_95*qd[6];
OPcp18_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp18_26 = ROcp18_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp18_35*S5-ROcp18_95*qd[4]);
OPcp18_36 = ROcp18_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp18_25*S5-ROcp18_85*qd[4]);

// = = Block_1_0_0_19_0_3 = = 
 
// Sensor Kinematics 


ROcp18_113 = ROcp18_16*C13-S13*S5;
ROcp18_213 = ROcp18_26*C13-ROcp18_85*S13;
ROcp18_313 = ROcp18_36*C13-ROcp18_95*S13;
ROcp18_713 = ROcp18_16*S13+C13*S5;
ROcp18_813 = ROcp18_26*S13+ROcp18_85*C13;
ROcp18_913 = ROcp18_36*S13+ROcp18_95*C13;
ROcp18_414 = ROcp18_46*C14+ROcp18_713*S14;
ROcp18_514 = ROcp18_56*C14+ROcp18_813*S14;
ROcp18_614 = ROcp18_66*C14+ROcp18_913*S14;
ROcp18_714 = -(ROcp18_46*S14-ROcp18_713*C14);
ROcp18_814 = -(ROcp18_56*S14-ROcp18_813*C14);
ROcp18_914 = -(ROcp18_66*S14-ROcp18_913*C14);
RLcp18_113 = s->dpt[1][2]*ROcp18_16+s->dpt[3][2]*S5+ROcp18_46*s->dpt[2][2];
RLcp18_213 = s->dpt[1][2]*ROcp18_26+s->dpt[3][2]*ROcp18_85+ROcp18_56*s->dpt[2][2];
RLcp18_313 = s->dpt[1][2]*ROcp18_36+s->dpt[3][2]*ROcp18_95+ROcp18_66*s->dpt[2][2];
OMcp18_113 = OMcp18_16+ROcp18_46*qd[13];
OMcp18_213 = OMcp18_26+ROcp18_56*qd[13];
OMcp18_313 = OMcp18_36+ROcp18_66*qd[13];
ORcp18_113 = OMcp18_26*RLcp18_313-OMcp18_36*RLcp18_213;
ORcp18_213 = -(OMcp18_16*RLcp18_313-OMcp18_36*RLcp18_113);
ORcp18_313 = OMcp18_16*RLcp18_213-OMcp18_26*RLcp18_113;
OPcp18_113 = OPcp18_16+ROcp18_46*qdd[13]+qd[13]*(OMcp18_26*ROcp18_66-OMcp18_36*ROcp18_56);
OPcp18_213 = OPcp18_26+ROcp18_56*qdd[13]-qd[13]*(OMcp18_16*ROcp18_66-OMcp18_36*ROcp18_46);
OPcp18_313 = OPcp18_36+ROcp18_66*qdd[13]+qd[13]*(OMcp18_16*ROcp18_56-OMcp18_26*ROcp18_46);
RLcp18_114 = s->dpt[1][22]*ROcp18_113+s->dpt[3][22]*ROcp18_713+ROcp18_46*s->dpt[2][22];
RLcp18_214 = s->dpt[1][22]*ROcp18_213+s->dpt[3][22]*ROcp18_813+ROcp18_56*s->dpt[2][22];
RLcp18_314 = s->dpt[1][22]*ROcp18_313+s->dpt[3][22]*ROcp18_913+ROcp18_66*s->dpt[2][22];
OMcp18_114 = OMcp18_113+ROcp18_113*qd[14];
OMcp18_214 = OMcp18_213+ROcp18_213*qd[14];
OMcp18_314 = OMcp18_313+ROcp18_313*qd[14];
ORcp18_114 = OMcp18_213*RLcp18_314-OMcp18_313*RLcp18_214;
ORcp18_214 = -(OMcp18_113*RLcp18_314-OMcp18_313*RLcp18_114);
ORcp18_314 = OMcp18_113*RLcp18_214-OMcp18_213*RLcp18_114;
OPcp18_114 = OPcp18_113+ROcp18_113*qdd[14]+qd[14]*(OMcp18_213*ROcp18_313-OMcp18_313*ROcp18_213);
OPcp18_214 = OPcp18_213+ROcp18_213*qdd[14]-qd[14]*(OMcp18_113*ROcp18_313-OMcp18_313*ROcp18_113);
OPcp18_314 = OPcp18_313+ROcp18_313*qdd[14]+qd[14]*(OMcp18_113*ROcp18_213-OMcp18_213*ROcp18_113);
RLcp18_148 = s->dpt[1][24]*ROcp18_113+s->dpt[2][24]*ROcp18_414+ROcp18_714*s->dpt[3][24];
RLcp18_248 = s->dpt[1][24]*ROcp18_213+s->dpt[2][24]*ROcp18_514+ROcp18_814*s->dpt[3][24];
RLcp18_348 = s->dpt[1][24]*ROcp18_313+s->dpt[2][24]*ROcp18_614+ROcp18_914*s->dpt[3][24];
POcp18_148 = RLcp18_113+RLcp18_114+RLcp18_148+q[1];
POcp18_248 = RLcp18_213+RLcp18_214+RLcp18_248+q[2];
POcp18_348 = RLcp18_313+RLcp18_314+RLcp18_348+q[3];
JTcp18_248_4 = -(RLcp18_313+RLcp18_314+RLcp18_348);
JTcp18_348_4 = RLcp18_213+RLcp18_214+RLcp18_248;
JTcp18_148_5 = C4*(RLcp18_313+RLcp18_314)-S4*(RLcp18_213+RLcp18_214)-RLcp18_248*S4+RLcp18_348*C4;
JTcp18_248_5 = S4*(RLcp18_113+RLcp18_114+RLcp18_148);
JTcp18_348_5 = -C4*(RLcp18_113+RLcp18_114+RLcp18_148);
JTcp18_148_6 = ROcp18_85*(RLcp18_313+RLcp18_314)-ROcp18_95*(RLcp18_213+RLcp18_214)-RLcp18_248*ROcp18_95+RLcp18_348*
 ROcp18_85;
JTcp18_248_6 = -(RLcp18_348*S5-ROcp18_95*(RLcp18_113+RLcp18_114+RLcp18_148)+S5*(RLcp18_313+RLcp18_314));
JTcp18_348_6 = RLcp18_248*S5-ROcp18_85*(RLcp18_113+RLcp18_114+RLcp18_148)+S5*(RLcp18_213+RLcp18_214);
JTcp18_148_7 = ROcp18_56*(RLcp18_314+RLcp18_348)-ROcp18_66*(RLcp18_214+RLcp18_248);
JTcp18_248_7 = -(ROcp18_46*(RLcp18_314+RLcp18_348)-ROcp18_66*(RLcp18_114+RLcp18_148));
JTcp18_348_7 = ROcp18_46*(RLcp18_214+RLcp18_248)-ROcp18_56*(RLcp18_114+RLcp18_148);
JTcp18_148_8 = -(RLcp18_248*ROcp18_313-RLcp18_348*ROcp18_213);
JTcp18_248_8 = RLcp18_148*ROcp18_313-RLcp18_348*ROcp18_113;
JTcp18_348_8 = -(RLcp18_148*ROcp18_213-RLcp18_248*ROcp18_113);
ORcp18_148 = OMcp18_214*RLcp18_348-OMcp18_314*RLcp18_248;
ORcp18_248 = -(OMcp18_114*RLcp18_348-OMcp18_314*RLcp18_148);
ORcp18_348 = OMcp18_114*RLcp18_248-OMcp18_214*RLcp18_148;
VIcp18_148 = ORcp18_113+ORcp18_114+ORcp18_148+qd[1];
VIcp18_248 = ORcp18_213+ORcp18_214+ORcp18_248+qd[2];
VIcp18_348 = ORcp18_313+ORcp18_314+ORcp18_348+qd[3];
ACcp18_148 = qdd[1]+OMcp18_213*ORcp18_314+OMcp18_214*ORcp18_348+OMcp18_26*ORcp18_313-OMcp18_313*ORcp18_214-OMcp18_314*
 ORcp18_248-OMcp18_36*ORcp18_213+OPcp18_213*RLcp18_314+OPcp18_214*RLcp18_348+OPcp18_26*RLcp18_313-OPcp18_313*RLcp18_214-
 OPcp18_314*RLcp18_248-OPcp18_36*RLcp18_213;
ACcp18_248 = qdd[2]-OMcp18_113*ORcp18_314-OMcp18_114*ORcp18_348-OMcp18_16*ORcp18_313+OMcp18_313*ORcp18_114+OMcp18_314*
 ORcp18_148+OMcp18_36*ORcp18_113-OPcp18_113*RLcp18_314-OPcp18_114*RLcp18_348-OPcp18_16*RLcp18_313+OPcp18_313*RLcp18_114+
 OPcp18_314*RLcp18_148+OPcp18_36*RLcp18_113;
ACcp18_348 = qdd[3]+OMcp18_113*ORcp18_214+OMcp18_114*ORcp18_248+OMcp18_16*ORcp18_213-OMcp18_213*ORcp18_114-OMcp18_214*
 ORcp18_148-OMcp18_26*ORcp18_113+OPcp18_113*RLcp18_214+OPcp18_114*RLcp18_248+OPcp18_16*RLcp18_213-OPcp18_213*RLcp18_114-
 OPcp18_214*RLcp18_148-OPcp18_26*RLcp18_113;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp18_148;
sens->P[2] = POcp18_248;
sens->P[3] = POcp18_348;
sens->R[1][1] = ROcp18_113;
sens->R[1][2] = ROcp18_213;
sens->R[1][3] = ROcp18_313;
sens->R[2][1] = ROcp18_414;
sens->R[2][2] = ROcp18_514;
sens->R[2][3] = ROcp18_614;
sens->R[3][1] = ROcp18_714;
sens->R[3][2] = ROcp18_814;
sens->R[3][3] = ROcp18_914;
sens->V[1] = VIcp18_148;
sens->V[2] = VIcp18_248;
sens->V[3] = VIcp18_348;
sens->OM[1] = OMcp18_114;
sens->OM[2] = OMcp18_214;
sens->OM[3] = OMcp18_314;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp18_148_5;
sens->J[1][6] = JTcp18_148_6;
sens->J[1][13] = JTcp18_148_7;
sens->J[1][14] = JTcp18_148_8;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp18_248_4;
sens->J[2][5] = JTcp18_248_5;
sens->J[2][6] = JTcp18_248_6;
sens->J[2][13] = JTcp18_248_7;
sens->J[2][14] = JTcp18_248_8;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp18_348_4;
sens->J[3][5] = JTcp18_348_5;
sens->J[3][6] = JTcp18_348_6;
sens->J[3][13] = JTcp18_348_7;
sens->J[3][14] = JTcp18_348_8;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp18_46;
sens->J[4][14] = ROcp18_113;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp18_85;
sens->J[5][13] = ROcp18_56;
sens->J[5][14] = ROcp18_213;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp18_95;
sens->J[6][13] = ROcp18_66;
sens->J[6][14] = ROcp18_313;
sens->A[1] = ACcp18_148;
sens->A[2] = ACcp18_248;
sens->A[3] = ACcp18_348;
sens->OMP[1] = OPcp18_114;
sens->OMP[2] = OPcp18_214;
sens->OMP[3] = OPcp18_314;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


ROcp19_25 = S4*S5;
ROcp19_35 = -C4*S5;
ROcp19_85 = -S4*C5;
ROcp19_95 = C4*C5;
ROcp19_16 = C5*C6;
ROcp19_26 = ROcp19_25*C6+C4*S6;
ROcp19_36 = ROcp19_35*C6+S4*S6;
ROcp19_46 = -C5*S6;
ROcp19_56 = -(ROcp19_25*S6-C4*C6);
ROcp19_66 = -(ROcp19_35*S6-S4*C6);
OMcp19_25 = qd[5]*C4;
OMcp19_35 = qd[5]*S4;
OMcp19_16 = qd[4]+qd[6]*S5;
OMcp19_26 = OMcp19_25+ROcp19_85*qd[6];
OMcp19_36 = OMcp19_35+ROcp19_95*qd[6];
OPcp19_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp19_26 = ROcp19_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp19_35*S5-ROcp19_95*qd[4]);
OPcp19_36 = ROcp19_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp19_25*S5-ROcp19_85*qd[4]);

// = = Block_1_0_0_20_0_3 = = 
 
// Sensor Kinematics 


ROcp19_113 = ROcp19_16*C13-S13*S5;
ROcp19_213 = ROcp19_26*C13-ROcp19_85*S13;
ROcp19_313 = ROcp19_36*C13-ROcp19_95*S13;
ROcp19_713 = ROcp19_16*S13+C13*S5;
ROcp19_813 = ROcp19_26*S13+ROcp19_85*C13;
ROcp19_913 = ROcp19_36*S13+ROcp19_95*C13;
ROcp19_414 = ROcp19_46*C14+ROcp19_713*S14;
ROcp19_514 = ROcp19_56*C14+ROcp19_813*S14;
ROcp19_614 = ROcp19_66*C14+ROcp19_913*S14;
ROcp19_714 = -(ROcp19_46*S14-ROcp19_713*C14);
ROcp19_814 = -(ROcp19_56*S14-ROcp19_813*C14);
ROcp19_914 = -(ROcp19_66*S14-ROcp19_913*C14);
RLcp19_113 = s->dpt[1][2]*ROcp19_16+s->dpt[3][2]*S5+ROcp19_46*s->dpt[2][2];
RLcp19_213 = s->dpt[1][2]*ROcp19_26+s->dpt[3][2]*ROcp19_85+ROcp19_56*s->dpt[2][2];
RLcp19_313 = s->dpt[1][2]*ROcp19_36+s->dpt[3][2]*ROcp19_95+ROcp19_66*s->dpt[2][2];
OMcp19_113 = OMcp19_16+ROcp19_46*qd[13];
OMcp19_213 = OMcp19_26+ROcp19_56*qd[13];
OMcp19_313 = OMcp19_36+ROcp19_66*qd[13];
ORcp19_113 = OMcp19_26*RLcp19_313-OMcp19_36*RLcp19_213;
ORcp19_213 = -(OMcp19_16*RLcp19_313-OMcp19_36*RLcp19_113);
ORcp19_313 = OMcp19_16*RLcp19_213-OMcp19_26*RLcp19_113;
OPcp19_113 = OPcp19_16+ROcp19_46*qdd[13]+qd[13]*(OMcp19_26*ROcp19_66-OMcp19_36*ROcp19_56);
OPcp19_213 = OPcp19_26+ROcp19_56*qdd[13]-qd[13]*(OMcp19_16*ROcp19_66-OMcp19_36*ROcp19_46);
OPcp19_313 = OPcp19_36+ROcp19_66*qdd[13]+qd[13]*(OMcp19_16*ROcp19_56-OMcp19_26*ROcp19_46);
RLcp19_114 = s->dpt[1][22]*ROcp19_113+s->dpt[3][22]*ROcp19_713+ROcp19_46*s->dpt[2][22];
RLcp19_214 = s->dpt[1][22]*ROcp19_213+s->dpt[3][22]*ROcp19_813+ROcp19_56*s->dpt[2][22];
RLcp19_314 = s->dpt[1][22]*ROcp19_313+s->dpt[3][22]*ROcp19_913+ROcp19_66*s->dpt[2][22];
OMcp19_114 = OMcp19_113+ROcp19_113*qd[14];
OMcp19_214 = OMcp19_213+ROcp19_213*qd[14];
OMcp19_314 = OMcp19_313+ROcp19_313*qd[14];
ORcp19_114 = OMcp19_213*RLcp19_314-OMcp19_313*RLcp19_214;
ORcp19_214 = -(OMcp19_113*RLcp19_314-OMcp19_313*RLcp19_114);
ORcp19_314 = OMcp19_113*RLcp19_214-OMcp19_213*RLcp19_114;
OPcp19_114 = OPcp19_113+ROcp19_113*qdd[14]+qd[14]*(OMcp19_213*ROcp19_313-OMcp19_313*ROcp19_213);
OPcp19_214 = OPcp19_213+ROcp19_213*qdd[14]-qd[14]*(OMcp19_113*ROcp19_313-OMcp19_313*ROcp19_113);
OPcp19_314 = OPcp19_313+ROcp19_313*qdd[14]+qd[14]*(OMcp19_113*ROcp19_213-OMcp19_213*ROcp19_113);
RLcp19_149 = ROcp19_113*s->dpt[1][25]+ROcp19_414*s->dpt[2][25]+ROcp19_714*s->dpt[3][25];
RLcp19_249 = ROcp19_213*s->dpt[1][25]+ROcp19_514*s->dpt[2][25]+ROcp19_814*s->dpt[3][25];
RLcp19_349 = ROcp19_313*s->dpt[1][25]+ROcp19_614*s->dpt[2][25]+ROcp19_914*s->dpt[3][25];
POcp19_149 = RLcp19_113+RLcp19_114+RLcp19_149+q[1];
POcp19_249 = RLcp19_213+RLcp19_214+RLcp19_249+q[2];
POcp19_349 = RLcp19_313+RLcp19_314+RLcp19_349+q[3];
JTcp19_249_4 = -(RLcp19_313+RLcp19_314+RLcp19_349);
JTcp19_349_4 = RLcp19_213+RLcp19_214+RLcp19_249;
JTcp19_149_5 = C4*(RLcp19_313+RLcp19_314)-S4*(RLcp19_213+RLcp19_214)-RLcp19_249*S4+RLcp19_349*C4;
JTcp19_249_5 = S4*(RLcp19_113+RLcp19_114+RLcp19_149);
JTcp19_349_5 = -C4*(RLcp19_113+RLcp19_114+RLcp19_149);
JTcp19_149_6 = ROcp19_85*(RLcp19_313+RLcp19_314)-ROcp19_95*(RLcp19_213+RLcp19_214)-RLcp19_249*ROcp19_95+RLcp19_349*
 ROcp19_85;
JTcp19_249_6 = -(RLcp19_349*S5-ROcp19_95*(RLcp19_113+RLcp19_114+RLcp19_149)+S5*(RLcp19_313+RLcp19_314));
JTcp19_349_6 = RLcp19_249*S5-ROcp19_85*(RLcp19_113+RLcp19_114+RLcp19_149)+S5*(RLcp19_213+RLcp19_214);
JTcp19_149_7 = ROcp19_56*(RLcp19_314+RLcp19_349)-ROcp19_66*(RLcp19_214+RLcp19_249);
JTcp19_249_7 = -(ROcp19_46*(RLcp19_314+RLcp19_349)-ROcp19_66*(RLcp19_114+RLcp19_149));
JTcp19_349_7 = ROcp19_46*(RLcp19_214+RLcp19_249)-ROcp19_56*(RLcp19_114+RLcp19_149);
JTcp19_149_8 = -(RLcp19_249*ROcp19_313-RLcp19_349*ROcp19_213);
JTcp19_249_8 = RLcp19_149*ROcp19_313-RLcp19_349*ROcp19_113;
JTcp19_349_8 = -(RLcp19_149*ROcp19_213-RLcp19_249*ROcp19_113);
ORcp19_149 = OMcp19_214*RLcp19_349-OMcp19_314*RLcp19_249;
ORcp19_249 = -(OMcp19_114*RLcp19_349-OMcp19_314*RLcp19_149);
ORcp19_349 = OMcp19_114*RLcp19_249-OMcp19_214*RLcp19_149;
VIcp19_149 = ORcp19_113+ORcp19_114+ORcp19_149+qd[1];
VIcp19_249 = ORcp19_213+ORcp19_214+ORcp19_249+qd[2];
VIcp19_349 = ORcp19_313+ORcp19_314+ORcp19_349+qd[3];
ACcp19_149 = qdd[1]+OMcp19_213*ORcp19_314+OMcp19_214*ORcp19_349+OMcp19_26*ORcp19_313-OMcp19_313*ORcp19_214-OMcp19_314*
 ORcp19_249-OMcp19_36*ORcp19_213+OPcp19_213*RLcp19_314+OPcp19_214*RLcp19_349+OPcp19_26*RLcp19_313-OPcp19_313*RLcp19_214-
 OPcp19_314*RLcp19_249-OPcp19_36*RLcp19_213;
ACcp19_249 = qdd[2]-OMcp19_113*ORcp19_314-OMcp19_114*ORcp19_349-OMcp19_16*ORcp19_313+OMcp19_313*ORcp19_114+OMcp19_314*
 ORcp19_149+OMcp19_36*ORcp19_113-OPcp19_113*RLcp19_314-OPcp19_114*RLcp19_349-OPcp19_16*RLcp19_313+OPcp19_313*RLcp19_114+
 OPcp19_314*RLcp19_149+OPcp19_36*RLcp19_113;
ACcp19_349 = qdd[3]+OMcp19_113*ORcp19_214+OMcp19_114*ORcp19_249+OMcp19_16*ORcp19_213-OMcp19_213*ORcp19_114-OMcp19_214*
 ORcp19_149-OMcp19_26*ORcp19_113+OPcp19_113*RLcp19_214+OPcp19_114*RLcp19_249+OPcp19_16*RLcp19_213-OPcp19_213*RLcp19_114-
 OPcp19_214*RLcp19_149-OPcp19_26*RLcp19_113;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp19_149;
sens->P[2] = POcp19_249;
sens->P[3] = POcp19_349;
sens->R[1][1] = ROcp19_113;
sens->R[1][2] = ROcp19_213;
sens->R[1][3] = ROcp19_313;
sens->R[2][1] = ROcp19_414;
sens->R[2][2] = ROcp19_514;
sens->R[2][3] = ROcp19_614;
sens->R[3][1] = ROcp19_714;
sens->R[3][2] = ROcp19_814;
sens->R[3][3] = ROcp19_914;
sens->V[1] = VIcp19_149;
sens->V[2] = VIcp19_249;
sens->V[3] = VIcp19_349;
sens->OM[1] = OMcp19_114;
sens->OM[2] = OMcp19_214;
sens->OM[3] = OMcp19_314;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp19_149_5;
sens->J[1][6] = JTcp19_149_6;
sens->J[1][13] = JTcp19_149_7;
sens->J[1][14] = JTcp19_149_8;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp19_249_4;
sens->J[2][5] = JTcp19_249_5;
sens->J[2][6] = JTcp19_249_6;
sens->J[2][13] = JTcp19_249_7;
sens->J[2][14] = JTcp19_249_8;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp19_349_4;
sens->J[3][5] = JTcp19_349_5;
sens->J[3][6] = JTcp19_349_6;
sens->J[3][13] = JTcp19_349_7;
sens->J[3][14] = JTcp19_349_8;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp19_46;
sens->J[4][14] = ROcp19_113;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp19_85;
sens->J[5][13] = ROcp19_56;
sens->J[5][14] = ROcp19_213;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp19_95;
sens->J[6][13] = ROcp19_66;
sens->J[6][14] = ROcp19_313;
sens->A[1] = ACcp19_149;
sens->A[2] = ACcp19_249;
sens->A[3] = ACcp19_349;
sens->OMP[1] = OPcp19_114;
sens->OMP[2] = OPcp19_214;
sens->OMP[3] = OPcp19_314;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


ROcp20_25 = S4*S5;
ROcp20_35 = -C4*S5;
ROcp20_85 = -S4*C5;
ROcp20_95 = C4*C5;
ROcp20_16 = C5*C6;
ROcp20_26 = ROcp20_25*C6+C4*S6;
ROcp20_36 = ROcp20_35*C6+S4*S6;
ROcp20_46 = -C5*S6;
ROcp20_56 = -(ROcp20_25*S6-C4*C6);
ROcp20_66 = -(ROcp20_35*S6-S4*C6);
OMcp20_25 = qd[5]*C4;
OMcp20_35 = qd[5]*S4;
OMcp20_16 = qd[4]+qd[6]*S5;
OMcp20_26 = OMcp20_25+ROcp20_85*qd[6];
OMcp20_36 = OMcp20_35+ROcp20_95*qd[6];
OPcp20_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp20_26 = ROcp20_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp20_35*S5-ROcp20_95*qd[4]);
OPcp20_36 = ROcp20_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp20_25*S5-ROcp20_85*qd[4]);

// = = Block_1_0_0_21_0_3 = = 
 
// Sensor Kinematics 


ROcp20_113 = ROcp20_16*C13-S13*S5;
ROcp20_213 = ROcp20_26*C13-ROcp20_85*S13;
ROcp20_313 = ROcp20_36*C13-ROcp20_95*S13;
ROcp20_713 = ROcp20_16*S13+C13*S5;
ROcp20_813 = ROcp20_26*S13+ROcp20_85*C13;
ROcp20_913 = ROcp20_36*S13+ROcp20_95*C13;
ROcp20_414 = ROcp20_46*C14+ROcp20_713*S14;
ROcp20_514 = ROcp20_56*C14+ROcp20_813*S14;
ROcp20_614 = ROcp20_66*C14+ROcp20_913*S14;
ROcp20_714 = -(ROcp20_46*S14-ROcp20_713*C14);
ROcp20_814 = -(ROcp20_56*S14-ROcp20_813*C14);
ROcp20_914 = -(ROcp20_66*S14-ROcp20_913*C14);
ROcp20_115 = ROcp20_113*C15+ROcp20_414*S15;
ROcp20_215 = ROcp20_213*C15+ROcp20_514*S15;
ROcp20_315 = ROcp20_313*C15+ROcp20_614*S15;
ROcp20_415 = -(ROcp20_113*S15-ROcp20_414*C15);
ROcp20_515 = -(ROcp20_213*S15-ROcp20_514*C15);
ROcp20_615 = -(ROcp20_313*S15-ROcp20_614*C15);
RLcp20_113 = s->dpt[1][2]*ROcp20_16+s->dpt[3][2]*S5+ROcp20_46*s->dpt[2][2];
RLcp20_213 = s->dpt[1][2]*ROcp20_26+s->dpt[3][2]*ROcp20_85+ROcp20_56*s->dpt[2][2];
RLcp20_313 = s->dpt[1][2]*ROcp20_36+s->dpt[3][2]*ROcp20_95+ROcp20_66*s->dpt[2][2];
OMcp20_113 = OMcp20_16+ROcp20_46*qd[13];
OMcp20_213 = OMcp20_26+ROcp20_56*qd[13];
OMcp20_313 = OMcp20_36+ROcp20_66*qd[13];
ORcp20_113 = OMcp20_26*RLcp20_313-OMcp20_36*RLcp20_213;
ORcp20_213 = -(OMcp20_16*RLcp20_313-OMcp20_36*RLcp20_113);
ORcp20_313 = OMcp20_16*RLcp20_213-OMcp20_26*RLcp20_113;
OPcp20_113 = OPcp20_16+ROcp20_46*qdd[13]+qd[13]*(OMcp20_26*ROcp20_66-OMcp20_36*ROcp20_56);
OPcp20_213 = OPcp20_26+ROcp20_56*qdd[13]-qd[13]*(OMcp20_16*ROcp20_66-OMcp20_36*ROcp20_46);
OPcp20_313 = OPcp20_36+ROcp20_66*qdd[13]+qd[13]*(OMcp20_16*ROcp20_56-OMcp20_26*ROcp20_46);
RLcp20_114 = s->dpt[1][22]*ROcp20_113+s->dpt[3][22]*ROcp20_713+ROcp20_46*s->dpt[2][22];
RLcp20_214 = s->dpt[1][22]*ROcp20_213+s->dpt[3][22]*ROcp20_813+ROcp20_56*s->dpt[2][22];
RLcp20_314 = s->dpt[1][22]*ROcp20_313+s->dpt[3][22]*ROcp20_913+ROcp20_66*s->dpt[2][22];
OMcp20_114 = OMcp20_113+ROcp20_113*qd[14];
OMcp20_214 = OMcp20_213+ROcp20_213*qd[14];
OMcp20_314 = OMcp20_313+ROcp20_313*qd[14];
ORcp20_114 = OMcp20_213*RLcp20_314-OMcp20_313*RLcp20_214;
ORcp20_214 = -(OMcp20_113*RLcp20_314-OMcp20_313*RLcp20_114);
ORcp20_314 = OMcp20_113*RLcp20_214-OMcp20_213*RLcp20_114;
OPcp20_114 = OPcp20_113+ROcp20_113*qdd[14]+qd[14]*(OMcp20_213*ROcp20_313-OMcp20_313*ROcp20_213);
OPcp20_214 = OPcp20_213+ROcp20_213*qdd[14]-qd[14]*(OMcp20_113*ROcp20_313-OMcp20_313*ROcp20_113);
OPcp20_314 = OPcp20_313+ROcp20_313*qdd[14]+qd[14]*(OMcp20_113*ROcp20_213-OMcp20_213*ROcp20_113);
RLcp20_115 = s->dpt[1][24]*ROcp20_113+s->dpt[2][24]*ROcp20_414+ROcp20_714*s->dpt[3][24];
RLcp20_215 = s->dpt[1][24]*ROcp20_213+s->dpt[2][24]*ROcp20_514+ROcp20_814*s->dpt[3][24];
RLcp20_315 = s->dpt[1][24]*ROcp20_313+s->dpt[2][24]*ROcp20_614+ROcp20_914*s->dpt[3][24];
OMcp20_115 = OMcp20_114+ROcp20_714*qd[15];
OMcp20_215 = OMcp20_214+ROcp20_814*qd[15];
OMcp20_315 = OMcp20_314+ROcp20_914*qd[15];
ORcp20_115 = OMcp20_214*RLcp20_315-OMcp20_314*RLcp20_215;
ORcp20_215 = -(OMcp20_114*RLcp20_315-OMcp20_314*RLcp20_115);
ORcp20_315 = OMcp20_114*RLcp20_215-OMcp20_214*RLcp20_115;
OPcp20_115 = OPcp20_114+ROcp20_714*qdd[15]+qd[15]*(OMcp20_214*ROcp20_914-OMcp20_314*ROcp20_814);
OPcp20_215 = OPcp20_214+ROcp20_814*qdd[15]-qd[15]*(OMcp20_114*ROcp20_914-OMcp20_314*ROcp20_714);
OPcp20_315 = OPcp20_314+ROcp20_914*qdd[15]+qd[15]*(OMcp20_114*ROcp20_814-OMcp20_214*ROcp20_714);
RLcp20_150 = s->dpt[1][26]*ROcp20_115+s->dpt[2][26]*ROcp20_415+ROcp20_714*s->dpt[3][26];
RLcp20_250 = s->dpt[1][26]*ROcp20_215+s->dpt[2][26]*ROcp20_515+ROcp20_814*s->dpt[3][26];
RLcp20_350 = s->dpt[1][26]*ROcp20_315+s->dpt[2][26]*ROcp20_615+ROcp20_914*s->dpt[3][26];
POcp20_150 = RLcp20_113+RLcp20_114+RLcp20_115+RLcp20_150+q[1];
POcp20_250 = RLcp20_213+RLcp20_214+RLcp20_215+RLcp20_250+q[2];
POcp20_350 = RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_350+q[3];
JTcp20_250_4 = -(RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_350);
JTcp20_350_4 = RLcp20_213+RLcp20_214+RLcp20_215+RLcp20_250;
JTcp20_150_5 = C4*(RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_350)-S4*(RLcp20_213+RLcp20_214)-S4*(RLcp20_215+RLcp20_250);
JTcp20_250_5 = S4*(RLcp20_113+RLcp20_114+RLcp20_115+RLcp20_150);
JTcp20_350_5 = -C4*(RLcp20_113+RLcp20_114+RLcp20_115+RLcp20_150);
JTcp20_150_6 = ROcp20_85*(RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_350)-ROcp20_95*(RLcp20_213+RLcp20_214)-ROcp20_95*(
 RLcp20_215+RLcp20_250);
JTcp20_250_6 = RLcp20_150*ROcp20_95-RLcp20_315*S5-RLcp20_350*S5+ROcp20_95*(RLcp20_113+RLcp20_114+RLcp20_115)-S5*(
 RLcp20_313+RLcp20_314);
JTcp20_350_6 = RLcp20_215*S5-ROcp20_85*(RLcp20_113+RLcp20_114+RLcp20_115)+S5*(RLcp20_213+RLcp20_214)-RLcp20_150*
 ROcp20_85+RLcp20_250*S5;
JTcp20_150_7 = ROcp20_56*(RLcp20_314+RLcp20_315)-ROcp20_66*(RLcp20_214+RLcp20_215)-RLcp20_250*ROcp20_66+RLcp20_350*
 ROcp20_56;
JTcp20_250_7 = RLcp20_150*ROcp20_66-RLcp20_350*ROcp20_46-ROcp20_46*(RLcp20_314+RLcp20_315)+ROcp20_66*(RLcp20_114+
 RLcp20_115);
JTcp20_350_7 = ROcp20_46*(RLcp20_214+RLcp20_215)-ROcp20_56*(RLcp20_114+RLcp20_115)-RLcp20_150*ROcp20_56+RLcp20_250*
 ROcp20_46;
JTcp20_150_8 = ROcp20_213*(RLcp20_315+RLcp20_350)-ROcp20_313*(RLcp20_215+RLcp20_250);
JTcp20_250_8 = -(ROcp20_113*(RLcp20_315+RLcp20_350)-ROcp20_313*(RLcp20_115+RLcp20_150));
JTcp20_350_8 = ROcp20_113*(RLcp20_215+RLcp20_250)-ROcp20_213*(RLcp20_115+RLcp20_150);
JTcp20_150_9 = -(RLcp20_250*ROcp20_914-RLcp20_350*ROcp20_814);
JTcp20_250_9 = RLcp20_150*ROcp20_914-RLcp20_350*ROcp20_714;
JTcp20_350_9 = -(RLcp20_150*ROcp20_814-RLcp20_250*ROcp20_714);
ORcp20_150 = OMcp20_215*RLcp20_350-OMcp20_315*RLcp20_250;
ORcp20_250 = -(OMcp20_115*RLcp20_350-OMcp20_315*RLcp20_150);
ORcp20_350 = OMcp20_115*RLcp20_250-OMcp20_215*RLcp20_150;
VIcp20_150 = ORcp20_113+ORcp20_114+ORcp20_115+ORcp20_150+qd[1];
VIcp20_250 = ORcp20_213+ORcp20_214+ORcp20_215+ORcp20_250+qd[2];
VIcp20_350 = ORcp20_313+ORcp20_314+ORcp20_315+ORcp20_350+qd[3];
ACcp20_150 = qdd[1]+OMcp20_213*ORcp20_314+OMcp20_214*ORcp20_315+OMcp20_215*ORcp20_350+OMcp20_26*ORcp20_313-OMcp20_313*
 ORcp20_214-OMcp20_314*ORcp20_215-OMcp20_315*ORcp20_250-OMcp20_36*ORcp20_213+OPcp20_213*RLcp20_314+OPcp20_214*RLcp20_315+
 OPcp20_215*RLcp20_350+OPcp20_26*RLcp20_313-OPcp20_313*RLcp20_214-OPcp20_314*RLcp20_215-OPcp20_315*RLcp20_250-OPcp20_36*
 RLcp20_213;
ACcp20_250 = qdd[2]-OMcp20_113*ORcp20_314-OMcp20_114*ORcp20_315-OMcp20_115*ORcp20_350-OMcp20_16*ORcp20_313+OMcp20_313*
 ORcp20_114+OMcp20_314*ORcp20_115+OMcp20_315*ORcp20_150+OMcp20_36*ORcp20_113-OPcp20_113*RLcp20_314-OPcp20_114*RLcp20_315-
 OPcp20_115*RLcp20_350-OPcp20_16*RLcp20_313+OPcp20_313*RLcp20_114+OPcp20_314*RLcp20_115+OPcp20_315*RLcp20_150+OPcp20_36*
 RLcp20_113;
ACcp20_350 = qdd[3]+OMcp20_113*ORcp20_214+OMcp20_114*ORcp20_215+OMcp20_115*ORcp20_250+OMcp20_16*ORcp20_213-OMcp20_213*
 ORcp20_114-OMcp20_214*ORcp20_115-OMcp20_215*ORcp20_150-OMcp20_26*ORcp20_113+OPcp20_113*RLcp20_214+OPcp20_114*RLcp20_215+
 OPcp20_115*RLcp20_250+OPcp20_16*RLcp20_213-OPcp20_213*RLcp20_114-OPcp20_214*RLcp20_115-OPcp20_215*RLcp20_150-OPcp20_26*
 RLcp20_113;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp20_150;
sens->P[2] = POcp20_250;
sens->P[3] = POcp20_350;
sens->R[1][1] = ROcp20_115;
sens->R[1][2] = ROcp20_215;
sens->R[1][3] = ROcp20_315;
sens->R[2][1] = ROcp20_415;
sens->R[2][2] = ROcp20_515;
sens->R[2][3] = ROcp20_615;
sens->R[3][1] = ROcp20_714;
sens->R[3][2] = ROcp20_814;
sens->R[3][3] = ROcp20_914;
sens->V[1] = VIcp20_150;
sens->V[2] = VIcp20_250;
sens->V[3] = VIcp20_350;
sens->OM[1] = OMcp20_115;
sens->OM[2] = OMcp20_215;
sens->OM[3] = OMcp20_315;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp20_150_5;
sens->J[1][6] = JTcp20_150_6;
sens->J[1][13] = JTcp20_150_7;
sens->J[1][14] = JTcp20_150_8;
sens->J[1][15] = JTcp20_150_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp20_250_4;
sens->J[2][5] = JTcp20_250_5;
sens->J[2][6] = JTcp20_250_6;
sens->J[2][13] = JTcp20_250_7;
sens->J[2][14] = JTcp20_250_8;
sens->J[2][15] = JTcp20_250_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp20_350_4;
sens->J[3][5] = JTcp20_350_5;
sens->J[3][6] = JTcp20_350_6;
sens->J[3][13] = JTcp20_350_7;
sens->J[3][14] = JTcp20_350_8;
sens->J[3][15] = JTcp20_350_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp20_46;
sens->J[4][14] = ROcp20_113;
sens->J[4][15] = ROcp20_714;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp20_85;
sens->J[5][13] = ROcp20_56;
sens->J[5][14] = ROcp20_213;
sens->J[5][15] = ROcp20_814;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp20_95;
sens->J[6][13] = ROcp20_66;
sens->J[6][14] = ROcp20_313;
sens->J[6][15] = ROcp20_914;
sens->A[1] = ACcp20_150;
sens->A[2] = ACcp20_250;
sens->A[3] = ACcp20_350;
sens->OMP[1] = OPcp20_115;
sens->OMP[2] = OPcp20_215;
sens->OMP[3] = OPcp20_315;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


ROcp21_25 = S4*S5;
ROcp21_35 = -C4*S5;
ROcp21_85 = -S4*C5;
ROcp21_95 = C4*C5;
ROcp21_16 = C5*C6;
ROcp21_26 = ROcp21_25*C6+C4*S6;
ROcp21_36 = ROcp21_35*C6+S4*S6;
ROcp21_46 = -C5*S6;
ROcp21_56 = -(ROcp21_25*S6-C4*C6);
ROcp21_66 = -(ROcp21_35*S6-S4*C6);
OMcp21_25 = qd[5]*C4;
OMcp21_35 = qd[5]*S4;
OMcp21_16 = qd[4]+qd[6]*S5;
OMcp21_26 = OMcp21_25+ROcp21_85*qd[6];
OMcp21_36 = OMcp21_35+ROcp21_95*qd[6];
OPcp21_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp21_26 = ROcp21_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp21_35*S5-ROcp21_95*qd[4]);
OPcp21_36 = ROcp21_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp21_25*S5-ROcp21_85*qd[4]);

// = = Block_1_0_0_22_0_3 = = 
 
// Sensor Kinematics 


ROcp21_113 = ROcp21_16*C13-S13*S5;
ROcp21_213 = ROcp21_26*C13-ROcp21_85*S13;
ROcp21_313 = ROcp21_36*C13-ROcp21_95*S13;
ROcp21_713 = ROcp21_16*S13+C13*S5;
ROcp21_813 = ROcp21_26*S13+ROcp21_85*C13;
ROcp21_913 = ROcp21_36*S13+ROcp21_95*C13;
ROcp21_414 = ROcp21_46*C14+ROcp21_713*S14;
ROcp21_514 = ROcp21_56*C14+ROcp21_813*S14;
ROcp21_614 = ROcp21_66*C14+ROcp21_913*S14;
ROcp21_714 = -(ROcp21_46*S14-ROcp21_713*C14);
ROcp21_814 = -(ROcp21_56*S14-ROcp21_813*C14);
ROcp21_914 = -(ROcp21_66*S14-ROcp21_913*C14);
ROcp21_115 = ROcp21_113*C15+ROcp21_414*S15;
ROcp21_215 = ROcp21_213*C15+ROcp21_514*S15;
ROcp21_315 = ROcp21_313*C15+ROcp21_614*S15;
ROcp21_415 = -(ROcp21_113*S15-ROcp21_414*C15);
ROcp21_515 = -(ROcp21_213*S15-ROcp21_514*C15);
ROcp21_615 = -(ROcp21_313*S15-ROcp21_614*C15);
RLcp21_113 = s->dpt[1][2]*ROcp21_16+s->dpt[3][2]*S5+ROcp21_46*s->dpt[2][2];
RLcp21_213 = s->dpt[1][2]*ROcp21_26+s->dpt[3][2]*ROcp21_85+ROcp21_56*s->dpt[2][2];
RLcp21_313 = s->dpt[1][2]*ROcp21_36+s->dpt[3][2]*ROcp21_95+ROcp21_66*s->dpt[2][2];
OMcp21_113 = OMcp21_16+ROcp21_46*qd[13];
OMcp21_213 = OMcp21_26+ROcp21_56*qd[13];
OMcp21_313 = OMcp21_36+ROcp21_66*qd[13];
ORcp21_113 = OMcp21_26*RLcp21_313-OMcp21_36*RLcp21_213;
ORcp21_213 = -(OMcp21_16*RLcp21_313-OMcp21_36*RLcp21_113);
ORcp21_313 = OMcp21_16*RLcp21_213-OMcp21_26*RLcp21_113;
OPcp21_113 = OPcp21_16+ROcp21_46*qdd[13]+qd[13]*(OMcp21_26*ROcp21_66-OMcp21_36*ROcp21_56);
OPcp21_213 = OPcp21_26+ROcp21_56*qdd[13]-qd[13]*(OMcp21_16*ROcp21_66-OMcp21_36*ROcp21_46);
OPcp21_313 = OPcp21_36+ROcp21_66*qdd[13]+qd[13]*(OMcp21_16*ROcp21_56-OMcp21_26*ROcp21_46);
RLcp21_114 = s->dpt[1][22]*ROcp21_113+s->dpt[3][22]*ROcp21_713+ROcp21_46*s->dpt[2][22];
RLcp21_214 = s->dpt[1][22]*ROcp21_213+s->dpt[3][22]*ROcp21_813+ROcp21_56*s->dpt[2][22];
RLcp21_314 = s->dpt[1][22]*ROcp21_313+s->dpt[3][22]*ROcp21_913+ROcp21_66*s->dpt[2][22];
OMcp21_114 = OMcp21_113+ROcp21_113*qd[14];
OMcp21_214 = OMcp21_213+ROcp21_213*qd[14];
OMcp21_314 = OMcp21_313+ROcp21_313*qd[14];
ORcp21_114 = OMcp21_213*RLcp21_314-OMcp21_313*RLcp21_214;
ORcp21_214 = -(OMcp21_113*RLcp21_314-OMcp21_313*RLcp21_114);
ORcp21_314 = OMcp21_113*RLcp21_214-OMcp21_213*RLcp21_114;
OPcp21_114 = OPcp21_113+ROcp21_113*qdd[14]+qd[14]*(OMcp21_213*ROcp21_313-OMcp21_313*ROcp21_213);
OPcp21_214 = OPcp21_213+ROcp21_213*qdd[14]-qd[14]*(OMcp21_113*ROcp21_313-OMcp21_313*ROcp21_113);
OPcp21_314 = OPcp21_313+ROcp21_313*qdd[14]+qd[14]*(OMcp21_113*ROcp21_213-OMcp21_213*ROcp21_113);
RLcp21_115 = s->dpt[1][24]*ROcp21_113+s->dpt[2][24]*ROcp21_414+ROcp21_714*s->dpt[3][24];
RLcp21_215 = s->dpt[1][24]*ROcp21_213+s->dpt[2][24]*ROcp21_514+ROcp21_814*s->dpt[3][24];
RLcp21_315 = s->dpt[1][24]*ROcp21_313+s->dpt[2][24]*ROcp21_614+ROcp21_914*s->dpt[3][24];
OMcp21_115 = OMcp21_114+ROcp21_714*qd[15];
OMcp21_215 = OMcp21_214+ROcp21_814*qd[15];
OMcp21_315 = OMcp21_314+ROcp21_914*qd[15];
ORcp21_115 = OMcp21_214*RLcp21_315-OMcp21_314*RLcp21_215;
ORcp21_215 = -(OMcp21_114*RLcp21_315-OMcp21_314*RLcp21_115);
ORcp21_315 = OMcp21_114*RLcp21_215-OMcp21_214*RLcp21_115;
OPcp21_115 = OPcp21_114+ROcp21_714*qdd[15]+qd[15]*(OMcp21_214*ROcp21_914-OMcp21_314*ROcp21_814);
OPcp21_215 = OPcp21_214+ROcp21_814*qdd[15]-qd[15]*(OMcp21_114*ROcp21_914-OMcp21_314*ROcp21_714);
OPcp21_315 = OPcp21_314+ROcp21_914*qdd[15]+qd[15]*(OMcp21_114*ROcp21_814-OMcp21_214*ROcp21_714);
RLcp21_151 = ROcp21_115*s->dpt[1][27]+ROcp21_415*s->dpt[2][27]+ROcp21_714*s->dpt[3][27];
RLcp21_251 = ROcp21_215*s->dpt[1][27]+ROcp21_515*s->dpt[2][27]+ROcp21_814*s->dpt[3][27];
RLcp21_351 = ROcp21_315*s->dpt[1][27]+ROcp21_615*s->dpt[2][27]+ROcp21_914*s->dpt[3][27];
POcp21_151 = RLcp21_113+RLcp21_114+RLcp21_115+RLcp21_151+q[1];
POcp21_251 = RLcp21_213+RLcp21_214+RLcp21_215+RLcp21_251+q[2];
POcp21_351 = RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_351+q[3];
JTcp21_251_4 = -(RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_351);
JTcp21_351_4 = RLcp21_213+RLcp21_214+RLcp21_215+RLcp21_251;
JTcp21_151_5 = C4*(RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_351)-S4*(RLcp21_213+RLcp21_214)-S4*(RLcp21_215+RLcp21_251);
JTcp21_251_5 = S4*(RLcp21_113+RLcp21_114+RLcp21_115+RLcp21_151);
JTcp21_351_5 = -C4*(RLcp21_113+RLcp21_114+RLcp21_115+RLcp21_151);
JTcp21_151_6 = ROcp21_85*(RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_351)-ROcp21_95*(RLcp21_213+RLcp21_214)-ROcp21_95*(
 RLcp21_215+RLcp21_251);
JTcp21_251_6 = RLcp21_151*ROcp21_95-RLcp21_315*S5-RLcp21_351*S5+ROcp21_95*(RLcp21_113+RLcp21_114+RLcp21_115)-S5*(
 RLcp21_313+RLcp21_314);
JTcp21_351_6 = RLcp21_215*S5-ROcp21_85*(RLcp21_113+RLcp21_114+RLcp21_115)+S5*(RLcp21_213+RLcp21_214)-RLcp21_151*
 ROcp21_85+RLcp21_251*S5;
JTcp21_151_7 = ROcp21_56*(RLcp21_314+RLcp21_315)-ROcp21_66*(RLcp21_214+RLcp21_215)-RLcp21_251*ROcp21_66+RLcp21_351*
 ROcp21_56;
JTcp21_251_7 = RLcp21_151*ROcp21_66-RLcp21_351*ROcp21_46-ROcp21_46*(RLcp21_314+RLcp21_315)+ROcp21_66*(RLcp21_114+
 RLcp21_115);
JTcp21_351_7 = ROcp21_46*(RLcp21_214+RLcp21_215)-ROcp21_56*(RLcp21_114+RLcp21_115)-RLcp21_151*ROcp21_56+RLcp21_251*
 ROcp21_46;
JTcp21_151_8 = ROcp21_213*(RLcp21_315+RLcp21_351)-ROcp21_313*(RLcp21_215+RLcp21_251);
JTcp21_251_8 = -(ROcp21_113*(RLcp21_315+RLcp21_351)-ROcp21_313*(RLcp21_115+RLcp21_151));
JTcp21_351_8 = ROcp21_113*(RLcp21_215+RLcp21_251)-ROcp21_213*(RLcp21_115+RLcp21_151);
JTcp21_151_9 = -(RLcp21_251*ROcp21_914-RLcp21_351*ROcp21_814);
JTcp21_251_9 = RLcp21_151*ROcp21_914-RLcp21_351*ROcp21_714;
JTcp21_351_9 = -(RLcp21_151*ROcp21_814-RLcp21_251*ROcp21_714);
ORcp21_151 = OMcp21_215*RLcp21_351-OMcp21_315*RLcp21_251;
ORcp21_251 = -(OMcp21_115*RLcp21_351-OMcp21_315*RLcp21_151);
ORcp21_351 = OMcp21_115*RLcp21_251-OMcp21_215*RLcp21_151;
VIcp21_151 = ORcp21_113+ORcp21_114+ORcp21_115+ORcp21_151+qd[1];
VIcp21_251 = ORcp21_213+ORcp21_214+ORcp21_215+ORcp21_251+qd[2];
VIcp21_351 = ORcp21_313+ORcp21_314+ORcp21_315+ORcp21_351+qd[3];
ACcp21_151 = qdd[1]+OMcp21_213*ORcp21_314+OMcp21_214*ORcp21_315+OMcp21_215*ORcp21_351+OMcp21_26*ORcp21_313-OMcp21_313*
 ORcp21_214-OMcp21_314*ORcp21_215-OMcp21_315*ORcp21_251-OMcp21_36*ORcp21_213+OPcp21_213*RLcp21_314+OPcp21_214*RLcp21_315+
 OPcp21_215*RLcp21_351+OPcp21_26*RLcp21_313-OPcp21_313*RLcp21_214-OPcp21_314*RLcp21_215-OPcp21_315*RLcp21_251-OPcp21_36*
 RLcp21_213;
ACcp21_251 = qdd[2]-OMcp21_113*ORcp21_314-OMcp21_114*ORcp21_315-OMcp21_115*ORcp21_351-OMcp21_16*ORcp21_313+OMcp21_313*
 ORcp21_114+OMcp21_314*ORcp21_115+OMcp21_315*ORcp21_151+OMcp21_36*ORcp21_113-OPcp21_113*RLcp21_314-OPcp21_114*RLcp21_315-
 OPcp21_115*RLcp21_351-OPcp21_16*RLcp21_313+OPcp21_313*RLcp21_114+OPcp21_314*RLcp21_115+OPcp21_315*RLcp21_151+OPcp21_36*
 RLcp21_113;
ACcp21_351 = qdd[3]+OMcp21_113*ORcp21_214+OMcp21_114*ORcp21_215+OMcp21_115*ORcp21_251+OMcp21_16*ORcp21_213-OMcp21_213*
 ORcp21_114-OMcp21_214*ORcp21_115-OMcp21_215*ORcp21_151-OMcp21_26*ORcp21_113+OPcp21_113*RLcp21_214+OPcp21_114*RLcp21_215+
 OPcp21_115*RLcp21_251+OPcp21_16*RLcp21_213-OPcp21_213*RLcp21_114-OPcp21_214*RLcp21_115-OPcp21_215*RLcp21_151-OPcp21_26*
 RLcp21_113;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp21_151;
sens->P[2] = POcp21_251;
sens->P[3] = POcp21_351;
sens->R[1][1] = ROcp21_115;
sens->R[1][2] = ROcp21_215;
sens->R[1][3] = ROcp21_315;
sens->R[2][1] = ROcp21_415;
sens->R[2][2] = ROcp21_515;
sens->R[2][3] = ROcp21_615;
sens->R[3][1] = ROcp21_714;
sens->R[3][2] = ROcp21_814;
sens->R[3][3] = ROcp21_914;
sens->V[1] = VIcp21_151;
sens->V[2] = VIcp21_251;
sens->V[3] = VIcp21_351;
sens->OM[1] = OMcp21_115;
sens->OM[2] = OMcp21_215;
sens->OM[3] = OMcp21_315;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp21_151_5;
sens->J[1][6] = JTcp21_151_6;
sens->J[1][13] = JTcp21_151_7;
sens->J[1][14] = JTcp21_151_8;
sens->J[1][15] = JTcp21_151_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp21_251_4;
sens->J[2][5] = JTcp21_251_5;
sens->J[2][6] = JTcp21_251_6;
sens->J[2][13] = JTcp21_251_7;
sens->J[2][14] = JTcp21_251_8;
sens->J[2][15] = JTcp21_251_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp21_351_4;
sens->J[3][5] = JTcp21_351_5;
sens->J[3][6] = JTcp21_351_6;
sens->J[3][13] = JTcp21_351_7;
sens->J[3][14] = JTcp21_351_8;
sens->J[3][15] = JTcp21_351_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp21_46;
sens->J[4][14] = ROcp21_113;
sens->J[4][15] = ROcp21_714;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp21_85;
sens->J[5][13] = ROcp21_56;
sens->J[5][14] = ROcp21_213;
sens->J[5][15] = ROcp21_814;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp21_95;
sens->J[6][13] = ROcp21_66;
sens->J[6][14] = ROcp21_313;
sens->J[6][15] = ROcp21_914;
sens->A[1] = ACcp21_151;
sens->A[2] = ACcp21_251;
sens->A[3] = ACcp21_351;
sens->OMP[1] = OPcp21_115;
sens->OMP[2] = OPcp21_215;
sens->OMP[3] = OPcp21_315;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


ROcp22_25 = S4*S5;
ROcp22_35 = -C4*S5;
ROcp22_85 = -S4*C5;
ROcp22_95 = C4*C5;
ROcp22_16 = C5*C6;
ROcp22_26 = ROcp22_25*C6+C4*S6;
ROcp22_36 = ROcp22_35*C6+S4*S6;
ROcp22_46 = -C5*S6;
ROcp22_56 = -(ROcp22_25*S6-C4*C6);
ROcp22_66 = -(ROcp22_35*S6-S4*C6);
OMcp22_25 = qd[5]*C4;
OMcp22_35 = qd[5]*S4;
OMcp22_16 = qd[4]+qd[6]*S5;
OMcp22_26 = OMcp22_25+ROcp22_85*qd[6];
OMcp22_36 = OMcp22_35+ROcp22_95*qd[6];
OPcp22_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp22_26 = ROcp22_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp22_35*S5-ROcp22_95*qd[4]);
OPcp22_36 = ROcp22_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp22_25*S5-ROcp22_85*qd[4]);

// = = Block_1_0_0_23_0_3 = = 
 
// Sensor Kinematics 


ROcp22_113 = ROcp22_16*C13-S13*S5;
ROcp22_213 = ROcp22_26*C13-ROcp22_85*S13;
ROcp22_313 = ROcp22_36*C13-ROcp22_95*S13;
ROcp22_713 = ROcp22_16*S13+C13*S5;
ROcp22_813 = ROcp22_26*S13+ROcp22_85*C13;
ROcp22_913 = ROcp22_36*S13+ROcp22_95*C13;
ROcp22_414 = ROcp22_46*C14+ROcp22_713*S14;
ROcp22_514 = ROcp22_56*C14+ROcp22_813*S14;
ROcp22_614 = ROcp22_66*C14+ROcp22_913*S14;
ROcp22_714 = -(ROcp22_46*S14-ROcp22_713*C14);
ROcp22_814 = -(ROcp22_56*S14-ROcp22_813*C14);
ROcp22_914 = -(ROcp22_66*S14-ROcp22_913*C14);
ROcp22_115 = ROcp22_113*C15+ROcp22_414*S15;
ROcp22_215 = ROcp22_213*C15+ROcp22_514*S15;
ROcp22_315 = ROcp22_313*C15+ROcp22_614*S15;
ROcp22_415 = -(ROcp22_113*S15-ROcp22_414*C15);
ROcp22_515 = -(ROcp22_213*S15-ROcp22_514*C15);
ROcp22_615 = -(ROcp22_313*S15-ROcp22_614*C15);
ROcp22_116 = ROcp22_115*C16-ROcp22_714*S16;
ROcp22_216 = ROcp22_215*C16-ROcp22_814*S16;
ROcp22_316 = ROcp22_315*C16-ROcp22_914*S16;
ROcp22_716 = ROcp22_115*S16+ROcp22_714*C16;
ROcp22_816 = ROcp22_215*S16+ROcp22_814*C16;
ROcp22_916 = ROcp22_315*S16+ROcp22_914*C16;
RLcp22_113 = s->dpt[1][2]*ROcp22_16+s->dpt[3][2]*S5+ROcp22_46*s->dpt[2][2];
RLcp22_213 = s->dpt[1][2]*ROcp22_26+s->dpt[3][2]*ROcp22_85+ROcp22_56*s->dpt[2][2];
RLcp22_313 = s->dpt[1][2]*ROcp22_36+s->dpt[3][2]*ROcp22_95+ROcp22_66*s->dpt[2][2];
OMcp22_113 = OMcp22_16+ROcp22_46*qd[13];
OMcp22_213 = OMcp22_26+ROcp22_56*qd[13];
OMcp22_313 = OMcp22_36+ROcp22_66*qd[13];
ORcp22_113 = OMcp22_26*RLcp22_313-OMcp22_36*RLcp22_213;
ORcp22_213 = -(OMcp22_16*RLcp22_313-OMcp22_36*RLcp22_113);
ORcp22_313 = OMcp22_16*RLcp22_213-OMcp22_26*RLcp22_113;
OPcp22_113 = OPcp22_16+ROcp22_46*qdd[13]+qd[13]*(OMcp22_26*ROcp22_66-OMcp22_36*ROcp22_56);
OPcp22_213 = OPcp22_26+ROcp22_56*qdd[13]-qd[13]*(OMcp22_16*ROcp22_66-OMcp22_36*ROcp22_46);
OPcp22_313 = OPcp22_36+ROcp22_66*qdd[13]+qd[13]*(OMcp22_16*ROcp22_56-OMcp22_26*ROcp22_46);
RLcp22_114 = s->dpt[1][22]*ROcp22_113+s->dpt[3][22]*ROcp22_713+ROcp22_46*s->dpt[2][22];
RLcp22_214 = s->dpt[1][22]*ROcp22_213+s->dpt[3][22]*ROcp22_813+ROcp22_56*s->dpt[2][22];
RLcp22_314 = s->dpt[1][22]*ROcp22_313+s->dpt[3][22]*ROcp22_913+ROcp22_66*s->dpt[2][22];
OMcp22_114 = OMcp22_113+ROcp22_113*qd[14];
OMcp22_214 = OMcp22_213+ROcp22_213*qd[14];
OMcp22_314 = OMcp22_313+ROcp22_313*qd[14];
ORcp22_114 = OMcp22_213*RLcp22_314-OMcp22_313*RLcp22_214;
ORcp22_214 = -(OMcp22_113*RLcp22_314-OMcp22_313*RLcp22_114);
ORcp22_314 = OMcp22_113*RLcp22_214-OMcp22_213*RLcp22_114;
OPcp22_114 = OPcp22_113+ROcp22_113*qdd[14]+qd[14]*(OMcp22_213*ROcp22_313-OMcp22_313*ROcp22_213);
OPcp22_214 = OPcp22_213+ROcp22_213*qdd[14]-qd[14]*(OMcp22_113*ROcp22_313-OMcp22_313*ROcp22_113);
OPcp22_314 = OPcp22_313+ROcp22_313*qdd[14]+qd[14]*(OMcp22_113*ROcp22_213-OMcp22_213*ROcp22_113);
RLcp22_115 = s->dpt[1][24]*ROcp22_113+s->dpt[2][24]*ROcp22_414+ROcp22_714*s->dpt[3][24];
RLcp22_215 = s->dpt[1][24]*ROcp22_213+s->dpt[2][24]*ROcp22_514+ROcp22_814*s->dpt[3][24];
RLcp22_315 = s->dpt[1][24]*ROcp22_313+s->dpt[2][24]*ROcp22_614+ROcp22_914*s->dpt[3][24];
OMcp22_115 = OMcp22_114+ROcp22_714*qd[15];
OMcp22_215 = OMcp22_214+ROcp22_814*qd[15];
OMcp22_315 = OMcp22_314+ROcp22_914*qd[15];
ORcp22_115 = OMcp22_214*RLcp22_315-OMcp22_314*RLcp22_215;
ORcp22_215 = -(OMcp22_114*RLcp22_315-OMcp22_314*RLcp22_115);
ORcp22_315 = OMcp22_114*RLcp22_215-OMcp22_214*RLcp22_115;
OPcp22_115 = OPcp22_114+ROcp22_714*qdd[15]+qd[15]*(OMcp22_214*ROcp22_914-OMcp22_314*ROcp22_814);
OPcp22_215 = OPcp22_214+ROcp22_814*qdd[15]-qd[15]*(OMcp22_114*ROcp22_914-OMcp22_314*ROcp22_714);
OPcp22_315 = OPcp22_314+ROcp22_914*qdd[15]+qd[15]*(OMcp22_114*ROcp22_814-OMcp22_214*ROcp22_714);
RLcp22_116 = s->dpt[1][26]*ROcp22_115+s->dpt[2][26]*ROcp22_415+ROcp22_714*s->dpt[3][26];
RLcp22_216 = s->dpt[1][26]*ROcp22_215+s->dpt[2][26]*ROcp22_515+ROcp22_814*s->dpt[3][26];
RLcp22_316 = s->dpt[1][26]*ROcp22_315+s->dpt[2][26]*ROcp22_615+ROcp22_914*s->dpt[3][26];
OMcp22_116 = OMcp22_115+ROcp22_415*qd[16];
OMcp22_216 = OMcp22_215+ROcp22_515*qd[16];
OMcp22_316 = OMcp22_315+ROcp22_615*qd[16];
ORcp22_116 = OMcp22_215*RLcp22_316-OMcp22_315*RLcp22_216;
ORcp22_216 = -(OMcp22_115*RLcp22_316-OMcp22_315*RLcp22_116);
ORcp22_316 = OMcp22_115*RLcp22_216-OMcp22_215*RLcp22_116;
OPcp22_116 = OPcp22_115+ROcp22_415*qdd[16]+qd[16]*(OMcp22_215*ROcp22_615-OMcp22_315*ROcp22_515);
OPcp22_216 = OPcp22_215+ROcp22_515*qdd[16]-qd[16]*(OMcp22_115*ROcp22_615-OMcp22_315*ROcp22_415);
OPcp22_316 = OPcp22_315+ROcp22_615*qdd[16]+qd[16]*(OMcp22_115*ROcp22_515-OMcp22_215*ROcp22_415);
RLcp22_152 = s->dpt[1][28]*ROcp22_116+s->dpt[2][28]*ROcp22_415+ROcp22_716*s->dpt[3][28];
RLcp22_252 = s->dpt[1][28]*ROcp22_216+s->dpt[2][28]*ROcp22_515+ROcp22_816*s->dpt[3][28];
RLcp22_352 = s->dpt[1][28]*ROcp22_316+s->dpt[2][28]*ROcp22_615+ROcp22_916*s->dpt[3][28];
POcp22_152 = RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_152+q[1];
POcp22_252 = RLcp22_213+RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_252+q[2];
POcp22_352 = RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_352+q[3];
JTcp22_252_4 = -(RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_352);
JTcp22_352_4 = RLcp22_213+RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_252;
JTcp22_152_5 = C4*(RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316)-S4*(RLcp22_213+RLcp22_214)-S4*(RLcp22_215+RLcp22_216)-
 RLcp22_252*S4+RLcp22_352*C4;
JTcp22_252_5 = S4*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_152);
JTcp22_352_5 = -C4*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_152);
JTcp22_152_6 = ROcp22_85*(RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316)-ROcp22_95*(RLcp22_213+RLcp22_214)-ROcp22_95*(
 RLcp22_215+RLcp22_216)-RLcp22_252*ROcp22_95+RLcp22_352*ROcp22_85;
JTcp22_252_6 = -(RLcp22_352*S5-ROcp22_95*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_152)+S5*(RLcp22_313+
 RLcp22_314)+S5*(RLcp22_315+RLcp22_316));
JTcp22_352_6 = RLcp22_252*S5-ROcp22_85*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_152)+S5*(RLcp22_213+
 RLcp22_214)+S5*(RLcp22_215+RLcp22_216);
JTcp22_152_7 = ROcp22_56*(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_352)-ROcp22_66*(RLcp22_214+RLcp22_215)-ROcp22_66*(
 RLcp22_216+RLcp22_252);
JTcp22_252_7 = -(ROcp22_46*(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_352)-ROcp22_66*(RLcp22_114+RLcp22_115)-ROcp22_66*(
 RLcp22_116+RLcp22_152));
JTcp22_352_7 = ROcp22_46*(RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_252)-ROcp22_56*(RLcp22_114+RLcp22_115)-ROcp22_56*(
 RLcp22_116+RLcp22_152);
JTcp22_152_8 = ROcp22_213*(RLcp22_315+RLcp22_316)-ROcp22_313*(RLcp22_215+RLcp22_216)-RLcp22_252*ROcp22_313+RLcp22_352*
 ROcp22_213;
JTcp22_252_8 = RLcp22_152*ROcp22_313-RLcp22_352*ROcp22_113-ROcp22_113*(RLcp22_315+RLcp22_316)+ROcp22_313*(RLcp22_115+
 RLcp22_116);
JTcp22_352_8 = ROcp22_113*(RLcp22_215+RLcp22_216)-ROcp22_213*(RLcp22_115+RLcp22_116)-RLcp22_152*ROcp22_213+RLcp22_252*
 ROcp22_113;
JTcp22_152_9 = ROcp22_814*(RLcp22_316+RLcp22_352)-ROcp22_914*(RLcp22_216+RLcp22_252);
JTcp22_252_9 = -(ROcp22_714*(RLcp22_316+RLcp22_352)-ROcp22_914*(RLcp22_116+RLcp22_152));
JTcp22_352_9 = ROcp22_714*(RLcp22_216+RLcp22_252)-ROcp22_814*(RLcp22_116+RLcp22_152);
JTcp22_152_10 = -(RLcp22_252*ROcp22_615-RLcp22_352*ROcp22_515);
JTcp22_252_10 = RLcp22_152*ROcp22_615-RLcp22_352*ROcp22_415;
JTcp22_352_10 = -(RLcp22_152*ROcp22_515-RLcp22_252*ROcp22_415);
ORcp22_152 = OMcp22_216*RLcp22_352-OMcp22_316*RLcp22_252;
ORcp22_252 = -(OMcp22_116*RLcp22_352-OMcp22_316*RLcp22_152);
ORcp22_352 = OMcp22_116*RLcp22_252-OMcp22_216*RLcp22_152;
VIcp22_152 = ORcp22_113+ORcp22_114+ORcp22_115+ORcp22_116+ORcp22_152+qd[1];
VIcp22_252 = ORcp22_213+ORcp22_214+ORcp22_215+ORcp22_216+ORcp22_252+qd[2];
VIcp22_352 = ORcp22_313+ORcp22_314+ORcp22_315+ORcp22_316+ORcp22_352+qd[3];
ACcp22_152 = qdd[1]+OMcp22_213*ORcp22_314+OMcp22_214*ORcp22_315+OMcp22_215*ORcp22_316+OMcp22_216*ORcp22_352+OMcp22_26*
 ORcp22_313-OMcp22_313*ORcp22_214-OMcp22_314*ORcp22_215-OMcp22_315*ORcp22_216-OMcp22_316*ORcp22_252-OMcp22_36*ORcp22_213+
 OPcp22_213*RLcp22_314+OPcp22_214*RLcp22_315+OPcp22_215*RLcp22_316+OPcp22_216*RLcp22_352+OPcp22_26*RLcp22_313-OPcp22_313*
 RLcp22_214-OPcp22_314*RLcp22_215-OPcp22_315*RLcp22_216-OPcp22_316*RLcp22_252-OPcp22_36*RLcp22_213;
ACcp22_252 = qdd[2]-OMcp22_113*ORcp22_314-OMcp22_114*ORcp22_315-OMcp22_115*ORcp22_316-OMcp22_116*ORcp22_352-OMcp22_16*
 ORcp22_313+OMcp22_313*ORcp22_114+OMcp22_314*ORcp22_115+OMcp22_315*ORcp22_116+OMcp22_316*ORcp22_152+OMcp22_36*ORcp22_113-
 OPcp22_113*RLcp22_314-OPcp22_114*RLcp22_315-OPcp22_115*RLcp22_316-OPcp22_116*RLcp22_352-OPcp22_16*RLcp22_313+OPcp22_313*
 RLcp22_114+OPcp22_314*RLcp22_115+OPcp22_315*RLcp22_116+OPcp22_316*RLcp22_152+OPcp22_36*RLcp22_113;
ACcp22_352 = qdd[3]+OMcp22_113*ORcp22_214+OMcp22_114*ORcp22_215+OMcp22_115*ORcp22_216+OMcp22_116*ORcp22_252+OMcp22_16*
 ORcp22_213-OMcp22_213*ORcp22_114-OMcp22_214*ORcp22_115-OMcp22_215*ORcp22_116-OMcp22_216*ORcp22_152-OMcp22_26*ORcp22_113+
 OPcp22_113*RLcp22_214+OPcp22_114*RLcp22_215+OPcp22_115*RLcp22_216+OPcp22_116*RLcp22_252+OPcp22_16*RLcp22_213-OPcp22_213*
 RLcp22_114-OPcp22_214*RLcp22_115-OPcp22_215*RLcp22_116-OPcp22_216*RLcp22_152-OPcp22_26*RLcp22_113;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp22_152;
sens->P[2] = POcp22_252;
sens->P[3] = POcp22_352;
sens->R[1][1] = ROcp22_116;
sens->R[1][2] = ROcp22_216;
sens->R[1][3] = ROcp22_316;
sens->R[2][1] = ROcp22_415;
sens->R[2][2] = ROcp22_515;
sens->R[2][3] = ROcp22_615;
sens->R[3][1] = ROcp22_716;
sens->R[3][2] = ROcp22_816;
sens->R[3][3] = ROcp22_916;
sens->V[1] = VIcp22_152;
sens->V[2] = VIcp22_252;
sens->V[3] = VIcp22_352;
sens->OM[1] = OMcp22_116;
sens->OM[2] = OMcp22_216;
sens->OM[3] = OMcp22_316;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp22_152_5;
sens->J[1][6] = JTcp22_152_6;
sens->J[1][13] = JTcp22_152_7;
sens->J[1][14] = JTcp22_152_8;
sens->J[1][15] = JTcp22_152_9;
sens->J[1][16] = JTcp22_152_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp22_252_4;
sens->J[2][5] = JTcp22_252_5;
sens->J[2][6] = JTcp22_252_6;
sens->J[2][13] = JTcp22_252_7;
sens->J[2][14] = JTcp22_252_8;
sens->J[2][15] = JTcp22_252_9;
sens->J[2][16] = JTcp22_252_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp22_352_4;
sens->J[3][5] = JTcp22_352_5;
sens->J[3][6] = JTcp22_352_6;
sens->J[3][13] = JTcp22_352_7;
sens->J[3][14] = JTcp22_352_8;
sens->J[3][15] = JTcp22_352_9;
sens->J[3][16] = JTcp22_352_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp22_46;
sens->J[4][14] = ROcp22_113;
sens->J[4][15] = ROcp22_714;
sens->J[4][16] = ROcp22_415;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp22_85;
sens->J[5][13] = ROcp22_56;
sens->J[5][14] = ROcp22_213;
sens->J[5][15] = ROcp22_814;
sens->J[5][16] = ROcp22_515;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp22_95;
sens->J[6][13] = ROcp22_66;
sens->J[6][14] = ROcp22_313;
sens->J[6][15] = ROcp22_914;
sens->J[6][16] = ROcp22_615;
sens->A[1] = ACcp22_152;
sens->A[2] = ACcp22_252;
sens->A[3] = ACcp22_352;
sens->OMP[1] = OPcp22_116;
sens->OMP[2] = OPcp22_216;
sens->OMP[3] = OPcp22_316;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


ROcp23_25 = S4*S5;
ROcp23_35 = -C4*S5;
ROcp23_85 = -S4*C5;
ROcp23_95 = C4*C5;
ROcp23_16 = C5*C6;
ROcp23_26 = ROcp23_25*C6+C4*S6;
ROcp23_36 = ROcp23_35*C6+S4*S6;
ROcp23_46 = -C5*S6;
ROcp23_56 = -(ROcp23_25*S6-C4*C6);
ROcp23_66 = -(ROcp23_35*S6-S4*C6);
OMcp23_25 = qd[5]*C4;
OMcp23_35 = qd[5]*S4;
OMcp23_16 = qd[4]+qd[6]*S5;
OMcp23_26 = OMcp23_25+ROcp23_85*qd[6];
OMcp23_36 = OMcp23_35+ROcp23_95*qd[6];
OPcp23_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp23_26 = ROcp23_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp23_35*S5-ROcp23_95*qd[4]);
OPcp23_36 = ROcp23_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp23_25*S5-ROcp23_85*qd[4]);

// = = Block_1_0_0_24_0_3 = = 
 
// Sensor Kinematics 


ROcp23_113 = ROcp23_16*C13-S13*S5;
ROcp23_213 = ROcp23_26*C13-ROcp23_85*S13;
ROcp23_313 = ROcp23_36*C13-ROcp23_95*S13;
ROcp23_713 = ROcp23_16*S13+C13*S5;
ROcp23_813 = ROcp23_26*S13+ROcp23_85*C13;
ROcp23_913 = ROcp23_36*S13+ROcp23_95*C13;
ROcp23_414 = ROcp23_46*C14+ROcp23_713*S14;
ROcp23_514 = ROcp23_56*C14+ROcp23_813*S14;
ROcp23_614 = ROcp23_66*C14+ROcp23_913*S14;
ROcp23_714 = -(ROcp23_46*S14-ROcp23_713*C14);
ROcp23_814 = -(ROcp23_56*S14-ROcp23_813*C14);
ROcp23_914 = -(ROcp23_66*S14-ROcp23_913*C14);
ROcp23_115 = ROcp23_113*C15+ROcp23_414*S15;
ROcp23_215 = ROcp23_213*C15+ROcp23_514*S15;
ROcp23_315 = ROcp23_313*C15+ROcp23_614*S15;
ROcp23_415 = -(ROcp23_113*S15-ROcp23_414*C15);
ROcp23_515 = -(ROcp23_213*S15-ROcp23_514*C15);
ROcp23_615 = -(ROcp23_313*S15-ROcp23_614*C15);
ROcp23_116 = ROcp23_115*C16-ROcp23_714*S16;
ROcp23_216 = ROcp23_215*C16-ROcp23_814*S16;
ROcp23_316 = ROcp23_315*C16-ROcp23_914*S16;
ROcp23_716 = ROcp23_115*S16+ROcp23_714*C16;
ROcp23_816 = ROcp23_215*S16+ROcp23_814*C16;
ROcp23_916 = ROcp23_315*S16+ROcp23_914*C16;
RLcp23_113 = s->dpt[1][2]*ROcp23_16+s->dpt[3][2]*S5+ROcp23_46*s->dpt[2][2];
RLcp23_213 = s->dpt[1][2]*ROcp23_26+s->dpt[3][2]*ROcp23_85+ROcp23_56*s->dpt[2][2];
RLcp23_313 = s->dpt[1][2]*ROcp23_36+s->dpt[3][2]*ROcp23_95+ROcp23_66*s->dpt[2][2];
OMcp23_113 = OMcp23_16+ROcp23_46*qd[13];
OMcp23_213 = OMcp23_26+ROcp23_56*qd[13];
OMcp23_313 = OMcp23_36+ROcp23_66*qd[13];
ORcp23_113 = OMcp23_26*RLcp23_313-OMcp23_36*RLcp23_213;
ORcp23_213 = -(OMcp23_16*RLcp23_313-OMcp23_36*RLcp23_113);
ORcp23_313 = OMcp23_16*RLcp23_213-OMcp23_26*RLcp23_113;
OPcp23_113 = OPcp23_16+ROcp23_46*qdd[13]+qd[13]*(OMcp23_26*ROcp23_66-OMcp23_36*ROcp23_56);
OPcp23_213 = OPcp23_26+ROcp23_56*qdd[13]-qd[13]*(OMcp23_16*ROcp23_66-OMcp23_36*ROcp23_46);
OPcp23_313 = OPcp23_36+ROcp23_66*qdd[13]+qd[13]*(OMcp23_16*ROcp23_56-OMcp23_26*ROcp23_46);
RLcp23_114 = s->dpt[1][22]*ROcp23_113+s->dpt[3][22]*ROcp23_713+ROcp23_46*s->dpt[2][22];
RLcp23_214 = s->dpt[1][22]*ROcp23_213+s->dpt[3][22]*ROcp23_813+ROcp23_56*s->dpt[2][22];
RLcp23_314 = s->dpt[1][22]*ROcp23_313+s->dpt[3][22]*ROcp23_913+ROcp23_66*s->dpt[2][22];
OMcp23_114 = OMcp23_113+ROcp23_113*qd[14];
OMcp23_214 = OMcp23_213+ROcp23_213*qd[14];
OMcp23_314 = OMcp23_313+ROcp23_313*qd[14];
ORcp23_114 = OMcp23_213*RLcp23_314-OMcp23_313*RLcp23_214;
ORcp23_214 = -(OMcp23_113*RLcp23_314-OMcp23_313*RLcp23_114);
ORcp23_314 = OMcp23_113*RLcp23_214-OMcp23_213*RLcp23_114;
OPcp23_114 = OPcp23_113+ROcp23_113*qdd[14]+qd[14]*(OMcp23_213*ROcp23_313-OMcp23_313*ROcp23_213);
OPcp23_214 = OPcp23_213+ROcp23_213*qdd[14]-qd[14]*(OMcp23_113*ROcp23_313-OMcp23_313*ROcp23_113);
OPcp23_314 = OPcp23_313+ROcp23_313*qdd[14]+qd[14]*(OMcp23_113*ROcp23_213-OMcp23_213*ROcp23_113);
RLcp23_115 = s->dpt[1][24]*ROcp23_113+s->dpt[2][24]*ROcp23_414+ROcp23_714*s->dpt[3][24];
RLcp23_215 = s->dpt[1][24]*ROcp23_213+s->dpt[2][24]*ROcp23_514+ROcp23_814*s->dpt[3][24];
RLcp23_315 = s->dpt[1][24]*ROcp23_313+s->dpt[2][24]*ROcp23_614+ROcp23_914*s->dpt[3][24];
OMcp23_115 = OMcp23_114+ROcp23_714*qd[15];
OMcp23_215 = OMcp23_214+ROcp23_814*qd[15];
OMcp23_315 = OMcp23_314+ROcp23_914*qd[15];
ORcp23_115 = OMcp23_214*RLcp23_315-OMcp23_314*RLcp23_215;
ORcp23_215 = -(OMcp23_114*RLcp23_315-OMcp23_314*RLcp23_115);
ORcp23_315 = OMcp23_114*RLcp23_215-OMcp23_214*RLcp23_115;
OPcp23_115 = OPcp23_114+ROcp23_714*qdd[15]+qd[15]*(OMcp23_214*ROcp23_914-OMcp23_314*ROcp23_814);
OPcp23_215 = OPcp23_214+ROcp23_814*qdd[15]-qd[15]*(OMcp23_114*ROcp23_914-OMcp23_314*ROcp23_714);
OPcp23_315 = OPcp23_314+ROcp23_914*qdd[15]+qd[15]*(OMcp23_114*ROcp23_814-OMcp23_214*ROcp23_714);
RLcp23_116 = s->dpt[1][26]*ROcp23_115+s->dpt[2][26]*ROcp23_415+ROcp23_714*s->dpt[3][26];
RLcp23_216 = s->dpt[1][26]*ROcp23_215+s->dpt[2][26]*ROcp23_515+ROcp23_814*s->dpt[3][26];
RLcp23_316 = s->dpt[1][26]*ROcp23_315+s->dpt[2][26]*ROcp23_615+ROcp23_914*s->dpt[3][26];
OMcp23_116 = OMcp23_115+ROcp23_415*qd[16];
OMcp23_216 = OMcp23_215+ROcp23_515*qd[16];
OMcp23_316 = OMcp23_315+ROcp23_615*qd[16];
ORcp23_116 = OMcp23_215*RLcp23_316-OMcp23_315*RLcp23_216;
ORcp23_216 = -(OMcp23_115*RLcp23_316-OMcp23_315*RLcp23_116);
ORcp23_316 = OMcp23_115*RLcp23_216-OMcp23_215*RLcp23_116;
OPcp23_116 = OPcp23_115+ROcp23_415*qdd[16]+qd[16]*(OMcp23_215*ROcp23_615-OMcp23_315*ROcp23_515);
OPcp23_216 = OPcp23_215+ROcp23_515*qdd[16]-qd[16]*(OMcp23_115*ROcp23_615-OMcp23_315*ROcp23_415);
OPcp23_316 = OPcp23_315+ROcp23_615*qdd[16]+qd[16]*(OMcp23_115*ROcp23_515-OMcp23_215*ROcp23_415);
RLcp23_153 = ROcp23_116*s->dpt[1][29]+ROcp23_415*s->dpt[2][29]+ROcp23_716*s->dpt[3][29];
RLcp23_253 = ROcp23_216*s->dpt[1][29]+ROcp23_515*s->dpt[2][29]+ROcp23_816*s->dpt[3][29];
RLcp23_353 = ROcp23_316*s->dpt[1][29]+ROcp23_615*s->dpt[2][29]+ROcp23_916*s->dpt[3][29];
POcp23_153 = RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_153+q[1];
POcp23_253 = RLcp23_213+RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_253+q[2];
POcp23_353 = RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_353+q[3];
JTcp23_253_4 = -(RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_353);
JTcp23_353_4 = RLcp23_213+RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_253;
JTcp23_153_5 = C4*(RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316)-S4*(RLcp23_213+RLcp23_214)-S4*(RLcp23_215+RLcp23_216)-
 RLcp23_253*S4+RLcp23_353*C4;
JTcp23_253_5 = S4*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_153);
JTcp23_353_5 = -C4*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_153);
JTcp23_153_6 = ROcp23_85*(RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316)-ROcp23_95*(RLcp23_213+RLcp23_214)-ROcp23_95*(
 RLcp23_215+RLcp23_216)-RLcp23_253*ROcp23_95+RLcp23_353*ROcp23_85;
JTcp23_253_6 = -(RLcp23_353*S5-ROcp23_95*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_153)+S5*(RLcp23_313+
 RLcp23_314)+S5*(RLcp23_315+RLcp23_316));
JTcp23_353_6 = RLcp23_253*S5-ROcp23_85*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_153)+S5*(RLcp23_213+
 RLcp23_214)+S5*(RLcp23_215+RLcp23_216);
JTcp23_153_7 = ROcp23_56*(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_353)-ROcp23_66*(RLcp23_214+RLcp23_215)-ROcp23_66*(
 RLcp23_216+RLcp23_253);
JTcp23_253_7 = -(ROcp23_46*(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_353)-ROcp23_66*(RLcp23_114+RLcp23_115)-ROcp23_66*(
 RLcp23_116+RLcp23_153));
JTcp23_353_7 = ROcp23_46*(RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_253)-ROcp23_56*(RLcp23_114+RLcp23_115)-ROcp23_56*(
 RLcp23_116+RLcp23_153);
JTcp23_153_8 = ROcp23_213*(RLcp23_315+RLcp23_316)-ROcp23_313*(RLcp23_215+RLcp23_216)-RLcp23_253*ROcp23_313+RLcp23_353*
 ROcp23_213;
JTcp23_253_8 = RLcp23_153*ROcp23_313-RLcp23_353*ROcp23_113-ROcp23_113*(RLcp23_315+RLcp23_316)+ROcp23_313*(RLcp23_115+
 RLcp23_116);
JTcp23_353_8 = ROcp23_113*(RLcp23_215+RLcp23_216)-ROcp23_213*(RLcp23_115+RLcp23_116)-RLcp23_153*ROcp23_213+RLcp23_253*
 ROcp23_113;
JTcp23_153_9 = ROcp23_814*(RLcp23_316+RLcp23_353)-ROcp23_914*(RLcp23_216+RLcp23_253);
JTcp23_253_9 = -(ROcp23_714*(RLcp23_316+RLcp23_353)-ROcp23_914*(RLcp23_116+RLcp23_153));
JTcp23_353_9 = ROcp23_714*(RLcp23_216+RLcp23_253)-ROcp23_814*(RLcp23_116+RLcp23_153);
JTcp23_153_10 = -(RLcp23_253*ROcp23_615-RLcp23_353*ROcp23_515);
JTcp23_253_10 = RLcp23_153*ROcp23_615-RLcp23_353*ROcp23_415;
JTcp23_353_10 = -(RLcp23_153*ROcp23_515-RLcp23_253*ROcp23_415);
ORcp23_153 = OMcp23_216*RLcp23_353-OMcp23_316*RLcp23_253;
ORcp23_253 = -(OMcp23_116*RLcp23_353-OMcp23_316*RLcp23_153);
ORcp23_353 = OMcp23_116*RLcp23_253-OMcp23_216*RLcp23_153;
VIcp23_153 = ORcp23_113+ORcp23_114+ORcp23_115+ORcp23_116+ORcp23_153+qd[1];
VIcp23_253 = ORcp23_213+ORcp23_214+ORcp23_215+ORcp23_216+ORcp23_253+qd[2];
VIcp23_353 = ORcp23_313+ORcp23_314+ORcp23_315+ORcp23_316+ORcp23_353+qd[3];
ACcp23_153 = qdd[1]+OMcp23_213*ORcp23_314+OMcp23_214*ORcp23_315+OMcp23_215*ORcp23_316+OMcp23_216*ORcp23_353+OMcp23_26*
 ORcp23_313-OMcp23_313*ORcp23_214-OMcp23_314*ORcp23_215-OMcp23_315*ORcp23_216-OMcp23_316*ORcp23_253-OMcp23_36*ORcp23_213+
 OPcp23_213*RLcp23_314+OPcp23_214*RLcp23_315+OPcp23_215*RLcp23_316+OPcp23_216*RLcp23_353+OPcp23_26*RLcp23_313-OPcp23_313*
 RLcp23_214-OPcp23_314*RLcp23_215-OPcp23_315*RLcp23_216-OPcp23_316*RLcp23_253-OPcp23_36*RLcp23_213;
ACcp23_253 = qdd[2]-OMcp23_113*ORcp23_314-OMcp23_114*ORcp23_315-OMcp23_115*ORcp23_316-OMcp23_116*ORcp23_353-OMcp23_16*
 ORcp23_313+OMcp23_313*ORcp23_114+OMcp23_314*ORcp23_115+OMcp23_315*ORcp23_116+OMcp23_316*ORcp23_153+OMcp23_36*ORcp23_113-
 OPcp23_113*RLcp23_314-OPcp23_114*RLcp23_315-OPcp23_115*RLcp23_316-OPcp23_116*RLcp23_353-OPcp23_16*RLcp23_313+OPcp23_313*
 RLcp23_114+OPcp23_314*RLcp23_115+OPcp23_315*RLcp23_116+OPcp23_316*RLcp23_153+OPcp23_36*RLcp23_113;
ACcp23_353 = qdd[3]+OMcp23_113*ORcp23_214+OMcp23_114*ORcp23_215+OMcp23_115*ORcp23_216+OMcp23_116*ORcp23_253+OMcp23_16*
 ORcp23_213-OMcp23_213*ORcp23_114-OMcp23_214*ORcp23_115-OMcp23_215*ORcp23_116-OMcp23_216*ORcp23_153-OMcp23_26*ORcp23_113+
 OPcp23_113*RLcp23_214+OPcp23_114*RLcp23_215+OPcp23_115*RLcp23_216+OPcp23_116*RLcp23_253+OPcp23_16*RLcp23_213-OPcp23_213*
 RLcp23_114-OPcp23_214*RLcp23_115-OPcp23_215*RLcp23_116-OPcp23_216*RLcp23_153-OPcp23_26*RLcp23_113;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp23_153;
sens->P[2] = POcp23_253;
sens->P[3] = POcp23_353;
sens->R[1][1] = ROcp23_116;
sens->R[1][2] = ROcp23_216;
sens->R[1][3] = ROcp23_316;
sens->R[2][1] = ROcp23_415;
sens->R[2][2] = ROcp23_515;
sens->R[2][3] = ROcp23_615;
sens->R[3][1] = ROcp23_716;
sens->R[3][2] = ROcp23_816;
sens->R[3][3] = ROcp23_916;
sens->V[1] = VIcp23_153;
sens->V[2] = VIcp23_253;
sens->V[3] = VIcp23_353;
sens->OM[1] = OMcp23_116;
sens->OM[2] = OMcp23_216;
sens->OM[3] = OMcp23_316;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp23_153_5;
sens->J[1][6] = JTcp23_153_6;
sens->J[1][13] = JTcp23_153_7;
sens->J[1][14] = JTcp23_153_8;
sens->J[1][15] = JTcp23_153_9;
sens->J[1][16] = JTcp23_153_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp23_253_4;
sens->J[2][5] = JTcp23_253_5;
sens->J[2][6] = JTcp23_253_6;
sens->J[2][13] = JTcp23_253_7;
sens->J[2][14] = JTcp23_253_8;
sens->J[2][15] = JTcp23_253_9;
sens->J[2][16] = JTcp23_253_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp23_353_4;
sens->J[3][5] = JTcp23_353_5;
sens->J[3][6] = JTcp23_353_6;
sens->J[3][13] = JTcp23_353_7;
sens->J[3][14] = JTcp23_353_8;
sens->J[3][15] = JTcp23_353_9;
sens->J[3][16] = JTcp23_353_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp23_46;
sens->J[4][14] = ROcp23_113;
sens->J[4][15] = ROcp23_714;
sens->J[4][16] = ROcp23_415;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp23_85;
sens->J[5][13] = ROcp23_56;
sens->J[5][14] = ROcp23_213;
sens->J[5][15] = ROcp23_814;
sens->J[5][16] = ROcp23_515;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp23_95;
sens->J[6][13] = ROcp23_66;
sens->J[6][14] = ROcp23_313;
sens->J[6][15] = ROcp23_914;
sens->J[6][16] = ROcp23_615;
sens->A[1] = ACcp23_153;
sens->A[2] = ACcp23_253;
sens->A[3] = ACcp23_353;
sens->OMP[1] = OPcp23_116;
sens->OMP[2] = OPcp23_216;
sens->OMP[3] = OPcp23_316;
 
// 
break;
case 25:
 


// = = Block_1_0_0_25_0_1 = = 
 
// Sensor Kinematics 


ROcp24_25 = S4*S5;
ROcp24_35 = -C4*S5;
ROcp24_85 = -S4*C5;
ROcp24_95 = C4*C5;
ROcp24_16 = C5*C6;
ROcp24_26 = ROcp24_25*C6+C4*S6;
ROcp24_36 = ROcp24_35*C6+S4*S6;
ROcp24_46 = -C5*S6;
ROcp24_56 = -(ROcp24_25*S6-C4*C6);
ROcp24_66 = -(ROcp24_35*S6-S4*C6);
OMcp24_25 = qd[5]*C4;
OMcp24_35 = qd[5]*S4;
OMcp24_16 = qd[4]+qd[6]*S5;
OMcp24_26 = OMcp24_25+ROcp24_85*qd[6];
OMcp24_36 = OMcp24_35+ROcp24_95*qd[6];
OPcp24_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp24_26 = ROcp24_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp24_35*S5-ROcp24_95*qd[4]);
OPcp24_36 = ROcp24_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp24_25*S5-ROcp24_85*qd[4]);

// = = Block_1_0_0_25_0_3 = = 
 
// Sensor Kinematics 


ROcp24_113 = ROcp24_16*C13-S13*S5;
ROcp24_213 = ROcp24_26*C13-ROcp24_85*S13;
ROcp24_313 = ROcp24_36*C13-ROcp24_95*S13;
ROcp24_713 = ROcp24_16*S13+C13*S5;
ROcp24_813 = ROcp24_26*S13+ROcp24_85*C13;
ROcp24_913 = ROcp24_36*S13+ROcp24_95*C13;
ROcp24_414 = ROcp24_46*C14+ROcp24_713*S14;
ROcp24_514 = ROcp24_56*C14+ROcp24_813*S14;
ROcp24_614 = ROcp24_66*C14+ROcp24_913*S14;
ROcp24_714 = -(ROcp24_46*S14-ROcp24_713*C14);
ROcp24_814 = -(ROcp24_56*S14-ROcp24_813*C14);
ROcp24_914 = -(ROcp24_66*S14-ROcp24_913*C14);
ROcp24_115 = ROcp24_113*C15+ROcp24_414*S15;
ROcp24_215 = ROcp24_213*C15+ROcp24_514*S15;
ROcp24_315 = ROcp24_313*C15+ROcp24_614*S15;
ROcp24_415 = -(ROcp24_113*S15-ROcp24_414*C15);
ROcp24_515 = -(ROcp24_213*S15-ROcp24_514*C15);
ROcp24_615 = -(ROcp24_313*S15-ROcp24_614*C15);
ROcp24_116 = ROcp24_115*C16-ROcp24_714*S16;
ROcp24_216 = ROcp24_215*C16-ROcp24_814*S16;
ROcp24_316 = ROcp24_315*C16-ROcp24_914*S16;
ROcp24_716 = ROcp24_115*S16+ROcp24_714*C16;
ROcp24_816 = ROcp24_215*S16+ROcp24_814*C16;
ROcp24_916 = ROcp24_315*S16+ROcp24_914*C16;
ROcp24_417 = ROcp24_415*C17+ROcp24_716*S17;
ROcp24_517 = ROcp24_515*C17+ROcp24_816*S17;
ROcp24_617 = ROcp24_615*C17+ROcp24_916*S17;
ROcp24_717 = -(ROcp24_415*S17-ROcp24_716*C17);
ROcp24_817 = -(ROcp24_515*S17-ROcp24_816*C17);
ROcp24_917 = -(ROcp24_615*S17-ROcp24_916*C17);
RLcp24_113 = s->dpt[1][2]*ROcp24_16+s->dpt[3][2]*S5+ROcp24_46*s->dpt[2][2];
RLcp24_213 = s->dpt[1][2]*ROcp24_26+s->dpt[3][2]*ROcp24_85+ROcp24_56*s->dpt[2][2];
RLcp24_313 = s->dpt[1][2]*ROcp24_36+s->dpt[3][2]*ROcp24_95+ROcp24_66*s->dpt[2][2];
OMcp24_113 = OMcp24_16+ROcp24_46*qd[13];
OMcp24_213 = OMcp24_26+ROcp24_56*qd[13];
OMcp24_313 = OMcp24_36+ROcp24_66*qd[13];
ORcp24_113 = OMcp24_26*RLcp24_313-OMcp24_36*RLcp24_213;
ORcp24_213 = -(OMcp24_16*RLcp24_313-OMcp24_36*RLcp24_113);
ORcp24_313 = OMcp24_16*RLcp24_213-OMcp24_26*RLcp24_113;
OPcp24_113 = OPcp24_16+ROcp24_46*qdd[13]+qd[13]*(OMcp24_26*ROcp24_66-OMcp24_36*ROcp24_56);
OPcp24_213 = OPcp24_26+ROcp24_56*qdd[13]-qd[13]*(OMcp24_16*ROcp24_66-OMcp24_36*ROcp24_46);
OPcp24_313 = OPcp24_36+ROcp24_66*qdd[13]+qd[13]*(OMcp24_16*ROcp24_56-OMcp24_26*ROcp24_46);
RLcp24_114 = s->dpt[1][22]*ROcp24_113+s->dpt[3][22]*ROcp24_713+ROcp24_46*s->dpt[2][22];
RLcp24_214 = s->dpt[1][22]*ROcp24_213+s->dpt[3][22]*ROcp24_813+ROcp24_56*s->dpt[2][22];
RLcp24_314 = s->dpt[1][22]*ROcp24_313+s->dpt[3][22]*ROcp24_913+ROcp24_66*s->dpt[2][22];
OMcp24_114 = OMcp24_113+ROcp24_113*qd[14];
OMcp24_214 = OMcp24_213+ROcp24_213*qd[14];
OMcp24_314 = OMcp24_313+ROcp24_313*qd[14];
ORcp24_114 = OMcp24_213*RLcp24_314-OMcp24_313*RLcp24_214;
ORcp24_214 = -(OMcp24_113*RLcp24_314-OMcp24_313*RLcp24_114);
ORcp24_314 = OMcp24_113*RLcp24_214-OMcp24_213*RLcp24_114;
OPcp24_114 = OPcp24_113+ROcp24_113*qdd[14]+qd[14]*(OMcp24_213*ROcp24_313-OMcp24_313*ROcp24_213);
OPcp24_214 = OPcp24_213+ROcp24_213*qdd[14]-qd[14]*(OMcp24_113*ROcp24_313-OMcp24_313*ROcp24_113);
OPcp24_314 = OPcp24_313+ROcp24_313*qdd[14]+qd[14]*(OMcp24_113*ROcp24_213-OMcp24_213*ROcp24_113);
RLcp24_115 = s->dpt[1][24]*ROcp24_113+s->dpt[2][24]*ROcp24_414+ROcp24_714*s->dpt[3][24];
RLcp24_215 = s->dpt[1][24]*ROcp24_213+s->dpt[2][24]*ROcp24_514+ROcp24_814*s->dpt[3][24];
RLcp24_315 = s->dpt[1][24]*ROcp24_313+s->dpt[2][24]*ROcp24_614+ROcp24_914*s->dpt[3][24];
OMcp24_115 = OMcp24_114+ROcp24_714*qd[15];
OMcp24_215 = OMcp24_214+ROcp24_814*qd[15];
OMcp24_315 = OMcp24_314+ROcp24_914*qd[15];
ORcp24_115 = OMcp24_214*RLcp24_315-OMcp24_314*RLcp24_215;
ORcp24_215 = -(OMcp24_114*RLcp24_315-OMcp24_314*RLcp24_115);
ORcp24_315 = OMcp24_114*RLcp24_215-OMcp24_214*RLcp24_115;
OPcp24_115 = OPcp24_114+ROcp24_714*qdd[15]+qd[15]*(OMcp24_214*ROcp24_914-OMcp24_314*ROcp24_814);
OPcp24_215 = OPcp24_214+ROcp24_814*qdd[15]-qd[15]*(OMcp24_114*ROcp24_914-OMcp24_314*ROcp24_714);
OPcp24_315 = OPcp24_314+ROcp24_914*qdd[15]+qd[15]*(OMcp24_114*ROcp24_814-OMcp24_214*ROcp24_714);
RLcp24_116 = s->dpt[1][26]*ROcp24_115+s->dpt[2][26]*ROcp24_415+ROcp24_714*s->dpt[3][26];
RLcp24_216 = s->dpt[1][26]*ROcp24_215+s->dpt[2][26]*ROcp24_515+ROcp24_814*s->dpt[3][26];
RLcp24_316 = s->dpt[1][26]*ROcp24_315+s->dpt[2][26]*ROcp24_615+ROcp24_914*s->dpt[3][26];
OMcp24_116 = OMcp24_115+ROcp24_415*qd[16];
OMcp24_216 = OMcp24_215+ROcp24_515*qd[16];
OMcp24_316 = OMcp24_315+ROcp24_615*qd[16];
ORcp24_116 = OMcp24_215*RLcp24_316-OMcp24_315*RLcp24_216;
ORcp24_216 = -(OMcp24_115*RLcp24_316-OMcp24_315*RLcp24_116);
ORcp24_316 = OMcp24_115*RLcp24_216-OMcp24_215*RLcp24_116;
OPcp24_116 = OPcp24_115+ROcp24_415*qdd[16]+qd[16]*(OMcp24_215*ROcp24_615-OMcp24_315*ROcp24_515);
OPcp24_216 = OPcp24_215+ROcp24_515*qdd[16]-qd[16]*(OMcp24_115*ROcp24_615-OMcp24_315*ROcp24_415);
OPcp24_316 = OPcp24_315+ROcp24_615*qdd[16]+qd[16]*(OMcp24_115*ROcp24_515-OMcp24_215*ROcp24_415);
RLcp24_117 = s->dpt[1][28]*ROcp24_116+s->dpt[2][28]*ROcp24_415+ROcp24_716*s->dpt[3][28];
RLcp24_217 = s->dpt[1][28]*ROcp24_216+s->dpt[2][28]*ROcp24_515+ROcp24_816*s->dpt[3][28];
RLcp24_317 = s->dpt[1][28]*ROcp24_316+s->dpt[2][28]*ROcp24_615+ROcp24_916*s->dpt[3][28];
OMcp24_117 = OMcp24_116+ROcp24_116*qd[17];
OMcp24_217 = OMcp24_216+ROcp24_216*qd[17];
OMcp24_317 = OMcp24_316+ROcp24_316*qd[17];
ORcp24_117 = OMcp24_216*RLcp24_317-OMcp24_316*RLcp24_217;
ORcp24_217 = -(OMcp24_116*RLcp24_317-OMcp24_316*RLcp24_117);
ORcp24_317 = OMcp24_116*RLcp24_217-OMcp24_216*RLcp24_117;
OPcp24_117 = OPcp24_116+ROcp24_116*qdd[17]+qd[17]*(OMcp24_216*ROcp24_316-OMcp24_316*ROcp24_216);
OPcp24_217 = OPcp24_216+ROcp24_216*qdd[17]-qd[17]*(OMcp24_116*ROcp24_316-OMcp24_316*ROcp24_116);
OPcp24_317 = OPcp24_316+ROcp24_316*qdd[17]+qd[17]*(OMcp24_116*ROcp24_216-OMcp24_216*ROcp24_116);
RLcp24_154 = ROcp24_116*s->dpt[1][31]+ROcp24_417*s->dpt[2][31]+ROcp24_717*s->dpt[3][31];
RLcp24_254 = ROcp24_216*s->dpt[1][31]+ROcp24_517*s->dpt[2][31]+ROcp24_817*s->dpt[3][31];
RLcp24_354 = ROcp24_316*s->dpt[1][31]+ROcp24_617*s->dpt[2][31]+ROcp24_917*s->dpt[3][31];
POcp24_154 = RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_154+q[1];
POcp24_254 = RLcp24_213+RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_254+q[2];
POcp24_354 = RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_354+q[3];
JTcp24_254_4 = -(RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_354);
JTcp24_354_4 = RLcp24_213+RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_254;
JTcp24_154_5 = C4*(RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_354)-S4*(RLcp24_213+RLcp24_214)-S4*(
 RLcp24_215+RLcp24_216)-S4*(RLcp24_217+RLcp24_254);
JTcp24_254_5 = S4*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_154);
JTcp24_354_5 = -C4*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_154);
JTcp24_154_6 = ROcp24_85*(RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_354)-ROcp24_95*(RLcp24_213+
 RLcp24_214)-ROcp24_95*(RLcp24_215+RLcp24_216)-ROcp24_95*(RLcp24_217+RLcp24_254);
JTcp24_254_6 = RLcp24_154*ROcp24_95-RLcp24_317*S5-RLcp24_354*S5+ROcp24_95*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116
 +RLcp24_117)-S5*(RLcp24_313+RLcp24_314)-S5*(RLcp24_315+RLcp24_316);
JTcp24_354_6 = RLcp24_217*S5-ROcp24_85*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117)+S5*(RLcp24_213+
 RLcp24_214)+S5*(RLcp24_215+RLcp24_216)-RLcp24_154*ROcp24_85+RLcp24_254*S5;
JTcp24_154_7 = ROcp24_56*(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317)-ROcp24_66*(RLcp24_214+RLcp24_215)-ROcp24_66*(
 RLcp24_216+RLcp24_217)-RLcp24_254*ROcp24_66+RLcp24_354*ROcp24_56;
JTcp24_254_7 = RLcp24_154*ROcp24_66-RLcp24_354*ROcp24_46-ROcp24_46*(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317)+
 ROcp24_66*(RLcp24_114+RLcp24_115)+ROcp24_66*(RLcp24_116+RLcp24_117);
JTcp24_354_7 = ROcp24_46*(RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217)-ROcp24_56*(RLcp24_114+RLcp24_115)-ROcp24_56*(
 RLcp24_116+RLcp24_117)-RLcp24_154*ROcp24_56+RLcp24_254*ROcp24_46;
JTcp24_154_8 = ROcp24_213*(RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_354)-ROcp24_313*(RLcp24_215+RLcp24_216)-ROcp24_313*
 (RLcp24_217+RLcp24_254);
JTcp24_254_8 = -(ROcp24_113*(RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_354)-ROcp24_313*(RLcp24_115+RLcp24_116)-
 ROcp24_313*(RLcp24_117+RLcp24_154));
JTcp24_354_8 = ROcp24_113*(RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_254)-ROcp24_213*(RLcp24_115+RLcp24_116)-ROcp24_213*
 (RLcp24_117+RLcp24_154);
JTcp24_154_9 = ROcp24_814*(RLcp24_316+RLcp24_317)-ROcp24_914*(RLcp24_216+RLcp24_217)-RLcp24_254*ROcp24_914+RLcp24_354*
 ROcp24_814;
JTcp24_254_9 = RLcp24_154*ROcp24_914-RLcp24_354*ROcp24_714-ROcp24_714*(RLcp24_316+RLcp24_317)+ROcp24_914*(RLcp24_116+
 RLcp24_117);
JTcp24_354_9 = ROcp24_714*(RLcp24_216+RLcp24_217)-ROcp24_814*(RLcp24_116+RLcp24_117)-RLcp24_154*ROcp24_814+RLcp24_254*
 ROcp24_714;
JTcp24_154_10 = ROcp24_515*(RLcp24_317+RLcp24_354)-ROcp24_615*(RLcp24_217+RLcp24_254);
JTcp24_254_10 = -(ROcp24_415*(RLcp24_317+RLcp24_354)-ROcp24_615*(RLcp24_117+RLcp24_154));
JTcp24_354_10 = ROcp24_415*(RLcp24_217+RLcp24_254)-ROcp24_515*(RLcp24_117+RLcp24_154);
JTcp24_154_11 = -(RLcp24_254*ROcp24_316-RLcp24_354*ROcp24_216);
JTcp24_254_11 = RLcp24_154*ROcp24_316-RLcp24_354*ROcp24_116;
JTcp24_354_11 = -(RLcp24_154*ROcp24_216-RLcp24_254*ROcp24_116);
ORcp24_154 = OMcp24_217*RLcp24_354-OMcp24_317*RLcp24_254;
ORcp24_254 = -(OMcp24_117*RLcp24_354-OMcp24_317*RLcp24_154);
ORcp24_354 = OMcp24_117*RLcp24_254-OMcp24_217*RLcp24_154;
VIcp24_154 = ORcp24_113+ORcp24_114+ORcp24_115+ORcp24_116+ORcp24_117+ORcp24_154+qd[1];
VIcp24_254 = ORcp24_213+ORcp24_214+ORcp24_215+ORcp24_216+ORcp24_217+ORcp24_254+qd[2];
VIcp24_354 = ORcp24_313+ORcp24_314+ORcp24_315+ORcp24_316+ORcp24_317+ORcp24_354+qd[3];
ACcp24_154 = qdd[1]+OMcp24_213*ORcp24_314+OMcp24_214*ORcp24_315+OMcp24_215*ORcp24_316+OMcp24_216*ORcp24_317+OMcp24_217
 *ORcp24_354+OMcp24_26*ORcp24_313-OMcp24_313*ORcp24_214-OMcp24_314*ORcp24_215-OMcp24_315*ORcp24_216-OMcp24_316*ORcp24_217-
 OMcp24_317*ORcp24_254-OMcp24_36*ORcp24_213+OPcp24_213*RLcp24_314+OPcp24_214*RLcp24_315+OPcp24_215*RLcp24_316+OPcp24_216*
 RLcp24_317+OPcp24_217*RLcp24_354+OPcp24_26*RLcp24_313-OPcp24_313*RLcp24_214-OPcp24_314*RLcp24_215-OPcp24_315*RLcp24_216-
 OPcp24_316*RLcp24_217-OPcp24_317*RLcp24_254-OPcp24_36*RLcp24_213;
ACcp24_254 = qdd[2]-OMcp24_113*ORcp24_314-OMcp24_114*ORcp24_315-OMcp24_115*ORcp24_316-OMcp24_116*ORcp24_317-OMcp24_117
 *ORcp24_354-OMcp24_16*ORcp24_313+OMcp24_313*ORcp24_114+OMcp24_314*ORcp24_115+OMcp24_315*ORcp24_116+OMcp24_316*ORcp24_117+
 OMcp24_317*ORcp24_154+OMcp24_36*ORcp24_113-OPcp24_113*RLcp24_314-OPcp24_114*RLcp24_315-OPcp24_115*RLcp24_316-OPcp24_116*
 RLcp24_317-OPcp24_117*RLcp24_354-OPcp24_16*RLcp24_313+OPcp24_313*RLcp24_114+OPcp24_314*RLcp24_115+OPcp24_315*RLcp24_116+
 OPcp24_316*RLcp24_117+OPcp24_317*RLcp24_154+OPcp24_36*RLcp24_113;
ACcp24_354 = qdd[3]+OMcp24_113*ORcp24_214+OMcp24_114*ORcp24_215+OMcp24_115*ORcp24_216+OMcp24_116*ORcp24_217+OMcp24_117
 *ORcp24_254+OMcp24_16*ORcp24_213-OMcp24_213*ORcp24_114-OMcp24_214*ORcp24_115-OMcp24_215*ORcp24_116-OMcp24_216*ORcp24_117-
 OMcp24_217*ORcp24_154-OMcp24_26*ORcp24_113+OPcp24_113*RLcp24_214+OPcp24_114*RLcp24_215+OPcp24_115*RLcp24_216+OPcp24_116*
 RLcp24_217+OPcp24_117*RLcp24_254+OPcp24_16*RLcp24_213-OPcp24_213*RLcp24_114-OPcp24_214*RLcp24_115-OPcp24_215*RLcp24_116-
 OPcp24_216*RLcp24_117-OPcp24_217*RLcp24_154-OPcp24_26*RLcp24_113;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp24_154;
sens->P[2] = POcp24_254;
sens->P[3] = POcp24_354;
sens->R[1][1] = ROcp24_116;
sens->R[1][2] = ROcp24_216;
sens->R[1][3] = ROcp24_316;
sens->R[2][1] = ROcp24_417;
sens->R[2][2] = ROcp24_517;
sens->R[2][3] = ROcp24_617;
sens->R[3][1] = ROcp24_717;
sens->R[3][2] = ROcp24_817;
sens->R[3][3] = ROcp24_917;
sens->V[1] = VIcp24_154;
sens->V[2] = VIcp24_254;
sens->V[3] = VIcp24_354;
sens->OM[1] = OMcp24_117;
sens->OM[2] = OMcp24_217;
sens->OM[3] = OMcp24_317;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp24_154_5;
sens->J[1][6] = JTcp24_154_6;
sens->J[1][13] = JTcp24_154_7;
sens->J[1][14] = JTcp24_154_8;
sens->J[1][15] = JTcp24_154_9;
sens->J[1][16] = JTcp24_154_10;
sens->J[1][17] = JTcp24_154_11;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp24_254_4;
sens->J[2][5] = JTcp24_254_5;
sens->J[2][6] = JTcp24_254_6;
sens->J[2][13] = JTcp24_254_7;
sens->J[2][14] = JTcp24_254_8;
sens->J[2][15] = JTcp24_254_9;
sens->J[2][16] = JTcp24_254_10;
sens->J[2][17] = JTcp24_254_11;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp24_354_4;
sens->J[3][5] = JTcp24_354_5;
sens->J[3][6] = JTcp24_354_6;
sens->J[3][13] = JTcp24_354_7;
sens->J[3][14] = JTcp24_354_8;
sens->J[3][15] = JTcp24_354_9;
sens->J[3][16] = JTcp24_354_10;
sens->J[3][17] = JTcp24_354_11;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp24_46;
sens->J[4][14] = ROcp24_113;
sens->J[4][15] = ROcp24_714;
sens->J[4][16] = ROcp24_415;
sens->J[4][17] = ROcp24_116;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp24_85;
sens->J[5][13] = ROcp24_56;
sens->J[5][14] = ROcp24_213;
sens->J[5][15] = ROcp24_814;
sens->J[5][16] = ROcp24_515;
sens->J[5][17] = ROcp24_216;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp24_95;
sens->J[6][13] = ROcp24_66;
sens->J[6][14] = ROcp24_313;
sens->J[6][15] = ROcp24_914;
sens->J[6][16] = ROcp24_615;
sens->J[6][17] = ROcp24_316;
sens->A[1] = ACcp24_154;
sens->A[2] = ACcp24_254;
sens->A[3] = ACcp24_354;
sens->OMP[1] = OPcp24_117;
sens->OMP[2] = OPcp24_217;
sens->OMP[3] = OPcp24_317;
 
// 
break;
case 26:
 


// = = Block_1_0_0_26_0_1 = = 
 
// Sensor Kinematics 


ROcp25_25 = S4*S5;
ROcp25_35 = -C4*S5;
ROcp25_85 = -S4*C5;
ROcp25_95 = C4*C5;
ROcp25_16 = C5*C6;
ROcp25_26 = ROcp25_25*C6+C4*S6;
ROcp25_36 = ROcp25_35*C6+S4*S6;
ROcp25_46 = -C5*S6;
ROcp25_56 = -(ROcp25_25*S6-C4*C6);
ROcp25_66 = -(ROcp25_35*S6-S4*C6);
OMcp25_25 = qd[5]*C4;
OMcp25_35 = qd[5]*S4;
OMcp25_16 = qd[4]+qd[6]*S5;
OMcp25_26 = OMcp25_25+ROcp25_85*qd[6];
OMcp25_36 = OMcp25_35+ROcp25_95*qd[6];
OPcp25_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp25_26 = ROcp25_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp25_35*S5-ROcp25_95*qd[4]);
OPcp25_36 = ROcp25_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp25_25*S5-ROcp25_85*qd[4]);

// = = Block_1_0_0_26_0_3 = = 
 
// Sensor Kinematics 


ROcp25_113 = ROcp25_16*C13-S13*S5;
ROcp25_213 = ROcp25_26*C13-ROcp25_85*S13;
ROcp25_313 = ROcp25_36*C13-ROcp25_95*S13;
ROcp25_713 = ROcp25_16*S13+C13*S5;
ROcp25_813 = ROcp25_26*S13+ROcp25_85*C13;
ROcp25_913 = ROcp25_36*S13+ROcp25_95*C13;
ROcp25_414 = ROcp25_46*C14+ROcp25_713*S14;
ROcp25_514 = ROcp25_56*C14+ROcp25_813*S14;
ROcp25_614 = ROcp25_66*C14+ROcp25_913*S14;
ROcp25_714 = -(ROcp25_46*S14-ROcp25_713*C14);
ROcp25_814 = -(ROcp25_56*S14-ROcp25_813*C14);
ROcp25_914 = -(ROcp25_66*S14-ROcp25_913*C14);
ROcp25_115 = ROcp25_113*C15+ROcp25_414*S15;
ROcp25_215 = ROcp25_213*C15+ROcp25_514*S15;
ROcp25_315 = ROcp25_313*C15+ROcp25_614*S15;
ROcp25_415 = -(ROcp25_113*S15-ROcp25_414*C15);
ROcp25_515 = -(ROcp25_213*S15-ROcp25_514*C15);
ROcp25_615 = -(ROcp25_313*S15-ROcp25_614*C15);
ROcp25_116 = ROcp25_115*C16-ROcp25_714*S16;
ROcp25_216 = ROcp25_215*C16-ROcp25_814*S16;
ROcp25_316 = ROcp25_315*C16-ROcp25_914*S16;
ROcp25_716 = ROcp25_115*S16+ROcp25_714*C16;
ROcp25_816 = ROcp25_215*S16+ROcp25_814*C16;
ROcp25_916 = ROcp25_315*S16+ROcp25_914*C16;
ROcp25_417 = ROcp25_415*C17+ROcp25_716*S17;
ROcp25_517 = ROcp25_515*C17+ROcp25_816*S17;
ROcp25_617 = ROcp25_615*C17+ROcp25_916*S17;
ROcp25_717 = -(ROcp25_415*S17-ROcp25_716*C17);
ROcp25_817 = -(ROcp25_515*S17-ROcp25_816*C17);
ROcp25_917 = -(ROcp25_615*S17-ROcp25_916*C17);
ROcp25_118 = ROcp25_116*C18-ROcp25_717*S18;
ROcp25_218 = ROcp25_216*C18-ROcp25_817*S18;
ROcp25_318 = ROcp25_316*C18-ROcp25_917*S18;
ROcp25_718 = ROcp25_116*S18+ROcp25_717*C18;
ROcp25_818 = ROcp25_216*S18+ROcp25_817*C18;
ROcp25_918 = ROcp25_316*S18+ROcp25_917*C18;
RLcp25_113 = s->dpt[1][2]*ROcp25_16+s->dpt[3][2]*S5+ROcp25_46*s->dpt[2][2];
RLcp25_213 = s->dpt[1][2]*ROcp25_26+s->dpt[3][2]*ROcp25_85+ROcp25_56*s->dpt[2][2];
RLcp25_313 = s->dpt[1][2]*ROcp25_36+s->dpt[3][2]*ROcp25_95+ROcp25_66*s->dpt[2][2];
OMcp25_113 = OMcp25_16+ROcp25_46*qd[13];
OMcp25_213 = OMcp25_26+ROcp25_56*qd[13];
OMcp25_313 = OMcp25_36+ROcp25_66*qd[13];
ORcp25_113 = OMcp25_26*RLcp25_313-OMcp25_36*RLcp25_213;
ORcp25_213 = -(OMcp25_16*RLcp25_313-OMcp25_36*RLcp25_113);
ORcp25_313 = OMcp25_16*RLcp25_213-OMcp25_26*RLcp25_113;
OPcp25_113 = OPcp25_16+ROcp25_46*qdd[13]+qd[13]*(OMcp25_26*ROcp25_66-OMcp25_36*ROcp25_56);
OPcp25_213 = OPcp25_26+ROcp25_56*qdd[13]-qd[13]*(OMcp25_16*ROcp25_66-OMcp25_36*ROcp25_46);
OPcp25_313 = OPcp25_36+ROcp25_66*qdd[13]+qd[13]*(OMcp25_16*ROcp25_56-OMcp25_26*ROcp25_46);
RLcp25_114 = s->dpt[1][22]*ROcp25_113+s->dpt[3][22]*ROcp25_713+ROcp25_46*s->dpt[2][22];
RLcp25_214 = s->dpt[1][22]*ROcp25_213+s->dpt[3][22]*ROcp25_813+ROcp25_56*s->dpt[2][22];
RLcp25_314 = s->dpt[1][22]*ROcp25_313+s->dpt[3][22]*ROcp25_913+ROcp25_66*s->dpt[2][22];
OMcp25_114 = OMcp25_113+ROcp25_113*qd[14];
OMcp25_214 = OMcp25_213+ROcp25_213*qd[14];
OMcp25_314 = OMcp25_313+ROcp25_313*qd[14];
ORcp25_114 = OMcp25_213*RLcp25_314-OMcp25_313*RLcp25_214;
ORcp25_214 = -(OMcp25_113*RLcp25_314-OMcp25_313*RLcp25_114);
ORcp25_314 = OMcp25_113*RLcp25_214-OMcp25_213*RLcp25_114;
OPcp25_114 = OPcp25_113+ROcp25_113*qdd[14]+qd[14]*(OMcp25_213*ROcp25_313-OMcp25_313*ROcp25_213);
OPcp25_214 = OPcp25_213+ROcp25_213*qdd[14]-qd[14]*(OMcp25_113*ROcp25_313-OMcp25_313*ROcp25_113);
OPcp25_314 = OPcp25_313+ROcp25_313*qdd[14]+qd[14]*(OMcp25_113*ROcp25_213-OMcp25_213*ROcp25_113);
RLcp25_115 = s->dpt[1][24]*ROcp25_113+s->dpt[2][24]*ROcp25_414+ROcp25_714*s->dpt[3][24];
RLcp25_215 = s->dpt[1][24]*ROcp25_213+s->dpt[2][24]*ROcp25_514+ROcp25_814*s->dpt[3][24];
RLcp25_315 = s->dpt[1][24]*ROcp25_313+s->dpt[2][24]*ROcp25_614+ROcp25_914*s->dpt[3][24];
OMcp25_115 = OMcp25_114+ROcp25_714*qd[15];
OMcp25_215 = OMcp25_214+ROcp25_814*qd[15];
OMcp25_315 = OMcp25_314+ROcp25_914*qd[15];
ORcp25_115 = OMcp25_214*RLcp25_315-OMcp25_314*RLcp25_215;
ORcp25_215 = -(OMcp25_114*RLcp25_315-OMcp25_314*RLcp25_115);
ORcp25_315 = OMcp25_114*RLcp25_215-OMcp25_214*RLcp25_115;
OPcp25_115 = OPcp25_114+ROcp25_714*qdd[15]+qd[15]*(OMcp25_214*ROcp25_914-OMcp25_314*ROcp25_814);
OPcp25_215 = OPcp25_214+ROcp25_814*qdd[15]-qd[15]*(OMcp25_114*ROcp25_914-OMcp25_314*ROcp25_714);
OPcp25_315 = OPcp25_314+ROcp25_914*qdd[15]+qd[15]*(OMcp25_114*ROcp25_814-OMcp25_214*ROcp25_714);
RLcp25_116 = s->dpt[1][26]*ROcp25_115+s->dpt[2][26]*ROcp25_415+ROcp25_714*s->dpt[3][26];
RLcp25_216 = s->dpt[1][26]*ROcp25_215+s->dpt[2][26]*ROcp25_515+ROcp25_814*s->dpt[3][26];
RLcp25_316 = s->dpt[1][26]*ROcp25_315+s->dpt[2][26]*ROcp25_615+ROcp25_914*s->dpt[3][26];
OMcp25_116 = OMcp25_115+ROcp25_415*qd[16];
OMcp25_216 = OMcp25_215+ROcp25_515*qd[16];
OMcp25_316 = OMcp25_315+ROcp25_615*qd[16];
ORcp25_116 = OMcp25_215*RLcp25_316-OMcp25_315*RLcp25_216;
ORcp25_216 = -(OMcp25_115*RLcp25_316-OMcp25_315*RLcp25_116);
ORcp25_316 = OMcp25_115*RLcp25_216-OMcp25_215*RLcp25_116;
OPcp25_116 = OPcp25_115+ROcp25_415*qdd[16]+qd[16]*(OMcp25_215*ROcp25_615-OMcp25_315*ROcp25_515);
OPcp25_216 = OPcp25_215+ROcp25_515*qdd[16]-qd[16]*(OMcp25_115*ROcp25_615-OMcp25_315*ROcp25_415);
OPcp25_316 = OPcp25_315+ROcp25_615*qdd[16]+qd[16]*(OMcp25_115*ROcp25_515-OMcp25_215*ROcp25_415);
RLcp25_117 = s->dpt[1][28]*ROcp25_116+s->dpt[2][28]*ROcp25_415+ROcp25_716*s->dpt[3][28];
RLcp25_217 = s->dpt[1][28]*ROcp25_216+s->dpt[2][28]*ROcp25_515+ROcp25_816*s->dpt[3][28];
RLcp25_317 = s->dpt[1][28]*ROcp25_316+s->dpt[2][28]*ROcp25_615+ROcp25_916*s->dpt[3][28];
OMcp25_117 = OMcp25_116+ROcp25_116*qd[17];
OMcp25_217 = OMcp25_216+ROcp25_216*qd[17];
OMcp25_317 = OMcp25_316+ROcp25_316*qd[17];
ORcp25_117 = OMcp25_216*RLcp25_317-OMcp25_316*RLcp25_217;
ORcp25_217 = -(OMcp25_116*RLcp25_317-OMcp25_316*RLcp25_117);
ORcp25_317 = OMcp25_116*RLcp25_217-OMcp25_216*RLcp25_117;
OPcp25_117 = OPcp25_116+ROcp25_116*qdd[17]+qd[17]*(OMcp25_216*ROcp25_316-OMcp25_316*ROcp25_216);
OPcp25_217 = OPcp25_216+ROcp25_216*qdd[17]-qd[17]*(OMcp25_116*ROcp25_316-OMcp25_316*ROcp25_116);
OPcp25_317 = OPcp25_316+ROcp25_316*qdd[17]+qd[17]*(OMcp25_116*ROcp25_216-OMcp25_216*ROcp25_116);
RLcp25_118 = s->dpt[1][30]*ROcp25_116+s->dpt[2][30]*ROcp25_417+s->dpt[3][30]*ROcp25_717;
RLcp25_218 = s->dpt[1][30]*ROcp25_216+s->dpt[2][30]*ROcp25_517+s->dpt[3][30]*ROcp25_817;
RLcp25_318 = s->dpt[1][30]*ROcp25_316+s->dpt[2][30]*ROcp25_617+s->dpt[3][30]*ROcp25_917;
OMcp25_118 = OMcp25_117+ROcp25_417*qd[18];
OMcp25_218 = OMcp25_217+ROcp25_517*qd[18];
OMcp25_318 = OMcp25_317+ROcp25_617*qd[18];
ORcp25_118 = OMcp25_217*RLcp25_318-OMcp25_317*RLcp25_218;
ORcp25_218 = -(OMcp25_117*RLcp25_318-OMcp25_317*RLcp25_118);
ORcp25_318 = OMcp25_117*RLcp25_218-OMcp25_217*RLcp25_118;
OPcp25_118 = OPcp25_117+ROcp25_417*qdd[18]+qd[18]*(OMcp25_217*ROcp25_617-OMcp25_317*ROcp25_517);
OPcp25_218 = OPcp25_217+ROcp25_517*qdd[18]-qd[18]*(OMcp25_117*ROcp25_617-OMcp25_317*ROcp25_417);
OPcp25_318 = OPcp25_317+ROcp25_617*qdd[18]+qd[18]*(OMcp25_117*ROcp25_517-OMcp25_217*ROcp25_417);
RLcp25_155 = ROcp25_118*s->dpt[1][32]+ROcp25_417*s->dpt[2][32]+ROcp25_718*s->dpt[3][32];
RLcp25_255 = ROcp25_218*s->dpt[1][32]+ROcp25_517*s->dpt[2][32]+ROcp25_818*s->dpt[3][32];
RLcp25_355 = ROcp25_318*s->dpt[1][32]+ROcp25_617*s->dpt[2][32]+ROcp25_918*s->dpt[3][32];
POcp25_155 = RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_155+q[1];
POcp25_255 = RLcp25_213+RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_255+q[2];
POcp25_355 = RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_355+q[3];
JTcp25_255_4 = -(RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_355);
JTcp25_355_4 = RLcp25_213+RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_255;
JTcp25_155_5 = C4*(RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-S4*(RLcp25_213+RLcp25_214)-S4*(
 RLcp25_215+RLcp25_216)-S4*(RLcp25_217+RLcp25_218)-RLcp25_255*S4+RLcp25_355*C4;
JTcp25_255_5 = S4*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_155);
JTcp25_355_5 = -C4*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_155);
JTcp25_155_6 = ROcp25_85*(RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-ROcp25_95*(RLcp25_213+
 RLcp25_214)-ROcp25_95*(RLcp25_215+RLcp25_216)-ROcp25_95*(RLcp25_217+RLcp25_218)-RLcp25_255*ROcp25_95+RLcp25_355*ROcp25_85;
JTcp25_255_6 = -(RLcp25_355*S5-ROcp25_95*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_155
 )+S5*(RLcp25_313+RLcp25_314)+S5*(RLcp25_315+RLcp25_316)+S5*(RLcp25_317+RLcp25_318));
JTcp25_355_6 = RLcp25_255*S5-ROcp25_85*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_155)+
 S5*(RLcp25_213+RLcp25_214)+S5*(RLcp25_215+RLcp25_216)+S5*(RLcp25_217+RLcp25_218);
JTcp25_155_7 = ROcp25_56*(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_355)-ROcp25_66*(RLcp25_214+
 RLcp25_215)-ROcp25_66*(RLcp25_216+RLcp25_217)-ROcp25_66*(RLcp25_218+RLcp25_255);
JTcp25_255_7 = -(ROcp25_46*(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_355)-ROcp25_66*(RLcp25_114+
 RLcp25_115)-ROcp25_66*(RLcp25_116+RLcp25_117)-ROcp25_66*(RLcp25_118+RLcp25_155));
JTcp25_355_7 = ROcp25_46*(RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_255)-ROcp25_56*(RLcp25_114+
 RLcp25_115)-ROcp25_56*(RLcp25_116+RLcp25_117)-ROcp25_56*(RLcp25_118+RLcp25_155);
JTcp25_155_8 = ROcp25_213*(RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-ROcp25_313*(RLcp25_215+RLcp25_216)-ROcp25_313*
 (RLcp25_217+RLcp25_218)-RLcp25_255*ROcp25_313+RLcp25_355*ROcp25_213;
JTcp25_255_8 = RLcp25_155*ROcp25_313-RLcp25_355*ROcp25_113-ROcp25_113*(RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)+
 ROcp25_313*(RLcp25_115+RLcp25_116)+ROcp25_313*(RLcp25_117+RLcp25_118);
JTcp25_355_8 = ROcp25_113*(RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218)-ROcp25_213*(RLcp25_115+RLcp25_116)-ROcp25_213*
 (RLcp25_117+RLcp25_118)-RLcp25_155*ROcp25_213+RLcp25_255*ROcp25_113;
JTcp25_155_9 = ROcp25_814*(RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_355)-ROcp25_914*(RLcp25_216+RLcp25_217)-ROcp25_914*
 (RLcp25_218+RLcp25_255);
JTcp25_255_9 = -(ROcp25_714*(RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_355)-ROcp25_914*(RLcp25_116+RLcp25_117)-
 ROcp25_914*(RLcp25_118+RLcp25_155));
JTcp25_355_9 = ROcp25_714*(RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_255)-ROcp25_814*(RLcp25_116+RLcp25_117)-ROcp25_814*
 (RLcp25_118+RLcp25_155);
JTcp25_155_10 = ROcp25_515*(RLcp25_317+RLcp25_318)-ROcp25_615*(RLcp25_217+RLcp25_218)-RLcp25_255*ROcp25_615+RLcp25_355
 *ROcp25_515;
JTcp25_255_10 = RLcp25_155*ROcp25_615-RLcp25_355*ROcp25_415-ROcp25_415*(RLcp25_317+RLcp25_318)+ROcp25_615*(RLcp25_117+
 RLcp25_118);
JTcp25_355_10 = ROcp25_415*(RLcp25_217+RLcp25_218)-ROcp25_515*(RLcp25_117+RLcp25_118)-RLcp25_155*ROcp25_515+RLcp25_255
 *ROcp25_415;
JTcp25_155_11 = ROcp25_216*(RLcp25_318+RLcp25_355)-ROcp25_316*(RLcp25_218+RLcp25_255);
JTcp25_255_11 = -(ROcp25_116*(RLcp25_318+RLcp25_355)-ROcp25_316*(RLcp25_118+RLcp25_155));
JTcp25_355_11 = ROcp25_116*(RLcp25_218+RLcp25_255)-ROcp25_216*(RLcp25_118+RLcp25_155);
JTcp25_155_12 = -(RLcp25_255*ROcp25_617-RLcp25_355*ROcp25_517);
JTcp25_255_12 = RLcp25_155*ROcp25_617-RLcp25_355*ROcp25_417;
JTcp25_355_12 = -(RLcp25_155*ROcp25_517-RLcp25_255*ROcp25_417);
ORcp25_155 = OMcp25_218*RLcp25_355-OMcp25_318*RLcp25_255;
ORcp25_255 = -(OMcp25_118*RLcp25_355-OMcp25_318*RLcp25_155);
ORcp25_355 = OMcp25_118*RLcp25_255-OMcp25_218*RLcp25_155;
VIcp25_155 = ORcp25_113+ORcp25_114+ORcp25_115+ORcp25_116+ORcp25_117+ORcp25_118+ORcp25_155+qd[1];
VIcp25_255 = ORcp25_213+ORcp25_214+ORcp25_215+ORcp25_216+ORcp25_217+ORcp25_218+ORcp25_255+qd[2];
VIcp25_355 = ORcp25_313+ORcp25_314+ORcp25_315+ORcp25_316+ORcp25_317+ORcp25_318+ORcp25_355+qd[3];
ACcp25_155 = qdd[1]+OMcp25_213*ORcp25_314+OMcp25_214*ORcp25_315+OMcp25_215*ORcp25_316+OMcp25_216*ORcp25_317+OMcp25_217
 *ORcp25_318+OMcp25_218*ORcp25_355+OMcp25_26*ORcp25_313-OMcp25_313*ORcp25_214-OMcp25_314*ORcp25_215-OMcp25_315*ORcp25_216-
 OMcp25_316*ORcp25_217-OMcp25_317*ORcp25_218-OMcp25_318*ORcp25_255-OMcp25_36*ORcp25_213+OPcp25_213*RLcp25_314+OPcp25_214*
 RLcp25_315+OPcp25_215*RLcp25_316+OPcp25_216*RLcp25_317+OPcp25_217*RLcp25_318+OPcp25_218*RLcp25_355+OPcp25_26*RLcp25_313-
 OPcp25_313*RLcp25_214-OPcp25_314*RLcp25_215-OPcp25_315*RLcp25_216-OPcp25_316*RLcp25_217-OPcp25_317*RLcp25_218-OPcp25_318*
 RLcp25_255-OPcp25_36*RLcp25_213;
ACcp25_255 = qdd[2]-OMcp25_113*ORcp25_314-OMcp25_114*ORcp25_315-OMcp25_115*ORcp25_316-OMcp25_116*ORcp25_317-OMcp25_117
 *ORcp25_318-OMcp25_118*ORcp25_355-OMcp25_16*ORcp25_313+OMcp25_313*ORcp25_114+OMcp25_314*ORcp25_115+OMcp25_315*ORcp25_116+
 OMcp25_316*ORcp25_117+OMcp25_317*ORcp25_118+OMcp25_318*ORcp25_155+OMcp25_36*ORcp25_113-OPcp25_113*RLcp25_314-OPcp25_114*
 RLcp25_315-OPcp25_115*RLcp25_316-OPcp25_116*RLcp25_317-OPcp25_117*RLcp25_318-OPcp25_118*RLcp25_355-OPcp25_16*RLcp25_313+
 OPcp25_313*RLcp25_114+OPcp25_314*RLcp25_115+OPcp25_315*RLcp25_116+OPcp25_316*RLcp25_117+OPcp25_317*RLcp25_118+OPcp25_318*
 RLcp25_155+OPcp25_36*RLcp25_113;
ACcp25_355 = qdd[3]+OMcp25_113*ORcp25_214+OMcp25_114*ORcp25_215+OMcp25_115*ORcp25_216+OMcp25_116*ORcp25_217+OMcp25_117
 *ORcp25_218+OMcp25_118*ORcp25_255+OMcp25_16*ORcp25_213-OMcp25_213*ORcp25_114-OMcp25_214*ORcp25_115-OMcp25_215*ORcp25_116-
 OMcp25_216*ORcp25_117-OMcp25_217*ORcp25_118-OMcp25_218*ORcp25_155-OMcp25_26*ORcp25_113+OPcp25_113*RLcp25_214+OPcp25_114*
 RLcp25_215+OPcp25_115*RLcp25_216+OPcp25_116*RLcp25_217+OPcp25_117*RLcp25_218+OPcp25_118*RLcp25_255+OPcp25_16*RLcp25_213-
 OPcp25_213*RLcp25_114-OPcp25_214*RLcp25_115-OPcp25_215*RLcp25_116-OPcp25_216*RLcp25_117-OPcp25_217*RLcp25_118-OPcp25_218*
 RLcp25_155-OPcp25_26*RLcp25_113;

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp25_155;
sens->P[2] = POcp25_255;
sens->P[3] = POcp25_355;
sens->R[1][1] = ROcp25_118;
sens->R[1][2] = ROcp25_218;
sens->R[1][3] = ROcp25_318;
sens->R[2][1] = ROcp25_417;
sens->R[2][2] = ROcp25_517;
sens->R[2][3] = ROcp25_617;
sens->R[3][1] = ROcp25_718;
sens->R[3][2] = ROcp25_818;
sens->R[3][3] = ROcp25_918;
sens->V[1] = VIcp25_155;
sens->V[2] = VIcp25_255;
sens->V[3] = VIcp25_355;
sens->OM[1] = OMcp25_118;
sens->OM[2] = OMcp25_218;
sens->OM[3] = OMcp25_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp25_155_5;
sens->J[1][6] = JTcp25_155_6;
sens->J[1][13] = JTcp25_155_7;
sens->J[1][14] = JTcp25_155_8;
sens->J[1][15] = JTcp25_155_9;
sens->J[1][16] = JTcp25_155_10;
sens->J[1][17] = JTcp25_155_11;
sens->J[1][18] = JTcp25_155_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp25_255_4;
sens->J[2][5] = JTcp25_255_5;
sens->J[2][6] = JTcp25_255_6;
sens->J[2][13] = JTcp25_255_7;
sens->J[2][14] = JTcp25_255_8;
sens->J[2][15] = JTcp25_255_9;
sens->J[2][16] = JTcp25_255_10;
sens->J[2][17] = JTcp25_255_11;
sens->J[2][18] = JTcp25_255_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp25_355_4;
sens->J[3][5] = JTcp25_355_5;
sens->J[3][6] = JTcp25_355_6;
sens->J[3][13] = JTcp25_355_7;
sens->J[3][14] = JTcp25_355_8;
sens->J[3][15] = JTcp25_355_9;
sens->J[3][16] = JTcp25_355_10;
sens->J[3][17] = JTcp25_355_11;
sens->J[3][18] = JTcp25_355_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp25_46;
sens->J[4][14] = ROcp25_113;
sens->J[4][15] = ROcp25_714;
sens->J[4][16] = ROcp25_415;
sens->J[4][17] = ROcp25_116;
sens->J[4][18] = ROcp25_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp25_85;
sens->J[5][13] = ROcp25_56;
sens->J[5][14] = ROcp25_213;
sens->J[5][15] = ROcp25_814;
sens->J[5][16] = ROcp25_515;
sens->J[5][17] = ROcp25_216;
sens->J[5][18] = ROcp25_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp25_95;
sens->J[6][13] = ROcp25_66;
sens->J[6][14] = ROcp25_313;
sens->J[6][15] = ROcp25_914;
sens->J[6][16] = ROcp25_615;
sens->J[6][17] = ROcp25_316;
sens->J[6][18] = ROcp25_617;
sens->A[1] = ACcp25_155;
sens->A[2] = ACcp25_255;
sens->A[3] = ACcp25_355;
sens->OMP[1] = OPcp25_118;
sens->OMP[2] = OPcp25_218;
sens->OMP[3] = OPcp25_318;
 
// 
break;
case 27:
 


// = = Block_1_0_0_27_0_1 = = 
 
// Sensor Kinematics 


ROcp26_25 = S4*S5;
ROcp26_35 = -C4*S5;
ROcp26_85 = -S4*C5;
ROcp26_95 = C4*C5;
ROcp26_16 = C5*C6;
ROcp26_26 = ROcp26_25*C6+C4*S6;
ROcp26_36 = ROcp26_35*C6+S4*S6;
ROcp26_46 = -C5*S6;
ROcp26_56 = -(ROcp26_25*S6-C4*C6);
ROcp26_66 = -(ROcp26_35*S6-S4*C6);
OMcp26_25 = qd[5]*C4;
OMcp26_35 = qd[5]*S4;
OMcp26_16 = qd[4]+qd[6]*S5;
OMcp26_26 = OMcp26_25+ROcp26_85*qd[6];
OMcp26_36 = OMcp26_35+ROcp26_95*qd[6];
OPcp26_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp26_26 = ROcp26_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp26_35*S5-ROcp26_95*qd[4]);
OPcp26_36 = ROcp26_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp26_25*S5-ROcp26_85*qd[4]);

// = = Block_1_0_0_27_0_3 = = 
 
// Sensor Kinematics 


ROcp26_113 = ROcp26_16*C13-S13*S5;
ROcp26_213 = ROcp26_26*C13-ROcp26_85*S13;
ROcp26_313 = ROcp26_36*C13-ROcp26_95*S13;
ROcp26_713 = ROcp26_16*S13+C13*S5;
ROcp26_813 = ROcp26_26*S13+ROcp26_85*C13;
ROcp26_913 = ROcp26_36*S13+ROcp26_95*C13;
ROcp26_414 = ROcp26_46*C14+ROcp26_713*S14;
ROcp26_514 = ROcp26_56*C14+ROcp26_813*S14;
ROcp26_614 = ROcp26_66*C14+ROcp26_913*S14;
ROcp26_714 = -(ROcp26_46*S14-ROcp26_713*C14);
ROcp26_814 = -(ROcp26_56*S14-ROcp26_813*C14);
ROcp26_914 = -(ROcp26_66*S14-ROcp26_913*C14);
ROcp26_115 = ROcp26_113*C15+ROcp26_414*S15;
ROcp26_215 = ROcp26_213*C15+ROcp26_514*S15;
ROcp26_315 = ROcp26_313*C15+ROcp26_614*S15;
ROcp26_415 = -(ROcp26_113*S15-ROcp26_414*C15);
ROcp26_515 = -(ROcp26_213*S15-ROcp26_514*C15);
ROcp26_615 = -(ROcp26_313*S15-ROcp26_614*C15);
ROcp26_116 = ROcp26_115*C16-ROcp26_714*S16;
ROcp26_216 = ROcp26_215*C16-ROcp26_814*S16;
ROcp26_316 = ROcp26_315*C16-ROcp26_914*S16;
ROcp26_716 = ROcp26_115*S16+ROcp26_714*C16;
ROcp26_816 = ROcp26_215*S16+ROcp26_814*C16;
ROcp26_916 = ROcp26_315*S16+ROcp26_914*C16;
ROcp26_417 = ROcp26_415*C17+ROcp26_716*S17;
ROcp26_517 = ROcp26_515*C17+ROcp26_816*S17;
ROcp26_617 = ROcp26_615*C17+ROcp26_916*S17;
ROcp26_717 = -(ROcp26_415*S17-ROcp26_716*C17);
ROcp26_817 = -(ROcp26_515*S17-ROcp26_816*C17);
ROcp26_917 = -(ROcp26_615*S17-ROcp26_916*C17);
ROcp26_118 = ROcp26_116*C18-ROcp26_717*S18;
ROcp26_218 = ROcp26_216*C18-ROcp26_817*S18;
ROcp26_318 = ROcp26_316*C18-ROcp26_917*S18;
ROcp26_718 = ROcp26_116*S18+ROcp26_717*C18;
ROcp26_818 = ROcp26_216*S18+ROcp26_817*C18;
ROcp26_918 = ROcp26_316*S18+ROcp26_917*C18;
RLcp26_113 = s->dpt[1][2]*ROcp26_16+s->dpt[3][2]*S5+ROcp26_46*s->dpt[2][2];
RLcp26_213 = s->dpt[1][2]*ROcp26_26+s->dpt[3][2]*ROcp26_85+ROcp26_56*s->dpt[2][2];
RLcp26_313 = s->dpt[1][2]*ROcp26_36+s->dpt[3][2]*ROcp26_95+ROcp26_66*s->dpt[2][2];
OMcp26_113 = OMcp26_16+ROcp26_46*qd[13];
OMcp26_213 = OMcp26_26+ROcp26_56*qd[13];
OMcp26_313 = OMcp26_36+ROcp26_66*qd[13];
ORcp26_113 = OMcp26_26*RLcp26_313-OMcp26_36*RLcp26_213;
ORcp26_213 = -(OMcp26_16*RLcp26_313-OMcp26_36*RLcp26_113);
ORcp26_313 = OMcp26_16*RLcp26_213-OMcp26_26*RLcp26_113;
OPcp26_113 = OPcp26_16+ROcp26_46*qdd[13]+qd[13]*(OMcp26_26*ROcp26_66-OMcp26_36*ROcp26_56);
OPcp26_213 = OPcp26_26+ROcp26_56*qdd[13]-qd[13]*(OMcp26_16*ROcp26_66-OMcp26_36*ROcp26_46);
OPcp26_313 = OPcp26_36+ROcp26_66*qdd[13]+qd[13]*(OMcp26_16*ROcp26_56-OMcp26_26*ROcp26_46);
RLcp26_114 = s->dpt[1][22]*ROcp26_113+s->dpt[3][22]*ROcp26_713+ROcp26_46*s->dpt[2][22];
RLcp26_214 = s->dpt[1][22]*ROcp26_213+s->dpt[3][22]*ROcp26_813+ROcp26_56*s->dpt[2][22];
RLcp26_314 = s->dpt[1][22]*ROcp26_313+s->dpt[3][22]*ROcp26_913+ROcp26_66*s->dpt[2][22];
OMcp26_114 = OMcp26_113+ROcp26_113*qd[14];
OMcp26_214 = OMcp26_213+ROcp26_213*qd[14];
OMcp26_314 = OMcp26_313+ROcp26_313*qd[14];
ORcp26_114 = OMcp26_213*RLcp26_314-OMcp26_313*RLcp26_214;
ORcp26_214 = -(OMcp26_113*RLcp26_314-OMcp26_313*RLcp26_114);
ORcp26_314 = OMcp26_113*RLcp26_214-OMcp26_213*RLcp26_114;
OPcp26_114 = OPcp26_113+ROcp26_113*qdd[14]+qd[14]*(OMcp26_213*ROcp26_313-OMcp26_313*ROcp26_213);
OPcp26_214 = OPcp26_213+ROcp26_213*qdd[14]-qd[14]*(OMcp26_113*ROcp26_313-OMcp26_313*ROcp26_113);
OPcp26_314 = OPcp26_313+ROcp26_313*qdd[14]+qd[14]*(OMcp26_113*ROcp26_213-OMcp26_213*ROcp26_113);
RLcp26_115 = s->dpt[1][24]*ROcp26_113+s->dpt[2][24]*ROcp26_414+ROcp26_714*s->dpt[3][24];
RLcp26_215 = s->dpt[1][24]*ROcp26_213+s->dpt[2][24]*ROcp26_514+ROcp26_814*s->dpt[3][24];
RLcp26_315 = s->dpt[1][24]*ROcp26_313+s->dpt[2][24]*ROcp26_614+ROcp26_914*s->dpt[3][24];
OMcp26_115 = OMcp26_114+ROcp26_714*qd[15];
OMcp26_215 = OMcp26_214+ROcp26_814*qd[15];
OMcp26_315 = OMcp26_314+ROcp26_914*qd[15];
ORcp26_115 = OMcp26_214*RLcp26_315-OMcp26_314*RLcp26_215;
ORcp26_215 = -(OMcp26_114*RLcp26_315-OMcp26_314*RLcp26_115);
ORcp26_315 = OMcp26_114*RLcp26_215-OMcp26_214*RLcp26_115;
OPcp26_115 = OPcp26_114+ROcp26_714*qdd[15]+qd[15]*(OMcp26_214*ROcp26_914-OMcp26_314*ROcp26_814);
OPcp26_215 = OPcp26_214+ROcp26_814*qdd[15]-qd[15]*(OMcp26_114*ROcp26_914-OMcp26_314*ROcp26_714);
OPcp26_315 = OPcp26_314+ROcp26_914*qdd[15]+qd[15]*(OMcp26_114*ROcp26_814-OMcp26_214*ROcp26_714);
RLcp26_116 = s->dpt[1][26]*ROcp26_115+s->dpt[2][26]*ROcp26_415+ROcp26_714*s->dpt[3][26];
RLcp26_216 = s->dpt[1][26]*ROcp26_215+s->dpt[2][26]*ROcp26_515+ROcp26_814*s->dpt[3][26];
RLcp26_316 = s->dpt[1][26]*ROcp26_315+s->dpt[2][26]*ROcp26_615+ROcp26_914*s->dpt[3][26];
OMcp26_116 = OMcp26_115+ROcp26_415*qd[16];
OMcp26_216 = OMcp26_215+ROcp26_515*qd[16];
OMcp26_316 = OMcp26_315+ROcp26_615*qd[16];
ORcp26_116 = OMcp26_215*RLcp26_316-OMcp26_315*RLcp26_216;
ORcp26_216 = -(OMcp26_115*RLcp26_316-OMcp26_315*RLcp26_116);
ORcp26_316 = OMcp26_115*RLcp26_216-OMcp26_215*RLcp26_116;
OPcp26_116 = OPcp26_115+ROcp26_415*qdd[16]+qd[16]*(OMcp26_215*ROcp26_615-OMcp26_315*ROcp26_515);
OPcp26_216 = OPcp26_215+ROcp26_515*qdd[16]-qd[16]*(OMcp26_115*ROcp26_615-OMcp26_315*ROcp26_415);
OPcp26_316 = OPcp26_315+ROcp26_615*qdd[16]+qd[16]*(OMcp26_115*ROcp26_515-OMcp26_215*ROcp26_415);
RLcp26_117 = s->dpt[1][28]*ROcp26_116+s->dpt[2][28]*ROcp26_415+ROcp26_716*s->dpt[3][28];
RLcp26_217 = s->dpt[1][28]*ROcp26_216+s->dpt[2][28]*ROcp26_515+ROcp26_816*s->dpt[3][28];
RLcp26_317 = s->dpt[1][28]*ROcp26_316+s->dpt[2][28]*ROcp26_615+ROcp26_916*s->dpt[3][28];
OMcp26_117 = OMcp26_116+ROcp26_116*qd[17];
OMcp26_217 = OMcp26_216+ROcp26_216*qd[17];
OMcp26_317 = OMcp26_316+ROcp26_316*qd[17];
ORcp26_117 = OMcp26_216*RLcp26_317-OMcp26_316*RLcp26_217;
ORcp26_217 = -(OMcp26_116*RLcp26_317-OMcp26_316*RLcp26_117);
ORcp26_317 = OMcp26_116*RLcp26_217-OMcp26_216*RLcp26_117;
OPcp26_117 = OPcp26_116+ROcp26_116*qdd[17]+qd[17]*(OMcp26_216*ROcp26_316-OMcp26_316*ROcp26_216);
OPcp26_217 = OPcp26_216+ROcp26_216*qdd[17]-qd[17]*(OMcp26_116*ROcp26_316-OMcp26_316*ROcp26_116);
OPcp26_317 = OPcp26_316+ROcp26_316*qdd[17]+qd[17]*(OMcp26_116*ROcp26_216-OMcp26_216*ROcp26_116);
RLcp26_118 = s->dpt[1][30]*ROcp26_116+s->dpt[2][30]*ROcp26_417+s->dpt[3][30]*ROcp26_717;
RLcp26_218 = s->dpt[1][30]*ROcp26_216+s->dpt[2][30]*ROcp26_517+s->dpt[3][30]*ROcp26_817;
RLcp26_318 = s->dpt[1][30]*ROcp26_316+s->dpt[2][30]*ROcp26_617+s->dpt[3][30]*ROcp26_917;
OMcp26_118 = OMcp26_117+ROcp26_417*qd[18];
OMcp26_218 = OMcp26_217+ROcp26_517*qd[18];
OMcp26_318 = OMcp26_317+ROcp26_617*qd[18];
ORcp26_118 = OMcp26_217*RLcp26_318-OMcp26_317*RLcp26_218;
ORcp26_218 = -(OMcp26_117*RLcp26_318-OMcp26_317*RLcp26_118);
ORcp26_318 = OMcp26_117*RLcp26_218-OMcp26_217*RLcp26_118;
OPcp26_118 = OPcp26_117+ROcp26_417*qdd[18]+qd[18]*(OMcp26_217*ROcp26_617-OMcp26_317*ROcp26_517);
OPcp26_218 = OPcp26_217+ROcp26_517*qdd[18]-qd[18]*(OMcp26_117*ROcp26_617-OMcp26_317*ROcp26_417);
OPcp26_318 = OPcp26_317+ROcp26_617*qdd[18]+qd[18]*(OMcp26_117*ROcp26_517-OMcp26_217*ROcp26_417);
RLcp26_156 = s->dpt[1][33]*ROcp26_118+s->dpt[2][33]*ROcp26_417+ROcp26_718*s->dpt[3][33];
RLcp26_256 = s->dpt[1][33]*ROcp26_218+s->dpt[2][33]*ROcp26_517+ROcp26_818*s->dpt[3][33];
RLcp26_356 = s->dpt[1][33]*ROcp26_318+s->dpt[2][33]*ROcp26_617+ROcp26_918*s->dpt[3][33];
POcp26_156 = RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_156+q[1];
POcp26_256 = RLcp26_213+RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_256+q[2];
POcp26_356 = RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_356+q[3];
JTcp26_256_4 = -(RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_356);
JTcp26_356_4 = RLcp26_213+RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_256;
JTcp26_156_5 = C4*(RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-S4*(RLcp26_213+RLcp26_214)-S4*(
 RLcp26_215+RLcp26_216)-S4*(RLcp26_217+RLcp26_218)-RLcp26_256*S4+RLcp26_356*C4;
JTcp26_256_5 = S4*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_156);
JTcp26_356_5 = -C4*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_156);
JTcp26_156_6 = ROcp26_85*(RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-ROcp26_95*(RLcp26_213+
 RLcp26_214)-ROcp26_95*(RLcp26_215+RLcp26_216)-ROcp26_95*(RLcp26_217+RLcp26_218)-RLcp26_256*ROcp26_95+RLcp26_356*ROcp26_85;
JTcp26_256_6 = -(RLcp26_356*S5-ROcp26_95*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_156
 )+S5*(RLcp26_313+RLcp26_314)+S5*(RLcp26_315+RLcp26_316)+S5*(RLcp26_317+RLcp26_318));
JTcp26_356_6 = RLcp26_256*S5-ROcp26_85*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_156)+
 S5*(RLcp26_213+RLcp26_214)+S5*(RLcp26_215+RLcp26_216)+S5*(RLcp26_217+RLcp26_218);
JTcp26_156_7 = ROcp26_56*(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_356)-ROcp26_66*(RLcp26_214+
 RLcp26_215)-ROcp26_66*(RLcp26_216+RLcp26_217)-ROcp26_66*(RLcp26_218+RLcp26_256);
JTcp26_256_7 = -(ROcp26_46*(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_356)-ROcp26_66*(RLcp26_114+
 RLcp26_115)-ROcp26_66*(RLcp26_116+RLcp26_117)-ROcp26_66*(RLcp26_118+RLcp26_156));
JTcp26_356_7 = ROcp26_46*(RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_256)-ROcp26_56*(RLcp26_114+
 RLcp26_115)-ROcp26_56*(RLcp26_116+RLcp26_117)-ROcp26_56*(RLcp26_118+RLcp26_156);
JTcp26_156_8 = ROcp26_213*(RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-ROcp26_313*(RLcp26_215+RLcp26_216)-ROcp26_313*
 (RLcp26_217+RLcp26_218)-RLcp26_256*ROcp26_313+RLcp26_356*ROcp26_213;
JTcp26_256_8 = RLcp26_156*ROcp26_313-RLcp26_356*ROcp26_113-ROcp26_113*(RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)+
 ROcp26_313*(RLcp26_115+RLcp26_116)+ROcp26_313*(RLcp26_117+RLcp26_118);
JTcp26_356_8 = ROcp26_113*(RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218)-ROcp26_213*(RLcp26_115+RLcp26_116)-ROcp26_213*
 (RLcp26_117+RLcp26_118)-RLcp26_156*ROcp26_213+RLcp26_256*ROcp26_113;
JTcp26_156_9 = ROcp26_814*(RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_356)-ROcp26_914*(RLcp26_216+RLcp26_217)-ROcp26_914*
 (RLcp26_218+RLcp26_256);
JTcp26_256_9 = -(ROcp26_714*(RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_356)-ROcp26_914*(RLcp26_116+RLcp26_117)-
 ROcp26_914*(RLcp26_118+RLcp26_156));
JTcp26_356_9 = ROcp26_714*(RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_256)-ROcp26_814*(RLcp26_116+RLcp26_117)-ROcp26_814*
 (RLcp26_118+RLcp26_156);
JTcp26_156_10 = ROcp26_515*(RLcp26_317+RLcp26_318)-ROcp26_615*(RLcp26_217+RLcp26_218)-RLcp26_256*ROcp26_615+RLcp26_356
 *ROcp26_515;
JTcp26_256_10 = RLcp26_156*ROcp26_615-RLcp26_356*ROcp26_415-ROcp26_415*(RLcp26_317+RLcp26_318)+ROcp26_615*(RLcp26_117+
 RLcp26_118);
JTcp26_356_10 = ROcp26_415*(RLcp26_217+RLcp26_218)-ROcp26_515*(RLcp26_117+RLcp26_118)-RLcp26_156*ROcp26_515+RLcp26_256
 *ROcp26_415;
JTcp26_156_11 = ROcp26_216*(RLcp26_318+RLcp26_356)-ROcp26_316*(RLcp26_218+RLcp26_256);
JTcp26_256_11 = -(ROcp26_116*(RLcp26_318+RLcp26_356)-ROcp26_316*(RLcp26_118+RLcp26_156));
JTcp26_356_11 = ROcp26_116*(RLcp26_218+RLcp26_256)-ROcp26_216*(RLcp26_118+RLcp26_156);
JTcp26_156_12 = -(RLcp26_256*ROcp26_617-RLcp26_356*ROcp26_517);
JTcp26_256_12 = RLcp26_156*ROcp26_617-RLcp26_356*ROcp26_417;
JTcp26_356_12 = -(RLcp26_156*ROcp26_517-RLcp26_256*ROcp26_417);
ORcp26_156 = OMcp26_218*RLcp26_356-OMcp26_318*RLcp26_256;
ORcp26_256 = -(OMcp26_118*RLcp26_356-OMcp26_318*RLcp26_156);
ORcp26_356 = OMcp26_118*RLcp26_256-OMcp26_218*RLcp26_156;
VIcp26_156 = ORcp26_113+ORcp26_114+ORcp26_115+ORcp26_116+ORcp26_117+ORcp26_118+ORcp26_156+qd[1];
VIcp26_256 = ORcp26_213+ORcp26_214+ORcp26_215+ORcp26_216+ORcp26_217+ORcp26_218+ORcp26_256+qd[2];
VIcp26_356 = ORcp26_313+ORcp26_314+ORcp26_315+ORcp26_316+ORcp26_317+ORcp26_318+ORcp26_356+qd[3];
ACcp26_156 = qdd[1]+OMcp26_213*ORcp26_314+OMcp26_214*ORcp26_315+OMcp26_215*ORcp26_316+OMcp26_216*ORcp26_317+OMcp26_217
 *ORcp26_318+OMcp26_218*ORcp26_356+OMcp26_26*ORcp26_313-OMcp26_313*ORcp26_214-OMcp26_314*ORcp26_215-OMcp26_315*ORcp26_216-
 OMcp26_316*ORcp26_217-OMcp26_317*ORcp26_218-OMcp26_318*ORcp26_256-OMcp26_36*ORcp26_213+OPcp26_213*RLcp26_314+OPcp26_214*
 RLcp26_315+OPcp26_215*RLcp26_316+OPcp26_216*RLcp26_317+OPcp26_217*RLcp26_318+OPcp26_218*RLcp26_356+OPcp26_26*RLcp26_313-
 OPcp26_313*RLcp26_214-OPcp26_314*RLcp26_215-OPcp26_315*RLcp26_216-OPcp26_316*RLcp26_217-OPcp26_317*RLcp26_218-OPcp26_318*
 RLcp26_256-OPcp26_36*RLcp26_213;
ACcp26_256 = qdd[2]-OMcp26_113*ORcp26_314-OMcp26_114*ORcp26_315-OMcp26_115*ORcp26_316-OMcp26_116*ORcp26_317-OMcp26_117
 *ORcp26_318-OMcp26_118*ORcp26_356-OMcp26_16*ORcp26_313+OMcp26_313*ORcp26_114+OMcp26_314*ORcp26_115+OMcp26_315*ORcp26_116+
 OMcp26_316*ORcp26_117+OMcp26_317*ORcp26_118+OMcp26_318*ORcp26_156+OMcp26_36*ORcp26_113-OPcp26_113*RLcp26_314-OPcp26_114*
 RLcp26_315-OPcp26_115*RLcp26_316-OPcp26_116*RLcp26_317-OPcp26_117*RLcp26_318-OPcp26_118*RLcp26_356-OPcp26_16*RLcp26_313+
 OPcp26_313*RLcp26_114+OPcp26_314*RLcp26_115+OPcp26_315*RLcp26_116+OPcp26_316*RLcp26_117+OPcp26_317*RLcp26_118+OPcp26_318*
 RLcp26_156+OPcp26_36*RLcp26_113;
ACcp26_356 = qdd[3]+OMcp26_113*ORcp26_214+OMcp26_114*ORcp26_215+OMcp26_115*ORcp26_216+OMcp26_116*ORcp26_217+OMcp26_117
 *ORcp26_218+OMcp26_118*ORcp26_256+OMcp26_16*ORcp26_213-OMcp26_213*ORcp26_114-OMcp26_214*ORcp26_115-OMcp26_215*ORcp26_116-
 OMcp26_216*ORcp26_117-OMcp26_217*ORcp26_118-OMcp26_218*ORcp26_156-OMcp26_26*ORcp26_113+OPcp26_113*RLcp26_214+OPcp26_114*
 RLcp26_215+OPcp26_115*RLcp26_216+OPcp26_116*RLcp26_217+OPcp26_117*RLcp26_218+OPcp26_118*RLcp26_256+OPcp26_16*RLcp26_213-
 OPcp26_213*RLcp26_114-OPcp26_214*RLcp26_115-OPcp26_215*RLcp26_116-OPcp26_216*RLcp26_117-OPcp26_217*RLcp26_118-OPcp26_218*
 RLcp26_156-OPcp26_26*RLcp26_113;

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp26_156;
sens->P[2] = POcp26_256;
sens->P[3] = POcp26_356;
sens->R[1][1] = ROcp26_118;
sens->R[1][2] = ROcp26_218;
sens->R[1][3] = ROcp26_318;
sens->R[2][1] = ROcp26_417;
sens->R[2][2] = ROcp26_517;
sens->R[2][3] = ROcp26_617;
sens->R[3][1] = ROcp26_718;
sens->R[3][2] = ROcp26_818;
sens->R[3][3] = ROcp26_918;
sens->V[1] = VIcp26_156;
sens->V[2] = VIcp26_256;
sens->V[3] = VIcp26_356;
sens->OM[1] = OMcp26_118;
sens->OM[2] = OMcp26_218;
sens->OM[3] = OMcp26_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp26_156_5;
sens->J[1][6] = JTcp26_156_6;
sens->J[1][13] = JTcp26_156_7;
sens->J[1][14] = JTcp26_156_8;
sens->J[1][15] = JTcp26_156_9;
sens->J[1][16] = JTcp26_156_10;
sens->J[1][17] = JTcp26_156_11;
sens->J[1][18] = JTcp26_156_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp26_256_4;
sens->J[2][5] = JTcp26_256_5;
sens->J[2][6] = JTcp26_256_6;
sens->J[2][13] = JTcp26_256_7;
sens->J[2][14] = JTcp26_256_8;
sens->J[2][15] = JTcp26_256_9;
sens->J[2][16] = JTcp26_256_10;
sens->J[2][17] = JTcp26_256_11;
sens->J[2][18] = JTcp26_256_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp26_356_4;
sens->J[3][5] = JTcp26_356_5;
sens->J[3][6] = JTcp26_356_6;
sens->J[3][13] = JTcp26_356_7;
sens->J[3][14] = JTcp26_356_8;
sens->J[3][15] = JTcp26_356_9;
sens->J[3][16] = JTcp26_356_10;
sens->J[3][17] = JTcp26_356_11;
sens->J[3][18] = JTcp26_356_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp26_46;
sens->J[4][14] = ROcp26_113;
sens->J[4][15] = ROcp26_714;
sens->J[4][16] = ROcp26_415;
sens->J[4][17] = ROcp26_116;
sens->J[4][18] = ROcp26_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp26_85;
sens->J[5][13] = ROcp26_56;
sens->J[5][14] = ROcp26_213;
sens->J[5][15] = ROcp26_814;
sens->J[5][16] = ROcp26_515;
sens->J[5][17] = ROcp26_216;
sens->J[5][18] = ROcp26_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp26_95;
sens->J[6][13] = ROcp26_66;
sens->J[6][14] = ROcp26_313;
sens->J[6][15] = ROcp26_914;
sens->J[6][16] = ROcp26_615;
sens->J[6][17] = ROcp26_316;
sens->J[6][18] = ROcp26_617;
sens->A[1] = ACcp26_156;
sens->A[2] = ACcp26_256;
sens->A[3] = ACcp26_356;
sens->OMP[1] = OPcp26_118;
sens->OMP[2] = OPcp26_218;
sens->OMP[3] = OPcp26_318;
 
// 
break;
case 28:
 


// = = Block_1_0_0_28_0_1 = = 
 
// Sensor Kinematics 


ROcp27_25 = S4*S5;
ROcp27_35 = -C4*S5;
ROcp27_85 = -S4*C5;
ROcp27_95 = C4*C5;
ROcp27_16 = C5*C6;
ROcp27_26 = ROcp27_25*C6+C4*S6;
ROcp27_36 = ROcp27_35*C6+S4*S6;
ROcp27_46 = -C5*S6;
ROcp27_56 = -(ROcp27_25*S6-C4*C6);
ROcp27_66 = -(ROcp27_35*S6-S4*C6);
OMcp27_25 = qd[5]*C4;
OMcp27_35 = qd[5]*S4;
OMcp27_16 = qd[4]+qd[6]*S5;
OMcp27_26 = OMcp27_25+ROcp27_85*qd[6];
OMcp27_36 = OMcp27_35+ROcp27_95*qd[6];
OPcp27_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp27_26 = ROcp27_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp27_35*S5-ROcp27_95*qd[4]);
OPcp27_36 = ROcp27_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp27_25*S5-ROcp27_85*qd[4]);

// = = Block_1_0_0_28_0_4 = = 
 
// Sensor Kinematics 


ROcp27_419 = ROcp27_46*C19+S19*S5;
ROcp27_519 = ROcp27_56*C19+ROcp27_85*S19;
ROcp27_619 = ROcp27_66*C19+ROcp27_95*S19;
ROcp27_719 = -(ROcp27_46*S19-C19*S5);
ROcp27_819 = -(ROcp27_56*S19-ROcp27_85*C19);
ROcp27_919 = -(ROcp27_66*S19-ROcp27_95*C19);
RLcp27_119 = s->dpt[2][3]*ROcp27_46+ROcp27_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp27_219 = s->dpt[2][3]*ROcp27_56+ROcp27_26*s->dpt[1][3]+ROcp27_85*s->dpt[3][3];
RLcp27_319 = s->dpt[2][3]*ROcp27_66+ROcp27_36*s->dpt[1][3]+ROcp27_95*s->dpt[3][3];
OMcp27_119 = OMcp27_16+ROcp27_16*qd[19];
OMcp27_219 = OMcp27_26+ROcp27_26*qd[19];
OMcp27_319 = OMcp27_36+ROcp27_36*qd[19];
ORcp27_119 = OMcp27_26*RLcp27_319-OMcp27_36*RLcp27_219;
ORcp27_219 = -(OMcp27_16*RLcp27_319-OMcp27_36*RLcp27_119);
ORcp27_319 = OMcp27_16*RLcp27_219-OMcp27_26*RLcp27_119;
OPcp27_119 = OPcp27_16+ROcp27_16*qdd[19]+qd[19]*(OMcp27_26*ROcp27_36-OMcp27_36*ROcp27_26);
OPcp27_219 = OPcp27_26+ROcp27_26*qdd[19]-qd[19]*(OMcp27_16*ROcp27_36-OMcp27_36*ROcp27_16);
OPcp27_319 = OPcp27_36+ROcp27_36*qdd[19]+qd[19]*(OMcp27_16*ROcp27_26-OMcp27_26*ROcp27_16);
RLcp27_157 = s->dpt[1][38]*ROcp27_16+s->dpt[2][38]*ROcp27_419+s->dpt[3][38]*ROcp27_719;
RLcp27_257 = s->dpt[1][38]*ROcp27_26+s->dpt[2][38]*ROcp27_519+s->dpt[3][38]*ROcp27_819;
RLcp27_357 = s->dpt[1][38]*ROcp27_36+s->dpt[2][38]*ROcp27_619+s->dpt[3][38]*ROcp27_919;
POcp27_157 = RLcp27_119+RLcp27_157+q[1];
POcp27_257 = RLcp27_219+RLcp27_257+q[2];
POcp27_357 = RLcp27_319+RLcp27_357+q[3];
JTcp27_257_4 = -(RLcp27_319+RLcp27_357);
JTcp27_357_4 = RLcp27_219+RLcp27_257;
JTcp27_157_5 = C4*(RLcp27_319+RLcp27_357)-S4*(RLcp27_219+RLcp27_257);
JTcp27_257_5 = S4*(RLcp27_119+RLcp27_157);
JTcp27_357_5 = -C4*(RLcp27_119+RLcp27_157);
JTcp27_157_6 = ROcp27_85*(RLcp27_319+RLcp27_357)-ROcp27_95*(RLcp27_219+RLcp27_257);
JTcp27_257_6 = ROcp27_95*(RLcp27_119+RLcp27_157)-S5*(RLcp27_319+RLcp27_357);
JTcp27_357_6 = -(ROcp27_85*(RLcp27_119+RLcp27_157)-S5*(RLcp27_219+RLcp27_257));
JTcp27_157_7 = -(RLcp27_257*ROcp27_36-RLcp27_357*ROcp27_26);
JTcp27_257_7 = RLcp27_157*ROcp27_36-RLcp27_357*ROcp27_16;
JTcp27_357_7 = -(RLcp27_157*ROcp27_26-RLcp27_257*ROcp27_16);
ORcp27_157 = OMcp27_219*RLcp27_357-OMcp27_319*RLcp27_257;
ORcp27_257 = -(OMcp27_119*RLcp27_357-OMcp27_319*RLcp27_157);
ORcp27_357 = OMcp27_119*RLcp27_257-OMcp27_219*RLcp27_157;
VIcp27_157 = ORcp27_119+ORcp27_157+qd[1];
VIcp27_257 = ORcp27_219+ORcp27_257+qd[2];
VIcp27_357 = ORcp27_319+ORcp27_357+qd[3];
ACcp27_157 = qdd[1]+OMcp27_219*ORcp27_357+OMcp27_26*ORcp27_319-OMcp27_319*ORcp27_257-OMcp27_36*ORcp27_219+OPcp27_219*
 RLcp27_357+OPcp27_26*RLcp27_319-OPcp27_319*RLcp27_257-OPcp27_36*RLcp27_219;
ACcp27_257 = qdd[2]-OMcp27_119*ORcp27_357-OMcp27_16*ORcp27_319+OMcp27_319*ORcp27_157+OMcp27_36*ORcp27_119-OPcp27_119*
 RLcp27_357-OPcp27_16*RLcp27_319+OPcp27_319*RLcp27_157+OPcp27_36*RLcp27_119;
ACcp27_357 = qdd[3]+OMcp27_119*ORcp27_257+OMcp27_16*ORcp27_219-OMcp27_219*ORcp27_157-OMcp27_26*ORcp27_119+OPcp27_119*
 RLcp27_257+OPcp27_16*RLcp27_219-OPcp27_219*RLcp27_157-OPcp27_26*RLcp27_119;

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp27_157;
sens->P[2] = POcp27_257;
sens->P[3] = POcp27_357;
sens->R[1][1] = ROcp27_16;
sens->R[1][2] = ROcp27_26;
sens->R[1][3] = ROcp27_36;
sens->R[2][1] = ROcp27_419;
sens->R[2][2] = ROcp27_519;
sens->R[2][3] = ROcp27_619;
sens->R[3][1] = ROcp27_719;
sens->R[3][2] = ROcp27_819;
sens->R[3][3] = ROcp27_919;
sens->V[1] = VIcp27_157;
sens->V[2] = VIcp27_257;
sens->V[3] = VIcp27_357;
sens->OM[1] = OMcp27_119;
sens->OM[2] = OMcp27_219;
sens->OM[3] = OMcp27_319;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp27_157_5;
sens->J[1][6] = JTcp27_157_6;
sens->J[1][19] = JTcp27_157_7;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp27_257_4;
sens->J[2][5] = JTcp27_257_5;
sens->J[2][6] = JTcp27_257_6;
sens->J[2][19] = JTcp27_257_7;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp27_357_4;
sens->J[3][5] = JTcp27_357_5;
sens->J[3][6] = JTcp27_357_6;
sens->J[3][19] = JTcp27_357_7;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp27_16;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp27_85;
sens->J[5][19] = ROcp27_26;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp27_95;
sens->J[6][19] = ROcp27_36;
sens->A[1] = ACcp27_157;
sens->A[2] = ACcp27_257;
sens->A[3] = ACcp27_357;
sens->OMP[1] = OPcp27_119;
sens->OMP[2] = OPcp27_219;
sens->OMP[3] = OPcp27_319;
 
// 
break;
case 29:
 


// = = Block_1_0_0_29_0_1 = = 
 
// Sensor Kinematics 


ROcp28_25 = S4*S5;
ROcp28_35 = -C4*S5;
ROcp28_85 = -S4*C5;
ROcp28_95 = C4*C5;
ROcp28_16 = C5*C6;
ROcp28_26 = ROcp28_25*C6+C4*S6;
ROcp28_36 = ROcp28_35*C6+S4*S6;
ROcp28_46 = -C5*S6;
ROcp28_56 = -(ROcp28_25*S6-C4*C6);
ROcp28_66 = -(ROcp28_35*S6-S4*C6);
OMcp28_25 = qd[5]*C4;
OMcp28_35 = qd[5]*S4;
OMcp28_16 = qd[4]+qd[6]*S5;
OMcp28_26 = OMcp28_25+ROcp28_85*qd[6];
OMcp28_36 = OMcp28_35+ROcp28_95*qd[6];
OPcp28_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp28_26 = ROcp28_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp28_35*S5-ROcp28_95*qd[4]);
OPcp28_36 = ROcp28_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp28_25*S5-ROcp28_85*qd[4]);

// = = Block_1_0_0_29_0_4 = = 
 
// Sensor Kinematics 


ROcp28_419 = ROcp28_46*C19+S19*S5;
ROcp28_519 = ROcp28_56*C19+ROcp28_85*S19;
ROcp28_619 = ROcp28_66*C19+ROcp28_95*S19;
ROcp28_719 = -(ROcp28_46*S19-C19*S5);
ROcp28_819 = -(ROcp28_56*S19-ROcp28_85*C19);
ROcp28_919 = -(ROcp28_66*S19-ROcp28_95*C19);
RLcp28_119 = s->dpt[2][3]*ROcp28_46+ROcp28_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp28_219 = s->dpt[2][3]*ROcp28_56+ROcp28_26*s->dpt[1][3]+ROcp28_85*s->dpt[3][3];
RLcp28_319 = s->dpt[2][3]*ROcp28_66+ROcp28_36*s->dpt[1][3]+ROcp28_95*s->dpt[3][3];
OMcp28_119 = OMcp28_16+ROcp28_16*qd[19];
OMcp28_219 = OMcp28_26+ROcp28_26*qd[19];
OMcp28_319 = OMcp28_36+ROcp28_36*qd[19];
ORcp28_119 = OMcp28_26*RLcp28_319-OMcp28_36*RLcp28_219;
ORcp28_219 = -(OMcp28_16*RLcp28_319-OMcp28_36*RLcp28_119);
ORcp28_319 = OMcp28_16*RLcp28_219-OMcp28_26*RLcp28_119;
OPcp28_119 = OPcp28_16+ROcp28_16*qdd[19]+qd[19]*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26);
OPcp28_219 = OPcp28_26+ROcp28_26*qdd[19]-qd[19]*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16);
OPcp28_319 = OPcp28_36+ROcp28_36*qdd[19]+qd[19]*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16);
RLcp28_158 = ROcp28_16*s->dpt[1][39]+ROcp28_419*s->dpt[2][39]+ROcp28_719*s->dpt[3][39];
RLcp28_258 = ROcp28_26*s->dpt[1][39]+ROcp28_519*s->dpt[2][39]+ROcp28_819*s->dpt[3][39];
RLcp28_358 = ROcp28_36*s->dpt[1][39]+ROcp28_619*s->dpt[2][39]+ROcp28_919*s->dpt[3][39];
POcp28_158 = RLcp28_119+RLcp28_158+q[1];
POcp28_258 = RLcp28_219+RLcp28_258+q[2];
POcp28_358 = RLcp28_319+RLcp28_358+q[3];
JTcp28_258_4 = -(RLcp28_319+RLcp28_358);
JTcp28_358_4 = RLcp28_219+RLcp28_258;
JTcp28_158_5 = C4*(RLcp28_319+RLcp28_358)-S4*(RLcp28_219+RLcp28_258);
JTcp28_258_5 = S4*(RLcp28_119+RLcp28_158);
JTcp28_358_5 = -C4*(RLcp28_119+RLcp28_158);
JTcp28_158_6 = ROcp28_85*(RLcp28_319+RLcp28_358)-ROcp28_95*(RLcp28_219+RLcp28_258);
JTcp28_258_6 = ROcp28_95*(RLcp28_119+RLcp28_158)-S5*(RLcp28_319+RLcp28_358);
JTcp28_358_6 = -(ROcp28_85*(RLcp28_119+RLcp28_158)-S5*(RLcp28_219+RLcp28_258));
JTcp28_158_7 = -(RLcp28_258*ROcp28_36-RLcp28_358*ROcp28_26);
JTcp28_258_7 = RLcp28_158*ROcp28_36-RLcp28_358*ROcp28_16;
JTcp28_358_7 = -(RLcp28_158*ROcp28_26-RLcp28_258*ROcp28_16);
ORcp28_158 = OMcp28_219*RLcp28_358-OMcp28_319*RLcp28_258;
ORcp28_258 = -(OMcp28_119*RLcp28_358-OMcp28_319*RLcp28_158);
ORcp28_358 = OMcp28_119*RLcp28_258-OMcp28_219*RLcp28_158;
VIcp28_158 = ORcp28_119+ORcp28_158+qd[1];
VIcp28_258 = ORcp28_219+ORcp28_258+qd[2];
VIcp28_358 = ORcp28_319+ORcp28_358+qd[3];
ACcp28_158 = qdd[1]+OMcp28_219*ORcp28_358+OMcp28_26*ORcp28_319-OMcp28_319*ORcp28_258-OMcp28_36*ORcp28_219+OPcp28_219*
 RLcp28_358+OPcp28_26*RLcp28_319-OPcp28_319*RLcp28_258-OPcp28_36*RLcp28_219;
ACcp28_258 = qdd[2]-OMcp28_119*ORcp28_358-OMcp28_16*ORcp28_319+OMcp28_319*ORcp28_158+OMcp28_36*ORcp28_119-OPcp28_119*
 RLcp28_358-OPcp28_16*RLcp28_319+OPcp28_319*RLcp28_158+OPcp28_36*RLcp28_119;
ACcp28_358 = qdd[3]+OMcp28_119*ORcp28_258+OMcp28_16*ORcp28_219-OMcp28_219*ORcp28_158-OMcp28_26*ORcp28_119+OPcp28_119*
 RLcp28_258+OPcp28_16*RLcp28_219-OPcp28_219*RLcp28_158-OPcp28_26*RLcp28_119;

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp28_158;
sens->P[2] = POcp28_258;
sens->P[3] = POcp28_358;
sens->R[1][1] = ROcp28_16;
sens->R[1][2] = ROcp28_26;
sens->R[1][3] = ROcp28_36;
sens->R[2][1] = ROcp28_419;
sens->R[2][2] = ROcp28_519;
sens->R[2][3] = ROcp28_619;
sens->R[3][1] = ROcp28_719;
sens->R[3][2] = ROcp28_819;
sens->R[3][3] = ROcp28_919;
sens->V[1] = VIcp28_158;
sens->V[2] = VIcp28_258;
sens->V[3] = VIcp28_358;
sens->OM[1] = OMcp28_119;
sens->OM[2] = OMcp28_219;
sens->OM[3] = OMcp28_319;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp28_158_5;
sens->J[1][6] = JTcp28_158_6;
sens->J[1][19] = JTcp28_158_7;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp28_258_4;
sens->J[2][5] = JTcp28_258_5;
sens->J[2][6] = JTcp28_258_6;
sens->J[2][19] = JTcp28_258_7;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp28_358_4;
sens->J[3][5] = JTcp28_358_5;
sens->J[3][6] = JTcp28_358_6;
sens->J[3][19] = JTcp28_358_7;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp28_16;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp28_85;
sens->J[5][19] = ROcp28_26;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp28_95;
sens->J[6][19] = ROcp28_36;
sens->A[1] = ACcp28_158;
sens->A[2] = ACcp28_258;
sens->A[3] = ACcp28_358;
sens->OMP[1] = OPcp28_119;
sens->OMP[2] = OPcp28_219;
sens->OMP[3] = OPcp28_319;
 
// 
break;
case 30:
 


// = = Block_1_0_0_30_0_1 = = 
 
// Sensor Kinematics 


ROcp29_25 = S4*S5;
ROcp29_35 = -C4*S5;
ROcp29_85 = -S4*C5;
ROcp29_95 = C4*C5;
ROcp29_16 = C5*C6;
ROcp29_26 = ROcp29_25*C6+C4*S6;
ROcp29_36 = ROcp29_35*C6+S4*S6;
ROcp29_46 = -C5*S6;
ROcp29_56 = -(ROcp29_25*S6-C4*C6);
ROcp29_66 = -(ROcp29_35*S6-S4*C6);
OMcp29_25 = qd[5]*C4;
OMcp29_35 = qd[5]*S4;
OMcp29_16 = qd[4]+qd[6]*S5;
OMcp29_26 = OMcp29_25+ROcp29_85*qd[6];
OMcp29_36 = OMcp29_35+ROcp29_95*qd[6];
OPcp29_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp29_26 = ROcp29_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp29_35*S5-ROcp29_95*qd[4]);
OPcp29_36 = ROcp29_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp29_25*S5-ROcp29_85*qd[4]);

// = = Block_1_0_0_30_0_4 = = 
 
// Sensor Kinematics 


ROcp29_419 = ROcp29_46*C19+S19*S5;
ROcp29_519 = ROcp29_56*C19+ROcp29_85*S19;
ROcp29_619 = ROcp29_66*C19+ROcp29_95*S19;
ROcp29_719 = -(ROcp29_46*S19-C19*S5);
ROcp29_819 = -(ROcp29_56*S19-ROcp29_85*C19);
ROcp29_919 = -(ROcp29_66*S19-ROcp29_95*C19);
ROcp29_120 = ROcp29_16*C20-ROcp29_719*S20;
ROcp29_220 = ROcp29_26*C20-ROcp29_819*S20;
ROcp29_320 = ROcp29_36*C20-ROcp29_919*S20;
ROcp29_720 = ROcp29_16*S20+ROcp29_719*C20;
ROcp29_820 = ROcp29_26*S20+ROcp29_819*C20;
ROcp29_920 = ROcp29_36*S20+ROcp29_919*C20;
RLcp29_119 = s->dpt[2][3]*ROcp29_46+ROcp29_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp29_219 = s->dpt[2][3]*ROcp29_56+ROcp29_26*s->dpt[1][3]+ROcp29_85*s->dpt[3][3];
RLcp29_319 = s->dpt[2][3]*ROcp29_66+ROcp29_36*s->dpt[1][3]+ROcp29_95*s->dpt[3][3];
OMcp29_119 = OMcp29_16+ROcp29_16*qd[19];
OMcp29_219 = OMcp29_26+ROcp29_26*qd[19];
OMcp29_319 = OMcp29_36+ROcp29_36*qd[19];
ORcp29_119 = OMcp29_26*RLcp29_319-OMcp29_36*RLcp29_219;
ORcp29_219 = -(OMcp29_16*RLcp29_319-OMcp29_36*RLcp29_119);
ORcp29_319 = OMcp29_16*RLcp29_219-OMcp29_26*RLcp29_119;
OPcp29_119 = OPcp29_16+ROcp29_16*qdd[19]+qd[19]*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26);
OPcp29_219 = OPcp29_26+ROcp29_26*qdd[19]-qd[19]*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16);
OPcp29_319 = OPcp29_36+ROcp29_36*qdd[19]+qd[19]*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16);
RLcp29_120 = s->dpt[1][38]*ROcp29_16+s->dpt[2][38]*ROcp29_419+s->dpt[3][38]*ROcp29_719;
RLcp29_220 = s->dpt[1][38]*ROcp29_26+s->dpt[2][38]*ROcp29_519+s->dpt[3][38]*ROcp29_819;
RLcp29_320 = s->dpt[1][38]*ROcp29_36+s->dpt[2][38]*ROcp29_619+s->dpt[3][38]*ROcp29_919;
OMcp29_120 = OMcp29_119+ROcp29_419*qd[20];
OMcp29_220 = OMcp29_219+ROcp29_519*qd[20];
OMcp29_320 = OMcp29_319+ROcp29_619*qd[20];
ORcp29_120 = OMcp29_219*RLcp29_320-OMcp29_319*RLcp29_220;
ORcp29_220 = -(OMcp29_119*RLcp29_320-OMcp29_319*RLcp29_120);
ORcp29_320 = OMcp29_119*RLcp29_220-OMcp29_219*RLcp29_120;
OPcp29_120 = OPcp29_119+ROcp29_419*qdd[20]+qd[20]*(OMcp29_219*ROcp29_619-OMcp29_319*ROcp29_519);
OPcp29_220 = OPcp29_219+ROcp29_519*qdd[20]-qd[20]*(OMcp29_119*ROcp29_619-OMcp29_319*ROcp29_419);
OPcp29_320 = OPcp29_319+ROcp29_619*qdd[20]+qd[20]*(OMcp29_119*ROcp29_519-OMcp29_219*ROcp29_419);
RLcp29_159 = s->dpt[1][40]*ROcp29_120+s->dpt[2][40]*ROcp29_419+ROcp29_720*s->dpt[3][40];
RLcp29_259 = s->dpt[1][40]*ROcp29_220+s->dpt[2][40]*ROcp29_519+ROcp29_820*s->dpt[3][40];
RLcp29_359 = s->dpt[1][40]*ROcp29_320+s->dpt[2][40]*ROcp29_619+ROcp29_920*s->dpt[3][40];
POcp29_159 = RLcp29_119+RLcp29_120+RLcp29_159+q[1];
POcp29_259 = RLcp29_219+RLcp29_220+RLcp29_259+q[2];
POcp29_359 = RLcp29_319+RLcp29_320+RLcp29_359+q[3];
JTcp29_259_4 = -(RLcp29_319+RLcp29_320+RLcp29_359);
JTcp29_359_4 = RLcp29_219+RLcp29_220+RLcp29_259;
JTcp29_159_5 = C4*(RLcp29_319+RLcp29_320)-S4*(RLcp29_219+RLcp29_220)-RLcp29_259*S4+RLcp29_359*C4;
JTcp29_259_5 = S4*(RLcp29_119+RLcp29_120+RLcp29_159);
JTcp29_359_5 = -C4*(RLcp29_119+RLcp29_120+RLcp29_159);
JTcp29_159_6 = ROcp29_85*(RLcp29_319+RLcp29_320)-ROcp29_95*(RLcp29_219+RLcp29_220)-RLcp29_259*ROcp29_95+RLcp29_359*
 ROcp29_85;
JTcp29_259_6 = -(RLcp29_359*S5-ROcp29_95*(RLcp29_119+RLcp29_120+RLcp29_159)+S5*(RLcp29_319+RLcp29_320));
JTcp29_359_6 = RLcp29_259*S5-ROcp29_85*(RLcp29_119+RLcp29_120+RLcp29_159)+S5*(RLcp29_219+RLcp29_220);
JTcp29_159_7 = ROcp29_26*(RLcp29_320+RLcp29_359)-ROcp29_36*(RLcp29_220+RLcp29_259);
JTcp29_259_7 = -(ROcp29_16*(RLcp29_320+RLcp29_359)-ROcp29_36*(RLcp29_120+RLcp29_159));
JTcp29_359_7 = ROcp29_16*(RLcp29_220+RLcp29_259)-ROcp29_26*(RLcp29_120+RLcp29_159);
JTcp29_159_8 = -(RLcp29_259*ROcp29_619-RLcp29_359*ROcp29_519);
JTcp29_259_8 = RLcp29_159*ROcp29_619-RLcp29_359*ROcp29_419;
JTcp29_359_8 = -(RLcp29_159*ROcp29_519-RLcp29_259*ROcp29_419);
ORcp29_159 = OMcp29_220*RLcp29_359-OMcp29_320*RLcp29_259;
ORcp29_259 = -(OMcp29_120*RLcp29_359-OMcp29_320*RLcp29_159);
ORcp29_359 = OMcp29_120*RLcp29_259-OMcp29_220*RLcp29_159;
VIcp29_159 = ORcp29_119+ORcp29_120+ORcp29_159+qd[1];
VIcp29_259 = ORcp29_219+ORcp29_220+ORcp29_259+qd[2];
VIcp29_359 = ORcp29_319+ORcp29_320+ORcp29_359+qd[3];
ACcp29_159 = qdd[1]+OMcp29_219*ORcp29_320+OMcp29_220*ORcp29_359+OMcp29_26*ORcp29_319-OMcp29_319*ORcp29_220-OMcp29_320*
 ORcp29_259-OMcp29_36*ORcp29_219+OPcp29_219*RLcp29_320+OPcp29_220*RLcp29_359+OPcp29_26*RLcp29_319-OPcp29_319*RLcp29_220-
 OPcp29_320*RLcp29_259-OPcp29_36*RLcp29_219;
ACcp29_259 = qdd[2]-OMcp29_119*ORcp29_320-OMcp29_120*ORcp29_359-OMcp29_16*ORcp29_319+OMcp29_319*ORcp29_120+OMcp29_320*
 ORcp29_159+OMcp29_36*ORcp29_119-OPcp29_119*RLcp29_320-OPcp29_120*RLcp29_359-OPcp29_16*RLcp29_319+OPcp29_319*RLcp29_120+
 OPcp29_320*RLcp29_159+OPcp29_36*RLcp29_119;
ACcp29_359 = qdd[3]+OMcp29_119*ORcp29_220+OMcp29_120*ORcp29_259+OMcp29_16*ORcp29_219-OMcp29_219*ORcp29_120-OMcp29_220*
 ORcp29_159-OMcp29_26*ORcp29_119+OPcp29_119*RLcp29_220+OPcp29_120*RLcp29_259+OPcp29_16*RLcp29_219-OPcp29_219*RLcp29_120-
 OPcp29_220*RLcp29_159-OPcp29_26*RLcp29_119;

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp29_159;
sens->P[2] = POcp29_259;
sens->P[3] = POcp29_359;
sens->R[1][1] = ROcp29_120;
sens->R[1][2] = ROcp29_220;
sens->R[1][3] = ROcp29_320;
sens->R[2][1] = ROcp29_419;
sens->R[2][2] = ROcp29_519;
sens->R[2][3] = ROcp29_619;
sens->R[3][1] = ROcp29_720;
sens->R[3][2] = ROcp29_820;
sens->R[3][3] = ROcp29_920;
sens->V[1] = VIcp29_159;
sens->V[2] = VIcp29_259;
sens->V[3] = VIcp29_359;
sens->OM[1] = OMcp29_120;
sens->OM[2] = OMcp29_220;
sens->OM[3] = OMcp29_320;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp29_159_5;
sens->J[1][6] = JTcp29_159_6;
sens->J[1][19] = JTcp29_159_7;
sens->J[1][20] = JTcp29_159_8;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp29_259_4;
sens->J[2][5] = JTcp29_259_5;
sens->J[2][6] = JTcp29_259_6;
sens->J[2][19] = JTcp29_259_7;
sens->J[2][20] = JTcp29_259_8;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp29_359_4;
sens->J[3][5] = JTcp29_359_5;
sens->J[3][6] = JTcp29_359_6;
sens->J[3][19] = JTcp29_359_7;
sens->J[3][20] = JTcp29_359_8;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp29_16;
sens->J[4][20] = ROcp29_419;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp29_85;
sens->J[5][19] = ROcp29_26;
sens->J[5][20] = ROcp29_519;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp29_95;
sens->J[6][19] = ROcp29_36;
sens->J[6][20] = ROcp29_619;
sens->A[1] = ACcp29_159;
sens->A[2] = ACcp29_259;
sens->A[3] = ACcp29_359;
sens->OMP[1] = OPcp29_120;
sens->OMP[2] = OPcp29_220;
sens->OMP[3] = OPcp29_320;
 
// 
break;
case 31:
 


// = = Block_1_0_0_31_0_1 = = 
 
// Sensor Kinematics 


ROcp30_25 = S4*S5;
ROcp30_35 = -C4*S5;
ROcp30_85 = -S4*C5;
ROcp30_95 = C4*C5;
ROcp30_16 = C5*C6;
ROcp30_26 = ROcp30_25*C6+C4*S6;
ROcp30_36 = ROcp30_35*C6+S4*S6;
ROcp30_46 = -C5*S6;
ROcp30_56 = -(ROcp30_25*S6-C4*C6);
ROcp30_66 = -(ROcp30_35*S6-S4*C6);
OMcp30_25 = qd[5]*C4;
OMcp30_35 = qd[5]*S4;
OMcp30_16 = qd[4]+qd[6]*S5;
OMcp30_26 = OMcp30_25+ROcp30_85*qd[6];
OMcp30_36 = OMcp30_35+ROcp30_95*qd[6];
OPcp30_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp30_26 = ROcp30_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp30_35*S5-ROcp30_95*qd[4]);
OPcp30_36 = ROcp30_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp30_25*S5-ROcp30_85*qd[4]);

// = = Block_1_0_0_31_0_4 = = 
 
// Sensor Kinematics 


ROcp30_419 = ROcp30_46*C19+S19*S5;
ROcp30_519 = ROcp30_56*C19+ROcp30_85*S19;
ROcp30_619 = ROcp30_66*C19+ROcp30_95*S19;
ROcp30_719 = -(ROcp30_46*S19-C19*S5);
ROcp30_819 = -(ROcp30_56*S19-ROcp30_85*C19);
ROcp30_919 = -(ROcp30_66*S19-ROcp30_95*C19);
ROcp30_120 = ROcp30_16*C20-ROcp30_719*S20;
ROcp30_220 = ROcp30_26*C20-ROcp30_819*S20;
ROcp30_320 = ROcp30_36*C20-ROcp30_919*S20;
ROcp30_720 = ROcp30_16*S20+ROcp30_719*C20;
ROcp30_820 = ROcp30_26*S20+ROcp30_819*C20;
ROcp30_920 = ROcp30_36*S20+ROcp30_919*C20;
RLcp30_119 = s->dpt[2][3]*ROcp30_46+ROcp30_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp30_219 = s->dpt[2][3]*ROcp30_56+ROcp30_26*s->dpt[1][3]+ROcp30_85*s->dpt[3][3];
RLcp30_319 = s->dpt[2][3]*ROcp30_66+ROcp30_36*s->dpt[1][3]+ROcp30_95*s->dpt[3][3];
OMcp30_119 = OMcp30_16+ROcp30_16*qd[19];
OMcp30_219 = OMcp30_26+ROcp30_26*qd[19];
OMcp30_319 = OMcp30_36+ROcp30_36*qd[19];
ORcp30_119 = OMcp30_26*RLcp30_319-OMcp30_36*RLcp30_219;
ORcp30_219 = -(OMcp30_16*RLcp30_319-OMcp30_36*RLcp30_119);
ORcp30_319 = OMcp30_16*RLcp30_219-OMcp30_26*RLcp30_119;
OPcp30_119 = OPcp30_16+ROcp30_16*qdd[19]+qd[19]*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26);
OPcp30_219 = OPcp30_26+ROcp30_26*qdd[19]-qd[19]*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16);
OPcp30_319 = OPcp30_36+ROcp30_36*qdd[19]+qd[19]*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16);
RLcp30_120 = s->dpt[1][38]*ROcp30_16+s->dpt[2][38]*ROcp30_419+s->dpt[3][38]*ROcp30_719;
RLcp30_220 = s->dpt[1][38]*ROcp30_26+s->dpt[2][38]*ROcp30_519+s->dpt[3][38]*ROcp30_819;
RLcp30_320 = s->dpt[1][38]*ROcp30_36+s->dpt[2][38]*ROcp30_619+s->dpt[3][38]*ROcp30_919;
OMcp30_120 = OMcp30_119+ROcp30_419*qd[20];
OMcp30_220 = OMcp30_219+ROcp30_519*qd[20];
OMcp30_320 = OMcp30_319+ROcp30_619*qd[20];
ORcp30_120 = OMcp30_219*RLcp30_320-OMcp30_319*RLcp30_220;
ORcp30_220 = -(OMcp30_119*RLcp30_320-OMcp30_319*RLcp30_120);
ORcp30_320 = OMcp30_119*RLcp30_220-OMcp30_219*RLcp30_120;
OPcp30_120 = OPcp30_119+ROcp30_419*qdd[20]+qd[20]*(OMcp30_219*ROcp30_619-OMcp30_319*ROcp30_519);
OPcp30_220 = OPcp30_219+ROcp30_519*qdd[20]-qd[20]*(OMcp30_119*ROcp30_619-OMcp30_319*ROcp30_419);
OPcp30_320 = OPcp30_319+ROcp30_619*qdd[20]+qd[20]*(OMcp30_119*ROcp30_519-OMcp30_219*ROcp30_419);
RLcp30_160 = ROcp30_120*s->dpt[1][41]+ROcp30_419*s->dpt[2][41]+ROcp30_720*s->dpt[3][41];
RLcp30_260 = ROcp30_220*s->dpt[1][41]+ROcp30_519*s->dpt[2][41]+ROcp30_820*s->dpt[3][41];
RLcp30_360 = ROcp30_320*s->dpt[1][41]+ROcp30_619*s->dpt[2][41]+ROcp30_920*s->dpt[3][41];
POcp30_160 = RLcp30_119+RLcp30_120+RLcp30_160+q[1];
POcp30_260 = RLcp30_219+RLcp30_220+RLcp30_260+q[2];
POcp30_360 = RLcp30_319+RLcp30_320+RLcp30_360+q[3];
JTcp30_260_4 = -(RLcp30_319+RLcp30_320+RLcp30_360);
JTcp30_360_4 = RLcp30_219+RLcp30_220+RLcp30_260;
JTcp30_160_5 = C4*(RLcp30_319+RLcp30_320)-S4*(RLcp30_219+RLcp30_220)-RLcp30_260*S4+RLcp30_360*C4;
JTcp30_260_5 = S4*(RLcp30_119+RLcp30_120+RLcp30_160);
JTcp30_360_5 = -C4*(RLcp30_119+RLcp30_120+RLcp30_160);
JTcp30_160_6 = ROcp30_85*(RLcp30_319+RLcp30_320)-ROcp30_95*(RLcp30_219+RLcp30_220)-RLcp30_260*ROcp30_95+RLcp30_360*
 ROcp30_85;
JTcp30_260_6 = -(RLcp30_360*S5-ROcp30_95*(RLcp30_119+RLcp30_120+RLcp30_160)+S5*(RLcp30_319+RLcp30_320));
JTcp30_360_6 = RLcp30_260*S5-ROcp30_85*(RLcp30_119+RLcp30_120+RLcp30_160)+S5*(RLcp30_219+RLcp30_220);
JTcp30_160_7 = ROcp30_26*(RLcp30_320+RLcp30_360)-ROcp30_36*(RLcp30_220+RLcp30_260);
JTcp30_260_7 = -(ROcp30_16*(RLcp30_320+RLcp30_360)-ROcp30_36*(RLcp30_120+RLcp30_160));
JTcp30_360_7 = ROcp30_16*(RLcp30_220+RLcp30_260)-ROcp30_26*(RLcp30_120+RLcp30_160);
JTcp30_160_8 = -(RLcp30_260*ROcp30_619-RLcp30_360*ROcp30_519);
JTcp30_260_8 = RLcp30_160*ROcp30_619-RLcp30_360*ROcp30_419;
JTcp30_360_8 = -(RLcp30_160*ROcp30_519-RLcp30_260*ROcp30_419);
ORcp30_160 = OMcp30_220*RLcp30_360-OMcp30_320*RLcp30_260;
ORcp30_260 = -(OMcp30_120*RLcp30_360-OMcp30_320*RLcp30_160);
ORcp30_360 = OMcp30_120*RLcp30_260-OMcp30_220*RLcp30_160;
VIcp30_160 = ORcp30_119+ORcp30_120+ORcp30_160+qd[1];
VIcp30_260 = ORcp30_219+ORcp30_220+ORcp30_260+qd[2];
VIcp30_360 = ORcp30_319+ORcp30_320+ORcp30_360+qd[3];
ACcp30_160 = qdd[1]+OMcp30_219*ORcp30_320+OMcp30_220*ORcp30_360+OMcp30_26*ORcp30_319-OMcp30_319*ORcp30_220-OMcp30_320*
 ORcp30_260-OMcp30_36*ORcp30_219+OPcp30_219*RLcp30_320+OPcp30_220*RLcp30_360+OPcp30_26*RLcp30_319-OPcp30_319*RLcp30_220-
 OPcp30_320*RLcp30_260-OPcp30_36*RLcp30_219;
ACcp30_260 = qdd[2]-OMcp30_119*ORcp30_320-OMcp30_120*ORcp30_360-OMcp30_16*ORcp30_319+OMcp30_319*ORcp30_120+OMcp30_320*
 ORcp30_160+OMcp30_36*ORcp30_119-OPcp30_119*RLcp30_320-OPcp30_120*RLcp30_360-OPcp30_16*RLcp30_319+OPcp30_319*RLcp30_120+
 OPcp30_320*RLcp30_160+OPcp30_36*RLcp30_119;
ACcp30_360 = qdd[3]+OMcp30_119*ORcp30_220+OMcp30_120*ORcp30_260+OMcp30_16*ORcp30_219-OMcp30_219*ORcp30_120-OMcp30_220*
 ORcp30_160-OMcp30_26*ORcp30_119+OPcp30_119*RLcp30_220+OPcp30_120*RLcp30_260+OPcp30_16*RLcp30_219-OPcp30_219*RLcp30_120-
 OPcp30_220*RLcp30_160-OPcp30_26*RLcp30_119;

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp30_160;
sens->P[2] = POcp30_260;
sens->P[3] = POcp30_360;
sens->R[1][1] = ROcp30_120;
sens->R[1][2] = ROcp30_220;
sens->R[1][3] = ROcp30_320;
sens->R[2][1] = ROcp30_419;
sens->R[2][2] = ROcp30_519;
sens->R[2][3] = ROcp30_619;
sens->R[3][1] = ROcp30_720;
sens->R[3][2] = ROcp30_820;
sens->R[3][3] = ROcp30_920;
sens->V[1] = VIcp30_160;
sens->V[2] = VIcp30_260;
sens->V[3] = VIcp30_360;
sens->OM[1] = OMcp30_120;
sens->OM[2] = OMcp30_220;
sens->OM[3] = OMcp30_320;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp30_160_5;
sens->J[1][6] = JTcp30_160_6;
sens->J[1][19] = JTcp30_160_7;
sens->J[1][20] = JTcp30_160_8;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp30_260_4;
sens->J[2][5] = JTcp30_260_5;
sens->J[2][6] = JTcp30_260_6;
sens->J[2][19] = JTcp30_260_7;
sens->J[2][20] = JTcp30_260_8;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp30_360_4;
sens->J[3][5] = JTcp30_360_5;
sens->J[3][6] = JTcp30_360_6;
sens->J[3][19] = JTcp30_360_7;
sens->J[3][20] = JTcp30_360_8;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp30_16;
sens->J[4][20] = ROcp30_419;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp30_85;
sens->J[5][19] = ROcp30_26;
sens->J[5][20] = ROcp30_519;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp30_95;
sens->J[6][19] = ROcp30_36;
sens->J[6][20] = ROcp30_619;
sens->A[1] = ACcp30_160;
sens->A[2] = ACcp30_260;
sens->A[3] = ACcp30_360;
sens->OMP[1] = OPcp30_120;
sens->OMP[2] = OPcp30_220;
sens->OMP[3] = OPcp30_320;
 
// 
break;
case 32:
 


// = = Block_1_0_0_32_0_1 = = 
 
// Sensor Kinematics 


ROcp31_25 = S4*S5;
ROcp31_35 = -C4*S5;
ROcp31_85 = -S4*C5;
ROcp31_95 = C4*C5;
ROcp31_16 = C5*C6;
ROcp31_26 = ROcp31_25*C6+C4*S6;
ROcp31_36 = ROcp31_35*C6+S4*S6;
ROcp31_46 = -C5*S6;
ROcp31_56 = -(ROcp31_25*S6-C4*C6);
ROcp31_66 = -(ROcp31_35*S6-S4*C6);
OMcp31_25 = qd[5]*C4;
OMcp31_35 = qd[5]*S4;
OMcp31_16 = qd[4]+qd[6]*S5;
OMcp31_26 = OMcp31_25+ROcp31_85*qd[6];
OMcp31_36 = OMcp31_35+ROcp31_95*qd[6];
OPcp31_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp31_26 = ROcp31_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp31_35*S5-ROcp31_95*qd[4]);
OPcp31_36 = ROcp31_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp31_25*S5-ROcp31_85*qd[4]);

// = = Block_1_0_0_32_0_4 = = 
 
// Sensor Kinematics 


ROcp31_419 = ROcp31_46*C19+S19*S5;
ROcp31_519 = ROcp31_56*C19+ROcp31_85*S19;
ROcp31_619 = ROcp31_66*C19+ROcp31_95*S19;
ROcp31_719 = -(ROcp31_46*S19-C19*S5);
ROcp31_819 = -(ROcp31_56*S19-ROcp31_85*C19);
ROcp31_919 = -(ROcp31_66*S19-ROcp31_95*C19);
ROcp31_120 = ROcp31_16*C20-ROcp31_719*S20;
ROcp31_220 = ROcp31_26*C20-ROcp31_819*S20;
ROcp31_320 = ROcp31_36*C20-ROcp31_919*S20;
ROcp31_720 = ROcp31_16*S20+ROcp31_719*C20;
ROcp31_820 = ROcp31_26*S20+ROcp31_819*C20;
ROcp31_920 = ROcp31_36*S20+ROcp31_919*C20;
ROcp31_121 = ROcp31_120*C21+ROcp31_419*S21;
ROcp31_221 = ROcp31_220*C21+ROcp31_519*S21;
ROcp31_321 = ROcp31_320*C21+ROcp31_619*S21;
ROcp31_421 = -(ROcp31_120*S21-ROcp31_419*C21);
ROcp31_521 = -(ROcp31_220*S21-ROcp31_519*C21);
ROcp31_621 = -(ROcp31_320*S21-ROcp31_619*C21);
RLcp31_119 = s->dpt[2][3]*ROcp31_46+ROcp31_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp31_219 = s->dpt[2][3]*ROcp31_56+ROcp31_26*s->dpt[1][3]+ROcp31_85*s->dpt[3][3];
RLcp31_319 = s->dpt[2][3]*ROcp31_66+ROcp31_36*s->dpt[1][3]+ROcp31_95*s->dpt[3][3];
OMcp31_119 = OMcp31_16+ROcp31_16*qd[19];
OMcp31_219 = OMcp31_26+ROcp31_26*qd[19];
OMcp31_319 = OMcp31_36+ROcp31_36*qd[19];
ORcp31_119 = OMcp31_26*RLcp31_319-OMcp31_36*RLcp31_219;
ORcp31_219 = -(OMcp31_16*RLcp31_319-OMcp31_36*RLcp31_119);
ORcp31_319 = OMcp31_16*RLcp31_219-OMcp31_26*RLcp31_119;
OPcp31_119 = OPcp31_16+ROcp31_16*qdd[19]+qd[19]*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26);
OPcp31_219 = OPcp31_26+ROcp31_26*qdd[19]-qd[19]*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16);
OPcp31_319 = OPcp31_36+ROcp31_36*qdd[19]+qd[19]*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16);
RLcp31_120 = s->dpt[1][38]*ROcp31_16+s->dpt[2][38]*ROcp31_419+s->dpt[3][38]*ROcp31_719;
RLcp31_220 = s->dpt[1][38]*ROcp31_26+s->dpt[2][38]*ROcp31_519+s->dpt[3][38]*ROcp31_819;
RLcp31_320 = s->dpt[1][38]*ROcp31_36+s->dpt[2][38]*ROcp31_619+s->dpt[3][38]*ROcp31_919;
OMcp31_120 = OMcp31_119+ROcp31_419*qd[20];
OMcp31_220 = OMcp31_219+ROcp31_519*qd[20];
OMcp31_320 = OMcp31_319+ROcp31_619*qd[20];
ORcp31_120 = OMcp31_219*RLcp31_320-OMcp31_319*RLcp31_220;
ORcp31_220 = -(OMcp31_119*RLcp31_320-OMcp31_319*RLcp31_120);
ORcp31_320 = OMcp31_119*RLcp31_220-OMcp31_219*RLcp31_120;
OPcp31_120 = OPcp31_119+ROcp31_419*qdd[20]+qd[20]*(OMcp31_219*ROcp31_619-OMcp31_319*ROcp31_519);
OPcp31_220 = OPcp31_219+ROcp31_519*qdd[20]-qd[20]*(OMcp31_119*ROcp31_619-OMcp31_319*ROcp31_419);
OPcp31_320 = OPcp31_319+ROcp31_619*qdd[20]+qd[20]*(OMcp31_119*ROcp31_519-OMcp31_219*ROcp31_419);
RLcp31_121 = s->dpt[1][40]*ROcp31_120+s->dpt[2][40]*ROcp31_419+ROcp31_720*s->dpt[3][40];
RLcp31_221 = s->dpt[1][40]*ROcp31_220+s->dpt[2][40]*ROcp31_519+ROcp31_820*s->dpt[3][40];
RLcp31_321 = s->dpt[1][40]*ROcp31_320+s->dpt[2][40]*ROcp31_619+ROcp31_920*s->dpt[3][40];
OMcp31_121 = OMcp31_120+ROcp31_720*qd[21];
OMcp31_221 = OMcp31_220+ROcp31_820*qd[21];
OMcp31_321 = OMcp31_320+ROcp31_920*qd[21];
ORcp31_121 = OMcp31_220*RLcp31_321-OMcp31_320*RLcp31_221;
ORcp31_221 = -(OMcp31_120*RLcp31_321-OMcp31_320*RLcp31_121);
ORcp31_321 = OMcp31_120*RLcp31_221-OMcp31_220*RLcp31_121;
OPcp31_121 = OPcp31_120+ROcp31_720*qdd[21]+qd[21]*(OMcp31_220*ROcp31_920-OMcp31_320*ROcp31_820);
OPcp31_221 = OPcp31_220+ROcp31_820*qdd[21]-qd[21]*(OMcp31_120*ROcp31_920-OMcp31_320*ROcp31_720);
OPcp31_321 = OPcp31_320+ROcp31_920*qdd[21]+qd[21]*(OMcp31_120*ROcp31_820-OMcp31_220*ROcp31_720);
RLcp31_161 = s->dpt[1][42]*ROcp31_121+s->dpt[2][42]*ROcp31_421+ROcp31_720*s->dpt[3][42];
RLcp31_261 = s->dpt[1][42]*ROcp31_221+s->dpt[2][42]*ROcp31_521+ROcp31_820*s->dpt[3][42];
RLcp31_361 = s->dpt[1][42]*ROcp31_321+s->dpt[2][42]*ROcp31_621+ROcp31_920*s->dpt[3][42];
POcp31_161 = RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_161+q[1];
POcp31_261 = RLcp31_219+RLcp31_220+RLcp31_221+RLcp31_261+q[2];
POcp31_361 = RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_361+q[3];
JTcp31_261_4 = -(RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_361);
JTcp31_361_4 = RLcp31_219+RLcp31_220+RLcp31_221+RLcp31_261;
JTcp31_161_5 = C4*(RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_361)-S4*(RLcp31_219+RLcp31_220)-S4*(RLcp31_221+RLcp31_261);
JTcp31_261_5 = S4*(RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_161);
JTcp31_361_5 = -C4*(RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_161);
JTcp31_161_6 = ROcp31_85*(RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_361)-ROcp31_95*(RLcp31_219+RLcp31_220)-ROcp31_95*(
 RLcp31_221+RLcp31_261);
JTcp31_261_6 = RLcp31_161*ROcp31_95-RLcp31_321*S5-RLcp31_361*S5+ROcp31_95*(RLcp31_119+RLcp31_120+RLcp31_121)-S5*(
 RLcp31_319+RLcp31_320);
JTcp31_361_6 = RLcp31_221*S5-ROcp31_85*(RLcp31_119+RLcp31_120+RLcp31_121)+S5*(RLcp31_219+RLcp31_220)-RLcp31_161*
 ROcp31_85+RLcp31_261*S5;
JTcp31_161_7 = ROcp31_26*(RLcp31_320+RLcp31_321)-ROcp31_36*(RLcp31_220+RLcp31_221)-RLcp31_261*ROcp31_36+RLcp31_361*
 ROcp31_26;
JTcp31_261_7 = RLcp31_161*ROcp31_36-RLcp31_361*ROcp31_16-ROcp31_16*(RLcp31_320+RLcp31_321)+ROcp31_36*(RLcp31_120+
 RLcp31_121);
JTcp31_361_7 = ROcp31_16*(RLcp31_220+RLcp31_221)-ROcp31_26*(RLcp31_120+RLcp31_121)-RLcp31_161*ROcp31_26+RLcp31_261*
 ROcp31_16;
JTcp31_161_8 = ROcp31_519*(RLcp31_321+RLcp31_361)-ROcp31_619*(RLcp31_221+RLcp31_261);
JTcp31_261_8 = -(ROcp31_419*(RLcp31_321+RLcp31_361)-ROcp31_619*(RLcp31_121+RLcp31_161));
JTcp31_361_8 = ROcp31_419*(RLcp31_221+RLcp31_261)-ROcp31_519*(RLcp31_121+RLcp31_161);
JTcp31_161_9 = -(RLcp31_261*ROcp31_920-RLcp31_361*ROcp31_820);
JTcp31_261_9 = RLcp31_161*ROcp31_920-RLcp31_361*ROcp31_720;
JTcp31_361_9 = -(RLcp31_161*ROcp31_820-RLcp31_261*ROcp31_720);
ORcp31_161 = OMcp31_221*RLcp31_361-OMcp31_321*RLcp31_261;
ORcp31_261 = -(OMcp31_121*RLcp31_361-OMcp31_321*RLcp31_161);
ORcp31_361 = OMcp31_121*RLcp31_261-OMcp31_221*RLcp31_161;
VIcp31_161 = ORcp31_119+ORcp31_120+ORcp31_121+ORcp31_161+qd[1];
VIcp31_261 = ORcp31_219+ORcp31_220+ORcp31_221+ORcp31_261+qd[2];
VIcp31_361 = ORcp31_319+ORcp31_320+ORcp31_321+ORcp31_361+qd[3];
ACcp31_161 = qdd[1]+OMcp31_219*ORcp31_320+OMcp31_220*ORcp31_321+OMcp31_221*ORcp31_361+OMcp31_26*ORcp31_319-OMcp31_319*
 ORcp31_220-OMcp31_320*ORcp31_221-OMcp31_321*ORcp31_261-OMcp31_36*ORcp31_219+OPcp31_219*RLcp31_320+OPcp31_220*RLcp31_321+
 OPcp31_221*RLcp31_361+OPcp31_26*RLcp31_319-OPcp31_319*RLcp31_220-OPcp31_320*RLcp31_221-OPcp31_321*RLcp31_261-OPcp31_36*
 RLcp31_219;
ACcp31_261 = qdd[2]-OMcp31_119*ORcp31_320-OMcp31_120*ORcp31_321-OMcp31_121*ORcp31_361-OMcp31_16*ORcp31_319+OMcp31_319*
 ORcp31_120+OMcp31_320*ORcp31_121+OMcp31_321*ORcp31_161+OMcp31_36*ORcp31_119-OPcp31_119*RLcp31_320-OPcp31_120*RLcp31_321-
 OPcp31_121*RLcp31_361-OPcp31_16*RLcp31_319+OPcp31_319*RLcp31_120+OPcp31_320*RLcp31_121+OPcp31_321*RLcp31_161+OPcp31_36*
 RLcp31_119;
ACcp31_361 = qdd[3]+OMcp31_119*ORcp31_220+OMcp31_120*ORcp31_221+OMcp31_121*ORcp31_261+OMcp31_16*ORcp31_219-OMcp31_219*
 ORcp31_120-OMcp31_220*ORcp31_121-OMcp31_221*ORcp31_161-OMcp31_26*ORcp31_119+OPcp31_119*RLcp31_220+OPcp31_120*RLcp31_221+
 OPcp31_121*RLcp31_261+OPcp31_16*RLcp31_219-OPcp31_219*RLcp31_120-OPcp31_220*RLcp31_121-OPcp31_221*RLcp31_161-OPcp31_26*
 RLcp31_119;

// = = Block_1_0_0_32_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp31_161;
sens->P[2] = POcp31_261;
sens->P[3] = POcp31_361;
sens->R[1][1] = ROcp31_121;
sens->R[1][2] = ROcp31_221;
sens->R[1][3] = ROcp31_321;
sens->R[2][1] = ROcp31_421;
sens->R[2][2] = ROcp31_521;
sens->R[2][3] = ROcp31_621;
sens->R[3][1] = ROcp31_720;
sens->R[3][2] = ROcp31_820;
sens->R[3][3] = ROcp31_920;
sens->V[1] = VIcp31_161;
sens->V[2] = VIcp31_261;
sens->V[3] = VIcp31_361;
sens->OM[1] = OMcp31_121;
sens->OM[2] = OMcp31_221;
sens->OM[3] = OMcp31_321;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp31_161_5;
sens->J[1][6] = JTcp31_161_6;
sens->J[1][19] = JTcp31_161_7;
sens->J[1][20] = JTcp31_161_8;
sens->J[1][21] = JTcp31_161_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp31_261_4;
sens->J[2][5] = JTcp31_261_5;
sens->J[2][6] = JTcp31_261_6;
sens->J[2][19] = JTcp31_261_7;
sens->J[2][20] = JTcp31_261_8;
sens->J[2][21] = JTcp31_261_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp31_361_4;
sens->J[3][5] = JTcp31_361_5;
sens->J[3][6] = JTcp31_361_6;
sens->J[3][19] = JTcp31_361_7;
sens->J[3][20] = JTcp31_361_8;
sens->J[3][21] = JTcp31_361_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp31_16;
sens->J[4][20] = ROcp31_419;
sens->J[4][21] = ROcp31_720;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp31_85;
sens->J[5][19] = ROcp31_26;
sens->J[5][20] = ROcp31_519;
sens->J[5][21] = ROcp31_820;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp31_95;
sens->J[6][19] = ROcp31_36;
sens->J[6][20] = ROcp31_619;
sens->J[6][21] = ROcp31_920;
sens->A[1] = ACcp31_161;
sens->A[2] = ACcp31_261;
sens->A[3] = ACcp31_361;
sens->OMP[1] = OPcp31_121;
sens->OMP[2] = OPcp31_221;
sens->OMP[3] = OPcp31_321;
 
// 
break;
case 33:
 


// = = Block_1_0_0_33_0_1 = = 
 
// Sensor Kinematics 


ROcp32_25 = S4*S5;
ROcp32_35 = -C4*S5;
ROcp32_85 = -S4*C5;
ROcp32_95 = C4*C5;
ROcp32_16 = C5*C6;
ROcp32_26 = ROcp32_25*C6+C4*S6;
ROcp32_36 = ROcp32_35*C6+S4*S6;
ROcp32_46 = -C5*S6;
ROcp32_56 = -(ROcp32_25*S6-C4*C6);
ROcp32_66 = -(ROcp32_35*S6-S4*C6);
OMcp32_25 = qd[5]*C4;
OMcp32_35 = qd[5]*S4;
OMcp32_16 = qd[4]+qd[6]*S5;
OMcp32_26 = OMcp32_25+ROcp32_85*qd[6];
OMcp32_36 = OMcp32_35+ROcp32_95*qd[6];
OPcp32_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp32_26 = ROcp32_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp32_35*S5-ROcp32_95*qd[4]);
OPcp32_36 = ROcp32_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp32_25*S5-ROcp32_85*qd[4]);

// = = Block_1_0_0_33_0_4 = = 
 
// Sensor Kinematics 


ROcp32_419 = ROcp32_46*C19+S19*S5;
ROcp32_519 = ROcp32_56*C19+ROcp32_85*S19;
ROcp32_619 = ROcp32_66*C19+ROcp32_95*S19;
ROcp32_719 = -(ROcp32_46*S19-C19*S5);
ROcp32_819 = -(ROcp32_56*S19-ROcp32_85*C19);
ROcp32_919 = -(ROcp32_66*S19-ROcp32_95*C19);
ROcp32_120 = ROcp32_16*C20-ROcp32_719*S20;
ROcp32_220 = ROcp32_26*C20-ROcp32_819*S20;
ROcp32_320 = ROcp32_36*C20-ROcp32_919*S20;
ROcp32_720 = ROcp32_16*S20+ROcp32_719*C20;
ROcp32_820 = ROcp32_26*S20+ROcp32_819*C20;
ROcp32_920 = ROcp32_36*S20+ROcp32_919*C20;
ROcp32_121 = ROcp32_120*C21+ROcp32_419*S21;
ROcp32_221 = ROcp32_220*C21+ROcp32_519*S21;
ROcp32_321 = ROcp32_320*C21+ROcp32_619*S21;
ROcp32_421 = -(ROcp32_120*S21-ROcp32_419*C21);
ROcp32_521 = -(ROcp32_220*S21-ROcp32_519*C21);
ROcp32_621 = -(ROcp32_320*S21-ROcp32_619*C21);
RLcp32_119 = s->dpt[2][3]*ROcp32_46+ROcp32_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp32_219 = s->dpt[2][3]*ROcp32_56+ROcp32_26*s->dpt[1][3]+ROcp32_85*s->dpt[3][3];
RLcp32_319 = s->dpt[2][3]*ROcp32_66+ROcp32_36*s->dpt[1][3]+ROcp32_95*s->dpt[3][3];
OMcp32_119 = OMcp32_16+ROcp32_16*qd[19];
OMcp32_219 = OMcp32_26+ROcp32_26*qd[19];
OMcp32_319 = OMcp32_36+ROcp32_36*qd[19];
ORcp32_119 = OMcp32_26*RLcp32_319-OMcp32_36*RLcp32_219;
ORcp32_219 = -(OMcp32_16*RLcp32_319-OMcp32_36*RLcp32_119);
ORcp32_319 = OMcp32_16*RLcp32_219-OMcp32_26*RLcp32_119;
OPcp32_119 = OPcp32_16+ROcp32_16*qdd[19]+qd[19]*(OMcp32_26*ROcp32_36-OMcp32_36*ROcp32_26);
OPcp32_219 = OPcp32_26+ROcp32_26*qdd[19]-qd[19]*(OMcp32_16*ROcp32_36-OMcp32_36*ROcp32_16);
OPcp32_319 = OPcp32_36+ROcp32_36*qdd[19]+qd[19]*(OMcp32_16*ROcp32_26-OMcp32_26*ROcp32_16);
RLcp32_120 = s->dpt[1][38]*ROcp32_16+s->dpt[2][38]*ROcp32_419+s->dpt[3][38]*ROcp32_719;
RLcp32_220 = s->dpt[1][38]*ROcp32_26+s->dpt[2][38]*ROcp32_519+s->dpt[3][38]*ROcp32_819;
RLcp32_320 = s->dpt[1][38]*ROcp32_36+s->dpt[2][38]*ROcp32_619+s->dpt[3][38]*ROcp32_919;
OMcp32_120 = OMcp32_119+ROcp32_419*qd[20];
OMcp32_220 = OMcp32_219+ROcp32_519*qd[20];
OMcp32_320 = OMcp32_319+ROcp32_619*qd[20];
ORcp32_120 = OMcp32_219*RLcp32_320-OMcp32_319*RLcp32_220;
ORcp32_220 = -(OMcp32_119*RLcp32_320-OMcp32_319*RLcp32_120);
ORcp32_320 = OMcp32_119*RLcp32_220-OMcp32_219*RLcp32_120;
OPcp32_120 = OPcp32_119+ROcp32_419*qdd[20]+qd[20]*(OMcp32_219*ROcp32_619-OMcp32_319*ROcp32_519);
OPcp32_220 = OPcp32_219+ROcp32_519*qdd[20]-qd[20]*(OMcp32_119*ROcp32_619-OMcp32_319*ROcp32_419);
OPcp32_320 = OPcp32_319+ROcp32_619*qdd[20]+qd[20]*(OMcp32_119*ROcp32_519-OMcp32_219*ROcp32_419);
RLcp32_121 = s->dpt[1][40]*ROcp32_120+s->dpt[2][40]*ROcp32_419+ROcp32_720*s->dpt[3][40];
RLcp32_221 = s->dpt[1][40]*ROcp32_220+s->dpt[2][40]*ROcp32_519+ROcp32_820*s->dpt[3][40];
RLcp32_321 = s->dpt[1][40]*ROcp32_320+s->dpt[2][40]*ROcp32_619+ROcp32_920*s->dpt[3][40];
OMcp32_121 = OMcp32_120+ROcp32_720*qd[21];
OMcp32_221 = OMcp32_220+ROcp32_820*qd[21];
OMcp32_321 = OMcp32_320+ROcp32_920*qd[21];
ORcp32_121 = OMcp32_220*RLcp32_321-OMcp32_320*RLcp32_221;
ORcp32_221 = -(OMcp32_120*RLcp32_321-OMcp32_320*RLcp32_121);
ORcp32_321 = OMcp32_120*RLcp32_221-OMcp32_220*RLcp32_121;
OPcp32_121 = OPcp32_120+ROcp32_720*qdd[21]+qd[21]*(OMcp32_220*ROcp32_920-OMcp32_320*ROcp32_820);
OPcp32_221 = OPcp32_220+ROcp32_820*qdd[21]-qd[21]*(OMcp32_120*ROcp32_920-OMcp32_320*ROcp32_720);
OPcp32_321 = OPcp32_320+ROcp32_920*qdd[21]+qd[21]*(OMcp32_120*ROcp32_820-OMcp32_220*ROcp32_720);
RLcp32_162 = ROcp32_121*s->dpt[1][43]+ROcp32_421*s->dpt[2][43]+ROcp32_720*s->dpt[3][43];
RLcp32_262 = ROcp32_221*s->dpt[1][43]+ROcp32_521*s->dpt[2][43]+ROcp32_820*s->dpt[3][43];
RLcp32_362 = ROcp32_321*s->dpt[1][43]+ROcp32_621*s->dpt[2][43]+ROcp32_920*s->dpt[3][43];
POcp32_162 = RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_162+q[1];
POcp32_262 = RLcp32_219+RLcp32_220+RLcp32_221+RLcp32_262+q[2];
POcp32_362 = RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_362+q[3];
JTcp32_262_4 = -(RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_362);
JTcp32_362_4 = RLcp32_219+RLcp32_220+RLcp32_221+RLcp32_262;
JTcp32_162_5 = C4*(RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_362)-S4*(RLcp32_219+RLcp32_220)-S4*(RLcp32_221+RLcp32_262);
JTcp32_262_5 = S4*(RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_162);
JTcp32_362_5 = -C4*(RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_162);
JTcp32_162_6 = ROcp32_85*(RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_362)-ROcp32_95*(RLcp32_219+RLcp32_220)-ROcp32_95*(
 RLcp32_221+RLcp32_262);
JTcp32_262_6 = RLcp32_162*ROcp32_95-RLcp32_321*S5-RLcp32_362*S5+ROcp32_95*(RLcp32_119+RLcp32_120+RLcp32_121)-S5*(
 RLcp32_319+RLcp32_320);
JTcp32_362_6 = RLcp32_221*S5-ROcp32_85*(RLcp32_119+RLcp32_120+RLcp32_121)+S5*(RLcp32_219+RLcp32_220)-RLcp32_162*
 ROcp32_85+RLcp32_262*S5;
JTcp32_162_7 = ROcp32_26*(RLcp32_320+RLcp32_321)-ROcp32_36*(RLcp32_220+RLcp32_221)-RLcp32_262*ROcp32_36+RLcp32_362*
 ROcp32_26;
JTcp32_262_7 = RLcp32_162*ROcp32_36-RLcp32_362*ROcp32_16-ROcp32_16*(RLcp32_320+RLcp32_321)+ROcp32_36*(RLcp32_120+
 RLcp32_121);
JTcp32_362_7 = ROcp32_16*(RLcp32_220+RLcp32_221)-ROcp32_26*(RLcp32_120+RLcp32_121)-RLcp32_162*ROcp32_26+RLcp32_262*
 ROcp32_16;
JTcp32_162_8 = ROcp32_519*(RLcp32_321+RLcp32_362)-ROcp32_619*(RLcp32_221+RLcp32_262);
JTcp32_262_8 = -(ROcp32_419*(RLcp32_321+RLcp32_362)-ROcp32_619*(RLcp32_121+RLcp32_162));
JTcp32_362_8 = ROcp32_419*(RLcp32_221+RLcp32_262)-ROcp32_519*(RLcp32_121+RLcp32_162);
JTcp32_162_9 = -(RLcp32_262*ROcp32_920-RLcp32_362*ROcp32_820);
JTcp32_262_9 = RLcp32_162*ROcp32_920-RLcp32_362*ROcp32_720;
JTcp32_362_9 = -(RLcp32_162*ROcp32_820-RLcp32_262*ROcp32_720);
ORcp32_162 = OMcp32_221*RLcp32_362-OMcp32_321*RLcp32_262;
ORcp32_262 = -(OMcp32_121*RLcp32_362-OMcp32_321*RLcp32_162);
ORcp32_362 = OMcp32_121*RLcp32_262-OMcp32_221*RLcp32_162;
VIcp32_162 = ORcp32_119+ORcp32_120+ORcp32_121+ORcp32_162+qd[1];
VIcp32_262 = ORcp32_219+ORcp32_220+ORcp32_221+ORcp32_262+qd[2];
VIcp32_362 = ORcp32_319+ORcp32_320+ORcp32_321+ORcp32_362+qd[3];
ACcp32_162 = qdd[1]+OMcp32_219*ORcp32_320+OMcp32_220*ORcp32_321+OMcp32_221*ORcp32_362+OMcp32_26*ORcp32_319-OMcp32_319*
 ORcp32_220-OMcp32_320*ORcp32_221-OMcp32_321*ORcp32_262-OMcp32_36*ORcp32_219+OPcp32_219*RLcp32_320+OPcp32_220*RLcp32_321+
 OPcp32_221*RLcp32_362+OPcp32_26*RLcp32_319-OPcp32_319*RLcp32_220-OPcp32_320*RLcp32_221-OPcp32_321*RLcp32_262-OPcp32_36*
 RLcp32_219;
ACcp32_262 = qdd[2]-OMcp32_119*ORcp32_320-OMcp32_120*ORcp32_321-OMcp32_121*ORcp32_362-OMcp32_16*ORcp32_319+OMcp32_319*
 ORcp32_120+OMcp32_320*ORcp32_121+OMcp32_321*ORcp32_162+OMcp32_36*ORcp32_119-OPcp32_119*RLcp32_320-OPcp32_120*RLcp32_321-
 OPcp32_121*RLcp32_362-OPcp32_16*RLcp32_319+OPcp32_319*RLcp32_120+OPcp32_320*RLcp32_121+OPcp32_321*RLcp32_162+OPcp32_36*
 RLcp32_119;
ACcp32_362 = qdd[3]+OMcp32_119*ORcp32_220+OMcp32_120*ORcp32_221+OMcp32_121*ORcp32_262+OMcp32_16*ORcp32_219-OMcp32_219*
 ORcp32_120-OMcp32_220*ORcp32_121-OMcp32_221*ORcp32_162-OMcp32_26*ORcp32_119+OPcp32_119*RLcp32_220+OPcp32_120*RLcp32_221+
 OPcp32_121*RLcp32_262+OPcp32_16*RLcp32_219-OPcp32_219*RLcp32_120-OPcp32_220*RLcp32_121-OPcp32_221*RLcp32_162-OPcp32_26*
 RLcp32_119;

// = = Block_1_0_0_33_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp32_162;
sens->P[2] = POcp32_262;
sens->P[3] = POcp32_362;
sens->R[1][1] = ROcp32_121;
sens->R[1][2] = ROcp32_221;
sens->R[1][3] = ROcp32_321;
sens->R[2][1] = ROcp32_421;
sens->R[2][2] = ROcp32_521;
sens->R[2][3] = ROcp32_621;
sens->R[3][1] = ROcp32_720;
sens->R[3][2] = ROcp32_820;
sens->R[3][3] = ROcp32_920;
sens->V[1] = VIcp32_162;
sens->V[2] = VIcp32_262;
sens->V[3] = VIcp32_362;
sens->OM[1] = OMcp32_121;
sens->OM[2] = OMcp32_221;
sens->OM[3] = OMcp32_321;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp32_162_5;
sens->J[1][6] = JTcp32_162_6;
sens->J[1][19] = JTcp32_162_7;
sens->J[1][20] = JTcp32_162_8;
sens->J[1][21] = JTcp32_162_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp32_262_4;
sens->J[2][5] = JTcp32_262_5;
sens->J[2][6] = JTcp32_262_6;
sens->J[2][19] = JTcp32_262_7;
sens->J[2][20] = JTcp32_262_8;
sens->J[2][21] = JTcp32_262_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp32_362_4;
sens->J[3][5] = JTcp32_362_5;
sens->J[3][6] = JTcp32_362_6;
sens->J[3][19] = JTcp32_362_7;
sens->J[3][20] = JTcp32_362_8;
sens->J[3][21] = JTcp32_362_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp32_16;
sens->J[4][20] = ROcp32_419;
sens->J[4][21] = ROcp32_720;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp32_85;
sens->J[5][19] = ROcp32_26;
sens->J[5][20] = ROcp32_519;
sens->J[5][21] = ROcp32_820;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp32_95;
sens->J[6][19] = ROcp32_36;
sens->J[6][20] = ROcp32_619;
sens->J[6][21] = ROcp32_920;
sens->A[1] = ACcp32_162;
sens->A[2] = ACcp32_262;
sens->A[3] = ACcp32_362;
sens->OMP[1] = OPcp32_121;
sens->OMP[2] = OPcp32_221;
sens->OMP[3] = OPcp32_321;
 
// 
break;
case 34:
 


// = = Block_1_0_0_34_0_1 = = 
 
// Sensor Kinematics 


ROcp33_25 = S4*S5;
ROcp33_35 = -C4*S5;
ROcp33_85 = -S4*C5;
ROcp33_95 = C4*C5;
ROcp33_16 = C5*C6;
ROcp33_26 = ROcp33_25*C6+C4*S6;
ROcp33_36 = ROcp33_35*C6+S4*S6;
ROcp33_46 = -C5*S6;
ROcp33_56 = -(ROcp33_25*S6-C4*C6);
ROcp33_66 = -(ROcp33_35*S6-S4*C6);
OMcp33_25 = qd[5]*C4;
OMcp33_35 = qd[5]*S4;
OMcp33_16 = qd[4]+qd[6]*S5;
OMcp33_26 = OMcp33_25+ROcp33_85*qd[6];
OMcp33_36 = OMcp33_35+ROcp33_95*qd[6];
OPcp33_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp33_26 = ROcp33_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp33_35*S5-ROcp33_95*qd[4]);
OPcp33_36 = ROcp33_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp33_25*S5-ROcp33_85*qd[4]);

// = = Block_1_0_0_34_0_4 = = 
 
// Sensor Kinematics 


ROcp33_419 = ROcp33_46*C19+S19*S5;
ROcp33_519 = ROcp33_56*C19+ROcp33_85*S19;
ROcp33_619 = ROcp33_66*C19+ROcp33_95*S19;
ROcp33_719 = -(ROcp33_46*S19-C19*S5);
ROcp33_819 = -(ROcp33_56*S19-ROcp33_85*C19);
ROcp33_919 = -(ROcp33_66*S19-ROcp33_95*C19);
ROcp33_120 = ROcp33_16*C20-ROcp33_719*S20;
ROcp33_220 = ROcp33_26*C20-ROcp33_819*S20;
ROcp33_320 = ROcp33_36*C20-ROcp33_919*S20;
ROcp33_720 = ROcp33_16*S20+ROcp33_719*C20;
ROcp33_820 = ROcp33_26*S20+ROcp33_819*C20;
ROcp33_920 = ROcp33_36*S20+ROcp33_919*C20;
ROcp33_121 = ROcp33_120*C21+ROcp33_419*S21;
ROcp33_221 = ROcp33_220*C21+ROcp33_519*S21;
ROcp33_321 = ROcp33_320*C21+ROcp33_619*S21;
ROcp33_421 = -(ROcp33_120*S21-ROcp33_419*C21);
ROcp33_521 = -(ROcp33_220*S21-ROcp33_519*C21);
ROcp33_621 = -(ROcp33_320*S21-ROcp33_619*C21);
RLcp33_119 = s->dpt[2][3]*ROcp33_46+ROcp33_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp33_219 = s->dpt[2][3]*ROcp33_56+ROcp33_26*s->dpt[1][3]+ROcp33_85*s->dpt[3][3];
RLcp33_319 = s->dpt[2][3]*ROcp33_66+ROcp33_36*s->dpt[1][3]+ROcp33_95*s->dpt[3][3];
OMcp33_119 = OMcp33_16+ROcp33_16*qd[19];
OMcp33_219 = OMcp33_26+ROcp33_26*qd[19];
OMcp33_319 = OMcp33_36+ROcp33_36*qd[19];
ORcp33_119 = OMcp33_26*RLcp33_319-OMcp33_36*RLcp33_219;
ORcp33_219 = -(OMcp33_16*RLcp33_319-OMcp33_36*RLcp33_119);
ORcp33_319 = OMcp33_16*RLcp33_219-OMcp33_26*RLcp33_119;
OPcp33_119 = OPcp33_16+ROcp33_16*qdd[19]+qd[19]*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26);
OPcp33_219 = OPcp33_26+ROcp33_26*qdd[19]-qd[19]*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16);
OPcp33_319 = OPcp33_36+ROcp33_36*qdd[19]+qd[19]*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16);
RLcp33_120 = s->dpt[1][38]*ROcp33_16+s->dpt[2][38]*ROcp33_419+s->dpt[3][38]*ROcp33_719;
RLcp33_220 = s->dpt[1][38]*ROcp33_26+s->dpt[2][38]*ROcp33_519+s->dpt[3][38]*ROcp33_819;
RLcp33_320 = s->dpt[1][38]*ROcp33_36+s->dpt[2][38]*ROcp33_619+s->dpt[3][38]*ROcp33_919;
OMcp33_120 = OMcp33_119+ROcp33_419*qd[20];
OMcp33_220 = OMcp33_219+ROcp33_519*qd[20];
OMcp33_320 = OMcp33_319+ROcp33_619*qd[20];
ORcp33_120 = OMcp33_219*RLcp33_320-OMcp33_319*RLcp33_220;
ORcp33_220 = -(OMcp33_119*RLcp33_320-OMcp33_319*RLcp33_120);
ORcp33_320 = OMcp33_119*RLcp33_220-OMcp33_219*RLcp33_120;
OPcp33_120 = OPcp33_119+ROcp33_419*qdd[20]+qd[20]*(OMcp33_219*ROcp33_619-OMcp33_319*ROcp33_519);
OPcp33_220 = OPcp33_219+ROcp33_519*qdd[20]-qd[20]*(OMcp33_119*ROcp33_619-OMcp33_319*ROcp33_419);
OPcp33_320 = OPcp33_319+ROcp33_619*qdd[20]+qd[20]*(OMcp33_119*ROcp33_519-OMcp33_219*ROcp33_419);
RLcp33_121 = s->dpt[1][40]*ROcp33_120+s->dpt[2][40]*ROcp33_419+ROcp33_720*s->dpt[3][40];
RLcp33_221 = s->dpt[1][40]*ROcp33_220+s->dpt[2][40]*ROcp33_519+ROcp33_820*s->dpt[3][40];
RLcp33_321 = s->dpt[1][40]*ROcp33_320+s->dpt[2][40]*ROcp33_619+ROcp33_920*s->dpt[3][40];
OMcp33_121 = OMcp33_120+ROcp33_720*qd[21];
OMcp33_221 = OMcp33_220+ROcp33_820*qd[21];
OMcp33_321 = OMcp33_320+ROcp33_920*qd[21];
ORcp33_121 = OMcp33_220*RLcp33_321-OMcp33_320*RLcp33_221;
ORcp33_221 = -(OMcp33_120*RLcp33_321-OMcp33_320*RLcp33_121);
ORcp33_321 = OMcp33_120*RLcp33_221-OMcp33_220*RLcp33_121;
OPcp33_121 = OPcp33_120+ROcp33_720*qdd[21]+qd[21]*(OMcp33_220*ROcp33_920-OMcp33_320*ROcp33_820);
OPcp33_221 = OPcp33_220+ROcp33_820*qdd[21]-qd[21]*(OMcp33_120*ROcp33_920-OMcp33_320*ROcp33_720);
OPcp33_321 = OPcp33_320+ROcp33_920*qdd[21]+qd[21]*(OMcp33_120*ROcp33_820-OMcp33_220*ROcp33_720);
RLcp33_163 = ROcp33_121*s->dpt[1][44]+ROcp33_421*s->dpt[2][44]+ROcp33_720*s->dpt[3][44];
RLcp33_263 = ROcp33_221*s->dpt[1][44]+ROcp33_521*s->dpt[2][44]+ROcp33_820*s->dpt[3][44];
RLcp33_363 = ROcp33_321*s->dpt[1][44]+ROcp33_621*s->dpt[2][44]+ROcp33_920*s->dpt[3][44];
POcp33_163 = RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_163+q[1];
POcp33_263 = RLcp33_219+RLcp33_220+RLcp33_221+RLcp33_263+q[2];
POcp33_363 = RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_363+q[3];
JTcp33_263_4 = -(RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_363);
JTcp33_363_4 = RLcp33_219+RLcp33_220+RLcp33_221+RLcp33_263;
JTcp33_163_5 = C4*(RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_363)-S4*(RLcp33_219+RLcp33_220)-S4*(RLcp33_221+RLcp33_263);
JTcp33_263_5 = S4*(RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_163);
JTcp33_363_5 = -C4*(RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_163);
JTcp33_163_6 = ROcp33_85*(RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_363)-ROcp33_95*(RLcp33_219+RLcp33_220)-ROcp33_95*(
 RLcp33_221+RLcp33_263);
JTcp33_263_6 = RLcp33_163*ROcp33_95-RLcp33_321*S5-RLcp33_363*S5+ROcp33_95*(RLcp33_119+RLcp33_120+RLcp33_121)-S5*(
 RLcp33_319+RLcp33_320);
JTcp33_363_6 = RLcp33_221*S5-ROcp33_85*(RLcp33_119+RLcp33_120+RLcp33_121)+S5*(RLcp33_219+RLcp33_220)-RLcp33_163*
 ROcp33_85+RLcp33_263*S5;
JTcp33_163_7 = ROcp33_26*(RLcp33_320+RLcp33_321)-ROcp33_36*(RLcp33_220+RLcp33_221)-RLcp33_263*ROcp33_36+RLcp33_363*
 ROcp33_26;
JTcp33_263_7 = RLcp33_163*ROcp33_36-RLcp33_363*ROcp33_16-ROcp33_16*(RLcp33_320+RLcp33_321)+ROcp33_36*(RLcp33_120+
 RLcp33_121);
JTcp33_363_7 = ROcp33_16*(RLcp33_220+RLcp33_221)-ROcp33_26*(RLcp33_120+RLcp33_121)-RLcp33_163*ROcp33_26+RLcp33_263*
 ROcp33_16;
JTcp33_163_8 = ROcp33_519*(RLcp33_321+RLcp33_363)-ROcp33_619*(RLcp33_221+RLcp33_263);
JTcp33_263_8 = -(ROcp33_419*(RLcp33_321+RLcp33_363)-ROcp33_619*(RLcp33_121+RLcp33_163));
JTcp33_363_8 = ROcp33_419*(RLcp33_221+RLcp33_263)-ROcp33_519*(RLcp33_121+RLcp33_163);
JTcp33_163_9 = -(RLcp33_263*ROcp33_920-RLcp33_363*ROcp33_820);
JTcp33_263_9 = RLcp33_163*ROcp33_920-RLcp33_363*ROcp33_720;
JTcp33_363_9 = -(RLcp33_163*ROcp33_820-RLcp33_263*ROcp33_720);
ORcp33_163 = OMcp33_221*RLcp33_363-OMcp33_321*RLcp33_263;
ORcp33_263 = -(OMcp33_121*RLcp33_363-OMcp33_321*RLcp33_163);
ORcp33_363 = OMcp33_121*RLcp33_263-OMcp33_221*RLcp33_163;
VIcp33_163 = ORcp33_119+ORcp33_120+ORcp33_121+ORcp33_163+qd[1];
VIcp33_263 = ORcp33_219+ORcp33_220+ORcp33_221+ORcp33_263+qd[2];
VIcp33_363 = ORcp33_319+ORcp33_320+ORcp33_321+ORcp33_363+qd[3];
ACcp33_163 = qdd[1]+OMcp33_219*ORcp33_320+OMcp33_220*ORcp33_321+OMcp33_221*ORcp33_363+OMcp33_26*ORcp33_319-OMcp33_319*
 ORcp33_220-OMcp33_320*ORcp33_221-OMcp33_321*ORcp33_263-OMcp33_36*ORcp33_219+OPcp33_219*RLcp33_320+OPcp33_220*RLcp33_321+
 OPcp33_221*RLcp33_363+OPcp33_26*RLcp33_319-OPcp33_319*RLcp33_220-OPcp33_320*RLcp33_221-OPcp33_321*RLcp33_263-OPcp33_36*
 RLcp33_219;
ACcp33_263 = qdd[2]-OMcp33_119*ORcp33_320-OMcp33_120*ORcp33_321-OMcp33_121*ORcp33_363-OMcp33_16*ORcp33_319+OMcp33_319*
 ORcp33_120+OMcp33_320*ORcp33_121+OMcp33_321*ORcp33_163+OMcp33_36*ORcp33_119-OPcp33_119*RLcp33_320-OPcp33_120*RLcp33_321-
 OPcp33_121*RLcp33_363-OPcp33_16*RLcp33_319+OPcp33_319*RLcp33_120+OPcp33_320*RLcp33_121+OPcp33_321*RLcp33_163+OPcp33_36*
 RLcp33_119;
ACcp33_363 = qdd[3]+OMcp33_119*ORcp33_220+OMcp33_120*ORcp33_221+OMcp33_121*ORcp33_263+OMcp33_16*ORcp33_219-OMcp33_219*
 ORcp33_120-OMcp33_220*ORcp33_121-OMcp33_221*ORcp33_163-OMcp33_26*ORcp33_119+OPcp33_119*RLcp33_220+OPcp33_120*RLcp33_221+
 OPcp33_121*RLcp33_263+OPcp33_16*RLcp33_219-OPcp33_219*RLcp33_120-OPcp33_220*RLcp33_121-OPcp33_221*RLcp33_163-OPcp33_26*
 RLcp33_119;

// = = Block_1_0_0_34_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp33_163;
sens->P[2] = POcp33_263;
sens->P[3] = POcp33_363;
sens->R[1][1] = ROcp33_121;
sens->R[1][2] = ROcp33_221;
sens->R[1][3] = ROcp33_321;
sens->R[2][1] = ROcp33_421;
sens->R[2][2] = ROcp33_521;
sens->R[2][3] = ROcp33_621;
sens->R[3][1] = ROcp33_720;
sens->R[3][2] = ROcp33_820;
sens->R[3][3] = ROcp33_920;
sens->V[1] = VIcp33_163;
sens->V[2] = VIcp33_263;
sens->V[3] = VIcp33_363;
sens->OM[1] = OMcp33_121;
sens->OM[2] = OMcp33_221;
sens->OM[3] = OMcp33_321;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp33_163_5;
sens->J[1][6] = JTcp33_163_6;
sens->J[1][19] = JTcp33_163_7;
sens->J[1][20] = JTcp33_163_8;
sens->J[1][21] = JTcp33_163_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp33_263_4;
sens->J[2][5] = JTcp33_263_5;
sens->J[2][6] = JTcp33_263_6;
sens->J[2][19] = JTcp33_263_7;
sens->J[2][20] = JTcp33_263_8;
sens->J[2][21] = JTcp33_263_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp33_363_4;
sens->J[3][5] = JTcp33_363_5;
sens->J[3][6] = JTcp33_363_6;
sens->J[3][19] = JTcp33_363_7;
sens->J[3][20] = JTcp33_363_8;
sens->J[3][21] = JTcp33_363_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp33_16;
sens->J[4][20] = ROcp33_419;
sens->J[4][21] = ROcp33_720;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp33_85;
sens->J[5][19] = ROcp33_26;
sens->J[5][20] = ROcp33_519;
sens->J[5][21] = ROcp33_820;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp33_95;
sens->J[6][19] = ROcp33_36;
sens->J[6][20] = ROcp33_619;
sens->J[6][21] = ROcp33_920;
sens->A[1] = ACcp33_163;
sens->A[2] = ACcp33_263;
sens->A[3] = ACcp33_363;
sens->OMP[1] = OPcp33_121;
sens->OMP[2] = OPcp33_221;
sens->OMP[3] = OPcp33_321;
 
// 
break;
case 35:
 


// = = Block_1_0_0_35_0_1 = = 
 
// Sensor Kinematics 


ROcp34_25 = S4*S5;
ROcp34_35 = -C4*S5;
ROcp34_85 = -S4*C5;
ROcp34_95 = C4*C5;
ROcp34_16 = C5*C6;
ROcp34_26 = ROcp34_25*C6+C4*S6;
ROcp34_36 = ROcp34_35*C6+S4*S6;
ROcp34_46 = -C5*S6;
ROcp34_56 = -(ROcp34_25*S6-C4*C6);
ROcp34_66 = -(ROcp34_35*S6-S4*C6);
OMcp34_25 = qd[5]*C4;
OMcp34_35 = qd[5]*S4;
OMcp34_16 = qd[4]+qd[6]*S5;
OMcp34_26 = OMcp34_25+ROcp34_85*qd[6];
OMcp34_36 = OMcp34_35+ROcp34_95*qd[6];
OPcp34_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp34_26 = ROcp34_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp34_35*S5-ROcp34_95*qd[4]);
OPcp34_36 = ROcp34_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp34_25*S5-ROcp34_85*qd[4]);

// = = Block_1_0_0_35_0_4 = = 
 
// Sensor Kinematics 


ROcp34_419 = ROcp34_46*C19+S19*S5;
ROcp34_519 = ROcp34_56*C19+ROcp34_85*S19;
ROcp34_619 = ROcp34_66*C19+ROcp34_95*S19;
ROcp34_719 = -(ROcp34_46*S19-C19*S5);
ROcp34_819 = -(ROcp34_56*S19-ROcp34_85*C19);
ROcp34_919 = -(ROcp34_66*S19-ROcp34_95*C19);
ROcp34_120 = ROcp34_16*C20-ROcp34_719*S20;
ROcp34_220 = ROcp34_26*C20-ROcp34_819*S20;
ROcp34_320 = ROcp34_36*C20-ROcp34_919*S20;
ROcp34_720 = ROcp34_16*S20+ROcp34_719*C20;
ROcp34_820 = ROcp34_26*S20+ROcp34_819*C20;
ROcp34_920 = ROcp34_36*S20+ROcp34_919*C20;
ROcp34_121 = ROcp34_120*C21+ROcp34_419*S21;
ROcp34_221 = ROcp34_220*C21+ROcp34_519*S21;
ROcp34_321 = ROcp34_320*C21+ROcp34_619*S21;
ROcp34_421 = -(ROcp34_120*S21-ROcp34_419*C21);
ROcp34_521 = -(ROcp34_220*S21-ROcp34_519*C21);
ROcp34_621 = -(ROcp34_320*S21-ROcp34_619*C21);
RLcp34_119 = s->dpt[2][3]*ROcp34_46+ROcp34_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp34_219 = s->dpt[2][3]*ROcp34_56+ROcp34_26*s->dpt[1][3]+ROcp34_85*s->dpt[3][3];
RLcp34_319 = s->dpt[2][3]*ROcp34_66+ROcp34_36*s->dpt[1][3]+ROcp34_95*s->dpt[3][3];
OMcp34_119 = OMcp34_16+ROcp34_16*qd[19];
OMcp34_219 = OMcp34_26+ROcp34_26*qd[19];
OMcp34_319 = OMcp34_36+ROcp34_36*qd[19];
ORcp34_119 = OMcp34_26*RLcp34_319-OMcp34_36*RLcp34_219;
ORcp34_219 = -(OMcp34_16*RLcp34_319-OMcp34_36*RLcp34_119);
ORcp34_319 = OMcp34_16*RLcp34_219-OMcp34_26*RLcp34_119;
OPcp34_119 = OPcp34_16+ROcp34_16*qdd[19]+qd[19]*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26);
OPcp34_219 = OPcp34_26+ROcp34_26*qdd[19]-qd[19]*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16);
OPcp34_319 = OPcp34_36+ROcp34_36*qdd[19]+qd[19]*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16);
RLcp34_120 = s->dpt[1][38]*ROcp34_16+s->dpt[2][38]*ROcp34_419+s->dpt[3][38]*ROcp34_719;
RLcp34_220 = s->dpt[1][38]*ROcp34_26+s->dpt[2][38]*ROcp34_519+s->dpt[3][38]*ROcp34_819;
RLcp34_320 = s->dpt[1][38]*ROcp34_36+s->dpt[2][38]*ROcp34_619+s->dpt[3][38]*ROcp34_919;
OMcp34_120 = OMcp34_119+ROcp34_419*qd[20];
OMcp34_220 = OMcp34_219+ROcp34_519*qd[20];
OMcp34_320 = OMcp34_319+ROcp34_619*qd[20];
ORcp34_120 = OMcp34_219*RLcp34_320-OMcp34_319*RLcp34_220;
ORcp34_220 = -(OMcp34_119*RLcp34_320-OMcp34_319*RLcp34_120);
ORcp34_320 = OMcp34_119*RLcp34_220-OMcp34_219*RLcp34_120;
OPcp34_120 = OPcp34_119+ROcp34_419*qdd[20]+qd[20]*(OMcp34_219*ROcp34_619-OMcp34_319*ROcp34_519);
OPcp34_220 = OPcp34_219+ROcp34_519*qdd[20]-qd[20]*(OMcp34_119*ROcp34_619-OMcp34_319*ROcp34_419);
OPcp34_320 = OPcp34_319+ROcp34_619*qdd[20]+qd[20]*(OMcp34_119*ROcp34_519-OMcp34_219*ROcp34_419);
RLcp34_121 = s->dpt[1][40]*ROcp34_120+s->dpt[2][40]*ROcp34_419+ROcp34_720*s->dpt[3][40];
RLcp34_221 = s->dpt[1][40]*ROcp34_220+s->dpt[2][40]*ROcp34_519+ROcp34_820*s->dpt[3][40];
RLcp34_321 = s->dpt[1][40]*ROcp34_320+s->dpt[2][40]*ROcp34_619+ROcp34_920*s->dpt[3][40];
OMcp34_121 = OMcp34_120+ROcp34_720*qd[21];
OMcp34_221 = OMcp34_220+ROcp34_820*qd[21];
OMcp34_321 = OMcp34_320+ROcp34_920*qd[21];
ORcp34_121 = OMcp34_220*RLcp34_321-OMcp34_320*RLcp34_221;
ORcp34_221 = -(OMcp34_120*RLcp34_321-OMcp34_320*RLcp34_121);
ORcp34_321 = OMcp34_120*RLcp34_221-OMcp34_220*RLcp34_121;
OPcp34_121 = OPcp34_120+ROcp34_720*qdd[21]+qd[21]*(OMcp34_220*ROcp34_920-OMcp34_320*ROcp34_820);
OPcp34_221 = OPcp34_220+ROcp34_820*qdd[21]-qd[21]*(OMcp34_120*ROcp34_920-OMcp34_320*ROcp34_720);
OPcp34_321 = OPcp34_320+ROcp34_920*qdd[21]+qd[21]*(OMcp34_120*ROcp34_820-OMcp34_220*ROcp34_720);
RLcp34_164 = ROcp34_121*s->dpt[1][45]+ROcp34_421*s->dpt[2][45]+ROcp34_720*s->dpt[3][45];
RLcp34_264 = ROcp34_221*s->dpt[1][45]+ROcp34_521*s->dpt[2][45]+ROcp34_820*s->dpt[3][45];
RLcp34_364 = ROcp34_321*s->dpt[1][45]+ROcp34_621*s->dpt[2][45]+ROcp34_920*s->dpt[3][45];
POcp34_164 = RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_164+q[1];
POcp34_264 = RLcp34_219+RLcp34_220+RLcp34_221+RLcp34_264+q[2];
POcp34_364 = RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_364+q[3];
JTcp34_264_4 = -(RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_364);
JTcp34_364_4 = RLcp34_219+RLcp34_220+RLcp34_221+RLcp34_264;
JTcp34_164_5 = C4*(RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_364)-S4*(RLcp34_219+RLcp34_220)-S4*(RLcp34_221+RLcp34_264);
JTcp34_264_5 = S4*(RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_164);
JTcp34_364_5 = -C4*(RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_164);
JTcp34_164_6 = ROcp34_85*(RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_364)-ROcp34_95*(RLcp34_219+RLcp34_220)-ROcp34_95*(
 RLcp34_221+RLcp34_264);
JTcp34_264_6 = RLcp34_164*ROcp34_95-RLcp34_321*S5-RLcp34_364*S5+ROcp34_95*(RLcp34_119+RLcp34_120+RLcp34_121)-S5*(
 RLcp34_319+RLcp34_320);
JTcp34_364_6 = RLcp34_221*S5-ROcp34_85*(RLcp34_119+RLcp34_120+RLcp34_121)+S5*(RLcp34_219+RLcp34_220)-RLcp34_164*
 ROcp34_85+RLcp34_264*S5;
JTcp34_164_7 = ROcp34_26*(RLcp34_320+RLcp34_321)-ROcp34_36*(RLcp34_220+RLcp34_221)-RLcp34_264*ROcp34_36+RLcp34_364*
 ROcp34_26;
JTcp34_264_7 = RLcp34_164*ROcp34_36-RLcp34_364*ROcp34_16-ROcp34_16*(RLcp34_320+RLcp34_321)+ROcp34_36*(RLcp34_120+
 RLcp34_121);
JTcp34_364_7 = ROcp34_16*(RLcp34_220+RLcp34_221)-ROcp34_26*(RLcp34_120+RLcp34_121)-RLcp34_164*ROcp34_26+RLcp34_264*
 ROcp34_16;
JTcp34_164_8 = ROcp34_519*(RLcp34_321+RLcp34_364)-ROcp34_619*(RLcp34_221+RLcp34_264);
JTcp34_264_8 = -(ROcp34_419*(RLcp34_321+RLcp34_364)-ROcp34_619*(RLcp34_121+RLcp34_164));
JTcp34_364_8 = ROcp34_419*(RLcp34_221+RLcp34_264)-ROcp34_519*(RLcp34_121+RLcp34_164);
JTcp34_164_9 = -(RLcp34_264*ROcp34_920-RLcp34_364*ROcp34_820);
JTcp34_264_9 = RLcp34_164*ROcp34_920-RLcp34_364*ROcp34_720;
JTcp34_364_9 = -(RLcp34_164*ROcp34_820-RLcp34_264*ROcp34_720);
ORcp34_164 = OMcp34_221*RLcp34_364-OMcp34_321*RLcp34_264;
ORcp34_264 = -(OMcp34_121*RLcp34_364-OMcp34_321*RLcp34_164);
ORcp34_364 = OMcp34_121*RLcp34_264-OMcp34_221*RLcp34_164;
VIcp34_164 = ORcp34_119+ORcp34_120+ORcp34_121+ORcp34_164+qd[1];
VIcp34_264 = ORcp34_219+ORcp34_220+ORcp34_221+ORcp34_264+qd[2];
VIcp34_364 = ORcp34_319+ORcp34_320+ORcp34_321+ORcp34_364+qd[3];
ACcp34_164 = qdd[1]+OMcp34_219*ORcp34_320+OMcp34_220*ORcp34_321+OMcp34_221*ORcp34_364+OMcp34_26*ORcp34_319-OMcp34_319*
 ORcp34_220-OMcp34_320*ORcp34_221-OMcp34_321*ORcp34_264-OMcp34_36*ORcp34_219+OPcp34_219*RLcp34_320+OPcp34_220*RLcp34_321+
 OPcp34_221*RLcp34_364+OPcp34_26*RLcp34_319-OPcp34_319*RLcp34_220-OPcp34_320*RLcp34_221-OPcp34_321*RLcp34_264-OPcp34_36*
 RLcp34_219;
ACcp34_264 = qdd[2]-OMcp34_119*ORcp34_320-OMcp34_120*ORcp34_321-OMcp34_121*ORcp34_364-OMcp34_16*ORcp34_319+OMcp34_319*
 ORcp34_120+OMcp34_320*ORcp34_121+OMcp34_321*ORcp34_164+OMcp34_36*ORcp34_119-OPcp34_119*RLcp34_320-OPcp34_120*RLcp34_321-
 OPcp34_121*RLcp34_364-OPcp34_16*RLcp34_319+OPcp34_319*RLcp34_120+OPcp34_320*RLcp34_121+OPcp34_321*RLcp34_164+OPcp34_36*
 RLcp34_119;
ACcp34_364 = qdd[3]+OMcp34_119*ORcp34_220+OMcp34_120*ORcp34_221+OMcp34_121*ORcp34_264+OMcp34_16*ORcp34_219-OMcp34_219*
 ORcp34_120-OMcp34_220*ORcp34_121-OMcp34_221*ORcp34_164-OMcp34_26*ORcp34_119+OPcp34_119*RLcp34_220+OPcp34_120*RLcp34_221+
 OPcp34_121*RLcp34_264+OPcp34_16*RLcp34_219-OPcp34_219*RLcp34_120-OPcp34_220*RLcp34_121-OPcp34_221*RLcp34_164-OPcp34_26*
 RLcp34_119;

// = = Block_1_0_0_35_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp34_164;
sens->P[2] = POcp34_264;
sens->P[3] = POcp34_364;
sens->R[1][1] = ROcp34_121;
sens->R[1][2] = ROcp34_221;
sens->R[1][3] = ROcp34_321;
sens->R[2][1] = ROcp34_421;
sens->R[2][2] = ROcp34_521;
sens->R[2][3] = ROcp34_621;
sens->R[3][1] = ROcp34_720;
sens->R[3][2] = ROcp34_820;
sens->R[3][3] = ROcp34_920;
sens->V[1] = VIcp34_164;
sens->V[2] = VIcp34_264;
sens->V[3] = VIcp34_364;
sens->OM[1] = OMcp34_121;
sens->OM[2] = OMcp34_221;
sens->OM[3] = OMcp34_321;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp34_164_5;
sens->J[1][6] = JTcp34_164_6;
sens->J[1][19] = JTcp34_164_7;
sens->J[1][20] = JTcp34_164_8;
sens->J[1][21] = JTcp34_164_9;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp34_264_4;
sens->J[2][5] = JTcp34_264_5;
sens->J[2][6] = JTcp34_264_6;
sens->J[2][19] = JTcp34_264_7;
sens->J[2][20] = JTcp34_264_8;
sens->J[2][21] = JTcp34_264_9;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp34_364_4;
sens->J[3][5] = JTcp34_364_5;
sens->J[3][6] = JTcp34_364_6;
sens->J[3][19] = JTcp34_364_7;
sens->J[3][20] = JTcp34_364_8;
sens->J[3][21] = JTcp34_364_9;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp34_16;
sens->J[4][20] = ROcp34_419;
sens->J[4][21] = ROcp34_720;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp34_85;
sens->J[5][19] = ROcp34_26;
sens->J[5][20] = ROcp34_519;
sens->J[5][21] = ROcp34_820;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp34_95;
sens->J[6][19] = ROcp34_36;
sens->J[6][20] = ROcp34_619;
sens->J[6][21] = ROcp34_920;
sens->A[1] = ACcp34_164;
sens->A[2] = ACcp34_264;
sens->A[3] = ACcp34_364;
sens->OMP[1] = OPcp34_121;
sens->OMP[2] = OPcp34_221;
sens->OMP[3] = OPcp34_321;
 
// 
break;
case 36:
 


// = = Block_1_0_0_36_0_1 = = 
 
// Sensor Kinematics 


ROcp35_25 = S4*S5;
ROcp35_35 = -C4*S5;
ROcp35_85 = -S4*C5;
ROcp35_95 = C4*C5;
ROcp35_16 = C5*C6;
ROcp35_26 = ROcp35_25*C6+C4*S6;
ROcp35_36 = ROcp35_35*C6+S4*S6;
ROcp35_46 = -C5*S6;
ROcp35_56 = -(ROcp35_25*S6-C4*C6);
ROcp35_66 = -(ROcp35_35*S6-S4*C6);
OMcp35_25 = qd[5]*C4;
OMcp35_35 = qd[5]*S4;
OMcp35_16 = qd[4]+qd[6]*S5;
OMcp35_26 = OMcp35_25+ROcp35_85*qd[6];
OMcp35_36 = OMcp35_35+ROcp35_95*qd[6];
OPcp35_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp35_26 = ROcp35_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp35_35*S5-ROcp35_95*qd[4]);
OPcp35_36 = ROcp35_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp35_25*S5-ROcp35_85*qd[4]);

// = = Block_1_0_0_36_0_4 = = 
 
// Sensor Kinematics 


ROcp35_419 = ROcp35_46*C19+S19*S5;
ROcp35_519 = ROcp35_56*C19+ROcp35_85*S19;
ROcp35_619 = ROcp35_66*C19+ROcp35_95*S19;
ROcp35_719 = -(ROcp35_46*S19-C19*S5);
ROcp35_819 = -(ROcp35_56*S19-ROcp35_85*C19);
ROcp35_919 = -(ROcp35_66*S19-ROcp35_95*C19);
ROcp35_120 = ROcp35_16*C20-ROcp35_719*S20;
ROcp35_220 = ROcp35_26*C20-ROcp35_819*S20;
ROcp35_320 = ROcp35_36*C20-ROcp35_919*S20;
ROcp35_720 = ROcp35_16*S20+ROcp35_719*C20;
ROcp35_820 = ROcp35_26*S20+ROcp35_819*C20;
ROcp35_920 = ROcp35_36*S20+ROcp35_919*C20;
ROcp35_121 = ROcp35_120*C21+ROcp35_419*S21;
ROcp35_221 = ROcp35_220*C21+ROcp35_519*S21;
ROcp35_321 = ROcp35_320*C21+ROcp35_619*S21;
ROcp35_421 = -(ROcp35_120*S21-ROcp35_419*C21);
ROcp35_521 = -(ROcp35_220*S21-ROcp35_519*C21);
ROcp35_621 = -(ROcp35_320*S21-ROcp35_619*C21);
RLcp35_119 = s->dpt[2][3]*ROcp35_46+ROcp35_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp35_219 = s->dpt[2][3]*ROcp35_56+ROcp35_26*s->dpt[1][3]+ROcp35_85*s->dpt[3][3];
RLcp35_319 = s->dpt[2][3]*ROcp35_66+ROcp35_36*s->dpt[1][3]+ROcp35_95*s->dpt[3][3];
OMcp35_119 = OMcp35_16+ROcp35_16*qd[19];
OMcp35_219 = OMcp35_26+ROcp35_26*qd[19];
OMcp35_319 = OMcp35_36+ROcp35_36*qd[19];
ORcp35_119 = OMcp35_26*RLcp35_319-OMcp35_36*RLcp35_219;
ORcp35_219 = -(OMcp35_16*RLcp35_319-OMcp35_36*RLcp35_119);
ORcp35_319 = OMcp35_16*RLcp35_219-OMcp35_26*RLcp35_119;
OPcp35_119 = OPcp35_16+ROcp35_16*qdd[19]+qd[19]*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26);
OPcp35_219 = OPcp35_26+ROcp35_26*qdd[19]-qd[19]*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16);
OPcp35_319 = OPcp35_36+ROcp35_36*qdd[19]+qd[19]*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16);
RLcp35_120 = s->dpt[1][38]*ROcp35_16+s->dpt[2][38]*ROcp35_419+s->dpt[3][38]*ROcp35_719;
RLcp35_220 = s->dpt[1][38]*ROcp35_26+s->dpt[2][38]*ROcp35_519+s->dpt[3][38]*ROcp35_819;
RLcp35_320 = s->dpt[1][38]*ROcp35_36+s->dpt[2][38]*ROcp35_619+s->dpt[3][38]*ROcp35_919;
OMcp35_120 = OMcp35_119+ROcp35_419*qd[20];
OMcp35_220 = OMcp35_219+ROcp35_519*qd[20];
OMcp35_320 = OMcp35_319+ROcp35_619*qd[20];
ORcp35_120 = OMcp35_219*RLcp35_320-OMcp35_319*RLcp35_220;
ORcp35_220 = -(OMcp35_119*RLcp35_320-OMcp35_319*RLcp35_120);
ORcp35_320 = OMcp35_119*RLcp35_220-OMcp35_219*RLcp35_120;
OPcp35_120 = OPcp35_119+ROcp35_419*qdd[20]+qd[20]*(OMcp35_219*ROcp35_619-OMcp35_319*ROcp35_519);
OPcp35_220 = OPcp35_219+ROcp35_519*qdd[20]-qd[20]*(OMcp35_119*ROcp35_619-OMcp35_319*ROcp35_419);
OPcp35_320 = OPcp35_319+ROcp35_619*qdd[20]+qd[20]*(OMcp35_119*ROcp35_519-OMcp35_219*ROcp35_419);
RLcp35_121 = s->dpt[1][40]*ROcp35_120+s->dpt[2][40]*ROcp35_419+ROcp35_720*s->dpt[3][40];
RLcp35_221 = s->dpt[1][40]*ROcp35_220+s->dpt[2][40]*ROcp35_519+ROcp35_820*s->dpt[3][40];
RLcp35_321 = s->dpt[1][40]*ROcp35_320+s->dpt[2][40]*ROcp35_619+ROcp35_920*s->dpt[3][40];
OMcp35_121 = OMcp35_120+ROcp35_720*qd[21];
OMcp35_221 = OMcp35_220+ROcp35_820*qd[21];
OMcp35_321 = OMcp35_320+ROcp35_920*qd[21];
ORcp35_121 = OMcp35_220*RLcp35_321-OMcp35_320*RLcp35_221;
ORcp35_221 = -(OMcp35_120*RLcp35_321-OMcp35_320*RLcp35_121);
ORcp35_321 = OMcp35_120*RLcp35_221-OMcp35_220*RLcp35_121;
OPcp35_121 = OPcp35_120+ROcp35_720*qdd[21]+qd[21]*(OMcp35_220*ROcp35_920-OMcp35_320*ROcp35_820);
OPcp35_221 = OPcp35_220+ROcp35_820*qdd[21]-qd[21]*(OMcp35_120*ROcp35_920-OMcp35_320*ROcp35_720);
OPcp35_321 = OPcp35_320+ROcp35_920*qdd[21]+qd[21]*(OMcp35_120*ROcp35_820-OMcp35_220*ROcp35_720);

// = = Block_1_0_0_36_0_5 = = 
 
// Sensor Kinematics 


ROcp35_122 = ROcp35_121*C22-ROcp35_720*S22;
ROcp35_222 = ROcp35_221*C22-ROcp35_820*S22;
ROcp35_322 = ROcp35_321*C22-ROcp35_920*S22;
ROcp35_722 = ROcp35_121*S22+ROcp35_720*C22;
ROcp35_822 = ROcp35_221*S22+ROcp35_820*C22;
ROcp35_922 = ROcp35_321*S22+ROcp35_920*C22;
RLcp35_122 = ROcp35_121*s->dpt[1][44]+ROcp35_421*s->dpt[2][44]+ROcp35_720*s->dpt[3][44];
RLcp35_222 = ROcp35_221*s->dpt[1][44]+ROcp35_521*s->dpt[2][44]+ROcp35_820*s->dpt[3][44];
RLcp35_322 = ROcp35_321*s->dpt[1][44]+ROcp35_621*s->dpt[2][44]+ROcp35_920*s->dpt[3][44];
OMcp35_122 = OMcp35_121+ROcp35_421*qd[22];
OMcp35_222 = OMcp35_221+ROcp35_521*qd[22];
OMcp35_322 = OMcp35_321+ROcp35_621*qd[22];
ORcp35_122 = OMcp35_221*RLcp35_322-OMcp35_321*RLcp35_222;
ORcp35_222 = -(OMcp35_121*RLcp35_322-OMcp35_321*RLcp35_122);
ORcp35_322 = OMcp35_121*RLcp35_222-OMcp35_221*RLcp35_122;
OPcp35_122 = OPcp35_121+ROcp35_421*qdd[22]+qd[22]*(OMcp35_221*ROcp35_621-OMcp35_321*ROcp35_521);
OPcp35_222 = OPcp35_221+ROcp35_521*qdd[22]-qd[22]*(OMcp35_121*ROcp35_621-OMcp35_321*ROcp35_421);
OPcp35_322 = OPcp35_321+ROcp35_621*qdd[22]+qd[22]*(OMcp35_121*ROcp35_521-OMcp35_221*ROcp35_421);
RLcp35_165 = ROcp35_122*s->dpt[1][46]+ROcp35_421*s->dpt[2][46]+ROcp35_722*s->dpt[3][46];
RLcp35_265 = ROcp35_222*s->dpt[1][46]+ROcp35_521*s->dpt[2][46]+ROcp35_822*s->dpt[3][46];
RLcp35_365 = ROcp35_322*s->dpt[1][46]+ROcp35_621*s->dpt[2][46]+ROcp35_922*s->dpt[3][46];
POcp35_165 = RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_165+q[1];
POcp35_265 = RLcp35_219+RLcp35_220+RLcp35_221+RLcp35_222+RLcp35_265+q[2];
POcp35_365 = RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_365+q[3];
JTcp35_265_4 = -(RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_365);
JTcp35_365_4 = RLcp35_219+RLcp35_220+RLcp35_221+RLcp35_222+RLcp35_265;
JTcp35_165_5 = C4*(RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322)-S4*(RLcp35_219+RLcp35_220)-S4*(RLcp35_221+RLcp35_222)-
 RLcp35_265*S4+RLcp35_365*C4;
JTcp35_265_5 = S4*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_165);
JTcp35_365_5 = -C4*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_165);
JTcp35_165_6 = ROcp35_85*(RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322)-ROcp35_95*(RLcp35_219+RLcp35_220)-ROcp35_95*(
 RLcp35_221+RLcp35_222)-RLcp35_265*ROcp35_95+RLcp35_365*ROcp35_85;
JTcp35_265_6 = -(RLcp35_365*S5-ROcp35_95*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_165)+S5*(RLcp35_319+
 RLcp35_320)+S5*(RLcp35_321+RLcp35_322));
JTcp35_365_6 = RLcp35_265*S5-ROcp35_85*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_165)+S5*(RLcp35_219+
 RLcp35_220)+S5*(RLcp35_221+RLcp35_222);
JTcp35_165_7 = ROcp35_26*(RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_365)-ROcp35_36*(RLcp35_220+RLcp35_221)-ROcp35_36*(
 RLcp35_222+RLcp35_265);
JTcp35_265_7 = -(ROcp35_16*(RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_365)-ROcp35_36*(RLcp35_120+RLcp35_121)-ROcp35_36*(
 RLcp35_122+RLcp35_165));
JTcp35_365_7 = ROcp35_16*(RLcp35_220+RLcp35_221+RLcp35_222+RLcp35_265)-ROcp35_26*(RLcp35_120+RLcp35_121)-ROcp35_26*(
 RLcp35_122+RLcp35_165);
JTcp35_165_8 = ROcp35_519*(RLcp35_321+RLcp35_322)-ROcp35_619*(RLcp35_221+RLcp35_222)-RLcp35_265*ROcp35_619+RLcp35_365*
 ROcp35_519;
JTcp35_265_8 = RLcp35_165*ROcp35_619-RLcp35_365*ROcp35_419-ROcp35_419*(RLcp35_321+RLcp35_322)+ROcp35_619*(RLcp35_121+
 RLcp35_122);
JTcp35_365_8 = ROcp35_419*(RLcp35_221+RLcp35_222)-ROcp35_519*(RLcp35_121+RLcp35_122)-RLcp35_165*ROcp35_519+RLcp35_265*
 ROcp35_419;
JTcp35_165_9 = ROcp35_820*(RLcp35_322+RLcp35_365)-ROcp35_920*(RLcp35_222+RLcp35_265);
JTcp35_265_9 = -(ROcp35_720*(RLcp35_322+RLcp35_365)-ROcp35_920*(RLcp35_122+RLcp35_165));
JTcp35_365_9 = ROcp35_720*(RLcp35_222+RLcp35_265)-ROcp35_820*(RLcp35_122+RLcp35_165);
JTcp35_165_10 = -(RLcp35_265*ROcp35_621-RLcp35_365*ROcp35_521);
JTcp35_265_10 = RLcp35_165*ROcp35_621-RLcp35_365*ROcp35_421;
JTcp35_365_10 = -(RLcp35_165*ROcp35_521-RLcp35_265*ROcp35_421);
ORcp35_165 = OMcp35_222*RLcp35_365-OMcp35_322*RLcp35_265;
ORcp35_265 = -(OMcp35_122*RLcp35_365-OMcp35_322*RLcp35_165);
ORcp35_365 = OMcp35_122*RLcp35_265-OMcp35_222*RLcp35_165;
VIcp35_165 = ORcp35_119+ORcp35_120+ORcp35_121+ORcp35_122+ORcp35_165+qd[1];
VIcp35_265 = ORcp35_219+ORcp35_220+ORcp35_221+ORcp35_222+ORcp35_265+qd[2];
VIcp35_365 = ORcp35_319+ORcp35_320+ORcp35_321+ORcp35_322+ORcp35_365+qd[3];
ACcp35_165 = qdd[1]+OMcp35_219*ORcp35_320+OMcp35_220*ORcp35_321+OMcp35_221*ORcp35_322+OMcp35_222*ORcp35_365+OMcp35_26*
 ORcp35_319-OMcp35_319*ORcp35_220-OMcp35_320*ORcp35_221-OMcp35_321*ORcp35_222-OMcp35_322*ORcp35_265-OMcp35_36*ORcp35_219+
 OPcp35_219*RLcp35_320+OPcp35_220*RLcp35_321+OPcp35_221*RLcp35_322+OPcp35_222*RLcp35_365+OPcp35_26*RLcp35_319-OPcp35_319*
 RLcp35_220-OPcp35_320*RLcp35_221-OPcp35_321*RLcp35_222-OPcp35_322*RLcp35_265-OPcp35_36*RLcp35_219;
ACcp35_265 = qdd[2]-OMcp35_119*ORcp35_320-OMcp35_120*ORcp35_321-OMcp35_121*ORcp35_322-OMcp35_122*ORcp35_365-OMcp35_16*
 ORcp35_319+OMcp35_319*ORcp35_120+OMcp35_320*ORcp35_121+OMcp35_321*ORcp35_122+OMcp35_322*ORcp35_165+OMcp35_36*ORcp35_119-
 OPcp35_119*RLcp35_320-OPcp35_120*RLcp35_321-OPcp35_121*RLcp35_322-OPcp35_122*RLcp35_365-OPcp35_16*RLcp35_319+OPcp35_319*
 RLcp35_120+OPcp35_320*RLcp35_121+OPcp35_321*RLcp35_122+OPcp35_322*RLcp35_165+OPcp35_36*RLcp35_119;
ACcp35_365 = qdd[3]+OMcp35_119*ORcp35_220+OMcp35_120*ORcp35_221+OMcp35_121*ORcp35_222+OMcp35_122*ORcp35_265+OMcp35_16*
 ORcp35_219-OMcp35_219*ORcp35_120-OMcp35_220*ORcp35_121-OMcp35_221*ORcp35_122-OMcp35_222*ORcp35_165-OMcp35_26*ORcp35_119+
 OPcp35_119*RLcp35_220+OPcp35_120*RLcp35_221+OPcp35_121*RLcp35_222+OPcp35_122*RLcp35_265+OPcp35_16*RLcp35_219-OPcp35_219*
 RLcp35_120-OPcp35_220*RLcp35_121-OPcp35_221*RLcp35_122-OPcp35_222*RLcp35_165-OPcp35_26*RLcp35_119;

// = = Block_1_0_0_36_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp35_165;
sens->P[2] = POcp35_265;
sens->P[3] = POcp35_365;
sens->R[1][1] = ROcp35_122;
sens->R[1][2] = ROcp35_222;
sens->R[1][3] = ROcp35_322;
sens->R[2][1] = ROcp35_421;
sens->R[2][2] = ROcp35_521;
sens->R[2][3] = ROcp35_621;
sens->R[3][1] = ROcp35_722;
sens->R[3][2] = ROcp35_822;
sens->R[3][3] = ROcp35_922;
sens->V[1] = VIcp35_165;
sens->V[2] = VIcp35_265;
sens->V[3] = VIcp35_365;
sens->OM[1] = OMcp35_122;
sens->OM[2] = OMcp35_222;
sens->OM[3] = OMcp35_322;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp35_165_5;
sens->J[1][6] = JTcp35_165_6;
sens->J[1][19] = JTcp35_165_7;
sens->J[1][20] = JTcp35_165_8;
sens->J[1][21] = JTcp35_165_9;
sens->J[1][22] = JTcp35_165_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp35_265_4;
sens->J[2][5] = JTcp35_265_5;
sens->J[2][6] = JTcp35_265_6;
sens->J[2][19] = JTcp35_265_7;
sens->J[2][20] = JTcp35_265_8;
sens->J[2][21] = JTcp35_265_9;
sens->J[2][22] = JTcp35_265_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp35_365_4;
sens->J[3][5] = JTcp35_365_5;
sens->J[3][6] = JTcp35_365_6;
sens->J[3][19] = JTcp35_365_7;
sens->J[3][20] = JTcp35_365_8;
sens->J[3][21] = JTcp35_365_9;
sens->J[3][22] = JTcp35_365_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp35_16;
sens->J[4][20] = ROcp35_419;
sens->J[4][21] = ROcp35_720;
sens->J[4][22] = ROcp35_421;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp35_85;
sens->J[5][19] = ROcp35_26;
sens->J[5][20] = ROcp35_519;
sens->J[5][21] = ROcp35_820;
sens->J[5][22] = ROcp35_521;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp35_95;
sens->J[6][19] = ROcp35_36;
sens->J[6][20] = ROcp35_619;
sens->J[6][21] = ROcp35_920;
sens->J[6][22] = ROcp35_621;
sens->A[1] = ACcp35_165;
sens->A[2] = ACcp35_265;
sens->A[3] = ACcp35_365;
sens->OMP[1] = OPcp35_122;
sens->OMP[2] = OPcp35_222;
sens->OMP[3] = OPcp35_322;
 
// 
break;
case 37:
 


// = = Block_1_0_0_37_0_1 = = 
 
// Sensor Kinematics 


ROcp36_25 = S4*S5;
ROcp36_35 = -C4*S5;
ROcp36_85 = -S4*C5;
ROcp36_95 = C4*C5;
ROcp36_16 = C5*C6;
ROcp36_26 = ROcp36_25*C6+C4*S6;
ROcp36_36 = ROcp36_35*C6+S4*S6;
ROcp36_46 = -C5*S6;
ROcp36_56 = -(ROcp36_25*S6-C4*C6);
ROcp36_66 = -(ROcp36_35*S6-S4*C6);
OMcp36_25 = qd[5]*C4;
OMcp36_35 = qd[5]*S4;
OMcp36_16 = qd[4]+qd[6]*S5;
OMcp36_26 = OMcp36_25+ROcp36_85*qd[6];
OMcp36_36 = OMcp36_35+ROcp36_95*qd[6];
OPcp36_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp36_26 = ROcp36_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp36_35*S5-ROcp36_95*qd[4]);
OPcp36_36 = ROcp36_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp36_25*S5-ROcp36_85*qd[4]);

// = = Block_1_0_0_37_0_4 = = 
 
// Sensor Kinematics 


ROcp36_419 = ROcp36_46*C19+S19*S5;
ROcp36_519 = ROcp36_56*C19+ROcp36_85*S19;
ROcp36_619 = ROcp36_66*C19+ROcp36_95*S19;
ROcp36_719 = -(ROcp36_46*S19-C19*S5);
ROcp36_819 = -(ROcp36_56*S19-ROcp36_85*C19);
ROcp36_919 = -(ROcp36_66*S19-ROcp36_95*C19);
ROcp36_120 = ROcp36_16*C20-ROcp36_719*S20;
ROcp36_220 = ROcp36_26*C20-ROcp36_819*S20;
ROcp36_320 = ROcp36_36*C20-ROcp36_919*S20;
ROcp36_720 = ROcp36_16*S20+ROcp36_719*C20;
ROcp36_820 = ROcp36_26*S20+ROcp36_819*C20;
ROcp36_920 = ROcp36_36*S20+ROcp36_919*C20;
ROcp36_121 = ROcp36_120*C21+ROcp36_419*S21;
ROcp36_221 = ROcp36_220*C21+ROcp36_519*S21;
ROcp36_321 = ROcp36_320*C21+ROcp36_619*S21;
ROcp36_421 = -(ROcp36_120*S21-ROcp36_419*C21);
ROcp36_521 = -(ROcp36_220*S21-ROcp36_519*C21);
ROcp36_621 = -(ROcp36_320*S21-ROcp36_619*C21);
RLcp36_119 = s->dpt[2][3]*ROcp36_46+ROcp36_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp36_219 = s->dpt[2][3]*ROcp36_56+ROcp36_26*s->dpt[1][3]+ROcp36_85*s->dpt[3][3];
RLcp36_319 = s->dpt[2][3]*ROcp36_66+ROcp36_36*s->dpt[1][3]+ROcp36_95*s->dpt[3][3];
OMcp36_119 = OMcp36_16+ROcp36_16*qd[19];
OMcp36_219 = OMcp36_26+ROcp36_26*qd[19];
OMcp36_319 = OMcp36_36+ROcp36_36*qd[19];
ORcp36_119 = OMcp36_26*RLcp36_319-OMcp36_36*RLcp36_219;
ORcp36_219 = -(OMcp36_16*RLcp36_319-OMcp36_36*RLcp36_119);
ORcp36_319 = OMcp36_16*RLcp36_219-OMcp36_26*RLcp36_119;
OPcp36_119 = OPcp36_16+ROcp36_16*qdd[19]+qd[19]*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26);
OPcp36_219 = OPcp36_26+ROcp36_26*qdd[19]-qd[19]*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16);
OPcp36_319 = OPcp36_36+ROcp36_36*qdd[19]+qd[19]*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16);
RLcp36_120 = s->dpt[1][38]*ROcp36_16+s->dpt[2][38]*ROcp36_419+s->dpt[3][38]*ROcp36_719;
RLcp36_220 = s->dpt[1][38]*ROcp36_26+s->dpt[2][38]*ROcp36_519+s->dpt[3][38]*ROcp36_819;
RLcp36_320 = s->dpt[1][38]*ROcp36_36+s->dpt[2][38]*ROcp36_619+s->dpt[3][38]*ROcp36_919;
OMcp36_120 = OMcp36_119+ROcp36_419*qd[20];
OMcp36_220 = OMcp36_219+ROcp36_519*qd[20];
OMcp36_320 = OMcp36_319+ROcp36_619*qd[20];
ORcp36_120 = OMcp36_219*RLcp36_320-OMcp36_319*RLcp36_220;
ORcp36_220 = -(OMcp36_119*RLcp36_320-OMcp36_319*RLcp36_120);
ORcp36_320 = OMcp36_119*RLcp36_220-OMcp36_219*RLcp36_120;
OPcp36_120 = OPcp36_119+ROcp36_419*qdd[20]+qd[20]*(OMcp36_219*ROcp36_619-OMcp36_319*ROcp36_519);
OPcp36_220 = OPcp36_219+ROcp36_519*qdd[20]-qd[20]*(OMcp36_119*ROcp36_619-OMcp36_319*ROcp36_419);
OPcp36_320 = OPcp36_319+ROcp36_619*qdd[20]+qd[20]*(OMcp36_119*ROcp36_519-OMcp36_219*ROcp36_419);
RLcp36_121 = s->dpt[1][40]*ROcp36_120+s->dpt[2][40]*ROcp36_419+ROcp36_720*s->dpt[3][40];
RLcp36_221 = s->dpt[1][40]*ROcp36_220+s->dpt[2][40]*ROcp36_519+ROcp36_820*s->dpt[3][40];
RLcp36_321 = s->dpt[1][40]*ROcp36_320+s->dpt[2][40]*ROcp36_619+ROcp36_920*s->dpt[3][40];
OMcp36_121 = OMcp36_120+ROcp36_720*qd[21];
OMcp36_221 = OMcp36_220+ROcp36_820*qd[21];
OMcp36_321 = OMcp36_320+ROcp36_920*qd[21];
ORcp36_121 = OMcp36_220*RLcp36_321-OMcp36_320*RLcp36_221;
ORcp36_221 = -(OMcp36_120*RLcp36_321-OMcp36_320*RLcp36_121);
ORcp36_321 = OMcp36_120*RLcp36_221-OMcp36_220*RLcp36_121;
OPcp36_121 = OPcp36_120+ROcp36_720*qdd[21]+qd[21]*(OMcp36_220*ROcp36_920-OMcp36_320*ROcp36_820);
OPcp36_221 = OPcp36_220+ROcp36_820*qdd[21]-qd[21]*(OMcp36_120*ROcp36_920-OMcp36_320*ROcp36_720);
OPcp36_321 = OPcp36_320+ROcp36_920*qdd[21]+qd[21]*(OMcp36_120*ROcp36_820-OMcp36_220*ROcp36_720);

// = = Block_1_0_0_37_0_5 = = 
 
// Sensor Kinematics 


ROcp36_122 = ROcp36_121*C22-ROcp36_720*S22;
ROcp36_222 = ROcp36_221*C22-ROcp36_820*S22;
ROcp36_322 = ROcp36_321*C22-ROcp36_920*S22;
ROcp36_722 = ROcp36_121*S22+ROcp36_720*C22;
ROcp36_822 = ROcp36_221*S22+ROcp36_820*C22;
ROcp36_922 = ROcp36_321*S22+ROcp36_920*C22;
RLcp36_122 = ROcp36_121*s->dpt[1][44]+ROcp36_421*s->dpt[2][44]+ROcp36_720*s->dpt[3][44];
RLcp36_222 = ROcp36_221*s->dpt[1][44]+ROcp36_521*s->dpt[2][44]+ROcp36_820*s->dpt[3][44];
RLcp36_322 = ROcp36_321*s->dpt[1][44]+ROcp36_621*s->dpt[2][44]+ROcp36_920*s->dpt[3][44];
OMcp36_122 = OMcp36_121+ROcp36_421*qd[22];
OMcp36_222 = OMcp36_221+ROcp36_521*qd[22];
OMcp36_322 = OMcp36_321+ROcp36_621*qd[22];
ORcp36_122 = OMcp36_221*RLcp36_322-OMcp36_321*RLcp36_222;
ORcp36_222 = -(OMcp36_121*RLcp36_322-OMcp36_321*RLcp36_122);
ORcp36_322 = OMcp36_121*RLcp36_222-OMcp36_221*RLcp36_122;
OPcp36_122 = OPcp36_121+ROcp36_421*qdd[22]+qd[22]*(OMcp36_221*ROcp36_621-OMcp36_321*ROcp36_521);
OPcp36_222 = OPcp36_221+ROcp36_521*qdd[22]-qd[22]*(OMcp36_121*ROcp36_621-OMcp36_321*ROcp36_421);
OPcp36_322 = OPcp36_321+ROcp36_621*qdd[22]+qd[22]*(OMcp36_121*ROcp36_521-OMcp36_221*ROcp36_421);
RLcp36_166 = s->dpt[1][47]*ROcp36_122+s->dpt[3][47]*ROcp36_722+ROcp36_421*s->dpt[2][47];
RLcp36_266 = s->dpt[1][47]*ROcp36_222+s->dpt[3][47]*ROcp36_822+ROcp36_521*s->dpt[2][47];
RLcp36_366 = s->dpt[1][47]*ROcp36_322+s->dpt[3][47]*ROcp36_922+ROcp36_621*s->dpt[2][47];
POcp36_166 = RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_166+q[1];
POcp36_266 = RLcp36_219+RLcp36_220+RLcp36_221+RLcp36_222+RLcp36_266+q[2];
POcp36_366 = RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_366+q[3];
JTcp36_266_4 = -(RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_366);
JTcp36_366_4 = RLcp36_219+RLcp36_220+RLcp36_221+RLcp36_222+RLcp36_266;
JTcp36_166_5 = C4*(RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322)-S4*(RLcp36_219+RLcp36_220)-S4*(RLcp36_221+RLcp36_222)-
 RLcp36_266*S4+RLcp36_366*C4;
JTcp36_266_5 = S4*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_166);
JTcp36_366_5 = -C4*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_166);
JTcp36_166_6 = ROcp36_85*(RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322)-ROcp36_95*(RLcp36_219+RLcp36_220)-ROcp36_95*(
 RLcp36_221+RLcp36_222)-RLcp36_266*ROcp36_95+RLcp36_366*ROcp36_85;
JTcp36_266_6 = -(RLcp36_366*S5-ROcp36_95*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_166)+S5*(RLcp36_319+
 RLcp36_320)+S5*(RLcp36_321+RLcp36_322));
JTcp36_366_6 = RLcp36_266*S5-ROcp36_85*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_166)+S5*(RLcp36_219+
 RLcp36_220)+S5*(RLcp36_221+RLcp36_222);
JTcp36_166_7 = ROcp36_26*(RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_366)-ROcp36_36*(RLcp36_220+RLcp36_221)-ROcp36_36*(
 RLcp36_222+RLcp36_266);
JTcp36_266_7 = -(ROcp36_16*(RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_366)-ROcp36_36*(RLcp36_120+RLcp36_121)-ROcp36_36*(
 RLcp36_122+RLcp36_166));
JTcp36_366_7 = ROcp36_16*(RLcp36_220+RLcp36_221+RLcp36_222+RLcp36_266)-ROcp36_26*(RLcp36_120+RLcp36_121)-ROcp36_26*(
 RLcp36_122+RLcp36_166);
JTcp36_166_8 = ROcp36_519*(RLcp36_321+RLcp36_322)-ROcp36_619*(RLcp36_221+RLcp36_222)-RLcp36_266*ROcp36_619+RLcp36_366*
 ROcp36_519;
JTcp36_266_8 = RLcp36_166*ROcp36_619-RLcp36_366*ROcp36_419-ROcp36_419*(RLcp36_321+RLcp36_322)+ROcp36_619*(RLcp36_121+
 RLcp36_122);
JTcp36_366_8 = ROcp36_419*(RLcp36_221+RLcp36_222)-ROcp36_519*(RLcp36_121+RLcp36_122)-RLcp36_166*ROcp36_519+RLcp36_266*
 ROcp36_419;
JTcp36_166_9 = ROcp36_820*(RLcp36_322+RLcp36_366)-ROcp36_920*(RLcp36_222+RLcp36_266);
JTcp36_266_9 = -(ROcp36_720*(RLcp36_322+RLcp36_366)-ROcp36_920*(RLcp36_122+RLcp36_166));
JTcp36_366_9 = ROcp36_720*(RLcp36_222+RLcp36_266)-ROcp36_820*(RLcp36_122+RLcp36_166);
JTcp36_166_10 = -(RLcp36_266*ROcp36_621-RLcp36_366*ROcp36_521);
JTcp36_266_10 = RLcp36_166*ROcp36_621-RLcp36_366*ROcp36_421;
JTcp36_366_10 = -(RLcp36_166*ROcp36_521-RLcp36_266*ROcp36_421);
ORcp36_166 = OMcp36_222*RLcp36_366-OMcp36_322*RLcp36_266;
ORcp36_266 = -(OMcp36_122*RLcp36_366-OMcp36_322*RLcp36_166);
ORcp36_366 = OMcp36_122*RLcp36_266-OMcp36_222*RLcp36_166;
VIcp36_166 = ORcp36_119+ORcp36_120+ORcp36_121+ORcp36_122+ORcp36_166+qd[1];
VIcp36_266 = ORcp36_219+ORcp36_220+ORcp36_221+ORcp36_222+ORcp36_266+qd[2];
VIcp36_366 = ORcp36_319+ORcp36_320+ORcp36_321+ORcp36_322+ORcp36_366+qd[3];
ACcp36_166 = qdd[1]+OMcp36_219*ORcp36_320+OMcp36_220*ORcp36_321+OMcp36_221*ORcp36_322+OMcp36_222*ORcp36_366+OMcp36_26*
 ORcp36_319-OMcp36_319*ORcp36_220-OMcp36_320*ORcp36_221-OMcp36_321*ORcp36_222-OMcp36_322*ORcp36_266-OMcp36_36*ORcp36_219+
 OPcp36_219*RLcp36_320+OPcp36_220*RLcp36_321+OPcp36_221*RLcp36_322+OPcp36_222*RLcp36_366+OPcp36_26*RLcp36_319-OPcp36_319*
 RLcp36_220-OPcp36_320*RLcp36_221-OPcp36_321*RLcp36_222-OPcp36_322*RLcp36_266-OPcp36_36*RLcp36_219;
ACcp36_266 = qdd[2]-OMcp36_119*ORcp36_320-OMcp36_120*ORcp36_321-OMcp36_121*ORcp36_322-OMcp36_122*ORcp36_366-OMcp36_16*
 ORcp36_319+OMcp36_319*ORcp36_120+OMcp36_320*ORcp36_121+OMcp36_321*ORcp36_122+OMcp36_322*ORcp36_166+OMcp36_36*ORcp36_119-
 OPcp36_119*RLcp36_320-OPcp36_120*RLcp36_321-OPcp36_121*RLcp36_322-OPcp36_122*RLcp36_366-OPcp36_16*RLcp36_319+OPcp36_319*
 RLcp36_120+OPcp36_320*RLcp36_121+OPcp36_321*RLcp36_122+OPcp36_322*RLcp36_166+OPcp36_36*RLcp36_119;
ACcp36_366 = qdd[3]+OMcp36_119*ORcp36_220+OMcp36_120*ORcp36_221+OMcp36_121*ORcp36_222+OMcp36_122*ORcp36_266+OMcp36_16*
 ORcp36_219-OMcp36_219*ORcp36_120-OMcp36_220*ORcp36_121-OMcp36_221*ORcp36_122-OMcp36_222*ORcp36_166-OMcp36_26*ORcp36_119+
 OPcp36_119*RLcp36_220+OPcp36_120*RLcp36_221+OPcp36_121*RLcp36_222+OPcp36_122*RLcp36_266+OPcp36_16*RLcp36_219-OPcp36_219*
 RLcp36_120-OPcp36_220*RLcp36_121-OPcp36_221*RLcp36_122-OPcp36_222*RLcp36_166-OPcp36_26*RLcp36_119;

// = = Block_1_0_0_37_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp36_166;
sens->P[2] = POcp36_266;
sens->P[3] = POcp36_366;
sens->R[1][1] = ROcp36_122;
sens->R[1][2] = ROcp36_222;
sens->R[1][3] = ROcp36_322;
sens->R[2][1] = ROcp36_421;
sens->R[2][2] = ROcp36_521;
sens->R[2][3] = ROcp36_621;
sens->R[3][1] = ROcp36_722;
sens->R[3][2] = ROcp36_822;
sens->R[3][3] = ROcp36_922;
sens->V[1] = VIcp36_166;
sens->V[2] = VIcp36_266;
sens->V[3] = VIcp36_366;
sens->OM[1] = OMcp36_122;
sens->OM[2] = OMcp36_222;
sens->OM[3] = OMcp36_322;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp36_166_5;
sens->J[1][6] = JTcp36_166_6;
sens->J[1][19] = JTcp36_166_7;
sens->J[1][20] = JTcp36_166_8;
sens->J[1][21] = JTcp36_166_9;
sens->J[1][22] = JTcp36_166_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp36_266_4;
sens->J[2][5] = JTcp36_266_5;
sens->J[2][6] = JTcp36_266_6;
sens->J[2][19] = JTcp36_266_7;
sens->J[2][20] = JTcp36_266_8;
sens->J[2][21] = JTcp36_266_9;
sens->J[2][22] = JTcp36_266_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp36_366_4;
sens->J[3][5] = JTcp36_366_5;
sens->J[3][6] = JTcp36_366_6;
sens->J[3][19] = JTcp36_366_7;
sens->J[3][20] = JTcp36_366_8;
sens->J[3][21] = JTcp36_366_9;
sens->J[3][22] = JTcp36_366_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp36_16;
sens->J[4][20] = ROcp36_419;
sens->J[4][21] = ROcp36_720;
sens->J[4][22] = ROcp36_421;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp36_85;
sens->J[5][19] = ROcp36_26;
sens->J[5][20] = ROcp36_519;
sens->J[5][21] = ROcp36_820;
sens->J[5][22] = ROcp36_521;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp36_95;
sens->J[6][19] = ROcp36_36;
sens->J[6][20] = ROcp36_619;
sens->J[6][21] = ROcp36_920;
sens->J[6][22] = ROcp36_621;
sens->A[1] = ACcp36_166;
sens->A[2] = ACcp36_266;
sens->A[3] = ACcp36_366;
sens->OMP[1] = OPcp36_122;
sens->OMP[2] = OPcp36_222;
sens->OMP[3] = OPcp36_322;
 
// 
break;
case 38:
 


// = = Block_1_0_0_38_0_1 = = 
 
// Sensor Kinematics 


ROcp37_25 = S4*S5;
ROcp37_35 = -C4*S5;
ROcp37_85 = -S4*C5;
ROcp37_95 = C4*C5;
ROcp37_16 = C5*C6;
ROcp37_26 = ROcp37_25*C6+C4*S6;
ROcp37_36 = ROcp37_35*C6+S4*S6;
ROcp37_46 = -C5*S6;
ROcp37_56 = -(ROcp37_25*S6-C4*C6);
ROcp37_66 = -(ROcp37_35*S6-S4*C6);
OMcp37_25 = qd[5]*C4;
OMcp37_35 = qd[5]*S4;
OMcp37_16 = qd[4]+qd[6]*S5;
OMcp37_26 = OMcp37_25+ROcp37_85*qd[6];
OMcp37_36 = OMcp37_35+ROcp37_95*qd[6];
OPcp37_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp37_26 = ROcp37_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp37_35*S5-ROcp37_95*qd[4]);
OPcp37_36 = ROcp37_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp37_25*S5-ROcp37_85*qd[4]);

// = = Block_1_0_0_38_0_4 = = 
 
// Sensor Kinematics 


ROcp37_419 = ROcp37_46*C19+S19*S5;
ROcp37_519 = ROcp37_56*C19+ROcp37_85*S19;
ROcp37_619 = ROcp37_66*C19+ROcp37_95*S19;
ROcp37_719 = -(ROcp37_46*S19-C19*S5);
ROcp37_819 = -(ROcp37_56*S19-ROcp37_85*C19);
ROcp37_919 = -(ROcp37_66*S19-ROcp37_95*C19);
ROcp37_120 = ROcp37_16*C20-ROcp37_719*S20;
ROcp37_220 = ROcp37_26*C20-ROcp37_819*S20;
ROcp37_320 = ROcp37_36*C20-ROcp37_919*S20;
ROcp37_720 = ROcp37_16*S20+ROcp37_719*C20;
ROcp37_820 = ROcp37_26*S20+ROcp37_819*C20;
ROcp37_920 = ROcp37_36*S20+ROcp37_919*C20;
ROcp37_121 = ROcp37_120*C21+ROcp37_419*S21;
ROcp37_221 = ROcp37_220*C21+ROcp37_519*S21;
ROcp37_321 = ROcp37_320*C21+ROcp37_619*S21;
ROcp37_421 = -(ROcp37_120*S21-ROcp37_419*C21);
ROcp37_521 = -(ROcp37_220*S21-ROcp37_519*C21);
ROcp37_621 = -(ROcp37_320*S21-ROcp37_619*C21);
RLcp37_119 = s->dpt[2][3]*ROcp37_46+ROcp37_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp37_219 = s->dpt[2][3]*ROcp37_56+ROcp37_26*s->dpt[1][3]+ROcp37_85*s->dpt[3][3];
RLcp37_319 = s->dpt[2][3]*ROcp37_66+ROcp37_36*s->dpt[1][3]+ROcp37_95*s->dpt[3][3];
OMcp37_119 = OMcp37_16+ROcp37_16*qd[19];
OMcp37_219 = OMcp37_26+ROcp37_26*qd[19];
OMcp37_319 = OMcp37_36+ROcp37_36*qd[19];
ORcp37_119 = OMcp37_26*RLcp37_319-OMcp37_36*RLcp37_219;
ORcp37_219 = -(OMcp37_16*RLcp37_319-OMcp37_36*RLcp37_119);
ORcp37_319 = OMcp37_16*RLcp37_219-OMcp37_26*RLcp37_119;
OPcp37_119 = OPcp37_16+ROcp37_16*qdd[19]+qd[19]*(OMcp37_26*ROcp37_36-OMcp37_36*ROcp37_26);
OPcp37_219 = OPcp37_26+ROcp37_26*qdd[19]-qd[19]*(OMcp37_16*ROcp37_36-OMcp37_36*ROcp37_16);
OPcp37_319 = OPcp37_36+ROcp37_36*qdd[19]+qd[19]*(OMcp37_16*ROcp37_26-OMcp37_26*ROcp37_16);
RLcp37_120 = s->dpt[1][38]*ROcp37_16+s->dpt[2][38]*ROcp37_419+s->dpt[3][38]*ROcp37_719;
RLcp37_220 = s->dpt[1][38]*ROcp37_26+s->dpt[2][38]*ROcp37_519+s->dpt[3][38]*ROcp37_819;
RLcp37_320 = s->dpt[1][38]*ROcp37_36+s->dpt[2][38]*ROcp37_619+s->dpt[3][38]*ROcp37_919;
OMcp37_120 = OMcp37_119+ROcp37_419*qd[20];
OMcp37_220 = OMcp37_219+ROcp37_519*qd[20];
OMcp37_320 = OMcp37_319+ROcp37_619*qd[20];
ORcp37_120 = OMcp37_219*RLcp37_320-OMcp37_319*RLcp37_220;
ORcp37_220 = -(OMcp37_119*RLcp37_320-OMcp37_319*RLcp37_120);
ORcp37_320 = OMcp37_119*RLcp37_220-OMcp37_219*RLcp37_120;
OPcp37_120 = OPcp37_119+ROcp37_419*qdd[20]+qd[20]*(OMcp37_219*ROcp37_619-OMcp37_319*ROcp37_519);
OPcp37_220 = OPcp37_219+ROcp37_519*qdd[20]-qd[20]*(OMcp37_119*ROcp37_619-OMcp37_319*ROcp37_419);
OPcp37_320 = OPcp37_319+ROcp37_619*qdd[20]+qd[20]*(OMcp37_119*ROcp37_519-OMcp37_219*ROcp37_419);
RLcp37_121 = s->dpt[1][40]*ROcp37_120+s->dpt[2][40]*ROcp37_419+ROcp37_720*s->dpt[3][40];
RLcp37_221 = s->dpt[1][40]*ROcp37_220+s->dpt[2][40]*ROcp37_519+ROcp37_820*s->dpt[3][40];
RLcp37_321 = s->dpt[1][40]*ROcp37_320+s->dpt[2][40]*ROcp37_619+ROcp37_920*s->dpt[3][40];
OMcp37_121 = OMcp37_120+ROcp37_720*qd[21];
OMcp37_221 = OMcp37_220+ROcp37_820*qd[21];
OMcp37_321 = OMcp37_320+ROcp37_920*qd[21];
ORcp37_121 = OMcp37_220*RLcp37_321-OMcp37_320*RLcp37_221;
ORcp37_221 = -(OMcp37_120*RLcp37_321-OMcp37_320*RLcp37_121);
ORcp37_321 = OMcp37_120*RLcp37_221-OMcp37_220*RLcp37_121;
OPcp37_121 = OPcp37_120+ROcp37_720*qdd[21]+qd[21]*(OMcp37_220*ROcp37_920-OMcp37_320*ROcp37_820);
OPcp37_221 = OPcp37_220+ROcp37_820*qdd[21]-qd[21]*(OMcp37_120*ROcp37_920-OMcp37_320*ROcp37_720);
OPcp37_321 = OPcp37_320+ROcp37_920*qdd[21]+qd[21]*(OMcp37_120*ROcp37_820-OMcp37_220*ROcp37_720);

// = = Block_1_0_0_38_0_5 = = 
 
// Sensor Kinematics 


ROcp37_122 = ROcp37_121*C22-ROcp37_720*S22;
ROcp37_222 = ROcp37_221*C22-ROcp37_820*S22;
ROcp37_322 = ROcp37_321*C22-ROcp37_920*S22;
ROcp37_722 = ROcp37_121*S22+ROcp37_720*C22;
ROcp37_822 = ROcp37_221*S22+ROcp37_820*C22;
ROcp37_922 = ROcp37_321*S22+ROcp37_920*C22;
ROcp37_423 = ROcp37_421*C23+ROcp37_722*S23;
ROcp37_523 = ROcp37_521*C23+ROcp37_822*S23;
ROcp37_623 = ROcp37_621*C23+ROcp37_922*S23;
ROcp37_723 = -(ROcp37_421*S23-ROcp37_722*C23);
ROcp37_823 = -(ROcp37_521*S23-ROcp37_822*C23);
ROcp37_923 = -(ROcp37_621*S23-ROcp37_922*C23);
RLcp37_122 = ROcp37_121*s->dpt[1][44]+ROcp37_421*s->dpt[2][44]+ROcp37_720*s->dpt[3][44];
RLcp37_222 = ROcp37_221*s->dpt[1][44]+ROcp37_521*s->dpt[2][44]+ROcp37_820*s->dpt[3][44];
RLcp37_322 = ROcp37_321*s->dpt[1][44]+ROcp37_621*s->dpt[2][44]+ROcp37_920*s->dpt[3][44];
OMcp37_122 = OMcp37_121+ROcp37_421*qd[22];
OMcp37_222 = OMcp37_221+ROcp37_521*qd[22];
OMcp37_322 = OMcp37_321+ROcp37_621*qd[22];
ORcp37_122 = OMcp37_221*RLcp37_322-OMcp37_321*RLcp37_222;
ORcp37_222 = -(OMcp37_121*RLcp37_322-OMcp37_321*RLcp37_122);
ORcp37_322 = OMcp37_121*RLcp37_222-OMcp37_221*RLcp37_122;
OPcp37_122 = OPcp37_121+ROcp37_421*qdd[22]+qd[22]*(OMcp37_221*ROcp37_621-OMcp37_321*ROcp37_521);
OPcp37_222 = OPcp37_221+ROcp37_521*qdd[22]-qd[22]*(OMcp37_121*ROcp37_621-OMcp37_321*ROcp37_421);
OPcp37_322 = OPcp37_321+ROcp37_621*qdd[22]+qd[22]*(OMcp37_121*ROcp37_521-OMcp37_221*ROcp37_421);
RLcp37_123 = s->dpt[1][47]*ROcp37_122+s->dpt[3][47]*ROcp37_722+ROcp37_421*s->dpt[2][47];
RLcp37_223 = s->dpt[1][47]*ROcp37_222+s->dpt[3][47]*ROcp37_822+ROcp37_521*s->dpt[2][47];
RLcp37_323 = s->dpt[1][47]*ROcp37_322+s->dpt[3][47]*ROcp37_922+ROcp37_621*s->dpt[2][47];
OMcp37_123 = OMcp37_122+ROcp37_122*qd[23];
OMcp37_223 = OMcp37_222+ROcp37_222*qd[23];
OMcp37_323 = OMcp37_322+ROcp37_322*qd[23];
ORcp37_123 = OMcp37_222*RLcp37_323-OMcp37_322*RLcp37_223;
ORcp37_223 = -(OMcp37_122*RLcp37_323-OMcp37_322*RLcp37_123);
ORcp37_323 = OMcp37_122*RLcp37_223-OMcp37_222*RLcp37_123;
OPcp37_123 = OPcp37_122+ROcp37_122*qdd[23]+qd[23]*(OMcp37_222*ROcp37_322-OMcp37_322*ROcp37_222);
OPcp37_223 = OPcp37_222+ROcp37_222*qdd[23]-qd[23]*(OMcp37_122*ROcp37_322-OMcp37_322*ROcp37_122);
OPcp37_323 = OPcp37_322+ROcp37_322*qdd[23]+qd[23]*(OMcp37_122*ROcp37_222-OMcp37_222*ROcp37_122);
RLcp37_167 = ROcp37_122*s->dpt[1][48]+ROcp37_423*s->dpt[2][48]+ROcp37_723*s->dpt[3][48];
RLcp37_267 = ROcp37_222*s->dpt[1][48]+ROcp37_523*s->dpt[2][48]+ROcp37_823*s->dpt[3][48];
RLcp37_367 = ROcp37_322*s->dpt[1][48]+ROcp37_623*s->dpt[2][48]+ROcp37_923*s->dpt[3][48];
POcp37_167 = RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123+RLcp37_167+q[1];
POcp37_267 = RLcp37_219+RLcp37_220+RLcp37_221+RLcp37_222+RLcp37_223+RLcp37_267+q[2];
POcp37_367 = RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_367+q[3];
JTcp37_267_4 = -(RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_367);
JTcp37_367_4 = RLcp37_219+RLcp37_220+RLcp37_221+RLcp37_222+RLcp37_223+RLcp37_267;
JTcp37_167_5 = C4*(RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_367)-S4*(RLcp37_219+RLcp37_220)-S4*(
 RLcp37_221+RLcp37_222)-S4*(RLcp37_223+RLcp37_267);
JTcp37_267_5 = S4*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123+RLcp37_167);
JTcp37_367_5 = -C4*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123+RLcp37_167);
JTcp37_167_6 = ROcp37_85*(RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_367)-ROcp37_95*(RLcp37_219+
 RLcp37_220)-ROcp37_95*(RLcp37_221+RLcp37_222)-ROcp37_95*(RLcp37_223+RLcp37_267);
JTcp37_267_6 = RLcp37_167*ROcp37_95-RLcp37_323*S5-RLcp37_367*S5+ROcp37_95*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122
 +RLcp37_123)-S5*(RLcp37_319+RLcp37_320)-S5*(RLcp37_321+RLcp37_322);
JTcp37_367_6 = RLcp37_223*S5-ROcp37_85*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123)+S5*(RLcp37_219+
 RLcp37_220)+S5*(RLcp37_221+RLcp37_222)-RLcp37_167*ROcp37_85+RLcp37_267*S5;
JTcp37_167_7 = ROcp37_26*(RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323)-ROcp37_36*(RLcp37_220+RLcp37_221)-ROcp37_36*(
 RLcp37_222+RLcp37_223)-RLcp37_267*ROcp37_36+RLcp37_367*ROcp37_26;
JTcp37_267_7 = RLcp37_167*ROcp37_36-RLcp37_367*ROcp37_16-ROcp37_16*(RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323)+
 ROcp37_36*(RLcp37_120+RLcp37_121)+ROcp37_36*(RLcp37_122+RLcp37_123);
JTcp37_367_7 = ROcp37_16*(RLcp37_220+RLcp37_221+RLcp37_222+RLcp37_223)-ROcp37_26*(RLcp37_120+RLcp37_121)-ROcp37_26*(
 RLcp37_122+RLcp37_123)-RLcp37_167*ROcp37_26+RLcp37_267*ROcp37_16;
JTcp37_167_8 = ROcp37_519*(RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_367)-ROcp37_619*(RLcp37_221+RLcp37_222)-ROcp37_619*
 (RLcp37_223+RLcp37_267);
JTcp37_267_8 = -(ROcp37_419*(RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_367)-ROcp37_619*(RLcp37_121+RLcp37_122)-
 ROcp37_619*(RLcp37_123+RLcp37_167));
JTcp37_367_8 = ROcp37_419*(RLcp37_221+RLcp37_222+RLcp37_223+RLcp37_267)-ROcp37_519*(RLcp37_121+RLcp37_122)-ROcp37_519*
 (RLcp37_123+RLcp37_167);
JTcp37_167_9 = ROcp37_820*(RLcp37_322+RLcp37_323)-ROcp37_920*(RLcp37_222+RLcp37_223)-RLcp37_267*ROcp37_920+RLcp37_367*
 ROcp37_820;
JTcp37_267_9 = RLcp37_167*ROcp37_920-RLcp37_367*ROcp37_720-ROcp37_720*(RLcp37_322+RLcp37_323)+ROcp37_920*(RLcp37_122+
 RLcp37_123);
JTcp37_367_9 = ROcp37_720*(RLcp37_222+RLcp37_223)-ROcp37_820*(RLcp37_122+RLcp37_123)-RLcp37_167*ROcp37_820+RLcp37_267*
 ROcp37_720;
JTcp37_167_10 = ROcp37_521*(RLcp37_323+RLcp37_367)-ROcp37_621*(RLcp37_223+RLcp37_267);
JTcp37_267_10 = -(ROcp37_421*(RLcp37_323+RLcp37_367)-ROcp37_621*(RLcp37_123+RLcp37_167));
JTcp37_367_10 = ROcp37_421*(RLcp37_223+RLcp37_267)-ROcp37_521*(RLcp37_123+RLcp37_167);
JTcp37_167_11 = -(RLcp37_267*ROcp37_322-RLcp37_367*ROcp37_222);
JTcp37_267_11 = RLcp37_167*ROcp37_322-RLcp37_367*ROcp37_122;
JTcp37_367_11 = -(RLcp37_167*ROcp37_222-RLcp37_267*ROcp37_122);
ORcp37_167 = OMcp37_223*RLcp37_367-OMcp37_323*RLcp37_267;
ORcp37_267 = -(OMcp37_123*RLcp37_367-OMcp37_323*RLcp37_167);
ORcp37_367 = OMcp37_123*RLcp37_267-OMcp37_223*RLcp37_167;
VIcp37_167 = ORcp37_119+ORcp37_120+ORcp37_121+ORcp37_122+ORcp37_123+ORcp37_167+qd[1];
VIcp37_267 = ORcp37_219+ORcp37_220+ORcp37_221+ORcp37_222+ORcp37_223+ORcp37_267+qd[2];
VIcp37_367 = ORcp37_319+ORcp37_320+ORcp37_321+ORcp37_322+ORcp37_323+ORcp37_367+qd[3];
ACcp37_167 = qdd[1]+OMcp37_219*ORcp37_320+OMcp37_220*ORcp37_321+OMcp37_221*ORcp37_322+OMcp37_222*ORcp37_323+OMcp37_223
 *ORcp37_367+OMcp37_26*ORcp37_319-OMcp37_319*ORcp37_220-OMcp37_320*ORcp37_221-OMcp37_321*ORcp37_222-OMcp37_322*ORcp37_223-
 OMcp37_323*ORcp37_267-OMcp37_36*ORcp37_219+OPcp37_219*RLcp37_320+OPcp37_220*RLcp37_321+OPcp37_221*RLcp37_322+OPcp37_222*
 RLcp37_323+OPcp37_223*RLcp37_367+OPcp37_26*RLcp37_319-OPcp37_319*RLcp37_220-OPcp37_320*RLcp37_221-OPcp37_321*RLcp37_222-
 OPcp37_322*RLcp37_223-OPcp37_323*RLcp37_267-OPcp37_36*RLcp37_219;
ACcp37_267 = qdd[2]-OMcp37_119*ORcp37_320-OMcp37_120*ORcp37_321-OMcp37_121*ORcp37_322-OMcp37_122*ORcp37_323-OMcp37_123
 *ORcp37_367-OMcp37_16*ORcp37_319+OMcp37_319*ORcp37_120+OMcp37_320*ORcp37_121+OMcp37_321*ORcp37_122+OMcp37_322*ORcp37_123+
 OMcp37_323*ORcp37_167+OMcp37_36*ORcp37_119-OPcp37_119*RLcp37_320-OPcp37_120*RLcp37_321-OPcp37_121*RLcp37_322-OPcp37_122*
 RLcp37_323-OPcp37_123*RLcp37_367-OPcp37_16*RLcp37_319+OPcp37_319*RLcp37_120+OPcp37_320*RLcp37_121+OPcp37_321*RLcp37_122+
 OPcp37_322*RLcp37_123+OPcp37_323*RLcp37_167+OPcp37_36*RLcp37_119;
ACcp37_367 = qdd[3]+OMcp37_119*ORcp37_220+OMcp37_120*ORcp37_221+OMcp37_121*ORcp37_222+OMcp37_122*ORcp37_223+OMcp37_123
 *ORcp37_267+OMcp37_16*ORcp37_219-OMcp37_219*ORcp37_120-OMcp37_220*ORcp37_121-OMcp37_221*ORcp37_122-OMcp37_222*ORcp37_123-
 OMcp37_223*ORcp37_167-OMcp37_26*ORcp37_119+OPcp37_119*RLcp37_220+OPcp37_120*RLcp37_221+OPcp37_121*RLcp37_222+OPcp37_122*
 RLcp37_223+OPcp37_123*RLcp37_267+OPcp37_16*RLcp37_219-OPcp37_219*RLcp37_120-OPcp37_220*RLcp37_121-OPcp37_221*RLcp37_122-
 OPcp37_222*RLcp37_123-OPcp37_223*RLcp37_167-OPcp37_26*RLcp37_119;

// = = Block_1_0_0_38_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp37_167;
sens->P[2] = POcp37_267;
sens->P[3] = POcp37_367;
sens->R[1][1] = ROcp37_122;
sens->R[1][2] = ROcp37_222;
sens->R[1][3] = ROcp37_322;
sens->R[2][1] = ROcp37_423;
sens->R[2][2] = ROcp37_523;
sens->R[2][3] = ROcp37_623;
sens->R[3][1] = ROcp37_723;
sens->R[3][2] = ROcp37_823;
sens->R[3][3] = ROcp37_923;
sens->V[1] = VIcp37_167;
sens->V[2] = VIcp37_267;
sens->V[3] = VIcp37_367;
sens->OM[1] = OMcp37_123;
sens->OM[2] = OMcp37_223;
sens->OM[3] = OMcp37_323;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp37_167_5;
sens->J[1][6] = JTcp37_167_6;
sens->J[1][19] = JTcp37_167_7;
sens->J[1][20] = JTcp37_167_8;
sens->J[1][21] = JTcp37_167_9;
sens->J[1][22] = JTcp37_167_10;
sens->J[1][23] = JTcp37_167_11;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp37_267_4;
sens->J[2][5] = JTcp37_267_5;
sens->J[2][6] = JTcp37_267_6;
sens->J[2][19] = JTcp37_267_7;
sens->J[2][20] = JTcp37_267_8;
sens->J[2][21] = JTcp37_267_9;
sens->J[2][22] = JTcp37_267_10;
sens->J[2][23] = JTcp37_267_11;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp37_367_4;
sens->J[3][5] = JTcp37_367_5;
sens->J[3][6] = JTcp37_367_6;
sens->J[3][19] = JTcp37_367_7;
sens->J[3][20] = JTcp37_367_8;
sens->J[3][21] = JTcp37_367_9;
sens->J[3][22] = JTcp37_367_10;
sens->J[3][23] = JTcp37_367_11;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp37_16;
sens->J[4][20] = ROcp37_419;
sens->J[4][21] = ROcp37_720;
sens->J[4][22] = ROcp37_421;
sens->J[4][23] = ROcp37_122;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp37_85;
sens->J[5][19] = ROcp37_26;
sens->J[5][20] = ROcp37_519;
sens->J[5][21] = ROcp37_820;
sens->J[5][22] = ROcp37_521;
sens->J[5][23] = ROcp37_222;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp37_95;
sens->J[6][19] = ROcp37_36;
sens->J[6][20] = ROcp37_619;
sens->J[6][21] = ROcp37_920;
sens->J[6][22] = ROcp37_621;
sens->J[6][23] = ROcp37_322;
sens->A[1] = ACcp37_167;
sens->A[2] = ACcp37_267;
sens->A[3] = ACcp37_367;
sens->OMP[1] = OPcp37_123;
sens->OMP[2] = OPcp37_223;
sens->OMP[3] = OPcp37_323;
 
// 
break;
case 39:
 


// = = Block_1_0_0_39_0_1 = = 
 
// Sensor Kinematics 


ROcp38_25 = S4*S5;
ROcp38_35 = -C4*S5;
ROcp38_85 = -S4*C5;
ROcp38_95 = C4*C5;
ROcp38_16 = C5*C6;
ROcp38_26 = ROcp38_25*C6+C4*S6;
ROcp38_36 = ROcp38_35*C6+S4*S6;
ROcp38_46 = -C5*S6;
ROcp38_56 = -(ROcp38_25*S6-C4*C6);
ROcp38_66 = -(ROcp38_35*S6-S4*C6);
OMcp38_25 = qd[5]*C4;
OMcp38_35 = qd[5]*S4;
OMcp38_16 = qd[4]+qd[6]*S5;
OMcp38_26 = OMcp38_25+ROcp38_85*qd[6];
OMcp38_36 = OMcp38_35+ROcp38_95*qd[6];
OPcp38_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp38_26 = ROcp38_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp38_35*S5-ROcp38_95*qd[4]);
OPcp38_36 = ROcp38_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp38_25*S5-ROcp38_85*qd[4]);

// = = Block_1_0_0_39_0_4 = = 
 
// Sensor Kinematics 


ROcp38_419 = ROcp38_46*C19+S19*S5;
ROcp38_519 = ROcp38_56*C19+ROcp38_85*S19;
ROcp38_619 = ROcp38_66*C19+ROcp38_95*S19;
ROcp38_719 = -(ROcp38_46*S19-C19*S5);
ROcp38_819 = -(ROcp38_56*S19-ROcp38_85*C19);
ROcp38_919 = -(ROcp38_66*S19-ROcp38_95*C19);
ROcp38_120 = ROcp38_16*C20-ROcp38_719*S20;
ROcp38_220 = ROcp38_26*C20-ROcp38_819*S20;
ROcp38_320 = ROcp38_36*C20-ROcp38_919*S20;
ROcp38_720 = ROcp38_16*S20+ROcp38_719*C20;
ROcp38_820 = ROcp38_26*S20+ROcp38_819*C20;
ROcp38_920 = ROcp38_36*S20+ROcp38_919*C20;
ROcp38_121 = ROcp38_120*C21+ROcp38_419*S21;
ROcp38_221 = ROcp38_220*C21+ROcp38_519*S21;
ROcp38_321 = ROcp38_320*C21+ROcp38_619*S21;
ROcp38_421 = -(ROcp38_120*S21-ROcp38_419*C21);
ROcp38_521 = -(ROcp38_220*S21-ROcp38_519*C21);
ROcp38_621 = -(ROcp38_320*S21-ROcp38_619*C21);
RLcp38_119 = s->dpt[2][3]*ROcp38_46+ROcp38_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp38_219 = s->dpt[2][3]*ROcp38_56+ROcp38_26*s->dpt[1][3]+ROcp38_85*s->dpt[3][3];
RLcp38_319 = s->dpt[2][3]*ROcp38_66+ROcp38_36*s->dpt[1][3]+ROcp38_95*s->dpt[3][3];
OMcp38_119 = OMcp38_16+ROcp38_16*qd[19];
OMcp38_219 = OMcp38_26+ROcp38_26*qd[19];
OMcp38_319 = OMcp38_36+ROcp38_36*qd[19];
ORcp38_119 = OMcp38_26*RLcp38_319-OMcp38_36*RLcp38_219;
ORcp38_219 = -(OMcp38_16*RLcp38_319-OMcp38_36*RLcp38_119);
ORcp38_319 = OMcp38_16*RLcp38_219-OMcp38_26*RLcp38_119;
OPcp38_119 = OPcp38_16+ROcp38_16*qdd[19]+qd[19]*(OMcp38_26*ROcp38_36-OMcp38_36*ROcp38_26);
OPcp38_219 = OPcp38_26+ROcp38_26*qdd[19]-qd[19]*(OMcp38_16*ROcp38_36-OMcp38_36*ROcp38_16);
OPcp38_319 = OPcp38_36+ROcp38_36*qdd[19]+qd[19]*(OMcp38_16*ROcp38_26-OMcp38_26*ROcp38_16);
RLcp38_120 = s->dpt[1][38]*ROcp38_16+s->dpt[2][38]*ROcp38_419+s->dpt[3][38]*ROcp38_719;
RLcp38_220 = s->dpt[1][38]*ROcp38_26+s->dpt[2][38]*ROcp38_519+s->dpt[3][38]*ROcp38_819;
RLcp38_320 = s->dpt[1][38]*ROcp38_36+s->dpt[2][38]*ROcp38_619+s->dpt[3][38]*ROcp38_919;
OMcp38_120 = OMcp38_119+ROcp38_419*qd[20];
OMcp38_220 = OMcp38_219+ROcp38_519*qd[20];
OMcp38_320 = OMcp38_319+ROcp38_619*qd[20];
ORcp38_120 = OMcp38_219*RLcp38_320-OMcp38_319*RLcp38_220;
ORcp38_220 = -(OMcp38_119*RLcp38_320-OMcp38_319*RLcp38_120);
ORcp38_320 = OMcp38_119*RLcp38_220-OMcp38_219*RLcp38_120;
OPcp38_120 = OPcp38_119+ROcp38_419*qdd[20]+qd[20]*(OMcp38_219*ROcp38_619-OMcp38_319*ROcp38_519);
OPcp38_220 = OPcp38_219+ROcp38_519*qdd[20]-qd[20]*(OMcp38_119*ROcp38_619-OMcp38_319*ROcp38_419);
OPcp38_320 = OPcp38_319+ROcp38_619*qdd[20]+qd[20]*(OMcp38_119*ROcp38_519-OMcp38_219*ROcp38_419);
RLcp38_121 = s->dpt[1][40]*ROcp38_120+s->dpt[2][40]*ROcp38_419+ROcp38_720*s->dpt[3][40];
RLcp38_221 = s->dpt[1][40]*ROcp38_220+s->dpt[2][40]*ROcp38_519+ROcp38_820*s->dpt[3][40];
RLcp38_321 = s->dpt[1][40]*ROcp38_320+s->dpt[2][40]*ROcp38_619+ROcp38_920*s->dpt[3][40];
OMcp38_121 = OMcp38_120+ROcp38_720*qd[21];
OMcp38_221 = OMcp38_220+ROcp38_820*qd[21];
OMcp38_321 = OMcp38_320+ROcp38_920*qd[21];
ORcp38_121 = OMcp38_220*RLcp38_321-OMcp38_320*RLcp38_221;
ORcp38_221 = -(OMcp38_120*RLcp38_321-OMcp38_320*RLcp38_121);
ORcp38_321 = OMcp38_120*RLcp38_221-OMcp38_220*RLcp38_121;
OPcp38_121 = OPcp38_120+ROcp38_720*qdd[21]+qd[21]*(OMcp38_220*ROcp38_920-OMcp38_320*ROcp38_820);
OPcp38_221 = OPcp38_220+ROcp38_820*qdd[21]-qd[21]*(OMcp38_120*ROcp38_920-OMcp38_320*ROcp38_720);
OPcp38_321 = OPcp38_320+ROcp38_920*qdd[21]+qd[21]*(OMcp38_120*ROcp38_820-OMcp38_220*ROcp38_720);

// = = Block_1_0_0_39_0_5 = = 
 
// Sensor Kinematics 


ROcp38_122 = ROcp38_121*C22-ROcp38_720*S22;
ROcp38_222 = ROcp38_221*C22-ROcp38_820*S22;
ROcp38_322 = ROcp38_321*C22-ROcp38_920*S22;
ROcp38_722 = ROcp38_121*S22+ROcp38_720*C22;
ROcp38_822 = ROcp38_221*S22+ROcp38_820*C22;
ROcp38_922 = ROcp38_321*S22+ROcp38_920*C22;
ROcp38_423 = ROcp38_421*C23+ROcp38_722*S23;
ROcp38_523 = ROcp38_521*C23+ROcp38_822*S23;
ROcp38_623 = ROcp38_621*C23+ROcp38_922*S23;
ROcp38_723 = -(ROcp38_421*S23-ROcp38_722*C23);
ROcp38_823 = -(ROcp38_521*S23-ROcp38_822*C23);
ROcp38_923 = -(ROcp38_621*S23-ROcp38_922*C23);
RLcp38_122 = ROcp38_121*s->dpt[1][44]+ROcp38_421*s->dpt[2][44]+ROcp38_720*s->dpt[3][44];
RLcp38_222 = ROcp38_221*s->dpt[1][44]+ROcp38_521*s->dpt[2][44]+ROcp38_820*s->dpt[3][44];
RLcp38_322 = ROcp38_321*s->dpt[1][44]+ROcp38_621*s->dpt[2][44]+ROcp38_920*s->dpt[3][44];
OMcp38_122 = OMcp38_121+ROcp38_421*qd[22];
OMcp38_222 = OMcp38_221+ROcp38_521*qd[22];
OMcp38_322 = OMcp38_321+ROcp38_621*qd[22];
ORcp38_122 = OMcp38_221*RLcp38_322-OMcp38_321*RLcp38_222;
ORcp38_222 = -(OMcp38_121*RLcp38_322-OMcp38_321*RLcp38_122);
ORcp38_322 = OMcp38_121*RLcp38_222-OMcp38_221*RLcp38_122;
OPcp38_122 = OPcp38_121+ROcp38_421*qdd[22]+qd[22]*(OMcp38_221*ROcp38_621-OMcp38_321*ROcp38_521);
OPcp38_222 = OPcp38_221+ROcp38_521*qdd[22]-qd[22]*(OMcp38_121*ROcp38_621-OMcp38_321*ROcp38_421);
OPcp38_322 = OPcp38_321+ROcp38_621*qdd[22]+qd[22]*(OMcp38_121*ROcp38_521-OMcp38_221*ROcp38_421);
RLcp38_123 = s->dpt[1][47]*ROcp38_122+s->dpt[3][47]*ROcp38_722+ROcp38_421*s->dpt[2][47];
RLcp38_223 = s->dpt[1][47]*ROcp38_222+s->dpt[3][47]*ROcp38_822+ROcp38_521*s->dpt[2][47];
RLcp38_323 = s->dpt[1][47]*ROcp38_322+s->dpt[3][47]*ROcp38_922+ROcp38_621*s->dpt[2][47];
OMcp38_123 = OMcp38_122+ROcp38_122*qd[23];
OMcp38_223 = OMcp38_222+ROcp38_222*qd[23];
OMcp38_323 = OMcp38_322+ROcp38_322*qd[23];
ORcp38_123 = OMcp38_222*RLcp38_323-OMcp38_322*RLcp38_223;
ORcp38_223 = -(OMcp38_122*RLcp38_323-OMcp38_322*RLcp38_123);
ORcp38_323 = OMcp38_122*RLcp38_223-OMcp38_222*RLcp38_123;
OPcp38_123 = OPcp38_122+ROcp38_122*qdd[23]+qd[23]*(OMcp38_222*ROcp38_322-OMcp38_322*ROcp38_222);
OPcp38_223 = OPcp38_222+ROcp38_222*qdd[23]-qd[23]*(OMcp38_122*ROcp38_322-OMcp38_322*ROcp38_122);
OPcp38_323 = OPcp38_322+ROcp38_322*qdd[23]+qd[23]*(OMcp38_122*ROcp38_222-OMcp38_222*ROcp38_122);
RLcp38_168 = s->dpt[1][49]*ROcp38_122+s->dpt[2][49]*ROcp38_423+ROcp38_723*s->dpt[3][49];
RLcp38_268 = s->dpt[1][49]*ROcp38_222+s->dpt[2][49]*ROcp38_523+ROcp38_823*s->dpt[3][49];
RLcp38_368 = s->dpt[1][49]*ROcp38_322+s->dpt[2][49]*ROcp38_623+ROcp38_923*s->dpt[3][49];
POcp38_168 = RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123+RLcp38_168+q[1];
POcp38_268 = RLcp38_219+RLcp38_220+RLcp38_221+RLcp38_222+RLcp38_223+RLcp38_268+q[2];
POcp38_368 = RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_368+q[3];
JTcp38_268_4 = -(RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_368);
JTcp38_368_4 = RLcp38_219+RLcp38_220+RLcp38_221+RLcp38_222+RLcp38_223+RLcp38_268;
JTcp38_168_5 = C4*(RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_368)-S4*(RLcp38_219+RLcp38_220)-S4*(
 RLcp38_221+RLcp38_222)-S4*(RLcp38_223+RLcp38_268);
JTcp38_268_5 = S4*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123+RLcp38_168);
JTcp38_368_5 = -C4*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123+RLcp38_168);
JTcp38_168_6 = ROcp38_85*(RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_368)-ROcp38_95*(RLcp38_219+
 RLcp38_220)-ROcp38_95*(RLcp38_221+RLcp38_222)-ROcp38_95*(RLcp38_223+RLcp38_268);
JTcp38_268_6 = RLcp38_168*ROcp38_95-RLcp38_323*S5-RLcp38_368*S5+ROcp38_95*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122
 +RLcp38_123)-S5*(RLcp38_319+RLcp38_320)-S5*(RLcp38_321+RLcp38_322);
JTcp38_368_6 = RLcp38_223*S5-ROcp38_85*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123)+S5*(RLcp38_219+
 RLcp38_220)+S5*(RLcp38_221+RLcp38_222)-RLcp38_168*ROcp38_85+RLcp38_268*S5;
JTcp38_168_7 = ROcp38_26*(RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323)-ROcp38_36*(RLcp38_220+RLcp38_221)-ROcp38_36*(
 RLcp38_222+RLcp38_223)-RLcp38_268*ROcp38_36+RLcp38_368*ROcp38_26;
JTcp38_268_7 = RLcp38_168*ROcp38_36-RLcp38_368*ROcp38_16-ROcp38_16*(RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323)+
 ROcp38_36*(RLcp38_120+RLcp38_121)+ROcp38_36*(RLcp38_122+RLcp38_123);
JTcp38_368_7 = ROcp38_16*(RLcp38_220+RLcp38_221+RLcp38_222+RLcp38_223)-ROcp38_26*(RLcp38_120+RLcp38_121)-ROcp38_26*(
 RLcp38_122+RLcp38_123)-RLcp38_168*ROcp38_26+RLcp38_268*ROcp38_16;
JTcp38_168_8 = ROcp38_519*(RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_368)-ROcp38_619*(RLcp38_221+RLcp38_222)-ROcp38_619*
 (RLcp38_223+RLcp38_268);
JTcp38_268_8 = -(ROcp38_419*(RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_368)-ROcp38_619*(RLcp38_121+RLcp38_122)-
 ROcp38_619*(RLcp38_123+RLcp38_168));
JTcp38_368_8 = ROcp38_419*(RLcp38_221+RLcp38_222+RLcp38_223+RLcp38_268)-ROcp38_519*(RLcp38_121+RLcp38_122)-ROcp38_519*
 (RLcp38_123+RLcp38_168);
JTcp38_168_9 = ROcp38_820*(RLcp38_322+RLcp38_323)-ROcp38_920*(RLcp38_222+RLcp38_223)-RLcp38_268*ROcp38_920+RLcp38_368*
 ROcp38_820;
JTcp38_268_9 = RLcp38_168*ROcp38_920-RLcp38_368*ROcp38_720-ROcp38_720*(RLcp38_322+RLcp38_323)+ROcp38_920*(RLcp38_122+
 RLcp38_123);
JTcp38_368_9 = ROcp38_720*(RLcp38_222+RLcp38_223)-ROcp38_820*(RLcp38_122+RLcp38_123)-RLcp38_168*ROcp38_820+RLcp38_268*
 ROcp38_720;
JTcp38_168_10 = ROcp38_521*(RLcp38_323+RLcp38_368)-ROcp38_621*(RLcp38_223+RLcp38_268);
JTcp38_268_10 = -(ROcp38_421*(RLcp38_323+RLcp38_368)-ROcp38_621*(RLcp38_123+RLcp38_168));
JTcp38_368_10 = ROcp38_421*(RLcp38_223+RLcp38_268)-ROcp38_521*(RLcp38_123+RLcp38_168);
JTcp38_168_11 = -(RLcp38_268*ROcp38_322-RLcp38_368*ROcp38_222);
JTcp38_268_11 = RLcp38_168*ROcp38_322-RLcp38_368*ROcp38_122;
JTcp38_368_11 = -(RLcp38_168*ROcp38_222-RLcp38_268*ROcp38_122);
ORcp38_168 = OMcp38_223*RLcp38_368-OMcp38_323*RLcp38_268;
ORcp38_268 = -(OMcp38_123*RLcp38_368-OMcp38_323*RLcp38_168);
ORcp38_368 = OMcp38_123*RLcp38_268-OMcp38_223*RLcp38_168;
VIcp38_168 = ORcp38_119+ORcp38_120+ORcp38_121+ORcp38_122+ORcp38_123+ORcp38_168+qd[1];
VIcp38_268 = ORcp38_219+ORcp38_220+ORcp38_221+ORcp38_222+ORcp38_223+ORcp38_268+qd[2];
VIcp38_368 = ORcp38_319+ORcp38_320+ORcp38_321+ORcp38_322+ORcp38_323+ORcp38_368+qd[3];
ACcp38_168 = qdd[1]+OMcp38_219*ORcp38_320+OMcp38_220*ORcp38_321+OMcp38_221*ORcp38_322+OMcp38_222*ORcp38_323+OMcp38_223
 *ORcp38_368+OMcp38_26*ORcp38_319-OMcp38_319*ORcp38_220-OMcp38_320*ORcp38_221-OMcp38_321*ORcp38_222-OMcp38_322*ORcp38_223-
 OMcp38_323*ORcp38_268-OMcp38_36*ORcp38_219+OPcp38_219*RLcp38_320+OPcp38_220*RLcp38_321+OPcp38_221*RLcp38_322+OPcp38_222*
 RLcp38_323+OPcp38_223*RLcp38_368+OPcp38_26*RLcp38_319-OPcp38_319*RLcp38_220-OPcp38_320*RLcp38_221-OPcp38_321*RLcp38_222-
 OPcp38_322*RLcp38_223-OPcp38_323*RLcp38_268-OPcp38_36*RLcp38_219;
ACcp38_268 = qdd[2]-OMcp38_119*ORcp38_320-OMcp38_120*ORcp38_321-OMcp38_121*ORcp38_322-OMcp38_122*ORcp38_323-OMcp38_123
 *ORcp38_368-OMcp38_16*ORcp38_319+OMcp38_319*ORcp38_120+OMcp38_320*ORcp38_121+OMcp38_321*ORcp38_122+OMcp38_322*ORcp38_123+
 OMcp38_323*ORcp38_168+OMcp38_36*ORcp38_119-OPcp38_119*RLcp38_320-OPcp38_120*RLcp38_321-OPcp38_121*RLcp38_322-OPcp38_122*
 RLcp38_323-OPcp38_123*RLcp38_368-OPcp38_16*RLcp38_319+OPcp38_319*RLcp38_120+OPcp38_320*RLcp38_121+OPcp38_321*RLcp38_122+
 OPcp38_322*RLcp38_123+OPcp38_323*RLcp38_168+OPcp38_36*RLcp38_119;
ACcp38_368 = qdd[3]+OMcp38_119*ORcp38_220+OMcp38_120*ORcp38_221+OMcp38_121*ORcp38_222+OMcp38_122*ORcp38_223+OMcp38_123
 *ORcp38_268+OMcp38_16*ORcp38_219-OMcp38_219*ORcp38_120-OMcp38_220*ORcp38_121-OMcp38_221*ORcp38_122-OMcp38_222*ORcp38_123-
 OMcp38_223*ORcp38_168-OMcp38_26*ORcp38_119+OPcp38_119*RLcp38_220+OPcp38_120*RLcp38_221+OPcp38_121*RLcp38_222+OPcp38_122*
 RLcp38_223+OPcp38_123*RLcp38_268+OPcp38_16*RLcp38_219-OPcp38_219*RLcp38_120-OPcp38_220*RLcp38_121-OPcp38_221*RLcp38_122-
 OPcp38_222*RLcp38_123-OPcp38_223*RLcp38_168-OPcp38_26*RLcp38_119;

// = = Block_1_0_0_39_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp38_168;
sens->P[2] = POcp38_268;
sens->P[3] = POcp38_368;
sens->R[1][1] = ROcp38_122;
sens->R[1][2] = ROcp38_222;
sens->R[1][3] = ROcp38_322;
sens->R[2][1] = ROcp38_423;
sens->R[2][2] = ROcp38_523;
sens->R[2][3] = ROcp38_623;
sens->R[3][1] = ROcp38_723;
sens->R[3][2] = ROcp38_823;
sens->R[3][3] = ROcp38_923;
sens->V[1] = VIcp38_168;
sens->V[2] = VIcp38_268;
sens->V[3] = VIcp38_368;
sens->OM[1] = OMcp38_123;
sens->OM[2] = OMcp38_223;
sens->OM[3] = OMcp38_323;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp38_168_5;
sens->J[1][6] = JTcp38_168_6;
sens->J[1][19] = JTcp38_168_7;
sens->J[1][20] = JTcp38_168_8;
sens->J[1][21] = JTcp38_168_9;
sens->J[1][22] = JTcp38_168_10;
sens->J[1][23] = JTcp38_168_11;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp38_268_4;
sens->J[2][5] = JTcp38_268_5;
sens->J[2][6] = JTcp38_268_6;
sens->J[2][19] = JTcp38_268_7;
sens->J[2][20] = JTcp38_268_8;
sens->J[2][21] = JTcp38_268_9;
sens->J[2][22] = JTcp38_268_10;
sens->J[2][23] = JTcp38_268_11;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp38_368_4;
sens->J[3][5] = JTcp38_368_5;
sens->J[3][6] = JTcp38_368_6;
sens->J[3][19] = JTcp38_368_7;
sens->J[3][20] = JTcp38_368_8;
sens->J[3][21] = JTcp38_368_9;
sens->J[3][22] = JTcp38_368_10;
sens->J[3][23] = JTcp38_368_11;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp38_16;
sens->J[4][20] = ROcp38_419;
sens->J[4][21] = ROcp38_720;
sens->J[4][22] = ROcp38_421;
sens->J[4][23] = ROcp38_122;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp38_85;
sens->J[5][19] = ROcp38_26;
sens->J[5][20] = ROcp38_519;
sens->J[5][21] = ROcp38_820;
sens->J[5][22] = ROcp38_521;
sens->J[5][23] = ROcp38_222;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp38_95;
sens->J[6][19] = ROcp38_36;
sens->J[6][20] = ROcp38_619;
sens->J[6][21] = ROcp38_920;
sens->J[6][22] = ROcp38_621;
sens->J[6][23] = ROcp38_322;
sens->A[1] = ACcp38_168;
sens->A[2] = ACcp38_268;
sens->A[3] = ACcp38_368;
sens->OMP[1] = OPcp38_123;
sens->OMP[2] = OPcp38_223;
sens->OMP[3] = OPcp38_323;
 
// 
break;
case 40:
 


// = = Block_1_0_0_40_0_1 = = 
 
// Sensor Kinematics 


ROcp39_25 = S4*S5;
ROcp39_35 = -C4*S5;
ROcp39_85 = -S4*C5;
ROcp39_95 = C4*C5;
ROcp39_16 = C5*C6;
ROcp39_26 = ROcp39_25*C6+C4*S6;
ROcp39_36 = ROcp39_35*C6+S4*S6;
ROcp39_46 = -C5*S6;
ROcp39_56 = -(ROcp39_25*S6-C4*C6);
ROcp39_66 = -(ROcp39_35*S6-S4*C6);
OMcp39_25 = qd[5]*C4;
OMcp39_35 = qd[5]*S4;
OMcp39_16 = qd[4]+qd[6]*S5;
OMcp39_26 = OMcp39_25+ROcp39_85*qd[6];
OMcp39_36 = OMcp39_35+ROcp39_95*qd[6];
OPcp39_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp39_26 = ROcp39_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp39_35*S5-ROcp39_95*qd[4]);
OPcp39_36 = ROcp39_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp39_25*S5-ROcp39_85*qd[4]);

// = = Block_1_0_0_40_0_4 = = 
 
// Sensor Kinematics 


ROcp39_419 = ROcp39_46*C19+S19*S5;
ROcp39_519 = ROcp39_56*C19+ROcp39_85*S19;
ROcp39_619 = ROcp39_66*C19+ROcp39_95*S19;
ROcp39_719 = -(ROcp39_46*S19-C19*S5);
ROcp39_819 = -(ROcp39_56*S19-ROcp39_85*C19);
ROcp39_919 = -(ROcp39_66*S19-ROcp39_95*C19);
ROcp39_120 = ROcp39_16*C20-ROcp39_719*S20;
ROcp39_220 = ROcp39_26*C20-ROcp39_819*S20;
ROcp39_320 = ROcp39_36*C20-ROcp39_919*S20;
ROcp39_720 = ROcp39_16*S20+ROcp39_719*C20;
ROcp39_820 = ROcp39_26*S20+ROcp39_819*C20;
ROcp39_920 = ROcp39_36*S20+ROcp39_919*C20;
ROcp39_121 = ROcp39_120*C21+ROcp39_419*S21;
ROcp39_221 = ROcp39_220*C21+ROcp39_519*S21;
ROcp39_321 = ROcp39_320*C21+ROcp39_619*S21;
ROcp39_421 = -(ROcp39_120*S21-ROcp39_419*C21);
ROcp39_521 = -(ROcp39_220*S21-ROcp39_519*C21);
ROcp39_621 = -(ROcp39_320*S21-ROcp39_619*C21);
RLcp39_119 = s->dpt[2][3]*ROcp39_46+ROcp39_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp39_219 = s->dpt[2][3]*ROcp39_56+ROcp39_26*s->dpt[1][3]+ROcp39_85*s->dpt[3][3];
RLcp39_319 = s->dpt[2][3]*ROcp39_66+ROcp39_36*s->dpt[1][3]+ROcp39_95*s->dpt[3][3];
OMcp39_119 = OMcp39_16+ROcp39_16*qd[19];
OMcp39_219 = OMcp39_26+ROcp39_26*qd[19];
OMcp39_319 = OMcp39_36+ROcp39_36*qd[19];
ORcp39_119 = OMcp39_26*RLcp39_319-OMcp39_36*RLcp39_219;
ORcp39_219 = -(OMcp39_16*RLcp39_319-OMcp39_36*RLcp39_119);
ORcp39_319 = OMcp39_16*RLcp39_219-OMcp39_26*RLcp39_119;
OPcp39_119 = OPcp39_16+ROcp39_16*qdd[19]+qd[19]*(OMcp39_26*ROcp39_36-OMcp39_36*ROcp39_26);
OPcp39_219 = OPcp39_26+ROcp39_26*qdd[19]-qd[19]*(OMcp39_16*ROcp39_36-OMcp39_36*ROcp39_16);
OPcp39_319 = OPcp39_36+ROcp39_36*qdd[19]+qd[19]*(OMcp39_16*ROcp39_26-OMcp39_26*ROcp39_16);
RLcp39_120 = s->dpt[1][38]*ROcp39_16+s->dpt[2][38]*ROcp39_419+s->dpt[3][38]*ROcp39_719;
RLcp39_220 = s->dpt[1][38]*ROcp39_26+s->dpt[2][38]*ROcp39_519+s->dpt[3][38]*ROcp39_819;
RLcp39_320 = s->dpt[1][38]*ROcp39_36+s->dpt[2][38]*ROcp39_619+s->dpt[3][38]*ROcp39_919;
OMcp39_120 = OMcp39_119+ROcp39_419*qd[20];
OMcp39_220 = OMcp39_219+ROcp39_519*qd[20];
OMcp39_320 = OMcp39_319+ROcp39_619*qd[20];
ORcp39_120 = OMcp39_219*RLcp39_320-OMcp39_319*RLcp39_220;
ORcp39_220 = -(OMcp39_119*RLcp39_320-OMcp39_319*RLcp39_120);
ORcp39_320 = OMcp39_119*RLcp39_220-OMcp39_219*RLcp39_120;
OPcp39_120 = OPcp39_119+ROcp39_419*qdd[20]+qd[20]*(OMcp39_219*ROcp39_619-OMcp39_319*ROcp39_519);
OPcp39_220 = OPcp39_219+ROcp39_519*qdd[20]-qd[20]*(OMcp39_119*ROcp39_619-OMcp39_319*ROcp39_419);
OPcp39_320 = OPcp39_319+ROcp39_619*qdd[20]+qd[20]*(OMcp39_119*ROcp39_519-OMcp39_219*ROcp39_419);
RLcp39_121 = s->dpt[1][40]*ROcp39_120+s->dpt[2][40]*ROcp39_419+ROcp39_720*s->dpt[3][40];
RLcp39_221 = s->dpt[1][40]*ROcp39_220+s->dpt[2][40]*ROcp39_519+ROcp39_820*s->dpt[3][40];
RLcp39_321 = s->dpt[1][40]*ROcp39_320+s->dpt[2][40]*ROcp39_619+ROcp39_920*s->dpt[3][40];
OMcp39_121 = OMcp39_120+ROcp39_720*qd[21];
OMcp39_221 = OMcp39_220+ROcp39_820*qd[21];
OMcp39_321 = OMcp39_320+ROcp39_920*qd[21];
ORcp39_121 = OMcp39_220*RLcp39_321-OMcp39_320*RLcp39_221;
ORcp39_221 = -(OMcp39_120*RLcp39_321-OMcp39_320*RLcp39_121);
ORcp39_321 = OMcp39_120*RLcp39_221-OMcp39_220*RLcp39_121;
OPcp39_121 = OPcp39_120+ROcp39_720*qdd[21]+qd[21]*(OMcp39_220*ROcp39_920-OMcp39_320*ROcp39_820);
OPcp39_221 = OPcp39_220+ROcp39_820*qdd[21]-qd[21]*(OMcp39_120*ROcp39_920-OMcp39_320*ROcp39_720);
OPcp39_321 = OPcp39_320+ROcp39_920*qdd[21]+qd[21]*(OMcp39_120*ROcp39_820-OMcp39_220*ROcp39_720);

// = = Block_1_0_0_40_0_5 = = 
 
// Sensor Kinematics 


ROcp39_122 = ROcp39_121*C22-ROcp39_720*S22;
ROcp39_222 = ROcp39_221*C22-ROcp39_820*S22;
ROcp39_322 = ROcp39_321*C22-ROcp39_920*S22;
ROcp39_722 = ROcp39_121*S22+ROcp39_720*C22;
ROcp39_822 = ROcp39_221*S22+ROcp39_820*C22;
ROcp39_922 = ROcp39_321*S22+ROcp39_920*C22;
ROcp39_423 = ROcp39_421*C23+ROcp39_722*S23;
ROcp39_523 = ROcp39_521*C23+ROcp39_822*S23;
ROcp39_623 = ROcp39_621*C23+ROcp39_922*S23;
ROcp39_723 = -(ROcp39_421*S23-ROcp39_722*C23);
ROcp39_823 = -(ROcp39_521*S23-ROcp39_822*C23);
ROcp39_923 = -(ROcp39_621*S23-ROcp39_922*C23);
ROcp39_124 = ROcp39_122*C24+ROcp39_423*S24;
ROcp39_224 = ROcp39_222*C24+ROcp39_523*S24;
ROcp39_324 = ROcp39_322*C24+ROcp39_623*S24;
ROcp39_424 = -(ROcp39_122*S24-ROcp39_423*C24);
ROcp39_524 = -(ROcp39_222*S24-ROcp39_523*C24);
ROcp39_624 = -(ROcp39_322*S24-ROcp39_623*C24);
RLcp39_122 = ROcp39_121*s->dpt[1][44]+ROcp39_421*s->dpt[2][44]+ROcp39_720*s->dpt[3][44];
RLcp39_222 = ROcp39_221*s->dpt[1][44]+ROcp39_521*s->dpt[2][44]+ROcp39_820*s->dpt[3][44];
RLcp39_322 = ROcp39_321*s->dpt[1][44]+ROcp39_621*s->dpt[2][44]+ROcp39_920*s->dpt[3][44];
OMcp39_122 = OMcp39_121+ROcp39_421*qd[22];
OMcp39_222 = OMcp39_221+ROcp39_521*qd[22];
OMcp39_322 = OMcp39_321+ROcp39_621*qd[22];
ORcp39_122 = OMcp39_221*RLcp39_322-OMcp39_321*RLcp39_222;
ORcp39_222 = -(OMcp39_121*RLcp39_322-OMcp39_321*RLcp39_122);
ORcp39_322 = OMcp39_121*RLcp39_222-OMcp39_221*RLcp39_122;
OPcp39_122 = OPcp39_121+ROcp39_421*qdd[22]+qd[22]*(OMcp39_221*ROcp39_621-OMcp39_321*ROcp39_521);
OPcp39_222 = OPcp39_221+ROcp39_521*qdd[22]-qd[22]*(OMcp39_121*ROcp39_621-OMcp39_321*ROcp39_421);
OPcp39_322 = OPcp39_321+ROcp39_621*qdd[22]+qd[22]*(OMcp39_121*ROcp39_521-OMcp39_221*ROcp39_421);
RLcp39_123 = s->dpt[1][47]*ROcp39_122+s->dpt[3][47]*ROcp39_722+ROcp39_421*s->dpt[2][47];
RLcp39_223 = s->dpt[1][47]*ROcp39_222+s->dpt[3][47]*ROcp39_822+ROcp39_521*s->dpt[2][47];
RLcp39_323 = s->dpt[1][47]*ROcp39_322+s->dpt[3][47]*ROcp39_922+ROcp39_621*s->dpt[2][47];
OMcp39_123 = OMcp39_122+ROcp39_122*qd[23];
OMcp39_223 = OMcp39_222+ROcp39_222*qd[23];
OMcp39_323 = OMcp39_322+ROcp39_322*qd[23];
ORcp39_123 = OMcp39_222*RLcp39_323-OMcp39_322*RLcp39_223;
ORcp39_223 = -(OMcp39_122*RLcp39_323-OMcp39_322*RLcp39_123);
ORcp39_323 = OMcp39_122*RLcp39_223-OMcp39_222*RLcp39_123;
OPcp39_123 = OPcp39_122+ROcp39_122*qdd[23]+qd[23]*(OMcp39_222*ROcp39_322-OMcp39_322*ROcp39_222);
OPcp39_223 = OPcp39_222+ROcp39_222*qdd[23]-qd[23]*(OMcp39_122*ROcp39_322-OMcp39_322*ROcp39_122);
OPcp39_323 = OPcp39_322+ROcp39_322*qdd[23]+qd[23]*(OMcp39_122*ROcp39_222-OMcp39_222*ROcp39_122);
RLcp39_124 = s->dpt[1][49]*ROcp39_122+s->dpt[2][49]*ROcp39_423+ROcp39_723*s->dpt[3][49];
RLcp39_224 = s->dpt[1][49]*ROcp39_222+s->dpt[2][49]*ROcp39_523+ROcp39_823*s->dpt[3][49];
RLcp39_324 = s->dpt[1][49]*ROcp39_322+s->dpt[2][49]*ROcp39_623+ROcp39_923*s->dpt[3][49];
OMcp39_124 = OMcp39_123+ROcp39_723*qd[24];
OMcp39_224 = OMcp39_223+ROcp39_823*qd[24];
OMcp39_324 = OMcp39_323+ROcp39_923*qd[24];
ORcp39_124 = OMcp39_223*RLcp39_324-OMcp39_323*RLcp39_224;
ORcp39_224 = -(OMcp39_123*RLcp39_324-OMcp39_323*RLcp39_124);
ORcp39_324 = OMcp39_123*RLcp39_224-OMcp39_223*RLcp39_124;
OPcp39_124 = OPcp39_123+ROcp39_723*qdd[24]+qd[24]*(OMcp39_223*ROcp39_923-OMcp39_323*ROcp39_823);
OPcp39_224 = OPcp39_223+ROcp39_823*qdd[24]-qd[24]*(OMcp39_123*ROcp39_923-OMcp39_323*ROcp39_723);
OPcp39_324 = OPcp39_323+ROcp39_923*qdd[24]+qd[24]*(OMcp39_123*ROcp39_823-OMcp39_223*ROcp39_723);
RLcp39_169 = ROcp39_124*s->dpt[1][50]+ROcp39_424*s->dpt[2][50]+ROcp39_723*s->dpt[3][50];
RLcp39_269 = ROcp39_224*s->dpt[1][50]+ROcp39_524*s->dpt[2][50]+ROcp39_823*s->dpt[3][50];
RLcp39_369 = ROcp39_324*s->dpt[1][50]+ROcp39_624*s->dpt[2][50]+ROcp39_923*s->dpt[3][50];
POcp39_169 = RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_169+q[1];
POcp39_269 = RLcp39_219+RLcp39_220+RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_269+q[2];
POcp39_369 = RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_369+q[3];
JTcp39_269_4 = -(RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_369);
JTcp39_369_4 = RLcp39_219+RLcp39_220+RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_269;
JTcp39_169_5 = C4*(RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)-S4*(RLcp39_219+RLcp39_220)-S4*(
 RLcp39_221+RLcp39_222)-S4*(RLcp39_223+RLcp39_224)-RLcp39_269*S4+RLcp39_369*C4;
JTcp39_269_5 = S4*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_169);
JTcp39_369_5 = -C4*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_169);
JTcp39_169_6 = ROcp39_85*(RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)-ROcp39_95*(RLcp39_219+
 RLcp39_220)-ROcp39_95*(RLcp39_221+RLcp39_222)-ROcp39_95*(RLcp39_223+RLcp39_224)-RLcp39_269*ROcp39_95+RLcp39_369*ROcp39_85;
JTcp39_269_6 = -(RLcp39_369*S5-ROcp39_95*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_169
 )+S5*(RLcp39_319+RLcp39_320)+S5*(RLcp39_321+RLcp39_322)+S5*(RLcp39_323+RLcp39_324));
JTcp39_369_6 = RLcp39_269*S5-ROcp39_85*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_169)+
 S5*(RLcp39_219+RLcp39_220)+S5*(RLcp39_221+RLcp39_222)+S5*(RLcp39_223+RLcp39_224);
JTcp39_169_7 = ROcp39_26*(RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_369)-ROcp39_36*(RLcp39_220+
 RLcp39_221)-ROcp39_36*(RLcp39_222+RLcp39_223)-ROcp39_36*(RLcp39_224+RLcp39_269);
JTcp39_269_7 = -(ROcp39_16*(RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_369)-ROcp39_36*(RLcp39_120+
 RLcp39_121)-ROcp39_36*(RLcp39_122+RLcp39_123)-ROcp39_36*(RLcp39_124+RLcp39_169));
JTcp39_369_7 = ROcp39_16*(RLcp39_220+RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_269)-ROcp39_26*(RLcp39_120+
 RLcp39_121)-ROcp39_26*(RLcp39_122+RLcp39_123)-ROcp39_26*(RLcp39_124+RLcp39_169);
JTcp39_169_8 = ROcp39_519*(RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)-ROcp39_619*(RLcp39_221+RLcp39_222)-ROcp39_619*
 (RLcp39_223+RLcp39_224)-RLcp39_269*ROcp39_619+RLcp39_369*ROcp39_519;
JTcp39_269_8 = RLcp39_169*ROcp39_619-RLcp39_369*ROcp39_419-ROcp39_419*(RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)+
 ROcp39_619*(RLcp39_121+RLcp39_122)+ROcp39_619*(RLcp39_123+RLcp39_124);
JTcp39_369_8 = ROcp39_419*(RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224)-ROcp39_519*(RLcp39_121+RLcp39_122)-ROcp39_519*
 (RLcp39_123+RLcp39_124)-RLcp39_169*ROcp39_519+RLcp39_269*ROcp39_419;
JTcp39_169_9 = ROcp39_820*(RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_369)-ROcp39_920*(RLcp39_222+RLcp39_223)-ROcp39_920*
 (RLcp39_224+RLcp39_269);
JTcp39_269_9 = -(ROcp39_720*(RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_369)-ROcp39_920*(RLcp39_122+RLcp39_123)-
 ROcp39_920*(RLcp39_124+RLcp39_169));
JTcp39_369_9 = ROcp39_720*(RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_269)-ROcp39_820*(RLcp39_122+RLcp39_123)-ROcp39_820*
 (RLcp39_124+RLcp39_169);
JTcp39_169_10 = ROcp39_521*(RLcp39_323+RLcp39_324)-ROcp39_621*(RLcp39_223+RLcp39_224)-RLcp39_269*ROcp39_621+RLcp39_369
 *ROcp39_521;
JTcp39_269_10 = RLcp39_169*ROcp39_621-RLcp39_369*ROcp39_421-ROcp39_421*(RLcp39_323+RLcp39_324)+ROcp39_621*(RLcp39_123+
 RLcp39_124);
JTcp39_369_10 = ROcp39_421*(RLcp39_223+RLcp39_224)-ROcp39_521*(RLcp39_123+RLcp39_124)-RLcp39_169*ROcp39_521+RLcp39_269
 *ROcp39_421;
JTcp39_169_11 = ROcp39_222*(RLcp39_324+RLcp39_369)-ROcp39_322*(RLcp39_224+RLcp39_269);
JTcp39_269_11 = -(ROcp39_122*(RLcp39_324+RLcp39_369)-ROcp39_322*(RLcp39_124+RLcp39_169));
JTcp39_369_11 = ROcp39_122*(RLcp39_224+RLcp39_269)-ROcp39_222*(RLcp39_124+RLcp39_169);
JTcp39_169_12 = -(RLcp39_269*ROcp39_923-RLcp39_369*ROcp39_823);
JTcp39_269_12 = RLcp39_169*ROcp39_923-RLcp39_369*ROcp39_723;
JTcp39_369_12 = -(RLcp39_169*ROcp39_823-RLcp39_269*ROcp39_723);
ORcp39_169 = OMcp39_224*RLcp39_369-OMcp39_324*RLcp39_269;
ORcp39_269 = -(OMcp39_124*RLcp39_369-OMcp39_324*RLcp39_169);
ORcp39_369 = OMcp39_124*RLcp39_269-OMcp39_224*RLcp39_169;
VIcp39_169 = ORcp39_119+ORcp39_120+ORcp39_121+ORcp39_122+ORcp39_123+ORcp39_124+ORcp39_169+qd[1];
VIcp39_269 = ORcp39_219+ORcp39_220+ORcp39_221+ORcp39_222+ORcp39_223+ORcp39_224+ORcp39_269+qd[2];
VIcp39_369 = ORcp39_319+ORcp39_320+ORcp39_321+ORcp39_322+ORcp39_323+ORcp39_324+ORcp39_369+qd[3];
ACcp39_169 = qdd[1]+OMcp39_219*ORcp39_320+OMcp39_220*ORcp39_321+OMcp39_221*ORcp39_322+OMcp39_222*ORcp39_323+OMcp39_223
 *ORcp39_324+OMcp39_224*ORcp39_369+OMcp39_26*ORcp39_319-OMcp39_319*ORcp39_220-OMcp39_320*ORcp39_221-OMcp39_321*ORcp39_222-
 OMcp39_322*ORcp39_223-OMcp39_323*ORcp39_224-OMcp39_324*ORcp39_269-OMcp39_36*ORcp39_219+OPcp39_219*RLcp39_320+OPcp39_220*
 RLcp39_321+OPcp39_221*RLcp39_322+OPcp39_222*RLcp39_323+OPcp39_223*RLcp39_324+OPcp39_224*RLcp39_369+OPcp39_26*RLcp39_319-
 OPcp39_319*RLcp39_220-OPcp39_320*RLcp39_221-OPcp39_321*RLcp39_222-OPcp39_322*RLcp39_223-OPcp39_323*RLcp39_224-OPcp39_324*
 RLcp39_269-OPcp39_36*RLcp39_219;
ACcp39_269 = qdd[2]-OMcp39_119*ORcp39_320-OMcp39_120*ORcp39_321-OMcp39_121*ORcp39_322-OMcp39_122*ORcp39_323-OMcp39_123
 *ORcp39_324-OMcp39_124*ORcp39_369-OMcp39_16*ORcp39_319+OMcp39_319*ORcp39_120+OMcp39_320*ORcp39_121+OMcp39_321*ORcp39_122+
 OMcp39_322*ORcp39_123+OMcp39_323*ORcp39_124+OMcp39_324*ORcp39_169+OMcp39_36*ORcp39_119-OPcp39_119*RLcp39_320-OPcp39_120*
 RLcp39_321-OPcp39_121*RLcp39_322-OPcp39_122*RLcp39_323-OPcp39_123*RLcp39_324-OPcp39_124*RLcp39_369-OPcp39_16*RLcp39_319+
 OPcp39_319*RLcp39_120+OPcp39_320*RLcp39_121+OPcp39_321*RLcp39_122+OPcp39_322*RLcp39_123+OPcp39_323*RLcp39_124+OPcp39_324*
 RLcp39_169+OPcp39_36*RLcp39_119;
ACcp39_369 = qdd[3]+OMcp39_119*ORcp39_220+OMcp39_120*ORcp39_221+OMcp39_121*ORcp39_222+OMcp39_122*ORcp39_223+OMcp39_123
 *ORcp39_224+OMcp39_124*ORcp39_269+OMcp39_16*ORcp39_219-OMcp39_219*ORcp39_120-OMcp39_220*ORcp39_121-OMcp39_221*ORcp39_122-
 OMcp39_222*ORcp39_123-OMcp39_223*ORcp39_124-OMcp39_224*ORcp39_169-OMcp39_26*ORcp39_119+OPcp39_119*RLcp39_220+OPcp39_120*
 RLcp39_221+OPcp39_121*RLcp39_222+OPcp39_122*RLcp39_223+OPcp39_123*RLcp39_224+OPcp39_124*RLcp39_269+OPcp39_16*RLcp39_219-
 OPcp39_219*RLcp39_120-OPcp39_220*RLcp39_121-OPcp39_221*RLcp39_122-OPcp39_222*RLcp39_123-OPcp39_223*RLcp39_124-OPcp39_224*
 RLcp39_169-OPcp39_26*RLcp39_119;

// = = Block_1_0_0_40_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp39_169;
sens->P[2] = POcp39_269;
sens->P[3] = POcp39_369;
sens->R[1][1] = ROcp39_124;
sens->R[1][2] = ROcp39_224;
sens->R[1][3] = ROcp39_324;
sens->R[2][1] = ROcp39_424;
sens->R[2][2] = ROcp39_524;
sens->R[2][3] = ROcp39_624;
sens->R[3][1] = ROcp39_723;
sens->R[3][2] = ROcp39_823;
sens->R[3][3] = ROcp39_923;
sens->V[1] = VIcp39_169;
sens->V[2] = VIcp39_269;
sens->V[3] = VIcp39_369;
sens->OM[1] = OMcp39_124;
sens->OM[2] = OMcp39_224;
sens->OM[3] = OMcp39_324;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp39_169_5;
sens->J[1][6] = JTcp39_169_6;
sens->J[1][19] = JTcp39_169_7;
sens->J[1][20] = JTcp39_169_8;
sens->J[1][21] = JTcp39_169_9;
sens->J[1][22] = JTcp39_169_10;
sens->J[1][23] = JTcp39_169_11;
sens->J[1][24] = JTcp39_169_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp39_269_4;
sens->J[2][5] = JTcp39_269_5;
sens->J[2][6] = JTcp39_269_6;
sens->J[2][19] = JTcp39_269_7;
sens->J[2][20] = JTcp39_269_8;
sens->J[2][21] = JTcp39_269_9;
sens->J[2][22] = JTcp39_269_10;
sens->J[2][23] = JTcp39_269_11;
sens->J[2][24] = JTcp39_269_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp39_369_4;
sens->J[3][5] = JTcp39_369_5;
sens->J[3][6] = JTcp39_369_6;
sens->J[3][19] = JTcp39_369_7;
sens->J[3][20] = JTcp39_369_8;
sens->J[3][21] = JTcp39_369_9;
sens->J[3][22] = JTcp39_369_10;
sens->J[3][23] = JTcp39_369_11;
sens->J[3][24] = JTcp39_369_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp39_16;
sens->J[4][20] = ROcp39_419;
sens->J[4][21] = ROcp39_720;
sens->J[4][22] = ROcp39_421;
sens->J[4][23] = ROcp39_122;
sens->J[4][24] = ROcp39_723;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp39_85;
sens->J[5][19] = ROcp39_26;
sens->J[5][20] = ROcp39_519;
sens->J[5][21] = ROcp39_820;
sens->J[5][22] = ROcp39_521;
sens->J[5][23] = ROcp39_222;
sens->J[5][24] = ROcp39_823;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp39_95;
sens->J[6][19] = ROcp39_36;
sens->J[6][20] = ROcp39_619;
sens->J[6][21] = ROcp39_920;
sens->J[6][22] = ROcp39_621;
sens->J[6][23] = ROcp39_322;
sens->J[6][24] = ROcp39_923;
sens->A[1] = ACcp39_169;
sens->A[2] = ACcp39_269;
sens->A[3] = ACcp39_369;
sens->OMP[1] = OPcp39_124;
sens->OMP[2] = OPcp39_224;
sens->OMP[3] = OPcp39_324;
 
// 
break;
case 41:
 


// = = Block_1_0_0_41_0_1 = = 
 
// Sensor Kinematics 


ROcp40_25 = S4*S5;
ROcp40_35 = -C4*S5;
ROcp40_85 = -S4*C5;
ROcp40_95 = C4*C5;
ROcp40_16 = C5*C6;
ROcp40_26 = ROcp40_25*C6+C4*S6;
ROcp40_36 = ROcp40_35*C6+S4*S6;
ROcp40_46 = -C5*S6;
ROcp40_56 = -(ROcp40_25*S6-C4*C6);
ROcp40_66 = -(ROcp40_35*S6-S4*C6);
OMcp40_25 = qd[5]*C4;
OMcp40_35 = qd[5]*S4;
OMcp40_16 = qd[4]+qd[6]*S5;
OMcp40_26 = OMcp40_25+ROcp40_85*qd[6];
OMcp40_36 = OMcp40_35+ROcp40_95*qd[6];
OPcp40_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp40_26 = ROcp40_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp40_35*S5-ROcp40_95*qd[4]);
OPcp40_36 = ROcp40_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp40_25*S5-ROcp40_85*qd[4]);

// = = Block_1_0_0_41_0_4 = = 
 
// Sensor Kinematics 


ROcp40_419 = ROcp40_46*C19+S19*S5;
ROcp40_519 = ROcp40_56*C19+ROcp40_85*S19;
ROcp40_619 = ROcp40_66*C19+ROcp40_95*S19;
ROcp40_719 = -(ROcp40_46*S19-C19*S5);
ROcp40_819 = -(ROcp40_56*S19-ROcp40_85*C19);
ROcp40_919 = -(ROcp40_66*S19-ROcp40_95*C19);
ROcp40_120 = ROcp40_16*C20-ROcp40_719*S20;
ROcp40_220 = ROcp40_26*C20-ROcp40_819*S20;
ROcp40_320 = ROcp40_36*C20-ROcp40_919*S20;
ROcp40_720 = ROcp40_16*S20+ROcp40_719*C20;
ROcp40_820 = ROcp40_26*S20+ROcp40_819*C20;
ROcp40_920 = ROcp40_36*S20+ROcp40_919*C20;
ROcp40_121 = ROcp40_120*C21+ROcp40_419*S21;
ROcp40_221 = ROcp40_220*C21+ROcp40_519*S21;
ROcp40_321 = ROcp40_320*C21+ROcp40_619*S21;
ROcp40_421 = -(ROcp40_120*S21-ROcp40_419*C21);
ROcp40_521 = -(ROcp40_220*S21-ROcp40_519*C21);
ROcp40_621 = -(ROcp40_320*S21-ROcp40_619*C21);
RLcp40_119 = s->dpt[2][3]*ROcp40_46+ROcp40_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp40_219 = s->dpt[2][3]*ROcp40_56+ROcp40_26*s->dpt[1][3]+ROcp40_85*s->dpt[3][3];
RLcp40_319 = s->dpt[2][3]*ROcp40_66+ROcp40_36*s->dpt[1][3]+ROcp40_95*s->dpt[3][3];
OMcp40_119 = OMcp40_16+ROcp40_16*qd[19];
OMcp40_219 = OMcp40_26+ROcp40_26*qd[19];
OMcp40_319 = OMcp40_36+ROcp40_36*qd[19];
ORcp40_119 = OMcp40_26*RLcp40_319-OMcp40_36*RLcp40_219;
ORcp40_219 = -(OMcp40_16*RLcp40_319-OMcp40_36*RLcp40_119);
ORcp40_319 = OMcp40_16*RLcp40_219-OMcp40_26*RLcp40_119;
OPcp40_119 = OPcp40_16+ROcp40_16*qdd[19]+qd[19]*(OMcp40_26*ROcp40_36-OMcp40_36*ROcp40_26);
OPcp40_219 = OPcp40_26+ROcp40_26*qdd[19]-qd[19]*(OMcp40_16*ROcp40_36-OMcp40_36*ROcp40_16);
OPcp40_319 = OPcp40_36+ROcp40_36*qdd[19]+qd[19]*(OMcp40_16*ROcp40_26-OMcp40_26*ROcp40_16);
RLcp40_120 = s->dpt[1][38]*ROcp40_16+s->dpt[2][38]*ROcp40_419+s->dpt[3][38]*ROcp40_719;
RLcp40_220 = s->dpt[1][38]*ROcp40_26+s->dpt[2][38]*ROcp40_519+s->dpt[3][38]*ROcp40_819;
RLcp40_320 = s->dpt[1][38]*ROcp40_36+s->dpt[2][38]*ROcp40_619+s->dpt[3][38]*ROcp40_919;
OMcp40_120 = OMcp40_119+ROcp40_419*qd[20];
OMcp40_220 = OMcp40_219+ROcp40_519*qd[20];
OMcp40_320 = OMcp40_319+ROcp40_619*qd[20];
ORcp40_120 = OMcp40_219*RLcp40_320-OMcp40_319*RLcp40_220;
ORcp40_220 = -(OMcp40_119*RLcp40_320-OMcp40_319*RLcp40_120);
ORcp40_320 = OMcp40_119*RLcp40_220-OMcp40_219*RLcp40_120;
OPcp40_120 = OPcp40_119+ROcp40_419*qdd[20]+qd[20]*(OMcp40_219*ROcp40_619-OMcp40_319*ROcp40_519);
OPcp40_220 = OPcp40_219+ROcp40_519*qdd[20]-qd[20]*(OMcp40_119*ROcp40_619-OMcp40_319*ROcp40_419);
OPcp40_320 = OPcp40_319+ROcp40_619*qdd[20]+qd[20]*(OMcp40_119*ROcp40_519-OMcp40_219*ROcp40_419);
RLcp40_121 = s->dpt[1][40]*ROcp40_120+s->dpt[2][40]*ROcp40_419+ROcp40_720*s->dpt[3][40];
RLcp40_221 = s->dpt[1][40]*ROcp40_220+s->dpt[2][40]*ROcp40_519+ROcp40_820*s->dpt[3][40];
RLcp40_321 = s->dpt[1][40]*ROcp40_320+s->dpt[2][40]*ROcp40_619+ROcp40_920*s->dpt[3][40];
OMcp40_121 = OMcp40_120+ROcp40_720*qd[21];
OMcp40_221 = OMcp40_220+ROcp40_820*qd[21];
OMcp40_321 = OMcp40_320+ROcp40_920*qd[21];
ORcp40_121 = OMcp40_220*RLcp40_321-OMcp40_320*RLcp40_221;
ORcp40_221 = -(OMcp40_120*RLcp40_321-OMcp40_320*RLcp40_121);
ORcp40_321 = OMcp40_120*RLcp40_221-OMcp40_220*RLcp40_121;
OPcp40_121 = OPcp40_120+ROcp40_720*qdd[21]+qd[21]*(OMcp40_220*ROcp40_920-OMcp40_320*ROcp40_820);
OPcp40_221 = OPcp40_220+ROcp40_820*qdd[21]-qd[21]*(OMcp40_120*ROcp40_920-OMcp40_320*ROcp40_720);
OPcp40_321 = OPcp40_320+ROcp40_920*qdd[21]+qd[21]*(OMcp40_120*ROcp40_820-OMcp40_220*ROcp40_720);

// = = Block_1_0_0_41_0_5 = = 
 
// Sensor Kinematics 


ROcp40_122 = ROcp40_121*C22-ROcp40_720*S22;
ROcp40_222 = ROcp40_221*C22-ROcp40_820*S22;
ROcp40_322 = ROcp40_321*C22-ROcp40_920*S22;
ROcp40_722 = ROcp40_121*S22+ROcp40_720*C22;
ROcp40_822 = ROcp40_221*S22+ROcp40_820*C22;
ROcp40_922 = ROcp40_321*S22+ROcp40_920*C22;
ROcp40_423 = ROcp40_421*C23+ROcp40_722*S23;
ROcp40_523 = ROcp40_521*C23+ROcp40_822*S23;
ROcp40_623 = ROcp40_621*C23+ROcp40_922*S23;
ROcp40_723 = -(ROcp40_421*S23-ROcp40_722*C23);
ROcp40_823 = -(ROcp40_521*S23-ROcp40_822*C23);
ROcp40_923 = -(ROcp40_621*S23-ROcp40_922*C23);
ROcp40_124 = ROcp40_122*C24+ROcp40_423*S24;
ROcp40_224 = ROcp40_222*C24+ROcp40_523*S24;
ROcp40_324 = ROcp40_322*C24+ROcp40_623*S24;
ROcp40_424 = -(ROcp40_122*S24-ROcp40_423*C24);
ROcp40_524 = -(ROcp40_222*S24-ROcp40_523*C24);
ROcp40_624 = -(ROcp40_322*S24-ROcp40_623*C24);
RLcp40_122 = ROcp40_121*s->dpt[1][44]+ROcp40_421*s->dpt[2][44]+ROcp40_720*s->dpt[3][44];
RLcp40_222 = ROcp40_221*s->dpt[1][44]+ROcp40_521*s->dpt[2][44]+ROcp40_820*s->dpt[3][44];
RLcp40_322 = ROcp40_321*s->dpt[1][44]+ROcp40_621*s->dpt[2][44]+ROcp40_920*s->dpt[3][44];
OMcp40_122 = OMcp40_121+ROcp40_421*qd[22];
OMcp40_222 = OMcp40_221+ROcp40_521*qd[22];
OMcp40_322 = OMcp40_321+ROcp40_621*qd[22];
ORcp40_122 = OMcp40_221*RLcp40_322-OMcp40_321*RLcp40_222;
ORcp40_222 = -(OMcp40_121*RLcp40_322-OMcp40_321*RLcp40_122);
ORcp40_322 = OMcp40_121*RLcp40_222-OMcp40_221*RLcp40_122;
OPcp40_122 = OPcp40_121+ROcp40_421*qdd[22]+qd[22]*(OMcp40_221*ROcp40_621-OMcp40_321*ROcp40_521);
OPcp40_222 = OPcp40_221+ROcp40_521*qdd[22]-qd[22]*(OMcp40_121*ROcp40_621-OMcp40_321*ROcp40_421);
OPcp40_322 = OPcp40_321+ROcp40_621*qdd[22]+qd[22]*(OMcp40_121*ROcp40_521-OMcp40_221*ROcp40_421);
RLcp40_123 = s->dpt[1][47]*ROcp40_122+s->dpt[3][47]*ROcp40_722+ROcp40_421*s->dpt[2][47];
RLcp40_223 = s->dpt[1][47]*ROcp40_222+s->dpt[3][47]*ROcp40_822+ROcp40_521*s->dpt[2][47];
RLcp40_323 = s->dpt[1][47]*ROcp40_322+s->dpt[3][47]*ROcp40_922+ROcp40_621*s->dpt[2][47];
OMcp40_123 = OMcp40_122+ROcp40_122*qd[23];
OMcp40_223 = OMcp40_222+ROcp40_222*qd[23];
OMcp40_323 = OMcp40_322+ROcp40_322*qd[23];
ORcp40_123 = OMcp40_222*RLcp40_323-OMcp40_322*RLcp40_223;
ORcp40_223 = -(OMcp40_122*RLcp40_323-OMcp40_322*RLcp40_123);
ORcp40_323 = OMcp40_122*RLcp40_223-OMcp40_222*RLcp40_123;
OPcp40_123 = OPcp40_122+ROcp40_122*qdd[23]+qd[23]*(OMcp40_222*ROcp40_322-OMcp40_322*ROcp40_222);
OPcp40_223 = OPcp40_222+ROcp40_222*qdd[23]-qd[23]*(OMcp40_122*ROcp40_322-OMcp40_322*ROcp40_122);
OPcp40_323 = OPcp40_322+ROcp40_322*qdd[23]+qd[23]*(OMcp40_122*ROcp40_222-OMcp40_222*ROcp40_122);
RLcp40_124 = s->dpt[1][49]*ROcp40_122+s->dpt[2][49]*ROcp40_423+ROcp40_723*s->dpt[3][49];
RLcp40_224 = s->dpt[1][49]*ROcp40_222+s->dpt[2][49]*ROcp40_523+ROcp40_823*s->dpt[3][49];
RLcp40_324 = s->dpt[1][49]*ROcp40_322+s->dpt[2][49]*ROcp40_623+ROcp40_923*s->dpt[3][49];
OMcp40_124 = OMcp40_123+ROcp40_723*qd[24];
OMcp40_224 = OMcp40_223+ROcp40_823*qd[24];
OMcp40_324 = OMcp40_323+ROcp40_923*qd[24];
ORcp40_124 = OMcp40_223*RLcp40_324-OMcp40_323*RLcp40_224;
ORcp40_224 = -(OMcp40_123*RLcp40_324-OMcp40_323*RLcp40_124);
ORcp40_324 = OMcp40_123*RLcp40_224-OMcp40_223*RLcp40_124;
OPcp40_124 = OPcp40_123+ROcp40_723*qdd[24]+qd[24]*(OMcp40_223*ROcp40_923-OMcp40_323*ROcp40_823);
OPcp40_224 = OPcp40_223+ROcp40_823*qdd[24]-qd[24]*(OMcp40_123*ROcp40_923-OMcp40_323*ROcp40_723);
OPcp40_324 = OPcp40_323+ROcp40_923*qdd[24]+qd[24]*(OMcp40_123*ROcp40_823-OMcp40_223*ROcp40_723);
RLcp40_170 = s->dpt[1][51]*ROcp40_124+s->dpt[2][51]*ROcp40_424+ROcp40_723*s->dpt[3][51];
RLcp40_270 = s->dpt[1][51]*ROcp40_224+s->dpt[2][51]*ROcp40_524+ROcp40_823*s->dpt[3][51];
RLcp40_370 = s->dpt[1][51]*ROcp40_324+s->dpt[2][51]*ROcp40_624+ROcp40_923*s->dpt[3][51];
POcp40_170 = RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_170+q[1];
POcp40_270 = RLcp40_219+RLcp40_220+RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_270+q[2];
POcp40_370 = RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_370+q[3];
JTcp40_270_4 = -(RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_370);
JTcp40_370_4 = RLcp40_219+RLcp40_220+RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_270;
JTcp40_170_5 = C4*(RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)-S4*(RLcp40_219+RLcp40_220)-S4*(
 RLcp40_221+RLcp40_222)-S4*(RLcp40_223+RLcp40_224)-RLcp40_270*S4+RLcp40_370*C4;
JTcp40_270_5 = S4*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_170);
JTcp40_370_5 = -C4*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_170);
JTcp40_170_6 = ROcp40_85*(RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)-ROcp40_95*(RLcp40_219+
 RLcp40_220)-ROcp40_95*(RLcp40_221+RLcp40_222)-ROcp40_95*(RLcp40_223+RLcp40_224)-RLcp40_270*ROcp40_95+RLcp40_370*ROcp40_85;
JTcp40_270_6 = -(RLcp40_370*S5-ROcp40_95*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_170
 )+S5*(RLcp40_319+RLcp40_320)+S5*(RLcp40_321+RLcp40_322)+S5*(RLcp40_323+RLcp40_324));
JTcp40_370_6 = RLcp40_270*S5-ROcp40_85*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_170)+
 S5*(RLcp40_219+RLcp40_220)+S5*(RLcp40_221+RLcp40_222)+S5*(RLcp40_223+RLcp40_224);
JTcp40_170_7 = ROcp40_26*(RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_370)-ROcp40_36*(RLcp40_220+
 RLcp40_221)-ROcp40_36*(RLcp40_222+RLcp40_223)-ROcp40_36*(RLcp40_224+RLcp40_270);
JTcp40_270_7 = -(ROcp40_16*(RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_370)-ROcp40_36*(RLcp40_120+
 RLcp40_121)-ROcp40_36*(RLcp40_122+RLcp40_123)-ROcp40_36*(RLcp40_124+RLcp40_170));
JTcp40_370_7 = ROcp40_16*(RLcp40_220+RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_270)-ROcp40_26*(RLcp40_120+
 RLcp40_121)-ROcp40_26*(RLcp40_122+RLcp40_123)-ROcp40_26*(RLcp40_124+RLcp40_170);
JTcp40_170_8 = ROcp40_519*(RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)-ROcp40_619*(RLcp40_221+RLcp40_222)-ROcp40_619*
 (RLcp40_223+RLcp40_224)-RLcp40_270*ROcp40_619+RLcp40_370*ROcp40_519;
JTcp40_270_8 = RLcp40_170*ROcp40_619-RLcp40_370*ROcp40_419-ROcp40_419*(RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)+
 ROcp40_619*(RLcp40_121+RLcp40_122)+ROcp40_619*(RLcp40_123+RLcp40_124);
JTcp40_370_8 = ROcp40_419*(RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224)-ROcp40_519*(RLcp40_121+RLcp40_122)-ROcp40_519*
 (RLcp40_123+RLcp40_124)-RLcp40_170*ROcp40_519+RLcp40_270*ROcp40_419;
JTcp40_170_9 = ROcp40_820*(RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_370)-ROcp40_920*(RLcp40_222+RLcp40_223)-ROcp40_920*
 (RLcp40_224+RLcp40_270);
JTcp40_270_9 = -(ROcp40_720*(RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_370)-ROcp40_920*(RLcp40_122+RLcp40_123)-
 ROcp40_920*(RLcp40_124+RLcp40_170));
JTcp40_370_9 = ROcp40_720*(RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_270)-ROcp40_820*(RLcp40_122+RLcp40_123)-ROcp40_820*
 (RLcp40_124+RLcp40_170);
JTcp40_170_10 = ROcp40_521*(RLcp40_323+RLcp40_324)-ROcp40_621*(RLcp40_223+RLcp40_224)-RLcp40_270*ROcp40_621+RLcp40_370
 *ROcp40_521;
JTcp40_270_10 = RLcp40_170*ROcp40_621-RLcp40_370*ROcp40_421-ROcp40_421*(RLcp40_323+RLcp40_324)+ROcp40_621*(RLcp40_123+
 RLcp40_124);
JTcp40_370_10 = ROcp40_421*(RLcp40_223+RLcp40_224)-ROcp40_521*(RLcp40_123+RLcp40_124)-RLcp40_170*ROcp40_521+RLcp40_270
 *ROcp40_421;
JTcp40_170_11 = ROcp40_222*(RLcp40_324+RLcp40_370)-ROcp40_322*(RLcp40_224+RLcp40_270);
JTcp40_270_11 = -(ROcp40_122*(RLcp40_324+RLcp40_370)-ROcp40_322*(RLcp40_124+RLcp40_170));
JTcp40_370_11 = ROcp40_122*(RLcp40_224+RLcp40_270)-ROcp40_222*(RLcp40_124+RLcp40_170);
JTcp40_170_12 = -(RLcp40_270*ROcp40_923-RLcp40_370*ROcp40_823);
JTcp40_270_12 = RLcp40_170*ROcp40_923-RLcp40_370*ROcp40_723;
JTcp40_370_12 = -(RLcp40_170*ROcp40_823-RLcp40_270*ROcp40_723);
ORcp40_170 = OMcp40_224*RLcp40_370-OMcp40_324*RLcp40_270;
ORcp40_270 = -(OMcp40_124*RLcp40_370-OMcp40_324*RLcp40_170);
ORcp40_370 = OMcp40_124*RLcp40_270-OMcp40_224*RLcp40_170;
VIcp40_170 = ORcp40_119+ORcp40_120+ORcp40_121+ORcp40_122+ORcp40_123+ORcp40_124+ORcp40_170+qd[1];
VIcp40_270 = ORcp40_219+ORcp40_220+ORcp40_221+ORcp40_222+ORcp40_223+ORcp40_224+ORcp40_270+qd[2];
VIcp40_370 = ORcp40_319+ORcp40_320+ORcp40_321+ORcp40_322+ORcp40_323+ORcp40_324+ORcp40_370+qd[3];
ACcp40_170 = qdd[1]+OMcp40_219*ORcp40_320+OMcp40_220*ORcp40_321+OMcp40_221*ORcp40_322+OMcp40_222*ORcp40_323+OMcp40_223
 *ORcp40_324+OMcp40_224*ORcp40_370+OMcp40_26*ORcp40_319-OMcp40_319*ORcp40_220-OMcp40_320*ORcp40_221-OMcp40_321*ORcp40_222-
 OMcp40_322*ORcp40_223-OMcp40_323*ORcp40_224-OMcp40_324*ORcp40_270-OMcp40_36*ORcp40_219+OPcp40_219*RLcp40_320+OPcp40_220*
 RLcp40_321+OPcp40_221*RLcp40_322+OPcp40_222*RLcp40_323+OPcp40_223*RLcp40_324+OPcp40_224*RLcp40_370+OPcp40_26*RLcp40_319-
 OPcp40_319*RLcp40_220-OPcp40_320*RLcp40_221-OPcp40_321*RLcp40_222-OPcp40_322*RLcp40_223-OPcp40_323*RLcp40_224-OPcp40_324*
 RLcp40_270-OPcp40_36*RLcp40_219;
ACcp40_270 = qdd[2]-OMcp40_119*ORcp40_320-OMcp40_120*ORcp40_321-OMcp40_121*ORcp40_322-OMcp40_122*ORcp40_323-OMcp40_123
 *ORcp40_324-OMcp40_124*ORcp40_370-OMcp40_16*ORcp40_319+OMcp40_319*ORcp40_120+OMcp40_320*ORcp40_121+OMcp40_321*ORcp40_122+
 OMcp40_322*ORcp40_123+OMcp40_323*ORcp40_124+OMcp40_324*ORcp40_170+OMcp40_36*ORcp40_119-OPcp40_119*RLcp40_320-OPcp40_120*
 RLcp40_321-OPcp40_121*RLcp40_322-OPcp40_122*RLcp40_323-OPcp40_123*RLcp40_324-OPcp40_124*RLcp40_370-OPcp40_16*RLcp40_319+
 OPcp40_319*RLcp40_120+OPcp40_320*RLcp40_121+OPcp40_321*RLcp40_122+OPcp40_322*RLcp40_123+OPcp40_323*RLcp40_124+OPcp40_324*
 RLcp40_170+OPcp40_36*RLcp40_119;
ACcp40_370 = qdd[3]+OMcp40_119*ORcp40_220+OMcp40_120*ORcp40_221+OMcp40_121*ORcp40_222+OMcp40_122*ORcp40_223+OMcp40_123
 *ORcp40_224+OMcp40_124*ORcp40_270+OMcp40_16*ORcp40_219-OMcp40_219*ORcp40_120-OMcp40_220*ORcp40_121-OMcp40_221*ORcp40_122-
 OMcp40_222*ORcp40_123-OMcp40_223*ORcp40_124-OMcp40_224*ORcp40_170-OMcp40_26*ORcp40_119+OPcp40_119*RLcp40_220+OPcp40_120*
 RLcp40_221+OPcp40_121*RLcp40_222+OPcp40_122*RLcp40_223+OPcp40_123*RLcp40_224+OPcp40_124*RLcp40_270+OPcp40_16*RLcp40_219-
 OPcp40_219*RLcp40_120-OPcp40_220*RLcp40_121-OPcp40_221*RLcp40_122-OPcp40_222*RLcp40_123-OPcp40_223*RLcp40_124-OPcp40_224*
 RLcp40_170-OPcp40_26*RLcp40_119;

// = = Block_1_0_0_41_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp40_170;
sens->P[2] = POcp40_270;
sens->P[3] = POcp40_370;
sens->R[1][1] = ROcp40_124;
sens->R[1][2] = ROcp40_224;
sens->R[1][3] = ROcp40_324;
sens->R[2][1] = ROcp40_424;
sens->R[2][2] = ROcp40_524;
sens->R[2][3] = ROcp40_624;
sens->R[3][1] = ROcp40_723;
sens->R[3][2] = ROcp40_823;
sens->R[3][3] = ROcp40_923;
sens->V[1] = VIcp40_170;
sens->V[2] = VIcp40_270;
sens->V[3] = VIcp40_370;
sens->OM[1] = OMcp40_124;
sens->OM[2] = OMcp40_224;
sens->OM[3] = OMcp40_324;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp40_170_5;
sens->J[1][6] = JTcp40_170_6;
sens->J[1][19] = JTcp40_170_7;
sens->J[1][20] = JTcp40_170_8;
sens->J[1][21] = JTcp40_170_9;
sens->J[1][22] = JTcp40_170_10;
sens->J[1][23] = JTcp40_170_11;
sens->J[1][24] = JTcp40_170_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp40_270_4;
sens->J[2][5] = JTcp40_270_5;
sens->J[2][6] = JTcp40_270_6;
sens->J[2][19] = JTcp40_270_7;
sens->J[2][20] = JTcp40_270_8;
sens->J[2][21] = JTcp40_270_9;
sens->J[2][22] = JTcp40_270_10;
sens->J[2][23] = JTcp40_270_11;
sens->J[2][24] = JTcp40_270_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp40_370_4;
sens->J[3][5] = JTcp40_370_5;
sens->J[3][6] = JTcp40_370_6;
sens->J[3][19] = JTcp40_370_7;
sens->J[3][20] = JTcp40_370_8;
sens->J[3][21] = JTcp40_370_9;
sens->J[3][22] = JTcp40_370_10;
sens->J[3][23] = JTcp40_370_11;
sens->J[3][24] = JTcp40_370_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp40_16;
sens->J[4][20] = ROcp40_419;
sens->J[4][21] = ROcp40_720;
sens->J[4][22] = ROcp40_421;
sens->J[4][23] = ROcp40_122;
sens->J[4][24] = ROcp40_723;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp40_85;
sens->J[5][19] = ROcp40_26;
sens->J[5][20] = ROcp40_519;
sens->J[5][21] = ROcp40_820;
sens->J[5][22] = ROcp40_521;
sens->J[5][23] = ROcp40_222;
sens->J[5][24] = ROcp40_823;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp40_95;
sens->J[6][19] = ROcp40_36;
sens->J[6][20] = ROcp40_619;
sens->J[6][21] = ROcp40_920;
sens->J[6][22] = ROcp40_621;
sens->J[6][23] = ROcp40_322;
sens->J[6][24] = ROcp40_923;
sens->A[1] = ACcp40_170;
sens->A[2] = ACcp40_270;
sens->A[3] = ACcp40_370;
sens->OMP[1] = OPcp40_124;
sens->OMP[2] = OPcp40_224;
sens->OMP[3] = OPcp40_324;
 
// 
break;
case 42:
 


// = = Block_1_0_0_42_0_1 = = 
 
// Sensor Kinematics 


ROcp41_25 = S4*S5;
ROcp41_35 = -C4*S5;
ROcp41_85 = -S4*C5;
ROcp41_95 = C4*C5;
ROcp41_16 = C5*C6;
ROcp41_26 = ROcp41_25*C6+C4*S6;
ROcp41_36 = ROcp41_35*C6+S4*S6;
ROcp41_46 = -C5*S6;
ROcp41_56 = -(ROcp41_25*S6-C4*C6);
ROcp41_66 = -(ROcp41_35*S6-S4*C6);
OMcp41_25 = qd[5]*C4;
OMcp41_35 = qd[5]*S4;
OMcp41_16 = qd[4]+qd[6]*S5;
OMcp41_26 = OMcp41_25+ROcp41_85*qd[6];
OMcp41_36 = OMcp41_35+ROcp41_95*qd[6];
OPcp41_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp41_26 = ROcp41_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp41_35*S5-ROcp41_95*qd[4]);
OPcp41_36 = ROcp41_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp41_25*S5-ROcp41_85*qd[4]);

// = = Block_1_0_0_42_0_4 = = 
 
// Sensor Kinematics 


ROcp41_419 = ROcp41_46*C19+S19*S5;
ROcp41_519 = ROcp41_56*C19+ROcp41_85*S19;
ROcp41_619 = ROcp41_66*C19+ROcp41_95*S19;
ROcp41_719 = -(ROcp41_46*S19-C19*S5);
ROcp41_819 = -(ROcp41_56*S19-ROcp41_85*C19);
ROcp41_919 = -(ROcp41_66*S19-ROcp41_95*C19);
ROcp41_120 = ROcp41_16*C20-ROcp41_719*S20;
ROcp41_220 = ROcp41_26*C20-ROcp41_819*S20;
ROcp41_320 = ROcp41_36*C20-ROcp41_919*S20;
ROcp41_720 = ROcp41_16*S20+ROcp41_719*C20;
ROcp41_820 = ROcp41_26*S20+ROcp41_819*C20;
ROcp41_920 = ROcp41_36*S20+ROcp41_919*C20;
ROcp41_121 = ROcp41_120*C21+ROcp41_419*S21;
ROcp41_221 = ROcp41_220*C21+ROcp41_519*S21;
ROcp41_321 = ROcp41_320*C21+ROcp41_619*S21;
ROcp41_421 = -(ROcp41_120*S21-ROcp41_419*C21);
ROcp41_521 = -(ROcp41_220*S21-ROcp41_519*C21);
ROcp41_621 = -(ROcp41_320*S21-ROcp41_619*C21);
RLcp41_119 = s->dpt[2][3]*ROcp41_46+ROcp41_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp41_219 = s->dpt[2][3]*ROcp41_56+ROcp41_26*s->dpt[1][3]+ROcp41_85*s->dpt[3][3];
RLcp41_319 = s->dpt[2][3]*ROcp41_66+ROcp41_36*s->dpt[1][3]+ROcp41_95*s->dpt[3][3];
OMcp41_119 = OMcp41_16+ROcp41_16*qd[19];
OMcp41_219 = OMcp41_26+ROcp41_26*qd[19];
OMcp41_319 = OMcp41_36+ROcp41_36*qd[19];
ORcp41_119 = OMcp41_26*RLcp41_319-OMcp41_36*RLcp41_219;
ORcp41_219 = -(OMcp41_16*RLcp41_319-OMcp41_36*RLcp41_119);
ORcp41_319 = OMcp41_16*RLcp41_219-OMcp41_26*RLcp41_119;
OPcp41_119 = OPcp41_16+ROcp41_16*qdd[19]+qd[19]*(OMcp41_26*ROcp41_36-OMcp41_36*ROcp41_26);
OPcp41_219 = OPcp41_26+ROcp41_26*qdd[19]-qd[19]*(OMcp41_16*ROcp41_36-OMcp41_36*ROcp41_16);
OPcp41_319 = OPcp41_36+ROcp41_36*qdd[19]+qd[19]*(OMcp41_16*ROcp41_26-OMcp41_26*ROcp41_16);
RLcp41_120 = s->dpt[1][38]*ROcp41_16+s->dpt[2][38]*ROcp41_419+s->dpt[3][38]*ROcp41_719;
RLcp41_220 = s->dpt[1][38]*ROcp41_26+s->dpt[2][38]*ROcp41_519+s->dpt[3][38]*ROcp41_819;
RLcp41_320 = s->dpt[1][38]*ROcp41_36+s->dpt[2][38]*ROcp41_619+s->dpt[3][38]*ROcp41_919;
OMcp41_120 = OMcp41_119+ROcp41_419*qd[20];
OMcp41_220 = OMcp41_219+ROcp41_519*qd[20];
OMcp41_320 = OMcp41_319+ROcp41_619*qd[20];
ORcp41_120 = OMcp41_219*RLcp41_320-OMcp41_319*RLcp41_220;
ORcp41_220 = -(OMcp41_119*RLcp41_320-OMcp41_319*RLcp41_120);
ORcp41_320 = OMcp41_119*RLcp41_220-OMcp41_219*RLcp41_120;
OPcp41_120 = OPcp41_119+ROcp41_419*qdd[20]+qd[20]*(OMcp41_219*ROcp41_619-OMcp41_319*ROcp41_519);
OPcp41_220 = OPcp41_219+ROcp41_519*qdd[20]-qd[20]*(OMcp41_119*ROcp41_619-OMcp41_319*ROcp41_419);
OPcp41_320 = OPcp41_319+ROcp41_619*qdd[20]+qd[20]*(OMcp41_119*ROcp41_519-OMcp41_219*ROcp41_419);
RLcp41_121 = s->dpt[1][40]*ROcp41_120+s->dpt[2][40]*ROcp41_419+ROcp41_720*s->dpt[3][40];
RLcp41_221 = s->dpt[1][40]*ROcp41_220+s->dpt[2][40]*ROcp41_519+ROcp41_820*s->dpt[3][40];
RLcp41_321 = s->dpt[1][40]*ROcp41_320+s->dpt[2][40]*ROcp41_619+ROcp41_920*s->dpt[3][40];
OMcp41_121 = OMcp41_120+ROcp41_720*qd[21];
OMcp41_221 = OMcp41_220+ROcp41_820*qd[21];
OMcp41_321 = OMcp41_320+ROcp41_920*qd[21];
ORcp41_121 = OMcp41_220*RLcp41_321-OMcp41_320*RLcp41_221;
ORcp41_221 = -(OMcp41_120*RLcp41_321-OMcp41_320*RLcp41_121);
ORcp41_321 = OMcp41_120*RLcp41_221-OMcp41_220*RLcp41_121;
OPcp41_121 = OPcp41_120+ROcp41_720*qdd[21]+qd[21]*(OMcp41_220*ROcp41_920-OMcp41_320*ROcp41_820);
OPcp41_221 = OPcp41_220+ROcp41_820*qdd[21]-qd[21]*(OMcp41_120*ROcp41_920-OMcp41_320*ROcp41_720);
OPcp41_321 = OPcp41_320+ROcp41_920*qdd[21]+qd[21]*(OMcp41_120*ROcp41_820-OMcp41_220*ROcp41_720);

// = = Block_1_0_0_42_0_5 = = 
 
// Sensor Kinematics 


ROcp41_122 = ROcp41_121*C22-ROcp41_720*S22;
ROcp41_222 = ROcp41_221*C22-ROcp41_820*S22;
ROcp41_322 = ROcp41_321*C22-ROcp41_920*S22;
ROcp41_722 = ROcp41_121*S22+ROcp41_720*C22;
ROcp41_822 = ROcp41_221*S22+ROcp41_820*C22;
ROcp41_922 = ROcp41_321*S22+ROcp41_920*C22;
ROcp41_423 = ROcp41_421*C23+ROcp41_722*S23;
ROcp41_523 = ROcp41_521*C23+ROcp41_822*S23;
ROcp41_623 = ROcp41_621*C23+ROcp41_922*S23;
ROcp41_723 = -(ROcp41_421*S23-ROcp41_722*C23);
ROcp41_823 = -(ROcp41_521*S23-ROcp41_822*C23);
ROcp41_923 = -(ROcp41_621*S23-ROcp41_922*C23);
ROcp41_124 = ROcp41_122*C24+ROcp41_423*S24;
ROcp41_224 = ROcp41_222*C24+ROcp41_523*S24;
ROcp41_324 = ROcp41_322*C24+ROcp41_623*S24;
ROcp41_424 = -(ROcp41_122*S24-ROcp41_423*C24);
ROcp41_524 = -(ROcp41_222*S24-ROcp41_523*C24);
ROcp41_624 = -(ROcp41_322*S24-ROcp41_623*C24);
ROcp41_125 = ROcp41_124*C25-ROcp41_723*S25;
ROcp41_225 = ROcp41_224*C25-ROcp41_823*S25;
ROcp41_325 = ROcp41_324*C25-ROcp41_923*S25;
ROcp41_725 = ROcp41_124*S25+ROcp41_723*C25;
ROcp41_825 = ROcp41_224*S25+ROcp41_823*C25;
ROcp41_925 = ROcp41_324*S25+ROcp41_923*C25;
RLcp41_122 = ROcp41_121*s->dpt[1][44]+ROcp41_421*s->dpt[2][44]+ROcp41_720*s->dpt[3][44];
RLcp41_222 = ROcp41_221*s->dpt[1][44]+ROcp41_521*s->dpt[2][44]+ROcp41_820*s->dpt[3][44];
RLcp41_322 = ROcp41_321*s->dpt[1][44]+ROcp41_621*s->dpt[2][44]+ROcp41_920*s->dpt[3][44];
OMcp41_122 = OMcp41_121+ROcp41_421*qd[22];
OMcp41_222 = OMcp41_221+ROcp41_521*qd[22];
OMcp41_322 = OMcp41_321+ROcp41_621*qd[22];
ORcp41_122 = OMcp41_221*RLcp41_322-OMcp41_321*RLcp41_222;
ORcp41_222 = -(OMcp41_121*RLcp41_322-OMcp41_321*RLcp41_122);
ORcp41_322 = OMcp41_121*RLcp41_222-OMcp41_221*RLcp41_122;
OPcp41_122 = OPcp41_121+ROcp41_421*qdd[22]+qd[22]*(OMcp41_221*ROcp41_621-OMcp41_321*ROcp41_521);
OPcp41_222 = OPcp41_221+ROcp41_521*qdd[22]-qd[22]*(OMcp41_121*ROcp41_621-OMcp41_321*ROcp41_421);
OPcp41_322 = OPcp41_321+ROcp41_621*qdd[22]+qd[22]*(OMcp41_121*ROcp41_521-OMcp41_221*ROcp41_421);
RLcp41_123 = s->dpt[1][47]*ROcp41_122+s->dpt[3][47]*ROcp41_722+ROcp41_421*s->dpt[2][47];
RLcp41_223 = s->dpt[1][47]*ROcp41_222+s->dpt[3][47]*ROcp41_822+ROcp41_521*s->dpt[2][47];
RLcp41_323 = s->dpt[1][47]*ROcp41_322+s->dpt[3][47]*ROcp41_922+ROcp41_621*s->dpt[2][47];
OMcp41_123 = OMcp41_122+ROcp41_122*qd[23];
OMcp41_223 = OMcp41_222+ROcp41_222*qd[23];
OMcp41_323 = OMcp41_322+ROcp41_322*qd[23];
ORcp41_123 = OMcp41_222*RLcp41_323-OMcp41_322*RLcp41_223;
ORcp41_223 = -(OMcp41_122*RLcp41_323-OMcp41_322*RLcp41_123);
ORcp41_323 = OMcp41_122*RLcp41_223-OMcp41_222*RLcp41_123;
OPcp41_123 = OPcp41_122+ROcp41_122*qdd[23]+qd[23]*(OMcp41_222*ROcp41_322-OMcp41_322*ROcp41_222);
OPcp41_223 = OPcp41_222+ROcp41_222*qdd[23]-qd[23]*(OMcp41_122*ROcp41_322-OMcp41_322*ROcp41_122);
OPcp41_323 = OPcp41_322+ROcp41_322*qdd[23]+qd[23]*(OMcp41_122*ROcp41_222-OMcp41_222*ROcp41_122);
RLcp41_124 = s->dpt[1][49]*ROcp41_122+s->dpt[2][49]*ROcp41_423+ROcp41_723*s->dpt[3][49];
RLcp41_224 = s->dpt[1][49]*ROcp41_222+s->dpt[2][49]*ROcp41_523+ROcp41_823*s->dpt[3][49];
RLcp41_324 = s->dpt[1][49]*ROcp41_322+s->dpt[2][49]*ROcp41_623+ROcp41_923*s->dpt[3][49];
OMcp41_124 = OMcp41_123+ROcp41_723*qd[24];
OMcp41_224 = OMcp41_223+ROcp41_823*qd[24];
OMcp41_324 = OMcp41_323+ROcp41_923*qd[24];
ORcp41_124 = OMcp41_223*RLcp41_324-OMcp41_323*RLcp41_224;
ORcp41_224 = -(OMcp41_123*RLcp41_324-OMcp41_323*RLcp41_124);
ORcp41_324 = OMcp41_123*RLcp41_224-OMcp41_223*RLcp41_124;
OPcp41_124 = OPcp41_123+ROcp41_723*qdd[24]+qd[24]*(OMcp41_223*ROcp41_923-OMcp41_323*ROcp41_823);
OPcp41_224 = OPcp41_223+ROcp41_823*qdd[24]-qd[24]*(OMcp41_123*ROcp41_923-OMcp41_323*ROcp41_723);
OPcp41_324 = OPcp41_323+ROcp41_923*qdd[24]+qd[24]*(OMcp41_123*ROcp41_823-OMcp41_223*ROcp41_723);
RLcp41_125 = s->dpt[1][51]*ROcp41_124+s->dpt[2][51]*ROcp41_424+ROcp41_723*s->dpt[3][51];
RLcp41_225 = s->dpt[1][51]*ROcp41_224+s->dpt[2][51]*ROcp41_524+ROcp41_823*s->dpt[3][51];
RLcp41_325 = s->dpt[1][51]*ROcp41_324+s->dpt[2][51]*ROcp41_624+ROcp41_923*s->dpt[3][51];
OMcp41_125 = OMcp41_124+ROcp41_424*qd[25];
OMcp41_225 = OMcp41_224+ROcp41_524*qd[25];
OMcp41_325 = OMcp41_324+ROcp41_624*qd[25];
ORcp41_125 = OMcp41_224*RLcp41_325-OMcp41_324*RLcp41_225;
ORcp41_225 = -(OMcp41_124*RLcp41_325-OMcp41_324*RLcp41_125);
ORcp41_325 = OMcp41_124*RLcp41_225-OMcp41_224*RLcp41_125;
OPcp41_125 = OPcp41_124+ROcp41_424*qdd[25]+qd[25]*(OMcp41_224*ROcp41_624-OMcp41_324*ROcp41_524);
OPcp41_225 = OPcp41_224+ROcp41_524*qdd[25]-qd[25]*(OMcp41_124*ROcp41_624-OMcp41_324*ROcp41_424);
OPcp41_325 = OPcp41_324+ROcp41_624*qdd[25]+qd[25]*(OMcp41_124*ROcp41_524-OMcp41_224*ROcp41_424);
RLcp41_171 = ROcp41_125*s->dpt[1][52]+ROcp41_424*s->dpt[2][52]+ROcp41_725*s->dpt[3][52];
RLcp41_271 = ROcp41_225*s->dpt[1][52]+ROcp41_524*s->dpt[2][52]+ROcp41_825*s->dpt[3][52];
RLcp41_371 = ROcp41_325*s->dpt[1][52]+ROcp41_624*s->dpt[2][52]+ROcp41_925*s->dpt[3][52];
POcp41_171 = RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_171+q[1];
POcp41_271 = RLcp41_219+RLcp41_220+RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_271+q[2];
POcp41_371 = RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371+q[3];
JTcp41_271_4 = -(RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371);
JTcp41_371_4 = RLcp41_219+RLcp41_220+RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_271;
JTcp41_171_5 = C4*(RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371)-S4*(
 RLcp41_219+RLcp41_220)-S4*(RLcp41_221+RLcp41_222)-S4*(RLcp41_223+RLcp41_224)-S4*(RLcp41_225+RLcp41_271);
JTcp41_271_5 = S4*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_171);
JTcp41_371_5 = -C4*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_171);
JTcp41_171_6 = ROcp41_85*(RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371)-
 ROcp41_95*(RLcp41_219+RLcp41_220)-ROcp41_95*(RLcp41_221+RLcp41_222)-ROcp41_95*(RLcp41_223+RLcp41_224)-ROcp41_95*(RLcp41_225+
 RLcp41_271);
JTcp41_271_6 = RLcp41_171*ROcp41_95-RLcp41_325*S5-RLcp41_371*S5+ROcp41_95*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122
 +RLcp41_123+RLcp41_124+RLcp41_125)-S5*(RLcp41_319+RLcp41_320)-S5*(RLcp41_321+RLcp41_322)-S5*(RLcp41_323+RLcp41_324);
JTcp41_371_6 = RLcp41_225*S5-ROcp41_85*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125)+
 S5*(RLcp41_219+RLcp41_220)+S5*(RLcp41_221+RLcp41_222)+S5*(RLcp41_223+RLcp41_224)-RLcp41_171*ROcp41_85+RLcp41_271*S5;
JTcp41_171_7 = ROcp41_26*(RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325)-ROcp41_36*(RLcp41_220+
 RLcp41_221)-ROcp41_36*(RLcp41_222+RLcp41_223)-ROcp41_36*(RLcp41_224+RLcp41_225)-RLcp41_271*ROcp41_36+RLcp41_371*ROcp41_26;
JTcp41_271_7 = RLcp41_171*ROcp41_36-RLcp41_371*ROcp41_16-ROcp41_16*(RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+
 RLcp41_324+RLcp41_325)+ROcp41_36*(RLcp41_120+RLcp41_121)+ROcp41_36*(RLcp41_122+RLcp41_123)+ROcp41_36*(RLcp41_124+RLcp41_125);
JTcp41_371_7 = ROcp41_16*(RLcp41_220+RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225)-ROcp41_26*(RLcp41_120+
 RLcp41_121)-ROcp41_26*(RLcp41_122+RLcp41_123)-ROcp41_26*(RLcp41_124+RLcp41_125)-RLcp41_171*ROcp41_26+RLcp41_271*ROcp41_16;
JTcp41_171_8 = ROcp41_519*(RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371)-ROcp41_619*(RLcp41_221+
 RLcp41_222)-ROcp41_619*(RLcp41_223+RLcp41_224)-ROcp41_619*(RLcp41_225+RLcp41_271);
JTcp41_271_8 = -(ROcp41_419*(RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371)-ROcp41_619*(RLcp41_121
 +RLcp41_122)-ROcp41_619*(RLcp41_123+RLcp41_124)-ROcp41_619*(RLcp41_125+RLcp41_171));
JTcp41_371_8 = ROcp41_419*(RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_271)-ROcp41_519*(RLcp41_121+
 RLcp41_122)-ROcp41_519*(RLcp41_123+RLcp41_124)-ROcp41_519*(RLcp41_125+RLcp41_171);
JTcp41_171_9 = ROcp41_820*(RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325)-ROcp41_920*(RLcp41_222+RLcp41_223)-ROcp41_920*
 (RLcp41_224+RLcp41_225)-RLcp41_271*ROcp41_920+RLcp41_371*ROcp41_820;
JTcp41_271_9 = RLcp41_171*ROcp41_920-RLcp41_371*ROcp41_720-ROcp41_720*(RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325)+
 ROcp41_920*(RLcp41_122+RLcp41_123)+ROcp41_920*(RLcp41_124+RLcp41_125);
JTcp41_371_9 = ROcp41_720*(RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225)-ROcp41_820*(RLcp41_122+RLcp41_123)-ROcp41_820*
 (RLcp41_124+RLcp41_125)-RLcp41_171*ROcp41_820+RLcp41_271*ROcp41_720;
JTcp41_171_10 = ROcp41_521*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371)-ROcp41_621*(RLcp41_223+RLcp41_224)-ROcp41_621
 *(RLcp41_225+RLcp41_271);
JTcp41_271_10 = -(ROcp41_421*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_371)-ROcp41_621*(RLcp41_123+RLcp41_124)-
 ROcp41_621*(RLcp41_125+RLcp41_171));
JTcp41_371_10 = ROcp41_421*(RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_271)-ROcp41_521*(RLcp41_123+RLcp41_124)-ROcp41_521
 *(RLcp41_125+RLcp41_171);
JTcp41_171_11 = ROcp41_222*(RLcp41_324+RLcp41_325)-ROcp41_322*(RLcp41_224+RLcp41_225)-RLcp41_271*ROcp41_322+RLcp41_371
 *ROcp41_222;
JTcp41_271_11 = RLcp41_171*ROcp41_322-RLcp41_371*ROcp41_122-ROcp41_122*(RLcp41_324+RLcp41_325)+ROcp41_322*(RLcp41_124+
 RLcp41_125);
JTcp41_371_11 = ROcp41_122*(RLcp41_224+RLcp41_225)-ROcp41_222*(RLcp41_124+RLcp41_125)-RLcp41_171*ROcp41_222+RLcp41_271
 *ROcp41_122;
JTcp41_171_12 = ROcp41_823*(RLcp41_325+RLcp41_371)-ROcp41_923*(RLcp41_225+RLcp41_271);
JTcp41_271_12 = -(ROcp41_723*(RLcp41_325+RLcp41_371)-ROcp41_923*(RLcp41_125+RLcp41_171));
JTcp41_371_12 = ROcp41_723*(RLcp41_225+RLcp41_271)-ROcp41_823*(RLcp41_125+RLcp41_171);
JTcp41_171_13 = -(RLcp41_271*ROcp41_624-RLcp41_371*ROcp41_524);
JTcp41_271_13 = RLcp41_171*ROcp41_624-RLcp41_371*ROcp41_424;
JTcp41_371_13 = -(RLcp41_171*ROcp41_524-RLcp41_271*ROcp41_424);
ORcp41_171 = OMcp41_225*RLcp41_371-OMcp41_325*RLcp41_271;
ORcp41_271 = -(OMcp41_125*RLcp41_371-OMcp41_325*RLcp41_171);
ORcp41_371 = OMcp41_125*RLcp41_271-OMcp41_225*RLcp41_171;
VIcp41_171 = ORcp41_119+ORcp41_120+ORcp41_121+ORcp41_122+ORcp41_123+ORcp41_124+ORcp41_125+ORcp41_171+qd[1];
VIcp41_271 = ORcp41_219+ORcp41_220+ORcp41_221+ORcp41_222+ORcp41_223+ORcp41_224+ORcp41_225+ORcp41_271+qd[2];
VIcp41_371 = ORcp41_319+ORcp41_320+ORcp41_321+ORcp41_322+ORcp41_323+ORcp41_324+ORcp41_325+ORcp41_371+qd[3];
ACcp41_171 = qdd[1]+OMcp41_219*ORcp41_320+OMcp41_220*ORcp41_321+OMcp41_221*ORcp41_322+OMcp41_222*ORcp41_323+OMcp41_223
 *ORcp41_324+OMcp41_224*ORcp41_325+OMcp41_225*ORcp41_371+OMcp41_26*ORcp41_319-OMcp41_319*ORcp41_220-OMcp41_320*ORcp41_221-
 OMcp41_321*ORcp41_222-OMcp41_322*ORcp41_223-OMcp41_323*ORcp41_224-OMcp41_324*ORcp41_225-OMcp41_325*ORcp41_271-OMcp41_36*
 ORcp41_219+OPcp41_219*RLcp41_320+OPcp41_220*RLcp41_321+OPcp41_221*RLcp41_322+OPcp41_222*RLcp41_323+OPcp41_223*RLcp41_324+
 OPcp41_224*RLcp41_325+OPcp41_225*RLcp41_371+OPcp41_26*RLcp41_319-OPcp41_319*RLcp41_220-OPcp41_320*RLcp41_221-OPcp41_321*
 RLcp41_222-OPcp41_322*RLcp41_223-OPcp41_323*RLcp41_224-OPcp41_324*RLcp41_225-OPcp41_325*RLcp41_271-OPcp41_36*RLcp41_219;
ACcp41_271 = qdd[2]-OMcp41_119*ORcp41_320-OMcp41_120*ORcp41_321-OMcp41_121*ORcp41_322-OMcp41_122*ORcp41_323-OMcp41_123
 *ORcp41_324-OMcp41_124*ORcp41_325-OMcp41_125*ORcp41_371-OMcp41_16*ORcp41_319+OMcp41_319*ORcp41_120+OMcp41_320*ORcp41_121+
 OMcp41_321*ORcp41_122+OMcp41_322*ORcp41_123+OMcp41_323*ORcp41_124+OMcp41_324*ORcp41_125+OMcp41_325*ORcp41_171+OMcp41_36*
 ORcp41_119-OPcp41_119*RLcp41_320-OPcp41_120*RLcp41_321-OPcp41_121*RLcp41_322-OPcp41_122*RLcp41_323-OPcp41_123*RLcp41_324-
 OPcp41_124*RLcp41_325-OPcp41_125*RLcp41_371-OPcp41_16*RLcp41_319+OPcp41_319*RLcp41_120+OPcp41_320*RLcp41_121+OPcp41_321*
 RLcp41_122+OPcp41_322*RLcp41_123+OPcp41_323*RLcp41_124+OPcp41_324*RLcp41_125+OPcp41_325*RLcp41_171+OPcp41_36*RLcp41_119;
ACcp41_371 = qdd[3]+OMcp41_119*ORcp41_220+OMcp41_120*ORcp41_221+OMcp41_121*ORcp41_222+OMcp41_122*ORcp41_223+OMcp41_123
 *ORcp41_224+OMcp41_124*ORcp41_225+OMcp41_125*ORcp41_271+OMcp41_16*ORcp41_219-OMcp41_219*ORcp41_120-OMcp41_220*ORcp41_121-
 OMcp41_221*ORcp41_122-OMcp41_222*ORcp41_123-OMcp41_223*ORcp41_124-OMcp41_224*ORcp41_125-OMcp41_225*ORcp41_171-OMcp41_26*
 ORcp41_119+OPcp41_119*RLcp41_220+OPcp41_120*RLcp41_221+OPcp41_121*RLcp41_222+OPcp41_122*RLcp41_223+OPcp41_123*RLcp41_224+
 OPcp41_124*RLcp41_225+OPcp41_125*RLcp41_271+OPcp41_16*RLcp41_219-OPcp41_219*RLcp41_120-OPcp41_220*RLcp41_121-OPcp41_221*
 RLcp41_122-OPcp41_222*RLcp41_123-OPcp41_223*RLcp41_124-OPcp41_224*RLcp41_125-OPcp41_225*RLcp41_171-OPcp41_26*RLcp41_119;

// = = Block_1_0_0_42_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp41_171;
sens->P[2] = POcp41_271;
sens->P[3] = POcp41_371;
sens->R[1][1] = ROcp41_125;
sens->R[1][2] = ROcp41_225;
sens->R[1][3] = ROcp41_325;
sens->R[2][1] = ROcp41_424;
sens->R[2][2] = ROcp41_524;
sens->R[2][3] = ROcp41_624;
sens->R[3][1] = ROcp41_725;
sens->R[3][2] = ROcp41_825;
sens->R[3][3] = ROcp41_925;
sens->V[1] = VIcp41_171;
sens->V[2] = VIcp41_271;
sens->V[3] = VIcp41_371;
sens->OM[1] = OMcp41_125;
sens->OM[2] = OMcp41_225;
sens->OM[3] = OMcp41_325;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp41_171_5;
sens->J[1][6] = JTcp41_171_6;
sens->J[1][19] = JTcp41_171_7;
sens->J[1][20] = JTcp41_171_8;
sens->J[1][21] = JTcp41_171_9;
sens->J[1][22] = JTcp41_171_10;
sens->J[1][23] = JTcp41_171_11;
sens->J[1][24] = JTcp41_171_12;
sens->J[1][25] = JTcp41_171_13;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp41_271_4;
sens->J[2][5] = JTcp41_271_5;
sens->J[2][6] = JTcp41_271_6;
sens->J[2][19] = JTcp41_271_7;
sens->J[2][20] = JTcp41_271_8;
sens->J[2][21] = JTcp41_271_9;
sens->J[2][22] = JTcp41_271_10;
sens->J[2][23] = JTcp41_271_11;
sens->J[2][24] = JTcp41_271_12;
sens->J[2][25] = JTcp41_271_13;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp41_371_4;
sens->J[3][5] = JTcp41_371_5;
sens->J[3][6] = JTcp41_371_6;
sens->J[3][19] = JTcp41_371_7;
sens->J[3][20] = JTcp41_371_8;
sens->J[3][21] = JTcp41_371_9;
sens->J[3][22] = JTcp41_371_10;
sens->J[3][23] = JTcp41_371_11;
sens->J[3][24] = JTcp41_371_12;
sens->J[3][25] = JTcp41_371_13;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp41_16;
sens->J[4][20] = ROcp41_419;
sens->J[4][21] = ROcp41_720;
sens->J[4][22] = ROcp41_421;
sens->J[4][23] = ROcp41_122;
sens->J[4][24] = ROcp41_723;
sens->J[4][25] = ROcp41_424;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp41_85;
sens->J[5][19] = ROcp41_26;
sens->J[5][20] = ROcp41_519;
sens->J[5][21] = ROcp41_820;
sens->J[5][22] = ROcp41_521;
sens->J[5][23] = ROcp41_222;
sens->J[5][24] = ROcp41_823;
sens->J[5][25] = ROcp41_524;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp41_95;
sens->J[6][19] = ROcp41_36;
sens->J[6][20] = ROcp41_619;
sens->J[6][21] = ROcp41_920;
sens->J[6][22] = ROcp41_621;
sens->J[6][23] = ROcp41_322;
sens->J[6][24] = ROcp41_923;
sens->J[6][25] = ROcp41_624;
sens->A[1] = ACcp41_171;
sens->A[2] = ACcp41_271;
sens->A[3] = ACcp41_371;
sens->OMP[1] = OPcp41_125;
sens->OMP[2] = OPcp41_225;
sens->OMP[3] = OPcp41_325;
 
// 
break;
case 43:
 


// = = Block_1_0_0_43_0_1 = = 
 
// Sensor Kinematics 


ROcp42_25 = S4*S5;
ROcp42_35 = -C4*S5;
ROcp42_85 = -S4*C5;
ROcp42_95 = C4*C5;
ROcp42_16 = C5*C6;
ROcp42_26 = ROcp42_25*C6+C4*S6;
ROcp42_36 = ROcp42_35*C6+S4*S6;
ROcp42_46 = -C5*S6;
ROcp42_56 = -(ROcp42_25*S6-C4*C6);
ROcp42_66 = -(ROcp42_35*S6-S4*C6);
OMcp42_25 = qd[5]*C4;
OMcp42_35 = qd[5]*S4;
OMcp42_16 = qd[4]+qd[6]*S5;
OMcp42_26 = OMcp42_25+ROcp42_85*qd[6];
OMcp42_36 = OMcp42_35+ROcp42_95*qd[6];
OPcp42_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp42_26 = ROcp42_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp42_35*S5-ROcp42_95*qd[4]);
OPcp42_36 = ROcp42_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp42_25*S5-ROcp42_85*qd[4]);

// = = Block_1_0_0_43_0_4 = = 
 
// Sensor Kinematics 


ROcp42_419 = ROcp42_46*C19+S19*S5;
ROcp42_519 = ROcp42_56*C19+ROcp42_85*S19;
ROcp42_619 = ROcp42_66*C19+ROcp42_95*S19;
ROcp42_719 = -(ROcp42_46*S19-C19*S5);
ROcp42_819 = -(ROcp42_56*S19-ROcp42_85*C19);
ROcp42_919 = -(ROcp42_66*S19-ROcp42_95*C19);
ROcp42_120 = ROcp42_16*C20-ROcp42_719*S20;
ROcp42_220 = ROcp42_26*C20-ROcp42_819*S20;
ROcp42_320 = ROcp42_36*C20-ROcp42_919*S20;
ROcp42_720 = ROcp42_16*S20+ROcp42_719*C20;
ROcp42_820 = ROcp42_26*S20+ROcp42_819*C20;
ROcp42_920 = ROcp42_36*S20+ROcp42_919*C20;
ROcp42_121 = ROcp42_120*C21+ROcp42_419*S21;
ROcp42_221 = ROcp42_220*C21+ROcp42_519*S21;
ROcp42_321 = ROcp42_320*C21+ROcp42_619*S21;
ROcp42_421 = -(ROcp42_120*S21-ROcp42_419*C21);
ROcp42_521 = -(ROcp42_220*S21-ROcp42_519*C21);
ROcp42_621 = -(ROcp42_320*S21-ROcp42_619*C21);
RLcp42_119 = s->dpt[2][3]*ROcp42_46+ROcp42_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp42_219 = s->dpt[2][3]*ROcp42_56+ROcp42_26*s->dpt[1][3]+ROcp42_85*s->dpt[3][3];
RLcp42_319 = s->dpt[2][3]*ROcp42_66+ROcp42_36*s->dpt[1][3]+ROcp42_95*s->dpt[3][3];
OMcp42_119 = OMcp42_16+ROcp42_16*qd[19];
OMcp42_219 = OMcp42_26+ROcp42_26*qd[19];
OMcp42_319 = OMcp42_36+ROcp42_36*qd[19];
ORcp42_119 = OMcp42_26*RLcp42_319-OMcp42_36*RLcp42_219;
ORcp42_219 = -(OMcp42_16*RLcp42_319-OMcp42_36*RLcp42_119);
ORcp42_319 = OMcp42_16*RLcp42_219-OMcp42_26*RLcp42_119;
OPcp42_119 = OPcp42_16+ROcp42_16*qdd[19]+qd[19]*(OMcp42_26*ROcp42_36-OMcp42_36*ROcp42_26);
OPcp42_219 = OPcp42_26+ROcp42_26*qdd[19]-qd[19]*(OMcp42_16*ROcp42_36-OMcp42_36*ROcp42_16);
OPcp42_319 = OPcp42_36+ROcp42_36*qdd[19]+qd[19]*(OMcp42_16*ROcp42_26-OMcp42_26*ROcp42_16);
RLcp42_120 = s->dpt[1][38]*ROcp42_16+s->dpt[2][38]*ROcp42_419+s->dpt[3][38]*ROcp42_719;
RLcp42_220 = s->dpt[1][38]*ROcp42_26+s->dpt[2][38]*ROcp42_519+s->dpt[3][38]*ROcp42_819;
RLcp42_320 = s->dpt[1][38]*ROcp42_36+s->dpt[2][38]*ROcp42_619+s->dpt[3][38]*ROcp42_919;
OMcp42_120 = OMcp42_119+ROcp42_419*qd[20];
OMcp42_220 = OMcp42_219+ROcp42_519*qd[20];
OMcp42_320 = OMcp42_319+ROcp42_619*qd[20];
ORcp42_120 = OMcp42_219*RLcp42_320-OMcp42_319*RLcp42_220;
ORcp42_220 = -(OMcp42_119*RLcp42_320-OMcp42_319*RLcp42_120);
ORcp42_320 = OMcp42_119*RLcp42_220-OMcp42_219*RLcp42_120;
OPcp42_120 = OPcp42_119+ROcp42_419*qdd[20]+qd[20]*(OMcp42_219*ROcp42_619-OMcp42_319*ROcp42_519);
OPcp42_220 = OPcp42_219+ROcp42_519*qdd[20]-qd[20]*(OMcp42_119*ROcp42_619-OMcp42_319*ROcp42_419);
OPcp42_320 = OPcp42_319+ROcp42_619*qdd[20]+qd[20]*(OMcp42_119*ROcp42_519-OMcp42_219*ROcp42_419);
RLcp42_121 = s->dpt[1][40]*ROcp42_120+s->dpt[2][40]*ROcp42_419+ROcp42_720*s->dpt[3][40];
RLcp42_221 = s->dpt[1][40]*ROcp42_220+s->dpt[2][40]*ROcp42_519+ROcp42_820*s->dpt[3][40];
RLcp42_321 = s->dpt[1][40]*ROcp42_320+s->dpt[2][40]*ROcp42_619+ROcp42_920*s->dpt[3][40];
OMcp42_121 = OMcp42_120+ROcp42_720*qd[21];
OMcp42_221 = OMcp42_220+ROcp42_820*qd[21];
OMcp42_321 = OMcp42_320+ROcp42_920*qd[21];
ORcp42_121 = OMcp42_220*RLcp42_321-OMcp42_320*RLcp42_221;
ORcp42_221 = -(OMcp42_120*RLcp42_321-OMcp42_320*RLcp42_121);
ORcp42_321 = OMcp42_120*RLcp42_221-OMcp42_220*RLcp42_121;
OPcp42_121 = OPcp42_120+ROcp42_720*qdd[21]+qd[21]*(OMcp42_220*ROcp42_920-OMcp42_320*ROcp42_820);
OPcp42_221 = OPcp42_220+ROcp42_820*qdd[21]-qd[21]*(OMcp42_120*ROcp42_920-OMcp42_320*ROcp42_720);
OPcp42_321 = OPcp42_320+ROcp42_920*qdd[21]+qd[21]*(OMcp42_120*ROcp42_820-OMcp42_220*ROcp42_720);

// = = Block_1_0_0_43_0_5 = = 
 
// Sensor Kinematics 


ROcp42_122 = ROcp42_121*C22-ROcp42_720*S22;
ROcp42_222 = ROcp42_221*C22-ROcp42_820*S22;
ROcp42_322 = ROcp42_321*C22-ROcp42_920*S22;
ROcp42_722 = ROcp42_121*S22+ROcp42_720*C22;
ROcp42_822 = ROcp42_221*S22+ROcp42_820*C22;
ROcp42_922 = ROcp42_321*S22+ROcp42_920*C22;
ROcp42_423 = ROcp42_421*C23+ROcp42_722*S23;
ROcp42_523 = ROcp42_521*C23+ROcp42_822*S23;
ROcp42_623 = ROcp42_621*C23+ROcp42_922*S23;
ROcp42_723 = -(ROcp42_421*S23-ROcp42_722*C23);
ROcp42_823 = -(ROcp42_521*S23-ROcp42_822*C23);
ROcp42_923 = -(ROcp42_621*S23-ROcp42_922*C23);
ROcp42_124 = ROcp42_122*C24+ROcp42_423*S24;
ROcp42_224 = ROcp42_222*C24+ROcp42_523*S24;
ROcp42_324 = ROcp42_322*C24+ROcp42_623*S24;
ROcp42_424 = -(ROcp42_122*S24-ROcp42_423*C24);
ROcp42_524 = -(ROcp42_222*S24-ROcp42_523*C24);
ROcp42_624 = -(ROcp42_322*S24-ROcp42_623*C24);
ROcp42_125 = ROcp42_124*C25-ROcp42_723*S25;
ROcp42_225 = ROcp42_224*C25-ROcp42_823*S25;
ROcp42_325 = ROcp42_324*C25-ROcp42_923*S25;
ROcp42_725 = ROcp42_124*S25+ROcp42_723*C25;
ROcp42_825 = ROcp42_224*S25+ROcp42_823*C25;
ROcp42_925 = ROcp42_324*S25+ROcp42_923*C25;
RLcp42_122 = ROcp42_121*s->dpt[1][44]+ROcp42_421*s->dpt[2][44]+ROcp42_720*s->dpt[3][44];
RLcp42_222 = ROcp42_221*s->dpt[1][44]+ROcp42_521*s->dpt[2][44]+ROcp42_820*s->dpt[3][44];
RLcp42_322 = ROcp42_321*s->dpt[1][44]+ROcp42_621*s->dpt[2][44]+ROcp42_920*s->dpt[3][44];
OMcp42_122 = OMcp42_121+ROcp42_421*qd[22];
OMcp42_222 = OMcp42_221+ROcp42_521*qd[22];
OMcp42_322 = OMcp42_321+ROcp42_621*qd[22];
ORcp42_122 = OMcp42_221*RLcp42_322-OMcp42_321*RLcp42_222;
ORcp42_222 = -(OMcp42_121*RLcp42_322-OMcp42_321*RLcp42_122);
ORcp42_322 = OMcp42_121*RLcp42_222-OMcp42_221*RLcp42_122;
OPcp42_122 = OPcp42_121+ROcp42_421*qdd[22]+qd[22]*(OMcp42_221*ROcp42_621-OMcp42_321*ROcp42_521);
OPcp42_222 = OPcp42_221+ROcp42_521*qdd[22]-qd[22]*(OMcp42_121*ROcp42_621-OMcp42_321*ROcp42_421);
OPcp42_322 = OPcp42_321+ROcp42_621*qdd[22]+qd[22]*(OMcp42_121*ROcp42_521-OMcp42_221*ROcp42_421);
RLcp42_123 = s->dpt[1][47]*ROcp42_122+s->dpt[3][47]*ROcp42_722+ROcp42_421*s->dpt[2][47];
RLcp42_223 = s->dpt[1][47]*ROcp42_222+s->dpt[3][47]*ROcp42_822+ROcp42_521*s->dpt[2][47];
RLcp42_323 = s->dpt[1][47]*ROcp42_322+s->dpt[3][47]*ROcp42_922+ROcp42_621*s->dpt[2][47];
OMcp42_123 = OMcp42_122+ROcp42_122*qd[23];
OMcp42_223 = OMcp42_222+ROcp42_222*qd[23];
OMcp42_323 = OMcp42_322+ROcp42_322*qd[23];
ORcp42_123 = OMcp42_222*RLcp42_323-OMcp42_322*RLcp42_223;
ORcp42_223 = -(OMcp42_122*RLcp42_323-OMcp42_322*RLcp42_123);
ORcp42_323 = OMcp42_122*RLcp42_223-OMcp42_222*RLcp42_123;
OPcp42_123 = OPcp42_122+ROcp42_122*qdd[23]+qd[23]*(OMcp42_222*ROcp42_322-OMcp42_322*ROcp42_222);
OPcp42_223 = OPcp42_222+ROcp42_222*qdd[23]-qd[23]*(OMcp42_122*ROcp42_322-OMcp42_322*ROcp42_122);
OPcp42_323 = OPcp42_322+ROcp42_322*qdd[23]+qd[23]*(OMcp42_122*ROcp42_222-OMcp42_222*ROcp42_122);
RLcp42_124 = s->dpt[1][49]*ROcp42_122+s->dpt[2][49]*ROcp42_423+ROcp42_723*s->dpt[3][49];
RLcp42_224 = s->dpt[1][49]*ROcp42_222+s->dpt[2][49]*ROcp42_523+ROcp42_823*s->dpt[3][49];
RLcp42_324 = s->dpt[1][49]*ROcp42_322+s->dpt[2][49]*ROcp42_623+ROcp42_923*s->dpt[3][49];
OMcp42_124 = OMcp42_123+ROcp42_723*qd[24];
OMcp42_224 = OMcp42_223+ROcp42_823*qd[24];
OMcp42_324 = OMcp42_323+ROcp42_923*qd[24];
ORcp42_124 = OMcp42_223*RLcp42_324-OMcp42_323*RLcp42_224;
ORcp42_224 = -(OMcp42_123*RLcp42_324-OMcp42_323*RLcp42_124);
ORcp42_324 = OMcp42_123*RLcp42_224-OMcp42_223*RLcp42_124;
OPcp42_124 = OPcp42_123+ROcp42_723*qdd[24]+qd[24]*(OMcp42_223*ROcp42_923-OMcp42_323*ROcp42_823);
OPcp42_224 = OPcp42_223+ROcp42_823*qdd[24]-qd[24]*(OMcp42_123*ROcp42_923-OMcp42_323*ROcp42_723);
OPcp42_324 = OPcp42_323+ROcp42_923*qdd[24]+qd[24]*(OMcp42_123*ROcp42_823-OMcp42_223*ROcp42_723);
RLcp42_125 = s->dpt[1][51]*ROcp42_124+s->dpt[2][51]*ROcp42_424+ROcp42_723*s->dpt[3][51];
RLcp42_225 = s->dpt[1][51]*ROcp42_224+s->dpt[2][51]*ROcp42_524+ROcp42_823*s->dpt[3][51];
RLcp42_325 = s->dpt[1][51]*ROcp42_324+s->dpt[2][51]*ROcp42_624+ROcp42_923*s->dpt[3][51];
OMcp42_125 = OMcp42_124+ROcp42_424*qd[25];
OMcp42_225 = OMcp42_224+ROcp42_524*qd[25];
OMcp42_325 = OMcp42_324+ROcp42_624*qd[25];
ORcp42_125 = OMcp42_224*RLcp42_325-OMcp42_324*RLcp42_225;
ORcp42_225 = -(OMcp42_124*RLcp42_325-OMcp42_324*RLcp42_125);
ORcp42_325 = OMcp42_124*RLcp42_225-OMcp42_224*RLcp42_125;
OPcp42_125 = OPcp42_124+ROcp42_424*qdd[25]+qd[25]*(OMcp42_224*ROcp42_624-OMcp42_324*ROcp42_524);
OPcp42_225 = OPcp42_224+ROcp42_524*qdd[25]-qd[25]*(OMcp42_124*ROcp42_624-OMcp42_324*ROcp42_424);
OPcp42_325 = OPcp42_324+ROcp42_624*qdd[25]+qd[25]*(OMcp42_124*ROcp42_524-OMcp42_224*ROcp42_424);
RLcp42_172 = s->dpt[1][53]*ROcp42_125+s->dpt[2][53]*ROcp42_424+ROcp42_725*s->dpt[3][53];
RLcp42_272 = s->dpt[1][53]*ROcp42_225+s->dpt[2][53]*ROcp42_524+ROcp42_825*s->dpt[3][53];
RLcp42_372 = s->dpt[1][53]*ROcp42_325+s->dpt[2][53]*ROcp42_624+ROcp42_925*s->dpt[3][53];
POcp42_172 = RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_172+q[1];
POcp42_272 = RLcp42_219+RLcp42_220+RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_272+q[2];
POcp42_372 = RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372+q[3];
JTcp42_272_4 = -(RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372);
JTcp42_372_4 = RLcp42_219+RLcp42_220+RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_272;
JTcp42_172_5 = C4*(RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372)-S4*(
 RLcp42_219+RLcp42_220)-S4*(RLcp42_221+RLcp42_222)-S4*(RLcp42_223+RLcp42_224)-S4*(RLcp42_225+RLcp42_272);
JTcp42_272_5 = S4*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_172);
JTcp42_372_5 = -C4*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_172);
JTcp42_172_6 = ROcp42_85*(RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372)-
 ROcp42_95*(RLcp42_219+RLcp42_220)-ROcp42_95*(RLcp42_221+RLcp42_222)-ROcp42_95*(RLcp42_223+RLcp42_224)-ROcp42_95*(RLcp42_225+
 RLcp42_272);
JTcp42_272_6 = RLcp42_172*ROcp42_95-RLcp42_325*S5-RLcp42_372*S5+ROcp42_95*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122
 +RLcp42_123+RLcp42_124+RLcp42_125)-S5*(RLcp42_319+RLcp42_320)-S5*(RLcp42_321+RLcp42_322)-S5*(RLcp42_323+RLcp42_324);
JTcp42_372_6 = RLcp42_225*S5-ROcp42_85*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125)+
 S5*(RLcp42_219+RLcp42_220)+S5*(RLcp42_221+RLcp42_222)+S5*(RLcp42_223+RLcp42_224)-RLcp42_172*ROcp42_85+RLcp42_272*S5;
JTcp42_172_7 = ROcp42_26*(RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325)-ROcp42_36*(RLcp42_220+
 RLcp42_221)-ROcp42_36*(RLcp42_222+RLcp42_223)-ROcp42_36*(RLcp42_224+RLcp42_225)-RLcp42_272*ROcp42_36+RLcp42_372*ROcp42_26;
JTcp42_272_7 = RLcp42_172*ROcp42_36-RLcp42_372*ROcp42_16-ROcp42_16*(RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+
 RLcp42_324+RLcp42_325)+ROcp42_36*(RLcp42_120+RLcp42_121)+ROcp42_36*(RLcp42_122+RLcp42_123)+ROcp42_36*(RLcp42_124+RLcp42_125);
JTcp42_372_7 = ROcp42_16*(RLcp42_220+RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225)-ROcp42_26*(RLcp42_120+
 RLcp42_121)-ROcp42_26*(RLcp42_122+RLcp42_123)-ROcp42_26*(RLcp42_124+RLcp42_125)-RLcp42_172*ROcp42_26+RLcp42_272*ROcp42_16;
JTcp42_172_8 = ROcp42_519*(RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372)-ROcp42_619*(RLcp42_221+
 RLcp42_222)-ROcp42_619*(RLcp42_223+RLcp42_224)-ROcp42_619*(RLcp42_225+RLcp42_272);
JTcp42_272_8 = -(ROcp42_419*(RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372)-ROcp42_619*(RLcp42_121
 +RLcp42_122)-ROcp42_619*(RLcp42_123+RLcp42_124)-ROcp42_619*(RLcp42_125+RLcp42_172));
JTcp42_372_8 = ROcp42_419*(RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_272)-ROcp42_519*(RLcp42_121+
 RLcp42_122)-ROcp42_519*(RLcp42_123+RLcp42_124)-ROcp42_519*(RLcp42_125+RLcp42_172);
JTcp42_172_9 = ROcp42_820*(RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325)-ROcp42_920*(RLcp42_222+RLcp42_223)-ROcp42_920*
 (RLcp42_224+RLcp42_225)-RLcp42_272*ROcp42_920+RLcp42_372*ROcp42_820;
JTcp42_272_9 = RLcp42_172*ROcp42_920-RLcp42_372*ROcp42_720-ROcp42_720*(RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325)+
 ROcp42_920*(RLcp42_122+RLcp42_123)+ROcp42_920*(RLcp42_124+RLcp42_125);
JTcp42_372_9 = ROcp42_720*(RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225)-ROcp42_820*(RLcp42_122+RLcp42_123)-ROcp42_820*
 (RLcp42_124+RLcp42_125)-RLcp42_172*ROcp42_820+RLcp42_272*ROcp42_720;
JTcp42_172_10 = ROcp42_521*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372)-ROcp42_621*(RLcp42_223+RLcp42_224)-ROcp42_621
 *(RLcp42_225+RLcp42_272);
JTcp42_272_10 = -(ROcp42_421*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_372)-ROcp42_621*(RLcp42_123+RLcp42_124)-
 ROcp42_621*(RLcp42_125+RLcp42_172));
JTcp42_372_10 = ROcp42_421*(RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_272)-ROcp42_521*(RLcp42_123+RLcp42_124)-ROcp42_521
 *(RLcp42_125+RLcp42_172);
JTcp42_172_11 = ROcp42_222*(RLcp42_324+RLcp42_325)-ROcp42_322*(RLcp42_224+RLcp42_225)-RLcp42_272*ROcp42_322+RLcp42_372
 *ROcp42_222;
JTcp42_272_11 = RLcp42_172*ROcp42_322-RLcp42_372*ROcp42_122-ROcp42_122*(RLcp42_324+RLcp42_325)+ROcp42_322*(RLcp42_124+
 RLcp42_125);
JTcp42_372_11 = ROcp42_122*(RLcp42_224+RLcp42_225)-ROcp42_222*(RLcp42_124+RLcp42_125)-RLcp42_172*ROcp42_222+RLcp42_272
 *ROcp42_122;
JTcp42_172_12 = ROcp42_823*(RLcp42_325+RLcp42_372)-ROcp42_923*(RLcp42_225+RLcp42_272);
JTcp42_272_12 = -(ROcp42_723*(RLcp42_325+RLcp42_372)-ROcp42_923*(RLcp42_125+RLcp42_172));
JTcp42_372_12 = ROcp42_723*(RLcp42_225+RLcp42_272)-ROcp42_823*(RLcp42_125+RLcp42_172);
JTcp42_172_13 = -(RLcp42_272*ROcp42_624-RLcp42_372*ROcp42_524);
JTcp42_272_13 = RLcp42_172*ROcp42_624-RLcp42_372*ROcp42_424;
JTcp42_372_13 = -(RLcp42_172*ROcp42_524-RLcp42_272*ROcp42_424);
ORcp42_172 = OMcp42_225*RLcp42_372-OMcp42_325*RLcp42_272;
ORcp42_272 = -(OMcp42_125*RLcp42_372-OMcp42_325*RLcp42_172);
ORcp42_372 = OMcp42_125*RLcp42_272-OMcp42_225*RLcp42_172;
VIcp42_172 = ORcp42_119+ORcp42_120+ORcp42_121+ORcp42_122+ORcp42_123+ORcp42_124+ORcp42_125+ORcp42_172+qd[1];
VIcp42_272 = ORcp42_219+ORcp42_220+ORcp42_221+ORcp42_222+ORcp42_223+ORcp42_224+ORcp42_225+ORcp42_272+qd[2];
VIcp42_372 = ORcp42_319+ORcp42_320+ORcp42_321+ORcp42_322+ORcp42_323+ORcp42_324+ORcp42_325+ORcp42_372+qd[3];
ACcp42_172 = qdd[1]+OMcp42_219*ORcp42_320+OMcp42_220*ORcp42_321+OMcp42_221*ORcp42_322+OMcp42_222*ORcp42_323+OMcp42_223
 *ORcp42_324+OMcp42_224*ORcp42_325+OMcp42_225*ORcp42_372+OMcp42_26*ORcp42_319-OMcp42_319*ORcp42_220-OMcp42_320*ORcp42_221-
 OMcp42_321*ORcp42_222-OMcp42_322*ORcp42_223-OMcp42_323*ORcp42_224-OMcp42_324*ORcp42_225-OMcp42_325*ORcp42_272-OMcp42_36*
 ORcp42_219+OPcp42_219*RLcp42_320+OPcp42_220*RLcp42_321+OPcp42_221*RLcp42_322+OPcp42_222*RLcp42_323+OPcp42_223*RLcp42_324+
 OPcp42_224*RLcp42_325+OPcp42_225*RLcp42_372+OPcp42_26*RLcp42_319-OPcp42_319*RLcp42_220-OPcp42_320*RLcp42_221-OPcp42_321*
 RLcp42_222-OPcp42_322*RLcp42_223-OPcp42_323*RLcp42_224-OPcp42_324*RLcp42_225-OPcp42_325*RLcp42_272-OPcp42_36*RLcp42_219;
ACcp42_272 = qdd[2]-OMcp42_119*ORcp42_320-OMcp42_120*ORcp42_321-OMcp42_121*ORcp42_322-OMcp42_122*ORcp42_323-OMcp42_123
 *ORcp42_324-OMcp42_124*ORcp42_325-OMcp42_125*ORcp42_372-OMcp42_16*ORcp42_319+OMcp42_319*ORcp42_120+OMcp42_320*ORcp42_121+
 OMcp42_321*ORcp42_122+OMcp42_322*ORcp42_123+OMcp42_323*ORcp42_124+OMcp42_324*ORcp42_125+OMcp42_325*ORcp42_172+OMcp42_36*
 ORcp42_119-OPcp42_119*RLcp42_320-OPcp42_120*RLcp42_321-OPcp42_121*RLcp42_322-OPcp42_122*RLcp42_323-OPcp42_123*RLcp42_324-
 OPcp42_124*RLcp42_325-OPcp42_125*RLcp42_372-OPcp42_16*RLcp42_319+OPcp42_319*RLcp42_120+OPcp42_320*RLcp42_121+OPcp42_321*
 RLcp42_122+OPcp42_322*RLcp42_123+OPcp42_323*RLcp42_124+OPcp42_324*RLcp42_125+OPcp42_325*RLcp42_172+OPcp42_36*RLcp42_119;
ACcp42_372 = qdd[3]+OMcp42_119*ORcp42_220+OMcp42_120*ORcp42_221+OMcp42_121*ORcp42_222+OMcp42_122*ORcp42_223+OMcp42_123
 *ORcp42_224+OMcp42_124*ORcp42_225+OMcp42_125*ORcp42_272+OMcp42_16*ORcp42_219-OMcp42_219*ORcp42_120-OMcp42_220*ORcp42_121-
 OMcp42_221*ORcp42_122-OMcp42_222*ORcp42_123-OMcp42_223*ORcp42_124-OMcp42_224*ORcp42_125-OMcp42_225*ORcp42_172-OMcp42_26*
 ORcp42_119+OPcp42_119*RLcp42_220+OPcp42_120*RLcp42_221+OPcp42_121*RLcp42_222+OPcp42_122*RLcp42_223+OPcp42_123*RLcp42_224+
 OPcp42_124*RLcp42_225+OPcp42_125*RLcp42_272+OPcp42_16*RLcp42_219-OPcp42_219*RLcp42_120-OPcp42_220*RLcp42_121-OPcp42_221*
 RLcp42_122-OPcp42_222*RLcp42_123-OPcp42_223*RLcp42_124-OPcp42_224*RLcp42_125-OPcp42_225*RLcp42_172-OPcp42_26*RLcp42_119;

// = = Block_1_0_0_43_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp42_172;
sens->P[2] = POcp42_272;
sens->P[3] = POcp42_372;
sens->R[1][1] = ROcp42_125;
sens->R[1][2] = ROcp42_225;
sens->R[1][3] = ROcp42_325;
sens->R[2][1] = ROcp42_424;
sens->R[2][2] = ROcp42_524;
sens->R[2][3] = ROcp42_624;
sens->R[3][1] = ROcp42_725;
sens->R[3][2] = ROcp42_825;
sens->R[3][3] = ROcp42_925;
sens->V[1] = VIcp42_172;
sens->V[2] = VIcp42_272;
sens->V[3] = VIcp42_372;
sens->OM[1] = OMcp42_125;
sens->OM[2] = OMcp42_225;
sens->OM[3] = OMcp42_325;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp42_172_5;
sens->J[1][6] = JTcp42_172_6;
sens->J[1][19] = JTcp42_172_7;
sens->J[1][20] = JTcp42_172_8;
sens->J[1][21] = JTcp42_172_9;
sens->J[1][22] = JTcp42_172_10;
sens->J[1][23] = JTcp42_172_11;
sens->J[1][24] = JTcp42_172_12;
sens->J[1][25] = JTcp42_172_13;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp42_272_4;
sens->J[2][5] = JTcp42_272_5;
sens->J[2][6] = JTcp42_272_6;
sens->J[2][19] = JTcp42_272_7;
sens->J[2][20] = JTcp42_272_8;
sens->J[2][21] = JTcp42_272_9;
sens->J[2][22] = JTcp42_272_10;
sens->J[2][23] = JTcp42_272_11;
sens->J[2][24] = JTcp42_272_12;
sens->J[2][25] = JTcp42_272_13;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp42_372_4;
sens->J[3][5] = JTcp42_372_5;
sens->J[3][6] = JTcp42_372_6;
sens->J[3][19] = JTcp42_372_7;
sens->J[3][20] = JTcp42_372_8;
sens->J[3][21] = JTcp42_372_9;
sens->J[3][22] = JTcp42_372_10;
sens->J[3][23] = JTcp42_372_11;
sens->J[3][24] = JTcp42_372_12;
sens->J[3][25] = JTcp42_372_13;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp42_16;
sens->J[4][20] = ROcp42_419;
sens->J[4][21] = ROcp42_720;
sens->J[4][22] = ROcp42_421;
sens->J[4][23] = ROcp42_122;
sens->J[4][24] = ROcp42_723;
sens->J[4][25] = ROcp42_424;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp42_85;
sens->J[5][19] = ROcp42_26;
sens->J[5][20] = ROcp42_519;
sens->J[5][21] = ROcp42_820;
sens->J[5][22] = ROcp42_521;
sens->J[5][23] = ROcp42_222;
sens->J[5][24] = ROcp42_823;
sens->J[5][25] = ROcp42_524;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp42_95;
sens->J[6][19] = ROcp42_36;
sens->J[6][20] = ROcp42_619;
sens->J[6][21] = ROcp42_920;
sens->J[6][22] = ROcp42_621;
sens->J[6][23] = ROcp42_322;
sens->J[6][24] = ROcp42_923;
sens->J[6][25] = ROcp42_624;
sens->A[1] = ACcp42_172;
sens->A[2] = ACcp42_272;
sens->A[3] = ACcp42_372;
sens->OMP[1] = OPcp42_125;
sens->OMP[2] = OPcp42_225;
sens->OMP[3] = OPcp42_325;
 
// 
break;
case 44:
 


// = = Block_1_0_0_44_0_1 = = 
 
// Sensor Kinematics 


ROcp43_25 = S4*S5;
ROcp43_35 = -C4*S5;
ROcp43_85 = -S4*C5;
ROcp43_95 = C4*C5;
ROcp43_16 = C5*C6;
ROcp43_26 = ROcp43_25*C6+C4*S6;
ROcp43_36 = ROcp43_35*C6+S4*S6;
ROcp43_46 = -C5*S6;
ROcp43_56 = -(ROcp43_25*S6-C4*C6);
ROcp43_66 = -(ROcp43_35*S6-S4*C6);
OMcp43_25 = qd[5]*C4;
OMcp43_35 = qd[5]*S4;
OMcp43_16 = qd[4]+qd[6]*S5;
OMcp43_26 = OMcp43_25+ROcp43_85*qd[6];
OMcp43_36 = OMcp43_35+ROcp43_95*qd[6];
OPcp43_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp43_26 = ROcp43_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp43_35*S5-ROcp43_95*qd[4]);
OPcp43_36 = ROcp43_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp43_25*S5-ROcp43_85*qd[4]);

// = = Block_1_0_0_44_0_4 = = 
 
// Sensor Kinematics 


ROcp43_419 = ROcp43_46*C19+S19*S5;
ROcp43_519 = ROcp43_56*C19+ROcp43_85*S19;
ROcp43_619 = ROcp43_66*C19+ROcp43_95*S19;
ROcp43_719 = -(ROcp43_46*S19-C19*S5);
ROcp43_819 = -(ROcp43_56*S19-ROcp43_85*C19);
ROcp43_919 = -(ROcp43_66*S19-ROcp43_95*C19);
ROcp43_120 = ROcp43_16*C20-ROcp43_719*S20;
ROcp43_220 = ROcp43_26*C20-ROcp43_819*S20;
ROcp43_320 = ROcp43_36*C20-ROcp43_919*S20;
ROcp43_720 = ROcp43_16*S20+ROcp43_719*C20;
ROcp43_820 = ROcp43_26*S20+ROcp43_819*C20;
ROcp43_920 = ROcp43_36*S20+ROcp43_919*C20;
ROcp43_121 = ROcp43_120*C21+ROcp43_419*S21;
ROcp43_221 = ROcp43_220*C21+ROcp43_519*S21;
ROcp43_321 = ROcp43_320*C21+ROcp43_619*S21;
ROcp43_421 = -(ROcp43_120*S21-ROcp43_419*C21);
ROcp43_521 = -(ROcp43_220*S21-ROcp43_519*C21);
ROcp43_621 = -(ROcp43_320*S21-ROcp43_619*C21);
RLcp43_119 = s->dpt[2][3]*ROcp43_46+ROcp43_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp43_219 = s->dpt[2][3]*ROcp43_56+ROcp43_26*s->dpt[1][3]+ROcp43_85*s->dpt[3][3];
RLcp43_319 = s->dpt[2][3]*ROcp43_66+ROcp43_36*s->dpt[1][3]+ROcp43_95*s->dpt[3][3];
OMcp43_119 = OMcp43_16+ROcp43_16*qd[19];
OMcp43_219 = OMcp43_26+ROcp43_26*qd[19];
OMcp43_319 = OMcp43_36+ROcp43_36*qd[19];
ORcp43_119 = OMcp43_26*RLcp43_319-OMcp43_36*RLcp43_219;
ORcp43_219 = -(OMcp43_16*RLcp43_319-OMcp43_36*RLcp43_119);
ORcp43_319 = OMcp43_16*RLcp43_219-OMcp43_26*RLcp43_119;
OPcp43_119 = OPcp43_16+ROcp43_16*qdd[19]+qd[19]*(OMcp43_26*ROcp43_36-OMcp43_36*ROcp43_26);
OPcp43_219 = OPcp43_26+ROcp43_26*qdd[19]-qd[19]*(OMcp43_16*ROcp43_36-OMcp43_36*ROcp43_16);
OPcp43_319 = OPcp43_36+ROcp43_36*qdd[19]+qd[19]*(OMcp43_16*ROcp43_26-OMcp43_26*ROcp43_16);
RLcp43_120 = s->dpt[1][38]*ROcp43_16+s->dpt[2][38]*ROcp43_419+s->dpt[3][38]*ROcp43_719;
RLcp43_220 = s->dpt[1][38]*ROcp43_26+s->dpt[2][38]*ROcp43_519+s->dpt[3][38]*ROcp43_819;
RLcp43_320 = s->dpt[1][38]*ROcp43_36+s->dpt[2][38]*ROcp43_619+s->dpt[3][38]*ROcp43_919;
OMcp43_120 = OMcp43_119+ROcp43_419*qd[20];
OMcp43_220 = OMcp43_219+ROcp43_519*qd[20];
OMcp43_320 = OMcp43_319+ROcp43_619*qd[20];
ORcp43_120 = OMcp43_219*RLcp43_320-OMcp43_319*RLcp43_220;
ORcp43_220 = -(OMcp43_119*RLcp43_320-OMcp43_319*RLcp43_120);
ORcp43_320 = OMcp43_119*RLcp43_220-OMcp43_219*RLcp43_120;
OPcp43_120 = OPcp43_119+ROcp43_419*qdd[20]+qd[20]*(OMcp43_219*ROcp43_619-OMcp43_319*ROcp43_519);
OPcp43_220 = OPcp43_219+ROcp43_519*qdd[20]-qd[20]*(OMcp43_119*ROcp43_619-OMcp43_319*ROcp43_419);
OPcp43_320 = OPcp43_319+ROcp43_619*qdd[20]+qd[20]*(OMcp43_119*ROcp43_519-OMcp43_219*ROcp43_419);
RLcp43_121 = s->dpt[1][40]*ROcp43_120+s->dpt[2][40]*ROcp43_419+ROcp43_720*s->dpt[3][40];
RLcp43_221 = s->dpt[1][40]*ROcp43_220+s->dpt[2][40]*ROcp43_519+ROcp43_820*s->dpt[3][40];
RLcp43_321 = s->dpt[1][40]*ROcp43_320+s->dpt[2][40]*ROcp43_619+ROcp43_920*s->dpt[3][40];
OMcp43_121 = OMcp43_120+ROcp43_720*qd[21];
OMcp43_221 = OMcp43_220+ROcp43_820*qd[21];
OMcp43_321 = OMcp43_320+ROcp43_920*qd[21];
ORcp43_121 = OMcp43_220*RLcp43_321-OMcp43_320*RLcp43_221;
ORcp43_221 = -(OMcp43_120*RLcp43_321-OMcp43_320*RLcp43_121);
ORcp43_321 = OMcp43_120*RLcp43_221-OMcp43_220*RLcp43_121;
OPcp43_121 = OPcp43_120+ROcp43_720*qdd[21]+qd[21]*(OMcp43_220*ROcp43_920-OMcp43_320*ROcp43_820);
OPcp43_221 = OPcp43_220+ROcp43_820*qdd[21]-qd[21]*(OMcp43_120*ROcp43_920-OMcp43_320*ROcp43_720);
OPcp43_321 = OPcp43_320+ROcp43_920*qdd[21]+qd[21]*(OMcp43_120*ROcp43_820-OMcp43_220*ROcp43_720);

// = = Block_1_0_0_44_0_6 = = 
 
// Sensor Kinematics 


ROcp43_126 = ROcp43_121*C26-ROcp43_720*S26;
ROcp43_226 = ROcp43_221*C26-ROcp43_820*S26;
ROcp43_326 = ROcp43_321*C26-ROcp43_920*S26;
ROcp43_726 = ROcp43_121*S26+ROcp43_720*C26;
ROcp43_826 = ROcp43_221*S26+ROcp43_820*C26;
ROcp43_926 = ROcp43_321*S26+ROcp43_920*C26;
RLcp43_126 = ROcp43_121*s->dpt[1][45]+ROcp43_421*s->dpt[2][45]+ROcp43_720*s->dpt[3][45];
RLcp43_226 = ROcp43_221*s->dpt[1][45]+ROcp43_521*s->dpt[2][45]+ROcp43_820*s->dpt[3][45];
RLcp43_326 = ROcp43_321*s->dpt[1][45]+ROcp43_621*s->dpt[2][45]+ROcp43_920*s->dpt[3][45];
OMcp43_126 = OMcp43_121+ROcp43_421*qd[26];
OMcp43_226 = OMcp43_221+ROcp43_521*qd[26];
OMcp43_326 = OMcp43_321+ROcp43_621*qd[26];
ORcp43_126 = OMcp43_221*RLcp43_326-OMcp43_321*RLcp43_226;
ORcp43_226 = -(OMcp43_121*RLcp43_326-OMcp43_321*RLcp43_126);
ORcp43_326 = OMcp43_121*RLcp43_226-OMcp43_221*RLcp43_126;
OPcp43_126 = OPcp43_121+ROcp43_421*qdd[26]+qd[26]*(OMcp43_221*ROcp43_621-OMcp43_321*ROcp43_521);
OPcp43_226 = OPcp43_221+ROcp43_521*qdd[26]-qd[26]*(OMcp43_121*ROcp43_621-OMcp43_321*ROcp43_421);
OPcp43_326 = OPcp43_321+ROcp43_621*qdd[26]+qd[26]*(OMcp43_121*ROcp43_521-OMcp43_221*ROcp43_421);
RLcp43_173 = ROcp43_126*s->dpt[1][54]+ROcp43_421*s->dpt[2][54]+ROcp43_726*s->dpt[3][54];
RLcp43_273 = ROcp43_226*s->dpt[1][54]+ROcp43_521*s->dpt[2][54]+ROcp43_826*s->dpt[3][54];
RLcp43_373 = ROcp43_326*s->dpt[1][54]+ROcp43_621*s->dpt[2][54]+ROcp43_926*s->dpt[3][54];
POcp43_173 = RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_126+RLcp43_173+q[1];
POcp43_273 = RLcp43_219+RLcp43_220+RLcp43_221+RLcp43_226+RLcp43_273+q[2];
POcp43_373 = RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_326+RLcp43_373+q[3];
JTcp43_273_4 = -(RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_326+RLcp43_373);
JTcp43_373_4 = RLcp43_219+RLcp43_220+RLcp43_221+RLcp43_226+RLcp43_273;
JTcp43_173_5 = C4*(RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_326)-S4*(RLcp43_219+RLcp43_220)-S4*(RLcp43_221+RLcp43_226)-
 RLcp43_273*S4+RLcp43_373*C4;
JTcp43_273_5 = S4*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_126+RLcp43_173);
JTcp43_373_5 = -C4*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_126+RLcp43_173);
JTcp43_173_6 = ROcp43_85*(RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_326)-ROcp43_95*(RLcp43_219+RLcp43_220)-ROcp43_95*(
 RLcp43_221+RLcp43_226)-RLcp43_273*ROcp43_95+RLcp43_373*ROcp43_85;
JTcp43_273_6 = -(RLcp43_373*S5-ROcp43_95*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_126+RLcp43_173)+S5*(RLcp43_319+
 RLcp43_320)+S5*(RLcp43_321+RLcp43_326));
JTcp43_373_6 = RLcp43_273*S5-ROcp43_85*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_126+RLcp43_173)+S5*(RLcp43_219+
 RLcp43_220)+S5*(RLcp43_221+RLcp43_226);
JTcp43_173_7 = ROcp43_26*(RLcp43_320+RLcp43_321+RLcp43_326+RLcp43_373)-ROcp43_36*(RLcp43_220+RLcp43_221)-ROcp43_36*(
 RLcp43_226+RLcp43_273);
JTcp43_273_7 = -(ROcp43_16*(RLcp43_320+RLcp43_321+RLcp43_326+RLcp43_373)-ROcp43_36*(RLcp43_120+RLcp43_121)-ROcp43_36*(
 RLcp43_126+RLcp43_173));
JTcp43_373_7 = ROcp43_16*(RLcp43_220+RLcp43_221+RLcp43_226+RLcp43_273)-ROcp43_26*(RLcp43_120+RLcp43_121)-ROcp43_26*(
 RLcp43_126+RLcp43_173);
JTcp43_173_8 = ROcp43_519*(RLcp43_321+RLcp43_326)-ROcp43_619*(RLcp43_221+RLcp43_226)-RLcp43_273*ROcp43_619+RLcp43_373*
 ROcp43_519;
JTcp43_273_8 = RLcp43_173*ROcp43_619-RLcp43_373*ROcp43_419-ROcp43_419*(RLcp43_321+RLcp43_326)+ROcp43_619*(RLcp43_121+
 RLcp43_126);
JTcp43_373_8 = ROcp43_419*(RLcp43_221+RLcp43_226)-ROcp43_519*(RLcp43_121+RLcp43_126)-RLcp43_173*ROcp43_519+RLcp43_273*
 ROcp43_419;
JTcp43_173_9 = ROcp43_820*(RLcp43_326+RLcp43_373)-ROcp43_920*(RLcp43_226+RLcp43_273);
JTcp43_273_9 = -(ROcp43_720*(RLcp43_326+RLcp43_373)-ROcp43_920*(RLcp43_126+RLcp43_173));
JTcp43_373_9 = ROcp43_720*(RLcp43_226+RLcp43_273)-ROcp43_820*(RLcp43_126+RLcp43_173);
JTcp43_173_10 = -(RLcp43_273*ROcp43_621-RLcp43_373*ROcp43_521);
JTcp43_273_10 = RLcp43_173*ROcp43_621-RLcp43_373*ROcp43_421;
JTcp43_373_10 = -(RLcp43_173*ROcp43_521-RLcp43_273*ROcp43_421);
ORcp43_173 = OMcp43_226*RLcp43_373-OMcp43_326*RLcp43_273;
ORcp43_273 = -(OMcp43_126*RLcp43_373-OMcp43_326*RLcp43_173);
ORcp43_373 = OMcp43_126*RLcp43_273-OMcp43_226*RLcp43_173;
VIcp43_173 = ORcp43_119+ORcp43_120+ORcp43_121+ORcp43_126+ORcp43_173+qd[1];
VIcp43_273 = ORcp43_219+ORcp43_220+ORcp43_221+ORcp43_226+ORcp43_273+qd[2];
VIcp43_373 = ORcp43_319+ORcp43_320+ORcp43_321+ORcp43_326+ORcp43_373+qd[3];
ACcp43_173 = qdd[1]+OMcp43_219*ORcp43_320+OMcp43_220*ORcp43_321+OMcp43_221*ORcp43_326+OMcp43_226*ORcp43_373+OMcp43_26*
 ORcp43_319-OMcp43_319*ORcp43_220-OMcp43_320*ORcp43_221-OMcp43_321*ORcp43_226-OMcp43_326*ORcp43_273-OMcp43_36*ORcp43_219+
 OPcp43_219*RLcp43_320+OPcp43_220*RLcp43_321+OPcp43_221*RLcp43_326+OPcp43_226*RLcp43_373+OPcp43_26*RLcp43_319-OPcp43_319*
 RLcp43_220-OPcp43_320*RLcp43_221-OPcp43_321*RLcp43_226-OPcp43_326*RLcp43_273-OPcp43_36*RLcp43_219;
ACcp43_273 = qdd[2]-OMcp43_119*ORcp43_320-OMcp43_120*ORcp43_321-OMcp43_121*ORcp43_326-OMcp43_126*ORcp43_373-OMcp43_16*
 ORcp43_319+OMcp43_319*ORcp43_120+OMcp43_320*ORcp43_121+OMcp43_321*ORcp43_126+OMcp43_326*ORcp43_173+OMcp43_36*ORcp43_119-
 OPcp43_119*RLcp43_320-OPcp43_120*RLcp43_321-OPcp43_121*RLcp43_326-OPcp43_126*RLcp43_373-OPcp43_16*RLcp43_319+OPcp43_319*
 RLcp43_120+OPcp43_320*RLcp43_121+OPcp43_321*RLcp43_126+OPcp43_326*RLcp43_173+OPcp43_36*RLcp43_119;
ACcp43_373 = qdd[3]+OMcp43_119*ORcp43_220+OMcp43_120*ORcp43_221+OMcp43_121*ORcp43_226+OMcp43_126*ORcp43_273+OMcp43_16*
 ORcp43_219-OMcp43_219*ORcp43_120-OMcp43_220*ORcp43_121-OMcp43_221*ORcp43_126-OMcp43_226*ORcp43_173-OMcp43_26*ORcp43_119+
 OPcp43_119*RLcp43_220+OPcp43_120*RLcp43_221+OPcp43_121*RLcp43_226+OPcp43_126*RLcp43_273+OPcp43_16*RLcp43_219-OPcp43_219*
 RLcp43_120-OPcp43_220*RLcp43_121-OPcp43_221*RLcp43_126-OPcp43_226*RLcp43_173-OPcp43_26*RLcp43_119;

// = = Block_1_0_0_44_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp43_173;
sens->P[2] = POcp43_273;
sens->P[3] = POcp43_373;
sens->R[1][1] = ROcp43_126;
sens->R[1][2] = ROcp43_226;
sens->R[1][3] = ROcp43_326;
sens->R[2][1] = ROcp43_421;
sens->R[2][2] = ROcp43_521;
sens->R[2][3] = ROcp43_621;
sens->R[3][1] = ROcp43_726;
sens->R[3][2] = ROcp43_826;
sens->R[3][3] = ROcp43_926;
sens->V[1] = VIcp43_173;
sens->V[2] = VIcp43_273;
sens->V[3] = VIcp43_373;
sens->OM[1] = OMcp43_126;
sens->OM[2] = OMcp43_226;
sens->OM[3] = OMcp43_326;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp43_173_5;
sens->J[1][6] = JTcp43_173_6;
sens->J[1][19] = JTcp43_173_7;
sens->J[1][20] = JTcp43_173_8;
sens->J[1][21] = JTcp43_173_9;
sens->J[1][26] = JTcp43_173_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp43_273_4;
sens->J[2][5] = JTcp43_273_5;
sens->J[2][6] = JTcp43_273_6;
sens->J[2][19] = JTcp43_273_7;
sens->J[2][20] = JTcp43_273_8;
sens->J[2][21] = JTcp43_273_9;
sens->J[2][26] = JTcp43_273_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp43_373_4;
sens->J[3][5] = JTcp43_373_5;
sens->J[3][6] = JTcp43_373_6;
sens->J[3][19] = JTcp43_373_7;
sens->J[3][20] = JTcp43_373_8;
sens->J[3][21] = JTcp43_373_9;
sens->J[3][26] = JTcp43_373_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp43_16;
sens->J[4][20] = ROcp43_419;
sens->J[4][21] = ROcp43_720;
sens->J[4][26] = ROcp43_421;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp43_85;
sens->J[5][19] = ROcp43_26;
sens->J[5][20] = ROcp43_519;
sens->J[5][21] = ROcp43_820;
sens->J[5][26] = ROcp43_521;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp43_95;
sens->J[6][19] = ROcp43_36;
sens->J[6][20] = ROcp43_619;
sens->J[6][21] = ROcp43_920;
sens->J[6][26] = ROcp43_621;
sens->A[1] = ACcp43_173;
sens->A[2] = ACcp43_273;
sens->A[3] = ACcp43_373;
sens->OMP[1] = OPcp43_126;
sens->OMP[2] = OPcp43_226;
sens->OMP[3] = OPcp43_326;
 
// 
break;
case 45:
 


// = = Block_1_0_0_45_0_1 = = 
 
// Sensor Kinematics 


ROcp44_25 = S4*S5;
ROcp44_35 = -C4*S5;
ROcp44_85 = -S4*C5;
ROcp44_95 = C4*C5;
ROcp44_16 = C5*C6;
ROcp44_26 = ROcp44_25*C6+C4*S6;
ROcp44_36 = ROcp44_35*C6+S4*S6;
ROcp44_46 = -C5*S6;
ROcp44_56 = -(ROcp44_25*S6-C4*C6);
ROcp44_66 = -(ROcp44_35*S6-S4*C6);
OMcp44_25 = qd[5]*C4;
OMcp44_35 = qd[5]*S4;
OMcp44_16 = qd[4]+qd[6]*S5;
OMcp44_26 = OMcp44_25+ROcp44_85*qd[6];
OMcp44_36 = OMcp44_35+ROcp44_95*qd[6];
OPcp44_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp44_26 = ROcp44_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp44_35*S5-ROcp44_95*qd[4]);
OPcp44_36 = ROcp44_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp44_25*S5-ROcp44_85*qd[4]);

// = = Block_1_0_0_45_0_4 = = 
 
// Sensor Kinematics 


ROcp44_419 = ROcp44_46*C19+S19*S5;
ROcp44_519 = ROcp44_56*C19+ROcp44_85*S19;
ROcp44_619 = ROcp44_66*C19+ROcp44_95*S19;
ROcp44_719 = -(ROcp44_46*S19-C19*S5);
ROcp44_819 = -(ROcp44_56*S19-ROcp44_85*C19);
ROcp44_919 = -(ROcp44_66*S19-ROcp44_95*C19);
ROcp44_120 = ROcp44_16*C20-ROcp44_719*S20;
ROcp44_220 = ROcp44_26*C20-ROcp44_819*S20;
ROcp44_320 = ROcp44_36*C20-ROcp44_919*S20;
ROcp44_720 = ROcp44_16*S20+ROcp44_719*C20;
ROcp44_820 = ROcp44_26*S20+ROcp44_819*C20;
ROcp44_920 = ROcp44_36*S20+ROcp44_919*C20;
ROcp44_121 = ROcp44_120*C21+ROcp44_419*S21;
ROcp44_221 = ROcp44_220*C21+ROcp44_519*S21;
ROcp44_321 = ROcp44_320*C21+ROcp44_619*S21;
ROcp44_421 = -(ROcp44_120*S21-ROcp44_419*C21);
ROcp44_521 = -(ROcp44_220*S21-ROcp44_519*C21);
ROcp44_621 = -(ROcp44_320*S21-ROcp44_619*C21);
RLcp44_119 = s->dpt[2][3]*ROcp44_46+ROcp44_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp44_219 = s->dpt[2][3]*ROcp44_56+ROcp44_26*s->dpt[1][3]+ROcp44_85*s->dpt[3][3];
RLcp44_319 = s->dpt[2][3]*ROcp44_66+ROcp44_36*s->dpt[1][3]+ROcp44_95*s->dpt[3][3];
OMcp44_119 = OMcp44_16+ROcp44_16*qd[19];
OMcp44_219 = OMcp44_26+ROcp44_26*qd[19];
OMcp44_319 = OMcp44_36+ROcp44_36*qd[19];
ORcp44_119 = OMcp44_26*RLcp44_319-OMcp44_36*RLcp44_219;
ORcp44_219 = -(OMcp44_16*RLcp44_319-OMcp44_36*RLcp44_119);
ORcp44_319 = OMcp44_16*RLcp44_219-OMcp44_26*RLcp44_119;
OPcp44_119 = OPcp44_16+ROcp44_16*qdd[19]+qd[19]*(OMcp44_26*ROcp44_36-OMcp44_36*ROcp44_26);
OPcp44_219 = OPcp44_26+ROcp44_26*qdd[19]-qd[19]*(OMcp44_16*ROcp44_36-OMcp44_36*ROcp44_16);
OPcp44_319 = OPcp44_36+ROcp44_36*qdd[19]+qd[19]*(OMcp44_16*ROcp44_26-OMcp44_26*ROcp44_16);
RLcp44_120 = s->dpt[1][38]*ROcp44_16+s->dpt[2][38]*ROcp44_419+s->dpt[3][38]*ROcp44_719;
RLcp44_220 = s->dpt[1][38]*ROcp44_26+s->dpt[2][38]*ROcp44_519+s->dpt[3][38]*ROcp44_819;
RLcp44_320 = s->dpt[1][38]*ROcp44_36+s->dpt[2][38]*ROcp44_619+s->dpt[3][38]*ROcp44_919;
OMcp44_120 = OMcp44_119+ROcp44_419*qd[20];
OMcp44_220 = OMcp44_219+ROcp44_519*qd[20];
OMcp44_320 = OMcp44_319+ROcp44_619*qd[20];
ORcp44_120 = OMcp44_219*RLcp44_320-OMcp44_319*RLcp44_220;
ORcp44_220 = -(OMcp44_119*RLcp44_320-OMcp44_319*RLcp44_120);
ORcp44_320 = OMcp44_119*RLcp44_220-OMcp44_219*RLcp44_120;
OPcp44_120 = OPcp44_119+ROcp44_419*qdd[20]+qd[20]*(OMcp44_219*ROcp44_619-OMcp44_319*ROcp44_519);
OPcp44_220 = OPcp44_219+ROcp44_519*qdd[20]-qd[20]*(OMcp44_119*ROcp44_619-OMcp44_319*ROcp44_419);
OPcp44_320 = OPcp44_319+ROcp44_619*qdd[20]+qd[20]*(OMcp44_119*ROcp44_519-OMcp44_219*ROcp44_419);
RLcp44_121 = s->dpt[1][40]*ROcp44_120+s->dpt[2][40]*ROcp44_419+ROcp44_720*s->dpt[3][40];
RLcp44_221 = s->dpt[1][40]*ROcp44_220+s->dpt[2][40]*ROcp44_519+ROcp44_820*s->dpt[3][40];
RLcp44_321 = s->dpt[1][40]*ROcp44_320+s->dpt[2][40]*ROcp44_619+ROcp44_920*s->dpt[3][40];
OMcp44_121 = OMcp44_120+ROcp44_720*qd[21];
OMcp44_221 = OMcp44_220+ROcp44_820*qd[21];
OMcp44_321 = OMcp44_320+ROcp44_920*qd[21];
ORcp44_121 = OMcp44_220*RLcp44_321-OMcp44_320*RLcp44_221;
ORcp44_221 = -(OMcp44_120*RLcp44_321-OMcp44_320*RLcp44_121);
ORcp44_321 = OMcp44_120*RLcp44_221-OMcp44_220*RLcp44_121;
OPcp44_121 = OPcp44_120+ROcp44_720*qdd[21]+qd[21]*(OMcp44_220*ROcp44_920-OMcp44_320*ROcp44_820);
OPcp44_221 = OPcp44_220+ROcp44_820*qdd[21]-qd[21]*(OMcp44_120*ROcp44_920-OMcp44_320*ROcp44_720);
OPcp44_321 = OPcp44_320+ROcp44_920*qdd[21]+qd[21]*(OMcp44_120*ROcp44_820-OMcp44_220*ROcp44_720);

// = = Block_1_0_0_45_0_6 = = 
 
// Sensor Kinematics 


ROcp44_126 = ROcp44_121*C26-ROcp44_720*S26;
ROcp44_226 = ROcp44_221*C26-ROcp44_820*S26;
ROcp44_326 = ROcp44_321*C26-ROcp44_920*S26;
ROcp44_726 = ROcp44_121*S26+ROcp44_720*C26;
ROcp44_826 = ROcp44_221*S26+ROcp44_820*C26;
ROcp44_926 = ROcp44_321*S26+ROcp44_920*C26;
RLcp44_126 = ROcp44_121*s->dpt[1][45]+ROcp44_421*s->dpt[2][45]+ROcp44_720*s->dpt[3][45];
RLcp44_226 = ROcp44_221*s->dpt[1][45]+ROcp44_521*s->dpt[2][45]+ROcp44_820*s->dpt[3][45];
RLcp44_326 = ROcp44_321*s->dpt[1][45]+ROcp44_621*s->dpt[2][45]+ROcp44_920*s->dpt[3][45];
OMcp44_126 = OMcp44_121+ROcp44_421*qd[26];
OMcp44_226 = OMcp44_221+ROcp44_521*qd[26];
OMcp44_326 = OMcp44_321+ROcp44_621*qd[26];
ORcp44_126 = OMcp44_221*RLcp44_326-OMcp44_321*RLcp44_226;
ORcp44_226 = -(OMcp44_121*RLcp44_326-OMcp44_321*RLcp44_126);
ORcp44_326 = OMcp44_121*RLcp44_226-OMcp44_221*RLcp44_126;
OPcp44_126 = OPcp44_121+ROcp44_421*qdd[26]+qd[26]*(OMcp44_221*ROcp44_621-OMcp44_321*ROcp44_521);
OPcp44_226 = OPcp44_221+ROcp44_521*qdd[26]-qd[26]*(OMcp44_121*ROcp44_621-OMcp44_321*ROcp44_421);
OPcp44_326 = OPcp44_321+ROcp44_621*qdd[26]+qd[26]*(OMcp44_121*ROcp44_521-OMcp44_221*ROcp44_421);
RLcp44_174 = s->dpt[1][55]*ROcp44_126+s->dpt[3][55]*ROcp44_726+ROcp44_421*s->dpt[2][55];
RLcp44_274 = s->dpt[1][55]*ROcp44_226+s->dpt[3][55]*ROcp44_826+ROcp44_521*s->dpt[2][55];
RLcp44_374 = s->dpt[1][55]*ROcp44_326+s->dpt[3][55]*ROcp44_926+ROcp44_621*s->dpt[2][55];
POcp44_174 = RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_126+RLcp44_174+q[1];
POcp44_274 = RLcp44_219+RLcp44_220+RLcp44_221+RLcp44_226+RLcp44_274+q[2];
POcp44_374 = RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_326+RLcp44_374+q[3];
JTcp44_274_4 = -(RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_326+RLcp44_374);
JTcp44_374_4 = RLcp44_219+RLcp44_220+RLcp44_221+RLcp44_226+RLcp44_274;
JTcp44_174_5 = C4*(RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_326)-S4*(RLcp44_219+RLcp44_220)-S4*(RLcp44_221+RLcp44_226)-
 RLcp44_274*S4+RLcp44_374*C4;
JTcp44_274_5 = S4*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_126+RLcp44_174);
JTcp44_374_5 = -C4*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_126+RLcp44_174);
JTcp44_174_6 = ROcp44_85*(RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_326)-ROcp44_95*(RLcp44_219+RLcp44_220)-ROcp44_95*(
 RLcp44_221+RLcp44_226)-RLcp44_274*ROcp44_95+RLcp44_374*ROcp44_85;
JTcp44_274_6 = -(RLcp44_374*S5-ROcp44_95*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_126+RLcp44_174)+S5*(RLcp44_319+
 RLcp44_320)+S5*(RLcp44_321+RLcp44_326));
JTcp44_374_6 = RLcp44_274*S5-ROcp44_85*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_126+RLcp44_174)+S5*(RLcp44_219+
 RLcp44_220)+S5*(RLcp44_221+RLcp44_226);
JTcp44_174_7 = ROcp44_26*(RLcp44_320+RLcp44_321+RLcp44_326+RLcp44_374)-ROcp44_36*(RLcp44_220+RLcp44_221)-ROcp44_36*(
 RLcp44_226+RLcp44_274);
JTcp44_274_7 = -(ROcp44_16*(RLcp44_320+RLcp44_321+RLcp44_326+RLcp44_374)-ROcp44_36*(RLcp44_120+RLcp44_121)-ROcp44_36*(
 RLcp44_126+RLcp44_174));
JTcp44_374_7 = ROcp44_16*(RLcp44_220+RLcp44_221+RLcp44_226+RLcp44_274)-ROcp44_26*(RLcp44_120+RLcp44_121)-ROcp44_26*(
 RLcp44_126+RLcp44_174);
JTcp44_174_8 = ROcp44_519*(RLcp44_321+RLcp44_326)-ROcp44_619*(RLcp44_221+RLcp44_226)-RLcp44_274*ROcp44_619+RLcp44_374*
 ROcp44_519;
JTcp44_274_8 = RLcp44_174*ROcp44_619-RLcp44_374*ROcp44_419-ROcp44_419*(RLcp44_321+RLcp44_326)+ROcp44_619*(RLcp44_121+
 RLcp44_126);
JTcp44_374_8 = ROcp44_419*(RLcp44_221+RLcp44_226)-ROcp44_519*(RLcp44_121+RLcp44_126)-RLcp44_174*ROcp44_519+RLcp44_274*
 ROcp44_419;
JTcp44_174_9 = ROcp44_820*(RLcp44_326+RLcp44_374)-ROcp44_920*(RLcp44_226+RLcp44_274);
JTcp44_274_9 = -(ROcp44_720*(RLcp44_326+RLcp44_374)-ROcp44_920*(RLcp44_126+RLcp44_174));
JTcp44_374_9 = ROcp44_720*(RLcp44_226+RLcp44_274)-ROcp44_820*(RLcp44_126+RLcp44_174);
JTcp44_174_10 = -(RLcp44_274*ROcp44_621-RLcp44_374*ROcp44_521);
JTcp44_274_10 = RLcp44_174*ROcp44_621-RLcp44_374*ROcp44_421;
JTcp44_374_10 = -(RLcp44_174*ROcp44_521-RLcp44_274*ROcp44_421);
ORcp44_174 = OMcp44_226*RLcp44_374-OMcp44_326*RLcp44_274;
ORcp44_274 = -(OMcp44_126*RLcp44_374-OMcp44_326*RLcp44_174);
ORcp44_374 = OMcp44_126*RLcp44_274-OMcp44_226*RLcp44_174;
VIcp44_174 = ORcp44_119+ORcp44_120+ORcp44_121+ORcp44_126+ORcp44_174+qd[1];
VIcp44_274 = ORcp44_219+ORcp44_220+ORcp44_221+ORcp44_226+ORcp44_274+qd[2];
VIcp44_374 = ORcp44_319+ORcp44_320+ORcp44_321+ORcp44_326+ORcp44_374+qd[3];
ACcp44_174 = qdd[1]+OMcp44_219*ORcp44_320+OMcp44_220*ORcp44_321+OMcp44_221*ORcp44_326+OMcp44_226*ORcp44_374+OMcp44_26*
 ORcp44_319-OMcp44_319*ORcp44_220-OMcp44_320*ORcp44_221-OMcp44_321*ORcp44_226-OMcp44_326*ORcp44_274-OMcp44_36*ORcp44_219+
 OPcp44_219*RLcp44_320+OPcp44_220*RLcp44_321+OPcp44_221*RLcp44_326+OPcp44_226*RLcp44_374+OPcp44_26*RLcp44_319-OPcp44_319*
 RLcp44_220-OPcp44_320*RLcp44_221-OPcp44_321*RLcp44_226-OPcp44_326*RLcp44_274-OPcp44_36*RLcp44_219;
ACcp44_274 = qdd[2]-OMcp44_119*ORcp44_320-OMcp44_120*ORcp44_321-OMcp44_121*ORcp44_326-OMcp44_126*ORcp44_374-OMcp44_16*
 ORcp44_319+OMcp44_319*ORcp44_120+OMcp44_320*ORcp44_121+OMcp44_321*ORcp44_126+OMcp44_326*ORcp44_174+OMcp44_36*ORcp44_119-
 OPcp44_119*RLcp44_320-OPcp44_120*RLcp44_321-OPcp44_121*RLcp44_326-OPcp44_126*RLcp44_374-OPcp44_16*RLcp44_319+OPcp44_319*
 RLcp44_120+OPcp44_320*RLcp44_121+OPcp44_321*RLcp44_126+OPcp44_326*RLcp44_174+OPcp44_36*RLcp44_119;
ACcp44_374 = qdd[3]+OMcp44_119*ORcp44_220+OMcp44_120*ORcp44_221+OMcp44_121*ORcp44_226+OMcp44_126*ORcp44_274+OMcp44_16*
 ORcp44_219-OMcp44_219*ORcp44_120-OMcp44_220*ORcp44_121-OMcp44_221*ORcp44_126-OMcp44_226*ORcp44_174-OMcp44_26*ORcp44_119+
 OPcp44_119*RLcp44_220+OPcp44_120*RLcp44_221+OPcp44_121*RLcp44_226+OPcp44_126*RLcp44_274+OPcp44_16*RLcp44_219-OPcp44_219*
 RLcp44_120-OPcp44_220*RLcp44_121-OPcp44_221*RLcp44_126-OPcp44_226*RLcp44_174-OPcp44_26*RLcp44_119;

// = = Block_1_0_0_45_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp44_174;
sens->P[2] = POcp44_274;
sens->P[3] = POcp44_374;
sens->R[1][1] = ROcp44_126;
sens->R[1][2] = ROcp44_226;
sens->R[1][3] = ROcp44_326;
sens->R[2][1] = ROcp44_421;
sens->R[2][2] = ROcp44_521;
sens->R[2][3] = ROcp44_621;
sens->R[3][1] = ROcp44_726;
sens->R[3][2] = ROcp44_826;
sens->R[3][3] = ROcp44_926;
sens->V[1] = VIcp44_174;
sens->V[2] = VIcp44_274;
sens->V[3] = VIcp44_374;
sens->OM[1] = OMcp44_126;
sens->OM[2] = OMcp44_226;
sens->OM[3] = OMcp44_326;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp44_174_5;
sens->J[1][6] = JTcp44_174_6;
sens->J[1][19] = JTcp44_174_7;
sens->J[1][20] = JTcp44_174_8;
sens->J[1][21] = JTcp44_174_9;
sens->J[1][26] = JTcp44_174_10;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp44_274_4;
sens->J[2][5] = JTcp44_274_5;
sens->J[2][6] = JTcp44_274_6;
sens->J[2][19] = JTcp44_274_7;
sens->J[2][20] = JTcp44_274_8;
sens->J[2][21] = JTcp44_274_9;
sens->J[2][26] = JTcp44_274_10;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp44_374_4;
sens->J[3][5] = JTcp44_374_5;
sens->J[3][6] = JTcp44_374_6;
sens->J[3][19] = JTcp44_374_7;
sens->J[3][20] = JTcp44_374_8;
sens->J[3][21] = JTcp44_374_9;
sens->J[3][26] = JTcp44_374_10;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp44_16;
sens->J[4][20] = ROcp44_419;
sens->J[4][21] = ROcp44_720;
sens->J[4][26] = ROcp44_421;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp44_85;
sens->J[5][19] = ROcp44_26;
sens->J[5][20] = ROcp44_519;
sens->J[5][21] = ROcp44_820;
sens->J[5][26] = ROcp44_521;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp44_95;
sens->J[6][19] = ROcp44_36;
sens->J[6][20] = ROcp44_619;
sens->J[6][21] = ROcp44_920;
sens->J[6][26] = ROcp44_621;
sens->A[1] = ACcp44_174;
sens->A[2] = ACcp44_274;
sens->A[3] = ACcp44_374;
sens->OMP[1] = OPcp44_126;
sens->OMP[2] = OPcp44_226;
sens->OMP[3] = OPcp44_326;
 
// 
break;
case 46:
 


// = = Block_1_0_0_46_0_1 = = 
 
// Sensor Kinematics 


ROcp45_25 = S4*S5;
ROcp45_35 = -C4*S5;
ROcp45_85 = -S4*C5;
ROcp45_95 = C4*C5;
ROcp45_16 = C5*C6;
ROcp45_26 = ROcp45_25*C6+C4*S6;
ROcp45_36 = ROcp45_35*C6+S4*S6;
ROcp45_46 = -C5*S6;
ROcp45_56 = -(ROcp45_25*S6-C4*C6);
ROcp45_66 = -(ROcp45_35*S6-S4*C6);
OMcp45_25 = qd[5]*C4;
OMcp45_35 = qd[5]*S4;
OMcp45_16 = qd[4]+qd[6]*S5;
OMcp45_26 = OMcp45_25+ROcp45_85*qd[6];
OMcp45_36 = OMcp45_35+ROcp45_95*qd[6];
OPcp45_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp45_26 = ROcp45_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp45_35*S5-ROcp45_95*qd[4]);
OPcp45_36 = ROcp45_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp45_25*S5-ROcp45_85*qd[4]);

// = = Block_1_0_0_46_0_4 = = 
 
// Sensor Kinematics 


ROcp45_419 = ROcp45_46*C19+S19*S5;
ROcp45_519 = ROcp45_56*C19+ROcp45_85*S19;
ROcp45_619 = ROcp45_66*C19+ROcp45_95*S19;
ROcp45_719 = -(ROcp45_46*S19-C19*S5);
ROcp45_819 = -(ROcp45_56*S19-ROcp45_85*C19);
ROcp45_919 = -(ROcp45_66*S19-ROcp45_95*C19);
ROcp45_120 = ROcp45_16*C20-ROcp45_719*S20;
ROcp45_220 = ROcp45_26*C20-ROcp45_819*S20;
ROcp45_320 = ROcp45_36*C20-ROcp45_919*S20;
ROcp45_720 = ROcp45_16*S20+ROcp45_719*C20;
ROcp45_820 = ROcp45_26*S20+ROcp45_819*C20;
ROcp45_920 = ROcp45_36*S20+ROcp45_919*C20;
ROcp45_121 = ROcp45_120*C21+ROcp45_419*S21;
ROcp45_221 = ROcp45_220*C21+ROcp45_519*S21;
ROcp45_321 = ROcp45_320*C21+ROcp45_619*S21;
ROcp45_421 = -(ROcp45_120*S21-ROcp45_419*C21);
ROcp45_521 = -(ROcp45_220*S21-ROcp45_519*C21);
ROcp45_621 = -(ROcp45_320*S21-ROcp45_619*C21);
RLcp45_119 = s->dpt[2][3]*ROcp45_46+ROcp45_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp45_219 = s->dpt[2][3]*ROcp45_56+ROcp45_26*s->dpt[1][3]+ROcp45_85*s->dpt[3][3];
RLcp45_319 = s->dpt[2][3]*ROcp45_66+ROcp45_36*s->dpt[1][3]+ROcp45_95*s->dpt[3][3];
OMcp45_119 = OMcp45_16+ROcp45_16*qd[19];
OMcp45_219 = OMcp45_26+ROcp45_26*qd[19];
OMcp45_319 = OMcp45_36+ROcp45_36*qd[19];
ORcp45_119 = OMcp45_26*RLcp45_319-OMcp45_36*RLcp45_219;
ORcp45_219 = -(OMcp45_16*RLcp45_319-OMcp45_36*RLcp45_119);
ORcp45_319 = OMcp45_16*RLcp45_219-OMcp45_26*RLcp45_119;
OPcp45_119 = OPcp45_16+ROcp45_16*qdd[19]+qd[19]*(OMcp45_26*ROcp45_36-OMcp45_36*ROcp45_26);
OPcp45_219 = OPcp45_26+ROcp45_26*qdd[19]-qd[19]*(OMcp45_16*ROcp45_36-OMcp45_36*ROcp45_16);
OPcp45_319 = OPcp45_36+ROcp45_36*qdd[19]+qd[19]*(OMcp45_16*ROcp45_26-OMcp45_26*ROcp45_16);
RLcp45_120 = s->dpt[1][38]*ROcp45_16+s->dpt[2][38]*ROcp45_419+s->dpt[3][38]*ROcp45_719;
RLcp45_220 = s->dpt[1][38]*ROcp45_26+s->dpt[2][38]*ROcp45_519+s->dpt[3][38]*ROcp45_819;
RLcp45_320 = s->dpt[1][38]*ROcp45_36+s->dpt[2][38]*ROcp45_619+s->dpt[3][38]*ROcp45_919;
OMcp45_120 = OMcp45_119+ROcp45_419*qd[20];
OMcp45_220 = OMcp45_219+ROcp45_519*qd[20];
OMcp45_320 = OMcp45_319+ROcp45_619*qd[20];
ORcp45_120 = OMcp45_219*RLcp45_320-OMcp45_319*RLcp45_220;
ORcp45_220 = -(OMcp45_119*RLcp45_320-OMcp45_319*RLcp45_120);
ORcp45_320 = OMcp45_119*RLcp45_220-OMcp45_219*RLcp45_120;
OPcp45_120 = OPcp45_119+ROcp45_419*qdd[20]+qd[20]*(OMcp45_219*ROcp45_619-OMcp45_319*ROcp45_519);
OPcp45_220 = OPcp45_219+ROcp45_519*qdd[20]-qd[20]*(OMcp45_119*ROcp45_619-OMcp45_319*ROcp45_419);
OPcp45_320 = OPcp45_319+ROcp45_619*qdd[20]+qd[20]*(OMcp45_119*ROcp45_519-OMcp45_219*ROcp45_419);
RLcp45_121 = s->dpt[1][40]*ROcp45_120+s->dpt[2][40]*ROcp45_419+ROcp45_720*s->dpt[3][40];
RLcp45_221 = s->dpt[1][40]*ROcp45_220+s->dpt[2][40]*ROcp45_519+ROcp45_820*s->dpt[3][40];
RLcp45_321 = s->dpt[1][40]*ROcp45_320+s->dpt[2][40]*ROcp45_619+ROcp45_920*s->dpt[3][40];
OMcp45_121 = OMcp45_120+ROcp45_720*qd[21];
OMcp45_221 = OMcp45_220+ROcp45_820*qd[21];
OMcp45_321 = OMcp45_320+ROcp45_920*qd[21];
ORcp45_121 = OMcp45_220*RLcp45_321-OMcp45_320*RLcp45_221;
ORcp45_221 = -(OMcp45_120*RLcp45_321-OMcp45_320*RLcp45_121);
ORcp45_321 = OMcp45_120*RLcp45_221-OMcp45_220*RLcp45_121;
OPcp45_121 = OPcp45_120+ROcp45_720*qdd[21]+qd[21]*(OMcp45_220*ROcp45_920-OMcp45_320*ROcp45_820);
OPcp45_221 = OPcp45_220+ROcp45_820*qdd[21]-qd[21]*(OMcp45_120*ROcp45_920-OMcp45_320*ROcp45_720);
OPcp45_321 = OPcp45_320+ROcp45_920*qdd[21]+qd[21]*(OMcp45_120*ROcp45_820-OMcp45_220*ROcp45_720);

// = = Block_1_0_0_46_0_6 = = 
 
// Sensor Kinematics 


ROcp45_126 = ROcp45_121*C26-ROcp45_720*S26;
ROcp45_226 = ROcp45_221*C26-ROcp45_820*S26;
ROcp45_326 = ROcp45_321*C26-ROcp45_920*S26;
ROcp45_726 = ROcp45_121*S26+ROcp45_720*C26;
ROcp45_826 = ROcp45_221*S26+ROcp45_820*C26;
ROcp45_926 = ROcp45_321*S26+ROcp45_920*C26;
ROcp45_427 = ROcp45_421*C27+ROcp45_726*S27;
ROcp45_527 = ROcp45_521*C27+ROcp45_826*S27;
ROcp45_627 = ROcp45_621*C27+ROcp45_926*S27;
ROcp45_727 = -(ROcp45_421*S27-ROcp45_726*C27);
ROcp45_827 = -(ROcp45_521*S27-ROcp45_826*C27);
ROcp45_927 = -(ROcp45_621*S27-ROcp45_926*C27);
RLcp45_126 = ROcp45_121*s->dpt[1][45]+ROcp45_421*s->dpt[2][45]+ROcp45_720*s->dpt[3][45];
RLcp45_226 = ROcp45_221*s->dpt[1][45]+ROcp45_521*s->dpt[2][45]+ROcp45_820*s->dpt[3][45];
RLcp45_326 = ROcp45_321*s->dpt[1][45]+ROcp45_621*s->dpt[2][45]+ROcp45_920*s->dpt[3][45];
OMcp45_126 = OMcp45_121+ROcp45_421*qd[26];
OMcp45_226 = OMcp45_221+ROcp45_521*qd[26];
OMcp45_326 = OMcp45_321+ROcp45_621*qd[26];
ORcp45_126 = OMcp45_221*RLcp45_326-OMcp45_321*RLcp45_226;
ORcp45_226 = -(OMcp45_121*RLcp45_326-OMcp45_321*RLcp45_126);
ORcp45_326 = OMcp45_121*RLcp45_226-OMcp45_221*RLcp45_126;
OPcp45_126 = OPcp45_121+ROcp45_421*qdd[26]+qd[26]*(OMcp45_221*ROcp45_621-OMcp45_321*ROcp45_521);
OPcp45_226 = OPcp45_221+ROcp45_521*qdd[26]-qd[26]*(OMcp45_121*ROcp45_621-OMcp45_321*ROcp45_421);
OPcp45_326 = OPcp45_321+ROcp45_621*qdd[26]+qd[26]*(OMcp45_121*ROcp45_521-OMcp45_221*ROcp45_421);
RLcp45_127 = s->dpt[1][55]*ROcp45_126+s->dpt[3][55]*ROcp45_726+ROcp45_421*s->dpt[2][55];
RLcp45_227 = s->dpt[1][55]*ROcp45_226+s->dpt[3][55]*ROcp45_826+ROcp45_521*s->dpt[2][55];
RLcp45_327 = s->dpt[1][55]*ROcp45_326+s->dpt[3][55]*ROcp45_926+ROcp45_621*s->dpt[2][55];
OMcp45_127 = OMcp45_126+ROcp45_126*qd[27];
OMcp45_227 = OMcp45_226+ROcp45_226*qd[27];
OMcp45_327 = OMcp45_326+ROcp45_326*qd[27];
ORcp45_127 = OMcp45_226*RLcp45_327-OMcp45_326*RLcp45_227;
ORcp45_227 = -(OMcp45_126*RLcp45_327-OMcp45_326*RLcp45_127);
ORcp45_327 = OMcp45_126*RLcp45_227-OMcp45_226*RLcp45_127;
OPcp45_127 = OPcp45_126+ROcp45_126*qdd[27]+qd[27]*(OMcp45_226*ROcp45_326-OMcp45_326*ROcp45_226);
OPcp45_227 = OPcp45_226+ROcp45_226*qdd[27]-qd[27]*(OMcp45_126*ROcp45_326-OMcp45_326*ROcp45_126);
OPcp45_327 = OPcp45_326+ROcp45_326*qdd[27]+qd[27]*(OMcp45_126*ROcp45_226-OMcp45_226*ROcp45_126);
RLcp45_175 = ROcp45_126*s->dpt[1][56]+ROcp45_427*s->dpt[2][56]+ROcp45_727*s->dpt[3][56];
RLcp45_275 = ROcp45_226*s->dpt[1][56]+ROcp45_527*s->dpt[2][56]+ROcp45_827*s->dpt[3][56];
RLcp45_375 = ROcp45_326*s->dpt[1][56]+ROcp45_627*s->dpt[2][56]+ROcp45_927*s->dpt[3][56];
POcp45_175 = RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_126+RLcp45_127+RLcp45_175+q[1];
POcp45_275 = RLcp45_219+RLcp45_220+RLcp45_221+RLcp45_226+RLcp45_227+RLcp45_275+q[2];
POcp45_375 = RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_326+RLcp45_327+RLcp45_375+q[3];
JTcp45_275_4 = -(RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_326+RLcp45_327+RLcp45_375);
JTcp45_375_4 = RLcp45_219+RLcp45_220+RLcp45_221+RLcp45_226+RLcp45_227+RLcp45_275;
JTcp45_175_5 = C4*(RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_326+RLcp45_327+RLcp45_375)-S4*(RLcp45_219+RLcp45_220)-S4*(
 RLcp45_221+RLcp45_226)-S4*(RLcp45_227+RLcp45_275);
JTcp45_275_5 = S4*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_126+RLcp45_127+RLcp45_175);
JTcp45_375_5 = -C4*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_126+RLcp45_127+RLcp45_175);
JTcp45_175_6 = ROcp45_85*(RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_326+RLcp45_327+RLcp45_375)-ROcp45_95*(RLcp45_219+
 RLcp45_220)-ROcp45_95*(RLcp45_221+RLcp45_226)-ROcp45_95*(RLcp45_227+RLcp45_275);
JTcp45_275_6 = RLcp45_175*ROcp45_95-RLcp45_327*S5-RLcp45_375*S5+ROcp45_95*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_126
 +RLcp45_127)-S5*(RLcp45_319+RLcp45_320)-S5*(RLcp45_321+RLcp45_326);
JTcp45_375_6 = RLcp45_227*S5-ROcp45_85*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_126+RLcp45_127)+S5*(RLcp45_219+
 RLcp45_220)+S5*(RLcp45_221+RLcp45_226)-RLcp45_175*ROcp45_85+RLcp45_275*S5;
JTcp45_175_7 = ROcp45_26*(RLcp45_320+RLcp45_321+RLcp45_326+RLcp45_327)-ROcp45_36*(RLcp45_220+RLcp45_221)-ROcp45_36*(
 RLcp45_226+RLcp45_227)-RLcp45_275*ROcp45_36+RLcp45_375*ROcp45_26;
JTcp45_275_7 = RLcp45_175*ROcp45_36-RLcp45_375*ROcp45_16-ROcp45_16*(RLcp45_320+RLcp45_321+RLcp45_326+RLcp45_327)+
 ROcp45_36*(RLcp45_120+RLcp45_121)+ROcp45_36*(RLcp45_126+RLcp45_127);
JTcp45_375_7 = ROcp45_16*(RLcp45_220+RLcp45_221+RLcp45_226+RLcp45_227)-ROcp45_26*(RLcp45_120+RLcp45_121)-ROcp45_26*(
 RLcp45_126+RLcp45_127)-RLcp45_175*ROcp45_26+RLcp45_275*ROcp45_16;
JTcp45_175_8 = ROcp45_519*(RLcp45_321+RLcp45_326+RLcp45_327+RLcp45_375)-ROcp45_619*(RLcp45_221+RLcp45_226)-ROcp45_619*
 (RLcp45_227+RLcp45_275);
JTcp45_275_8 = -(ROcp45_419*(RLcp45_321+RLcp45_326+RLcp45_327+RLcp45_375)-ROcp45_619*(RLcp45_121+RLcp45_126)-
 ROcp45_619*(RLcp45_127+RLcp45_175));
JTcp45_375_8 = ROcp45_419*(RLcp45_221+RLcp45_226+RLcp45_227+RLcp45_275)-ROcp45_519*(RLcp45_121+RLcp45_126)-ROcp45_519*
 (RLcp45_127+RLcp45_175);
JTcp45_175_9 = ROcp45_820*(RLcp45_326+RLcp45_327)-ROcp45_920*(RLcp45_226+RLcp45_227)-RLcp45_275*ROcp45_920+RLcp45_375*
 ROcp45_820;
JTcp45_275_9 = RLcp45_175*ROcp45_920-RLcp45_375*ROcp45_720-ROcp45_720*(RLcp45_326+RLcp45_327)+ROcp45_920*(RLcp45_126+
 RLcp45_127);
JTcp45_375_9 = ROcp45_720*(RLcp45_226+RLcp45_227)-ROcp45_820*(RLcp45_126+RLcp45_127)-RLcp45_175*ROcp45_820+RLcp45_275*
 ROcp45_720;
JTcp45_175_10 = ROcp45_521*(RLcp45_327+RLcp45_375)-ROcp45_621*(RLcp45_227+RLcp45_275);
JTcp45_275_10 = -(ROcp45_421*(RLcp45_327+RLcp45_375)-ROcp45_621*(RLcp45_127+RLcp45_175));
JTcp45_375_10 = ROcp45_421*(RLcp45_227+RLcp45_275)-ROcp45_521*(RLcp45_127+RLcp45_175);
JTcp45_175_11 = -(RLcp45_275*ROcp45_326-RLcp45_375*ROcp45_226);
JTcp45_275_11 = RLcp45_175*ROcp45_326-RLcp45_375*ROcp45_126;
JTcp45_375_11 = -(RLcp45_175*ROcp45_226-RLcp45_275*ROcp45_126);
ORcp45_175 = OMcp45_227*RLcp45_375-OMcp45_327*RLcp45_275;
ORcp45_275 = -(OMcp45_127*RLcp45_375-OMcp45_327*RLcp45_175);
ORcp45_375 = OMcp45_127*RLcp45_275-OMcp45_227*RLcp45_175;
VIcp45_175 = ORcp45_119+ORcp45_120+ORcp45_121+ORcp45_126+ORcp45_127+ORcp45_175+qd[1];
VIcp45_275 = ORcp45_219+ORcp45_220+ORcp45_221+ORcp45_226+ORcp45_227+ORcp45_275+qd[2];
VIcp45_375 = ORcp45_319+ORcp45_320+ORcp45_321+ORcp45_326+ORcp45_327+ORcp45_375+qd[3];
ACcp45_175 = qdd[1]+OMcp45_219*ORcp45_320+OMcp45_220*ORcp45_321+OMcp45_221*ORcp45_326+OMcp45_226*ORcp45_327+OMcp45_227
 *ORcp45_375+OMcp45_26*ORcp45_319-OMcp45_319*ORcp45_220-OMcp45_320*ORcp45_221-OMcp45_321*ORcp45_226-OMcp45_326*ORcp45_227-
 OMcp45_327*ORcp45_275-OMcp45_36*ORcp45_219+OPcp45_219*RLcp45_320+OPcp45_220*RLcp45_321+OPcp45_221*RLcp45_326+OPcp45_226*
 RLcp45_327+OPcp45_227*RLcp45_375+OPcp45_26*RLcp45_319-OPcp45_319*RLcp45_220-OPcp45_320*RLcp45_221-OPcp45_321*RLcp45_226-
 OPcp45_326*RLcp45_227-OPcp45_327*RLcp45_275-OPcp45_36*RLcp45_219;
ACcp45_275 = qdd[2]-OMcp45_119*ORcp45_320-OMcp45_120*ORcp45_321-OMcp45_121*ORcp45_326-OMcp45_126*ORcp45_327-OMcp45_127
 *ORcp45_375-OMcp45_16*ORcp45_319+OMcp45_319*ORcp45_120+OMcp45_320*ORcp45_121+OMcp45_321*ORcp45_126+OMcp45_326*ORcp45_127+
 OMcp45_327*ORcp45_175+OMcp45_36*ORcp45_119-OPcp45_119*RLcp45_320-OPcp45_120*RLcp45_321-OPcp45_121*RLcp45_326-OPcp45_126*
 RLcp45_327-OPcp45_127*RLcp45_375-OPcp45_16*RLcp45_319+OPcp45_319*RLcp45_120+OPcp45_320*RLcp45_121+OPcp45_321*RLcp45_126+
 OPcp45_326*RLcp45_127+OPcp45_327*RLcp45_175+OPcp45_36*RLcp45_119;
ACcp45_375 = qdd[3]+OMcp45_119*ORcp45_220+OMcp45_120*ORcp45_221+OMcp45_121*ORcp45_226+OMcp45_126*ORcp45_227+OMcp45_127
 *ORcp45_275+OMcp45_16*ORcp45_219-OMcp45_219*ORcp45_120-OMcp45_220*ORcp45_121-OMcp45_221*ORcp45_126-OMcp45_226*ORcp45_127-
 OMcp45_227*ORcp45_175-OMcp45_26*ORcp45_119+OPcp45_119*RLcp45_220+OPcp45_120*RLcp45_221+OPcp45_121*RLcp45_226+OPcp45_126*
 RLcp45_227+OPcp45_127*RLcp45_275+OPcp45_16*RLcp45_219-OPcp45_219*RLcp45_120-OPcp45_220*RLcp45_121-OPcp45_221*RLcp45_126-
 OPcp45_226*RLcp45_127-OPcp45_227*RLcp45_175-OPcp45_26*RLcp45_119;

// = = Block_1_0_0_46_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp45_175;
sens->P[2] = POcp45_275;
sens->P[3] = POcp45_375;
sens->R[1][1] = ROcp45_126;
sens->R[1][2] = ROcp45_226;
sens->R[1][3] = ROcp45_326;
sens->R[2][1] = ROcp45_427;
sens->R[2][2] = ROcp45_527;
sens->R[2][3] = ROcp45_627;
sens->R[3][1] = ROcp45_727;
sens->R[3][2] = ROcp45_827;
sens->R[3][3] = ROcp45_927;
sens->V[1] = VIcp45_175;
sens->V[2] = VIcp45_275;
sens->V[3] = VIcp45_375;
sens->OM[1] = OMcp45_127;
sens->OM[2] = OMcp45_227;
sens->OM[3] = OMcp45_327;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp45_175_5;
sens->J[1][6] = JTcp45_175_6;
sens->J[1][19] = JTcp45_175_7;
sens->J[1][20] = JTcp45_175_8;
sens->J[1][21] = JTcp45_175_9;
sens->J[1][26] = JTcp45_175_10;
sens->J[1][27] = JTcp45_175_11;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp45_275_4;
sens->J[2][5] = JTcp45_275_5;
sens->J[2][6] = JTcp45_275_6;
sens->J[2][19] = JTcp45_275_7;
sens->J[2][20] = JTcp45_275_8;
sens->J[2][21] = JTcp45_275_9;
sens->J[2][26] = JTcp45_275_10;
sens->J[2][27] = JTcp45_275_11;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp45_375_4;
sens->J[3][5] = JTcp45_375_5;
sens->J[3][6] = JTcp45_375_6;
sens->J[3][19] = JTcp45_375_7;
sens->J[3][20] = JTcp45_375_8;
sens->J[3][21] = JTcp45_375_9;
sens->J[3][26] = JTcp45_375_10;
sens->J[3][27] = JTcp45_375_11;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp45_16;
sens->J[4][20] = ROcp45_419;
sens->J[4][21] = ROcp45_720;
sens->J[4][26] = ROcp45_421;
sens->J[4][27] = ROcp45_126;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp45_85;
sens->J[5][19] = ROcp45_26;
sens->J[5][20] = ROcp45_519;
sens->J[5][21] = ROcp45_820;
sens->J[5][26] = ROcp45_521;
sens->J[5][27] = ROcp45_226;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp45_95;
sens->J[6][19] = ROcp45_36;
sens->J[6][20] = ROcp45_619;
sens->J[6][21] = ROcp45_920;
sens->J[6][26] = ROcp45_621;
sens->J[6][27] = ROcp45_326;
sens->A[1] = ACcp45_175;
sens->A[2] = ACcp45_275;
sens->A[3] = ACcp45_375;
sens->OMP[1] = OPcp45_127;
sens->OMP[2] = OPcp45_227;
sens->OMP[3] = OPcp45_327;
 
// 
break;
case 47:
 


// = = Block_1_0_0_47_0_1 = = 
 
// Sensor Kinematics 


ROcp46_25 = S4*S5;
ROcp46_35 = -C4*S5;
ROcp46_85 = -S4*C5;
ROcp46_95 = C4*C5;
ROcp46_16 = C5*C6;
ROcp46_26 = ROcp46_25*C6+C4*S6;
ROcp46_36 = ROcp46_35*C6+S4*S6;
ROcp46_46 = -C5*S6;
ROcp46_56 = -(ROcp46_25*S6-C4*C6);
ROcp46_66 = -(ROcp46_35*S6-S4*C6);
OMcp46_25 = qd[5]*C4;
OMcp46_35 = qd[5]*S4;
OMcp46_16 = qd[4]+qd[6]*S5;
OMcp46_26 = OMcp46_25+ROcp46_85*qd[6];
OMcp46_36 = OMcp46_35+ROcp46_95*qd[6];
OPcp46_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp46_26 = ROcp46_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp46_35*S5-ROcp46_95*qd[4]);
OPcp46_36 = ROcp46_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp46_25*S5-ROcp46_85*qd[4]);

// = = Block_1_0_0_47_0_4 = = 
 
// Sensor Kinematics 


ROcp46_419 = ROcp46_46*C19+S19*S5;
ROcp46_519 = ROcp46_56*C19+ROcp46_85*S19;
ROcp46_619 = ROcp46_66*C19+ROcp46_95*S19;
ROcp46_719 = -(ROcp46_46*S19-C19*S5);
ROcp46_819 = -(ROcp46_56*S19-ROcp46_85*C19);
ROcp46_919 = -(ROcp46_66*S19-ROcp46_95*C19);
ROcp46_120 = ROcp46_16*C20-ROcp46_719*S20;
ROcp46_220 = ROcp46_26*C20-ROcp46_819*S20;
ROcp46_320 = ROcp46_36*C20-ROcp46_919*S20;
ROcp46_720 = ROcp46_16*S20+ROcp46_719*C20;
ROcp46_820 = ROcp46_26*S20+ROcp46_819*C20;
ROcp46_920 = ROcp46_36*S20+ROcp46_919*C20;
ROcp46_121 = ROcp46_120*C21+ROcp46_419*S21;
ROcp46_221 = ROcp46_220*C21+ROcp46_519*S21;
ROcp46_321 = ROcp46_320*C21+ROcp46_619*S21;
ROcp46_421 = -(ROcp46_120*S21-ROcp46_419*C21);
ROcp46_521 = -(ROcp46_220*S21-ROcp46_519*C21);
ROcp46_621 = -(ROcp46_320*S21-ROcp46_619*C21);
RLcp46_119 = s->dpt[2][3]*ROcp46_46+ROcp46_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp46_219 = s->dpt[2][3]*ROcp46_56+ROcp46_26*s->dpt[1][3]+ROcp46_85*s->dpt[3][3];
RLcp46_319 = s->dpt[2][3]*ROcp46_66+ROcp46_36*s->dpt[1][3]+ROcp46_95*s->dpt[3][3];
OMcp46_119 = OMcp46_16+ROcp46_16*qd[19];
OMcp46_219 = OMcp46_26+ROcp46_26*qd[19];
OMcp46_319 = OMcp46_36+ROcp46_36*qd[19];
ORcp46_119 = OMcp46_26*RLcp46_319-OMcp46_36*RLcp46_219;
ORcp46_219 = -(OMcp46_16*RLcp46_319-OMcp46_36*RLcp46_119);
ORcp46_319 = OMcp46_16*RLcp46_219-OMcp46_26*RLcp46_119;
OPcp46_119 = OPcp46_16+ROcp46_16*qdd[19]+qd[19]*(OMcp46_26*ROcp46_36-OMcp46_36*ROcp46_26);
OPcp46_219 = OPcp46_26+ROcp46_26*qdd[19]-qd[19]*(OMcp46_16*ROcp46_36-OMcp46_36*ROcp46_16);
OPcp46_319 = OPcp46_36+ROcp46_36*qdd[19]+qd[19]*(OMcp46_16*ROcp46_26-OMcp46_26*ROcp46_16);
RLcp46_120 = s->dpt[1][38]*ROcp46_16+s->dpt[2][38]*ROcp46_419+s->dpt[3][38]*ROcp46_719;
RLcp46_220 = s->dpt[1][38]*ROcp46_26+s->dpt[2][38]*ROcp46_519+s->dpt[3][38]*ROcp46_819;
RLcp46_320 = s->dpt[1][38]*ROcp46_36+s->dpt[2][38]*ROcp46_619+s->dpt[3][38]*ROcp46_919;
OMcp46_120 = OMcp46_119+ROcp46_419*qd[20];
OMcp46_220 = OMcp46_219+ROcp46_519*qd[20];
OMcp46_320 = OMcp46_319+ROcp46_619*qd[20];
ORcp46_120 = OMcp46_219*RLcp46_320-OMcp46_319*RLcp46_220;
ORcp46_220 = -(OMcp46_119*RLcp46_320-OMcp46_319*RLcp46_120);
ORcp46_320 = OMcp46_119*RLcp46_220-OMcp46_219*RLcp46_120;
OPcp46_120 = OPcp46_119+ROcp46_419*qdd[20]+qd[20]*(OMcp46_219*ROcp46_619-OMcp46_319*ROcp46_519);
OPcp46_220 = OPcp46_219+ROcp46_519*qdd[20]-qd[20]*(OMcp46_119*ROcp46_619-OMcp46_319*ROcp46_419);
OPcp46_320 = OPcp46_319+ROcp46_619*qdd[20]+qd[20]*(OMcp46_119*ROcp46_519-OMcp46_219*ROcp46_419);
RLcp46_121 = s->dpt[1][40]*ROcp46_120+s->dpt[2][40]*ROcp46_419+ROcp46_720*s->dpt[3][40];
RLcp46_221 = s->dpt[1][40]*ROcp46_220+s->dpt[2][40]*ROcp46_519+ROcp46_820*s->dpt[3][40];
RLcp46_321 = s->dpt[1][40]*ROcp46_320+s->dpt[2][40]*ROcp46_619+ROcp46_920*s->dpt[3][40];
OMcp46_121 = OMcp46_120+ROcp46_720*qd[21];
OMcp46_221 = OMcp46_220+ROcp46_820*qd[21];
OMcp46_321 = OMcp46_320+ROcp46_920*qd[21];
ORcp46_121 = OMcp46_220*RLcp46_321-OMcp46_320*RLcp46_221;
ORcp46_221 = -(OMcp46_120*RLcp46_321-OMcp46_320*RLcp46_121);
ORcp46_321 = OMcp46_120*RLcp46_221-OMcp46_220*RLcp46_121;
OPcp46_121 = OPcp46_120+ROcp46_720*qdd[21]+qd[21]*(OMcp46_220*ROcp46_920-OMcp46_320*ROcp46_820);
OPcp46_221 = OPcp46_220+ROcp46_820*qdd[21]-qd[21]*(OMcp46_120*ROcp46_920-OMcp46_320*ROcp46_720);
OPcp46_321 = OPcp46_320+ROcp46_920*qdd[21]+qd[21]*(OMcp46_120*ROcp46_820-OMcp46_220*ROcp46_720);

// = = Block_1_0_0_47_0_6 = = 
 
// Sensor Kinematics 


ROcp46_126 = ROcp46_121*C26-ROcp46_720*S26;
ROcp46_226 = ROcp46_221*C26-ROcp46_820*S26;
ROcp46_326 = ROcp46_321*C26-ROcp46_920*S26;
ROcp46_726 = ROcp46_121*S26+ROcp46_720*C26;
ROcp46_826 = ROcp46_221*S26+ROcp46_820*C26;
ROcp46_926 = ROcp46_321*S26+ROcp46_920*C26;
ROcp46_427 = ROcp46_421*C27+ROcp46_726*S27;
ROcp46_527 = ROcp46_521*C27+ROcp46_826*S27;
ROcp46_627 = ROcp46_621*C27+ROcp46_926*S27;
ROcp46_727 = -(ROcp46_421*S27-ROcp46_726*C27);
ROcp46_827 = -(ROcp46_521*S27-ROcp46_826*C27);
ROcp46_927 = -(ROcp46_621*S27-ROcp46_926*C27);
RLcp46_126 = ROcp46_121*s->dpt[1][45]+ROcp46_421*s->dpt[2][45]+ROcp46_720*s->dpt[3][45];
RLcp46_226 = ROcp46_221*s->dpt[1][45]+ROcp46_521*s->dpt[2][45]+ROcp46_820*s->dpt[3][45];
RLcp46_326 = ROcp46_321*s->dpt[1][45]+ROcp46_621*s->dpt[2][45]+ROcp46_920*s->dpt[3][45];
OMcp46_126 = OMcp46_121+ROcp46_421*qd[26];
OMcp46_226 = OMcp46_221+ROcp46_521*qd[26];
OMcp46_326 = OMcp46_321+ROcp46_621*qd[26];
ORcp46_126 = OMcp46_221*RLcp46_326-OMcp46_321*RLcp46_226;
ORcp46_226 = -(OMcp46_121*RLcp46_326-OMcp46_321*RLcp46_126);
ORcp46_326 = OMcp46_121*RLcp46_226-OMcp46_221*RLcp46_126;
OPcp46_126 = OPcp46_121+ROcp46_421*qdd[26]+qd[26]*(OMcp46_221*ROcp46_621-OMcp46_321*ROcp46_521);
OPcp46_226 = OPcp46_221+ROcp46_521*qdd[26]-qd[26]*(OMcp46_121*ROcp46_621-OMcp46_321*ROcp46_421);
OPcp46_326 = OPcp46_321+ROcp46_621*qdd[26]+qd[26]*(OMcp46_121*ROcp46_521-OMcp46_221*ROcp46_421);
RLcp46_127 = s->dpt[1][55]*ROcp46_126+s->dpt[3][55]*ROcp46_726+ROcp46_421*s->dpt[2][55];
RLcp46_227 = s->dpt[1][55]*ROcp46_226+s->dpt[3][55]*ROcp46_826+ROcp46_521*s->dpt[2][55];
RLcp46_327 = s->dpt[1][55]*ROcp46_326+s->dpt[3][55]*ROcp46_926+ROcp46_621*s->dpt[2][55];
OMcp46_127 = OMcp46_126+ROcp46_126*qd[27];
OMcp46_227 = OMcp46_226+ROcp46_226*qd[27];
OMcp46_327 = OMcp46_326+ROcp46_326*qd[27];
ORcp46_127 = OMcp46_226*RLcp46_327-OMcp46_326*RLcp46_227;
ORcp46_227 = -(OMcp46_126*RLcp46_327-OMcp46_326*RLcp46_127);
ORcp46_327 = OMcp46_126*RLcp46_227-OMcp46_226*RLcp46_127;
OPcp46_127 = OPcp46_126+ROcp46_126*qdd[27]+qd[27]*(OMcp46_226*ROcp46_326-OMcp46_326*ROcp46_226);
OPcp46_227 = OPcp46_226+ROcp46_226*qdd[27]-qd[27]*(OMcp46_126*ROcp46_326-OMcp46_326*ROcp46_126);
OPcp46_327 = OPcp46_326+ROcp46_326*qdd[27]+qd[27]*(OMcp46_126*ROcp46_226-OMcp46_226*ROcp46_126);
RLcp46_176 = s->dpt[1][57]*ROcp46_126+s->dpt[2][57]*ROcp46_427+ROcp46_727*s->dpt[3][57];
RLcp46_276 = s->dpt[1][57]*ROcp46_226+s->dpt[2][57]*ROcp46_527+ROcp46_827*s->dpt[3][57];
RLcp46_376 = s->dpt[1][57]*ROcp46_326+s->dpt[2][57]*ROcp46_627+ROcp46_927*s->dpt[3][57];
POcp46_176 = RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_126+RLcp46_127+RLcp46_176+q[1];
POcp46_276 = RLcp46_219+RLcp46_220+RLcp46_221+RLcp46_226+RLcp46_227+RLcp46_276+q[2];
POcp46_376 = RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_326+RLcp46_327+RLcp46_376+q[3];
JTcp46_276_4 = -(RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_326+RLcp46_327+RLcp46_376);
JTcp46_376_4 = RLcp46_219+RLcp46_220+RLcp46_221+RLcp46_226+RLcp46_227+RLcp46_276;
JTcp46_176_5 = C4*(RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_326+RLcp46_327+RLcp46_376)-S4*(RLcp46_219+RLcp46_220)-S4*(
 RLcp46_221+RLcp46_226)-S4*(RLcp46_227+RLcp46_276);
JTcp46_276_5 = S4*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_126+RLcp46_127+RLcp46_176);
JTcp46_376_5 = -C4*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_126+RLcp46_127+RLcp46_176);
JTcp46_176_6 = ROcp46_85*(RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_326+RLcp46_327+RLcp46_376)-ROcp46_95*(RLcp46_219+
 RLcp46_220)-ROcp46_95*(RLcp46_221+RLcp46_226)-ROcp46_95*(RLcp46_227+RLcp46_276);
JTcp46_276_6 = RLcp46_176*ROcp46_95-RLcp46_327*S5-RLcp46_376*S5+ROcp46_95*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_126
 +RLcp46_127)-S5*(RLcp46_319+RLcp46_320)-S5*(RLcp46_321+RLcp46_326);
JTcp46_376_6 = RLcp46_227*S5-ROcp46_85*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_126+RLcp46_127)+S5*(RLcp46_219+
 RLcp46_220)+S5*(RLcp46_221+RLcp46_226)-RLcp46_176*ROcp46_85+RLcp46_276*S5;
JTcp46_176_7 = ROcp46_26*(RLcp46_320+RLcp46_321+RLcp46_326+RLcp46_327)-ROcp46_36*(RLcp46_220+RLcp46_221)-ROcp46_36*(
 RLcp46_226+RLcp46_227)-RLcp46_276*ROcp46_36+RLcp46_376*ROcp46_26;
JTcp46_276_7 = RLcp46_176*ROcp46_36-RLcp46_376*ROcp46_16-ROcp46_16*(RLcp46_320+RLcp46_321+RLcp46_326+RLcp46_327)+
 ROcp46_36*(RLcp46_120+RLcp46_121)+ROcp46_36*(RLcp46_126+RLcp46_127);
JTcp46_376_7 = ROcp46_16*(RLcp46_220+RLcp46_221+RLcp46_226+RLcp46_227)-ROcp46_26*(RLcp46_120+RLcp46_121)-ROcp46_26*(
 RLcp46_126+RLcp46_127)-RLcp46_176*ROcp46_26+RLcp46_276*ROcp46_16;
JTcp46_176_8 = ROcp46_519*(RLcp46_321+RLcp46_326+RLcp46_327+RLcp46_376)-ROcp46_619*(RLcp46_221+RLcp46_226)-ROcp46_619*
 (RLcp46_227+RLcp46_276);
JTcp46_276_8 = -(ROcp46_419*(RLcp46_321+RLcp46_326+RLcp46_327+RLcp46_376)-ROcp46_619*(RLcp46_121+RLcp46_126)-
 ROcp46_619*(RLcp46_127+RLcp46_176));
JTcp46_376_8 = ROcp46_419*(RLcp46_221+RLcp46_226+RLcp46_227+RLcp46_276)-ROcp46_519*(RLcp46_121+RLcp46_126)-ROcp46_519*
 (RLcp46_127+RLcp46_176);
JTcp46_176_9 = ROcp46_820*(RLcp46_326+RLcp46_327)-ROcp46_920*(RLcp46_226+RLcp46_227)-RLcp46_276*ROcp46_920+RLcp46_376*
 ROcp46_820;
JTcp46_276_9 = RLcp46_176*ROcp46_920-RLcp46_376*ROcp46_720-ROcp46_720*(RLcp46_326+RLcp46_327)+ROcp46_920*(RLcp46_126+
 RLcp46_127);
JTcp46_376_9 = ROcp46_720*(RLcp46_226+RLcp46_227)-ROcp46_820*(RLcp46_126+RLcp46_127)-RLcp46_176*ROcp46_820+RLcp46_276*
 ROcp46_720;
JTcp46_176_10 = ROcp46_521*(RLcp46_327+RLcp46_376)-ROcp46_621*(RLcp46_227+RLcp46_276);
JTcp46_276_10 = -(ROcp46_421*(RLcp46_327+RLcp46_376)-ROcp46_621*(RLcp46_127+RLcp46_176));
JTcp46_376_10 = ROcp46_421*(RLcp46_227+RLcp46_276)-ROcp46_521*(RLcp46_127+RLcp46_176);
JTcp46_176_11 = -(RLcp46_276*ROcp46_326-RLcp46_376*ROcp46_226);
JTcp46_276_11 = RLcp46_176*ROcp46_326-RLcp46_376*ROcp46_126;
JTcp46_376_11 = -(RLcp46_176*ROcp46_226-RLcp46_276*ROcp46_126);
ORcp46_176 = OMcp46_227*RLcp46_376-OMcp46_327*RLcp46_276;
ORcp46_276 = -(OMcp46_127*RLcp46_376-OMcp46_327*RLcp46_176);
ORcp46_376 = OMcp46_127*RLcp46_276-OMcp46_227*RLcp46_176;
VIcp46_176 = ORcp46_119+ORcp46_120+ORcp46_121+ORcp46_126+ORcp46_127+ORcp46_176+qd[1];
VIcp46_276 = ORcp46_219+ORcp46_220+ORcp46_221+ORcp46_226+ORcp46_227+ORcp46_276+qd[2];
VIcp46_376 = ORcp46_319+ORcp46_320+ORcp46_321+ORcp46_326+ORcp46_327+ORcp46_376+qd[3];
ACcp46_176 = qdd[1]+OMcp46_219*ORcp46_320+OMcp46_220*ORcp46_321+OMcp46_221*ORcp46_326+OMcp46_226*ORcp46_327+OMcp46_227
 *ORcp46_376+OMcp46_26*ORcp46_319-OMcp46_319*ORcp46_220-OMcp46_320*ORcp46_221-OMcp46_321*ORcp46_226-OMcp46_326*ORcp46_227-
 OMcp46_327*ORcp46_276-OMcp46_36*ORcp46_219+OPcp46_219*RLcp46_320+OPcp46_220*RLcp46_321+OPcp46_221*RLcp46_326+OPcp46_226*
 RLcp46_327+OPcp46_227*RLcp46_376+OPcp46_26*RLcp46_319-OPcp46_319*RLcp46_220-OPcp46_320*RLcp46_221-OPcp46_321*RLcp46_226-
 OPcp46_326*RLcp46_227-OPcp46_327*RLcp46_276-OPcp46_36*RLcp46_219;
ACcp46_276 = qdd[2]-OMcp46_119*ORcp46_320-OMcp46_120*ORcp46_321-OMcp46_121*ORcp46_326-OMcp46_126*ORcp46_327-OMcp46_127
 *ORcp46_376-OMcp46_16*ORcp46_319+OMcp46_319*ORcp46_120+OMcp46_320*ORcp46_121+OMcp46_321*ORcp46_126+OMcp46_326*ORcp46_127+
 OMcp46_327*ORcp46_176+OMcp46_36*ORcp46_119-OPcp46_119*RLcp46_320-OPcp46_120*RLcp46_321-OPcp46_121*RLcp46_326-OPcp46_126*
 RLcp46_327-OPcp46_127*RLcp46_376-OPcp46_16*RLcp46_319+OPcp46_319*RLcp46_120+OPcp46_320*RLcp46_121+OPcp46_321*RLcp46_126+
 OPcp46_326*RLcp46_127+OPcp46_327*RLcp46_176+OPcp46_36*RLcp46_119;
ACcp46_376 = qdd[3]+OMcp46_119*ORcp46_220+OMcp46_120*ORcp46_221+OMcp46_121*ORcp46_226+OMcp46_126*ORcp46_227+OMcp46_127
 *ORcp46_276+OMcp46_16*ORcp46_219-OMcp46_219*ORcp46_120-OMcp46_220*ORcp46_121-OMcp46_221*ORcp46_126-OMcp46_226*ORcp46_127-
 OMcp46_227*ORcp46_176-OMcp46_26*ORcp46_119+OPcp46_119*RLcp46_220+OPcp46_120*RLcp46_221+OPcp46_121*RLcp46_226+OPcp46_126*
 RLcp46_227+OPcp46_127*RLcp46_276+OPcp46_16*RLcp46_219-OPcp46_219*RLcp46_120-OPcp46_220*RLcp46_121-OPcp46_221*RLcp46_126-
 OPcp46_226*RLcp46_127-OPcp46_227*RLcp46_176-OPcp46_26*RLcp46_119;

// = = Block_1_0_0_47_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp46_176;
sens->P[2] = POcp46_276;
sens->P[3] = POcp46_376;
sens->R[1][1] = ROcp46_126;
sens->R[1][2] = ROcp46_226;
sens->R[1][3] = ROcp46_326;
sens->R[2][1] = ROcp46_427;
sens->R[2][2] = ROcp46_527;
sens->R[2][3] = ROcp46_627;
sens->R[3][1] = ROcp46_727;
sens->R[3][2] = ROcp46_827;
sens->R[3][3] = ROcp46_927;
sens->V[1] = VIcp46_176;
sens->V[2] = VIcp46_276;
sens->V[3] = VIcp46_376;
sens->OM[1] = OMcp46_127;
sens->OM[2] = OMcp46_227;
sens->OM[3] = OMcp46_327;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp46_176_5;
sens->J[1][6] = JTcp46_176_6;
sens->J[1][19] = JTcp46_176_7;
sens->J[1][20] = JTcp46_176_8;
sens->J[1][21] = JTcp46_176_9;
sens->J[1][26] = JTcp46_176_10;
sens->J[1][27] = JTcp46_176_11;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp46_276_4;
sens->J[2][5] = JTcp46_276_5;
sens->J[2][6] = JTcp46_276_6;
sens->J[2][19] = JTcp46_276_7;
sens->J[2][20] = JTcp46_276_8;
sens->J[2][21] = JTcp46_276_9;
sens->J[2][26] = JTcp46_276_10;
sens->J[2][27] = JTcp46_276_11;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp46_376_4;
sens->J[3][5] = JTcp46_376_5;
sens->J[3][6] = JTcp46_376_6;
sens->J[3][19] = JTcp46_376_7;
sens->J[3][20] = JTcp46_376_8;
sens->J[3][21] = JTcp46_376_9;
sens->J[3][26] = JTcp46_376_10;
sens->J[3][27] = JTcp46_376_11;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp46_16;
sens->J[4][20] = ROcp46_419;
sens->J[4][21] = ROcp46_720;
sens->J[4][26] = ROcp46_421;
sens->J[4][27] = ROcp46_126;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp46_85;
sens->J[5][19] = ROcp46_26;
sens->J[5][20] = ROcp46_519;
sens->J[5][21] = ROcp46_820;
sens->J[5][26] = ROcp46_521;
sens->J[5][27] = ROcp46_226;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp46_95;
sens->J[6][19] = ROcp46_36;
sens->J[6][20] = ROcp46_619;
sens->J[6][21] = ROcp46_920;
sens->J[6][26] = ROcp46_621;
sens->J[6][27] = ROcp46_326;
sens->A[1] = ACcp46_176;
sens->A[2] = ACcp46_276;
sens->A[3] = ACcp46_376;
sens->OMP[1] = OPcp46_127;
sens->OMP[2] = OPcp46_227;
sens->OMP[3] = OPcp46_327;
 
// 
break;
case 48:
 


// = = Block_1_0_0_48_0_1 = = 
 
// Sensor Kinematics 


ROcp47_25 = S4*S5;
ROcp47_35 = -C4*S5;
ROcp47_85 = -S4*C5;
ROcp47_95 = C4*C5;
ROcp47_16 = C5*C6;
ROcp47_26 = ROcp47_25*C6+C4*S6;
ROcp47_36 = ROcp47_35*C6+S4*S6;
ROcp47_46 = -C5*S6;
ROcp47_56 = -(ROcp47_25*S6-C4*C6);
ROcp47_66 = -(ROcp47_35*S6-S4*C6);
OMcp47_25 = qd[5]*C4;
OMcp47_35 = qd[5]*S4;
OMcp47_16 = qd[4]+qd[6]*S5;
OMcp47_26 = OMcp47_25+ROcp47_85*qd[6];
OMcp47_36 = OMcp47_35+ROcp47_95*qd[6];
OPcp47_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp47_26 = ROcp47_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp47_35*S5-ROcp47_95*qd[4]);
OPcp47_36 = ROcp47_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp47_25*S5-ROcp47_85*qd[4]);

// = = Block_1_0_0_48_0_4 = = 
 
// Sensor Kinematics 


ROcp47_419 = ROcp47_46*C19+S19*S5;
ROcp47_519 = ROcp47_56*C19+ROcp47_85*S19;
ROcp47_619 = ROcp47_66*C19+ROcp47_95*S19;
ROcp47_719 = -(ROcp47_46*S19-C19*S5);
ROcp47_819 = -(ROcp47_56*S19-ROcp47_85*C19);
ROcp47_919 = -(ROcp47_66*S19-ROcp47_95*C19);
ROcp47_120 = ROcp47_16*C20-ROcp47_719*S20;
ROcp47_220 = ROcp47_26*C20-ROcp47_819*S20;
ROcp47_320 = ROcp47_36*C20-ROcp47_919*S20;
ROcp47_720 = ROcp47_16*S20+ROcp47_719*C20;
ROcp47_820 = ROcp47_26*S20+ROcp47_819*C20;
ROcp47_920 = ROcp47_36*S20+ROcp47_919*C20;
ROcp47_121 = ROcp47_120*C21+ROcp47_419*S21;
ROcp47_221 = ROcp47_220*C21+ROcp47_519*S21;
ROcp47_321 = ROcp47_320*C21+ROcp47_619*S21;
ROcp47_421 = -(ROcp47_120*S21-ROcp47_419*C21);
ROcp47_521 = -(ROcp47_220*S21-ROcp47_519*C21);
ROcp47_621 = -(ROcp47_320*S21-ROcp47_619*C21);
RLcp47_119 = s->dpt[2][3]*ROcp47_46+ROcp47_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp47_219 = s->dpt[2][3]*ROcp47_56+ROcp47_26*s->dpt[1][3]+ROcp47_85*s->dpt[3][3];
RLcp47_319 = s->dpt[2][3]*ROcp47_66+ROcp47_36*s->dpt[1][3]+ROcp47_95*s->dpt[3][3];
OMcp47_119 = OMcp47_16+ROcp47_16*qd[19];
OMcp47_219 = OMcp47_26+ROcp47_26*qd[19];
OMcp47_319 = OMcp47_36+ROcp47_36*qd[19];
ORcp47_119 = OMcp47_26*RLcp47_319-OMcp47_36*RLcp47_219;
ORcp47_219 = -(OMcp47_16*RLcp47_319-OMcp47_36*RLcp47_119);
ORcp47_319 = OMcp47_16*RLcp47_219-OMcp47_26*RLcp47_119;
OPcp47_119 = OPcp47_16+ROcp47_16*qdd[19]+qd[19]*(OMcp47_26*ROcp47_36-OMcp47_36*ROcp47_26);
OPcp47_219 = OPcp47_26+ROcp47_26*qdd[19]-qd[19]*(OMcp47_16*ROcp47_36-OMcp47_36*ROcp47_16);
OPcp47_319 = OPcp47_36+ROcp47_36*qdd[19]+qd[19]*(OMcp47_16*ROcp47_26-OMcp47_26*ROcp47_16);
RLcp47_120 = s->dpt[1][38]*ROcp47_16+s->dpt[2][38]*ROcp47_419+s->dpt[3][38]*ROcp47_719;
RLcp47_220 = s->dpt[1][38]*ROcp47_26+s->dpt[2][38]*ROcp47_519+s->dpt[3][38]*ROcp47_819;
RLcp47_320 = s->dpt[1][38]*ROcp47_36+s->dpt[2][38]*ROcp47_619+s->dpt[3][38]*ROcp47_919;
OMcp47_120 = OMcp47_119+ROcp47_419*qd[20];
OMcp47_220 = OMcp47_219+ROcp47_519*qd[20];
OMcp47_320 = OMcp47_319+ROcp47_619*qd[20];
ORcp47_120 = OMcp47_219*RLcp47_320-OMcp47_319*RLcp47_220;
ORcp47_220 = -(OMcp47_119*RLcp47_320-OMcp47_319*RLcp47_120);
ORcp47_320 = OMcp47_119*RLcp47_220-OMcp47_219*RLcp47_120;
OPcp47_120 = OPcp47_119+ROcp47_419*qdd[20]+qd[20]*(OMcp47_219*ROcp47_619-OMcp47_319*ROcp47_519);
OPcp47_220 = OPcp47_219+ROcp47_519*qdd[20]-qd[20]*(OMcp47_119*ROcp47_619-OMcp47_319*ROcp47_419);
OPcp47_320 = OPcp47_319+ROcp47_619*qdd[20]+qd[20]*(OMcp47_119*ROcp47_519-OMcp47_219*ROcp47_419);
RLcp47_121 = s->dpt[1][40]*ROcp47_120+s->dpt[2][40]*ROcp47_419+ROcp47_720*s->dpt[3][40];
RLcp47_221 = s->dpt[1][40]*ROcp47_220+s->dpt[2][40]*ROcp47_519+ROcp47_820*s->dpt[3][40];
RLcp47_321 = s->dpt[1][40]*ROcp47_320+s->dpt[2][40]*ROcp47_619+ROcp47_920*s->dpt[3][40];
OMcp47_121 = OMcp47_120+ROcp47_720*qd[21];
OMcp47_221 = OMcp47_220+ROcp47_820*qd[21];
OMcp47_321 = OMcp47_320+ROcp47_920*qd[21];
ORcp47_121 = OMcp47_220*RLcp47_321-OMcp47_320*RLcp47_221;
ORcp47_221 = -(OMcp47_120*RLcp47_321-OMcp47_320*RLcp47_121);
ORcp47_321 = OMcp47_120*RLcp47_221-OMcp47_220*RLcp47_121;
OPcp47_121 = OPcp47_120+ROcp47_720*qdd[21]+qd[21]*(OMcp47_220*ROcp47_920-OMcp47_320*ROcp47_820);
OPcp47_221 = OPcp47_220+ROcp47_820*qdd[21]-qd[21]*(OMcp47_120*ROcp47_920-OMcp47_320*ROcp47_720);
OPcp47_321 = OPcp47_320+ROcp47_920*qdd[21]+qd[21]*(OMcp47_120*ROcp47_820-OMcp47_220*ROcp47_720);

// = = Block_1_0_0_48_0_6 = = 
 
// Sensor Kinematics 


ROcp47_126 = ROcp47_121*C26-ROcp47_720*S26;
ROcp47_226 = ROcp47_221*C26-ROcp47_820*S26;
ROcp47_326 = ROcp47_321*C26-ROcp47_920*S26;
ROcp47_726 = ROcp47_121*S26+ROcp47_720*C26;
ROcp47_826 = ROcp47_221*S26+ROcp47_820*C26;
ROcp47_926 = ROcp47_321*S26+ROcp47_920*C26;
ROcp47_427 = ROcp47_421*C27+ROcp47_726*S27;
ROcp47_527 = ROcp47_521*C27+ROcp47_826*S27;
ROcp47_627 = ROcp47_621*C27+ROcp47_926*S27;
ROcp47_727 = -(ROcp47_421*S27-ROcp47_726*C27);
ROcp47_827 = -(ROcp47_521*S27-ROcp47_826*C27);
ROcp47_927 = -(ROcp47_621*S27-ROcp47_926*C27);
ROcp47_128 = ROcp47_126*C28+ROcp47_427*S28;
ROcp47_228 = ROcp47_226*C28+ROcp47_527*S28;
ROcp47_328 = ROcp47_326*C28+ROcp47_627*S28;
ROcp47_428 = -(ROcp47_126*S28-ROcp47_427*C28);
ROcp47_528 = -(ROcp47_226*S28-ROcp47_527*C28);
ROcp47_628 = -(ROcp47_326*S28-ROcp47_627*C28);
RLcp47_126 = ROcp47_121*s->dpt[1][45]+ROcp47_421*s->dpt[2][45]+ROcp47_720*s->dpt[3][45];
RLcp47_226 = ROcp47_221*s->dpt[1][45]+ROcp47_521*s->dpt[2][45]+ROcp47_820*s->dpt[3][45];
RLcp47_326 = ROcp47_321*s->dpt[1][45]+ROcp47_621*s->dpt[2][45]+ROcp47_920*s->dpt[3][45];
OMcp47_126 = OMcp47_121+ROcp47_421*qd[26];
OMcp47_226 = OMcp47_221+ROcp47_521*qd[26];
OMcp47_326 = OMcp47_321+ROcp47_621*qd[26];
ORcp47_126 = OMcp47_221*RLcp47_326-OMcp47_321*RLcp47_226;
ORcp47_226 = -(OMcp47_121*RLcp47_326-OMcp47_321*RLcp47_126);
ORcp47_326 = OMcp47_121*RLcp47_226-OMcp47_221*RLcp47_126;
OPcp47_126 = OPcp47_121+ROcp47_421*qdd[26]+qd[26]*(OMcp47_221*ROcp47_621-OMcp47_321*ROcp47_521);
OPcp47_226 = OPcp47_221+ROcp47_521*qdd[26]-qd[26]*(OMcp47_121*ROcp47_621-OMcp47_321*ROcp47_421);
OPcp47_326 = OPcp47_321+ROcp47_621*qdd[26]+qd[26]*(OMcp47_121*ROcp47_521-OMcp47_221*ROcp47_421);
RLcp47_127 = s->dpt[1][55]*ROcp47_126+s->dpt[3][55]*ROcp47_726+ROcp47_421*s->dpt[2][55];
RLcp47_227 = s->dpt[1][55]*ROcp47_226+s->dpt[3][55]*ROcp47_826+ROcp47_521*s->dpt[2][55];
RLcp47_327 = s->dpt[1][55]*ROcp47_326+s->dpt[3][55]*ROcp47_926+ROcp47_621*s->dpt[2][55];
OMcp47_127 = OMcp47_126+ROcp47_126*qd[27];
OMcp47_227 = OMcp47_226+ROcp47_226*qd[27];
OMcp47_327 = OMcp47_326+ROcp47_326*qd[27];
ORcp47_127 = OMcp47_226*RLcp47_327-OMcp47_326*RLcp47_227;
ORcp47_227 = -(OMcp47_126*RLcp47_327-OMcp47_326*RLcp47_127);
ORcp47_327 = OMcp47_126*RLcp47_227-OMcp47_226*RLcp47_127;
OPcp47_127 = OPcp47_126+ROcp47_126*qdd[27]+qd[27]*(OMcp47_226*ROcp47_326-OMcp47_326*ROcp47_226);
OPcp47_227 = OPcp47_226+ROcp47_226*qdd[27]-qd[27]*(OMcp47_126*ROcp47_326-OMcp47_326*ROcp47_126);
OPcp47_327 = OPcp47_326+ROcp47_326*qdd[27]+qd[27]*(OMcp47_126*ROcp47_226-OMcp47_226*ROcp47_126);
RLcp47_128 = s->dpt[1][57]*ROcp47_126+s->dpt[2][57]*ROcp47_427+ROcp47_727*s->dpt[3][57];
RLcp47_228 = s->dpt[1][57]*ROcp47_226+s->dpt[2][57]*ROcp47_527+ROcp47_827*s->dpt[3][57];
RLcp47_328 = s->dpt[1][57]*ROcp47_326+s->dpt[2][57]*ROcp47_627+ROcp47_927*s->dpt[3][57];
OMcp47_128 = OMcp47_127+ROcp47_727*qd[28];
OMcp47_228 = OMcp47_227+ROcp47_827*qd[28];
OMcp47_328 = OMcp47_327+ROcp47_927*qd[28];
ORcp47_128 = OMcp47_227*RLcp47_328-OMcp47_327*RLcp47_228;
ORcp47_228 = -(OMcp47_127*RLcp47_328-OMcp47_327*RLcp47_128);
ORcp47_328 = OMcp47_127*RLcp47_228-OMcp47_227*RLcp47_128;
OPcp47_128 = OPcp47_127+ROcp47_727*qdd[28]+qd[28]*(OMcp47_227*ROcp47_927-OMcp47_327*ROcp47_827);
OPcp47_228 = OPcp47_227+ROcp47_827*qdd[28]-qd[28]*(OMcp47_127*ROcp47_927-OMcp47_327*ROcp47_727);
OPcp47_328 = OPcp47_327+ROcp47_927*qdd[28]+qd[28]*(OMcp47_127*ROcp47_827-OMcp47_227*ROcp47_727);
RLcp47_177 = ROcp47_128*s->dpt[1][58]+ROcp47_428*s->dpt[2][58]+ROcp47_727*s->dpt[3][58];
RLcp47_277 = ROcp47_228*s->dpt[1][58]+ROcp47_528*s->dpt[2][58]+ROcp47_827*s->dpt[3][58];
RLcp47_377 = ROcp47_328*s->dpt[1][58]+ROcp47_628*s->dpt[2][58]+ROcp47_927*s->dpt[3][58];
POcp47_177 = RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_126+RLcp47_127+RLcp47_128+RLcp47_177+q[1];
POcp47_277 = RLcp47_219+RLcp47_220+RLcp47_221+RLcp47_226+RLcp47_227+RLcp47_228+RLcp47_277+q[2];
POcp47_377 = RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328+RLcp47_377+q[3];
JTcp47_277_4 = -(RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328+RLcp47_377);
JTcp47_377_4 = RLcp47_219+RLcp47_220+RLcp47_221+RLcp47_226+RLcp47_227+RLcp47_228+RLcp47_277;
JTcp47_177_5 = C4*(RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328)-S4*(RLcp47_219+RLcp47_220)-S4*(
 RLcp47_221+RLcp47_226)-S4*(RLcp47_227+RLcp47_228)-RLcp47_277*S4+RLcp47_377*C4;
JTcp47_277_5 = S4*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_126+RLcp47_127+RLcp47_128+RLcp47_177);
JTcp47_377_5 = -C4*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_126+RLcp47_127+RLcp47_128+RLcp47_177);
JTcp47_177_6 = ROcp47_85*(RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328)-ROcp47_95*(RLcp47_219+
 RLcp47_220)-ROcp47_95*(RLcp47_221+RLcp47_226)-ROcp47_95*(RLcp47_227+RLcp47_228)-RLcp47_277*ROcp47_95+RLcp47_377*ROcp47_85;
JTcp47_277_6 = -(RLcp47_377*S5-ROcp47_95*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_126+RLcp47_127+RLcp47_128+RLcp47_177
 )+S5*(RLcp47_319+RLcp47_320)+S5*(RLcp47_321+RLcp47_326)+S5*(RLcp47_327+RLcp47_328));
JTcp47_377_6 = RLcp47_277*S5-ROcp47_85*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_126+RLcp47_127+RLcp47_128+RLcp47_177)+
 S5*(RLcp47_219+RLcp47_220)+S5*(RLcp47_221+RLcp47_226)+S5*(RLcp47_227+RLcp47_228);
JTcp47_177_7 = ROcp47_26*(RLcp47_320+RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328+RLcp47_377)-ROcp47_36*(RLcp47_220+
 RLcp47_221)-ROcp47_36*(RLcp47_226+RLcp47_227)-ROcp47_36*(RLcp47_228+RLcp47_277);
JTcp47_277_7 = -(ROcp47_16*(RLcp47_320+RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328+RLcp47_377)-ROcp47_36*(RLcp47_120+
 RLcp47_121)-ROcp47_36*(RLcp47_126+RLcp47_127)-ROcp47_36*(RLcp47_128+RLcp47_177));
JTcp47_377_7 = ROcp47_16*(RLcp47_220+RLcp47_221+RLcp47_226+RLcp47_227+RLcp47_228+RLcp47_277)-ROcp47_26*(RLcp47_120+
 RLcp47_121)-ROcp47_26*(RLcp47_126+RLcp47_127)-ROcp47_26*(RLcp47_128+RLcp47_177);
JTcp47_177_8 = ROcp47_519*(RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328)-ROcp47_619*(RLcp47_221+RLcp47_226)-ROcp47_619*
 (RLcp47_227+RLcp47_228)-RLcp47_277*ROcp47_619+RLcp47_377*ROcp47_519;
JTcp47_277_8 = RLcp47_177*ROcp47_619-RLcp47_377*ROcp47_419-ROcp47_419*(RLcp47_321+RLcp47_326+RLcp47_327+RLcp47_328)+
 ROcp47_619*(RLcp47_121+RLcp47_126)+ROcp47_619*(RLcp47_127+RLcp47_128);
JTcp47_377_8 = ROcp47_419*(RLcp47_221+RLcp47_226+RLcp47_227+RLcp47_228)-ROcp47_519*(RLcp47_121+RLcp47_126)-ROcp47_519*
 (RLcp47_127+RLcp47_128)-RLcp47_177*ROcp47_519+RLcp47_277*ROcp47_419;
JTcp47_177_9 = ROcp47_820*(RLcp47_326+RLcp47_327+RLcp47_328+RLcp47_377)-ROcp47_920*(RLcp47_226+RLcp47_227)-ROcp47_920*
 (RLcp47_228+RLcp47_277);
JTcp47_277_9 = -(ROcp47_720*(RLcp47_326+RLcp47_327+RLcp47_328+RLcp47_377)-ROcp47_920*(RLcp47_126+RLcp47_127)-
 ROcp47_920*(RLcp47_128+RLcp47_177));
JTcp47_377_9 = ROcp47_720*(RLcp47_226+RLcp47_227+RLcp47_228+RLcp47_277)-ROcp47_820*(RLcp47_126+RLcp47_127)-ROcp47_820*
 (RLcp47_128+RLcp47_177);
JTcp47_177_10 = ROcp47_521*(RLcp47_327+RLcp47_328)-ROcp47_621*(RLcp47_227+RLcp47_228)-RLcp47_277*ROcp47_621+RLcp47_377
 *ROcp47_521;
JTcp47_277_10 = RLcp47_177*ROcp47_621-RLcp47_377*ROcp47_421-ROcp47_421*(RLcp47_327+RLcp47_328)+ROcp47_621*(RLcp47_127+
 RLcp47_128);
JTcp47_377_10 = ROcp47_421*(RLcp47_227+RLcp47_228)-ROcp47_521*(RLcp47_127+RLcp47_128)-RLcp47_177*ROcp47_521+RLcp47_277
 *ROcp47_421;
JTcp47_177_11 = ROcp47_226*(RLcp47_328+RLcp47_377)-ROcp47_326*(RLcp47_228+RLcp47_277);
JTcp47_277_11 = -(ROcp47_126*(RLcp47_328+RLcp47_377)-ROcp47_326*(RLcp47_128+RLcp47_177));
JTcp47_377_11 = ROcp47_126*(RLcp47_228+RLcp47_277)-ROcp47_226*(RLcp47_128+RLcp47_177);
JTcp47_177_12 = -(RLcp47_277*ROcp47_927-RLcp47_377*ROcp47_827);
JTcp47_277_12 = RLcp47_177*ROcp47_927-RLcp47_377*ROcp47_727;
JTcp47_377_12 = -(RLcp47_177*ROcp47_827-RLcp47_277*ROcp47_727);
ORcp47_177 = OMcp47_228*RLcp47_377-OMcp47_328*RLcp47_277;
ORcp47_277 = -(OMcp47_128*RLcp47_377-OMcp47_328*RLcp47_177);
ORcp47_377 = OMcp47_128*RLcp47_277-OMcp47_228*RLcp47_177;
VIcp47_177 = ORcp47_119+ORcp47_120+ORcp47_121+ORcp47_126+ORcp47_127+ORcp47_128+ORcp47_177+qd[1];
VIcp47_277 = ORcp47_219+ORcp47_220+ORcp47_221+ORcp47_226+ORcp47_227+ORcp47_228+ORcp47_277+qd[2];
VIcp47_377 = ORcp47_319+ORcp47_320+ORcp47_321+ORcp47_326+ORcp47_327+ORcp47_328+ORcp47_377+qd[3];
ACcp47_177 = qdd[1]+OMcp47_219*ORcp47_320+OMcp47_220*ORcp47_321+OMcp47_221*ORcp47_326+OMcp47_226*ORcp47_327+OMcp47_227
 *ORcp47_328+OMcp47_228*ORcp47_377+OMcp47_26*ORcp47_319-OMcp47_319*ORcp47_220-OMcp47_320*ORcp47_221-OMcp47_321*ORcp47_226-
 OMcp47_326*ORcp47_227-OMcp47_327*ORcp47_228-OMcp47_328*ORcp47_277-OMcp47_36*ORcp47_219+OPcp47_219*RLcp47_320+OPcp47_220*
 RLcp47_321+OPcp47_221*RLcp47_326+OPcp47_226*RLcp47_327+OPcp47_227*RLcp47_328+OPcp47_228*RLcp47_377+OPcp47_26*RLcp47_319-
 OPcp47_319*RLcp47_220-OPcp47_320*RLcp47_221-OPcp47_321*RLcp47_226-OPcp47_326*RLcp47_227-OPcp47_327*RLcp47_228-OPcp47_328*
 RLcp47_277-OPcp47_36*RLcp47_219;
ACcp47_277 = qdd[2]-OMcp47_119*ORcp47_320-OMcp47_120*ORcp47_321-OMcp47_121*ORcp47_326-OMcp47_126*ORcp47_327-OMcp47_127
 *ORcp47_328-OMcp47_128*ORcp47_377-OMcp47_16*ORcp47_319+OMcp47_319*ORcp47_120+OMcp47_320*ORcp47_121+OMcp47_321*ORcp47_126+
 OMcp47_326*ORcp47_127+OMcp47_327*ORcp47_128+OMcp47_328*ORcp47_177+OMcp47_36*ORcp47_119-OPcp47_119*RLcp47_320-OPcp47_120*
 RLcp47_321-OPcp47_121*RLcp47_326-OPcp47_126*RLcp47_327-OPcp47_127*RLcp47_328-OPcp47_128*RLcp47_377-OPcp47_16*RLcp47_319+
 OPcp47_319*RLcp47_120+OPcp47_320*RLcp47_121+OPcp47_321*RLcp47_126+OPcp47_326*RLcp47_127+OPcp47_327*RLcp47_128+OPcp47_328*
 RLcp47_177+OPcp47_36*RLcp47_119;
ACcp47_377 = qdd[3]+OMcp47_119*ORcp47_220+OMcp47_120*ORcp47_221+OMcp47_121*ORcp47_226+OMcp47_126*ORcp47_227+OMcp47_127
 *ORcp47_228+OMcp47_128*ORcp47_277+OMcp47_16*ORcp47_219-OMcp47_219*ORcp47_120-OMcp47_220*ORcp47_121-OMcp47_221*ORcp47_126-
 OMcp47_226*ORcp47_127-OMcp47_227*ORcp47_128-OMcp47_228*ORcp47_177-OMcp47_26*ORcp47_119+OPcp47_119*RLcp47_220+OPcp47_120*
 RLcp47_221+OPcp47_121*RLcp47_226+OPcp47_126*RLcp47_227+OPcp47_127*RLcp47_228+OPcp47_128*RLcp47_277+OPcp47_16*RLcp47_219-
 OPcp47_219*RLcp47_120-OPcp47_220*RLcp47_121-OPcp47_221*RLcp47_126-OPcp47_226*RLcp47_127-OPcp47_227*RLcp47_128-OPcp47_228*
 RLcp47_177-OPcp47_26*RLcp47_119;

// = = Block_1_0_0_48_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp47_177;
sens->P[2] = POcp47_277;
sens->P[3] = POcp47_377;
sens->R[1][1] = ROcp47_128;
sens->R[1][2] = ROcp47_228;
sens->R[1][3] = ROcp47_328;
sens->R[2][1] = ROcp47_428;
sens->R[2][2] = ROcp47_528;
sens->R[2][3] = ROcp47_628;
sens->R[3][1] = ROcp47_727;
sens->R[3][2] = ROcp47_827;
sens->R[3][3] = ROcp47_927;
sens->V[1] = VIcp47_177;
sens->V[2] = VIcp47_277;
sens->V[3] = VIcp47_377;
sens->OM[1] = OMcp47_128;
sens->OM[2] = OMcp47_228;
sens->OM[3] = OMcp47_328;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp47_177_5;
sens->J[1][6] = JTcp47_177_6;
sens->J[1][19] = JTcp47_177_7;
sens->J[1][20] = JTcp47_177_8;
sens->J[1][21] = JTcp47_177_9;
sens->J[1][26] = JTcp47_177_10;
sens->J[1][27] = JTcp47_177_11;
sens->J[1][28] = JTcp47_177_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp47_277_4;
sens->J[2][5] = JTcp47_277_5;
sens->J[2][6] = JTcp47_277_6;
sens->J[2][19] = JTcp47_277_7;
sens->J[2][20] = JTcp47_277_8;
sens->J[2][21] = JTcp47_277_9;
sens->J[2][26] = JTcp47_277_10;
sens->J[2][27] = JTcp47_277_11;
sens->J[2][28] = JTcp47_277_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp47_377_4;
sens->J[3][5] = JTcp47_377_5;
sens->J[3][6] = JTcp47_377_6;
sens->J[3][19] = JTcp47_377_7;
sens->J[3][20] = JTcp47_377_8;
sens->J[3][21] = JTcp47_377_9;
sens->J[3][26] = JTcp47_377_10;
sens->J[3][27] = JTcp47_377_11;
sens->J[3][28] = JTcp47_377_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp47_16;
sens->J[4][20] = ROcp47_419;
sens->J[4][21] = ROcp47_720;
sens->J[4][26] = ROcp47_421;
sens->J[4][27] = ROcp47_126;
sens->J[4][28] = ROcp47_727;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp47_85;
sens->J[5][19] = ROcp47_26;
sens->J[5][20] = ROcp47_519;
sens->J[5][21] = ROcp47_820;
sens->J[5][26] = ROcp47_521;
sens->J[5][27] = ROcp47_226;
sens->J[5][28] = ROcp47_827;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp47_95;
sens->J[6][19] = ROcp47_36;
sens->J[6][20] = ROcp47_619;
sens->J[6][21] = ROcp47_920;
sens->J[6][26] = ROcp47_621;
sens->J[6][27] = ROcp47_326;
sens->J[6][28] = ROcp47_927;
sens->A[1] = ACcp47_177;
sens->A[2] = ACcp47_277;
sens->A[3] = ACcp47_377;
sens->OMP[1] = OPcp47_128;
sens->OMP[2] = OPcp47_228;
sens->OMP[3] = OPcp47_328;
 
// 
break;
case 49:
 


// = = Block_1_0_0_49_0_1 = = 
 
// Sensor Kinematics 


ROcp48_25 = S4*S5;
ROcp48_35 = -C4*S5;
ROcp48_85 = -S4*C5;
ROcp48_95 = C4*C5;
ROcp48_16 = C5*C6;
ROcp48_26 = ROcp48_25*C6+C4*S6;
ROcp48_36 = ROcp48_35*C6+S4*S6;
ROcp48_46 = -C5*S6;
ROcp48_56 = -(ROcp48_25*S6-C4*C6);
ROcp48_66 = -(ROcp48_35*S6-S4*C6);
OMcp48_25 = qd[5]*C4;
OMcp48_35 = qd[5]*S4;
OMcp48_16 = qd[4]+qd[6]*S5;
OMcp48_26 = OMcp48_25+ROcp48_85*qd[6];
OMcp48_36 = OMcp48_35+ROcp48_95*qd[6];
OPcp48_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp48_26 = ROcp48_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp48_35*S5-ROcp48_95*qd[4]);
OPcp48_36 = ROcp48_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp48_25*S5-ROcp48_85*qd[4]);

// = = Block_1_0_0_49_0_4 = = 
 
// Sensor Kinematics 


ROcp48_419 = ROcp48_46*C19+S19*S5;
ROcp48_519 = ROcp48_56*C19+ROcp48_85*S19;
ROcp48_619 = ROcp48_66*C19+ROcp48_95*S19;
ROcp48_719 = -(ROcp48_46*S19-C19*S5);
ROcp48_819 = -(ROcp48_56*S19-ROcp48_85*C19);
ROcp48_919 = -(ROcp48_66*S19-ROcp48_95*C19);
ROcp48_120 = ROcp48_16*C20-ROcp48_719*S20;
ROcp48_220 = ROcp48_26*C20-ROcp48_819*S20;
ROcp48_320 = ROcp48_36*C20-ROcp48_919*S20;
ROcp48_720 = ROcp48_16*S20+ROcp48_719*C20;
ROcp48_820 = ROcp48_26*S20+ROcp48_819*C20;
ROcp48_920 = ROcp48_36*S20+ROcp48_919*C20;
ROcp48_121 = ROcp48_120*C21+ROcp48_419*S21;
ROcp48_221 = ROcp48_220*C21+ROcp48_519*S21;
ROcp48_321 = ROcp48_320*C21+ROcp48_619*S21;
ROcp48_421 = -(ROcp48_120*S21-ROcp48_419*C21);
ROcp48_521 = -(ROcp48_220*S21-ROcp48_519*C21);
ROcp48_621 = -(ROcp48_320*S21-ROcp48_619*C21);
RLcp48_119 = s->dpt[2][3]*ROcp48_46+ROcp48_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp48_219 = s->dpt[2][3]*ROcp48_56+ROcp48_26*s->dpt[1][3]+ROcp48_85*s->dpt[3][3];
RLcp48_319 = s->dpt[2][3]*ROcp48_66+ROcp48_36*s->dpt[1][3]+ROcp48_95*s->dpt[3][3];
OMcp48_119 = OMcp48_16+ROcp48_16*qd[19];
OMcp48_219 = OMcp48_26+ROcp48_26*qd[19];
OMcp48_319 = OMcp48_36+ROcp48_36*qd[19];
ORcp48_119 = OMcp48_26*RLcp48_319-OMcp48_36*RLcp48_219;
ORcp48_219 = -(OMcp48_16*RLcp48_319-OMcp48_36*RLcp48_119);
ORcp48_319 = OMcp48_16*RLcp48_219-OMcp48_26*RLcp48_119;
OPcp48_119 = OPcp48_16+ROcp48_16*qdd[19]+qd[19]*(OMcp48_26*ROcp48_36-OMcp48_36*ROcp48_26);
OPcp48_219 = OPcp48_26+ROcp48_26*qdd[19]-qd[19]*(OMcp48_16*ROcp48_36-OMcp48_36*ROcp48_16);
OPcp48_319 = OPcp48_36+ROcp48_36*qdd[19]+qd[19]*(OMcp48_16*ROcp48_26-OMcp48_26*ROcp48_16);
RLcp48_120 = s->dpt[1][38]*ROcp48_16+s->dpt[2][38]*ROcp48_419+s->dpt[3][38]*ROcp48_719;
RLcp48_220 = s->dpt[1][38]*ROcp48_26+s->dpt[2][38]*ROcp48_519+s->dpt[3][38]*ROcp48_819;
RLcp48_320 = s->dpt[1][38]*ROcp48_36+s->dpt[2][38]*ROcp48_619+s->dpt[3][38]*ROcp48_919;
OMcp48_120 = OMcp48_119+ROcp48_419*qd[20];
OMcp48_220 = OMcp48_219+ROcp48_519*qd[20];
OMcp48_320 = OMcp48_319+ROcp48_619*qd[20];
ORcp48_120 = OMcp48_219*RLcp48_320-OMcp48_319*RLcp48_220;
ORcp48_220 = -(OMcp48_119*RLcp48_320-OMcp48_319*RLcp48_120);
ORcp48_320 = OMcp48_119*RLcp48_220-OMcp48_219*RLcp48_120;
OPcp48_120 = OPcp48_119+ROcp48_419*qdd[20]+qd[20]*(OMcp48_219*ROcp48_619-OMcp48_319*ROcp48_519);
OPcp48_220 = OPcp48_219+ROcp48_519*qdd[20]-qd[20]*(OMcp48_119*ROcp48_619-OMcp48_319*ROcp48_419);
OPcp48_320 = OPcp48_319+ROcp48_619*qdd[20]+qd[20]*(OMcp48_119*ROcp48_519-OMcp48_219*ROcp48_419);
RLcp48_121 = s->dpt[1][40]*ROcp48_120+s->dpt[2][40]*ROcp48_419+ROcp48_720*s->dpt[3][40];
RLcp48_221 = s->dpt[1][40]*ROcp48_220+s->dpt[2][40]*ROcp48_519+ROcp48_820*s->dpt[3][40];
RLcp48_321 = s->dpt[1][40]*ROcp48_320+s->dpt[2][40]*ROcp48_619+ROcp48_920*s->dpt[3][40];
OMcp48_121 = OMcp48_120+ROcp48_720*qd[21];
OMcp48_221 = OMcp48_220+ROcp48_820*qd[21];
OMcp48_321 = OMcp48_320+ROcp48_920*qd[21];
ORcp48_121 = OMcp48_220*RLcp48_321-OMcp48_320*RLcp48_221;
ORcp48_221 = -(OMcp48_120*RLcp48_321-OMcp48_320*RLcp48_121);
ORcp48_321 = OMcp48_120*RLcp48_221-OMcp48_220*RLcp48_121;
OPcp48_121 = OPcp48_120+ROcp48_720*qdd[21]+qd[21]*(OMcp48_220*ROcp48_920-OMcp48_320*ROcp48_820);
OPcp48_221 = OPcp48_220+ROcp48_820*qdd[21]-qd[21]*(OMcp48_120*ROcp48_920-OMcp48_320*ROcp48_720);
OPcp48_321 = OPcp48_320+ROcp48_920*qdd[21]+qd[21]*(OMcp48_120*ROcp48_820-OMcp48_220*ROcp48_720);

// = = Block_1_0_0_49_0_6 = = 
 
// Sensor Kinematics 


ROcp48_126 = ROcp48_121*C26-ROcp48_720*S26;
ROcp48_226 = ROcp48_221*C26-ROcp48_820*S26;
ROcp48_326 = ROcp48_321*C26-ROcp48_920*S26;
ROcp48_726 = ROcp48_121*S26+ROcp48_720*C26;
ROcp48_826 = ROcp48_221*S26+ROcp48_820*C26;
ROcp48_926 = ROcp48_321*S26+ROcp48_920*C26;
ROcp48_427 = ROcp48_421*C27+ROcp48_726*S27;
ROcp48_527 = ROcp48_521*C27+ROcp48_826*S27;
ROcp48_627 = ROcp48_621*C27+ROcp48_926*S27;
ROcp48_727 = -(ROcp48_421*S27-ROcp48_726*C27);
ROcp48_827 = -(ROcp48_521*S27-ROcp48_826*C27);
ROcp48_927 = -(ROcp48_621*S27-ROcp48_926*C27);
ROcp48_128 = ROcp48_126*C28+ROcp48_427*S28;
ROcp48_228 = ROcp48_226*C28+ROcp48_527*S28;
ROcp48_328 = ROcp48_326*C28+ROcp48_627*S28;
ROcp48_428 = -(ROcp48_126*S28-ROcp48_427*C28);
ROcp48_528 = -(ROcp48_226*S28-ROcp48_527*C28);
ROcp48_628 = -(ROcp48_326*S28-ROcp48_627*C28);
RLcp48_126 = ROcp48_121*s->dpt[1][45]+ROcp48_421*s->dpt[2][45]+ROcp48_720*s->dpt[3][45];
RLcp48_226 = ROcp48_221*s->dpt[1][45]+ROcp48_521*s->dpt[2][45]+ROcp48_820*s->dpt[3][45];
RLcp48_326 = ROcp48_321*s->dpt[1][45]+ROcp48_621*s->dpt[2][45]+ROcp48_920*s->dpt[3][45];
OMcp48_126 = OMcp48_121+ROcp48_421*qd[26];
OMcp48_226 = OMcp48_221+ROcp48_521*qd[26];
OMcp48_326 = OMcp48_321+ROcp48_621*qd[26];
ORcp48_126 = OMcp48_221*RLcp48_326-OMcp48_321*RLcp48_226;
ORcp48_226 = -(OMcp48_121*RLcp48_326-OMcp48_321*RLcp48_126);
ORcp48_326 = OMcp48_121*RLcp48_226-OMcp48_221*RLcp48_126;
OPcp48_126 = OPcp48_121+ROcp48_421*qdd[26]+qd[26]*(OMcp48_221*ROcp48_621-OMcp48_321*ROcp48_521);
OPcp48_226 = OPcp48_221+ROcp48_521*qdd[26]-qd[26]*(OMcp48_121*ROcp48_621-OMcp48_321*ROcp48_421);
OPcp48_326 = OPcp48_321+ROcp48_621*qdd[26]+qd[26]*(OMcp48_121*ROcp48_521-OMcp48_221*ROcp48_421);
RLcp48_127 = s->dpt[1][55]*ROcp48_126+s->dpt[3][55]*ROcp48_726+ROcp48_421*s->dpt[2][55];
RLcp48_227 = s->dpt[1][55]*ROcp48_226+s->dpt[3][55]*ROcp48_826+ROcp48_521*s->dpt[2][55];
RLcp48_327 = s->dpt[1][55]*ROcp48_326+s->dpt[3][55]*ROcp48_926+ROcp48_621*s->dpt[2][55];
OMcp48_127 = OMcp48_126+ROcp48_126*qd[27];
OMcp48_227 = OMcp48_226+ROcp48_226*qd[27];
OMcp48_327 = OMcp48_326+ROcp48_326*qd[27];
ORcp48_127 = OMcp48_226*RLcp48_327-OMcp48_326*RLcp48_227;
ORcp48_227 = -(OMcp48_126*RLcp48_327-OMcp48_326*RLcp48_127);
ORcp48_327 = OMcp48_126*RLcp48_227-OMcp48_226*RLcp48_127;
OPcp48_127 = OPcp48_126+ROcp48_126*qdd[27]+qd[27]*(OMcp48_226*ROcp48_326-OMcp48_326*ROcp48_226);
OPcp48_227 = OPcp48_226+ROcp48_226*qdd[27]-qd[27]*(OMcp48_126*ROcp48_326-OMcp48_326*ROcp48_126);
OPcp48_327 = OPcp48_326+ROcp48_326*qdd[27]+qd[27]*(OMcp48_126*ROcp48_226-OMcp48_226*ROcp48_126);
RLcp48_128 = s->dpt[1][57]*ROcp48_126+s->dpt[2][57]*ROcp48_427+ROcp48_727*s->dpt[3][57];
RLcp48_228 = s->dpt[1][57]*ROcp48_226+s->dpt[2][57]*ROcp48_527+ROcp48_827*s->dpt[3][57];
RLcp48_328 = s->dpt[1][57]*ROcp48_326+s->dpt[2][57]*ROcp48_627+ROcp48_927*s->dpt[3][57];
OMcp48_128 = OMcp48_127+ROcp48_727*qd[28];
OMcp48_228 = OMcp48_227+ROcp48_827*qd[28];
OMcp48_328 = OMcp48_327+ROcp48_927*qd[28];
ORcp48_128 = OMcp48_227*RLcp48_328-OMcp48_327*RLcp48_228;
ORcp48_228 = -(OMcp48_127*RLcp48_328-OMcp48_327*RLcp48_128);
ORcp48_328 = OMcp48_127*RLcp48_228-OMcp48_227*RLcp48_128;
OPcp48_128 = OPcp48_127+ROcp48_727*qdd[28]+qd[28]*(OMcp48_227*ROcp48_927-OMcp48_327*ROcp48_827);
OPcp48_228 = OPcp48_227+ROcp48_827*qdd[28]-qd[28]*(OMcp48_127*ROcp48_927-OMcp48_327*ROcp48_727);
OPcp48_328 = OPcp48_327+ROcp48_927*qdd[28]+qd[28]*(OMcp48_127*ROcp48_827-OMcp48_227*ROcp48_727);
RLcp48_178 = s->dpt[1][59]*ROcp48_128+s->dpt[2][59]*ROcp48_428+ROcp48_727*s->dpt[3][59];
RLcp48_278 = s->dpt[1][59]*ROcp48_228+s->dpt[2][59]*ROcp48_528+ROcp48_827*s->dpt[3][59];
RLcp48_378 = s->dpt[1][59]*ROcp48_328+s->dpt[2][59]*ROcp48_628+ROcp48_927*s->dpt[3][59];
POcp48_178 = RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_126+RLcp48_127+RLcp48_128+RLcp48_178+q[1];
POcp48_278 = RLcp48_219+RLcp48_220+RLcp48_221+RLcp48_226+RLcp48_227+RLcp48_228+RLcp48_278+q[2];
POcp48_378 = RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328+RLcp48_378+q[3];
JTcp48_278_4 = -(RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328+RLcp48_378);
JTcp48_378_4 = RLcp48_219+RLcp48_220+RLcp48_221+RLcp48_226+RLcp48_227+RLcp48_228+RLcp48_278;
JTcp48_178_5 = C4*(RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328)-S4*(RLcp48_219+RLcp48_220)-S4*(
 RLcp48_221+RLcp48_226)-S4*(RLcp48_227+RLcp48_228)-RLcp48_278*S4+RLcp48_378*C4;
JTcp48_278_5 = S4*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_126+RLcp48_127+RLcp48_128+RLcp48_178);
JTcp48_378_5 = -C4*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_126+RLcp48_127+RLcp48_128+RLcp48_178);
JTcp48_178_6 = ROcp48_85*(RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328)-ROcp48_95*(RLcp48_219+
 RLcp48_220)-ROcp48_95*(RLcp48_221+RLcp48_226)-ROcp48_95*(RLcp48_227+RLcp48_228)-RLcp48_278*ROcp48_95+RLcp48_378*ROcp48_85;
JTcp48_278_6 = -(RLcp48_378*S5-ROcp48_95*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_126+RLcp48_127+RLcp48_128+RLcp48_178
 )+S5*(RLcp48_319+RLcp48_320)+S5*(RLcp48_321+RLcp48_326)+S5*(RLcp48_327+RLcp48_328));
JTcp48_378_6 = RLcp48_278*S5-ROcp48_85*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_126+RLcp48_127+RLcp48_128+RLcp48_178)+
 S5*(RLcp48_219+RLcp48_220)+S5*(RLcp48_221+RLcp48_226)+S5*(RLcp48_227+RLcp48_228);
JTcp48_178_7 = ROcp48_26*(RLcp48_320+RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328+RLcp48_378)-ROcp48_36*(RLcp48_220+
 RLcp48_221)-ROcp48_36*(RLcp48_226+RLcp48_227)-ROcp48_36*(RLcp48_228+RLcp48_278);
JTcp48_278_7 = -(ROcp48_16*(RLcp48_320+RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328+RLcp48_378)-ROcp48_36*(RLcp48_120+
 RLcp48_121)-ROcp48_36*(RLcp48_126+RLcp48_127)-ROcp48_36*(RLcp48_128+RLcp48_178));
JTcp48_378_7 = ROcp48_16*(RLcp48_220+RLcp48_221+RLcp48_226+RLcp48_227+RLcp48_228+RLcp48_278)-ROcp48_26*(RLcp48_120+
 RLcp48_121)-ROcp48_26*(RLcp48_126+RLcp48_127)-ROcp48_26*(RLcp48_128+RLcp48_178);
JTcp48_178_8 = ROcp48_519*(RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328)-ROcp48_619*(RLcp48_221+RLcp48_226)-ROcp48_619*
 (RLcp48_227+RLcp48_228)-RLcp48_278*ROcp48_619+RLcp48_378*ROcp48_519;
JTcp48_278_8 = RLcp48_178*ROcp48_619-RLcp48_378*ROcp48_419-ROcp48_419*(RLcp48_321+RLcp48_326+RLcp48_327+RLcp48_328)+
 ROcp48_619*(RLcp48_121+RLcp48_126)+ROcp48_619*(RLcp48_127+RLcp48_128);
JTcp48_378_8 = ROcp48_419*(RLcp48_221+RLcp48_226+RLcp48_227+RLcp48_228)-ROcp48_519*(RLcp48_121+RLcp48_126)-ROcp48_519*
 (RLcp48_127+RLcp48_128)-RLcp48_178*ROcp48_519+RLcp48_278*ROcp48_419;
JTcp48_178_9 = ROcp48_820*(RLcp48_326+RLcp48_327+RLcp48_328+RLcp48_378)-ROcp48_920*(RLcp48_226+RLcp48_227)-ROcp48_920*
 (RLcp48_228+RLcp48_278);
JTcp48_278_9 = -(ROcp48_720*(RLcp48_326+RLcp48_327+RLcp48_328+RLcp48_378)-ROcp48_920*(RLcp48_126+RLcp48_127)-
 ROcp48_920*(RLcp48_128+RLcp48_178));
JTcp48_378_9 = ROcp48_720*(RLcp48_226+RLcp48_227+RLcp48_228+RLcp48_278)-ROcp48_820*(RLcp48_126+RLcp48_127)-ROcp48_820*
 (RLcp48_128+RLcp48_178);
JTcp48_178_10 = ROcp48_521*(RLcp48_327+RLcp48_328)-ROcp48_621*(RLcp48_227+RLcp48_228)-RLcp48_278*ROcp48_621+RLcp48_378
 *ROcp48_521;
JTcp48_278_10 = RLcp48_178*ROcp48_621-RLcp48_378*ROcp48_421-ROcp48_421*(RLcp48_327+RLcp48_328)+ROcp48_621*(RLcp48_127+
 RLcp48_128);
JTcp48_378_10 = ROcp48_421*(RLcp48_227+RLcp48_228)-ROcp48_521*(RLcp48_127+RLcp48_128)-RLcp48_178*ROcp48_521+RLcp48_278
 *ROcp48_421;
JTcp48_178_11 = ROcp48_226*(RLcp48_328+RLcp48_378)-ROcp48_326*(RLcp48_228+RLcp48_278);
JTcp48_278_11 = -(ROcp48_126*(RLcp48_328+RLcp48_378)-ROcp48_326*(RLcp48_128+RLcp48_178));
JTcp48_378_11 = ROcp48_126*(RLcp48_228+RLcp48_278)-ROcp48_226*(RLcp48_128+RLcp48_178);
JTcp48_178_12 = -(RLcp48_278*ROcp48_927-RLcp48_378*ROcp48_827);
JTcp48_278_12 = RLcp48_178*ROcp48_927-RLcp48_378*ROcp48_727;
JTcp48_378_12 = -(RLcp48_178*ROcp48_827-RLcp48_278*ROcp48_727);
ORcp48_178 = OMcp48_228*RLcp48_378-OMcp48_328*RLcp48_278;
ORcp48_278 = -(OMcp48_128*RLcp48_378-OMcp48_328*RLcp48_178);
ORcp48_378 = OMcp48_128*RLcp48_278-OMcp48_228*RLcp48_178;
VIcp48_178 = ORcp48_119+ORcp48_120+ORcp48_121+ORcp48_126+ORcp48_127+ORcp48_128+ORcp48_178+qd[1];
VIcp48_278 = ORcp48_219+ORcp48_220+ORcp48_221+ORcp48_226+ORcp48_227+ORcp48_228+ORcp48_278+qd[2];
VIcp48_378 = ORcp48_319+ORcp48_320+ORcp48_321+ORcp48_326+ORcp48_327+ORcp48_328+ORcp48_378+qd[3];
ACcp48_178 = qdd[1]+OMcp48_219*ORcp48_320+OMcp48_220*ORcp48_321+OMcp48_221*ORcp48_326+OMcp48_226*ORcp48_327+OMcp48_227
 *ORcp48_328+OMcp48_228*ORcp48_378+OMcp48_26*ORcp48_319-OMcp48_319*ORcp48_220-OMcp48_320*ORcp48_221-OMcp48_321*ORcp48_226-
 OMcp48_326*ORcp48_227-OMcp48_327*ORcp48_228-OMcp48_328*ORcp48_278-OMcp48_36*ORcp48_219+OPcp48_219*RLcp48_320+OPcp48_220*
 RLcp48_321+OPcp48_221*RLcp48_326+OPcp48_226*RLcp48_327+OPcp48_227*RLcp48_328+OPcp48_228*RLcp48_378+OPcp48_26*RLcp48_319-
 OPcp48_319*RLcp48_220-OPcp48_320*RLcp48_221-OPcp48_321*RLcp48_226-OPcp48_326*RLcp48_227-OPcp48_327*RLcp48_228-OPcp48_328*
 RLcp48_278-OPcp48_36*RLcp48_219;
ACcp48_278 = qdd[2]-OMcp48_119*ORcp48_320-OMcp48_120*ORcp48_321-OMcp48_121*ORcp48_326-OMcp48_126*ORcp48_327-OMcp48_127
 *ORcp48_328-OMcp48_128*ORcp48_378-OMcp48_16*ORcp48_319+OMcp48_319*ORcp48_120+OMcp48_320*ORcp48_121+OMcp48_321*ORcp48_126+
 OMcp48_326*ORcp48_127+OMcp48_327*ORcp48_128+OMcp48_328*ORcp48_178+OMcp48_36*ORcp48_119-OPcp48_119*RLcp48_320-OPcp48_120*
 RLcp48_321-OPcp48_121*RLcp48_326-OPcp48_126*RLcp48_327-OPcp48_127*RLcp48_328-OPcp48_128*RLcp48_378-OPcp48_16*RLcp48_319+
 OPcp48_319*RLcp48_120+OPcp48_320*RLcp48_121+OPcp48_321*RLcp48_126+OPcp48_326*RLcp48_127+OPcp48_327*RLcp48_128+OPcp48_328*
 RLcp48_178+OPcp48_36*RLcp48_119;
ACcp48_378 = qdd[3]+OMcp48_119*ORcp48_220+OMcp48_120*ORcp48_221+OMcp48_121*ORcp48_226+OMcp48_126*ORcp48_227+OMcp48_127
 *ORcp48_228+OMcp48_128*ORcp48_278+OMcp48_16*ORcp48_219-OMcp48_219*ORcp48_120-OMcp48_220*ORcp48_121-OMcp48_221*ORcp48_126-
 OMcp48_226*ORcp48_127-OMcp48_227*ORcp48_128-OMcp48_228*ORcp48_178-OMcp48_26*ORcp48_119+OPcp48_119*RLcp48_220+OPcp48_120*
 RLcp48_221+OPcp48_121*RLcp48_226+OPcp48_126*RLcp48_227+OPcp48_127*RLcp48_228+OPcp48_128*RLcp48_278+OPcp48_16*RLcp48_219-
 OPcp48_219*RLcp48_120-OPcp48_220*RLcp48_121-OPcp48_221*RLcp48_126-OPcp48_226*RLcp48_127-OPcp48_227*RLcp48_128-OPcp48_228*
 RLcp48_178-OPcp48_26*RLcp48_119;

// = = Block_1_0_0_49_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp48_178;
sens->P[2] = POcp48_278;
sens->P[3] = POcp48_378;
sens->R[1][1] = ROcp48_128;
sens->R[1][2] = ROcp48_228;
sens->R[1][3] = ROcp48_328;
sens->R[2][1] = ROcp48_428;
sens->R[2][2] = ROcp48_528;
sens->R[2][3] = ROcp48_628;
sens->R[3][1] = ROcp48_727;
sens->R[3][2] = ROcp48_827;
sens->R[3][3] = ROcp48_927;
sens->V[1] = VIcp48_178;
sens->V[2] = VIcp48_278;
sens->V[3] = VIcp48_378;
sens->OM[1] = OMcp48_128;
sens->OM[2] = OMcp48_228;
sens->OM[3] = OMcp48_328;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp48_178_5;
sens->J[1][6] = JTcp48_178_6;
sens->J[1][19] = JTcp48_178_7;
sens->J[1][20] = JTcp48_178_8;
sens->J[1][21] = JTcp48_178_9;
sens->J[1][26] = JTcp48_178_10;
sens->J[1][27] = JTcp48_178_11;
sens->J[1][28] = JTcp48_178_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp48_278_4;
sens->J[2][5] = JTcp48_278_5;
sens->J[2][6] = JTcp48_278_6;
sens->J[2][19] = JTcp48_278_7;
sens->J[2][20] = JTcp48_278_8;
sens->J[2][21] = JTcp48_278_9;
sens->J[2][26] = JTcp48_278_10;
sens->J[2][27] = JTcp48_278_11;
sens->J[2][28] = JTcp48_278_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp48_378_4;
sens->J[3][5] = JTcp48_378_5;
sens->J[3][6] = JTcp48_378_6;
sens->J[3][19] = JTcp48_378_7;
sens->J[3][20] = JTcp48_378_8;
sens->J[3][21] = JTcp48_378_9;
sens->J[3][26] = JTcp48_378_10;
sens->J[3][27] = JTcp48_378_11;
sens->J[3][28] = JTcp48_378_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp48_16;
sens->J[4][20] = ROcp48_419;
sens->J[4][21] = ROcp48_720;
sens->J[4][26] = ROcp48_421;
sens->J[4][27] = ROcp48_126;
sens->J[4][28] = ROcp48_727;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp48_85;
sens->J[5][19] = ROcp48_26;
sens->J[5][20] = ROcp48_519;
sens->J[5][21] = ROcp48_820;
sens->J[5][26] = ROcp48_521;
sens->J[5][27] = ROcp48_226;
sens->J[5][28] = ROcp48_827;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp48_95;
sens->J[6][19] = ROcp48_36;
sens->J[6][20] = ROcp48_619;
sens->J[6][21] = ROcp48_920;
sens->J[6][26] = ROcp48_621;
sens->J[6][27] = ROcp48_326;
sens->J[6][28] = ROcp48_927;
sens->A[1] = ACcp48_178;
sens->A[2] = ACcp48_278;
sens->A[3] = ACcp48_378;
sens->OMP[1] = OPcp48_128;
sens->OMP[2] = OPcp48_228;
sens->OMP[3] = OPcp48_328;
 
// 
break;
case 50:
 


// = = Block_1_0_0_50_0_1 = = 
 
// Sensor Kinematics 


ROcp49_25 = S4*S5;
ROcp49_35 = -C4*S5;
ROcp49_85 = -S4*C5;
ROcp49_95 = C4*C5;
ROcp49_16 = C5*C6;
ROcp49_26 = ROcp49_25*C6+C4*S6;
ROcp49_36 = ROcp49_35*C6+S4*S6;
ROcp49_46 = -C5*S6;
ROcp49_56 = -(ROcp49_25*S6-C4*C6);
ROcp49_66 = -(ROcp49_35*S6-S4*C6);
OMcp49_25 = qd[5]*C4;
OMcp49_35 = qd[5]*S4;
OMcp49_16 = qd[4]+qd[6]*S5;
OMcp49_26 = OMcp49_25+ROcp49_85*qd[6];
OMcp49_36 = OMcp49_35+ROcp49_95*qd[6];
OPcp49_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp49_26 = ROcp49_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp49_35*S5-ROcp49_95*qd[4]);
OPcp49_36 = ROcp49_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp49_25*S5-ROcp49_85*qd[4]);

// = = Block_1_0_0_50_0_4 = = 
 
// Sensor Kinematics 


ROcp49_419 = ROcp49_46*C19+S19*S5;
ROcp49_519 = ROcp49_56*C19+ROcp49_85*S19;
ROcp49_619 = ROcp49_66*C19+ROcp49_95*S19;
ROcp49_719 = -(ROcp49_46*S19-C19*S5);
ROcp49_819 = -(ROcp49_56*S19-ROcp49_85*C19);
ROcp49_919 = -(ROcp49_66*S19-ROcp49_95*C19);
ROcp49_120 = ROcp49_16*C20-ROcp49_719*S20;
ROcp49_220 = ROcp49_26*C20-ROcp49_819*S20;
ROcp49_320 = ROcp49_36*C20-ROcp49_919*S20;
ROcp49_720 = ROcp49_16*S20+ROcp49_719*C20;
ROcp49_820 = ROcp49_26*S20+ROcp49_819*C20;
ROcp49_920 = ROcp49_36*S20+ROcp49_919*C20;
ROcp49_121 = ROcp49_120*C21+ROcp49_419*S21;
ROcp49_221 = ROcp49_220*C21+ROcp49_519*S21;
ROcp49_321 = ROcp49_320*C21+ROcp49_619*S21;
ROcp49_421 = -(ROcp49_120*S21-ROcp49_419*C21);
ROcp49_521 = -(ROcp49_220*S21-ROcp49_519*C21);
ROcp49_621 = -(ROcp49_320*S21-ROcp49_619*C21);
RLcp49_119 = s->dpt[2][3]*ROcp49_46+ROcp49_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp49_219 = s->dpt[2][3]*ROcp49_56+ROcp49_26*s->dpt[1][3]+ROcp49_85*s->dpt[3][3];
RLcp49_319 = s->dpt[2][3]*ROcp49_66+ROcp49_36*s->dpt[1][3]+ROcp49_95*s->dpt[3][3];
OMcp49_119 = OMcp49_16+ROcp49_16*qd[19];
OMcp49_219 = OMcp49_26+ROcp49_26*qd[19];
OMcp49_319 = OMcp49_36+ROcp49_36*qd[19];
ORcp49_119 = OMcp49_26*RLcp49_319-OMcp49_36*RLcp49_219;
ORcp49_219 = -(OMcp49_16*RLcp49_319-OMcp49_36*RLcp49_119);
ORcp49_319 = OMcp49_16*RLcp49_219-OMcp49_26*RLcp49_119;
OPcp49_119 = OPcp49_16+ROcp49_16*qdd[19]+qd[19]*(OMcp49_26*ROcp49_36-OMcp49_36*ROcp49_26);
OPcp49_219 = OPcp49_26+ROcp49_26*qdd[19]-qd[19]*(OMcp49_16*ROcp49_36-OMcp49_36*ROcp49_16);
OPcp49_319 = OPcp49_36+ROcp49_36*qdd[19]+qd[19]*(OMcp49_16*ROcp49_26-OMcp49_26*ROcp49_16);
RLcp49_120 = s->dpt[1][38]*ROcp49_16+s->dpt[2][38]*ROcp49_419+s->dpt[3][38]*ROcp49_719;
RLcp49_220 = s->dpt[1][38]*ROcp49_26+s->dpt[2][38]*ROcp49_519+s->dpt[3][38]*ROcp49_819;
RLcp49_320 = s->dpt[1][38]*ROcp49_36+s->dpt[2][38]*ROcp49_619+s->dpt[3][38]*ROcp49_919;
OMcp49_120 = OMcp49_119+ROcp49_419*qd[20];
OMcp49_220 = OMcp49_219+ROcp49_519*qd[20];
OMcp49_320 = OMcp49_319+ROcp49_619*qd[20];
ORcp49_120 = OMcp49_219*RLcp49_320-OMcp49_319*RLcp49_220;
ORcp49_220 = -(OMcp49_119*RLcp49_320-OMcp49_319*RLcp49_120);
ORcp49_320 = OMcp49_119*RLcp49_220-OMcp49_219*RLcp49_120;
OPcp49_120 = OPcp49_119+ROcp49_419*qdd[20]+qd[20]*(OMcp49_219*ROcp49_619-OMcp49_319*ROcp49_519);
OPcp49_220 = OPcp49_219+ROcp49_519*qdd[20]-qd[20]*(OMcp49_119*ROcp49_619-OMcp49_319*ROcp49_419);
OPcp49_320 = OPcp49_319+ROcp49_619*qdd[20]+qd[20]*(OMcp49_119*ROcp49_519-OMcp49_219*ROcp49_419);
RLcp49_121 = s->dpt[1][40]*ROcp49_120+s->dpt[2][40]*ROcp49_419+ROcp49_720*s->dpt[3][40];
RLcp49_221 = s->dpt[1][40]*ROcp49_220+s->dpt[2][40]*ROcp49_519+ROcp49_820*s->dpt[3][40];
RLcp49_321 = s->dpt[1][40]*ROcp49_320+s->dpt[2][40]*ROcp49_619+ROcp49_920*s->dpt[3][40];
OMcp49_121 = OMcp49_120+ROcp49_720*qd[21];
OMcp49_221 = OMcp49_220+ROcp49_820*qd[21];
OMcp49_321 = OMcp49_320+ROcp49_920*qd[21];
ORcp49_121 = OMcp49_220*RLcp49_321-OMcp49_320*RLcp49_221;
ORcp49_221 = -(OMcp49_120*RLcp49_321-OMcp49_320*RLcp49_121);
ORcp49_321 = OMcp49_120*RLcp49_221-OMcp49_220*RLcp49_121;
OPcp49_121 = OPcp49_120+ROcp49_720*qdd[21]+qd[21]*(OMcp49_220*ROcp49_920-OMcp49_320*ROcp49_820);
OPcp49_221 = OPcp49_220+ROcp49_820*qdd[21]-qd[21]*(OMcp49_120*ROcp49_920-OMcp49_320*ROcp49_720);
OPcp49_321 = OPcp49_320+ROcp49_920*qdd[21]+qd[21]*(OMcp49_120*ROcp49_820-OMcp49_220*ROcp49_720);

// = = Block_1_0_0_50_0_6 = = 
 
// Sensor Kinematics 


ROcp49_126 = ROcp49_121*C26-ROcp49_720*S26;
ROcp49_226 = ROcp49_221*C26-ROcp49_820*S26;
ROcp49_326 = ROcp49_321*C26-ROcp49_920*S26;
ROcp49_726 = ROcp49_121*S26+ROcp49_720*C26;
ROcp49_826 = ROcp49_221*S26+ROcp49_820*C26;
ROcp49_926 = ROcp49_321*S26+ROcp49_920*C26;
ROcp49_427 = ROcp49_421*C27+ROcp49_726*S27;
ROcp49_527 = ROcp49_521*C27+ROcp49_826*S27;
ROcp49_627 = ROcp49_621*C27+ROcp49_926*S27;
ROcp49_727 = -(ROcp49_421*S27-ROcp49_726*C27);
ROcp49_827 = -(ROcp49_521*S27-ROcp49_826*C27);
ROcp49_927 = -(ROcp49_621*S27-ROcp49_926*C27);
ROcp49_128 = ROcp49_126*C28+ROcp49_427*S28;
ROcp49_228 = ROcp49_226*C28+ROcp49_527*S28;
ROcp49_328 = ROcp49_326*C28+ROcp49_627*S28;
ROcp49_428 = -(ROcp49_126*S28-ROcp49_427*C28);
ROcp49_528 = -(ROcp49_226*S28-ROcp49_527*C28);
ROcp49_628 = -(ROcp49_326*S28-ROcp49_627*C28);
ROcp49_129 = ROcp49_128*C29-ROcp49_727*S29;
ROcp49_229 = ROcp49_228*C29-ROcp49_827*S29;
ROcp49_329 = ROcp49_328*C29-ROcp49_927*S29;
ROcp49_729 = ROcp49_128*S29+ROcp49_727*C29;
ROcp49_829 = ROcp49_228*S29+ROcp49_827*C29;
ROcp49_929 = ROcp49_328*S29+ROcp49_927*C29;
RLcp49_126 = ROcp49_121*s->dpt[1][45]+ROcp49_421*s->dpt[2][45]+ROcp49_720*s->dpt[3][45];
RLcp49_226 = ROcp49_221*s->dpt[1][45]+ROcp49_521*s->dpt[2][45]+ROcp49_820*s->dpt[3][45];
RLcp49_326 = ROcp49_321*s->dpt[1][45]+ROcp49_621*s->dpt[2][45]+ROcp49_920*s->dpt[3][45];
OMcp49_126 = OMcp49_121+ROcp49_421*qd[26];
OMcp49_226 = OMcp49_221+ROcp49_521*qd[26];
OMcp49_326 = OMcp49_321+ROcp49_621*qd[26];
ORcp49_126 = OMcp49_221*RLcp49_326-OMcp49_321*RLcp49_226;
ORcp49_226 = -(OMcp49_121*RLcp49_326-OMcp49_321*RLcp49_126);
ORcp49_326 = OMcp49_121*RLcp49_226-OMcp49_221*RLcp49_126;
OPcp49_126 = OPcp49_121+ROcp49_421*qdd[26]+qd[26]*(OMcp49_221*ROcp49_621-OMcp49_321*ROcp49_521);
OPcp49_226 = OPcp49_221+ROcp49_521*qdd[26]-qd[26]*(OMcp49_121*ROcp49_621-OMcp49_321*ROcp49_421);
OPcp49_326 = OPcp49_321+ROcp49_621*qdd[26]+qd[26]*(OMcp49_121*ROcp49_521-OMcp49_221*ROcp49_421);
RLcp49_127 = s->dpt[1][55]*ROcp49_126+s->dpt[3][55]*ROcp49_726+ROcp49_421*s->dpt[2][55];
RLcp49_227 = s->dpt[1][55]*ROcp49_226+s->dpt[3][55]*ROcp49_826+ROcp49_521*s->dpt[2][55];
RLcp49_327 = s->dpt[1][55]*ROcp49_326+s->dpt[3][55]*ROcp49_926+ROcp49_621*s->dpt[2][55];
OMcp49_127 = OMcp49_126+ROcp49_126*qd[27];
OMcp49_227 = OMcp49_226+ROcp49_226*qd[27];
OMcp49_327 = OMcp49_326+ROcp49_326*qd[27];
ORcp49_127 = OMcp49_226*RLcp49_327-OMcp49_326*RLcp49_227;
ORcp49_227 = -(OMcp49_126*RLcp49_327-OMcp49_326*RLcp49_127);
ORcp49_327 = OMcp49_126*RLcp49_227-OMcp49_226*RLcp49_127;
OPcp49_127 = OPcp49_126+ROcp49_126*qdd[27]+qd[27]*(OMcp49_226*ROcp49_326-OMcp49_326*ROcp49_226);
OPcp49_227 = OPcp49_226+ROcp49_226*qdd[27]-qd[27]*(OMcp49_126*ROcp49_326-OMcp49_326*ROcp49_126);
OPcp49_327 = OPcp49_326+ROcp49_326*qdd[27]+qd[27]*(OMcp49_126*ROcp49_226-OMcp49_226*ROcp49_126);
RLcp49_128 = s->dpt[1][57]*ROcp49_126+s->dpt[2][57]*ROcp49_427+ROcp49_727*s->dpt[3][57];
RLcp49_228 = s->dpt[1][57]*ROcp49_226+s->dpt[2][57]*ROcp49_527+ROcp49_827*s->dpt[3][57];
RLcp49_328 = s->dpt[1][57]*ROcp49_326+s->dpt[2][57]*ROcp49_627+ROcp49_927*s->dpt[3][57];
OMcp49_128 = OMcp49_127+ROcp49_727*qd[28];
OMcp49_228 = OMcp49_227+ROcp49_827*qd[28];
OMcp49_328 = OMcp49_327+ROcp49_927*qd[28];
ORcp49_128 = OMcp49_227*RLcp49_328-OMcp49_327*RLcp49_228;
ORcp49_228 = -(OMcp49_127*RLcp49_328-OMcp49_327*RLcp49_128);
ORcp49_328 = OMcp49_127*RLcp49_228-OMcp49_227*RLcp49_128;
OPcp49_128 = OPcp49_127+ROcp49_727*qdd[28]+qd[28]*(OMcp49_227*ROcp49_927-OMcp49_327*ROcp49_827);
OPcp49_228 = OPcp49_227+ROcp49_827*qdd[28]-qd[28]*(OMcp49_127*ROcp49_927-OMcp49_327*ROcp49_727);
OPcp49_328 = OPcp49_327+ROcp49_927*qdd[28]+qd[28]*(OMcp49_127*ROcp49_827-OMcp49_227*ROcp49_727);
RLcp49_129 = s->dpt[1][59]*ROcp49_128+s->dpt[2][59]*ROcp49_428+ROcp49_727*s->dpt[3][59];
RLcp49_229 = s->dpt[1][59]*ROcp49_228+s->dpt[2][59]*ROcp49_528+ROcp49_827*s->dpt[3][59];
RLcp49_329 = s->dpt[1][59]*ROcp49_328+s->dpt[2][59]*ROcp49_628+ROcp49_927*s->dpt[3][59];
OMcp49_129 = OMcp49_128+ROcp49_428*qd[29];
OMcp49_229 = OMcp49_228+ROcp49_528*qd[29];
OMcp49_329 = OMcp49_328+ROcp49_628*qd[29];
ORcp49_129 = OMcp49_228*RLcp49_329-OMcp49_328*RLcp49_229;
ORcp49_229 = -(OMcp49_128*RLcp49_329-OMcp49_328*RLcp49_129);
ORcp49_329 = OMcp49_128*RLcp49_229-OMcp49_228*RLcp49_129;
OPcp49_129 = OPcp49_128+ROcp49_428*qdd[29]+qd[29]*(OMcp49_228*ROcp49_628-OMcp49_328*ROcp49_528);
OPcp49_229 = OPcp49_228+ROcp49_528*qdd[29]-qd[29]*(OMcp49_128*ROcp49_628-OMcp49_328*ROcp49_428);
OPcp49_329 = OPcp49_328+ROcp49_628*qdd[29]+qd[29]*(OMcp49_128*ROcp49_528-OMcp49_228*ROcp49_428);
RLcp49_179 = ROcp49_129*s->dpt[1][60]+ROcp49_428*s->dpt[2][60]+ROcp49_729*s->dpt[3][60];
RLcp49_279 = ROcp49_229*s->dpt[1][60]+ROcp49_528*s->dpt[2][60]+ROcp49_829*s->dpt[3][60];
RLcp49_379 = ROcp49_329*s->dpt[1][60]+ROcp49_628*s->dpt[2][60]+ROcp49_929*s->dpt[3][60];
POcp49_179 = RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_126+RLcp49_127+RLcp49_128+RLcp49_129+RLcp49_179+q[1];
POcp49_279 = RLcp49_219+RLcp49_220+RLcp49_221+RLcp49_226+RLcp49_227+RLcp49_228+RLcp49_229+RLcp49_279+q[2];
POcp49_379 = RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379+q[3];
JTcp49_279_4 = -(RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379);
JTcp49_379_4 = RLcp49_219+RLcp49_220+RLcp49_221+RLcp49_226+RLcp49_227+RLcp49_228+RLcp49_229+RLcp49_279;
JTcp49_179_5 = C4*(RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379)-S4*(
 RLcp49_219+RLcp49_220)-S4*(RLcp49_221+RLcp49_226)-S4*(RLcp49_227+RLcp49_228)-S4*(RLcp49_229+RLcp49_279);
JTcp49_279_5 = S4*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_126+RLcp49_127+RLcp49_128+RLcp49_129+RLcp49_179);
JTcp49_379_5 = -C4*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_126+RLcp49_127+RLcp49_128+RLcp49_129+RLcp49_179);
JTcp49_179_6 = ROcp49_85*(RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379)-
 ROcp49_95*(RLcp49_219+RLcp49_220)-ROcp49_95*(RLcp49_221+RLcp49_226)-ROcp49_95*(RLcp49_227+RLcp49_228)-ROcp49_95*(RLcp49_229+
 RLcp49_279);
JTcp49_279_6 = RLcp49_179*ROcp49_95-RLcp49_329*S5-RLcp49_379*S5+ROcp49_95*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_126
 +RLcp49_127+RLcp49_128+RLcp49_129)-S5*(RLcp49_319+RLcp49_320)-S5*(RLcp49_321+RLcp49_326)-S5*(RLcp49_327+RLcp49_328);
JTcp49_379_6 = RLcp49_229*S5-ROcp49_85*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_126+RLcp49_127+RLcp49_128+RLcp49_129)+
 S5*(RLcp49_219+RLcp49_220)+S5*(RLcp49_221+RLcp49_226)+S5*(RLcp49_227+RLcp49_228)-RLcp49_179*ROcp49_85+RLcp49_279*S5;
JTcp49_179_7 = ROcp49_26*(RLcp49_320+RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329)-ROcp49_36*(RLcp49_220+
 RLcp49_221)-ROcp49_36*(RLcp49_226+RLcp49_227)-ROcp49_36*(RLcp49_228+RLcp49_229)-RLcp49_279*ROcp49_36+RLcp49_379*ROcp49_26;
JTcp49_279_7 = RLcp49_179*ROcp49_36-RLcp49_379*ROcp49_16-ROcp49_16*(RLcp49_320+RLcp49_321+RLcp49_326+RLcp49_327+
 RLcp49_328+RLcp49_329)+ROcp49_36*(RLcp49_120+RLcp49_121)+ROcp49_36*(RLcp49_126+RLcp49_127)+ROcp49_36*(RLcp49_128+RLcp49_129);
JTcp49_379_7 = ROcp49_16*(RLcp49_220+RLcp49_221+RLcp49_226+RLcp49_227+RLcp49_228+RLcp49_229)-ROcp49_26*(RLcp49_120+
 RLcp49_121)-ROcp49_26*(RLcp49_126+RLcp49_127)-ROcp49_26*(RLcp49_128+RLcp49_129)-RLcp49_179*ROcp49_26+RLcp49_279*ROcp49_16;
JTcp49_179_8 = ROcp49_519*(RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379)-ROcp49_619*(RLcp49_221+
 RLcp49_226)-ROcp49_619*(RLcp49_227+RLcp49_228)-ROcp49_619*(RLcp49_229+RLcp49_279);
JTcp49_279_8 = -(ROcp49_419*(RLcp49_321+RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379)-ROcp49_619*(RLcp49_121
 +RLcp49_126)-ROcp49_619*(RLcp49_127+RLcp49_128)-ROcp49_619*(RLcp49_129+RLcp49_179));
JTcp49_379_8 = ROcp49_419*(RLcp49_221+RLcp49_226+RLcp49_227+RLcp49_228+RLcp49_229+RLcp49_279)-ROcp49_519*(RLcp49_121+
 RLcp49_126)-ROcp49_519*(RLcp49_127+RLcp49_128)-ROcp49_519*(RLcp49_129+RLcp49_179);
JTcp49_179_9 = ROcp49_820*(RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329)-ROcp49_920*(RLcp49_226+RLcp49_227)-ROcp49_920*
 (RLcp49_228+RLcp49_229)-RLcp49_279*ROcp49_920+RLcp49_379*ROcp49_820;
JTcp49_279_9 = RLcp49_179*ROcp49_920-RLcp49_379*ROcp49_720-ROcp49_720*(RLcp49_326+RLcp49_327+RLcp49_328+RLcp49_329)+
 ROcp49_920*(RLcp49_126+RLcp49_127)+ROcp49_920*(RLcp49_128+RLcp49_129);
JTcp49_379_9 = ROcp49_720*(RLcp49_226+RLcp49_227+RLcp49_228+RLcp49_229)-ROcp49_820*(RLcp49_126+RLcp49_127)-ROcp49_820*
 (RLcp49_128+RLcp49_129)-RLcp49_179*ROcp49_820+RLcp49_279*ROcp49_720;
JTcp49_179_10 = ROcp49_521*(RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379)-ROcp49_621*(RLcp49_227+RLcp49_228)-ROcp49_621
 *(RLcp49_229+RLcp49_279);
JTcp49_279_10 = -(ROcp49_421*(RLcp49_327+RLcp49_328+RLcp49_329+RLcp49_379)-ROcp49_621*(RLcp49_127+RLcp49_128)-
 ROcp49_621*(RLcp49_129+RLcp49_179));
JTcp49_379_10 = ROcp49_421*(RLcp49_227+RLcp49_228+RLcp49_229+RLcp49_279)-ROcp49_521*(RLcp49_127+RLcp49_128)-ROcp49_521
 *(RLcp49_129+RLcp49_179);
JTcp49_179_11 = ROcp49_226*(RLcp49_328+RLcp49_329)-ROcp49_326*(RLcp49_228+RLcp49_229)-RLcp49_279*ROcp49_326+RLcp49_379
 *ROcp49_226;
JTcp49_279_11 = RLcp49_179*ROcp49_326-RLcp49_379*ROcp49_126-ROcp49_126*(RLcp49_328+RLcp49_329)+ROcp49_326*(RLcp49_128+
 RLcp49_129);
JTcp49_379_11 = ROcp49_126*(RLcp49_228+RLcp49_229)-ROcp49_226*(RLcp49_128+RLcp49_129)-RLcp49_179*ROcp49_226+RLcp49_279
 *ROcp49_126;
JTcp49_179_12 = ROcp49_827*(RLcp49_329+RLcp49_379)-ROcp49_927*(RLcp49_229+RLcp49_279);
JTcp49_279_12 = -(ROcp49_727*(RLcp49_329+RLcp49_379)-ROcp49_927*(RLcp49_129+RLcp49_179));
JTcp49_379_12 = ROcp49_727*(RLcp49_229+RLcp49_279)-ROcp49_827*(RLcp49_129+RLcp49_179);
JTcp49_179_13 = -(RLcp49_279*ROcp49_628-RLcp49_379*ROcp49_528);
JTcp49_279_13 = RLcp49_179*ROcp49_628-RLcp49_379*ROcp49_428;
JTcp49_379_13 = -(RLcp49_179*ROcp49_528-RLcp49_279*ROcp49_428);
ORcp49_179 = OMcp49_229*RLcp49_379-OMcp49_329*RLcp49_279;
ORcp49_279 = -(OMcp49_129*RLcp49_379-OMcp49_329*RLcp49_179);
ORcp49_379 = OMcp49_129*RLcp49_279-OMcp49_229*RLcp49_179;
VIcp49_179 = ORcp49_119+ORcp49_120+ORcp49_121+ORcp49_126+ORcp49_127+ORcp49_128+ORcp49_129+ORcp49_179+qd[1];
VIcp49_279 = ORcp49_219+ORcp49_220+ORcp49_221+ORcp49_226+ORcp49_227+ORcp49_228+ORcp49_229+ORcp49_279+qd[2];
VIcp49_379 = ORcp49_319+ORcp49_320+ORcp49_321+ORcp49_326+ORcp49_327+ORcp49_328+ORcp49_329+ORcp49_379+qd[3];
ACcp49_179 = qdd[1]+OMcp49_219*ORcp49_320+OMcp49_220*ORcp49_321+OMcp49_221*ORcp49_326+OMcp49_226*ORcp49_327+OMcp49_227
 *ORcp49_328+OMcp49_228*ORcp49_329+OMcp49_229*ORcp49_379+OMcp49_26*ORcp49_319-OMcp49_319*ORcp49_220-OMcp49_320*ORcp49_221-
 OMcp49_321*ORcp49_226-OMcp49_326*ORcp49_227-OMcp49_327*ORcp49_228-OMcp49_328*ORcp49_229-OMcp49_329*ORcp49_279-OMcp49_36*
 ORcp49_219+OPcp49_219*RLcp49_320+OPcp49_220*RLcp49_321+OPcp49_221*RLcp49_326+OPcp49_226*RLcp49_327+OPcp49_227*RLcp49_328+
 OPcp49_228*RLcp49_329+OPcp49_229*RLcp49_379+OPcp49_26*RLcp49_319-OPcp49_319*RLcp49_220-OPcp49_320*RLcp49_221-OPcp49_321*
 RLcp49_226-OPcp49_326*RLcp49_227-OPcp49_327*RLcp49_228-OPcp49_328*RLcp49_229-OPcp49_329*RLcp49_279-OPcp49_36*RLcp49_219;
ACcp49_279 = qdd[2]-OMcp49_119*ORcp49_320-OMcp49_120*ORcp49_321-OMcp49_121*ORcp49_326-OMcp49_126*ORcp49_327-OMcp49_127
 *ORcp49_328-OMcp49_128*ORcp49_329-OMcp49_129*ORcp49_379-OMcp49_16*ORcp49_319+OMcp49_319*ORcp49_120+OMcp49_320*ORcp49_121+
 OMcp49_321*ORcp49_126+OMcp49_326*ORcp49_127+OMcp49_327*ORcp49_128+OMcp49_328*ORcp49_129+OMcp49_329*ORcp49_179+OMcp49_36*
 ORcp49_119-OPcp49_119*RLcp49_320-OPcp49_120*RLcp49_321-OPcp49_121*RLcp49_326-OPcp49_126*RLcp49_327-OPcp49_127*RLcp49_328-
 OPcp49_128*RLcp49_329-OPcp49_129*RLcp49_379-OPcp49_16*RLcp49_319+OPcp49_319*RLcp49_120+OPcp49_320*RLcp49_121+OPcp49_321*
 RLcp49_126+OPcp49_326*RLcp49_127+OPcp49_327*RLcp49_128+OPcp49_328*RLcp49_129+OPcp49_329*RLcp49_179+OPcp49_36*RLcp49_119;
ACcp49_379 = qdd[3]+OMcp49_119*ORcp49_220+OMcp49_120*ORcp49_221+OMcp49_121*ORcp49_226+OMcp49_126*ORcp49_227+OMcp49_127
 *ORcp49_228+OMcp49_128*ORcp49_229+OMcp49_129*ORcp49_279+OMcp49_16*ORcp49_219-OMcp49_219*ORcp49_120-OMcp49_220*ORcp49_121-
 OMcp49_221*ORcp49_126-OMcp49_226*ORcp49_127-OMcp49_227*ORcp49_128-OMcp49_228*ORcp49_129-OMcp49_229*ORcp49_179-OMcp49_26*
 ORcp49_119+OPcp49_119*RLcp49_220+OPcp49_120*RLcp49_221+OPcp49_121*RLcp49_226+OPcp49_126*RLcp49_227+OPcp49_127*RLcp49_228+
 OPcp49_128*RLcp49_229+OPcp49_129*RLcp49_279+OPcp49_16*RLcp49_219-OPcp49_219*RLcp49_120-OPcp49_220*RLcp49_121-OPcp49_221*
 RLcp49_126-OPcp49_226*RLcp49_127-OPcp49_227*RLcp49_128-OPcp49_228*RLcp49_129-OPcp49_229*RLcp49_179-OPcp49_26*RLcp49_119;

// = = Block_1_0_0_50_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp49_179;
sens->P[2] = POcp49_279;
sens->P[3] = POcp49_379;
sens->R[1][1] = ROcp49_129;
sens->R[1][2] = ROcp49_229;
sens->R[1][3] = ROcp49_329;
sens->R[2][1] = ROcp49_428;
sens->R[2][2] = ROcp49_528;
sens->R[2][3] = ROcp49_628;
sens->R[3][1] = ROcp49_729;
sens->R[3][2] = ROcp49_829;
sens->R[3][3] = ROcp49_929;
sens->V[1] = VIcp49_179;
sens->V[2] = VIcp49_279;
sens->V[3] = VIcp49_379;
sens->OM[1] = OMcp49_129;
sens->OM[2] = OMcp49_229;
sens->OM[3] = OMcp49_329;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp49_179_5;
sens->J[1][6] = JTcp49_179_6;
sens->J[1][19] = JTcp49_179_7;
sens->J[1][20] = JTcp49_179_8;
sens->J[1][21] = JTcp49_179_9;
sens->J[1][26] = JTcp49_179_10;
sens->J[1][27] = JTcp49_179_11;
sens->J[1][28] = JTcp49_179_12;
sens->J[1][29] = JTcp49_179_13;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp49_279_4;
sens->J[2][5] = JTcp49_279_5;
sens->J[2][6] = JTcp49_279_6;
sens->J[2][19] = JTcp49_279_7;
sens->J[2][20] = JTcp49_279_8;
sens->J[2][21] = JTcp49_279_9;
sens->J[2][26] = JTcp49_279_10;
sens->J[2][27] = JTcp49_279_11;
sens->J[2][28] = JTcp49_279_12;
sens->J[2][29] = JTcp49_279_13;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp49_379_4;
sens->J[3][5] = JTcp49_379_5;
sens->J[3][6] = JTcp49_379_6;
sens->J[3][19] = JTcp49_379_7;
sens->J[3][20] = JTcp49_379_8;
sens->J[3][21] = JTcp49_379_9;
sens->J[3][26] = JTcp49_379_10;
sens->J[3][27] = JTcp49_379_11;
sens->J[3][28] = JTcp49_379_12;
sens->J[3][29] = JTcp49_379_13;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp49_16;
sens->J[4][20] = ROcp49_419;
sens->J[4][21] = ROcp49_720;
sens->J[4][26] = ROcp49_421;
sens->J[4][27] = ROcp49_126;
sens->J[4][28] = ROcp49_727;
sens->J[4][29] = ROcp49_428;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp49_85;
sens->J[5][19] = ROcp49_26;
sens->J[5][20] = ROcp49_519;
sens->J[5][21] = ROcp49_820;
sens->J[5][26] = ROcp49_521;
sens->J[5][27] = ROcp49_226;
sens->J[5][28] = ROcp49_827;
sens->J[5][29] = ROcp49_528;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp49_95;
sens->J[6][19] = ROcp49_36;
sens->J[6][20] = ROcp49_619;
sens->J[6][21] = ROcp49_920;
sens->J[6][26] = ROcp49_621;
sens->J[6][27] = ROcp49_326;
sens->J[6][28] = ROcp49_927;
sens->J[6][29] = ROcp49_628;
sens->A[1] = ACcp49_179;
sens->A[2] = ACcp49_279;
sens->A[3] = ACcp49_379;
sens->OMP[1] = OPcp49_129;
sens->OMP[2] = OPcp49_229;
sens->OMP[3] = OPcp49_329;
 
// 
break;
case 51:
 


// = = Block_1_0_0_51_0_1 = = 
 
// Sensor Kinematics 


ROcp50_25 = S4*S5;
ROcp50_35 = -C4*S5;
ROcp50_85 = -S4*C5;
ROcp50_95 = C4*C5;
ROcp50_16 = C5*C6;
ROcp50_26 = ROcp50_25*C6+C4*S6;
ROcp50_36 = ROcp50_35*C6+S4*S6;
ROcp50_46 = -C5*S6;
ROcp50_56 = -(ROcp50_25*S6-C4*C6);
ROcp50_66 = -(ROcp50_35*S6-S4*C6);
OMcp50_25 = qd[5]*C4;
OMcp50_35 = qd[5]*S4;
OMcp50_16 = qd[4]+qd[6]*S5;
OMcp50_26 = OMcp50_25+ROcp50_85*qd[6];
OMcp50_36 = OMcp50_35+ROcp50_95*qd[6];
OPcp50_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp50_26 = ROcp50_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp50_35*S5-ROcp50_95*qd[4]);
OPcp50_36 = ROcp50_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp50_25*S5-ROcp50_85*qd[4]);

// = = Block_1_0_0_51_0_4 = = 
 
// Sensor Kinematics 


ROcp50_419 = ROcp50_46*C19+S19*S5;
ROcp50_519 = ROcp50_56*C19+ROcp50_85*S19;
ROcp50_619 = ROcp50_66*C19+ROcp50_95*S19;
ROcp50_719 = -(ROcp50_46*S19-C19*S5);
ROcp50_819 = -(ROcp50_56*S19-ROcp50_85*C19);
ROcp50_919 = -(ROcp50_66*S19-ROcp50_95*C19);
ROcp50_120 = ROcp50_16*C20-ROcp50_719*S20;
ROcp50_220 = ROcp50_26*C20-ROcp50_819*S20;
ROcp50_320 = ROcp50_36*C20-ROcp50_919*S20;
ROcp50_720 = ROcp50_16*S20+ROcp50_719*C20;
ROcp50_820 = ROcp50_26*S20+ROcp50_819*C20;
ROcp50_920 = ROcp50_36*S20+ROcp50_919*C20;
ROcp50_121 = ROcp50_120*C21+ROcp50_419*S21;
ROcp50_221 = ROcp50_220*C21+ROcp50_519*S21;
ROcp50_321 = ROcp50_320*C21+ROcp50_619*S21;
ROcp50_421 = -(ROcp50_120*S21-ROcp50_419*C21);
ROcp50_521 = -(ROcp50_220*S21-ROcp50_519*C21);
ROcp50_621 = -(ROcp50_320*S21-ROcp50_619*C21);
RLcp50_119 = s->dpt[2][3]*ROcp50_46+ROcp50_16*s->dpt[1][3]+s->dpt[3][3]*S5;
RLcp50_219 = s->dpt[2][3]*ROcp50_56+ROcp50_26*s->dpt[1][3]+ROcp50_85*s->dpt[3][3];
RLcp50_319 = s->dpt[2][3]*ROcp50_66+ROcp50_36*s->dpt[1][3]+ROcp50_95*s->dpt[3][3];
OMcp50_119 = OMcp50_16+ROcp50_16*qd[19];
OMcp50_219 = OMcp50_26+ROcp50_26*qd[19];
OMcp50_319 = OMcp50_36+ROcp50_36*qd[19];
ORcp50_119 = OMcp50_26*RLcp50_319-OMcp50_36*RLcp50_219;
ORcp50_219 = -(OMcp50_16*RLcp50_319-OMcp50_36*RLcp50_119);
ORcp50_319 = OMcp50_16*RLcp50_219-OMcp50_26*RLcp50_119;
OPcp50_119 = OPcp50_16+ROcp50_16*qdd[19]+qd[19]*(OMcp50_26*ROcp50_36-OMcp50_36*ROcp50_26);
OPcp50_219 = OPcp50_26+ROcp50_26*qdd[19]-qd[19]*(OMcp50_16*ROcp50_36-OMcp50_36*ROcp50_16);
OPcp50_319 = OPcp50_36+ROcp50_36*qdd[19]+qd[19]*(OMcp50_16*ROcp50_26-OMcp50_26*ROcp50_16);
RLcp50_120 = s->dpt[1][38]*ROcp50_16+s->dpt[2][38]*ROcp50_419+s->dpt[3][38]*ROcp50_719;
RLcp50_220 = s->dpt[1][38]*ROcp50_26+s->dpt[2][38]*ROcp50_519+s->dpt[3][38]*ROcp50_819;
RLcp50_320 = s->dpt[1][38]*ROcp50_36+s->dpt[2][38]*ROcp50_619+s->dpt[3][38]*ROcp50_919;
OMcp50_120 = OMcp50_119+ROcp50_419*qd[20];
OMcp50_220 = OMcp50_219+ROcp50_519*qd[20];
OMcp50_320 = OMcp50_319+ROcp50_619*qd[20];
ORcp50_120 = OMcp50_219*RLcp50_320-OMcp50_319*RLcp50_220;
ORcp50_220 = -(OMcp50_119*RLcp50_320-OMcp50_319*RLcp50_120);
ORcp50_320 = OMcp50_119*RLcp50_220-OMcp50_219*RLcp50_120;
OPcp50_120 = OPcp50_119+ROcp50_419*qdd[20]+qd[20]*(OMcp50_219*ROcp50_619-OMcp50_319*ROcp50_519);
OPcp50_220 = OPcp50_219+ROcp50_519*qdd[20]-qd[20]*(OMcp50_119*ROcp50_619-OMcp50_319*ROcp50_419);
OPcp50_320 = OPcp50_319+ROcp50_619*qdd[20]+qd[20]*(OMcp50_119*ROcp50_519-OMcp50_219*ROcp50_419);
RLcp50_121 = s->dpt[1][40]*ROcp50_120+s->dpt[2][40]*ROcp50_419+ROcp50_720*s->dpt[3][40];
RLcp50_221 = s->dpt[1][40]*ROcp50_220+s->dpt[2][40]*ROcp50_519+ROcp50_820*s->dpt[3][40];
RLcp50_321 = s->dpt[1][40]*ROcp50_320+s->dpt[2][40]*ROcp50_619+ROcp50_920*s->dpt[3][40];
OMcp50_121 = OMcp50_120+ROcp50_720*qd[21];
OMcp50_221 = OMcp50_220+ROcp50_820*qd[21];
OMcp50_321 = OMcp50_320+ROcp50_920*qd[21];
ORcp50_121 = OMcp50_220*RLcp50_321-OMcp50_320*RLcp50_221;
ORcp50_221 = -(OMcp50_120*RLcp50_321-OMcp50_320*RLcp50_121);
ORcp50_321 = OMcp50_120*RLcp50_221-OMcp50_220*RLcp50_121;
OPcp50_121 = OPcp50_120+ROcp50_720*qdd[21]+qd[21]*(OMcp50_220*ROcp50_920-OMcp50_320*ROcp50_820);
OPcp50_221 = OPcp50_220+ROcp50_820*qdd[21]-qd[21]*(OMcp50_120*ROcp50_920-OMcp50_320*ROcp50_720);
OPcp50_321 = OPcp50_320+ROcp50_920*qdd[21]+qd[21]*(OMcp50_120*ROcp50_820-OMcp50_220*ROcp50_720);

// = = Block_1_0_0_51_0_6 = = 
 
// Sensor Kinematics 


ROcp50_126 = ROcp50_121*C26-ROcp50_720*S26;
ROcp50_226 = ROcp50_221*C26-ROcp50_820*S26;
ROcp50_326 = ROcp50_321*C26-ROcp50_920*S26;
ROcp50_726 = ROcp50_121*S26+ROcp50_720*C26;
ROcp50_826 = ROcp50_221*S26+ROcp50_820*C26;
ROcp50_926 = ROcp50_321*S26+ROcp50_920*C26;
ROcp50_427 = ROcp50_421*C27+ROcp50_726*S27;
ROcp50_527 = ROcp50_521*C27+ROcp50_826*S27;
ROcp50_627 = ROcp50_621*C27+ROcp50_926*S27;
ROcp50_727 = -(ROcp50_421*S27-ROcp50_726*C27);
ROcp50_827 = -(ROcp50_521*S27-ROcp50_826*C27);
ROcp50_927 = -(ROcp50_621*S27-ROcp50_926*C27);
ROcp50_128 = ROcp50_126*C28+ROcp50_427*S28;
ROcp50_228 = ROcp50_226*C28+ROcp50_527*S28;
ROcp50_328 = ROcp50_326*C28+ROcp50_627*S28;
ROcp50_428 = -(ROcp50_126*S28-ROcp50_427*C28);
ROcp50_528 = -(ROcp50_226*S28-ROcp50_527*C28);
ROcp50_628 = -(ROcp50_326*S28-ROcp50_627*C28);
ROcp50_129 = ROcp50_128*C29-ROcp50_727*S29;
ROcp50_229 = ROcp50_228*C29-ROcp50_827*S29;
ROcp50_329 = ROcp50_328*C29-ROcp50_927*S29;
ROcp50_729 = ROcp50_128*S29+ROcp50_727*C29;
ROcp50_829 = ROcp50_228*S29+ROcp50_827*C29;
ROcp50_929 = ROcp50_328*S29+ROcp50_927*C29;
RLcp50_126 = ROcp50_121*s->dpt[1][45]+ROcp50_421*s->dpt[2][45]+ROcp50_720*s->dpt[3][45];
RLcp50_226 = ROcp50_221*s->dpt[1][45]+ROcp50_521*s->dpt[2][45]+ROcp50_820*s->dpt[3][45];
RLcp50_326 = ROcp50_321*s->dpt[1][45]+ROcp50_621*s->dpt[2][45]+ROcp50_920*s->dpt[3][45];
OMcp50_126 = OMcp50_121+ROcp50_421*qd[26];
OMcp50_226 = OMcp50_221+ROcp50_521*qd[26];
OMcp50_326 = OMcp50_321+ROcp50_621*qd[26];
ORcp50_126 = OMcp50_221*RLcp50_326-OMcp50_321*RLcp50_226;
ORcp50_226 = -(OMcp50_121*RLcp50_326-OMcp50_321*RLcp50_126);
ORcp50_326 = OMcp50_121*RLcp50_226-OMcp50_221*RLcp50_126;
OPcp50_126 = OPcp50_121+ROcp50_421*qdd[26]+qd[26]*(OMcp50_221*ROcp50_621-OMcp50_321*ROcp50_521);
OPcp50_226 = OPcp50_221+ROcp50_521*qdd[26]-qd[26]*(OMcp50_121*ROcp50_621-OMcp50_321*ROcp50_421);
OPcp50_326 = OPcp50_321+ROcp50_621*qdd[26]+qd[26]*(OMcp50_121*ROcp50_521-OMcp50_221*ROcp50_421);
RLcp50_127 = s->dpt[1][55]*ROcp50_126+s->dpt[3][55]*ROcp50_726+ROcp50_421*s->dpt[2][55];
RLcp50_227 = s->dpt[1][55]*ROcp50_226+s->dpt[3][55]*ROcp50_826+ROcp50_521*s->dpt[2][55];
RLcp50_327 = s->dpt[1][55]*ROcp50_326+s->dpt[3][55]*ROcp50_926+ROcp50_621*s->dpt[2][55];
OMcp50_127 = OMcp50_126+ROcp50_126*qd[27];
OMcp50_227 = OMcp50_226+ROcp50_226*qd[27];
OMcp50_327 = OMcp50_326+ROcp50_326*qd[27];
ORcp50_127 = OMcp50_226*RLcp50_327-OMcp50_326*RLcp50_227;
ORcp50_227 = -(OMcp50_126*RLcp50_327-OMcp50_326*RLcp50_127);
ORcp50_327 = OMcp50_126*RLcp50_227-OMcp50_226*RLcp50_127;
OPcp50_127 = OPcp50_126+ROcp50_126*qdd[27]+qd[27]*(OMcp50_226*ROcp50_326-OMcp50_326*ROcp50_226);
OPcp50_227 = OPcp50_226+ROcp50_226*qdd[27]-qd[27]*(OMcp50_126*ROcp50_326-OMcp50_326*ROcp50_126);
OPcp50_327 = OPcp50_326+ROcp50_326*qdd[27]+qd[27]*(OMcp50_126*ROcp50_226-OMcp50_226*ROcp50_126);
RLcp50_128 = s->dpt[1][57]*ROcp50_126+s->dpt[2][57]*ROcp50_427+ROcp50_727*s->dpt[3][57];
RLcp50_228 = s->dpt[1][57]*ROcp50_226+s->dpt[2][57]*ROcp50_527+ROcp50_827*s->dpt[3][57];
RLcp50_328 = s->dpt[1][57]*ROcp50_326+s->dpt[2][57]*ROcp50_627+ROcp50_927*s->dpt[3][57];
OMcp50_128 = OMcp50_127+ROcp50_727*qd[28];
OMcp50_228 = OMcp50_227+ROcp50_827*qd[28];
OMcp50_328 = OMcp50_327+ROcp50_927*qd[28];
ORcp50_128 = OMcp50_227*RLcp50_328-OMcp50_327*RLcp50_228;
ORcp50_228 = -(OMcp50_127*RLcp50_328-OMcp50_327*RLcp50_128);
ORcp50_328 = OMcp50_127*RLcp50_228-OMcp50_227*RLcp50_128;
OPcp50_128 = OPcp50_127+ROcp50_727*qdd[28]+qd[28]*(OMcp50_227*ROcp50_927-OMcp50_327*ROcp50_827);
OPcp50_228 = OPcp50_227+ROcp50_827*qdd[28]-qd[28]*(OMcp50_127*ROcp50_927-OMcp50_327*ROcp50_727);
OPcp50_328 = OPcp50_327+ROcp50_927*qdd[28]+qd[28]*(OMcp50_127*ROcp50_827-OMcp50_227*ROcp50_727);
RLcp50_129 = s->dpt[1][59]*ROcp50_128+s->dpt[2][59]*ROcp50_428+ROcp50_727*s->dpt[3][59];
RLcp50_229 = s->dpt[1][59]*ROcp50_228+s->dpt[2][59]*ROcp50_528+ROcp50_827*s->dpt[3][59];
RLcp50_329 = s->dpt[1][59]*ROcp50_328+s->dpt[2][59]*ROcp50_628+ROcp50_927*s->dpt[3][59];
OMcp50_129 = OMcp50_128+ROcp50_428*qd[29];
OMcp50_229 = OMcp50_228+ROcp50_528*qd[29];
OMcp50_329 = OMcp50_328+ROcp50_628*qd[29];
ORcp50_129 = OMcp50_228*RLcp50_329-OMcp50_328*RLcp50_229;
ORcp50_229 = -(OMcp50_128*RLcp50_329-OMcp50_328*RLcp50_129);
ORcp50_329 = OMcp50_128*RLcp50_229-OMcp50_228*RLcp50_129;
OPcp50_129 = OPcp50_128+ROcp50_428*qdd[29]+qd[29]*(OMcp50_228*ROcp50_628-OMcp50_328*ROcp50_528);
OPcp50_229 = OPcp50_228+ROcp50_528*qdd[29]-qd[29]*(OMcp50_128*ROcp50_628-OMcp50_328*ROcp50_428);
OPcp50_329 = OPcp50_328+ROcp50_628*qdd[29]+qd[29]*(OMcp50_128*ROcp50_528-OMcp50_228*ROcp50_428);
RLcp50_180 = s->dpt[1][61]*ROcp50_129+s->dpt[2][61]*ROcp50_428+ROcp50_729*s->dpt[3][61];
RLcp50_280 = s->dpt[1][61]*ROcp50_229+s->dpt[2][61]*ROcp50_528+ROcp50_829*s->dpt[3][61];
RLcp50_380 = s->dpt[1][61]*ROcp50_329+s->dpt[2][61]*ROcp50_628+ROcp50_929*s->dpt[3][61];
POcp50_180 = RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_126+RLcp50_127+RLcp50_128+RLcp50_129+RLcp50_180+q[1];
POcp50_280 = RLcp50_219+RLcp50_220+RLcp50_221+RLcp50_226+RLcp50_227+RLcp50_228+RLcp50_229+RLcp50_280+q[2];
POcp50_380 = RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380+q[3];
JTcp50_280_4 = -(RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380);
JTcp50_380_4 = RLcp50_219+RLcp50_220+RLcp50_221+RLcp50_226+RLcp50_227+RLcp50_228+RLcp50_229+RLcp50_280;
JTcp50_180_5 = C4*(RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380)-S4*(
 RLcp50_219+RLcp50_220)-S4*(RLcp50_221+RLcp50_226)-S4*(RLcp50_227+RLcp50_228)-S4*(RLcp50_229+RLcp50_280);
JTcp50_280_5 = S4*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_126+RLcp50_127+RLcp50_128+RLcp50_129+RLcp50_180);
JTcp50_380_5 = -C4*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_126+RLcp50_127+RLcp50_128+RLcp50_129+RLcp50_180);
JTcp50_180_6 = ROcp50_85*(RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380)-
 ROcp50_95*(RLcp50_219+RLcp50_220)-ROcp50_95*(RLcp50_221+RLcp50_226)-ROcp50_95*(RLcp50_227+RLcp50_228)-ROcp50_95*(RLcp50_229+
 RLcp50_280);
JTcp50_280_6 = RLcp50_180*ROcp50_95-RLcp50_329*S5-RLcp50_380*S5+ROcp50_95*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_126
 +RLcp50_127+RLcp50_128+RLcp50_129)-S5*(RLcp50_319+RLcp50_320)-S5*(RLcp50_321+RLcp50_326)-S5*(RLcp50_327+RLcp50_328);
JTcp50_380_6 = RLcp50_229*S5-ROcp50_85*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_126+RLcp50_127+RLcp50_128+RLcp50_129)+
 S5*(RLcp50_219+RLcp50_220)+S5*(RLcp50_221+RLcp50_226)+S5*(RLcp50_227+RLcp50_228)-RLcp50_180*ROcp50_85+RLcp50_280*S5;
JTcp50_180_7 = ROcp50_26*(RLcp50_320+RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329)-ROcp50_36*(RLcp50_220+
 RLcp50_221)-ROcp50_36*(RLcp50_226+RLcp50_227)-ROcp50_36*(RLcp50_228+RLcp50_229)-RLcp50_280*ROcp50_36+RLcp50_380*ROcp50_26;
JTcp50_280_7 = RLcp50_180*ROcp50_36-RLcp50_380*ROcp50_16-ROcp50_16*(RLcp50_320+RLcp50_321+RLcp50_326+RLcp50_327+
 RLcp50_328+RLcp50_329)+ROcp50_36*(RLcp50_120+RLcp50_121)+ROcp50_36*(RLcp50_126+RLcp50_127)+ROcp50_36*(RLcp50_128+RLcp50_129);
JTcp50_380_7 = ROcp50_16*(RLcp50_220+RLcp50_221+RLcp50_226+RLcp50_227+RLcp50_228+RLcp50_229)-ROcp50_26*(RLcp50_120+
 RLcp50_121)-ROcp50_26*(RLcp50_126+RLcp50_127)-ROcp50_26*(RLcp50_128+RLcp50_129)-RLcp50_180*ROcp50_26+RLcp50_280*ROcp50_16;
JTcp50_180_8 = ROcp50_519*(RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380)-ROcp50_619*(RLcp50_221+
 RLcp50_226)-ROcp50_619*(RLcp50_227+RLcp50_228)-ROcp50_619*(RLcp50_229+RLcp50_280);
JTcp50_280_8 = -(ROcp50_419*(RLcp50_321+RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380)-ROcp50_619*(RLcp50_121
 +RLcp50_126)-ROcp50_619*(RLcp50_127+RLcp50_128)-ROcp50_619*(RLcp50_129+RLcp50_180));
JTcp50_380_8 = ROcp50_419*(RLcp50_221+RLcp50_226+RLcp50_227+RLcp50_228+RLcp50_229+RLcp50_280)-ROcp50_519*(RLcp50_121+
 RLcp50_126)-ROcp50_519*(RLcp50_127+RLcp50_128)-ROcp50_519*(RLcp50_129+RLcp50_180);
JTcp50_180_9 = ROcp50_820*(RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329)-ROcp50_920*(RLcp50_226+RLcp50_227)-ROcp50_920*
 (RLcp50_228+RLcp50_229)-RLcp50_280*ROcp50_920+RLcp50_380*ROcp50_820;
JTcp50_280_9 = RLcp50_180*ROcp50_920-RLcp50_380*ROcp50_720-ROcp50_720*(RLcp50_326+RLcp50_327+RLcp50_328+RLcp50_329)+
 ROcp50_920*(RLcp50_126+RLcp50_127)+ROcp50_920*(RLcp50_128+RLcp50_129);
JTcp50_380_9 = ROcp50_720*(RLcp50_226+RLcp50_227+RLcp50_228+RLcp50_229)-ROcp50_820*(RLcp50_126+RLcp50_127)-ROcp50_820*
 (RLcp50_128+RLcp50_129)-RLcp50_180*ROcp50_820+RLcp50_280*ROcp50_720;
JTcp50_180_10 = ROcp50_521*(RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380)-ROcp50_621*(RLcp50_227+RLcp50_228)-ROcp50_621
 *(RLcp50_229+RLcp50_280);
JTcp50_280_10 = -(ROcp50_421*(RLcp50_327+RLcp50_328+RLcp50_329+RLcp50_380)-ROcp50_621*(RLcp50_127+RLcp50_128)-
 ROcp50_621*(RLcp50_129+RLcp50_180));
JTcp50_380_10 = ROcp50_421*(RLcp50_227+RLcp50_228+RLcp50_229+RLcp50_280)-ROcp50_521*(RLcp50_127+RLcp50_128)-ROcp50_521
 *(RLcp50_129+RLcp50_180);
JTcp50_180_11 = ROcp50_226*(RLcp50_328+RLcp50_329)-ROcp50_326*(RLcp50_228+RLcp50_229)-RLcp50_280*ROcp50_326+RLcp50_380
 *ROcp50_226;
JTcp50_280_11 = RLcp50_180*ROcp50_326-RLcp50_380*ROcp50_126-ROcp50_126*(RLcp50_328+RLcp50_329)+ROcp50_326*(RLcp50_128+
 RLcp50_129);
JTcp50_380_11 = ROcp50_126*(RLcp50_228+RLcp50_229)-ROcp50_226*(RLcp50_128+RLcp50_129)-RLcp50_180*ROcp50_226+RLcp50_280
 *ROcp50_126;
JTcp50_180_12 = ROcp50_827*(RLcp50_329+RLcp50_380)-ROcp50_927*(RLcp50_229+RLcp50_280);
JTcp50_280_12 = -(ROcp50_727*(RLcp50_329+RLcp50_380)-ROcp50_927*(RLcp50_129+RLcp50_180));
JTcp50_380_12 = ROcp50_727*(RLcp50_229+RLcp50_280)-ROcp50_827*(RLcp50_129+RLcp50_180);
JTcp50_180_13 = -(RLcp50_280*ROcp50_628-RLcp50_380*ROcp50_528);
JTcp50_280_13 = RLcp50_180*ROcp50_628-RLcp50_380*ROcp50_428;
JTcp50_380_13 = -(RLcp50_180*ROcp50_528-RLcp50_280*ROcp50_428);
ORcp50_180 = OMcp50_229*RLcp50_380-OMcp50_329*RLcp50_280;
ORcp50_280 = -(OMcp50_129*RLcp50_380-OMcp50_329*RLcp50_180);
ORcp50_380 = OMcp50_129*RLcp50_280-OMcp50_229*RLcp50_180;
VIcp50_180 = ORcp50_119+ORcp50_120+ORcp50_121+ORcp50_126+ORcp50_127+ORcp50_128+ORcp50_129+ORcp50_180+qd[1];
VIcp50_280 = ORcp50_219+ORcp50_220+ORcp50_221+ORcp50_226+ORcp50_227+ORcp50_228+ORcp50_229+ORcp50_280+qd[2];
VIcp50_380 = ORcp50_319+ORcp50_320+ORcp50_321+ORcp50_326+ORcp50_327+ORcp50_328+ORcp50_329+ORcp50_380+qd[3];
ACcp50_180 = qdd[1]+OMcp50_219*ORcp50_320+OMcp50_220*ORcp50_321+OMcp50_221*ORcp50_326+OMcp50_226*ORcp50_327+OMcp50_227
 *ORcp50_328+OMcp50_228*ORcp50_329+OMcp50_229*ORcp50_380+OMcp50_26*ORcp50_319-OMcp50_319*ORcp50_220-OMcp50_320*ORcp50_221-
 OMcp50_321*ORcp50_226-OMcp50_326*ORcp50_227-OMcp50_327*ORcp50_228-OMcp50_328*ORcp50_229-OMcp50_329*ORcp50_280-OMcp50_36*
 ORcp50_219+OPcp50_219*RLcp50_320+OPcp50_220*RLcp50_321+OPcp50_221*RLcp50_326+OPcp50_226*RLcp50_327+OPcp50_227*RLcp50_328+
 OPcp50_228*RLcp50_329+OPcp50_229*RLcp50_380+OPcp50_26*RLcp50_319-OPcp50_319*RLcp50_220-OPcp50_320*RLcp50_221-OPcp50_321*
 RLcp50_226-OPcp50_326*RLcp50_227-OPcp50_327*RLcp50_228-OPcp50_328*RLcp50_229-OPcp50_329*RLcp50_280-OPcp50_36*RLcp50_219;
ACcp50_280 = qdd[2]-OMcp50_119*ORcp50_320-OMcp50_120*ORcp50_321-OMcp50_121*ORcp50_326-OMcp50_126*ORcp50_327-OMcp50_127
 *ORcp50_328-OMcp50_128*ORcp50_329-OMcp50_129*ORcp50_380-OMcp50_16*ORcp50_319+OMcp50_319*ORcp50_120+OMcp50_320*ORcp50_121+
 OMcp50_321*ORcp50_126+OMcp50_326*ORcp50_127+OMcp50_327*ORcp50_128+OMcp50_328*ORcp50_129+OMcp50_329*ORcp50_180+OMcp50_36*
 ORcp50_119-OPcp50_119*RLcp50_320-OPcp50_120*RLcp50_321-OPcp50_121*RLcp50_326-OPcp50_126*RLcp50_327-OPcp50_127*RLcp50_328-
 OPcp50_128*RLcp50_329-OPcp50_129*RLcp50_380-OPcp50_16*RLcp50_319+OPcp50_319*RLcp50_120+OPcp50_320*RLcp50_121+OPcp50_321*
 RLcp50_126+OPcp50_326*RLcp50_127+OPcp50_327*RLcp50_128+OPcp50_328*RLcp50_129+OPcp50_329*RLcp50_180+OPcp50_36*RLcp50_119;
ACcp50_380 = qdd[3]+OMcp50_119*ORcp50_220+OMcp50_120*ORcp50_221+OMcp50_121*ORcp50_226+OMcp50_126*ORcp50_227+OMcp50_127
 *ORcp50_228+OMcp50_128*ORcp50_229+OMcp50_129*ORcp50_280+OMcp50_16*ORcp50_219-OMcp50_219*ORcp50_120-OMcp50_220*ORcp50_121-
 OMcp50_221*ORcp50_126-OMcp50_226*ORcp50_127-OMcp50_227*ORcp50_128-OMcp50_228*ORcp50_129-OMcp50_229*ORcp50_180-OMcp50_26*
 ORcp50_119+OPcp50_119*RLcp50_220+OPcp50_120*RLcp50_221+OPcp50_121*RLcp50_226+OPcp50_126*RLcp50_227+OPcp50_127*RLcp50_228+
 OPcp50_128*RLcp50_229+OPcp50_129*RLcp50_280+OPcp50_16*RLcp50_219-OPcp50_219*RLcp50_120-OPcp50_220*RLcp50_121-OPcp50_221*
 RLcp50_126-OPcp50_226*RLcp50_127-OPcp50_227*RLcp50_128-OPcp50_228*RLcp50_129-OPcp50_229*RLcp50_180-OPcp50_26*RLcp50_119;

// = = Block_1_0_0_51_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp50_180;
sens->P[2] = POcp50_280;
sens->P[3] = POcp50_380;
sens->R[1][1] = ROcp50_129;
sens->R[1][2] = ROcp50_229;
sens->R[1][3] = ROcp50_329;
sens->R[2][1] = ROcp50_428;
sens->R[2][2] = ROcp50_528;
sens->R[2][3] = ROcp50_628;
sens->R[3][1] = ROcp50_729;
sens->R[3][2] = ROcp50_829;
sens->R[3][3] = ROcp50_929;
sens->V[1] = VIcp50_180;
sens->V[2] = VIcp50_280;
sens->V[3] = VIcp50_380;
sens->OM[1] = OMcp50_129;
sens->OM[2] = OMcp50_229;
sens->OM[3] = OMcp50_329;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp50_180_5;
sens->J[1][6] = JTcp50_180_6;
sens->J[1][19] = JTcp50_180_7;
sens->J[1][20] = JTcp50_180_8;
sens->J[1][21] = JTcp50_180_9;
sens->J[1][26] = JTcp50_180_10;
sens->J[1][27] = JTcp50_180_11;
sens->J[1][28] = JTcp50_180_12;
sens->J[1][29] = JTcp50_180_13;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp50_280_4;
sens->J[2][5] = JTcp50_280_5;
sens->J[2][6] = JTcp50_280_6;
sens->J[2][19] = JTcp50_280_7;
sens->J[2][20] = JTcp50_280_8;
sens->J[2][21] = JTcp50_280_9;
sens->J[2][26] = JTcp50_280_10;
sens->J[2][27] = JTcp50_280_11;
sens->J[2][28] = JTcp50_280_12;
sens->J[2][29] = JTcp50_280_13;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp50_380_4;
sens->J[3][5] = JTcp50_380_5;
sens->J[3][6] = JTcp50_380_6;
sens->J[3][19] = JTcp50_380_7;
sens->J[3][20] = JTcp50_380_8;
sens->J[3][21] = JTcp50_380_9;
sens->J[3][26] = JTcp50_380_10;
sens->J[3][27] = JTcp50_380_11;
sens->J[3][28] = JTcp50_380_12;
sens->J[3][29] = JTcp50_380_13;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][19] = ROcp50_16;
sens->J[4][20] = ROcp50_419;
sens->J[4][21] = ROcp50_720;
sens->J[4][26] = ROcp50_421;
sens->J[4][27] = ROcp50_126;
sens->J[4][28] = ROcp50_727;
sens->J[4][29] = ROcp50_428;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp50_85;
sens->J[5][19] = ROcp50_26;
sens->J[5][20] = ROcp50_519;
sens->J[5][21] = ROcp50_820;
sens->J[5][26] = ROcp50_521;
sens->J[5][27] = ROcp50_226;
sens->J[5][28] = ROcp50_827;
sens->J[5][29] = ROcp50_528;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp50_95;
sens->J[6][19] = ROcp50_36;
sens->J[6][20] = ROcp50_619;
sens->J[6][21] = ROcp50_920;
sens->J[6][26] = ROcp50_621;
sens->J[6][27] = ROcp50_326;
sens->J[6][28] = ROcp50_927;
sens->J[6][29] = ROcp50_628;
sens->A[1] = ACcp50_180;
sens->A[2] = ACcp50_280;
sens->A[3] = ACcp50_380;
sens->OMP[1] = OPcp50_129;
sens->OMP[2] = OPcp50_229;
sens->OMP[3] = OPcp50_329;
 
// 
break;
case 52:
 


// = = Block_1_0_0_52_0_1 = = 
 
// Sensor Kinematics 


ROcp51_25 = S4*S5;
ROcp51_35 = -C4*S5;
ROcp51_85 = -S4*C5;
ROcp51_95 = C4*C5;
ROcp51_16 = C5*C6;
ROcp51_26 = ROcp51_25*C6+C4*S6;
ROcp51_36 = ROcp51_35*C6+S4*S6;
ROcp51_46 = -C5*S6;
ROcp51_56 = -(ROcp51_25*S6-C4*C6);
ROcp51_66 = -(ROcp51_35*S6-S4*C6);
OMcp51_25 = qd[5]*C4;
OMcp51_35 = qd[5]*S4;
OMcp51_16 = qd[4]+qd[6]*S5;
OMcp51_26 = OMcp51_25+ROcp51_85*qd[6];
OMcp51_36 = OMcp51_35+ROcp51_95*qd[6];
OPcp51_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp51_26 = ROcp51_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp51_35*S5-ROcp51_95*qd[4]);
OPcp51_36 = ROcp51_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp51_25*S5-ROcp51_85*qd[4]);

// = = Block_1_0_0_52_0_2 = = 
 
// Sensor Kinematics 


ROcp51_17 = ROcp51_16*C7-S5*S7;
ROcp51_27 = ROcp51_26*C7-ROcp51_85*S7;
ROcp51_37 = ROcp51_36*C7-ROcp51_95*S7;
ROcp51_77 = ROcp51_16*S7+S5*C7;
ROcp51_87 = ROcp51_26*S7+ROcp51_85*C7;
ROcp51_97 = ROcp51_36*S7+ROcp51_95*C7;
ROcp51_48 = ROcp51_46*C8+ROcp51_77*S8;
ROcp51_58 = ROcp51_56*C8+ROcp51_87*S8;
ROcp51_68 = ROcp51_66*C8+ROcp51_97*S8;
ROcp51_78 = -(ROcp51_46*S8-ROcp51_77*C8);
ROcp51_88 = -(ROcp51_56*S8-ROcp51_87*C8);
ROcp51_98 = -(ROcp51_66*S8-ROcp51_97*C8);
ROcp51_19 = ROcp51_17*C9+ROcp51_48*S9;
ROcp51_29 = ROcp51_27*C9+ROcp51_58*S9;
ROcp51_39 = ROcp51_37*C9+ROcp51_68*S9;
ROcp51_49 = -(ROcp51_17*S9-ROcp51_48*C9);
ROcp51_59 = -(ROcp51_27*S9-ROcp51_58*C9);
ROcp51_69 = -(ROcp51_37*S9-ROcp51_68*C9);
ROcp51_110 = ROcp51_19*C10-ROcp51_78*S10;
ROcp51_210 = ROcp51_29*C10-ROcp51_88*S10;
ROcp51_310 = ROcp51_39*C10-ROcp51_98*S10;
ROcp51_710 = ROcp51_19*S10+ROcp51_78*C10;
ROcp51_810 = ROcp51_29*S10+ROcp51_88*C10;
ROcp51_910 = ROcp51_39*S10+ROcp51_98*C10;
ROcp51_411 = ROcp51_49*C11+ROcp51_710*S11;
ROcp51_511 = ROcp51_59*C11+ROcp51_810*S11;
ROcp51_611 = ROcp51_69*C11+ROcp51_910*S11;
ROcp51_711 = -(ROcp51_49*S11-ROcp51_710*C11);
ROcp51_811 = -(ROcp51_59*S11-ROcp51_810*C11);
ROcp51_911 = -(ROcp51_69*S11-ROcp51_910*C11);
ROcp51_112 = ROcp51_110*C12-ROcp51_711*S12;
ROcp51_212 = ROcp51_210*C12-ROcp51_811*S12;
ROcp51_312 = ROcp51_310*C12-ROcp51_911*S12;
ROcp51_712 = ROcp51_110*S12+ROcp51_711*C12;
ROcp51_812 = ROcp51_210*S12+ROcp51_811*C12;
ROcp51_912 = ROcp51_310*S12+ROcp51_911*C12;
RLcp51_17 = s->dpt[1][1]*ROcp51_16+s->dpt[3][1]*S5+ROcp51_46*s->dpt[2][1];
RLcp51_27 = s->dpt[1][1]*ROcp51_26+s->dpt[3][1]*ROcp51_85+ROcp51_56*s->dpt[2][1];
RLcp51_37 = s->dpt[1][1]*ROcp51_36+s->dpt[3][1]*ROcp51_95+ROcp51_66*s->dpt[2][1];
OMcp51_17 = OMcp51_16+ROcp51_46*qd[7];
OMcp51_27 = OMcp51_26+ROcp51_56*qd[7];
OMcp51_37 = OMcp51_36+ROcp51_66*qd[7];
ORcp51_17 = OMcp51_26*RLcp51_37-OMcp51_36*RLcp51_27;
ORcp51_27 = -(OMcp51_16*RLcp51_37-OMcp51_36*RLcp51_17);
ORcp51_37 = OMcp51_16*RLcp51_27-OMcp51_26*RLcp51_17;
OPcp51_17 = OPcp51_16+ROcp51_46*qdd[7]+qd[7]*(OMcp51_26*ROcp51_66-OMcp51_36*ROcp51_56);
OPcp51_27 = OPcp51_26+ROcp51_56*qdd[7]-qd[7]*(OMcp51_16*ROcp51_66-OMcp51_36*ROcp51_46);
OPcp51_37 = OPcp51_36+ROcp51_66*qdd[7]+qd[7]*(OMcp51_16*ROcp51_56-OMcp51_26*ROcp51_46);
RLcp51_18 = s->dpt[1][6]*ROcp51_17+s->dpt[3][6]*ROcp51_77+ROcp51_46*s->dpt[2][6];
RLcp51_28 = s->dpt[1][6]*ROcp51_27+s->dpt[3][6]*ROcp51_87+ROcp51_56*s->dpt[2][6];
RLcp51_38 = s->dpt[1][6]*ROcp51_37+s->dpt[3][6]*ROcp51_97+ROcp51_66*s->dpt[2][6];
OMcp51_18 = OMcp51_17+ROcp51_17*qd[8];
OMcp51_28 = OMcp51_27+ROcp51_27*qd[8];
OMcp51_38 = OMcp51_37+ROcp51_37*qd[8];
ORcp51_18 = OMcp51_27*RLcp51_38-OMcp51_37*RLcp51_28;
ORcp51_28 = -(OMcp51_17*RLcp51_38-OMcp51_37*RLcp51_18);
ORcp51_38 = OMcp51_17*RLcp51_28-OMcp51_27*RLcp51_18;
OPcp51_18 = OPcp51_17+ROcp51_17*qdd[8]+qd[8]*(OMcp51_27*ROcp51_37-OMcp51_37*ROcp51_27);
OPcp51_28 = OPcp51_27+ROcp51_27*qdd[8]-qd[8]*(OMcp51_17*ROcp51_37-OMcp51_37*ROcp51_17);
OPcp51_38 = OPcp51_37+ROcp51_37*qdd[8]+qd[8]*(OMcp51_17*ROcp51_27-OMcp51_27*ROcp51_17);
RLcp51_19 = s->dpt[1][8]*ROcp51_17+s->dpt[2][8]*ROcp51_48+ROcp51_78*s->dpt[3][8];
RLcp51_29 = s->dpt[1][8]*ROcp51_27+s->dpt[2][8]*ROcp51_58+ROcp51_88*s->dpt[3][8];
RLcp51_39 = s->dpt[1][8]*ROcp51_37+s->dpt[2][8]*ROcp51_68+ROcp51_98*s->dpt[3][8];
OMcp51_19 = OMcp51_18+ROcp51_78*qd[9];
OMcp51_29 = OMcp51_28+ROcp51_88*qd[9];
OMcp51_39 = OMcp51_38+ROcp51_98*qd[9];
ORcp51_19 = OMcp51_28*RLcp51_39-OMcp51_38*RLcp51_29;
ORcp51_29 = -(OMcp51_18*RLcp51_39-OMcp51_38*RLcp51_19);
ORcp51_39 = OMcp51_18*RLcp51_29-OMcp51_28*RLcp51_19;
OPcp51_19 = OPcp51_18+ROcp51_78*qdd[9]+qd[9]*(OMcp51_28*ROcp51_98-OMcp51_38*ROcp51_88);
OPcp51_29 = OPcp51_28+ROcp51_88*qdd[9]-qd[9]*(OMcp51_18*ROcp51_98-OMcp51_38*ROcp51_78);
OPcp51_39 = OPcp51_38+ROcp51_98*qdd[9]+qd[9]*(OMcp51_18*ROcp51_88-OMcp51_28*ROcp51_78);
RLcp51_110 = s->dpt[1][10]*ROcp51_19+s->dpt[2][10]*ROcp51_49+ROcp51_78*s->dpt[3][10];
RLcp51_210 = s->dpt[1][10]*ROcp51_29+s->dpt[2][10]*ROcp51_59+ROcp51_88*s->dpt[3][10];
RLcp51_310 = s->dpt[1][10]*ROcp51_39+s->dpt[2][10]*ROcp51_69+ROcp51_98*s->dpt[3][10];
OMcp51_110 = OMcp51_19+ROcp51_49*qd[10];
OMcp51_210 = OMcp51_29+ROcp51_59*qd[10];
OMcp51_310 = OMcp51_39+ROcp51_69*qd[10];
ORcp51_110 = OMcp51_29*RLcp51_310-OMcp51_39*RLcp51_210;
ORcp51_210 = -(OMcp51_19*RLcp51_310-OMcp51_39*RLcp51_110);
ORcp51_310 = OMcp51_19*RLcp51_210-OMcp51_29*RLcp51_110;
OPcp51_110 = OPcp51_19+ROcp51_49*qdd[10]+qd[10]*(OMcp51_29*ROcp51_69-OMcp51_39*ROcp51_59);
OPcp51_210 = OPcp51_29+ROcp51_59*qdd[10]-qd[10]*(OMcp51_19*ROcp51_69-OMcp51_39*ROcp51_49);
OPcp51_310 = OPcp51_39+ROcp51_69*qdd[10]+qd[10]*(OMcp51_19*ROcp51_59-OMcp51_29*ROcp51_49);
RLcp51_111 = s->dpt[1][12]*ROcp51_110+s->dpt[2][12]*ROcp51_49+ROcp51_710*s->dpt[3][12];
RLcp51_211 = s->dpt[1][12]*ROcp51_210+s->dpt[2][12]*ROcp51_59+ROcp51_810*s->dpt[3][12];
RLcp51_311 = s->dpt[1][12]*ROcp51_310+s->dpt[2][12]*ROcp51_69+ROcp51_910*s->dpt[3][12];
OMcp51_111 = OMcp51_110+ROcp51_110*qd[11];
OMcp51_211 = OMcp51_210+ROcp51_210*qd[11];
OMcp51_311 = OMcp51_310+ROcp51_310*qd[11];
ORcp51_111 = OMcp51_210*RLcp51_311-OMcp51_310*RLcp51_211;
ORcp51_211 = -(OMcp51_110*RLcp51_311-OMcp51_310*RLcp51_111);
ORcp51_311 = OMcp51_110*RLcp51_211-OMcp51_210*RLcp51_111;
OPcp51_111 = OPcp51_110+ROcp51_110*qdd[11]+qd[11]*(OMcp51_210*ROcp51_310-OMcp51_310*ROcp51_210);
OPcp51_211 = OPcp51_210+ROcp51_210*qdd[11]-qd[11]*(OMcp51_110*ROcp51_310-OMcp51_310*ROcp51_110);
OPcp51_311 = OPcp51_310+ROcp51_310*qdd[11]+qd[11]*(OMcp51_110*ROcp51_210-OMcp51_210*ROcp51_110);
RLcp51_112 = s->dpt[1][14]*ROcp51_110+s->dpt[2][14]*ROcp51_411+s->dpt[3][14]*ROcp51_711;
RLcp51_212 = s->dpt[1][14]*ROcp51_210+s->dpt[2][14]*ROcp51_511+s->dpt[3][14]*ROcp51_811;
RLcp51_312 = s->dpt[1][14]*ROcp51_310+s->dpt[2][14]*ROcp51_611+s->dpt[3][14]*ROcp51_911;
OMcp51_112 = OMcp51_111+ROcp51_411*qd[12];
OMcp51_212 = OMcp51_211+ROcp51_511*qd[12];
OMcp51_312 = OMcp51_311+ROcp51_611*qd[12];
ORcp51_112 = OMcp51_211*RLcp51_312-OMcp51_311*RLcp51_212;
ORcp51_212 = -(OMcp51_111*RLcp51_312-OMcp51_311*RLcp51_112);
ORcp51_312 = OMcp51_111*RLcp51_212-OMcp51_211*RLcp51_112;
OPcp51_112 = OPcp51_111+ROcp51_411*qdd[12]+qd[12]*(OMcp51_211*ROcp51_611-OMcp51_311*ROcp51_511);
OPcp51_212 = OPcp51_211+ROcp51_511*qdd[12]-qd[12]*(OMcp51_111*ROcp51_611-OMcp51_311*ROcp51_411);
OPcp51_312 = OPcp51_311+ROcp51_611*qdd[12]+qd[12]*(OMcp51_111*ROcp51_511-OMcp51_211*ROcp51_411);
RLcp51_181 = s->dpt[1][16]*ROcp51_112+s->dpt[2][16]*ROcp51_411+ROcp51_712*s->dpt[3][16];
RLcp51_281 = s->dpt[1][16]*ROcp51_212+s->dpt[2][16]*ROcp51_511+ROcp51_812*s->dpt[3][16];
RLcp51_381 = s->dpt[1][16]*ROcp51_312+s->dpt[2][16]*ROcp51_611+ROcp51_912*s->dpt[3][16];
POcp51_181 = RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_181+RLcp51_19+q[1];
POcp51_281 = RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_27+RLcp51_28+RLcp51_281+RLcp51_29+q[2];
POcp51_381 = RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_381+RLcp51_39+q[3];
JTcp51_281_4 = -(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_381+RLcp51_39);
JTcp51_381_4 = RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_27+RLcp51_28+RLcp51_281+RLcp51_29;
JTcp51_181_5 = C4*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_39)-S4*(RLcp51_210+RLcp51_29)-S4*(
 RLcp51_211+RLcp51_212)-S4*(RLcp51_27+RLcp51_28)-RLcp51_281*S4+RLcp51_381*C4;
JTcp51_281_5 = S4*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_181+RLcp51_19);
JTcp51_381_5 = -C4*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_181+RLcp51_19);
JTcp51_181_6 = ROcp51_85*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_39)-ROcp51_95*(RLcp51_210+
 RLcp51_29)-ROcp51_95*(RLcp51_211+RLcp51_212)-ROcp51_95*(RLcp51_27+RLcp51_28)-RLcp51_281*ROcp51_95+RLcp51_381*ROcp51_85;
JTcp51_281_6 = -(RLcp51_381*S5-ROcp51_95*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_181+RLcp51_19)+
 S5*(RLcp51_310+RLcp51_39)+S5*(RLcp51_311+RLcp51_312)+S5*(RLcp51_37+RLcp51_38));
JTcp51_381_6 = RLcp51_281*S5-ROcp51_85*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_181+RLcp51_19)+S5*
 (RLcp51_210+RLcp51_29)+S5*(RLcp51_211+RLcp51_212)+S5*(RLcp51_27+RLcp51_28);
JTcp51_181_7 = ROcp51_56*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_38+RLcp51_381+RLcp51_39)-ROcp51_66*(RLcp51_210+
 RLcp51_211)-ROcp51_66*(RLcp51_212+RLcp51_281)-ROcp51_66*(RLcp51_28+RLcp51_29);
JTcp51_281_7 = -(ROcp51_46*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_38+RLcp51_381+RLcp51_39)-ROcp51_66*(RLcp51_110+
 RLcp51_111)-ROcp51_66*(RLcp51_112+RLcp51_181)-ROcp51_66*(RLcp51_18+RLcp51_19));
JTcp51_381_7 = ROcp51_46*(RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_28+RLcp51_281+RLcp51_29)-ROcp51_56*(RLcp51_110+
 RLcp51_111)-ROcp51_56*(RLcp51_112+RLcp51_181)-ROcp51_56*(RLcp51_18+RLcp51_19);
JTcp51_181_8 = ROcp51_27*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_39)-ROcp51_37*(RLcp51_210+RLcp51_29)-ROcp51_37*(
 RLcp51_211+RLcp51_212)-RLcp51_281*ROcp51_37+RLcp51_381*ROcp51_27;
JTcp51_281_8 = RLcp51_181*ROcp51_37-RLcp51_381*ROcp51_17-ROcp51_17*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_39)+
 ROcp51_37*(RLcp51_110+RLcp51_19)+ROcp51_37*(RLcp51_111+RLcp51_112);
JTcp51_381_8 = ROcp51_17*(RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_29)-ROcp51_27*(RLcp51_110+RLcp51_19)-ROcp51_27*(
 RLcp51_111+RLcp51_112)-RLcp51_181*ROcp51_27+RLcp51_281*ROcp51_17;
JTcp51_181_9 = ROcp51_88*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_381)-ROcp51_98*(RLcp51_210+RLcp51_211)-ROcp51_98*(
 RLcp51_212+RLcp51_281);
JTcp51_281_9 = -(ROcp51_78*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_381)-ROcp51_98*(RLcp51_110+RLcp51_111)-ROcp51_98*(
 RLcp51_112+RLcp51_181));
JTcp51_381_9 = ROcp51_78*(RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_281)-ROcp51_88*(RLcp51_110+RLcp51_111)-ROcp51_88*(
 RLcp51_112+RLcp51_181);
JTcp51_181_10 = ROcp51_59*(RLcp51_311+RLcp51_312)-ROcp51_69*(RLcp51_211+RLcp51_212)-RLcp51_281*ROcp51_69+RLcp51_381*
 ROcp51_59;
JTcp51_281_10 = RLcp51_181*ROcp51_69-RLcp51_381*ROcp51_49-ROcp51_49*(RLcp51_311+RLcp51_312)+ROcp51_69*(RLcp51_111+
 RLcp51_112);
JTcp51_381_10 = ROcp51_49*(RLcp51_211+RLcp51_212)-ROcp51_59*(RLcp51_111+RLcp51_112)-RLcp51_181*ROcp51_59+RLcp51_281*
 ROcp51_49;
JTcp51_181_11 = ROcp51_210*(RLcp51_312+RLcp51_381)-ROcp51_310*(RLcp51_212+RLcp51_281);
JTcp51_281_11 = -(ROcp51_110*(RLcp51_312+RLcp51_381)-ROcp51_310*(RLcp51_112+RLcp51_181));
JTcp51_381_11 = ROcp51_110*(RLcp51_212+RLcp51_281)-ROcp51_210*(RLcp51_112+RLcp51_181);
JTcp51_181_12 = -(RLcp51_281*ROcp51_611-RLcp51_381*ROcp51_511);
JTcp51_281_12 = RLcp51_181*ROcp51_611-RLcp51_381*ROcp51_411;
JTcp51_381_12 = -(RLcp51_181*ROcp51_511-RLcp51_281*ROcp51_411);
ORcp51_181 = OMcp51_212*RLcp51_381-OMcp51_312*RLcp51_281;
ORcp51_281 = -(OMcp51_112*RLcp51_381-OMcp51_312*RLcp51_181);
ORcp51_381 = OMcp51_112*RLcp51_281-OMcp51_212*RLcp51_181;
VIcp51_181 = ORcp51_110+ORcp51_111+ORcp51_112+ORcp51_17+ORcp51_18+ORcp51_181+ORcp51_19+qd[1];
VIcp51_281 = ORcp51_210+ORcp51_211+ORcp51_212+ORcp51_27+ORcp51_28+ORcp51_281+ORcp51_29+qd[2];
VIcp51_381 = ORcp51_310+ORcp51_311+ORcp51_312+ORcp51_37+ORcp51_38+ORcp51_381+ORcp51_39+qd[3];
ACcp51_181 = qdd[1]+OMcp51_210*ORcp51_311+OMcp51_211*ORcp51_312+OMcp51_212*ORcp51_381+OMcp51_26*ORcp51_37+OMcp51_27*
 ORcp51_38+OMcp51_28*ORcp51_39+OMcp51_29*ORcp51_310-OMcp51_310*ORcp51_211-OMcp51_311*ORcp51_212-OMcp51_312*ORcp51_281-
 OMcp51_36*ORcp51_27-OMcp51_37*ORcp51_28-OMcp51_38*ORcp51_29-OMcp51_39*ORcp51_210+OPcp51_210*RLcp51_311+OPcp51_211*RLcp51_312
 +OPcp51_212*RLcp51_381+OPcp51_26*RLcp51_37+OPcp51_27*RLcp51_38+OPcp51_28*RLcp51_39+OPcp51_29*RLcp51_310-OPcp51_310*
 RLcp51_211-OPcp51_311*RLcp51_212-OPcp51_312*RLcp51_281-OPcp51_36*RLcp51_27-OPcp51_37*RLcp51_28-OPcp51_38*RLcp51_29-OPcp51_39
 *RLcp51_210;
ACcp51_281 = qdd[2]-OMcp51_110*ORcp51_311-OMcp51_111*ORcp51_312-OMcp51_112*ORcp51_381-OMcp51_16*ORcp51_37-OMcp51_17*
 ORcp51_38-OMcp51_18*ORcp51_39-OMcp51_19*ORcp51_310+OMcp51_310*ORcp51_111+OMcp51_311*ORcp51_112+OMcp51_312*ORcp51_181+
 OMcp51_36*ORcp51_17+OMcp51_37*ORcp51_18+OMcp51_38*ORcp51_19+OMcp51_39*ORcp51_110-OPcp51_110*RLcp51_311-OPcp51_111*RLcp51_312
 -OPcp51_112*RLcp51_381-OPcp51_16*RLcp51_37-OPcp51_17*RLcp51_38-OPcp51_18*RLcp51_39-OPcp51_19*RLcp51_310+OPcp51_310*
 RLcp51_111+OPcp51_311*RLcp51_112+OPcp51_312*RLcp51_181+OPcp51_36*RLcp51_17+OPcp51_37*RLcp51_18+OPcp51_38*RLcp51_19+OPcp51_39
 *RLcp51_110;
ACcp51_381 = qdd[3]+OMcp51_110*ORcp51_211+OMcp51_111*ORcp51_212+OMcp51_112*ORcp51_281+OMcp51_16*ORcp51_27+OMcp51_17*
 ORcp51_28+OMcp51_18*ORcp51_29+OMcp51_19*ORcp51_210-OMcp51_210*ORcp51_111-OMcp51_211*ORcp51_112-OMcp51_212*ORcp51_181-
 OMcp51_26*ORcp51_17-OMcp51_27*ORcp51_18-OMcp51_28*ORcp51_19-OMcp51_29*ORcp51_110+OPcp51_110*RLcp51_211+OPcp51_111*RLcp51_212
 +OPcp51_112*RLcp51_281+OPcp51_16*RLcp51_27+OPcp51_17*RLcp51_28+OPcp51_18*RLcp51_29+OPcp51_19*RLcp51_210-OPcp51_210*
 RLcp51_111-OPcp51_211*RLcp51_112-OPcp51_212*RLcp51_181-OPcp51_26*RLcp51_17-OPcp51_27*RLcp51_18-OPcp51_28*RLcp51_19-OPcp51_29
 *RLcp51_110;

// = = Block_1_0_0_52_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp51_181;
sens->P[2] = POcp51_281;
sens->P[3] = POcp51_381;
sens->R[1][1] = ROcp51_112;
sens->R[1][2] = ROcp51_212;
sens->R[1][3] = ROcp51_312;
sens->R[2][1] = ROcp51_411;
sens->R[2][2] = ROcp51_511;
sens->R[2][3] = ROcp51_611;
sens->R[3][1] = ROcp51_712;
sens->R[3][2] = ROcp51_812;
sens->R[3][3] = ROcp51_912;
sens->V[1] = VIcp51_181;
sens->V[2] = VIcp51_281;
sens->V[3] = VIcp51_381;
sens->OM[1] = OMcp51_112;
sens->OM[2] = OMcp51_212;
sens->OM[3] = OMcp51_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp51_181_5;
sens->J[1][6] = JTcp51_181_6;
sens->J[1][7] = JTcp51_181_7;
sens->J[1][8] = JTcp51_181_8;
sens->J[1][9] = JTcp51_181_9;
sens->J[1][10] = JTcp51_181_10;
sens->J[1][11] = JTcp51_181_11;
sens->J[1][12] = JTcp51_181_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp51_281_4;
sens->J[2][5] = JTcp51_281_5;
sens->J[2][6] = JTcp51_281_6;
sens->J[2][7] = JTcp51_281_7;
sens->J[2][8] = JTcp51_281_8;
sens->J[2][9] = JTcp51_281_9;
sens->J[2][10] = JTcp51_281_10;
sens->J[2][11] = JTcp51_281_11;
sens->J[2][12] = JTcp51_281_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp51_381_4;
sens->J[3][5] = JTcp51_381_5;
sens->J[3][6] = JTcp51_381_6;
sens->J[3][7] = JTcp51_381_7;
sens->J[3][8] = JTcp51_381_8;
sens->J[3][9] = JTcp51_381_9;
sens->J[3][10] = JTcp51_381_10;
sens->J[3][11] = JTcp51_381_11;
sens->J[3][12] = JTcp51_381_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp51_46;
sens->J[4][8] = ROcp51_17;
sens->J[4][9] = ROcp51_78;
sens->J[4][10] = ROcp51_49;
sens->J[4][11] = ROcp51_110;
sens->J[4][12] = ROcp51_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp51_85;
sens->J[5][7] = ROcp51_56;
sens->J[5][8] = ROcp51_27;
sens->J[5][9] = ROcp51_88;
sens->J[5][10] = ROcp51_59;
sens->J[5][11] = ROcp51_210;
sens->J[5][12] = ROcp51_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp51_95;
sens->J[6][7] = ROcp51_66;
sens->J[6][8] = ROcp51_37;
sens->J[6][9] = ROcp51_98;
sens->J[6][10] = ROcp51_69;
sens->J[6][11] = ROcp51_310;
sens->J[6][12] = ROcp51_611;
sens->A[1] = ACcp51_181;
sens->A[2] = ACcp51_281;
sens->A[3] = ACcp51_381;
sens->OMP[1] = OPcp51_112;
sens->OMP[2] = OPcp51_212;
sens->OMP[3] = OPcp51_312;
 
// 
break;
case 53:
 


// = = Block_1_0_0_53_0_1 = = 
 
// Sensor Kinematics 


ROcp52_25 = S4*S5;
ROcp52_35 = -C4*S5;
ROcp52_85 = -S4*C5;
ROcp52_95 = C4*C5;
ROcp52_16 = C5*C6;
ROcp52_26 = ROcp52_25*C6+C4*S6;
ROcp52_36 = ROcp52_35*C6+S4*S6;
ROcp52_46 = -C5*S6;
ROcp52_56 = -(ROcp52_25*S6-C4*C6);
ROcp52_66 = -(ROcp52_35*S6-S4*C6);
OMcp52_25 = qd[5]*C4;
OMcp52_35 = qd[5]*S4;
OMcp52_16 = qd[4]+qd[6]*S5;
OMcp52_26 = OMcp52_25+ROcp52_85*qd[6];
OMcp52_36 = OMcp52_35+ROcp52_95*qd[6];
OPcp52_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp52_26 = ROcp52_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp52_35*S5-ROcp52_95*qd[4]);
OPcp52_36 = ROcp52_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp52_25*S5-ROcp52_85*qd[4]);

// = = Block_1_0_0_53_0_2 = = 
 
// Sensor Kinematics 


ROcp52_17 = ROcp52_16*C7-S5*S7;
ROcp52_27 = ROcp52_26*C7-ROcp52_85*S7;
ROcp52_37 = ROcp52_36*C7-ROcp52_95*S7;
ROcp52_77 = ROcp52_16*S7+S5*C7;
ROcp52_87 = ROcp52_26*S7+ROcp52_85*C7;
ROcp52_97 = ROcp52_36*S7+ROcp52_95*C7;
ROcp52_48 = ROcp52_46*C8+ROcp52_77*S8;
ROcp52_58 = ROcp52_56*C8+ROcp52_87*S8;
ROcp52_68 = ROcp52_66*C8+ROcp52_97*S8;
ROcp52_78 = -(ROcp52_46*S8-ROcp52_77*C8);
ROcp52_88 = -(ROcp52_56*S8-ROcp52_87*C8);
ROcp52_98 = -(ROcp52_66*S8-ROcp52_97*C8);
ROcp52_19 = ROcp52_17*C9+ROcp52_48*S9;
ROcp52_29 = ROcp52_27*C9+ROcp52_58*S9;
ROcp52_39 = ROcp52_37*C9+ROcp52_68*S9;
ROcp52_49 = -(ROcp52_17*S9-ROcp52_48*C9);
ROcp52_59 = -(ROcp52_27*S9-ROcp52_58*C9);
ROcp52_69 = -(ROcp52_37*S9-ROcp52_68*C9);
ROcp52_110 = ROcp52_19*C10-ROcp52_78*S10;
ROcp52_210 = ROcp52_29*C10-ROcp52_88*S10;
ROcp52_310 = ROcp52_39*C10-ROcp52_98*S10;
ROcp52_710 = ROcp52_19*S10+ROcp52_78*C10;
ROcp52_810 = ROcp52_29*S10+ROcp52_88*C10;
ROcp52_910 = ROcp52_39*S10+ROcp52_98*C10;
ROcp52_411 = ROcp52_49*C11+ROcp52_710*S11;
ROcp52_511 = ROcp52_59*C11+ROcp52_810*S11;
ROcp52_611 = ROcp52_69*C11+ROcp52_910*S11;
ROcp52_711 = -(ROcp52_49*S11-ROcp52_710*C11);
ROcp52_811 = -(ROcp52_59*S11-ROcp52_810*C11);
ROcp52_911 = -(ROcp52_69*S11-ROcp52_910*C11);
ROcp52_112 = ROcp52_110*C12-ROcp52_711*S12;
ROcp52_212 = ROcp52_210*C12-ROcp52_811*S12;
ROcp52_312 = ROcp52_310*C12-ROcp52_911*S12;
ROcp52_712 = ROcp52_110*S12+ROcp52_711*C12;
ROcp52_812 = ROcp52_210*S12+ROcp52_811*C12;
ROcp52_912 = ROcp52_310*S12+ROcp52_911*C12;
RLcp52_17 = s->dpt[1][1]*ROcp52_16+s->dpt[3][1]*S5+ROcp52_46*s->dpt[2][1];
RLcp52_27 = s->dpt[1][1]*ROcp52_26+s->dpt[3][1]*ROcp52_85+ROcp52_56*s->dpt[2][1];
RLcp52_37 = s->dpt[1][1]*ROcp52_36+s->dpt[3][1]*ROcp52_95+ROcp52_66*s->dpt[2][1];
OMcp52_17 = OMcp52_16+ROcp52_46*qd[7];
OMcp52_27 = OMcp52_26+ROcp52_56*qd[7];
OMcp52_37 = OMcp52_36+ROcp52_66*qd[7];
ORcp52_17 = OMcp52_26*RLcp52_37-OMcp52_36*RLcp52_27;
ORcp52_27 = -(OMcp52_16*RLcp52_37-OMcp52_36*RLcp52_17);
ORcp52_37 = OMcp52_16*RLcp52_27-OMcp52_26*RLcp52_17;
OPcp52_17 = OPcp52_16+ROcp52_46*qdd[7]+qd[7]*(OMcp52_26*ROcp52_66-OMcp52_36*ROcp52_56);
OPcp52_27 = OPcp52_26+ROcp52_56*qdd[7]-qd[7]*(OMcp52_16*ROcp52_66-OMcp52_36*ROcp52_46);
OPcp52_37 = OPcp52_36+ROcp52_66*qdd[7]+qd[7]*(OMcp52_16*ROcp52_56-OMcp52_26*ROcp52_46);
RLcp52_18 = s->dpt[1][6]*ROcp52_17+s->dpt[3][6]*ROcp52_77+ROcp52_46*s->dpt[2][6];
RLcp52_28 = s->dpt[1][6]*ROcp52_27+s->dpt[3][6]*ROcp52_87+ROcp52_56*s->dpt[2][6];
RLcp52_38 = s->dpt[1][6]*ROcp52_37+s->dpt[3][6]*ROcp52_97+ROcp52_66*s->dpt[2][6];
OMcp52_18 = OMcp52_17+ROcp52_17*qd[8];
OMcp52_28 = OMcp52_27+ROcp52_27*qd[8];
OMcp52_38 = OMcp52_37+ROcp52_37*qd[8];
ORcp52_18 = OMcp52_27*RLcp52_38-OMcp52_37*RLcp52_28;
ORcp52_28 = -(OMcp52_17*RLcp52_38-OMcp52_37*RLcp52_18);
ORcp52_38 = OMcp52_17*RLcp52_28-OMcp52_27*RLcp52_18;
OPcp52_18 = OPcp52_17+ROcp52_17*qdd[8]+qd[8]*(OMcp52_27*ROcp52_37-OMcp52_37*ROcp52_27);
OPcp52_28 = OPcp52_27+ROcp52_27*qdd[8]-qd[8]*(OMcp52_17*ROcp52_37-OMcp52_37*ROcp52_17);
OPcp52_38 = OPcp52_37+ROcp52_37*qdd[8]+qd[8]*(OMcp52_17*ROcp52_27-OMcp52_27*ROcp52_17);
RLcp52_19 = s->dpt[1][8]*ROcp52_17+s->dpt[2][8]*ROcp52_48+ROcp52_78*s->dpt[3][8];
RLcp52_29 = s->dpt[1][8]*ROcp52_27+s->dpt[2][8]*ROcp52_58+ROcp52_88*s->dpt[3][8];
RLcp52_39 = s->dpt[1][8]*ROcp52_37+s->dpt[2][8]*ROcp52_68+ROcp52_98*s->dpt[3][8];
OMcp52_19 = OMcp52_18+ROcp52_78*qd[9];
OMcp52_29 = OMcp52_28+ROcp52_88*qd[9];
OMcp52_39 = OMcp52_38+ROcp52_98*qd[9];
ORcp52_19 = OMcp52_28*RLcp52_39-OMcp52_38*RLcp52_29;
ORcp52_29 = -(OMcp52_18*RLcp52_39-OMcp52_38*RLcp52_19);
ORcp52_39 = OMcp52_18*RLcp52_29-OMcp52_28*RLcp52_19;
OPcp52_19 = OPcp52_18+ROcp52_78*qdd[9]+qd[9]*(OMcp52_28*ROcp52_98-OMcp52_38*ROcp52_88);
OPcp52_29 = OPcp52_28+ROcp52_88*qdd[9]-qd[9]*(OMcp52_18*ROcp52_98-OMcp52_38*ROcp52_78);
OPcp52_39 = OPcp52_38+ROcp52_98*qdd[9]+qd[9]*(OMcp52_18*ROcp52_88-OMcp52_28*ROcp52_78);
RLcp52_110 = s->dpt[1][10]*ROcp52_19+s->dpt[2][10]*ROcp52_49+ROcp52_78*s->dpt[3][10];
RLcp52_210 = s->dpt[1][10]*ROcp52_29+s->dpt[2][10]*ROcp52_59+ROcp52_88*s->dpt[3][10];
RLcp52_310 = s->dpt[1][10]*ROcp52_39+s->dpt[2][10]*ROcp52_69+ROcp52_98*s->dpt[3][10];
OMcp52_110 = OMcp52_19+ROcp52_49*qd[10];
OMcp52_210 = OMcp52_29+ROcp52_59*qd[10];
OMcp52_310 = OMcp52_39+ROcp52_69*qd[10];
ORcp52_110 = OMcp52_29*RLcp52_310-OMcp52_39*RLcp52_210;
ORcp52_210 = -(OMcp52_19*RLcp52_310-OMcp52_39*RLcp52_110);
ORcp52_310 = OMcp52_19*RLcp52_210-OMcp52_29*RLcp52_110;
OPcp52_110 = OPcp52_19+ROcp52_49*qdd[10]+qd[10]*(OMcp52_29*ROcp52_69-OMcp52_39*ROcp52_59);
OPcp52_210 = OPcp52_29+ROcp52_59*qdd[10]-qd[10]*(OMcp52_19*ROcp52_69-OMcp52_39*ROcp52_49);
OPcp52_310 = OPcp52_39+ROcp52_69*qdd[10]+qd[10]*(OMcp52_19*ROcp52_59-OMcp52_29*ROcp52_49);
RLcp52_111 = s->dpt[1][12]*ROcp52_110+s->dpt[2][12]*ROcp52_49+ROcp52_710*s->dpt[3][12];
RLcp52_211 = s->dpt[1][12]*ROcp52_210+s->dpt[2][12]*ROcp52_59+ROcp52_810*s->dpt[3][12];
RLcp52_311 = s->dpt[1][12]*ROcp52_310+s->dpt[2][12]*ROcp52_69+ROcp52_910*s->dpt[3][12];
OMcp52_111 = OMcp52_110+ROcp52_110*qd[11];
OMcp52_211 = OMcp52_210+ROcp52_210*qd[11];
OMcp52_311 = OMcp52_310+ROcp52_310*qd[11];
ORcp52_111 = OMcp52_210*RLcp52_311-OMcp52_310*RLcp52_211;
ORcp52_211 = -(OMcp52_110*RLcp52_311-OMcp52_310*RLcp52_111);
ORcp52_311 = OMcp52_110*RLcp52_211-OMcp52_210*RLcp52_111;
OPcp52_111 = OPcp52_110+ROcp52_110*qdd[11]+qd[11]*(OMcp52_210*ROcp52_310-OMcp52_310*ROcp52_210);
OPcp52_211 = OPcp52_210+ROcp52_210*qdd[11]-qd[11]*(OMcp52_110*ROcp52_310-OMcp52_310*ROcp52_110);
OPcp52_311 = OPcp52_310+ROcp52_310*qdd[11]+qd[11]*(OMcp52_110*ROcp52_210-OMcp52_210*ROcp52_110);
RLcp52_112 = s->dpt[1][14]*ROcp52_110+s->dpt[2][14]*ROcp52_411+s->dpt[3][14]*ROcp52_711;
RLcp52_212 = s->dpt[1][14]*ROcp52_210+s->dpt[2][14]*ROcp52_511+s->dpt[3][14]*ROcp52_811;
RLcp52_312 = s->dpt[1][14]*ROcp52_310+s->dpt[2][14]*ROcp52_611+s->dpt[3][14]*ROcp52_911;
OMcp52_112 = OMcp52_111+ROcp52_411*qd[12];
OMcp52_212 = OMcp52_211+ROcp52_511*qd[12];
OMcp52_312 = OMcp52_311+ROcp52_611*qd[12];
ORcp52_112 = OMcp52_211*RLcp52_312-OMcp52_311*RLcp52_212;
ORcp52_212 = -(OMcp52_111*RLcp52_312-OMcp52_311*RLcp52_112);
ORcp52_312 = OMcp52_111*RLcp52_212-OMcp52_211*RLcp52_112;
OPcp52_112 = OPcp52_111+ROcp52_411*qdd[12]+qd[12]*(OMcp52_211*ROcp52_611-OMcp52_311*ROcp52_511);
OPcp52_212 = OPcp52_211+ROcp52_511*qdd[12]-qd[12]*(OMcp52_111*ROcp52_611-OMcp52_311*ROcp52_411);
OPcp52_312 = OPcp52_311+ROcp52_611*qdd[12]+qd[12]*(OMcp52_111*ROcp52_511-OMcp52_211*ROcp52_411);
RLcp52_182 = ROcp52_112*s->dpt[1][18]+ROcp52_411*s->dpt[2][18]+ROcp52_712*s->dpt[3][18];
RLcp52_282 = ROcp52_212*s->dpt[1][18]+ROcp52_511*s->dpt[2][18]+ROcp52_812*s->dpt[3][18];
RLcp52_382 = ROcp52_312*s->dpt[1][18]+ROcp52_611*s->dpt[2][18]+ROcp52_912*s->dpt[3][18];
POcp52_182 = RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_182+RLcp52_19+q[1];
POcp52_282 = RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_27+RLcp52_28+RLcp52_282+RLcp52_29+q[2];
POcp52_382 = RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_37+RLcp52_38+RLcp52_382+RLcp52_39+q[3];
JTcp52_282_4 = -(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_37+RLcp52_38+RLcp52_382+RLcp52_39);
JTcp52_382_4 = RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_27+RLcp52_28+RLcp52_282+RLcp52_29;
JTcp52_182_5 = C4*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_37+RLcp52_38+RLcp52_39)-S4*(RLcp52_210+RLcp52_29)-S4*(
 RLcp52_211+RLcp52_212)-S4*(RLcp52_27+RLcp52_28)-RLcp52_282*S4+RLcp52_382*C4;
JTcp52_282_5 = S4*(RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_182+RLcp52_19);
JTcp52_382_5 = -C4*(RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_182+RLcp52_19);
JTcp52_182_6 = ROcp52_85*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_37+RLcp52_38+RLcp52_39)-ROcp52_95*(RLcp52_210+
 RLcp52_29)-ROcp52_95*(RLcp52_211+RLcp52_212)-ROcp52_95*(RLcp52_27+RLcp52_28)-RLcp52_282*ROcp52_95+RLcp52_382*ROcp52_85;
JTcp52_282_6 = -(RLcp52_382*S5-ROcp52_95*(RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_182+RLcp52_19)+
 S5*(RLcp52_310+RLcp52_39)+S5*(RLcp52_311+RLcp52_312)+S5*(RLcp52_37+RLcp52_38));
JTcp52_382_6 = RLcp52_282*S5-ROcp52_85*(RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_182+RLcp52_19)+S5*
 (RLcp52_210+RLcp52_29)+S5*(RLcp52_211+RLcp52_212)+S5*(RLcp52_27+RLcp52_28);
JTcp52_182_7 = ROcp52_56*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_38+RLcp52_382+RLcp52_39)-ROcp52_66*(RLcp52_210+
 RLcp52_211)-ROcp52_66*(RLcp52_212+RLcp52_282)-ROcp52_66*(RLcp52_28+RLcp52_29);
JTcp52_282_7 = -(ROcp52_46*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_38+RLcp52_382+RLcp52_39)-ROcp52_66*(RLcp52_110+
 RLcp52_111)-ROcp52_66*(RLcp52_112+RLcp52_182)-ROcp52_66*(RLcp52_18+RLcp52_19));
JTcp52_382_7 = ROcp52_46*(RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_28+RLcp52_282+RLcp52_29)-ROcp52_56*(RLcp52_110+
 RLcp52_111)-ROcp52_56*(RLcp52_112+RLcp52_182)-ROcp52_56*(RLcp52_18+RLcp52_19);
JTcp52_182_8 = ROcp52_27*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_39)-ROcp52_37*(RLcp52_210+RLcp52_29)-ROcp52_37*(
 RLcp52_211+RLcp52_212)-RLcp52_282*ROcp52_37+RLcp52_382*ROcp52_27;
JTcp52_282_8 = RLcp52_182*ROcp52_37-RLcp52_382*ROcp52_17-ROcp52_17*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_39)+
 ROcp52_37*(RLcp52_110+RLcp52_19)+ROcp52_37*(RLcp52_111+RLcp52_112);
JTcp52_382_8 = ROcp52_17*(RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_29)-ROcp52_27*(RLcp52_110+RLcp52_19)-ROcp52_27*(
 RLcp52_111+RLcp52_112)-RLcp52_182*ROcp52_27+RLcp52_282*ROcp52_17;
JTcp52_182_9 = ROcp52_88*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_382)-ROcp52_98*(RLcp52_210+RLcp52_211)-ROcp52_98*(
 RLcp52_212+RLcp52_282);
JTcp52_282_9 = -(ROcp52_78*(RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_382)-ROcp52_98*(RLcp52_110+RLcp52_111)-ROcp52_98*(
 RLcp52_112+RLcp52_182));
JTcp52_382_9 = ROcp52_78*(RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_282)-ROcp52_88*(RLcp52_110+RLcp52_111)-ROcp52_88*(
 RLcp52_112+RLcp52_182);
JTcp52_182_10 = ROcp52_59*(RLcp52_311+RLcp52_312)-ROcp52_69*(RLcp52_211+RLcp52_212)-RLcp52_282*ROcp52_69+RLcp52_382*
 ROcp52_59;
JTcp52_282_10 = RLcp52_182*ROcp52_69-RLcp52_382*ROcp52_49-ROcp52_49*(RLcp52_311+RLcp52_312)+ROcp52_69*(RLcp52_111+
 RLcp52_112);
JTcp52_382_10 = ROcp52_49*(RLcp52_211+RLcp52_212)-ROcp52_59*(RLcp52_111+RLcp52_112)-RLcp52_182*ROcp52_59+RLcp52_282*
 ROcp52_49;
JTcp52_182_11 = ROcp52_210*(RLcp52_312+RLcp52_382)-ROcp52_310*(RLcp52_212+RLcp52_282);
JTcp52_282_11 = -(ROcp52_110*(RLcp52_312+RLcp52_382)-ROcp52_310*(RLcp52_112+RLcp52_182));
JTcp52_382_11 = ROcp52_110*(RLcp52_212+RLcp52_282)-ROcp52_210*(RLcp52_112+RLcp52_182);
JTcp52_182_12 = -(RLcp52_282*ROcp52_611-RLcp52_382*ROcp52_511);
JTcp52_282_12 = RLcp52_182*ROcp52_611-RLcp52_382*ROcp52_411;
JTcp52_382_12 = -(RLcp52_182*ROcp52_511-RLcp52_282*ROcp52_411);
ORcp52_182 = OMcp52_212*RLcp52_382-OMcp52_312*RLcp52_282;
ORcp52_282 = -(OMcp52_112*RLcp52_382-OMcp52_312*RLcp52_182);
ORcp52_382 = OMcp52_112*RLcp52_282-OMcp52_212*RLcp52_182;
VIcp52_182 = ORcp52_110+ORcp52_111+ORcp52_112+ORcp52_17+ORcp52_18+ORcp52_182+ORcp52_19+qd[1];
VIcp52_282 = ORcp52_210+ORcp52_211+ORcp52_212+ORcp52_27+ORcp52_28+ORcp52_282+ORcp52_29+qd[2];
VIcp52_382 = ORcp52_310+ORcp52_311+ORcp52_312+ORcp52_37+ORcp52_38+ORcp52_382+ORcp52_39+qd[3];
ACcp52_182 = qdd[1]+OMcp52_210*ORcp52_311+OMcp52_211*ORcp52_312+OMcp52_212*ORcp52_382+OMcp52_26*ORcp52_37+OMcp52_27*
 ORcp52_38+OMcp52_28*ORcp52_39+OMcp52_29*ORcp52_310-OMcp52_310*ORcp52_211-OMcp52_311*ORcp52_212-OMcp52_312*ORcp52_282-
 OMcp52_36*ORcp52_27-OMcp52_37*ORcp52_28-OMcp52_38*ORcp52_29-OMcp52_39*ORcp52_210+OPcp52_210*RLcp52_311+OPcp52_211*RLcp52_312
 +OPcp52_212*RLcp52_382+OPcp52_26*RLcp52_37+OPcp52_27*RLcp52_38+OPcp52_28*RLcp52_39+OPcp52_29*RLcp52_310-OPcp52_310*
 RLcp52_211-OPcp52_311*RLcp52_212-OPcp52_312*RLcp52_282-OPcp52_36*RLcp52_27-OPcp52_37*RLcp52_28-OPcp52_38*RLcp52_29-OPcp52_39
 *RLcp52_210;
ACcp52_282 = qdd[2]-OMcp52_110*ORcp52_311-OMcp52_111*ORcp52_312-OMcp52_112*ORcp52_382-OMcp52_16*ORcp52_37-OMcp52_17*
 ORcp52_38-OMcp52_18*ORcp52_39-OMcp52_19*ORcp52_310+OMcp52_310*ORcp52_111+OMcp52_311*ORcp52_112+OMcp52_312*ORcp52_182+
 OMcp52_36*ORcp52_17+OMcp52_37*ORcp52_18+OMcp52_38*ORcp52_19+OMcp52_39*ORcp52_110-OPcp52_110*RLcp52_311-OPcp52_111*RLcp52_312
 -OPcp52_112*RLcp52_382-OPcp52_16*RLcp52_37-OPcp52_17*RLcp52_38-OPcp52_18*RLcp52_39-OPcp52_19*RLcp52_310+OPcp52_310*
 RLcp52_111+OPcp52_311*RLcp52_112+OPcp52_312*RLcp52_182+OPcp52_36*RLcp52_17+OPcp52_37*RLcp52_18+OPcp52_38*RLcp52_19+OPcp52_39
 *RLcp52_110;
ACcp52_382 = qdd[3]+OMcp52_110*ORcp52_211+OMcp52_111*ORcp52_212+OMcp52_112*ORcp52_282+OMcp52_16*ORcp52_27+OMcp52_17*
 ORcp52_28+OMcp52_18*ORcp52_29+OMcp52_19*ORcp52_210-OMcp52_210*ORcp52_111-OMcp52_211*ORcp52_112-OMcp52_212*ORcp52_182-
 OMcp52_26*ORcp52_17-OMcp52_27*ORcp52_18-OMcp52_28*ORcp52_19-OMcp52_29*ORcp52_110+OPcp52_110*RLcp52_211+OPcp52_111*RLcp52_212
 +OPcp52_112*RLcp52_282+OPcp52_16*RLcp52_27+OPcp52_17*RLcp52_28+OPcp52_18*RLcp52_29+OPcp52_19*RLcp52_210-OPcp52_210*
 RLcp52_111-OPcp52_211*RLcp52_112-OPcp52_212*RLcp52_182-OPcp52_26*RLcp52_17-OPcp52_27*RLcp52_18-OPcp52_28*RLcp52_19-OPcp52_29
 *RLcp52_110;

// = = Block_1_0_0_53_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp52_182;
sens->P[2] = POcp52_282;
sens->P[3] = POcp52_382;
sens->R[1][1] = ROcp52_112;
sens->R[1][2] = ROcp52_212;
sens->R[1][3] = ROcp52_312;
sens->R[2][1] = ROcp52_411;
sens->R[2][2] = ROcp52_511;
sens->R[2][3] = ROcp52_611;
sens->R[3][1] = ROcp52_712;
sens->R[3][2] = ROcp52_812;
sens->R[3][3] = ROcp52_912;
sens->V[1] = VIcp52_182;
sens->V[2] = VIcp52_282;
sens->V[3] = VIcp52_382;
sens->OM[1] = OMcp52_112;
sens->OM[2] = OMcp52_212;
sens->OM[3] = OMcp52_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp52_182_5;
sens->J[1][6] = JTcp52_182_6;
sens->J[1][7] = JTcp52_182_7;
sens->J[1][8] = JTcp52_182_8;
sens->J[1][9] = JTcp52_182_9;
sens->J[1][10] = JTcp52_182_10;
sens->J[1][11] = JTcp52_182_11;
sens->J[1][12] = JTcp52_182_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp52_282_4;
sens->J[2][5] = JTcp52_282_5;
sens->J[2][6] = JTcp52_282_6;
sens->J[2][7] = JTcp52_282_7;
sens->J[2][8] = JTcp52_282_8;
sens->J[2][9] = JTcp52_282_9;
sens->J[2][10] = JTcp52_282_10;
sens->J[2][11] = JTcp52_282_11;
sens->J[2][12] = JTcp52_282_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp52_382_4;
sens->J[3][5] = JTcp52_382_5;
sens->J[3][6] = JTcp52_382_6;
sens->J[3][7] = JTcp52_382_7;
sens->J[3][8] = JTcp52_382_8;
sens->J[3][9] = JTcp52_382_9;
sens->J[3][10] = JTcp52_382_10;
sens->J[3][11] = JTcp52_382_11;
sens->J[3][12] = JTcp52_382_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp52_46;
sens->J[4][8] = ROcp52_17;
sens->J[4][9] = ROcp52_78;
sens->J[4][10] = ROcp52_49;
sens->J[4][11] = ROcp52_110;
sens->J[4][12] = ROcp52_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp52_85;
sens->J[5][7] = ROcp52_56;
sens->J[5][8] = ROcp52_27;
sens->J[5][9] = ROcp52_88;
sens->J[5][10] = ROcp52_59;
sens->J[5][11] = ROcp52_210;
sens->J[5][12] = ROcp52_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp52_95;
sens->J[6][7] = ROcp52_66;
sens->J[6][8] = ROcp52_37;
sens->J[6][9] = ROcp52_98;
sens->J[6][10] = ROcp52_69;
sens->J[6][11] = ROcp52_310;
sens->J[6][12] = ROcp52_611;
sens->A[1] = ACcp52_182;
sens->A[2] = ACcp52_282;
sens->A[3] = ACcp52_382;
sens->OMP[1] = OPcp52_112;
sens->OMP[2] = OPcp52_212;
sens->OMP[3] = OPcp52_312;
 
// 
break;
case 54:
 


// = = Block_1_0_0_54_0_1 = = 
 
// Sensor Kinematics 


ROcp53_25 = S4*S5;
ROcp53_35 = -C4*S5;
ROcp53_85 = -S4*C5;
ROcp53_95 = C4*C5;
ROcp53_16 = C5*C6;
ROcp53_26 = ROcp53_25*C6+C4*S6;
ROcp53_36 = ROcp53_35*C6+S4*S6;
ROcp53_46 = -C5*S6;
ROcp53_56 = -(ROcp53_25*S6-C4*C6);
ROcp53_66 = -(ROcp53_35*S6-S4*C6);
OMcp53_25 = qd[5]*C4;
OMcp53_35 = qd[5]*S4;
OMcp53_16 = qd[4]+qd[6]*S5;
OMcp53_26 = OMcp53_25+ROcp53_85*qd[6];
OMcp53_36 = OMcp53_35+ROcp53_95*qd[6];
OPcp53_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp53_26 = ROcp53_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp53_35*S5-ROcp53_95*qd[4]);
OPcp53_36 = ROcp53_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp53_25*S5-ROcp53_85*qd[4]);

// = = Block_1_0_0_54_0_2 = = 
 
// Sensor Kinematics 


ROcp53_17 = ROcp53_16*C7-S5*S7;
ROcp53_27 = ROcp53_26*C7-ROcp53_85*S7;
ROcp53_37 = ROcp53_36*C7-ROcp53_95*S7;
ROcp53_77 = ROcp53_16*S7+S5*C7;
ROcp53_87 = ROcp53_26*S7+ROcp53_85*C7;
ROcp53_97 = ROcp53_36*S7+ROcp53_95*C7;
ROcp53_48 = ROcp53_46*C8+ROcp53_77*S8;
ROcp53_58 = ROcp53_56*C8+ROcp53_87*S8;
ROcp53_68 = ROcp53_66*C8+ROcp53_97*S8;
ROcp53_78 = -(ROcp53_46*S8-ROcp53_77*C8);
ROcp53_88 = -(ROcp53_56*S8-ROcp53_87*C8);
ROcp53_98 = -(ROcp53_66*S8-ROcp53_97*C8);
ROcp53_19 = ROcp53_17*C9+ROcp53_48*S9;
ROcp53_29 = ROcp53_27*C9+ROcp53_58*S9;
ROcp53_39 = ROcp53_37*C9+ROcp53_68*S9;
ROcp53_49 = -(ROcp53_17*S9-ROcp53_48*C9);
ROcp53_59 = -(ROcp53_27*S9-ROcp53_58*C9);
ROcp53_69 = -(ROcp53_37*S9-ROcp53_68*C9);
ROcp53_110 = ROcp53_19*C10-ROcp53_78*S10;
ROcp53_210 = ROcp53_29*C10-ROcp53_88*S10;
ROcp53_310 = ROcp53_39*C10-ROcp53_98*S10;
ROcp53_710 = ROcp53_19*S10+ROcp53_78*C10;
ROcp53_810 = ROcp53_29*S10+ROcp53_88*C10;
ROcp53_910 = ROcp53_39*S10+ROcp53_98*C10;
ROcp53_411 = ROcp53_49*C11+ROcp53_710*S11;
ROcp53_511 = ROcp53_59*C11+ROcp53_810*S11;
ROcp53_611 = ROcp53_69*C11+ROcp53_910*S11;
ROcp53_711 = -(ROcp53_49*S11-ROcp53_710*C11);
ROcp53_811 = -(ROcp53_59*S11-ROcp53_810*C11);
ROcp53_911 = -(ROcp53_69*S11-ROcp53_910*C11);
ROcp53_112 = ROcp53_110*C12-ROcp53_711*S12;
ROcp53_212 = ROcp53_210*C12-ROcp53_811*S12;
ROcp53_312 = ROcp53_310*C12-ROcp53_911*S12;
ROcp53_712 = ROcp53_110*S12+ROcp53_711*C12;
ROcp53_812 = ROcp53_210*S12+ROcp53_811*C12;
ROcp53_912 = ROcp53_310*S12+ROcp53_911*C12;
RLcp53_17 = s->dpt[1][1]*ROcp53_16+s->dpt[3][1]*S5+ROcp53_46*s->dpt[2][1];
RLcp53_27 = s->dpt[1][1]*ROcp53_26+s->dpt[3][1]*ROcp53_85+ROcp53_56*s->dpt[2][1];
RLcp53_37 = s->dpt[1][1]*ROcp53_36+s->dpt[3][1]*ROcp53_95+ROcp53_66*s->dpt[2][1];
OMcp53_17 = OMcp53_16+ROcp53_46*qd[7];
OMcp53_27 = OMcp53_26+ROcp53_56*qd[7];
OMcp53_37 = OMcp53_36+ROcp53_66*qd[7];
ORcp53_17 = OMcp53_26*RLcp53_37-OMcp53_36*RLcp53_27;
ORcp53_27 = -(OMcp53_16*RLcp53_37-OMcp53_36*RLcp53_17);
ORcp53_37 = OMcp53_16*RLcp53_27-OMcp53_26*RLcp53_17;
OPcp53_17 = OPcp53_16+ROcp53_46*qdd[7]+qd[7]*(OMcp53_26*ROcp53_66-OMcp53_36*ROcp53_56);
OPcp53_27 = OPcp53_26+ROcp53_56*qdd[7]-qd[7]*(OMcp53_16*ROcp53_66-OMcp53_36*ROcp53_46);
OPcp53_37 = OPcp53_36+ROcp53_66*qdd[7]+qd[7]*(OMcp53_16*ROcp53_56-OMcp53_26*ROcp53_46);
RLcp53_18 = s->dpt[1][6]*ROcp53_17+s->dpt[3][6]*ROcp53_77+ROcp53_46*s->dpt[2][6];
RLcp53_28 = s->dpt[1][6]*ROcp53_27+s->dpt[3][6]*ROcp53_87+ROcp53_56*s->dpt[2][6];
RLcp53_38 = s->dpt[1][6]*ROcp53_37+s->dpt[3][6]*ROcp53_97+ROcp53_66*s->dpt[2][6];
OMcp53_18 = OMcp53_17+ROcp53_17*qd[8];
OMcp53_28 = OMcp53_27+ROcp53_27*qd[8];
OMcp53_38 = OMcp53_37+ROcp53_37*qd[8];
ORcp53_18 = OMcp53_27*RLcp53_38-OMcp53_37*RLcp53_28;
ORcp53_28 = -(OMcp53_17*RLcp53_38-OMcp53_37*RLcp53_18);
ORcp53_38 = OMcp53_17*RLcp53_28-OMcp53_27*RLcp53_18;
OPcp53_18 = OPcp53_17+ROcp53_17*qdd[8]+qd[8]*(OMcp53_27*ROcp53_37-OMcp53_37*ROcp53_27);
OPcp53_28 = OPcp53_27+ROcp53_27*qdd[8]-qd[8]*(OMcp53_17*ROcp53_37-OMcp53_37*ROcp53_17);
OPcp53_38 = OPcp53_37+ROcp53_37*qdd[8]+qd[8]*(OMcp53_17*ROcp53_27-OMcp53_27*ROcp53_17);
RLcp53_19 = s->dpt[1][8]*ROcp53_17+s->dpt[2][8]*ROcp53_48+ROcp53_78*s->dpt[3][8];
RLcp53_29 = s->dpt[1][8]*ROcp53_27+s->dpt[2][8]*ROcp53_58+ROcp53_88*s->dpt[3][8];
RLcp53_39 = s->dpt[1][8]*ROcp53_37+s->dpt[2][8]*ROcp53_68+ROcp53_98*s->dpt[3][8];
OMcp53_19 = OMcp53_18+ROcp53_78*qd[9];
OMcp53_29 = OMcp53_28+ROcp53_88*qd[9];
OMcp53_39 = OMcp53_38+ROcp53_98*qd[9];
ORcp53_19 = OMcp53_28*RLcp53_39-OMcp53_38*RLcp53_29;
ORcp53_29 = -(OMcp53_18*RLcp53_39-OMcp53_38*RLcp53_19);
ORcp53_39 = OMcp53_18*RLcp53_29-OMcp53_28*RLcp53_19;
OPcp53_19 = OPcp53_18+ROcp53_78*qdd[9]+qd[9]*(OMcp53_28*ROcp53_98-OMcp53_38*ROcp53_88);
OPcp53_29 = OPcp53_28+ROcp53_88*qdd[9]-qd[9]*(OMcp53_18*ROcp53_98-OMcp53_38*ROcp53_78);
OPcp53_39 = OPcp53_38+ROcp53_98*qdd[9]+qd[9]*(OMcp53_18*ROcp53_88-OMcp53_28*ROcp53_78);
RLcp53_110 = s->dpt[1][10]*ROcp53_19+s->dpt[2][10]*ROcp53_49+ROcp53_78*s->dpt[3][10];
RLcp53_210 = s->dpt[1][10]*ROcp53_29+s->dpt[2][10]*ROcp53_59+ROcp53_88*s->dpt[3][10];
RLcp53_310 = s->dpt[1][10]*ROcp53_39+s->dpt[2][10]*ROcp53_69+ROcp53_98*s->dpt[3][10];
OMcp53_110 = OMcp53_19+ROcp53_49*qd[10];
OMcp53_210 = OMcp53_29+ROcp53_59*qd[10];
OMcp53_310 = OMcp53_39+ROcp53_69*qd[10];
ORcp53_110 = OMcp53_29*RLcp53_310-OMcp53_39*RLcp53_210;
ORcp53_210 = -(OMcp53_19*RLcp53_310-OMcp53_39*RLcp53_110);
ORcp53_310 = OMcp53_19*RLcp53_210-OMcp53_29*RLcp53_110;
OPcp53_110 = OPcp53_19+ROcp53_49*qdd[10]+qd[10]*(OMcp53_29*ROcp53_69-OMcp53_39*ROcp53_59);
OPcp53_210 = OPcp53_29+ROcp53_59*qdd[10]-qd[10]*(OMcp53_19*ROcp53_69-OMcp53_39*ROcp53_49);
OPcp53_310 = OPcp53_39+ROcp53_69*qdd[10]+qd[10]*(OMcp53_19*ROcp53_59-OMcp53_29*ROcp53_49);
RLcp53_111 = s->dpt[1][12]*ROcp53_110+s->dpt[2][12]*ROcp53_49+ROcp53_710*s->dpt[3][12];
RLcp53_211 = s->dpt[1][12]*ROcp53_210+s->dpt[2][12]*ROcp53_59+ROcp53_810*s->dpt[3][12];
RLcp53_311 = s->dpt[1][12]*ROcp53_310+s->dpt[2][12]*ROcp53_69+ROcp53_910*s->dpt[3][12];
OMcp53_111 = OMcp53_110+ROcp53_110*qd[11];
OMcp53_211 = OMcp53_210+ROcp53_210*qd[11];
OMcp53_311 = OMcp53_310+ROcp53_310*qd[11];
ORcp53_111 = OMcp53_210*RLcp53_311-OMcp53_310*RLcp53_211;
ORcp53_211 = -(OMcp53_110*RLcp53_311-OMcp53_310*RLcp53_111);
ORcp53_311 = OMcp53_110*RLcp53_211-OMcp53_210*RLcp53_111;
OPcp53_111 = OPcp53_110+ROcp53_110*qdd[11]+qd[11]*(OMcp53_210*ROcp53_310-OMcp53_310*ROcp53_210);
OPcp53_211 = OPcp53_210+ROcp53_210*qdd[11]-qd[11]*(OMcp53_110*ROcp53_310-OMcp53_310*ROcp53_110);
OPcp53_311 = OPcp53_310+ROcp53_310*qdd[11]+qd[11]*(OMcp53_110*ROcp53_210-OMcp53_210*ROcp53_110);
RLcp53_112 = s->dpt[1][14]*ROcp53_110+s->dpt[2][14]*ROcp53_411+s->dpt[3][14]*ROcp53_711;
RLcp53_212 = s->dpt[1][14]*ROcp53_210+s->dpt[2][14]*ROcp53_511+s->dpt[3][14]*ROcp53_811;
RLcp53_312 = s->dpt[1][14]*ROcp53_310+s->dpt[2][14]*ROcp53_611+s->dpt[3][14]*ROcp53_911;
OMcp53_112 = OMcp53_111+ROcp53_411*qd[12];
OMcp53_212 = OMcp53_211+ROcp53_511*qd[12];
OMcp53_312 = OMcp53_311+ROcp53_611*qd[12];
ORcp53_112 = OMcp53_211*RLcp53_312-OMcp53_311*RLcp53_212;
ORcp53_212 = -(OMcp53_111*RLcp53_312-OMcp53_311*RLcp53_112);
ORcp53_312 = OMcp53_111*RLcp53_212-OMcp53_211*RLcp53_112;
OPcp53_112 = OPcp53_111+ROcp53_411*qdd[12]+qd[12]*(OMcp53_211*ROcp53_611-OMcp53_311*ROcp53_511);
OPcp53_212 = OPcp53_211+ROcp53_511*qdd[12]-qd[12]*(OMcp53_111*ROcp53_611-OMcp53_311*ROcp53_411);
OPcp53_312 = OPcp53_311+ROcp53_611*qdd[12]+qd[12]*(OMcp53_111*ROcp53_511-OMcp53_211*ROcp53_411);
RLcp53_183 = ROcp53_112*s->dpt[1][19]+ROcp53_411*s->dpt[2][19]+ROcp53_712*s->dpt[3][19];
RLcp53_283 = ROcp53_212*s->dpt[1][19]+ROcp53_511*s->dpt[2][19]+ROcp53_812*s->dpt[3][19];
RLcp53_383 = ROcp53_312*s->dpt[1][19]+ROcp53_611*s->dpt[2][19]+ROcp53_912*s->dpt[3][19];
POcp53_183 = RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_183+RLcp53_19+q[1];
POcp53_283 = RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_27+RLcp53_28+RLcp53_283+RLcp53_29+q[2];
POcp53_383 = RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_37+RLcp53_38+RLcp53_383+RLcp53_39+q[3];
JTcp53_283_4 = -(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_37+RLcp53_38+RLcp53_383+RLcp53_39);
JTcp53_383_4 = RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_27+RLcp53_28+RLcp53_283+RLcp53_29;
JTcp53_183_5 = C4*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_37+RLcp53_38+RLcp53_39)-S4*(RLcp53_210+RLcp53_29)-S4*(
 RLcp53_211+RLcp53_212)-S4*(RLcp53_27+RLcp53_28)-RLcp53_283*S4+RLcp53_383*C4;
JTcp53_283_5 = S4*(RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_183+RLcp53_19);
JTcp53_383_5 = -C4*(RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_183+RLcp53_19);
JTcp53_183_6 = ROcp53_85*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_37+RLcp53_38+RLcp53_39)-ROcp53_95*(RLcp53_210+
 RLcp53_29)-ROcp53_95*(RLcp53_211+RLcp53_212)-ROcp53_95*(RLcp53_27+RLcp53_28)-RLcp53_283*ROcp53_95+RLcp53_383*ROcp53_85;
JTcp53_283_6 = -(RLcp53_383*S5-ROcp53_95*(RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_183+RLcp53_19)+
 S5*(RLcp53_310+RLcp53_39)+S5*(RLcp53_311+RLcp53_312)+S5*(RLcp53_37+RLcp53_38));
JTcp53_383_6 = RLcp53_283*S5-ROcp53_85*(RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_183+RLcp53_19)+S5*
 (RLcp53_210+RLcp53_29)+S5*(RLcp53_211+RLcp53_212)+S5*(RLcp53_27+RLcp53_28);
JTcp53_183_7 = ROcp53_56*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_38+RLcp53_383+RLcp53_39)-ROcp53_66*(RLcp53_210+
 RLcp53_211)-ROcp53_66*(RLcp53_212+RLcp53_283)-ROcp53_66*(RLcp53_28+RLcp53_29);
JTcp53_283_7 = -(ROcp53_46*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_38+RLcp53_383+RLcp53_39)-ROcp53_66*(RLcp53_110+
 RLcp53_111)-ROcp53_66*(RLcp53_112+RLcp53_183)-ROcp53_66*(RLcp53_18+RLcp53_19));
JTcp53_383_7 = ROcp53_46*(RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_28+RLcp53_283+RLcp53_29)-ROcp53_56*(RLcp53_110+
 RLcp53_111)-ROcp53_56*(RLcp53_112+RLcp53_183)-ROcp53_56*(RLcp53_18+RLcp53_19);
JTcp53_183_8 = ROcp53_27*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_39)-ROcp53_37*(RLcp53_210+RLcp53_29)-ROcp53_37*(
 RLcp53_211+RLcp53_212)-RLcp53_283*ROcp53_37+RLcp53_383*ROcp53_27;
JTcp53_283_8 = RLcp53_183*ROcp53_37-RLcp53_383*ROcp53_17-ROcp53_17*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_39)+
 ROcp53_37*(RLcp53_110+RLcp53_19)+ROcp53_37*(RLcp53_111+RLcp53_112);
JTcp53_383_8 = ROcp53_17*(RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_29)-ROcp53_27*(RLcp53_110+RLcp53_19)-ROcp53_27*(
 RLcp53_111+RLcp53_112)-RLcp53_183*ROcp53_27+RLcp53_283*ROcp53_17;
JTcp53_183_9 = ROcp53_88*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_383)-ROcp53_98*(RLcp53_210+RLcp53_211)-ROcp53_98*(
 RLcp53_212+RLcp53_283);
JTcp53_283_9 = -(ROcp53_78*(RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_383)-ROcp53_98*(RLcp53_110+RLcp53_111)-ROcp53_98*(
 RLcp53_112+RLcp53_183));
JTcp53_383_9 = ROcp53_78*(RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_283)-ROcp53_88*(RLcp53_110+RLcp53_111)-ROcp53_88*(
 RLcp53_112+RLcp53_183);
JTcp53_183_10 = ROcp53_59*(RLcp53_311+RLcp53_312)-ROcp53_69*(RLcp53_211+RLcp53_212)-RLcp53_283*ROcp53_69+RLcp53_383*
 ROcp53_59;
JTcp53_283_10 = RLcp53_183*ROcp53_69-RLcp53_383*ROcp53_49-ROcp53_49*(RLcp53_311+RLcp53_312)+ROcp53_69*(RLcp53_111+
 RLcp53_112);
JTcp53_383_10 = ROcp53_49*(RLcp53_211+RLcp53_212)-ROcp53_59*(RLcp53_111+RLcp53_112)-RLcp53_183*ROcp53_59+RLcp53_283*
 ROcp53_49;
JTcp53_183_11 = ROcp53_210*(RLcp53_312+RLcp53_383)-ROcp53_310*(RLcp53_212+RLcp53_283);
JTcp53_283_11 = -(ROcp53_110*(RLcp53_312+RLcp53_383)-ROcp53_310*(RLcp53_112+RLcp53_183));
JTcp53_383_11 = ROcp53_110*(RLcp53_212+RLcp53_283)-ROcp53_210*(RLcp53_112+RLcp53_183);
JTcp53_183_12 = -(RLcp53_283*ROcp53_611-RLcp53_383*ROcp53_511);
JTcp53_283_12 = RLcp53_183*ROcp53_611-RLcp53_383*ROcp53_411;
JTcp53_383_12 = -(RLcp53_183*ROcp53_511-RLcp53_283*ROcp53_411);
ORcp53_183 = OMcp53_212*RLcp53_383-OMcp53_312*RLcp53_283;
ORcp53_283 = -(OMcp53_112*RLcp53_383-OMcp53_312*RLcp53_183);
ORcp53_383 = OMcp53_112*RLcp53_283-OMcp53_212*RLcp53_183;
VIcp53_183 = ORcp53_110+ORcp53_111+ORcp53_112+ORcp53_17+ORcp53_18+ORcp53_183+ORcp53_19+qd[1];
VIcp53_283 = ORcp53_210+ORcp53_211+ORcp53_212+ORcp53_27+ORcp53_28+ORcp53_283+ORcp53_29+qd[2];
VIcp53_383 = ORcp53_310+ORcp53_311+ORcp53_312+ORcp53_37+ORcp53_38+ORcp53_383+ORcp53_39+qd[3];
ACcp53_183 = qdd[1]+OMcp53_210*ORcp53_311+OMcp53_211*ORcp53_312+OMcp53_212*ORcp53_383+OMcp53_26*ORcp53_37+OMcp53_27*
 ORcp53_38+OMcp53_28*ORcp53_39+OMcp53_29*ORcp53_310-OMcp53_310*ORcp53_211-OMcp53_311*ORcp53_212-OMcp53_312*ORcp53_283-
 OMcp53_36*ORcp53_27-OMcp53_37*ORcp53_28-OMcp53_38*ORcp53_29-OMcp53_39*ORcp53_210+OPcp53_210*RLcp53_311+OPcp53_211*RLcp53_312
 +OPcp53_212*RLcp53_383+OPcp53_26*RLcp53_37+OPcp53_27*RLcp53_38+OPcp53_28*RLcp53_39+OPcp53_29*RLcp53_310-OPcp53_310*
 RLcp53_211-OPcp53_311*RLcp53_212-OPcp53_312*RLcp53_283-OPcp53_36*RLcp53_27-OPcp53_37*RLcp53_28-OPcp53_38*RLcp53_29-OPcp53_39
 *RLcp53_210;
ACcp53_283 = qdd[2]-OMcp53_110*ORcp53_311-OMcp53_111*ORcp53_312-OMcp53_112*ORcp53_383-OMcp53_16*ORcp53_37-OMcp53_17*
 ORcp53_38-OMcp53_18*ORcp53_39-OMcp53_19*ORcp53_310+OMcp53_310*ORcp53_111+OMcp53_311*ORcp53_112+OMcp53_312*ORcp53_183+
 OMcp53_36*ORcp53_17+OMcp53_37*ORcp53_18+OMcp53_38*ORcp53_19+OMcp53_39*ORcp53_110-OPcp53_110*RLcp53_311-OPcp53_111*RLcp53_312
 -OPcp53_112*RLcp53_383-OPcp53_16*RLcp53_37-OPcp53_17*RLcp53_38-OPcp53_18*RLcp53_39-OPcp53_19*RLcp53_310+OPcp53_310*
 RLcp53_111+OPcp53_311*RLcp53_112+OPcp53_312*RLcp53_183+OPcp53_36*RLcp53_17+OPcp53_37*RLcp53_18+OPcp53_38*RLcp53_19+OPcp53_39
 *RLcp53_110;
ACcp53_383 = qdd[3]+OMcp53_110*ORcp53_211+OMcp53_111*ORcp53_212+OMcp53_112*ORcp53_283+OMcp53_16*ORcp53_27+OMcp53_17*
 ORcp53_28+OMcp53_18*ORcp53_29+OMcp53_19*ORcp53_210-OMcp53_210*ORcp53_111-OMcp53_211*ORcp53_112-OMcp53_212*ORcp53_183-
 OMcp53_26*ORcp53_17-OMcp53_27*ORcp53_18-OMcp53_28*ORcp53_19-OMcp53_29*ORcp53_110+OPcp53_110*RLcp53_211+OPcp53_111*RLcp53_212
 +OPcp53_112*RLcp53_283+OPcp53_16*RLcp53_27+OPcp53_17*RLcp53_28+OPcp53_18*RLcp53_29+OPcp53_19*RLcp53_210-OPcp53_210*
 RLcp53_111-OPcp53_211*RLcp53_112-OPcp53_212*RLcp53_183-OPcp53_26*RLcp53_17-OPcp53_27*RLcp53_18-OPcp53_28*RLcp53_19-OPcp53_29
 *RLcp53_110;

// = = Block_1_0_0_54_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp53_183;
sens->P[2] = POcp53_283;
sens->P[3] = POcp53_383;
sens->R[1][1] = ROcp53_112;
sens->R[1][2] = ROcp53_212;
sens->R[1][3] = ROcp53_312;
sens->R[2][1] = ROcp53_411;
sens->R[2][2] = ROcp53_511;
sens->R[2][3] = ROcp53_611;
sens->R[3][1] = ROcp53_712;
sens->R[3][2] = ROcp53_812;
sens->R[3][3] = ROcp53_912;
sens->V[1] = VIcp53_183;
sens->V[2] = VIcp53_283;
sens->V[3] = VIcp53_383;
sens->OM[1] = OMcp53_112;
sens->OM[2] = OMcp53_212;
sens->OM[3] = OMcp53_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp53_183_5;
sens->J[1][6] = JTcp53_183_6;
sens->J[1][7] = JTcp53_183_7;
sens->J[1][8] = JTcp53_183_8;
sens->J[1][9] = JTcp53_183_9;
sens->J[1][10] = JTcp53_183_10;
sens->J[1][11] = JTcp53_183_11;
sens->J[1][12] = JTcp53_183_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp53_283_4;
sens->J[2][5] = JTcp53_283_5;
sens->J[2][6] = JTcp53_283_6;
sens->J[2][7] = JTcp53_283_7;
sens->J[2][8] = JTcp53_283_8;
sens->J[2][9] = JTcp53_283_9;
sens->J[2][10] = JTcp53_283_10;
sens->J[2][11] = JTcp53_283_11;
sens->J[2][12] = JTcp53_283_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp53_383_4;
sens->J[3][5] = JTcp53_383_5;
sens->J[3][6] = JTcp53_383_6;
sens->J[3][7] = JTcp53_383_7;
sens->J[3][8] = JTcp53_383_8;
sens->J[3][9] = JTcp53_383_9;
sens->J[3][10] = JTcp53_383_10;
sens->J[3][11] = JTcp53_383_11;
sens->J[3][12] = JTcp53_383_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp53_46;
sens->J[4][8] = ROcp53_17;
sens->J[4][9] = ROcp53_78;
sens->J[4][10] = ROcp53_49;
sens->J[4][11] = ROcp53_110;
sens->J[4][12] = ROcp53_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp53_85;
sens->J[5][7] = ROcp53_56;
sens->J[5][8] = ROcp53_27;
sens->J[5][9] = ROcp53_88;
sens->J[5][10] = ROcp53_59;
sens->J[5][11] = ROcp53_210;
sens->J[5][12] = ROcp53_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp53_95;
sens->J[6][7] = ROcp53_66;
sens->J[6][8] = ROcp53_37;
sens->J[6][9] = ROcp53_98;
sens->J[6][10] = ROcp53_69;
sens->J[6][11] = ROcp53_310;
sens->J[6][12] = ROcp53_611;
sens->A[1] = ACcp53_183;
sens->A[2] = ACcp53_283;
sens->A[3] = ACcp53_383;
sens->OMP[1] = OPcp53_112;
sens->OMP[2] = OPcp53_212;
sens->OMP[3] = OPcp53_312;
 
// 
break;
case 55:
 


// = = Block_1_0_0_55_0_1 = = 
 
// Sensor Kinematics 


ROcp54_25 = S4*S5;
ROcp54_35 = -C4*S5;
ROcp54_85 = -S4*C5;
ROcp54_95 = C4*C5;
ROcp54_16 = C5*C6;
ROcp54_26 = ROcp54_25*C6+C4*S6;
ROcp54_36 = ROcp54_35*C6+S4*S6;
ROcp54_46 = -C5*S6;
ROcp54_56 = -(ROcp54_25*S6-C4*C6);
ROcp54_66 = -(ROcp54_35*S6-S4*C6);
OMcp54_25 = qd[5]*C4;
OMcp54_35 = qd[5]*S4;
OMcp54_16 = qd[4]+qd[6]*S5;
OMcp54_26 = OMcp54_25+ROcp54_85*qd[6];
OMcp54_36 = OMcp54_35+ROcp54_95*qd[6];
OPcp54_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp54_26 = ROcp54_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp54_35*S5-ROcp54_95*qd[4]);
OPcp54_36 = ROcp54_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp54_25*S5-ROcp54_85*qd[4]);

// = = Block_1_0_0_55_0_2 = = 
 
// Sensor Kinematics 


ROcp54_17 = ROcp54_16*C7-S5*S7;
ROcp54_27 = ROcp54_26*C7-ROcp54_85*S7;
ROcp54_37 = ROcp54_36*C7-ROcp54_95*S7;
ROcp54_77 = ROcp54_16*S7+S5*C7;
ROcp54_87 = ROcp54_26*S7+ROcp54_85*C7;
ROcp54_97 = ROcp54_36*S7+ROcp54_95*C7;
ROcp54_48 = ROcp54_46*C8+ROcp54_77*S8;
ROcp54_58 = ROcp54_56*C8+ROcp54_87*S8;
ROcp54_68 = ROcp54_66*C8+ROcp54_97*S8;
ROcp54_78 = -(ROcp54_46*S8-ROcp54_77*C8);
ROcp54_88 = -(ROcp54_56*S8-ROcp54_87*C8);
ROcp54_98 = -(ROcp54_66*S8-ROcp54_97*C8);
ROcp54_19 = ROcp54_17*C9+ROcp54_48*S9;
ROcp54_29 = ROcp54_27*C9+ROcp54_58*S9;
ROcp54_39 = ROcp54_37*C9+ROcp54_68*S9;
ROcp54_49 = -(ROcp54_17*S9-ROcp54_48*C9);
ROcp54_59 = -(ROcp54_27*S9-ROcp54_58*C9);
ROcp54_69 = -(ROcp54_37*S9-ROcp54_68*C9);
ROcp54_110 = ROcp54_19*C10-ROcp54_78*S10;
ROcp54_210 = ROcp54_29*C10-ROcp54_88*S10;
ROcp54_310 = ROcp54_39*C10-ROcp54_98*S10;
ROcp54_710 = ROcp54_19*S10+ROcp54_78*C10;
ROcp54_810 = ROcp54_29*S10+ROcp54_88*C10;
ROcp54_910 = ROcp54_39*S10+ROcp54_98*C10;
ROcp54_411 = ROcp54_49*C11+ROcp54_710*S11;
ROcp54_511 = ROcp54_59*C11+ROcp54_810*S11;
ROcp54_611 = ROcp54_69*C11+ROcp54_910*S11;
ROcp54_711 = -(ROcp54_49*S11-ROcp54_710*C11);
ROcp54_811 = -(ROcp54_59*S11-ROcp54_810*C11);
ROcp54_911 = -(ROcp54_69*S11-ROcp54_910*C11);
ROcp54_112 = ROcp54_110*C12-ROcp54_711*S12;
ROcp54_212 = ROcp54_210*C12-ROcp54_811*S12;
ROcp54_312 = ROcp54_310*C12-ROcp54_911*S12;
ROcp54_712 = ROcp54_110*S12+ROcp54_711*C12;
ROcp54_812 = ROcp54_210*S12+ROcp54_811*C12;
ROcp54_912 = ROcp54_310*S12+ROcp54_911*C12;
RLcp54_17 = s->dpt[1][1]*ROcp54_16+s->dpt[3][1]*S5+ROcp54_46*s->dpt[2][1];
RLcp54_27 = s->dpt[1][1]*ROcp54_26+s->dpt[3][1]*ROcp54_85+ROcp54_56*s->dpt[2][1];
RLcp54_37 = s->dpt[1][1]*ROcp54_36+s->dpt[3][1]*ROcp54_95+ROcp54_66*s->dpt[2][1];
OMcp54_17 = OMcp54_16+ROcp54_46*qd[7];
OMcp54_27 = OMcp54_26+ROcp54_56*qd[7];
OMcp54_37 = OMcp54_36+ROcp54_66*qd[7];
ORcp54_17 = OMcp54_26*RLcp54_37-OMcp54_36*RLcp54_27;
ORcp54_27 = -(OMcp54_16*RLcp54_37-OMcp54_36*RLcp54_17);
ORcp54_37 = OMcp54_16*RLcp54_27-OMcp54_26*RLcp54_17;
OPcp54_17 = OPcp54_16+ROcp54_46*qdd[7]+qd[7]*(OMcp54_26*ROcp54_66-OMcp54_36*ROcp54_56);
OPcp54_27 = OPcp54_26+ROcp54_56*qdd[7]-qd[7]*(OMcp54_16*ROcp54_66-OMcp54_36*ROcp54_46);
OPcp54_37 = OPcp54_36+ROcp54_66*qdd[7]+qd[7]*(OMcp54_16*ROcp54_56-OMcp54_26*ROcp54_46);
RLcp54_18 = s->dpt[1][6]*ROcp54_17+s->dpt[3][6]*ROcp54_77+ROcp54_46*s->dpt[2][6];
RLcp54_28 = s->dpt[1][6]*ROcp54_27+s->dpt[3][6]*ROcp54_87+ROcp54_56*s->dpt[2][6];
RLcp54_38 = s->dpt[1][6]*ROcp54_37+s->dpt[3][6]*ROcp54_97+ROcp54_66*s->dpt[2][6];
OMcp54_18 = OMcp54_17+ROcp54_17*qd[8];
OMcp54_28 = OMcp54_27+ROcp54_27*qd[8];
OMcp54_38 = OMcp54_37+ROcp54_37*qd[8];
ORcp54_18 = OMcp54_27*RLcp54_38-OMcp54_37*RLcp54_28;
ORcp54_28 = -(OMcp54_17*RLcp54_38-OMcp54_37*RLcp54_18);
ORcp54_38 = OMcp54_17*RLcp54_28-OMcp54_27*RLcp54_18;
OPcp54_18 = OPcp54_17+ROcp54_17*qdd[8]+qd[8]*(OMcp54_27*ROcp54_37-OMcp54_37*ROcp54_27);
OPcp54_28 = OPcp54_27+ROcp54_27*qdd[8]-qd[8]*(OMcp54_17*ROcp54_37-OMcp54_37*ROcp54_17);
OPcp54_38 = OPcp54_37+ROcp54_37*qdd[8]+qd[8]*(OMcp54_17*ROcp54_27-OMcp54_27*ROcp54_17);
RLcp54_19 = s->dpt[1][8]*ROcp54_17+s->dpt[2][8]*ROcp54_48+ROcp54_78*s->dpt[3][8];
RLcp54_29 = s->dpt[1][8]*ROcp54_27+s->dpt[2][8]*ROcp54_58+ROcp54_88*s->dpt[3][8];
RLcp54_39 = s->dpt[1][8]*ROcp54_37+s->dpt[2][8]*ROcp54_68+ROcp54_98*s->dpt[3][8];
OMcp54_19 = OMcp54_18+ROcp54_78*qd[9];
OMcp54_29 = OMcp54_28+ROcp54_88*qd[9];
OMcp54_39 = OMcp54_38+ROcp54_98*qd[9];
ORcp54_19 = OMcp54_28*RLcp54_39-OMcp54_38*RLcp54_29;
ORcp54_29 = -(OMcp54_18*RLcp54_39-OMcp54_38*RLcp54_19);
ORcp54_39 = OMcp54_18*RLcp54_29-OMcp54_28*RLcp54_19;
OPcp54_19 = OPcp54_18+ROcp54_78*qdd[9]+qd[9]*(OMcp54_28*ROcp54_98-OMcp54_38*ROcp54_88);
OPcp54_29 = OPcp54_28+ROcp54_88*qdd[9]-qd[9]*(OMcp54_18*ROcp54_98-OMcp54_38*ROcp54_78);
OPcp54_39 = OPcp54_38+ROcp54_98*qdd[9]+qd[9]*(OMcp54_18*ROcp54_88-OMcp54_28*ROcp54_78);
RLcp54_110 = s->dpt[1][10]*ROcp54_19+s->dpt[2][10]*ROcp54_49+ROcp54_78*s->dpt[3][10];
RLcp54_210 = s->dpt[1][10]*ROcp54_29+s->dpt[2][10]*ROcp54_59+ROcp54_88*s->dpt[3][10];
RLcp54_310 = s->dpt[1][10]*ROcp54_39+s->dpt[2][10]*ROcp54_69+ROcp54_98*s->dpt[3][10];
OMcp54_110 = OMcp54_19+ROcp54_49*qd[10];
OMcp54_210 = OMcp54_29+ROcp54_59*qd[10];
OMcp54_310 = OMcp54_39+ROcp54_69*qd[10];
ORcp54_110 = OMcp54_29*RLcp54_310-OMcp54_39*RLcp54_210;
ORcp54_210 = -(OMcp54_19*RLcp54_310-OMcp54_39*RLcp54_110);
ORcp54_310 = OMcp54_19*RLcp54_210-OMcp54_29*RLcp54_110;
OPcp54_110 = OPcp54_19+ROcp54_49*qdd[10]+qd[10]*(OMcp54_29*ROcp54_69-OMcp54_39*ROcp54_59);
OPcp54_210 = OPcp54_29+ROcp54_59*qdd[10]-qd[10]*(OMcp54_19*ROcp54_69-OMcp54_39*ROcp54_49);
OPcp54_310 = OPcp54_39+ROcp54_69*qdd[10]+qd[10]*(OMcp54_19*ROcp54_59-OMcp54_29*ROcp54_49);
RLcp54_111 = s->dpt[1][12]*ROcp54_110+s->dpt[2][12]*ROcp54_49+ROcp54_710*s->dpt[3][12];
RLcp54_211 = s->dpt[1][12]*ROcp54_210+s->dpt[2][12]*ROcp54_59+ROcp54_810*s->dpt[3][12];
RLcp54_311 = s->dpt[1][12]*ROcp54_310+s->dpt[2][12]*ROcp54_69+ROcp54_910*s->dpt[3][12];
OMcp54_111 = OMcp54_110+ROcp54_110*qd[11];
OMcp54_211 = OMcp54_210+ROcp54_210*qd[11];
OMcp54_311 = OMcp54_310+ROcp54_310*qd[11];
ORcp54_111 = OMcp54_210*RLcp54_311-OMcp54_310*RLcp54_211;
ORcp54_211 = -(OMcp54_110*RLcp54_311-OMcp54_310*RLcp54_111);
ORcp54_311 = OMcp54_110*RLcp54_211-OMcp54_210*RLcp54_111;
OPcp54_111 = OPcp54_110+ROcp54_110*qdd[11]+qd[11]*(OMcp54_210*ROcp54_310-OMcp54_310*ROcp54_210);
OPcp54_211 = OPcp54_210+ROcp54_210*qdd[11]-qd[11]*(OMcp54_110*ROcp54_310-OMcp54_310*ROcp54_110);
OPcp54_311 = OPcp54_310+ROcp54_310*qdd[11]+qd[11]*(OMcp54_110*ROcp54_210-OMcp54_210*ROcp54_110);
RLcp54_112 = s->dpt[1][14]*ROcp54_110+s->dpt[2][14]*ROcp54_411+s->dpt[3][14]*ROcp54_711;
RLcp54_212 = s->dpt[1][14]*ROcp54_210+s->dpt[2][14]*ROcp54_511+s->dpt[3][14]*ROcp54_811;
RLcp54_312 = s->dpt[1][14]*ROcp54_310+s->dpt[2][14]*ROcp54_611+s->dpt[3][14]*ROcp54_911;
OMcp54_112 = OMcp54_111+ROcp54_411*qd[12];
OMcp54_212 = OMcp54_211+ROcp54_511*qd[12];
OMcp54_312 = OMcp54_311+ROcp54_611*qd[12];
ORcp54_112 = OMcp54_211*RLcp54_312-OMcp54_311*RLcp54_212;
ORcp54_212 = -(OMcp54_111*RLcp54_312-OMcp54_311*RLcp54_112);
ORcp54_312 = OMcp54_111*RLcp54_212-OMcp54_211*RLcp54_112;
OPcp54_112 = OPcp54_111+ROcp54_411*qdd[12]+qd[12]*(OMcp54_211*ROcp54_611-OMcp54_311*ROcp54_511);
OPcp54_212 = OPcp54_211+ROcp54_511*qdd[12]-qd[12]*(OMcp54_111*ROcp54_611-OMcp54_311*ROcp54_411);
OPcp54_312 = OPcp54_311+ROcp54_611*qdd[12]+qd[12]*(OMcp54_111*ROcp54_511-OMcp54_211*ROcp54_411);
RLcp54_184 = ROcp54_112*s->dpt[1][20]+ROcp54_411*s->dpt[2][20]+ROcp54_712*s->dpt[3][20];
RLcp54_284 = ROcp54_212*s->dpt[1][20]+ROcp54_511*s->dpt[2][20]+ROcp54_812*s->dpt[3][20];
RLcp54_384 = ROcp54_312*s->dpt[1][20]+ROcp54_611*s->dpt[2][20]+ROcp54_912*s->dpt[3][20];
POcp54_184 = RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_184+RLcp54_19+q[1];
POcp54_284 = RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_27+RLcp54_28+RLcp54_284+RLcp54_29+q[2];
POcp54_384 = RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_37+RLcp54_38+RLcp54_384+RLcp54_39+q[3];
JTcp54_284_4 = -(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_37+RLcp54_38+RLcp54_384+RLcp54_39);
JTcp54_384_4 = RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_27+RLcp54_28+RLcp54_284+RLcp54_29;
JTcp54_184_5 = C4*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_37+RLcp54_38+RLcp54_39)-S4*(RLcp54_210+RLcp54_29)-S4*(
 RLcp54_211+RLcp54_212)-S4*(RLcp54_27+RLcp54_28)-RLcp54_284*S4+RLcp54_384*C4;
JTcp54_284_5 = S4*(RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_184+RLcp54_19);
JTcp54_384_5 = -C4*(RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_184+RLcp54_19);
JTcp54_184_6 = ROcp54_85*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_37+RLcp54_38+RLcp54_39)-ROcp54_95*(RLcp54_210+
 RLcp54_29)-ROcp54_95*(RLcp54_211+RLcp54_212)-ROcp54_95*(RLcp54_27+RLcp54_28)-RLcp54_284*ROcp54_95+RLcp54_384*ROcp54_85;
JTcp54_284_6 = -(RLcp54_384*S5-ROcp54_95*(RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_184+RLcp54_19)+
 S5*(RLcp54_310+RLcp54_39)+S5*(RLcp54_311+RLcp54_312)+S5*(RLcp54_37+RLcp54_38));
JTcp54_384_6 = RLcp54_284*S5-ROcp54_85*(RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_184+RLcp54_19)+S5*
 (RLcp54_210+RLcp54_29)+S5*(RLcp54_211+RLcp54_212)+S5*(RLcp54_27+RLcp54_28);
JTcp54_184_7 = ROcp54_56*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_38+RLcp54_384+RLcp54_39)-ROcp54_66*(RLcp54_210+
 RLcp54_211)-ROcp54_66*(RLcp54_212+RLcp54_284)-ROcp54_66*(RLcp54_28+RLcp54_29);
JTcp54_284_7 = -(ROcp54_46*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_38+RLcp54_384+RLcp54_39)-ROcp54_66*(RLcp54_110+
 RLcp54_111)-ROcp54_66*(RLcp54_112+RLcp54_184)-ROcp54_66*(RLcp54_18+RLcp54_19));
JTcp54_384_7 = ROcp54_46*(RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_28+RLcp54_284+RLcp54_29)-ROcp54_56*(RLcp54_110+
 RLcp54_111)-ROcp54_56*(RLcp54_112+RLcp54_184)-ROcp54_56*(RLcp54_18+RLcp54_19);
JTcp54_184_8 = ROcp54_27*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_39)-ROcp54_37*(RLcp54_210+RLcp54_29)-ROcp54_37*(
 RLcp54_211+RLcp54_212)-RLcp54_284*ROcp54_37+RLcp54_384*ROcp54_27;
JTcp54_284_8 = RLcp54_184*ROcp54_37-RLcp54_384*ROcp54_17-ROcp54_17*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_39)+
 ROcp54_37*(RLcp54_110+RLcp54_19)+ROcp54_37*(RLcp54_111+RLcp54_112);
JTcp54_384_8 = ROcp54_17*(RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_29)-ROcp54_27*(RLcp54_110+RLcp54_19)-ROcp54_27*(
 RLcp54_111+RLcp54_112)-RLcp54_184*ROcp54_27+RLcp54_284*ROcp54_17;
JTcp54_184_9 = ROcp54_88*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_384)-ROcp54_98*(RLcp54_210+RLcp54_211)-ROcp54_98*(
 RLcp54_212+RLcp54_284);
JTcp54_284_9 = -(ROcp54_78*(RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_384)-ROcp54_98*(RLcp54_110+RLcp54_111)-ROcp54_98*(
 RLcp54_112+RLcp54_184));
JTcp54_384_9 = ROcp54_78*(RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_284)-ROcp54_88*(RLcp54_110+RLcp54_111)-ROcp54_88*(
 RLcp54_112+RLcp54_184);
JTcp54_184_10 = ROcp54_59*(RLcp54_311+RLcp54_312)-ROcp54_69*(RLcp54_211+RLcp54_212)-RLcp54_284*ROcp54_69+RLcp54_384*
 ROcp54_59;
JTcp54_284_10 = RLcp54_184*ROcp54_69-RLcp54_384*ROcp54_49-ROcp54_49*(RLcp54_311+RLcp54_312)+ROcp54_69*(RLcp54_111+
 RLcp54_112);
JTcp54_384_10 = ROcp54_49*(RLcp54_211+RLcp54_212)-ROcp54_59*(RLcp54_111+RLcp54_112)-RLcp54_184*ROcp54_59+RLcp54_284*
 ROcp54_49;
JTcp54_184_11 = ROcp54_210*(RLcp54_312+RLcp54_384)-ROcp54_310*(RLcp54_212+RLcp54_284);
JTcp54_284_11 = -(ROcp54_110*(RLcp54_312+RLcp54_384)-ROcp54_310*(RLcp54_112+RLcp54_184));
JTcp54_384_11 = ROcp54_110*(RLcp54_212+RLcp54_284)-ROcp54_210*(RLcp54_112+RLcp54_184);
JTcp54_184_12 = -(RLcp54_284*ROcp54_611-RLcp54_384*ROcp54_511);
JTcp54_284_12 = RLcp54_184*ROcp54_611-RLcp54_384*ROcp54_411;
JTcp54_384_12 = -(RLcp54_184*ROcp54_511-RLcp54_284*ROcp54_411);
ORcp54_184 = OMcp54_212*RLcp54_384-OMcp54_312*RLcp54_284;
ORcp54_284 = -(OMcp54_112*RLcp54_384-OMcp54_312*RLcp54_184);
ORcp54_384 = OMcp54_112*RLcp54_284-OMcp54_212*RLcp54_184;
VIcp54_184 = ORcp54_110+ORcp54_111+ORcp54_112+ORcp54_17+ORcp54_18+ORcp54_184+ORcp54_19+qd[1];
VIcp54_284 = ORcp54_210+ORcp54_211+ORcp54_212+ORcp54_27+ORcp54_28+ORcp54_284+ORcp54_29+qd[2];
VIcp54_384 = ORcp54_310+ORcp54_311+ORcp54_312+ORcp54_37+ORcp54_38+ORcp54_384+ORcp54_39+qd[3];
ACcp54_184 = qdd[1]+OMcp54_210*ORcp54_311+OMcp54_211*ORcp54_312+OMcp54_212*ORcp54_384+OMcp54_26*ORcp54_37+OMcp54_27*
 ORcp54_38+OMcp54_28*ORcp54_39+OMcp54_29*ORcp54_310-OMcp54_310*ORcp54_211-OMcp54_311*ORcp54_212-OMcp54_312*ORcp54_284-
 OMcp54_36*ORcp54_27-OMcp54_37*ORcp54_28-OMcp54_38*ORcp54_29-OMcp54_39*ORcp54_210+OPcp54_210*RLcp54_311+OPcp54_211*RLcp54_312
 +OPcp54_212*RLcp54_384+OPcp54_26*RLcp54_37+OPcp54_27*RLcp54_38+OPcp54_28*RLcp54_39+OPcp54_29*RLcp54_310-OPcp54_310*
 RLcp54_211-OPcp54_311*RLcp54_212-OPcp54_312*RLcp54_284-OPcp54_36*RLcp54_27-OPcp54_37*RLcp54_28-OPcp54_38*RLcp54_29-OPcp54_39
 *RLcp54_210;
ACcp54_284 = qdd[2]-OMcp54_110*ORcp54_311-OMcp54_111*ORcp54_312-OMcp54_112*ORcp54_384-OMcp54_16*ORcp54_37-OMcp54_17*
 ORcp54_38-OMcp54_18*ORcp54_39-OMcp54_19*ORcp54_310+OMcp54_310*ORcp54_111+OMcp54_311*ORcp54_112+OMcp54_312*ORcp54_184+
 OMcp54_36*ORcp54_17+OMcp54_37*ORcp54_18+OMcp54_38*ORcp54_19+OMcp54_39*ORcp54_110-OPcp54_110*RLcp54_311-OPcp54_111*RLcp54_312
 -OPcp54_112*RLcp54_384-OPcp54_16*RLcp54_37-OPcp54_17*RLcp54_38-OPcp54_18*RLcp54_39-OPcp54_19*RLcp54_310+OPcp54_310*
 RLcp54_111+OPcp54_311*RLcp54_112+OPcp54_312*RLcp54_184+OPcp54_36*RLcp54_17+OPcp54_37*RLcp54_18+OPcp54_38*RLcp54_19+OPcp54_39
 *RLcp54_110;
ACcp54_384 = qdd[3]+OMcp54_110*ORcp54_211+OMcp54_111*ORcp54_212+OMcp54_112*ORcp54_284+OMcp54_16*ORcp54_27+OMcp54_17*
 ORcp54_28+OMcp54_18*ORcp54_29+OMcp54_19*ORcp54_210-OMcp54_210*ORcp54_111-OMcp54_211*ORcp54_112-OMcp54_212*ORcp54_184-
 OMcp54_26*ORcp54_17-OMcp54_27*ORcp54_18-OMcp54_28*ORcp54_19-OMcp54_29*ORcp54_110+OPcp54_110*RLcp54_211+OPcp54_111*RLcp54_212
 +OPcp54_112*RLcp54_284+OPcp54_16*RLcp54_27+OPcp54_17*RLcp54_28+OPcp54_18*RLcp54_29+OPcp54_19*RLcp54_210-OPcp54_210*
 RLcp54_111-OPcp54_211*RLcp54_112-OPcp54_212*RLcp54_184-OPcp54_26*RLcp54_17-OPcp54_27*RLcp54_18-OPcp54_28*RLcp54_19-OPcp54_29
 *RLcp54_110;

// = = Block_1_0_0_55_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp54_184;
sens->P[2] = POcp54_284;
sens->P[3] = POcp54_384;
sens->R[1][1] = ROcp54_112;
sens->R[1][2] = ROcp54_212;
sens->R[1][3] = ROcp54_312;
sens->R[2][1] = ROcp54_411;
sens->R[2][2] = ROcp54_511;
sens->R[2][3] = ROcp54_611;
sens->R[3][1] = ROcp54_712;
sens->R[3][2] = ROcp54_812;
sens->R[3][3] = ROcp54_912;
sens->V[1] = VIcp54_184;
sens->V[2] = VIcp54_284;
sens->V[3] = VIcp54_384;
sens->OM[1] = OMcp54_112;
sens->OM[2] = OMcp54_212;
sens->OM[3] = OMcp54_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp54_184_5;
sens->J[1][6] = JTcp54_184_6;
sens->J[1][7] = JTcp54_184_7;
sens->J[1][8] = JTcp54_184_8;
sens->J[1][9] = JTcp54_184_9;
sens->J[1][10] = JTcp54_184_10;
sens->J[1][11] = JTcp54_184_11;
sens->J[1][12] = JTcp54_184_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp54_284_4;
sens->J[2][5] = JTcp54_284_5;
sens->J[2][6] = JTcp54_284_6;
sens->J[2][7] = JTcp54_284_7;
sens->J[2][8] = JTcp54_284_8;
sens->J[2][9] = JTcp54_284_9;
sens->J[2][10] = JTcp54_284_10;
sens->J[2][11] = JTcp54_284_11;
sens->J[2][12] = JTcp54_284_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp54_384_4;
sens->J[3][5] = JTcp54_384_5;
sens->J[3][6] = JTcp54_384_6;
sens->J[3][7] = JTcp54_384_7;
sens->J[3][8] = JTcp54_384_8;
sens->J[3][9] = JTcp54_384_9;
sens->J[3][10] = JTcp54_384_10;
sens->J[3][11] = JTcp54_384_11;
sens->J[3][12] = JTcp54_384_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp54_46;
sens->J[4][8] = ROcp54_17;
sens->J[4][9] = ROcp54_78;
sens->J[4][10] = ROcp54_49;
sens->J[4][11] = ROcp54_110;
sens->J[4][12] = ROcp54_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp54_85;
sens->J[5][7] = ROcp54_56;
sens->J[5][8] = ROcp54_27;
sens->J[5][9] = ROcp54_88;
sens->J[5][10] = ROcp54_59;
sens->J[5][11] = ROcp54_210;
sens->J[5][12] = ROcp54_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp54_95;
sens->J[6][7] = ROcp54_66;
sens->J[6][8] = ROcp54_37;
sens->J[6][9] = ROcp54_98;
sens->J[6][10] = ROcp54_69;
sens->J[6][11] = ROcp54_310;
sens->J[6][12] = ROcp54_611;
sens->A[1] = ACcp54_184;
sens->A[2] = ACcp54_284;
sens->A[3] = ACcp54_384;
sens->OMP[1] = OPcp54_112;
sens->OMP[2] = OPcp54_212;
sens->OMP[3] = OPcp54_312;
 
// 
break;
case 56:
 


// = = Block_1_0_0_56_0_1 = = 
 
// Sensor Kinematics 


ROcp55_25 = S4*S5;
ROcp55_35 = -C4*S5;
ROcp55_85 = -S4*C5;
ROcp55_95 = C4*C5;
ROcp55_16 = C5*C6;
ROcp55_26 = ROcp55_25*C6+C4*S6;
ROcp55_36 = ROcp55_35*C6+S4*S6;
ROcp55_46 = -C5*S6;
ROcp55_56 = -(ROcp55_25*S6-C4*C6);
ROcp55_66 = -(ROcp55_35*S6-S4*C6);
OMcp55_25 = qd[5]*C4;
OMcp55_35 = qd[5]*S4;
OMcp55_16 = qd[4]+qd[6]*S5;
OMcp55_26 = OMcp55_25+ROcp55_85*qd[6];
OMcp55_36 = OMcp55_35+ROcp55_95*qd[6];
OPcp55_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp55_26 = ROcp55_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp55_35*S5-ROcp55_95*qd[4]);
OPcp55_36 = ROcp55_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp55_25*S5-ROcp55_85*qd[4]);

// = = Block_1_0_0_56_0_2 = = 
 
// Sensor Kinematics 


ROcp55_17 = ROcp55_16*C7-S5*S7;
ROcp55_27 = ROcp55_26*C7-ROcp55_85*S7;
ROcp55_37 = ROcp55_36*C7-ROcp55_95*S7;
ROcp55_77 = ROcp55_16*S7+S5*C7;
ROcp55_87 = ROcp55_26*S7+ROcp55_85*C7;
ROcp55_97 = ROcp55_36*S7+ROcp55_95*C7;
ROcp55_48 = ROcp55_46*C8+ROcp55_77*S8;
ROcp55_58 = ROcp55_56*C8+ROcp55_87*S8;
ROcp55_68 = ROcp55_66*C8+ROcp55_97*S8;
ROcp55_78 = -(ROcp55_46*S8-ROcp55_77*C8);
ROcp55_88 = -(ROcp55_56*S8-ROcp55_87*C8);
ROcp55_98 = -(ROcp55_66*S8-ROcp55_97*C8);
ROcp55_19 = ROcp55_17*C9+ROcp55_48*S9;
ROcp55_29 = ROcp55_27*C9+ROcp55_58*S9;
ROcp55_39 = ROcp55_37*C9+ROcp55_68*S9;
ROcp55_49 = -(ROcp55_17*S9-ROcp55_48*C9);
ROcp55_59 = -(ROcp55_27*S9-ROcp55_58*C9);
ROcp55_69 = -(ROcp55_37*S9-ROcp55_68*C9);
ROcp55_110 = ROcp55_19*C10-ROcp55_78*S10;
ROcp55_210 = ROcp55_29*C10-ROcp55_88*S10;
ROcp55_310 = ROcp55_39*C10-ROcp55_98*S10;
ROcp55_710 = ROcp55_19*S10+ROcp55_78*C10;
ROcp55_810 = ROcp55_29*S10+ROcp55_88*C10;
ROcp55_910 = ROcp55_39*S10+ROcp55_98*C10;
ROcp55_411 = ROcp55_49*C11+ROcp55_710*S11;
ROcp55_511 = ROcp55_59*C11+ROcp55_810*S11;
ROcp55_611 = ROcp55_69*C11+ROcp55_910*S11;
ROcp55_711 = -(ROcp55_49*S11-ROcp55_710*C11);
ROcp55_811 = -(ROcp55_59*S11-ROcp55_810*C11);
ROcp55_911 = -(ROcp55_69*S11-ROcp55_910*C11);
ROcp55_112 = ROcp55_110*C12-ROcp55_711*S12;
ROcp55_212 = ROcp55_210*C12-ROcp55_811*S12;
ROcp55_312 = ROcp55_310*C12-ROcp55_911*S12;
ROcp55_712 = ROcp55_110*S12+ROcp55_711*C12;
ROcp55_812 = ROcp55_210*S12+ROcp55_811*C12;
ROcp55_912 = ROcp55_310*S12+ROcp55_911*C12;
RLcp55_17 = s->dpt[1][1]*ROcp55_16+s->dpt[3][1]*S5+ROcp55_46*s->dpt[2][1];
RLcp55_27 = s->dpt[1][1]*ROcp55_26+s->dpt[3][1]*ROcp55_85+ROcp55_56*s->dpt[2][1];
RLcp55_37 = s->dpt[1][1]*ROcp55_36+s->dpt[3][1]*ROcp55_95+ROcp55_66*s->dpt[2][1];
OMcp55_17 = OMcp55_16+ROcp55_46*qd[7];
OMcp55_27 = OMcp55_26+ROcp55_56*qd[7];
OMcp55_37 = OMcp55_36+ROcp55_66*qd[7];
ORcp55_17 = OMcp55_26*RLcp55_37-OMcp55_36*RLcp55_27;
ORcp55_27 = -(OMcp55_16*RLcp55_37-OMcp55_36*RLcp55_17);
ORcp55_37 = OMcp55_16*RLcp55_27-OMcp55_26*RLcp55_17;
OPcp55_17 = OPcp55_16+ROcp55_46*qdd[7]+qd[7]*(OMcp55_26*ROcp55_66-OMcp55_36*ROcp55_56);
OPcp55_27 = OPcp55_26+ROcp55_56*qdd[7]-qd[7]*(OMcp55_16*ROcp55_66-OMcp55_36*ROcp55_46);
OPcp55_37 = OPcp55_36+ROcp55_66*qdd[7]+qd[7]*(OMcp55_16*ROcp55_56-OMcp55_26*ROcp55_46);
RLcp55_18 = s->dpt[1][6]*ROcp55_17+s->dpt[3][6]*ROcp55_77+ROcp55_46*s->dpt[2][6];
RLcp55_28 = s->dpt[1][6]*ROcp55_27+s->dpt[3][6]*ROcp55_87+ROcp55_56*s->dpt[2][6];
RLcp55_38 = s->dpt[1][6]*ROcp55_37+s->dpt[3][6]*ROcp55_97+ROcp55_66*s->dpt[2][6];
OMcp55_18 = OMcp55_17+ROcp55_17*qd[8];
OMcp55_28 = OMcp55_27+ROcp55_27*qd[8];
OMcp55_38 = OMcp55_37+ROcp55_37*qd[8];
ORcp55_18 = OMcp55_27*RLcp55_38-OMcp55_37*RLcp55_28;
ORcp55_28 = -(OMcp55_17*RLcp55_38-OMcp55_37*RLcp55_18);
ORcp55_38 = OMcp55_17*RLcp55_28-OMcp55_27*RLcp55_18;
OPcp55_18 = OPcp55_17+ROcp55_17*qdd[8]+qd[8]*(OMcp55_27*ROcp55_37-OMcp55_37*ROcp55_27);
OPcp55_28 = OPcp55_27+ROcp55_27*qdd[8]-qd[8]*(OMcp55_17*ROcp55_37-OMcp55_37*ROcp55_17);
OPcp55_38 = OPcp55_37+ROcp55_37*qdd[8]+qd[8]*(OMcp55_17*ROcp55_27-OMcp55_27*ROcp55_17);
RLcp55_19 = s->dpt[1][8]*ROcp55_17+s->dpt[2][8]*ROcp55_48+ROcp55_78*s->dpt[3][8];
RLcp55_29 = s->dpt[1][8]*ROcp55_27+s->dpt[2][8]*ROcp55_58+ROcp55_88*s->dpt[3][8];
RLcp55_39 = s->dpt[1][8]*ROcp55_37+s->dpt[2][8]*ROcp55_68+ROcp55_98*s->dpt[3][8];
OMcp55_19 = OMcp55_18+ROcp55_78*qd[9];
OMcp55_29 = OMcp55_28+ROcp55_88*qd[9];
OMcp55_39 = OMcp55_38+ROcp55_98*qd[9];
ORcp55_19 = OMcp55_28*RLcp55_39-OMcp55_38*RLcp55_29;
ORcp55_29 = -(OMcp55_18*RLcp55_39-OMcp55_38*RLcp55_19);
ORcp55_39 = OMcp55_18*RLcp55_29-OMcp55_28*RLcp55_19;
OPcp55_19 = OPcp55_18+ROcp55_78*qdd[9]+qd[9]*(OMcp55_28*ROcp55_98-OMcp55_38*ROcp55_88);
OPcp55_29 = OPcp55_28+ROcp55_88*qdd[9]-qd[9]*(OMcp55_18*ROcp55_98-OMcp55_38*ROcp55_78);
OPcp55_39 = OPcp55_38+ROcp55_98*qdd[9]+qd[9]*(OMcp55_18*ROcp55_88-OMcp55_28*ROcp55_78);
RLcp55_110 = s->dpt[1][10]*ROcp55_19+s->dpt[2][10]*ROcp55_49+ROcp55_78*s->dpt[3][10];
RLcp55_210 = s->dpt[1][10]*ROcp55_29+s->dpt[2][10]*ROcp55_59+ROcp55_88*s->dpt[3][10];
RLcp55_310 = s->dpt[1][10]*ROcp55_39+s->dpt[2][10]*ROcp55_69+ROcp55_98*s->dpt[3][10];
OMcp55_110 = OMcp55_19+ROcp55_49*qd[10];
OMcp55_210 = OMcp55_29+ROcp55_59*qd[10];
OMcp55_310 = OMcp55_39+ROcp55_69*qd[10];
ORcp55_110 = OMcp55_29*RLcp55_310-OMcp55_39*RLcp55_210;
ORcp55_210 = -(OMcp55_19*RLcp55_310-OMcp55_39*RLcp55_110);
ORcp55_310 = OMcp55_19*RLcp55_210-OMcp55_29*RLcp55_110;
OPcp55_110 = OPcp55_19+ROcp55_49*qdd[10]+qd[10]*(OMcp55_29*ROcp55_69-OMcp55_39*ROcp55_59);
OPcp55_210 = OPcp55_29+ROcp55_59*qdd[10]-qd[10]*(OMcp55_19*ROcp55_69-OMcp55_39*ROcp55_49);
OPcp55_310 = OPcp55_39+ROcp55_69*qdd[10]+qd[10]*(OMcp55_19*ROcp55_59-OMcp55_29*ROcp55_49);
RLcp55_111 = s->dpt[1][12]*ROcp55_110+s->dpt[2][12]*ROcp55_49+ROcp55_710*s->dpt[3][12];
RLcp55_211 = s->dpt[1][12]*ROcp55_210+s->dpt[2][12]*ROcp55_59+ROcp55_810*s->dpt[3][12];
RLcp55_311 = s->dpt[1][12]*ROcp55_310+s->dpt[2][12]*ROcp55_69+ROcp55_910*s->dpt[3][12];
OMcp55_111 = OMcp55_110+ROcp55_110*qd[11];
OMcp55_211 = OMcp55_210+ROcp55_210*qd[11];
OMcp55_311 = OMcp55_310+ROcp55_310*qd[11];
ORcp55_111 = OMcp55_210*RLcp55_311-OMcp55_310*RLcp55_211;
ORcp55_211 = -(OMcp55_110*RLcp55_311-OMcp55_310*RLcp55_111);
ORcp55_311 = OMcp55_110*RLcp55_211-OMcp55_210*RLcp55_111;
OPcp55_111 = OPcp55_110+ROcp55_110*qdd[11]+qd[11]*(OMcp55_210*ROcp55_310-OMcp55_310*ROcp55_210);
OPcp55_211 = OPcp55_210+ROcp55_210*qdd[11]-qd[11]*(OMcp55_110*ROcp55_310-OMcp55_310*ROcp55_110);
OPcp55_311 = OPcp55_310+ROcp55_310*qdd[11]+qd[11]*(OMcp55_110*ROcp55_210-OMcp55_210*ROcp55_110);
RLcp55_112 = s->dpt[1][14]*ROcp55_110+s->dpt[2][14]*ROcp55_411+s->dpt[3][14]*ROcp55_711;
RLcp55_212 = s->dpt[1][14]*ROcp55_210+s->dpt[2][14]*ROcp55_511+s->dpt[3][14]*ROcp55_811;
RLcp55_312 = s->dpt[1][14]*ROcp55_310+s->dpt[2][14]*ROcp55_611+s->dpt[3][14]*ROcp55_911;
OMcp55_112 = OMcp55_111+ROcp55_411*qd[12];
OMcp55_212 = OMcp55_211+ROcp55_511*qd[12];
OMcp55_312 = OMcp55_311+ROcp55_611*qd[12];
ORcp55_112 = OMcp55_211*RLcp55_312-OMcp55_311*RLcp55_212;
ORcp55_212 = -(OMcp55_111*RLcp55_312-OMcp55_311*RLcp55_112);
ORcp55_312 = OMcp55_111*RLcp55_212-OMcp55_211*RLcp55_112;
OPcp55_112 = OPcp55_111+ROcp55_411*qdd[12]+qd[12]*(OMcp55_211*ROcp55_611-OMcp55_311*ROcp55_511);
OPcp55_212 = OPcp55_211+ROcp55_511*qdd[12]-qd[12]*(OMcp55_111*ROcp55_611-OMcp55_311*ROcp55_411);
OPcp55_312 = OPcp55_311+ROcp55_611*qdd[12]+qd[12]*(OMcp55_111*ROcp55_511-OMcp55_211*ROcp55_411);
RLcp55_185 = ROcp55_112*s->dpt[1][21]+ROcp55_411*s->dpt[2][21]+ROcp55_712*s->dpt[3][21];
RLcp55_285 = ROcp55_212*s->dpt[1][21]+ROcp55_511*s->dpt[2][21]+ROcp55_812*s->dpt[3][21];
RLcp55_385 = ROcp55_312*s->dpt[1][21]+ROcp55_611*s->dpt[2][21]+ROcp55_912*s->dpt[3][21];
POcp55_185 = RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_185+RLcp55_19+q[1];
POcp55_285 = RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_27+RLcp55_28+RLcp55_285+RLcp55_29+q[2];
POcp55_385 = RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_37+RLcp55_38+RLcp55_385+RLcp55_39+q[3];
JTcp55_285_4 = -(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_37+RLcp55_38+RLcp55_385+RLcp55_39);
JTcp55_385_4 = RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_27+RLcp55_28+RLcp55_285+RLcp55_29;
JTcp55_185_5 = C4*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_37+RLcp55_38+RLcp55_39)-S4*(RLcp55_210+RLcp55_29)-S4*(
 RLcp55_211+RLcp55_212)-S4*(RLcp55_27+RLcp55_28)-RLcp55_285*S4+RLcp55_385*C4;
JTcp55_285_5 = S4*(RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_185+RLcp55_19);
JTcp55_385_5 = -C4*(RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_185+RLcp55_19);
JTcp55_185_6 = ROcp55_85*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_37+RLcp55_38+RLcp55_39)-ROcp55_95*(RLcp55_210+
 RLcp55_29)-ROcp55_95*(RLcp55_211+RLcp55_212)-ROcp55_95*(RLcp55_27+RLcp55_28)-RLcp55_285*ROcp55_95+RLcp55_385*ROcp55_85;
JTcp55_285_6 = -(RLcp55_385*S5-ROcp55_95*(RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_185+RLcp55_19)+
 S5*(RLcp55_310+RLcp55_39)+S5*(RLcp55_311+RLcp55_312)+S5*(RLcp55_37+RLcp55_38));
JTcp55_385_6 = RLcp55_285*S5-ROcp55_85*(RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_185+RLcp55_19)+S5*
 (RLcp55_210+RLcp55_29)+S5*(RLcp55_211+RLcp55_212)+S5*(RLcp55_27+RLcp55_28);
JTcp55_185_7 = ROcp55_56*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_38+RLcp55_385+RLcp55_39)-ROcp55_66*(RLcp55_210+
 RLcp55_211)-ROcp55_66*(RLcp55_212+RLcp55_285)-ROcp55_66*(RLcp55_28+RLcp55_29);
JTcp55_285_7 = -(ROcp55_46*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_38+RLcp55_385+RLcp55_39)-ROcp55_66*(RLcp55_110+
 RLcp55_111)-ROcp55_66*(RLcp55_112+RLcp55_185)-ROcp55_66*(RLcp55_18+RLcp55_19));
JTcp55_385_7 = ROcp55_46*(RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_28+RLcp55_285+RLcp55_29)-ROcp55_56*(RLcp55_110+
 RLcp55_111)-ROcp55_56*(RLcp55_112+RLcp55_185)-ROcp55_56*(RLcp55_18+RLcp55_19);
JTcp55_185_8 = ROcp55_27*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_39)-ROcp55_37*(RLcp55_210+RLcp55_29)-ROcp55_37*(
 RLcp55_211+RLcp55_212)-RLcp55_285*ROcp55_37+RLcp55_385*ROcp55_27;
JTcp55_285_8 = RLcp55_185*ROcp55_37-RLcp55_385*ROcp55_17-ROcp55_17*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_39)+
 ROcp55_37*(RLcp55_110+RLcp55_19)+ROcp55_37*(RLcp55_111+RLcp55_112);
JTcp55_385_8 = ROcp55_17*(RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_29)-ROcp55_27*(RLcp55_110+RLcp55_19)-ROcp55_27*(
 RLcp55_111+RLcp55_112)-RLcp55_185*ROcp55_27+RLcp55_285*ROcp55_17;
JTcp55_185_9 = ROcp55_88*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_385)-ROcp55_98*(RLcp55_210+RLcp55_211)-ROcp55_98*(
 RLcp55_212+RLcp55_285);
JTcp55_285_9 = -(ROcp55_78*(RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_385)-ROcp55_98*(RLcp55_110+RLcp55_111)-ROcp55_98*(
 RLcp55_112+RLcp55_185));
JTcp55_385_9 = ROcp55_78*(RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_285)-ROcp55_88*(RLcp55_110+RLcp55_111)-ROcp55_88*(
 RLcp55_112+RLcp55_185);
JTcp55_185_10 = ROcp55_59*(RLcp55_311+RLcp55_312)-ROcp55_69*(RLcp55_211+RLcp55_212)-RLcp55_285*ROcp55_69+RLcp55_385*
 ROcp55_59;
JTcp55_285_10 = RLcp55_185*ROcp55_69-RLcp55_385*ROcp55_49-ROcp55_49*(RLcp55_311+RLcp55_312)+ROcp55_69*(RLcp55_111+
 RLcp55_112);
JTcp55_385_10 = ROcp55_49*(RLcp55_211+RLcp55_212)-ROcp55_59*(RLcp55_111+RLcp55_112)-RLcp55_185*ROcp55_59+RLcp55_285*
 ROcp55_49;
JTcp55_185_11 = ROcp55_210*(RLcp55_312+RLcp55_385)-ROcp55_310*(RLcp55_212+RLcp55_285);
JTcp55_285_11 = -(ROcp55_110*(RLcp55_312+RLcp55_385)-ROcp55_310*(RLcp55_112+RLcp55_185));
JTcp55_385_11 = ROcp55_110*(RLcp55_212+RLcp55_285)-ROcp55_210*(RLcp55_112+RLcp55_185);
JTcp55_185_12 = -(RLcp55_285*ROcp55_611-RLcp55_385*ROcp55_511);
JTcp55_285_12 = RLcp55_185*ROcp55_611-RLcp55_385*ROcp55_411;
JTcp55_385_12 = -(RLcp55_185*ROcp55_511-RLcp55_285*ROcp55_411);
ORcp55_185 = OMcp55_212*RLcp55_385-OMcp55_312*RLcp55_285;
ORcp55_285 = -(OMcp55_112*RLcp55_385-OMcp55_312*RLcp55_185);
ORcp55_385 = OMcp55_112*RLcp55_285-OMcp55_212*RLcp55_185;
VIcp55_185 = ORcp55_110+ORcp55_111+ORcp55_112+ORcp55_17+ORcp55_18+ORcp55_185+ORcp55_19+qd[1];
VIcp55_285 = ORcp55_210+ORcp55_211+ORcp55_212+ORcp55_27+ORcp55_28+ORcp55_285+ORcp55_29+qd[2];
VIcp55_385 = ORcp55_310+ORcp55_311+ORcp55_312+ORcp55_37+ORcp55_38+ORcp55_385+ORcp55_39+qd[3];
ACcp55_185 = qdd[1]+OMcp55_210*ORcp55_311+OMcp55_211*ORcp55_312+OMcp55_212*ORcp55_385+OMcp55_26*ORcp55_37+OMcp55_27*
 ORcp55_38+OMcp55_28*ORcp55_39+OMcp55_29*ORcp55_310-OMcp55_310*ORcp55_211-OMcp55_311*ORcp55_212-OMcp55_312*ORcp55_285-
 OMcp55_36*ORcp55_27-OMcp55_37*ORcp55_28-OMcp55_38*ORcp55_29-OMcp55_39*ORcp55_210+OPcp55_210*RLcp55_311+OPcp55_211*RLcp55_312
 +OPcp55_212*RLcp55_385+OPcp55_26*RLcp55_37+OPcp55_27*RLcp55_38+OPcp55_28*RLcp55_39+OPcp55_29*RLcp55_310-OPcp55_310*
 RLcp55_211-OPcp55_311*RLcp55_212-OPcp55_312*RLcp55_285-OPcp55_36*RLcp55_27-OPcp55_37*RLcp55_28-OPcp55_38*RLcp55_29-OPcp55_39
 *RLcp55_210;
ACcp55_285 = qdd[2]-OMcp55_110*ORcp55_311-OMcp55_111*ORcp55_312-OMcp55_112*ORcp55_385-OMcp55_16*ORcp55_37-OMcp55_17*
 ORcp55_38-OMcp55_18*ORcp55_39-OMcp55_19*ORcp55_310+OMcp55_310*ORcp55_111+OMcp55_311*ORcp55_112+OMcp55_312*ORcp55_185+
 OMcp55_36*ORcp55_17+OMcp55_37*ORcp55_18+OMcp55_38*ORcp55_19+OMcp55_39*ORcp55_110-OPcp55_110*RLcp55_311-OPcp55_111*RLcp55_312
 -OPcp55_112*RLcp55_385-OPcp55_16*RLcp55_37-OPcp55_17*RLcp55_38-OPcp55_18*RLcp55_39-OPcp55_19*RLcp55_310+OPcp55_310*
 RLcp55_111+OPcp55_311*RLcp55_112+OPcp55_312*RLcp55_185+OPcp55_36*RLcp55_17+OPcp55_37*RLcp55_18+OPcp55_38*RLcp55_19+OPcp55_39
 *RLcp55_110;
ACcp55_385 = qdd[3]+OMcp55_110*ORcp55_211+OMcp55_111*ORcp55_212+OMcp55_112*ORcp55_285+OMcp55_16*ORcp55_27+OMcp55_17*
 ORcp55_28+OMcp55_18*ORcp55_29+OMcp55_19*ORcp55_210-OMcp55_210*ORcp55_111-OMcp55_211*ORcp55_112-OMcp55_212*ORcp55_185-
 OMcp55_26*ORcp55_17-OMcp55_27*ORcp55_18-OMcp55_28*ORcp55_19-OMcp55_29*ORcp55_110+OPcp55_110*RLcp55_211+OPcp55_111*RLcp55_212
 +OPcp55_112*RLcp55_285+OPcp55_16*RLcp55_27+OPcp55_17*RLcp55_28+OPcp55_18*RLcp55_29+OPcp55_19*RLcp55_210-OPcp55_210*
 RLcp55_111-OPcp55_211*RLcp55_112-OPcp55_212*RLcp55_185-OPcp55_26*RLcp55_17-OPcp55_27*RLcp55_18-OPcp55_28*RLcp55_19-OPcp55_29
 *RLcp55_110;

// = = Block_1_0_0_56_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp55_185;
sens->P[2] = POcp55_285;
sens->P[3] = POcp55_385;
sens->R[1][1] = ROcp55_112;
sens->R[1][2] = ROcp55_212;
sens->R[1][3] = ROcp55_312;
sens->R[2][1] = ROcp55_411;
sens->R[2][2] = ROcp55_511;
sens->R[2][3] = ROcp55_611;
sens->R[3][1] = ROcp55_712;
sens->R[3][2] = ROcp55_812;
sens->R[3][3] = ROcp55_912;
sens->V[1] = VIcp55_185;
sens->V[2] = VIcp55_285;
sens->V[3] = VIcp55_385;
sens->OM[1] = OMcp55_112;
sens->OM[2] = OMcp55_212;
sens->OM[3] = OMcp55_312;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp55_185_5;
sens->J[1][6] = JTcp55_185_6;
sens->J[1][7] = JTcp55_185_7;
sens->J[1][8] = JTcp55_185_8;
sens->J[1][9] = JTcp55_185_9;
sens->J[1][10] = JTcp55_185_10;
sens->J[1][11] = JTcp55_185_11;
sens->J[1][12] = JTcp55_185_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp55_285_4;
sens->J[2][5] = JTcp55_285_5;
sens->J[2][6] = JTcp55_285_6;
sens->J[2][7] = JTcp55_285_7;
sens->J[2][8] = JTcp55_285_8;
sens->J[2][9] = JTcp55_285_9;
sens->J[2][10] = JTcp55_285_10;
sens->J[2][11] = JTcp55_285_11;
sens->J[2][12] = JTcp55_285_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp55_385_4;
sens->J[3][5] = JTcp55_385_5;
sens->J[3][6] = JTcp55_385_6;
sens->J[3][7] = JTcp55_385_7;
sens->J[3][8] = JTcp55_385_8;
sens->J[3][9] = JTcp55_385_9;
sens->J[3][10] = JTcp55_385_10;
sens->J[3][11] = JTcp55_385_11;
sens->J[3][12] = JTcp55_385_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][7] = ROcp55_46;
sens->J[4][8] = ROcp55_17;
sens->J[4][9] = ROcp55_78;
sens->J[4][10] = ROcp55_49;
sens->J[4][11] = ROcp55_110;
sens->J[4][12] = ROcp55_411;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp55_85;
sens->J[5][7] = ROcp55_56;
sens->J[5][8] = ROcp55_27;
sens->J[5][9] = ROcp55_88;
sens->J[5][10] = ROcp55_59;
sens->J[5][11] = ROcp55_210;
sens->J[5][12] = ROcp55_511;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp55_95;
sens->J[6][7] = ROcp55_66;
sens->J[6][8] = ROcp55_37;
sens->J[6][9] = ROcp55_98;
sens->J[6][10] = ROcp55_69;
sens->J[6][11] = ROcp55_310;
sens->J[6][12] = ROcp55_611;
sens->A[1] = ACcp55_185;
sens->A[2] = ACcp55_285;
sens->A[3] = ACcp55_385;
sens->OMP[1] = OPcp55_112;
sens->OMP[2] = OPcp55_212;
sens->OMP[3] = OPcp55_312;
 
// 
break;
case 57:
 


// = = Block_1_0_0_57_0_1 = = 
 
// Sensor Kinematics 


ROcp56_25 = S4*S5;
ROcp56_35 = -C4*S5;
ROcp56_85 = -S4*C5;
ROcp56_95 = C4*C5;
ROcp56_16 = C5*C6;
ROcp56_26 = ROcp56_25*C6+C4*S6;
ROcp56_36 = ROcp56_35*C6+S4*S6;
ROcp56_46 = -C5*S6;
ROcp56_56 = -(ROcp56_25*S6-C4*C6);
ROcp56_66 = -(ROcp56_35*S6-S4*C6);
OMcp56_25 = qd[5]*C4;
OMcp56_35 = qd[5]*S4;
OMcp56_16 = qd[4]+qd[6]*S5;
OMcp56_26 = OMcp56_25+ROcp56_85*qd[6];
OMcp56_36 = OMcp56_35+ROcp56_95*qd[6];
OPcp56_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp56_26 = ROcp56_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp56_35*S5-ROcp56_95*qd[4]);
OPcp56_36 = ROcp56_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp56_25*S5-ROcp56_85*qd[4]);

// = = Block_1_0_0_57_0_3 = = 
 
// Sensor Kinematics 


ROcp56_113 = ROcp56_16*C13-S13*S5;
ROcp56_213 = ROcp56_26*C13-ROcp56_85*S13;
ROcp56_313 = ROcp56_36*C13-ROcp56_95*S13;
ROcp56_713 = ROcp56_16*S13+C13*S5;
ROcp56_813 = ROcp56_26*S13+ROcp56_85*C13;
ROcp56_913 = ROcp56_36*S13+ROcp56_95*C13;
ROcp56_414 = ROcp56_46*C14+ROcp56_713*S14;
ROcp56_514 = ROcp56_56*C14+ROcp56_813*S14;
ROcp56_614 = ROcp56_66*C14+ROcp56_913*S14;
ROcp56_714 = -(ROcp56_46*S14-ROcp56_713*C14);
ROcp56_814 = -(ROcp56_56*S14-ROcp56_813*C14);
ROcp56_914 = -(ROcp56_66*S14-ROcp56_913*C14);
ROcp56_115 = ROcp56_113*C15+ROcp56_414*S15;
ROcp56_215 = ROcp56_213*C15+ROcp56_514*S15;
ROcp56_315 = ROcp56_313*C15+ROcp56_614*S15;
ROcp56_415 = -(ROcp56_113*S15-ROcp56_414*C15);
ROcp56_515 = -(ROcp56_213*S15-ROcp56_514*C15);
ROcp56_615 = -(ROcp56_313*S15-ROcp56_614*C15);
ROcp56_116 = ROcp56_115*C16-ROcp56_714*S16;
ROcp56_216 = ROcp56_215*C16-ROcp56_814*S16;
ROcp56_316 = ROcp56_315*C16-ROcp56_914*S16;
ROcp56_716 = ROcp56_115*S16+ROcp56_714*C16;
ROcp56_816 = ROcp56_215*S16+ROcp56_814*C16;
ROcp56_916 = ROcp56_315*S16+ROcp56_914*C16;
ROcp56_417 = ROcp56_415*C17+ROcp56_716*S17;
ROcp56_517 = ROcp56_515*C17+ROcp56_816*S17;
ROcp56_617 = ROcp56_615*C17+ROcp56_916*S17;
ROcp56_717 = -(ROcp56_415*S17-ROcp56_716*C17);
ROcp56_817 = -(ROcp56_515*S17-ROcp56_816*C17);
ROcp56_917 = -(ROcp56_615*S17-ROcp56_916*C17);
ROcp56_118 = ROcp56_116*C18-ROcp56_717*S18;
ROcp56_218 = ROcp56_216*C18-ROcp56_817*S18;
ROcp56_318 = ROcp56_316*C18-ROcp56_917*S18;
ROcp56_718 = ROcp56_116*S18+ROcp56_717*C18;
ROcp56_818 = ROcp56_216*S18+ROcp56_817*C18;
ROcp56_918 = ROcp56_316*S18+ROcp56_917*C18;
RLcp56_113 = s->dpt[1][2]*ROcp56_16+s->dpt[3][2]*S5+ROcp56_46*s->dpt[2][2];
RLcp56_213 = s->dpt[1][2]*ROcp56_26+s->dpt[3][2]*ROcp56_85+ROcp56_56*s->dpt[2][2];
RLcp56_313 = s->dpt[1][2]*ROcp56_36+s->dpt[3][2]*ROcp56_95+ROcp56_66*s->dpt[2][2];
OMcp56_113 = OMcp56_16+ROcp56_46*qd[13];
OMcp56_213 = OMcp56_26+ROcp56_56*qd[13];
OMcp56_313 = OMcp56_36+ROcp56_66*qd[13];
ORcp56_113 = OMcp56_26*RLcp56_313-OMcp56_36*RLcp56_213;
ORcp56_213 = -(OMcp56_16*RLcp56_313-OMcp56_36*RLcp56_113);
ORcp56_313 = OMcp56_16*RLcp56_213-OMcp56_26*RLcp56_113;
OPcp56_113 = OPcp56_16+ROcp56_46*qdd[13]+qd[13]*(OMcp56_26*ROcp56_66-OMcp56_36*ROcp56_56);
OPcp56_213 = OPcp56_26+ROcp56_56*qdd[13]-qd[13]*(OMcp56_16*ROcp56_66-OMcp56_36*ROcp56_46);
OPcp56_313 = OPcp56_36+ROcp56_66*qdd[13]+qd[13]*(OMcp56_16*ROcp56_56-OMcp56_26*ROcp56_46);
RLcp56_114 = s->dpt[1][22]*ROcp56_113+s->dpt[3][22]*ROcp56_713+ROcp56_46*s->dpt[2][22];
RLcp56_214 = s->dpt[1][22]*ROcp56_213+s->dpt[3][22]*ROcp56_813+ROcp56_56*s->dpt[2][22];
RLcp56_314 = s->dpt[1][22]*ROcp56_313+s->dpt[3][22]*ROcp56_913+ROcp56_66*s->dpt[2][22];
OMcp56_114 = OMcp56_113+ROcp56_113*qd[14];
OMcp56_214 = OMcp56_213+ROcp56_213*qd[14];
OMcp56_314 = OMcp56_313+ROcp56_313*qd[14];
ORcp56_114 = OMcp56_213*RLcp56_314-OMcp56_313*RLcp56_214;
ORcp56_214 = -(OMcp56_113*RLcp56_314-OMcp56_313*RLcp56_114);
ORcp56_314 = OMcp56_113*RLcp56_214-OMcp56_213*RLcp56_114;
OPcp56_114 = OPcp56_113+ROcp56_113*qdd[14]+qd[14]*(OMcp56_213*ROcp56_313-OMcp56_313*ROcp56_213);
OPcp56_214 = OPcp56_213+ROcp56_213*qdd[14]-qd[14]*(OMcp56_113*ROcp56_313-OMcp56_313*ROcp56_113);
OPcp56_314 = OPcp56_313+ROcp56_313*qdd[14]+qd[14]*(OMcp56_113*ROcp56_213-OMcp56_213*ROcp56_113);
RLcp56_115 = s->dpt[1][24]*ROcp56_113+s->dpt[2][24]*ROcp56_414+ROcp56_714*s->dpt[3][24];
RLcp56_215 = s->dpt[1][24]*ROcp56_213+s->dpt[2][24]*ROcp56_514+ROcp56_814*s->dpt[3][24];
RLcp56_315 = s->dpt[1][24]*ROcp56_313+s->dpt[2][24]*ROcp56_614+ROcp56_914*s->dpt[3][24];
OMcp56_115 = OMcp56_114+ROcp56_714*qd[15];
OMcp56_215 = OMcp56_214+ROcp56_814*qd[15];
OMcp56_315 = OMcp56_314+ROcp56_914*qd[15];
ORcp56_115 = OMcp56_214*RLcp56_315-OMcp56_314*RLcp56_215;
ORcp56_215 = -(OMcp56_114*RLcp56_315-OMcp56_314*RLcp56_115);
ORcp56_315 = OMcp56_114*RLcp56_215-OMcp56_214*RLcp56_115;
OPcp56_115 = OPcp56_114+ROcp56_714*qdd[15]+qd[15]*(OMcp56_214*ROcp56_914-OMcp56_314*ROcp56_814);
OPcp56_215 = OPcp56_214+ROcp56_814*qdd[15]-qd[15]*(OMcp56_114*ROcp56_914-OMcp56_314*ROcp56_714);
OPcp56_315 = OPcp56_314+ROcp56_914*qdd[15]+qd[15]*(OMcp56_114*ROcp56_814-OMcp56_214*ROcp56_714);
RLcp56_116 = s->dpt[1][26]*ROcp56_115+s->dpt[2][26]*ROcp56_415+ROcp56_714*s->dpt[3][26];
RLcp56_216 = s->dpt[1][26]*ROcp56_215+s->dpt[2][26]*ROcp56_515+ROcp56_814*s->dpt[3][26];
RLcp56_316 = s->dpt[1][26]*ROcp56_315+s->dpt[2][26]*ROcp56_615+ROcp56_914*s->dpt[3][26];
OMcp56_116 = OMcp56_115+ROcp56_415*qd[16];
OMcp56_216 = OMcp56_215+ROcp56_515*qd[16];
OMcp56_316 = OMcp56_315+ROcp56_615*qd[16];
ORcp56_116 = OMcp56_215*RLcp56_316-OMcp56_315*RLcp56_216;
ORcp56_216 = -(OMcp56_115*RLcp56_316-OMcp56_315*RLcp56_116);
ORcp56_316 = OMcp56_115*RLcp56_216-OMcp56_215*RLcp56_116;
OPcp56_116 = OPcp56_115+ROcp56_415*qdd[16]+qd[16]*(OMcp56_215*ROcp56_615-OMcp56_315*ROcp56_515);
OPcp56_216 = OPcp56_215+ROcp56_515*qdd[16]-qd[16]*(OMcp56_115*ROcp56_615-OMcp56_315*ROcp56_415);
OPcp56_316 = OPcp56_315+ROcp56_615*qdd[16]+qd[16]*(OMcp56_115*ROcp56_515-OMcp56_215*ROcp56_415);
RLcp56_117 = s->dpt[1][28]*ROcp56_116+s->dpt[2][28]*ROcp56_415+ROcp56_716*s->dpt[3][28];
RLcp56_217 = s->dpt[1][28]*ROcp56_216+s->dpt[2][28]*ROcp56_515+ROcp56_816*s->dpt[3][28];
RLcp56_317 = s->dpt[1][28]*ROcp56_316+s->dpt[2][28]*ROcp56_615+ROcp56_916*s->dpt[3][28];
OMcp56_117 = OMcp56_116+ROcp56_116*qd[17];
OMcp56_217 = OMcp56_216+ROcp56_216*qd[17];
OMcp56_317 = OMcp56_316+ROcp56_316*qd[17];
ORcp56_117 = OMcp56_216*RLcp56_317-OMcp56_316*RLcp56_217;
ORcp56_217 = -(OMcp56_116*RLcp56_317-OMcp56_316*RLcp56_117);
ORcp56_317 = OMcp56_116*RLcp56_217-OMcp56_216*RLcp56_117;
OPcp56_117 = OPcp56_116+ROcp56_116*qdd[17]+qd[17]*(OMcp56_216*ROcp56_316-OMcp56_316*ROcp56_216);
OPcp56_217 = OPcp56_216+ROcp56_216*qdd[17]-qd[17]*(OMcp56_116*ROcp56_316-OMcp56_316*ROcp56_116);
OPcp56_317 = OPcp56_316+ROcp56_316*qdd[17]+qd[17]*(OMcp56_116*ROcp56_216-OMcp56_216*ROcp56_116);
RLcp56_118 = s->dpt[1][30]*ROcp56_116+s->dpt[2][30]*ROcp56_417+s->dpt[3][30]*ROcp56_717;
RLcp56_218 = s->dpt[1][30]*ROcp56_216+s->dpt[2][30]*ROcp56_517+s->dpt[3][30]*ROcp56_817;
RLcp56_318 = s->dpt[1][30]*ROcp56_316+s->dpt[2][30]*ROcp56_617+s->dpt[3][30]*ROcp56_917;
OMcp56_118 = OMcp56_117+ROcp56_417*qd[18];
OMcp56_218 = OMcp56_217+ROcp56_517*qd[18];
OMcp56_318 = OMcp56_317+ROcp56_617*qd[18];
ORcp56_118 = OMcp56_217*RLcp56_318-OMcp56_317*RLcp56_218;
ORcp56_218 = -(OMcp56_117*RLcp56_318-OMcp56_317*RLcp56_118);
ORcp56_318 = OMcp56_117*RLcp56_218-OMcp56_217*RLcp56_118;
OPcp56_118 = OPcp56_117+ROcp56_417*qdd[18]+qd[18]*(OMcp56_217*ROcp56_617-OMcp56_317*ROcp56_517);
OPcp56_218 = OPcp56_217+ROcp56_517*qdd[18]-qd[18]*(OMcp56_117*ROcp56_617-OMcp56_317*ROcp56_417);
OPcp56_318 = OPcp56_317+ROcp56_617*qdd[18]+qd[18]*(OMcp56_117*ROcp56_517-OMcp56_217*ROcp56_417);
RLcp56_186 = s->dpt[1][33]*ROcp56_118+s->dpt[2][33]*ROcp56_417+ROcp56_718*s->dpt[3][33];
RLcp56_286 = s->dpt[1][33]*ROcp56_218+s->dpt[2][33]*ROcp56_517+ROcp56_818*s->dpt[3][33];
RLcp56_386 = s->dpt[1][33]*ROcp56_318+s->dpt[2][33]*ROcp56_617+ROcp56_918*s->dpt[3][33];
POcp56_186 = RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_186+q[1];
POcp56_286 = RLcp56_213+RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_286+q[2];
POcp56_386 = RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_386+q[3];
JTcp56_286_4 = -(RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_386);
JTcp56_386_4 = RLcp56_213+RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_286;
JTcp56_186_5 = C4*(RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)-S4*(RLcp56_213+RLcp56_214)-S4*(
 RLcp56_215+RLcp56_216)-S4*(RLcp56_217+RLcp56_218)-RLcp56_286*S4+RLcp56_386*C4;
JTcp56_286_5 = S4*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_186);
JTcp56_386_5 = -C4*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_186);
JTcp56_186_6 = ROcp56_85*(RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)-ROcp56_95*(RLcp56_213+
 RLcp56_214)-ROcp56_95*(RLcp56_215+RLcp56_216)-ROcp56_95*(RLcp56_217+RLcp56_218)-RLcp56_286*ROcp56_95+RLcp56_386*ROcp56_85;
JTcp56_286_6 = -(RLcp56_386*S5-ROcp56_95*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_186
 )+S5*(RLcp56_313+RLcp56_314)+S5*(RLcp56_315+RLcp56_316)+S5*(RLcp56_317+RLcp56_318));
JTcp56_386_6 = RLcp56_286*S5-ROcp56_85*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_186)+
 S5*(RLcp56_213+RLcp56_214)+S5*(RLcp56_215+RLcp56_216)+S5*(RLcp56_217+RLcp56_218);
JTcp56_186_7 = ROcp56_56*(RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_386)-ROcp56_66*(RLcp56_214+
 RLcp56_215)-ROcp56_66*(RLcp56_216+RLcp56_217)-ROcp56_66*(RLcp56_218+RLcp56_286);
JTcp56_286_7 = -(ROcp56_46*(RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_386)-ROcp56_66*(RLcp56_114+
 RLcp56_115)-ROcp56_66*(RLcp56_116+RLcp56_117)-ROcp56_66*(RLcp56_118+RLcp56_186));
JTcp56_386_7 = ROcp56_46*(RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_286)-ROcp56_56*(RLcp56_114+
 RLcp56_115)-ROcp56_56*(RLcp56_116+RLcp56_117)-ROcp56_56*(RLcp56_118+RLcp56_186);
JTcp56_186_8 = ROcp56_213*(RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)-ROcp56_313*(RLcp56_215+RLcp56_216)-ROcp56_313*
 (RLcp56_217+RLcp56_218)-RLcp56_286*ROcp56_313+RLcp56_386*ROcp56_213;
JTcp56_286_8 = RLcp56_186*ROcp56_313-RLcp56_386*ROcp56_113-ROcp56_113*(RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)+
 ROcp56_313*(RLcp56_115+RLcp56_116)+ROcp56_313*(RLcp56_117+RLcp56_118);
JTcp56_386_8 = ROcp56_113*(RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218)-ROcp56_213*(RLcp56_115+RLcp56_116)-ROcp56_213*
 (RLcp56_117+RLcp56_118)-RLcp56_186*ROcp56_213+RLcp56_286*ROcp56_113;
JTcp56_186_9 = ROcp56_814*(RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_386)-ROcp56_914*(RLcp56_216+RLcp56_217)-ROcp56_914*
 (RLcp56_218+RLcp56_286);
JTcp56_286_9 = -(ROcp56_714*(RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_386)-ROcp56_914*(RLcp56_116+RLcp56_117)-
 ROcp56_914*(RLcp56_118+RLcp56_186));
JTcp56_386_9 = ROcp56_714*(RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_286)-ROcp56_814*(RLcp56_116+RLcp56_117)-ROcp56_814*
 (RLcp56_118+RLcp56_186);
JTcp56_186_10 = ROcp56_515*(RLcp56_317+RLcp56_318)-ROcp56_615*(RLcp56_217+RLcp56_218)-RLcp56_286*ROcp56_615+RLcp56_386
 *ROcp56_515;
JTcp56_286_10 = RLcp56_186*ROcp56_615-RLcp56_386*ROcp56_415-ROcp56_415*(RLcp56_317+RLcp56_318)+ROcp56_615*(RLcp56_117+
 RLcp56_118);
JTcp56_386_10 = ROcp56_415*(RLcp56_217+RLcp56_218)-ROcp56_515*(RLcp56_117+RLcp56_118)-RLcp56_186*ROcp56_515+RLcp56_286
 *ROcp56_415;
JTcp56_186_11 = ROcp56_216*(RLcp56_318+RLcp56_386)-ROcp56_316*(RLcp56_218+RLcp56_286);
JTcp56_286_11 = -(ROcp56_116*(RLcp56_318+RLcp56_386)-ROcp56_316*(RLcp56_118+RLcp56_186));
JTcp56_386_11 = ROcp56_116*(RLcp56_218+RLcp56_286)-ROcp56_216*(RLcp56_118+RLcp56_186);
JTcp56_186_12 = -(RLcp56_286*ROcp56_617-RLcp56_386*ROcp56_517);
JTcp56_286_12 = RLcp56_186*ROcp56_617-RLcp56_386*ROcp56_417;
JTcp56_386_12 = -(RLcp56_186*ROcp56_517-RLcp56_286*ROcp56_417);
ORcp56_186 = OMcp56_218*RLcp56_386-OMcp56_318*RLcp56_286;
ORcp56_286 = -(OMcp56_118*RLcp56_386-OMcp56_318*RLcp56_186);
ORcp56_386 = OMcp56_118*RLcp56_286-OMcp56_218*RLcp56_186;
VIcp56_186 = ORcp56_113+ORcp56_114+ORcp56_115+ORcp56_116+ORcp56_117+ORcp56_118+ORcp56_186+qd[1];
VIcp56_286 = ORcp56_213+ORcp56_214+ORcp56_215+ORcp56_216+ORcp56_217+ORcp56_218+ORcp56_286+qd[2];
VIcp56_386 = ORcp56_313+ORcp56_314+ORcp56_315+ORcp56_316+ORcp56_317+ORcp56_318+ORcp56_386+qd[3];
ACcp56_186 = qdd[1]+OMcp56_213*ORcp56_314+OMcp56_214*ORcp56_315+OMcp56_215*ORcp56_316+OMcp56_216*ORcp56_317+OMcp56_217
 *ORcp56_318+OMcp56_218*ORcp56_386+OMcp56_26*ORcp56_313-OMcp56_313*ORcp56_214-OMcp56_314*ORcp56_215-OMcp56_315*ORcp56_216-
 OMcp56_316*ORcp56_217-OMcp56_317*ORcp56_218-OMcp56_318*ORcp56_286-OMcp56_36*ORcp56_213+OPcp56_213*RLcp56_314+OPcp56_214*
 RLcp56_315+OPcp56_215*RLcp56_316+OPcp56_216*RLcp56_317+OPcp56_217*RLcp56_318+OPcp56_218*RLcp56_386+OPcp56_26*RLcp56_313-
 OPcp56_313*RLcp56_214-OPcp56_314*RLcp56_215-OPcp56_315*RLcp56_216-OPcp56_316*RLcp56_217-OPcp56_317*RLcp56_218-OPcp56_318*
 RLcp56_286-OPcp56_36*RLcp56_213;
ACcp56_286 = qdd[2]-OMcp56_113*ORcp56_314-OMcp56_114*ORcp56_315-OMcp56_115*ORcp56_316-OMcp56_116*ORcp56_317-OMcp56_117
 *ORcp56_318-OMcp56_118*ORcp56_386-OMcp56_16*ORcp56_313+OMcp56_313*ORcp56_114+OMcp56_314*ORcp56_115+OMcp56_315*ORcp56_116+
 OMcp56_316*ORcp56_117+OMcp56_317*ORcp56_118+OMcp56_318*ORcp56_186+OMcp56_36*ORcp56_113-OPcp56_113*RLcp56_314-OPcp56_114*
 RLcp56_315-OPcp56_115*RLcp56_316-OPcp56_116*RLcp56_317-OPcp56_117*RLcp56_318-OPcp56_118*RLcp56_386-OPcp56_16*RLcp56_313+
 OPcp56_313*RLcp56_114+OPcp56_314*RLcp56_115+OPcp56_315*RLcp56_116+OPcp56_316*RLcp56_117+OPcp56_317*RLcp56_118+OPcp56_318*
 RLcp56_186+OPcp56_36*RLcp56_113;
ACcp56_386 = qdd[3]+OMcp56_113*ORcp56_214+OMcp56_114*ORcp56_215+OMcp56_115*ORcp56_216+OMcp56_116*ORcp56_217+OMcp56_117
 *ORcp56_218+OMcp56_118*ORcp56_286+OMcp56_16*ORcp56_213-OMcp56_213*ORcp56_114-OMcp56_214*ORcp56_115-OMcp56_215*ORcp56_116-
 OMcp56_216*ORcp56_117-OMcp56_217*ORcp56_118-OMcp56_218*ORcp56_186-OMcp56_26*ORcp56_113+OPcp56_113*RLcp56_214+OPcp56_114*
 RLcp56_215+OPcp56_115*RLcp56_216+OPcp56_116*RLcp56_217+OPcp56_117*RLcp56_218+OPcp56_118*RLcp56_286+OPcp56_16*RLcp56_213-
 OPcp56_213*RLcp56_114-OPcp56_214*RLcp56_115-OPcp56_215*RLcp56_116-OPcp56_216*RLcp56_117-OPcp56_217*RLcp56_118-OPcp56_218*
 RLcp56_186-OPcp56_26*RLcp56_113;

// = = Block_1_0_0_57_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp56_186;
sens->P[2] = POcp56_286;
sens->P[3] = POcp56_386;
sens->R[1][1] = ROcp56_118;
sens->R[1][2] = ROcp56_218;
sens->R[1][3] = ROcp56_318;
sens->R[2][1] = ROcp56_417;
sens->R[2][2] = ROcp56_517;
sens->R[2][3] = ROcp56_617;
sens->R[3][1] = ROcp56_718;
sens->R[3][2] = ROcp56_818;
sens->R[3][3] = ROcp56_918;
sens->V[1] = VIcp56_186;
sens->V[2] = VIcp56_286;
sens->V[3] = VIcp56_386;
sens->OM[1] = OMcp56_118;
sens->OM[2] = OMcp56_218;
sens->OM[3] = OMcp56_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp56_186_5;
sens->J[1][6] = JTcp56_186_6;
sens->J[1][13] = JTcp56_186_7;
sens->J[1][14] = JTcp56_186_8;
sens->J[1][15] = JTcp56_186_9;
sens->J[1][16] = JTcp56_186_10;
sens->J[1][17] = JTcp56_186_11;
sens->J[1][18] = JTcp56_186_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp56_286_4;
sens->J[2][5] = JTcp56_286_5;
sens->J[2][6] = JTcp56_286_6;
sens->J[2][13] = JTcp56_286_7;
sens->J[2][14] = JTcp56_286_8;
sens->J[2][15] = JTcp56_286_9;
sens->J[2][16] = JTcp56_286_10;
sens->J[2][17] = JTcp56_286_11;
sens->J[2][18] = JTcp56_286_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp56_386_4;
sens->J[3][5] = JTcp56_386_5;
sens->J[3][6] = JTcp56_386_6;
sens->J[3][13] = JTcp56_386_7;
sens->J[3][14] = JTcp56_386_8;
sens->J[3][15] = JTcp56_386_9;
sens->J[3][16] = JTcp56_386_10;
sens->J[3][17] = JTcp56_386_11;
sens->J[3][18] = JTcp56_386_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp56_46;
sens->J[4][14] = ROcp56_113;
sens->J[4][15] = ROcp56_714;
sens->J[4][16] = ROcp56_415;
sens->J[4][17] = ROcp56_116;
sens->J[4][18] = ROcp56_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp56_85;
sens->J[5][13] = ROcp56_56;
sens->J[5][14] = ROcp56_213;
sens->J[5][15] = ROcp56_814;
sens->J[5][16] = ROcp56_515;
sens->J[5][17] = ROcp56_216;
sens->J[5][18] = ROcp56_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp56_95;
sens->J[6][13] = ROcp56_66;
sens->J[6][14] = ROcp56_313;
sens->J[6][15] = ROcp56_914;
sens->J[6][16] = ROcp56_615;
sens->J[6][17] = ROcp56_316;
sens->J[6][18] = ROcp56_617;
sens->A[1] = ACcp56_186;
sens->A[2] = ACcp56_286;
sens->A[3] = ACcp56_386;
sens->OMP[1] = OPcp56_118;
sens->OMP[2] = OPcp56_218;
sens->OMP[3] = OPcp56_318;
 
// 
break;
case 58:
 


// = = Block_1_0_0_58_0_1 = = 
 
// Sensor Kinematics 


ROcp57_25 = S4*S5;
ROcp57_35 = -C4*S5;
ROcp57_85 = -S4*C5;
ROcp57_95 = C4*C5;
ROcp57_16 = C5*C6;
ROcp57_26 = ROcp57_25*C6+C4*S6;
ROcp57_36 = ROcp57_35*C6+S4*S6;
ROcp57_46 = -C5*S6;
ROcp57_56 = -(ROcp57_25*S6-C4*C6);
ROcp57_66 = -(ROcp57_35*S6-S4*C6);
OMcp57_25 = qd[5]*C4;
OMcp57_35 = qd[5]*S4;
OMcp57_16 = qd[4]+qd[6]*S5;
OMcp57_26 = OMcp57_25+ROcp57_85*qd[6];
OMcp57_36 = OMcp57_35+ROcp57_95*qd[6];
OPcp57_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp57_26 = ROcp57_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp57_35*S5-ROcp57_95*qd[4]);
OPcp57_36 = ROcp57_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp57_25*S5-ROcp57_85*qd[4]);

// = = Block_1_0_0_58_0_3 = = 
 
// Sensor Kinematics 


ROcp57_113 = ROcp57_16*C13-S13*S5;
ROcp57_213 = ROcp57_26*C13-ROcp57_85*S13;
ROcp57_313 = ROcp57_36*C13-ROcp57_95*S13;
ROcp57_713 = ROcp57_16*S13+C13*S5;
ROcp57_813 = ROcp57_26*S13+ROcp57_85*C13;
ROcp57_913 = ROcp57_36*S13+ROcp57_95*C13;
ROcp57_414 = ROcp57_46*C14+ROcp57_713*S14;
ROcp57_514 = ROcp57_56*C14+ROcp57_813*S14;
ROcp57_614 = ROcp57_66*C14+ROcp57_913*S14;
ROcp57_714 = -(ROcp57_46*S14-ROcp57_713*C14);
ROcp57_814 = -(ROcp57_56*S14-ROcp57_813*C14);
ROcp57_914 = -(ROcp57_66*S14-ROcp57_913*C14);
ROcp57_115 = ROcp57_113*C15+ROcp57_414*S15;
ROcp57_215 = ROcp57_213*C15+ROcp57_514*S15;
ROcp57_315 = ROcp57_313*C15+ROcp57_614*S15;
ROcp57_415 = -(ROcp57_113*S15-ROcp57_414*C15);
ROcp57_515 = -(ROcp57_213*S15-ROcp57_514*C15);
ROcp57_615 = -(ROcp57_313*S15-ROcp57_614*C15);
ROcp57_116 = ROcp57_115*C16-ROcp57_714*S16;
ROcp57_216 = ROcp57_215*C16-ROcp57_814*S16;
ROcp57_316 = ROcp57_315*C16-ROcp57_914*S16;
ROcp57_716 = ROcp57_115*S16+ROcp57_714*C16;
ROcp57_816 = ROcp57_215*S16+ROcp57_814*C16;
ROcp57_916 = ROcp57_315*S16+ROcp57_914*C16;
ROcp57_417 = ROcp57_415*C17+ROcp57_716*S17;
ROcp57_517 = ROcp57_515*C17+ROcp57_816*S17;
ROcp57_617 = ROcp57_615*C17+ROcp57_916*S17;
ROcp57_717 = -(ROcp57_415*S17-ROcp57_716*C17);
ROcp57_817 = -(ROcp57_515*S17-ROcp57_816*C17);
ROcp57_917 = -(ROcp57_615*S17-ROcp57_916*C17);
ROcp57_118 = ROcp57_116*C18-ROcp57_717*S18;
ROcp57_218 = ROcp57_216*C18-ROcp57_817*S18;
ROcp57_318 = ROcp57_316*C18-ROcp57_917*S18;
ROcp57_718 = ROcp57_116*S18+ROcp57_717*C18;
ROcp57_818 = ROcp57_216*S18+ROcp57_817*C18;
ROcp57_918 = ROcp57_316*S18+ROcp57_917*C18;
RLcp57_113 = s->dpt[1][2]*ROcp57_16+s->dpt[3][2]*S5+ROcp57_46*s->dpt[2][2];
RLcp57_213 = s->dpt[1][2]*ROcp57_26+s->dpt[3][2]*ROcp57_85+ROcp57_56*s->dpt[2][2];
RLcp57_313 = s->dpt[1][2]*ROcp57_36+s->dpt[3][2]*ROcp57_95+ROcp57_66*s->dpt[2][2];
OMcp57_113 = OMcp57_16+ROcp57_46*qd[13];
OMcp57_213 = OMcp57_26+ROcp57_56*qd[13];
OMcp57_313 = OMcp57_36+ROcp57_66*qd[13];
ORcp57_113 = OMcp57_26*RLcp57_313-OMcp57_36*RLcp57_213;
ORcp57_213 = -(OMcp57_16*RLcp57_313-OMcp57_36*RLcp57_113);
ORcp57_313 = OMcp57_16*RLcp57_213-OMcp57_26*RLcp57_113;
OPcp57_113 = OPcp57_16+ROcp57_46*qdd[13]+qd[13]*(OMcp57_26*ROcp57_66-OMcp57_36*ROcp57_56);
OPcp57_213 = OPcp57_26+ROcp57_56*qdd[13]-qd[13]*(OMcp57_16*ROcp57_66-OMcp57_36*ROcp57_46);
OPcp57_313 = OPcp57_36+ROcp57_66*qdd[13]+qd[13]*(OMcp57_16*ROcp57_56-OMcp57_26*ROcp57_46);
RLcp57_114 = s->dpt[1][22]*ROcp57_113+s->dpt[3][22]*ROcp57_713+ROcp57_46*s->dpt[2][22];
RLcp57_214 = s->dpt[1][22]*ROcp57_213+s->dpt[3][22]*ROcp57_813+ROcp57_56*s->dpt[2][22];
RLcp57_314 = s->dpt[1][22]*ROcp57_313+s->dpt[3][22]*ROcp57_913+ROcp57_66*s->dpt[2][22];
OMcp57_114 = OMcp57_113+ROcp57_113*qd[14];
OMcp57_214 = OMcp57_213+ROcp57_213*qd[14];
OMcp57_314 = OMcp57_313+ROcp57_313*qd[14];
ORcp57_114 = OMcp57_213*RLcp57_314-OMcp57_313*RLcp57_214;
ORcp57_214 = -(OMcp57_113*RLcp57_314-OMcp57_313*RLcp57_114);
ORcp57_314 = OMcp57_113*RLcp57_214-OMcp57_213*RLcp57_114;
OPcp57_114 = OPcp57_113+ROcp57_113*qdd[14]+qd[14]*(OMcp57_213*ROcp57_313-OMcp57_313*ROcp57_213);
OPcp57_214 = OPcp57_213+ROcp57_213*qdd[14]-qd[14]*(OMcp57_113*ROcp57_313-OMcp57_313*ROcp57_113);
OPcp57_314 = OPcp57_313+ROcp57_313*qdd[14]+qd[14]*(OMcp57_113*ROcp57_213-OMcp57_213*ROcp57_113);
RLcp57_115 = s->dpt[1][24]*ROcp57_113+s->dpt[2][24]*ROcp57_414+ROcp57_714*s->dpt[3][24];
RLcp57_215 = s->dpt[1][24]*ROcp57_213+s->dpt[2][24]*ROcp57_514+ROcp57_814*s->dpt[3][24];
RLcp57_315 = s->dpt[1][24]*ROcp57_313+s->dpt[2][24]*ROcp57_614+ROcp57_914*s->dpt[3][24];
OMcp57_115 = OMcp57_114+ROcp57_714*qd[15];
OMcp57_215 = OMcp57_214+ROcp57_814*qd[15];
OMcp57_315 = OMcp57_314+ROcp57_914*qd[15];
ORcp57_115 = OMcp57_214*RLcp57_315-OMcp57_314*RLcp57_215;
ORcp57_215 = -(OMcp57_114*RLcp57_315-OMcp57_314*RLcp57_115);
ORcp57_315 = OMcp57_114*RLcp57_215-OMcp57_214*RLcp57_115;
OPcp57_115 = OPcp57_114+ROcp57_714*qdd[15]+qd[15]*(OMcp57_214*ROcp57_914-OMcp57_314*ROcp57_814);
OPcp57_215 = OPcp57_214+ROcp57_814*qdd[15]-qd[15]*(OMcp57_114*ROcp57_914-OMcp57_314*ROcp57_714);
OPcp57_315 = OPcp57_314+ROcp57_914*qdd[15]+qd[15]*(OMcp57_114*ROcp57_814-OMcp57_214*ROcp57_714);
RLcp57_116 = s->dpt[1][26]*ROcp57_115+s->dpt[2][26]*ROcp57_415+ROcp57_714*s->dpt[3][26];
RLcp57_216 = s->dpt[1][26]*ROcp57_215+s->dpt[2][26]*ROcp57_515+ROcp57_814*s->dpt[3][26];
RLcp57_316 = s->dpt[1][26]*ROcp57_315+s->dpt[2][26]*ROcp57_615+ROcp57_914*s->dpt[3][26];
OMcp57_116 = OMcp57_115+ROcp57_415*qd[16];
OMcp57_216 = OMcp57_215+ROcp57_515*qd[16];
OMcp57_316 = OMcp57_315+ROcp57_615*qd[16];
ORcp57_116 = OMcp57_215*RLcp57_316-OMcp57_315*RLcp57_216;
ORcp57_216 = -(OMcp57_115*RLcp57_316-OMcp57_315*RLcp57_116);
ORcp57_316 = OMcp57_115*RLcp57_216-OMcp57_215*RLcp57_116;
OPcp57_116 = OPcp57_115+ROcp57_415*qdd[16]+qd[16]*(OMcp57_215*ROcp57_615-OMcp57_315*ROcp57_515);
OPcp57_216 = OPcp57_215+ROcp57_515*qdd[16]-qd[16]*(OMcp57_115*ROcp57_615-OMcp57_315*ROcp57_415);
OPcp57_316 = OPcp57_315+ROcp57_615*qdd[16]+qd[16]*(OMcp57_115*ROcp57_515-OMcp57_215*ROcp57_415);
RLcp57_117 = s->dpt[1][28]*ROcp57_116+s->dpt[2][28]*ROcp57_415+ROcp57_716*s->dpt[3][28];
RLcp57_217 = s->dpt[1][28]*ROcp57_216+s->dpt[2][28]*ROcp57_515+ROcp57_816*s->dpt[3][28];
RLcp57_317 = s->dpt[1][28]*ROcp57_316+s->dpt[2][28]*ROcp57_615+ROcp57_916*s->dpt[3][28];
OMcp57_117 = OMcp57_116+ROcp57_116*qd[17];
OMcp57_217 = OMcp57_216+ROcp57_216*qd[17];
OMcp57_317 = OMcp57_316+ROcp57_316*qd[17];
ORcp57_117 = OMcp57_216*RLcp57_317-OMcp57_316*RLcp57_217;
ORcp57_217 = -(OMcp57_116*RLcp57_317-OMcp57_316*RLcp57_117);
ORcp57_317 = OMcp57_116*RLcp57_217-OMcp57_216*RLcp57_117;
OPcp57_117 = OPcp57_116+ROcp57_116*qdd[17]+qd[17]*(OMcp57_216*ROcp57_316-OMcp57_316*ROcp57_216);
OPcp57_217 = OPcp57_216+ROcp57_216*qdd[17]-qd[17]*(OMcp57_116*ROcp57_316-OMcp57_316*ROcp57_116);
OPcp57_317 = OPcp57_316+ROcp57_316*qdd[17]+qd[17]*(OMcp57_116*ROcp57_216-OMcp57_216*ROcp57_116);
RLcp57_118 = s->dpt[1][30]*ROcp57_116+s->dpt[2][30]*ROcp57_417+s->dpt[3][30]*ROcp57_717;
RLcp57_218 = s->dpt[1][30]*ROcp57_216+s->dpt[2][30]*ROcp57_517+s->dpt[3][30]*ROcp57_817;
RLcp57_318 = s->dpt[1][30]*ROcp57_316+s->dpt[2][30]*ROcp57_617+s->dpt[3][30]*ROcp57_917;
OMcp57_118 = OMcp57_117+ROcp57_417*qd[18];
OMcp57_218 = OMcp57_217+ROcp57_517*qd[18];
OMcp57_318 = OMcp57_317+ROcp57_617*qd[18];
ORcp57_118 = OMcp57_217*RLcp57_318-OMcp57_317*RLcp57_218;
ORcp57_218 = -(OMcp57_117*RLcp57_318-OMcp57_317*RLcp57_118);
ORcp57_318 = OMcp57_117*RLcp57_218-OMcp57_217*RLcp57_118;
OPcp57_118 = OPcp57_117+ROcp57_417*qdd[18]+qd[18]*(OMcp57_217*ROcp57_617-OMcp57_317*ROcp57_517);
OPcp57_218 = OPcp57_217+ROcp57_517*qdd[18]-qd[18]*(OMcp57_117*ROcp57_617-OMcp57_317*ROcp57_417);
OPcp57_318 = OPcp57_317+ROcp57_617*qdd[18]+qd[18]*(OMcp57_117*ROcp57_517-OMcp57_217*ROcp57_417);
RLcp57_187 = ROcp57_118*s->dpt[1][34]+ROcp57_417*s->dpt[2][34]+ROcp57_718*s->dpt[3][34];
RLcp57_287 = ROcp57_218*s->dpt[1][34]+ROcp57_517*s->dpt[2][34]+ROcp57_818*s->dpt[3][34];
RLcp57_387 = ROcp57_318*s->dpt[1][34]+ROcp57_617*s->dpt[2][34]+ROcp57_918*s->dpt[3][34];
POcp57_187 = RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_187+q[1];
POcp57_287 = RLcp57_213+RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_287+q[2];
POcp57_387 = RLcp57_313+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_387+q[3];
JTcp57_287_4 = -(RLcp57_313+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_387);
JTcp57_387_4 = RLcp57_213+RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_287;
JTcp57_187_5 = C4*(RLcp57_313+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318)-S4*(RLcp57_213+RLcp57_214)-S4*(
 RLcp57_215+RLcp57_216)-S4*(RLcp57_217+RLcp57_218)-RLcp57_287*S4+RLcp57_387*C4;
JTcp57_287_5 = S4*(RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_187);
JTcp57_387_5 = -C4*(RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_187);
JTcp57_187_6 = ROcp57_85*(RLcp57_313+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318)-ROcp57_95*(RLcp57_213+
 RLcp57_214)-ROcp57_95*(RLcp57_215+RLcp57_216)-ROcp57_95*(RLcp57_217+RLcp57_218)-RLcp57_287*ROcp57_95+RLcp57_387*ROcp57_85;
JTcp57_287_6 = -(RLcp57_387*S5-ROcp57_95*(RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_187
 )+S5*(RLcp57_313+RLcp57_314)+S5*(RLcp57_315+RLcp57_316)+S5*(RLcp57_317+RLcp57_318));
JTcp57_387_6 = RLcp57_287*S5-ROcp57_85*(RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_187)+
 S5*(RLcp57_213+RLcp57_214)+S5*(RLcp57_215+RLcp57_216)+S5*(RLcp57_217+RLcp57_218);
JTcp57_187_7 = ROcp57_56*(RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_387)-ROcp57_66*(RLcp57_214+
 RLcp57_215)-ROcp57_66*(RLcp57_216+RLcp57_217)-ROcp57_66*(RLcp57_218+RLcp57_287);
JTcp57_287_7 = -(ROcp57_46*(RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_387)-ROcp57_66*(RLcp57_114+
 RLcp57_115)-ROcp57_66*(RLcp57_116+RLcp57_117)-ROcp57_66*(RLcp57_118+RLcp57_187));
JTcp57_387_7 = ROcp57_46*(RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_287)-ROcp57_56*(RLcp57_114+
 RLcp57_115)-ROcp57_56*(RLcp57_116+RLcp57_117)-ROcp57_56*(RLcp57_118+RLcp57_187);
JTcp57_187_8 = ROcp57_213*(RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318)-ROcp57_313*(RLcp57_215+RLcp57_216)-ROcp57_313*
 (RLcp57_217+RLcp57_218)-RLcp57_287*ROcp57_313+RLcp57_387*ROcp57_213;
JTcp57_287_8 = RLcp57_187*ROcp57_313-RLcp57_387*ROcp57_113-ROcp57_113*(RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318)+
 ROcp57_313*(RLcp57_115+RLcp57_116)+ROcp57_313*(RLcp57_117+RLcp57_118);
JTcp57_387_8 = ROcp57_113*(RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218)-ROcp57_213*(RLcp57_115+RLcp57_116)-ROcp57_213*
 (RLcp57_117+RLcp57_118)-RLcp57_187*ROcp57_213+RLcp57_287*ROcp57_113;
JTcp57_187_9 = ROcp57_814*(RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_387)-ROcp57_914*(RLcp57_216+RLcp57_217)-ROcp57_914*
 (RLcp57_218+RLcp57_287);
JTcp57_287_9 = -(ROcp57_714*(RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_387)-ROcp57_914*(RLcp57_116+RLcp57_117)-
 ROcp57_914*(RLcp57_118+RLcp57_187));
JTcp57_387_9 = ROcp57_714*(RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_287)-ROcp57_814*(RLcp57_116+RLcp57_117)-ROcp57_814*
 (RLcp57_118+RLcp57_187);
JTcp57_187_10 = ROcp57_515*(RLcp57_317+RLcp57_318)-ROcp57_615*(RLcp57_217+RLcp57_218)-RLcp57_287*ROcp57_615+RLcp57_387
 *ROcp57_515;
JTcp57_287_10 = RLcp57_187*ROcp57_615-RLcp57_387*ROcp57_415-ROcp57_415*(RLcp57_317+RLcp57_318)+ROcp57_615*(RLcp57_117+
 RLcp57_118);
JTcp57_387_10 = ROcp57_415*(RLcp57_217+RLcp57_218)-ROcp57_515*(RLcp57_117+RLcp57_118)-RLcp57_187*ROcp57_515+RLcp57_287
 *ROcp57_415;
JTcp57_187_11 = ROcp57_216*(RLcp57_318+RLcp57_387)-ROcp57_316*(RLcp57_218+RLcp57_287);
JTcp57_287_11 = -(ROcp57_116*(RLcp57_318+RLcp57_387)-ROcp57_316*(RLcp57_118+RLcp57_187));
JTcp57_387_11 = ROcp57_116*(RLcp57_218+RLcp57_287)-ROcp57_216*(RLcp57_118+RLcp57_187);
JTcp57_187_12 = -(RLcp57_287*ROcp57_617-RLcp57_387*ROcp57_517);
JTcp57_287_12 = RLcp57_187*ROcp57_617-RLcp57_387*ROcp57_417;
JTcp57_387_12 = -(RLcp57_187*ROcp57_517-RLcp57_287*ROcp57_417);
ORcp57_187 = OMcp57_218*RLcp57_387-OMcp57_318*RLcp57_287;
ORcp57_287 = -(OMcp57_118*RLcp57_387-OMcp57_318*RLcp57_187);
ORcp57_387 = OMcp57_118*RLcp57_287-OMcp57_218*RLcp57_187;
VIcp57_187 = ORcp57_113+ORcp57_114+ORcp57_115+ORcp57_116+ORcp57_117+ORcp57_118+ORcp57_187+qd[1];
VIcp57_287 = ORcp57_213+ORcp57_214+ORcp57_215+ORcp57_216+ORcp57_217+ORcp57_218+ORcp57_287+qd[2];
VIcp57_387 = ORcp57_313+ORcp57_314+ORcp57_315+ORcp57_316+ORcp57_317+ORcp57_318+ORcp57_387+qd[3];
ACcp57_187 = qdd[1]+OMcp57_213*ORcp57_314+OMcp57_214*ORcp57_315+OMcp57_215*ORcp57_316+OMcp57_216*ORcp57_317+OMcp57_217
 *ORcp57_318+OMcp57_218*ORcp57_387+OMcp57_26*ORcp57_313-OMcp57_313*ORcp57_214-OMcp57_314*ORcp57_215-OMcp57_315*ORcp57_216-
 OMcp57_316*ORcp57_217-OMcp57_317*ORcp57_218-OMcp57_318*ORcp57_287-OMcp57_36*ORcp57_213+OPcp57_213*RLcp57_314+OPcp57_214*
 RLcp57_315+OPcp57_215*RLcp57_316+OPcp57_216*RLcp57_317+OPcp57_217*RLcp57_318+OPcp57_218*RLcp57_387+OPcp57_26*RLcp57_313-
 OPcp57_313*RLcp57_214-OPcp57_314*RLcp57_215-OPcp57_315*RLcp57_216-OPcp57_316*RLcp57_217-OPcp57_317*RLcp57_218-OPcp57_318*
 RLcp57_287-OPcp57_36*RLcp57_213;
ACcp57_287 = qdd[2]-OMcp57_113*ORcp57_314-OMcp57_114*ORcp57_315-OMcp57_115*ORcp57_316-OMcp57_116*ORcp57_317-OMcp57_117
 *ORcp57_318-OMcp57_118*ORcp57_387-OMcp57_16*ORcp57_313+OMcp57_313*ORcp57_114+OMcp57_314*ORcp57_115+OMcp57_315*ORcp57_116+
 OMcp57_316*ORcp57_117+OMcp57_317*ORcp57_118+OMcp57_318*ORcp57_187+OMcp57_36*ORcp57_113-OPcp57_113*RLcp57_314-OPcp57_114*
 RLcp57_315-OPcp57_115*RLcp57_316-OPcp57_116*RLcp57_317-OPcp57_117*RLcp57_318-OPcp57_118*RLcp57_387-OPcp57_16*RLcp57_313+
 OPcp57_313*RLcp57_114+OPcp57_314*RLcp57_115+OPcp57_315*RLcp57_116+OPcp57_316*RLcp57_117+OPcp57_317*RLcp57_118+OPcp57_318*
 RLcp57_187+OPcp57_36*RLcp57_113;
ACcp57_387 = qdd[3]+OMcp57_113*ORcp57_214+OMcp57_114*ORcp57_215+OMcp57_115*ORcp57_216+OMcp57_116*ORcp57_217+OMcp57_117
 *ORcp57_218+OMcp57_118*ORcp57_287+OMcp57_16*ORcp57_213-OMcp57_213*ORcp57_114-OMcp57_214*ORcp57_115-OMcp57_215*ORcp57_116-
 OMcp57_216*ORcp57_117-OMcp57_217*ORcp57_118-OMcp57_218*ORcp57_187-OMcp57_26*ORcp57_113+OPcp57_113*RLcp57_214+OPcp57_114*
 RLcp57_215+OPcp57_115*RLcp57_216+OPcp57_116*RLcp57_217+OPcp57_117*RLcp57_218+OPcp57_118*RLcp57_287+OPcp57_16*RLcp57_213-
 OPcp57_213*RLcp57_114-OPcp57_214*RLcp57_115-OPcp57_215*RLcp57_116-OPcp57_216*RLcp57_117-OPcp57_217*RLcp57_118-OPcp57_218*
 RLcp57_187-OPcp57_26*RLcp57_113;

// = = Block_1_0_0_58_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp57_187;
sens->P[2] = POcp57_287;
sens->P[3] = POcp57_387;
sens->R[1][1] = ROcp57_118;
sens->R[1][2] = ROcp57_218;
sens->R[1][3] = ROcp57_318;
sens->R[2][1] = ROcp57_417;
sens->R[2][2] = ROcp57_517;
sens->R[2][3] = ROcp57_617;
sens->R[3][1] = ROcp57_718;
sens->R[3][2] = ROcp57_818;
sens->R[3][3] = ROcp57_918;
sens->V[1] = VIcp57_187;
sens->V[2] = VIcp57_287;
sens->V[3] = VIcp57_387;
sens->OM[1] = OMcp57_118;
sens->OM[2] = OMcp57_218;
sens->OM[3] = OMcp57_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp57_187_5;
sens->J[1][6] = JTcp57_187_6;
sens->J[1][13] = JTcp57_187_7;
sens->J[1][14] = JTcp57_187_8;
sens->J[1][15] = JTcp57_187_9;
sens->J[1][16] = JTcp57_187_10;
sens->J[1][17] = JTcp57_187_11;
sens->J[1][18] = JTcp57_187_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp57_287_4;
sens->J[2][5] = JTcp57_287_5;
sens->J[2][6] = JTcp57_287_6;
sens->J[2][13] = JTcp57_287_7;
sens->J[2][14] = JTcp57_287_8;
sens->J[2][15] = JTcp57_287_9;
sens->J[2][16] = JTcp57_287_10;
sens->J[2][17] = JTcp57_287_11;
sens->J[2][18] = JTcp57_287_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp57_387_4;
sens->J[3][5] = JTcp57_387_5;
sens->J[3][6] = JTcp57_387_6;
sens->J[3][13] = JTcp57_387_7;
sens->J[3][14] = JTcp57_387_8;
sens->J[3][15] = JTcp57_387_9;
sens->J[3][16] = JTcp57_387_10;
sens->J[3][17] = JTcp57_387_11;
sens->J[3][18] = JTcp57_387_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp57_46;
sens->J[4][14] = ROcp57_113;
sens->J[4][15] = ROcp57_714;
sens->J[4][16] = ROcp57_415;
sens->J[4][17] = ROcp57_116;
sens->J[4][18] = ROcp57_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp57_85;
sens->J[5][13] = ROcp57_56;
sens->J[5][14] = ROcp57_213;
sens->J[5][15] = ROcp57_814;
sens->J[5][16] = ROcp57_515;
sens->J[5][17] = ROcp57_216;
sens->J[5][18] = ROcp57_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp57_95;
sens->J[6][13] = ROcp57_66;
sens->J[6][14] = ROcp57_313;
sens->J[6][15] = ROcp57_914;
sens->J[6][16] = ROcp57_615;
sens->J[6][17] = ROcp57_316;
sens->J[6][18] = ROcp57_617;
sens->A[1] = ACcp57_187;
sens->A[2] = ACcp57_287;
sens->A[3] = ACcp57_387;
sens->OMP[1] = OPcp57_118;
sens->OMP[2] = OPcp57_218;
sens->OMP[3] = OPcp57_318;
 
// 
break;
case 59:
 


// = = Block_1_0_0_59_0_1 = = 
 
// Sensor Kinematics 


ROcp58_25 = S4*S5;
ROcp58_35 = -C4*S5;
ROcp58_85 = -S4*C5;
ROcp58_95 = C4*C5;
ROcp58_16 = C5*C6;
ROcp58_26 = ROcp58_25*C6+C4*S6;
ROcp58_36 = ROcp58_35*C6+S4*S6;
ROcp58_46 = -C5*S6;
ROcp58_56 = -(ROcp58_25*S6-C4*C6);
ROcp58_66 = -(ROcp58_35*S6-S4*C6);
OMcp58_25 = qd[5]*C4;
OMcp58_35 = qd[5]*S4;
OMcp58_16 = qd[4]+qd[6]*S5;
OMcp58_26 = OMcp58_25+ROcp58_85*qd[6];
OMcp58_36 = OMcp58_35+ROcp58_95*qd[6];
OPcp58_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp58_26 = ROcp58_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp58_35*S5-ROcp58_95*qd[4]);
OPcp58_36 = ROcp58_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp58_25*S5-ROcp58_85*qd[4]);

// = = Block_1_0_0_59_0_3 = = 
 
// Sensor Kinematics 


ROcp58_113 = ROcp58_16*C13-S13*S5;
ROcp58_213 = ROcp58_26*C13-ROcp58_85*S13;
ROcp58_313 = ROcp58_36*C13-ROcp58_95*S13;
ROcp58_713 = ROcp58_16*S13+C13*S5;
ROcp58_813 = ROcp58_26*S13+ROcp58_85*C13;
ROcp58_913 = ROcp58_36*S13+ROcp58_95*C13;
ROcp58_414 = ROcp58_46*C14+ROcp58_713*S14;
ROcp58_514 = ROcp58_56*C14+ROcp58_813*S14;
ROcp58_614 = ROcp58_66*C14+ROcp58_913*S14;
ROcp58_714 = -(ROcp58_46*S14-ROcp58_713*C14);
ROcp58_814 = -(ROcp58_56*S14-ROcp58_813*C14);
ROcp58_914 = -(ROcp58_66*S14-ROcp58_913*C14);
ROcp58_115 = ROcp58_113*C15+ROcp58_414*S15;
ROcp58_215 = ROcp58_213*C15+ROcp58_514*S15;
ROcp58_315 = ROcp58_313*C15+ROcp58_614*S15;
ROcp58_415 = -(ROcp58_113*S15-ROcp58_414*C15);
ROcp58_515 = -(ROcp58_213*S15-ROcp58_514*C15);
ROcp58_615 = -(ROcp58_313*S15-ROcp58_614*C15);
ROcp58_116 = ROcp58_115*C16-ROcp58_714*S16;
ROcp58_216 = ROcp58_215*C16-ROcp58_814*S16;
ROcp58_316 = ROcp58_315*C16-ROcp58_914*S16;
ROcp58_716 = ROcp58_115*S16+ROcp58_714*C16;
ROcp58_816 = ROcp58_215*S16+ROcp58_814*C16;
ROcp58_916 = ROcp58_315*S16+ROcp58_914*C16;
ROcp58_417 = ROcp58_415*C17+ROcp58_716*S17;
ROcp58_517 = ROcp58_515*C17+ROcp58_816*S17;
ROcp58_617 = ROcp58_615*C17+ROcp58_916*S17;
ROcp58_717 = -(ROcp58_415*S17-ROcp58_716*C17);
ROcp58_817 = -(ROcp58_515*S17-ROcp58_816*C17);
ROcp58_917 = -(ROcp58_615*S17-ROcp58_916*C17);
ROcp58_118 = ROcp58_116*C18-ROcp58_717*S18;
ROcp58_218 = ROcp58_216*C18-ROcp58_817*S18;
ROcp58_318 = ROcp58_316*C18-ROcp58_917*S18;
ROcp58_718 = ROcp58_116*S18+ROcp58_717*C18;
ROcp58_818 = ROcp58_216*S18+ROcp58_817*C18;
ROcp58_918 = ROcp58_316*S18+ROcp58_917*C18;
RLcp58_113 = s->dpt[1][2]*ROcp58_16+s->dpt[3][2]*S5+ROcp58_46*s->dpt[2][2];
RLcp58_213 = s->dpt[1][2]*ROcp58_26+s->dpt[3][2]*ROcp58_85+ROcp58_56*s->dpt[2][2];
RLcp58_313 = s->dpt[1][2]*ROcp58_36+s->dpt[3][2]*ROcp58_95+ROcp58_66*s->dpt[2][2];
OMcp58_113 = OMcp58_16+ROcp58_46*qd[13];
OMcp58_213 = OMcp58_26+ROcp58_56*qd[13];
OMcp58_313 = OMcp58_36+ROcp58_66*qd[13];
ORcp58_113 = OMcp58_26*RLcp58_313-OMcp58_36*RLcp58_213;
ORcp58_213 = -(OMcp58_16*RLcp58_313-OMcp58_36*RLcp58_113);
ORcp58_313 = OMcp58_16*RLcp58_213-OMcp58_26*RLcp58_113;
OPcp58_113 = OPcp58_16+ROcp58_46*qdd[13]+qd[13]*(OMcp58_26*ROcp58_66-OMcp58_36*ROcp58_56);
OPcp58_213 = OPcp58_26+ROcp58_56*qdd[13]-qd[13]*(OMcp58_16*ROcp58_66-OMcp58_36*ROcp58_46);
OPcp58_313 = OPcp58_36+ROcp58_66*qdd[13]+qd[13]*(OMcp58_16*ROcp58_56-OMcp58_26*ROcp58_46);
RLcp58_114 = s->dpt[1][22]*ROcp58_113+s->dpt[3][22]*ROcp58_713+ROcp58_46*s->dpt[2][22];
RLcp58_214 = s->dpt[1][22]*ROcp58_213+s->dpt[3][22]*ROcp58_813+ROcp58_56*s->dpt[2][22];
RLcp58_314 = s->dpt[1][22]*ROcp58_313+s->dpt[3][22]*ROcp58_913+ROcp58_66*s->dpt[2][22];
OMcp58_114 = OMcp58_113+ROcp58_113*qd[14];
OMcp58_214 = OMcp58_213+ROcp58_213*qd[14];
OMcp58_314 = OMcp58_313+ROcp58_313*qd[14];
ORcp58_114 = OMcp58_213*RLcp58_314-OMcp58_313*RLcp58_214;
ORcp58_214 = -(OMcp58_113*RLcp58_314-OMcp58_313*RLcp58_114);
ORcp58_314 = OMcp58_113*RLcp58_214-OMcp58_213*RLcp58_114;
OPcp58_114 = OPcp58_113+ROcp58_113*qdd[14]+qd[14]*(OMcp58_213*ROcp58_313-OMcp58_313*ROcp58_213);
OPcp58_214 = OPcp58_213+ROcp58_213*qdd[14]-qd[14]*(OMcp58_113*ROcp58_313-OMcp58_313*ROcp58_113);
OPcp58_314 = OPcp58_313+ROcp58_313*qdd[14]+qd[14]*(OMcp58_113*ROcp58_213-OMcp58_213*ROcp58_113);
RLcp58_115 = s->dpt[1][24]*ROcp58_113+s->dpt[2][24]*ROcp58_414+ROcp58_714*s->dpt[3][24];
RLcp58_215 = s->dpt[1][24]*ROcp58_213+s->dpt[2][24]*ROcp58_514+ROcp58_814*s->dpt[3][24];
RLcp58_315 = s->dpt[1][24]*ROcp58_313+s->dpt[2][24]*ROcp58_614+ROcp58_914*s->dpt[3][24];
OMcp58_115 = OMcp58_114+ROcp58_714*qd[15];
OMcp58_215 = OMcp58_214+ROcp58_814*qd[15];
OMcp58_315 = OMcp58_314+ROcp58_914*qd[15];
ORcp58_115 = OMcp58_214*RLcp58_315-OMcp58_314*RLcp58_215;
ORcp58_215 = -(OMcp58_114*RLcp58_315-OMcp58_314*RLcp58_115);
ORcp58_315 = OMcp58_114*RLcp58_215-OMcp58_214*RLcp58_115;
OPcp58_115 = OPcp58_114+ROcp58_714*qdd[15]+qd[15]*(OMcp58_214*ROcp58_914-OMcp58_314*ROcp58_814);
OPcp58_215 = OPcp58_214+ROcp58_814*qdd[15]-qd[15]*(OMcp58_114*ROcp58_914-OMcp58_314*ROcp58_714);
OPcp58_315 = OPcp58_314+ROcp58_914*qdd[15]+qd[15]*(OMcp58_114*ROcp58_814-OMcp58_214*ROcp58_714);
RLcp58_116 = s->dpt[1][26]*ROcp58_115+s->dpt[2][26]*ROcp58_415+ROcp58_714*s->dpt[3][26];
RLcp58_216 = s->dpt[1][26]*ROcp58_215+s->dpt[2][26]*ROcp58_515+ROcp58_814*s->dpt[3][26];
RLcp58_316 = s->dpt[1][26]*ROcp58_315+s->dpt[2][26]*ROcp58_615+ROcp58_914*s->dpt[3][26];
OMcp58_116 = OMcp58_115+ROcp58_415*qd[16];
OMcp58_216 = OMcp58_215+ROcp58_515*qd[16];
OMcp58_316 = OMcp58_315+ROcp58_615*qd[16];
ORcp58_116 = OMcp58_215*RLcp58_316-OMcp58_315*RLcp58_216;
ORcp58_216 = -(OMcp58_115*RLcp58_316-OMcp58_315*RLcp58_116);
ORcp58_316 = OMcp58_115*RLcp58_216-OMcp58_215*RLcp58_116;
OPcp58_116 = OPcp58_115+ROcp58_415*qdd[16]+qd[16]*(OMcp58_215*ROcp58_615-OMcp58_315*ROcp58_515);
OPcp58_216 = OPcp58_215+ROcp58_515*qdd[16]-qd[16]*(OMcp58_115*ROcp58_615-OMcp58_315*ROcp58_415);
OPcp58_316 = OPcp58_315+ROcp58_615*qdd[16]+qd[16]*(OMcp58_115*ROcp58_515-OMcp58_215*ROcp58_415);
RLcp58_117 = s->dpt[1][28]*ROcp58_116+s->dpt[2][28]*ROcp58_415+ROcp58_716*s->dpt[3][28];
RLcp58_217 = s->dpt[1][28]*ROcp58_216+s->dpt[2][28]*ROcp58_515+ROcp58_816*s->dpt[3][28];
RLcp58_317 = s->dpt[1][28]*ROcp58_316+s->dpt[2][28]*ROcp58_615+ROcp58_916*s->dpt[3][28];
OMcp58_117 = OMcp58_116+ROcp58_116*qd[17];
OMcp58_217 = OMcp58_216+ROcp58_216*qd[17];
OMcp58_317 = OMcp58_316+ROcp58_316*qd[17];
ORcp58_117 = OMcp58_216*RLcp58_317-OMcp58_316*RLcp58_217;
ORcp58_217 = -(OMcp58_116*RLcp58_317-OMcp58_316*RLcp58_117);
ORcp58_317 = OMcp58_116*RLcp58_217-OMcp58_216*RLcp58_117;
OPcp58_117 = OPcp58_116+ROcp58_116*qdd[17]+qd[17]*(OMcp58_216*ROcp58_316-OMcp58_316*ROcp58_216);
OPcp58_217 = OPcp58_216+ROcp58_216*qdd[17]-qd[17]*(OMcp58_116*ROcp58_316-OMcp58_316*ROcp58_116);
OPcp58_317 = OPcp58_316+ROcp58_316*qdd[17]+qd[17]*(OMcp58_116*ROcp58_216-OMcp58_216*ROcp58_116);
RLcp58_118 = s->dpt[1][30]*ROcp58_116+s->dpt[2][30]*ROcp58_417+s->dpt[3][30]*ROcp58_717;
RLcp58_218 = s->dpt[1][30]*ROcp58_216+s->dpt[2][30]*ROcp58_517+s->dpt[3][30]*ROcp58_817;
RLcp58_318 = s->dpt[1][30]*ROcp58_316+s->dpt[2][30]*ROcp58_617+s->dpt[3][30]*ROcp58_917;
OMcp58_118 = OMcp58_117+ROcp58_417*qd[18];
OMcp58_218 = OMcp58_217+ROcp58_517*qd[18];
OMcp58_318 = OMcp58_317+ROcp58_617*qd[18];
ORcp58_118 = OMcp58_217*RLcp58_318-OMcp58_317*RLcp58_218;
ORcp58_218 = -(OMcp58_117*RLcp58_318-OMcp58_317*RLcp58_118);
ORcp58_318 = OMcp58_117*RLcp58_218-OMcp58_217*RLcp58_118;
OPcp58_118 = OPcp58_117+ROcp58_417*qdd[18]+qd[18]*(OMcp58_217*ROcp58_617-OMcp58_317*ROcp58_517);
OPcp58_218 = OPcp58_217+ROcp58_517*qdd[18]-qd[18]*(OMcp58_117*ROcp58_617-OMcp58_317*ROcp58_417);
OPcp58_318 = OPcp58_317+ROcp58_617*qdd[18]+qd[18]*(OMcp58_117*ROcp58_517-OMcp58_217*ROcp58_417);
RLcp58_188 = ROcp58_118*s->dpt[1][35]+ROcp58_417*s->dpt[2][35]+ROcp58_718*s->dpt[3][35];
RLcp58_288 = ROcp58_218*s->dpt[1][35]+ROcp58_517*s->dpt[2][35]+ROcp58_818*s->dpt[3][35];
RLcp58_388 = ROcp58_318*s->dpt[1][35]+ROcp58_617*s->dpt[2][35]+ROcp58_918*s->dpt[3][35];
POcp58_188 = RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_188+q[1];
POcp58_288 = RLcp58_213+RLcp58_214+RLcp58_215+RLcp58_216+RLcp58_217+RLcp58_218+RLcp58_288+q[2];
POcp58_388 = RLcp58_313+RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_388+q[3];
JTcp58_288_4 = -(RLcp58_313+RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_388);
JTcp58_388_4 = RLcp58_213+RLcp58_214+RLcp58_215+RLcp58_216+RLcp58_217+RLcp58_218+RLcp58_288;
JTcp58_188_5 = C4*(RLcp58_313+RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318)-S4*(RLcp58_213+RLcp58_214)-S4*(
 RLcp58_215+RLcp58_216)-S4*(RLcp58_217+RLcp58_218)-RLcp58_288*S4+RLcp58_388*C4;
JTcp58_288_5 = S4*(RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_188);
JTcp58_388_5 = -C4*(RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_188);
JTcp58_188_6 = ROcp58_85*(RLcp58_313+RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318)-ROcp58_95*(RLcp58_213+
 RLcp58_214)-ROcp58_95*(RLcp58_215+RLcp58_216)-ROcp58_95*(RLcp58_217+RLcp58_218)-RLcp58_288*ROcp58_95+RLcp58_388*ROcp58_85;
JTcp58_288_6 = -(RLcp58_388*S5-ROcp58_95*(RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_188
 )+S5*(RLcp58_313+RLcp58_314)+S5*(RLcp58_315+RLcp58_316)+S5*(RLcp58_317+RLcp58_318));
JTcp58_388_6 = RLcp58_288*S5-ROcp58_85*(RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_188)+
 S5*(RLcp58_213+RLcp58_214)+S5*(RLcp58_215+RLcp58_216)+S5*(RLcp58_217+RLcp58_218);
JTcp58_188_7 = ROcp58_56*(RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_388)-ROcp58_66*(RLcp58_214+
 RLcp58_215)-ROcp58_66*(RLcp58_216+RLcp58_217)-ROcp58_66*(RLcp58_218+RLcp58_288);
JTcp58_288_7 = -(ROcp58_46*(RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_388)-ROcp58_66*(RLcp58_114+
 RLcp58_115)-ROcp58_66*(RLcp58_116+RLcp58_117)-ROcp58_66*(RLcp58_118+RLcp58_188));
JTcp58_388_7 = ROcp58_46*(RLcp58_214+RLcp58_215+RLcp58_216+RLcp58_217+RLcp58_218+RLcp58_288)-ROcp58_56*(RLcp58_114+
 RLcp58_115)-ROcp58_56*(RLcp58_116+RLcp58_117)-ROcp58_56*(RLcp58_118+RLcp58_188);
JTcp58_188_8 = ROcp58_213*(RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318)-ROcp58_313*(RLcp58_215+RLcp58_216)-ROcp58_313*
 (RLcp58_217+RLcp58_218)-RLcp58_288*ROcp58_313+RLcp58_388*ROcp58_213;
JTcp58_288_8 = RLcp58_188*ROcp58_313-RLcp58_388*ROcp58_113-ROcp58_113*(RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318)+
 ROcp58_313*(RLcp58_115+RLcp58_116)+ROcp58_313*(RLcp58_117+RLcp58_118);
JTcp58_388_8 = ROcp58_113*(RLcp58_215+RLcp58_216+RLcp58_217+RLcp58_218)-ROcp58_213*(RLcp58_115+RLcp58_116)-ROcp58_213*
 (RLcp58_117+RLcp58_118)-RLcp58_188*ROcp58_213+RLcp58_288*ROcp58_113;
JTcp58_188_9 = ROcp58_814*(RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_388)-ROcp58_914*(RLcp58_216+RLcp58_217)-ROcp58_914*
 (RLcp58_218+RLcp58_288);
JTcp58_288_9 = -(ROcp58_714*(RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_388)-ROcp58_914*(RLcp58_116+RLcp58_117)-
 ROcp58_914*(RLcp58_118+RLcp58_188));
JTcp58_388_9 = ROcp58_714*(RLcp58_216+RLcp58_217+RLcp58_218+RLcp58_288)-ROcp58_814*(RLcp58_116+RLcp58_117)-ROcp58_814*
 (RLcp58_118+RLcp58_188);
JTcp58_188_10 = ROcp58_515*(RLcp58_317+RLcp58_318)-ROcp58_615*(RLcp58_217+RLcp58_218)-RLcp58_288*ROcp58_615+RLcp58_388
 *ROcp58_515;
JTcp58_288_10 = RLcp58_188*ROcp58_615-RLcp58_388*ROcp58_415-ROcp58_415*(RLcp58_317+RLcp58_318)+ROcp58_615*(RLcp58_117+
 RLcp58_118);
JTcp58_388_10 = ROcp58_415*(RLcp58_217+RLcp58_218)-ROcp58_515*(RLcp58_117+RLcp58_118)-RLcp58_188*ROcp58_515+RLcp58_288
 *ROcp58_415;
JTcp58_188_11 = ROcp58_216*(RLcp58_318+RLcp58_388)-ROcp58_316*(RLcp58_218+RLcp58_288);
JTcp58_288_11 = -(ROcp58_116*(RLcp58_318+RLcp58_388)-ROcp58_316*(RLcp58_118+RLcp58_188));
JTcp58_388_11 = ROcp58_116*(RLcp58_218+RLcp58_288)-ROcp58_216*(RLcp58_118+RLcp58_188);
JTcp58_188_12 = -(RLcp58_288*ROcp58_617-RLcp58_388*ROcp58_517);
JTcp58_288_12 = RLcp58_188*ROcp58_617-RLcp58_388*ROcp58_417;
JTcp58_388_12 = -(RLcp58_188*ROcp58_517-RLcp58_288*ROcp58_417);
ORcp58_188 = OMcp58_218*RLcp58_388-OMcp58_318*RLcp58_288;
ORcp58_288 = -(OMcp58_118*RLcp58_388-OMcp58_318*RLcp58_188);
ORcp58_388 = OMcp58_118*RLcp58_288-OMcp58_218*RLcp58_188;
VIcp58_188 = ORcp58_113+ORcp58_114+ORcp58_115+ORcp58_116+ORcp58_117+ORcp58_118+ORcp58_188+qd[1];
VIcp58_288 = ORcp58_213+ORcp58_214+ORcp58_215+ORcp58_216+ORcp58_217+ORcp58_218+ORcp58_288+qd[2];
VIcp58_388 = ORcp58_313+ORcp58_314+ORcp58_315+ORcp58_316+ORcp58_317+ORcp58_318+ORcp58_388+qd[3];
ACcp58_188 = qdd[1]+OMcp58_213*ORcp58_314+OMcp58_214*ORcp58_315+OMcp58_215*ORcp58_316+OMcp58_216*ORcp58_317+OMcp58_217
 *ORcp58_318+OMcp58_218*ORcp58_388+OMcp58_26*ORcp58_313-OMcp58_313*ORcp58_214-OMcp58_314*ORcp58_215-OMcp58_315*ORcp58_216-
 OMcp58_316*ORcp58_217-OMcp58_317*ORcp58_218-OMcp58_318*ORcp58_288-OMcp58_36*ORcp58_213+OPcp58_213*RLcp58_314+OPcp58_214*
 RLcp58_315+OPcp58_215*RLcp58_316+OPcp58_216*RLcp58_317+OPcp58_217*RLcp58_318+OPcp58_218*RLcp58_388+OPcp58_26*RLcp58_313-
 OPcp58_313*RLcp58_214-OPcp58_314*RLcp58_215-OPcp58_315*RLcp58_216-OPcp58_316*RLcp58_217-OPcp58_317*RLcp58_218-OPcp58_318*
 RLcp58_288-OPcp58_36*RLcp58_213;
ACcp58_288 = qdd[2]-OMcp58_113*ORcp58_314-OMcp58_114*ORcp58_315-OMcp58_115*ORcp58_316-OMcp58_116*ORcp58_317-OMcp58_117
 *ORcp58_318-OMcp58_118*ORcp58_388-OMcp58_16*ORcp58_313+OMcp58_313*ORcp58_114+OMcp58_314*ORcp58_115+OMcp58_315*ORcp58_116+
 OMcp58_316*ORcp58_117+OMcp58_317*ORcp58_118+OMcp58_318*ORcp58_188+OMcp58_36*ORcp58_113-OPcp58_113*RLcp58_314-OPcp58_114*
 RLcp58_315-OPcp58_115*RLcp58_316-OPcp58_116*RLcp58_317-OPcp58_117*RLcp58_318-OPcp58_118*RLcp58_388-OPcp58_16*RLcp58_313+
 OPcp58_313*RLcp58_114+OPcp58_314*RLcp58_115+OPcp58_315*RLcp58_116+OPcp58_316*RLcp58_117+OPcp58_317*RLcp58_118+OPcp58_318*
 RLcp58_188+OPcp58_36*RLcp58_113;
ACcp58_388 = qdd[3]+OMcp58_113*ORcp58_214+OMcp58_114*ORcp58_215+OMcp58_115*ORcp58_216+OMcp58_116*ORcp58_217+OMcp58_117
 *ORcp58_218+OMcp58_118*ORcp58_288+OMcp58_16*ORcp58_213-OMcp58_213*ORcp58_114-OMcp58_214*ORcp58_115-OMcp58_215*ORcp58_116-
 OMcp58_216*ORcp58_117-OMcp58_217*ORcp58_118-OMcp58_218*ORcp58_188-OMcp58_26*ORcp58_113+OPcp58_113*RLcp58_214+OPcp58_114*
 RLcp58_215+OPcp58_115*RLcp58_216+OPcp58_116*RLcp58_217+OPcp58_117*RLcp58_218+OPcp58_118*RLcp58_288+OPcp58_16*RLcp58_213-
 OPcp58_213*RLcp58_114-OPcp58_214*RLcp58_115-OPcp58_215*RLcp58_116-OPcp58_216*RLcp58_117-OPcp58_217*RLcp58_118-OPcp58_218*
 RLcp58_188-OPcp58_26*RLcp58_113;

// = = Block_1_0_0_59_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp58_188;
sens->P[2] = POcp58_288;
sens->P[3] = POcp58_388;
sens->R[1][1] = ROcp58_118;
sens->R[1][2] = ROcp58_218;
sens->R[1][3] = ROcp58_318;
sens->R[2][1] = ROcp58_417;
sens->R[2][2] = ROcp58_517;
sens->R[2][3] = ROcp58_617;
sens->R[3][1] = ROcp58_718;
sens->R[3][2] = ROcp58_818;
sens->R[3][3] = ROcp58_918;
sens->V[1] = VIcp58_188;
sens->V[2] = VIcp58_288;
sens->V[3] = VIcp58_388;
sens->OM[1] = OMcp58_118;
sens->OM[2] = OMcp58_218;
sens->OM[3] = OMcp58_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp58_188_5;
sens->J[1][6] = JTcp58_188_6;
sens->J[1][13] = JTcp58_188_7;
sens->J[1][14] = JTcp58_188_8;
sens->J[1][15] = JTcp58_188_9;
sens->J[1][16] = JTcp58_188_10;
sens->J[1][17] = JTcp58_188_11;
sens->J[1][18] = JTcp58_188_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp58_288_4;
sens->J[2][5] = JTcp58_288_5;
sens->J[2][6] = JTcp58_288_6;
sens->J[2][13] = JTcp58_288_7;
sens->J[2][14] = JTcp58_288_8;
sens->J[2][15] = JTcp58_288_9;
sens->J[2][16] = JTcp58_288_10;
sens->J[2][17] = JTcp58_288_11;
sens->J[2][18] = JTcp58_288_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp58_388_4;
sens->J[3][5] = JTcp58_388_5;
sens->J[3][6] = JTcp58_388_6;
sens->J[3][13] = JTcp58_388_7;
sens->J[3][14] = JTcp58_388_8;
sens->J[3][15] = JTcp58_388_9;
sens->J[3][16] = JTcp58_388_10;
sens->J[3][17] = JTcp58_388_11;
sens->J[3][18] = JTcp58_388_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp58_46;
sens->J[4][14] = ROcp58_113;
sens->J[4][15] = ROcp58_714;
sens->J[4][16] = ROcp58_415;
sens->J[4][17] = ROcp58_116;
sens->J[4][18] = ROcp58_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp58_85;
sens->J[5][13] = ROcp58_56;
sens->J[5][14] = ROcp58_213;
sens->J[5][15] = ROcp58_814;
sens->J[5][16] = ROcp58_515;
sens->J[5][17] = ROcp58_216;
sens->J[5][18] = ROcp58_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp58_95;
sens->J[6][13] = ROcp58_66;
sens->J[6][14] = ROcp58_313;
sens->J[6][15] = ROcp58_914;
sens->J[6][16] = ROcp58_615;
sens->J[6][17] = ROcp58_316;
sens->J[6][18] = ROcp58_617;
sens->A[1] = ACcp58_188;
sens->A[2] = ACcp58_288;
sens->A[3] = ACcp58_388;
sens->OMP[1] = OPcp58_118;
sens->OMP[2] = OPcp58_218;
sens->OMP[3] = OPcp58_318;
 
// 
break;
case 60:
 


// = = Block_1_0_0_60_0_1 = = 
 
// Sensor Kinematics 


ROcp59_25 = S4*S5;
ROcp59_35 = -C4*S5;
ROcp59_85 = -S4*C5;
ROcp59_95 = C4*C5;
ROcp59_16 = C5*C6;
ROcp59_26 = ROcp59_25*C6+C4*S6;
ROcp59_36 = ROcp59_35*C6+S4*S6;
ROcp59_46 = -C5*S6;
ROcp59_56 = -(ROcp59_25*S6-C4*C6);
ROcp59_66 = -(ROcp59_35*S6-S4*C6);
OMcp59_25 = qd[5]*C4;
OMcp59_35 = qd[5]*S4;
OMcp59_16 = qd[4]+qd[6]*S5;
OMcp59_26 = OMcp59_25+ROcp59_85*qd[6];
OMcp59_36 = OMcp59_35+ROcp59_95*qd[6];
OPcp59_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp59_26 = ROcp59_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp59_35*S5-ROcp59_95*qd[4]);
OPcp59_36 = ROcp59_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp59_25*S5-ROcp59_85*qd[4]);

// = = Block_1_0_0_60_0_3 = = 
 
// Sensor Kinematics 


ROcp59_113 = ROcp59_16*C13-S13*S5;
ROcp59_213 = ROcp59_26*C13-ROcp59_85*S13;
ROcp59_313 = ROcp59_36*C13-ROcp59_95*S13;
ROcp59_713 = ROcp59_16*S13+C13*S5;
ROcp59_813 = ROcp59_26*S13+ROcp59_85*C13;
ROcp59_913 = ROcp59_36*S13+ROcp59_95*C13;
ROcp59_414 = ROcp59_46*C14+ROcp59_713*S14;
ROcp59_514 = ROcp59_56*C14+ROcp59_813*S14;
ROcp59_614 = ROcp59_66*C14+ROcp59_913*S14;
ROcp59_714 = -(ROcp59_46*S14-ROcp59_713*C14);
ROcp59_814 = -(ROcp59_56*S14-ROcp59_813*C14);
ROcp59_914 = -(ROcp59_66*S14-ROcp59_913*C14);
ROcp59_115 = ROcp59_113*C15+ROcp59_414*S15;
ROcp59_215 = ROcp59_213*C15+ROcp59_514*S15;
ROcp59_315 = ROcp59_313*C15+ROcp59_614*S15;
ROcp59_415 = -(ROcp59_113*S15-ROcp59_414*C15);
ROcp59_515 = -(ROcp59_213*S15-ROcp59_514*C15);
ROcp59_615 = -(ROcp59_313*S15-ROcp59_614*C15);
ROcp59_116 = ROcp59_115*C16-ROcp59_714*S16;
ROcp59_216 = ROcp59_215*C16-ROcp59_814*S16;
ROcp59_316 = ROcp59_315*C16-ROcp59_914*S16;
ROcp59_716 = ROcp59_115*S16+ROcp59_714*C16;
ROcp59_816 = ROcp59_215*S16+ROcp59_814*C16;
ROcp59_916 = ROcp59_315*S16+ROcp59_914*C16;
ROcp59_417 = ROcp59_415*C17+ROcp59_716*S17;
ROcp59_517 = ROcp59_515*C17+ROcp59_816*S17;
ROcp59_617 = ROcp59_615*C17+ROcp59_916*S17;
ROcp59_717 = -(ROcp59_415*S17-ROcp59_716*C17);
ROcp59_817 = -(ROcp59_515*S17-ROcp59_816*C17);
ROcp59_917 = -(ROcp59_615*S17-ROcp59_916*C17);
ROcp59_118 = ROcp59_116*C18-ROcp59_717*S18;
ROcp59_218 = ROcp59_216*C18-ROcp59_817*S18;
ROcp59_318 = ROcp59_316*C18-ROcp59_917*S18;
ROcp59_718 = ROcp59_116*S18+ROcp59_717*C18;
ROcp59_818 = ROcp59_216*S18+ROcp59_817*C18;
ROcp59_918 = ROcp59_316*S18+ROcp59_917*C18;
RLcp59_113 = s->dpt[1][2]*ROcp59_16+s->dpt[3][2]*S5+ROcp59_46*s->dpt[2][2];
RLcp59_213 = s->dpt[1][2]*ROcp59_26+s->dpt[3][2]*ROcp59_85+ROcp59_56*s->dpt[2][2];
RLcp59_313 = s->dpt[1][2]*ROcp59_36+s->dpt[3][2]*ROcp59_95+ROcp59_66*s->dpt[2][2];
OMcp59_113 = OMcp59_16+ROcp59_46*qd[13];
OMcp59_213 = OMcp59_26+ROcp59_56*qd[13];
OMcp59_313 = OMcp59_36+ROcp59_66*qd[13];
ORcp59_113 = OMcp59_26*RLcp59_313-OMcp59_36*RLcp59_213;
ORcp59_213 = -(OMcp59_16*RLcp59_313-OMcp59_36*RLcp59_113);
ORcp59_313 = OMcp59_16*RLcp59_213-OMcp59_26*RLcp59_113;
OPcp59_113 = OPcp59_16+ROcp59_46*qdd[13]+qd[13]*(OMcp59_26*ROcp59_66-OMcp59_36*ROcp59_56);
OPcp59_213 = OPcp59_26+ROcp59_56*qdd[13]-qd[13]*(OMcp59_16*ROcp59_66-OMcp59_36*ROcp59_46);
OPcp59_313 = OPcp59_36+ROcp59_66*qdd[13]+qd[13]*(OMcp59_16*ROcp59_56-OMcp59_26*ROcp59_46);
RLcp59_114 = s->dpt[1][22]*ROcp59_113+s->dpt[3][22]*ROcp59_713+ROcp59_46*s->dpt[2][22];
RLcp59_214 = s->dpt[1][22]*ROcp59_213+s->dpt[3][22]*ROcp59_813+ROcp59_56*s->dpt[2][22];
RLcp59_314 = s->dpt[1][22]*ROcp59_313+s->dpt[3][22]*ROcp59_913+ROcp59_66*s->dpt[2][22];
OMcp59_114 = OMcp59_113+ROcp59_113*qd[14];
OMcp59_214 = OMcp59_213+ROcp59_213*qd[14];
OMcp59_314 = OMcp59_313+ROcp59_313*qd[14];
ORcp59_114 = OMcp59_213*RLcp59_314-OMcp59_313*RLcp59_214;
ORcp59_214 = -(OMcp59_113*RLcp59_314-OMcp59_313*RLcp59_114);
ORcp59_314 = OMcp59_113*RLcp59_214-OMcp59_213*RLcp59_114;
OPcp59_114 = OPcp59_113+ROcp59_113*qdd[14]+qd[14]*(OMcp59_213*ROcp59_313-OMcp59_313*ROcp59_213);
OPcp59_214 = OPcp59_213+ROcp59_213*qdd[14]-qd[14]*(OMcp59_113*ROcp59_313-OMcp59_313*ROcp59_113);
OPcp59_314 = OPcp59_313+ROcp59_313*qdd[14]+qd[14]*(OMcp59_113*ROcp59_213-OMcp59_213*ROcp59_113);
RLcp59_115 = s->dpt[1][24]*ROcp59_113+s->dpt[2][24]*ROcp59_414+ROcp59_714*s->dpt[3][24];
RLcp59_215 = s->dpt[1][24]*ROcp59_213+s->dpt[2][24]*ROcp59_514+ROcp59_814*s->dpt[3][24];
RLcp59_315 = s->dpt[1][24]*ROcp59_313+s->dpt[2][24]*ROcp59_614+ROcp59_914*s->dpt[3][24];
OMcp59_115 = OMcp59_114+ROcp59_714*qd[15];
OMcp59_215 = OMcp59_214+ROcp59_814*qd[15];
OMcp59_315 = OMcp59_314+ROcp59_914*qd[15];
ORcp59_115 = OMcp59_214*RLcp59_315-OMcp59_314*RLcp59_215;
ORcp59_215 = -(OMcp59_114*RLcp59_315-OMcp59_314*RLcp59_115);
ORcp59_315 = OMcp59_114*RLcp59_215-OMcp59_214*RLcp59_115;
OPcp59_115 = OPcp59_114+ROcp59_714*qdd[15]+qd[15]*(OMcp59_214*ROcp59_914-OMcp59_314*ROcp59_814);
OPcp59_215 = OPcp59_214+ROcp59_814*qdd[15]-qd[15]*(OMcp59_114*ROcp59_914-OMcp59_314*ROcp59_714);
OPcp59_315 = OPcp59_314+ROcp59_914*qdd[15]+qd[15]*(OMcp59_114*ROcp59_814-OMcp59_214*ROcp59_714);
RLcp59_116 = s->dpt[1][26]*ROcp59_115+s->dpt[2][26]*ROcp59_415+ROcp59_714*s->dpt[3][26];
RLcp59_216 = s->dpt[1][26]*ROcp59_215+s->dpt[2][26]*ROcp59_515+ROcp59_814*s->dpt[3][26];
RLcp59_316 = s->dpt[1][26]*ROcp59_315+s->dpt[2][26]*ROcp59_615+ROcp59_914*s->dpt[3][26];
OMcp59_116 = OMcp59_115+ROcp59_415*qd[16];
OMcp59_216 = OMcp59_215+ROcp59_515*qd[16];
OMcp59_316 = OMcp59_315+ROcp59_615*qd[16];
ORcp59_116 = OMcp59_215*RLcp59_316-OMcp59_315*RLcp59_216;
ORcp59_216 = -(OMcp59_115*RLcp59_316-OMcp59_315*RLcp59_116);
ORcp59_316 = OMcp59_115*RLcp59_216-OMcp59_215*RLcp59_116;
OPcp59_116 = OPcp59_115+ROcp59_415*qdd[16]+qd[16]*(OMcp59_215*ROcp59_615-OMcp59_315*ROcp59_515);
OPcp59_216 = OPcp59_215+ROcp59_515*qdd[16]-qd[16]*(OMcp59_115*ROcp59_615-OMcp59_315*ROcp59_415);
OPcp59_316 = OPcp59_315+ROcp59_615*qdd[16]+qd[16]*(OMcp59_115*ROcp59_515-OMcp59_215*ROcp59_415);
RLcp59_117 = s->dpt[1][28]*ROcp59_116+s->dpt[2][28]*ROcp59_415+ROcp59_716*s->dpt[3][28];
RLcp59_217 = s->dpt[1][28]*ROcp59_216+s->dpt[2][28]*ROcp59_515+ROcp59_816*s->dpt[3][28];
RLcp59_317 = s->dpt[1][28]*ROcp59_316+s->dpt[2][28]*ROcp59_615+ROcp59_916*s->dpt[3][28];
OMcp59_117 = OMcp59_116+ROcp59_116*qd[17];
OMcp59_217 = OMcp59_216+ROcp59_216*qd[17];
OMcp59_317 = OMcp59_316+ROcp59_316*qd[17];
ORcp59_117 = OMcp59_216*RLcp59_317-OMcp59_316*RLcp59_217;
ORcp59_217 = -(OMcp59_116*RLcp59_317-OMcp59_316*RLcp59_117);
ORcp59_317 = OMcp59_116*RLcp59_217-OMcp59_216*RLcp59_117;
OPcp59_117 = OPcp59_116+ROcp59_116*qdd[17]+qd[17]*(OMcp59_216*ROcp59_316-OMcp59_316*ROcp59_216);
OPcp59_217 = OPcp59_216+ROcp59_216*qdd[17]-qd[17]*(OMcp59_116*ROcp59_316-OMcp59_316*ROcp59_116);
OPcp59_317 = OPcp59_316+ROcp59_316*qdd[17]+qd[17]*(OMcp59_116*ROcp59_216-OMcp59_216*ROcp59_116);
RLcp59_118 = s->dpt[1][30]*ROcp59_116+s->dpt[2][30]*ROcp59_417+s->dpt[3][30]*ROcp59_717;
RLcp59_218 = s->dpt[1][30]*ROcp59_216+s->dpt[2][30]*ROcp59_517+s->dpt[3][30]*ROcp59_817;
RLcp59_318 = s->dpt[1][30]*ROcp59_316+s->dpt[2][30]*ROcp59_617+s->dpt[3][30]*ROcp59_917;
OMcp59_118 = OMcp59_117+ROcp59_417*qd[18];
OMcp59_218 = OMcp59_217+ROcp59_517*qd[18];
OMcp59_318 = OMcp59_317+ROcp59_617*qd[18];
ORcp59_118 = OMcp59_217*RLcp59_318-OMcp59_317*RLcp59_218;
ORcp59_218 = -(OMcp59_117*RLcp59_318-OMcp59_317*RLcp59_118);
ORcp59_318 = OMcp59_117*RLcp59_218-OMcp59_217*RLcp59_118;
OPcp59_118 = OPcp59_117+ROcp59_417*qdd[18]+qd[18]*(OMcp59_217*ROcp59_617-OMcp59_317*ROcp59_517);
OPcp59_218 = OPcp59_217+ROcp59_517*qdd[18]-qd[18]*(OMcp59_117*ROcp59_617-OMcp59_317*ROcp59_417);
OPcp59_318 = OPcp59_317+ROcp59_617*qdd[18]+qd[18]*(OMcp59_117*ROcp59_517-OMcp59_217*ROcp59_417);
RLcp59_189 = ROcp59_118*s->dpt[1][36]+ROcp59_417*s->dpt[2][36]+ROcp59_718*s->dpt[3][36];
RLcp59_289 = ROcp59_218*s->dpt[1][36]+ROcp59_517*s->dpt[2][36]+ROcp59_818*s->dpt[3][36];
RLcp59_389 = ROcp59_318*s->dpt[1][36]+ROcp59_617*s->dpt[2][36]+ROcp59_918*s->dpt[3][36];
POcp59_189 = RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_189+q[1];
POcp59_289 = RLcp59_213+RLcp59_214+RLcp59_215+RLcp59_216+RLcp59_217+RLcp59_218+RLcp59_289+q[2];
POcp59_389 = RLcp59_313+RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_389+q[3];
JTcp59_289_4 = -(RLcp59_313+RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_389);
JTcp59_389_4 = RLcp59_213+RLcp59_214+RLcp59_215+RLcp59_216+RLcp59_217+RLcp59_218+RLcp59_289;
JTcp59_189_5 = C4*(RLcp59_313+RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318)-S4*(RLcp59_213+RLcp59_214)-S4*(
 RLcp59_215+RLcp59_216)-S4*(RLcp59_217+RLcp59_218)-RLcp59_289*S4+RLcp59_389*C4;
JTcp59_289_5 = S4*(RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_189);
JTcp59_389_5 = -C4*(RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_189);
JTcp59_189_6 = ROcp59_85*(RLcp59_313+RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318)-ROcp59_95*(RLcp59_213+
 RLcp59_214)-ROcp59_95*(RLcp59_215+RLcp59_216)-ROcp59_95*(RLcp59_217+RLcp59_218)-RLcp59_289*ROcp59_95+RLcp59_389*ROcp59_85;
JTcp59_289_6 = -(RLcp59_389*S5-ROcp59_95*(RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_189
 )+S5*(RLcp59_313+RLcp59_314)+S5*(RLcp59_315+RLcp59_316)+S5*(RLcp59_317+RLcp59_318));
JTcp59_389_6 = RLcp59_289*S5-ROcp59_85*(RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_189)+
 S5*(RLcp59_213+RLcp59_214)+S5*(RLcp59_215+RLcp59_216)+S5*(RLcp59_217+RLcp59_218);
JTcp59_189_7 = ROcp59_56*(RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_389)-ROcp59_66*(RLcp59_214+
 RLcp59_215)-ROcp59_66*(RLcp59_216+RLcp59_217)-ROcp59_66*(RLcp59_218+RLcp59_289);
JTcp59_289_7 = -(ROcp59_46*(RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_389)-ROcp59_66*(RLcp59_114+
 RLcp59_115)-ROcp59_66*(RLcp59_116+RLcp59_117)-ROcp59_66*(RLcp59_118+RLcp59_189));
JTcp59_389_7 = ROcp59_46*(RLcp59_214+RLcp59_215+RLcp59_216+RLcp59_217+RLcp59_218+RLcp59_289)-ROcp59_56*(RLcp59_114+
 RLcp59_115)-ROcp59_56*(RLcp59_116+RLcp59_117)-ROcp59_56*(RLcp59_118+RLcp59_189);
JTcp59_189_8 = ROcp59_213*(RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318)-ROcp59_313*(RLcp59_215+RLcp59_216)-ROcp59_313*
 (RLcp59_217+RLcp59_218)-RLcp59_289*ROcp59_313+RLcp59_389*ROcp59_213;
JTcp59_289_8 = RLcp59_189*ROcp59_313-RLcp59_389*ROcp59_113-ROcp59_113*(RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318)+
 ROcp59_313*(RLcp59_115+RLcp59_116)+ROcp59_313*(RLcp59_117+RLcp59_118);
JTcp59_389_8 = ROcp59_113*(RLcp59_215+RLcp59_216+RLcp59_217+RLcp59_218)-ROcp59_213*(RLcp59_115+RLcp59_116)-ROcp59_213*
 (RLcp59_117+RLcp59_118)-RLcp59_189*ROcp59_213+RLcp59_289*ROcp59_113;
JTcp59_189_9 = ROcp59_814*(RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_389)-ROcp59_914*(RLcp59_216+RLcp59_217)-ROcp59_914*
 (RLcp59_218+RLcp59_289);
JTcp59_289_9 = -(ROcp59_714*(RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_389)-ROcp59_914*(RLcp59_116+RLcp59_117)-
 ROcp59_914*(RLcp59_118+RLcp59_189));
JTcp59_389_9 = ROcp59_714*(RLcp59_216+RLcp59_217+RLcp59_218+RLcp59_289)-ROcp59_814*(RLcp59_116+RLcp59_117)-ROcp59_814*
 (RLcp59_118+RLcp59_189);
JTcp59_189_10 = ROcp59_515*(RLcp59_317+RLcp59_318)-ROcp59_615*(RLcp59_217+RLcp59_218)-RLcp59_289*ROcp59_615+RLcp59_389
 *ROcp59_515;
JTcp59_289_10 = RLcp59_189*ROcp59_615-RLcp59_389*ROcp59_415-ROcp59_415*(RLcp59_317+RLcp59_318)+ROcp59_615*(RLcp59_117+
 RLcp59_118);
JTcp59_389_10 = ROcp59_415*(RLcp59_217+RLcp59_218)-ROcp59_515*(RLcp59_117+RLcp59_118)-RLcp59_189*ROcp59_515+RLcp59_289
 *ROcp59_415;
JTcp59_189_11 = ROcp59_216*(RLcp59_318+RLcp59_389)-ROcp59_316*(RLcp59_218+RLcp59_289);
JTcp59_289_11 = -(ROcp59_116*(RLcp59_318+RLcp59_389)-ROcp59_316*(RLcp59_118+RLcp59_189));
JTcp59_389_11 = ROcp59_116*(RLcp59_218+RLcp59_289)-ROcp59_216*(RLcp59_118+RLcp59_189);
JTcp59_189_12 = -(RLcp59_289*ROcp59_617-RLcp59_389*ROcp59_517);
JTcp59_289_12 = RLcp59_189*ROcp59_617-RLcp59_389*ROcp59_417;
JTcp59_389_12 = -(RLcp59_189*ROcp59_517-RLcp59_289*ROcp59_417);
ORcp59_189 = OMcp59_218*RLcp59_389-OMcp59_318*RLcp59_289;
ORcp59_289 = -(OMcp59_118*RLcp59_389-OMcp59_318*RLcp59_189);
ORcp59_389 = OMcp59_118*RLcp59_289-OMcp59_218*RLcp59_189;
VIcp59_189 = ORcp59_113+ORcp59_114+ORcp59_115+ORcp59_116+ORcp59_117+ORcp59_118+ORcp59_189+qd[1];
VIcp59_289 = ORcp59_213+ORcp59_214+ORcp59_215+ORcp59_216+ORcp59_217+ORcp59_218+ORcp59_289+qd[2];
VIcp59_389 = ORcp59_313+ORcp59_314+ORcp59_315+ORcp59_316+ORcp59_317+ORcp59_318+ORcp59_389+qd[3];
ACcp59_189 = qdd[1]+OMcp59_213*ORcp59_314+OMcp59_214*ORcp59_315+OMcp59_215*ORcp59_316+OMcp59_216*ORcp59_317+OMcp59_217
 *ORcp59_318+OMcp59_218*ORcp59_389+OMcp59_26*ORcp59_313-OMcp59_313*ORcp59_214-OMcp59_314*ORcp59_215-OMcp59_315*ORcp59_216-
 OMcp59_316*ORcp59_217-OMcp59_317*ORcp59_218-OMcp59_318*ORcp59_289-OMcp59_36*ORcp59_213+OPcp59_213*RLcp59_314+OPcp59_214*
 RLcp59_315+OPcp59_215*RLcp59_316+OPcp59_216*RLcp59_317+OPcp59_217*RLcp59_318+OPcp59_218*RLcp59_389+OPcp59_26*RLcp59_313-
 OPcp59_313*RLcp59_214-OPcp59_314*RLcp59_215-OPcp59_315*RLcp59_216-OPcp59_316*RLcp59_217-OPcp59_317*RLcp59_218-OPcp59_318*
 RLcp59_289-OPcp59_36*RLcp59_213;
ACcp59_289 = qdd[2]-OMcp59_113*ORcp59_314-OMcp59_114*ORcp59_315-OMcp59_115*ORcp59_316-OMcp59_116*ORcp59_317-OMcp59_117
 *ORcp59_318-OMcp59_118*ORcp59_389-OMcp59_16*ORcp59_313+OMcp59_313*ORcp59_114+OMcp59_314*ORcp59_115+OMcp59_315*ORcp59_116+
 OMcp59_316*ORcp59_117+OMcp59_317*ORcp59_118+OMcp59_318*ORcp59_189+OMcp59_36*ORcp59_113-OPcp59_113*RLcp59_314-OPcp59_114*
 RLcp59_315-OPcp59_115*RLcp59_316-OPcp59_116*RLcp59_317-OPcp59_117*RLcp59_318-OPcp59_118*RLcp59_389-OPcp59_16*RLcp59_313+
 OPcp59_313*RLcp59_114+OPcp59_314*RLcp59_115+OPcp59_315*RLcp59_116+OPcp59_316*RLcp59_117+OPcp59_317*RLcp59_118+OPcp59_318*
 RLcp59_189+OPcp59_36*RLcp59_113;
ACcp59_389 = qdd[3]+OMcp59_113*ORcp59_214+OMcp59_114*ORcp59_215+OMcp59_115*ORcp59_216+OMcp59_116*ORcp59_217+OMcp59_117
 *ORcp59_218+OMcp59_118*ORcp59_289+OMcp59_16*ORcp59_213-OMcp59_213*ORcp59_114-OMcp59_214*ORcp59_115-OMcp59_215*ORcp59_116-
 OMcp59_216*ORcp59_117-OMcp59_217*ORcp59_118-OMcp59_218*ORcp59_189-OMcp59_26*ORcp59_113+OPcp59_113*RLcp59_214+OPcp59_114*
 RLcp59_215+OPcp59_115*RLcp59_216+OPcp59_116*RLcp59_217+OPcp59_117*RLcp59_218+OPcp59_118*RLcp59_289+OPcp59_16*RLcp59_213-
 OPcp59_213*RLcp59_114-OPcp59_214*RLcp59_115-OPcp59_215*RLcp59_116-OPcp59_216*RLcp59_117-OPcp59_217*RLcp59_118-OPcp59_218*
 RLcp59_189-OPcp59_26*RLcp59_113;

// = = Block_1_0_0_60_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp59_189;
sens->P[2] = POcp59_289;
sens->P[3] = POcp59_389;
sens->R[1][1] = ROcp59_118;
sens->R[1][2] = ROcp59_218;
sens->R[1][3] = ROcp59_318;
sens->R[2][1] = ROcp59_417;
sens->R[2][2] = ROcp59_517;
sens->R[2][3] = ROcp59_617;
sens->R[3][1] = ROcp59_718;
sens->R[3][2] = ROcp59_818;
sens->R[3][3] = ROcp59_918;
sens->V[1] = VIcp59_189;
sens->V[2] = VIcp59_289;
sens->V[3] = VIcp59_389;
sens->OM[1] = OMcp59_118;
sens->OM[2] = OMcp59_218;
sens->OM[3] = OMcp59_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp59_189_5;
sens->J[1][6] = JTcp59_189_6;
sens->J[1][13] = JTcp59_189_7;
sens->J[1][14] = JTcp59_189_8;
sens->J[1][15] = JTcp59_189_9;
sens->J[1][16] = JTcp59_189_10;
sens->J[1][17] = JTcp59_189_11;
sens->J[1][18] = JTcp59_189_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp59_289_4;
sens->J[2][5] = JTcp59_289_5;
sens->J[2][6] = JTcp59_289_6;
sens->J[2][13] = JTcp59_289_7;
sens->J[2][14] = JTcp59_289_8;
sens->J[2][15] = JTcp59_289_9;
sens->J[2][16] = JTcp59_289_10;
sens->J[2][17] = JTcp59_289_11;
sens->J[2][18] = JTcp59_289_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp59_389_4;
sens->J[3][5] = JTcp59_389_5;
sens->J[3][6] = JTcp59_389_6;
sens->J[3][13] = JTcp59_389_7;
sens->J[3][14] = JTcp59_389_8;
sens->J[3][15] = JTcp59_389_9;
sens->J[3][16] = JTcp59_389_10;
sens->J[3][17] = JTcp59_389_11;
sens->J[3][18] = JTcp59_389_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp59_46;
sens->J[4][14] = ROcp59_113;
sens->J[4][15] = ROcp59_714;
sens->J[4][16] = ROcp59_415;
sens->J[4][17] = ROcp59_116;
sens->J[4][18] = ROcp59_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp59_85;
sens->J[5][13] = ROcp59_56;
sens->J[5][14] = ROcp59_213;
sens->J[5][15] = ROcp59_814;
sens->J[5][16] = ROcp59_515;
sens->J[5][17] = ROcp59_216;
sens->J[5][18] = ROcp59_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp59_95;
sens->J[6][13] = ROcp59_66;
sens->J[6][14] = ROcp59_313;
sens->J[6][15] = ROcp59_914;
sens->J[6][16] = ROcp59_615;
sens->J[6][17] = ROcp59_316;
sens->J[6][18] = ROcp59_617;
sens->A[1] = ACcp59_189;
sens->A[2] = ACcp59_289;
sens->A[3] = ACcp59_389;
sens->OMP[1] = OPcp59_118;
sens->OMP[2] = OPcp59_218;
sens->OMP[3] = OPcp59_318;
 
// 
break;
case 61:
 


// = = Block_1_0_0_61_0_1 = = 
 
// Sensor Kinematics 


ROcp60_25 = S4*S5;
ROcp60_35 = -C4*S5;
ROcp60_85 = -S4*C5;
ROcp60_95 = C4*C5;
ROcp60_16 = C5*C6;
ROcp60_26 = ROcp60_25*C6+C4*S6;
ROcp60_36 = ROcp60_35*C6+S4*S6;
ROcp60_46 = -C5*S6;
ROcp60_56 = -(ROcp60_25*S6-C4*C6);
ROcp60_66 = -(ROcp60_35*S6-S4*C6);
OMcp60_25 = qd[5]*C4;
OMcp60_35 = qd[5]*S4;
OMcp60_16 = qd[4]+qd[6]*S5;
OMcp60_26 = OMcp60_25+ROcp60_85*qd[6];
OMcp60_36 = OMcp60_35+ROcp60_95*qd[6];
OPcp60_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
OPcp60_26 = ROcp60_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp60_35*S5-ROcp60_95*qd[4]);
OPcp60_36 = ROcp60_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp60_25*S5-ROcp60_85*qd[4]);

// = = Block_1_0_0_61_0_3 = = 
 
// Sensor Kinematics 


ROcp60_113 = ROcp60_16*C13-S13*S5;
ROcp60_213 = ROcp60_26*C13-ROcp60_85*S13;
ROcp60_313 = ROcp60_36*C13-ROcp60_95*S13;
ROcp60_713 = ROcp60_16*S13+C13*S5;
ROcp60_813 = ROcp60_26*S13+ROcp60_85*C13;
ROcp60_913 = ROcp60_36*S13+ROcp60_95*C13;
ROcp60_414 = ROcp60_46*C14+ROcp60_713*S14;
ROcp60_514 = ROcp60_56*C14+ROcp60_813*S14;
ROcp60_614 = ROcp60_66*C14+ROcp60_913*S14;
ROcp60_714 = -(ROcp60_46*S14-ROcp60_713*C14);
ROcp60_814 = -(ROcp60_56*S14-ROcp60_813*C14);
ROcp60_914 = -(ROcp60_66*S14-ROcp60_913*C14);
ROcp60_115 = ROcp60_113*C15+ROcp60_414*S15;
ROcp60_215 = ROcp60_213*C15+ROcp60_514*S15;
ROcp60_315 = ROcp60_313*C15+ROcp60_614*S15;
ROcp60_415 = -(ROcp60_113*S15-ROcp60_414*C15);
ROcp60_515 = -(ROcp60_213*S15-ROcp60_514*C15);
ROcp60_615 = -(ROcp60_313*S15-ROcp60_614*C15);
ROcp60_116 = ROcp60_115*C16-ROcp60_714*S16;
ROcp60_216 = ROcp60_215*C16-ROcp60_814*S16;
ROcp60_316 = ROcp60_315*C16-ROcp60_914*S16;
ROcp60_716 = ROcp60_115*S16+ROcp60_714*C16;
ROcp60_816 = ROcp60_215*S16+ROcp60_814*C16;
ROcp60_916 = ROcp60_315*S16+ROcp60_914*C16;
ROcp60_417 = ROcp60_415*C17+ROcp60_716*S17;
ROcp60_517 = ROcp60_515*C17+ROcp60_816*S17;
ROcp60_617 = ROcp60_615*C17+ROcp60_916*S17;
ROcp60_717 = -(ROcp60_415*S17-ROcp60_716*C17);
ROcp60_817 = -(ROcp60_515*S17-ROcp60_816*C17);
ROcp60_917 = -(ROcp60_615*S17-ROcp60_916*C17);
ROcp60_118 = ROcp60_116*C18-ROcp60_717*S18;
ROcp60_218 = ROcp60_216*C18-ROcp60_817*S18;
ROcp60_318 = ROcp60_316*C18-ROcp60_917*S18;
ROcp60_718 = ROcp60_116*S18+ROcp60_717*C18;
ROcp60_818 = ROcp60_216*S18+ROcp60_817*C18;
ROcp60_918 = ROcp60_316*S18+ROcp60_917*C18;
RLcp60_113 = s->dpt[1][2]*ROcp60_16+s->dpt[3][2]*S5+ROcp60_46*s->dpt[2][2];
RLcp60_213 = s->dpt[1][2]*ROcp60_26+s->dpt[3][2]*ROcp60_85+ROcp60_56*s->dpt[2][2];
RLcp60_313 = s->dpt[1][2]*ROcp60_36+s->dpt[3][2]*ROcp60_95+ROcp60_66*s->dpt[2][2];
OMcp60_113 = OMcp60_16+ROcp60_46*qd[13];
OMcp60_213 = OMcp60_26+ROcp60_56*qd[13];
OMcp60_313 = OMcp60_36+ROcp60_66*qd[13];
ORcp60_113 = OMcp60_26*RLcp60_313-OMcp60_36*RLcp60_213;
ORcp60_213 = -(OMcp60_16*RLcp60_313-OMcp60_36*RLcp60_113);
ORcp60_313 = OMcp60_16*RLcp60_213-OMcp60_26*RLcp60_113;
OPcp60_113 = OPcp60_16+ROcp60_46*qdd[13]+qd[13]*(OMcp60_26*ROcp60_66-OMcp60_36*ROcp60_56);
OPcp60_213 = OPcp60_26+ROcp60_56*qdd[13]-qd[13]*(OMcp60_16*ROcp60_66-OMcp60_36*ROcp60_46);
OPcp60_313 = OPcp60_36+ROcp60_66*qdd[13]+qd[13]*(OMcp60_16*ROcp60_56-OMcp60_26*ROcp60_46);
RLcp60_114 = s->dpt[1][22]*ROcp60_113+s->dpt[3][22]*ROcp60_713+ROcp60_46*s->dpt[2][22];
RLcp60_214 = s->dpt[1][22]*ROcp60_213+s->dpt[3][22]*ROcp60_813+ROcp60_56*s->dpt[2][22];
RLcp60_314 = s->dpt[1][22]*ROcp60_313+s->dpt[3][22]*ROcp60_913+ROcp60_66*s->dpt[2][22];
OMcp60_114 = OMcp60_113+ROcp60_113*qd[14];
OMcp60_214 = OMcp60_213+ROcp60_213*qd[14];
OMcp60_314 = OMcp60_313+ROcp60_313*qd[14];
ORcp60_114 = OMcp60_213*RLcp60_314-OMcp60_313*RLcp60_214;
ORcp60_214 = -(OMcp60_113*RLcp60_314-OMcp60_313*RLcp60_114);
ORcp60_314 = OMcp60_113*RLcp60_214-OMcp60_213*RLcp60_114;
OPcp60_114 = OPcp60_113+ROcp60_113*qdd[14]+qd[14]*(OMcp60_213*ROcp60_313-OMcp60_313*ROcp60_213);
OPcp60_214 = OPcp60_213+ROcp60_213*qdd[14]-qd[14]*(OMcp60_113*ROcp60_313-OMcp60_313*ROcp60_113);
OPcp60_314 = OPcp60_313+ROcp60_313*qdd[14]+qd[14]*(OMcp60_113*ROcp60_213-OMcp60_213*ROcp60_113);
RLcp60_115 = s->dpt[1][24]*ROcp60_113+s->dpt[2][24]*ROcp60_414+ROcp60_714*s->dpt[3][24];
RLcp60_215 = s->dpt[1][24]*ROcp60_213+s->dpt[2][24]*ROcp60_514+ROcp60_814*s->dpt[3][24];
RLcp60_315 = s->dpt[1][24]*ROcp60_313+s->dpt[2][24]*ROcp60_614+ROcp60_914*s->dpt[3][24];
OMcp60_115 = OMcp60_114+ROcp60_714*qd[15];
OMcp60_215 = OMcp60_214+ROcp60_814*qd[15];
OMcp60_315 = OMcp60_314+ROcp60_914*qd[15];
ORcp60_115 = OMcp60_214*RLcp60_315-OMcp60_314*RLcp60_215;
ORcp60_215 = -(OMcp60_114*RLcp60_315-OMcp60_314*RLcp60_115);
ORcp60_315 = OMcp60_114*RLcp60_215-OMcp60_214*RLcp60_115;
OPcp60_115 = OPcp60_114+ROcp60_714*qdd[15]+qd[15]*(OMcp60_214*ROcp60_914-OMcp60_314*ROcp60_814);
OPcp60_215 = OPcp60_214+ROcp60_814*qdd[15]-qd[15]*(OMcp60_114*ROcp60_914-OMcp60_314*ROcp60_714);
OPcp60_315 = OPcp60_314+ROcp60_914*qdd[15]+qd[15]*(OMcp60_114*ROcp60_814-OMcp60_214*ROcp60_714);
RLcp60_116 = s->dpt[1][26]*ROcp60_115+s->dpt[2][26]*ROcp60_415+ROcp60_714*s->dpt[3][26];
RLcp60_216 = s->dpt[1][26]*ROcp60_215+s->dpt[2][26]*ROcp60_515+ROcp60_814*s->dpt[3][26];
RLcp60_316 = s->dpt[1][26]*ROcp60_315+s->dpt[2][26]*ROcp60_615+ROcp60_914*s->dpt[3][26];
OMcp60_116 = OMcp60_115+ROcp60_415*qd[16];
OMcp60_216 = OMcp60_215+ROcp60_515*qd[16];
OMcp60_316 = OMcp60_315+ROcp60_615*qd[16];
ORcp60_116 = OMcp60_215*RLcp60_316-OMcp60_315*RLcp60_216;
ORcp60_216 = -(OMcp60_115*RLcp60_316-OMcp60_315*RLcp60_116);
ORcp60_316 = OMcp60_115*RLcp60_216-OMcp60_215*RLcp60_116;
OPcp60_116 = OPcp60_115+ROcp60_415*qdd[16]+qd[16]*(OMcp60_215*ROcp60_615-OMcp60_315*ROcp60_515);
OPcp60_216 = OPcp60_215+ROcp60_515*qdd[16]-qd[16]*(OMcp60_115*ROcp60_615-OMcp60_315*ROcp60_415);
OPcp60_316 = OPcp60_315+ROcp60_615*qdd[16]+qd[16]*(OMcp60_115*ROcp60_515-OMcp60_215*ROcp60_415);
RLcp60_117 = s->dpt[1][28]*ROcp60_116+s->dpt[2][28]*ROcp60_415+ROcp60_716*s->dpt[3][28];
RLcp60_217 = s->dpt[1][28]*ROcp60_216+s->dpt[2][28]*ROcp60_515+ROcp60_816*s->dpt[3][28];
RLcp60_317 = s->dpt[1][28]*ROcp60_316+s->dpt[2][28]*ROcp60_615+ROcp60_916*s->dpt[3][28];
OMcp60_117 = OMcp60_116+ROcp60_116*qd[17];
OMcp60_217 = OMcp60_216+ROcp60_216*qd[17];
OMcp60_317 = OMcp60_316+ROcp60_316*qd[17];
ORcp60_117 = OMcp60_216*RLcp60_317-OMcp60_316*RLcp60_217;
ORcp60_217 = -(OMcp60_116*RLcp60_317-OMcp60_316*RLcp60_117);
ORcp60_317 = OMcp60_116*RLcp60_217-OMcp60_216*RLcp60_117;
OPcp60_117 = OPcp60_116+ROcp60_116*qdd[17]+qd[17]*(OMcp60_216*ROcp60_316-OMcp60_316*ROcp60_216);
OPcp60_217 = OPcp60_216+ROcp60_216*qdd[17]-qd[17]*(OMcp60_116*ROcp60_316-OMcp60_316*ROcp60_116);
OPcp60_317 = OPcp60_316+ROcp60_316*qdd[17]+qd[17]*(OMcp60_116*ROcp60_216-OMcp60_216*ROcp60_116);
RLcp60_118 = s->dpt[1][30]*ROcp60_116+s->dpt[2][30]*ROcp60_417+s->dpt[3][30]*ROcp60_717;
RLcp60_218 = s->dpt[1][30]*ROcp60_216+s->dpt[2][30]*ROcp60_517+s->dpt[3][30]*ROcp60_817;
RLcp60_318 = s->dpt[1][30]*ROcp60_316+s->dpt[2][30]*ROcp60_617+s->dpt[3][30]*ROcp60_917;
OMcp60_118 = OMcp60_117+ROcp60_417*qd[18];
OMcp60_218 = OMcp60_217+ROcp60_517*qd[18];
OMcp60_318 = OMcp60_317+ROcp60_617*qd[18];
ORcp60_118 = OMcp60_217*RLcp60_318-OMcp60_317*RLcp60_218;
ORcp60_218 = -(OMcp60_117*RLcp60_318-OMcp60_317*RLcp60_118);
ORcp60_318 = OMcp60_117*RLcp60_218-OMcp60_217*RLcp60_118;
OPcp60_118 = OPcp60_117+ROcp60_417*qdd[18]+qd[18]*(OMcp60_217*ROcp60_617-OMcp60_317*ROcp60_517);
OPcp60_218 = OPcp60_217+ROcp60_517*qdd[18]-qd[18]*(OMcp60_117*ROcp60_617-OMcp60_317*ROcp60_417);
OPcp60_318 = OPcp60_317+ROcp60_617*qdd[18]+qd[18]*(OMcp60_117*ROcp60_517-OMcp60_217*ROcp60_417);
RLcp60_190 = ROcp60_118*s->dpt[1][37]+ROcp60_417*s->dpt[2][37]+ROcp60_718*s->dpt[3][37];
RLcp60_290 = ROcp60_218*s->dpt[1][37]+ROcp60_517*s->dpt[2][37]+ROcp60_818*s->dpt[3][37];
RLcp60_390 = ROcp60_318*s->dpt[1][37]+ROcp60_617*s->dpt[2][37]+ROcp60_918*s->dpt[3][37];
POcp60_190 = RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_190+q[1];
POcp60_290 = RLcp60_213+RLcp60_214+RLcp60_215+RLcp60_216+RLcp60_217+RLcp60_218+RLcp60_290+q[2];
POcp60_390 = RLcp60_313+RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_390+q[3];
JTcp60_290_4 = -(RLcp60_313+RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_390);
JTcp60_390_4 = RLcp60_213+RLcp60_214+RLcp60_215+RLcp60_216+RLcp60_217+RLcp60_218+RLcp60_290;
JTcp60_190_5 = C4*(RLcp60_313+RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318)-S4*(RLcp60_213+RLcp60_214)-S4*(
 RLcp60_215+RLcp60_216)-S4*(RLcp60_217+RLcp60_218)-RLcp60_290*S4+RLcp60_390*C4;
JTcp60_290_5 = S4*(RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_190);
JTcp60_390_5 = -C4*(RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_190);
JTcp60_190_6 = ROcp60_85*(RLcp60_313+RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318)-ROcp60_95*(RLcp60_213+
 RLcp60_214)-ROcp60_95*(RLcp60_215+RLcp60_216)-ROcp60_95*(RLcp60_217+RLcp60_218)-RLcp60_290*ROcp60_95+RLcp60_390*ROcp60_85;
JTcp60_290_6 = -(RLcp60_390*S5-ROcp60_95*(RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_190
 )+S5*(RLcp60_313+RLcp60_314)+S5*(RLcp60_315+RLcp60_316)+S5*(RLcp60_317+RLcp60_318));
JTcp60_390_6 = RLcp60_290*S5-ROcp60_85*(RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_190)+
 S5*(RLcp60_213+RLcp60_214)+S5*(RLcp60_215+RLcp60_216)+S5*(RLcp60_217+RLcp60_218);
JTcp60_190_7 = ROcp60_56*(RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_390)-ROcp60_66*(RLcp60_214+
 RLcp60_215)-ROcp60_66*(RLcp60_216+RLcp60_217)-ROcp60_66*(RLcp60_218+RLcp60_290);
JTcp60_290_7 = -(ROcp60_46*(RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_390)-ROcp60_66*(RLcp60_114+
 RLcp60_115)-ROcp60_66*(RLcp60_116+RLcp60_117)-ROcp60_66*(RLcp60_118+RLcp60_190));
JTcp60_390_7 = ROcp60_46*(RLcp60_214+RLcp60_215+RLcp60_216+RLcp60_217+RLcp60_218+RLcp60_290)-ROcp60_56*(RLcp60_114+
 RLcp60_115)-ROcp60_56*(RLcp60_116+RLcp60_117)-ROcp60_56*(RLcp60_118+RLcp60_190);
JTcp60_190_8 = ROcp60_213*(RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318)-ROcp60_313*(RLcp60_215+RLcp60_216)-ROcp60_313*
 (RLcp60_217+RLcp60_218)-RLcp60_290*ROcp60_313+RLcp60_390*ROcp60_213;
JTcp60_290_8 = RLcp60_190*ROcp60_313-RLcp60_390*ROcp60_113-ROcp60_113*(RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318)+
 ROcp60_313*(RLcp60_115+RLcp60_116)+ROcp60_313*(RLcp60_117+RLcp60_118);
JTcp60_390_8 = ROcp60_113*(RLcp60_215+RLcp60_216+RLcp60_217+RLcp60_218)-ROcp60_213*(RLcp60_115+RLcp60_116)-ROcp60_213*
 (RLcp60_117+RLcp60_118)-RLcp60_190*ROcp60_213+RLcp60_290*ROcp60_113;
JTcp60_190_9 = ROcp60_814*(RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_390)-ROcp60_914*(RLcp60_216+RLcp60_217)-ROcp60_914*
 (RLcp60_218+RLcp60_290);
JTcp60_290_9 = -(ROcp60_714*(RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_390)-ROcp60_914*(RLcp60_116+RLcp60_117)-
 ROcp60_914*(RLcp60_118+RLcp60_190));
JTcp60_390_9 = ROcp60_714*(RLcp60_216+RLcp60_217+RLcp60_218+RLcp60_290)-ROcp60_814*(RLcp60_116+RLcp60_117)-ROcp60_814*
 (RLcp60_118+RLcp60_190);
JTcp60_190_10 = ROcp60_515*(RLcp60_317+RLcp60_318)-ROcp60_615*(RLcp60_217+RLcp60_218)-RLcp60_290*ROcp60_615+RLcp60_390
 *ROcp60_515;
JTcp60_290_10 = RLcp60_190*ROcp60_615-RLcp60_390*ROcp60_415-ROcp60_415*(RLcp60_317+RLcp60_318)+ROcp60_615*(RLcp60_117+
 RLcp60_118);
JTcp60_390_10 = ROcp60_415*(RLcp60_217+RLcp60_218)-ROcp60_515*(RLcp60_117+RLcp60_118)-RLcp60_190*ROcp60_515+RLcp60_290
 *ROcp60_415;
JTcp60_190_11 = ROcp60_216*(RLcp60_318+RLcp60_390)-ROcp60_316*(RLcp60_218+RLcp60_290);
JTcp60_290_11 = -(ROcp60_116*(RLcp60_318+RLcp60_390)-ROcp60_316*(RLcp60_118+RLcp60_190));
JTcp60_390_11 = ROcp60_116*(RLcp60_218+RLcp60_290)-ROcp60_216*(RLcp60_118+RLcp60_190);
JTcp60_190_12 = -(RLcp60_290*ROcp60_617-RLcp60_390*ROcp60_517);
JTcp60_290_12 = RLcp60_190*ROcp60_617-RLcp60_390*ROcp60_417;
JTcp60_390_12 = -(RLcp60_190*ROcp60_517-RLcp60_290*ROcp60_417);
ORcp60_190 = OMcp60_218*RLcp60_390-OMcp60_318*RLcp60_290;
ORcp60_290 = -(OMcp60_118*RLcp60_390-OMcp60_318*RLcp60_190);
ORcp60_390 = OMcp60_118*RLcp60_290-OMcp60_218*RLcp60_190;
VIcp60_190 = ORcp60_113+ORcp60_114+ORcp60_115+ORcp60_116+ORcp60_117+ORcp60_118+ORcp60_190+qd[1];
VIcp60_290 = ORcp60_213+ORcp60_214+ORcp60_215+ORcp60_216+ORcp60_217+ORcp60_218+ORcp60_290+qd[2];
VIcp60_390 = ORcp60_313+ORcp60_314+ORcp60_315+ORcp60_316+ORcp60_317+ORcp60_318+ORcp60_390+qd[3];
ACcp60_190 = qdd[1]+OMcp60_213*ORcp60_314+OMcp60_214*ORcp60_315+OMcp60_215*ORcp60_316+OMcp60_216*ORcp60_317+OMcp60_217
 *ORcp60_318+OMcp60_218*ORcp60_390+OMcp60_26*ORcp60_313-OMcp60_313*ORcp60_214-OMcp60_314*ORcp60_215-OMcp60_315*ORcp60_216-
 OMcp60_316*ORcp60_217-OMcp60_317*ORcp60_218-OMcp60_318*ORcp60_290-OMcp60_36*ORcp60_213+OPcp60_213*RLcp60_314+OPcp60_214*
 RLcp60_315+OPcp60_215*RLcp60_316+OPcp60_216*RLcp60_317+OPcp60_217*RLcp60_318+OPcp60_218*RLcp60_390+OPcp60_26*RLcp60_313-
 OPcp60_313*RLcp60_214-OPcp60_314*RLcp60_215-OPcp60_315*RLcp60_216-OPcp60_316*RLcp60_217-OPcp60_317*RLcp60_218-OPcp60_318*
 RLcp60_290-OPcp60_36*RLcp60_213;
ACcp60_290 = qdd[2]-OMcp60_113*ORcp60_314-OMcp60_114*ORcp60_315-OMcp60_115*ORcp60_316-OMcp60_116*ORcp60_317-OMcp60_117
 *ORcp60_318-OMcp60_118*ORcp60_390-OMcp60_16*ORcp60_313+OMcp60_313*ORcp60_114+OMcp60_314*ORcp60_115+OMcp60_315*ORcp60_116+
 OMcp60_316*ORcp60_117+OMcp60_317*ORcp60_118+OMcp60_318*ORcp60_190+OMcp60_36*ORcp60_113-OPcp60_113*RLcp60_314-OPcp60_114*
 RLcp60_315-OPcp60_115*RLcp60_316-OPcp60_116*RLcp60_317-OPcp60_117*RLcp60_318-OPcp60_118*RLcp60_390-OPcp60_16*RLcp60_313+
 OPcp60_313*RLcp60_114+OPcp60_314*RLcp60_115+OPcp60_315*RLcp60_116+OPcp60_316*RLcp60_117+OPcp60_317*RLcp60_118+OPcp60_318*
 RLcp60_190+OPcp60_36*RLcp60_113;
ACcp60_390 = qdd[3]+OMcp60_113*ORcp60_214+OMcp60_114*ORcp60_215+OMcp60_115*ORcp60_216+OMcp60_116*ORcp60_217+OMcp60_117
 *ORcp60_218+OMcp60_118*ORcp60_290+OMcp60_16*ORcp60_213-OMcp60_213*ORcp60_114-OMcp60_214*ORcp60_115-OMcp60_215*ORcp60_116-
 OMcp60_216*ORcp60_117-OMcp60_217*ORcp60_118-OMcp60_218*ORcp60_190-OMcp60_26*ORcp60_113+OPcp60_113*RLcp60_214+OPcp60_114*
 RLcp60_215+OPcp60_115*RLcp60_216+OPcp60_116*RLcp60_217+OPcp60_117*RLcp60_218+OPcp60_118*RLcp60_290+OPcp60_16*RLcp60_213-
 OPcp60_213*RLcp60_114-OPcp60_214*RLcp60_115-OPcp60_215*RLcp60_116-OPcp60_216*RLcp60_117-OPcp60_217*RLcp60_118-OPcp60_218*
 RLcp60_190-OPcp60_26*RLcp60_113;

// = = Block_1_0_0_61_1_0 = = 
 
// Symbolic Outputs  

sens->P[1] = POcp60_190;
sens->P[2] = POcp60_290;
sens->P[3] = POcp60_390;
sens->R[1][1] = ROcp60_118;
sens->R[1][2] = ROcp60_218;
sens->R[1][3] = ROcp60_318;
sens->R[2][1] = ROcp60_417;
sens->R[2][2] = ROcp60_517;
sens->R[2][3] = ROcp60_617;
sens->R[3][1] = ROcp60_718;
sens->R[3][2] = ROcp60_818;
sens->R[3][3] = ROcp60_918;
sens->V[1] = VIcp60_190;
sens->V[2] = VIcp60_290;
sens->V[3] = VIcp60_390;
sens->OM[1] = OMcp60_118;
sens->OM[2] = OMcp60_218;
sens->OM[3] = OMcp60_318;
sens->J[1][1] = (1.0);
sens->J[1][5] = JTcp60_190_5;
sens->J[1][6] = JTcp60_190_6;
sens->J[1][13] = JTcp60_190_7;
sens->J[1][14] = JTcp60_190_8;
sens->J[1][15] = JTcp60_190_9;
sens->J[1][16] = JTcp60_190_10;
sens->J[1][17] = JTcp60_190_11;
sens->J[1][18] = JTcp60_190_12;
sens->J[2][2] = (1.0);
sens->J[2][4] = JTcp60_290_4;
sens->J[2][5] = JTcp60_290_5;
sens->J[2][6] = JTcp60_290_6;
sens->J[2][13] = JTcp60_290_7;
sens->J[2][14] = JTcp60_290_8;
sens->J[2][15] = JTcp60_290_9;
sens->J[2][16] = JTcp60_290_10;
sens->J[2][17] = JTcp60_290_11;
sens->J[2][18] = JTcp60_290_12;
sens->J[3][3] = (1.0);
sens->J[3][4] = JTcp60_390_4;
sens->J[3][5] = JTcp60_390_5;
sens->J[3][6] = JTcp60_390_6;
sens->J[3][13] = JTcp60_390_7;
sens->J[3][14] = JTcp60_390_8;
sens->J[3][15] = JTcp60_390_9;
sens->J[3][16] = JTcp60_390_10;
sens->J[3][17] = JTcp60_390_11;
sens->J[3][18] = JTcp60_390_12;
sens->J[4][4] = (1.0);
sens->J[4][6] = S5;
sens->J[4][13] = ROcp60_46;
sens->J[4][14] = ROcp60_113;
sens->J[4][15] = ROcp60_714;
sens->J[4][16] = ROcp60_415;
sens->J[4][17] = ROcp60_116;
sens->J[4][18] = ROcp60_417;
sens->J[5][5] = C4;
sens->J[5][6] = ROcp60_85;
sens->J[5][13] = ROcp60_56;
sens->J[5][14] = ROcp60_213;
sens->J[5][15] = ROcp60_814;
sens->J[5][16] = ROcp60_515;
sens->J[5][17] = ROcp60_216;
sens->J[5][18] = ROcp60_517;
sens->J[6][5] = S4;
sens->J[6][6] = ROcp60_95;
sens->J[6][13] = ROcp60_66;
sens->J[6][14] = ROcp60_313;
sens->J[6][15] = ROcp60_914;
sens->J[6][16] = ROcp60_615;
sens->J[6][17] = ROcp60_316;
sens->J[6][18] = ROcp60_617;
sens->A[1] = ACcp60_190;
sens->A[2] = ACcp60_290;
sens->A[3] = ACcp60_390;
sens->OMP[1] = OPcp60_118;
sens->OMP[2] = OPcp60_218;
sens->OMP[3] = OPcp60_318;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

