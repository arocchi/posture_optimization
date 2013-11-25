/*===========================================================================*
  *
  *  UserModelStruct.h
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

#ifndef UserModelStruct_h
 #define UserModelStruct_h
 /*--------------------*/
 #include "lut.h"
 
 
 typedef struct UserModelStruct 
 {
 
     struct GRF{
         double K;
         double D;
         double mu;
         double *FT;
         double *Flag_Feet;
         double *Temp_Feet;
     } GRF;
 
     struct Actuator{
         double *Control;
         double *KKs;
         double *DDs;
         double Jdrvies;
         double Ddrives;
         double VTgain;
     } Actuator;
 
 
 } UserModelStruct;
 
 /*--------------------*/
 #endif
 