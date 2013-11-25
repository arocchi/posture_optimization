/*===========================================================================*
  *
  *  UserModelStruct.c
  *
  *  Project:	SliderCrankPP
  *
  *  Generation date: 02-Apr-2013 14:01:03
  *
  *  (c) Universite catholique de Louvain
  *      Département de Mécanique
  *      Unité de Production Mécanique et Machines
  *      2, Place du Levant
  *      1348 Louvain-la-Neuve
  *  http://www.robotran.be//
  *
 /*===========================================================================*/

#include "MBSdataStruct.h"
#include "MBSfun.h"


#include "lut.h"
#include "mbs_xml_def.h"
#include <stdlib.h>


/**
 * Load the user models.
 * cur is supposed to be the user_model element
 */
UserModelStruct * loadUserModel_xml(const xmlDocPtr doc, const xmlNodePtr cur2)
{
    UserModelStruct *ums;

    ums = (UserModelStruct*) malloc(sizeof(UserModelStruct));

    return ums;
}

void freeUserModel(UserModelStruct *ums)
{

// RodSpring //

// LateralSpring //

// CrankTorque //

    free(ums);

}

void printUserModel(MBSdataStruct* s){
}
