

 /**
  * UCL-CEREM-MBS
  *
  * @version MBsysLab_s 1.7
  *
  * @author Nicolas Docquier, inspired from Allan Barrea
  *
  *
  * BUGZILLA
  * --------
  * creation
  * 03/04/2013       ND     bug ???
  *
  * modifcations
  * ??/??/????       ??     bug ???
  */

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "MBSfun.h"
#include "MBSdataStruct.h"
#include "mbs_xml_def.h"
#include "string.h"


MBSdataStruct *loadMBSsizes_xml(xmlDocPtr doc){

    xmlNodePtr root = NULL;
    xmlNodePtr cur = NULL;
    MBSdataStruct *s;
	s = (MBSdataStruct*) malloc(sizeof(MBSdataStruct));
    memset ((void*)s, 0, sizeof(MBSdataStruct));

	root = xmlDocGetRootElement(doc);

	if (root == NULL) {
		printf("Cannot load MBSdataStruct - empty document \n");
		xmlFreeDoc(doc);
		return NULL;
	}


    // parse data
    cur = root->xmlChildrenNode;
    while (cur != NULL){
        if(cur->type == XML_ELEMENT_NODE){

			// njoint and nbody
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Njoint"))){
				s->njoint = mbs_xml_ParseInt(doc,cur);
				s->nbody = s->njoint;
			}
			// npt
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"npt"))){
				s->npt = mbs_xml_ParseInt(doc,cur);
			}
			// nqu
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nqu"))){
				s->nqu = mbs_xml_ParseInt(doc,cur);
			}
			// nqu
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nqc"))){
				s->nqc = mbs_xml_ParseInt(doc,cur);
			}
			// nqv
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nqv"))){
				s->nqv = mbs_xml_ParseInt(doc,cur);
			}
			// nhu
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nhu"))){
				s->nhu = mbs_xml_ParseInt(doc,cur);
			}
			// nqlocked
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nqlocked"))){
				s->nqlocked = mbs_xml_ParseInt(doc,cur);
			}
			// nqa
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nqa"))){
				s->nqa = mbs_xml_ParseInt(doc,cur);
			}
			// nqdriven
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"nqdriven"))){
				s->nqdriven = mbs_xml_ParseInt(doc,cur);
			}
			// Nloopc
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nloopc"))){
				s->Nloopc = mbs_xml_ParseInt(doc,cur);
			}
			// Nuserc
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nuserc"))){
				s->Nuserc = mbs_xml_ParseInt(doc,cur);
			}
			// Ncons
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Ncons"))){
				s->Ncons = mbs_xml_ParseInt(doc,cur);
			}
			// Nlink
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nlink"))){
				s->Nlink = mbs_xml_ParseInt(doc,cur);
			}
			// Nlink3D
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nlink3D"))){
				s->Nlink3D = mbs_xml_ParseInt(doc,cur);
			}
			// Nsensor
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nsensor"))){
				s->Nsensor = mbs_xml_ParseInt(doc,cur);
			}
			// Nxfrc
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nxfrc"))){
				s->Nxfrc = mbs_xml_ParseInt(doc,cur);
			}
			// Nwheel
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nwheel"))){
				s->Nwheel = mbs_xml_ParseInt(doc,cur);
			}
            #ifndef SENSORKIN
			// Nuser_model
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nuser_model"))){
				s->Nuser_model = mbs_xml_ParseInt(doc,cur);
			}
            #endif
			// Nux
			if ((!xmlStrcmp(cur->name, (const xmlChar *)"Nux"))){
				s->Nux = mbs_xml_ParseInt(doc,cur);
			}
		}
        cur=cur->next;
	}

    return s;

}

MBSdataStruct* loadMBSdata_xml(xmlDocPtr doc){

    int i,j, ncol;
    xmlNodePtr root = NULL;
    xmlNodePtr cur = NULL;
    MBSdataStruct *s;

	s = loadMBSsizes_xml(doc);

	root = xmlDocGetRootElement(doc);

    cur = root;

    //===  Index parameters  ======================================//

	/* qc */
	if (s->nqc)
	{
		s->qc = (int*) calloc(s->nqc+1,sizeof(int));
		s->qc[0] = s->nqc;
		mbs_xml_parseVectorInt("qc", doc, cur, s->qc);
	}
	else
		s->qc = NULL;


	/* qu */
	if (s->nqu)
	{
		s->qu = (int*) calloc(s->nqu+1,sizeof(int));
		s->qu[0] = s->nqu;
		mbs_xml_parseVectorInt("qu", doc, cur, s->qu);
	}
	else
		s->qu = NULL;

	/* qa */
	if (s->nqa)
	{
		s->qa = (int*) calloc(s->nqa+1,sizeof(int));
		s->qa[0] = s->nqa;
		mbs_xml_parseVectorInt("qa", doc, cur, s->qa);
	}
	else
		s->qa = NULL;

	/* qv */
	if (s->nqv)
	{
		s->qv = (int*) calloc(s->nqv+1,sizeof(int));
		s->qv[0] = s->nqv;
		mbs_xml_parseVectorInt("qv", doc, cur, s->qv);
	}
	else
		s->qv = NULL;

	/* hu */
	if (s->nhu)
	{
		s->hu = (int*) calloc(s->nhu+1,sizeof(int));
		s->hu[0] = s->nhu;
		mbs_xml_parseVectorInt("hu", doc, cur, s->hu);
	}
	else
		s->hu = NULL;

	/* qlocked */
	if (s->nqlocked)
	{
		s->qlocked = (int*) calloc(s->nqlocked+1,sizeof(int));
		s->qlocked[0] = s->nqlocked;
		mbs_xml_parseVectorInt("qlocked", doc, cur, s->qlocked);
	}
	else
		s->qlocked = NULL;

	/* qdriven */
	if (s->nqdriven)
	{
		s->qdriven = (int*) calloc(s->nqdriven+1,sizeof(int));
		s->qdriven[0] = s->nqdriven;
		mbs_xml_parseVectorInt("qdriven", doc, cur, s->qdriven);
	}
	else
		s->qdriven = NULL;

//===   Work Variables   ==========================================================//

	/* q,qd,qdd */
	s->q   = (double*) calloc(s->njoint+1,sizeof(double));
	s->qd  = (double*) calloc(s->njoint+1,sizeof(double));
	s->qdd = (double*) calloc(s->njoint+1,sizeof(double));
	s->q[0]   = (double) s->njoint;
	s->qd[0]  = (double) s->njoint;
	s->qdd[0] = (double) s->njoint;
	mbs_xml_parseVectorDouble("q", doc, cur, s->q);
	mbs_xml_parseVectorDouble("qd", doc, cur, s->qd);
	mbs_xml_parseVectorDouble("qdd", doc, cur, s->qdd);

	/* frc */
	s->frc[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->frc[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->frc[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->frc[i][j] = 0.0;
	}

	/* trq */
	s->trq[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->trq[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->trq[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->trq[i][j] = 0.0;
	}

	/* Qq */
	s->Qq = (double*) calloc(s->njoint+1,sizeof(double));
	s->Qq[0] = (double) s->njoint;
	for(i=1;i<=s->njoint;i++)
		s->Qq[i] = 0.0;

	/* tsim */
	s->tsim = 0.0;


//===   System parameters   ====================================================//

	/* dpt */
	s->dpt[0] = NULL;
	if (s->npt)
	{
		for(i=1;i<=3;i++)
		{
			s->dpt[i] = (double*) calloc(s->npt+1,sizeof(double));
			s->dpt[i][0] = (double) s->npt;
		}
		mbs_xml_parseMatrixDouble("dpt", doc, cur, s->dpt);
	}
	else // je pense que ce cas est impossible mais il faudrait vérifier avant de supprimer
		for(i=1;i<=3;i++)
			s->dpt[i] = NULL;

	/* l */
	s->l[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->l[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->l[i][0] = (double) s->nbody;
	}
	mbs_xml_parseMatrixDouble("l", doc, cur, s->l);

	/* m */
	s->m = (double*) calloc(s->nbody+1,sizeof(double));
	s->m[0] = (double) s->nbody;
	mbs_xml_parseVectorDouble("m", doc, cur, s->m);

	/* In */
	s->In[0] = NULL;
	for(i=1;i<=9;i++)
	{
		s->In[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->In[i][0] = (double) s->nbody;
	}
	mbs_xml_parseMatrixDouble("In", doc, cur, s->In);

	/* g */
	s->g[0] = 0.0;
    mbs_xml_parseVectorDouble("g", doc, cur, s->g);

/* Constraint data */

	/* lrod */
	ncol = mbs_xml_getNbColElement("lrod", doc, cur);
	if (ncol)
	{
		s->lrod = (double*) calloc(ncol+1,sizeof(double));
		s->lrod[0] = ncol;
		mbs_xml_parseVectorDouble("lrod", doc, cur, s->lrod);
	}
	else
		s->lrod = NULL;

	/* NRerr */
	if (!mbs_xml_isElement(doc,"/mbsdata/NRerr"))
		s->NRerr = 1e-9; //default value
	else
		s->NRerr = mbs_xml_parseScalarInt("NRerr", doc, cur);

/* Link Data */

	/* Z,Zd,Fl */
	if (s->Nlink)
	{
		s->Z  = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Zd = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Fl = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Z[0]  = (double) s->Nlink;
		s->Zd[0] = (double) s->Nlink;
		s->Fl[0] = (double) s->Nlink;
		for(i=1;i<=s->Nlink;i++)
		{
			s->Z[i]  = 0.0;
			s->Zd[i] = 0.0;
			s->Fl[i] = 0.0;
		}
	}
	else
	{
		s->Z  = NULL;
		s->Zd = NULL;
		s->Fl = NULL;
	}

	/* l3DWr */
	if (s->Nlink3D)
	{
		s->l3DWr = (double**) calloc(s->Nlink3D+1,sizeof(double*));
		s->l3DWr[0] = NULL;
		for(i=1;i<=s->Nlink3D;i++)
		{
			s->l3DWr[i] = (double*) calloc(6+1,sizeof(double));
			for(j=0;j<=6;j++)
				s->l3DWr[i][j] = 0.0;
		}
	}
	else
		s->l3DWr = NULL;

/* Ext. Forces Data */

	/* xfidpt */
	if (s->Nxfrc)
	{
		s->xfidpt = (int*) calloc(s->Nxfrc+1,sizeof(int));
		s->xfidpt[0] = s->Nxfrc;
        mbs_xml_parseVectorInt("xfidpt", doc, cur, s->xfidpt);
	}
	else
		s->xfidpt = NULL;

	/* SWr */
	if (s->Nxfrc)
	{
		s->SWr = (double**) calloc(s->Nxfrc+1,sizeof(double*));
		s->SWr[0] = NULL;
		for(i=1;i<=s->Nxfrc;i++)
		{
			s->SWr[i] = (double*) calloc(9+1,sizeof(double));
			for(j=0;j<=9;j++)
				s->SWr[i][j] = 0.0;
		}
	}
	else
		s->SWr = NULL;

/* Wheel Data */

	/* rnom */
	if (s->Nwheel)
	{
		s->rnom = (double*) calloc(s->Nwheel+1,sizeof(double));
		s->rnom[0] = (double) s->Nwheel;
		mbs_xml_parseVectorDouble("rnom", doc, cur, s->rnom);
	}
	else
		s->rnom = NULL;

#if !defined SENSORKIN
/* User Model */

	/* user_model */
	if (s->Nuser_model)
	{
		s->user_model = loadUserModel_xml(doc, cur);
	}

	/* ux,uxd,ux0 */
	if (s->Nux)
	{
		s->ux  = (double*) calloc(s->Nux+1,sizeof(double));
		s->uxd = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux0 = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux[0]  = (double) s->Nux;
		s->uxd[0] = (double) s->Nux;
		s->ux0[0] = (double) s->Nux;
		mbs_xml_parseVectorDouble("ux", doc, cur, s->ux);
		mbs_xml_parseVectorDouble("uxd", doc, cur, s->uxd);
		mbs_xml_parseVectorDouble("ux0", doc, cur, s->ux0);
	}
	else
	{
		s->ux  = NULL;
		s->uxd = NULL;
		s->ux0 = NULL;
	}


    /* User IO */
	s->user_IO = initUserIO(s);

#endif

//===  Other  ========================================================//

	/* qddu */
	if (s->nqu)
	{
		s->qddu = (double*) calloc(s->nqu+1,sizeof(double));
		s->qddu[0] = (double) s->nqu;
	}
	else
		s->qddu = NULL;

	/* DonePart */
	s->DonePart = mbs_xml_parseScalarInt("DonePart", doc, cur);

    return s;

}


void print_vector(char* prefix, int n, double *vec){
    int i;
    printf(" %s = [", prefix);
    for(i=1; i < n ; i++){
       printf(" %lf ", vec[i]);
    }
    printf("]\n");
}

void printMBSdata(MBSdataStruct* s){
    printf("\n MBSdataStruct \n");

    printf(" nbody  . .  = %d \n", s->nbody);
    printf(" njoint . .  = %d \n", s->njoint);
    printf(" npt  . . .  = %d \n", s->npt);
    printf(" nqu  . . .  = %d \n", s->nqu);
    printf(" nqc  . . .  = %d \n", s->nqc);
    printf(" nqa  . . .  = %d \n", s->nqa);
    printf(" nqv  . . .  = %d \n", s->nqv);
    printf(" nhu  . . .  = %d \n", s->nhu);
    printf(" nqlocked .  = %d \n", s->nqlocked);
    printf(" nqdriven .  = %d \n", s->nqdriven);
    printf(" Nloopc . .  = %d \n", s->Nloopc);
    printf(" Nuserc . .  = %d \n", s->Nuserc);
    printf(" Ncons  . .  = %d \n", s->Ncons);
    printf(" Nlink  . .  = %d \n", s->Nlink);
    printf(" Nlink3D  .  = %d \n", s->Nlink3D);
    printf(" Nsensor  .  = %d \n", s->Nsensor);
    printf(" Nxfrc  . .  = %d \n", s->Nxfrc);
    printf(" Nwheel . .  = %d \n", s->Nwheel);
    printf(" Nuser_model = %d \n", s->Nuser_model);
    printf(" Nux . . . . = %d \n", s->Nux);

    print_vector("q  ", s->njoint, s->q);
    print_vector("qd ", s->njoint, s->qd);
    print_vector("qdd", s->njoint, s->qdd);

    printUserModel(s);
}


