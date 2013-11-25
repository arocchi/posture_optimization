
 /**
  * UCL-CEREM-MBS
  *
  * Definition of useful functions for loading the MBSdata structure from
  * an xml file.
  *
  * @version MBsysLab_s 1.7
  *
  * @author Nicolas Docquier
  *
  *
  * BUGZILLA
  * --------
  * creation
  * 04/04/2013       ND     bug ???
  *
  * modifcations
  * ??/??/????       ??     bug ???
  */


#ifndef _MBS_XML_DEF_H
#define	_MBS_XML_DEF_H

#include <stdio.h>
#include <stdlib.h>

//#include "libxml/xmlmemory.h"
#include "libxml/parser.h"
//#include <libxml2/libxml/parser.h>
#include <libxml/xpath.h>


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// USEFULL PARSING FUNCTIONS


int mbs_xml_getElemByXptah(const xmlChar* elementName, xmlDocPtr doc, xmlNodePtr* first);
int mbs_xml_isElement(xmlDocPtr doc, const char* elementName);

xmlChar* mbs_xml_parseElem(const xmlChar* elementName, xmlDocPtr doc, xmlNodePtr cur, int elementSize[2]);


// parse functons
int  mbs_xml_parseScalarInt(const char* elementName, xmlDocPtr doc, xmlNodePtr cur);
double  mbs_xml_parseScalarDouble(const char* elementName, xmlDocPtr doc, xmlNodePtr cur);

void mbs_xml_parseVectorInt(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, int vectorToExtract[]);

void mbs_xml_parseVectorDouble(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double vectorToExtract[]);

void mbs_xml_parseMatrixDouble(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double** matrixToExtract);

int mbs_xml_getNbColElement(const char* elementName, xmlDocPtr doc, xmlNodePtr cur);

void mbs_xml_getSize(xmlNodePtr cur, int tab[2]);


// string to math conversion functions
void mbs_xml_convertVectorInt(int* vectorToExtract, char* elementValue, int* elementSize);
void mbs_xml_convertVectorDouble(double* vectorToExtract, char* elementValue, int* elementSize);
void mbs_xml_convertMatrixDouble(double** matrixToExtract, char* elementValue, int* elementSize);

// utilities
int mbs_xml_maxInt(int a, int b);

// parse math values from the xml node
double* mbs_xml_parseElemToVectorDouble(xmlDocPtr doc, xmlNodePtr cur);


/**
 * Parse a real value form a xml element and return the value
 * as a double
 */
double mbs_xml_ParseValue(xmlDocPtr doc, xmlNodePtr cur);

/**
 * Parse a int value form a xml element and return the value
 * as a int
 */
double mbs_xml_ParseInt(xmlDocPtr doc, xmlNodePtr cur);

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

#endif
