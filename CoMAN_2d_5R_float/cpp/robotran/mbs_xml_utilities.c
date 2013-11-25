

 /**
  * UCL-CEREM-MBS
  *
  * Definition of useful functions for loading the MBSdata structure from
  * an xml file.
  *
  * @version MBsysLab_s 1.7
  *
  * @author Nicolas Docquier, Allan Barrea
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

#include "mbs_xml_def.h"
#include <string.h>

/**
 * Returns the biggest argument
 */
int mbs_xml_maxInt(int a, int b)
{
    return (a >= b) ? a : b;
}

/**
 * Find the first element corresponding to the xpath expression
 * denoted by elementName.
 * The element is tored in the pointer denoted by first.
 * The number of element matching th expath expressio is returned
 */
int mbs_xml_getElemByXptah(const xmlChar* elementName, xmlDocPtr doc, xmlNodePtr* first)
{
    int size;
    xmlXPathContextPtr xpathCtx;
    xmlXPathObjectPtr xpathObj;

    /* Create xpath evaluation context */
    xpathCtx = xmlXPathNewContext(doc);
    if(xpathCtx == NULL) {
        fprintf(stderr,"Error: unable to create new XPath context\n");
        xmlFreeDoc(doc);
        exit(-1);
    }

    /* Evaluate xpath expression */
    xpathObj = xmlXPathEvalExpression(elementName, xpathCtx);
    if(xpathObj == NULL) {
        fprintf(stderr,"Error using xpath: unable to find element \"%s\"\n", elementName);
        xmlXPathFreeContext(xpathCtx);
        xmlFreeDoc(doc);
        exit(-1);
    }

    size = (xpathObj->nodesetval) ? xpathObj->nodesetval->nodeNr : 0;
    if(size>0){
        *first = xpathObj->nodesetval->nodeTab[0];
    }
    return size;
}

/**
 * Checks the existence of a given element in the XML tree
 */
int mbs_xml_isElement(xmlDocPtr doc, const char* elementName)
{
    xmlNodePtr first;
    int nb;
    nb = mbs_xml_getElemByXptah(BAD_CAST(elementName), doc, &first);

    return nb;
}

/**
 * Walks through the children node of "cur" and try to find an element named "elementName".
 * When elementName is found, its size (int tab[2]) and value (xmlChar*) are extracted.
 * Note: xmlChar *elementValue is dynamically allocated inside this function and must therefore
 * be freed using xmlFree(elementValue) (do not forget !).
 */
xmlChar* mbs_xml_parseElem(const xmlChar* elementName, xmlDocPtr doc, xmlNodePtr cur, int elementSize[2])
{
	xmlChar* elementValue = NULL;
	cur = cur->xmlChildrenNode;

	while (cur != NULL) {
	    if ((!xmlStrcmp(cur->name, (const xmlChar *)elementName)))
	    {
		    elementValue = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1); // element value extraction
		    mbs_xml_getSize(cur, elementSize); // element size extraction and parsing
 	    }
        cur = cur->next;
	}

	return elementValue;
}

/**
 * Searches for the element "elementName" and parses it as an integer.
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
int mbs_xml_parseScalarInt(const char* elementName, xmlDocPtr doc, xmlNodePtr cur)
{
    int scalarToExtract = 0;
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = mbs_xml_parseElem(BAD_CAST(elementName), doc, cur, elementSize);

    if(elementValue != NULL)
    {
        scalarToExtract = atoi((char*)elementValue);
    }
    else
    {
        printf("error: element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);

    return scalarToExtract;
}

/**
 * Searches for the element "elementName" in the children of cur and parses it as a double.
 */
double mbs_xml_parseScalarDouble(const char* elementName, xmlDocPtr doc, xmlNodePtr cur)
{
    double scalarToExtract = 0;
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = mbs_xml_parseElem(BAD_CAST(elementName), doc, cur, elementSize);

    if(elementValue != NULL)
    {
        scalarToExtract = atof((char*)elementValue);
    }
    else
    {
        printf("error: element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);

    return scalarToExtract;
}

/**
 * Searches for the element "elementName" and parses it as a vector of int.
 */
void mbs_xml_parseVectorInt(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, int vectorToExtract[])
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = mbs_xml_parseElem(BAD_CAST(elementName), doc, cur, elementSize);

    if(elementValue != NULL)
    {
        mbs_xml_convertVectorInt(vectorToExtract, (char*)elementValue, elementSize);
    }
    else
    {
        printf("element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);
}

/**
 * Searches for the element "elementName" and parses it as a vector of double
 */
void mbs_xml_parseVectorDouble(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double vectorToExtract[])
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = mbs_xml_parseElem((const xmlChar *)elementName, doc, cur, elementSize);

    if(elementValue != NULL)
    {
        mbs_xml_convertVectorDouble(vectorToExtract, (char*)elementValue, elementSize);
//        printf("test %s\n", (char*)elementName);
    }
    else
    {
        printf("element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);
}

/**
 * Parse cur as a vector of double.
 * WARNING: this function allocate memory. The user has to free it manually using free.
 */
double* mbs_xml_parseElemToVectorDouble(xmlDocPtr doc, xmlNodePtr cur)
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};
    double* vectorToExtract;

    elementValue = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1); // element value extraction
    mbs_xml_getSize(cur, elementSize); // element size extraction and parsing



    if(elementValue != NULL && elementSize[1]>0)
    {
        vectorToExtract = (double*)calloc(elementSize[1]+1, sizeof(double));
        mbs_xml_convertVectorDouble(vectorToExtract, (char*)elementValue, elementSize);
    }
    else
    {
        printf("mbs_xml_parseElemToVectorDouble: unable to parse element '%s' n", cur->name);
    }

    xmlFree(elementValue);

    return vectorToExtract;
}


/**
 * Searches for the element "elementName" and parses it as a matrix of double
 * Note: cur is supposed to be the root element in the case of the mbsdata tree.
 */
void mbs_xml_parseMatrixDouble(const char* elementName, xmlDocPtr doc, xmlNodePtr cur, double** matrixToExtract)
{
    xmlChar *elementValue = NULL;
    int elementSize[2] = {0,0};

    elementValue = mbs_xml_parseElem(BAD_CAST(elementName), doc, cur, elementSize);

    if(elementValue != NULL)
    {
        mbs_xml_convertMatrixDouble(matrixToExtract, (char*)elementValue, elementSize);
    }
    else
    {
        printf("element '%s' not found\n", elementName);
    }

    xmlFree(elementValue);
}


/**
 * Get the "size" attribute of an XML element of the MBSdata structure
 * and parse it into a set of integers.
 */
void mbs_xml_getSize(xmlNodePtr cur, int tab[2])
{
    xmlChar *uri;

    uri = xmlGetProp(cur, (const xmlChar *)"size");
    //printf("size: %s\t", uri);

    // Parse size
    tab[0] = atoi(strtok((char*)uri, " "));
    tab[1] = atoi(strtok(NULL, " "));

    //printf(" --- %d, %d\t", tab[0], tab[1]);
    xmlFree(uri);

	return;
}

/**
 * Walks through the XML tree searching for the element "elementName".
 * When elementName is found, its size (int tab[2]) is extracted and parsed.
 * Finally, the number of columns of the elements (size[1]) is returned.
 */
int mbs_xml_getNbColElement(const char* elementName, xmlDocPtr doc, xmlNodePtr cur)
{
    xmlChar* elementValue;
    int elementSize[2] = {0,0};
    elementValue = mbs_xml_parseElem(BAD_CAST(elementName), doc, cur, elementSize);
    xmlFree(elementValue);
    return elementSize[1];
}

/*
 * Populates vectorToExtract with integer values extracted from the string elementValue
 */
void mbs_xml_convertVectorInt(int* vectorToExtract, char* elementValue, int* elementSize)
{
    int i = 0;
    int vectorSize = mbs_xml_maxInt(elementSize[0], elementSize[1]);

    vectorToExtract[0] = vectorSize;

    // Arrays indexes begin at 1 with the Robotran functions
    vectorToExtract[1] = atoi(strtok(elementValue, " "));

    if(vectorSize > 1)
    {
        for(i=2; i<=vectorSize; i++)
        {
            vectorToExtract[i] = atoi(strtok(NULL, " "));
        }
    }
}

/**
 * Populates vectorToExtract with double values extracted from the string elementValue
 */
void mbs_xml_convertVectorDouble(double* vectorToExtract, char* elementValue, int* elementSize)
{
    int i = 0;
    //char* strToTok;
    int vectorSize = mbs_xml_maxInt(elementSize[0], elementSize[1]);

    vectorToExtract[0] = vectorSize;


    //strToTok = strdup(elementValue);

    // Arrays indexes begin at 1 with the Robotran functions
    vectorToExtract[1] = atof(strtok(elementValue, " "));

    if(vectorSize > 1)
    {
        for(i=2; i<=vectorSize; i++)
        {
            vectorToExtract[i] = atof(strtok(NULL, " "));
        }
    }
    //free(strToTok);
}

/**
 * Populates matrixToExtract with double values extracted from the string elementValue
 */
void mbs_xml_convertMatrixDouble(double** matrixToExtract, char* elementValue, int* elementSize)
{
    int i,j;

    // Note: the external for loop must iterate on j and the internal one on i
    // so that matrix[i][j] receives the elements in the correct order
    // (i.e. so that (i-1) + nb_row*(j-1) = 0, 1, 2, 3, 4, ...)

    // Note: elementSize = (row, col)

    // Arrays indexes begin at 1 with the Robotran functions
    for(j=1; j<=elementSize[1]; j++)
    {
        for(i=1; i<=elementSize[0]; i++)
        {
            if(i == 1 && j == 1)
            {
                matrixToExtract[i][j] = atof(strtok(elementValue, " "));
            }
            else
            {
                matrixToExtract[i][j] = atof(strtok(NULL, " "));
            }
        }
    }

}

double mbs_xml_ParseValue(xmlDocPtr doc, xmlNodePtr cur){
    // inspired from R7parse_value.c by Tony Postiau for Robotran

    xmlChar *node;
    double value=0.0;

    node = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);

    if(node != NULL){
        sscanf((char*)node,"%lf",&value);
        xmlFree(node);
        //value = strtod(node);
    }
    return value;

}

double mbs_xml_ParseInt(xmlDocPtr doc, xmlNodePtr cur){
    // inspired from R7parse_value.c by Tony Postiau for Robotran

    xmlChar *node;
    int value=0;

    node = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);

    if(node != NULL){
        //sscanf(node,"%lf",&value);
        //xmlFree(node);
        value = atoi((char*)node);
    }
    return value;
}
