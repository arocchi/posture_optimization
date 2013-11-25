#include "CoMaN_2d_5R.h"
#include <libxml/parser.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <iostream>

namespace fs = boost::filesystem;

const zero_vector<double> CoMaN_2d_5R::q0(NJOINTS);
const double CoMaN_2d_5R::mu1(1.4);
const double CoMaN_2d_5R::mu2(1.4);

CoMaN_2d_5R::CoMaN_2d_5R()  : CoMaN(NULL) {

    fs::path full_path( fs::current_path() );
    full_path = full_path/ "robot" / "robot.mbsdata";
    xmlDocPtr CoMaN_xml = xmlReadFile(full_path.c_str(),NULL,0);
    this->CoMaN = loadMBSdata_xml(CoMaN_xml);
    xmlFreeDoc(CoMaN_xml);

    for(int i = 1; i <= CoMaN->njoint; ++i) {
        CoMaN->q[i] = 0.0;
        CoMaN->qd[i] = 0.0;
    }

    allocate_sensor(&this->e1,29);
    allocate_sensor(&this->e2,29);
    allocate_sensor(&this->j1,29);
    allocate_sensor(&this->j2,29);
    allocate_sensor(&this->j3,29);
    allocate_sensor(&this->j4,29);
    allocate_sensor(&this->j5,29);
    init_sensor(&this->e1,29);
    init_sensor(&this->e2,29);
    init_sensor(&this->j1,29);
    init_sensor(&this->j2,29);
    init_sensor(&this->j3,29);
    init_sensor(&this->j4,29);
    init_sensor(&this->j5,29);
}

CoMaN_2d_5R::~CoMaN_2d_5R() {
    this->plotObj = NULL;
    free_sensor(&this->e1);
    free_sensor(&this->e2);
    free_sensor(&this->j1);
    free_sensor(&this->j2);
    free_sensor(&this->j3);
    free_sensor(&this->j4);
    free_sensor(&this->j5);
    freeMBSdata(this->CoMaN);   this->CoMaN = NULL;
    if(this->plotObj != NULL) free(this->plotObj); this->plotObj = NULL;
}

/* loads of trouble here.. dirdyna is evil */
c_vector<double,NJOINTS> CoMaN_2d_5R::gravload(c_vector<double,NJOINTS> q) {
    c_vector<double,NJOINTS> g;
    /* setting velocity to 0 we get g(q) */
    this->update_sensors(q,q0);

    double c[29+1];
    double* M[29+1];
    for(int i = 0; i < (29+1); ++i) {
        M[i] = (double*)malloc((29+1)*sizeof(double));
    }
    dirdyna((double**)M,c,this->CoMaN,0.0);
    for(int i = 0; i < NJOINTS; ++i) {
        g[i] = c[PLANAR_JOINTS[i]];
    }
    for(int i = 0; i < (29+1); ++i) {
        free(M[i]);
        M[i] = NULL;
    }
    return g;
}

matrix<double> CoMaN_2d_5R::jacob0(c_vector<double, NJOINTS> q) {
    matrix<double> J0(6,NJOINTS);
    this->update_sensors(q);

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < NJOINTS; ++j) {
            /*we need to get row 1,3,5 */
            J0(i,j) = this->e1.J[i*2 +1][PLANAR_JOINTS[j]];
            J0(i+3,j) = this->e2.J[i*2 +1][PLANAR_JOINTS[j]];
        }
    }
    return J0;
}

matrix<double> CoMaN_2d_5R::fkine(c_vector<double, NJOINTS> q) {
    matrix<double> fk(3,21);
    this->update_sensors(q);

    /* we compute 7 fks in one go */
    for(int k = 0; k<7; ++k) {
        MBSsensorStruct* sens;
        switch(k) {
            case 0:
                sens = &this->e1;
                break;
            case 1:
                sens = &this->e2;
                break;
            case 2:
                sens = &this->j1;
                break;
            case 3:
                sens = &this->j2;
                break;
            case 4:
                sens = &this->j3;
                break;
            case 5:
                sens = &this->j4;
                break;
            case 6:
                sens = &this->j5;
                break;
            default:
                break;
        }

        for(int i = 0; i < 2; ++i) {
            for(int j = 0; j < 2; ++j) {
                /* we need to get row q,3 and column 1,3 from R*/
                fk(i,j+k*3) =   sens->R[i*2 + 1][j*2 + 1];
            }
            fk(i,2+k*3) = sens->P[i+1];
        }
        fk(2,0+k*3) = 0.0; fk(2,1+k*3) = 0.0; fk(2,2+k*3) = 1.0;
    }
    return fk;
}



void CoMaN_2d_5R::update_sensors(c_vector<double,NJOINTS> q, c_vector<double,NJOINTS> qd) {
    for(int i = 0; i < NJOINTS; ++i) {
        this->CoMaN->qd[PLANAR_JOINTS[i]] = qd[i];
    }
    this->update_sensors(q);
}

void CoMaN_2d_5R::update_sensors(c_vector<double,NJOINTS> q) {
    for(int i = 0; i < NJOINTS; ++i) {
        this->CoMaN->q[PLANAR_JOINTS[i]] = q[i];
    }
    sensor(&this->e1,this->CoMaN,S_ID_RFoots);
    sensor(&this->e2,this->CoMaN,S_ID_RWrJ);
    sensor(&this->j1,this->CoMaN,S_ID_RShSagJ);
    sensor(&this->j2,this->CoMaN,S_ID_RElbJ);
    sensor(&this->j3,this->CoMaN,S_ID_DWYJ);
    sensor(&this->j4,this->CoMaN,S_ID_RKneeJ);
    sensor(&this->j5,this->CoMaN,S_ID_RAnkJ);
}




void CoMaN_2d_5R::plot(c_vector<double,NJOINTS> q) {
    std::cout << "Creating new JNI_MBSysPad object" << std::endl;
    if(this->plotObj==NULL)
        this->plotObj = new JNI_MBSysPad();

    sleep(15);
    std::cout << "Calling JNI_MBSysPad.plot" << std::endl;
    this->plotObj->plot(q.size(),&q[0]);
    return;
}
