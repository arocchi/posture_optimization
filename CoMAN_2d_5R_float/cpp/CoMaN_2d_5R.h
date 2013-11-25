#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <cstdio>
/**
  @TODO Add get_sensor_by_id(..) // we do it by ourselves
  @TODO Add get_part_by_name(..) // what does iCub do?
  @TODO Add the code for the OpenRave visualizer
  */
extern "C" {
    #include "MBSfun.h"
    #include "MBSdef.h"
}
#include "JNI_MBSysPad.h"

#define NJOINTS 8

#define S_ID_RFoots     15
#define S_ID_RWrJ       43
#define S_ID_RShSagJ    34
#define S_ID_RElbJ      41
#define S_ID_DWYJ       30
#define S_ID_RKneeJ     10
#define S_ID_RAnkJ      12

#define J_ID_X      1
#define J_ID_Z      3
#define J_ID_THETA  5

#define J_ID_SHOULDER   22
#define J_ID_ELBOW      25
#define J_ID_WAIST      20
#define J_ID_HIP        7
#define J_ID_KNEE       10
#define J_ID_ANKLE      12

const int PLANAR_JOINTS[] = { J_ID_X, J_ID_Z, J_ID_THETA, \
                            J_ID_SHOULDER, J_ID_ELBOW, J_ID_HIP, \
                            J_ID_KNEE, J_ID_ANKLE};

using namespace boost::numeric::ublas;


class CoMaN_2d_5R {
    JNI_MBSysPad *plotObj;
    MBSdataStruct *CoMaN;
    MBSsensorStruct e1;
    MBSsensorStruct e2;
    MBSsensorStruct j1;
    MBSsensorStruct j2;
    MBSsensorStruct j3;
    MBSsensorStruct j4;
    MBSsensorStruct j5;
public:
    CoMaN_2d_5R();
    ~CoMaN_2d_5R();

    static const double mu1;
    static const double mu2;
    static const zero_vector<double> q0;
    c_vector<double,NJOINTS> gravload(c_vector<double,NJOINTS> q);
    matrix<double> jacob0(c_vector<double,NJOINTS> q);
    matrix<double> fkine(c_vector<double,NJOINTS> q);

    void update_sensors(c_vector<double,NJOINTS> q);
    void update_sensors(c_vector<double,NJOINTS> q, c_vector<double,NJOINTS> qd);
    void plot(c_vector<double,NJOINTS> q);
};

