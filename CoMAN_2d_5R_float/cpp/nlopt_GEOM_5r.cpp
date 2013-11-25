#include "nlopt_GEOM_5r.h"
#include "CoMaN_2d_5R.h"
#include <cstring>

double  nlopt_GEOM_5r( unsigned n, const double *x, double *grad, void *my_func_data ) {
    CoMaN_2d_5R* coman = (CoMaN_2d_5R*)my_func_data;

    c_vector<double,NJOINTS> q;
    /* we copy just the values which must stay in non-zero bounds */
    memcpy(q.data()+2, x, (NJOINTS-2)*sizeof(double));
    q[0] = 0.0; q[1] = 0.0;

    matrix<double> fk = coman->fkine(q);

    double y_e1 = fk(1,2);
    double y_e2 = fk(1,2+3);
    double y_j1 = fk(1,2+6);
    double y_j2 = fk(1,2+9);
    double y_j3 = fk(1,2+12);
    double y_j4 = fk(1,2+15);
    double y_j5 = fk(1,2+18);
    double x_e1 = fk(0,2);
    double x_e2 = fk(0,2+3);
    double x_j1 = fk(0,2+6);
    double x_j2 = fk(0,2+9);
    double x_j3 = fk(0,2+12);
    double x_j4 = fk(0,2+15);
    double x_j5 = fk(0,2+18);

    /*
        bounds on position of end effectors
        x_e2 > x_e1
        y_e2 > y_e1
        y_e1 at bottom
        x_e2 at right
        // double c3 = max([max([x_e1, x_j1, x_j2, x_j3, x_j4, x_j5]);y_e1]-[x_e2;min([y_e2, y_j1, y_j2, y_j3, y_j4])]); % bounds on ee position
    */

    double x_fk_[6] = {x_e1, x_j1, x_j2, x_j3, x_j4, x_j5};
    std::vector<double> x_fk(x_fk_,x_fk_+sizeof(x_fk_)/sizeof(x_fk_[0]));
    double x_max = *(std::max_element(x_fk.begin(),x_fk.end()));
    double y_fk_[6] = {y_e2, y_j1, y_j2, y_j3, y_j4};
    std::vector<double> y_fk(y_fk_,y_fk_+sizeof(y_fk_)/sizeof(y_fk_[0]));
    double y_min = *(std::min_element(y_fk.begin(),y_fk.end()));
    double c3_[2] = {x_max - x_e2, y_e1 - y_min};
    std::vector<double> c3(c3_,c3_+sizeof(c3_)/sizeof(c3_[0]));
    double c3_max = *(std::max_element(c3.begin(),c3.end()));

    return c3_max;
}
