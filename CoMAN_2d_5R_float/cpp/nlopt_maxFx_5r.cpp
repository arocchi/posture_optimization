#include <iostream>
#include <cstring>
#include "nlopt_maxFx_5r.h"
#include "CoMaN_2d_5R.h"
#include <armadillo>

/**
  * direct_RRR_q_maxFx maximizes force along the x direction
  */
double  nlopt_maxFx_5r( unsigned n, const double *x, double *grad, void *my_func_data ) {
    CoMaN_2d_5R* coman = (CoMaN_2d_5R*)my_func_data;

    c_vector<double,NJOINTS> q;
    /* we copy just the values which must stay in non-zero bounds */
    memcpy(q.data()+2, x, (NJOINTS-2)*sizeof(double));
    q[0] = 0.0; q[1] = 0.0;

    /* we copy all taus (x+NJOINTS-2) PLUS three q values which will get discarded*/
    arma::vec tau = arma::vec::fixed<NJOINTS>((const double*)(x+NJOINTS-5));
    tau[0] = 0.0; tau[1] = 0.0; tau[2] = 0.0;

    matrix<double> J_ublas = coman->jacob0(q);
    /* ublas is row-major by default, armadillo is column-major */
    arma::mat Jt = arma::mat::fixed<NJOINTS,3>((const double*)&((J_ublas.data())[0]));
    c_vector<double,NJOINTS> g_ublas = coman->gravload(q);
    arma::vec g = arma::vec::fixed<NJOINTS>((const double*)&((g_ublas.data())[0]));

    arma::mat pJt = arma::pinv(Jt);
    arma::vec w = pJt*(tau-g);

    double we2_x = -w[2];
    return we2_x;
}

