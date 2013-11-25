#include "nlopt_FC_5r.h"
#include "CoMaN_2d_5R.h"
#include <cstring>
#include <cmath>
#include <armadillo>

double  nlopt_FC_5r( unsigned n, const double *x, double *grad, void *my_func_data ) {
/* % check frìction cones */
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

    double fxe1 = w[0];
    double fye1 = w[1];
    double fxe2 = w[2];
    double fye2 = w[3];

    double fc_wall = -CoMaN_2d_5R::mu2*abs(fxe2) + abs(fye2);
    double fc_floor = -CoMaN_2d_5R::mu1*abs(fye1) + abs(fxe1);
    /*
    % |fye2| <= mu*|fxe2|
    % |fye1| <= mu*|fxe1|
    */
    double c1_vec_[2] = {fc_wall, fc_floor};
    std::vector<double> c1_vec(c1_vec_,c1_vec_+sizeof(c1_vec_)/sizeof(c1_vec_[0]));
    double c1 = *(std::max_element(c1_vec.begin(),c1_vec.end())); // c1 must be <= 0
    return c1;
}
