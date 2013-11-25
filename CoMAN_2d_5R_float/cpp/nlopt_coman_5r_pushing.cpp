#include <nlopt.hpp>
#include <cmath>
#include <boost/numeric/ublas/io.hpp>
#include <iostream>
#include "nlopt_maxFx_5r.h"
#include "nlopt_FC_5r.h"
#include "nlopt_GEOM_5r.h"
#include "nlopt_FLATFOOT_5r.h"
#include "CoMaN_2d_5R.h"
#include <ctime>
#include <armadillo>

/* first NJOINTS-2 are q limits, then NJOINTS-3 tau limits */
double ub_[2*NJOINTS-5] = {M_PI,  1.66, 0.0,   0.79,  1.92, 0.87, \
                           30,   30,    30,    30,   30};
double lb_[2*NJOINTS-5] = {-M_PI, -3.4, -2.36, -1.92, -0.17,-1.22, \
                           -30,  -30,   -30,   -30,  -30};

int main(int argc, char **argv) {
    std::cout << "Loading CoMaN" << std::endl;
    CoMaN_2d_5R coman;
/*
    std::cout << "Testing Jacobian" << std::endl;
    matrix<double> J = coman.jacob0(zero_vector<double>(NJOINTS));
    std::cout << J << std::endl;

    std::cout << "Testing ublas to armadillo" << std::endl;
    arma::mat Jt = arma::mat::fixed<NJOINTS,3>((const double*)&((J.data())[0]));
    std::cout << Jt << std::endl;

    std::cout << "Testing FK" << std::endl;
    matrix<double> fk = coman.fkine(zero_vector<double>(NJOINTS));
    std::cout << fk << std::endl;

    std::cout << "Testing gravload" << std::endl;
    c_vector<double,NJOINTS> gl = coman.gravload(zero_vector<double>(NJOINTS));
    std::cout << gl << std::endl;

    std::cout << "Testing functions" << std::endl;
    double zeros[2*NJOINTS-5] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0};
    std::cout << nlopt_maxFx_5r(2*NJOINTS-5,zeros,NULL,(void*)&coman) << std::endl;
    std::cout << nlopt_FC_5r(2*NJOINTS-5,zeros,NULL,(void*)&coman) << std::endl;
    std::cout << nlopt_GEOM_5r(2*NJOINTS-5,zeros,NULL,(void*)&coman) << std::endl;
    std::cout << nlopt_FLATFOOT_5r(2*NJOINTS-5,zeros,NULL,(void*)&coman) << std::endl;
*/

    std::vector<double> lb(lb_,lb_+sizeof(lb_)/sizeof(lb_[0]));
    std::vector<double> ub(ub_,ub_+sizeof(ub_)/sizeof(ub_[0]));

    std::cout << "Creating Optimizer" << std::endl;
    nlopt::opt local_optimizer = nlopt::opt(nlopt::LN_COBYLA,2*NJOINTS-5);
    local_optimizer.set_maxeval(50000);
    nlopt::opt optimizer = nlopt::opt(nlopt::GN_ISRES,2*NJOINTS-5);
    optimizer.set_xtol_rel(1E-4);
    optimizer.set_maxeval(50000);
   // optimizer.set_local_optimizer(local_optimizer);


    optimizer.set_min_objective(nlopt_maxFx_5r,(void*)&coman);
    optimizer.add_inequality_constraint(nlopt_FC_5r,(void*)&coman,1E-4);
    optimizer.add_inequality_constraint(nlopt_GEOM_5r,(void*)&coman,1E-4);
    optimizer.add_inequality_constraint(nlopt_FLATFOOT_5r,(void*)&coman,1E-3);

    optimizer.set_lower_bounds(lb);
    optimizer.set_upper_bounds(ub);
    std::vector<double> x(2*NJOINTS -5);
    double fmin;
    std::cout << "Starting Optimization" << std::endl;
    long unsigned int t0=clock(),t1;
    nlopt::result res = optimizer.optimize(x,fmin);
    t1=clock()-t0;

    std::cout << "Return code is: ";
    switch(res) {
        case nlopt::SUCCESS:
            std::cout << "Success";
            break;
        case nlopt::STOPVAL_REACHED :
            std::cout << "stopval reached";
            break;
        case nlopt::FTOL_REACHED:
            std::cout << "ftol_rel or ftol_abs reached";
            break;
        case nlopt::XTOL_REACHED:
            std::cout << "xtol_rel or xtol_abs reached";
            break;
        case nlopt::MAXEVAL_REACHED:
            std::cout << "maximum number of evaluations reached";
            break;
        case nlopt::MAXTIME_REACHED:
            std::cout << "maximum time for computation reached";
            break;
    }
    std::cout << std::endl;

    std::cout << "found minimum at f( ";
    for(int i = 0; i < x.size(); ++i) {
        std::cout << x[i] << " ";
    }
    std::cout << ") = " << fmin << std::endl;
    std::cout << std::scientific << "in: " <<((double)t1/CLOCKS_PER_SEC) * 1000.0 << " [ms]" <<std::endl;
/*
    c_vector<double,NJOINTS> q;
    q[0] = 0.0; q[1] = 0.0;
    for(int i = 0; i < NJOINTS-2;++i) q[i]=x[i];

    std::cout << "Invoking coman.plot(q)" << std::endl;
    // coman.plot is not working and core dumps
    coman.plot(q);
*/
	return 1;
}
