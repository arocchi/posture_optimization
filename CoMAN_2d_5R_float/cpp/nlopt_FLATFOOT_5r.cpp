#include "nlopt_FLATFOOT_5r.h"
#include "CoMaN_2d_5R.h"
#include <cmath>

double  nlopt_FLATFOOT_5r( unsigned n, const double *x, double *grad, void *my_func_data ) {
/* bounds on position of end effectors
   % theta_y + q3 + q4 + q5 = 0 */
double flat_foot = x[0]+x[3]+x[4]+x[5];
return abs(flat_foot);
}
