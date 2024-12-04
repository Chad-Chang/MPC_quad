#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "globalVariable.h"
#include <qpOASES.hpp>
#include <iostream>
#include <vector>

using namespace qpOASES;
using namespace std;
using namespace Eigen;

#define NUMOFX 12
#define NUMOFU 12
#define NUMOFLEG 4
#define HORIZON_T 1
#define Fz_min -1e20
#define Fz_max 1e20
#define fric_coef 0.6

class Optimizer
{
private:
    // TODO: work space is cartesian coordinate -> this might be modified in some way
    
    // *  state : 12 [theta, p,dtheta, dp] 
    // * theta_x, theta_y, theta_z, 
    // * p_x, p_y, p_z, 
    // * dtheta_x,dtheta_y,dtheta_z,
    // * dp_x, dp_y, dp_z,

    // * base coordinate 

    VectorXd x0_; //initial state
    VectorXd x_ref_; // reference state
    VectorXd gravity_; // gravity vector

public:
    Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base);
    ~Optimizer();
    StateModel_* FL_ptr_; StateModel_* FR_ptr_; StateModel_* RL_ptr_; StateModel_* RR_ptr_;
    TrunkModel_* Trunk_ptr_;
    
};
#endif