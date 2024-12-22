#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "globalVariable.h"
#include <qpOASES.hpp>
#include <iostream>
#include <vector>
// #inlcude <chrono>
using namespace qpOASES;
using namespace std;
using namespace Eigen;

#define NUMOFX 13
#define NUMOFU 12
#define NUMOFLEG 4
#define HORIZON_T 1
#define Fz_min 0
#define Fz_max 1e20
#define fric_coef 0.6

class Optimizer
{
public:
    Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base);
    ~Optimizer();

    VectorXd MPC_SRB();
    void reset();
    void update_state(); // parameterupdate
    void calculate_Ac();
    void calculate_Bc(); 
    void discretization(double dt); // euler discretization
    void solve_qp();

private:
    // TODO:  have to make vector container to accomodate state & velocity
    // TODO: body inertia, world rotation transform, euler angle, absolute 

    // *  state : 13 [theta, p,dtheta, dp] 
    // * theta_x, theta_y, theta_z, 
    // * p_x, p_y, p_z, 
    // * dtheta_x,dtheta_y,dtheta_z,
    // * dp_x, dp_y, dp_z,
    // * g

    // * base coordinate 
    double mu_; // friction coef
    double t;
    double robot_mass_;
    VectorXd state_; 
    VectorXd state_ref_; 

    VectorXd ctrl_input_;

    // VecotrXd ; 
    MatrixXd Ac_; MatrixXd Ad_; MatrixXd Aqp_;
    MatrixXd Bc_; MatrixXd Bd_; MatrixXd Bqp_;
    MatrixXd Bd_list_;
    
    MatrixXd hessian_; // qp form     

    MatrixXd gradient_; // qp form matrix2
    
    MatrixXd Q_; // weighting matrix - state
    MatrixXd R_; // weighting matrix - input

    MatrixXd constraint_; // constraint matrix :lb < Ax < ub
    real_t* constraint_qp_; // constraint matrix :lb < Ax < ub
    
    int nWSR_ = 100; // interation limit

    Matrix3d inertia_body_; Matrix3d inertia_world_;
    
    double running_time_; // MPC running time
    
    // *leg model pointer
    // StateModel_* FL_ptr_; StateModel_* FR_ptr_; StateModel_* RL_ptr_; StateModel_* RR_ptr_;
    StateModel_* leg_ptr_[4];

    // * trunk model pointer 
    TrunkModel_* Trunk_ptr_;  // state , state_ref, base2leg contains


};
#endif