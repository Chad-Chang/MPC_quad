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
    double t;
    
    VectorXd g_; // gravity vector
    // VectorXd xi_;
    std::vector<VectorXd> xi_;
    std::vector<VectorXd> ui_;
    VectorXd x0_;
    // VecotrXd ; 
    Matrix3d R_toEuler_; // euler matrix
    MatrixXd Ad_; MatrixXd A_cond_; MatrixXd Aqp_; 
    MatrixXd Bd_; MatrixXd B_cond_; MatrixXd Bqp_; 
    
    MatrixXd H_; // qp form matrix1 
    real_t* Hqp_; // qp form matrix1 

    MatrixXd G_; // qp form matrix2
    real_t* Gqp_; // qp form matrix1 

    MatrixXd L_; // weighting matrix - state
    MatrixXd K_; // weighting matrix - input

    MatrixXd P_; // constraint matrix :lb < Ax < ub
    real_t* Pqp_; // constraint matrix :lb < Ax < ub
    
    int nWSR_ = 100; // interation limit

    Matrix3d Inertia_;
    

    double running_time_; // MPC running time
    
    StateModel_* FL_ptr_; StateModel_* FR_ptr_; StateModel_* RL_ptr_; StateModel_* RR_ptr_;
    TrunkModel_* Trunk_ptr_;  // r_i involved

public:

    Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base);
    ~Optimizer();
    std::vector<VectorXd> MPC_SRB(VectorXd x0, VectorXd x_ref, Matrix3d inertia);

};
#endif