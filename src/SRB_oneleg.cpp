#include <qpOASES.hpp>
#include <iostream>
#include <chrono> // 시간 측정을 위한 라이브러리
#include <Eigen/Dense>
#include <vector>
#include "ExtendedDataSaver.hpp"

using namespace qpOASES;
using namespace std;
using namespace Eigen;
// mpc
#define PLAN_HORIZON 1
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20
#define NUM_LEG 1
#define NUM_DOF 3

#define Fz_min 0
#define Fz_max INFINITY
#define Ts 0.01
double mu = 0.3;

// double fz_min = 0;
// double fz_max;


double robot_mass = 15.0;
Eigen::Matrix<double, 3,3> a1_trunk_inertia;

Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;
Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;

Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_mat_c;
Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_c;

Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_mat_d;
Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d;

Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;
double cos_yaw = cos(0);
double sin_yaw = sin(0);
Matrix3d r_i;
double dt = 0.001;
VectorXd x0 = VectorXd::Zero(MPC_STATE_DIM,1);
VectorXd x_ref = VectorXd::Zero(MPC_STATE_DIM*PLAN_HORIZON,1);
MatrixXd hessian;
MatrixXd gradient;
int main()
{
    x0[12] = -9.81;
    x0[5] = 1;
    
    x_ref[12] = -9.81;
    x_ref[5] = 1;

    r_i << 0, 0.5, 0, // [0,0,-1] 
        -0.5, 0, 0,
        0, 0, 0; 
    q_weights_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
    q_weights_mpc << 1,1,1,
                    1,1,1,
                    1,1,1,
                    1,1,1, 1;
    a1_trunk_inertia << 0.0158533, 0.0, 0.0,
                0.0, 0.0377999, 0.0,
                0.0, 0.0, 0.0456542;
    // cout <<q_weights_mpc <<endl;
    Q.diagonal() = 2*q_weights_mpc;
    r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);
    r_weights_mpc << 1,1,1;
    R.diagonal() = 2*r_weights_mpc;
    // linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);    
    Eigen::Matrix3d ang_vel_to_rpy_rate;

    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
            -sin_yaw, cos_yaw, 0,
            0, 0, 1;
    A_mat_c.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    A_mat_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat_c(11, 12) = 1;

    B_mat_c.block<3, 3>(6, 0) =
                a1_trunk_inertia.inverse() * r_i;
    B_mat_c.block<3, 3>(9, 0) =
                (1 / robot_mass) * Eigen::Matrix3d::Identity();

    A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A_mat_c*dt;
    B_mat_d = B_mat_c*dt;
        
    // cout << "A_mat = "<<A_mat_d<<endl;
    // cout << "B_mat = "<<B_mat_d<<endl;

    hessian = (B_mat_d.transpose() * Q * B_mat_d);
    hessian += R;

    cout << "hessian = "<<hessian<<endl;
    // cout << "B_mat = "<<B_mat_d<<endl;
    Eigen::Matrix<double, 13*PLAN_HORIZON, 1> tmp_vec = A_mat_d* x0;
    tmp_vec -= x_ref;
    gradient = B_mat_d.transpose() * Q * tmp_vec;


    
}