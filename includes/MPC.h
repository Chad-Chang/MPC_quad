#ifndef MPC_H_
#define MPC_H_

#include "globalVariable.h"

class Kinematics;
using namespace std;

class MPC
{
private:
    integer state_size = 12;
    integer input_size = 12;
    integer measure_size = 12;
    double m_trunk = 43;

    double Ts; // 제어 주기
    int horizon_time ; // 예측 시간
    double f;
    double f1;
    double f2;

    // State 
    Vector x_ROPT;
    Vector xk_ROPT; 
    Vector xd_ROPT;
    Vector ek_ROPT;
    
    // State space equation
    Vector Ad_ROPT;
    Vector Bd_ROPT;
    Vector Cd_ROPT;
    Vector F_ROPT;
    Vector PHI_ROPT;


    // Inertia
    Vector I_W_ROPT;  // inertia matrix of trunk in World frame
    Vector I_W_inv_ROPT;
  

    // Body to foot vector
    Vector r_skew_FL;
    Vector r_skew_FR;
    Vector r_skew_RL;
    Vector r_skew_RR;

    // Cost

    double cost;
    double cost_u;
    double cost_e;
    VectorXd df;


    
    Matrix3d Zero;
    Matrix3d Identity;
    Matrix3d R_trunk; // from Body frame to World frame
    Matrix3d r_skew[4];
    
    // State space //
    VectorXd xk;
    MatrixXd A;
    MatrixXd Ad;
    MatrixXd B;
    MatrixXd Bd;
    MatrixXd C;
    MatrixXd Cd;
    VectorXd g;

    Matrix3d I_B;
    Matrix3d I_W;
    Matrix3d I_W_inv;
    Matrix3d I_W_inv_r_skew[4];
    //get_I_W_inv
    
    // MPC Formulation
    VectorXd y;
    MatrixXd F;
    MatrixXd phi;
    MatrixXd Fxk;
    VectorXd uk;
    VectorXd ek;
    MatrixXd phiuk;
    MatrixXd Lp;

    // Desired xref //
    VectorXd Desired_state;
    VectorXd xref;

    // Weighting Matrix //
    MatrixXd Q;
    MatrixXd R;
    

public:
    MPC(double Ts, int horizon_time);
    ~MPC();
    void Cal_Parameter(Kinematics K_FL, Kinematics K_FR, Kinematics K_RL, Kinematics K_RR);
    VectorXd getDesiredState(double t);
    realdp Cost_function(double t);
    void updateState(double t); 
    // VectorXd computeControlInput();
    VectorXd get_df() {return df;}
    MatrixXd get_Q() {return Q;}
    MatrixXd get_R() {return R;}
    VectorXd get_ek() {return ek;}
    MatrixXd get_F() {return F;}
    MatrixXd get_phi() {return phi;}
    VectorXd get_xk() {return xk;}
    VectorXd get_xref() {return xref;}
    MatrixXd get_Lp() {return Lp;}

    int get_horizon_time() {return horizon_time;}
    int get_state_size() {return state_size;}
    
};

#endif