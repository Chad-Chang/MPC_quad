#include <qpOASES.hpp>
#include <iostream>
#include <chrono> // 시간 측정을 위한 라이브러리
#include <Eigen/Dense>
#include <vector>
#include "ExtendedDataSaver.hpp"

using namespace qpOASES;
using namespace std;
using namespace Eigen;

#define NUMOFX 13
#define NUMOFU 3
#define NUMOFLEG 1
#define HORIZON_T 1
#define Fz_min 0
#define Fz_max INFINITY
#define Ts 0.01

double psi = 0;
double m = 100;
double fric_coef = 1;

double t = 0 ;

/*x : theta_x, theta_y, theta_z, 
    p_x, p_y, p_z, 
    dtheta_x,dtheta_y,dtheta_z,
    dp_x, dp_y, dp_z,
    g*/

ExtendedDataSaver dataSaver("../data/extended_output.csv");
ExtendedDataSaver dataSaver_cost("../data/cost.csv");
ExtendedDataSaver dataSaver_state("../data/state.csv");

VectorXd x0 = VectorXd::Zero(NUMOFX,1);
VectorXd x_ref = VectorXd::Zero(NUMOFX*HORIZON_T,1);
Matrix3d r_i;

MatrixXd A_c = MatrixXd::Zero(NUMOFX, NUMOFX); 
MatrixXd A_d = MatrixXd::Zero(NUMOFX,NUMOFX);
Matrix3d Inertia = Matrix3d::Identity()*0.1;
Matrix3d R_toEuler;
MatrixXd B_c = MatrixXd::Zero(NUMOFX, NUMOFU); 
MatrixXd B_d = MatrixXd::Zero(NUMOFX,NUMOFU);

// MatrixXd::Zero(3,3);

Matrix3d Zeros33 = Matrix3d::Zero();
Vector3d Zeros31 = Vector3d::Zero();
Vector3d Zeros13 = Vector3d::Zero().transpose();
Vector3d Onez31 = Vector3d::Zero();
Matrix3d Eye33 = Matrix3d::Zero();
VectorXd u_k = VectorXd::Zero(NUMOFU,1);

int main() 
{
    Onez31[2] = 1;
    x0[2] = psi;
    x0[11] = 0; // position z
    x0[12] = -9.81; // g
    
    x_ref[11] = 1; // position z
    x_ref[12] = -9.81; // position z

    //leg skew symmetric -> this is from base to legs
    r_i << 0, 0.5, 0, // [0,0,-1] 
            -0.5, 0, 0,
            0, 0, 0; 
        // // // // QP 솔버 초기화
    QProblem qp(NUMOFU*HORIZON_T,0); // u dimension & constraints
        // QProblem qp(NUMOFU*HORIZON_T, 4*NUMOFLEG); // u dimension & constraints
        qp.reset();
        Options options;
        options.epsIterRef = 1e-8;
        qp.setOptions(options);
    auto start_time = std::chrono::high_resolution_clock::now();

    real_t xOpt[NUMOFU*HORIZON_T];
    while(t <1000)
    {    // timer start
            
        t++;
        R_toEuler << cos(x0[2]), -sin(x0[2]), 0, 
                    sin(x0[2]), cos(x0[2]), 0, 
                    0, 0, 1;
        A_c.block<3,3>(0,6) = R_toEuler;
        A_c.block<3,3>(3,9) = Matrix3d::Identity();
        A_c(11,12) = 1;
        // cout << "A_c = " << A_c<<endl;
        B_c.block<3,3>(6,0) = Inertia.inverse()*r_i;
        B_c.block<3,3>(9,0) = Matrix3d::Identity()/m;
        
        A_d = MatrixXd::Identity(NUMOFX,NUMOFX)+A_c*Ts;
        B_d = B_c*Ts;
        cout <<"B = " <<  B_d<<endl;
        x0 = A_d * x0 + B_d*u_k;
 
        MatrixXd Q = 1000*MatrixXd::Identity(NUMOFX*HORIZON_T,NUMOFX*HORIZON_T); // reference tracking weight
        // Q(11,11) = 1
    
        MatrixXd R = 0.0001*MatrixXd::Identity(NUMOFU*HORIZON_T,NUMOFU*HORIZON_T); // input weight
    //     R(2,2) = 0.0000001;
        MatrixXd H_eigen = 2 * (B_d.transpose()*Q*B_d+R);
        VectorXd error = A_d * x0 - x_ref;
        MatrixXd G_eigen = 2 * B_d.transpose()*Q*error;
        cout <<" H = "<<H_eigen<<endl;
        cout <<" G = "<<G_eigen<<endl;
        cout << "u = " << u_k<<endl;
        
        real_t* H = H_eigen.data();
        real_t* g = G_eigen.data(); // "column major order" 

        MatrixXd A_eigen = MatrixXd::Zero(4*NUMOFLEG,3*NUMOFLEG);
        for(int i = 0; i < NUMOFLEG; i++)
            A_eigen.block(NUMOFLEG*i,NUMOFU*i,NUMOFLEG,NUMOFU) 
                            << 1,0, fric_coef;
                                -1, 0, fric_coef,
                                0, 1, fric_coef,
                                0, -1, fric_coef; 

        real_t* A = A_eigen.data();// constraint matrix


        real_t lb[3*NUMOFLEG] 
                        = {-1e20, -1e20, Fz_min};
        real_t ub[3*NUMOFLEG] 
                        = {1e20, 1e20, Fz_max}; 
        real_t lbA[4*NUMOFLEG] 
                        = {-1e20, -1e20, -1e20, -1e20}; // constraint bound 
        real_t ubA[4*NUMOFLEG]
                        = {1e20, 1e20, 1e20, 1e20};

        int nWSR = 1000;
        qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

        qp.getPrimalSolution(xOpt);

        
        for(int i = 0 ; i< NUMOFU ; i++)
        {
            u_k[i] = xOpt[i];
        }
        dataSaver_cost.writeRow(u_k.transpose()*H_eigen*u_k+u_k.transpose()*G_eigen);
        dataSaver.writeRow(u_k);
        dataSaver_state.writeRow(x0);
    }
    //     r_i = r_i * x0[5];

    //     cout <<"cost = "<< u_k.transpose()*H_eigen/2*u_k +u_k.transpose()*G_eigen <<endl;
    //     cout << "H_eigen = "<< H_eigen<<endl;
    //     cout << "G_eigen = "<< G_eigen<<endl;
    //     cout << "x0 = "<< x0<<endl;
    //     cout << "uk = "<< u_k<<endl;
    //     cout << "state z = "<< x0[5]<<endl;

    //     // cout << " cost = "<< u_k.transpose()*H_eigen*u_k+G_eigen.transpose()*u_k<<endl;
        

    //     // // 최적화 종료 시간 기록
    //     auto end_time = std::chrono::high_resolution_clock::now();
        
    //     // // 최적화 시간 계산
    //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    //     // std::cout << "Optimization time: " << duration.count() << " microseconds" << std::endl;

    // }
    // dataSaver.closeFile();
    return 0;
}