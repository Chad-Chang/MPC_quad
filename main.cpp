#include <qpOASES.hpp>
#include <iostream>
#include <chrono> // 시간 측정을 위한 라이브러리
#include <Eigen/Dense>
#include <vector>

using namespace qpOASES;
using namespace std;
using namespace Eigen;

#define NUMOFX 13
#define NUMOFU 12
#define NUMOFLEG 4
#define HORIZON_T 1
#define Fz_min 0.001
#define Fz_max 100
#define Ts 0.002

double psi = 0;
double m = 10;
double fric_coef = 0.6;
double gravity = -9.81;

Matrix3d R_toEuler;
MatrixXd A_c = MatrixXd::Zero(NUMOFX, NUMOFX); 
MatrixXd A_d = MatrixXd::Zero(NUMOFX,NUMOFX);

MatrixXd A_qp = MatrixXd::Zero(0, NUMOFX);
MatrixXd A_buf = MatrixXd::Identity(NUMOFX,NUMOFX);

Matrix3d Inertia = Matrix3d::Identity();
std::vector<Matrix3d> r_i(NUMOFLEG);

MatrixXd B_c = MatrixXd::Zero(NUMOFX, NUMOFU); 
MatrixXd B_d = MatrixXd::Zero(NUMOFX,NUMOFU);

MatrixXd B_qp = MatrixXd::Zero(NUMOFX*HORIZON_T, NUMOFU*HORIZON_T);
MatrixXd B_buf = MatrixXd::Identity(NUMOFX, NUMOFU*HORIZON_T);

VectorXd x0 = VectorXd::Zero(NUMOFX,1);
VectorXd x_ref = VectorXd::Zero(NUMOFX*HORIZON_T,1);

/*x : theta_x, theta_y, theta_z, 
    p_x, p_y, p_z, 
    dtheta_x,dtheta_y,dtheta_z,
    dp_x, dp_y, dp_z,
    g*/
int main() {
    
    x0[2] = psi;
    x0[5] = 1; // position z
    x0[12]= gravity; // 
    
    x_ref[5] = 1; // position z
    x_ref[12] = gravity;

    //leg skew symmetric
    r_i[0] << 0, 0, -1, // [1,-1,0] : FL
            0, 0, -1,
            1, 1, 0; 
    r_i[1] << 0, 0, 1, // [1,1,0] : FR
            0, 0, -1,
            -1, 1, 0; 
    r_i[2] << 0, 0, -1, // [-1,-1,0] : RL
            0, 0, 1,
            1, -1, 0;
    r_i[3] << 0, 0, 1, // [-1,1,0] : RR
            0, 0, -1,
            -1, 1, 0; 

    // timer start
    auto start_time = std::chrono::high_resolution_clock::now();

    R_toEuler << cos(psi), sin(psi), 0, 
                -sin(psi), cos(psi), 0, 
                0, 0, 1;

    A_c.block(0,6,3,3) = R_toEuler;
    A_c.block(3,9,3,3) = Matrix3d::Identity();
    A_c(8,12) = 1;// for the gravity
    A_d = MatrixXd::Identity(NUMOFX,NUMOFX) + Ts*A_c;
    B_c.block(6,0, 3, NUMOFU) << Inertia.inverse()*r_i[0], Inertia.inverse()*r_i[1]
                            , Inertia.inverse()*r_i[2], Inertia.inverse()*r_i[3];
    B_c.block(9,0,3,NUMOFU) << Matrix3d::Identity()/m, Matrix3d::Identity()/m,
                            Matrix3d::Identity()/m, Matrix3d::Identity()/m;    
    B_d = Ts * B_c;// discrete time


    // considering horizon time

    // for(int i = 0 ; i < HORIZON_T; i++)//
    // {
    //     for(int j = 0 ; j<=i; ++j)
    //     {
    //         MatrixXd powerA = Eigen::MatrixXd::Identity(NUMOFX, NUMOFX); // 13 x 13
    //         for(int k = 0 ; k<i-j;++k)
    //         {
    //             powerA*= A_d;
    //         }
    //         B_qp.block(i*NUMOFX, j*NUMOFU, NUMOFX, NUMOFU) = powerA*B_d;
    //         if(j == 0)
    //         {    
    //             A_buf = A_buf*A_d;
    //             A_qp.conservativeResize(A_qp.rows()+A_buf.rows(),A_buf.cols());
    //             A_qp.block(A_qp.rows()-A_buf.rows(),0, A_buf.rows(),A_buf.cols()) = A_buf;
    //         }
    //     }
    // }
    B_qp = B_d;
    A_qp = A_d;
    
    MatrixXd L = 10*MatrixXd::Identity(NUMOFX*HORIZON_T,NUMOFX*HORIZON_T);
    MatrixXd K = MatrixXd::Identity(NUMOFU*HORIZON_T,NUMOFU*HORIZON_T);
        
    MatrixXd H_eigen = 2*(B_qp.transpose()*L*B_qp+K);
    MatrixXd G_eigen = 2*B_qp.transpose()*L*(A_qp*x0-x_ref);

    real_t* H = H_eigen.data();
    real_t* g = G_eigen.data(); // "column major order" 
    
    MatrixXd A_eigen = MatrixXd::Zero(4*NUMOFLEG,3*NUMOFLEG);
    for(int i = 0; i < 4; i++)
        A_eigen.block(4*i,3*i,4,3) 
                        << 1,0, fric_coef,
                            1, 0, -fric_coef,
                            0, 1, fric_coef,
                            0, 1, -fric_coef; 

    real_t* A = A_eigen.data();// constraint matrix

    real_t lb[NUMOFX] 
                    = {-1e20, -1e20, Fz_min,
                        -1e20, -1e20, Fz_min,
                        -1e20, -1e20, Fz_min,
                        -1e20, -1e20, Fz_min}; 
    real_t ub[NUMOFX] 
                    = {1e20, 1e20, Fz_max,
                        1e20, 1e20, Fz_max,
                        1e20, 1e20, Fz_max,
                        1e20, 1e20, Fz_max}; 
    real_t lbA[4*NUMOFLEG] 
                    = {0, -INFINITY, 0, -INFINITY,
                        0, -INFINITY, 0, -INFINITY,
                        0, -INFINITY, 0, -INFINITY,
                        0, -INFINITY, 0, -INFINITY}; // constraint bound 
    real_t ubA[4*NUMOFLEG]
                     = {INFINITY, 0, INFINITY, 0,
                        INFINITY, 0, INFINITY, 0,
                        INFINITY, 0, INFINITY, 0,
                        INFINITY, 0, INFINITY, 0};
    real_t xOpt[NUMOFX*HORIZON_T];


    // QP 솔버 초기화
    QProblem qp(NUMOFX*HORIZON_T, 4*NUMOFLEG); // dim(x), 
    Options options;
    qp.setOptions(options);

    // 문제 풀기
    int nWSR = 10;
    // qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    // 결과 출력
    qp.getPrimalSolution(xOpt);
    for(int i = 0 ; i< NUMOFX ; i++)
        std::cout << "Optimal solution: x"<< i << " = " << xOpt[i]<< std::endl;
    
    // 최적화 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // 최적화 시간 계산
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Optimization time: " << duration.count() << " microseconds" << std::endl;

    return 0;
}