#include <qpOASES.hpp>
#include <iostream>
#include <chrono> // 시간 측정을 위한 라이브러리
#include <Eigen/Dense>
#include <vector>

#define NUMOFSTATE 12
#define NUMOFLEG 4
#define HORIZON_T 1
#define Fz_min 0
#define Fz_max 100
using namespace qpOASES;
using namespace std;
using namespace Eigen;

double psi = M_PI/4;
double m = 10;
double fric_coef = 0.6;


Matrix3d R_toEuler;
MatrixXd A_ss = MatrixXd::Zero(NUMOFSTATE, NUMOFSTATE); 
MatrixXd A_qp = MatrixXd::Zero(0, NUMOFSTATE);
MatrixXd A_buf = MatrixXd::Identity(NUMOFSTATE,NUMOFSTATE);

MatrixXd B_ss = MatrixXd::Zero(NUMOFSTATE, NUMOFSTATE); 


Matrix3d Inertia = Matrix3d::Identity();
std::vector<Matrix3d> r_i(NUMOFLEG);

MatrixXd B_qp = MatrixXd::Zero(NUMOFSTATE*HORIZON_T,NUMOFSTATE*HORIZON_T);
MatrixXd B_buf = MatrixXd::Identity(NUMOFSTATE,NUMOFSTATE*HORIZON_T);
VectorXd x0 = VectorXd::Zero(NUMOFSTATE,1);
VectorXd x_ref = VectorXd::Zero(NUMOFSTATE*HORIZON_T,1);
int main() {

    x0[2] = psi;
    x_ref[3] = 1;
    x_ref[9] = 1;
    
    //leg skew symmetric
    r_i[0] << 0,-1,-1,
            1,0,-1,
            1,1,0; // FL
    r_i[1] << 0,-1,1,
            1,0,-1,
            -1,1,0; // FL
    r_i[2] << 0,-1,-1,
            1,0,1,
            1,-1,0; // FL
    r_i[3] << 0,-1,1,
            1,0,1,
            -1,1,0; // FL

    // timer start
    auto start_time = std::chrono::high_resolution_clock::now();
    R_toEuler << cos(psi), sin(psi), 0, 
                    -sin(psi), cos(psi), 0, 
                    0, 0, 1;
    A_ss.block(0,6,3,3) = R_toEuler;
    A_ss.block(3,9,3,3) = Matrix3d::Identity();
    B_ss.block(6,0, 3, NUMOFSTATE) << Inertia.inverse()*r_i[0], Inertia.inverse()*r_i[1]
                            , Inertia.inverse()*r_i[2], Inertia.inverse()*r_i[3] ;
    B_ss.block(9,0,3,NUMOFSTATE) << Matrix3d::Identity()/m, Matrix3d::Identity()/m,
                            Matrix3d::Identity()/m, Matrix3d::Identity()/m;

    // start from here
    for(int i = 0 ; i < HORIZON_T; i++)//
    {
        for(int j = 0 ; j<=i; ++j)
        {
            MatrixXd powerA = Eigen::MatrixXd::Identity(NUMOFSTATE, NUMOFSTATE);
            for(int k = 0 ; k<i-j;++k)
            {
                powerA*= A_ss;
            }
            B_qp.block(i*NUMOFSTATE, j*NUMOFSTATE, NUMOFSTATE, NUMOFSTATE) = powerA*B_ss;
            if(j == 0)
            {    
                A_buf = A_buf*A_ss;
                A_qp.conservativeResize(A_qp.rows()+A_buf.rows(),A_buf.cols());
                A_qp.block(A_qp.rows()-A_buf.rows(),0, A_buf.rows(),A_buf.cols()) = A_buf;
            }
        }
    }
    MatrixXd L = 10*MatrixXd::Identity(NUMOFSTATE*HORIZON_T,NUMOFSTATE*HORIZON_T);
    MatrixXd K = MatrixXd::Identity(NUMOFSTATE*HORIZON_T,NUMOFSTATE*HORIZON_T);
        
    MatrixXd H_eigen = 2*(B_qp.transpose()*L*B_qp+K);
    MatrixXd G_eigen = 2*(2*B_qp.transpose()*L*(A_qp*x0-x_ref));

    real_t* H = H_eigen.data();
    real_t* g = G_eigen.data(); // "column major order" 
    
    MatrixXd A_eigen = MatrixXd::Zero(4*NUMOFLEG,3*NUMOFLEG);
    for(int i = 0; i < 4; i++)
        A_eigen.block(4*i,3*i,4,3) 
                        << -1,0, -fric_coef,
                            1, 0, -fric_coef,
                            0, -1, -fric_coef,
                            0, 1, -fric_coef; 

    real_t* A = A_eigen.data();// constraint matrix

    real_t lb[NUMOFSTATE] 
                    = {-INFTY,-INFTY,Fz_min,
                        -INFTY,-INFTY,Fz_min,
                        -INFTY,-INFTY,Fz_min,
                        -INFTY,-INFTY,Fz_min}; 
    real_t ub[NUMOFSTATE] 
                    = {INFTY,INFTY,Fz_max,
                        INFTY,INFTY,Fz_max,
                        INFTY,INFTY,Fz_max,
                        INFTY,INFTY,Fz_max}; 
    real_t lbA[4*NUMOFLEG] 
                    = {-INFTY, -INFTY,-INFTY, -INFTY,
                        -INFTY, -INFTY,-INFTY, -INFTY,
                        -INFTY, -INFTY,-INFTY, -INFTY,
                        -INFTY, -INFTY,-INFTY, -INFTY}; // constraint bound 
    real_t ubA[4*NUMOFLEG]
                     = {0,0,0,0,
                        0,0,0,0,
                        0,0,0,0,
                        0,0,0,0};
    real_t xOpt[NUMOFSTATE*HORIZON_T];

    // QP 솔버 초기화
    QProblem qp(NUMOFSTATE*HORIZON_T, 4*NUMOFLEG); // dim(x), 
    Options options;
    qp.setOptions(options);

    // 문제 풀기
    int nWSR = 1000;
    // qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    // 결과 출력
    qp.getPrimalSolution(xOpt);
    for(int i = 0 ; i< NUMOFSTATE ; i++)
        std::cout << "Optimal solution: x"<< i << " = " << xOpt[i]<< std::endl;
    
    // 최적화 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // 최적화 시간 계산
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Optimization time: " << duration.count() << " microseconds" << std::endl;

    return 0;
}