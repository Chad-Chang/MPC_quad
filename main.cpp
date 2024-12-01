#include <qpOASES.hpp>
#include <iostream>
#include <chrono> // 시간 측정을 위한 라이브러리
#include <Eigen/Dense>
#include <vector>

using namespace qpOASES;

double psi = M_PI/4;

using namespace std;
using namespace Eigen;

Matrix3d R_toEuler;
MatrixXd A_ss = MatrixXd::Zero(12, 12); 
MatrixXd A_qp = MatrixXd::Zero(0, 12);
MatrixXd A_buf = MatrixXd::Identity(12,12);

MatrixXd B_ss = MatrixXd::Zero(12, 12); 

double m = 10;
Matrix3d Inertia = Matrix3d::Identity();
std::vector<Matrix3d> r_i(4);
int horizon_t = 10;
MatrixXd B_qp = MatrixXd::Zero(12*horizon_t,12*horizon_t);
MatrixXd B_buf = MatrixXd::Identity(12,12*horizon_t);

int main() {
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
    B_ss.block(6,0, 3, 12) << Inertia.inverse()*r_i[0], Inertia.inverse()*r_i[1]
                            , Inertia.inverse()*r_i[2], Inertia.inverse()*r_i[3] ;
    B_ss.block(9,0,3,12) << Matrix3d::Identity()/m, Matrix3d::Identity()/m,
                            Matrix3d::Identity()/m, Matrix3d::Identity()/m;
    
    for(int i = 0 ; i < horizon_t; i++)
    {
        for(int j = 0 ; j<=i; ++j)
        {
            MatrixXd powerA = Eigen::MatrixXd::Identity(m, m);
            for(int k = 0 ; k<i-j;++k)
            {
                powerA*= A_ss;
            }
            B_qp.block(i*12, j*12, 12, 12) = powerA*B_ss;
            if(j == 0)
            {    
                A_buf = A_buf*A_ss;
                A_qp.conservativeResize(A_qp.rows()+A_buf.rows(),A_buf.cols());
                A_qp.block(A_qp.rows()-A_buf.rows(),0, A_buf.rows(),A_buf.cols()) = A_buf;
            }
        }   
    }
    cout <<"A = "<<  A_qp.rows()<< " "<< A_qp.cols() << "  " << A_qp.row(1)<<endl;
    cout <<"B = "<<  B_qp.rows()<< " "<< B_qp.cols() << "  " << B_qp.row(2)<<endl;
    
    
    // real_t* H = A.data();
    // QP 문제 정의
    
    real_t H[2*2] = {1,0,0,1};
    real_t g[2] = {1.5, 1.0};
    real_t A[1 * 2] = {1.0, 1.0};
    real_t lb[2] = {0.0, 0.0};
    real_t ub[2] = {5.0, 5.0};
    real_t lbA[1] = {1.0};
    real_t ubA[1] = {2.0};
    real_t xOpt[2];

    // QP 솔버 초기화
    QProblem qp(2, 1);
    Options options;
    qp.setOptions(options);

    // 문제 풀기
    int nWSR = 1000;
    qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    // 최적화 시간 확인
//    double runtime = qp.getRuntime();
  //  std::cout << "Optimization runtime (qpOASES internal): " << runtime << " seconds" << std::endl;

    // 결과 출력
    qp.getPrimalSolution(xOpt);
    std::cout << "Optimal solution: x1 = " << xOpt[0] << ", x2 = " << xOpt[1] << std::endl;
// 최적화 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();
// 최적화 시간 계산
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Optimization time: " << duration.count() << " microseconds" << std::endl;



    return 0;
}

