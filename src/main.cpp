#include <qpOASES.hpp>
#include <iostream>
#include <chrono> // 시간 측정을 위한 라이브러리
#include <Eigen/Dense>
#include <vector>
#include "ExtendedDataSaver.hpp"
// #include <OSQP.h>

using namespace qpOASES;
using namespace Eigen;
using namespace std;

// 시스템 상수 정의
const double m = 10.0;         // 질량 (kg)
const double B = 1.5;          // 감쇠 계수
const double K = 100.0;         // 스프링 계수
const double Ts = 0.001;        // 샘플링 시간 (s)

ExtendedDataSaver dataSaver("../data/extended_output.csv");
ExtendedDataSaver dataSaver_cost("../data/cost.csv");
ExtendedDataSaver dataSaver_state("../data/state.csv");

int main() {
    // 상태공간 행렬 정의 (연속 시간)
    Matrix2d A_c;
    A_c << 0, 1, -K / m, -B / m;

    Vector2d B_c(0, 1.0 / m);

    // 이산화 (Euler 근사)
    Matrix2d A_d = Matrix2d::Identity() + Ts * A_c;
    Vector2d B_d = Ts * B_c;

    // 상태 및 목표 상태 초기화
    Vector2d x0(10, 0);    // 초기 상태 (위치 = 10, 속도 = 0)
    Vector2d x_ref(0, 0);  // 목표 상태 (위치 = 0, 속도 = 0)

    // 비용 함수 가중치
    Matrix2d Q = 1000* Matrix2d::Identity();  // 상태 가중치
    double R = 0.001;                     // 제어 입력 가중치

    // QP 설정
    QProblem qp(1, 0); // 변수 1개 (u), 제약조건 없음
    Options options;
    // options.setToMPC();
    
    qp.setOptions(options);
    options.epsIterRef = 1e-8;

    for(int i = 0 ; i < 1000; i++ )
    {
        // QP 행렬 초기화
        double H_eigen = 2 * (B_d.transpose() * Q * B_d + R);
        real_t H[1] = {H_eigen}; // H는 1x1 행렬
        real_t lb[1] = {-500.0};  // 제어 입력 하한
        real_t ub[1] = {500.0};   // 제어 입력 상한
        int nWSR = 100;          // 최대 반복 횟수

        Vector2d error = A_d * x0 - x_ref;
        auto g_eigen = 2 * (B_d.transpose() * Q * error);
        real_t g[1] = {g_eigen(0)};
        real_t u_opt[1];
        qp.init(H, g, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR);

        qp.getPrimalSolution(u_opt);

        // 제어 입력 및 상태 업데이트
        VectorXd u_k = VectorXd::Zero(1);
        u_k[0] = u_opt[0];
        // u_k = 0;
        x0 = A_d * x0 + B_d * u_k;


        // dataSaver_cost.writeRow(u_k.transpose()*H_eigen*u_k+G_eigen.transpose()*u_k);
        dataSaver.writeRow(u_k);
        dataSaver_state.writeRow(x0);

        // 결과 출력
        cout << " | Control input u: " << u_k << " | State x: " << x0.transpose() << endl;
    }

    return 0;
}
