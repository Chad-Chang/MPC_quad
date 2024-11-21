#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace casadi;
Matrix3d R_toEuler;

double psi = M_PI/4;

int main() 
{
    R_toEuler << cos(psi), sin(psi), 0, 
                -sin(psi), cos(psi), 0, 
                0, 0, 1;
    DM R_toEuler_cas(R_toEuler.rows(), R_toEuler.cols());

    // 시스템 변수 정의
    int horizon = 15 ; // 예측 지평 길이
    double dt = 0.001; // 타임스텝
    int num_feet = 4; // 발 접촉 지점

    // 상태 및 제어 변수 정의
    std::vector<SX> states(horizon + 1);
    std::vector<SX> controls(horizon);

    for (int t = 0; t <= horizon; ++t) {
        states[t] = SX::sym("x_" + std::to_string(t), 12);  // 상태 변수
        if (t < horizon) {
            controls[t] = SX::sym("u_" + std::to_string(t), num_feet * 3);  // 제어 입력 변수
        }
    }

    // 상태 공간 동역학 정의
    SX A = SX::zeros(12, 12); 
    SX B = SX::zeros(12, num_feet * 3);

    A(Slice(0, 3), Slice(6, 9)) = R_toEuler_cas;  // 회전 행렬
    A(Slice(3, 6), Slice(9, 12)) = SX::eye(3);  // 선형 속도
    
    B(Slice(6, 9), Slice(0, num_feet * 3)) = SX::ones(3, num_feet * 3); // angular momentum
    B(Slice(9, 12), Slice(0, num_feet * 3)) = SX::ones(3, num_feet * 3) / 40;

    SX g = SX::zeros(12, 1);
    g(11) = -9.8;  // 중력

    // 비용 함수 정의
    SX x_ref = SX::zeros(12, 1); 
    x_ref(3) = 1 * dt; 
    x_ref(6) = 1; 

    SX Q = SX::diag(SX::ones(12) * 10); 
    SX R = SX::diag(SX::ones(num_feet * 3) * 0.01); 

    SX cost = 0;
    for (int t = 0; t < horizon; ++t) {
        cost += SX::sumsqr(Q * (states[t] - x_ref)) + SX::sumsqr(R * controls[t]);
    }

    // 제약 조건 정의

    // for(int i =0 ; i<100; i++){
    std::vector<SX> constraints;

    std::vector<DM> lbg_values;
    std::vector<DM> ubg_values;


    // 동역학 제약 추가
    for (int t = 0; t < horizon; ++t) {
        SX dyn_constraint = states[t + 1] - (mtimes(A, states[t]) + mtimes(B, controls[t]) * dt + g * dt);
        for(int i = 0 ; i<12; i++)
        {
            constraints.push_back(dyn_constraint(i));
            lbg_values.push_back(0);
            ubg_values.push_back(0);
        }
    }

    // 힘 제약 조건 추가 
    for (int t = 0; t < horizon; ++t) {
        for (int i = 0; i < num_feet; ++i) {
            SX fx = controls[t](i * 3);
            SX fy = controls[t](i * 3 + 1);
            SX fz = controls[t](i * 3 + 2);

            DM fz_val = (DM)fz;
            double fz_double = fz_val.scalar();

            constraints.push_back(fz); // fz min
            lbg_values.push_back(-100);
            ubg_values.push_back(100);

            constraints.push_back(fabs(fx) - 0.6 * fz); // |fx| < 
            lbg_values.push_back(-INFINITY);
            ubg_values.push_back(0);

            constraints.push_back(fabs(fy) - 0.6 * fz);
            lbg_values.push_back(-INFINITY);
            ubg_values.push_back(0);
        }
    }

    // NLP : nonlinear programming 정의
    std::vector<SX> all_vars;
    all_vars.insert(all_vars.end(), states.begin(), states.end()); //after all_vars 
    all_vars.insert(all_vars.end(), controls.begin(), controls.end());

    // constraint number has to be same how many variable I add.
    SXDict nlp = {{"x", vertcat(all_vars)}, {"f", cost}, {"g", vertcat(constraints)}}; 

    // 솔버 옵션
    Dict solver_options = {
        {"ipopt.tol", 1e-4},
        {"ipopt.print_level", 0},
        {"ipopt.max_iter", 20},
        {"ipopt.linear_solver", "ma27"}
    };

    Function solver = nlpsol("solver", "ipopt", nlp, solver_options);

    // // 초기 조건 및 경계 설정
    DM x0 = DM::zeros(12); 
    DM u0 = DM::zeros(num_feet * 3); 

    DM x0_combined = vertcat(std::vector<DM>{
        DM::repmat(x0, horizon + 1, 1), 
        DM::repmat(u0, horizon , 1)
    });
    
    DM lbx = DM::repmat(-inf, 12 * (horizon + 1) + 12 * horizon, 1);  // variable boundary
    DM ubx = DM::repmat(inf, 12 * (horizon + 1) + 12 * horizon, 1);  // -inf < 12 * (horizon + 1) + 12 * horizon < inf


    // lbg와 ubg를 DM으로 변환
    DM lbg = DM::vertcat(lbg_values);
    DM ubg = DM::vertcat(ubg_values);

    std::map<std::string, DM> arg = {
        {"x0", x0_combined},  // 수정된 초기값
        {"lbx", lbx},
        {"ubx", ubx},
        {"lbg", lbg},
        {"ubg", ubg}
    };

    auto result = solver(arg);

    // // 결과 출력
    std::cout << "Optimal solution: " << result.at("x") << std::endl;
// }
    return 0;
}
