#include "optimizer.h"

Optimizer::Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base)
// :FL_ptr_(FL),FR_ptr_(FR),RL_ptr_(RL),RR_ptr_(RR),Trunk_ptr_(Base)
{
    leg_ptr_[0] = FL; leg_ptr_[1]= FR; leg_ptr_[2]= RL ; leg_ptr_[3]= RR;
    Trunk_ptr_ = Base;
    //* foot position with respect to inertial coordinate. r_{base -> foot}

    Ac_ = MatrixXd::Zero(NUMOFX, NUMOFX);
    Ad_ = MatrixXd::Zero(NUMOFX, NUMOFX); Aqp_ = MatrixXd::Zero(NUMOFX, NUMOFX); 
    Bc_ = MatrixXd::Zero(NUMOFX, NUMOFU);
    Bd_ = MatrixXd::Zero(NUMOFX, NUMOFU); Bqp_ = MatrixXd::Zero(NUMOFX, NUMOFU); 
    
    
    Q_ = MatrixXd::Identity(NUMOFX*HORIZON_T,NUMOFX*HORIZON_T);
    R_ = MatrixXd::Identity(NUMOFU*HORIZON_T,NUMOFU*HORIZON_T);
    constraint_ = MatrixXd::Zero(4*NUMOFLEG,3*NUMOFLEG);
    mu_ = 0.6;
    robot_mass_ = 42.64;

    state_ = VectorXd::Zero(NUMOFX);
    state_ref_ = VectorXd::Zero(NUMOFX);
    // state_ref_ = Trunk_ptr_ -> body_state_ref;
    ctrl_input_ = VectorXd::Zero(NUMOFU);
    
};

Optimizer::~Optimizer()
{

};

void Optimizer::update_state()
{
    // get state update
    state_ = Trunk_ptr_->body_state;
    state_ref_ = Trunk_ptr_->body_state_ref;
    inertia_tensor_ = Trunk_ptr_->inertia_tensor;
}

VectorXd Optimizer::MPC_SRB()
{
    Optimizer::update_state();
    Optimizer::calculate_Ac();
    Optimizer::calculate_Bc(Matrix3d::Identity());
    Optimizer::discretization(0.001);
    Optimizer::solve_qp();
    return ctrl_input_;
};

void Optimizer::reset()
{
    Ac_.setZero();
    Bc_.setZero();

    Ad_.setZero();
    Bd_.setZero();
    Bd_list_.setZero(NUMOFX*HORIZON_T,NUMOFU);


    Aqp_.setZero();
    Bqp_.setZero();
    gradient_.setZero();
    hessian_.setZero();

}

void Optimizer::calculate_Ac() // checked
{


    double cos_yaw = cos(state_[2]);
    double sin_yaw = sin(state_[2]);
    Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
            -sin_yaw, cos_yaw, 0,
            0, 0, 1;
    Ac_.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    Ac_.block<3, 3>(3, 9) = Matrix3d::Identity();
    Ac_(11, 12) = 1; 
}

void Optimizer::calculate_Bc(Matrix3d body2global_R)
{
    inertia_global_ = body2global_R*inertia_tensor_*body2global_R.transpose();
    for(int i = 0 ; i< NUMOFLEG; ++i)
    {
        // FL, FR, RL, RR 
        Bc_.block<3,3>(6, 3*i) = inertia_global_.inverse() * Trunk_ptr_ -> skew_base2leg_CS[i];
        Bc_.block<3,3>(9, 3*i) = Matrix3d::Identity()/robot_mass_;
    }
}

void Optimizer::discretization(double dt)
{
    // auto t1 = std::chrono::high_resolution_clock::now();

    Ad_ = MatrixXd::Identity(NUMOFX, NUMOFX) + Ac_*dt;
    Bd_ = Bc_*dt;
    // auto t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;

}

void Optimizer::solve_qp()
{
    // standard QP formulation
    // minimize 1/2 * x' * P * x + q' * x
    // subject to lb <= Ac * x <= ub
    // P: hessian
    // q: gradient
    // Ac: linear constraints


    // A_qp = [A,
    //         A^2,
    //         A^3,
    //         ...
    //         A^k]'

    // B_qp = [A^0*B(0),
    //         A^1*B(0),     B(1),
    //         A^2*B(0),     A*B(1),       B(2),
    //         ...
    //         A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]

    // keep A_qp as a storage list 
    for (int i = 0; i < HORIZON_T; ++i) {
        if (i == 0) {
            Aqp_.block<NUMOFX, NUMOFX>(NUMOFX * i, 0) = Ad_;
        }
        else {
            Aqp_.block<NUMOFX, NUMOFX>(NUMOFX * i, 0) = Aqp_.block<NUMOFX, NUMOFX>(NUMOFX * (i-1), 0)*Ad_;
                
        }
        for (int j = 0; j < i + 1; ++j) {
            if (i-j == 0) {
                
                Bqp_.block<NUMOFX, NUMOFU>(NUMOFX * i, NUMOFU * j) =Bd_;
            } else {
                // Bqp_.block<NUMOFX, NUMOFU>(NUMOFX * i, NUMOFU * j) =
                //         Aqp_.block<NUMOFX, NUMOFX>(NUMOFX * (i-j-1), 0) 
                //         * Bd_list_.block<NUMOFX, NUMOFU>(j * NUMOFX, 0);
            }
        }
    }

    hessian_ = (Bqp_.transpose()*Q_*Bqp_ + R_);
    
    gradient_ = Bqp_.transpose()*Q_*(Aqp_*state_-state_ref_);

    real_t* hessian_qp= hessian_.data();
    real_t* gradient_qp = gradient_.data();

    for(int i = 0; i < NUMOFLEG; i++)
    {
        constraint_.block(4*i, 3*i,4, 3) << 
                                1,0, fric_coef,
                                1, 0, -fric_coef,
                                0, 1, fric_coef,
                                0, 1, -fric_coef; 
    }
    real_t* constraint_qp = constraint_.data();
    real_t lb[3*NUMOFLEG] 
                        = {-1e20, -1e20, Fz_min,
                            -1e20, -1e20, Fz_min,
                            -1e20, -1e20, Fz_min,
                            -1e20, -1e20, Fz_min}; 
    real_t ub[3*NUMOFLEG] 
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
    real_t UOpt[NUMOFX*HORIZON_T];

    // QP 솔버 초기화
    QProblem qp(NUMOFU*HORIZON_T, 4*NUMOFLEG); // dim(u), 
    Options options;
    qp.setOptions(options);

    qp.init(hessian_qp, gradient_qp, constraint_qp, lb, ub, lbA, ubA, nWSR_);
    qp.getPrimalSolution(UOpt);

    
    for(int i = 0 ; i< NUMOFU ; i++)
    {
        std::cout << "Optimal solution: u"<< i << " = " << UOpt[i]<< std::endl;
        ctrl_input_[i] = UOpt[i];
    }
    // state_ = Aqp_ * state_ + Bqp_ * ctrl_input_;
    // state_.push_back(Aqp_*xi_[t]+Bqp_*u_k + Ts*g_); // state update 
}