#include "optimizer.h"

Optimizer::Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base)
:FL_ptr_(FL),FR_ptr_(FR),RL_ptr_(RL),RR_ptr_(RR),Trunk_ptr_(Base)
{
    g_ = VectorXd::Zero(NUMOFX,1);
    g_[NUMOFX-1] = -9.81;
    //* foot position with respect to inertial coordinate. r_{base -> foot}

    Ad_ = MatrixXd::Zero(NUMOFX, NUMOFX); Aqp_ = MatrixXd::Zero(NUMOFX, NUMOFX); 
    Bd_ = MatrixXd::Zero(NUMOFX, NUMOFX); Bqp_ = MatrixXd::Zero(NUMOFX, NUMOFX); 
    
    L_ = MatrixXd::Identity(NUMOFX*HORIZON_T,NUMOFX*HORIZON_T);
    K_ = MatrixXd::Identity(NUMOFU*HORIZON_T,NUMOFU*HORIZON_T);
    P_ = MatrixXd::Zero(4*NUMOFLEG,3*NUMOFLEG);
    
};

Optimizer::~Optimizer()
{
};

std::vector<VectorXd> Optimizer::MPC_SRB(VectorXd x0, VectorXd x_ref,Matrix3d inertia)
{
    std::cout <<"working"<<std::endl;
    xi_.push_back(x0);
    //*x : theta_x, theta_y, theta_z, 
    //*p_x, p_y, p_z, 
    //*dtheta_x,dtheta_y,dtheta_z,
    //*dp_x, dp_y, dp_z,

    // t++;
    while(t<HORIZON_T)
    {
        
        // x0_ = x0;
        double phi = x0[2]; // yaw angle
        double m = Trunk_ptr_->mass;
        // inertia update
        Inertia_ = inertia; 

        R_toEuler_ << cos(phi), -sin(phi),0, // 
                    sin(phi), cos(phi),0,
                    0, 0, 1;

        // continuous A
        Ad_.block(0,6,3,3) = R_toEuler_;
        Ad_.block(3,9,3,3) = Matrix3d::Identity();
        // descrete A
        Ad_ = MatrixXd::Identity(NUMOFX,NUMOFX) + Ts*Ad_;

        // continuous B
        Bd_.block(6,0, 3, NUMOFU) <<Inertia_.inverse()*Trunk_ptr_->skew_base2FL,
                                    Inertia_.inverse()*Trunk_ptr_->skew_base2FR,
                                    Inertia_.inverse()*Trunk_ptr_->skew_base2RL,
                                    Inertia_.inverse()*Trunk_ptr_->skew_base2RR;

        Bd_.block(9,0,3,NUMOFU) << Matrix3d::Identity()/m, Matrix3d::Identity()/m,
                                    Matrix3d::Identity()/m, Matrix3d::Identity()/m;    
        // descrete B
        Bd_ = Ts * Bd_;// discrete time

        // * qp form matrix update
        //     for(int i = 0 ; i < HORIZON_T; i++)//
        //     {
        //         for(int j = 0 ; j<=i; ++j)
        //         {
        //             MatrixXd powerA = Eigen::MatrixXd::Identity(NUMOFX, NUMOFX); // 13 x 13
        //             for(int k = 0 ; k<i-j;++k)
        //             {
        //                 powerA*= Ad_;
        //             }
        //             B_cond_.block(i*NUMOFX, j*NUMOFU, NUMOFX, NUMOFU) = powerA*B_d;
        //             if(j == 0)
        //             {    
        //                 A_buf = A_buf*Ad_;
        //                 A_cond_.conservativeResize(A_cond_.rows()+A_buf.rows(),A_buf.cols());
        //                 A_cond_.block(A_cond_.rows()-A_buf.rows(),0, A_buf.rows(),A_buf.cols()) = A_buf;
        //             }
        //         }
        //     }
        B_cond_ = Bd_;
        A_cond_ = Ad_;

        H_ = 2*(B_cond_.transpose() * L_ * B_cond_+ K_);
        G_ = 2*B_cond_.transpose() * L_ *( A_cond_ * x0 - x_ref + g_);
        Hqp_= H_.data();
        Gqp_= G_.data();
        for(int i = 0; i < 4; i++)
        {
            P_.block(4*i, 3*i,4, 3) << 1,0, fric_coef,
                                    1, 0, -fric_coef,
                                    0, 1, fric_coef,
                                    0, 1, -fric_coef; 
        }
        Pqp_ = P_.data();

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
        real_t lbP[4*NUMOFLEG] 
                        = {0, -INFINITY, 0, -INFINITY,
                            0, -INFINITY, 0, -INFINITY,
                            0, -INFINITY, 0, -INFINITY,
                            0, -INFINITY, 0, -INFINITY}; // constraint bound 
        real_t ubP[4*NUMOFLEG]
                        = {INFINITY, 0, INFINITY, 0,
                            INFINITY, 0, INFINITY, 0,
                            INFINITY, 0, INFINITY, 0,
                            INFINITY, 0, INFINITY, 0};
        real_t xOpt[NUMOFX*HORIZON_T];


        // QP 솔버 초기화
        QProblem qp(NUMOFX*HORIZON_T, 4*NUMOFLEG); // dim(x), 
        Options options;
        qp.setOptions(options);

        qp.init(Hqp_, Gqp_, Pqp_, lb, ub, lbP, ubP, nWSR_);
        
        // 결과 출력
        qp.getPrimalSolution(xOpt);
        VectorXd u_k = VectorXd::Zero(NUMOFX,1);
        for(int i = 0 ; i< NUMOFX ; i++)
        {
            std::cout << "Optimal solution: x"<< i << " = " << xOpt[i]<< std::endl;
            u_k[i] = xOpt[i];
        }
        ui_.push_back(u_k);
        xi_.push_back(Aqp_*xi_[t]+Bqp_*u_k + Ts*g_); // state update 
    }
    t = 0;
    return ui_;
};

