#include "MPC.h"


MPC::MPC(double Ts, int horizon_time)
    : Ts(Ts), xk_ROPT(state_size,state_size),
      xd_ROPT(state_size,state_size), ek_ROPT(state_size,state_size), 
      horizon_time(horizon_time),
      Ad_ROPT(state_size,state_size), 
      Bd_ROPT(state_size, state_size), Cd_ROPT(state_size,state_size),
      r_skew_FL(3,3), r_skew_FR(3,3), r_skew_RL(3,3), r_skew_RR(3,3)
{
    Zero = MatrixXd::Zero(3,3);
    Identity = MatrixXd::Identity(3,3);
    
    
    // Robot parameter //
    I_W_inv = MatrixXd::Zero(3,3);


    // State space equation //
    xk = VectorXd::Zero(state_size);
    A = MatrixXd::Zero(state_size, state_size);
    B = MatrixXd::Zero(input_size, input_size);
    Cd = MatrixXd::Identity(measure_size, measure_size);
    uk = VectorXd::Zero(measure_size * horizon_time, 1);
    y = VectorXd::Zero(measure_size * horizon_time, 1);


    // MPC Formulation //
    F = MatrixXd::Zero(horizon_time * state_size ,state_size);
    phi = MatrixXd::Zero(horizon_time * state_size, horizon_time * state_size);
    
    phiuk = MatrixXd::Zero(state_size * horizon_time ,1);
    Fxk = MatrixXd::Zero(horizon_time * state_size, 1);

    Lp = MatrixXd::Zero(horizon_time * state_size, state_size);

    // Desired state //
    xref = VectorXd::Zero(state_size * horizon_time);
    Desired_state = VectorXd::Zero(state_size);
    
    ek = VectorXd::Zero(state_size * horizon_time);

    // Weighting Matrix //
    Q = MatrixXd::Identity(measure_size * horizon_time, measure_size * horizon_time) * 10;
    

    // X direction 
    for (int i = 3; i < Q.rows(); i += 6) 
    Q(i, i) = 3000;
    // Y direction 
    for (int i = 4; i < Q.rows(); i += 6) 
    Q(i, i) = 10;
    // Z direction
    for (int i = 5; i < Q.rows(); i += 6)
    Q(i, i) = 9000;

    // cout << Q << endl;

    R = MatrixXd::Identity(measure_size * horizon_time, measure_size * horizon_time) * 1;

    // Cost //
    df = VectorXd::Zero(state_size);

}

MPC::~MPC()
{
    
}

VectorXd MPC::getDesiredState(double t)
{   
    /******** Position *********/    
    Desired_state[0] = 0; // Roll
    Desired_state[1] = 0; // Pitch
    Desired_state[2] = 0; // Yaw
    
    Desired_state[3] = 0; // x
    Desired_state[4] = 0;     // y
    Desired_state[5] = 0.3536;     // z

    /******** Velocity *********/ 
    Desired_state[6] = 0;
    Desired_state[7] = 0;
    Desired_state[8] = 0;

    Desired_state[9] = 0;
    Desired_state[10] = 0;
    Desired_state[11] = 0;

    return Desired_state;
}

void MPC::updateState(double t)
{
    for(int i = 0; i < horizon_time; i++)
    {
        xref.block(state_size * i, 0, state_size, 1) = getDesiredState(t + Ts * i); // horizon_time, Ts
    }   

}

void MPC::Cal_Parameter(Kinematics K_FL, Kinematics K_FR, Kinematics K_RL, Kinematics K_RR)
{
    // Eigen Libraray //
    xk = K_FL.get_state();
    // cout << xk[3] << "   " << xk[9] << endl;

    R_trunk = K_FL.get_R_trunk();  
    I_W_inv = K_FL.get_I_W_inv();

    // body to foot vector -> skew symmetric //
    r_skew[0] = K_FL.get_foot_pos_skew();
    r_skew[1] = K_FR.get_foot_pos_skew();
    r_skew[2] = K_RL.get_foot_pos_skew();
    r_skew[3] = K_RR.get_foot_pos_skew();

        
    // A, B matrix //
    // A << Zero, Zero, R_trunk, Zero,
    //      Zero, Zero, Zero, Identity,
    //      Zero, Zero, Zero, Zero,
    //      Zero, Zero, Zero, Zero;

    A << Zero, Zero, Identity, Zero,
        Zero, Zero, Zero, Identity,
        Zero, Zero, Zero, Zero,
        Zero, Zero, Zero, Zero;

    B << Zero, Zero, Zero, Zero,
        Zero, Zero, Zero, Zero,
        I_W_inv * r_skew[0], I_W_inv * r_skew[1], I_W_inv * r_skew[2], I_W_inv * r_skew[3],
        Identity/m_trunk, Identity/m_trunk, Identity/m_trunk, Identity/m_trunk;  

    Ad = A * Ts + MatrixXd::Identity(state_size, state_size);
    Bd = B * Ts;
    
    // cout << Bd << endl;
    // Ad =  MatrixXd::Identity(state_size, state_size);
    // Bd = MatrixXd::Identity(state_size, state_size);
    // Cd = MatrixXd::Identity(state_size, state_size);

    // MPC formulation //
    for(int i = 0; i < horizon_time; i++)
    {
        F.block(i*state_size,0,state_size, state_size) = Cd * Ad.pow(i+1);
        Lp.block(i*state_size,0,state_size, state_size) = Cd * Ad.pow(i) * Bd;
    }

    for(int i = 1; i < horizon_time; i++)
    {
        for(int j = 0; j < i && j < horizon_time -1 ; j++)
        {
            phi.block((i) * state_size, j * state_size, state_size, input_size) = Cd * Ad.pow(i - j -1) * Bd;
            // cout << "i: " << i << "   j: " << j << "   A^" << i - j - 1 << endl;
        }
    }
    // cout << "phi\n" << phi << endl;

    

}   