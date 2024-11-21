#include "trajectory.h"
#include <cmath>

trajectory::trajectory()
{
    squat_T_pause = 1.5;
    freq_squat = 1;
    squat_r0 = 0.3536;
    squat_rc = 0.2;

    K_thrust = 500;
    zeta_thrust = 0;
    K_land = 1000;
    zeta_land = 1;

    jump_r0 = 0.3526;
    jump_rc = 0.2;
    jump_rt = 0.4;

    jump_T_stand = 2;
    jump_T_crouch = 1;
    jump_T_pause = 0.5;
    jump_T_land = 1;
    jump_T_recover = 1;
    jump_qd_max = 20;
};

trajectory::~trajectory(){};

void trajectory::Squat(double t,StateModel_* state_model)
{
    double deltaR = squat_r0 - squat_rc;

    double T_squat = 1 / freq_squat;
    double T_period = T_squat + squat_T_pause;

    double t_norm = t - T_period * floor(t / T_period); // nominalized time
    double t1 = squat_T_pause;
    double t2 = t1 + T_squat;

    if (0 <= t_norm && t_norm < t1)
    {
        state_model->posRW_ref[0] = squat_r0;
        state_model->posRW_des[0] = squat_r0;
        state_model->posRW_ref[1] = pi / 2;
        state_model->posRW_des[0] = pi/2;
    }
    else if (t1 <= t_norm && t_norm < t2)
    {
        state_model->posRW_ref[0] = squat_r0 - 0.5 * deltaR * (1 - cos(2 * pi * freq_squat * (t_norm - t1)));
        state_model->posRW_des[0] = squat_r0 - 0.5 * deltaR * (1 - cos(2 * pi * freq_squat * (t_norm - t1)));
        state_model->posRW_ref[1] = pi / 2;
        state_model->posRW_des[0] = pi/2;

        state_model->velRW_ref[0] = -deltaR * (pi * freq_squat) * sin(2 * pi * freq_squat * (t_norm - t1));
        
        
    }
};

//Jumaping need debugging. it need admittance parameter
void trajectory::Jumping(double t, StateModel_* state_model, int mode_admitt)
{
}; // Jumping;

void trajectory::Hold(StateModel_* state_model)
{
    state_model->posRW_ref[0] = 0.3536;
    state_model->posRW_ref[1] = pi /2;
};


//this cubic function is wered -> besie curve is needed or fifth polynomials
Vector2d trajectory::cubic_trajectory(double T_f, double T_curr, Vector2d P0, Vector2d Pf)
{   
    double x1 = Pf[0]; double dx1 = Pf[1]; double x0=  P0[0]; double dx0 = P0[1];
    double t = T_curr/T_f;
    Vector2d St; 
    if(T_f > T_curr)
    {
        St[0] = (-2*x1+dx0+2*x0 +dx1)*pow(t,3) + (3*x1-dx1-3*x0 - 2*dx0)*pow(t,2) + dx0 *t + x0;
        St[1] = 3*(-2*x1+dx0+2*x0 +dx1)*pow(t,2) + 2*(3*x1-dx1-3*x0 - 2*dx0)*pow(t,1) + dx0;
        // cout << St[0] << "cutt = "<< T_curr<< " " << T_f <<endl;
        return St;
    }
    else
    {
        St[0] = x1;
        St[1] = dx1;
        return St;
    }
};

void trajectory::Walking(double T_walk, double t, StateModel_* state_model,double vx, double vx_est, int leg_num)
{   
    double T_walk_ = T_walk; // walking time
    double T_phase = T_walk / 4; // phase time
    double T_stance = T_phase * 3;
    
    r0_ = state_model->posRW_ref[0];  

    //steady state swept angleT_stand

    // raibert 
    double kp = 0.00;
    double swept_angle_ref = vx/r0_;
    double swept_angle_des = swept_angle_ref * T_stance + kp * ((vx-vx_est)/r0_) * T_stance/2;
    wd_ = - swept_angle_des / T_stance; // desired angular vel

    
    
    RW_r_des_0[0] = r0_; RW_r_des_0[1] = 0;
    RW_r_des_f[0] = r0_; RW_r_des_f[1] = 0;
    RW_r_des_f2[0] = r0_; RW_r_des_f2[1] = 0;
    
    RW_th_td_des_0[0] = state_model->posRW[1]; RW_th_td_des_0[1] = 0;
    RW_th_td_des_f[0] = pi/2 + swept_angle_des/2; RW_th_td_des_f[1] = 0;
    // if(leg_num == 0)
    //     cout << "swept vel -= "<< wd_<<  "swept_angle_des = " << swept_angle_des <<"  time = "<<  t <<endl;

    if(t - t_start_ < T_walk) 
    {
        if (std::floor((t-t_start_)/T_phase) == 0) // RL : flight : leg_label 2
        {
            if(leg_num == 6) 
            {
                double t_buf;
                if(T_phase/2 > fmod(t-t_start_,T_phase))
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_) ,T_phase), RW_r_des_0, RW_r_des_f);
                }
                else
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_),T_phase)-T_phase/2, RW_r_des_f, RW_r_des_f2);
                }
                RW_th_swing = this->cubic_trajectory(T_phase, fmod((t-t_start_) ,T_phase), RW_th_td_des_0, RW_th_td_des_f);
                state_model->posRW_ref[0] = RW_r_swing[0];
                state_model->posRW_ref[1] = RW_th_swing[0] ;// sign is right and need to have trajectory
                
                // cubic trajectory need
            }
            else // other legs : stance
            {
                state_model-> posRW_ref[0] = r0_;
                state_model-> posRW_ref[1] += Ts * wd_/4;
            }
        }
        else if(std::floor((t-t_start_)/T_phase) == 1) // FL : flight : leg_label 0
        {
            if(leg_num == 0) 
            {
                if(T_phase/2 > fmod(t-t_start_,T_phase))
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_) ,T_phase), RW_r_des_0, RW_r_des_f);
                }
                else
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_),T_phase)-T_phase/2, RW_r_des_f, RW_r_des_f2);
                }
                RW_th_swing = this->cubic_trajectory(T_phase, fmod((t-t_start_) ,T_phase), RW_th_td_des_0, RW_th_td_des_f);
                state_model->posRW_ref[0] = RW_r_swing[0];
                state_model->posRW_ref[1] = RW_th_swing[0] ;

                // cubic trajectory need
            }
            else // other legs : stance
            {
                state_model-> posRW_ref[0] = r0_;
                state_model-> posRW_ref[1] += Ts * wd_/4;
            }
        }
        
        else if(std::floor((t-t_start_)/T_phase) == 2) // RR : flight : leg_label 3
        {
            if(leg_num == 9) // RL : flight : leg_label 0
            {
                
                if(T_phase/2 > fmod(t-t_start_,T_phase))
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_) ,T_phase), RW_r_des_0, RW_r_des_f);
                }
                else
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_),T_phase)-T_phase/2, RW_r_des_f, RW_r_des_f2);
                }
                RW_th_swing = this->cubic_trajectory(T_phase, fmod((t-t_start_) ,T_phase), RW_th_td_des_0, RW_th_td_des_f);
                state_model->posRW_ref[0] = RW_r_swing[0];
                state_model->posRW_ref[1] = RW_th_swing[0] ;
                // cubic trajectory need
            }
            else // other legs : stance
            {
                state_model-> posRW_ref[0] = r0_;
                state_model-> posRW_ref[1] += Ts * wd_/4;
            }
        }
        else if(std::floor((t-t_start_)/T_phase) == 3) // FR : flight : leg_label 1
        {
            if(leg_num == 3) 
            {
                if(T_phase/2 > fmod(t-t_start_,T_phase))
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_) ,T_phase), RW_r_des_0, RW_r_des_f);
                }
                else
                {
                    RW_r_swing = this->cubic_trajectory(T_phase/2, fmod((t-t_start_),T_phase)-T_phase/2, RW_r_des_f, RW_r_des_f2);
                }
                RW_th_swing = this->cubic_trajectory(T_phase, fmod((t-t_start_) ,T_phase), RW_th_td_des_0, RW_th_td_des_f);
                state_model->posRW_ref[0] = RW_r_swing[0];
                state_model->posRW_ref[1] = RW_th_swing[0] ;
                // cubic trajectory need
            }
            else // other legs : stance
            {
                state_model-> posRW_ref[0] = r0_;
                state_model-> posRW_ref[1] += Ts * wd_/4;
            }
        }
        else if(std::floor((t-t_start_)/T_phase) >=4)
            t_start_ = t;
    }
    else 
        t_start_ = t;
}