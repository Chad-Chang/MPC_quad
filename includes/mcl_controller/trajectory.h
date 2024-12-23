#ifndef __TRAJECTORY_H__
#define	__TRAJECTORY_H__

#include "globalVariable.h"
#include <iostream>
using namespace std;
class trajectory
{
    public:
        trajectory();
        ~trajectory();
        void Squat(double t,StateModel_* state_model);
        void Jumping(double t, StateModel_* state_model, int mode_admitt);
        void Hold(StateModel_* state_model);
        void Hold_SRB(StateModel_* state_model, TrunkModel_* TrunkModel);
        void Walking(double T_walk, double t, StateModel_* state_model,double vx, double vx_est, int leg_num);
        Vector2d cubic_trajectory(double T_f, double T_curr, Vector2d P0, Vector2d Pf);

    private:
        double squat_T_pause;
        double freq_squat;
        double squat_r0;
        double squat_rc;

        double K_thrust;
        double zeta_thrust;
        double K_land;
        double zeta_land;

        double jump_r0;
        double jump_rc;
        double jump_rt;

        double jump_T_stand;
        double jump_T_crouch;
        double jump_T_pause;
        double jump_T_land;
        double jump_T_recover;
        double jump_qd_max;

        //walking 
        double wd_;    
        double r0_;
        double t_start_;
        double t_cub;
        
        Vector2d RW_th_swing;
        Vector2d RW_r_swing;

        Vector2d RW_r_des_0; Vector2d RW_r_des_f; Vector2d RW_r_des_f2;
        Vector2d RW_th_td_des_0; Vector2d RW_th_td_des_f;


        Vector2d vel_trunk_des;
        Vector2d pos_trunk_des;



};



#endif // !__TRAJECTORY_H__
