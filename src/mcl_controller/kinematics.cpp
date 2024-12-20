#include "kinematics.h"
#include <iostream>
using namespace std;
kinematics::kinematics(){};

kinematics::~kinematics(){};

void kinematics::state_update(StateModel_* state_model)
{

    //printf("%f, %f %f \n\n", state_model->forceExt_hat[0], state_model->forceExt_hat_old[0], state_model->forceExt_hat_old2[0]);
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint
        state_model->q_old[i] = state_model->q[i];
        state_model->q_bi_old[i] = state_model->q_bi[i]; //okay
        state_model->qdot_bi_old[i] = state_model->qdot_bi[i];
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i]; //okay
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i]; //okay

        // Feedback - RW Kinematics
        state_model->posRW_old[i] = state_model->posRW[i];
        state_model->velRW_old[i] = state_model->velRW[i];
        state_model->posRW_ref_old2[i] = state_model->posRW_ref_old[i];
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        state_model->velRW_ref_old[i] = state_model->velRW_ref[i];

        // control input
        state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];
        state_model->tau_bi_old[i] = state_model->tau_bi[i];   
    }

    state_model->H_old = state_model->H;
    //printf("%f, %f \n", state_model->lhs_fob_LPF[0], state_model->lhs_fob_LPF_old[0]);
};


void kinematics::model_param_cal(const mjModel* m, mjData* d, StateModel_* state_model, TrunkModel_* TrunkModel)
{
    cut_off_cal = 1/(2*pi*150);
    // cout << state_model -> q[1] <<endl;
    /* Trunk Parameters */
    m_hip = 2.5;
    m_trunk_front = 10.;
    m_trunk_rear = 18.;
    m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;

    /* Leg Parameters */
    L = 0.25;
    d_thigh = 0.11017; // local position of CoM of thigh
    d_shank = 0.12997; // local position of CoM of shank
    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    m_thigh = 1.017; // mass of thigh link
    m_shank = 0.143; // mass of shank link
    m_leg = m_thigh + m_shank;
    m_total = m_trunk + 4 * m_leg;
    // printf("m_thigh : %f, m_shank : %f \n", m_thigh, m_shank);

    Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM
    // printf("Izz_thigh : %f, Izz_shank : %f \n", Izz_thigh, Izz_shank);

    Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE
    // printf("Jzz_thigh : %f, Jzz_shank : %f \n", Jzz_thigh, Jzz_shank);

    double M1 = Jzz_thigh + m_shank * pow(L, 2);
    double M2 = m_shank * d_shank * L * cos(state_model->q[2]);
    double M12 = Jzz_shank;

    MatInertia_bi(0,0) = M1;
    MatInertia_bi(0,1) = M12;
    MatInertia_bi(1,0) = M12;
    MatInertia_bi(1,1) = M2;

    state_model->Lamda_nominal_FOB(0,0) = M1;
    state_model->Lamda_nominal_FOB(0,1) = M12;
    state_model->Lamda_nominal_FOB(1,0) = M12;
    state_model->Lamda_nominal_FOB(1,1) = M2;
    
    

    JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(state_model->q[2]);
    JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(state_model->q[2]);
    // printf("JzzR_thigh : %f, JzzR_shank : %f, JzzR_couple : %f \n", JzzR_thigh, JzzR_shank,
    // JzzR_couple);

    MatInertia_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(state_model->q[2] / 2), 2));
    MatInertia_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(state_model->q[2]));
    MatInertia_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(state_model->q[2]));
    MatInertia_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(state_model->q[2] / 2), 2));
        
    
    Inertia_DOB(0,0) = MatInertia_RW(0,0);
    Inertia_DOB(0,1) = 0;
    Inertia_DOB(1,0) = 0;
    Inertia_DOB(1,1) = MatInertia_RW(1,1);
    

    double check[4] = { 0 };
    
    state_model->Lamda_nominal_DOB = state_model->jacbRW_trans*Inertia_DOB*state_model->jacbRW;
    // cout << state_model->jacbRW_trans << endl;
    // cout << state_model->Lamda_nominal_DOB <<endl;
    
    //Coriolis & Gravity

    // H[0] = -m_shank * d_shank * L * sin(state_model->q[1]) * pow(state_model->qdot_bi[1], 2)
    //      - g * (m_thigh * d_thigh + m_shank * L) * cos(state_model->q_bi[0]);

    // H[1] = m_shank * d_shank * L * sin(state_model->q[1]) * pow(state_model->qdot_bi[0], 2)
    //      - g * m_shank * d_shank * cos(state_model->q_bi[1]);

    coriolis_bi_[0] = -m_shank*d_shank*L*sin(state_model->q[2])*pow(state_model->qddot_bi[1],2);
    coriolis_bi_[1] = m_thigh*d_shank*L*sin(state_model->q[2]) * pow(state_model->qdot_bi[0], 2);
    gravity_bi_[0] = g * (m_thigh * d_thigh + m_shank * L) * cos(state_model->q_bi[0]);
    gravity_bi_[1] = g * m_shank * d_shank * cos(state_model->q_bi[2]);

    off_diag_inertia_bi_(0,0)= 0;
    off_diag_inertia_bi_(0,1)= m_shank*d_shank*L*cos(state_model->q[2]);
    off_diag_inertia_bi_(1,0)= off_diag_inertia_bi_(0,1);
    off_diag_inertia_bi_(1,1)= 0;
    
    state_model -> corriolis_bi_torq = coriolis_bi_;
    state_model -> gravity_bi_torq = gravity_bi_;
    state_model -> off_diag_inertia_bi = off_diag_inertia_bi_;

    TrunkModel -> inertia_tensor = Matrix3d::Identity();
    // diagonal inertia는 DOB의 nominal inertia
}; // param_model parameter

//
void kinematics::sensor_measure(const mjModel* m, mjData* d, StateModel_* state_model, TrunkModel_* TrunkModel, int leg_no)
{

    TrunkModel -> mass = 42.64;

    cut_off_cal = 1/(2*pi*100);

    state_model->q[0] = d->qpos[leg_no + 7];
    state_model->q[1] = d->qpos[leg_no + 8]; // (relative) HFE angle
    state_model->q[2] = d->qpos[leg_no + 9]; // (relative) KFE angle
    
    state_model->qdot[0] = d->qvel[leg_no + 6]; // 속도는 이게 맞음.
    state_model->qdot[1] = d->qvel[leg_no + 7];
    state_model->qdot[2] = d->qvel[leg_no + 8];

    /*** Biarticular Transformation ***/ 
    state_model->q_bi[0] = d->qpos[leg_no + 8];                             // (absolute) HFE angle
    state_model->q_bi[1] = d->qpos[leg_no + 8] + d->qpos[leg_no + 9]; // (absolute) KFE angle

    //* cartesian pos and vel  w.r.t ineritial frame
    state_model -> posCS[0] = d->sensordata[leg_no+44];
    state_model -> posCS[1] = d->sensordata[leg_no+45];
    state_model -> posCS[2] = d->sensordata[leg_no+46];

    TrunkModel->velCS[0] = d->sensordata[34]; // CoM vx
    TrunkModel->velCS[1] = d->sensordata[35]; // CoM vy
    TrunkModel->velCS[2] = d->sensordata[36]; // CoM vz

    TrunkModel->posCS[0] = d->sensordata[37]; // CoM x
    TrunkModel->posCS[1] = d->sensordata[38]; // CoM y
    TrunkModel->posCS[2] = d->sensordata[39]; // CoM z

    // TrunkModel->body_state << 

    // update leg position
    int leg_num = leg_no/3;
    TrunkModel->pos_base2leg_CS[leg_num] = state_model->posCS - TrunkModel->posCS;
    TrunkModel->skew_base2leg_CS[leg_num] = Utils::skew(TrunkModel->pos_base2leg_CS[leg_num]);
    //*

// angular velocity data_ real data
    state_model->qdot_bi[0] = d->qvel[leg_no + 7];
    state_model->qdot_bi[1] = d->qvel[leg_no + 7] + d->qvel[leg_no + 8];

//angular acceleration data_real data
    // state_model->qddot_bi[0] = d->qacc[leg_no + 8];
    // state_model->qddot_bi[1] = d->qacc[leg_no + 8] + d->qacc[leg_no + 9];

    for (int i = 0; i < NDOF_LEG; i++)
    {
        state_model->qdot_bi_tustin[i] =
            tustin_derivative(state_model->q_bi[i], state_model->q_bi_old[i], state_model->qdot_bi_tustin_old[i],
                cut_off_cal);
        state_model->qddot_bi_tustin[i] =
            tustin_derivative(state_model->qdot_bi_tustin[i], state_model->qdot_bi_tustin_old[i],
                state_model->qddot_bi_tustin_old[i], cut_off_cal);
    }
};

void kinematics::jacobianRW(StateModel_* state_model)
{
    /*** Rotating Workspace ***/
    state_model->jacbRW(0,0) =  L * sin(state_model->q[2] / 2);
    state_model->jacbRW(0,1) = -L * sin(state_model->q[2] / 2);
    state_model->jacbRW(1,0) =  L * cos(state_model->q[2] / 2);
    state_model->jacbRW(1,1) =  L * cos(state_model->q[2] / 2);
    
    state_model->jacbRW_trans = state_model->jacbRW.transpose(); 

    state_model->jacbRW_trans_inv = state_model->jacbRW_trans.inverse();
};

void kinematics::fwdKinematics_cal(StateModel_* state_model)
{
    state_model->posRW[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2); // r
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;                           // qr

    state_model->velRW = state_model->jacbRW*state_model->qdot_bi_tustin;
    
};

void kinematics::state_init(const mjModel* m, mjData* d, StateModel_* state_model)
{
    state_model->q[0] = d->qpos[1];
    state_model->q[1] = d->qpos[2];

    state_model->q_bi[0] = d->qpos[1];
    state_model->q_bi[1] = d->qpos[1] + d->qpos[2];

    // RW coordinates initialization
    state_model->r0 = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);

    state_model->posRW[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;

    state_model->posRW_ref[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    state_model->posRW_ref[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;


    state_model->touch_sensor = 0;

    state_model->Lamda_nominal_DOB(0,0)= 0.0;
    state_model->Lamda_nominal_DOB(0,1)= 0.0;
    state_model->Lamda_nominal_DOB(1,0)= 0.0;
    state_model->Lamda_nominal_DOB(1,1)= 0.0;

    //printf("%f \n", state_model->r0);
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint coordinates [k-1] values
        state_model->q_bi_old[i] = state_model->q_bi[i];

        state_model->qdot_bi[i] = 0.0;
        state_model->qdot_bi_tustin[i] = 0.;
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i];

        state_model->qddot_bi[i] = 0.;
        state_model->qddot_bi_tustin[i] = 0.;
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i];

        state_model->tau_bi[i] = 0.;
        state_model->tau_bi_old[i] = state_model->tau_bi[i];

        // RW coordinates [k-1] values
        state_model->posRW_old[i] = state_model->posRW[i];
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        state_model->posRW_ref_old2[i] = state_model->posRW_ref_old[i];

        state_model->velRW[i] = .0;
        state_model->velRW_old[i] = state_model->velRW[i];
        state_model->velRW_ref[i] = 0.;
        state_model->velRW_ref_old[i] = state_model->velRW_ref[i];


        state_model->ctrl_input_RW[i] = 0.;
        state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];


        // Mg Trajectory
        state_model->tau_ff[i]=0.;
    }
    // printf("%f, %f \n", state_model->q_bi_old[0], state_model->q_bi_old[1]);
};

