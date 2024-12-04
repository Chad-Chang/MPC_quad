#include "optimizer.h"

Optimizer::Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base)
{
    x0_ = VectorXd::Zero(NUMOFX,1);
    x_ref_ = VectorXd::Zero(NUMOFX*HORIZON_T,1);
    gravity_ = VectorXd::Zero(NUMOFX,1);
    gravity_[NUMOFX-1] = -9.81;
    //* foot position with respect to inertial coordinate. r_{base -> foot}
    
    int n_FL = 0; int n_FR = 1; int n_RL = 2; int n_RR = 3;
    trunk2foot_[n_FL] = Base->posCS - FL->posCS;
    trunk2foot_[n_FR] = Base->posCS - FR->posCS;
    trunk2foot_[n_RL] = Base->posCS - RL->posCS;
    trunk2foot_[n_RR] = Base->posCS - RR->posCS;    
    r_i_[n_FL] << 
            0, -trunk2foot_[n_FL](2), trunk2foot_[n_FL](1),
            trunk2foot_[n_FL](2), 0 ,-trunk2foot_[n_FL](0),
            -trunk2foot_[n_FL](1), -trunk2foot_[n_FL](0), 0;
    r_i_[n_FR] << 
            0, -trunk2foot_[n_FR](2), trunk2foot_[n_FR](1),
            trunk2foot_[n_FR](2), 0 ,-trunk2foot_[n_FR](0),
            -trunk2foot_[n_FR](1), -trunk2foot_[n_FR](0), 0;
    r_i_[n_RL] << 
            0, -trunk2foot_[n_RL](2), trunk2foot_[n_RL](1),
            trunk2foot_[n_RL](2), 0 ,-trunk2foot_[n_RL](0),
            -trunk2foot_[n_RL](1), -trunk2foot_[n_RL](0), 0;
    r_i_[n_RR] << 
            0, -trunk2foot_[n_RR](2), trunk2foot_[n_RR](1),
            trunk2foot_[n_RR](2), 0 ,-trunk2foot_[n_RR](0),
            -trunk2foot_[n_RR](1), -trunk2foot_[n_RR](0), 0;
    

}