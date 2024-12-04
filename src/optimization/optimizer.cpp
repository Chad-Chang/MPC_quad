#include "optimizer.h"

Optimizer::Optimizer(StateModel_* FL, StateModel_* FR, StateModel_* RL, StateModel_* RR, TrunkModel_* Base)
:FL_ptr_(FL),FR_ptr_(FR),RL_ptr_(RL),RR_ptr_(RR),Trunk_ptr_(Base)
{
    x0_ = VectorXd::Zero(NUMOFX,1);
    x_ref_ = VectorXd::Zero(NUMOFX*HORIZON_T,1);
    gravity_ = VectorXd::Zero(NUMOFX,1);
    gravity_[NUMOFX-1] = -9.81;
    //* foot position with respect to inertial coordinate. r_{base -> foot}
    

}