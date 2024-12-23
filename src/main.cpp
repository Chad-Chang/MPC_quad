// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <qpOASES.hpp>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include "globalVariable.h"
#include "trajectory.h"
#include "controller.h"
#include "kinematics.h"
#include "filter.h"
#include "optimizer.h"


#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;
using namespace std;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;

// char filename[] = "../assets/double pendulum.xml";

char filename[] = "../mcl/scene.xml";


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- simulation -------------------------------------------


mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}


double simEndtime = 5;	// Simulation End Time
// state parameters
StateModel_ state_Model_FL;
StateModel_ state_Model_FR;
StateModel_ state_Model_RL;
StateModel_ state_Model_RR;
TrunkModel_ TrunkModel;


const int leg_FL_no = 0;
const int leg_FR_no = 3;
const int leg_RL_no = 6;
const int leg_RR_no = 9;

controller ctrl_FL; // other class is in main loop
controller ctrl_FR;
controller ctrl_RL;
controller ctrl_RR;

kinematics kin_FL;
kinematics kin_FR;
kinematics kin_RL;
kinematics kin_RR;

trajectory tra_FL;
trajectory tra_FR;
trajectory tra_RL;
trajectory tra_RR;

Optimizer opt(&state_Model_FL,&state_Model_FR,&state_Model_RL,&state_Model_RR,&TrunkModel);

Vector2d disturbance;

double vx = 0.0;
int t ;

bool start =false;
double swept_angle;
double vx_est;
double T_walking = 1;  // total time of walking


void mycontroller(const mjModel* m,mjData *d){
  // d->qpos[2] =  1;
  if(d->time < 0.0000001)// settings
    {
      TrunkModel.body_state = VectorXd::Zero(NUMOFX);
      TrunkModel.body_state_ref = VectorXd::Zero(NUMOFX);
      TrunkModel.body_state_ref[12] = -9.81;
      for(int i = 0; i < 4; i++)
      {
      // Initialization
        d->qpos[0] = 0;
        d->qpos[1] = 0;
        // d->qpos[2] =  0.3536;   // qpos[0,1,2] : trunk pos                                                                                                                 
        d->qpos[2] =  0.5;   // qpos[0,1,2] : trunk pos                                                                                                                 
                            // qpos[3,4,5.6] : trunk orientation quaternian
        d->qpos[3] = 0.7071;
        d->qpos[4] = -0.7071;
        d->qpos[5] = 0;
        d->qpos[6] = 0;
        d->qpos[7] = 0; //FLHAA         //d->ctrl[0] FLHAA
        d->qpos[8] = pi/4; //FLHIP       //d->ctrl[1] FLHIP
        d->qpos[9] = pi/2; //FLKNEE        //d->ctrl[2] FLKNEE
        d->qpos[10] = 0; //FRHAA        //d->ctrl[3] FRHAA
        d->qpos[11] = pi/4; //FRHIP        //d->ctrl[4] FRHIP
        d->qpos[12] = pi/2; //FRKNEE       //d->ctrl[5] FRKNEE
        d->qpos[13] = 0; //RLHAA        //d->ctrl[6] RLHAA
        d->qpos[14] = pi/4; //RLHIP        //d->ctrl[7] RLHIP
        d->qpos[15] = pi/2; //RLKNEE       //d->ctrl[8] RLKNEE
        d->qpos[16] = 0; //RRHAA        //d->ctrl[9] RRHAA
        d->qpos[17] = pi/4; //RRHIP        //d->ctrl[10] RRHIP
        d->qpos[18] = pi/2; //RRKNEE       //d->ctrl[11] RRKNEE
        
        state_Model_FL.posRW[1] = pi/2 + pi/6;
        state_Model_FL.posRW_ref[0] = 0.3536;
        state_Model_FL.posRW_des[0] = state_Model_FL.posRW_ref[0];
        state_Model_FL.posRW_ref[1] = pi /2;

        state_Model_FR.posRW_ref[0] = 0.3536;
        state_Model_FR.posRW_des[0] = state_Model_FR.posRW_ref[0];
        state_Model_FR.posRW_ref[1] = pi /2;

        state_Model_RL.posRW_ref[0] = 0.3536;
        state_Model_RL.posRW_des[0] = state_Model_RL.posRW_ref[0];
        state_Model_RL.posRW_ref[1] = pi /2;

        state_Model_RR.posRW_ref[0] = 0.3536;
        state_Model_RR.posRW_des[0] = state_Model_RR.posRW_ref[0];
        state_Model_RR.posRW_ref[1] = pi /2;

        kin_FL.model_param_cal(m, d, &state_Model_FL, &TrunkModel); // state init is before. Caution Error.
        kin_FR.model_param_cal(m, d, &state_Model_FR, &TrunkModel);
        kin_RL.model_param_cal(m, d, &state_Model_RL, &TrunkModel);
        kin_RR.model_param_cal(m, d, &state_Model_RR, &TrunkModel);
        
        kin_FL.state_init(m,d, &state_Model_FL, &TrunkModel);
        kin_FR.state_init(m,d, &state_Model_FR, &TrunkModel); 
        kin_RL.state_init(m,d, &state_Model_RL, &TrunkModel); 
        kin_RR.state_init(m,d, &state_Model_RR, &TrunkModel);  
      }
    }
    
    if(d->time > 1) start = 1; // after 1sec -> start
    
    if(start) // base vel
    {
      if(1<=d->time && d->time <3 )
      {   
          vx = (0.3)/(2)*(d->time - 1);
      }
      else if(3<=d->time&& d->time<5)
      {
          vx = 0.3;

      }
    }

   //* trajectory generation
    int cmd_motion_type = 2;
    int mode_admitt = 1;
    vx_est = d->sensordata[34];
    
    if (cmd_motion_type == 0)   // Squat
    {
        tra_FL.Squat(d->time, &state_Model_FL);
        tra_FR.Squat(d->time, &state_Model_FR);
        tra_RL.Squat(d->time, &state_Model_RL);
        tra_RR.Squat(d->time, &state_Model_RR);
    }
    else if(cmd_motion_type == 1)
    {
      if(start)
      {  
        tra_FL.Walking(T_walking, d->time, &state_Model_FL, vx, vx_est, leg_FL_no);
        tra_FR.Walking(T_walking, d->time, &state_Model_FR, vx, vx_est, leg_FR_no);
        tra_RL.Walking(T_walking, d->time, &state_Model_RL, vx, vx_est, leg_RL_no);
        tra_RR.Walking(T_walking, d->time, &state_Model_RR, vx, vx_est, leg_RR_no);
      }
      
    }
    else
    {  
      tra_FL.Hold_SRB(&state_Model_FL, &TrunkModel);  // Hold stance
      tra_FR.Hold_SRB(&state_Model_FR, &TrunkModel);
      tra_RL.Hold_SRB(&state_Model_RL, &TrunkModel);
      tra_RR.Hold_SRB(&state_Model_RR, &TrunkModel);
    }

    if(d->time>0.5)
      {
        TrunkModel.GRF = opt.MPC_SRB();
        //* GRF check
        // cout <<"FL = "<<  TrunkModel.GRF[0]<< "  " <<TrunkModel.GRF[1]<< "  " <<  TrunkModel.GRF[2] << endl;
        // cout <<"FR = "<<  TrunkModel.GRF[3]<< "  " <<TrunkModel.GRF[4]<< "  " <<  TrunkModel.GRF[5] << endl;
        // cout <<"RL = "<<  TrunkModel.GRF[6]<< "  " <<TrunkModel.GRF[7]<< "  " <<  TrunkModel.GRF[8] << endl;
        // cout <<"RR = "<<  TrunkModel.GRF[9]<< "  " <<TrunkModel.GRF[10]<< "  " <<  TrunkModel.GRF[11] << endl;
      }
    
    //* update sensor
    kin_FL.sensor_measure(m, d, &state_Model_FL, &TrunkModel, leg_FL_no); // get joint sensor data & calculate biarticular angles
    kin_FR.sensor_measure(m, d, &state_Model_FR, &TrunkModel, leg_FR_no);
    kin_RL.sensor_measure(m, d, &state_Model_RL, &TrunkModel, leg_RL_no);
    kin_RR.sensor_measure(m, d, &state_Model_RR, &TrunkModel, leg_RR_no);   

    //* model parameter: ..
    kin_FL.model_param_cal(m, d,&state_Model_FL, &TrunkModel); // calculate model parameters
    kin_FR.model_param_cal(m, d,&state_Model_FR, &TrunkModel);
    kin_RL.model_param_cal(m, d,&state_Model_RL, &TrunkModel);
    kin_RR.model_param_cal(m, d,&state_Model_RR, &TrunkModel);

    kin_FL.jacobianRW(&state_Model_FL);
    kin_FR.jacobianRW(&state_Model_FR);
    kin_RL.jacobianRW(&state_Model_RL);
    kin_RR.jacobianRW(&state_Model_RR);            // calculate RW Jacobian
  
    kin_FL.fwdKinematics_cal(&state_Model_FL);     // calculate RW Kinematics
    kin_FR.fwdKinematics_cal(&state_Model_FR);
    kin_RL.fwdKinematics_cal(&state_Model_RL);
    kin_RR.fwdKinematics_cal(&state_Model_RR);

    // d-> qpos[2] = 0.3813;
    
    // disturbance << 20*sin(d->time *10),0; // r disturbance
    /* Controllers */
    int flag_DOB = 1;           // flag for switching ON/OFF RWDOB
    int flag_admitt = 0;        // flag for switching ON/OFF admittance control
    double time_run = d->time;
    // d->qpos[2] =0.5;
    
    //Admittance Control
    ctrl_FL.admittanceCtrl(&state_Model_FL,5,2,5000, flag_admitt); //parameter(omega_n,zeta,k)
    ctrl_FR.admittanceCtrl(&state_Model_FL,5,2,5000, flag_admitt);
    ctrl_RL.admittanceCtrl(&state_Model_FL,5,2,5000, flag_admitt);
    ctrl_RR.admittanceCtrl(&state_Model_FL,5,2,5000, flag_admitt);
 // PID Control
    ctrl_FL.pid_gain_pos(10000,100, 150); //(kp,kd,freq)
    ctrl_FR.pid_gain_pos(10000,100, 150); 
    ctrl_RL.pid_gain_pos(10000,100, 150); 
    ctrl_RR.pid_gain_pos(10000,100, 150); 

    
    // if(d->time <1)
    // {
    state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans * (ctrl_FL.PID_pos(&state_Model_FL) ); // RW position feedback
    state_Model_FR.tau_bi = state_Model_FR.jacbRW_trans * (ctrl_FR.PID_pos(&state_Model_FR) );
    state_Model_RL.tau_bi = state_Model_RL.jacbRW_trans * (ctrl_RL.PID_pos(&state_Model_RL) );
    state_Model_RR.tau_bi = state_Model_RR.jacbRW_trans * (ctrl_RR.PID_pos(&state_Model_RR) );

    // state_Model_FL.tau_bi += ctrl_FL.DOBRW(&state_Model_FL, 150, flag_DOB); 
    // state_Model_FR.tau_bi += ctrl_FR.DOBRW(&state_Model_FR, 150, flag_DOB); 
    // state_Model_RL.tau_bi += ctrl_RL.DOBRW(&state_Model_RL, 150, flag_DOB); 
    // state_Model_RR.tau_bi += ctrl_RR.DOBRW(&state_Model_RR, 150, flag_DOB); 

    // Force Observer
    ctrl_FL.FOBRW(&state_Model_FL, 100); // Rotating Workspace Force Observer (RWFOB)
    ctrl_FR.FOBRW(&state_Model_FR, 100); 
    ctrl_RL.FOBRW(&state_Model_RL, 100); 
    ctrl_RR.FOBRW(&state_Model_RR, 100); 
    


    if(d->time<=3)
    {
//    // Torque input Biarticular
      d->ctrl[0] = 5000*(0-d->qpos[7]); //FLHAA  
      d->ctrl[1] = state_Model_FL.tau_bi[0] + state_Model_FL.tau_bi[1] + disturbance[0];
      d->ctrl[2] = state_Model_FL.tau_bi[1];

      d->ctrl[3] = 5000*(0-d->qpos[10]); //FRHAA  
      d->ctrl[4] = state_Model_FR.tau_bi[0] + state_Model_FR.tau_bi[1] +disturbance[0];
      d->ctrl[5] = state_Model_FR.tau_bi[1];

      d->ctrl[6] = 5000*(0-d->qpos[13]); //RLHAA  
      d->ctrl[7] = state_Model_RL.tau_bi[0] + state_Model_RL.tau_bi[1] +disturbance[0];
      d->ctrl[8] = state_Model_RL.tau_bi[1];

      d->ctrl[9] = 5000*(0-d->qpos[16]); //FLHAA  
      d->ctrl[10] = state_Model_RR.tau_bi[0] + state_Model_RR.tau_bi[1] +disturbance[0];
      d->ctrl[11] = state_Model_RR.tau_bi[1];
    }
    else
    {
      // d->ctrl[0] = ; //FLHAA  
      // d->ctrl[1] = ;
      // d->ctrl[2] = ;

      // d->ctrl[3] = ;  //FRHAA 
      // d->ctrl[4] = ;
      // d->ctrl[5] = ;

      // d->ctrl[6] = ; //FRHAA  
      // d->ctrl[7] = ;
      // d->ctrl[8] = ;

      // d->ctrl[9] = ; //FRHAA  
      // d->ctrl[10] = ;
      // d->ctrl[11] = ;
    }
    
  kin_FL.state_update(&state_Model_FL);
  kin_FR.state_update(&state_Model_FR);
  kin_RL.state_update(&state_Model_RL);
  kin_RR.state_update(&state_Model_RR);

  ctrl_FL.ctrl_update();
  ctrl_FR.ctrl_update();
  ctrl_RL.ctrl_update();
  ctrl_RR.ctrl_update();

}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  
  // cpu-sim syncronization point
  mjcb_control = mycontroller;
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);
        

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // inject noise
          if (sim.ctrl_noise_std) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
            mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            stepped = true;
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // call mj_step
              mj_step(m, d);
              stepped = true;

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);
    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char** argv) {
  
  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  // const char* filename = nullptr;
  // if (argc >  1) {
  //   filename = argv[1];
  // }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();  
  physicsthreadhandle.join();

  return 0;
}
