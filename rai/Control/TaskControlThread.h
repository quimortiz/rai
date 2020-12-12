/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "ctrlMsg.h"
#include "control.h"
#include "RTControllerSimulation.h"
#include "gravityCompensation.h"
#include "../Core/thread.h"

/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlObjectives
struct TaskControlThread : rai::Thread {

  rai::Var<rai::Configuration> ctrl_config;
  rai::Var<CtrlMsg> ctrl_ref;
  rai::Var<CtrlMsg> ctrl_state;
  rai::Var<CtrlObjectiveL> ctrl_tasks;

  arr q_real, qdot_real, torques_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  arr Hmetric;

  arr Kp_base, Kd_base; //< Kp, Kd parameters defined in the model file
  double kp_factor, kd_factor, ki_factor;
  bool useSwift;
  bool requiresInitialSync; //< whether the step() should reinit the state from the ros message
  int verbose;

  TaskControlThread(const rai::Var<rai::Configuration>& _ctrl_config,
                    const rai::Var<CtrlMsg>& _ctrl_ref,
                    const rai::Var<CtrlMsg>& _ctrl_state,
                    const rai::Var<CtrlObjectiveL>& _ctrl_tasks);
  ~TaskControlThread();

  arr whatsTheForce(const ptr<CtrlObjective>& t);

  void step();
};

#if 0 //draft
struct TaskControlUserInterface {
  rai::Var<rai::Configuration> ctrl_config;
  rai::Var<CtrlObjectiveL> ctrl_tasks;

  TaskControlUserInterface(const rai::Var<rai::Configuration>& _ctrl_config, const rai::Var<CtrlObjectiveL>& _ctrl_tasks);

  //add your wish function
};

#endif

ptr<CtrlObjective> addCtrlObjective(rai::Var<CtrlObjectiveL>& ctrlTasks,
                                    rai::Var<rai::Configuration>& ctrl_config,
                                    const char* name, const ptr<Feature>& map,
                                    const ptr<CtrlMovingTarget>& ref);

ptr<CtrlObjective> addCtrlObjective(rai::Var<CtrlObjectiveL>& ctrlTasks,
                                    rai::Var<rai::Configuration>& ctrl_config,
                                    const char* name, FeatureSymbol fs, const StringA& frames,
                                    const ptr<CtrlMovingTarget>& ref);

ptr<CtrlObjective> addCtrlObjective(rai::Var<CtrlObjectiveL>& ctrlTasks,
                                    rai::Var<rai::Configuration>& ctrl_config,
                                    const char* name, FeatureSymbol fs, const StringA& frames,
                                    double duration);

ptr<CtrlObjective> addCompliance(rai::Var<CtrlObjectiveL>& ctrlTasks,
                                 rai::Var<rai::Configuration>& ctrl_config,
                                 const char* name, FeatureSymbol fs, const StringA& frames,
                                 const arr& compliance);

void removeCtrlObjective(rai::Var<CtrlObjectiveL>& ctrlTasks, CtrlObjective* t);

