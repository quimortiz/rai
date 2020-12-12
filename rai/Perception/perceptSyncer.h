/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "percept.h"
#include "../Core/thread.h"

/// syncs percepts with modelWorld
struct SyncFiltered : rai::Thread {
  rai::Var<PerceptL> percepts;
  rai::Var<rai::Configuration> kin;

  SyncFiltered(rai::Var<PerceptL>& _percepts, rai::Var<rai::Configuration>& _kin);
  ~SyncFiltered();

  virtual void step();
};
