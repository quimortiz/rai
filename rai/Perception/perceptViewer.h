/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "percept.h"
#include "../Core/thread.h"

namespace rai {
  struct OpenGL;
}

struct PerceptViewer : rai::Thread {
  rai::Var<PerceptL> percepts;
  rai::Var<rai::Configuration> kin;
  PerceptL copy;
  rai::MeshA modelCopy;
  rai::OpenGL* gl;

  PerceptViewer(rai::Var<PerceptL>& _percepts, rai::Var<rai::Configuration> _kin);
  ~PerceptViewer();
  void open();
  void step();
  void close();
};
