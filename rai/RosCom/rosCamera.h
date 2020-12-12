/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/kin.h"
#include "../Core/thread.h"

struct RosCamera {
  rai::Var<byteA> rgb;
  rai::Var<floatA> depth;
  unique_ptr<struct sRosCamera> s;

  RosCamera(rai::Var<byteA>& _rgb, rai::Var<floatA> _depth,
            const char* rosNodeName = "rai_node",
            const char* rgb_topic = "/camera/rgb/image_rect_color",
            const char* depth_topic = "/camera/depth_registered/image_raw",
            bool useUint=false);
  ~RosCamera();
};
