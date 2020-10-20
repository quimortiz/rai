rai::Frame* rai::Configuration::addFile(const char* filename, const char* parentOfRoot, const rai::Transformation& relOfRoot) {
  rai::Frame* f = addFile(filename);
  if(parentOfRoot) {
    CHECK(f, "nothing added?");
    f->linkFrom(getFrameByName(parentOfRoot));
    new rai::Joint(*f, rai::JT_rigid);
    f->set_Q() = relOfRoot;
  }
  return f;
}

///// find shape with specific name
//Shape* Configuration::getShapeByName(const char* name, bool warnIfNotExist) const {
//  Frame *f = getFrameByName(name, warnIfNotExist);
//  return f->shape;
//}

///// find shape with specific name
//Joint* Configuration::getJointByName(const char* name, bool warnIfNotExist) const {
//  Frame *f = getFrameByName(name, warnIfNotExist);
//  return f->joint();
//}

/// find joint connecting two bodies
//Link* Configuration::getLinkByBodies(const Frame* from, const Frame* to) const {
//  if(to->link && to->link->from==from) return to->link;
//  return nullptr;
//}

/// find joint connecting two bodies
Joint* Configuration::getJointByFrames(const Frame* from, const Frame* to) const {
  if(to->joint && to->parent==from) return to->joint;
  return nullptr;
}

/// find joint connecting two bodies with specific names
Joint* Configuration::getJointByFrameNames(const char* from, const char* to) const {
  Frame* f = getFrameByName(from);
  Frame* t = getFrameByName(to);
  if(!f || !t) return nullptr;
  return getJointByFrames(f, t);
}

/// find joint connecting two bodies with specific names
Joint* Configuration::getJointByFrameIndices(uint ifrom, uint ito) const {
  if(ifrom>=frames.N || ito>=frames.N) return nullptr;
  Frame* f = frames.elem(ifrom);
  Frame* t = frames.elem(ito);
  return getJointByFrames(f, t);
}

uintA Configuration::getQindicesByNames(const StringA& jointNames) const {
  FrameL F = getFramesByNames(jointNames);
  uintA Qidx;
  for(Frame* f: F) {
    CHECK(f->joint, "");
    Qidx.append(f->joint->qIndex);
  }
  return Qidx;
}

/// @name variable (groups of DOFs, e.g. agents, joints, contacts) interface
FrameL vars_frames;
void vars_ensureFrames();
uint vars_getNum() { vars_ensureFrames();  return vars_frames.N; }
const String& vars_getName(uint i);
uint vars_getDim(uint i);
void vars_activate(uint i);
void vars_deactivate(uint i);
void vars_qIndex2varIndex(uint& varId, uint& varIndex, uint qIndex);

void Configuration::vars_ensureFrames() {
  if(!vars_frames.N) {
    for(Frame* f: frames) if(f->joint && f->joint->type!=JT_rigid) {
        bool isVar = true;
        Frame* p=f;
        while(p->parent) {
          if(p->parent->joint && p->parent->joint->type!=JT_rigid) { isVar=false; break; }
          p=p->parent;
        }
        if(isVar) vars_frames.append(f);
      }
  }
}

const String& Configuration::vars_getName(uint i) {
  vars_ensureFrames();
  return vars_frames(i)->name;
}

uint Configuration::vars_getDim(uint i) {
  Frame* f = vars_frames(i);
  FrameL F = {f};
  f->getSubtree(F);
  uint d =0;
  for(Frame* f:F) if(f->joint) d += f->joint->qDim();
  return d;
}

void Configuration::vars_activate(uint i) {
  Frame* f = vars_frames(i);
  FrameL F = {f};
  f->getSubtree(F);
  for(Frame* f:F) if(f->joint) f->joint->active=true;
  reset_q();
}

void Configuration::vars_deactivate(uint i) {
  Frame* f = vars_frames(i);
  FrameL F = {f};
  f->getSubtree(F);
  for(Frame* f:F) if(f->joint) f->joint->active=false;
  reset_q();
}

#else
void Configuration::jacobianPos(arr& J, Frame* a, const Vector& pos_world) const {
  J.resize(3, getJointStateDimension()).setZero();
  while(a) { //loop backward down the kinematic tree
    if(!a->parent) break; //frame has no inlink -> done
    Joint* j=a->joint;
    if(j && j->active) {
      uint j_idx=j->qIndex;
      arr screw = j->getScrewMatrix();
      for(uint d=0; d<j->dim; d++) {
        Vector axis = screw(0, d, {});
        Vector tmp = axis ^ pos_world;
        J(0, j_idx+d) += tmp.x + screw(1, d, 0);
        J(1, j_idx+d) += tmp.y + screw(1, d, 1);
        J(2, j_idx+d) += tmp.z + screw(1, d, 2);
      }
    }
    a = a->parent;
  }
}
#endif

uint Configuration::kinematicsJoints(arr& y, arr& J, const FrameL& F, bool relative_q0) const {
  CHECK_EQ(F.nd, 1, "");
  uint m=0;
  for(Frame *f: F){
    Joint *j = f->joint;
    CHECK(j, "selected frame " <<*f <<" ('" <<f->name <<"') is not a joint");
    m += j->qDim();
  }
  if(!y) return m;

  kinematicsZero(y, J, m);

  m=0;
  for(Frame *f: F){
    Joint *j = f->joint;
    for(uint k=0; k<j->dim; k++) {
      if(j->active){
        y.elem(m) = q.elem(j->qIndex+k);
      }else{
        y.elem(m) = qInactive.elem(j->qIndex+k);
      }
//      if(flipSign) q.elem(m) *= -1.;
      if(relative_q0 && j->q0.N) y.elem(m) -= j->q0(k);
      if(j->active) {
//        if(flipSign) J.elem(m, j->qIndex+k) = -1.; else
        J.elem(m, j->qIndex+k) = 1.;
      }
      m++;
    }
  }

  CHECK_EQ(y.N, m, "");
  return m;
}

/// The vector vec1, attached to b1, relative to the frame of b2
void Configuration::kinematicsRelVec(arr& y, arr& J, Frame* a, const Vector& vec1, Frame* b) const {
  arr y1, J1;
  a->ensure_X();
  b->ensure_X();
  kinematicsVec(y1, J1, a, vec1);
  //  kinematicsVec(y2, J2, b2, vec2);
  arr Rinv = ~(b->ensure_X().rot.getArr());
  y = Rinv * y1;
  if(!!J) {
    arr A;
    jacobian_angular(A, b);
    J = Rinv * (J1 - crossProduct(A, y1));
  }
}

////* This Jacobian directly gives the implied rotation vector: multiplied with \dot q it gives the angular velocity of body b */
//void Configuration::posMatrix(arr& J, Frame *a) const {
//  uint N = getJointStateDimension();
//  J.resize(3, N).setZero();

//  while(a) { //loop backward down the kinematic tree
//    Joint *j=a->joint;
//    if(j && j->active) {
//      uint j_idx=j->qIndex;
//      if(j_idx>=N) CHECK_EQ(j->type, JT_rigid, "");
//      if(j_idx<N){
//        J(0, j_idx) += j->X().pos.x;
//        J(1, j_idx) += j->X().pos.y;
//        J(2, j_idx) += j->X().pos.z;
//        }
//      }
//    }
//    a = a->parent;
//  }
//}

#if 0
/// The position vec1, attached to b1, relative to the frame of b2 (plus vec2)
void Configuration::kinematicsRelRot(arr& y, arr& J, Frame* a, Frame* b) const {
  Quaternion rot_b = a->X.rot;
  if(!!y) y = conv_vec2arr(rot_b.getVec());
  if(!!J) {
    double phi=acos(rot_b.w);
    double s=2.*phi/sin(phi);
    double ss=-2./(1.-sqr(rot_b.w)) * (1.-phi/tan(phi));
    arr A;
    axesMatrix(A, a, useSparseJacobians);
    J = 0.5 * (rot_b.w*A*s + crossProduct(A, y));
    J -= 0.5 * ss/s/s*(y*~y*A);
  }
}
#endif

void Configuration::kinematicsContactPOA(arr& y, arr& J, const ForceExchange* c) const {
  kinematicsZero(y, J, 3);

  y = c->poa;
  if(!!J) {
    for(uint i=0; i<3; i++) J.elem(i, c->qIndex+i) = 1.;
  }
}

void Configuration::kinematicsContactForce(arr& y, arr& J, const ForceExchange* c) const {
  y = c->force;

  if(!!J) {
    if(jacMode==JM_dense) {
      J.resize(3, q.N).setZero();
    } else if(jacMode==JM_sparse){
      J.sparse().resize(3, q.N, 0);
    } else if(jacMode==JM_noArr){
      J.setNoArr();
      return;
    }

    for(uint i=0; i<3; i++) J.elem(i, c->qIndex+3+i) = 1.;
  }
}

void Configuration::kinematicsLimits(arr& y, arr& J, const arr& limits) const {
  kinematicsZero(y, J, 1);
//  y.resize(1).setZero();
//  if(!!J) J.resize(1, getJointStateDimension()).setZero();
  double d;
  for(uint i=0; i<limits.d0; i++) if(limits(i, 1)>limits(i, 0)) { //only consider proper limits (non-zero interval)
      d = limits(i, 0) - q(i); //lo
      if(d>0.) {  y.elem(0) += d;  if(!!J) J.elem(0, i)-=1.;  }
      d = q(i) - limits(i, 1); //up
      if(d>0.) {  y.elem(0) += d;  if(!!J) J.elem(0, i)+=1.;  }
  }
}


#if 0
/// center of mass of the whole configuration (3 vector)
double Configuration::getCenterOfMass(arr& x_) const {
  double M=0.;
  Vector x;
  x.setZero();
  for(Frame* f: frames) if(f->inertia) {
      M += f->inertia->mass;
      x += f->inertia->mass*f->X.pos;
    }
  x /= M;
  x_ = conv_vec2arr(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void Configuration::getComGradient(arr& grad) const {
  double M=0.;
  arr J(3, getJointStateDimension());
  grad.resizeAs(J); grad.setZero();
  for(Frame* f: frames) if(f->inertia) {
      M += f->inertia->mass;
      kinematicsPos(NoArr, J, f);
      grad += f->inertia->mass * J;
    }
  grad/=M;
}

const Proxy* Configuration::getContact(uint a, uint b) const {
  for(const Proxy& p: proxies) if(p.d<0.) {
      if(p.a->ID==a && p.b->ID==b) return &p;
      if(p.a->ID==b && p.b->ID==a) return &p;
    }
  return nullptr;
}

#endif

void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const Configuration& to, const Configuration& from) {
  arr q = to.getJointState();
  uint T = qfrom.d0;
  uint Nfrom = qfrom.d1;

  if(qfrom.d1==0) {T = 1; Nfrom = qfrom.d0;}

  qto = repmat(~q, T, 1);

  intA match(Nfrom);
  match = -1;
  Joint* jfrom;
  for(Frame* f: from.frames) if((jfrom=f->joint)) {
    rai::Joint* jto = to.getFrame(jfrom->frame->name)->joint;
//    Joint* jto = to.getJointByFrameNames(jfrom->from()->name, jfrom->frame->name);
      if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
      CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
      for(uint i=0; i<jfrom->qDim(); i++) {
        match(jfrom->qIndex+i) = jto->qIndex+i;
      }
    }

  for(uint i=0; i<match.N; i++) if(match(i)!=-1) {
      for(uint t=0; t<T; t++) {
        if(qfrom.d1==0) {
          qto(t, match(i)) = qfrom(i);
        } else {
          qto(t, match(i)) = qfrom(t, i);
        }
      }
    }

  if(qfrom.d1==0) qto.reshape(qto.N);
}

#if 0 //nonsensical
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const Configuration& to, const Configuration& from) {
  //TODO: for saveness reasons, the velocities are zeroed.
  arr qDot;
  qDot = zeros(to.getJointStateDimension());
  uint T, dim;
  if(qDotFrom.d1 > 0) {
    T = qDotFrom.d0;
    qDotTo = repmat(~qDot, T, 1);
    dim = qDotFrom.d1;
  } else {
    T = 1;
    qDotTo = qDot;
    dim = qDotFrom.d0;
  }

  intA match(dim);
  match = -1;
  for(Joint* jfrom:from.joints) {
    Joint* jto = to.getJointByName(jfrom->name, false); //OLD: to.getJointByBodyNames(jfrom->from->name, jfrom->to->name); why???
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++) {
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }
  if(qDotFrom.d1 > 0) {
    for(uint i=0; i<match.N; i++) if(match(i)!=-1) {
        for(uint t=0; t<T; t++) {
          qDotTo(t, match(i)) = qDotFrom(t, i);
        }
      }
  } else {
    for(uint i=0; i<match.N; i++) if(match(i)!=-1) {
        qDotTo(match(i)) = qDotFrom(i);
      }
  }

}

void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const Configuration& to, const Configuration& from) {
  KpTo = zeros(to.getJointStateDimension(), to.getJointStateDimension());
  //use Kp gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr* info;
      info = j->ats.find<arr>("gains");
      if(info) {
        KpTo(j->qIndex, j->qIndex)=info->elem(0);
      }
    }
  }

  intA match(KpFrom.d0);
  match = -1;
  for(Joint* jfrom : from.joints) {
    Joint* jto = to.getJointByName(jfrom->name, false); // OLD: Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++) {
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0; i<match.N; i++) {
    for(uint j=0; j<match.N; j++) {
      KpTo(match(i), match(j)) = KpFrom(i, j);
    }
  }
}

void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const Configuration& to, const Configuration& from) {
  KdTo = zeros(to.getJointStateDimension(), to.getJointStateDimension());

  //use Kd gains from ors file for toWorld, if there are no entries of this joint in fromWorld
  for_list(Joint, j, to.joints) {
    if(j->qDim()>0) {
      arr* info;
      info = j->ats.find<arr>("gains");
      if(info) {
        KdTo(j->qIndex, j->qIndex)=info->elem(1);
      }
    }
  }

  intA match(KdFrom.d0);
  match = -1;
  for(Joint* jfrom : from.joints) {
    Joint* jto = to.getJointByName(jfrom->name, false); // OLD: Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++) {
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0; i<match.N; i++) {
    for(uint j=0; j<match.N; j++) {
      KdTo(match(i), match(j)) = KdFrom(i, j);
    }
  }
}

void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const Configuration& to, const Configuration& from) {
  u0To = zeros(to.getJointStateDimension());

  intA match(u0From.d0);
  match = -1;
  for(Joint* jfrom : from.joints) {
    Joint* jto = to.getJointByName(jfrom->name, false); // OLD: Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++) {
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0; i<match.N; i++) {
    u0To(match(i)) = u0From(i);
  }
}

void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const Configuration& to, const Configuration& from) {
  uint numberOfColumns = KI_ft_From.d1;
  if(KI_ft_From.d1 == 0) {
    numberOfColumns = 1;
    KI_ft_To = zeros(to.getJointStateDimension());
  } else {
    KI_ft_To = zeros(to.getJointStateDimension(), KI_ft_From.d1);
  }

  intA match(KI_ft_From.d0);
  match = -1;
  for(Joint* jfrom : from.joints) {
    Joint* jto = to.getJointByName(jfrom->name, false); // OLD: Joint* jto = to.getJointByBodyNames(jfrom->from->name, jfrom->to->name);
    if(!jto || !jfrom->qDim() || !jto->qDim()) continue;
    CHECK_EQ(jfrom->qDim(), jto->qDim(), "joints must have same dimensionality");
    for(uint i=0; i<jfrom->qDim(); i++) {
      match(jfrom->qIndex+i) = jto->qIndex+i;
    }
  }

  for(uint i=0; i<match.N; i++) {
    for(uint j=0; j < numberOfColumns; j++) {
      if(numberOfColumns > 1) {
        KI_ft_To(match(i), j) = KI_ft_From(i, j);
      } else {
        KI_ft_To(match(i)) = KI_ft_From(i);
      }
    }
  }
}
#endif


void transferQbetweenTwoWorlds(arr& qto, const arr& qfrom, const Configuration& to, const Configuration& from);
void transferQDotbetweenTwoWorlds(arr& qDotTo, const arr& qDotFrom, const Configuration& to, const Configuration& from);
void transferKpBetweenTwoWorlds(arr& KpTo, const arr& KpFrom, const Configuration& to, const Configuration& from);
void transferKdBetweenTwoWorlds(arr& KdTo, const arr& KdFrom, const Configuration& to, const Configuration& from);
void transferU0BetweenTwoWorlds(arr& u0To, const arr& u0From, const Configuration& to, const Configuration& from);
void transferKI_ft_BetweenTwoWorlds(arr& KI_ft_To, const arr& KI_ft_From, const Configuration& to, const Configuration& from);

void displayState(const arr& x, Configuration& G, const char* tag);
void displayTrajectory(const arr& x, int steps, Configuration& G, const KinematicSwitchL& switches, const char* tag, double delay=0., uint dim_z=0, bool copyG=false);
inline void displayTrajectory(const arr& x, int steps, Configuration& G, const char* tag, double delay=0., uint dim_z=0, bool copyG=false) {
  displayTrajectory(x, steps, G, {}, tag, delay, dim_z, copyG);
}

//===========================================================================

void kinVelocity(arr& y, arr& J, uint frameId, const ConfigurationL& Ktuple, double tau);
void kinAngVelocity(arr& y, arr& J, uint frameId, const ConfigurationL& Ktuple, double tau);

void kinVelocity(arr& y, arr& J, uint frameId, const ConfigurationL& Ktuple, double tau) {
  CHECK_GE(Ktuple.N, 1, "");
  Configuration& K0 = *Ktuple(-2);
  Configuration& K1 = *Ktuple(-1);
  Frame* f0 = K0.frames.elem(frameId);
  Frame* f1 = K1.frames.elem(frameId);

  arr y0, J0;
  K0.kinematicsPos(y0, J0, f0);
  K1.kinematicsPos(y, J, f1);
  y -= y0;
  J -= J0;
  y /= tau;
  J /= tau;
}

double forceClosureFromProxies(Configuration& K, uint frameIndex, double distanceThreshold, double mu, double torqueWeights) {
  Vector c, cn;
  arr C, Cn;
  for(const Proxy& p: K.proxies) {
    int body_a = p.a?p.a->ID:-1;
    int body_b = p.b?p.b->ID:-1;
    if(p.d<distanceThreshold && (body_a==(int)frameIndex || body_b==(int)frameIndex)) {
      if(body_a==(int)frameIndex) {
        c = p.posA;
        cn=-p.normal;
      } else {
        c = p.posB;
        cn= p.normal;
      }
      C.append(conv_vec2arr(c));
      Cn.append(conv_vec2arr(cn));
    }
  }
  C .reshape(C.N/3, 3);
  Cn.reshape(C.N/3, 3);
  double fc=forceClosure(C, Cn, K.frames.elem(frameIndex)->ensure_X().pos, mu, torqueWeights, nullptr);
  return fc;
}
