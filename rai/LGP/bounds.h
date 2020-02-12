/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../KOMO/komo.h"
#include "compute.h"

//===========================================================================

struct KOMO_based_bound : ComputeObject {
  ptr<KOMO> komo;

  KOMO_based_bound(ptr<KOMO>& komo) : komo(komo) {}

  virtual ptr<ComputeReport> run(double timeBudget=-1.){NIY}
};

//===========================================================================

struct PoseBound : KOMO_based_bound {
  //the constructor creates the komo problem -- absorbes what previously was defined in skeleton2Bound!
  PoseBound(ptr<KOMO>& komo, //TODO: eventually remove this!
            const Skeleton& S,
            const rai::Configuration& startKinematics,
            bool collisions);
};

//===========================================================================

struct SeqBound : KOMO_based_bound {
  SeqBound(ptr<KOMO>& komo, //TODO: eventually remove this!
            const Skeleton& S,
            const rai::Configuration& startKinematics,
            bool collisions);
};

//===========================================================================

struct PathBound : KOMO_based_bound {
  PathBound(ptr<KOMO>& komo, //TODO: eventually remove this!
            const Skeleton& S,
            const rai::Configuration& startKinematics,
            bool collisions);
};

//===========================================================================

struct SeqPathBound : KOMO_based_bound {
  SeqPathBound(ptr<KOMO>& komo, //TODO: eventually remove this!
               const Skeleton& S,
               const rai::Configuration& startKinematics,
               bool collisions, const arrA& waypoints);
};

//===========================================================================

struct SeqVelPathBound : KOMO_based_bound {
  SeqVelPathBound(ptr<KOMO>& komo, //TODO: eventually remove this!
                  const Skeleton& S,
                  const rai::Configuration& startKinematics,
                  bool collisions, const arrA& waypoints);
};

//===========================================================================

enum BoundType { BD_all=-1,
                 BD_symbolic=0,
                 BD_pose,
                 BD_seq,
                 BD_path,
                 BD_seqPath,
                 BD_seqVelPath,
                 BD_poseFromSeq,
                 BD_max
               };

ptr<ComputeObject> skeleton2Bound(ptr<KOMO>& komo,
                    BoundType boundType,
                    const Skeleton& S,
                    const rai::Configuration& startKinematics,
                    const rai::Configuration& parentEffKinematics,
                    bool collisions,
                    const arrA& waypoints= {}
                   );

struct SubCG {
  NodeL frames;
  NodeL constraints;
  uint maxT;
  int merged=-1;
  void write(ostream& os) const {
    cout <<"*** subproblem (merged:" <<merged <<")" <<endl;
    cout <<"  frames:";
    for(Node* f:frames) cout <<' ' <<*f;
    cout <<"\n  constraints:";
    for(Node* c:constraints) cout <<"\n    " <<*c;
    cout <<endl;
  }
};
stdOutPipe(SubCG)
struct CG {
  Graph G;
  rai::Array<std::shared_ptr<SubCG>> subproblems;
};

ptr<CG> skeleton2CGO(const Skeleton& S,
                     const rai::Configuration& startKinematics,
                     bool collisions);

void CG2komo(KOMO& komo,
             const SubCG& scg,
             const rai::Configuration& C,
             bool collisions);
