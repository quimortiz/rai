/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

struct aiNode;
struct aiMesh;
struct aiScene;

namespace rai {

struct AssimpLoader {
  rai::Array<MeshA> meshes;
  rai::Array<rai::Transformation> poses;
  StringA names;
  StringA parents;
  std::string directory;
  int verbose=0;

  AssimpLoader(std::string const& path, bool flipYZ=true, bool relativeMeshPoses=false);
  AssimpLoader(const aiScene* scene);

  rai::Mesh getSingleMesh();

 private:
  void loadNode(const aiNode* node, const aiScene* scene, arr T, bool relativeMeshPoses);
  rai::Mesh loadMesh(const aiMesh* mesh, const aiScene* scene);
};

void buildAiMesh(const rai::Mesh& M, aiMesh* pMesh);
void writeAssimp(const rai::Mesh& M, const char* filename, const char* format);

} //namespace
