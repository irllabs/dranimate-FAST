#include <stdio.h>
#include <emscripten.h>
#include <igl/arap_dof.h>

extern "C" {

typedef float SSCALAR;
typedef Eigen::MatrixXd LbsMatrixType;

EMSCRIPTEN_KEEPALIVE
int createMesh() {
  // Test Verts
  const Eigen::MatrixXd vertices = (Eigen::MatrixXd(8,3)<<
    0.0,0.0,0.0,
    0.0,0.0,1.0,
    0.0,1.0,0.0,
    0.0,1.0,1.0,
    1.0,0.0,0.0,
    1.0,0.0,1.0,
    1.0,1.0,0.0,
    1.0,1.0,1.0).finished();

  // Test Faces
  const Eigen::MatrixXi faces = (Eigen::MatrixXi(12,3)<<
    1,7,5,
    1,3,7,
    1,4,3,
    1,2,4,
    3,8,7,
    3,4,8,
    5,7,8,
    5,8,6,
    1,5,6,
    1,6,2,
    2,6,8,
    2,8,4).finished().array()-1;

  // Handles
  LbsMatrixType handles;
 
  // Vertex Groups
  Eigen::Matrix<int,Eigen::Dynamic,1> groups;

  // Return data
  igl::ArapDOFData<LbsMatrixType,SSCALAR> arap_dof;

  // Precompute
  arap_dof_precomputation(vertices, faces, handles, groups, arap_dof);

  return 10;
}

EMSCRIPTEN_KEEPALIVE
int test() {
  return 1;
}

}
