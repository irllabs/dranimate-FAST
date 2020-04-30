#include <stdio.h>
#include <vector>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <Eigen/Core> 
#include <igl/cat.h>
#include <igl/bbw.h>
#include <igl/arap_dof.h>
#include <igl/lbs_matrix.h>
#include <igl/partition.h>
#include <igl/exterior_edges.h>
#include <igl/boundary_conditions.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>

// Linear blend skinning scalar types
typedef float LbsScalarType; 
typedef Eigen::MatrixXd LbsMatrixType;

class Shape {
public:
  // CTOR
  Shape(
    const emscripten::val &jsVertices,
    const emscripten::val &jsFaces,
    const emscripten::val &jsHandles,
    int dims) {
    // Convert vertices from js
    std::vector<double> vecVertices = emscripten::vecFromJSArray<double>(jsVertices);
    Eigen::MatrixXd vertices = 
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vecVertices.data(), vecVertices.size()/dims, dims);

    // Convert faces from js
    std::vector<int> vecFaces = emscripten::vecFromJSArray<int>(jsFaces);
    Eigen::MatrixXi faces = 
      Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vecFaces.data(), vecFaces.size()/3,3);

    // Convert handles from js 
    std::vector<double> vecHandles = emscripten::vecFromJSArray<double>(jsHandles);
    Eigen::MatrixXd handles = 
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vecHandles.data(), vecHandles.size()/dims, dims);

    // If 2D convert to 3D
    if(dims == 2) {
      // Increase vertex dimenstions 
      vertices.conservativeResize(vertices.rows(), 3);
      vertices.col(2).setZero(); 
      // Increase handle dimenstions
      handles.conservativeResize(handles.rows(), 3);
      handles.col(2).setZero(); 
    }

    std::cout << "Vertices " << vertices.rows() << " [" << vertices << "]" << std::endl;
    std::cout << "Faces " << faces.rows() << " [" << faces << "]" << std::endl;
    std::cout << "Handles " << handles.rows() << " [" << handles << "]" << std::endl;

    Eigen::VectorXi b;
    Eigen::MatrixXd bc;
    Eigen::VectorXi pointHandleIndicies(handles.rows());
    for (int i = 0; i < handles.rows(); i++) pointHandleIndicies(i) = i;
    std::cout << "Point Handle Indicies [" << pointHandleIndicies << "]" << std::endl;

    Eigen::VectorXi boundaryIndicies;
    Eigen::MatrixXd boundaryConditions;
    igl::boundary_conditions(
      vertices, 
      faces, 
      handles, 
      pointHandleIndicies, 
      Eigen::MatrixXi(), 
      Eigen::MatrixXi(), 
      boundaryIndicies, 
      boundaryConditions);
    std::cout << "Generated Boundary Indicies [" << boundaryIndicies << "]" << std::endl;
    std::cout << "Generated Boundary Conditions [" << boundaryConditions << "]" << std::endl;

    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 16;
    bbw_data.verbosity = 2;
    igl::bbw(
      vertices, 
      faces, 
      boundaryIndicies, 
      boundaryConditions, 
      bbw_data, 
      m_weights); 
    bbw_data.print();
    std::cout << "Generated Weights [" << m_weights << "]" << std::endl;

    // Vertex Groups
    //Eigen::Matrix<int,Eigen::Dynamic,1> groups;
    // Return data
    igl::ArapDOFData<LbsMatrixType,float> arap_dof;

    // Convert Weights
    Eigen::MatrixXd M;
    igl::lbs_matrix_column(vertices, m_weights, M);
    // Create groups
    Eigen::VectorXi groups;
    {
      Eigen::VectorXi S;
      Eigen::VectorXd D;
      igl::partition(m_weights,50,groups,S,D); 
    }
    igl::arap_dof_precomputation(vertices,faces,M,groups,m_arap_dof_data);
  }
  void recompute(void) {
  }
  void update(void) {
  }
  emscripten::val getWeights() {
    std::vector<double> flatWeights;
    for(int r=0; r<m_weights.rows(); r++) {
      for(int c=0; c<m_weights.cols(); c++) {
        flatWeights.push_back(m_weights.coeff(r,c));
      }
    }
    return emscripten::val(emscripten::typed_memory_view(flatWeights.size(), flatWeights.data()));
  }
private:
  void tetrahedralize(
    Eigen::MatrixXd vertices, 
    Eigen::MatrixXi faces,
    Eigen::MatrixXd tetVertices,
    Eigen::MatrixXi tetFaces,
    Eigen::MatrixXi tetBoundryFaces) {
    std::cout << "Tetrahedralize" << std::endl; 
    igl::copyleft::tetgen::tetrahedralize(vertices, faces, "Ypq100", tetVertices, tetFaces, tetBoundryFaces);
  }
  void computeWeights(Eigen::MatrixXd vertices, Eigen::MatrixXi faces) {
    /*
    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 8;
    bbw_data.verbosity = 2;
    */ 
    //if(!igl::bbw(vertices, faces
  }  
  igl::ArapDOFData<LbsMatrixType, LbsScalarType> m_arap_dof_data; 
  Eigen::MatrixXd m_weights;
};

EMSCRIPTEN_BINDINGS(shape) {
  emscripten::class_<Shape>("Shape")
    .constructor<const emscripten::val &, const emscripten::val &, const emscripten::val &, int>()
    .function("getWeights", &Shape::getWeights)
    .function("recompute", &Shape::recompute)
    .function("update", &Shape::update);
}

/*
      //for(int v=0; v<vertices.size(); v++) {
      //  std::cout << vertices.at(v) << std::endl; 
      //} 

EMSCRIPTEN_KEEPALIVE
int createShape() {
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
  //arap_dof_precomputation(vertices, faces, handles, groups, arap_dof);
  std::cout << vertices << "\n\n"; 
  std::cout << faces; 

  return 10;
}

EMSCRIPTEN_KEEPALIVE
int test() {
  return 1;
}
*/
