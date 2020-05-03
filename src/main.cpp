#include <stdio.h>
#include <vector>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <Eigen/Core> 
#include <Eigen/SparseCore> 
#include <igl/cat.h>
#include <igl/bbw.h>
#include <igl/arap_dof.h>
#include <igl/lbs_matrix.h>
#include <igl/mat_max.h>
#include <igl/partition.h>
#include <igl/exterior_edges.h>
#include <igl/boundary_conditions.h>

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

    m_dims = dims;

    // Convert vertices from js
    std::vector<double> vecVertices = emscripten::vecFromJSArray<double>(jsVertices);
    m_vertices = 
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
      // Increase vertex dimensions 
      m_vertices.conservativeResize(m_vertices.rows(), 3);
      m_vertices.col(2).setZero(); 

      // Increase handle dimensions
      handles.conservativeResize(handles.rows(), 3);
      handles.col(2).setZero(); 
    }

    //std::cout << "Vertices " << m_vertices.rows() << " [" << m_vertices << "]" << std::endl;
    //std::cout << "Faces " << faces.rows() << " [" << faces << "]" << std::endl;
    //std::cout << "Handles " << handles.rows() << " [" << handles << "]" << std::endl;

    Eigen::VectorXi b;
    Eigen::MatrixXd bc;
    Eigen::VectorXi pointHandleIndicies(handles.rows());
    for (int i = 0; i < handles.rows(); i++) pointHandleIndicies(i) = i;

    //std::cout << "Point Handle Indicies [" << pointHandleIndicies << "]" << std::endl;

    // Compute boundary conditions for weight generation
    Eigen::VectorXi boundaryIndicies;
    Eigen::MatrixXd boundaryConditions;
    igl::boundary_conditions(
      m_vertices, 
      faces, 
      handles, 
      pointHandleIndicies, 
      Eigen::MatrixXi(), 
      Eigen::MatrixXi(), 
      boundaryIndicies, 
      boundaryConditions);

    //std::cout << "Generated Boundary Indicies [" << boundaryIndicies << "]" << std::endl;
    //std::cout << "Generated Boundary Conditions [" << boundaryConditions << "]" << std::endl;

    // Compute weights
    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 16;
    bbw_data.verbosity = 2;
    igl::bbw(
      m_vertices, 
      faces, 
      boundaryIndicies, 
      boundaryConditions, 
      bbw_data, 
      m_weights); 
    bbw_data.print();
    //std::cout << "Generated Weights [" << m_weights << "]" << std::endl;

    // Convert weights into FAST precompute format
    Eigen::MatrixXd weightsByHandle;
    igl::lbs_matrix_column(m_vertices, m_weights, weightsByHandle);

    // Create groups
    Eigen::VectorXi groups;
    {
      Eigen::VectorXi S;
      Eigen::VectorXd D;
      igl::partition(m_weights, 50, groups, S, D);
    }

    // Find indicies of handle vertices
    {
      Eigen::VectorXd maxWeight;
      igl::mat_max(m_weights, 1, maxWeight, m_handleIndicies);
    }

    // FAST precompute

    //std::cout << "Weights per handle: " << weightsByHandle << std::endl;
    //std::cout << "groups: " << groups << std::endl;

    igl::arap_dof_precomputation(
      m_vertices,
      faces,
      weightsByHandle,
      groups,
      m_arap_dof_data);

    // FAST initial recompute

    const int m = m_weights.cols();
    Eigen::SparseMatrix<double> constraints;
    constraints.resize(m*3,m*3*(3+1));
    std::vector<Eigen::Triplet<double>> ijv;
    for(int i=0; i<m; i++) {
      Eigen::RowVector4d homo;
      homo << m_vertices.row(m_handleIndicies(i)),1.;
      for(int d=0; d<3; d++) {
        for(int c=0; c<(3+1); c++) {
          ijv.push_back(Eigen::Triplet<double>(3*i + d,i + c*m*3 + d*m, homo(c)));
        }
      }
    }
    constraints.setFromTriplets(ijv.begin(),ijv.end());
    igl::arap_dof_recomputation(Eigen::VectorXi(), constraints, m_arap_dof_data);

    Eigen::MatrixXd Istack = Eigen::MatrixXd::Identity(3, 3+1).replicate(1, m);
    igl::columnize(Istack, m, 2, m_handleTransforms);


  }
  emscripten::val update(const emscripten::val &jsHandles) {
    // Convert handles from js 
    std::vector<double> vecHandles = emscripten::vecFromJSArray<double>(jsHandles);
    Eigen::MatrixXd handles = 
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vecHandles.data(), vecHandles.size()/m_dims, m_dims);
    Eigen::MatrixXd bcp(m_handleIndicies.size(),m_vertices.cols());
    Eigen::VectorXd Beq(3*m_handleIndicies.size());
    for(int i = 0;i<m_handleIndicies.size();i++) {
      bcp(i,0) = vecHandles[i*2+0];
      bcp(i,1) = vecHandles[i*2+1];
      bcp(i,2) = 0.0; 
      Beq(3*i+0) = bcp(i,0);
      Beq(3*i+1) = bcp(i,1);
      Beq(3*i+2) = bcp(i,2);
    }
    Eigen::MatrixXd L0 = m_handleTransforms;

    //std::cout << "m_arap_dof_data.dim: " << m_arap_dof_data.dim << std::endl;

    igl::arap_dof_update(m_arap_dof_data, Beq, L0, 30, 0, m_handleTransforms);

    std::vector<double> flatHandleTransforms;
    for(int r=0; r<m_handleTransforms.rows(); r++) {
      for(int c=0; c<m_handleTransforms.cols(); c++) {
        flatHandleTransforms.push_back(m_handleTransforms.coeff(r,c));
      }
    }

    //std::cout << "HTCols" << m_handleTransforms.cols() << std::endl;
    //std::cout << "HTRows" << m_handleTransforms.rows() << std::endl;

    return emscripten::val(emscripten::typed_memory_view(flatHandleTransforms.size(), flatHandleTransforms.data()));
  }
  emscripten::val getWeights() {
    // Return flat array of computed weights
    std::vector<double> flatWeights;
    for(int r=0; r<m_weights.rows(); r++) {
      for(int c=0; c<m_weights.cols(); c++) {
        flatWeights.push_back(m_weights.coeff(r,c));
      }
    }
    return emscripten::val(emscripten::typed_memory_view(flatWeights.size(), flatWeights.data()));
  }
private:
  void recompute() {
  }
  Eigen::MatrixXd m_weights;
  Eigen::VectorXi m_handleIndicies;
  igl::ArapDOFData<LbsMatrixType, LbsScalarType> m_arap_dof_data;
  Eigen::MatrixXd m_handleTransforms;
  Eigen::MatrixXd m_vertices;
  int m_dims;
};

// Export C++ class to JS
EMSCRIPTEN_BINDINGS(shape) {
  emscripten::class_<Shape>("Shape")
    .constructor<const emscripten::val &, const emscripten::val &, const emscripten::val &, int>()
    .function("getWeights", &Shape::getWeights)
    .function("update", &Shape::update);
}
