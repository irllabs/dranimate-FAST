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
  Shape(const emscripten::val &jsVertices, const emscripten::val &jsFaces, int dims) {
		// Store dims
    m_dims = dims;
    // Convert vertices from js
    std::vector<double> vecVertices = emscripten::vecFromJSArray<double>(jsVertices);
    m_vertices = 
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vecVertices.data(), vecVertices.size()/dims, dims);
    // Convert faces from js
    std::vector<int> vecFaces = emscripten::vecFromJSArray<int>(jsFaces);
    m_faces = 
      Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vecFaces.data(), vecFaces.size()/3,3);
  }
  void addControlPoint(int vertexIndex) {
    //std::cout << "Add control point: " << vertexIndex << std::endl;
		int size = m_controlPointIndicies.size();
		m_controlPointIndicies.conservativeResize(size+1);
		m_controlPointIndicies(size) = vertexIndex;
    m_controlPointsFlat.push_back(0.0);
    m_controlPointsFlat.push_back(0.0);
  }
  void setControlPointPosition(int controlPointIndex, double x, double y) {
    //std::cout << "Set control point " << controlPointIndex << " position: (" << x << "," << y << ")" << std::endl;
    m_controlPointsFlat[controlPointIndex*2+0] = x;
    m_controlPointsFlat[controlPointIndex*2+1] = y;
  }
  void precompute() {
    m_controlPoints = 
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(m_controlPointsFlat.data(), m_controlPointsFlat.size()/m_dims, m_dims);

    // If 2D convert to 3D
    if(m_dims == 2) {
      // Increase vertex dimensions 
      m_vertices.conservativeResize(m_vertices.rows(), 3);
      m_vertices.col(2).setZero(); 

      // Increase handle dimensions
      m_controlPoints.conservativeResize(m_controlPoints.rows(), 3);
      m_controlPoints.col(2).setZero(); 
    }

   	//std::cout << "Vertices " << m_vertices.rows() << " [" << m_vertices << "]" << std::endl;
    //std::cout << "Faces " << m_faces.rows() << " [" << m_faces << "]" << std::endl;
    //std::cout << "Control Points " << " [" << m_controlPoints << "]" << std::endl;

    m_transformedVertices = m_vertices;

    Eigen::VectorXi b;
    Eigen::MatrixXd bc;
    Eigen::VectorXi pointHandleIndicies(m_controlPoints.rows());
    for (int i = 0; i < m_controlPoints.rows(); i++) pointHandleIndicies(i) = i;

    //std::cout << "Point Handle Indicies [" << pointHandleIndicies << "]" << std::endl;

    // Compute boundary conditions for weight generation
    Eigen::VectorXi boundaryIndicies;
    Eigen::MatrixXd boundaryConditions;
    igl::boundary_conditions(
      m_vertices, 
      m_faces, 
      m_controlPoints, 
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
      m_faces, 
      boundaryIndicies, 
      boundaryConditions, 
      bbw_data, 
      m_weights); 

    //bbw_data.print();
    //std::cout << "Generated Weights [" << m_weights << "]" << std::endl;

		// If we dont do this lbs_matrix_column leaves us with uninitialized (NaN) values
		int dim = m_vertices.cols();
		int lbsRows = m_vertices.rows()*dim;
		int lbsCols = m_weights.cols()*dim*(dim+1);
		m_lbsMatrix.resize(lbsRows, lbsCols);
		m_lbsMatrix.setZero(lbsRows, lbsCols);

    // Create linear blend skinning matrix from vertices and weights
    igl::lbs_matrix_column(m_vertices, m_weights, m_lbsMatrix);

		//std::cout << "LBS: " << m_lbsMatrix << std::endl;
 
    // Create groups
    Eigen::VectorXi groups;
    {
      Eigen::VectorXi S;
      Eigen::VectorXd D;
      igl::partition(m_weights, 50, groups, S, D);
    }

		//std::cout << "Control Point Indicies: " << m_controlPointIndicies << std::endl;

    //std::cout << "LBS: " << m_lbsMatrix << std::endl;
    //std::cout << "groups: " << groups << std::endl;
    
    // FAST precompute
    igl::arap_dof_precomputation(
      m_vertices,
      m_faces,
      m_lbsMatrix,
      groups,
      m_arap_dof_data);

    // FAST initial recompute
    const int m = m_weights.cols();
    Eigen::SparseMatrix<double> constraints;
    constraints.resize(m*3,m*3*(3+1));
    std::vector<Eigen::Triplet<double>> ijv;
    for(int i=0; i<m; i++) {
      Eigen::RowVector4d homo;
      homo << m_vertices.row(m_controlPointIndicies(i)),1.;
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
  emscripten::val update() {
    Eigen::MatrixXd bcp(m_controlPointIndicies.size(),m_vertices.cols());
    Eigen::VectorXd beq(3*m_controlPointIndicies.size());
    for(int i = 0;i<m_controlPointIndicies.size();i++) {
      bcp(i,0) = m_controlPointsFlat[i*2+0];
      bcp(i,1) = m_controlPointsFlat[i*2+1];
      bcp(i,2) = 0.0; 
      beq(3*i+0) = bcp(i,0);
      beq(3*i+1) = bcp(i,1);
      beq(3*i+2) = bcp(i,2);
    }
    Eigen::MatrixXd L0 = m_handleTransforms;

		//std::cout << "BCP " << bcp << std::endl; 
		//std::cout << "BEQ " << beq << std::endl; 

    igl::arap_dof_update(m_arap_dof_data, beq, L0, 30, 0, m_handleTransforms);

    const auto & Ucol = m_lbsMatrix * m_handleTransforms;

    m_transformedVertices.col(0) = Ucol.block(0*m_transformedVertices.rows(),0,m_transformedVertices.rows(),1);
    m_transformedVertices.col(1) = Ucol.block(1*m_transformedVertices.rows(),0,m_transformedVertices.rows(),1);
    m_transformedVertices.col(2) = Ucol.block(2*m_transformedVertices.rows(),0,m_transformedVertices.rows(),1);

    //std::cout << "TRANSFORMED VERTS " << m_transformedVertices << std::endl;

    //std::cout << "--" << std::endl;

    m_flatTransformedVertices.clear();

    for(int tv=0; tv<m_transformedVertices.rows(); tv++) {
      double x = m_transformedVertices(tv,0);
      double y = m_transformedVertices(tv,1);
      double z = m_transformedVertices(tv,2);
     
      if(x != x) x = 0.0;
      if(y != y) y = 0.0;
      if(z != z) z = 0.0;

      m_flatTransformedVertices.push_back(x);
      m_flatTransformedVertices.push_back(y);
      m_flatTransformedVertices.push_back(z);
    }

    return emscripten::val(emscripten::typed_memory_view(m_flatTransformedVertices.size(), m_flatTransformedVertices.data()));
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
  Eigen::MatrixXd m_weights;
  Eigen::VectorXi m_controlPointIndicies;
  igl::ArapDOFData<LbsMatrixType, LbsScalarType> m_arap_dof_data;
  Eigen::MatrixXd m_handleTransforms;
  Eigen::MatrixXd m_vertices;
  Eigen::MatrixXd m_controlPoints;
	Eigen::MatrixXi m_faces; 
  Eigen::MatrixXd m_transformedVertices;
  std::vector<double> m_flatTransformedVertices;
  std::vector<double> m_controlPointsFlat;
  Eigen::MatrixXd m_lbsMatrix;
  int m_dims;
};

// Export C++ class to JS
EMSCRIPTEN_BINDINGS(shape) {
  emscripten::class_<Shape>("Shape")
    .constructor<const emscripten::val &, const emscripten::val &, int>()
    .function("addControlPoint", &Shape::addControlPoint)
    .function("setControlPointPosition", &Shape::setControlPointPosition)
    .function("precompute", &Shape::precompute)
    .function("getWeights", &Shape::getWeights)
    .function("update", &Shape::update);
}
