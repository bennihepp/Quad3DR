//==================================================
// triangle_mesh.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 20, 2016
//==================================================

#include <quad_planner/rendering/triangle_mesh.h>
#include <quad_planner/rendering/utilities.h>
#include <iostream>
#include <map>


namespace quad_planner
{
namespace rendering
{

TriangleMeshVectorStorage::TriangleMeshVectorStorage()
{
}

TriangleMeshVectorStorage::TriangleMeshVectorStorage(const TriangleMeshMatrixStorage &tri_mesh_mat)
{
  Utilities::convertMatrixToVector(tri_mesh_mat.vertices, &vertices);
  Utilities::convertMatrixToVector(tri_mesh_mat.triangles, &triangles);
}

TriangleMeshMatrixStorage::TriangleMeshMatrixStorage()
{
}

TriangleMeshMatrixStorage::TriangleMeshMatrixStorage(const TriangleMeshVectorStorage &tri_mesh_vec)
{
  Utilities::convertVectorToMatrix(tri_mesh_vec.vertices, &vertices);
  Utilities::convertVectorToMatrix(tri_mesh_vec.triangles, &triangles);
}


TriangleMeshVectorStorage TriangleMeshBuilder::createIcosahedronVec(double radius)
{
  assert(radius > 0.0);
  TriangleMeshVectorStorage tri_mesh_vec;
  std::vector<Eigen::Vector3i> &triangle_vec = tri_mesh_vec.triangles;
  std::vector<Eigen::Vector3d> &vertex_vec = tri_mesh_vec.vertices;

  // Golden ratio.
  double phi = (1.0 + std::sqrt(5)) / 2.0;
  // Compute vertex coordinates for Icosahedron/
  double b = 1.0 / std::sqrt(1.0 + phi * phi);
  double a = phi * b;
  // Scale coordinates.
  a *= radius;
  b *= radius;

  // Generate vertices.
  vertex_vec.push_back(Eigen::Vector3d( a,  b,  0));
  vertex_vec.push_back(Eigen::Vector3d(-a,  b,  0));
  vertex_vec.push_back(Eigen::Vector3d(-a, -b,  0));

  vertex_vec.push_back(Eigen::Vector3d( a, -b,  0));
  vertex_vec.push_back(Eigen::Vector3d( b,  0,  a));
  vertex_vec.push_back(Eigen::Vector3d( b,  0, -a));

  vertex_vec.push_back(Eigen::Vector3d(-b,  0, -a));
  vertex_vec.push_back(Eigen::Vector3d(-b,  0,  a));
  vertex_vec.push_back(Eigen::Vector3d( 0,  a,  b));

  vertex_vec.push_back(Eigen::Vector3d( 0, -a,  b));
  vertex_vec.push_back(Eigen::Vector3d( 0, -a, -b));
  vertex_vec.push_back(Eigen::Vector3d( 0,  a, -b));

  // Generate corresponding triangles.
  enum icosahedron_index {ZA, ZB, ZC, ZD, YA, YB, YC, YD, XA, XB, XC, XD};
  triangle_vec.push_back(Eigen::Vector3i(YA, XA, YD));
  triangle_vec.push_back(Eigen::Vector3i(YA, YD, XB));
  triangle_vec.push_back(Eigen::Vector3i(YB, YC, XD));
  triangle_vec.push_back(Eigen::Vector3i(YB, XC, YC));
  triangle_vec.push_back(Eigen::Vector3i(ZA, YA, ZD));
  triangle_vec.push_back(Eigen::Vector3i(ZA, ZD, YB));
  triangle_vec.push_back(Eigen::Vector3i(ZC, YD, ZB));
  triangle_vec.push_back(Eigen::Vector3i(ZC, ZB, YC));
  triangle_vec.push_back(Eigen::Vector3i(XA, ZA, XD));
  triangle_vec.push_back(Eigen::Vector3i(XA, XD, ZB));
  triangle_vec.push_back(Eigen::Vector3i(XB, XC, ZD));
  triangle_vec.push_back(Eigen::Vector3i(XB, ZC, XC));
  triangle_vec.push_back(Eigen::Vector3i(XA, YA, ZA));
  triangle_vec.push_back(Eigen::Vector3i(XD, ZA, YB));
  triangle_vec.push_back(Eigen::Vector3i(YA, XB, ZD));
  triangle_vec.push_back(Eigen::Vector3i(YB, ZD, XC));
  triangle_vec.push_back(Eigen::Vector3i(YD, XA, ZB));
  triangle_vec.push_back(Eigen::Vector3i(YC, ZB, XD));
  triangle_vec.push_back(Eigen::Vector3i(YD, ZC, XB));
  triangle_vec.push_back(Eigen::Vector3i(YC, XC, ZC));

  return tri_mesh_vec;
}

TriangleMeshMatrixStorage TriangleMeshBuilder::createIcosahedronMat(double radius)
{
  TriangleMeshVectorStorage tri_mesh_vec = createIcosahedronVec(radius);
  return TriangleMeshMatrixStorage(tri_mesh_vec);
}

TriangleMeshVectorStorage TriangleMeshBuilder::createSphereVec(double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = createSphereMat(radius, subdivisions);
  return TriangleMeshVectorStorage(tri_mesh_mat);
}

TriangleMeshMatrixStorage TriangleMeshBuilder::createSphereMat(double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshBuilder::createIcosahedronMat(radius);
  for (int level = 0; level < subdivisions; ++level)
  {
    TriangleMeshBuilder::subdivideMesh(&tri_mesh_mat);
    // Normalize vertices
    for (int i = 0; i < tri_mesh_mat.vertices.cols(); ++i)
    {
      Eigen::Vector3d vert = tri_mesh_mat.vertices.col(i);
      tri_mesh_mat.vertices.col(i) = vert / vert.norm();
    }
  }
  return tri_mesh_mat;
}


void TriangleMeshBuilder::subdivideMesh(TriangleMeshMatrixStorage *tri_mesh_mat)
{
  std::vector<Eigen::Vector3i> triangles;
  std::vector<Eigen::Vector3d> vertices;
  // Copy old vertices.
  for (int i = 0; i < tri_mesh_mat->vertices.cols(); ++i)
  {
    vertices.push_back(tri_mesh_mat->vertices.col(i));
  }

  // Divide each triangle and keep track of the new vertices between old vertices (connected by edges).
  using Edge = std::pair<int, int>;
  using EdgeToVertexIndexMap = std::map<Edge, int>;
  using FindReturnType = std::pair<EdgeToVertexIndexMap::iterator, bool>;
  EdgeToVertexIndexMap edgeToVertexIndexMap;
  for (int i = 0; i < tri_mesh_mat->triangles.cols(); ++i)
  {
    const Eigen::Vector3i &tri = tri_mesh_mat->triangles.col(i);
    int vert1_index = tri(0);
    int vert2_index = tri(1);
    int vert3_index = tri(2);
    const Eigen::Vector3d &vert1 = tri_mesh_mat->vertices.col(vert1_index);
    const Eigen::Vector3d &vert2 = tri_mesh_mat->vertices.col(vert2_index);
    const Eigen::Vector3d &vert3 = tri_mesh_mat->vertices.col(vert3_index);

    // This lambda searches for an edge entry in the map. If there is no entry it inserts the corresponding vertex and
    // returns the new index. Edges are uniquely identified by ordering the indices.
    auto findEdgeVertexIndexOrInsertLambda = [&vertices, &edgeToVertexIndexMap, tri_mesh_mat](int index_a, int index_b) -> int
    {
      FindReturnType find_ret;
      bool inserted;
      int found_vertex_index;
      if (index_a > index_b)
      {
        std::swap(index_a, index_b);
      }
      find_ret = edgeToVertexIndexMap.insert(std::make_pair(Edge(index_a, index_b), vertices.size()));
      inserted = find_ret.second;
      found_vertex_index = find_ret.first->second;
      int vert_ab_index;
      if (inserted)
      {
        const Eigen::Vector3d &vert_a = tri_mesh_mat->vertices.col(index_a);
        const Eigen::Vector3d &vert_b = tri_mesh_mat->vertices.col(index_b);
        vert_ab_index = vertices.size();
        Eigen::Vector3d vert_ab = (vert_a + vert_b) / 2;
        vertices.push_back(vert_ab);
      }
      else
      {
        vert_ab_index = found_vertex_index;
      }
      return vert_ab_index;
    };

    int vert12_index = findEdgeVertexIndexOrInsertLambda(tri(0), tri(1));
    int vert23_index = findEdgeVertexIndexOrInsertLambda(tri(1), tri(2));
    int vert31_index = findEdgeVertexIndexOrInsertLambda(tri(2), tri(0));

    // Insert the four new triangles resulting from the split of one triangle.
    triangles.push_back(Eigen::Vector3i(vert1_index, vert12_index, vert31_index));
    triangles.push_back(Eigen::Vector3i(vert12_index, vert2_index, vert23_index));
    triangles.push_back(Eigen::Vector3i(vert23_index, vert3_index, vert31_index));
    triangles.push_back(Eigen::Vector3i(vert12_index, vert23_index, vert31_index));
  }

  Utilities::convertVectorToMatrix(triangles, &tri_mesh_mat->triangles);
  Utilities::convertVectorToMatrix(vertices, &tri_mesh_mat->vertices);
}



TriangleMeshStorageOgl::TriangleMeshStorageOgl(const TriangleMeshMatrixStorage tri_mesh_mat)
{
  init(tri_mesh_mat.triangles, tri_mesh_mat.vertices);
}

TriangleMeshStorageOgl::TriangleMeshStorageOgl(const Eigen::MatrixXi &triangles, const Eigen::MatrixXd &vertices)
{
  init(triangles, vertices);
}

TriangleMeshStorageOgl::~TriangleMeshStorageOgl()
{
  glDeleteBuffers(1, &index_buffer_id_);
  glDeleteBuffers(1, &vertex_buffer_id_);
}

void TriangleMeshStorageOgl::init(const Eigen::MatrixXi &triangles, const Eigen::MatrixXd &vertices)
{
  //  std::cout << "Mesh has " << (triangles.rows()) << " triangles and "
  //            << (vertices.size()) << " vertices and "
  //            << (colors.size()) << " colors and "
  //            << (vertices.size() * 3) << " values" << std::endl;
  //    std::cout << triangles << std::endl;
  //    std::cout << vertices << std::endl;
  //    for (int i=0; i < vertices.size(); ++i)
  //    {
  //      std::cout << "i: " << vertices.data()[i] << std::endl;
  //    }

  // Vertex attribute buffer
  glGenBuffers(1, &vertex_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLdouble), vertices.data(), GL_STATIC_DRAW);

  // Index buffer
  glGenBuffers(1, &index_buffer_id_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.size() * sizeof(GLuint), triangles.data(), GL_STATIC_DRAW);
  num_of_elements_ = triangles.size();
}

void TriangleMeshStorageOgl::bindVertexBuffer()
{
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
}

void TriangleMeshStorageOgl::bindIndexBuffer()
{
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
}

GLsizei TriangleMeshStorageOgl::getNumOfElements() const
{
  return num_of_elements_;
}


ColorTriangleMesh::ColorTriangleMesh(const std::shared_ptr<TriangleMeshStorageOgl> &tri_mesh_ogl_ptr, const Eigen::MatrixXd &colors)
: tri_mesh_ogl_ptr_(tri_mesh_ogl_ptr)
{
  initVAO();
  initColorVBO(colors);
  enableVertexVBOs();
  finishInit();
}

ColorTriangleMesh::ColorTriangleMesh(const TriangleMeshMatrixStorage tri_mesh_mat, const Eigen::MatrixXd &colors)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(tri_mesh_mat.triangles, tri_mesh_mat.vertices);
  enableVertexVBOs();
  initColorVBO(colors);
  finishInit();
}

ColorTriangleMesh::ColorTriangleMesh(const Eigen::MatrixXi &triangles, const Eigen::MatrixXd &vertices, const Eigen::MatrixXd &colors)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(triangles, vertices);
  enableVertexVBOs();
  initColorVBO(colors);
  finishInit();
}

ColorTriangleMesh::~ColorTriangleMesh()
{
  glDeleteBuffers(1, &color_buffer_id_);
  glDeleteVertexArrays(1, &vertex_array_id_);
}

void ColorTriangleMesh::initVAO()
{
  // Setup vertex arrays
  glGenVertexArrays(1, &vertex_array_id_);
  glBindVertexArray(vertex_array_id_);
}

void ColorTriangleMesh::enableVertexVBOs()
{
  // Vertex attribute buffer
  tri_mesh_ogl_ptr_->bindVertexBuffer();
  glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(0);

  // Index buffer
  tri_mesh_ogl_ptr_->bindIndexBuffer();
}

void ColorTriangleMesh::initColorVBO(const Eigen::MatrixXd &colors)
{
  // Color attribute buffer
  glGenBuffers(1, &color_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, color_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLdouble), colors.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(1);
}

void ColorTriangleMesh::finishInit()
{
  glBindVertexArray(0);
}

void ColorTriangleMesh::render() const
{
  glBindVertexArray(vertex_array_id_);
  glDrawElements(GL_TRIANGLES, tri_mesh_ogl_ptr_->getNumOfElements(), GL_UNSIGNED_INT, nullptr);
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createTriangle(const Eigen::Vector3d &color)
{
  Eigen::MatrixXi triangles(1, 3);
  Eigen::MatrixXd vertices(3, 3);
  Eigen::MatrixXd colors(3, 3);
  triangles(0) = 0;
  triangles(1) = 1;
  triangles(2) = 2;
  vertices.row(0)(0) = -1.0f;
  vertices.row(0)(1) =  0.0f;
  vertices.row(0)(2) =  0.0f;
  vertices.row(1)(0) = +1.0f;
  vertices.row(1)(1) =  0.0f;
  vertices.row(1)(2) =  0.0f;
  vertices.row(2)(0) =  0.0f;
  vertices.row(2)(1) =  1.0f;
  vertices.row(2)(2) =  0.0f;
  colors.col(0) = color;
  colors.col(1) = color;
  colors.col(2) = color;
  return std::make_shared<ColorTriangleMesh>(triangles.transpose(), vertices.transpose(), colors.transpose());
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createIcosahedron(double radius, const Eigen::Vector3d &color)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshBuilder::createIcosahedronMat(radius);
  Eigen::MatrixXd colors(3, tri_mesh_mat.vertices.cols());
  for (int i = 0; i < colors.cols(); ++i)
  {
    colors.col(i) = color;
  }
  return std::make_shared<ColorTriangleMesh>(tri_mesh_mat, colors);
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createSphere(double radius, const Eigen::Vector3d &color, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshBuilder::createSphereMat(radius, subdivisions);
  Eigen::MatrixXd colors(3, tri_mesh_mat.vertices.cols());
  for (int i = 0; i < colors.cols(); ++i)
  {
    colors.col(i) = color;
  }
  return std::make_shared<ColorTriangleMesh>(tri_mesh_mat, colors);
}

} /* namespace rendering */
} /* namespace quad_planner */
