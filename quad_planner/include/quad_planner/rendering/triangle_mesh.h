//==================================================
// triangle_mesh.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 20, 2016
//==================================================

#pragma once

#include <memory>
#include <Eigen/Dense>
#include <GL/glew.h>
#include <quad_planner/rendering/render_object.h>

namespace quad_planner
{
namespace rendering
{

struct TriangleMeshMatrixStorage;

struct TriangleMeshVectorStorage
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;

  explicit TriangleMeshVectorStorage();
  explicit TriangleMeshVectorStorage(const TriangleMeshMatrixStorage &tri_mesh_mat);
};

struct TriangleMeshMatrixStorage
{
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi triangles;

  explicit TriangleMeshMatrixStorage();
  explicit TriangleMeshMatrixStorage(const TriangleMeshVectorStorage &tri_mesh_mat);
};

class TriangleMeshStorageOgl
{
  GLuint vertex_buffer_id_;
  GLuint index_buffer_id_;
  GLsizei num_of_elements_;

  void init(const Eigen::MatrixXi &triangles, const Eigen::MatrixXd &vertices);

public:
  explicit TriangleMeshStorageOgl(const TriangleMeshMatrixStorage tri_mesh_mat);
  explicit TriangleMeshStorageOgl(const Eigen::MatrixXi &triangles, const Eigen::MatrixXd &vertices);
  virtual ~TriangleMeshStorageOgl();

  void bindVertexBuffer();
  void bindIndexBuffer();
  GLsizei getNumOfElements() const;
};

class TriangleMeshBuilder
{
public:
  TriangleMeshBuilder() = delete;
  TriangleMeshBuilder(const TriangleMeshBuilder&) = delete;
  TriangleMeshBuilder& operator=(const TriangleMeshBuilder&) = delete;

  static TriangleMeshVectorStorage createTriangleVec();
  static TriangleMeshMatrixStorage createTriangleMat();
  static TriangleMeshVectorStorage createIcosahedronVec(double radius);
  static TriangleMeshMatrixStorage createIcosahedronMat(double radius);
  static TriangleMeshVectorStorage createSphereVec(double radius, int subdivisions);
  static TriangleMeshMatrixStorage createSphereMat(double radius, int subdivisions);

  static TriangleMeshMatrixStorage convertToMatrixStorage(const TriangleMeshVectorStorage &tri_mesh_vec);
  static TriangleMeshVectorStorage convertToVectorStorage(const TriangleMeshMatrixStorage &tri_mesh_mat);

  static void subdivideMesh(TriangleMeshMatrixStorage *tri_mesh_mat);
};

class ColorTriangleMesh : public RenderObject
{
  GLuint vertex_array_id_;
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl_ptr_;
  GLuint color_buffer_id_;

  void initVAO();
  void enableVertexVBOs();
  void initColorVBO(const Eigen::MatrixXd &colors);
  void finishInit();

public:
  explicit ColorTriangleMesh(const std::shared_ptr<TriangleMeshStorageOgl> &tri_mesh_ogl_ptr, const Eigen::MatrixXd &colors);
  explicit ColorTriangleMesh(const TriangleMeshMatrixStorage tri_mesh_mat, const Eigen::MatrixXd &colors);
  explicit ColorTriangleMesh(const Eigen::MatrixXi &triangles, const Eigen::MatrixXd &vertices, const Eigen::MatrixXd &colors);
  virtual ~ColorTriangleMesh();

  void render() const;

  static std::shared_ptr<ColorTriangleMesh> createTriangle(const Eigen::Vector3d &color);
  static std::shared_ptr<ColorTriangleMesh> createIcosahedron(double radius, const Eigen::Vector3d &color);
  static std::shared_ptr<ColorTriangleMesh> createSphere(double radius, const Eigen::Vector3d &color, int subdivisions=10);
};

} /* namespace rendering */
} /* namespace quad_planner */
