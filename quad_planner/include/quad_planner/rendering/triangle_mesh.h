//==================================================
// triangle_mesh.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 20, 2016
//==================================================

#pragma once

#include <map>
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
  std::vector<Eigen::Vector3d> normals;
  std::vector<Eigen::Vector3i> triangles;

  explicit TriangleMeshVectorStorage();
  explicit TriangleMeshVectorStorage(const TriangleMeshMatrixStorage& tri_mesh_mat);

  //! Add vertex (if not already existing) and return corresponding index.
  int find_or_add_vertex(const Eigen::Vector3d& vertex, const Eigen::Vector3d& normal);
  //! Add triangle (if not already existing) and return corresponding index.
  int find_or_add_triangle(int vertexId1, int vertexId2, int vertexId3);
  int find_or_add_triangle(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const Eigen::Vector3d& vec3);

  void compute_averaged_normals();

  static void compute_averaged_normals(const std::vector<Eigen::Vector3i>& triangles, const std::vector<Eigen::Vector3d>& vertices, std::vector<Eigen::Vector3d>* normals);
  static std::vector<Eigen::Vector3d> compute_averaged_normals(const std::vector<Eigen::Vector3i>& triangles, const std::vector<Eigen::Vector3d>& vertices);
};

struct TriangleMeshMatrixStorage
{
  Eigen::MatrixXd vertices;
  Eigen::MatrixXd normals;
  Eigen::MatrixXi triangles;

  explicit TriangleMeshMatrixStorage();
  explicit TriangleMeshMatrixStorage(const TriangleMeshVectorStorage& tri_mesh_mat);
};

class TriangleMeshStorageOgl
{
  GLuint vertex_buffer_id_;
  GLuint index_buffer_id_;
  GLuint normal_buffer_id_;
  GLsizei num_of_elements_;

  void init(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices);
  void init(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& normals);

public:
  explicit TriangleMeshStorageOgl(const TriangleMeshMatrixStorage& tri_mesh_mat);
  explicit TriangleMeshStorageOgl(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices);
  explicit TriangleMeshStorageOgl(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& normals);
  virtual ~TriangleMeshStorageOgl();

  void bindVertexBuffer();
  void bindNormalBuffer();
  void bindIndexBuffer();
  GLsizei getNumOfElements() const;
};

class TriangleMeshFactory
{
  static std::map<int, std::weak_ptr<TriangleMeshStorageOgl>> unit_sphere_map_;
  static std::map<std::pair<double, int>, std::weak_ptr<TriangleMeshStorageOgl>> unit_radius_cylinder_map_;

public:
  TriangleMeshFactory() = delete;
  TriangleMeshFactory(const TriangleMeshFactory&) = delete;
  TriangleMeshFactory& operator=(const TriangleMeshFactory&) = delete;

  static TriangleMeshVectorStorage createTriangleVec();
  static TriangleMeshMatrixStorage createTriangleMat();
  static std::shared_ptr<TriangleMeshStorageOgl> createTriangleOgl();
  static TriangleMeshVectorStorage createIcosahedronVec(double radius);
  static TriangleMeshMatrixStorage createIcosahedronMat(double radius);
  static std::shared_ptr<TriangleMeshStorageOgl> createIcosahedronOgl(double radius);
  static TriangleMeshVectorStorage createTriangularPrismVec(double height, double radius);
  static TriangleMeshMatrixStorage createTriangularPrismMat(double height, double radius);
  static std::shared_ptr<TriangleMeshStorageOgl> createTriangularPrismOgl(double height, double radius);
  static TriangleMeshVectorStorage createSphereVec(double radius, int subdivisions);
  static TriangleMeshMatrixStorage createSphereMat(double radius, int subdivisions);
  static std::shared_ptr<TriangleMeshStorageOgl> createSphereOgl(double radius, int subdivisions);
  static std::shared_ptr<TriangleMeshStorageOgl> createUnitSphereOgl(int subdivisions);
  static TriangleMeshVectorStorage createCylinderVec(double height, double radius, int subdivisions);
  static TriangleMeshMatrixStorage createCylinderMat(double height, double radius, int subdivisions);
  static std::shared_ptr<TriangleMeshStorageOgl> createCylinderOgl(double height, double radius, int subdivisions);
  static std::shared_ptr<TriangleMeshStorageOgl> createUnitRadiusCylinderOgl(double cylinder_length, int subdivisions);

  static TriangleMeshMatrixStorage convertToMatrixStorage(const TriangleMeshVectorStorage& tri_mesh_vec);
  static TriangleMeshVectorStorage convertToVectorStorage(const TriangleMeshMatrixStorage& tri_mesh_mat);

  static void subdivideMesh(TriangleMeshMatrixStorage *tri_mesh_mat);
};

class TriangleMesh : public RenderObject
{
  GLuint vertex_array_id_;
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl_ptr_;

  void initVAO();
  void enableVertexVBOs();
  void finishInit();

public:
  explicit TriangleMesh(const std::shared_ptr<TriangleMeshStorageOgl>& tri_mesh_ogl_ptr);
  explicit TriangleMesh(const TriangleMeshMatrixStorage& tri_mesh_mat);
  explicit TriangleMesh(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices);
  explicit TriangleMesh(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& normals);
  virtual ~TriangleMesh();

  void render() const;

  static std::shared_ptr<TriangleMesh> createTriangle();
  static std::shared_ptr<TriangleMesh> createIcosahedron(double radius);
  static std::shared_ptr<TriangleMesh> createSphere(double radius, int subdivisions=10);
  static std::shared_ptr<TriangleMesh> createUnitSphere(int subdivisions=10);
  static std::shared_ptr<TriangleMesh> createUnitRadiusCylinder(double cylinder_length, int subdivisions=10);
};

class ColorTriangleMesh : public RenderObject
{
  GLuint vertex_array_id_;
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl_ptr_;
  GLuint color_buffer_id_;

  void initVAO();
  void enableVertexVBOs();
  void initColorVBO(const Eigen::MatrixXd& colors);
  void finishInit();

public:
  explicit ColorTriangleMesh(const std::shared_ptr<TriangleMeshStorageOgl>& tri_mesh_ogl_ptr, const Eigen::MatrixXd& colors);
  explicit ColorTriangleMesh(const TriangleMeshMatrixStorage& tri_mesh_mat, const Eigen::MatrixXd& colors);
  explicit ColorTriangleMesh(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& colors);
  virtual ~ColorTriangleMesh();

  void render() const;

  static std::shared_ptr<ColorTriangleMesh> createTriangle(const Eigen::Vector3d& color);
  static std::shared_ptr<ColorTriangleMesh> createIcosahedron(double radius, const Eigen::Vector3d& color);
  static std::shared_ptr<ColorTriangleMesh> createSphere(double radius, const Eigen::Vector3d& color, int subdivisions=10);
  static std::shared_ptr<ColorTriangleMesh> createUnitSphere(const Eigen::Vector3d& color, int subdivisions=10);
  static std::shared_ptr<ColorTriangleMesh> createUnitRadiusCylinder(double cylinder_length, const Eigen::Vector3d& color, int subdivisions=10);
};

} /* namespace rendering */
} /* namespace quad_planner */
