//==================================================
// lines.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Oct 16, 2016
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

struct Lines3DMatrixStorage;

struct Lines3DVectorStorage
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> lines;

  explicit Lines3DVectorStorage();
  explicit Lines3DVectorStorage(const Lines3DMatrixStorage& lines_mat);

  //! Add vertex (if not already existing) and return corresponding index.
  int find_or_add_vertex(const Eigen::Vector3d& vertex);
  //! Add line (if not already existing) and return corresponding index.
  int find_or_add_line(int vertexId1, int vertexId2);
  int find_or_add_line(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);
};

struct Lines3DMatrixStorage
{
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi lines;

  explicit Lines3DMatrixStorage();
  explicit Lines3DMatrixStorage(const Lines3DVectorStorage& lines_vec);
};

class Lines3DStorageOgl
{
  GLuint vertex_buffer_id_;
  GLuint index_buffer_id_;
  GLsizei num_of_elements_;

  void init(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices);

public:
  explicit Lines3DStorageOgl(const Lines3DMatrixStorage& lines_mat);
  explicit Lines3DStorageOgl(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices);
  virtual ~Lines3DStorageOgl();

  void bindVertexBuffer();
  void bindIndexBuffer();
  GLsizei getNumOfElements() const;
};

class Lines3D : public RenderObject
{
  GLuint vertex_array_id_;
  std::shared_ptr<Lines3DStorageOgl> lines_ogl_ptr_;

  void init(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices);
  void initVAO();
  void enableVertexVBOs();
  void finishInit();

public:
  explicit Lines3D(const std::shared_ptr<Lines3DStorageOgl>& lines_ogl_ptr);
  explicit Lines3D(const Lines3DMatrixStorage& lines_mat);
  explicit Lines3D(const Lines3DVectorStorage& lines_vec);
  explicit Lines3D(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices);
  virtual ~Lines3D();

  void render() const;

  static std::shared_ptr<Lines3D> createLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
};

class ColorLines3D : public RenderObject
{
  GLuint vertex_array_id_;
  std::shared_ptr<Lines3DStorageOgl> lines_ogl_ptr_;
  GLuint color_buffer_id_;

  void init(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& colors);
  void initVAO();
  void enableVertexVBOs();
  void initColorVBO(const Eigen::MatrixXd& colors);
  void finishInit();

public:
  explicit ColorLines3D(const std::shared_ptr<Lines3DStorageOgl>& lines_ogl_ptr, const Eigen::MatrixXd& colors);
  explicit ColorLines3D(const Lines3DMatrixStorage& lines_mat, const Eigen::MatrixXd& colors);
  explicit ColorLines3D(const Lines3DVectorStorage& lines_vec, const Eigen::MatrixXd& colors);
  explicit ColorLines3D(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& colors);
  virtual ~ColorLines3D();

  void render() const;

  static std::shared_ptr<ColorLines3D> createLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& color);
};

} /* namespace rendering */
} /* namespace quad_planner */
