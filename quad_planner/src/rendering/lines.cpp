//==================================================
// lines.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Oct 16, 2016
//==================================================

#include <quad_planner/rendering/lines.h>
#include <quad_planner/rendering/utilities.h>


namespace quad_planner
{
namespace rendering
{

Lines3DVectorStorage::Lines3DVectorStorage()
{
}

Lines3DVectorStorage::Lines3DVectorStorage(const Lines3DMatrixStorage& lines_mat)
{
  Utilities::convertMatrixToVector(lines_mat.vertices, &vertices);
  Utilities::convertMatrixToVector(lines_mat.lines, &lines);
}

int Lines3DVectorStorage::find_or_add_vertex(const Eigen::Vector3d& vertex)
{
  for (auto it = vertices.cbegin(); it != vertices.cend(); ++it) {
    if (*it == vertex) {
      return it - vertices.cbegin();
    }
  }
  vertices.push_back(vertex);
  return vertices.size() - 1;
}

int Lines3DVectorStorage::find_or_add_line(int vertexId1, int vertexId2)
{
  Eigen::Vector3i line1(vertexId1, vertexId2);
  Eigen::Vector3i line2(vertexId2, vertexId1);
  for (auto it = lines.cbegin(); it != lines.cend(); ++it) {
    if (*it == line1 || *it == line2) {
      return it - lines.cbegin();
    }
  }
  lines.push_back(line1);
  return lines.size() - 1;
}

int Lines3DVectorStorage::find_or_add_line(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2)
{
  int a = find_or_add_vertex(vec1);
  int b = find_or_add_vertex(vec2);
  return find_or_add_line(a, b);
}

Lines3DMatrixStorage::Lines3DMatrixStorage()
{
}

Lines3DMatrixStorage::Lines3DMatrixStorage(const Lines3DVectorStorage& lines_vec)
{
  Utilities::convertVectorToMatrix(lines_vec.lines, &lines);
  Utilities::convertVectorToMatrix(lines_vec.vertices, &vertices);
}


Lines3DStorageOgl::Lines3DStorageOgl(const Lines3DMatrixStorage& lines_mat)
{
  init(lines_mat.lines, lines_mat.vertices);
}

Lines3DStorageOgl::Lines3DStorageOgl(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices)
{
  init(lines, vertices);
}

Lines3DStorageOgl::~Lines3DStorageOgl()
{
  glDeleteBuffers(1, &index_buffer_id_);
  glDeleteBuffers(1, &vertex_buffer_id_);
}

void Lines3DStorageOgl::init(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices)
{
  assert(lines.rows() == 2);
  assert(vertices.rows() == 3);

  //  std::cout << "Lines3D has " << (lines.rows()) << " lines and "
  //            << (vertices.size()) << " vertices and "
  //            << (vertices.size() * 3) << " values" << std::endl;
  //    std::cout << lines << std::endl;
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
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, lines.size() * sizeof(GLuint), lines.data(), GL_STATIC_DRAW);
  num_of_elements_ = lines.size();
}

void Lines3DStorageOgl::bindVertexBuffer()
{
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
}

void Lines3DStorageOgl::bindIndexBuffer()
{
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
}

GLsizei Lines3DStorageOgl::getNumOfElements() const
{
  return num_of_elements_;
}


Lines3D::Lines3D(const std::shared_ptr<Lines3DStorageOgl>& lines_ogl_ptr)
: lines_ogl_ptr_(lines_ogl_ptr)
{
  initVAO();
  enableVertexVBOs();
  finishInit();
}

Lines3D::Lines3D(const Lines3DMatrixStorage& lines_mat)
{
  init(lines_mat.lines, lines_mat.vertices);
}

Lines3D::Lines3D(const Lines3DVectorStorage& lines_vec)
{
  Lines3DMatrixStorage lines_mat(lines_vec);
  init(lines_mat.lines, lines_mat.vertices);
}

Lines3D::Lines3D(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices)
{
  init(lines, vertices);
}

Lines3D::~Lines3D()
{
  glDeleteVertexArrays(1, &vertex_array_id_);
}

void Lines3D::init(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices)
{
  initVAO();
  lines_ogl_ptr_ = std::make_shared<Lines3DStorageOgl>(lines, vertices);
  enableVertexVBOs();
  finishInit();
}

void Lines3D::initVAO()
{
  // Setup vertex arrays
  glGenVertexArrays(1, &vertex_array_id_);
  glBindVertexArray(vertex_array_id_);
}

void Lines3D::enableVertexVBOs()
{
  // Vertex attribute buffer
  lines_ogl_ptr_->bindVertexBuffer();
  glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(0);

  // Index buffer
  lines_ogl_ptr_->bindIndexBuffer();
}

void Lines3D::finishInit()
{
  glBindVertexArray(0);
}

void Lines3D::render() const
{
  glBindVertexArray(vertex_array_id_);
  glDrawElements(GL_LINES, lines_ogl_ptr_->getNumOfElements(), GL_UNSIGNED_INT, nullptr);
}

std::shared_ptr<Lines3D> Lines3D::createLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2)
{
  Eigen::MatrixXi lines(2, 1);
  Eigen::MatrixXd vertices(3, 2);
  lines(0) = 0;
  lines(1) = 1;
  vertices.col(0) = point1;
  vertices.col(1) = point2;
  return std::make_shared<Lines3D>(lines, vertices);
}


ColorLines3D::ColorLines3D(const std::shared_ptr<Lines3DStorageOgl>& lines_ogl_ptr, const Eigen::MatrixXd& colors)
: lines_ogl_ptr_(lines_ogl_ptr)
{
  initVAO();
  initColorVBO(colors);
  enableVertexVBOs();
  finishInit();
}

ColorLines3D::ColorLines3D(const Lines3DMatrixStorage& lines_mat, const Eigen::MatrixXd& colors)
{
  init(lines_mat.lines, lines_mat.vertices, colors);
}

ColorLines3D::ColorLines3D(const Lines3DVectorStorage& lines_vec, const Eigen::MatrixXd& colors)
{
  Lines3DMatrixStorage lines_mat(lines_vec);
  init(lines_mat.lines, lines_mat.vertices, colors);
}

ColorLines3D::ColorLines3D(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& colors)
{
  init(lines, vertices, colors);
}

ColorLines3D::~ColorLines3D()
{
  glDeleteBuffers(1, &color_buffer_id_);
  glDeleteVertexArrays(1, &vertex_array_id_);
}

void ColorLines3D::init(const Eigen::MatrixXi& lines, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& colors)
{
  initVAO();
  lines_ogl_ptr_ = std::make_shared<Lines3DStorageOgl>(lines, vertices);
  enableVertexVBOs();
  initColorVBO(colors);
  finishInit();
}

void ColorLines3D::initVAO()
{
  // Setup vertex arrays
  glGenVertexArrays(1, &vertex_array_id_);
  glBindVertexArray(vertex_array_id_);
}

void ColorLines3D::enableVertexVBOs()
{
  // Vertex attribute buffer
  lines_ogl_ptr_->bindVertexBuffer();
  glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(0);

  // Index buffer
  lines_ogl_ptr_->bindIndexBuffer();
}

void ColorLines3D::initColorVBO(const Eigen::MatrixXd& colors)
{
  // Color attribute buffer
  glGenBuffers(1, &color_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, color_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLdouble), colors.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(1);
}

void ColorLines3D::finishInit()
{
  glBindVertexArray(0);
}

void ColorLines3D::render() const
{
  glBindVertexArray(vertex_array_id_);
  glDrawElements(GL_LINES, lines_ogl_ptr_->getNumOfElements(), GL_UNSIGNED_INT, nullptr);
}

std::shared_ptr<ColorLines3D> ColorLines3D::createLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& color)
{
  Eigen::MatrixXi lines(2, 1);
  Eigen::MatrixXd vertices(3, 2);
  Eigen::MatrixXd colors(3, 2);
  lines(0) = 0;
  lines(1) = 1;
  vertices.col(0) = point1;
  vertices.col(1) = point2;
  colors.col(0) = color;
  colors.col(1) = color;
  return std::make_shared<ColorLines3D>(lines, vertices, colors);
}

} /* namespace rendering */
} /* namespace quad_planner */
