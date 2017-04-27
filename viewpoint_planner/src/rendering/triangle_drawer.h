//==================================================
// triangle_drawer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 8, 2016
//==================================================
#pragma once

#include <array>
#include <QtOpenGL>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_5_Core>
#include <bh/common.h>
#include <bh/color.h>
#include "point_drawer.h"

namespace rendering {

using FloatType = float;

struct OGLTriangleData {
  OGLTriangleData() {}

  OGLTriangleData(const OGLVertexDataRGBA &vertex1, const OGLVertexDataRGBA &vertex2, const OGLVertexDataRGBA &vertex3)
          : vertex1(vertex1), vertex2(vertex2), vertex3(vertex3) {}

  OGLVertexDataRGBA vertex1;
  OGLVertexDataRGBA vertex2;
  OGLVertexDataRGBA vertex3;
};

struct OGLTriangleWithNormalData {
  OGLTriangleWithNormalData() {}

  OGLTriangleWithNormalData(
          const OGLVertexDataRGBA &vertex1, const OGLVertexDataRGBA &vertex2, const OGLVertexDataRGBA &vertex3,
          const OGLNormalData &normal1, const OGLNormalData &normal2, const OGLNormalData &normal3) {
    this->vertex1 = OGLVertexNormalDataRGBA(vertex1, normal1);
    this->vertex2 = OGLVertexNormalDataRGBA(vertex2, normal2);
    this->vertex3 = OGLVertexNormalDataRGBA(vertex3, normal3);
  }

  OGLVertexNormalDataRGBA vertex1;
  OGLVertexNormalDataRGBA vertex2;
  OGLVertexNormalDataRGBA vertex3;
};

inline std::ostream &operator<<(std::ostream &out, const OGLTriangleData &triangle) {
  out << "[ " << triangle.vertex1 << " " << triangle.vertex2 << " " << triangle.vertex3 << " ]";
  return out;
}

inline std::ostream &operator<<(std::ostream &out, const OGLTriangleWithNormalData &triangle) {
  out << "[ " << triangle.vertex1 << " " << triangle.vertex2 << " " << triangle.vertex3 << " ]";
  return out;
}

class TriangleDrawer : QOpenGLFunctions_4_5_Core {
public:
  TriangleDrawer()
          : draw_triangles_(true), num_triangles_(0), has_normals_(false) {
  }

  ~TriangleDrawer() {
    clear();

    if (program_.isLinked()) {
      program_.release();
      program_.removeAllShaders();
    }

    if (normals_program_.isLinked()) {
      normals_program_.release();
      normals_program_.removeAllShaders();

    }

    if (depth_program_.isLinked()) {
      depth_program_.release();
      depth_program_.removeAllShaders();

    }

    if (indices_program_.isLinked()) {
      indices_program_.release();
      indices_program_.removeAllShaders();

    }
  }

  void clear() {
    num_triangles_ = 0;
    if (vao_.isCreated()) {
      vao_.destroy();
    }
    if (vbo_.isCreated()) {
      vbo_.destroy();
    }
    if (normals_vao_.isCreated()) {
      normals_vao_.destroy();
    }
    if (normals_vbo_.isCreated()) {
      normals_vbo_.destroy();
    }
  }

  void setDrawTriangles(bool draw_triangles) {
    draw_triangles_ = draw_triangles;
  }

  void init() {
    QOpenGLFunctions_4_5_Core::initializeOpenGLFunctions();

    clear();

    if (program_.isLinked()) {
      program_.release();
      program_.removeAllShaders();
    }

    bool success;

    success = program_.create();
    BH_ASSERT(success);
    success = program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles.v.glsl");
    BH_ASSERT(success);
    success = program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles.f.glsl");
    BH_ASSERT(success);
    success = program_.link();
    BH_ASSERT(success);

    if (normals_program_.isLinked()) {
      normals_program_.release();
      normals_program_.removeAllShaders();

    }

    success = normals_program_.create();
    BH_ASSERT(success);
    success = normals_program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles_normals.v.glsl");
    BH_ASSERT(success);
    success = normals_program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles_normals.f.glsl");
    BH_ASSERT(success);
    success = normals_program_.link();
    BH_ASSERT(success);

    if (depth_program_.isLinked()) {
      depth_program_.release();
      depth_program_.removeAllShaders();

    }

    success = depth_program_.create();
    BH_ASSERT(success);
    success = depth_program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles_depth.v.glsl");
    BH_ASSERT(success);
    success = depth_program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles_depth.f.glsl");
    BH_ASSERT(success);
    success = depth_program_.link();
    BH_ASSERT(success);

    if (indices_program_.isLinked()) {
      indices_program_.release();
      indices_program_.removeAllShaders();

    }

    success = indices_program_.create();
    BH_ASSERT(success);
    success = indices_program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles_indices.v.glsl");
    BH_ASSERT(success);
    success = indices_program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles_indices.f.glsl");
    BH_ASSERT(success);
    success = indices_program_.link();
    BH_ASSERT(success);

    vao_.create();
    vbo_.create();

    normals_vao_.create();
    normals_vbo_.create();
  }

  void upload(const std::vector<OGLTriangleData> &triangle_data) {
    num_triangles_ = triangle_data.size();
    has_normals_ = false;

    bool success;

    program_.bind();
    vao_.bind();
    vbo_.bind();

    vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
//    vbo_.allocate(triangle_data.data(), triangle_data.size() * sizeof(OGLTriangleData));
    glNamedBufferData(vbo_.bufferId(), triangle_data.size() * sizeof(OGLTriangleData), triangle_data.data(), GL_STATIC_DRAW);
    GLenum gl_err = glGetError();
    if (gl_err != GL_NO_ERROR) {
      if (gl_err == GL_OUT_OF_MEMORY) {
        throw bh::Error("Not enough memory for allocating vertex buffer");
      }
      throw bh::Error("Unknown error when allocating vertex buffer");
    }

    program_.enableAttributeArray(0);
    program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexDataRGBA));

    program_.enableAttributeArray(1);
    program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 4, sizeof(OGLVertexDataRGBA));

    vbo_.release();
    vao_.release();
    program_.release();

    success = program_.link();
    BH_ASSERT(success);

    depth_program_.bind();
    vao_.bind();
    vbo_.bind();

    depth_program_.enableAttributeArray(0);
    depth_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexDataRGBA));

    depth_program_.enableAttributeArray(1);
    depth_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 4, sizeof(OGLVertexDataRGBA));

    vbo_.release();
    vao_.release();
    depth_program_.release();

    success = depth_program_.link();
    BH_ASSERT(success);

    indices_program_.bind();
    vao_.bind();
    vbo_.bind();

    indices_program_.enableAttributeArray(0);
    indices_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexDataRGBA));

    indices_program_.enableAttributeArray(1);
    indices_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 4, sizeof(OGLVertexDataRGBA));

    vbo_.release();
    vao_.release();
    indices_program_.release();

    success = indices_program_.link();
    BH_ASSERT(success);
  }

  void upload(const std::vector<OGLTriangleWithNormalData> &triangle_normal_data) {
    num_triangles_ = triangle_normal_data.size();
    has_normals_ = true;

    bool success;

    program_.bind();
    vao_.bind();
    vbo_.bind();

    vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
//    vbo_.allocate(triangle_normal_data.data(), triangle_normal_data.size() * sizeof(OGLTriangleWithNormalData));
    glNamedBufferData(vbo_.bufferId(), triangle_normal_data.size() * sizeof(OGLTriangleWithNormalData), triangle_normal_data.data(), GL_STATIC_DRAW);
    GLenum gl_err = glGetError();
    if (gl_err != GL_NO_ERROR) {
      if (gl_err == GL_OUT_OF_MEMORY) {
        throw bh::Error("Not enough memory for allocating vertex buffer");
      }
      throw bh::Error("Unknown error when allocating vertex buffer");
    }

    // Vertex
    program_.enableAttributeArray(0);
    program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexNormalDataRGBA));

    // Color
    program_.enableAttributeArray(1);
    program_.setAttributeBuffer(1, GL_FLOAT, 6 * sizeof(float), 4, sizeof(OGLVertexNormalDataRGBA));

    // Normal
    program_.enableAttributeArray(2);
    program_.setAttributeBuffer(2, GL_FLOAT, 3 * sizeof(float), 3, sizeof(OGLVertexNormalDataRGBA));

    vbo_.release();
    vao_.release();
    program_.release();

    success = program_.link();
    BH_ASSERT(success);

    normals_program_.bind();
    normals_vao_.bind();
    normals_vbo_.bind();

    normals_vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
//    normals_vbo_.allocate(triangle_normal_data.data(), triangle_normal_data.size() * sizeof(OGLTriangleWithNormalData));
    glNamedBufferData(normals_vbo_.bufferId(), triangle_normal_data.size() * sizeof(OGLTriangleWithNormalData), triangle_normal_data.data(), GL_STATIC_DRAW);
    gl_err = glGetError();
    if (gl_err != GL_NO_ERROR) {
      if (gl_err == GL_OUT_OF_MEMORY) {
        throw bh::Error("Not enough memory for allocating vertex buffer");
      }
      throw bh::Error("Unknown error when allocating vertex buffer");
    }

    // Vertex
    normals_program_.enableAttributeArray(0);
    normals_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexNormalDataRGBA));

    // Normal
    normals_program_.enableAttributeArray(1);
    normals_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, sizeof(OGLVertexNormalDataRGBA));

    // Color
    normals_program_.enableAttributeArray(2);
    normals_program_.setAttributeBuffer(2, GL_FLOAT, 6 * sizeof(float), 4, sizeof(OGLVertexNormalDataRGBA));

    normals_vbo_.release();
    normals_vao_.release();
    normals_program_.release();

    success = normals_program_.link();
    BH_ASSERT(success);

    depth_program_.bind();
    vbo_.bind();
    vao_.bind();

    // Vertex
    depth_program_.enableAttributeArray(0);
    depth_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexNormalDataRGBA));

    // Color
    depth_program_.enableAttributeArray(1);
    depth_program_.setAttributeBuffer(1, GL_FLOAT, 6 * sizeof(float), 4, sizeof(OGLVertexNormalDataRGBA));

    vbo_.release();
    vao_.release();
    depth_program_.release();

    success = depth_program_.link();
    BH_ASSERT(success);

    indices_program_.bind();
    vao_.bind();
    vbo_.bind();

    // Vertex
    indices_program_.enableAttributeArray(0);
    indices_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexNormalDataRGBA));

    // Color
    indices_program_.enableAttributeArray(1);
    indices_program_.setAttributeBuffer(1, GL_FLOAT, 6 * sizeof(float), 4, sizeof(OGLVertexNormalDataRGBA));

    vbo_.release();
    vao_.release();
    indices_program_.release();

    success = indices_program_.link();
    BH_ASSERT(success);
  }

  void draw(const QMatrix4x4 &pvm_matrix) {
    if (!draw_triangles_ || num_triangles_ == 0) {
      return;
    }

    program_.bind();
    vao_.bind();

    program_.setUniformValue("u_pvm_matrix", pvm_matrix);

    drawTriangles();

    vao_.release();
    program_.release();
  }

  void draw(const QMatrix4x4 &pvm_matrix, QOpenGLFunctions *gl_functions) {
    if (!draw_triangles_ || num_triangles_ == 0) {
      return;
    }

    program_.bind();
    vao_.bind();

    program_.setUniformValue("u_pvm_matrix", pvm_matrix);

    drawTriangles(gl_functions);

    vao_.release();
    program_.release();
  }

  void drawNormals(const QMatrix4x4 &pvm_matrix) {
    if (!draw_triangles_ || num_triangles_ == 0 || !has_normals_) {
      return;
    }

    normals_program_.bind();
    normals_vao_.bind();

    normals_program_.setUniformValue("u_pvm_matrix", pvm_matrix);

    drawTriangles();

    normals_vao_.release();
    normals_program_.release();
  }

  void drawNormals(const QMatrix4x4 &pvm_matrix, QOpenGLFunctions *gl_functions) {
    if (!draw_triangles_ || num_triangles_ == 0 || !has_normals_) {
      return;
    }

    normals_program_.bind();
    normals_vao_.bind();

    normals_program_.setUniformValue("u_pvm_matrix", pvm_matrix);

    drawTriangles(gl_functions);

    normals_vao_.release();
    normals_program_.release();
  }

  void drawDepth(const QMatrix4x4 &pvm_matrix, const QMatrix4x4 &vm_matrix) {
    if (!draw_triangles_ || num_triangles_ == 0 || !has_normals_) {
      return;
    }

    depth_program_.bind();
    vao_.bind();

    depth_program_.setUniformValue("u_pvm_matrix", pvm_matrix);
    depth_program_.setUniformValue("u_vm_matrix", vm_matrix);

    drawTriangles();

    vao_.release();
    depth_program_.release();
  }

  void drawDepth(const QMatrix4x4 &pvm_matrix, const QMatrix4x4 &vm_matrix, QOpenGLFunctions *gl_functions) {
    if (!draw_triangles_ || num_triangles_ == 0 || !has_normals_) {
      return;
    }

    depth_program_.bind();
    vao_.bind();

    depth_program_.setUniformValue("u_pvm_matrix", pvm_matrix);
    depth_program_.setUniformValue("u_vm_matrix", vm_matrix);

    drawTriangles(gl_functions);

    vao_.release();
    depth_program_.release();
  }

  void drawIndices(const QMatrix4x4 &pvm_matrix, const size_t index_offset = 0) {
    if (!draw_triangles_ || num_triangles_ == 0) {
      return;
    }

    indices_program_.bind();
    vao_.bind();

    indices_program_.setUniformValue("u_pvm_matrix", pvm_matrix);
    indices_program_.setUniformValue("u_index_offset", (GLint)index_offset);

    drawTriangles();

    vao_.release();
    indices_program_.release();
  }

  void drawIndices(const QMatrix4x4 &pvm_matrix, QOpenGLFunctions *gl_functions, const size_t index_offset = 0) {
    if (!draw_triangles_ || num_triangles_ == 0) {
      return;
    }

    indices_program_.bind();
    vao_.bind();

    indices_program_.setUniformValue("u_pvm_matrix", pvm_matrix);
    indices_program_.setUniformValue("u_index_offset", (GLint)index_offset);

    drawTriangles(gl_functions);

    vao_.release();
    indices_program_.release();
  }

  bh::Color4<uint8_t> encodeDepthValue(const FloatType depth) const {
    const FloatType multiplier(256.);
    const FloatType red = std::floor(depth / multiplier);
    const FloatType green = std::floor(depth - red * multiplier);
    const FloatType blue = std::floor((depth - red * multiplier - green) * multiplier);
    bh::Color4<uint8_t> color(
            static_cast<uint8_t>(bh::clamp<FloatType>(red, 0, 255)),
            static_cast<uint8_t>(bh::clamp<FloatType>(green, 0, 255)),
            static_cast<uint8_t>(bh::clamp<FloatType>(blue, 0, 255)), 255);
    return color;
  }

  FloatType decodeDepthValue(const bh::Color4<uint8_t> &color) const {
    const FloatType depth = color.r() * FloatType(256) + color.g() + color.b() / FloatType(256.);
    return depth;
  }

  FloatType decodeDepthValue(const QColor &pixel) const {
    const FloatType depth = pixel.red() * FloatType(256) + pixel.green() + pixel.blue() / FloatType(256.);
    return depth;
  }

  QImage convertEncodedDepthImageToRGB(
          const QImage &encoded_image, const FloatType min_depth, const FloatType max_depth) const {
    Eigen::MatrixXf depth_matrix(encoded_image.height(), encoded_image.width());
    for (int y = 0; y < encoded_image.height(); ++y) {
      for (int x = 0; x < encoded_image.width(); ++x) {
        depth_matrix(y, x) = decodeDepthValue(QColor(encoded_image.pixel(x, y)));
      }
    }
    QImage rgb_image(encoded_image.width(), encoded_image.height(), QImage::Format_ARGB32);
    bh::ColorMapHot<FloatType> cmap;
    for (int y = 0; y < encoded_image.height(); ++y) {
      for (int x = 0; x < encoded_image.width(); ++x) {
        const FloatType depth = bh::clamp(depth_matrix(y, x), min_depth, max_depth);
        const bh::Color3<FloatType> color = cmap.map(bh::normalize(depth, min_depth, max_depth));
        const QColor pixel(
                bh::clamp<int>(255 * color.r(), 0, 255),
                bh::clamp<int>(255 * color.g(), 0, 255),
                bh::clamp<int>(255 * color.b(), 0, 255));
        rgb_image.setPixel(x, y, pixel.rgba());
      }
    }
    return rgb_image;
  }

  size_t colorToIndex(const QColor& color) const {
    return colorToIndex(bh::Color4<FloatType>(
            color.redF(),
            color.greenF(),
            color.blueF(),
            color.alphaF()));
  }

  size_t colorToIndex(const bh::Color4<FloatType>& color) const {
//  BH_PRINT_VALUE(static_cast<size_t>(color.r() * FloatType(255)) << 0);
//  BH_PRINT_VALUE(static_cast<size_t>(color.g() * FloatType(255)) << 8);
//  BH_PRINT_VALUE(static_cast<size_t>(color.b() * FloatType(255)) << 16);
    return (static_cast<size_t>(color.r() * FloatType(255)) << 0) +
           (static_cast<size_t>(color.g() * FloatType(255)) << 8) +
           (static_cast<size_t>(color.b() * FloatType(255)) << 16);
  }

  bh::Color4<FloatType> indexToColor(const size_t index) const {
    const FloatType r = ((index & 0x000000FF) >> 0) / FloatType(255);
    const FloatType g = ((index & 0x0000FF00) >> 8) / FloatType(255);
    const FloatType b = ((index & 0x00FF0000) >> 16) / FloatType(255);
    const FloatType a = 1;
    return bh::Color4<FloatType>(r, g, b, a);
  }

private:
  void drawTriangles() {
    const size_t num_vertices = 3 * num_triangles_;
    const size_t max_vertices_to_draw = std::min(num_vertices, (size_t)GL_MAX_ELEMENTS_VERTICES);

    size_t i = 0;
    while (i < num_vertices) {
      const size_t num_vertices_to_draw = std::min(max_vertices_to_draw, 3 * num_vertices - i);
      glDrawArrays(GL_TRIANGLES, i, num_vertices_to_draw);
      i += num_vertices_to_draw;
    }
  }

  void drawTriangles(QOpenGLFunctions *gl_functions) {
    const size_t num_vertices = 3 * num_triangles_;
    const size_t max_vertices_to_draw = std::min(num_vertices, (size_t)GL_MAX_ELEMENTS_VERTICES);

    size_t i = 0;
    while (i < num_vertices) {
      const size_t num_vertices_to_draw = std::min(max_vertices_to_draw, 3 * num_vertices - i);
      gl_functions->glDrawArrays(GL_TRIANGLES, i, num_vertices_to_draw);
      i += num_vertices_to_draw;
    }
  }

  bool draw_triangles_;
  size_t num_triangles_;
  bool has_normals_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer vbo_;
  QOpenGLShaderProgram program_;
  QOpenGLVertexArrayObject normals_vao_;
  QOpenGLBuffer normals_vbo_;
  QOpenGLShaderProgram normals_program_;
  QOpenGLShaderProgram depth_program_;
  QOpenGLShaderProgram indices_program_;
};

}