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
#include <ait/common.h>
#include "point_drawer.h"

struct OGLTriangleData
{
  OGLTriangleData() {}

  OGLTriangleData(const OGLVertexDataRGBA& vertex1, const OGLVertexDataRGBA& vertex2, const OGLVertexDataRGBA& vertex3)
  : vertex1(vertex1), vertex2(vertex2), vertex3(vertex3) {}

  OGLVertexDataRGBA vertex1;
  OGLVertexDataRGBA vertex2;
  OGLVertexDataRGBA vertex3;
};

struct OGLTriangleWithNormalData
{
  OGLTriangleWithNormalData() {}

  OGLTriangleWithNormalData(
      const OGLVertexDataRGBA& vertex1, const OGLVertexDataRGBA& vertex2, const OGLVertexDataRGBA& vertex3,
      const OGLNormalData& normal1, const OGLNormalData& normal2, const OGLNormalData& normal3) {
    this->vertex1 = OGLVertexNormalDataRGBA(vertex1, normal1);
    this->vertex2 = OGLVertexNormalDataRGBA(vertex2, normal2);
    this->vertex3 = OGLVertexNormalDataRGBA(vertex3, normal3);
  }

  OGLVertexNormalDataRGBA vertex1;
  OGLVertexNormalDataRGBA vertex2;
  OGLVertexNormalDataRGBA vertex3;
};

inline std::ostream& operator<<(std::ostream& out, const OGLTriangleData& triangle) {
    out << "[ " << triangle.vertex1 << " " << triangle.vertex2 << " " << triangle.vertex3 << " ]";
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const OGLTriangleWithNormalData& triangle) {
    out << "[ " << triangle.vertex1 << " " << triangle.vertex2 << " " << triangle.vertex3 << " ]";
    return out;
}

class TriangleDrawer
{
public:
    TriangleDrawer()
    : draw_triangles_(true), num_triangles_(0), has_normals_(false) {
    }

    ~TriangleDrawer() {
      clear();
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
      clear();

      if (program_.isLinked()) {
        program_.release();
        program_.removeAllShaders();
      }

      program_.create();
      program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles.v.glsl");
      program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles.f.glsl");
      program_.link();

      if (normals_program_.isLinked()) {
        normals_program_.release();
        normals_program_.removeAllShaders();

      }

      normals_program_.create();
      normals_program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles_normals.v.glsl");
      normals_program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles_normals.f.glsl");
      normals_program_.link();

      vao_.create();
      vbo_.create();

      normals_vao_.create();
      normals_vbo_.create();
    }

    void upload(const std::vector<OGLTriangleData>& triangle_data) {
        num_triangles_ = triangle_data.size();
        has_normals_ = false;

        program_.bind();
        vao_.bind();
        vbo_.bind();

        vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo_.allocate(triangle_data.data(), triangle_data.size() * sizeof(OGLTriangleData));

        program_.enableAttributeArray(0);
        program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexDataRGBA));

        program_.enableAttributeArray(1);
        program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 4, sizeof(OGLVertexDataRGBA));

        vbo_.release();
        vao_.release();
        program_.release();

        program_.link();
    }

    void upload(const std::vector<OGLTriangleWithNormalData>& triangle_normal_data) {
      num_triangles_ = triangle_normal_data.size();
      has_normals_ = true;

      program_.bind();
      vao_.bind();
      vbo_.bind();

      vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
      vbo_.allocate(triangle_normal_data.data(), triangle_normal_data.size() * sizeof(OGLTriangleWithNormalData));

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

      program_.link();

      normals_program_.bind();
      normals_vao_.bind();
      normals_vbo_.bind();

      normals_vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
      normals_vbo_.allocate(triangle_normal_data.data(), triangle_normal_data.size() * sizeof(OGLTriangleWithNormalData));

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

      normals_program_.link();
    }

    void draw(const QMatrix4x4& pvm_matrix) {
        if (!draw_triangles_ || num_triangles_ == 0) {
            return;
        }

        program_.bind();
        vao_.bind();

        program_.setUniformValue("u_pvm_matrix", pvm_matrix);

        glDrawArrays(GL_TRIANGLES, 0, 3 * num_triangles_);

        vao_.release();
        program_.release();
    }

    void draw(const QMatrix4x4& pvm_matrix, QOpenGLFunctions* gl_functions) {
      if (!draw_triangles_ || num_triangles_ == 0) {
          return;
      }

      program_.bind();
      vao_.bind();

      program_.setUniformValue("u_pvm_matrix", pvm_matrix);

      gl_functions->glDrawArrays(GL_TRIANGLES, 0, 3 * num_triangles_);

      vao_.release();
      program_.release();
    }

    void drawNormals(const QMatrix4x4& pvm_matrix) {
        if (!draw_triangles_ || num_triangles_ == 0 || !has_normals_) {
            return;
        }

        normals_program_.bind();
        normals_vao_.bind();

        normals_program_.setUniformValue("u_pvm_matrix", pvm_matrix);

        glDrawArrays(GL_TRIANGLES, 0, 3 * num_triangles_);

        normals_vao_.release();
        normals_program_.release();
    }

    void drawNormals(const QMatrix4x4& pvm_matrix, QOpenGLFunctions* gl_functions) {
      if (!draw_triangles_ || num_triangles_ == 0 || !has_normals_) {
          return;
      }

      normals_program_.bind();
      normals_vao_.bind();

      normals_program_.setUniformValue("u_pvm_matrix", pvm_matrix);

      gl_functions->glDrawArrays(GL_TRIANGLES, 0, 3 * num_triangles_);

      normals_vao_.release();
      normals_program_.release();
    }

private:
    bool draw_triangles_;
    size_t num_triangles_;
    bool has_normals_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;
    QOpenGLShaderProgram program_;
    QOpenGLVertexArrayObject normals_vao_;
    QOpenGLBuffer normals_vbo_;
    QOpenGLShaderProgram normals_program_;
};
