//==================================================
// voxel_drawer.h
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
#include <QOpenGLTexture>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#include "point_drawer.h"

inline std::ostream& operator<<(std::ostream& out, const QVector3D& vec) {
  out << "(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const QMatrix4x4& mat) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (j > 0)
        std::cout << ", ";
      std::cout << mat(i, j);
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  return out;
}

struct OGLVoxelData
{
  OGLVoxelData()
  : size(1) {}

  OGLVoxelData(const OGLVertexData& vertex, float size)
  : vertex(vertex), size(size) {}

  OGLVertexData vertex;
  float size;
};

inline std::ostream& operator<<(std::ostream& out, const OGLVoxelData& voxel) {
  out << voxel.vertex << " [" << voxel.size << "]";
  return out;
}

class VoxelDrawer : protected QOpenGLFunctions_3_3_Core
{
public:
  const unsigned int kVerticesPerVoxel = 12 * 3;

  VoxelDrawer()
  : num_voxels_(0),
    voxel_position_tex_(QOpenGLTexture::TargetBuffer),
    vertex_offset_normal_tex_(QOpenGLTexture::TargetBuffer),
    color_tex_(QOpenGLTexture::TargetBuffer),
    alpha_override_(-1.0f),
    light_position_(-10, 10, 10) {}

  ~VoxelDrawer() {
    clear();
  }

  void clear() {
    num_voxels_ = 0;
    if (color_tex_.isCreated()) {
      color_tex_.destroy();
    }
    if (color_vbo_.isCreated()) {
      color_vbo_.destroy();
    }
    if (vertex_offset_normal_tex_.isCreated()) {
      vertex_offset_normal_tex_.destroy();
    }
    if (vertex_offset_normal_vbo_.isCreated()) {
      vertex_offset_normal_vbo_.destroy();
    }
    if (voxel_position_tex_.isCreated()) {
      voxel_position_tex_.destroy();
    }
    if (voxel_position_vbo_.isCreated()) {
      voxel_position_vbo_.destroy();
    }
    if (vao_.isCreated()) {
      vao_.destroy();
    }
  }

  void init() {
    initializeOpenGLFunctions();
    clear();

    if (program_.isLinked()) {
      program_.release();
      program_.removeAllShaders();
    }

    program_.create();
    program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/voxels.v.glsl");
    program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/voxels.f.glsl");
    program_.link();

    vao_.create();
    voxel_position_vbo_.create();
    voxel_position_tex_.create();
    vertex_offset_normal_vbo_.create();
    vertex_offset_normal_tex_.create();
    color_vbo_.create();
    color_tex_.create();
  }

  size_t numOfVoxels() const {
    return num_voxels_;
  }

  std::vector<OGLVertexData> computeVertexOffsets() {
    // Corners
    std::vector<OGLVertexData> offset_corners(8);

    offset_corners[0].x = -0.5f;
    offset_corners[0].y = -0.5f;
    offset_corners[0].z = -0.5f;

    offset_corners[1].x = +0.5f;
    offset_corners[1].y = -0.5f;
    offset_corners[1].z = -0.5f;

    offset_corners[2].x = -0.5f;
    offset_corners[2].y = +0.5f;
    offset_corners[2].z = -0.5f;

    offset_corners[3].x = +0.5f;
    offset_corners[3].y = +0.5f;
    offset_corners[3].z = -0.5f;

    offset_corners[4].x = -0.5f;
    offset_corners[4].y = -0.5f;
    offset_corners[4].z = +0.5f;

    offset_corners[5].x = +0.5f;
    offset_corners[5].y = -0.5f;
    offset_corners[5].z = +0.5f;

    offset_corners[6].x = -0.5f;
    offset_corners[6].y = +0.5f;
    offset_corners[6].z = +0.5f;

    offset_corners[7].x = +0.5f;
    offset_corners[7].y = +0.5f;
    offset_corners[7].z = +0.5f;

    OGLVertexData unit_x(1, 0, 0);
    OGLVertexData unit_y(0, 1, 0);
    OGLVertexData unit_z(0, 0, 1);

    // Save offset vertex positions and corresponding normals
    std::vector<OGLVertexData> vertex_offsets(2 * kVerticesPerVoxel);
    // Triangles

    // BOTTOM 1
    vertex_offsets[0] = offset_corners[0];
    vertex_offsets[1] = offset_corners[2];
    vertex_offsets[2] = offset_corners[1];
    // BOTTOM 1 normal
    vertex_offsets[kVerticesPerVoxel + 0] = -unit_z;
    vertex_offsets[kVerticesPerVoxel + 1] = -unit_z;
    vertex_offsets[kVerticesPerVoxel + 2] = -unit_z;
    // BOTTOM 2
    vertex_offsets[3] = offset_corners[1];
    vertex_offsets[4] = offset_corners[2];
    vertex_offsets[5] = offset_corners[3];
    // BOTTOM 2 normal
    vertex_offsets[kVerticesPerVoxel + 3] = -unit_z;
    vertex_offsets[kVerticesPerVoxel + 4] = -unit_z;
    vertex_offsets[kVerticesPerVoxel + 5] = -unit_z;

    // TOP 1
    vertex_offsets[6] = offset_corners[4];
    vertex_offsets[7] = offset_corners[5];
    vertex_offsets[8] = offset_corners[6];
    // TOP 1 normal
    vertex_offsets[kVerticesPerVoxel + 6] = +unit_z;
    vertex_offsets[kVerticesPerVoxel + 7] = +unit_z;
    vertex_offsets[kVerticesPerVoxel + 8] = +unit_z;
    // TOP 2
    vertex_offsets[9]  = offset_corners[6];
    vertex_offsets[10] = offset_corners[5];
    vertex_offsets[11] = offset_corners[7];
    // TOP 2 normal
    vertex_offsets[kVerticesPerVoxel + 9]  = +unit_z;
    vertex_offsets[kVerticesPerVoxel + 10] = +unit_z;
    vertex_offsets[kVerticesPerVoxel + 11] = +unit_z;

    // LEFT 1
    vertex_offsets[12] = offset_corners[0];
    vertex_offsets[13] = offset_corners[4];
    vertex_offsets[14] = offset_corners[2];
    // LEFT 1 normal
    vertex_offsets[kVerticesPerVoxel + 12] = -unit_x;
    vertex_offsets[kVerticesPerVoxel + 13] = -unit_x;
    vertex_offsets[kVerticesPerVoxel + 14] = -unit_x;
    // LEFT 2
    vertex_offsets[15] = offset_corners[2];
    vertex_offsets[16] = offset_corners[4];
    vertex_offsets[17] = offset_corners[6];
    // LEFT 2 normal
    vertex_offsets[kVerticesPerVoxel + 15] = -unit_x;
    vertex_offsets[kVerticesPerVoxel + 16] = -unit_x;
    vertex_offsets[kVerticesPerVoxel + 17] = -unit_x;

    // RIGHT 1
    vertex_offsets[18] = offset_corners[1];
    vertex_offsets[19] = offset_corners[3];
    vertex_offsets[20] = offset_corners[5];
    // RIGHT 1 normal
    vertex_offsets[kVerticesPerVoxel + 18] = +unit_x;
    vertex_offsets[kVerticesPerVoxel + 19] = +unit_x;
    vertex_offsets[kVerticesPerVoxel + 20] = +unit_x;
    // RIGHT 2
    vertex_offsets[21] = offset_corners[5];
    vertex_offsets[22] = offset_corners[3];
    vertex_offsets[23] = offset_corners[7];
    // RIGHT 2 normal
    vertex_offsets[kVerticesPerVoxel + 21] = +unit_x;
    vertex_offsets[kVerticesPerVoxel + 22] = +unit_x;
    vertex_offsets[kVerticesPerVoxel + 23] = +unit_x;

    // BACK 1
    vertex_offsets[24] = offset_corners[0];
    vertex_offsets[25] = offset_corners[4];
    vertex_offsets[26] = offset_corners[1];
    // BACK 1 normal
    vertex_offsets[kVerticesPerVoxel + 24] = -unit_y;
    vertex_offsets[kVerticesPerVoxel + 25] = -unit_y;
    vertex_offsets[kVerticesPerVoxel + 26] = -unit_y;
    // BACK 2
    vertex_offsets[27] = offset_corners[1];
    vertex_offsets[28] = offset_corners[4];
    vertex_offsets[29] = offset_corners[5];
    // BACK 2 normal
    vertex_offsets[kVerticesPerVoxel + 27] = -unit_y;
    vertex_offsets[kVerticesPerVoxel + 28] = -unit_y;
    vertex_offsets[kVerticesPerVoxel + 29] = -unit_y;

    // FRONT 1
    vertex_offsets[30] = offset_corners[2];
    vertex_offsets[31] = offset_corners[3];
    vertex_offsets[32] = offset_corners[6];
    // FRONT 1 normal
    vertex_offsets[kVerticesPerVoxel + 30] = +unit_y;
    vertex_offsets[kVerticesPerVoxel + 31] = +unit_y;
    vertex_offsets[kVerticesPerVoxel + 32] = +unit_y;
    // FRONT 2
    vertex_offsets[33] = offset_corners[6];
    vertex_offsets[34] = offset_corners[3];
    vertex_offsets[35] = offset_corners[7];
    // FRONT 2 normal
    vertex_offsets[kVerticesPerVoxel + 33] = +unit_y;
    vertex_offsets[kVerticesPerVoxel + 34] = +unit_y;
    vertex_offsets[kVerticesPerVoxel + 35] = +unit_y;

    return vertex_offsets;
  }

  void uploadColors(const std::vector<OGLColorData>& color_data) {
    assert(color_data.size() == num_voxels_);

    // Create texture with voxel positions
    glBindBuffer(GL_TEXTURE_BUFFER, color_vbo_.bufferId());
    glBufferData(GL_TEXTURE_BUFFER, color_data.size() * sizeof(OGLColorData), color_data.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
  }

  void upload(const std::vector<OGLVoxelData>& voxel_data, const std::vector<OGLColorData>& color_data) {
    std::cout << "Uploading " << voxel_data.size() << " voxels" << std::endl;
    num_voxels_ = voxel_data.size();

    program_.bind();
    vao_.bind();

    // Create texture with voxel positions
    glBindBuffer(GL_TEXTURE_BUFFER, voxel_position_vbo_.bufferId());
    glBufferData(GL_TEXTURE_BUFFER, voxel_data.size() * sizeof(OGLVoxelData), voxel_data.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);

    // Create texture with vertex offsets (same for each voxel)glGenBuffers(1, &ubo);
    std::vector<OGLVertexData> vertex_offsets = computeVertexOffsets();
    glBindBuffer(GL_TEXTURE_BUFFER, vertex_offset_normal_vbo_.bufferId());
    glBufferData(GL_TEXTURE_BUFFER, vertex_offsets.size() * sizeof(OGLVertexData), vertex_offsets.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);

    vao_.release();
    program_.release();

    uploadColors(color_data);
  }

  void draw(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix) {
    if (num_voxels_ == 0) {
      return;
    }

    program_.bind();
    vao_.bind();

    program_.setUniformValue("u_alpha_override", alpha_override_);
    program_.setUniformValue("u_light_position", light_position_);

    // Set transformation matrices
    program_.setUniformValue("u_pvm_matrix", pvm_matrix);
    program_.setUniformValue("u_view_matrix", view_matrix);
    program_.setUniformValue("u_model_matrix", model_matrix);

    // TEXTURE (Position)
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, voxel_position_tex_.textureId());
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, voxel_position_vbo_.bufferId());
    glUniform1i(program_.uniformLocation("u_pos_texture"), 0);
    // TEXTURE (Offset)
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER, vertex_offset_normal_tex_.textureId());
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGB32F, vertex_offset_normal_vbo_.bufferId());
    glUniform1i(program_.uniformLocation("u_offset_normal_texture"), 1);
    // TEXTURE (Color)
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_BUFFER, color_tex_.textureId());
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, color_vbo_.bufferId());
    glUniform1i(program_.uniformLocation("u_color_texture"), 2);

    glDrawArrays(GL_TRIANGLES, 0, kVerticesPerVoxel * num_voxels_);

    glBindTexture(GL_TEXTURE_BUFFER, 0);

    vao_.release();
    program_.release();
  }

  void overrideAlpha(float alpha_override) {
    alpha_override_ = alpha_override;
  }

  void resetAlpha() {
    alpha_override_ = -1.0f;
  }

  void setLightPosition(const QVector3D& light_position) {
    light_position_ = light_position;
  }

private:
  size_t num_voxels_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer voxel_position_vbo_;
  QOpenGLTexture voxel_position_tex_;
  QOpenGLBuffer vertex_offset_normal_vbo_;
  QOpenGLTexture vertex_offset_normal_tex_;
  QOpenGLBuffer color_vbo_;
  QOpenGLTexture color_tex_;
  QOpenGLShaderProgram program_;
  float alpha_override_;
  QVector3D light_position_;
};
