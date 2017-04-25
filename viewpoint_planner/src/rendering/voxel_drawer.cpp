//==================================================
// voxel_drawer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 15, 2016
//==================================================

#include "voxel_drawer.h"
#include <array>
#include <boost/assign.hpp>
#include <bh/qt/utils.h>

namespace rendering {

inline std::ostream &operator<<(std::ostream &out, const OGLVoxelData &voxel) {
  out << voxel.vertex << " [" << voxel.size << "]";
  return out;
}

VoxelDrawer::VoxelDrawer()
        : num_voxels_(0),
          voxel_position_tex_(QOpenGLTexture::TargetBuffer),
          vertex_offset_normal_tex_(QOpenGLTexture::TargetBuffer),
          color_tex_(QOpenGLTexture::TargetBuffer),
          voxel_info_tex_(QOpenGLTexture::TargetBuffer),
          voxel_size_factor_(1),
          color_flags_(ColorFlags::Fixed),
          weight_color_scale_(1),
          weight_color_offset_(0),
          observation_count_color_scale_(1),
          observation_count_color_offset_(0),
          information_color_scale_(1),
          information_color_offset_(0),
          alpha_override_(-1.0f),
          light_position_(0, 0, 100),
          min_occupancy_(std::numeric_limits<float>::lowest()),
          max_occupancy_(std::numeric_limits<float>::max()),
          min_observations_(std::numeric_limits<float>::lowest()),
          max_observations_(std::numeric_limits<float>::max()),
          min_voxel_size_(std::numeric_limits<float>::lowest()),
          max_voxel_size_(std::numeric_limits<float>::max()),
          min_weight_(std::numeric_limits<float>::lowest()),
          max_weight_(std::numeric_limits<float>::max()),
          min_information_(std::numeric_limits<float>::lowest()),
          max_information_(std::numeric_limits<float>::max()) {}

VoxelDrawer::~VoxelDrawer() {
  clear();
}

void VoxelDrawer::clear() {
  num_voxels_ = 0;
  if (voxel_info_tex_.isCreated()) {
    voxel_info_tex_.destroy();
  }
  if (voxel_info_vbo_.isCreated()) {
    voxel_info_vbo_.destroy();
  }
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

void VoxelDrawer::init() {
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
  voxel_info_vbo_.create();
  voxel_info_tex_.create();
}

const QThread* VoxelDrawer::getVaoThread() const {
  return vao_.thread();
}

size_t VoxelDrawer::numOfVoxels() const {
  return num_voxels_;
}

std::vector<OGLVertexData> VoxelDrawer::computeVertexOffsets() {
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
  vertex_offsets[9] = offset_corners[6];
  vertex_offsets[10] = offset_corners[5];
  vertex_offsets[11] = offset_corners[7];
  // TOP 2 normal
  vertex_offsets[kVerticesPerVoxel + 9] = +unit_z;
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

void VoxelDrawer::uploadColors(const std::vector<OGLColorData> &color_data) {
  assert(color_data.size() == num_voxels_);

  // Create texture with voxel positions
  glBindBuffer(GL_TEXTURE_BUFFER, color_vbo_.bufferId());
  glBufferData(GL_TEXTURE_BUFFER, color_data.size() * sizeof(OGLColorData), color_data.data(), GL_DYNAMIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);
}

void VoxelDrawer::upload(const std::vector<OGLVoxelData> &voxel_data, const std::vector<OGLColorData> &color_data,
                         const std::vector<OGLVoxelInfoData> &info_data) {
  num_voxels_ = voxel_data.size();

  program_.bind();
  vao_.bind();

  // Create texture with voxel positions
  glBindBuffer(GL_TEXTURE_BUFFER, voxel_position_vbo_.bufferId());
  glBufferData(GL_TEXTURE_BUFFER, voxel_data.size() * sizeof(OGLVoxelData), voxel_data.data(), GL_DYNAMIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);

  // Create texture with vertex offsets (same for each voxel)
  std::vector<OGLVertexData> vertex_offsets = computeVertexOffsets();
  glBindBuffer(GL_TEXTURE_BUFFER, vertex_offset_normal_vbo_.bufferId());
  glBufferData(GL_TEXTURE_BUFFER, vertex_offsets.size() * sizeof(OGLVertexData), vertex_offsets.data(),
               GL_DYNAMIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);

  // Create texture with voxel info
  glBindBuffer(GL_TEXTURE_BUFFER, voxel_info_vbo_.bufferId());
  glBufferData(GL_TEXTURE_BUFFER, info_data.size() * sizeof(OGLVoxelInfoData), info_data.data(), GL_DYNAMIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);

  vao_.release();
  program_.release();

  uploadColors(color_data);
}

void VoxelDrawer::draw(const QMatrix4x4 &pvm_matrix, const QMatrix4x4 &view_matrix, const QMatrix4x4 &model_matrix) {
  draw(pvm_matrix, view_matrix * model_matrix);
}

void VoxelDrawer::draw(const QMatrix4x4 &pvm_matrix, const QMatrix4x4 &vm_matrix) {
  if (num_voxels_ == 0) {
    return;
  }

  program_.bind();
  vao_.bind();

  program_.setUniformValue("u_voxel_size_factor", voxel_size_factor_);

  glUniform1ui(program_.uniformLocation("u_color_mode"), static_cast<uint32_t>(color_flags_));
  program_.setUniformValue("u_weight_color_scale", weight_color_scale_);
  program_.setUniformValue("u_weight_color_offset", weight_color_offset_);
  program_.setUniformValue("u_observation_count_color_scale", observation_count_color_scale_);
  program_.setUniformValue("u_observation_count_color_offset", observation_count_color_offset_);
  program_.setUniformValue("u_information_color_scale", information_color_scale_);
  program_.setUniformValue("u_information_color_offset", information_color_offset_);

  program_.setUniformValue("u_alpha_override", alpha_override_);
  program_.setUniformValue("u_light_position", light_position_);
  program_.setUniformValue("u_min_occupancy", min_occupancy_);
  program_.setUniformValue("u_max_occupancy", max_occupancy_);
  program_.setUniformValue("u_min_observations", min_observations_);
  program_.setUniformValue("u_max_observations", max_observations_);
  program_.setUniformValue("u_min_voxel_size", min_voxel_size_);
  program_.setUniformValue("u_max_voxel_size", max_voxel_size_);
  program_.setUniformValue("u_min_weight", min_weight_);
  program_.setUniformValue("u_max_weight", max_weight_);
  program_.setUniformValue("u_min_information", min_information_);
  program_.setUniformValue("u_max_information", max_information_);

  // Set transformation matrices
  program_.setUniformValue("u_pvm_matrix", pvm_matrix);
  program_.setUniformValue("u_vm_matrix", vm_matrix);

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
  // TEXTURE (Info)
  glActiveTexture(GL_TEXTURE3);
  glBindTexture(GL_TEXTURE_BUFFER, voxel_info_tex_.textureId());
  glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, voxel_info_vbo_.bufferId());
  glUniform1i(program_.uniformLocation("u_info_texture"), 3);

  glDrawArrays(GL_TRIANGLES, 0, kVerticesPerVoxel * num_voxels_);

  glBindTexture(GL_TEXTURE_BUFFER, 0);

  vao_.release();
  program_.release();
}

void VoxelDrawer::setVoxelSizeFactor(const float voxel_size_factor) {
  voxel_size_factor_ = voxel_size_factor;
}

std::vector<std::pair<std::string, VoxelDrawer::ColorFlags>> VoxelDrawer::getAvailableColorFlags() {
  std::vector<std::pair<std::string, VoxelDrawer::ColorFlags>> modes = boost::assign::pair_list_of
          ("Fixed", Fixed)
          ("Weight", Weight)
          ("Occupancy", Occupancy)
          ("ObservationCount", ObservationCount)
          ("UnknownLowAlpha", UnknownLowAlpha)
          ("Information", Information)
          ("Index", Index);
  return modes;
}

VoxelDrawer::ColorFlags VoxelDrawer::getColorFlags() const {
  return color_flags_;
}

void VoxelDrawer::setColorFlags(const ColorFlags color_flags) {
  color_flags_ = color_flags;
}

void VoxelDrawer::overrideAlpha(float alpha_override) {
//  std::cout << "Alpha override: " << alpha_override << std::endl;
  alpha_override_ = alpha_override;
}

void VoxelDrawer::resetAlpha() {
  alpha_override_ = -1.0f;
}

void VoxelDrawer::setLightPosition(const QVector3D &light_position) {
  light_position_ = light_position;
}

void VoxelDrawer::setObservationsRange(const uint32_t low_observations, const uint32_t high_observations) {
  observation_count_color_scale_ = 1 / (float) (high_observations - low_observations);
  observation_count_color_offset_ = low_observations;
}

void VoxelDrawer::setWeightRange(const float low_weight, const float high_weight) {
  weight_color_scale_ = 1 / (float) (high_weight - low_weight);
  weight_color_offset_ = low_weight;
}

void VoxelDrawer::setInformationRange(const float low_information, const float high_information) {
  information_color_scale_ = 1 / (float) (high_information - low_information);
  information_color_offset_ = low_information;
}

void VoxelDrawer::setMinOccupancy(const float min_occupancy) {
  min_occupancy_ = std::max(min_occupancy, std::numeric_limits<float>::lowest());
}

void VoxelDrawer::setMaxOccupancy(const float max_occupancy) {
  max_occupancy_ = std::min(max_occupancy, std::numeric_limits<float>::max());
}

void VoxelDrawer::setMinObservations(const uint32_t min_observations) {
  min_observations_ = static_cast<float>(min_observations);
}

void VoxelDrawer::setMaxObservations(const uint32_t max_observations) {
  max_observations_ = static_cast<float>(max_observations);
}

void VoxelDrawer::setMinVoxelSize(const float min_voxel_size) {
  min_voxel_size_ = min_voxel_size;
}

void VoxelDrawer::setMaxVoxelSize(const float max_voxel_size) {
  max_voxel_size_ = max_voxel_size;
}

void VoxelDrawer::setMinWeight(const float min_weight) {
  min_weight_ = std::max(min_weight, std::numeric_limits<float>::lowest());
}

void VoxelDrawer::setMaxWeight(const float max_weight) {
  max_weight_ = std::min(max_weight, std::numeric_limits<float>::max());
}

void VoxelDrawer::setMinInformation(const float min_information) {
  min_information_ = std::max(min_information, std::numeric_limits<float>::lowest());
}

void VoxelDrawer::setMaxInformation(const float max_information) {
  max_information_ = std::min(max_information, std::numeric_limits<float>::max());
}

}
