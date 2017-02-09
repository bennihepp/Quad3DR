//==================================================
// voxel_drawer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 8, 2016
//==================================================
#pragma once

#include <iostream>
#include <QtOpenGL>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#include "point_drawer.h"

struct OGLVoxelData
{
  OGLVoxelData()
  : size(1) {}

  OGLVoxelData(const OGLVertexData& vertex, float size)
  : vertex(vertex), size(size) {}

  OGLVertexData vertex;
  float size;
};

struct OGLVoxelInfoData
{
  OGLVoxelInfoData(float occupancy, float observation_count, float weight, float information = 0.0f)
  : occupancy(occupancy), observation_count(observation_count),
    weight(weight), information(information) {}

  float occupancy;
  float observation_count;
  float weight;
  float information;
};

std::ostream& operator<<(std::ostream& out, const OGLVoxelData& voxel);

class VoxelDrawer : protected QOpenGLFunctions_3_3_Core
{
public:
  const unsigned int kVerticesPerVoxel = 12 * 3;

  enum ColorFlags : uint32_t {
    Fixed = 1 << 1,
    Weight = 1 << 3,
    Occupancy = 1 << 4,
    ObservationCount = 1 << 5,
    UnknownLowAlpha = 1 << 6,
    Information = 1 << 7,
  };

  VoxelDrawer();

  ~VoxelDrawer();

  void clear();

  void init();

  size_t numOfVoxels() const;

  std::vector<OGLVertexData> computeVertexOffsets();

  void uploadColors(const std::vector<OGLColorData>& color_data);

  void upload(const std::vector<OGLVoxelData>& voxel_data, const std::vector<OGLColorData>& color_data,
      const std::vector<OGLVoxelInfoData>& info_data);

  void draw(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix);

  void setVoxelSizeFactor(const float voxel_size_factor);

  static std::vector<std::pair<std::string, ColorFlags>> getAvailableColorFlags();

  ColorFlags getColorFlags() const;

  void setColorFlags(const ColorFlags color_mode);

  void overrideAlpha(float alpha_override);

  void resetAlpha();

  void setLightPosition(const QVector3D& light_position);

  // Ranges for color computation (has to be set from outside because multiple VoxelDrawers could be displaying different ranges)

  void setObservationsRange(const uint32_t low_observations, const uint32_t high_observations);
  void setWeightRange(const float low_weight, const float high_weight);
  void setInformationRange(const float low_information, const float high_information);

  // Voxel display filters

  void setMinOccupancy(const float min_occupancy);
  void setMaxOccupancy(const float max_occupancy);

  void setMinObservations(const uint32_t min_observations);
  void setMaxObservations(const uint32_t max_observations);

  void setMinVoxelSize(const float min_voxel_size);
  void setMaxVoxelSize(const float max_voxel_size);

  void setMinWeight(const float min_weight);
  void setMaxWeight(const float max_weight);

  void setMinInformation(const float min_information);
  void setMaxInformation(const float max_information);

private:
  size_t num_voxels_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer voxel_position_vbo_;
  QOpenGLTexture voxel_position_tex_;
  QOpenGLBuffer vertex_offset_normal_vbo_;
  QOpenGLTexture vertex_offset_normal_tex_;
  QOpenGLBuffer color_vbo_;
  QOpenGLTexture color_tex_;
  QOpenGLBuffer voxel_info_vbo_;
  QOpenGLTexture voxel_info_tex_;
  QOpenGLShaderProgram program_;

  float voxel_size_factor_;
  ColorFlags color_flags_;
  float weight_color_scale_;
  float weight_color_offset_;
  float observation_count_color_scale_;
  float observation_count_color_offset_;
  float information_color_scale_;
  float information_color_offset_;
  float alpha_override_;
  QVector3D light_position_;
  float min_occupancy_;
  float max_occupancy_;
  float min_observations_;
  float max_observations_;
  float min_voxel_size_;
  float max_voxel_size_;
  float min_weight_;
  float max_weight_;
  float min_information_;
  float max_information_;
};
