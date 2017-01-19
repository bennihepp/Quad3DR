//==================================================
// dense_reconstruction.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 11, 2016
//==================================================

#include <ait/filesystem.h>
#include <iostream>
#include "dense_reconstruction.h"

using namespace reconstruction;

DenseReconstruction::DenseReconstruction() {}

DenseReconstruction::~DenseReconstruction() {}

void DenseReconstruction::read(const std::string& path) {
  path_ = path;
  depth_maps_.clear();
  normal_maps_.clear();
  SparseReconstruction::read(ait::joinPaths(path, "sparse"));
}

const DenseReconstruction::DepthMap& DenseReconstruction::getDepthMap(ImageId image_id, DenseMapType dense_map_type) const {
  auto it = depth_maps_.find(image_id);
  if (it == depth_maps_.cend()) {
    readAndCacheDepthMap(image_id, dense_map_type);
    return depth_maps_.at(image_id);
  }
  return it->second;
}

const DenseReconstruction::NormalMap& DenseReconstruction::getNormalMap(ImageId image_id, DenseMapType dense_map_type) const {
  auto it = normal_maps_.find(image_id);
  if (it == normal_maps_.cend()) {
    readAndCacheNormalMap(image_id, dense_map_type);
    return normal_maps_.at(image_id);
  }
  return it->second;
}

DenseReconstruction::DepthMap DenseReconstruction::readDepthMap(ImageId image_id, DenseMapType dense_map_type) const {
  const ImageColmap& image = getImages().at(image_id);
  std::string depth_maps_path = ait::joinPaths(path_, "stereo", "depth_maps");
  std::string depth_map_filename = image.name() + "." + denseTypeToString(dense_map_type) + ".bin";
  std::cout << "Reading depth map " << depth_map_filename << " for image " << image.name() << std::endl;
  DepthMap depth_map;
  depth_map.readColmapFormat(ait::joinPaths(depth_maps_path, depth_map_filename));
  AIT_ASSERT_STR(depth_map.channels() == 1, "Depth map must have 1 channel");
  return depth_map;
}

DenseReconstruction::NormalMap DenseReconstruction::readNormalMap(ImageId image_id, DenseMapType dense_map_type) const {
  const ImageColmap& image = getImages().at(image_id);
  std::string normal_maps_path = ait::joinPaths(path_, "stereo", "normal_maps");
  std::string normal_map_filename = image.name() + "." + denseTypeToString(dense_map_type) + ".bin";
  std::cout << "Reading normal map " << normal_map_filename << " for image " << image.name() << std::endl;
  NormalMap normal_map;
  normal_map.readColmapFormat(ait::joinPaths(normal_maps_path, normal_map_filename));
  AIT_ASSERT_STR(normal_map.channels() == 3, "Normal map must have 3 channel");
  return normal_map;
}

std::string DenseReconstruction::denseTypeToString(DenseMapType dense_map_type) const {
  switch (dense_map_type) {
  case PHOTOMETRIC:
    return "photometric";
  case PHOTOMETRIC_FUSED:
    return "photometric.fused";
  case GEOMETRIC:
    return "geometric";
  case GEOMETRIC_FUSED:
    return "geometric.fused";
  }
  throw AIT_EXCEPTION("Unknown dense map type");
}

void DenseReconstruction::readAndCacheDepthMap(ImageId image_id, DenseMapType dense_map_type) const {
  DepthMap depth_map = readDepthMap(image_id, dense_map_type);
  depth_maps_.emplace(image_id, depth_map);
}

void DenseReconstruction::readAndCacheNormalMap(ImageId image_id, DenseMapType dense_map_type) const {
  NormalMap normal_map = readNormalMap(image_id, dense_map_type);
  normal_maps_.emplace(image_id, normal_map);
}
