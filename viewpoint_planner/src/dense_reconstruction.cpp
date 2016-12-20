//==================================================
// dense_reconstruction.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 11, 2016
//==================================================

#include <iostream>
#include <ait/boost_utilities.h>
#include "dense_reconstruction.h"

DenseReconstruction::DenseReconstruction() {}

DenseReconstruction::~DenseReconstruction() {}

void DenseReconstruction::read(const std::string& path) {
  path_ = path;
  depth_maps_.clear();
  normal_maps_.clear();
  SparseReconstruction::read(ait::joinPaths(path, "sparse"));
}

const DenseReconstruction::DepthMap& DenseReconstruction::getDepthMap(ImageId image_id, DenseMapType dense_map_type) {
  auto it = depth_maps_.find(image_id);
  if (it == depth_maps_.cend()) {
    const Image& image = getImages().at(image_id);
    readDepthMap(image, dense_map_type);
    return depth_maps_.at(image_id);
  }
  return it->second;
}

const DenseReconstruction::DepthMap& DenseReconstruction::getNormalMap(ImageId image_id, DenseMapType dense_map_type) {
  auto it = normal_maps_.find(image_id);
  if (it == normal_maps_.cend()) {
    const Image& image = getImages().at(image_id);
    readNormalMap(image, dense_map_type);
    return normal_maps_.at(image_id);
  }
  return it->second;
}

std::string DenseReconstruction::denseTypeToString(DenseMapType dense_map_type) const {
  switch (dense_map_type) {
  case PHOTOMETRIC:
    return "photometric";
  case GEOMETRIC:
    return "geometric";
  }
  throw AIT_EXCEPTION("Unknown dense map type");
}

void DenseReconstruction::readDepthMap(const Image& image, DenseMapType dense_map_type) {
  std::cout << "Reading depth map for image " << image.name << std::endl;
  DepthMap depth_map;
  std::string depth_maps_path = ait::joinPaths(path_, "stereo", "depth_maps");
  std::string depth_map_filename = image.name + "." + denseTypeToString(dense_map_type) + ".bin";
  depth_map.readColmapFormat(ait::joinPaths(depth_maps_path, depth_map_filename));
  AIT_ASSERT_STR(depth_map.channels() == 1, "Depth map must have 1 channel");
  depth_maps_.emplace(image.id, depth_map);
}

void DenseReconstruction::readNormalMap(const Image& image, DenseMapType dense_map_type) {
  std::cout << "Reading normal map for image " << image.name << std::endl;
  NormalMap normal_map;
  std::string normal_maps_path = ait::joinPaths(path_, "stereo", "normal_maps");
  std::string normal_map_filename = image.name + "." + denseTypeToString(dense_map_type) + ".bin";
  normal_map.readColmapFormat(ait::joinPaths(normal_maps_path, normal_map_filename));
  AIT_ASSERT_STR(normal_map.channels() == 3, "Normal map must have 3 channel");
  normal_maps_.emplace(image.id, normal_map);
}
