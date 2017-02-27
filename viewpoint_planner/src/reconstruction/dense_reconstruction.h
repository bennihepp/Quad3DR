//==================================================
// dense_reconstruction.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 11, 2016
//==================================================
#pragma once

#include <fstream>
#include <ait/common.h>
#include "sparse_reconstruction.h"

namespace reconstruction {

template <typename T>
class DataArray {
public:
  using ValueType = T;

  DataArray()
  : width_(0), height_(0), channels_(0) {}

  DataArray(size_t width, size_t height, size_t channels)
  : width_(width), height_(height), channels_(channels) {
    data_.resize(width * height * channels);
  }

  size_t width() const {
    return width_;
  }

  size_t height() const {
    return height_;
  }

  size_t channels() const {
    return channels_;
  }

  size_t getElementIndex(size_t row, size_t col, size_t channel = 0) const {
    return col + width() * row + width() * height() * channel;
  }

  const T& operator()(size_t row, size_t col, size_t channel = 0) const {
    return data_[getElementIndex(row, col, channel)];
  }

  T& operator()(size_t row, size_t col, size_t channel = 0) {
    return data_[getElementIndex(row, col, channel)];
  }

  const std::vector<T>& getData() const {
    return data_;
  }

  std::vector<T>& getData() {
    return data_;
  }

  void readColmapFormat(const std::string& filename) {
    std::ifstream text_in(filename, std::ios_base::in);
    AIT_ASSERT_STR(text_in, std::string("Unable to open colmap array for reading: ") + filename);

    char unused_char;
    text_in >> width_ >> unused_char
            >> height_ >> unused_char
            >> channels_ >> unused_char;
    std::streampos pos = text_in.tellg();
    text_in.close();

    AIT_ASSERT(width_ > 0);
    AIT_ASSERT(height_ > 0);
    AIT_ASSERT(channels_ > 0);
    data_.resize(width_ * height_ * channels_);

    std::ifstream binary_in(filename, std::ios_base::in | std::ios_base::binary);
    AIT_ASSERT_STR(binary_in, std::string("Unable to open colmap array for reading: ") + filename);
    binary_in.seekg(pos);
    binary_in.read(reinterpret_cast<char*>(data_.data()), data_.size() * sizeof(T));
    binary_in.close();
  }

  void writeColmapFormat(const std::string& filename) {
    std::ofstream text_out(filename, std::ios_base::out);
    AIT_ASSERT_STR(text_out, std::string("Unable to open colmap array for writing: ") + filename);
    text_out << width_ << "&" << height_ << "&" << channels_ << "&";
    text_out.close();

    std::ofstream binary_out(filename, std::ios_base::out | std::ios_base::binary | std::ios_base::app);
    AIT_ASSERT_STR(binary_out, std::string("Unable to open colmap array for writing: ") + filename);
    binary_out.write(reinterpret_cast<const char*>(data_.data()), sizeof(T) * data_.size());
    binary_out.close();
  }

private:
  size_t width_;
  size_t height_;
  size_t channels_;
  std::vector<T> data_;
};

class DenseReconstruction : public SparseReconstruction {
public:
  using DepthMap = DataArray<FloatType>;
  using NormalMap = DataArray<FloatType>;
  using DepthMapMapType = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, DepthMap);
  using NormalMapMapType = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, NormalMap);

  enum DenseMapType {
    PHOTOMETRIC,
    PHOTOMETRIC_FUSED,
    GEOMETRIC,
    GEOMETRIC_FUSED,
  };

  DenseReconstruction();

  ~DenseReconstruction() override;

  void read(const std::string& path, const bool read_sfm_gps_transformation=true) override;

  /// Returns the depth map corresponding to the image (lazy loading of depth maps)
  const DepthMap& getDepthMap(ImageId image_id, DenseMapType dense_map_type = GEOMETRIC) const;

  /// Returns the normal map corresponding to the image (lazy loading of normal maps)
  const NormalMap& getNormalMap(ImageId image_id, DenseMapType dense_map_type = GEOMETRIC) const;

  /// Reads the depth map corresponding to the image (does not store it internally)
  DepthMap readDepthMap(ImageId image_id, DenseMapType dense_map_type = GEOMETRIC) const;

  /// Reads the normal map corresponding to the image (does not store it internally)
  NormalMap readNormalMap(ImageId image_id, DenseMapType dense_map_type = GEOMETRIC) const;

private:
  std::string denseTypeToString(DenseMapType dense_map_type) const;
  void readAndCacheDepthMap(const ImageId image_id, DenseMapType dense_map_type) const;
  void readAndCacheNormalMap(const ImageId image_id, DenseMapType dense_map_type) const;

  std::string path_;
  mutable DepthMapMapType depth_maps_;
  mutable NormalMapMapType normal_maps_;
};

}
