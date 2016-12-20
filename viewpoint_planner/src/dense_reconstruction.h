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

template <typename T>
class DataArray {
public:
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
    return col + width() * row + width() * height() * channels;
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
  using DepthMap = DataArray<float>;
  using NormalMap = DataArray<float>;
  using DepthMapMapType = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, DepthMap);
  using NormalMapMapType = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, NormalMap);

  enum DenseMapType {
    PHOTOMETRIC,
    GEOMETRIC,
  };

  DenseReconstruction();

  ~DenseReconstruction() override;

  void read(const std::string& path) override;

  /// Returns the depth map corresponding to the image (lazy loading of depth maps)
  const DepthMap& getDepthMap(ImageId image_id, DenseMapType dense_map_type = GEOMETRIC);

  /// Returns the normal map corresponding to the image (lazy loading of normal maps)
  const DepthMap& getNormalMap(ImageId image_id, DenseMapType dense_map_type = GEOMETRIC);

private:
  std::string denseTypeToString(DenseMapType dense_map_type) const;
  void readDepthMap(const Image& image, DenseMapType dense_map_type);
  void readNormalMap(const Image& image, DenseMapType dense_map_type);

  std::string path_;
  DepthMapMapType depth_maps_;
  NormalMapMapType normal_maps_;
};
