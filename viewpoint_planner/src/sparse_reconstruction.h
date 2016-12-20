//==================================================
// sparse_reconstruction.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================
#pragma once

#include <unordered_map>
#include <ait/eigen.h>
#include "pose.h"

// Representations for a sparse reconstruction from Colmap.
// File input adapted from Colmap.
// Original copyright notice:
//
// COLMAP - Structure-from-Motion and Multi-View Stereo.
// Copyright (C) 2016  Johannes L. Schoenberger <jsch at inf.ethz.ch>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

using CameraId = size_t;
using ImageId = size_t;
using Point3DId = size_t;
using FeatureId = size_t;

using CameraMatrix = Eigen::Matrix4d;

class PinholeCamera {
public:
  PinholeCamera();

  PinholeCamera(size_t width, size_t height, const CameraMatrix& intrinsics);

  size_t width() const;

  size_t height() const;

  const CameraMatrix& intrinsics() const;

  Eigen::Vector2d projectPoint(const Eigen::Vector3d& hom_point_camera) const;

  Eigen::Vector3d getCameraRay(double x, double y) const;

  Eigen::Vector3d getCameraRay(const Eigen::Vector2d& point_image) const;

  Eigen::Vector3d unprojectPoint(double x, double y, double distance) const;

  Eigen::Vector3d unprojectPoint(const Eigen::Vector2d& point_image, double distance) const;

  double getMeanFocalLength() const;

  double getFocalLengthX() const;

  double getFocalLengthY() const;

  bool isPointInViewport(const Eigen::Vector2d& point) const;

  bool isPointInViewport(const Eigen::Vector2d& point, double margin) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  size_t width_;
  size_t height_;
  CameraMatrix intrinsics_;
};

class PinholeCameraColmap : public PinholeCamera {
public:
  PinholeCameraColmap(CameraId id, size_t width, size_t height, const std::vector<double>& params);

  CameraId id() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static CameraMatrix makeIntrinsicsFromParameters(const std::vector<double>& params);

  CameraId id_;
};

struct Feature {
  Eigen::Vector2d point;
  Point3DId point3d_id;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Image {
  ImageId id;
  Pose pose;
  std::string name;
  std::vector<Feature> features;
  CameraId camera_id;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Color : public Eigen::Matrix<uint8_t, 3, 1> {
  uint8_t& r() { return (*this)(0); };
  uint8_t& g() { return (*this)(1); };
  uint8_t& b() { return (*this)(2); };
  const uint8_t& r() const { return (*this)(0); };
  const uint8_t& g() const { return (*this)(1); };
  const uint8_t& b() const { return (*this)(2); };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Point3DStatistics {
public:
  Point3DStatistics();

  Point3DStatistics(double average_distance, double stddev_distance,
          double stddev_one_minus_dot_product);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double averageDistance() const;

  double stddevDistance() const;

  double stddevOneMinusDotProduct() const;

private:
  double average_distance_;
  double stddev_distance_;
  double stddev_one_minus_dot_product_;
};

struct Point3D {

  struct TrackEntry {
    ImageId image_id;
    FeatureId feature_index;
  };

  Point3DId getId() const;

  const Eigen::Vector3d& getPosition() const;

  const Eigen::Vector3d& getNormal() const;

  const Point3DStatistics& getStatistics() const;

  Point3DId id;
  Eigen::Vector3d pos;
  Color color;
  double error;
  std::vector<TrackEntry> feature_track;
  Eigen::Vector3d normal;
  Point3DStatistics statistics;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

const Point3DId invalid_point3d_id = std::numeric_limits<Point3DId>::max();

class SparseReconstruction {
public:
  using CameraMapType = EIGEN_ALIGNED_UNORDERED_MAP(CameraId, PinholeCameraColmap);
  using ImageMapType = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, Image);
  using Point3DMapType = EIGEN_ALIGNED_UNORDERED_MAP(Point3DId, Point3D);

  SparseReconstruction();

  virtual ~SparseReconstruction();

  virtual void read(const std::string& path);

  const CameraMapType& getCameras() const;

  CameraMapType& getCameras();

  const ImageMapType& getImages() const;

  ImageMapType& getImages();

  const Point3DMapType& getPoints3D() const;

  Point3DMapType& getPoints3D();

private:
  void computePoint3DNormalAndStatistics(Point3D& point) const;

  void readCameras(std::string filename);

  void readCameras(std::istream& in);

  void readImages(std::string filename);

  void readImages(std::istream& in);

  void readPoints3D(std::string filename);

  void readPoints3D(std::istream& in);

  CameraMapType cameras_;
  ImageMapType images_;
  Point3DMapType points3D_;
};
