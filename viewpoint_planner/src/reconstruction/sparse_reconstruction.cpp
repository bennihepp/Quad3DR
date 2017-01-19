//==================================================
// sparse_reconstruction.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 11, 2016
//==================================================

#include "sparse_reconstruction.h"
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <ait/boost.h>
#include <boost/functional.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <ait/eigen.h>
#include <ait/eigen_utils.h>
#include <ait/common.h>
#include <ait/filesystem.h>
#include <ait/utilities.h>
#include <ait/string_utils.h>

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

using namespace reconstruction;

using CameraMapType = SparseReconstruction::CameraMapType;
using ImageMapType = SparseReconstruction::ImageMapType;
using Point3DMapType = SparseReconstruction::Point3DMapType;

PinholeCamera::PinholeCamera()
: width_(0), height_(0), intrinsics_(CameraMatrix::Zero()) {}

PinholeCamera::PinholeCamera(size_t width, size_t height, const CameraMatrix& intrinsics)
: width_(width), height_(height), intrinsics_(intrinsics) {}

bool PinholeCamera::isValid() const {
  return width_ > 0 && height_ > 0
      && intrinsics_(0, 0) > 0
      && intrinsics_(1, 1) > 0
      && intrinsics_(2, 2) > 0;
}

size_t PinholeCamera::width() const {
  return width_;
}

size_t PinholeCamera::height() const {
  return height_;
}

const CameraMatrix& PinholeCamera::intrinsics() const {
  return intrinsics_;
}

Vector2 PinholeCamera::projectPoint(const Vector3& hom_point_camera) const {
  Vector2 point_camera(hom_point_camera.hnormalized());
  Vector2 point_image;
  point_image(0) = intrinsics_(0, 0) * point_camera(0) + intrinsics_(0, 2);
  point_image(1) = intrinsics_(1, 1) * point_camera(1) + intrinsics_(1, 2);
  return point_image;
}

Vector3 PinholeCamera::getCameraRay(FloatType x, FloatType y) const {
  Vector3 camera_ray;
  camera_ray(0) = (x - intrinsics_(0, 2)) / intrinsics_(0, 0);
  camera_ray(1) = (y - intrinsics_(1, 2)) / intrinsics_(1, 1);
  camera_ray(2) = 1;
  return camera_ray;
}

Vector3 PinholeCamera::getCameraRay(const Vector2& point_image) const {
  Vector3 camera_ray;
  camera_ray(0) = (point_image(0) - intrinsics_(0, 2)) / intrinsics_(0, 0);
  camera_ray(1) = (point_image(1) - intrinsics_(1, 2)) / intrinsics_(1, 1);
  camera_ray(2) = 1;
  return camera_ray;
}

Vector3 PinholeCamera::unprojectPoint(FloatType x, FloatType y, FloatType distance) const {
  Vector2 point_image;
  point_image << x, y;
  return unprojectPoint(point_image, distance);
}

Vector3 PinholeCamera::unprojectPoint(const Vector2& point_image, FloatType distance) const {
  Vector3 hom_point_camera;
  hom_point_camera(0) = (point_image(0) - intrinsics_(0, 2)) / intrinsics_(0, 0);
  hom_point_camera(1) = (point_image(1) - intrinsics_(1, 2)) / intrinsics_(1, 1);
  hom_point_camera(2) = 1;
  hom_point_camera *= distance;
  return hom_point_camera;
}

FloatType PinholeCamera::getMeanFocalLength() const {
  return (intrinsics_(0, 0) + intrinsics_(1, 1)) / 2.0;
}

FloatType PinholeCamera::getFocalLengthX() const {
  return intrinsics_(0, 0);
}

FloatType PinholeCamera::getFocalLengthY() const {
  return intrinsics_(1, 1);
}

bool PinholeCamera::isPointInViewport(const Vector2& point) const {
  return point(0) >= 0 && point(0) < width_
          && point(1) >= 0 && point(1) < height_;
}

bool PinholeCamera::isPointInViewport(const Vector2& point, FloatType margin) const {
  return point(0) >= margin && point(0) < width_ - margin
          && point(1) >= margin && point(1) < height_ - margin;
}

PinholeCameraColmap::PinholeCameraColmap(CameraId id, size_t width, size_t height, const std::vector<FloatType>& params)
: PinholeCamera(width, height, makeIntrinsicsFromParameters(params)), id_(id) {}

CameraId PinholeCameraColmap::id() const {
  return id_;
}

CameraMatrix PinholeCameraColmap::makeIntrinsicsFromParameters(const std::vector<FloatType>& params) {
  if (params.size() != 4) {
    throw AIT_EXCEPTION(std::string("Expected 4 parameters but got ") + std::to_string(params.size()));
  }
  CameraMatrix intrinsics;
  intrinsics.setIdentity();
  intrinsics(0, 0) = params[0];
  intrinsics(1, 1) = params[1];
  intrinsics(0, 2) = params[2];
  intrinsics(1, 2) = params[3];
  return intrinsics;
}


ImageColmap::ImageColmap(
    ImageId id, const Pose& pose, const std::string& name,
    const std::vector<Feature>& features, CameraId camera_id)
: id_(id), pose_(pose), name_(name), features_(features), camera_id_(camera_id) {}

ImageId ImageColmap::id() const {
  return id_;
}

const Pose& ImageColmap::pose() const {
  return pose_;
}

const std::string& ImageColmap::name() const {
  return name_;
}

const std::vector<Feature>& ImageColmap::features() const {
  return features_;
}

const CameraId& ImageColmap::camera_id() const {
  return camera_id_;
}


Point3DStatistics::Point3DStatistics()
: average_distance_(std::numeric_limits<FloatType>::quiet_NaN()),
  stddev_distance_(std::numeric_limits<FloatType>::quiet_NaN()),
  stddev_one_minus_dot_product_(std::numeric_limits<FloatType>::quiet_NaN()) {}

Point3DStatistics::Point3DStatistics(FloatType average_distance, FloatType stddev_distance,
    FloatType stddev_one_minus_dot_product)
: average_distance_(average_distance), stddev_distance_(stddev_distance),
  stddev_one_minus_dot_product_(stddev_one_minus_dot_product) {}

FloatType Point3DStatistics::averageDistance() const {
  return average_distance_;
}

FloatType Point3DStatistics::stddevDistance() const {
  return stddev_distance_;
}

FloatType Point3DStatistics::stddevOneMinusDotProduct() const {
  return stddev_one_minus_dot_product_;
}

Point3DId Point3D::getId() const {
  return id;
}

const Vector3& Point3D::getPosition() const {
  return pos;
}

const Vector3& Point3D::getNormal() const {
  return normal;
}

const Point3DStatistics& Point3D::getStatistics() const {
  return statistics;
}

SparseReconstruction::SparseReconstruction() {}

SparseReconstruction::~SparseReconstruction() {}

void SparseReconstruction::read(const std::string& path) {
  cameras_.clear();
  images_.clear();
  points3D_.clear();
  readCameras(ait::joinPaths(path, "cameras.txt"));
  readImages(ait::joinPaths(path, "images.txt"));
  readPoints3D(ait::joinPaths(path, "points3D.txt"));
  readGpsTransformation(ait::joinPaths(path, "gps_transformation.txt"));
}

const CameraMapType& SparseReconstruction::getCameras() const {
  return cameras_;
}

CameraMapType& SparseReconstruction::getCameras() {
  return cameras_;
}

const ImageMapType& SparseReconstruction::getImages() const {
  return images_;
}

ImageMapType& SparseReconstruction::getImages() {
  return images_;
}

const Point3DMapType& SparseReconstruction::getPoints3D() const {
  return points3D_;
}

Point3DMapType& SparseReconstruction::getPoints3D() {
  return points3D_;
}

const SfmToGpsTransformation& SparseReconstruction::sfmGpsTransformation() const {
  return sfm_gps_transformation_;
}

SfmToGpsTransformation& SparseReconstruction::sfmGpsTransformation() {
  return sfm_gps_transformation_;
}

void SparseReconstruction::computePoint3DNormalAndStatistics(Point3D& point) const {
    // Retrieve all observation distances and normals
  std::vector<FloatType> distances;
  std::vector<Vector3> normals;
  for (const auto& feature_entry : point.feature_track) {
    const ImageColmap& image = images_.at(feature_entry.image_id);
    std::tuple<FloatType, Vector3> result = ait::computeDistanceAndDirection(point.getPosition(), image.pose().getWorldPosition());
    distances.push_back(std::get<0>(result));
    normals.push_back(std::get<1>(result));
  }

  // Compute distance average and stddev
  FloatType average_distance = std::accumulate(distances.cbegin(), distances.cend(), 0.0) / distances.size();
  FloatType acc_squared_dist_deviations = std::accumulate(distances.cbegin(), distances.cend(), 0.0, [&](FloatType acc, FloatType dist) {
    FloatType deviation = dist - average_distance;
    return acc + deviation * deviation;
  });
  FloatType stddev_distance = std::sqrt(acc_squared_dist_deviations / (distances.size() - 1));

  // Compute normal average and dot product stddev
  Vector3 average_normal = std::accumulate(normals.cbegin(), normals.cend(), Vector3::Zero().eval()) / normals.size();
  FloatType acc_squared_1_minus_dot_product = std::accumulate(normals.cbegin(), normals.cend(), 0.0, [&](FloatType acc, const Vector3& normal) {
    FloatType one_minus_dot_product = 1.0 - normal.dot(average_normal);
    return acc + one_minus_dot_product * one_minus_dot_product;
  });
  FloatType stddev_1_minus_dot_product = std::sqrt(acc_squared_1_minus_dot_product / (distances.size() - 1));

  Point3DStatistics statistics(average_distance, stddev_distance, stddev_1_minus_dot_product);
  point.normal = average_normal;
  point.statistics = statistics;

//  std::cout << "point_id=" << point.getId() << std::endl;
//  std::cout << "    distance: " << average_distance << " +- " << stddev_distance << std::endl;
//  std::cout << "  normal dot: " << "+- " << stddev_dot_product << std::endl;
//  std::cout << "      normal: " << average_normal.transpose() << std::endl;
}

void SparseReconstruction::readCameras(std::string filename) {
  std::ifstream in(filename);
  if (!in) {
    throw AIT_EXCEPTION("Unable to open cameras file");
  }
  readCameras(in);
}

void SparseReconstruction::readCameras(std::istream& in) {
  std::string line;
  std::string item;

  while (std::getline(in, line)) {
    ait::trim(line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    // ID
    std::getline(line_stream, item, ' ');
    ait::trim(item);
    CameraId camera_id = boost::lexical_cast<CameraId>(item);

    // MODEL
    std::getline(line_stream, item, ' ');
    ait::trim(item);
    if (item != "PINHOLE") {
      throw AIT_EXCEPTION(std::string("Unsupported camera model: ") + item);
    }

    // WIDTH
    std::getline(line_stream, item, ' ');
    ait::trim(item);
    size_t width = boost::lexical_cast<size_t>(item);

    // HEIGHT
    std::getline(line_stream, item, ' ');
    ait::trim(item);
    size_t height = boost::lexical_cast<size_t>(item);

    // PARAMS
    std::vector<FloatType> params;
    while (!line_stream.eof()) {
      std::getline(line_stream, item, ' ');
      ait::trim(item);
      params.push_back(boost::lexical_cast<FloatType>(item));
    }

    PinholeCameraColmap camera(camera_id, width, height, params);

    cameras_.emplace(camera_id, camera);
  }
}

void SparseReconstruction::readImages(std::string filename) {
  std::ifstream in(filename);
  if (!in) {
    throw AIT_EXCEPTION("Unable to open images file");
  }
  readImages(in);
}

void SparseReconstruction::readImages(std::istream& in) {
  std::string line;
  std::string item;

  while (std::getline(in, line)) {
    ait::trim(line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');
    ImageId image_id = boost::lexical_cast<ImageId>(item);

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    FloatType qw = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream1, item, ' ');
    FloatType qx = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream1, item, ' ');
    FloatType qy = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream1, item, ' ');
    FloatType qz = boost::lexical_cast<FloatType>(item);

    Pose image_pose_world_to_image;
    image_pose_world_to_image.quaternion() = Quaternion(qw, qx, qy, qz);
    image_pose_world_to_image.quaternion().normalize();

    // TVEC
    std::getline(line_stream1, item, ' ');
    image_pose_world_to_image.translation()(0) = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream1, item, ' ');
    image_pose_world_to_image.translation()(1) = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream1, item, ' ');
    image_pose_world_to_image.translation()(2) = boost::lexical_cast<FloatType>(item);

    const Pose image_pose = image_pose_world_to_image.inverse();
//    std::cout << "image_pose: " << image_pose << std::endl;

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    CameraId camera_id = boost::lexical_cast<CameraId>(item);

    // NAME
    std::getline(line_stream1, item, ' ');
    std::string image_name = item;

    // POINTS2D
    std::getline(in, line);
    ait::trim(line);
    std::stringstream line_stream2(line);

    std::vector<Vector2> points;
    std::vector<Point3DId> point3D_ids;

    std::vector<Feature> image_features;
    while (!line_stream2.eof()) {
      Feature feature;

      std::getline(line_stream2, item, ' ');
      feature.point(0) = boost::lexical_cast<FloatType>(item);

      std::getline(line_stream2, item, ' ');
      feature.point(1) = boost::lexical_cast<FloatType>(item);

      std::getline(line_stream2, item, ' ');
      if (item == "-1") {
        feature.point3d_id = invalid_point3d_id;
      }
      else {
        feature.point3d_id = boost::lexical_cast<Point3DId>(item);
      }

      image_features.push_back(feature);
    }
    image_features.shrink_to_fit();

    ImageColmap image(image_id, image_pose, image_name, image_features, camera_id);
    //std::cout << "Image " << image_id << " has pose " << image_pose << std::endl;

    images_.emplace(image.id(), image);
  }
}

void SparseReconstruction::readPoints3D(std::string filename) {
  std::ifstream in(filename);
  if (!in) {
    throw AIT_EXCEPTION("Unable to open points3D file");
  }
  readPoints3D(in);
}

void SparseReconstruction::readPoints3D(std::istream& in) {
  std::string line;
  std::string item;

  while (std::getline(in, line)) {
    ait::trim(line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    Point3D point3D;

    // ID
    std::getline(line_stream, item, ' ');
    point3D.id = boost::lexical_cast<Point3DId>(item);

    // XYZ
    std::getline(line_stream, item, ' ');
    point3D.pos(0) = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream, item, ' ');
    point3D.pos(1) = boost::lexical_cast<FloatType>(item);

    std::getline(line_stream, item, ' ');
    point3D.pos(2) = boost::lexical_cast<FloatType>(item);

    // Color
    std::getline(line_stream, item, ' ');
    point3D.color.r() = static_cast<uint8_t>(boost::lexical_cast<int>(item));

    std::getline(line_stream, item, ' ');
    point3D.color.g() = static_cast<uint8_t>(boost::lexical_cast<int>(item));

    std::getline(line_stream, item, ' ');
    point3D.color.b() = static_cast<uint8_t>(boost::lexical_cast<int>(item));

    // ERROR
    std::getline(line_stream, item, ' ');
    point3D.error = boost::lexical_cast<FloatType>(item);

    // TRACK
    while (!line_stream.eof()) {
      Point3D::TrackEntry track_entry;

      std::getline(line_stream, item, ' ');
      ait::trim(item);
      if (item.empty()) {
        break;
      }
      track_entry.image_id = boost::lexical_cast<ImageId>(item);

      std::getline(line_stream, item, ' ');
      track_entry.feature_index = boost::lexical_cast<FeatureId>(item);

      point3D.feature_track.push_back(track_entry);
    }
    point3D.feature_track.shrink_to_fit();
    computePoint3DNormalAndStatistics(point3D);

    points3D_.emplace(point3D.id, std::move(point3D));
  }
}

void SparseReconstruction::readGpsTransformation(std::string filename) {
  std::ifstream in(filename);
  if (!in) {
    throw AIT_EXCEPTION("Unable to open GPS transformation file");
  }
  readGpsTransformation(in);
}

void SparseReconstruction::readGpsTransformation(std::istream& in) {
  std::string line;
  std::string item;

  std::getline(in, line);
  ait::trim(line);
  while (line.empty() || line[0] == '#') {
    std::getline(in, line);
    ait::trim(line);
  }

  {
    // Scale factors
    std::stringstream line_stream1(line);
    std::getline(line_stream1, item, ' ');
    sfm_gps_transformation_.gps_scale = boost::lexical_cast<FloatType>(item);
    std::getline(line_stream1, item, ' ');
    sfm_gps_transformation_.sfm_scale = boost::lexical_cast<FloatType>(item);
    std::getline(line_stream1, item, ' ');
    sfm_gps_transformation_.gps_to_sfm_ratio = boost::lexical_cast<FloatType>(item);
  }

  std::getline(in, line);
  ait::trim(line);
  {
    // SFM centroid factors
    std::stringstream line_stream1(line);
    for (std::size_t i = 0; i < 3; ++i) {
      std::getline(line_stream1, item, ' ');
      sfm_gps_transformation_.sfm_centroid(i) = boost::lexical_cast<FloatType>(item);
    }
  }

  std::getline(in, line);
  ait::trim(line);
  {
    // GPS centroid factors
    std::stringstream line_stream1(line);
    for (std::size_t i = 0; i < 3; ++i) {
      std::getline(line_stream1, item, ' ');
      sfm_gps_transformation_.gps_centroid(i) = boost::lexical_cast<FloatType>(item);
    }
  }

  std::getline(in, line);
  ait::trim(line);
  {
    // SFM to GPS quaternion
    std::stringstream line_stream1(line);
    std::getline(line_stream1, item, ' ');
    FloatType qw = boost::lexical_cast<FloatType>(item);
    std::getline(line_stream1, item, ' ');
    FloatType qx = boost::lexical_cast<FloatType>(item);
    std::getline(line_stream1, item, ' ');
    FloatType qy = boost::lexical_cast<FloatType>(item);
    std::getline(line_stream1, item, ' ');
    FloatType qz = boost::lexical_cast<FloatType>(item);
    sfm_gps_transformation_.sfm_to_gps_quaternion = Quaternion(qw, qx, qy, qz);
  }

  std::getline(in, line);
  ait::trim(line);
  {
    // GPS reference coordinates
    std::stringstream line_stream1(line);
    Eigen::Matrix<GpsFloatType, 3, 1> gps_vec;
    for (std::size_t i = 0; i < 3; ++i) {
      std::getline(line_stream1, item, ' ');
      gps_vec(i) = boost::lexical_cast<GpsFloatType>(item);
    }
    sfm_gps_transformation_.gps_reference = SfmToGpsTransformation::GpsCoordinate(gps_vec);
  }
}
