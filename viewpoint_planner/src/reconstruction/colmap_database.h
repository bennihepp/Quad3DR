//==================================================
// colmap_database.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 14, 2016
//==================================================
#pragma once

#include <iostream>
#include <sstream>
#include <unordered_map>
#include <boost/functional.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <ait/eigen_alignment.h>
#include <ait/common.h>
#include <ait/utilities.h>
#include <ait/string_utils.h>
#include <ait/boost_utilities.h>
#include <ait/pose.h>

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

//using CameraId = size_t;
//using ImageId = size_t;
//using Point3DId = size_t;
//using FeatureId = size_t;
//
//using CameraMatrix = Eigen::Matrix4d;
//
//class PinholeCamera
//{
//public:
//  PinholeCamera()
//  : width_(0), height_(0), intrinsics_(CameraMatrix::Zero()) {}
//
//  PinholeCamera(size_t width, size_t height, const CameraMatrix& intrinsics)
//    : width_(width), height_(height), intrinsics_(intrinsics) {}
//
//    size_t width() const {
//        return width_;
//    }
//
//    size_t height() const {
//        return height_;
//    }
//
//    const CameraMatrix& intrinsics() const {
//        return intrinsics_;
//    }
//
//    Eigen::Vector2d projectPoint(const Eigen::Vector3d& hom_point_camera) const {
//      Eigen::Vector2d point_camera(hom_point_camera.hnormalized());
//      Eigen::Vector2d point_image;
//      point_image(0) = intrinsics_(0, 0) * point_camera(0) + intrinsics_(0, 2);
//      point_image(1) = intrinsics_(1, 1) * point_camera(1) + intrinsics_(1, 2);
//      return point_image;
//    }
//
//    Eigen::Vector3d getCameraRay(double x, double y) const {
//      Eigen::Vector3d camera_ray;
//      camera_ray(0) = (x - intrinsics_(0, 2)) / intrinsics_(0, 0);
//      camera_ray(1) = (y - intrinsics_(1, 2)) / intrinsics_(1, 1);
//      camera_ray(2) = 1;
//      return camera_ray;
//    }
//
//    Eigen::Vector3d getCameraRay(const Eigen::Vector2d& point_image) const {
//      Eigen::Vector3d camera_ray;
//      camera_ray(0) = (point_image(0) - intrinsics_(0, 2)) / intrinsics_(0, 0);
//      camera_ray(1) = (point_image(1) - intrinsics_(1, 2)) / intrinsics_(1, 1);
//      camera_ray(2) = 1;
//      return camera_ray;
//    }
//
//    Eigen::Vector3d unprojectPoint(double x, double y, double distance) const {
//      Eigen::Vector2d point_image;
//      point_image << x, y;
//      return unprojectPoint(point_image, distance);
//    }
//
//    Eigen::Vector3d unprojectPoint(const Eigen::Vector2d& point_image, double distance) const {
//      Eigen::Vector3d hom_point_camera;
//      hom_point_camera(0) = (point_image(0) - intrinsics_(0, 2)) / intrinsics_(0, 0);
//      hom_point_camera(1) = (point_image(1) - intrinsics_(1, 2)) / intrinsics_(1, 1);
//      hom_point_camera(2) = 1;
//      hom_point_camera *= distance;
//      return hom_point_camera;
//    }
//
//    double getMeanFocalLength() const {
//        return (intrinsics_(0, 0) + intrinsics_(1, 1)) / 2.0;
//    }
//
//    double getFocalLengthX() const {
//        return intrinsics_(0, 0);
//    }
//
//    double getFocalLengthY() const {
//        return intrinsics_(1, 1);
//    }
//
//    bool isPointInViewport(const Eigen::Vector2d& point) const {
//        return point(0) >= 0 && point(0) < width_
//                && point(1) >= 0 && point(1) < height_;
//    }
//
//    bool isPointInViewport(const Eigen::Vector2d& point, double margin) const {
//        return point(0) >= margin && point(0) < width_ - margin
//                && point(1) >= margin && point(1) < height_ - margin;
//    }
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//private:
//    size_t width_;
//    size_t height_;
//    CameraMatrix intrinsics_;
//};
//
//class PinholeCameraColmap : public PinholeCamera
//{
//public:
//    PinholeCameraColmap(CameraId id, size_t width, size_t height, const std::vector<double>& params)
//    : PinholeCamera(width, height, makeIntrinsicsFromParameters(params)), id_(id) {}
//
//    CameraId id() const {
//        return id_;
//    }
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//private:
//    static CameraMatrix makeIntrinsicsFromParameters(const std::vector<double>& params) {
//        if (params.size() != 4) {
//            throw AIT_EXCEPTION(std::string("Expected 4 parameters but got ") + std::to_string(params.size()));
//        }
//        CameraMatrix intrinsics;
//        intrinsics.setIdentity();
//        intrinsics(0, 0) = params[0];
//        intrinsics(1, 1) = params[1];
//        intrinsics(0, 2) = params[2];
//        intrinsics(1, 2) = params[3];
//        return intrinsics;
//    }
//
//    CameraId id_;
//};
//
//struct Feature
//{
//    Eigen::Vector2d point;
//    Point3DId point3d_id;
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};
//
//struct ImageColmap
//{
//    ImageId id;
//    Pose pose;
//    std::string name;
//    std::vector<Feature> features;
//    CameraId camera_id;
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};
//
//struct Color : public Eigen::Matrix<uint8_t, 3, 1>
//{
//    uint8_t& r() { return (*this)(0); };
//    uint8_t& g() { return (*this)(1); };
//    uint8_t& b() { return (*this)(2); };
//    const uint8_t& r() const { return (*this)(0); };
//    const uint8_t& g() const { return (*this)(1); };
//    const uint8_t& b() const { return (*this)(2); };
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};
//
//class Point3DStatistics
//{
//public:
//    Point3DStatistics()
//    : average_distance_(std::numeric_limits<double>::quiet_NaN()),
//      stddev_distance_(std::numeric_limits<double>::quiet_NaN()),
//      stddev_one_minus_dot_product_(std::numeric_limits<double>::quiet_NaN()) {}
//
//    Point3DStatistics(double average_distance, double stddev_distance,
//            double stddev_one_minus_dot_product)
//    : average_distance_(average_distance), stddev_distance_(stddev_distance),
//      stddev_one_minus_dot_product_(stddev_one_minus_dot_product) {}
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//    double averageDistance() const {
//        return average_distance_;
//    }
//
//    double stddevDistance() const {
//        return stddev_distance_;
//    }
//
//    double stddevOneMinusDotProduct() const {
//        return stddev_one_minus_dot_product_;
//    }
//
//private:
//    double average_distance_;
//    double stddev_distance_;
//    double stddev_one_minus_dot_product_;
//};
//
//struct Point3D
//{
//    struct TrackEntry
//    {
//        ImageId image_id;
//        FeatureId feature_index;
//    };
//
//    Point3DId getId() const {
//        return id;
//    }
//
//    const Eigen::Vector3d& getPosition() const {
//        return pos;
//    }
//
//    const Eigen::Vector3d& getNormal() const {
//        return normal;
//    }
//
//    const Point3DStatistics& getStatistics() const {
//        return statistics;
//    }
//
//    Point3DId id;
//    Eigen::Vector3d pos;
//    Color color;
//    double error;
//    std::vector<TrackEntry> feature_track;
//    Eigen::Vector3d normal;
//    Point3DStatistics statistics;
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};
//
//const Point3DId invalid_point3d_id = std::numeric_limits<Point3DId>::max();
//
//class SparseReconstruction
//{
//public:
//    using CameraMapType = EIGEN_ALIGNED_UNORDERED_MAP(CameraId, PinholeCameraColmap);
//    using ImageMapType = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, ImageColmap);
//    using Point3DMapType = EIGEN_ALIGNED_UNORDERED_MAP(Point3DId, Point3D);
//
//    SparseReconstruction() {}
//
//    virtual ~SparseReconstruction() {}
//
//    virtual void read(const std::string& path) {
//        cameras_.clear();
//        images_.clear();
//        points3D_.clear();
//        readCameras(ait::joinPaths(path, "cameras.txt"));
//        readImages(ait::joinPaths(path, "images.txt"));
//        readPoints3D(ait::joinPaths(path, "points3D.txt"));
//    }
//
//    const CameraMapType& getCameras() const {
//        return cameras_;
//    }
//
//    CameraMapType& getCameras() {
//        return cameras_;
//    }
//
//    const ImageMapType& getImages() const {
//        return images_;
//    }
//
//    ImageMapType& getImages() {
//        return images_;
//    }
//
//    const Point3DMapType& getPoints3D() const {
//        return points3D_;
//    }
//
//    Point3DMapType& getPoints3D() {
//        return points3D_;
//    }
//
//private:
//    void computePoint3DNormalAndStatistics(Point3D& point) const {
//        // Retrieve all observation distances and normals
//        std::vector<double> distances;
//        std::vector<Eigen::Vector3d> normals;
//        for (const auto& feature_entry : point.feature_track) {
//            const ImageColmap& image = images_.at(feature_entry.image_id);
//            std::tuple<double, Eigen::Vector3d> result = ait::computeDistanceAndDirection(point.getPosition(), image.pose.getWorldPosition());
//            distances.push_back(std::get<0>(result));
//            normals.push_back(std::get<1>(result));
//        }
//
//        // Compute distance average and stddev
//        double average_distance = std::accumulate(distances.cbegin(), distances.cend(), 0.0) / distances.size();
//        double acc_squared_dist_deviations = std::accumulate(distances.cbegin(), distances.cend(), 0.0, [&](double acc, double dist) {
//            double deviation = dist - average_distance;
//            return acc + deviation * deviation;
//        });
//        double stddev_distance = std::sqrt(acc_squared_dist_deviations / (distances.size() - 1));
//
//        // Compute normal average and dot product stddev
//        Eigen::Vector3d average_normal = std::accumulate(normals.cbegin(), normals.cend(), Eigen::Vector3d::Zero().eval()) / normals.size();
//        double acc_squared_1_minus_dot_product = std::accumulate(normals.cbegin(), normals.cend(), 0.0, [&](double acc, const Eigen::Vector3d& normal) {
//            double one_minus_dot_product = 1.0 - normal.dot(average_normal);
//            return acc + one_minus_dot_product * one_minus_dot_product;
//        });
//        double stddev_1_minus_dot_product = std::sqrt(acc_squared_1_minus_dot_product / (distances.size() - 1));
//
//        Point3DStatistics statistics(average_distance, stddev_distance, stddev_1_minus_dot_product);
//        point.normal = average_normal;
//        point.statistics = statistics;
//
////        std::cout << "point_id=" << point.getId() << std::endl;
////        std::cout << "    distance: " << average_distance << " +- " << stddev_distance << std::endl;
////        std::cout << "  normal dot: " << "+- " << stddev_dot_product << std::endl;
////        std::cout << "      normal: " << average_normal.transpose() << std::endl;
//    }
//
//    void readCameras(std::string filename) {
//        std::ifstream in(filename);
//        if (!in) {
//            throw AIT_EXCEPTION("Unable to open cameras file");
//        }
//        readCameras(in);
//    }
//
//    void readCameras(std::istream& in) {
//        std::string line;
//        std::string item;
//
//        while (std::getline(in, line)) {
//            ait::trim(line);
//
//            if (line.empty() || line[0] == '#') {
//                continue;
//            }
//
//            std::stringstream line_stream(line);
//
//            // ID
//            std::getline(line_stream, item, ' ');
//            ait::trim(item);
//            CameraId camera_id = boost::lexical_cast<CameraId>(item);
//
//            // MODEL
//            std::getline(line_stream, item, ' ');
//            ait::trim(item);
//            if (item != "PINHOLE") {
//              throw AIT_EXCEPTION(std::string("Unsupported camera model: ") + item);
//            }
//
//            // WIDTH
//            std::getline(line_stream, item, ' ');
//            ait::trim(item);
//            size_t width = boost::lexical_cast<size_t>(item);
//
//            // HEIGHT
//            std::getline(line_stream, item, ' ');
//            ait::trim(item);
//            size_t height = boost::lexical_cast<size_t>(item);
//
//            // PARAMS
//            std::vector<double> params;
//            while (!line_stream.eof()) {
//                std::getline(line_stream, item, ' ');
//                ait::trim(item);
//                params.push_back(boost::lexical_cast<double>(item));
//            }
//
//            PinholeCameraColmap camera(camera_id, width, height, params);
//
//            cameras_.emplace(camera_id, camera);
//        }
//
//    }
//
//    void readImages(std::string filename) {
//        std::ifstream in(filename);
//        if (!in) {
//            throw AIT_EXCEPTION("Unable to open images file");
//        }
//        readImages(in);
//    }
//
//    void readImages(std::istream& in) {
//        std::string line;
//        std::string item;
//
//        while (std::getline(in, line)) {
//            ait::trim(line);
//
//            if (line.empty() || line[0] == '#') {
//                continue;
//            }
//
//            std::stringstream line_stream1(line);
//
//            ImageColmap image;
//
//            // ID
//            std::getline(line_stream1, item, ' ');
//            image.id = boost::lexical_cast<ImageId>(item);
//
//            // QVEC (qw, qx, qy, qz)
//            std::getline(line_stream1, item, ' ');
//            double qw = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream1, item, ' ');
//            double qx = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream1, item, ' ');
//            double qy = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream1, item, ' ');
//            double qz = boost::lexical_cast<double>(item);
//
//            image.pose.quaternion() = Eigen::Quaterniond(qw, qx, qy, qz);
//            image.pose.quaternion().normalize();
//
//            // TVEC
//            std::getline(line_stream1, item, ' ');
//            image.pose.translation()(0) = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream1, item, ' ');
//            image.pose.translation()(1) = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream1, item, ' ');
//            image.pose.translation()(2) = boost::lexical_cast<double>(item);
//
//            // CAMERA_ID
//            std::getline(line_stream1, item, ' ');
//            image.camera_id = boost::lexical_cast<CameraId>(item);
//
//            // NAME
//            std::getline(line_stream1, item, ' ');
//            image.name = item;
//
//            // POINTS2D
//            std::getline(in, line);
//            ait::trim(line);
//            std::stringstream line_stream2(line);
//
//            std::vector<Eigen::Vector2d> points;
//            std::vector<Point3DId> point3D_ids;
//
//            while (!line_stream2.eof()) {
//                Feature feature;
//
//                std::getline(line_stream2, item, ' ');
//                feature.point(0) = boost::lexical_cast<double>(item);
//
//                std::getline(line_stream2, item, ' ');
//                feature.point(1) = boost::lexical_cast<double>(item);
//
//                std::getline(line_stream2, item, ' ');
//                if (item == "-1") {
//                    feature.point3d_id = invalid_point3d_id;
//                }
//                else {
//                    feature.point3d_id = boost::lexical_cast<Point3DId>(item);
//                }
//
//                image.features.push_back(feature);
//            }
//            image.features.shrink_to_fit();
//
//            images_[image.id] = image;
//        }
//    }
//
//    void readPoints3D(std::string filename) {
//        std::ifstream in(filename);
//        if (!in) {
//            throw AIT_EXCEPTION("Unable to open points3D file");
//        }
//        readPoints3D(in);
//    }
//
//    void readPoints3D(std::istream& in) {
//        std::string line;
//        std::string item;
//
//        while (std::getline(in, line)) {
//            ait::trim(line);
//
//            if (line.empty() || line[0] == '#') {
//                continue;
//            }
//
//            std::stringstream line_stream(line);
//
//            Point3D point3D;
//
//            // ID
//            std::getline(line_stream, item, ' ');
//            point3D.id = boost::lexical_cast<Point3DId>(item);
//
//            // XYZ
//            std::getline(line_stream, item, ' ');
//            point3D.pos(0) = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream, item, ' ');
//            point3D.pos(1) = boost::lexical_cast<double>(item);
//
//            std::getline(line_stream, item, ' ');
//            point3D.pos(2) = boost::lexical_cast<double>(item);
//
//            // Color
//            std::getline(line_stream, item, ' ');
//            point3D.color.r() = static_cast<uint8_t>(boost::lexical_cast<int>(item));
//
//            std::getline(line_stream, item, ' ');
//            point3D.color.g() = static_cast<uint8_t>(boost::lexical_cast<int>(item));
//
//            std::getline(line_stream, item, ' ');
//            point3D.color.b() = static_cast<uint8_t>(boost::lexical_cast<int>(item));
//
//            // ERROR
//            std::getline(line_stream, item, ' ');
//            point3D.error = boost::lexical_cast<double>(item);
//
//            // TRACK
//            while (!line_stream.eof()) {
//                Point3D::TrackEntry track_entry;
//
//                std::getline(line_stream, item, ' ');
//                ait::trim(item);
//                if (item.empty()) {
//                break;
//                }
//                track_entry.image_id = boost::lexical_cast<ImageId>(item);
//
//                std::getline(line_stream, item, ' ');
//                track_entry.feature_index = boost::lexical_cast<FeatureId>(item);
//
//                point3D.feature_track.push_back(track_entry);
//            }
//            point3D.feature_track.shrink_to_fit();
//            computePoint3DNormalAndStatistics(point3D);
//
//            points3D_.emplace(point3D.id, std::move(point3D));
//        }
//    }
//
//    CameraMapType cameras_;
//    ImageMapType images_;
//    Point3DMapType points3D_;
//};
