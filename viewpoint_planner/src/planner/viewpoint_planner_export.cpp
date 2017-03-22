//==================================================
// viewpoint_planner_export.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 6, 2017
//==================================================

#include <ait/filesystem.h>
#include <boost/filesystem.hpp>
#include <rapidjson/document.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include "viewpoint_planner.h"

#pragma GCC optimize("O0")

rapidjson::Document ViewpointPlanner::getViewpointPathAsJson(const ViewpointPath& viewpoint_path) const {
  std::cout << "Converting viewpoint path to JSON" << std::endl;
  BH_ASSERT(hasReconstruction());
  BH_ASSERT(hasGpsTransformation());

  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = ait::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = data_->reconstruction_->sfmGpsTransformation().gps_reference;
  std::cout << "GPS reference: " << gps_reference << std::endl;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);

  using rapidjson::Document;
  using rapidjson::Value;
  using rapidjson::kArrayType;
  using rapidjson::kObjectType;
  Document d;
  d.SetObject();
  auto& allocator = d.GetAllocator();

  Value viewpoints(kArrayType);
  using IteratorType = typename std::vector<std::size_t>::const_iterator;
  const auto generate_json_viewpoint_lambda = [&](IteratorType prev_it, IteratorType it) {
    const ViewpointPathEntry& path_entry = viewpoint_path.entries[*it];
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
    const GpsCoordinateType gps = gps_converter.convertEnuToGps(viewpoint_entry.viewpoint.pose().getWorldPosition().cast<GpsFloatType>());
    Value json_pose(kObjectType);
    const GpsFloatType latitude = gps.latitude();
    const GpsFloatType longitude = gps.longitude();
    const GpsFloatType altitude = gps.altitude();
    // East is yaw = 0, south is yaw = 90, rotation axis is pointing in z-direction
    const Vector3 viewing_direction = viewpoint_entry.viewpoint.pose().rotation().col(2);
    const FloatType yaw_radians = -std::atan2(viewing_direction(1), viewing_direction(0));
    const FloatType camera_pitch_radians = std::atan2(viewing_direction(2), viewing_direction.topRows(2).squaredNorm());
    const FloatType yaw = ait::radiansToDegrees(yaw_radians);
    const FloatType camera_yaw = 0;
    const FloatType camera_pitch = ait::radiansToDegrees(camera_pitch_radians);
    json_pose.AddMember("latitude", latitude, allocator);
    json_pose.AddMember("longitude", longitude, allocator);
    json_pose.AddMember("altitude", altitude, allocator);
    json_pose.AddMember("yaw", yaw, allocator);
    Value json_path_array(kArrayType);
    if (prev_it != it) {
      const ViewpointEntryIndex prev_idx = viewpoint_path.entries[*prev_it].viewpoint_index;
      const ViewpointEntryIndex idx = viewpoint_path.entries[*it].viewpoint_index;
      if (!hasViewpointMotion(prev_idx, idx)) {
        std::cout << "No viewpoint motion between " << prev_idx << " and " << idx << std::endl;
      }
      const ViewpointMotion motion = getViewpointMotion(prev_idx, idx);
      for (const SE3Motion& se3_motion : motion.se3Motions()) {
        for (auto path_it = se3_motion.poses.begin(); path_it != se3_motion.poses.end(); ++path_it) {
          const GpsCoordinateType path_gps = gps_converter.convertEnuToGps(path_it->getWorldPosition().cast<GpsFloatType>());
          Value json_pose(kObjectType);
          const GpsFloatType latitude = path_gps.latitude();
          const GpsFloatType longitude = path_gps.longitude();
          const GpsFloatType altitude = path_gps.altitude();
          Value json_path_entry(kObjectType);
          json_path_entry.AddMember("index", path_it - se3_motion.poses.begin(), allocator);
          json_path_entry.AddMember("latitude", latitude, allocator);
          json_path_entry.AddMember("longitude", longitude, allocator);
          json_path_entry.AddMember("altitude", altitude, allocator);
          // TODO: Use actual yaw from motion?
          json_path_entry.AddMember("yaw", yaw, allocator);
          json_path_array.PushBack(json_path_entry, allocator);
        }
      }
    }
    else {
      Value json_path_entry(kObjectType);
      json_path_entry.AddMember("index", 0, allocator);
      json_path_entry.AddMember("latitude", latitude, allocator);
      json_path_entry.AddMember("longitude", longitude, allocator);
      json_path_entry.AddMember("altitude", altitude, allocator);
      // TODO: Use actual yaw from motion?
      json_path_entry.AddMember("yaw", yaw, allocator);
      json_path_array.PushBack(json_path_entry, allocator);
    }
    Value json_viewpoint(kObjectType);
    json_viewpoint.AddMember("index", *it, allocator);
    json_viewpoint.AddMember("pose", json_pose, allocator);
    json_viewpoint.AddMember("path", json_path_array, allocator);
    json_viewpoint.AddMember("camera_yaw", camera_yaw, allocator);
    json_viewpoint.AddMember("camera_pitch", camera_pitch, allocator);
    json_viewpoint.AddMember("take_picture", true, allocator);
    json_viewpoint.AddMember("mvs", path_entry.mvs_viewpoint, allocator);
    std::cout << "i=" << *it << ", latitude=" << latitude << ", longitude=" << longitude << std::endl;
    return json_viewpoint;
  };

  if (viewpoint_path.order.size() > 1) {
    // TODO: Handle start position to first viewpoint
    for (auto it = viewpoint_path.order.begin(); it != viewpoint_path.order.end(); ++it) {
      auto prev_it = it - 1;
      if (it == viewpoint_path.order.begin()) {
        prev_it = it;
      }
      Value json_viewpoint = generate_json_viewpoint_lambda(prev_it, it);
      const bool take_picture = true;
      json_viewpoint.AddMember("take_picture", take_picture, allocator);
      viewpoints.PushBack(json_viewpoint, allocator);
    }
    Value json_viewpoint = generate_json_viewpoint_lambda(
        viewpoint_path.order.begin() + viewpoint_path.order.size() - 1, viewpoint_path.order.begin());
    const bool take_picture = false;
    json_viewpoint.AddMember("take_picture", take_picture, allocator);
  }

  d.AddMember("viewpoints", viewpoints, allocator);
  return d;
}

std::string ViewpointPlanner::getViewpointPathAsJsonString(const ViewpointPath& viewpoint_path) const {
  rapidjson::Document d = getViewpointPathAsJson(viewpoint_path);
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  d.Accept(writer);
  return buffer.GetString();
}

void ViewpointPlanner::exportViewpointPathAsJson(const std::string& filename, const ViewpointPath& viewpoint_path) const {
  rapidjson::Document d = getViewpointPathAsJson(viewpoint_path);
  std::cout << "Exporting viewpoint path as JSON to" << filename << std::endl;
  std::ofstream ofs(filename);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  d.Accept(writer);
}

void ViewpointPlanner::exportViewpointPathAsText(const std::string& filename, const ViewpointPath& viewpoint_path) const {
  std::cout << "Converting viewpoint path to text" << std::endl;
  std::ofstream ofs(filename);
  ofs << "# Camera: width, height, fx, fy" << std::endl;
  ofs << "# Camera: intrinsics" << std::endl;
  ofs << "# Viewpoints: idx, mvs, Qx, Qy, Qz, Qw, Tx, Ty, Tz" << std::endl;
  const PinholeCamera& camera = getVirtualCamera();
  ofs << camera.width() << ", " << camera.height() << ", "
      << camera.getFocalLengthX() << ", " << camera.getFocalLengthY() << std::endl;
  for (std::size_t i = 0; i < static_cast<std::size_t>(camera.intrinsics().size()); ++i) {
    if (i > 0) {
      ofs << ", ";
    }
    ofs << camera.intrinsics()(i);
  }
  ofs << std::endl;
  std::vector<std::size_t> order = viewpoint_path.order;
  if (order.empty()) {
    for (std::size_t i = 0; i < viewpoint_path.entries.size(); ++i) {
      order.push_back(i);
    }
  }
  for (auto it = order.begin(); it != order.end(); ++it) {
    const ViewpointPathEntry& path_entry = viewpoint_path.entries[*it];
    const Pose& pose = path_entry.viewpoint.pose();
    ofs << path_entry.viewpoint_index << ", "
        << path_entry.mvs_viewpoint << ", "
        << pose.quaternion().x() << ", " << pose.quaternion().y() << ", "
        << pose.quaternion().z() << ", " << pose.quaternion().w() << ", "
        << pose.translation()(0) << ", " << pose.translation()(1) << ", " << pose.translation()(2) << std::endl;
    auto next_it = it + 1;
    if (next_it == order.end()) {
      next_it = order.begin();
    }
    if (hasViewpointMotion(path_entry.viewpoint_index, viewpoint_path.entries[*next_it].viewpoint_index)) {
      const ViewpointMotion motion = getViewpointMotion(path_entry.viewpoint_index, viewpoint_path.entries[*next_it].viewpoint_index);
      for (auto motion_it = motion.viewpointIndices().begin() + 1; motion_it != motion.viewpointIndices().end() - 1; ++motion_it) {
        const ViewpointEntryIndex viewpoint_index = *motion_it;
        const bool mvs_viewpoint = false;
        const Pose& pose = viewpoint_entries_[viewpoint_index].viewpoint.pose();
        ofs << viewpoint_index << ", "
            << mvs_viewpoint << ", "
            << pose.quaternion().x() << ", " << pose.quaternion().y() << ", "
            << pose.quaternion().z() << ", " << pose.quaternion().w() << ", "
            << pose.translation()(0) << ", " << pose.translation()(1) << ", " << pose.translation()(2) << std::endl;
      }
    }
  }
}

void ViewpointPlanner::exportViewpointPathAsSparseReconstruction(
    const std::string& path, const ViewpointPath& viewpoint_path) const {
  std::cout << "Converting viewpoint path to sparse reconstruction" << std::endl;
  const std::size_t camera_id = 1;

  std::ofstream ofs(ait::joinPaths(path, "cameras.txt"));
  ofs << "# Camera list with one line of data per camera:" << std::endl;
  ofs << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]" << std::endl;
  ofs << "# Number of cameras: " << viewpoint_path.entries.size() << std::endl;
  ofs << camera_id << " SIMPLE_RADIAL " << getVirtualCamera().width() << " " << getVirtualCamera().height() << " "
      << getVirtualCamera().getFocalLengthX() << " "
      << getVirtualCamera().width() / 2 << " " << getVirtualCamera().height() / 2 << " " << 0 << std::endl;
  ofs.close();

  ofs.open(ait::joinPaths(path, "images.txt"));
  ofs << "# Image list with two lines of data per image:" << std::endl;
  ofs << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME" << std::endl;
  ofs << "#   POINTS2D[] as (X, Y, POINT3D_ID)" << std::endl;
  ofs << "# Number of images: " << viewpoint_path.entries.size() << ", mean observations per image: 1" << std::endl;
  for (std::size_t i = 0; i < viewpoint_path.entries.size(); ++i) {
    const ViewpointPathEntry& path_entry = viewpoint_path.entries[i];
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
    const Pose image_pose_world_to_image = viewpoint_entry.viewpoint.pose().inverse();
    const FloatType qw = image_pose_world_to_image.quaternion().w();
    const FloatType qx = image_pose_world_to_image.quaternion().x();
    const FloatType qy = image_pose_world_to_image.quaternion().y();
    const FloatType qz = image_pose_world_to_image.quaternion().z();
    const FloatType tx = image_pose_world_to_image.translation()(0);
    const FloatType ty = image_pose_world_to_image.translation()(1);
    const FloatType tz = image_pose_world_to_image.translation()(2);
    const std::string name = "HighresScreenshot000";
    ofs << i << " " << qw << " " << qx << " " << qy << " " << qz << " " << tx << " " << ty << " " << tz << " "
        << camera_id << " " << name << (95 + 2*i) << ".png" << std::endl;
    const FloatType point_x = 0;
    const FloatType point_y = 0;
    std::size_t point3d_id = i;
    ofs << point_x << " " << point_y << " " << point3d_id << std::endl;
  }
  ofs.close();

  ofs.open(ait::joinPaths(path, "points3D.txt"));
  ofs << "# 3D point list with one line of data per point:" << std::endl;
  ofs << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)" << std::endl;
  ofs << "# Number of points: " << viewpoint_path.entries.size() << ", mean track length: 0" << std::endl;
  for (std::size_t i = 0; i < viewpoint_path.entries.size(); ++i) {
    const FloatType x = 0;
    const FloatType y = 0;
    const FloatType z = 0;
    const int r = 0;
    const int g = 255;
    const int b = 0;
    const FloatType error = 0;
    ofs << i << " " << x << " " << y << " " << z << " " << r << " " << g << " " << b << " "
        << error << " " << i << 0 << std::endl;
  }
  ofs.close();
}
