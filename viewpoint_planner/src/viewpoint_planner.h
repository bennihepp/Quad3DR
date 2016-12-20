//==================================================
// viewpoint_planner.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <memory>
#include <unordered_set>
#include <algorithm>
#include <type_traits>
#include <ait/common.h>
#include <ait/eigen_utils.h>
#include <ait/vision_utilities.h>
//#include <octomap/OcTree.h>
#include "occupancy_map.h"
#include "dense_reconstruction.h"
#include "bvh/bvh.h"

struct Ray {
  Ray(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction)
  : origin_(origin), direction_(direction.normalized()) {}

  Ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
  : origin_(origin.cast<float>()), direction_(direction.normalized().cast<float>()) {}

  const Eigen::Vector3f& origin() const {
    return origin_;
  }

  const Eigen::Vector3f& direction() const {
    return direction_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Eigen::Vector3f origin_;
  Eigen::Vector3f direction_;
};

class Viewpoint
{
public:
    static constexpr double DEFAULT_PROJECTION_MARGIN = 10;
    static constexpr double MAX_DISTANCE_DEVIATION_BY_STDDEV = 3;
    static constexpr double MAX_NORMAL_DEVIATION_BY_STDDEV = 3;

    Viewpoint(const PinholeCamera* camera, const Pose& pose, double projection_margin=DEFAULT_PROJECTION_MARGIN);

    const Pose& pose() const {
      return pose_;
    }

    const PinholeCamera& camera() const {
      return *camera_;
    }

    std::unordered_set<Point3DId> getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const;

    std::unordered_set<Point3DId> getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const;

    bool isPointFiltered(const Point3D& point) const;

    Eigen::Vector2d projectWorldPointIntoImage(const Eigen::Vector3d& point_world) const;

    Ray getCameraRay(double x, double y) const;

private:
    const PinholeCamera* camera_;
    const Pose pose_;
    const double projection_margin_;
    const Eigen::Matrix3x4d transformation_world_to_image_;
};

struct TestNode {

};

class ViewpointPlanner;

class ViewpointPlannerMaps {
public:
  using InputOccupancyMapType = OccupancyMap<OccupancyNode>;
  using OccupiedTreeType = bvh::Tree<TestNode, float>;

  ViewpointPlannerMaps(const InputOccupancyMapType* octree);

private:
  friend class ViewpointPlanner;

  OccupiedTreeType occupied_bvh_;
};

class ViewpointPlanner
{
public:
    static constexpr double OCCLUSION_DIST_MARGIN_FACTOR = 4;
    static constexpr double OCCUPANCY_WEIGHT_DEPTH = 12;
    static constexpr double OCCUPANCY_WEIGHT_REACH = 2;
    static constexpr double OCCUPANCY_WEIGHT_DEPTH_CUTOFF = 16;

//    using OctomapType = octomap::OcTree;
    using InputOccupancyMapType = OccupancyMap<OccupancyNode>;
    using OccupancyMapType = OccupancyMap<AugmentedOccupancyNode>;
    using TreeNavigatorType = TreeNavigator<OccupancyMapType, OccupancyMapType::NodeType>;
    using ConstTreeNavigatorType = TreeNavigator<const OccupancyMapType, const OccupancyMapType::NodeType>;
    using OccupancyType = OccupancyMapType::NodeType::OccupancyType;
    using CounterType = OccupancyMapType::NodeType::CounterType;
    using WeightType = OccupancyMapType::NodeType::WeightType;

    static std::unique_ptr<OccupancyMapType> getAugmentedOccupancyMap(
        std::unique_ptr<InputOccupancyMapType> input_tree);

    ViewpointPlanner(std::unique_ptr<DenseReconstruction> dense_recon, std::unique_ptr<OccupancyMapType> octree,
        std::unique_ptr<ViewpointPlannerMaps> maps);

    const OccupancyMapType* getOctree() const {
        return octree_.get();
    }

    const DenseReconstruction* getReconstruction() const {
        return reconstruction_.get();
    }

    void setVirtualCamera(size_t virtual_width, size_t virtual_height, const Eigen::Matrix4d& virtual_intrinsics) {
      virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
      std::cout << "virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
    }

    void setScaledVirtualCamera(CameraId camera_id, double scale_factor) {
      const PinholeCameraColmap& camera = reconstruction_->getCameras().at(camera_id);
      size_t virtual_width = static_cast<size_t>(scale_factor * camera.width());
      size_t virtual_height = static_cast<size_t>(scale_factor * camera.height());
      CameraMatrix virtual_intrinsics = ait::getScaledIntrinsics(camera.intrinsics(), scale_factor);
      virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
      std::cout << "Virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
    }

    void run();

    template <typename T>
    T rayCastAccumulate(const CameraId camera_id, const Pose& pose_world_to_image, bool ignore_unknown,
        std::function<T(bool, const octomap::point3d&)> func, T init = 0) const;

    template <typename T>
    T rayCastAccumulate(const Viewpoint& viewpoint, bool ignore_unknown,
        std::function<T(bool, const octomap::point3d&)> func, T init = 0) const;

    std::vector<std::pair<ViewpointPlannerMaps::OccupiedTreeType::NodeType*, float>> getRaycastHitVoxelsBVH(
        const Pose& pose_world_to_image) const;

    std::vector<std::pair<ConstTreeNavigatorType, float>> getRaycastHitVoxels(const Pose& pose_world_to_image) const;

    double computeInformationScore(const Pose& pose) const;

    std::unordered_set<Point3DId> computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

    std::unordered_set<Point3DId> computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

    std::unordered_set<Point3DId> computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

    std::unordered_set<Point3DId> computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

    template <typename Set1, typename Set2>
    size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2) const;

    template <typename Set1, typename Set2>
    Set1 computeSetIntersection(const Set1& set1, const Set2& set2) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    template <typename TreeT>
    static bool isTreeConsistent(const TreeT& tree);

    void removeOccludedPoints(const Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, double dist_margin) const;

    static WeightType computeWeightContribution(
        const Eigen::Vector3f& query_pos, float dist_cutoff_sq, const ConstTreeNavigatorType& nav);
    static WeightType computeObservationWeightFactor(CounterType observation_count);
    static WeightType computeObservationInformationFactor(CounterType observation_count);

    std::unique_ptr<DenseReconstruction> reconstruction_;
    std::unique_ptr<OccupancyMapType> octree_;
    std::unique_ptr<ViewpointPlannerMaps> maps_;

    PinholeCamera virtual_camera_;
};

//template <typename T>
//T ViewpointPlanner::rayCastAccumulate(const CameraId camera_id, const Pose& pose_world_to_image, bool ignore_unknown,
//    std::function<T(bool, const octomap::point3d&)> func, T init) const {
//  const PinholeCamera& camera = reconstruction_->getCameras().at(camera_id);
//  Viewpoint viewpoint(&camera, pose_world_to_image);
//  return rayCastAccumulate(viewpoint, ignore_unknown, func, init);
//}
//
//template <typename T>
//T ViewpointPlanner::rayCastAccumulate(const Viewpoint& viewpoint, bool ignore_unknown,
//    std::function<T(bool, const octomap::point3d&)> acc_func, T init) const {
//  ait::Timer timer;
//  Pose pose_image_to_world = viewpoint.pose().inverse();
//  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
//  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
//  octomap::OcTreeKey origin_key;
//  if (!octree_->coordToKeyChecked(origin, origin_key)) {
//    std::cout << "WARNING: Requesting ray cast for out of bounds viewpoint" << std::endl;
//    return std::numeric_limits<double>::quiet_NaN();
//  }
//  T acc = init;
//  for (size_t y = 0; y < virtual_height_; ++y) {
//    std::cout << "Raycasting row row " << y << std::endl;
//    for (size_t x = 0; x < virtual_width_; ++x) {
//      Eigen::Vector3d ray = viewpoint.camera().getCameraRay(x, y);
//      octomap::point3d direction(ray(0), ray(1), ray(2));
////      double max_range = -1.0;
//      octomap::point3d hit_point;
////      bool result = octree_->castRay(origin, direction, hit_point, ignore_unknown);
//      bool result = octree_->castRayFast(origin, direction, hit_point, ignore_unknown);
//      // TODO
//      acc += func(result, hit_point);
//    }
//  }
//  timer.printTiming("rayCastAccumulate");
//  return acc;
//}
