//==================================================
// viewpoint_planner.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <ait/eigen.h>
#include <memory>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <ait/common.h>
#include <ait/options.h>
#include <ait/serialization.h>
#include "../occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
#include "viewpoint_planner_data.h"
#include "../graph/graph.h"

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

class Viewpoint {
public:
  using FloatType = ait::Pose::FloatType;

  static constexpr FloatType DEFAULT_PROJECTION_MARGIN = 10;
  static constexpr FloatType MAX_DISTANCE_DEVIATION_BY_STDDEV = 3;
  static constexpr FloatType MAX_NORMAL_DEVIATION_BY_STDDEV = 3;

  Viewpoint(FloatType projection_margin = DEFAULT_PROJECTION_MARGIN);

  Viewpoint(const PinholeCamera* camera, const ait::Pose& pose, FloatType projection_margin = DEFAULT_PROJECTION_MARGIN);

  Viewpoint(const Viewpoint& other);

  Viewpoint(Viewpoint&& other);

  Viewpoint& operator=(const Viewpoint& other);

  Viewpoint& operator=(Viewpoint&& other);

  const ait::Pose& pose() const {
    return pose_;
  }

  const PinholeCamera& camera() const {
    return *camera_;
  }

  FloatType getDistanceTo(const Viewpoint& other) const;

  FloatType getDistanceTo(const Viewpoint& other, FloatType rotation_factor) const;

  std::unordered_set<Point3DId> getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const;

  std::unordered_set<Point3DId> getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const;

  bool isPointFiltered(const Point3D& point) const;

  Eigen::Vector2d projectWorldPointIntoImage(const Eigen::Vector3d& point_world) const;

  Ray getCameraRay(FloatType x, FloatType y) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const PinholeCamera* camera_;
  ait::Pose pose_;
  FloatType projection_margin_;
  Eigen::Matrix3x4d transformation_world_to_image_;
};

class ViewpointPlanner {
public:
  using FloatType = ViewpointPlannerData::FloatType;
  using Vector3 = ViewpointPlannerData::Vector3;
  using BoundingBoxType = ViewpointPlannerData::BoundingBoxType;

  struct Options : ait::ConfigOptions {
    Options()
    : ait::ConfigOptions("viewpoint_planner", "ViewpointPlanner options") {
      addOption<size_t>("rng_seed", 0);
      addOption<FloatType>("virtual_camera_scale", 0.05f);
      addOption<size_t>("num_sampled_poses", 100);
      addOption<size_t>("num_planned_viewpoints", 10);
    }

    ~Options() override {}
  };

  static constexpr FloatType OCCLUSION_DIST_MARGIN_FACTOR = 4;

  using MeshType = ViewpointPlannerData::MeshType;
//    using OctomapType = octomap::OcTree;
  using InputOccupancyMapType = OccupancyMap<OccupancyNode>;
  using OccupancyMapType = OccupancyMap<AugmentedOccupancyNode>;
  using TreeNavigatorType = TreeNavigator<OccupancyMapType, OccupancyMapType::NodeType>;
  using ConstTreeNavigatorType = TreeNavigator<const OccupancyMapType, const OccupancyMapType::NodeType>;
  using OccupancyType = OccupancyMapType::NodeType::OccupancyType;
  using CounterType = OccupancyMapType::NodeType::CounterType;
  using WeightType = OccupancyMapType::NodeType::WeightType;
  using VoxelType = ViewpointPlannerData::OccupiedTreeType::NodeType;

  struct VoxelWithInformation {
    VoxelWithInformation(const VoxelType* voxel, const FloatType information)
    : voxel(voxel), information(information) {}

    const VoxelType* voxel;
    FloatType information;

    bool operator==(const VoxelWithInformation& other) const {
      return voxel == other.voxel && information == other.information;
    }

    struct Hash {
      std::size_t operator()(const VoxelWithInformation& voxel_with_information) const {
        std::size_t val { 0 };
              boost::hash_combine(val, voxel_with_information.voxel);
              boost::hash_combine(val, voxel_with_information.information);
              return val;
      }
    };
  };

  using VoxelWithInformationSet = std::unordered_set<VoxelWithInformation, VoxelWithInformation::Hash>;

  struct ViewpointNode {
    ViewpointNode()
    : total_information(0) {}

    ViewpointNode(const Viewpoint& viewpoint, const FloatType total_information,
        const VoxelWithInformationSet& voxel_set)
    : viewpoint(viewpoint), total_information(total_information), voxel_set(voxel_set) {}

    ViewpointNode(const Viewpoint& viewpoint, const FloatType total_information,
        VoxelWithInformationSet&& voxel_set)
    : viewpoint(viewpoint), total_information(total_information), voxel_set(std::move(voxel_set)) {}

    ViewpointNode(const ViewpointNode& other)
    : viewpoint(other.viewpoint), total_information(other.total_information),
      voxel_set(other.voxel_set) {}

    ViewpointNode(ViewpointNode&& other)
    : viewpoint(std::move(other.viewpoint)), total_information(other.total_information),
      voxel_set(std::move(other.voxel_set)) {}

    ViewpointNode& operator=(const ViewpointNode& other) {
      if (this != &other) {
        viewpoint = other.viewpoint;
        total_information = other.total_information;
        voxel_set = other.voxel_set;
      }
      return *this;
    }

    ViewpointNode& operator=(ViewpointNode&& other) {
      if (this != &other) {
        viewpoint = std::move(other.viewpoint);
        total_information = other.total_information;
        voxel_set = std::move(other.voxel_set);
      }
      return *this;
    }

    // Members cannot be constant because std::sort requires move-assignment
    Viewpoint viewpoint;
    FloatType total_information;
    VoxelWithInformationSet voxel_set;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct ViewpointPathEntry {
    ViewpointPathEntry(const ViewpointNode* node, FloatType information)
    : node(node), information(information) {}

    const ViewpointNode* node;
    FloatType information;
  };

  using ViewpointGraph = Graph<ViewpointNode, FloatType>;
  using ViewpointPath = std::vector<ViewpointPathEntry>;

  ViewpointPlanner(const Options* options, std::unique_ptr<ViewpointPlannerData> data);

  const OccupancyMapType* getOctree() const {
      return data_->octree_.get();
  }

  const DenseReconstruction* getReconstruction() const {
      return data_->reconstruction_.get();
  }

  const MeshType* getMesh() const {
    return data_->poisson_mesh_.get();
  }

  const ViewpointGraph& getViewpointGraph() const {
    return viewpoint_graph_;
  }

  const ViewpointPath& getViewpointPath() const {
    return viewpoint_path_;
  }

  std::pair<bool, ait::Pose> samplePose(size_t max_trials = 500);

  std::pair<bool, ait::Pose> samplePose(const BoundingBoxType& bbox,
      const Vector3& object_extent, size_t max_trials = 500);

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(size_t max_trials = 500);

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const BoundingBoxType& bbox,
      const Vector3& object_extent, size_t max_trials = 500);

  void setVirtualCamera(size_t virtual_width, size_t virtual_height, const Eigen::Matrix4d& virtual_intrinsics);

  void setScaledVirtualCamera(CameraId camera_id, double scale_factor);

  void run();

  template <typename T>
  T rayCastAccumulate(const CameraId camera_id, const ait::Pose& pose_world_to_image, bool ignore_unknown,
      std::function<T(bool, const octomap::point3d&)> func, T init = 0) const;

  template <typename T>
  T rayCastAccumulate(const Viewpoint& viewpoint, bool ignore_unknown,
      std::function<T(bool, const octomap::point3d&)> func, T init = 0) const;

  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsBVH(
      const ait::Pose& pose_world_to_image) const;

  std::vector<std::pair<ConstTreeNavigatorType, float>> getRaycastHitVoxels(const ait::Pose& pose_world_to_image) const;

  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScoreBVH(const ait::Pose& pose_world_to_image) const;

  FloatType computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const;

  FloatType computeInformationScoreBVH(const ait::Pose& pose_world_to_image) const;

  FloatType computeInformationScore(const ait::Pose& pose_world_to_image) const;

  std::unordered_set<Point3DId> computeProjectedMapPoints(const CameraId camera_id, const ait::Pose& pose,
          double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeFilteredMapPoints(const CameraId camera_id, const ait::Pose& pose,
          double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPoints(const CameraId camera_id, const ait::Pose& pose,
          double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPointsFiltered(const CameraId camera_id, const ait::Pose& pose,
          double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  template <typename Set1, typename Set2>
  size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2) const;

  template <typename Set1, typename Set2>
  Set1 computeSetIntersection(const Set1& set1, const Set2& set2) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void removeOccludedPoints(const ait::Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, double dist_margin) const;

  static WeightType computeObservationWeightFactor(CounterType observation_count);
  static WeightType computeObservationInformationFactor(CounterType observation_count);

  Options options_;

  std::unique_ptr<ViewpointPlannerData> data_;
  std::mt19937_64 rng_;
  PinholeCamera virtual_camera_;

  ViewpointGraph viewpoint_graph_;
  ViewpointPath viewpoint_path_;
};

//template <typename T>
//T ViewpointPlanner::rayCastAccumulate(const CameraId camera_id, const ait::Pose& pose_world_to_image, bool ignore_unknown,
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
//  ait::Pose pose_image_to_world = viewpoint.pose().inverse();
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
