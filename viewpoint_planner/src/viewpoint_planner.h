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
#include <octomap/OcTree.h>
#include "sparse_reconstruction.h"

class Viewpoint
{
public:
    static constexpr double DEFAULT_PROJECTION_MARGIN = 10;
    static constexpr double MAX_DISTANCE_DEVIATION_BY_STDDEV = 3;
    static constexpr double MAX_NORMAL_DEVIATION_BY_STDDEV = 3;

    Viewpoint(const PinholeCamera* camera, const Pose& pose, double projection_margin=DEFAULT_PROJECTION_MARGIN)
    : camera_(camera), pose_(pose), projection_margin_(projection_margin),
      transformation_world_to_image_(pose_.getTransformationWorldToImage()) {}

    std::unordered_set<Point3DId> getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const {
        ait::Timer timer;
        std::unordered_set<Point3DId> proj_point_ids;
        for (const auto& entry : points) {
            const Point3DId point_id = entry.first;
            const Point3D& point_world = entry.second;
            Eigen::Vector2d point_image = projectWorldPointIntoImage(point_world.pos);
            bool projected = camera_->isPointInViewport(point_image, projection_margin_);
            if (projected) {
                proj_point_ids.insert(point_id);
            }
        }
        timer.printTiming("getProjectedPoint3DIds");
        return proj_point_ids;
    }

    std::unordered_set<Point3DId> getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const {
        ait::Timer timer;
        std::unordered_set<Point3DId> proj_point_ids;
        for (const auto& entry : points) {
            const Point3DId point_id = entry.first;
            const Point3D& point_world = entry.second;
            Eigen::Vector2d point_image = projectWorldPointIntoImage(point_world.getPosition());
            bool projected = camera_->isPointInViewport(point_image, projection_margin_);
            if (projected) {
                if (!isPointFiltered(point_world)) {
                    proj_point_ids.insert(point_id);
                }
            }
        }
        timer.printTiming("getProjectedPoint3DIdsFiltered");
        return proj_point_ids;
    }

    bool isPointFiltered(const Point3D& point) const {
        const Point3DStatistics& statistics = point.getStatistics();
        std::tuple<double, Eigen::Vector3d> result = ait::computeDistanceAndDirection(point.getPosition(), pose_.getWorldPosition());
        double dist_deviation = std::abs(std::get<0>(result) - statistics.averageDistance());
        if (dist_deviation > MAX_DISTANCE_DEVIATION_BY_STDDEV * statistics.stddevDistance()) {
            return true;
        }
        double one_minus_dot_product = 1 - std::get<1>(result).dot(point.getNormal());
        double one_minus_dot_deviation = std::abs(one_minus_dot_product);
        if (one_minus_dot_deviation > MAX_NORMAL_DEVIATION_BY_STDDEV * statistics.stddevOneMinusDotProduct()) {
            return true;
        }
        return false;
    }

    Eigen::Vector2d projectWorldPointIntoImage(const Eigen::Vector3d& point_world) const {
        Eigen::Vector3d point_camera  = transformation_world_to_image_ * point_world.homogeneous();
        Eigen::Vector2d point_image = camera_->projectPoint(point_camera);
        return point_image;
    }

private:
    const PinholeCamera* camera_;
    const Pose pose_;
    const double projection_margin_;
    const Eigen::Matrix3x4d transformation_world_to_image_;
};

class ViewpointPlanner
{
public:
    static constexpr double OCCLUSION_DIST_MARGIN_FACTOR = 4;

    using OctomapType = octomap::OcTree;

    ViewpointPlanner(std::unique_ptr<SparseReconstruction> sparse_recon, std::unique_ptr<OctomapType> octree)
    : sparse_recon_(std::move(sparse_recon)), octree_(std::move(octree)) {

    }

    const OctomapType* getOctree() const {
        return octree_.get();
    }

//    OctomapType* getOctree() {
//        return octree_.get();
//    }

    const SparseReconstruction* getSparseReconstruction() const {
        return sparse_recon_.get();
    }

    std::unordered_set<Point3DId> computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const {
        Viewpoint viewpoint(&sparse_recon_->getCameras().at(camera_id), pose, projection_margin);
        std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(sparse_recon_->getPoints3D());
        return proj_points;
    }

    std::unordered_set<Point3DId> computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const {
        Viewpoint viewpoint(&sparse_recon_->getCameras().at(camera_id), pose, projection_margin);
        std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(sparse_recon_->getPoints3D());
        return proj_points;
    }

    std::unordered_set<Point3DId> computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const {
        Viewpoint viewpoint(&sparse_recon_->getCameras().at(camera_id), pose, projection_margin);
        std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(sparse_recon_->getPoints3D());
        removeOccludedPoints(pose, proj_points, OCCLUSION_DIST_MARGIN_FACTOR * octree_->getResolution());
        return proj_points;
    }

    std::unordered_set<Point3DId> computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
            double projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const {
        Viewpoint viewpoint(&sparse_recon_->getCameras().at(camera_id), pose, projection_margin);
        std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(sparse_recon_->getPoints3D());
        removeOccludedPoints(pose, proj_points, OCCLUSION_DIST_MARGIN_FACTOR * octree_->getResolution());
        return proj_points;
    }

    void run() {
      return;
        // Check how many map points fall in an occupied voxel
//        for (size_t max_depth = 1; max_depth <= octree_->getTreeDepth(); ++max_depth) {
//            size_t num_map_point_nodes = 0;
//            double average_occupancy_level = 0.0;
//            for (const auto& entry : sparse_recon_->getPoints3D()) {
//                const Point3D& point_world = entry.second;
//                const OctomapType::NodeType* node = octree_->search(point_world.pos(0), point_world.pos(1), point_world.pos(2), max_depth);
//                if (node == nullptr) {
////                    std::cerr << "ERROR: Map point could not be found in octree" << std::endl;
//                }
//                else {
//                    average_occupancy_level += node->getOccupancy();
//                    ++num_map_point_nodes;
//                }
//            }
//            average_occupancy_level /= num_map_point_nodes;
//            std::cout << "Search depth: " << max_depth << std::endl;
//            std::cout << "  Found " << num_map_point_nodes << " map point nodes out of " << sparse_recon_->getPoints3D().size() << " points" << std::endl;
//            std::cout << "  Average map point occupancy: " << average_occupancy_level << std::endl;
//        }

        // Check how many map points are projected into each image without being occluded
        for (const auto& entry : sparse_recon_->getImages()) {
          ait::Timer timer2;
          const Image& image = entry.second;
          const PinholeCamera& camera = sparse_recon_->getCameras().at(image.camera_id);
          std::cout << "image_id=" << image.id << std::endl;
          std::cout << "  features: " << image.features.size() << std::endl;
          const double projection_margin = 0;
          std::unordered_set<Point3DId> proj_points = computeProjectedMapPoints(camera.id(), image.pose, projection_margin);
          std::unordered_set<Point3DId> filtered_points = computeFilteredMapPoints(camera.id(), image.pose, projection_margin);
          std::unordered_set<Point3DId> visible_points = computeVisibleMapPoints(camera.id(), image.pose, projection_margin);
          ait::Timer timer;
          std::unordered_set<Point3DId> filtered_visible_points = setIntersection(filtered_points, visible_points);
          timer.printTiming("Intersection computation");
//            std::unordered_set<Point3DId> filtered_points(proj_points);
//            std::unordered_set<Point3DId> visible_points(proj_points);
//            removeOccludedPoints(image.pose, visible_points);
          std::cout << "  projected points: " << proj_points.size() << std::endl;
          std::cout << "  filtered points: " << filtered_points.size() << std::endl;
          std::cout << "  non-occluded points: " << visible_points.size() << std::endl;
          std::cout << "  filtered and non-occluded: " << setIntersectionSize(filtered_points, visible_points) << std::endl;
          std::unordered_set<Point3DId> feature_map_points;
          for (const auto& feature : image.features) {
              if (feature.point3d_id != invalid_point3d_id) {
                  feature_map_points.insert(feature.point3d_id);
              }
          }
          timer = ait::Timer();
          std::cout << "  Features with map point: " << feature_map_points.size() << std::endl;
          std::cout << "    and projected: " << setIntersectionSize(proj_points, feature_map_points) << std::endl;
          std::cout << "    and filtered: " << setIntersectionSize(filtered_points, feature_map_points) << std::endl;
          std::cout << "    and non-occluded: " << setIntersectionSize(visible_points, feature_map_points) << std::endl;
          std::cout << "    and filtered and non-occluded: " << setIntersectionSize(filtered_visible_points, feature_map_points) << std::endl;
          timer.printTiming("Intersection size computation");
//            AIT_ASSERT(found_features == features_with_map_point);
          timer2.printTiming("loop");
        }
    }

    template <typename Set1, typename Set2>
    size_t setIntersectionSize(const Set1& set1, const Set2& set2) {
        static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
        if (set1.size() > set2.size()) {
            return setIntersectionSize(set2, set1);
        }
        size_t size = 0;
        for (const auto& key : set1) {
            if (set2.find(key) != set2.cend()) {
                ++size;
            }
        }
        return size;
    }

    template <typename Set1, typename Set2>
    Set1 setIntersection(const Set1& set1, const Set2& set2) {
        static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
        if (set1.size() > set2.size()) {
            return setIntersection(set2, set1);
        }
        Set1 intersection;
        for (const auto& key : set1) {
            if (set2.find(key) != set2.cend()) {
                intersection.insert(key);
            }
        }
        return intersection;
    }

private:
    void removeOccludedPoints(const Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, double dist_margin) const {
        ait::Timer timer;
        Pose pose_image_to_world = pose_world_to_image.inverse();
        Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
        octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
        octomap::KeyRay key_ray;
        octomap::OcTreeKey origin_key;
        if (!octree_->coordToKeyChecked(origin, origin_key)) {
            point3D_ids.clear();
            return;
        }
        for (auto it = point3D_ids.begin(); it != point3D_ids.end(); ) {
            const Point3D& point3D = sparse_recon_->getPoints3D().at(*it);
            octomap::point3d end_point(point3D.pos(0), point3D.pos(1), point3D.pos(2));
            octomap::point3d end_point_from_origin = end_point - origin;
            octomap::OcTreeKey end_point_key;
            bool occluded = false;
            if (!octree_->coordToKeyChecked(end_point, end_point_key)) {
                occluded = true;
            } else {
//                bool success = octree_->computeRayKeys(origin, end_point, key_ray);
                double end_point_dist = end_point_from_origin.norm();
                octomap::point3d hit_point;
                const bool ignore_unknown = true;
                octree_->castRay(origin, end_point_from_origin, hit_point, ignore_unknown, end_point_dist);
                double hit_dist = (hit_point - origin).norm();
                if (end_point_key != octree_->coordToKey(hit_point) && hit_dist + dist_margin < end_point_dist) {
                    occluded = true;
                }
                else {
                    occluded = false;
                }
            }
            if (occluded) {
                it = point3D_ids.erase(it);
            }
            else {
                ++it;
            }
        }
        timer.printTiming("removeOccludedPoints");
    }

    std::unique_ptr<SparseReconstruction> sparse_recon_;
    std::unique_ptr<OctomapType> octree_;
};
