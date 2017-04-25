//==================================================
// viewpoint_score.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.03.17
//

#include <functional>
#include "viewpoint_planner_types.h"
#include "occupied_tree.h"
#include "viewpoint.h"
#include <bh/config_options.h>

namespace viewpoint_planner {

class ViewpointScore {
public:
  struct Options : bh::ConfigOptions {
    Options() {
      addOption<FloatType>("voxel_sensor_size_ratio_threshold", &voxel_sensor_size_ratio_threshold);
      addOption<FloatType>("voxel_sensor_size_ratio_inv_falloff_factor", &voxel_sensor_size_ratio_inv_falloff_factor);
      addOption<bool>("incidence_ignore_dot_product_sign", &incidence_ignore_dot_product_sign);
      addOption<FloatType>("incidence_angle_threshold_degrees", &incidence_angle_threshold_degrees);
      addOption<FloatType>("incidence_angle_inv_falloff_factor_degrees", &incidence_angle_inv_falloff_factor_degrees);
    }

    ~Options() override {}

    // Relative voxel size on sensor above which full information is achieved
    FloatType voxel_sensor_size_ratio_threshold = FloatType(0.002);
    // Inverse information falloff factor above the sensor size threshold
    FloatType voxel_sensor_size_ratio_inv_falloff_factor = FloatType(0.001);

    // Whether to ignore the dot-product sign when computing incidence information factor
    bool incidence_ignore_dot_product_sign = false;
    // Incidence angle below which the full information is achieved
    FloatType incidence_angle_threshold_degrees = 30;
    // Inverse information falloff factor above the incidence angle threshold
    FloatType incidence_angle_inv_falloff_factor_degrees = 30;
  };

  ViewpointScore(
          const Options& options,
          std::function<Vector3(const Viewpoint&, const VoxelType*, const Vector2&)> normal_vector_function);

  WeightType computeResolutionInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  WeightType computeIncidenceInformationFactor(
          const Viewpoint& viewpoint, const VoxelType* node, const Vector3& normal_vector) const;

  WeightType computeIncidenceInformationFactor(
          const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const;

  WeightType computeIncidenceInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  FloatType computeViewpointObservationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  FloatType computeViewpointObservationFactor(
          const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const;

  FloatType  computeViewpointObservationScore(
          const Viewpoint& viewpoint,
          const OccupiedTreeType::IntersectionResult& ir) const;

  FloatType computeViewpointObservationScore(
          const Viewpoint& viewpoint,
          const OccupiedTreeType::IntersectionResultWithScreenCoordinates& ir) const;

  FloatType computeViewpointObservationScore(
          const Viewpoint& viewpoint, const VoxelType* node) const;

  FloatType computeViewpointObservationScore(
          const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const;

private:
  Options options_;
  FloatType voxel_sensor_size_ratio_falloff_factor_;
  FloatType incidence_angle_threshold_;
  FloatType incidence_angle_falloff_factor_;
  std::function<Vector3(const Viewpoint&, const VoxelType*, const Vector2&)> normal_vector_function_;
};

}
