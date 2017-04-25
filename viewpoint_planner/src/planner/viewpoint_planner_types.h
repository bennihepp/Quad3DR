//==================================================
// viewpoint_planner_types.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.03.17
//

#pragma once

#include <bh/eigen.h>
#include <bh/mLib/mLib.h>
#include <bh/common.h>
#include <bh/math/geometry.h>
#include <bh/pose.h>
#include "../octree/occupancy_map.h"
#include "../bvh/bvh.h"

namespace viewpoint_planner {

using FloatType = float;

USE_FIXED_EIGEN_TYPES(FloatType);

using RayType = bh::Ray<FloatType>;
using Pose = bh::Pose<FloatType>;

using PointCloudIOType = ml::PointCloudIO<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using MeshIOType = ml::MeshIO<FloatType>;
using MeshType = ml::MeshData<FloatType>;
using BoundingBoxType = bh::BoundingBox3D<FloatType>;
using Vector3 = Eigen::Vector3f;
using Vector3i = Eigen::Vector3i;
using DistanceFieldType = ml::DistanceField3f;
using RegionType = bh::PolygonWithLowerAndUpperPlane<FloatType>;

using RawOccupancyMapType = OccupancyMap<OccupancyNode>;
using OccupancyMapType = OccupancyMap<AugmentedOccupancyNode>;

using TreeNavigatorType = TreeNavigator<OccupancyMapType, OccupancyMapType::NodeType>;
using ConstTreeNavigatorType = TreeNavigator<const OccupancyMapType, const OccupancyMapType::NodeType>;
using CounterType = OccupancyMapType::NodeType::CounterType;
using WeightType = OccupancyMapType::NodeType::WeightType;

}
