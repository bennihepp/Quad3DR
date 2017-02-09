/*
 * viewpoint_planner_data.cpp
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#include "viewpoint_planner_data.h"
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ait/common.h>
#include <ait/gps.h>
#include <ait/geometry.h>
#include <ait/nn/approximate_nearest_neighbor.h>
#include <Eigen/Core>

ViewpointPlannerData::ViewpointPlannerData(const Options* options)
: options_(*options) {
  bvh_bbox_ = BoundingBoxType(
      Vector3(options->getValue<FloatType>("bvh_bbox_min_x"),
          options->getValue<FloatType>("bvh_bbox_min_y"),
          options->getValue<FloatType>("bvh_bbox_min_z")),
      Vector3(options->getValue<FloatType>("bvh_bbox_max_x"),
          options->getValue<FloatType>("bvh_bbox_max_y"),
          options->getValue<FloatType>("bvh_bbox_max_z")));

  const std::string reconstruction_path = options->getValue<std::string>("dense_reconstruction_path");
  readDenseReconstruction(reconstruction_path);

  // Get ROI (for sampling and weight computation)
  if (options_.regions_json_filename.empty()) {
//    roi_bbox_ = BoundingBoxType(
//        Vector3(options->getValue<FloatType>("roi_bbox_min_x"),
//            options->getValue<FloatType>("roi_bbox_min_y"),
//            options->getValue<FloatType>("roi_bbox_min_z")),
//        Vector3(options->getValue<FloatType>("roi_bbox_max_x"),
//            options->getValue<FloatType>("roi_bbox_max_y"),
//            options->getValue<FloatType>("roi_bbox_max_z")));
    const FloatType roi_min_x = options->getValue<FloatType>("roi_bbox_min_x");
    const FloatType roi_min_y = options->getValue<FloatType>("roi_bbox_min_y");
    const FloatType roi_min_z = options->getValue<FloatType>("roi_bbox_min_z");
    const FloatType roi_max_x = options->getValue<FloatType>("roi_bbox_max_x");
    const FloatType roi_max_y = options->getValue<FloatType>("roi_bbox_max_y");
    const FloatType roi_max_z = options->getValue<FloatType>("roi_bbox_max_z");
    std::vector<RegionType::Vector2> vertices;
    vertices.push_back(RegionType::Vector2(roi_min_x, roi_min_y));
    vertices.push_back(RegionType::Vector2(roi_min_x, roi_max_y));
    vertices.push_back(RegionType::Vector2(roi_max_x, roi_max_y));
    vertices.push_back(RegionType::Vector2(roi_max_x, roi_min_y));
    roi_ = RegionType(vertices, roi_min_z, roi_max_z);
    std::cout << "ROI bounding box: " << roi_.getBoundingBox() << std::endl;
    // Test code
//    std::cout << roi_min_x << ", " << roi_min_y << std::endl;
//    std::cout << roi_min_x << ", " << roi_max_y << std::endl;
//    std::cout << roi_max_x << ", " << roi_max_y << std::endl;
//    std::cout << roi_max_x << ", " << roi_min_y << std::endl;
//    std::cout << roi_min_z << ", " << roi_max_z << std::endl;
//    std::cout << roi_.isPointInside(Vector3(0, 0, 0)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(50, 0, 0)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(0, 50, 0)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(0, 0, 50)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(200, 0, 0)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(200, 0, 0)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(0, -200, 0)) << std::endl;
//    std::cout << roi_.isPointInside(Vector3(0, 0, 150)) << std::endl;
//    std::cout << roi_.distanceToPoint(Vector3(0, 0, 0)) << std::endl;
//    std::cout << roi_.distanceToPoint(Vector3(200, 0, 0)) << std::endl;
//    std::cout << roi_.distanceToPoint(Vector3(0, 0, -150)) << std::endl;
//    std::cout << roi_.distanceToPoint(Vector3(150, 0, 150)) << std::endl;
  }
  else {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(options_.regions_json_filename, pt);
    std::cout << "Region of interest" << std::endl;
    roi_ = convertGpsRegionToEnuRegion(pt.get_child("regionOfInterest"));
    std::cout << "ROI bounding box: " << roi_.getBoundingBox() << std::endl;
    for (const boost::property_tree::ptree::value_type& v : pt.get_child("noFlyZones")) {
      std::cout << "No-Fly-Zones" << std::endl;
      no_fly_zones_.push_back(convertGpsRegionToEnuRegion(v.second));
    }
  }
  roi_bbox_ = roi_.getBoundingBox();

  const std::string raw_octree_filename = options->getValue<std::string>("raw_octree_filename");
  const std::string dense_points_filename = options->getValue<std::string>("dense_points_filename");
  const std::string mesh_filename = options->getValue<std::string>("poisson_mesh_filename");
  std::string octree_filename = options->getValue<std::string>("octree_filename");
  if (octree_filename.empty()) {
    octree_filename = raw_octree_filename + ".aug";
  }
  std::string bvh_filename = options->getValue<std::string>("bvh_filename");
  if (bvh_filename.empty()) {
    bvh_filename = octree_filename + ".bvh";
  }
  std::string df_filename = options->getValue<std::string>("distance_field_filename");
  if (df_filename.empty()) {
    df_filename = mesh_filename + ".df";
  }

  readDensePoints(dense_points_filename);
  readPoissonMesh(mesh_filename);
  bool augmented_octree_generated = readAndAugmentOctree(octree_filename, raw_octree_filename);
  bool bvh_generated = readBVHTree(bvh_filename, octree_filename);
  generateWeightGrid();
  bool df_generated = false;
  if (options_.use_distance_field) {
    df_generated = readMeshDistanceField(df_filename, mesh_filename);
  }
  bool update_weights = augmented_octree_generated || bvh_generated || df_generated
      || options_.force_weights_update;
  if (!options_.regions_json_filename.empty()) {
    if (getLastWriteTime(options_.regions_json_filename) > getLastWriteTime(bvh_filename)) {
      update_weights = true;
    }
    else if (getLastWriteTime(options_.regions_json_filename) > getLastWriteTime(octree_filename)) {
      update_weights = true;
    }
  }
  if (update_weights) {
    std::cout << "Updating weights" << std::endl;
    updateWeights();
    std::cout << "Writing updated augmented octree" << std::endl;
    octree_->write(octree_filename);
    std::cout << "Writing updated BVH tree" << std::endl;
    writeBVHTree(bvh_filename);
  }
}

ViewpointPlannerData::RegionType ViewpointPlannerData::convertGpsRegionToEnuRegion(const boost::property_tree::ptree& pt) const {
  const bool verbose = true;

  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = ait::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = reconstruction_->sfmGpsTransformation().gps_reference;
  std::cout << "GPS reference: " << gps_reference << std::endl;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);
  const FloatType lower_altitude = pt.get<FloatType>("lower_altitude");
  const FloatType upper_altitude = pt.get<FloatType>("upper_altitude");
  const FloatType ref_altitude = gps_reference.altitude();
  std::vector<GpsCoordinateType> gps_coordinates;
  for (const boost::property_tree::ptree::value_type& v : pt.get_child("latlong_vertices")) {
    gps_coordinates.push_back(GpsCoordinateType(v.second.get<FloatType>("latitude"), v.second.get<FloatType>("longitude"), ref_altitude));
  }
  std::vector<RegionType::Vector2> vertices;
  for (const GpsCoordinateType& gps : gps_coordinates) {
    const Vector3 enu = gps_converter.convertGpsToEnu(gps).cast<FloatType>();
    if (verbose) {
      std::cout << "GPS: " << gps << ", ENU: " << enu.transpose() << std::endl;
    }
    vertices.emplace_back(enu(0), enu(1));

    // Test code for GPS conversion
//    GpsCoordinateType gps2 = gps;
//    AIT_PRINT_VALUE(gps2);
//    AIT_PRINT_VALUE(gps_converter.convertGpsToEcef(gps2));
//    AIT_PRINT_VALUE(gps_converter.convertGpsToEnu(gps2));
//    gps2 = GpsCoordinateType(gps2.latitude(), gps2.longitude(), gps2.altitude() + 360);
//    AIT_PRINT_VALUE(gps2);
//    AIT_PRINT_VALUE(gps_converter.convertGpsToEcef(gps2));
//    AIT_PRINT_VALUE(gps_converter.convertGpsToEnu(gps2));
//    Vector3 enu2 = enu;
//    AIT_PRINT_VALUE(enu2);
//    AIT_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
//    enu2(2) = 360;
//    AIT_PRINT_VALUE(enu2);
//    AIT_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
//    enu2(2) = 0;
//    AIT_PRINT_VALUE(enu2);
//    AIT_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
  }
  if (verbose) {
    std::cout << "Lower altitude: " << lower_altitude << ", upper_altitude: " << upper_altitude << std::endl;
  }
  const RegionType region(vertices, lower_altitude, upper_altitude);
  return region;
}

bool ViewpointPlannerData::isValidObjectPosition(const Vector3& position, const Vector3& object_extent) const {
  for (const RegionType& no_fly_zone : no_fly_zones_) {
    if (no_fly_zone.isPointInside(position)) {
      return false;
    }
  }
  BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(position, object_extent);
  std::vector<ViewpointPlannerData::OccupiedTreeType::ConstBBoxIntersectionResult> results =
      occupied_bvh_.intersects(object_bbox);
  return results.empty();
}

void ViewpointPlannerData::readDenseReconstruction(const std::string& path) {
  std::cout << "Reading dense reconstruction workspace" << std::endl;
  ait::Timer timer;
  reconstruction_.reset(new reconstruction::DenseReconstruction());
  reconstruction_->read(path);
  timer.printTiming("Loading dense reconstruction");
}

std::unique_ptr<ViewpointPlannerData::RawOccupancyMapType>
ViewpointPlannerData::readRawOctree(const std::string& octree_filename, bool binary) const {
  ait::Timer timer;
  std::unique_ptr<RawOccupancyMapType> raw_octree;
  if (binary) {
//      octree.reset(new ViewpointPlanner::OccupancyMapType(filename));
    throw AIT_EXCEPTION("Binary occupancy maps not supported");
  }
  else {
    raw_octree = RawOccupancyMapType::read(octree_filename);
  }
  if (!raw_octree) {
    throw std::runtime_error("Unable to read octomap file");
  }
  timer.printTiming("Loading octree");

  std::cout << "Loaded octree" << std::endl;
  std::cout << "Octree has " << raw_octree->getNumLeafNodes() << " leaf nodes and " << raw_octree->size() << " total nodes" << std::endl;
  std::cout << "Metric extents:" << std::endl;
  double x, y, z;
  timer = ait::Timer();
  raw_octree->getMetricSize(x, y, z);
  timer.printTiming("Computing octree size");
  std::cout << "  size=(" << x << ", " << y << ", " << z << ")" << std::endl;
  timer = ait::Timer();
  raw_octree->getMetricMin(x, y, z);
  timer.printTiming("Computing octree min");
  std::cout << "   min=(" << x << ", " << y << ", " << z << ")" << std::endl;
  timer = ait::Timer();
  raw_octree->getMetricMax(x, y, z);
  timer.printTiming("Computing octree max");
  std::cout << "   max=(" << x << ", " << y << ", " << z << ")" << std::endl;

  size_t count_unknown = 0;
  size_t count_unknown_leaf = 0;
  for (auto it = raw_octree->begin_tree(); it != raw_octree->end_tree(); ++it) {
    if (it->getObservationCount() == 0) {
      ++count_unknown;
      if (it.isLeaf()) {
        ++count_unknown_leaf;
      }
    }
  }
  std::cout << "Unknown voxels: " << count_unknown << std::endl;
  std::cout << "Unknown leaf voxels: " << count_unknown_leaf << std::endl;
  return std::move(raw_octree);
}

void ViewpointPlannerData::readDensePoints(const std::string& dense_points_filename) {
  // Read reconstructed dense point cloud
  dense_points_.reset(new ViewpointPlannerData::PointCloudType);
  ViewpointPlannerData::PointCloudIOType::loadFromFile(dense_points_filename, *dense_points_.get());
  std::cout << "Number of vertices in dense points: " << dense_points_->m_points.size() << std::endl;
}

void ViewpointPlannerData::readPoissonMesh(const std::string& mesh_filename) {
  // Read poisson reconstructed mesh
  poisson_mesh_.reset(new ViewpointPlannerData::MeshType);
  ViewpointPlannerData::MeshIOType::loadFromFile(mesh_filename, *poisson_mesh_.get());
  std::cout << "Number of triangles in mesh: " << poisson_mesh_->m_FaceIndicesVertices.size() << std::endl;
  std::cout << "Number of vertices in mesh: " << poisson_mesh_->m_Vertices.size() << std::endl;
  std::cout << "Number of normals in mesh: " << poisson_mesh_->m_Normals.size() << std::endl;
}

bool ViewpointPlannerData::readBVHTree(std::string bvh_filename, const std::string& octree_filename) {
  // Read cached BVH tree (if up-to-date) or generate it
  bool read_cached_tree = false;
  if (!options_.regenerate_bvh_tree && boost::filesystem::exists(bvh_filename)) {
    if (boost::filesystem::last_write_time(bvh_filename) > boost::filesystem::last_write_time(octree_filename)) {
      std::cout << "Loading up-to-date cached BVH tree." << std::endl;
      readCachedBVHTree(bvh_filename);
      read_cached_tree = true;
    }
    else {
      std::cout << "Found cached BVH tree to be old. Ignoring it." << std::endl;
    }
  }
  if (!read_cached_tree) {
    std::cout << "Generating BVH tree." << std::endl;
    generateBVHTree(octree_.get());
    writeBVHTree(bvh_filename);
  }
  std::cout << "BVH tree bounding box: " << occupied_bvh_.getRoot()->getBoundingBox() << std::endl;
  return !read_cached_tree;
}

bool ViewpointPlannerData::readMeshDistanceField(std::string df_filename, const std::string& mesh_filename) {
  // Read cached distance field (if up-to-date) or generate it.
  bool read_cached_df = false;
  if (!options_.regenerate_distance_field && boost::filesystem::exists(df_filename)) {
    if (boost::filesystem::last_write_time(df_filename) > boost::filesystem::last_write_time(mesh_filename)) {
      std::cout << "Loading up-to-date cached distance field." << std::endl;
      ml::BinaryDataStreamFile file_stream(df_filename, false);
      file_stream >> distance_field_;
      read_cached_df = true;
    }
    else {
      std::cout << "Found cached BVH tree to be old. Ignoring it." << std::endl;
    }
  }
  if (!read_cached_df) {
    std::cout << "Generating weight grid and distance field." << std::endl;
    generateDistanceField();
    std::cout << "Done" << std::endl;
    ml::BinaryDataStreamFile file_stream(df_filename, true);
    file_stream << distance_field_;
  }
  std::cout << "BVH tree bounding box: " << occupied_bvh_.getRoot()->getBoundingBox() << std::endl;
  return !read_cached_df;
}

void ViewpointPlannerData::updateWeights() {
  for (auto it = octree_->begin_tree(); it != octree_->end_tree(); ++it) {
    it->setWeight(0);
  }
//  FloatType min_distance = std::numeric_limits<FloatType>::max();
  FloatType max_distance = std::numeric_limits<FloatType>::lowest();
  if (options_.use_distance_field) {
    for (int x = 0; x < grid_dim_(0); ++x) {
      for (int y = 0; y < grid_dim_(1); ++y) {
        for (int z = 0; z < grid_dim_(2); ++z) {
//          min_distance = std::min(min_distance, distance_field_(x, y, z));
          max_distance = std::max(max_distance, distance_field_(x, y, z));
        }
      }
    }
  }
//  std::cout << "min_distance: " << min_distance << std::endl;
//  std::cout << "max_distance: " << max_distance << std::endl;
//  FloatType max_weight = std::numeric_limits<FloatType>::lowest();
//  FloatType min_weight = std::numeric_limits<FloatType>::max();
//  const BoundingBoxType roi_bbox = roi_.getBoundingBox();
  for (int ix = 0; ix < grid_dim_(0); ++ix) {
    for (int iy = 0; iy < grid_dim_(1); ++iy) {
      for (int iz = 0; iz < grid_dim_(2); ++iz) {
        const BoundingBoxType::Vector3 xyz = getGridPosition(ix, iy, iz);
//        std::cout << "roi_distance=" << roi_distance << ", roi_weight=" << roi_weight << std::endl;
        FloatType roi_weight = 1;
//        if (roi_.isPointOutside(xyz) != roi_bbox.isOutside(xyz)) {
//          AIT_PRINT_VAR(roi_.isPointOutside(xyz));
//          AIT_PRINT_VAR(roi_bbox.isOutside(xyz));
//          AIT_PRINT_VAR(xyz);
//        }
        if (roi_.isPointOutside(xyz)) {
//        if (roi_bbox.isOutside(xyz)) {
          FloatType roi_distance = roi_.distanceToPoint(xyz);
//          FloatType roi_distance = roi_bbox.distanceTo(xyz);
          roi_distance = std::min(roi_distance, options_.roi_falloff_distance);
          roi_weight = (options_.roi_falloff_distance - roi_distance) / options_.roi_falloff_distance;
        }
        FloatType weight;
        if (options_.use_distance_field) {
          const FloatType distance = distance_field_(ix, iy, iz);
          const FloatType inv_distance = (max_distance - distance) / (FloatType)max_distance;
          if (options_.weight_falloff_quadratic) {
            weight = roi_weight * inv_distance * inv_distance;
          }
          else {
            weight = roi_weight * inv_distance;
          }
        }
        else {
          weight = roi_weight;
        }
        const BoundingBoxType bbox(xyz, grid_increment_);
        const std::vector<OccupiedTreeType::BBoxIntersectionResult> results =
            occupied_bvh_.intersects(bbox);
        for (const OccupiedTreeType::BBoxIntersectionResult& result : results) {
          result.node->getObject()->weight = weight;
        }
        const octomap::point3d oct_min(bbox.getMinimum(0), bbox.getMinimum(1), bbox.getMinimum(2));
        const octomap::point3d oct_max(bbox.getMaximum(0), bbox.getMaximum(1), bbox.getMaximum(2));
        for (auto it = octree_->begin_leafs_bbx(oct_min, oct_max); it != octree_->end_leafs_bbx(); ++it) {
          it->setWeight(weight);
        }
      }
    }
  }
//  std::cout << "min_weight: " << min_weight<< std::endl;
//  std::cout << "max_weight: " << max_weight<< std::endl;
  octree_->updateInnerOccupancy();
}

bool ViewpointPlannerData::readAndAugmentOctree(
    std::string octree_filename, const std::string& raw_octree_filename, bool binary) {
  // Read cached augmented tree (if up-to-date) or generate it
  bool read_cached_tree = false;
  if (!options_.regenerate_augmented_octree && boost::filesystem::exists(octree_filename)) {
    if (boost::filesystem::last_write_time(octree_filename) > boost::filesystem::last_write_time(raw_octree_filename)) {
      std::cout << "Loading up-to-date cached augmented tree [" << octree_filename << "]" << std::endl;
      octree_ = OccupancyMapType::read(octree_filename);
      read_cached_tree = true;
    }
    else {
      std::cout << "Found cached augmented tree to be old. Ignoring it." << std::endl;
    }
  }
  if (!read_cached_tree) {
    std::cout << "Reading non-augmented input tree [" << raw_octree_filename << "]" << std::endl;
    std::unique_ptr<RawOccupancyMapType> raw_octree = readRawOctree(raw_octree_filename, binary);
    std::cout << "Generating augmented tree." << std::endl;
    octree_ = generateAugmentedOctree(std::move(raw_octree));
    octree_->write(octree_filename);
  }
  return !read_cached_tree;
}

std::unique_ptr<ViewpointPlannerData::OccupancyMapType>
ViewpointPlannerData::generateAugmentedOctree(std::unique_ptr<RawOccupancyMapType> raw_octree) const {
  ait::Timer timer;
  if (!isTreeConsistent(*raw_octree.get())) {
    throw AIT_EXCEPTION("Input tree is inconsistent");
  }

  std::unique_ptr<OccupancyMapType> output_tree(convertToAugmentedMap(raw_octree.get()));

  if (!isTreeConsistent(*output_tree.get())) {
    throw AIT_EXCEPTION("Augmented tree is inconsistent");
  }
  timer.printTimingMs("Converting raw input tree");

  // TODO: Augmented tree with weights necessary?
//  timer = ait::Timer();
//  // Augment tree with weights
//  std::vector<TreeNavigatorType> query_nodes;
//  AIT_ASSERT(OCCUPANCY_WEIGHT_DEPTH - OCCUPANCY_WEIGHT_REACH > 0);
//  AIT_ASSERT(OCCUPANCY_WEIGHT_DEPTH_CUTOFF > OCCUPANCY_WEIGHT_DEPTH);
//  for (auto it = output_tree->begin_tree(OCCUPANCY_WEIGHT_DEPTH); it != output_tree->end_tree(); ++it) {
//    if (it.getDepth() == OCCUPANCY_WEIGHT_DEPTH) {
//      query_nodes.push_back(TreeNavigatorType(output_tree.get(), it.getKey(), &(*it), it.getDepth()));
//    }
//  }
//
//  float max_total_weight = 0;
//  for (const TreeNavigatorType& query_nav : query_nodes) {
//    const float dist_cutoff = 0.5f * query_nav.getSize();
//    const float dist_cutoff_sq = dist_cutoff * dist_cutoff;
//    const Eigen::Vector3f query_pos = query_nav.getPosition();
//
//    ConstTreeNavigatorType parent_nav = query_nav;
//    for (size_t i = 0; i < OCCUPANCY_WEIGHT_REACH; ++i) {
//      parent_nav.gotoParent();
//    }
//    std::stack<ConstTreeNavigatorType> node_stack;
//    node_stack.push(parent_nav);
//
//    WeightType total_weight = 0;
//    size_t total_count = 0;
//    while (!node_stack.empty()) {
//      ConstTreeNavigatorType nav = node_stack.top();
//      node_stack.pop();
//      if (nav.hasChildren() && nav.getDepth() < OCCUPANCY_WEIGHT_DEPTH_CUTOFF) {
//        for (size_t i = 0; i < 8; ++i) {
//          if (nav.hasChild(i)) {
//            node_stack.push(nav.child(i));
//          }
//        }
//      }
//      else {
//        if (nav->getObservationCount() > 0 && output_tree->isNodeOccupied(nav.getNode())) {
////        if (nav->getObservationCount() > 0) {
//          // TODO: How to initialize weights of augmented octree
////          WeightType weight = computeWeightContribution(query_pos, dist_cutoff_sq, nav);
//          WeightType weight = 0;
////          size_t count;
////          WeightType weight;
////          if (nav->hasChildren()) {
////            size_t count = 1 << (output_tree->getTreeDepth() - nav.getDepth());
////            weight = nav->getMeanChildOccupancy() * count;
////          } else {
////            count = 1;
////            weight = nav->getOccupancy();
////          }
////          total_count += count;
//          total_weight += weight;
//        }
//      }
//    }
//
////    total_weight = total_count > 0 ? total_weight / total_count : 0;
////    std::cout << "total_weight=" << total_weight << ", total_count=" << total_count << std::endl;
//
////    std::cout << "total_weight: " << total_weight << std::endl;
//    if (total_weight > max_total_weight) {
//      max_total_weight = total_weight;
//    }
//
//    // Pass weight down the tree to the leaf nodes
//    std::stack<TreeNavigatorType> node_stack2;
//    node_stack2.push(query_nav);
//    while (!node_stack2.empty()) {
//      TreeNavigatorType nav = node_stack2.top();
//      node_stack2.pop();
//      AIT_ASSERT(nav->getWeight() == 0);
//      nav->setWeight(total_weight);
//      if (nav.hasChildren()) {
//        for (size_t i = 0; i < 8; ++i) {
//          if (nav.hasChild(i)) {
//            node_stack2.push(nav.child(i));
//          }
//        }
//      }
//    }
//  }
//  std::cout << "Maximum weight: " << max_total_weight << std::endl;
//  timer.printTimingMs("Augmenting tree");

  return std::move(output_tree);
}

template <typename TreeT>
bool ViewpointPlannerData::isTreeConsistent(const TreeT& tree) {
  bool consistent = true;
  for (auto it = tree.begin_tree(); it != tree.end_tree(); ++it) {
    if (it->hasChildren()) {
      for (size_t i = 0; i < 8; ++i) {
        if (it->hasChild(i)) {
          if (it->getObservationCount() > it->getChild(i)->getObservationCount()) {
            consistent = false;
            break;
          }
          if (it->getOccupancy() < it->getChild(i)->getOccupancy()) {
            consistent = false;
            break;
          }
        }
      }
      if (!consistent) {
        break;
      }
    }
  }
  return consistent;
}

void ViewpointPlannerData::generateBVHTree(const OccupancyMapType* octree) {
  // Initialize nearest neighbor index for mesh faces
  using MeshAnn = ait::ApproximateNearestNeighbor<FloatType, 3>;
  MeshAnn mesh_ann;
  std::vector<Vector3> triangle_centers;
  triangle_centers.resize(poisson_mesh_->m_FaceIndicesVertices.size());
#pragma omp parallel for
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    AIT_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
    const ml::vec3f& v1 = poisson_mesh_->m_Vertices[face[0]];
    const ml::vec3f& v2 = poisson_mesh_->m_Vertices[face[1]];
    const ml::vec3f& v3 = poisson_mesh_->m_Vertices[face[2]];
    Vector3 triangle_center(v1.x + v2.x + v3.x, v1.y + v2.y + v3.y, v1.z + v2.z + v3.z);
    triangle_center /= 3;
    triangle_centers[i] = triangle_center;
  }
  mesh_ann.initIndex(triangle_centers.begin(), triangle_centers.end());

  const std::size_t mesh_knn = options_.bvh_normal_mesh_knn;
  const FloatType max_dist_square = options_.bvh_normal_mesh_max_dist * options_.bvh_normal_mesh_max_dist;

  std::vector<std::vector<typename OccupiedTreeType::ObjectWithBoundingBox>> objects_vector;
#pragma omp parallel
  {
    const std::size_t thread_index = (std::size_t)omp_get_thread_num();
    const std::size_t num_threads = (std::size_t)omp_get_num_threads();
#pragma omp single
    {
      objects_vector.resize(num_threads);
    }
    static std::vector<MeshAnn::IndexType> knn_indices;
    static std::vector<MeshAnn::DistanceType> knn_distances;
    std::vector<typename OccupiedTreeType::ObjectWithBoundingBox>& local_objects = objects_vector[thread_index];
    std::size_t i = 0;
    for (auto it = octree->begin_tree(); it != octree->end_tree(); ++it, ++i) {
      if (i % num_threads != thread_index) {
        continue;
      }
      if (it.isLeaf()) {
  //      if (octree->isNodeFree(&(*it)) || octree->isNodeUnknown(&(*it))) {
        if (octree->isNodeFree(&(*it)) && octree->isNodeKnown(&(*it))) {
          continue;
        }
        typename OccupiedTreeType::ObjectWithBoundingBox object_with_bbox;
        octomap::point3d center_octomap = it.getCoordinate();
        Eigen::Vector3f center;
        center << center_octomap.x(), center_octomap.y(), center_octomap.z();
        const float size = it.getSize();
        object_with_bbox.bounding_box = typename OccupiedTreeType::BoundingBoxType(center, size);
        object_with_bbox.object = new NodeObjectType();
        object_with_bbox.object->occupancy = it->getOccupancy();
        object_with_bbox.object->observation_count = it->getObservationCount();
        object_with_bbox.object->weight = it->getWeight();
        object_with_bbox.bounding_box.constrainTo(bvh_bbox_);
        if (object_with_bbox.bounding_box.isEmpty()) {
          continue;
        }
        object_with_bbox.object->normal.setZero();
        // Find nearest neighbor faces to compute normal of voxel/node
        knn_indices.resize(mesh_knn);
        knn_distances.resize(mesh_knn);
        mesh_ann.knnSearch(center, mesh_knn, &knn_indices, &knn_distances);
        for (std::size_t i = 0; i < knn_distances.size(); ++i) {
          const MeshAnn::DistanceType dist_square = knn_distances[i];
          const MeshAnn::IndexType index = knn_indices[i];
          if (dist_square <= max_dist_square) {
            const MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[index];
            const ml::vec3f& ml_v1 = poisson_mesh_->m_Vertices[face[0]];
            const ml::vec3f& ml_v2 = poisson_mesh_->m_Vertices[face[1]];
            const ml::vec3f& ml_v3 = poisson_mesh_->m_Vertices[face[2]];
            const Vector3 v1(ml_v1.x, ml_v1.y, ml_v1.z);
            const Vector3 v2(ml_v2.x, ml_v2.y, ml_v2.z);
            const Vector3 v3(ml_v3.x, ml_v3.y, ml_v3.z);
  //          Vector3 triangle_center(v1.x + v2.x + v3.x, v1.y + v2.y + v3.y, v1.z + v2.z + v3.z);
  //          triangle_center /= 3;
            const Vector3 normal = (v1 - v2).cross(v2 - v3).normalized();
            const FloatType normal_weight = 1 / dist_square;
            object_with_bbox.object->normal += normal_weight * normal;
          }
        }
        if (object_with_bbox.object->normal != Vector3::Zero()) {
          object_with_bbox.object->normal.normalize();
        }

//        objects.push_back(object_with_bbox);
        local_objects.push_back(object_with_bbox);
      }
    }
  }
  std::cout << "Merging local object vectors" << std::endl;
  std::vector<typename OccupiedTreeType::ObjectWithBoundingBox> objects;
  for (auto& local_objects : objects_vector) {
    objects.insert(std::end(objects), std::begin(local_objects), std::end(local_objects));
    local_objects.clear();
  }
  std::cout << "Building BVH tree with " << objects.size() << " objects" << std::endl;
  ait::Timer timer;
  occupied_bvh_.build(std::move(objects));
  timer.printTimingMs("Building BVH tree");
}

void ViewpointPlannerData::writeBVHTree(const std::string& filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs) {
    throw AIT_EXCEPTION(std::string("Unable to open file for writing: ") + filename);
  }
  boost::archive::binary_oarchive oa(ofs);
  oa << occupied_bvh_;
}

void ViewpointPlannerData::readCachedBVHTree(const std::string& filename) {
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    throw AIT_EXCEPTION(std::string("Unable to open file for reading: ") + filename);
  }
  boost::archive::binary_iarchive ia(ifs);
  ia >> occupied_bvh_;
}

void ViewpointPlannerData::generateWeightGrid() {
  const BoundingBoxType& bbox = occupied_bvh_.getRoot()->getBoundingBox();
  grid_dim_ = Vector3i(options_.grid_dimension, options_.grid_dimension, options_.grid_dimension);
  grid_origin_ = bbox.getMinimum();
  grid_increment_ = bbox.getMaxExtent() / (options_.grid_dimension + 1);
  Vector3 grid_max = grid_origin_ + Vector3(grid_increment_, grid_increment_, grid_increment_) * (options_.grid_dimension + 1);
  grid_bbox_ = BoundingBoxType(grid_origin_, grid_max);
  //  std::cout << "grid_dim_: " << grid_dim_ << std::endl;
  //  std::cout << "grid_origin_: " << grid_origin_ << std::endl;
  //  std::cout << "grid_increment_: " << grid_increment_ << std::endl;
  //  std::cout << "grid_max: " << grid_max << std::endl;
  //  std::cout << "grid_bbox_: " << grid_bbox_ << std::endl;
}

void ViewpointPlannerData::generateDistanceField() {
  ml::Grid3f seed_grid(options_.grid_dimension, options_.grid_dimension, options_.grid_dimension);
  seed_grid.setValues(std::numeric_limits<float>::max());
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    AIT_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
    const ml::vec3f& vertex1 = poisson_mesh_->m_Vertices[face[0]];
    const ml::vec3f& vertex2 = poisson_mesh_->m_Vertices[face[1]];
    const ml::vec3f& vertex3 = poisson_mesh_->m_Vertices[face[2]];
    const ml::vec3f ml_tri_xyz = (vertex1 + vertex2 + vertex3) / 3;
    const Vector3 tri_xyz(ml_tri_xyz.x, ml_tri_xyz.y, ml_tri_xyz.z);
    if (isInsideGrid(tri_xyz)) {
      const Vector3i indices = getGridIndices(tri_xyz);
      const Vector3 xyz = getGridPosition(indices);
      const float cur_dist = seed_grid(indices(0), indices(1), indices(2));
      const float new_dist = (xyz - tri_xyz).norm() / grid_increment_;
      if (new_dist < cur_dist) {
        seed_grid(indices(0), indices(1), indices(2)) = new_dist;
      }
    }
  }
  distance_field_ = DistanceFieldType(seed_grid);
  for (int x = 0; x < grid_dim_(0); ++x) {
    for (int y = 0; y < grid_dim_(1); ++y) {
      for (int z = 0; z < grid_dim_(2); ++z) {
        if (distance_field_(x, y, z) > options_.distance_field_cutoff) {
          distance_field_(x, y, z) = options_.distance_field_cutoff;
        }
      }
    }
  }
}

bool ViewpointPlannerData::isInsideGrid(const Vector3& xyz) const {
  return grid_bbox_.isInside(xyz);
}

ViewpointPlannerData::Vector3i ViewpointPlannerData::getGridIndices(const Vector3& xyz) const {
  Vector3 indices_float = (xyz - grid_origin_) / grid_increment_;
  Vector3i indices(indices_float.array().round().cast<int>());
  for (int i = 0; i < indices.rows(); ++i) {
    if (indices(i) >= grid_dim_(i) && xyz(i) < grid_bbox_.getMaximum()(i)) {
      --indices(i);
    }
  }
  return indices;
}

ViewpointPlannerData::Vector3 ViewpointPlannerData::getGridPosition(const Vector3i& indices) const {
  return grid_origin_ + grid_increment_ * indices.cast<float>();
}

ViewpointPlannerData::Vector3 ViewpointPlannerData::getGridPosition(int ix, int iy, int iz) const {
  return getGridPosition(Vector3i(ix, iy, iz));
}

// TODO: Can be removed?
//ViewpointPlannerData::WeightType ViewpointPlannerData::computeWeightContribution(
//    const Eigen::Vector3f& query_pos, float dist_cutoff_sq, const ConstTreeNavigatorType& nav) {
////  const WeightType observation_factor = computeObservationWeightFactor(nav->getObservationCountSum());
////  WeightType weight = nav->getOccupancy() * observation_factor;
//  WeightType weight = nav->getOccupancy() * nav->getObservationCountSum();
//  const Eigen::Vector3f node_pos = nav.getPosition();
//  float dist_sq = (node_pos - query_pos).squaredNorm();
//  dist_sq = std::max(dist_cutoff_sq, dist_sq);
//  weight /= std::sqrt(dist_cutoff_sq / dist_sq);
//  return weight;
//}
