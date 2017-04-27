/*
 * viewpoint_planner_data.cpp
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/assign/list_of.hpp>
#include <bh/common.h>
#include <bh/eigen.h>
#include <bh/gps.h>
#include <bh/math/geometry.h>
#include <bh/nn/approximate_nearest_neighbor.h>
#include <bh/vision/cameras.h>
#include "viewpoint_planner_data.h"
#include "viewpoint.h"
#include "viewpoint_raycast.h"
#include "viewpoint_score.h"
#include "viewpoint_offscreen_renderer.h"

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
  if (!reconstruction_path.empty()) {
    readDenseReconstruction(reconstruction_path);
  }

  // Get ROI (for sampling and weight computation)
  if (!options_.regions_json_filename.empty()) {
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
  else if (options_.isSet("roi_z_low")) {
    const FloatType z_low = options_.getValue<FloatType>("roi_z_low");
    const FloatType z_high = options_.getValue<FloatType>("roi_z_high");
    const Vector2 vertex1 = options_.getValue<Vector2>("roi_vertex1");
    const Vector2 vertex2 = options_.getValue<Vector2>("roi_vertex2");
    const Vector2 vertex3 = options_.getValue<Vector2>("roi_vertex3");
    const Vector2 vertex4 = options_.getValue<Vector2>("roi_vertex4");
    const std::vector<Vector2> vertices = boost::assign::list_of(vertex1)(vertex2)(vertex3)(vertex4);
    std::cout << "Constructing ROI from 4 vertices:" << std::endl;
    for (const Vector2& vertex : vertices) {
      std::cout << "  " << vertex.transpose() << std::endl;
    }
    roi_ = RegionType(vertices, z_low, z_high);
    std::cout << "ROI bounding box: " << roi_.getBoundingBox() << std::endl;
  }
  else {
//    roi_bbox_ = BoundingBoxType(
//        Vector3(options->getValue<FloatType>("roi_bbox_min_x"),
//            options->getValue<FloatType>("roi_bbox_min_y"),
//            options->getValue<FloatType>("roi_bbox_min_z")),
//        Vector3(options->getValue<FloatType>("roi_bbox_max_x"),
//            options->getValue<FloatType>("roi_bbox_max_y"),
//            options->getValue<FloatType>("roi_bbox_max_z")));
    const FloatType roi_min_x = options_.getValue<FloatType>("roi_bbox_min_x");
    const FloatType roi_min_y = options_.getValue<FloatType>("roi_bbox_min_y");
    const FloatType roi_min_z = options_.getValue<FloatType>("roi_bbox_min_z");
    const FloatType roi_max_x = options_.getValue<FloatType>("roi_bbox_max_x");
    const FloatType roi_max_y = options_.getValue<FloatType>("roi_bbox_max_y");
    const FloatType roi_max_z = options_.getValue<FloatType>("roi_bbox_max_z");
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
    df_filename = mesh_filename + ".df.bs";
  }

  if (!dense_points_filename.empty()) {
    readDensePoints(dense_points_filename);
  }
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
    if (reconstruction_) {
      updateWeightsWithRealViewpoints();
    }
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
  using GpsConverter = bh::GpsConverter<GpsFloatType>;
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
//    BH_PRINT_VALUE(gps2);
//    BH_PRINT_VALUE(gps_converter.convertGpsToEcef(gps2));
//    BH_PRINT_VALUE(gps_converter.convertGpsToEnu(gps2));
//    gps2 = GpsCoordinateType(gps2.latitude(), gps2.longitude(), gps2.altitude() + 360);
//    BH_PRINT_VALUE(gps2);
//    BH_PRINT_VALUE(gps_converter.convertGpsToEcef(gps2));
//    BH_PRINT_VALUE(gps_converter.convertGpsToEnu(gps2));
//    Vector3 enu2 = enu;
//    BH_PRINT_VALUE(enu2);
//    BH_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
//    enu2(2) = 360;
//    BH_PRINT_VALUE(enu2);
//    BH_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
//    enu2(2) = 0;
//    BH_PRINT_VALUE(enu2);
//    BH_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
  }
  if (verbose) {
    std::cout << "Lower altitude: " << lower_altitude << ", upper_altitude: " << upper_altitude << std::endl;
  }
  const RegionType region(vertices, lower_altitude, upper_altitude);
  return region;
}

bool ViewpointPlannerData::isValidObjectPosition(const Vector3& position, const BoundingBoxType& object_bbox) const {
  for (const RegionType& no_fly_zone : no_fly_zones_) {
    if (no_fly_zone.isPointInside(position)) {
      return false;
    }
  }
  if (position(2) >= options_.obstacle_free_height) {
    return bvh_bbox_.isInside(position);
  }
  else {
    BoundingBoxType centered_object_bbox = object_bbox + position;
    if (centered_object_bbox.getMaximum(2) >= options_.obstacle_free_height) {
      Vector3 cropped_maximum = object_bbox.getMaximum();
      cropped_maximum(2) = options_.obstacle_free_height;
      centered_object_bbox = BoundingBoxType(centered_object_bbox.getMinimum(), cropped_maximum);
    }
    std::vector<ViewpointPlannerData::OccupiedTreeType::ConstBBoxIntersectionResult> results =
            occupied_bvh_.intersects(centered_object_bbox);
    return results.empty();
  }
}

const reconstruction::DenseReconstruction& ViewpointPlannerData::getReconstruction() const {
  return *reconstruction_;
}

const ViewpointPlannerData::DistanceFieldType& ViewpointPlannerData::getDistanceField() const {
  return distance_field_;
}

void ViewpointPlannerData::readDenseReconstruction(const std::string& path) {
  std::cout << "Reading dense reconstruction workspace" << std::endl;
  bh::Timer timer;
  const bool dense_reconstruction_has_gps = options_.getValue<bool>("dense_reconstruction_has_gps");
  reconstruction_.reset(new reconstruction::DenseReconstruction());
  reconstruction_->read(path, dense_reconstruction_has_gps);
  timer.printTiming("Loading dense reconstruction");
}

std::unique_ptr<ViewpointPlannerData::RawOccupancyMapType>
ViewpointPlannerData::readRawOctree(const std::string& octree_filename, bool binary) const {
  bh::Timer timer;
  std::unique_ptr<RawOccupancyMapType> raw_octree;
  if (binary) {
//      octree.reset(new ViewpointPlanner::OccupancyMapType(filename));
    throw BH_EXCEPTION("Binary occupancy maps not supported");
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
  timer = bh::Timer();
  raw_octree->getMetricSize(x, y, z);
  timer.printTiming("Computing octree size");
  std::cout << "  size=(" << x << ", " << y << ", " << z << ")" << std::endl;
  timer = bh::Timer();
  raw_octree->getMetricMin(x, y, z);
  timer.printTiming("Computing octree min");
  std::cout << "   min=(" << x << ", " << y << ", " << z << ")" << std::endl;
  timer = bh::Timer();
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
  // Check if mesh was read correctly
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
    BH_ASSERT_STR(face[0] < poisson_mesh_->m_Vertices.size(), "Poisson mesh was not read correctly");
    BH_ASSERT_STR(face[1] < poisson_mesh_->m_Vertices.size(), "Poisson mesh was not read correctly");
    BH_ASSERT_STR(face[2] < poisson_mesh_->m_Vertices.size(), "Poisson mesh was not read correctly");
  }
  std::cout << "Number of triangles in mesh: " << poisson_mesh_->m_FaceIndicesVertices.size() << std::endl;
  std::cout << "Number of vertices in mesh: " << poisson_mesh_->m_Vertices.size() << std::endl;
  std::cout << "Number of normals in mesh: " << poisson_mesh_->m_Normals.size() << std::endl;
}

bool ViewpointPlannerData::readBVHTree(std::string bvh_filename, const std::string& octree_filename) {
#if WITH_CUDA
  if (options_.enable_cuda) {
    std::cout << "Selecting CUDA device " << options_.cuda_gpu_id << std::endl;
    bh::CudaDevice cuda_dev(options_.cuda_gpu_id);
    cuda_dev.activate();
    std::cout << "Previous CUDA stack size was " << cuda_dev.getStackSize() << std::endl;
    std::cout << "Setting CUDA stack size to " << options_.cuda_stack_size << std::endl;
    cuda_dev.setStackSize(options_.cuda_stack_size);
  }
#endif

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
      // TODO: Remove
//      ml::BinaryDataStreamFile file_stream(df_filename, false);
//      file_stream >> distance_field_;
      _readMeshDistanceField(df_filename, &distance_field_);
      read_cached_df = true;
      std::cout << "Done" << std::endl;
    }
    else {
      std::cout << "Found cached BVH tree to be old. Ignoring it." << std::endl;
    }
  }
  if (!read_cached_df) {
    std::cout << "Generating weight grid and distance field." << std::endl;
    generateDistanceField();
    std::cout << "Done" << std::endl;
    // TODO: Remove
//    ml::BinaryDataStreamFile file_stream(df_filename, true);
//    file_stream << distance_field_;
    _writeMeshDistanceField(df_filename, distance_field_);
  }
  std::cout << "BVH tree bounding box: " << occupied_bvh_.getRoot()->getBoundingBox() << std::endl;
  return !read_cached_df;
}

void ViewpointPlannerData::_readMeshDistanceField(const std::string& filename, DistanceFieldType* distance_field) {
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    throw BH_EXCEPTION(std::string("Unable to open file for reading: ") + filename);
  }
  boost::archive::binary_iarchive ia(ifs);
  std::size_t dim_x;
  std::size_t dim_y;
  std::size_t dim_z;
  ia >> dim_x;
  ia >> dim_y;
  ia >> dim_z;
  *distance_field = DistanceFieldType(dim_x, dim_y, dim_z);
  ia >> boost::serialization::make_array(distance_field->getData(), distance_field->getNumElements());
}

void ViewpointPlannerData::_writeMeshDistanceField(const std::string& filename, const DistanceFieldType& distance_field) {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs) {
    throw BH_EXCEPTION(std::string("Unable to open file for writing: ") + filename);
  }
  boost::archive::binary_oarchive oa(ofs);
  oa << distance_field.getDimX();
  oa << distance_field.getDimY();
  oa << distance_field.getDimZ();
  oa << boost::serialization::make_array(distance_field.getData(), distance_field.getNumElements());
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
//          BH_PRINT_VAR(roi_.isPointOutside(xyz));
//          BH_PRINT_VAR(roi_bbox.isOutside(xyz));
//          BH_PRINT_VAR(xyz);
//        }
        if (roi_.isPointOutside(xyz)) {
//        if (roi_bbox.isOutside(xyz)) {
          FloatType roi_distance = roi_.distanceToPoint(xyz);
//          FloatType roi_distance = roi_bbox.distanceTo(xyz);
          roi_distance = std::min(roi_distance, options_.roi_falloff_distance);
          roi_weight = (options_.roi_falloff_distance - roi_distance) / options_.roi_falloff_distance;
        }
        WeightType weight;
        if (options_.use_distance_field) {
          const FloatType distance = distance_field_(ix, iy, iz);
          if (distance <= options_.weight_falloff_distance_start) {
            weight = roi_weight;
          }
          else {
            const FloatType inv_distance = (max_distance - distance) / (max_distance - options_.weight_falloff_distance_start);
            if (options_.weight_falloff_quadratic) {
              weight = roi_weight * inv_distance * inv_distance;
            }
            else {
              weight = roi_weight * inv_distance;
            }
          }
        }
        else {
          weight = roi_weight;
        }
        const BoundingBoxType bbox(xyz, grid_increment_);
        const std::vector<OccupiedTreeType::BBoxIntersectionResult> results =
            occupied_bvh_.intersects(bbox);
        for (const OccupiedTreeType::BBoxIntersectionResult& result : results) {
          WeightType observation_count_factor = computeObservationCountFactor(result.node->getObject()->observation_count);
          BH_ASSERT(observation_count_factor <= 1);
          result.node->getObject()->weight = weight * observation_count_factor;
        }
        const octomap::point3d oct_min(bbox.getMinimum(0), bbox.getMinimum(1), bbox.getMinimum(2));
        const octomap::point3d oct_max(bbox.getMaximum(0), bbox.getMaximum(1), bbox.getMaximum(2));
        for (auto it = octree_->begin_leafs_bbx(oct_min, oct_max); it != octree_->end_leafs_bbx(); ++it) {
          WeightType observation_count_factor = computeObservationCountFactor(it->getObservationCount());
          BH_ASSERT(observation_count_factor <= 1);
          it->setWeight(weight * observation_count_factor);
        }
      }
    }
  }
//  std::cout << "min_weight: " << min_weight<< std::endl;
//  std::cout << "max_weight: " << max_weight<< std::endl;
  octree_->updateInnerOccupancy();
}

void ViewpointPlannerData::updateWeightsWithRealViewpoints() {
  if (!options_.ignore_real_observed_voxels && options_.invalid_pixel_observation_factor > 0) {
    std::cout << "Computing observed voxels for " << reconstruction_->getImages().size()
              << " previous camera viewpoints" << std::endl;
    std::unique_ptr<viewpoint_planner::ViewpointOffscreenRenderer> offscreen_renderer;
    for (const auto& entry : reconstruction_->getImages()) {
      const reconstruction::PinholeCamera &real_camera = reconstruction_->getCameras().at(entry.second.camera_id());
      const reconstruction::DenseReconstruction::DepthMap& depth_map = reconstruction_->readDepthMap(
              entry.second.id(), reconstruction::DenseReconstruction::DenseMapType::GEOMETRIC_FUSED);
      const FloatType depth_camera_scale_factor = depth_map.width() / FloatType(real_camera.width());
      const reconstruction::PinholeCamera depth_camera = real_camera.getScaledCamera(depth_camera_scale_factor);
      if (!offscreen_renderer) {
        viewpoint_planner::ViewpointOffscreenRenderer::Options offscreen_renderer_options;
//        offscreen_renderer_options.dump_poisson_mesh_depth_image = true;
//        offscreen_renderer_options.dump_poisson_mesh_normals_image = true;
        offscreen_renderer.reset(new viewpoint_planner::ViewpointOffscreenRenderer(
                offscreen_renderer_options, depth_camera, poisson_mesh_.get()));
      }
      else {
        offscreen_renderer->setCamera(depth_camera);
      }
      const reconstruction::Pose& pose = entry.second.pose();
      const Viewpoint viewpoint(&depth_camera, pose);
      // Compute observed voxels and information
      const bool remove_duplicates = false;
      const FloatType min_range = options_.real_observed_voxels_raycast_min_range;
      const FloatType max_range = options_.real_observed_voxels_raycast_max_range > 0 ?
                                  options_.real_observed_voxels_raycast_max_range :
                                  std::numeric_limits<FloatType>::max();
      viewpoint_planner::ViewpointRaycast raycaster(&occupied_bvh_, min_range, max_range);
      viewpoint_planner::ViewpointScore scorer(
              options_.getOptionsAs<viewpoint_planner::ViewpointScore::Options>(),
              [&](const Viewpoint& viewpoint,
                  const viewpoint_planner::VoxelType* node,
                  const viewpoint_planner::Vector2& image_coordinates) -> Vector3 {
                Vector3 normal_vector;
                if (options_.enable_opengl) {
                  const std::size_t x = static_cast<std::size_t>(image_coordinates(0));
                  const std::size_t y = static_cast<std::size_t>(image_coordinates(1));
                  normal_vector = offscreen_renderer->computePoissonMeshNormalVector(viewpoint, x, y);
                }
                else {
                  normal_vector = node->getObject()->normal;
                }
                return normal_vector;
              });
#if WITH_CUDA
      raycaster.setEnableCuda(options_.enable_cuda);
#endif
      // TODO: Add minimum range to raycast query. Otherwise unknown voxels are captured instead of the interesting ones.
      std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates> raycast_results =
              raycaster.getRaycastHitVoxelsWithScreenCoordinates(viewpoint, remove_duplicates);
      for (size_t i = 0; i < raycast_results.size(); ++i) {
        OccupiedTreeType::IntersectionResultWithScreenCoordinates& ir = raycast_results[i];
        const FloatType depth = depth_map(ir.screen_coordinates(1), ir.screen_coordinates(0));
        if (depth <= 0 || std::isinf(depth)) {
          const WeightType information = scorer.computeViewpointObservationScore(
                  viewpoint, ir.intersection_result.node, ir.screen_coordinates);
          viewpoint_planner::VoxelType* voxel = ir.intersection_result.node;
          const WeightType new_weight = std::max<WeightType>(
                  0, voxel->getObject()->weight - options_.invalid_pixel_observation_factor * information);
          voxel->getObject()->weight = new_weight;
          BH_ASSERT(voxel->getObject()->weight >= 0);
          const BoundingBoxType& bbox = voxel->getBoundingBox();
          const octomap::point3d oct_min(bbox.getMinimum(0), bbox.getMinimum(1), bbox.getMinimum(2));
          const octomap::point3d oct_max(bbox.getMaximum(0), bbox.getMaximum(1), bbox.getMaximum(2));
          for (auto it = octree_->begin_leafs_bbx(oct_min, oct_max); it != octree_->end_leafs_bbx(); ++it) {
            it->setWeight(new_weight);
          }
        }
      }
    }
  }
}

ViewpointPlannerData::WeightType ViewpointPlannerData::computeObservationCountFactor(CounterType observation_count) const {
  const FloatType information_factor = std::exp(- options_.voxel_information_lambda * observation_count);
  return information_factor;
//  return observation_count == 0 ? 1 : 0;
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
  std::cout << "Octree resolution: " << octree_->getResolution() << std::endl;
  return !read_cached_tree;
}

std::unique_ptr<ViewpointPlannerData::OccupancyMapType>
ViewpointPlannerData::generateAugmentedOctree(std::unique_ptr<RawOccupancyMapType> raw_octree) const {
  bh::Timer timer;
  if (!isTreeConsistent(*raw_octree.get())) {
    throw BH_EXCEPTION("Input tree is inconsistent");
  }

  std::unique_ptr<OccupancyMapType> output_tree(convertToAugmentedMap(raw_octree.get()));

  if (!isTreeConsistent(*output_tree.get())) {
    throw BH_EXCEPTION("Augmented tree is inconsistent");
  }
  std::stack<TreeNavigatorType> expansion_stack;
  for (auto it = output_tree->begin_tree(); it != output_tree->end_tree(); ++it) {
    BH_ASSERT(it->getWeight() >= 0);
  }
  for (auto it = output_tree->begin_leafs(); it != output_tree->end_leafs(); ++it) {
    const Vector3 position(it.getX(), it.getY(), it.getZ());
    const FloatType size = it.getSize();
    const BoundingBoxType bbox(position, size);
    if (!bbox.intersects(bvh_bbox_)) {
      continue;
    }
    if (bbox.getMinimum(2) >= options_.obstacle_free_height) {
      it->setOccupancy(0);
      it->setObservationCount(1);
    }
    else if (bbox.getMaximum(2) >= options_.obstacle_free_height
             && it.getDepth() < output_tree->getTreeDepth()) {
      OccupancyMapType::NodeType *node = &(*it);
      TreeNavigatorType navigator(output_tree.get(), it.getKey(), node, it.getDepth());
      expansion_stack.push(navigator);
    }
  }
  std::cout << "Expanding " << expansion_stack.size() << " nodes" << std::endl;
  while (!expansion_stack.empty()) {
    TreeNavigatorType navigator = expansion_stack.top();
    expansion_stack.pop();
    output_tree->expandNode(navigator.getNode());
    const FloatType size = navigator.getSize();
    for (size_t child_idx = 0; child_idx < 8; ++child_idx) {
      const BoundingBoxType bbox(navigator.getPosition(), size);
      if (!bbox.intersects(bvh_bbox_)) {
        continue;
      }
      if (bbox.getMinimum(2) >= options_.obstacle_free_height) {
        navigator->setOccupancy(0);
        navigator->setObservationCount(1);
      }
      else if (bbox.getMaximum(2) >= options_.obstacle_free_height
               && navigator.getDepth() < output_tree->getTreeDepth()) {
        expansion_stack.push(navigator.child(child_idx));
      }
    }
  }
  std::cout << "Done" << std::endl;
  timer.printTimingMs("Converting raw input tree");

  // TODO: Augmented tree with weights necessary?
//  timer = bh::Timer();
//  // Augment tree with weights
//  std::vector<TreeNavigatorType> query_nodes;
//  BH_ASSERT(OCCUPANCY_WEIGHT_DEPTH - OCCUPANCY_WEIGHT_REACH > 0);
//  BH_ASSERT(OCCUPANCY_WEIGHT_DEPTH_CUTOFF > OCCUPANCY_WEIGHT_DEPTH);
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
//      BH_ASSERT(nav->getWeight() == 0);
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
  using MeshAnn = bh::ApproximateNearestNeighbor<FloatType, 3>;
  MeshAnn mesh_ann;
  std::vector<Vector3> triangle_centers;
  triangle_centers.resize(poisson_mesh_->m_FaceIndicesVertices.size());
//#pragma omp parallel for
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const MeshType::Indices::Face& face = poisson_mesh_->m_FaceIndicesVertices[i];
    BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
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
//#pragma omp parallel
  {
    const std::size_t thread_index = (std::size_t)omp_get_thread_num();
    const std::size_t num_threads = (std::size_t)omp_get_num_threads();
//#pragma omp single
    {
      objects_vector.resize(num_threads);
    }
    std::vector<MeshAnn::IndexType> knn_indices;
    std::vector<MeshAnn::DistanceType> knn_distances;
    std::vector<typename OccupiedTreeType::ObjectWithBoundingBox>& local_objects = objects_vector[thread_index];
    const size_t report_leaf_interval = (size_t)octree->size() / 50.0;
    std::size_t i = 0;
    for (auto it = octree->begin_tree(); it != octree->end_tree(); ++it, ++i) {
      if (thread_index == 0 && i % report_leaf_interval == 0) {
        const FloatType percentage = FloatType(100) * i / FloatType(octree->size());
        std::cout << "  processed " << percentage << " % of octree nodes" << std::endl;
      }
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
        object_with_bbox.bounding_box.constrainTo(bvh_bbox_);
        if (object_with_bbox.bounding_box.getMaximum(2) >= options_.obstacle_free_height) {
          const Vector3 min = object_with_bbox.bounding_box.getMinimum();
          Vector3 max = object_with_bbox.bounding_box.getMaximum();
          max(2) = options_.obstacle_free_height;
          object_with_bbox.bounding_box = typename OccupiedTreeType::BoundingBoxType(min, max);
        }
        if (object_with_bbox.bounding_box.isEmpty()) {
          continue;
        }
        object_with_bbox.object = new NodeObjectType();
        object_with_bbox.object->occupancy = it->getOccupancy();
        object_with_bbox.object->observation_count = it->getObservationCount();
        object_with_bbox.object->weight = it->getWeight();
        object_with_bbox.object->normal.setZero();
        // Find nearest neighbor faces to compute normal of voxel/node
        if (!options_.enable_opengl) {
          // If normals are not computed with OpenGL we average nearest neighbors
          knn_indices.resize(mesh_knn);
          knn_distances.resize(mesh_knn);
          mesh_ann.knnSearch(center, mesh_knn, &knn_indices, &knn_distances);
          for (std::size_t i = 0; i < knn_distances.size(); ++i) {
            const MeshAnn::DistanceType dist_square = knn_distances[i];
            const MeshAnn::IndexType index = knn_indices[i];
            if (dist_square <= max_dist_square) {
              const MeshType::Indices::Face &face = poisson_mesh_->m_FaceIndicesVertices[index];
              const ml::vec3f &ml_v1 = poisson_mesh_->m_Vertices[face[0]];
              const ml::vec3f &ml_v2 = poisson_mesh_->m_Vertices[face[1]];
              const ml::vec3f &ml_v3 = poisson_mesh_->m_Vertices[face[2]];
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
  bh::Timer timer;
  occupied_bvh_.build(std::move(objects));
  timer.printTimingMs("Building BVH tree");
}

void ViewpointPlannerData::writeBVHTree(const std::string& filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs) {
    throw BH_EXCEPTION(std::string("Unable to open file for writing: ") + filename);
  }
  boost::archive::binary_oarchive oa(ofs);
  oa << occupied_bvh_;
}

void ViewpointPlannerData::readCachedBVHTree(const std::string& filename) {
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    throw BH_EXCEPTION(std::string("Unable to open file for reading: ") + filename);
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
  std::cout << "Grid dimensions: " << grid_dim_.transpose() << std::endl;
  std::cout << "Grid increment: " << grid_increment_ << std::endl;
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
    BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
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
  for (int ix = 0; ix < grid_dim_(0); ++ix) {
    for (int iy = 0; iy < grid_dim_(1); ++iy) {
      for (int iz = 0; iz < grid_dim_(2); ++iz) {
        if (distance_field_(ix, iy, iz) > options_.distance_field_cutoff) {
          distance_field_(ix, iy, iz) = options_.distance_field_cutoff;
        }
      }
    }
  }
  // So far only sparse voxels have been updated. Adjust distance field until all distances are approximately correct.
  FloatType distance_kernel[3][3][3];
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        FloatType distance = grid_increment_ * Vector3((FloatType)k, (FloatType)j, (FloatType)i).norm();
        distance_kernel[k + 1][j + 1][i + 1] = distance;
      }
    }
  }
  bool distance_changed = true;
  while (distance_changed) {
    distance_changed = false;
    for (std::size_t z = 0; z < distance_field_.getDimZ(); z++) {
      for (std::size_t y = 0; y < distance_field_.getDimY(); y++) {
        for (std::size_t x = 0; x < distance_field_.getDimX(); x++) {
          FloatType d_min = distance_field_(x, y, z);
          // Iterate over all neighbors
          for (int k = -1; k <= 1; k++) {
            for (int j = -1; j <= 1; j++) {
              for (int i = -1; i <= 1; i++) {
                const Eigen::Matrix<unsigned long, 3, 1> n(x + i, y + j, z + k);
                if (distance_field_.isValidCoordinate(n(0), n(1), n(2))) {
                  FloatType d_curr = distance_field_(n(0), n(1), n(2)) + distance_kernel[i + 1][j + 1][k + 1];
                  if (d_curr < d_min && d_curr <= options_.distance_field_cutoff) {
                    d_min = d_curr;
                    distance_changed = true;
                  }
                }
              }
            }
          }
          if (distance_changed) {
            distance_field_(x, y, z) = d_min;
          }
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
    if (indices(i) < 0 && xyz(i) >= grid_bbox_.getMinimum(i)) {
      indices(i) = 0;
    }
    if (indices(i) >= grid_dim_(i) && xyz(i) <= grid_bbox_.getMaximum(i)) {
      indices(i) = grid_dim_(i);
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
