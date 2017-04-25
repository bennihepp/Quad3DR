//==================================================
// compute_ground_truth_mesh.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 13, 2017
//==================================================

#include <iostream>
#include <memory>
#include <csignal>
#include <algorithm>

#include <bh/boost.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

#include <bh/common.h>
//#include <bh/eigen.h>
//#include <bh/eigen_serialization.h>
#include <bh/utilities.h>
#include <bh/math/utilities.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/math/geometry.h>
#include <bh/nn/approximate_nearest_neighbor.h>

#include <bh/mLib/mLib.h>
#include <bh/mLib/mLibUtils.h>

#include <bh/eigen.h>
#include <bh/eigen_serialization.h>
#include <bh/mesh/triangle_mesh.h>
#include <bh/aabb/aabb_tree.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>

#include "../octree/occupancy_map.h"
#include "../octree/occupancy_node.h"
#include "../planner/viewpoint_planner.h"

//#pragma GCC optimize("O0")

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = bh::BoundingBox3D<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using PointCloudIOType = ml::PointCloudIO<FloatType>;
using MeshDataType = ml::MeshData<FloatType>;
using MeshIOType = ml::MeshIO<FloatType>;
//using TriMeshType = ml::TriMesh<FloatType>;
//using RayType = ml::Ray<FloatType>;
using TriMeshAcceleratorType = ml::TriMeshAcceleratorBruteForce<FloatType>;
//using TriMeshAcceleratorType = ml::TriMeshAcceleratorBVH<FloatType>;
using Triangle = bh::Triangle<FloatType>;
using TriangleMeshType = bh::TriangleMesh<FloatType>;
using TriAABBTree = bh::AABBTree<Triangle, FloatType>;

class ComputeGroundTruthMeshCmdline {
public:
  struct Options : bh::ConfigOptions {
    static const string kPrefix;

    Options()
    : bh::ConfigOptions(kPrefix) {
      addOptionRequired<Vector3>("roi_bbox_min", &roi_bbox_min);
      addOptionRequired<Vector3>("roi_bbox_max", &roi_bbox_max);
      addOption<FloatType>("max_triangle_area", &max_triangle_area);
      addOption<size_t>("sphere_subdivisions", &sphere_subdivisions);
      addOption<FloatType>("min_visible_rays_ratio", &min_visible_rays_ratio);
      addOption<int>("cuda_gpu_id", 0);
      addOption<size_t>("cuda_stack_size", 32 * 1024);
      addOption<string>("refined_mesh");
      addOption<string>("octree_filename");
      addOptionalOption<string>("out_reachable_voxel_positions");
      addOptionalOption<string>("in_reachable_voxel_positions");
      addOptionalOption<string>("out_valid_position_bvh_tree");
      addOptionalOption<string>("in_valid_position_bvh_tree");
    }

    ~Options() override {}

    Vector3 roi_bbox_min;
    Vector3 roi_bbox_max;
    FloatType max_triangle_area = FloatType(0.5);
    size_t sphere_subdivisions = 3;
    FloatType min_visible_rays_ratio = FloatType(0.05);
  };

  struct NodeObject {
    bool is_valid_object_position;

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & is_valid_object_position;
    }
  };

  using OctreeType = OccupancyMap<OccupancyNode>;
//  using OctreeType = OccupancyMap<AugmentedOccupancyNode>;
  using BvhTreeType = bvh::Tree<NodeObject, FloatType>;

  static std::map<string, std::unique_ptr<bh::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
      std::forward_as_tuple("viewpoint_planner.data"),
      std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new ViewpointPlannerData::Options())));
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new Options())));
    config_options.emplace(std::piecewise_construct,
      std::forward_as_tuple("viewpoint_planner"),
      std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new ViewpointPlanner::Options())));
    config_options.emplace(std::piecewise_construct,
      std::forward_as_tuple("motion_planner"),
      std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new ViewpointPlanner::MotionPlannerType::Options())));
    return config_options;
  }

  ComputeGroundTruthMeshCmdline(
      const std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options,
      const string& in_mesh_filename,
      const string& out_mesh_filename,
      const string& out_unobs_mesh_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    planner_options_(*dynamic_cast<ViewpointPlanner::Options*>(config_options.at("viewpoint_planner").get())),
    planner_data_options_(*dynamic_cast<ViewpointPlannerData::Options*>(config_options.at("viewpoint_planner.data").get())),
    in_mesh_filename_(in_mesh_filename),
    out_mesh_filename_(out_mesh_filename),
    out_unobs_mesh_filename_(out_unobs_mesh_filename),
    roi_bbox_(options_.roi_bbox_min, options_.roi_bbox_max),
    pose_sample_bbox_(
        planner_options_.getValue<Vector3>("sampling_bbox_min"),
        planner_options_.getValue<Vector3>("sampling_bbox_max")),
    drone_extent_(
        planner_options_.getValue<FloatType>("drone_extent_x"),
        planner_options_.getValue<FloatType>("drone_extent_y"),
        planner_options_.getValue<FloatType>("drone_extent_z")) {
//    const ViewpointPlannerData::Options* viewpoint_planner_data_options =
//      dynamic_cast<ViewpointPlannerData::Options*>(config_options.at("viewpoint_planner.data").get());
//    std::unique_ptr<ViewpointPlannerData> planner_data(
//      new ViewpointPlannerData(viewpoint_planner_data_options));
//    planner_ptr_.reset(new ViewpointPlanner(
//      dynamic_cast<ViewpointPlanner::Options*>(config_options.at("viewpoint_planner").get()),
//      dynamic_cast<ViewpointPlanner::MotionPlannerType::Options*>(config_options.at("motion_planner").get()),
//      std::move(planner_data)));
  }

  ~ComputeGroundTruthMeshCmdline() {
  }

  bool isValidObjectPosition(const Vector3& position, const Vector3& object_extent, const ViewpointPlanner::OccupiedTreeType& bvh_tree) {
    BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(position, object_extent);
    std::vector<ViewpointPlanner::OccupiedTreeType::ConstBBoxIntersectionResult> results =
        bvh_tree.intersects(object_bbox);
    return results.empty();
//    const size_t occupied_count =
//        std::count_if(results.begin(), results.end(), [&] (const ViewpointPlanner::OccupiedTreeType::ConstBBoxIntersectionResult& result) {
//      return !result.node->getObject()->is_free;
//    });
//    return occupied_count == 0;
  }

  MeshDataType computeRefinedMesh(const MeshDataType& mesh, const FloatType max_triangle_area) const {
    std::vector<Triangle> triangles;
    for (std::size_t i = 0; i < mesh.m_FaceIndicesVertices.size(); ++i) {
      const MeshDataType::Indices::Face& face = mesh.m_FaceIndicesVertices[i];
      BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");

      Triangle tri(
          bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[0]]),
          bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[1]]),
          bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[2]]));
      if (!roi_bbox_.isInside(tri.v1())
          || !roi_bbox_.isInside(tri.v2())
          || !roi_bbox_.isInside(tri.v3())) {
        continue;
      }
      triangles.push_back(tri);
    }
    cout << "Clipped mesh has " << triangles.size() << " triangles" << endl;

    MeshDataType clipped_mesh;
    clipped_mesh.m_FaceIndicesVertices.resize(triangles.size(), 3);
    std::unordered_map<Vector3, std::size_t> vertex_index_map;
    const auto get_vertex_index_lambda = [&] (const Vector3& v) {
      const auto it = vertex_index_map.find(v);
      size_t idx;
      if (it == vertex_index_map.end()) {
        idx = clipped_mesh.m_Vertices.size();
        clipped_mesh.m_Vertices.push_back(bh::MLibUtilities::convertEigenToMlib(v));
        vertex_index_map.emplace(v, idx);
      }
      else {
        idx = vertex_index_map.at(v);
      }
      return idx;
    };
    for (size_t i = 0; i < triangles.size(); ++i) {
      const Triangle& tri = triangles[i];
      const size_t idx1 = get_vertex_index_lambda(tri.v1());
      const size_t idx2 = get_vertex_index_lambda(tri.v2());
      const size_t idx3 = get_vertex_index_lambda(tri.v3());
      MeshDataType::Indices::Face& face = clipped_mesh.m_FaceIndicesVertices[i];
      face[0] = idx1;
      face[1] = idx2;
      face[2] = idx3;
    }
    cout << "Clipped mesh has " << clipped_mesh.m_FaceIndicesVertices.size() << " triangles" << endl;

    cout << "Refining mesh until maximum triangle area is " << std::sqrt(max_triangle_area) << endl;
    TriangleMeshType tri_mesh = bh::MLibUtilities::convertMlibToBh(clipped_mesh);
//    const size_t subdivisions = tri_mesh.subdivideTrianglesUntilMaxArea(max_triangle_area);
    const size_t subdivisions = tri_mesh.subdivideTrianglesUntilMaxArea(
            max_triangle_area, TriangleMeshType::SUBDIVIDE_MIDPOINTS_4);
    cout << "Performed " << subdivisions << " subdivisions" << endl;
    cout << "Refined mesh has " << tri_mesh.triangleVertexIndices().size() << " triangles" << endl;
    MeshDataType refined_mesh = bh::MLibUtilities::convertBhToMlib(tri_mesh);
    cout << "Refined mlib mesh has " << refined_mesh.m_FaceIndicesVertices.size() << " triangles" << endl;
    return refined_mesh;
  }

  MeshDataType computeRefinedMeshOrig(const MeshDataType& mesh, const FloatType max_triangle_area_square) const {
    std::vector<Triangle> triangles;
    for (std::size_t i = 0; i < mesh.m_FaceIndicesVertices.size(); ++i) {
      const MeshDataType::Indices::Face& face = mesh.m_FaceIndicesVertices[i];
      BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");

      Triangle tri(
              bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[0]]),
              bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[1]]),
              bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[2]]));
      if (!roi_bbox_.isInside(tri.v1())
          || !roi_bbox_.isInside(tri.v2())
          || !roi_bbox_.isInside(tri.v3())) {
        continue;
      }

      std::stack<Triangle> tri_stack;
      tri_stack.push(tri);
      while (!tri_stack.empty()) {
        Triangle tri = tri_stack.top();
        tri_stack.pop();
        const FloatType area_square = tri.computeTriangleAreaSquare();
        if (bh::isApproxGreater(area_square, max_triangle_area_square, FloatType(1e-3))) {
          std::array<Triangle, 2> tri_array = tri.splitTriangle();
          BH_ASSERT(tri_array[0].v1().array().isFinite().all());
          BH_ASSERT(tri_array[0].v2().array().isFinite().all());
          BH_ASSERT(tri_array[0].v3().array().isFinite().all());
          BH_ASSERT(tri_array[1].v1().array().isFinite().all());
          BH_ASSERT(tri_array[1].v2().array().isFinite().all());
          BH_ASSERT(tri_array[1].v3().array().isFinite().all());
          tri_stack.push(tri_array[0]);
          tri_stack.push(tri_array[1]);
        }
        else {
          triangles.push_back(tri);
        }
      }
    }
    cout << "Refined mesh has " << triangles.size() << " triangles" << endl;

    MeshDataType refined_mesh;
    refined_mesh.m_FaceIndicesVertices.resize(triangles.size(), 3);
    std::unordered_map<Vector3, std::size_t> vertex_index_map;
    const auto get_vertex_index_lambda = [&] (const Vector3& v) {
      const auto it = vertex_index_map.find(v);
      size_t idx;
      if (it == vertex_index_map.end()) {
        idx = refined_mesh.m_Vertices.size();
        refined_mesh.m_Vertices.push_back(bh::MLibUtilities::convertEigenToMlib(v));
        vertex_index_map.emplace(v, idx);
      }
      else {
        idx = vertex_index_map.at(v);
      }
      return idx;
    };
    for (size_t i = 0; i < triangles.size(); ++i) {
      const Triangle& tri = triangles[i];
      const size_t idx1 = get_vertex_index_lambda(tri.v1());
      const size_t idx2 = get_vertex_index_lambda(tri.v2());
      const size_t idx3 = get_vertex_index_lambda(tri.v3());
      MeshDataType::Indices::Face& face = refined_mesh.m_FaceIndicesVertices[i];
      face[0] = idx1;
      face[1] = idx2;
      face[2] = idx3;
    }
    return refined_mesh;
  }

  std::unique_ptr<BvhTreeType> generateValidPositionBVHTree(const OctreeType& octree) {
    cout << "Reading occupancy BVH tree" << endl;
    ViewpointPlanner::OccupiedTreeType occupancy_bvh_tree;
    std::ifstream ifs(planner_data_options_.getValue<string>("bvh_filename"));
    boost::archive::binary_iarchive ia(ifs);
    ia >> occupancy_bvh_tree;
    ifs.close();
    cout << "Done" << endl;

    size_t invalid_free_voxels = 0;
    size_t valid_free_voxels = 0;
    size_t non_free_voxels = 0;
    const size_t report_leaf_interval = (size_t)octree.size() / 50.0;
    std::vector<typename BvhTreeType::ObjectWithBoundingBox> objects;
    size_t i = 0;
    for (auto it = octree.begin_tree(); it != octree.end_tree(); ++i, ++it) {
      if (i % report_leaf_interval == 0) {
        const FloatType percentage = FloatType(100) * i / FloatType(octree.size());
        cout << "  processed " << percentage << " % of octree nodes" << endl;
      }
      const bool is_free = octree.isNodeFree(&(*it));
      const bool is_known = octree.isNodeKnown(&(*it));
      if (it.isLeaf()) {
        bool valid_position;
        const Vector3 center = Vector3(it.getX(), it.getY(), it.getZ());
        if (is_free && is_known) {
          valid_position = isValidObjectPosition(center, drone_extent_, occupancy_bvh_tree);
          if (!valid_position) {
            ++invalid_free_voxels;
            continue;
          }
          ++valid_free_voxels;
        }
        else {
          valid_position = false;
          ++non_free_voxels;
        }
        typename BvhTreeType::ObjectWithBoundingBox object_with_bbox;
        const float size = it.getSize();
        object_with_bbox.bounding_box = typename BvhTreeType::BoundingBoxType(center, size);
        if (pose_sample_bbox_.isOutside(center)) {
          continue;
        }
        object_with_bbox.object = new NodeObject();
        object_with_bbox.object->is_valid_object_position = valid_position;
//        object_with_bbox.bounding_box.constrainTo(pose_sample_bbox_);
//        if (object_with_bbox.bounding_box.isEmpty()) {
//          continue;
//        }

        objects.push_back(object_with_bbox);
      }
    }
    occupancy_bvh_tree.clear();

    BH_PRINT_VALUE(invalid_free_voxels);
    BH_PRINT_VALUE(valid_free_voxels);
    BH_PRINT_VALUE(non_free_voxels);

    cout << "Building BVH tree with " << objects.size() << " objects" << endl;
    bh::Timer timer;
    std::unique_ptr<BvhTreeType> bvh_tree(new BvhTreeType());
    bvh_tree->build(std::move(objects));
    timer.printTimingMs("Building BVH tree");
    return std::move(bvh_tree);
  }

  std::vector<Vector3> computeReachableVoxelPositions(
      const OctreeType& octree) {
    cout << "Reading occupancy BVH tree" << endl;
    ViewpointPlanner::OccupiedTreeType occupancy_bvh_tree;
    std::ifstream ifs(planner_data_options_.getValue<string>("bvh_filename"));
    boost::archive::binary_iarchive ia(ifs);
    ia >> occupancy_bvh_tree;
    ifs.close();
    cout << "Done" << endl;

    std::unordered_set<Vector3> reachable_voxel_positions_set;
    for (auto it = octree.begin_tree(); it != octree.end_tree(); ++it) {
      if (it.isLeaf()) {
  //      if (octree.isNodeFree(&(*it)) || octree.isNodeUnknown(&(*it))) {
        if (octree.isNodeOccupied(&(*it)) && octree.isNodeKnown(&(*it))) {
          continue;
        }
        octomap::point3d center_octomap = it.getCoordinate();
        Eigen::Vector3f center;
        center << center_octomap.x(), center_octomap.y(), center_octomap.z();
        if (!pose_sample_bbox_.isInside(center)) {
          continue;
        }
        BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(center, drone_extent_);
        std::vector<ViewpointPlanner::OccupiedTreeType::BBoxIntersectionResult> results =
            occupancy_bvh_tree.intersects(object_bbox);
        for (ViewpointPlanner::OccupiedTreeType::BBoxIntersectionResult& result : results) {
          reachable_voxel_positions_set.emplace(result.node->getBoundingBox().getCenter());
        }
      }
    }
    std::vector<Vector3> reachable_voxel_positions(reachable_voxel_positions_set.begin(), reachable_voxel_positions_set.end());
    return reachable_voxel_positions;
  }

  MeshDataType createMeshFromTriangles(const std::vector<Triangle>& triangles) const {
    MeshDataType mesh;
    mesh.m_FaceIndicesVertices.resize(triangles.size(), 3);
    std::unordered_map<Vector3, std::size_t> vertex_index_map;
    const auto get_vertex_index_lambda = [&] (const Vector3& v) {
      const auto it = vertex_index_map.find(v);
      size_t idx;
      if (it == vertex_index_map.end()) {
        idx = mesh.m_Vertices.size();
        mesh.m_Vertices.push_back(bh::MLibUtilities::convertEigenToMlib(v));
        vertex_index_map.emplace(v, idx);
      }
      else {
        idx = vertex_index_map.at(v);
      }
      return idx;
    };
    for (size_t i = 0; i < triangles.size(); ++i) {
      const Triangle& tri = triangles[i];
      const size_t idx1 = get_vertex_index_lambda(tri.v1());
      const size_t idx2 = get_vertex_index_lambda(tri.v2());
      const size_t idx3 = get_vertex_index_lambda(tri.v3());
      MeshDataType::Indices::Face& face = mesh.m_FaceIndicesVertices[i];
      face[0] = idx1;
      face[1] = idx2;
      face[2] = idx3;
    }
    return mesh;
  }

  MeshDataType filterUnobservableTriangles(const MeshDataType& mesh, MeshDataType* non_filtered_mesh = nullptr) {
    cout << "Loading occupancy octree" << endl;
    const string octree_filename = options_.getValue<string>("octree_filename");
    std::unique_ptr<OctreeType> octree =
        OctreeType::read(octree_filename);

    cout << "Computing reachable voxel positions" << endl;
    std::vector<Vector3> reachable_voxel_positions;
    if (!options_.isSet("in_reachable_voxel_positions")) {
      reachable_voxel_positions = computeReachableVoxelPositions(*octree);
      if (options_.isSet("out_reachable_voxel_positions")) {
        const string rvp_filename = options_.getValue<string>("out_reachable_voxel_positions");
        std::ofstream rvp_ofs(rvp_filename, std::ios::binary);
        if (!rvp_ofs) {
          throw BH_EXCEPTION(string("Unable to open file for writing: ") + rvp_filename);
        }
        boost::archive::binary_oarchive rvp_oa(rvp_ofs);
        rvp_oa << reachable_voxel_positions;
        rvp_ofs.close();
      }
    }
    else {
      const string rvp_filename = options_.getValue<string>("in_reachable_voxel_positions");
      std::ifstream rvp_ifs(rvp_filename, std::ios::binary);
      if (!rvp_ifs) {
        throw BH_EXCEPTION(string("Unable to open file for reading: ") + rvp_filename);
      }
      boost::archive::binary_iarchive rvp_ia(rvp_ifs);
      rvp_ia >> reachable_voxel_positions;
      rvp_ifs.close();
    }

    cout << "Found " << reachable_voxel_positions.size() << " reachable voxel positions" << endl;

    cout << "Generating BVH tree" << endl;
    std::unique_ptr<BvhTreeType> valid_position_bvh_tree;

    if (!options_.isSet("in_valid_position_bvh_tree")) {
      valid_position_bvh_tree = generateValidPositionBVHTree(*octree);
      if (options_.isSet("out_valid_position_bvh_tree")) {
        const string bvh_tree_filename = options_.getValue<string>("out_valid_position_bvh_tree");
        std::ofstream bvh_ofs(bvh_tree_filename, std::ios::binary);
        if (!bvh_ofs) {
          throw BH_EXCEPTION(string("Unable to open file for writing: ") + bvh_tree_filename);
        }
        boost::archive::binary_oarchive bvh_oa(bvh_ofs);
        bvh_oa << *valid_position_bvh_tree;
      }
    }
    else {
      valid_position_bvh_tree.reset(new BvhTreeType());
      const string bvh_tree_filename = options_.getValue<string>("in_valid_position_bvh_tree");
      std::ifstream bvh_ifs(bvh_tree_filename, std::ios::binary);
      if (!bvh_ifs) {
        throw BH_EXCEPTION(string("Unable to open file for reading: ") + bvh_tree_filename);
      }
      boost::archive::binary_iarchive bvh_ia(bvh_ifs);
      bvh_ia >> *valid_position_bvh_tree;
      bvh_ifs.close();
    }

    std::mt19937_64 rnd;
    std::shuffle(reachable_voxel_positions.begin(), reachable_voxel_positions.end(), rnd);
    reachable_voxel_positions.resize(1000);

    cout << "Filtering triangles" << endl;
//    ViewpointPlannerData::OccupiedTreeType& bvh_tree = planner_ptr_->getPlannerData().getOccupancyBVHTree();
//    const FloatType min_range = 1.5 * octree->getResolution();
    const FloatType min_range = 5.0f;
    const FloatType max_range = 50.0f;
//    std::vector<ViewpointPlanner::OccupiedTreeType::RayType> rays(reachable_voxel_positions.size());
    std::vector<Triangle> observed_triangles;
    std::vector<Triangle> non_observed_triangles;
    std::vector<size_t> observed_triangle_indices;
    std::vector<size_t> non_observed_triangle_indices;

    ml::TriMesh<FloatType> ml_tri_mesh(mesh);
    std::vector<typename ml::TriMesh<FloatType>::Triangle> ml_triangles;
    for (size_t q = 0; q < ml_tri_mesh.m_indices.size(); ++q) {
      const auto& v1 = ml_tri_mesh.m_vertices[ml_tri_mesh.m_indices[q][0]];
      const auto& v2 = ml_tri_mesh.m_vertices[ml_tri_mesh.m_indices[q][1]];
      const auto& v3 = ml_tri_mesh.m_vertices[ml_tri_mesh.m_indices[q][2]];
      typename ml::TriMesh<FloatType>::Triangle triangle(&v1, &v2, &v3);
      ml_triangles.emplace_back(&v1, &v2, &v3);
    }

    cout << "Refined mesh has " << mesh.m_FaceIndicesVertices.size() << " vertices" << endl;
    cout << "Building triangle AABB tree" << endl;
    const TriangleMeshType tri_mesh = bh::MLibUtilities::convertMlibToBh(mesh);
    const std::vector<Triangle> triangles = tri_mesh.getTriangles();
    TriAABBTree tri_aabb_tree(triangles);
    cout << "done" << endl;
    TriMeshAcceleratorType tri_mesh_acc(mesh);

    // Generate ray directions on the unit sphere
    TriangleMeshType icosahedron_mesh = bh::TriangleMeshFactory<FloatType>::createIcosahedron(1);
    const MeshDataType ml_icosahedron_mesh = bh::MLibUtilities::convertBhToMlib(icosahedron_mesh);
    MeshIOType ::saveToFile("icosahedron_mesh.ply", ml_icosahedron_mesh);
    TriangleMeshType sphere_mesh = bh::TriangleMeshFactory<FloatType>::createUnitSphere(
            options_.sphere_subdivisions);
    cout << "Sphere mesh has " << sphere_mesh.vertices().size() << " vertices" << endl;
    const MeshDataType ml_sphere_mesh = bh::MLibUtilities::convertBhToMlib(sphere_mesh);
    MeshIOType::saveToFile("sphere_mesh.ply", ml_sphere_mesh);

    const FloatType mesh_min_range = FloatType(1e-5);
    const FloatType min_visible_rays_ratio = options_.min_visible_rays_ratio;
    const size_t min_visible_rays = size_t(min_visible_rays_ratio * sphere_mesh.vertices().size());
    // TODO
//    size_t min_visible_rays = size_t(min_visible_rays_ratio * sphere_mesh.vertices().size());
//    min_visible_rays = 1;
    cout << "Minimum number of visible rays for observation = " << min_visible_rays << endl;

    const size_t report_interval = (size_t)(mesh.m_FaceIndicesVertices.size() / 1000.0);
    size_t report_counter = 0;
    const size_t batch_size = 100000;
    for (std::size_t i = 0; i < mesh.m_FaceIndicesVertices.size(); ++i, ++report_counter) {
//      if (i != 15934) {
//        continue;
//      }
      if (report_counter >= report_interval) {
        cout << "processed " << i << " out of " << mesh.m_FaceIndicesVertices.size() << " triangles" << endl;
        report_counter = 0;
      }

      const MeshDataType::Indices::Face& face = mesh.m_FaceIndicesVertices[i];
      BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");

      Triangle tri(
              bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[0]]),
              bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[1]]),
              bh::MLibUtilities::convertMlibToEigen(mesh.m_Vertices[face[2]]));
      const Vector3 center = tri.getCenter();
      const Vector3 normal = tri.getNormal();

      bool filter = false;
      size_t visible_rays = 0;
      for (size_t j = 0; j < sphere_mesh.vertices().size(); j += batch_size) {
        std::vector<BvhTreeType::RayType> rays;
        for (size_t k = 0; k < batch_size; ++k) {
          size_t idx = j + k;
          if (idx >= sphere_mesh.vertices().size()) {
            break;
          }
          const Vector3 direction = sphere_mesh.vertices()[idx];
          const BvhTreeType::RayType ray(center + min_range * direction, direction);
          // Additional checks: Make sure ray direction lies in triangle's oriented half-sphere
          const bool valid_direction = ray.direction.dot(normal) > 0;
          if (valid_direction) {
            rays.push_back(ray);
          }
        }
//        if (i == 14225) {
//          BH_PRINT_VALUE(i);
//        }
        std::vector<BvhTreeType::IntersectionResult> results = valid_position_bvh_tree->intersectsCuda(rays, 0, max_range);
        BH_ASSERT(results.size() == rays.size());
        for (size_t k = 0; k < results.size(); ++k) {
          const BvhTreeType::IntersectionResult& result = results[k];
          const BvhTreeType::RayType& ray = rays[k];
//          if (i == 14225) {
//            BH_PRINT_VALUE(result.node != nullptr);
//            BH_PRINT_VALUE(result.node->getObject()->is_valid_object_position);
//            BH_PRINT_VALUE(pose_sample_bbox_.isInside(result.intersection));
//          }
          if (result.node != nullptr && result.node->getObject()->is_valid_object_position
              && pose_sample_bbox_.isInside(result.intersection)) {
            const TriAABBTree::RayDataType mesh_ray(center + mesh_min_range * ray.direction, ray.direction);
            const TriAABBTree::RayIntersection ri = tri_aabb_tree.intersect(mesh_ray, 0, max_range);
//            const ml::Ray<FloatType> ml_ray(bh::MLibUtilities::convertEigenToMlib(mesh_ray.origin),
//                                            bh::MLibUtilities::convertEigenToMlib(mesh_ray.direction));
//            const TriMeshAcceleratorType::Intersection ml_ri = tri_mesh_acc.intersect(ml_ray, 0, max_range);
//            if (ri.doesIntersect() != ml_ri.isValid()) {
//              BH_PRINT_VALUE(ri.doesIntersect());
//              BH_PRINT_VALUE(ml_ri.isValid());
//            }
//            BH_ASSERT(ri.doesIntersect() == ml_ri.isValid());
            if (!ri.doesIntersect()) {
              ++visible_rays;
              if (visible_rays >= min_visible_rays) {
                filter = true;
                break;
              }
            }
//            if (i == 14225) {
//              BH_PRINT_VALUE(i);
//              BH_PRINT_VALUE(ri.doesIntersect());
//              BH_PRINT_VALUE(mesh_ray.origin.transpose());
//              BH_PRINT_VALUE(mesh_ray.direction.transpose());
//            }
          }
        }
        if (filter) {
          break;
        }
      }

//      if (i == 14225) {
//        BH_PRINT_VALUE(i);
//        BH_PRINT_VALUE(visible_rays);
//      }
      if (filter) {
        observed_triangles.push_back(tri);
        observed_triangle_indices.push_back(i);
      }
      else {
        non_observed_triangles.push_back(tri);
        non_observed_triangle_indices.push_back(i);
      }
    }

    cout << "Observed mesh has " << observed_triangles.size() << " triangles" << endl;
    cout << "Non-observed mesh has " << non_observed_triangles.size() << " triangles" << endl;

    std::ofstream ofs("observed_triangle_indices.txt");
    for (size_t i = 0; i < observed_triangle_indices.size(); ++i) {
      ofs << observed_triangle_indices[i] << endl;
    }
    ofs.close();

    ofs.open("non_observed_triangle_indices.txt");
    for (size_t i = 0; i < non_observed_triangle_indices.size(); ++i) {
      ofs << non_observed_triangle_indices[i] << endl;
    }
    ofs.close();

    MeshDataType observed_mesh = createMeshFromTriangles(observed_triangles);
    if (non_filtered_mesh != nullptr) {
      *non_filtered_mesh = createMeshFromTriangles(non_observed_triangles);
    }
    return observed_mesh;
  }

  bool run() {
    MeshDataType mesh;
    MeshIOType::loadFromFile(in_mesh_filename_, mesh);
    cout << "Number of vertices in mesh: " << mesh.m_Vertices.size() << endl;
    cout << "Number of triangles in mesh: " << mesh.m_FaceIndicesVertices.size() << endl;

    cout << "Refining mesh" << endl;
    const FloatType max_triangle_area = options_.max_triangle_area;
    mesh = computeRefinedMesh(mesh, max_triangle_area);
    if (options_.isSet("refined_mesh")) {
      const string refined_mesh_filename = options_.getValue<string>("refined_mesh");
      cout << "Saving refined mesh to file " << refined_mesh_filename << endl;
      MeshIOType::saveToFile(refined_mesh_filename, mesh);
    }

    const int cuda_gpu_id = options_.getValue<int>("cuda_gpu_id");
    cout << "Selecting CUDA device " << cuda_gpu_id << endl;
    bh::CudaManager::setActiveGpuId(cuda_gpu_id);
    cout << "Previous CUDA stack size was " << bh::CudaManager::getStackSize() << endl;
    size_t cuda_stack_size = options_.getValue<size_t>("cuda_stack_size");
    cout << "Setting CUDA stack size to " << cuda_stack_size << endl;
    bh::CudaManager::setStackSize(cuda_stack_size);

    cout << "Filtering observable triangles" << endl;
    MeshDataType unobservable_mesh;
    mesh = filterUnobservableTriangles(mesh, &unobservable_mesh);
    // Add colors to gt mesh
    for (size_t i = 0; i < mesh.m_Vertices.size(); ++i) {
      mesh.m_Colors.push_back(ml::vec4<FloatType>(0.6, 0.6, 0, 1));
    }
    MeshIOType::saveToFile(out_mesh_filename_, mesh);

    if (!out_unobs_mesh_filename_.empty()) {
      // Add colors to unobs mesh
      for (size_t i = 0; i < unobservable_mesh.m_Vertices.size(); ++i) {
        unobservable_mesh.m_Colors.push_back(ml::vec4<FloatType>(0.6, 0, 0.6, 1));
      }
      MeshIOType::saveToFile(out_unobs_mesh_filename_, unobservable_mesh);
    }

    return true;
  }

private:
  Options options_;
  ViewpointPlanner::Options planner_options_;
  ViewpointPlannerData::Options planner_data_options_;
  string in_mesh_filename_;
  string out_mesh_filename_;
  string out_unobs_mesh_filename_;
  BoundingBoxType roi_bbox_;
  BoundingBoxType pose_sample_bbox_;
  Vector3 drone_extent_;
//  std::unique_ptr<ViewpointPlanner> planner_ptr_;
};

const string ComputeGroundTruthMeshCmdline::Options::kPrefix = "compute_ground_truth_mesh";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("compute_ground_truth_mesh.cfg"), "Config file.")
        ("in-mesh", po::value<string>()->required(), "File to load the input mesh from.")
        ("out-gt-mesh", po::value<string>()->default_value(""), "File to write the ground truth mesh to.")
        ("out-unobs-mesh", po::value<string>()->default_value(""), "File to write the unobservable triangle meshto.")
        ;

    po::options_description options;
    options.add(generic_options);
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help")) {
      cout << options << endl;
      return std::make_pair(false, vm);
    }
    po::notify(vm);

    po::options_description config_file_options;
    for (auto& entry : config_options) {
      config_file_options.add(entry.second->getBoostOptions());
    }
    std::ifstream config_in(vm["config-file"].as<string>());
    if (!config_in) {
      throw BH_EXCEPTION("Unable to open config file");
    }
    else {
      const bool allow_unregistered = false;
      po::store(parse_config_file(config_in, config_file_options, allow_unregistered), vm);
      notify(vm);
    }

    for (auto& entry : config_options) {
      entry.second->setVariablesMap(vm);
    }

    return std::make_pair(true, vm);
  }
  catch (const po::required_option& err) {
    cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << endl;
    return std::make_pair(false, vm);
  }
  catch (const po::error& err) {
    cerr << "Error parsing command line: " << err.what() << endl;
    return std::make_pair(false, vm);
  }
}

bool ctrl_c_pressed = false;
void signalIntHandler(int sig) {
  ctrl_c_pressed = true;
}

void enableCtrlCHandler(void (*signalHandler)(int)) {
  std::signal(SIGINT, signalHandler);
}

void disableCtrlCHandler() {
  std::signal(SIGINT, SIG_DFL);
}

int main(int argc, char** argv)
{
  std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options =
      ComputeGroundTruthMeshCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  ComputeGroundTruthMeshCmdline gt_mesh_cmdline(
      config_options,
      vm["in-mesh"].as<string>(),
      vm["out-gt-mesh"].as<string>(),
      vm["out-unobs-mesh"].as<string>());

  if (gt_mesh_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
