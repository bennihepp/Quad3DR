//==================================================
// occupancy_map_from_mesh.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 19, 2017
//==================================================


#include <iostream>

#include <boost/program_options.hpp>

#include <octomap/octomap.h>

#include <ait/eigen.h>
#include <ait/options.h>
#include <ait/eigen_options.h>
#include <ait/geometry.h>

#include "../octree/occupancy_map.h"

#include <ait/mLib.h>
#include <ait/mLibUtils.h>

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::size_t;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = ait::BoundingBox3D<FloatType>;
using MeshType = ml::MeshData<FloatType>;
using MeshIOType = ml::MeshIO<FloatType>;
using TriMeshType = ml::TriMesh<FloatType>;
using RayType = ml::Ray<FloatType>;
using TriMeshAcceleratorType = ml::TriMeshAcceleratorBVH<FloatType>;
using OccupancyMapType = OccupancyMap<OccupancyNode>;

class OccupancyMapFromMeshCmdline {
public:

  class Options : public ait::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : ait::ConfigOptions(kPrefix) {
      addOption<Vector3>("clip_bbox_min", &clip_bbox_min);
      addOption<Vector3>("clip_bbox_max", &clip_bbox_max);
      addOption<FloatType>("mesh_scale", &mesh_scale);
      addOption<FloatType>("resolution", &resolution);
      addOption<bool>("make_dense", &make_dense);
      addOption<bool>("lazy_eval", &lazy_eval);
      addOption<bool>("allow_raycast_from_inside_mesh", &allow_raycast_from_inside_mesh);
      addOption<bool>("fill_to_bottom_as_occupied", &fill_to_bottom_as_occupied);
      addOption<bool>("verbose", &verbose);
    }

    ~Options() override {}

    Vector3 clip_bbox_min = Vector3::Constant(-10);
    Vector3 clip_bbox_max = Vector3::Constant(10);
    FloatType resolution = FloatType(0.2);
    FloatType mesh_scale = FloatType(1);
    bool make_dense = false;
    bool lazy_eval = false;
    bool allow_raycast_from_inside_mesh = false;
    bool fill_to_bottom_as_occupied = false;
    bool verbose = false;
  };

  static std::map<string, std::unique_ptr<ait::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<ait::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new Options())));
    return config_options;
  }

  OccupancyMapFromMeshCmdline(const std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options,
      const string& mesh_filename, const string& occupancy_map_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    mesh_filename_(mesh_filename),
    occupancy_map_filename_(occupancy_map_filename) {}

  ~OccupancyMapFromMeshCmdline() {
  }

  void voxelizeMesh(const OccupancyMapFromMeshCmdline::Options& options,
                    const TriMeshType& tri_mesh,
                    OccupancyMapType& tree) {
    const bool verbose = options.verbose;

    BoundingBoxType clip_bbox(
        options.clip_bbox_min,
        options.clip_bbox_max);
    const ml::vec3<FloatType> ml_clip_bbox_min(ait::MLibUtilities::convertEigenToMlib(options.clip_bbox_min));
    const ml::vec3<FloatType> ml_clip_bbox_max(ait::MLibUtilities::convertEigenToMlib(options.clip_bbox_max));
    const ml::BoundingBox3<FloatType> ml_clip_bbox(ml_clip_bbox_min, ml_clip_bbox_max);
    const ml::vec3<FloatType> grid_dimension = ml_clip_bbox.getExtent() * 2 / options.resolution;
//    for (size_t i = 0; i < 3; ++i) {
//      grid_dimension = ml_clip_bbox.getExtent() / options.resolution;
//    }
    ml::BinaryGrid3 binary_grid(ml::vec3i(grid_dimension[0], grid_dimension[1], grid_dimension[2]));
    ml::Matrix4x4<FloatType> world_to_voxel;
    ml::vec3<FloatType> scale_factors;
    for (size_t i = 0; i < 3; ++i) {
      scale_factors[i] = grid_dimension[i] / ml_clip_bbox.getExtent()[i];
    }
    AIT_PRINT_VALUE(scale_factors);
    world_to_voxel = ml::Matrix4x4<FloatType>::scale(scale_factors);
    AIT_PRINT_VALUE(world_to_voxel);
    world_to_voxel *= ml::Matrix4x4<FloatType>::translation(-ml_clip_bbox.getMin());
    AIT_PRINT_VALUE(world_to_voxel);
    const bool solid = false;
    tri_mesh.voxelize(binary_grid, world_to_voxel, solid, verbose);
    const ml::Matrix4x4<FloatType> voxel_to_world = world_to_voxel.getInverse();
    for (size_t ix = 0; ix < binary_grid.getDimX(); ++ix) {
      for (size_t iy = 0; iy < binary_grid.getDimY(); ++iy) {
        for (size_t iz = 0; iz < binary_grid.getDimZ(); ++iz) {
          const bool occupied = binary_grid.isVoxelSet(ix, iy, iz);
          if (occupied) {
            const ml::vec3<FloatType> ml_point = voxel_to_world * ml::vec3<FloatType>(ix, iy, iz);
            const octomap::point3d point(ml_point[0], ml_point[1], ml_point[2]);
            tree.updateNode(point, occupied, options.lazy_eval);
          }
        }
      }
    }
  }

  void sweepAxisAndUpdateOccupiedNodes(size_t sweep_axis_index,
                                       const OccupancyMapFromMeshCmdline::Options& options,
                                       const TriMeshAcceleratorType& tri_mesh_acc,
                                       OccupancyMapType& tree) {
    cout << "Sweeping axis " << sweep_axis_index << endl;

    const bool verbose = options.verbose;

    BoundingBoxType clip_bbox(
        options.clip_bbox_min,
        options.clip_bbox_max);

    octomap::point3d clip_bbox_min(clip_bbox.getMinimum(0), clip_bbox.getMinimum(1), clip_bbox.getMinimum(2));
    octomap::point3d clip_bbox_max(clip_bbox.getMaximum(0), clip_bbox.getMaximum(1), clip_bbox.getMaximum(2));
    octomap::OcTreeKey key_min = tree.coordToKey(clip_bbox_min);
    octomap::OcTreeKey key_max = tree.coordToKey(clip_bbox_max);
    clip_bbox_min = tree.keyToCoord(key_min);
    clip_bbox_max = tree.keyToCoord(key_max);

    ml::vec3<FloatType> ray_direction;
    ray_direction[sweep_axis_index] = -1;
    const FloatType tmin = FloatType(1e-5) / options.mesh_scale;
    const FloatType tmax = std::numeric_limits<FloatType>::max();

    size_t free_count = 0;
    size_t occupied_count = 0;

    cout << "clip_bbox_min=" << clip_bbox_min << endl;
    cout << "clip_bbox_max=" << clip_bbox_max << endl;

    size_t axis1_index = sweep_axis_index + 1;
    if (axis1_index >= 3) {
      axis1_index = 0;
    }
    size_t axis2_index = axis1_index + 1;
    if (axis2_index >= 3) {
      axis2_index = 0;
    }
    cout << "sweep_axis_index=" << sweep_axis_index << ", axis1_index=" << axis1_index << ", axis2_index=" << axis2_index << endl;

    for (FloatType x1 = clip_bbox_min(axis1_index); x1 <= clip_bbox_max(axis1_index); x1 += options.resolution) {
      cout << "x1=" << x1 << endl;
      for (FloatType x2 = clip_bbox_min(axis2_index); x2 <= clip_bbox_max(axis2_index); x2 += options.resolution) {
        cout << "x2=" << x2 << endl;
//    FloatType x = -4.1f;
//    FloatType y = 3.7f;
//    {
//      {
        const FloatType x_sweep_min = clip_bbox_min(sweep_axis_index);
        const FloatType x_sweep_max = clip_bbox_max(sweep_axis_index);
        // Shoot ray down along z-axis and find intersections with mesh.

        bool occupied_space = false;
        bool first_intersection = true;
        FloatType x_sweep_previous = x_sweep_max;
        FloatType sweep_progress = std::numeric_limits<FloatType>::max();
        while (x_sweep_previous > x_sweep_min && sweep_progress > 1e-5) {
          ml::vec3<FloatType> ray_origin;
          ray_origin[axis1_index] = x1;
          ray_origin[axis2_index] = x2;
          ray_origin[sweep_axis_index] = x_sweep_previous;
          const RayType ray(ray_origin / options.mesh_scale, ray_direction / options.mesh_scale);
          const bool only_front_faces = false;
          TriMeshAcceleratorType::Intersection intersection = tri_mesh_acc.intersect(ray, tmin, tmax, only_front_faces);
          if (verbose) {
            cout << "Cast ray for x1=" << x1 << ", x2=" << x2 << ", sweep_previous=" << x_sweep_previous << endl;
            cout << "Intersection: " << intersection.isValid() << endl;
          }
          if (intersection.isValid()) {
            const FloatType x_sweep_next = intersection.getSurfacePosition().z * options.mesh_scale;
            const FloatType dot_product = ml::vec3<FloatType>::dot(intersection.getSurfaceNormal(), ray_direction);
            if (verbose) {
              cout << "Hit at z_next=" << x_sweep_next << endl;
              cout << "dot_product=" << dot_product << endl;
              cout << "surface=" << intersection.getSurfaceNormal() << endl;
            }
            // We started a ray from inside the mesh
            bool transition_to_occupied = false;
            const ml::vec3<FloatType> next_ray_origin(x1, x2, x_sweep_next);
            const RayType next_ray(next_ray_origin / options.mesh_scale, ray_direction / options.mesh_scale);
            TriMeshAcceleratorType::Intersection next_intersection = tri_mesh_acc.intersect(next_ray, tmin, tmax, only_front_faces);
            if (next_intersection.isValid()) {
              const FloatType next_dot_product = ml::vec3<FloatType>::dot(next_intersection.getSurfaceNormal(), ray_direction);
              if (verbose) {
                cout << "next_next_z=" << next_intersection.getSurfacePosition().z << endl;
                cout << "next_surfacez=" << next_intersection.getSurfaceNormal() << endl;
                cout << "next_dot_product=" << next_dot_product << endl;
              }
              if (dot_product < 0) {
                transition_to_occupied = true;
              }
//              if (dot_product < 0 && next_dot_product > 0) {
//                transition_to_occupied = true;
//              }
            }
            else {
              if (verbose) {
                cout << "No next intersection" << endl;
              }
              transition_to_occupied = true;
            }
            if (transition_to_occupied) {
              if (verbose) {
                cout << "Transition to occupied space" << endl;
              }
            }
            if (dot_product > 0 && first_intersection && options.allow_raycast_from_inside_mesh) {
              if (verbose) {
                cout << "Started ray from inside mesh" << endl;
              }
              occupied_space = true;
            }
            for (FloatType x_sweep = x_sweep_previous; x_sweep >= x_sweep_next; x_sweep -= options.resolution) {
              const bool occupied = occupied_space;
              octomap::point3d point;
              point(axis1_index) = x1;
              point(axis2_index) = x2;
              point(sweep_axis_index) = x_sweep;
#pragma omp critical
              if (!options_.make_dense || occupied) {
                tree.updateNode(point, occupied, options_.lazy_eval);
//                OccupancyNode* node = tree.search(point, tree.getTreeDepth());
//                AIT_PRINT_VALUE(point);
//                AIT_PRINT_VALUE(node);
//                AIT_PRINT_VALUE(tree.keyToCoord(tree.coordToKey(point)));
              }
              {
                if (occupied) {
                  ++occupied_count;
                }
                else {
                  ++free_count;
                }
              }
            }
            // Make sure the end voxel of the intersecting ray is marked as occupied
            const bool occupied = occupied_space;
            octomap::point3d point;
            point(axis1_index) = x1;
            point(axis2_index) = x2;
            point(sweep_axis_index) = x_sweep_next;
            if (!options_.make_dense || occupied) {
              tree.updateNode(point, occupied, options_.lazy_eval);
            }
            {
              if (occupied) {
                ++occupied_count;
              }
              else {
                ++free_count;
              }
            }
            if (transition_to_occupied) {
//              if (z_next >= 45) {
//                cout << "Cast ray for x=" << x << ", y=" << y << ", z_start=" << z_previous << endl;
//                cout << "Intersection: " << intersection.isValid() << endl;
//                cout << "dot_product=" << dot_product << endl;
//                cout << "surface=" << intersection.getSurfaceNormal() << endl;
//                cout << "z_next=" << z_next << endl;
//              }
//              AIT_ASSERT(z_next < 45);
              // Transition from outside to inside
              if (verbose) {
                cout << "Switching to occupied space" << endl;
              }
              occupied_space = true;
            }
            else {
              if (verbose) {
                cout << "Switching to free space" << endl;
              }
              occupied_space = false;
            }
            sweep_progress = x_sweep_previous - x_sweep_next;
            x_sweep_previous = x_sweep_next;
            first_intersection = false;
          }
          else {
            if (options_.fill_to_bottom_as_occupied) {
              if (verbose) {
                cout << "Filling nodes to bottom as occupied=" << occupied_space << endl;
              }
              for (FloatType x_sweep = x_sweep_previous; x_sweep >= x_sweep_min; x_sweep -= options_.resolution) {
                const bool occupied = occupied_space;
                octomap::point3d point;
                point(axis1_index) = x1;
                point(axis2_index) = x2;
                point(sweep_axis_index) = x_sweep;
  #pragma omp critical
                {
                  if (!options_.make_dense || occupied) {
                    tree.updateNode(point, occupied, options_.lazy_eval);
                  }
                  if (occupied) {
                    ++occupied_count;
                  }
                  else {
                    ++free_count;
                  }
                }
              }
            }
            sweep_progress = x_sweep_previous - x_sweep_min;
            x_sweep_previous = x_sweep_min;
          }
        }
      }
    }
  }

  bool run() {
    MeshType mesh_data;
    MeshIOType::loadFromFile(mesh_filename_, mesh_data);
    cout << "Number of vertices in mesh: " << mesh_data.m_Vertices.size() << endl;
    TriMeshType tri_mesh(mesh_data);

    OccupancyMapType tree(options_.resolution);

//    // Build accelerator structure
//    cout << "Building mesh accelerator structure" << endl;
//    TriMeshAcceleratorType tri_mesh_acc(tri_mesh);
//    cout << "Done" << endl;

//    sweepAxisAndUpdateOccupiedNodes(2, options_, tri_mesh_acc, tree);
//    sweepAxisAndUpdateOccupiedNodes(0, options_, tri_mesh_acc, tree);
//    sweepAxisAndUpdateOccupiedNodes(1, options_, tri_mesh_acc, tree);

    voxelizeMesh(options_, tri_mesh, tree);

    if (options_.lazy_eval) {
      cout << "Updating inner nodes" << endl;
      tree.updateInnerOccupancy();
    }

    octomap::point3d point(-4.1f, 3.7f, 11.5f);
    AIT_PRINT_VALUE(tree.keyToCoord(tree.coordToKey(point)));
    OccupancyNode* node = tree.search(point, tree.getTreeDepth());
    AIT_PRINT_VALUE(static_cast<void*>(node));
    if (node != nullptr) {
      AIT_PRINT_VALUE(node->getOccupancy());
      AIT_PRINT_VALUE(node->getObservationCount());
    }

    if (options_.make_dense) {
      cout << "Octree has " << tree.getNumLeafNodes() << " leaf nodes and " << tree.size() << " total nodes" << endl;
      cout << "Filling unknown nodes" << endl;
      for (auto it = tree.begin_tree(); it != tree.end_tree(); ++it) {
        if (!it.isLeaf()) {
          for (size_t i = 0; i < 8; ++i) {
            if (!tree.nodeChildExists(&(*it), i)) {
              tree.createNodeChild(&(*it), i);
              OccupancyNode* child = tree.getNodeChild(&(*it), i);
              child->setOccupancy(0);
              child->setObservationCount(1);
            }
          }
        }
      }
      cout << "Updating inner nodes" << endl;
      tree.updateInnerOccupancy();
    }

//    cout << "Pruning tree" << endl;
//    tree.prune();

    size_t unknown_count = 0;
    size_t free_count = 0;
    size_t occupied_count = 0;
    for (auto it = tree.begin_tree(); it != tree.end_tree(); ++it) {
      if (it.isLeaf()) {
        if (it->getObservationCount() == 0) {
          ++unknown_count;
        }
        else if (tree.isNodeOccupied(it->getOccupancy())) {
          ++occupied_count;
        }
        else {
          ++free_count;
        }
      }
    }
    std::cout << "unknown=" << unknown_count << ", occupied=" << occupied_count << ", free=" << free_count << std::endl;

    cout << "Octree has " << tree.getNumLeafNodes() << " leaf nodes and " << tree.size() << " total nodes" << endl;
//    std::cout << "Metric extents:" << std::endl;
//    double x, y, z;
//    tree.getMetricSize(x, y, z);
//    std::cout << "  size=(" << x << ", " << y << ", " << z << ")" << std::endl;
//    tree.getMetricMin(x, y, z);
//    std::cout << "   min=(" << x << ", " << y << ", " << z << ")" << std::endl;
//    tree.getMetricMax(x, y, z);
//    std::cout << "   max=(" << x << ", " << y << ", " << z << ")" << std::endl;


    tree.write(occupancy_map_filename_);

    return true;
  }

private:
  Options options_;
  string mesh_filename_;
  string occupancy_map_filename_;
};

const string OccupancyMapFromMeshCmdline::Options::kPrefix = "occupancy_map_from_mesh";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("occupancy_map_from_mesh.cfg"), "Config file.")
        ("in-mesh", po::value<string>(), "File to load the input mesh from.")
        ("out-occupancy-map", po::value<string>()->required(), "File to save the occupancy map to.")
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
      throw AIT_EXCEPTION("Unable to open config file");
    }
    else {
      po::store(parse_config_file(config_in, config_file_options), vm);
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

int main(int argc, char** argv) {
  std::map<std::string, std::unique_ptr<ait::ConfigOptions>> config_options =
      OccupancyMapFromMeshCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  OccupancyMapFromMeshCmdline occupancy_map_from_mesh_cmdline(
      config_options, vm["in-mesh"].as<string>(), vm["out-occupancy-map"].as<string>());

  if (occupancy_map_from_mesh_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
