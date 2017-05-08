//==================================================
// evaluate_dense_points.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 12, 2017
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <bh/boost.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

#include <bh/common.h>
#include <bh/eigen.h>
#include <bh/utilities.h>
#include <bh/math/utilities.h>
#include <bh/math/histogram.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/math/geometry.h>
#include <bh/mLib/mLibUtils.h>
#include <bh/nn/approximate_nearest_neighbor.h>

#include <bh/mLib/mLib.h>

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
using TriMeshType = ml::TriMesh<FloatType>;
using MeshIOType = ml::MeshIO<FloatType>;
using RayType = ml::Ray<FloatType>;
using TriMeshAcceleratorType = ml::TriMeshAcceleratorBVH<FloatType>;
using Triangle = bh::Triangle<FloatType>;

class EvaluateDensePointsCmdline {
public:
  struct Options : bh::ConfigOptions {
    static const string kPrefix;

    Options()
    : bh::ConfigOptions(kPrefix) {
      addOptionRequired<Vector3>("roi_bbox_min", &roi_bbox_min);
      addOptionRequired<Vector3>("roi_bbox_max", &roi_bbox_max);
      addOption<FloatType>("max_correspondence_distance", &max_correspondence_distance);
      addOption<size_t>("max_radius_search_results", &max_radius_search_results);
      addOption<FloatType>("dist_truncation", &dist_truncation);
    }

    ~Options() override {}

    Vector3 roi_bbox_min;
    Vector3 roi_bbox_max;
    FloatType max_correspondence_distance = FloatType(0.2);
    size_t max_radius_search_results = 500;
    FloatType dist_truncation = FloatType(0.2);
  };

  static std::map<string, std::unique_ptr<bh::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new Options())));
    return config_options;
  }

  EvaluateDensePointsCmdline(
      const std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options,
      const string& ground_truth_mesh_filename,
      const string& in_point_cloud_filename,
      const string& negative_output_mesh_filename,
      const string& positive_output_mesh_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    ground_truth_mesh_filename_(ground_truth_mesh_filename),
    in_point_cloud_filename_(in_point_cloud_filename),
    negative_output_mesh_filename_(negative_output_mesh_filename),
    positive_output_mesh_filename_(positive_output_mesh_filename),
    roi_bbox_(options_.roi_bbox_min, options_.roi_bbox_max) {}

  ~EvaluateDensePointsCmdline() {
  }

  struct Vector3Hash {
    size_t operator()(const Vector3& v) const {
      size_t val { 0 };
      boost::hash_combine(val, boost::hash<float>{}(v(0)));
      boost::hash_combine(val, boost::hash<float>{}(v(1)));
      boost::hash_combine(val, boost::hash<float>{}(v(2)));
      return val;
    }
  };

  std::tuple<MeshDataType, MeshDataType> evaluate(const MeshDataType& gt_mesh, const PointCloudType& input_point_cloud) const {
    std::vector<FloatType> triangle_areas;
    triangle_areas.reserve(gt_mesh.m_FaceIndicesVertices.size());
#pragma omp parallel for
    for (std::size_t i = 0; i < gt_mesh.m_FaceIndicesVertices.size(); ++i) {
      const MeshDataType::Indices::Face &face = gt_mesh.m_FaceIndicesVertices[i];
      BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");

      Triangle tri(
              bh::MLibUtilities::convertMlibToEigen(gt_mesh.m_Vertices[face[0]]),
              bh::MLibUtilities::convertMlibToEigen(gt_mesh.m_Vertices[face[1]]),
              bh::MLibUtilities::convertMlibToEigen(gt_mesh.m_Vertices[face[2]]));
      const FloatType area = std::sqrt(tri.computeTriangleAreaSquare());
      triangle_areas.push_back(area);
    }

    std::vector<Vector3> eigen_point_cloud;
    eigen_point_cloud.reserve(input_point_cloud.m_points.size());
    for (size_t i = 0; i < input_point_cloud.m_points.size(); ++i) {
      const ml::vec3f& v = input_point_cloud.m_points[i];
      eigen_point_cloud.push_back(bh::MLibUtilities::convertMlibToEigen(v));
    }
    bh::ApproximateNearestNeighbor<FloatType, 3> point_cloud_ann;
    point_cloud_ann.initIndex(eigen_point_cloud.begin(), eigen_point_cloud.end());

//    std::vector<size_t> ind;
//    std::vector<FloatType> dist;
//    flann::SearchParams search_params;
//    search_params.checks = flann::FLANN_CHECKS_UNLIMITED;
//    point_cloud_ann.setSearchParams(search_params);
//    point_cloud_ann.radiusSearch(eigen_point_cloud.front(), 100, 1000, &ind, &dist);
//    BH_PRINT_VALUE(ind.size());
//    BH_PRINT_VALUE(dist.size());
//    for (size_t i = 0; i < ind.size(); ++i) {
//      BH_PRINT_VALUE(i);
//      BH_PRINT_VALUE(ind[i]);
//      BH_PRINT_VALUE(dist[i]);
//    }
//    ind.resize(1000);
//    dist.resize(1000);
//    point_cloud_ann.knnSearch(eigen_point_cloud.front(), 1000, &ind, &dist);
//    BH_PRINT_VALUE(ind.size());
//    BH_PRINT_VALUE(dist.size());
//    for (size_t i = 0; i < ind.size(); ++i) {
//      BH_PRINT_VALUE(i);
//      BH_PRINT_VALUE(ind[i]);
//      BH_PRINT_VALUE(dist[i]);
//    }
//    point_cloud_ann.radiusSearchExact(eigen_point_cloud.front(), 100, 1000, &ind, &dist);
//    BH_PRINT_VALUE(ind.size());
//    BH_PRINT_VALUE(dist.size());
//    for (size_t i = 0; i < ind.size(); ++i) {
//      BH_PRINT_VALUE(i);
//      BH_PRINT_VALUE(ind[i]);
//      BH_PRINT_VALUE(dist[i]);
//    }
//    BH_DEBUG_BREAK;

//    point_cloud_ann.

    const FloatType max_correspondence_dist = options_.max_correspondence_distance;
    const size_t max_results = options_.max_radius_search_results;

    std::vector<size_t> num_correspondences(gt_mesh.m_FaceIndicesVertices.size(), 0);
    std::vector<FloatType> dist_square_closest_point(gt_mesh.m_FaceIndicesVertices.size(), std::numeric_limits<FloatType>::max());
    std::vector<Vector3> closest_point(gt_mesh.m_FaceIndicesVertices.size());
#pragma omp parallel for
    for (std::size_t i = 0; i < gt_mesh.m_FaceIndicesVertices.size(); ++i) {
//      if (i != 9012) {
//        continue;
//      }
      const MeshDataType::Indices::Face& face = gt_mesh.m_FaceIndicesVertices[i];
      BH_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");

      Triangle tri(
              bh::MLibUtilities::convertMlibToEigen(gt_mesh.m_Vertices[face[0]]),
              bh::MLibUtilities::convertMlibToEigen(gt_mesh.m_Vertices[face[1]]),
              bh::MLibUtilities::convertMlibToEigen(gt_mesh.m_Vertices[face[2]]));
      const Vector3 center = tri.getCenter();
//      const FloatType l1 = (tri.v1() - center).squaredNorm();
//      const FloatType l2 = (tri.v2() - center).squaredNorm();
//      const FloatType l3 = (tri.v3() - center).squaredNorm();
//      const FloatType max_length_square = std::max(l1, std::max(l2, l3));
//      const FloatType max_length = std::sqrt(max_length_square);

//      const FloatType radius = std::max(max_length, options_.max_correspondence_distance);
//      const FloatType radius_square = radius * radius;
      std::vector<size_t> indices;
      std::vector<FloatType> distances;
//      point_cloud_ann.radiusSearch(center, radius_square, max_results, &indices, &distances);
      indices.resize(max_results);
      distances.resize(max_results);
      point_cloud_ann.knnSearch(center, max_results, &indices, &distances);
//      // Testing radius search
//      std::vector<size_t> indices2;
//      std::vector<FloatType> distances2;
//      point_cloud_ann.radiusSearchExact(center, radius_square, max_results, &indices2, &distances2);
////      BH_ASSERT(indices.size() == indices2.size());
//      indices = indices2;
//      distances = distances2;
      for (size_t j = 0; j < indices.size(); ++j) {
        const size_t index = indices[j];
//        const FloatType distance_square = distances[j];
        const Vector3 point = eigen_point_cloud[index];
        const bool projects_onto_triangle = tri.doesPointProjectOntoTriangle(point);
        if (projects_onto_triangle) {
          const FloatType distance_to_triangle = tri.distanceToSurface(point);
          const FloatType distance_to_triangle_square = distance_to_triangle * distance_to_triangle;
          if (distance_to_triangle_square <= dist_square_closest_point[i]) {
            dist_square_closest_point[i] = distance_to_triangle_square;
            closest_point[i] = point;
          }
          if (std::abs(distance_to_triangle) <= max_correspondence_dist) {
            ++num_correspondences[i];
          }
        }
      }
    }

    const size_t min_num_correspondences_for_coverage = 1;
    const FloatType dist_square_truncation = options_.dist_truncation * options_.dist_truncation;

    const FloatType sum_trunc_dist_square = std::accumulate(dist_square_closest_point.begin(), dist_square_closest_point.end(), FloatType(0),
                                                    [&] (const FloatType value, const FloatType dist_square) {
      const FloatType dist_square_truncated = std::min(dist_square, dist_square_truncation);
      return value + dist_square_truncated;
    });
    const FloatType average_trunc_dist_square = sum_trunc_dist_square / dist_square_closest_point.size();
    cout << "average_trunc_dist_square = " << average_trunc_dist_square << endl;

    const size_t num_covered_triangles = std::count_if(num_correspondences.begin(), num_correspondences.end(),
                                                       [&] (const size_t num_correspondence) {
      return num_correspondence >= min_num_correspondences_for_coverage;
    });
    const FloatType coverage_ratio = num_covered_triangles / FloatType(num_correspondences.size());
    cout << "coverage_ratio = " << coverage_ratio << endl;

    // TODO: Compute covered area
    FloatType covered_area= 0;
    FloatType total_area = 0;
    for (size_t i = 0; i < triangle_areas.size(); ++i) {
      if (num_correspondences[i] >= min_num_correspondences_for_coverage) {
        covered_area += triangle_areas[i];
      }
      total_area += triangle_areas[i];
    }
    const FloatType coverage_area_ratio = covered_area / FloatType(total_area);
    cout << "coverage_area_ratio = " << coverage_area_ratio << endl;

    // Compute histogram of truncated squared distances
//    std::vector<FloatType> dist_square_trunc_closest_point;
//    std::transform(dist_square_closest_point.begin(), dist_square_closest_point.end(),
//                   std::back_inserter(dist_square_trunc_closest_point),
//                   [&](const FloatType dist_square) {
//                     return std::min(dist_square, dist_square_truncation);
//                   });
    const size_t num_bins = 10;
    std::vector<FloatType> histogram_bins;
    for (size_t i = 0; i < num_bins; ++i) {
      const FloatType factor = i / FloatType(num_bins);
      histogram_bins.push_back(factor * dist_square_truncation);
    }
    const std::vector<size_t> histogram = bh::computeHistogram(
            dist_square_closest_point.begin(), dist_square_closest_point.end(),
            histogram_bins.begin(), histogram_bins.end());
    cout << "Histogram of distances:" << endl;
    for (size_t i = 0; i < histogram_bins.size(); ++i) {
      const FloatType bin_low = std::sqrt(histogram_bins[i]);
      FloatType bin_high;
      if (i + 1< histogram_bins.size()) {
        bin_high = std::sqrt(histogram_bins[i + 1]);
      }
      else {
        bin_high = std::numeric_limits<FloatType>::infinity();
      }
      const size_t count = histogram[i];
      const FloatType relative_count = count / (FloatType)dist_square_closest_point.size();
      cout << "  Bin [" << bin_low << ", " << bin_high << "): " << relative_count << endl;
    }

    // Copy mesh and add different colors for covered and non-covered triangles
    MeshDataType negative_output_mesh;
    MeshDataType positive_output_mesh;
    const size_t face_valence = 3;
    negative_output_mesh.m_FaceIndicesVertices.resize(gt_mesh.m_FaceIndicesVertices.size(), face_valence);
    positive_output_mesh.m_FaceIndicesVertices.resize(gt_mesh.m_FaceIndicesVertices.size(), face_valence);
    for (size_t i = 0; i < gt_mesh.m_FaceIndicesVertices.size(); ++i) {
      if (num_correspondences[i] >= min_num_correspondences_for_coverage) {
        positive_output_mesh.m_FaceIndicesVertices.push_back(gt_mesh.m_FaceIndicesVertices[i]);
      }
      else {
        negative_output_mesh.m_FaceIndicesVertices.push_back(gt_mesh.m_FaceIndicesVertices[i]);
      }
    }
    for (size_t i = 0; i < gt_mesh.m_Vertices.size(); ++i) {
      negative_output_mesh.m_Vertices.push_back(gt_mesh.m_Vertices[i]);
      positive_output_mesh.m_Vertices.push_back(gt_mesh.m_Vertices[i]);
      negative_output_mesh.m_Colors.push_back(ml::vec4<FloatType>(0.8, 0, 0, 1));
      positive_output_mesh.m_Colors.push_back(ml::vec4<FloatType>(0, 0.8, 0, 1));
    }
    return std::make_tuple(std::move(negative_output_mesh), std::move(positive_output_mesh));
  }

  bool run() {
    MeshDataType gt_mesh;
    MeshIOType::loadFromFile(ground_truth_mesh_filename_, gt_mesh);
    cout << "Number of vertices in ground truth mesh: " << gt_mesh.m_Vertices.size() << endl;

    PointCloudType input_point_cloud;
    PointCloudIOType::loadFromFile(in_point_cloud_filename_, input_point_cloud);
    cout << "Number of vertices in input point cloud: " << input_point_cloud.m_points.size() << endl;

    MeshDataType negative_output_mesh;
    MeshDataType positive_output_mesh;
    std::tie(negative_output_mesh, positive_output_mesh) = evaluate(gt_mesh, input_point_cloud);
    if (!negative_output_mesh_filename_.empty()) {
      MeshIOType::saveToFile(negative_output_mesh_filename_, negative_output_mesh);
    }
    if (!positive_output_mesh_filename_.empty()) {
      MeshIOType::saveToFile(positive_output_mesh_filename_, positive_output_mesh);
    }

    return true;
  }

private:
  Options options_;
  string ground_truth_mesh_filename_;
  string in_point_cloud_filename_;
  string negative_output_mesh_filename_;
  string positive_output_mesh_filename_;
  BoundingBoxType roi_bbox_;
};

const string EvaluateDensePointsCmdline::Options::kPrefix = "evaluate_dense_points";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("evaluate_dense_points.cfg"), "Config file.")
        ("ground-truth-mesh", po::value<string>()->required(), "File to load the ground truth mesh from.")
        ("in-point-cloud", po::value<string>()->required(), "File to load the input point cloud from.")
        ("negative-output-mesh", po::value<string>()->default_value(""), "File to write the negative mesh to.")
        ("positive-output-mesh", po::value<string>()->default_value(""), "File to write the positive mesh to.")
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
  std::map<std::string, std::unique_ptr<bh::ConfigOptions>> config_options =
      EvaluateDensePointsCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  EvaluateDensePointsCmdline evaluate_cmdline(
      config_options,
      vm["ground-truth-mesh"].as<string>(),
      vm["in-point-cloud"].as<string>(),
      vm["negative-output-mesh"].as<string>(),
      vm["positive-output-mesh"].as<string>());

  if (evaluate_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
