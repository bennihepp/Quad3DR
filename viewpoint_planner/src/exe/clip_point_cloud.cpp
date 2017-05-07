//==================================================
// clip_point_cloud.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 28, 2016
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <bh/boost.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <bh/common.h>
#include <bh/eigen.h>
#include <bh/utilities.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/math/geometry.h>
#include <bh/gps.h>

#include <bh/mLib/mLib.h>

#include "../reconstruction/dense_reconstruction.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = bh::BoundingBox3D<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using PointCloudIOType = ml::PointCloudIO<FloatType>;

class ClipPointCloudCmdline {
public:

  class Options : public bh::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : bh::ConfigOptions(kPrefix) {
      addOption<Vector3>("offset_vector", &offset_vector);
      addOption<FloatType>("clip_distance", &clip_distance);
      addOption<Vector3>("clip_bbox_min", &clip_bbox_min);
      addOption<Vector3>("clip_bbox_max", &clip_bbox_max);
      addOption<bool>("use_region_file", &use_region_file);
      addOption<string>("regions_json_filename", &regions_json_filename);
      addOption<string>("dense_reconstruction_path", &dense_reconstruction_path);
    }

    ~Options() override {}

    Vector3 offset_vector = Vector3(0, 0, 0);
    FloatType clip_distance = 0;
    Vector3 clip_bbox_min;
    Vector3 clip_bbox_max;
    bool use_region_file = false;
    string regions_json_filename;
    string dense_reconstruction_path;
  };

  static std::map<string, std::unique_ptr<bh::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new Options())));
    return config_options;
  }

  ClipPointCloudCmdline(const std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options,
      const string& in_point_cloud_filename, const string& out_point_cloud_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    in_point_cloud_filename_(in_point_cloud_filename),
    out_point_cloud_filename_(out_point_cloud_filename) {}

  ~ClipPointCloudCmdline() {
  }

  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = bh::GpsConverter<GpsFloatType>;
  using RegionType = bh::PolygonWithLowerAndUpperPlane<FloatType>;

  // TODO: Move into viewpoint_planner_common
  RegionType convertGpsRegionToEnuRegion(
          const GpsConverter& gps_converter,
          const boost::property_tree::ptree& pt) const {
    const bool verbose = true;
    const FloatType lower_altitude = pt.get<FloatType>("lower_altitude");
    const FloatType upper_altitude = pt.get<FloatType>("upper_altitude");
    std::vector<GpsCoordinateType> gps_coordinates;
    for (const boost::property_tree::ptree::value_type& v : pt.get_child("latlong_vertices")) {
      gps_coordinates.push_back(GpsCoordinateType(v.second.get<FloatType>("latitude"), v.second.get<FloatType>("longitude"), 0));
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

  template <typename Predicate>
  PointCloudType clipPointCloud(const PointCloudType& point_cloud, Predicate&& predicate) {
    PointCloudType clipped_point_cloud;
    for (std::size_t i = 0; i < point_cloud.m_points.size(); ++i) {
      const ml::vec3<FloatType>& ml_point_coord = point_cloud.m_points[i];
      const Vector3 point_coord(ml_point_coord.x, ml_point_coord.y, ml_point_coord.z);
      if (std::forward<Predicate>(predicate)(point_coord)) {
        clipped_point_cloud.m_points.push_back(point_cloud.m_points[i]);
        if (point_cloud.hasColors()) {
          clipped_point_cloud.m_colors.push_back(point_cloud.m_colors[i]);
        }
        if (point_cloud.hasNormals()) {
          clipped_point_cloud.m_normals.push_back(point_cloud.m_normals[i]);
        }
        if (point_cloud.hasTexCoords()) {
          clipped_point_cloud.m_texCoords.push_back(point_cloud.m_texCoords[i]);
        }
      }
    }
    return clipped_point_cloud;
  }

  bool run() {
    PointCloudType point_cloud;
    PointCloudIOType::loadFromFile(in_point_cloud_filename_, point_cloud);
    cout << "Number of vertices in point cloud: " << point_cloud.m_points.size() << endl;
    PointCloudType clipped_point_cloud;

    if (options_.use_region_file) {
      BH_ASSERT(options_.isSet("regions_json_filename"));
      BH_ASSERT(options_.isSet("dense_reconstruction_path"));
      reconstruction::DenseReconstruction dense_reconstruction;
      const bool read_sfm_gps_transformation = true;
      dense_reconstruction.read(options_.dense_reconstruction_path, read_sfm_gps_transformation);
      const GpsCoordinateType gps_reference = dense_reconstruction.sfmGpsTransformation().gps_reference;
      std::cout << "GPS reference: " << gps_reference << std::endl;
      const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);

      // Get ROI for clipping point cloud
      boost::property_tree::ptree pt;
      boost::property_tree::read_json(options_.regions_json_filename, pt);
      std::cout << "Region of interest" << std::endl;
      const RegionType roi = convertGpsRegionToEnuRegion(gps_converter, pt.get_child("regionOfInterest"));
      std::vector<RegionType> no_fly_zones;
      std::cout << "ROI bounding box: " << roi.getBoundingBox() << std::endl;
      for (const boost::property_tree::ptree::value_type& v : pt.get_child("noFlyZones")) {
        std::cout << "No-Fly-Zones" << std::endl;
        no_fly_zones.push_back(convertGpsRegionToEnuRegion(gps_converter, v.second));
      }
      clipped_point_cloud = clipPointCloud(point_cloud, [&](const Vector3& point) -> bool {
        const Vector3 point_with_offset = point + options_.offset_vector;
        for (const RegionType& no_fly_zone : no_fly_zones) {
          if (no_fly_zone.isPointInside(point_with_offset)) {
            return false;
          }
        }
        if (roi.isPointOutside(point_with_offset)) {
          return roi.distanceToPoint(point_with_offset) < options_.clip_distance;
        }
        return true;
      });
    }
    else {
      BoundingBoxType clip_bbox(options_.clip_bbox_min, options_.clip_bbox_max);
      clipped_point_cloud = clipPointCloud(point_cloud, [&](const Vector3& point) -> bool {
        const Vector3 point_with_offset = point + options_.offset_vector;
        if (clip_bbox.isOutside(point_with_offset)) {
          return clip_bbox.distanceTo(point_with_offset) < options_.clip_distance;
        }
        return true;
      });
    }

    cout << "Number of vertices in clipped point cloud: " << clipped_point_cloud.m_points.size() << endl;
    PointCloudIOType::saveToFile(out_point_cloud_filename_, clipped_point_cloud);

    return true;
  }

private:
  Options options_;
  string in_point_cloud_filename_;
  string out_point_cloud_filename_;
};

const string ClipPointCloudCmdline::Options::kPrefix = "clip_point_cloud";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("clip_point_cloud.cfg"), "Config file.")
        ("in-point-cloud", po::value<string>(), "File to load the input point cloud from.")
        ("out-point-cloud", po::value<string>()->required(), "File to save the clipped point cloud to.")
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
      ClipPointCloudCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  ClipPointCloudCmdline clip_cmdline(
      config_options, vm["in-point-cloud"].as<string>(), vm["out-point-cloud"].as<string>());

  if (clip_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
