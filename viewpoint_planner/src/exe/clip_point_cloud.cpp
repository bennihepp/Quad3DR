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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <ait/common.h>
#include <ait/eigen.h>
#include <ait/utilities.h>
#include <ait/options.h>
#include <ait/geometry.h>

#include <ait/mLib.h>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = ait::BoundingBox3D<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using PointCloudIOType = ml::PointCloudIO<FloatType>;

class ClipPointCloudCmdline {
public:

  class Options : public ait::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : ait::ConfigOptions(kPrefix) {
      addOption<FloatType>("clip_bbox_min_x");
      addOption<FloatType>("clip_bbox_min_y");
      addOption<FloatType>("clip_bbox_min_z");
      addOption<FloatType>("clip_bbox_max_x");
      addOption<FloatType>("clip_bbox_max_y");
      addOption<FloatType>("clip_bbox_max_z");
    }

    ~Options() override {}
  };

  static std::map<string, std::unique_ptr<ait::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<ait::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new Options())));
    return config_options;
  }

  ClipPointCloudCmdline(const std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options,
      const string& in_point_cloud_filename, const string& out_point_cloud_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    in_point_cloud_filename_(in_point_cloud_filename),
    out_point_cloud_filename_(out_point_cloud_filename) {}

  ~ClipPointCloudCmdline() {
  }

  bool run() {
    BoundingBoxType clip_bbox(
        Vector3(options_.getValue<FloatType>("clip_bbox_min_x"),
            options_.getValue<FloatType>("clip_bbox_min_y"),
            options_.getValue<FloatType>("clip_bbox_min_z")),
        Vector3(options_.getValue<FloatType>("clip_bbox_max_x"),
            options_.getValue<FloatType>("clip_bbox_max_y"),
            options_.getValue<FloatType>("clip_bbox_max_z")));

    PointCloudType point_cloud;
    PointCloudIOType::loadFromFile(in_point_cloud_filename_, point_cloud);
    cout << "Number of vertices in point cloud: " << point_cloud.m_points.size() << endl;
    PointCloudType clipped_point_cloud;
    for (std::size_t i = 0; i < point_cloud.m_points.size(); ++i) {
      const ml::vec3<FloatType>& ml_point_coord = point_cloud.m_points[i];
      const Vector3 point_coord(ml_point_coord.x, ml_point_coord.y, ml_point_coord.z);
      if (clip_bbox.isInside(point_coord)) {
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
    int argc, char** argv, std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options)
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
  std::map<std::string, std::unique_ptr<ait::ConfigOptions>> config_options =
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
