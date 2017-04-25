//==================================================
// create_baseline_viewpoint_path.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 27, 2017
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <bh/boost.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <bh/common.h>
#include <bh/eigen.h>
#include <bh/utilities.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/eigen_utils.h>
#include <bh/math/geometry.h>

#include <bh/mLib/mLib.h>

#include "../planner/viewpoint_planner.h"
#include "../planner/viewpoint_planner_serialization.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = bh::BoundingBox3D<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using PointCloudIOType = ml::PointCloudIO<FloatType>;

class BaselineViewpointPathCmdline {
public:

  class Options : public bh::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : bh::ConfigOptions(kPrefix) {
      addOption<bool>("verbose", &verbose);
      addOption<Vector3>("object_center", &object_center);
      addOption<std::size_t>("num_of_viewpoints", &num_of_viewpoints);
      addOption<string>("mode", &mode);
      addOption<FloatType>("circle_radius", &circle_radius);
      addOption<FloatType>("circle_height", &circle_height);
      addOption<FloatType>("circle_factor", &circle_factor);
    }

    ~Options() override {}

    bool verbose = false;
    Vector3 object_center = Vector3::Zero();
    std::size_t num_of_viewpoints = 15;
    string mode = "automatic_circle";
    FloatType circle_radius = 5;
    FloatType circle_height = 15;
    FloatType circle_factor = FloatType(1);
  };

  static std::map<string, std::unique_ptr<bh::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new Options())));
    config_options.emplace(std::piecewise_construct,
      std::forward_as_tuple("viewpoint_planner.data"),
      std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new ViewpointPlannerData::Options())));
    config_options.emplace(std::piecewise_construct,
      std::forward_as_tuple("viewpoint_planner"),
      std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new ViewpointPlanner::Options())));
    config_options.emplace(std::piecewise_construct,
      std::forward_as_tuple("motion_planner"),
      std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new ViewpointPlanner::MotionPlannerType::Options())));
    return config_options;
  }

  BaselineViewpointPathCmdline(const std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options,
      const string& viewpoint_path_filename, const string& viewpoint_path_filename_txt)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    planner_ptr_(nullptr),
    viewpoint_path_filename_(viewpoint_path_filename),
    viewpoint_path_filename_txt_(viewpoint_path_filename_txt) {
    const ViewpointPlannerData::Options* viewpoint_planner_data_options =
      dynamic_cast<ViewpointPlannerData::Options*>(config_options.at("viewpoint_planner.data").get());
    std::unique_ptr<ViewpointPlannerData> planner_data(
      new ViewpointPlannerData(viewpoint_planner_data_options));
    planner_ptr_ = new ViewpointPlanner(
      dynamic_cast<ViewpointPlanner::Options*>(config_options.at("viewpoint_planner").get()),
      dynamic_cast<ViewpointPlanner::MotionPlannerType::Options*>(config_options.at("motion_planner").get()),
      std::move(planner_data));
  }

  ~BaselineViewpointPathCmdline() {
    SAFE_DELETE(planner_ptr_);
  }

  bool run() {
    const bool verbose = options_.verbose;

    const Vector3 object_center = options_.object_center;
    const FloatType num_of_viewpoints = options_.num_of_viewpoints;
    if (options_.mode == "automatic_circle" || options_.mode == "manual_circle") {
      FloatType circle_radius;
      FloatType circle_height;
      if (options_.mode == "manual_circle") {
        circle_radius = options_.circle_radius;
        circle_height = options_.circle_height;
      }
      else {
        const BoundingBoxType roi_bbox = planner_ptr_->getRoiBbox();
        const FloatType extent_x = roi_bbox.getExtent(0);
        const FloatType extent_y = roi_bbox.getExtent(1);
        circle_radius = std::sqrt(extent_x * extent_x + extent_y * extent_y);
//        circle_height = circle_radius;
        circle_height = options_.circle_height;
        std::cout << "Automatic circle with radius " << circle_radius << " and height " << circle_height << std::endl;
      }

      ViewpointPlanner::ViewpointPath viewpoint_path;
      ViewpointPlanner::ViewpointPathComputationData comp_data;
      comp_data.num_connected_entries = 0;
      for (std::size_t i = 0; i < num_of_viewpoints; ++i) {
        if (verbose) {
          BH_PRINT_VALUE(i);
        }
        const FloatType current_angle = 2 * M_PI * (i / FloatType(num_of_viewpoints));
        if (verbose) {
          BH_PRINT_VALUE(current_angle);
        }
        const float x = circle_radius * std::cos(current_angle);
        const float y = circle_radius * std::sin(current_angle);
        const float z = circle_height;
        const Vector3 viewpoint_position = Vector3(object_center(0), object_center(1), 0) + Vector3(x, y, z);
        if (verbose) {
          BH_PRINT_VALUE(viewpoint_position);
        }
        const Vector3 rotation_z_axis = (object_center - viewpoint_position).normalized();
        const Vector3 rotation_x_axis = rotation_z_axis.cross(Vector3::UnitZ()).normalized();
        const Vector3 rotation_y_axis = rotation_z_axis.cross(rotation_x_axis).normalized();
        if (verbose) {
          BH_PRINT_VALUE(rotation_x_axis);
          BH_PRINT_VALUE(rotation_y_axis);
          BH_PRINT_VALUE(rotation_z_axis);
        }
        Matrix3x3 rotation_matrix;
        rotation_matrix.col(0) = rotation_x_axis;
        rotation_matrix.col(1) = rotation_y_axis;
        rotation_matrix.col(2) = rotation_z_axis;
        if (verbose) {
          BH_PRINT_VALUE(rotation_matrix);
        }
        const Quaternion viewpoint_quaternion(rotation_matrix);
        const Quaternion viewpoint_quaternion2 = bh::getZLookAtQuaternion(object_center - viewpoint_position, Vector3::UnitZ());
        if (verbose) {
          BH_PRINT_VALUE(viewpoint_quaternion);
          BH_PRINT_VALUE(viewpoint_quaternion2);
        }
        const ViewpointPlanner::Pose pose =
            ViewpointPlanner::Pose::createFromImageToWorldTransformation(viewpoint_position, viewpoint_quaternion);
        const PinholeCamera* camera = nullptr;
        const Viewpoint viewpoint(camera, pose);
        ViewpointPlanner::ViewpointPathEntry path_entry;
  //      path_entry.viewpoint_index = (ViewpointPlanner::ViewpointEntryIndex)-1;
        path_entry.viewpoint_index = i;
        path_entry.viewpoint = viewpoint;
        viewpoint_path.entries.push_back(path_entry);
      }

      std::vector<ViewpointPlanner::ViewpointPath> viewpoint_paths;
      std::vector<ViewpointPlanner::ViewpointPathComputationData> viewpoint_paths_data;
      ViewpointPlannerData::OccupiedTreeType bvh_tree;
      viewpoint_paths.push_back(viewpoint_path);
      viewpoint_paths_data.push_back(comp_data);
      std::cout << "Writing viewpoint path to " << viewpoint_path_filename_ << std::endl;
      std::ofstream ofs(viewpoint_path_filename_);
      boost::archive::binary_oarchive oa(ofs);
      ViewpointPathSaver vps(viewpoint_paths, viewpoint_paths_data, bvh_tree);
      oa << vps;
      std::unordered_map<
        ViewpointPlanner::ViewpointIndexPair,
        ViewpointPlanner::ViewpointMotion,
        ViewpointPlanner::ViewpointIndexPair::Hash> local_viewpoint_path_motions;
      oa << local_viewpoint_path_motions;
      std::cout << "Done" << std::endl;

      if (!viewpoint_path_filename_txt_.empty()) {
        planner_ptr_->exportViewpointPathAsText(viewpoint_path_filename_txt_, viewpoint_path);
      }
    }
    else {
      std::cout << "ERROR: Unknown baseline mode: " << options_.mode << std::endl;
    }

    return true;
  }

private:
  Options options_;
  ViewpointPlanner *planner_ptr_;
  string viewpoint_path_filename_;
  string viewpoint_path_filename_txt_;
};

const string BaselineViewpointPathCmdline::Options::kPrefix = "baseline_viewpoint_path";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("baseline_viewpoint_path.cfg"), "Config file.")
        ("out-viewpoint-path", po::value<string>()->required(), "File to save the viewpoint-path to.")
        ("out-viewpoint-path-txt", po::value<string>()->default_value(""), "File to save the viewpoint-path to as text description.")
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
      BaselineViewpointPathCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  BaselineViewpointPathCmdline baseline_cmdline(
      config_options, vm["out-viewpoint-path"].as<string>(), vm["out-viewpoint-path-txt"].as<string>());

  if (baseline_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
