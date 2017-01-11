//==================================================
// viewpoint_planner_cmdline.cpp
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
#include <ait/utilities.h>
#include <ait/options.h>

#include "../reconstruction/dense_reconstruction.h"
#include "../octree/occupancy_map.h"
#include "../planner/viewpoint_planner.h"

using std::cout;
using std::endl;
using std::string;

class ViewpointPlannerCmdline {
public:
  static std::map<std::string, std::unique_ptr<ait::ConfigOptions>> getConfigOptions() {
    std::map<std::string, std::unique_ptr<ait::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple("viewpoint_planner.data"),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new ViewpointPlannerData::Options())));
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple("viewpoint_planner"),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new ViewpointPlanner::Options())));
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple("motion_planner"),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new ViewpointPlanner::MotionPlannerType::Options())));
    return config_options;
  }

    ViewpointPlannerCmdline(const std::map<std::string, std::unique_ptr<ait::ConfigOptions>>& config_options) {
      const ViewpointPlannerData::Options* viewpoint_planner_data_options =
          dynamic_cast<ViewpointPlannerData::Options*>(config_options.at("viewpoint_planner.data").get());
      std::unique_ptr<ViewpointPlannerData> planner_data(
          new ViewpointPlannerData(viewpoint_planner_data_options));
      planner_ptr_ = new ViewpointPlanner(
          dynamic_cast<ViewpointPlanner::Options*>(config_options.at("viewpoint_planner").get()),
          dynamic_cast<ViewpointPlanner::MotionPlannerType::Options*>(config_options.at("motion_planner").get()),
          std::move(planner_data));
    }

    ~ViewpointPlannerCmdline() {
        SAFE_DELETE(planner_ptr_);
    }

    const ViewpointPlanner& getPlanner() const {
        return *planner_ptr_;
    }

    ViewpointPlanner& getPlanner() {
        return *planner_ptr_;
    }

    bool runIteration() {
      bool result = planner_ptr_->generateNextViewpointEntry();
      std::cout << "Graph size: " << planner_ptr_->getViewpointGraph().size();
      std::cout << "Path size: " << planner_ptr_->getBestViewpointPath().entries.size();
      return result;
    }

private:
  ViewpointPlanner *planner_ptr_;
};

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<std::string, std::unique_ptr<ait::ConfigOptions>>& config_options)
{
    namespace po = boost::program_options;

    po::variables_map vm;
    try {
        po::options_description generic_options("Generic options");
        generic_options.add_options()
            ("help", "Produce help message")
            ("config-file", po::value<string>()->default_value("viewpoint_planner.cfg"), "Config file.")
            ("num-candidates", po::value<std::size_t>()->default_value(1000), "Number of candidate viewpoints to sample.")
            ("num-viewpoints", po::value<std::size_t>()->default_value(25), "Number of path viewpoints to compute.")
            ("in-viewpoint-graph-file", po::value<std::string>(), "Viewpoint graph file to load before processing.")
            ("out-viewpoint-graph-file", po::value<std::string>()->required(), "File to save the viewpoint graph to after processing.")
            ;

        po::options_description options;
        options.add(generic_options);
        po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
        if (vm.count("help"))
        {
            std::cout << options << std::endl;
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
    catch (const po::required_option& err)
    {
        std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
        return std::make_pair(false, vm);
    }
    catch (const po::error& err)
    {
        std::cerr << "Error parsing command line: " << err.what() << std::endl;
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
        ViewpointPlannerCmdline::getConfigOptions();

    // Handle command line and config file
    std::pair<bool, boost::program_options::variables_map> cmdline_result =
        processOptions(argc, argv, config_options);
    if (!cmdline_result.first) {
        return 1;
    }
    boost::program_options::variables_map vm = std::move(cmdline_result.second);

    ViewpointPlannerCmdline planner_cmdline(config_options);

    if (vm.count("in-viewpoint-graph-file") > 0) {
      planner_cmdline.getPlanner().loadViewpointGraph(vm["in-viewpoint-graph-file"].as<std::string>());
    }

    enableCtrlCHandler(signalIntHandler);
    std::cout << "Sampling viewpoint candidates" << std::endl;
    while (!ctrl_c_pressed && planner_cmdline.getPlanner().getViewpointGraph().numVertices() < vm["num-candidates"].as<std::size_t>()) {
      const bool result = planner_cmdline.getPlanner().generateNextViewpointEntry();
      std::cout << "Generate next viewpoint result -> " << result << std::endl;
      std::cout << "Sampled " << planner_cmdline.getPlanner().getViewpointGraph().numVertices()
          << " of " << vm["num-candidates"].as<std::size_t>() << " viewpoints" << std::endl;
    }
    std::cout << "Done" << std::endl;
    disableCtrlCHandler();

    planner_cmdline.getPlanner().saveViewpointGraph(vm["out-viewpoint-graph-file"].as<std::string>());

    std::cout << "Computing motions" << std::endl;
    planner_cmdline.getPlanner().computeViewpointMotions();
    std::cout << "Done" << std::endl;
    planner_cmdline.getPlanner().saveViewpointGraph(vm["out-viewpoint-graph-file"].as<std::string>());

    enableCtrlCHandler(signalIntHandler);
    std::cout << "Computing viewpoint paths" << std::endl;
    while (!ctrl_c_pressed && planner_cmdline.getPlanner().getBestViewpointPath().entries.size() < vm["num-viewpoints"].as<std::size_t>()) {
      bool result = planner_cmdline.getPlanner().findNextViewpointPathEntries();
      std::cout << "Find next viewpoint path entries result -> " << result << std::endl;
      std::cout << "Computed " << planner_cmdline.getPlanner().getBestViewpointPath().entries.size()
          << " of " << vm["num-viewpoints"].as<std::size_t>() << " viewpoint path entries" << std::endl;
    }
    disableCtrlCHandler();
    std::cout << "Done" << std::endl;

    return 0;
}
