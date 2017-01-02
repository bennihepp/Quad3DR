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
    return config_options;
  }

    ViewpointPlannerCmdline(const std::map<std::string, std::unique_ptr<ait::ConfigOptions>>& config_options) {
      const ViewpointPlannerData::Options* viewpoint_planner_data_options =
          dynamic_cast<ViewpointPlannerData::Options*>(config_options.at("viewpoint_planner.data").get());
      std::unique_ptr<ViewpointPlannerData> planner_data(
          new ViewpointPlannerData(viewpoint_planner_data_options));
      planner_ptr_ = new ViewpointPlanner(
          dynamic_cast<ViewpointPlanner::Options*>(config_options.at("viewpoint_planner").get()),
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

bool ctrl_pressed = false;
void signalIntHandler(int sig) {
  ctrl_pressed = true;
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

    std::signal(SIGINT, signalIntHandler);
    bool stop;
    do {
      bool result = planner_cmdline.runIteration();
      std::cout << "Result -> " << result << std::endl;
      stop = false;
    }
    while (!stop && !ctrl_pressed);

    return 0;
}
