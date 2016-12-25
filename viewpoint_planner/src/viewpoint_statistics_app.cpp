//==================================================
// viewpoint_statistics_app.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 14, 2016
//==================================================

#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

#define AIT_MLIB_COMPATIBILITY 1
#include <ait/common.h>
//#include <ait/mLib.h>

#include "reconstruction/dense_reconstruction.h"
#include "occupancy_map.h"
#include "planner/viewpoint_planner.h"

using std::cout;
using std::endl;
using std::string;

class ViewpointStatisticsApp
{
public:
  ViewpointStatisticsApp(std::string dense_recon_path, std::string octree_filename) {
      std::unique_ptr<DenseReconstruction> dense_recon = readDenseReconstruction(dense_recon_path);
      std::unique_ptr<ViewpointPlanner::OccupancyMapType> octree = readOctree(octree_filename);
      planner_ptr_ = new ViewpointPlanner(std::move(dense_recon), std::move(octree));
  }

  ~ViewpointStatisticsApp() {
      SAFE_DELETE(planner_ptr_);
  }

  const ViewpointPlanner& getPlanner() const {
      return *planner_ptr_;
  }

  ViewpointPlanner& getPlanner() {
      return *planner_ptr_;
  }

  void run() {
      planner_ptr_->run();
//        window_ptr_->start();
//        std::shared_ptr<ob::ProblemDefinition> pdef = quad_planner_ptr_->run();
      //        window_ptr_->join();
  }

private:
  static std::unique_ptr<DenseReconstruction>readDenseReconstruction(std::string path) {
    ait::Timer timer;
    std::unique_ptr<DenseReconstruction> dense_recon(new DenseReconstruction());
    dense_recon->read(path);
    timer.printTiming("Loading dense reconstruction");
    return dense_recon;
  }

  static std::unique_ptr<ViewpointPlanner::OccupancyMapType> readOctree(std::string filename, bool binary=false) {
    ait::Timer timer;
    std::unique_ptr<ViewpointPlanner::OccupancyMapType> octree;
    if (binary) {
//      octree.reset(new ViewpointPlanner::OccupancyMapType(filename));
      throw AIT_EXCEPTION("Binary occupancy maps not supported");
    }
    else {
      octree.reset(reinterpret_cast<ViewpointPlanner::OccupancyMapType*>(ViewpointPlanner::OccupancyMapType::read(filename)));
    }
    if (!octree) {
      throw std::runtime_error("Unable to read octomap file");
    }
    timer.printTiming("Loading octree");

    std::cout << "Loaded octree" << std::endl;
    std::cout << "Octree has " << octree->getNumLeafNodes() << " leaf nodes and " << octree->size() << " total nodes" << std::endl;
    std::cout << "Metric extents:" << std::endl;
    double x, y, z;
    timer = ait::Timer();
    octree->getMetricSize(x, y, z);
    timer.printTiming("Computing octree size");
    std::cout << "  size=(" << x << ", " << y << ", " << z << ")" << std::endl;
    timer = ait::Timer();
    octree->getMetricMin(x, y, z);
    timer.printTiming("Computing octree min");
    std::cout << "   min=(" << x << ", " << y << ", " << z << ")" << std::endl;
    timer = ait::Timer();
    octree->getMetricMax(x, y, z);
    timer.printTiming("Computing octree max");
    std::cout << "   max=(" << x << ", " << y << ", " << z << ")" << std::endl;

    size_t count_unknown = 0;
    size_t count_unknown_leaf = 0;
    size_t count_irregular = 0;
    for (auto it = octree->begin_tree(); it != octree->end_tree(); ++it) {
      if (it->getObservationCount() == 0) {
        ++count_unknown;
        if (it.isLeaf()) {
          ++count_unknown_leaf;
        }
      }
      if (!it.isLeaf() && it->getObservationCount() == 0) {
        ++count_irregular;
      }
    }
    std::cout << "count_unknown: " << count_unknown << std::endl;
    std::cout << "count_unknown_leaf: " << count_unknown_leaf << std::endl;
    std::cout << "count_irregular: " << count_irregular << std::endl;

    return octree;
  }

private:
  ViewpointPlanner *planner_ptr_;
  ViewerWindow *window_ptr_;
};

std::pair<bool, boost::program_options::variables_map> process_commandline(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::variables_map vm;
    try {
        po::options_description generic_options("Generic options");
        generic_options.add_options()
            ("help", "Produce help message")
            ("dense-recon-path", po::value<std::string>()->required(), "Path to Colmap MVS workspace.")
            ;

        po::options_description octomap_options("Octomap options");
        octomap_options.add_options()
            ("map-file", po::value<std::string>()->required(), "Filename of octomap.")
            ;

        po::options_description options;
        options.add(generic_options);
        options.add(octomap_options);
        po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
        if (vm.count("help"))
        {
            std::cout << options << std::endl;
            return std::make_pair(false, vm);
        }

        po::notify(vm);

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

int main(int argc, char** argv) {
    // Handle command line
    std::pair<bool, boost::program_options::variables_map> cmdline_result = process_commandline(argc, argv);
    if (!cmdline_result.first) {
        return 1;
    }
    boost::program_options::variables_map vm = std::move(cmdline_result.second);

    ViewpointStatisticsApp statistics_app(vm["dense-recon-path"].as<string>(), vm["map-file"].as<string>());
    statistics_app.run();

    return 0;
}
