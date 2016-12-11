//==================================================
// viewpoint_planner_app.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 4, 2016
//==================================================

#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#define AIT_MLIB_COMPATIBILITY 1
#include <ait/common.h>
//#include <ait/mLib.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <QApplication>

#include "sparse_reconstruction.h"
#include "viewpoint_planner.h"
#include "viewer_window.h"

using std::cout;
using std::endl;
using std::string;

class ViewpointPlannerApp
{
public:
    ViewpointPlannerApp(std::string sparse_recon_path, std::string octree_filename, bool binary=false) {
        std::unique_ptr<SparseReconstruction> sparse_recon = readSparseReconstruction(sparse_recon_path);
        std::unique_ptr<ViewpointPlanner::OctomapType> octree = readOctree(octree_filename, binary);
        planner_ptr_ = new ViewpointPlanner(std::move(sparse_recon), std::move(octree));
        window_ptr_ = new ViewerWindow(planner_ptr_);
    }

    ~ViewpointPlannerApp() {
        SAFE_DELETE(window_ptr_);
        SAFE_DELETE(planner_ptr_);
    }

    const ViewpointPlanner& getPlanner() const {
        return *planner_ptr_;
    }

    ViewpointPlanner& getPlanner() {
        return *planner_ptr_;
    }

    ViewerWindow& getWindow() {
        return *window_ptr_;
    }

    const ViewerWindow& getWindow() const {
        return *window_ptr_;
    }

    void run() {
        planner_ptr_->run();
//        window_ptr_->start();
//        std::shared_ptr<ob::ProblemDefinition> pdef = quad_planner_ptr_->run();
        //        window_ptr_->join();
    }

private:
  static std::unique_ptr<SparseReconstruction>readSparseReconstruction(std::string path) {
    ait::Timer timer;
    std::unique_ptr<SparseReconstruction> sparse_recon(new SparseReconstruction());
    sparse_recon->read(path);
    timer.printTiming("Loading sparse reconstruction");
    return sparse_recon;
  }

  static std::unique_ptr<ViewpointPlanner::OctomapType> readOctree(std::string filename, bool binary=false) {
    ait::Timer timer;
    std::unique_ptr<ViewpointPlanner::OctomapType> octree;
    if (binary) {
      octree.reset(new ViewpointPlanner::OctomapType(filename));
    }
    else {
      octree.reset(reinterpret_cast<ViewpointPlanner::OctomapType*>(ViewpointPlanner::OctomapType::read(filename)));
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
            ("sparse-recon-path", po::value<std::string>()->required(), "Path to Colmap MVS workspace.")
            ("disable-gui", po::bool_switch()->default_value(false), "Disable graphics.")
            ;

        po::options_description octomap_options("Octomap options");
        octomap_options.add_options()
            ("map-file", po::value<std::string>()->required(), "Filename of octomap.")
//            ("resolution", po::value<double>()->default_value(0.1), "Resolution of octomap.")
            ("binary", po::bool_switch()->default_value(false), "Read a binary octree.")
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

int main(int argc, char** argv)
{
    QApplication qapp(argc, argv);
    qapp.setApplicationName("Quad3DR viewpoint planner");

    // Handle command line
    std::pair<bool, boost::program_options::variables_map> cmdline_result = process_commandline(argc, argv);
    if (!cmdline_result.first) {
        return 1;
    }
    boost::program_options::variables_map vm = std::move(cmdline_result.second);

    ViewpointPlannerApp planner_app(vm["sparse-recon-path"].as<string>(), vm["map-file"].as<string>(), vm["binary"].as<bool>());
    planner_app.getWindow().show();
    planner_app.run();
    if (planner_app.getWindow().isVisible()) {
        return qapp.exec();
    }
    else {
        return -1;
    }
}
