//==================================================
// quad_planner_app.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#include <boost/program_options.hpp>
#include <memory>
#include <quad_planner/quad_planner.h>
#include <quad_planner/rendering/visualizer.h>


namespace quad_planner
{

class QuadPlannerApp
{
private:
  QuadPlanner *quad_planner_ptr_;
  rendering::Visualizer window;

public:
  QuadPlannerApp(const std::string &octomap_filename, double octomap_resolution)
  {
    quad_planner_ptr_ = new QuadPlanner(octomap_resolution);
    if (!octomap_filename.empty())
    {
      quad_planner_ptr_->loadOctomapFile(octomap_filename);
    }

    window.init();
    window.getOctomapRenderer()->setOctomap(quad_planner_ptr_->getOctomap());
  }

  ~QuadPlannerApp()
  {
    delete quad_planner_ptr_;
  }

  const QuadPlanner& getQuadPlanner() const
  {
    return *quad_planner_ptr_;
  }

  QuadPlanner& getQuadPlanner()
  {
    return *quad_planner_ptr_;
  }

  void run()
  {
    std::shared_ptr<ob::ProblemDefinition> pdef = quad_planner_ptr_->run();
    window.run(pdef);
  }
};

}

int main(int argc, char **argv)
{
  namespace po = boost::program_options;
  try
  {
    std::string octomap_filename;
    double octomap_resolution;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("octomap", po::value<std::string>(&octomap_filename)->default_value("/home/bhepp/Projects/Quad3DR/gazebo_octomap.bt"), "Filename of octomap.")
            ("resolution", po::value<double>(&octomap_resolution)->default_value(0.1), "Resolution of octomap.")
            ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    po::notify(vm);

    quad_planner::QuadPlannerApp app(octomap_filename, octomap_resolution);
    app.run();

  }
  catch (const po::required_option &err)
  {
    std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
    return 1;
  }
  catch (const po::error &err)
  {
    std::cerr << "Error parsing command line: " << err.what() << std::endl;
    return 2;
  }

  return 0;
}

