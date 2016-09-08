#pragma once

#include <memory>
#include <boost/shared_ptr.hpp>
#include <octomap/octomap.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <quad_planner/optimizing_rrt_planner.h>


namespace quad_planner
{

// Local namespace abbreviations
namespace ob = ompl::base;
namespace og = ompl::geometric;

class QuadPlanner
{
private:
  const double OCCUPANCY_FREE_THRESHOLD = 0.2;

  using StateSpaceT = ob::SE3StateSpace;
//  using StateSpaceT = ob::RealVectorStateSpace;
  using PlannerT = OptimizingRRT;
//  using PlannerT = og::RRT;

  boost::shared_ptr<StateSpaceT> state_space_;
  boost::shared_ptr<ob::SpaceInformation> space_info_;
  boost::shared_ptr<PlannerT> planner_;
  std::shared_ptr<octomap::OcTree> octomap_ptr_;


public:
  QuadPlanner(double octomap_resolution);
  virtual ~QuadPlanner();

  std::shared_ptr<const octomap::OcTree> getOctomap() const;
  void loadOctomapFile(const std::string &filename);

  bool isStateValid(const ob::State *state) const;
  void run();
};

}
