//==================================================
// quad_planner.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#pragma once

#include <memory>
#include <memory>
#include <octomap/octomap.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <quad_planner/optimizing_rrt_planner.h>


namespace quad_planner
{

// Local namespace abbreviations
namespace ob = ::ompl::base;
namespace og = ::ompl::geometric;

class QuadPlanner
{
private:
  const double OCCUPANCY_FREE_THRESHOLD = 0.2;

  using StateSpaceT = ob::SE3StateSpace;
//  using StateSpaceT = ob::RealVectorStateSpace;
  using PlannerT = OptimizingRRT;
//  using PlannerT = og::RRT;

  std::shared_ptr<StateSpaceT> state_space_;
  std::shared_ptr<ob::SpaceInformation> space_info_;
  std::shared_ptr<PlannerT> planner_;
  std::shared_ptr<octomap::OcTree> octomap_ptr_;


public:
  QuadPlanner(double octomap_resolution);
  virtual ~QuadPlanner();

  std::shared_ptr<const octomap::OcTree> getOctomap() const;
  void loadOctomapFile(const std::string &filename);

  struct MotionValidator : public ob::MotionValidator
  {
    MotionValidator(const QuadPlanner* parent, ob::SpaceInformation* si);

    bool checkMotion(const ob::State* state1, const ob::State* state2,
        std::pair<ob::State*, double>& lastValid) const;
    bool checkMotion(const ob::State* state1, const ob::State* state2) const;

    const QuadPlanner* planner_;
  };

  bool isCoordinateValid(double x, double y, double z) const;
  bool isStateValid(const ob::State* state) const;

  std::shared_ptr<ob::ProblemDefinition> run();
};

}
