/*
 * motion_planner.h
 *
 *  Created on: Dec 30, 2016
 *      Author: bhepp
 */
#pragma once

#include <memory>
#include <functional>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ait/eigen.h>
#include <ait/math.h>
#include <ait/options.h>
#include "viewpoint_planner_data.h"

namespace ob = ::ompl::base;
namespace og = ::ompl::geometric;

template <typename FloatT>
class MotionPlanner {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using BoundingBoxType = ViewpointPlannerData::BoundingBoxType;
  using Pose = ait::Pose<FloatType>;

  using StateSpaceType = ob::SE3StateSpace;
//  using StateSpaceType = ob::RealVectorStateSpace;
//  using PlannerType = og::RRTConnect;
  using PlannerType = og::RRTstar;
//  using PlannerType = og::PRMstar;

  struct Options : ait::ConfigOptions {
    Options()
    : ait::ConfigOptions("motion_planner", "MotionPlanner options") {
      addOption<FloatType>("max_motion_range", &max_motion_range);
      addOption<FloatType>("max_time_per_solve", &max_time_per_solve);
      addOption<std::size_t>("max_iterations_per_solve", &max_iterations_per_solve);
    }

    ~Options() override {}

    // Maximum length of a motion segment
    FloatType max_motion_range = 5;
    // Maximum time that a solve is allowed to take (-1 for not limit)
    FloatType max_time_per_solve = 0.01f;
    // Maximum number of iterations that a solve is allowed to take (0 for no limit)
    std::size_t max_iterations_per_solve = 1000;
  };

  MotionPlanner(ViewpointPlannerData* data)
  : initialized_(false), data_(data) {}

  void setSpaceBoundingBox(const BoundingBoxType& space_bbox) {
    space_bbox_ = space_bbox;
  }

  void setObjectExtent(const Vector3& object_extent) {
    object_extent_ = object_extent;
  }

  void initialize() {
    AIT_ASSERT((object_extent_.array() != 0).all());
    AIT_ASSERT(!space_bbox_.isEmpty());

//    // Setup state space
//    space_ = std::make_shared<StateSpaceType>();
//    ob::RealVectorBounds bounds(3);
//    for (std::size_t i = 0; i < 3; ++i) {
//      bounds.setLow(i, space_bbox_.getMinimum(i));
//      bounds.setHigh(i, space_bbox_.getMaximum(i));
//    }
//    std::cout << "Bounding box=" << space_bbox_ << std::endl;
//    space_->setBounds(bounds);
////
//    space_info_ = std::make_shared<ob::SpaceInformation>(space_);
//    space_info_->setStateValidityChecker(std::bind(&MotionPlanner::isStateValid, this, std::placeholders::_1));
//
//    std::cout << "Space information settings:" << std::endl;
//    space_info_->printSettings(std::cout);

//    problem_def_ = std::make_shared<ob::ProblemDefinition>(space_info_);
//    problem_def_->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(space_info_));
//
//    planner_ = std::make_shared<PlannerType>(space_info_);
//    planner_->setRange(options_.max_motion_range);
//    planner_->setGoalBias(0.1);
//    planner_->setProblemDefinition(problem_def_);
//
//    planner_->setup();
//
//    std::cout << "Problem definition:" << std::endl;
//    problem_def_->print(std::cout);

//    double construction_time = 10;
//    std::cout << "Building roadmap for " << construction_time << " seconds" << std::endl;
//    planner_->constructRoadmap(ob::timedPlannerTerminationCondition(construction_time, construction_time / 10.0));
//    std::cout << "Done" << std::endl;

    initialized_ = true;
  }

  /// Check if an object can be placed at a position (i.e. is it free space)
  bool isValidObjectPosition(const Vector3& position, const Vector3& object_extent) {
    BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(position, object_extent);
    std::vector<ViewpointPlannerData::OccupiedTreeType::BBoxIntersectionResult> results =
        data_->occupied_bvh_.intersects(object_bbox);
    return results.empty();
  }

  std::pair<bool, FloatType> findMotion(const Pose& from, const Pose& to) {
    if (!initialized_) {
      initialize();
    }
    ait::Timer timer;

    // TODO: Initialize a pool of structures in the beginning for thread-safety

    // Setup state space
    std::shared_ptr<StateSpaceType> space_ = std::make_shared<StateSpaceType>();
    ob::RealVectorBounds bounds(3);
    for (std::size_t i = 0; i < 3; ++i) {
      bounds.setLow(i, space_bbox_.getMinimum(i));
      bounds.setHigh(i, space_bbox_.getMaximum(i));
    }
//    std::cout << "Bounding box=" << space_bbox_ << std::endl;
    space_->setBounds(bounds);
//
    std::shared_ptr<ob::SpaceInformation> space_info_ = std::make_shared<ob::SpaceInformation>(space_);
    space_info_->setStateValidityChecker(std::bind(&MotionPlanner::isStateValid, this, std::placeholders::_1));

    // Create start and goal state
    ob::ScopedState<StateSpaceType> start = createStateFromPose(from, space_info_);
    ob::ScopedState<StateSpaceType> goal = createStateFromPose(to, space_info_);

    if (!isStateValid(start.get())) {
      std::cerr << "ERROR: start state not valid: from=" << from << std::endl;
    }
    if (!isStateValid(goal.get())) {
      std::cerr << "ERROR: goal state not valid: to=" << to << std::endl;
    }
    AIT_ASSERT(isStateValid(start.get()));
    AIT_ASSERT(isStateValid(goal.get()));

    std::shared_ptr<ob::ProblemDefinition> problem_def_ = std::make_shared<ob::ProblemDefinition>(space_info_);
    problem_def_->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(space_info_));
    problem_def_->setStartAndGoalStates(start, goal);

    std::shared_ptr<PlannerType> planner_ = std::make_shared<PlannerType>(space_info_);
    planner_->setRange(options_.max_motion_range);
    planner_->setGoalBias(0.1);
    planner_->setProblemDefinition(problem_def_);

    planner_->setup();

////    planner_->clearQuery();
//    planner_->clear();
//
//    problem_def_->clearSolutionNonExistenceProof();
//    problem_def_->clearSolutionPaths();
//    problem_def_->clearStartStates();
//    problem_def_->clearGoal();
//    problem_def_->setStartAndGoalStates(start, goal);
//    planner_->setProblemDefinition(problem_def_);

//    std::cout << "Problem definition:" << std::endl;
//    problem_def_->print(std::cout);

    ob::PlannerTerminationCondition termination_cond = getTerminationCondition();

    ob::PlannerStatus solved = planner_->solve(termination_cond);

    timer.printTimingMs("MotionPlanner::findMotion()");

    if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
//      std::cout << "Found exact solution:" << std::endl;
      const ob::PathPtr path = problem_def_->getSolutionPath();;
      const og::PathGeometric* geo_path = dynamic_cast<og::PathGeometric*>(path.get());

      // print the path to screen
//      path->print(std::cout);

//      ob::PlannerSolution solution(path);
//      problem_def_->getSolution(solution);
//      std::cout << "Length solution path: " << problem_def_->getSolutionPath()->length() << std::endl;
//      std::cout << "Solution path length: " << geo_path->length() << std::endl;
//      std::cout << "Solution difference: " << solution.difference_ << std::endl;
//      std::cout << "Solution cost: " <<  << std::endl;
      // TODO: Remove
      FloatType motion_distance = geo_path->cost(problem_def_->getOptimizationObjective()).value();
      FloatType distance_square = (from.getWorldPosition() - to.getWorldPosition()).squaredNorm();
      FloatType distance2 = space_info_->distance(start.get(), goal.get());
      if (motion_distance * motion_distance < distance_square) {
        std::cout << "distance_square=" << distance_square << std::endl;
        std::cout << "distance2 * distance2=" << distance2 * distance2 << std::endl;
        std::cout << "motion_distance * motion_distance=" << motion_distance * motion_distance << std::endl;
      }
      AIT_ASSERT(ait::isApproxGreater(motion_distance * motion_distance, distance_square, 1e-2));
      AIT_ASSERT(ait::isApproxGreater(motion_distance * motion_distance, distance2 * distance2, 1e-2));
      return std::make_pair(true, motion_distance);
    }
    else {
//      std::cout << "No solution found" << std::endl;
      return std::make_pair(false, -1);
    }
  }

private:
  bool isStateValid(const ob::State *state) {
    const StateSpaceType::StateType* state_tmp = static_cast<const StateSpaceType::StateType*>(state);
    Vector3 position(state_tmp->getX(), state_tmp->getY(), state_tmp->getZ());
    return isValidObjectPosition(position, object_extent_);
  }

  ob::ScopedState<StateSpaceType> createStateFromPose(const Pose& pose, const std::shared_ptr<ob::SpaceInformation>& space_info) const {
    ob::ScopedState<StateSpaceType> state(space_info);
    const Vector3& from_position = pose.getWorldPosition();
    state->setXYZ(from_position(0), from_position(1), from_position(2));
//    state->rotation().w = pose.quaternion().w();
//    state->rotation().x = pose.quaternion().x();
//    state->rotation().y = pose.quaternion().y();
//    state->rotation().z = pose.quaternion().z();
//    // TODO
    state->rotation().setIdentity();
    return state;
  }

  ob::PlannerTerminationCondition getTerminationCondition() const {
    AIT_ASSERT(options_.max_time_per_solve > 0 || options_.max_iterations_per_solve > 0);
    if (options_.max_time_per_solve > 0) {
      double duration = options_.max_time_per_solve;
      ob::PlannerTerminationCondition termination_cond = ob::timedPlannerTerminationCondition(duration, duration / 10);
      if (options_.max_iterations_per_solve > 0) {
        ob::PlannerTerminationCondition termination_cond2 = ob::IterationTerminationCondition(options_.max_iterations_per_solve);
        return ob::plannerOrTerminationCondition(termination_cond, termination_cond2);
      }
      return termination_cond;
    }
    else if (options_.max_iterations_per_solve > 0) {
      return ob::IterationTerminationCondition(options_.max_iterations_per_solve);
    }
    std::cout << "WARNING: Invalid planner termination conditions. Using default of 1000 iterations." << std::endl;
    return ob::IterationTerminationCondition(1000);
  }

  Options options_;

  bool initialized_;
  ViewpointPlannerData* data_;
  BoundingBoxType space_bbox_;
  Vector3 object_extent_;

  std::shared_ptr<StateSpaceType> space_;
  std::shared_ptr<ob::SpaceInformation> space_info_;
  std::shared_ptr<ob::ProblemDefinition> problem_def_;
  std::shared_ptr<PlannerType> planner_;
};
