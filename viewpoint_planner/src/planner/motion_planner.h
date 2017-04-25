/*
 * motion_planner.h
 *
 *  Created on: Dec 30, 2016
 *      Author: bhepp
 */
#pragma once

#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <boost/serialization/access.hpp>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <bh/eigen.h>
#include <bh/math/utilities.h>
#include <bh/config_options.h>
#include "viewpoint_planner_data.h"

namespace ob = ::ompl::base;
namespace og = ::ompl::geometric;

template <typename FloatT>
class MotionPlanner {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using BoundingBoxType = ViewpointPlannerData::BoundingBoxType;
  using Pose = bh::Pose<FloatType>;

  using StateSpaceType = ob::SE3StateSpace;
//  using StateSpaceType = ob::RealVectorStateSpace;
//  using PlannerType = og::RRTConnect;
  using PlannerType = og::RRTstar;
//  using PlannerType = og::PRMstar;

  struct Options : bh::ConfigOptions {
    Options()
    : bh::ConfigOptions("motion_planner", "MotionPlanner options") {
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

  struct Motion {
    using PoseVector = EIGEN_ALIGNED_VECTOR(Pose);

    Motion()
    : distance(0), cost(0) {}

    bool isValid() const {
      return !poses.empty();
    }

    void append(const Motion& other) {
      distance += other.distance;
      cost += other.cost;
      std::copy(other.poses.begin(), other.poses.end(), std::back_inserter(poses));
    }

    FloatType computeDistance() const {
      if (!isValid()) {
        return std::numeric_limits<FloatType>::quiet_NaN();
      }
      FloatType accumulated_distance = 0;
      for (auto it = poses.begin() + 1; it != poses.end(); ++it) {
        auto prev_it = it - 1;
        const FloatType local_distance = prev_it->getPositionDistanceTo(*it);
        accumulated_distance += local_distance;
      }
      return accumulated_distance;
    }

    FloatType computeSE3Distance() const {
      if (!isValid()) {
        return std::numeric_limits<FloatType>::quiet_NaN();
      }
      FloatType accumulated_distance = 0;
      for (auto it = poses.begin() + 1; it != poses.end(); ++it) {
        auto prev_it = it - 1;
        const FloatType local_distance = prev_it->getDistanceTo(*it);
        accumulated_distance += local_distance;
      }
      return accumulated_distance;
    }

    FloatType distance;
    FloatType cost;
    FloatType se3_distance;
    PoseVector poses;

private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & distance;
      ar & cost;
      if (version >= 1) {
        ar & se3_distance;
      }
      ar & poses;
      if (version < 1) {
        se3_distance = computeSE3Distance();
      }
    }
  };

  MotionPlanner(const Options* options, ViewpointPlannerData* data)
  : options_(*options), data_(data), initialized_(false), num_planner_data_in_use_(0) {}

  MotionPlanner(const Options* options, ViewpointPlannerData* data, const std::string& log_filename)
  : options_(*options), data_(data), initialized_(false), num_planner_data_in_use_(0) {
    ompl_output_handler_ = std::make_shared<ompl::msg::OutputHandlerFile>(log_filename.c_str());
    ompl::msg::useOutputHandler(ompl_output_handler_.get());
  }

  void setSpaceBoundingBox(const BoundingBoxType& space_bbox) {
    space_bbox_ = space_bbox;
  }

  void setObjectBoundingBox(const BoundingBoxType& object_bbox) {
    object_bbox_ = object_bbox;
  }

  void initialize(const std::size_t planner_data_pool_size = std::thread::hardware_concurrency()) const {
    std::lock_guard<std::mutex> lock(pool_mutex_);

    if (initialized_) {
      return;
    }

    BH_ASSERT(!object_bbox_.isEmpty());
    BH_ASSERT(!space_bbox_.isEmpty());

    initialized_ = true;

    for (std::size_t i = 0; i < planner_data_pool_size; ++i) {
      PlannerData planner_data = createPlannerData();
      planner_data_pool_.push_back(std::move(planner_data));
    }
  }

  class SE3DistanceOptimizationObjective : public ob::OptimizationObjective {
  public:
    SE3DistanceOptimizationObjective(const ob::SpaceInformationPtr& space_info)
    : ob::OptimizationObjective(space_info) {
      re3_space_ = std::static_pointer_cast<ob::RealVectorStateSpace>(
          std::static_pointer_cast<ob::CompoundStateSpace>(si_->getStateSpace())->getSubspace(0));
    }

    ob::Cost stateCost(const ob::State* s) const {
      return identityCost();
    }

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override {
      const ob::CompoundStateSpace::StateType* c_s1 = static_cast<const ob::CompoundStateSpace::StateType*>(s1);
      const ob::CompoundStateSpace::StateType* c_s2 = static_cast<const ob::CompoundStateSpace::StateType*>(s2);
      const ob::RealVectorStateSpace::StateType* re3_s1 = c_s1->as<ob::RealVectorStateSpace::StateType>(0);
      const ob::RealVectorStateSpace::StateType* re3_s2 = c_s2->as<ob::RealVectorStateSpace::StateType>(0);
      return ob::Cost(re3_space_->distance(re3_s1, re3_s2));
    }

    ob::Cost motionCostHeuristic(const ob::State* s1, const ob::State* s2) const override {
      return motionCost(s1, s2);
    }

  private:
    std::shared_ptr<const ob::RealVectorStateSpace> re3_space_;
  };

  std::pair<Motion, bool> findMotion(const Pose& from, const Pose& to) const {
    if (!initialized_) {
      initialize();
    }
//    bh::Timer timer;

    LockedPlannerData planner_data = requestPlannerData();

    // Create start and goal state
    ob::ScopedState<StateSpaceType> start = createStateFromPose(from, planner_data->space_info);
    ob::ScopedState<StateSpaceType> goal = createStateFromPose(to, planner_data->space_info);

    if (!isStateValid(start.get())) {
      std::cerr << "ERROR: start state not valid: from=" << from << std::endl;
    }
    if (!isStateValid(goal.get())) {
      std::cerr << "ERROR: goal state not valid: to=" << to << std::endl;
    }
    BH_ASSERT(isStateValid(start.get()));
    BH_ASSERT(isStateValid(goal.get()));

    planner_data->problem_def->clearSolutionNonExistenceProof();
    planner_data->problem_def->clearSolutionPaths();
    planner_data->problem_def->clearStartStates();
    planner_data->problem_def->clearGoal();
    planner_data->problem_def->setStartAndGoalStates(start, goal);

    planner_data->planner->clear();
//    planner_data->planner->clearQuery();
    planner_data->planner->setProblemDefinition(planner_data->problem_def);
    planner_data->planner->setup();

    ob::PlannerTerminationCondition termination_cond = getTerminationCondition();

    ob::PlannerStatus solved = planner_data->planner->solve(termination_cond);

//    timer.printTimingMs("MotionPlanner::findMotion()");

    if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
//      std::cout << "Found exact solution:" << std::endl;
      const ob::PathPtr path = planner_data->problem_def->getSolutionPath();;
      const og::PathGeometric* geo_path = dynamic_cast<og::PathGeometric*>(path.get());
      FloatType motion_distance = geo_path->cost(planner_data->problem_def->getOptimizationObjective()).value();

      // print the path to screen
//      path->print(std::cout);

//      ob::PlannerSolution solution(path);
//      planner_data->problem_def->getSolution(solution);
//      std::cout << "Length solution path: " << planner_data->problem_def->getSolutionPath()->length() << std::endl;
//      std::cout << "Solution path length: " << geo_path->length() << std::endl;
//      std::cout << "Solution difference: " << solution.difference_ << std::endl;
//      std::cout << "Solution cost: " << solution.cost_ << std::endl;
//      std::cout << "Start: " << start->getX() << " " << start->getY() << " " << start->getZ() << std::endl;
//      std::cout << "Goal: " << goal->getX() << " " << goal->getY() << " " << goal->getZ() << std::endl;

      // Sanity check
      FloatType distance_square = (from.getWorldPosition() - to.getWorldPosition()).squaredNorm();
      FloatType distance_ompl = planner_data->problem_def->getOptimizationObjective()->motionCost(start.get(), goal.get()).value();
      if (bh::isApproxSmaller(distance_ompl * distance_ompl, distance_square, FloatType(1e-2))) {
        std::cout << "WARNING: distance_ompl * distance_ompl < distance_square" << std::endl;
        BH_PRINT_VALUE(distance_square);
        BH_PRINT_VALUE(distance_ompl * distance_ompl);
        BH_PRINT_VALUE(distance_ompl);
        BH_DEBUG_BREAK;
      }
      if (bh::isApproxSmaller(motion_distance * motion_distance, distance_square, FloatType(1e-2))) {
        std::cout << "WARNING: motion_distance * motion_distance < distance_square" << std::endl;
        BH_PRINT_VALUE(motion_distance);
        BH_PRINT_VALUE(motion_distance * motion_distance);
        BH_PRINT_VALUE(distance_square);
        BH_DEBUG_BREAK;
      }

      Motion motion;
      motion.distance = motion_distance;
      motion.cost = motion_distance;
      FloatType accumulated_motion_distance = 0;
      FloatType accumulated_motion_distance2 = 0;
      for (std::size_t i = 0; i < geo_path->getStateCount(); ++i) {
        const StateSpaceType::StateType* state = static_cast<const StateSpaceType::StateType*>(geo_path->getState(i));
        Vector3 position(state->getX(), state->getY(), state->getZ());
        Quaternion quat;
        if (i > 0 && motion.distance > 0) {
          const StateSpaceType::StateType* prev_state = static_cast<const StateSpaceType::StateType*>(geo_path->getState(i - 1));
          accumulated_motion_distance += planner_data->problem_def->getOptimizationObjective()->motionCost(prev_state, state).value();
          accumulated_motion_distance2 += (motion.poses.back().getWorldPosition() - position).norm();
          const FloatType fraction = accumulated_motion_distance / motion.distance;
          if (i < geo_path->getStateCount() - 1) {
            quat = from.quaternion().slerp(fraction, to.quaternion());
          }
          else {
            quat = to.quaternion();
          }
        }
        else if (i == geo_path->getStateCount() - 1) {
          quat = to.quaternion();
        }
        else {
          quat = from.quaternion();
        }
        Pose pose = Pose::createFromImageToWorldTransformation(position, quat);
        BH_ASSERT(pose.quaternion().coeffs().array().isFinite().all());
        motion.poses.push_back(pose);
      }
      motion.distance = motion.computeDistance();
      motion.cost = motion.distance;
      motion.se3_distance = motion.computeSE3Distance();;
      BH_ASSERT(bh::isApproxEqual(accumulated_motion_distance, motion_distance, FloatType(1e-2)));
      BH_ASSERT(bh::isApproxEqual(accumulated_motion_distance2, motion_distance, FloatType(1e-2)));
      BH_ASSERT(motion.poses.front() == from);
      BH_ASSERT(motion.poses.back() == to);
      return std::make_pair(motion, true);
    }
    else {
//      std::cout << "No solution found" << std::endl;
      return std::make_pair(Motion(), false);
    }
  }

private:
  bool isStateValid(const ob::State *state) const {
    const StateSpaceType::StateType* state_tmp = static_cast<const StateSpaceType::StateType*>(state);
    Vector3 position(state_tmp->getX(), state_tmp->getY(), state_tmp->getZ());
    return data_->isValidObjectPosition(position, object_bbox_);
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
    BH_ASSERT(options_.max_time_per_solve > 0 || options_.max_iterations_per_solve > 0);
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

  struct PlannerData {
    bool in_use;
    std::shared_ptr<StateSpaceType> space;
    std::shared_ptr<ob::SpaceInformation> space_info;
    std::shared_ptr<ob::ProblemDefinition> problem_def;
    std::shared_ptr<PlannerType> planner;
  };

  class LockedPlannerData {
  public:
    static LockedPlannerData request(const MotionPlanner* motion_planner) {
      std::unique_lock<std::mutex> lock(motion_planner->pool_mutex_);

      while (motion_planner->num_planner_data_in_use_ >= motion_planner->planner_data_pool_.size()) {
        motion_planner->pool_condition_.wait(lock);
      }

      PlannerData* planner_data_ptr = nullptr;
      for (PlannerData& planner_data : motion_planner->planner_data_pool_) {
        if (!planner_data.in_use) {
          planner_data_ptr = &planner_data;
          break;
        }
      }
      BH_ASSERT_STR(planner_data_ptr != nullptr, "Unable to find a free planner data object");
      planner_data_ptr->in_use = true;
      ++motion_planner->num_planner_data_in_use_;
      return LockedPlannerData(planner_data_ptr, motion_planner);
    }

    PlannerData* operator->() {
      return planner_data_;
    }

    PlannerData& operator*() {
      return *planner_data_;
    }

    ~LockedPlannerData() {
      if (planner_data_ != nullptr) {
        std::lock_guard<std::mutex> lock(motion_planner_->pool_mutex_);
        planner_data_->in_use = false;
        --motion_planner_->num_planner_data_in_use_;
        motion_planner_->pool_condition_.notify_one();
      }
    }

  private:
    friend class MotionPlanner;

    LockedPlannerData(PlannerData* planner_data, const MotionPlanner* motion_planner)
    : planner_data_(planner_data), motion_planner_(motion_planner) {
    }

    LockedPlannerData(const LockedPlannerData& other) = delete;

    LockedPlannerData(LockedPlannerData&& other) {
      planner_data_ = other.planner_data_;
      motion_planner_ = other.motion_planner_;
      other.planner_data_ = nullptr;
      other.motion_planner_ = nullptr;
    }

    PlannerData* planner_data_;
    const MotionPlanner* motion_planner_;
  };

  LockedPlannerData requestPlannerData() const {
    return LockedPlannerData::request(this);
  }

  PlannerData createPlannerData() const {
    PlannerData planner_data;
    planner_data.in_use = false;
    planner_data.space = std::make_shared<StateSpaceType>();
    ob::RealVectorBounds bounds(3);
    for (std::size_t i = 0; i < 3; ++i) {
      bounds.setLow(i, space_bbox_.getMinimum(i));
      bounds.setHigh(i, space_bbox_.getMaximum(i));
    }
//    std::cout << "Bounding box=" << space_bbox_ << std::endl;
    planner_data.space->setBounds(bounds);
    FloatType fraction = object_bbox_.getMaxExtent() / (2 * space_bbox_.getMaxExtent());
    planner_data.space->setLongestValidSegmentFraction(fraction);

    planner_data.space_info = std::make_shared<ob::SpaceInformation>(planner_data.space);
    planner_data.space_info->setStateValidityChecker(std::bind(&MotionPlanner::isStateValid, this, std::placeholders::_1));

    planner_data.problem_def = std::make_shared<ob::ProblemDefinition>(planner_data.space_info);
    planner_data.problem_def->setOptimizationObjective(std::make_shared<SE3DistanceOptimizationObjective>(planner_data.space_info));

    planner_data.planner = std::make_shared<PlannerType>(planner_data.space_info);
    planner_data.planner->setRange(options_.max_motion_range);
    planner_data.planner->setGoalBias(0.1);

    planner_data.planner->setup();

    return planner_data;
  }

  Options options_;

  BoundingBoxType space_bbox_;
  BoundingBoxType object_bbox_;

  std::shared_ptr<ompl::msg::OutputHandler> ompl_output_handler_;

  ViewpointPlannerData* data_;

  mutable bool initialized_;
  mutable std::mutex pool_mutex_;
  mutable std::condition_variable pool_condition_;
  mutable std::size_t num_planner_data_in_use_;
  mutable std::vector<PlannerData> planner_data_pool_;
};

BOOST_CLASS_VERSION(typename MotionPlanner<double>::Motion, 1);
BOOST_CLASS_VERSION(typename MotionPlanner<float>::Motion, 1);
