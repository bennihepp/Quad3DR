// Adapted by Benjamin Hepp from OMPL [http://ompl.kavrakilab.org/].
//
// Modified work Copyright (c) 2016 Benjamin Hepp.
//
// Original work Copyright (c) 2008, Willow Garage, Inc.
// Original Copyright notice:
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#pragma once

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace quad_planner
{

// Local namespace abbreviations
namespace ob = ::ompl::base;
namespace og = ::ompl::geometric;

/** \brief Rapidly-exploring Random Trees */
class OptimizingRRT : public ob::Planner
{
public:
  /** \brief Constructor */
  OptimizingRRT(const ob::SpaceInformationPtr &si);

  virtual ~OptimizingRRT();

  virtual void getPlannerData(ob::PlannerData &data) const;

  ob::PlannerTerminationCondition getTimePlannerTerminationCondition(double solve_time) const;
  virtual ob::PlannerStatus solveWithMinNumOfSamples(int max_num_of_sampled_states, double solve_time=0.0);
  virtual ob::PlannerStatus solveWithMinNumOfValidSamples(int max_num_of_sampled_states, double solve_time=0.0);
  virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);

  virtual void clear();

  /** \brief Set the goal bias

      In the process of randomly selecting states in
      the state space to attempt to go towards, the
      algorithm may in fact choose the actual goal state, if
      it knows it, with some probability. This probability
      is a real number between 0.0 and 1.0; its value should
      usually be around 0.05 and should not be too large. It
      is probably a good idea to use the default value. */
  void setGoalBias(double goalBias)
  {
    goalBias_ = goalBias;
  }

  /** \brief Get the goal bias the planner is using */
  double getGoalBias() const
  {
    return goalBias_;
  }

  /** \brief Set the range the planner is supposed to use.

      This parameter greatly influences the runtime of the
      algorithm. It represents the maximum length of a
      motion to be added in the tree of motions. */
  void setRange(double distance)
  {
    maxDistance_ = distance;
  }

  /** \brief Get the range the planner is using */
  double getRange() const
  {
    return maxDistance_;
  }

  /** \brief Set a different nearest neighbors datastructure */
  template<template<typename T> class NN>
  void setNearestNeighbors()
  {
    nn_.reset(new NN<Motion*>());
  }

  virtual void setup();

protected:
  /** \brief Representation of a motion

      This only contains pointers to parent motions as we
      only need to go backwards in the tree. */
  class Motion
  {
  public:

    Motion() : state(nullptr), parent(nullptr)
    {
    }

    /** \brief Constructor that allocates memory for the state */
    Motion(const ob::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr)
    {
    }

    ~Motion()
    {
    }

    /** \brief The state contained by the motion */
    ob::State       *state;

    /** \brief The parent motion in the exploration tree */
    Motion            *parent;

    /** \brief The cost up to this motion */
    ob::Cost cost;

    /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
     * computations in the updateChildCosts() method) */
    ob::Cost incCost;

  };

  /** \brief Free the memory allocated by this planner */
  void freeMemory();

  /** \brief Compute distance between motions (actually distance between contained states) */
  double distanceFunction(const Motion *a, const Motion *b) const
  {
    return si_->distance(a->state, b->state);
  }

  /** \brief State sampler */
  ob::StateSamplerPtr                          sampler_;

  /** \brief A nearest-neighbors datastructure containing the tree of motions */
  std::shared_ptr<ompl::NearestNeighbors<Motion*>> nn_;

  /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
  double                                         goalBias_;

  /** \brief The maximum length of a motion to be added to a tree */
  double                                         maxDistance_;

  /** \brief The random number generator */
  ompl::RNG                                            rng_;

  /** \brief Objective we're optimizing */
  ob::OptimizationObjectivePtr opt_;

  /** \brief Stores the start states as Motions. */
  std::vector<Motion *> startMotions_;

  /** \brief Best cost found so far by algorithm */
  ob::Cost bestCost_;

  /** \brief The most recent goal motion.  Used for PlannerData computation */
  Motion                                         *lastGoalMotion_;

  /** \brief State sample counter for termination condition */
  int num_of_sampled_states_;
  int num_of_valid_sampled_states_;
};

}
