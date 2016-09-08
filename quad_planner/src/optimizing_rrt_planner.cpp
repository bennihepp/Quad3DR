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

#include <quad_planner/optimizing_rrt_planner.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

using namespace quad_planner;


OptimizingRRT::OptimizingRRT(const ob::SpaceInformationPtr &si) : ob::Planner(si, "RRT")
{
  specs_.approximateSolutions = true;
  specs_.directed = true;

  goalBias_ = 0.05;
  maxDistance_ = 0.0;
  lastGoalMotion_ = nullptr;

  Planner::declareParam<double>("range", this, &OptimizingRRT::setRange, &OptimizingRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &OptimizingRRT::setGoalBias, &OptimizingRRT::getGoalBias, "0.:.05:1.");
}

OptimizingRRT::~OptimizingRRT()
{
  freeMemory();
}

void OptimizingRRT::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_)
    nn_->clear();
  lastGoalMotion_ = nullptr;
}

void OptimizingRRT::setup()
{
  Planner::setup();
  ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!nn_)
    nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
  nn_->setDistanceFunction(std::bind(&OptimizingRRT::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void OptimizingRRT::freeMemory()
{
  if (nn_)
  {
    std::vector<Motion*> motions;
    nn_->list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

ob::PlannerTerminationCondition OptimizingRRT::getTimePlannerTerminationCondition(double solve_time) const
{
  if (solve_time < 1.0)
  {
    return ob::timedPlannerTerminationCondition(solve_time);
  }
  else
  {
    return ob::timedPlannerTerminationCondition(solve_time, std::min(solve_time / 100.0, 0.1));
  }
}

ob::PlannerStatus OptimizingRRT::solveWithMinNumOfSamples(int fixed_num_of_samples, double solve_time)
{
  ob::PlannerTerminationCondition num_of_samples_ptc([this, fixed_num_of_samples]() -> bool
  {
    return num_of_sampled_states_ > fixed_num_of_samples;
  });
  if (solve_time <= 0.0)
  {
    return solve(num_of_samples_ptc);
  }
  else
  {
    ob::PlannerTerminationCondition time_ptc = getTimePlannerTerminationCondition(solve_time);
    ob::PlannerTerminationCondition ptc = ob::plannerAndTerminationCondition(num_of_samples_ptc, time_ptc);
    return solve(ptc);
  }
}

ob::PlannerStatus OptimizingRRT::solveWithMinNumOfValidSamples(int fixed_num_of_valid_samples, double solve_time)
{
  ob::PlannerTerminationCondition num_of_samples_ptc([this, fixed_num_of_valid_samples]() -> bool
  {
    return num_of_valid_sampled_states_ > fixed_num_of_valid_samples;
  });
  if (solve_time <= 0.0)
  {
    return solve(num_of_samples_ptc);
  }
  else
  {
    ob::PlannerTerminationCondition time_ptc = getTimePlannerTerminationCondition(solve_time);
    ob::PlannerTerminationCondition ptc = ob::plannerAndTerminationCondition(num_of_samples_ptc, time_ptc);
    return solve(ptc);
  }
}

ob::PlannerStatus OptimizingRRT::solve(const ob::PlannerTerminationCondition &ptc)
{
  checkValidity();
  ob::Goal                 *goal   = pdef_->getGoal().get();
  ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

  while (const ob::State *st = pis_.nextStart())
  {
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
  }

  if (nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ob::PlannerStatus::INVALID_START;
  }

  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

  Motion *solution  = nullptr;
  Motion *approxsol = nullptr;
  double  approxdif = std::numeric_limits<double>::infinity();
  Motion *rmotion   = new Motion(si_);
  ob::State *rstate = rmotion->state;
  ob::State *xstate = si_->allocState();

  num_of_sampled_states_ = 0;
  num_of_valid_sampled_states_ = 0;

  while (ptc == false)
  {
    /* sample random state (with goal biasing) */
    if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
      goal_s->sampleGoal(rstate);
    else
      sampler_->sampleUniform(rstate);
    ++num_of_sampled_states_;

    /* find closest state in the tree */
    Motion *nmotion = nn_->nearest(rmotion);
    ob::State *dstate = rstate;

    /* find state to add */
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_)
    {
      si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
      dstate = xstate;
    }

    if (si_->checkMotion(nmotion->state, dstate))
    {
      ++num_of_valid_sampled_states_;
      /* create a motion */
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;

      nn_->add(motion);
      double dist = 0.0;
      bool sat = goal->isSatisfied(motion->state, &dist);
      if (sat)
      {
        approxdif = dist;
        solution = motion;
        break;
      }
      if (dist < approxdif)
      {
        approxdif = dist;
        approxsol = motion;
        std::cout << "Approximate distance to goal: " << approxdif << std::endl;
      }
      std::cout << "Sampled states: " << num_of_sampled_states_ << " of which " << num_of_valid_sampled_states_ << " are valid" << std::endl;
    }
  }

  bool solved = false;
  bool approximate = false;
  if (solution == nullptr)
  {
    solution = approxsol;
    approximate = true;
  }

  if (solution != nullptr)
  {
    lastGoalMotion_ = solution;

    /* construct the solution path */
    std::vector<Motion*> mpath;
    while (solution != nullptr)
    {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    /* set the solution path */
    og::PathGeometric *path = new og::PathGeometric(si_);
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
      path->append(mpath[i]->state);
    pdef_->addSolutionPath(ob::PathPtr(path), approximate, approxdif, getName());
    solved = true;
  }

  si_->freeState(xstate);
  if (rmotion->state)
    si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

  return ob::PlannerStatus(solved, approximate);
}

void OptimizingRRT::getPlannerData(ob::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (nn_)
    nn_->list(motions);

  if (lastGoalMotion_)
    data.addGoalVertex(ob::PlannerDataVertex(lastGoalMotion_->state));

  for (unsigned int i = 0 ; i < motions.size() ; ++i)
  {
    if (motions[i]->parent == nullptr)
      data.addStartVertex(ob::PlannerDataVertex(motions[i]->state));
    else
      data.addEdge(ob::PlannerDataVertex(motions[i]->parent->state),
                   ob::PlannerDataVertex(motions[i]->state));
  }
}
