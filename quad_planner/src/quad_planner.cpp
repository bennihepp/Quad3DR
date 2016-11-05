//==================================================
// quad_planner.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#include "quad_planner/quad_planner.h"
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <memory>
#include <functional>

using namespace quad_planner;


static std::ostream& operator<<(std::ostream& stream, const octomap::OcTreeKey &key)
{
  stream << "[" << key[0] << ", " << key[1] << ", " << key[2] << "]";
  return stream;
}

QuadPlanner::QuadPlanner(double octomap_resolution)
{
  octomap_ptr_ = std::make_shared<octomap::OcTree>(octomap_resolution);
}

QuadPlanner::~QuadPlanner()
{
}

std::shared_ptr<const octomap::OcTree> QuadPlanner::getOctomap() const
{
  return octomap_ptr_;
}

void QuadPlanner::loadOctomapFile(const std::string &filename)
{
  bool success = octomap_ptr_->readBinary(filename);
  if (!success)
  {
    throw std::runtime_error("Unable to read octomap file");
  }
}

bool QuadPlanner::isCoordinateValid(double x, double y, double z) const
{
  if (z <= 0) {
    return false;
  }

  return true;
}

bool QuadPlanner::isStateValid(const ob::State *state) const
{
  // cast the abstract state type to the type we expect
  const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  double pos_x = se3state->getX();
  double pos_y = se3state->getY();
  double pos_z = se3state->getZ();
//  double pos_x = pos->values[0];
//  double pos_y = pos->values[1];
//  double pos_z = pos->values[2];

  //  std::cout << "checking state=(" << pos_x << ", " << pos_y << ", " << pos_z << ")" << std::endl;

  if (!isCoordinateValid(pos_x, pos_y, pos_z)) {
    return false;
  }

  int depth = 0;
  auto node = octomap_ptr_->search(pos_x, pos_y, pos_z, depth);

  if (node == nullptr)
  {
    return false;
  }

  double occupancy = node->getOccupancy();
  bool valid_state = occupancy <= OCCUPANCY_FREE_THRESHOLD;
  return valid_state;
}


QuadPlanner::MotionValidator::MotionValidator(const QuadPlanner* planner, ob::SpaceInformation* si)
: ob::MotionValidator(si), planner_(planner)
{
}

bool QuadPlanner::MotionValidator::checkMotion(const ob::State* state1, const ob::State* state2,
    std::pair<ob::State*, double> &lastValid) const
{
  bool result = checkMotion(state1, state2);
  if (result) {
    return true;
  }
  else {
    // TODO: Need to copy state?
    lastValid.first = si_->getStateSpace().get()->allocState();
    si_->getStateSpace()->copyState(lastValid.first, state1);
    lastValid.second = 0;
    return false;
  }
}

bool QuadPlanner::MotionValidator::checkMotion(const ob::State *state1, const ob::State *state2) const
{
  /* assume motion starts in a valid configuration so s1 is valid */
  if (!planner_->isStateValid(state2)) {
    invalid_++;
    return false;
  }

  const ob::SE3StateSpace::StateType *se3state1 = state1->as<ob::SE3StateSpace::StateType>();
  const ob::SE3StateSpace::StateType *se3state2 = state2->as<ob::SE3StateSpace::StateType>();

  octomap::point3d origin(se3state1->getX(), se3state1->getY(), se3state1->getZ());
  octomap::point3d end(se3state2->getX(), se3state2->getY(), se3state2->getZ());
  octomap::KeyRay ray;
  if (!planner_->octomap_ptr_->computeRayKeys(origin, end, ray)) {
    std::cerr << "WARNING: Start or stop state are out of Octree range." << std::endl;
    invalid_++;
    return false;
  }

  bool result = true;
  std::cout << "Checking " << ray.size() << " cells" << std::endl;
  for (octomap::KeyRay::const_iterator it = ray.begin(); it != ray.end(); ++it) {
    int depth = 0;
    auto node = planner_->octomap_ptr_->search(*it, depth);
    if (node == nullptr) {
      result = false;
      break;
    }

    double occupancy = node->getOccupancy();
    bool valid_state = occupancy <= planner_->OCCUPANCY_FREE_THRESHOLD;
    if (!valid_state) {
      std::cout << "Found occupied cell" << std::endl;
      result = false;
      break;
    }

    octomap::point3d coord = planner_->octomap_ptr_->keyToCoord(*it);
    if (!planner_->isCoordinateValid(coord.x(), coord.y(), coord.z())) {
      result = false;
      break;
    }
  }

  if (result) {
    valid_++;
  }
  else {
    invalid_++;
  }

  return result;
}

std::shared_ptr<ob::ProblemDefinition> QuadPlanner::run()
{
//  octomap::point3d bbx_min(-0.1, -0.1, -1.2);
//  octomap::point3d bbx_max(+0.1, +0.1, +0.1);
//  for (auto it = octomap_ptr_->begin_leafs_bbx(bbx_min, bbx_max); it != octomap_ptr_->end_leafs_bbx(); ++it)
//  {
//    const octomap::point3d &coord = it.getCoordinate();
//    octomap::OcTreeKey key = it.getKey();
//    double size = it.getSize();
//    double occupancy = it->getOccupancy();
//    std::cout << "Leaf node at " << coord << " has occupancy " << occupancy << " [size=" << size << ", key=" << key << "]" << std::endl;
//  }
//  for (int i=-8; i < 8; ++i)
//  {
//    octomap::point3d point(0, 0, 0.05 + i * 0.1);
//    auto node = octomap_ptr_->search(point, 0);
//    std::cout << "node occupancy at " << point << ": " << node->getOccupancy() << std::endl;
//    auto key = octomap_ptr_->coordToKey(point);
//    std::cout << "node key: " << key << std::endl;
//    auto it = octomap_ptr_->begin_leafs_bbx(key, key);
//    std::cout << "node size: " << it.getSize() << " at coord: " << it.getCoordinate() << std::endl;
//  }

  std::cout << "Bounding box min: " << octomap_ptr_->getBBXMin() << std::endl;
  std::cout << "Bounding box max: " << octomap_ptr_->getBBXMax() << std::endl;
  std::cout << "Bounding box center: " << octomap_ptr_->getBBXCenter() << std::endl;
  std::cout << "Bounding box bounds: " << octomap_ptr_->getBBXBounds() << std::endl;
  std::cout << "Volume: " << octomap_ptr_->volume() << std::endl;
  double xmin, ymin, zmin;
  double xmax, ymax, zmax;
  octomap_ptr_->getMetricMin(xmin, ymin, zmin);
  octomap_ptr_->getMetricMax(xmax, ymax, zmax);
  std::cout << "Metric max: (" << xmax << ", " << ymax << ", " << zmax << ")" << std::endl;
//  std::cout << "Searching for node occupancy..." << std::endl;
//  octomap::point3d point(1.5, ymin, 0.1);
//  bool occupied = false;
//  while (true) {
////    point.y() += validityCheckingResolution;
//    point.y() += validityCheckingResolution;
//    if (point.y() > ymax) {
//      std::cout << "Reached bounding box: " << point << std::endl;
//      break;
//    }
//    auto node = octomap_ptr_->search(point, 0);
//    if (!occupied && node->getOccupancy() > 0.2) {
//      occupied = true;
//      auto key = octomap_ptr_->coordToKey(point);
//      auto it = octomap_ptr_->begin_leafs_bbx(key, key);
//      std::cout << "switch to occupied at " << it.getCoordinate() << std::endl;
//    }
//    else if (occupied && node->getOccupancy() <= 0.2) {
//      occupied = false;
//      auto key = octomap_ptr_->coordToKey(point);
//      auto it = octomap_ptr_->begin_leafs_bbx(key, key);
//      std::cout << "switch to unoccupied at " << it.getCoordinate() << std::endl;
//    }
//  }

  // construct the state space we are planning in
  auto space(std::make_shared<StateSpaceT>());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-100);
  bounds.setHigh(+100);

  space->setBounds(bounds);

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(space));

  // set state validity checking for this space
  si->setStateValidityChecker(std::bind(&QuadPlanner::isStateValid, this, std::placeholders::_1));
  double validityCheckingResolution = 0.01 * octomap_ptr_->getResolution() / space->getMaximumExtent();
  si->setStateValidityCheckingResolution(validityCheckingResolution);

  MotionValidator motionValidator(this, si.get());
  ob::MotionValidatorPtr motionValidatorPtr(&motionValidator);
  // set motion validity checking
  si->setMotionValidator(motionValidatorPtr);

  // create a random start state
  ob::ScopedState<StateSpaceT> start(si);
//  start.random();
  start->setXYZ(0.0, 0.0, 0.15);
//  start->values[0] = 0.0;
//  start->values[1] = 0.0;
//  start->values[2] = 0.5;
  start->rotation().setIdentity();

  // create a random goal state
  ob::ScopedState<StateSpaceT> goal(si);
//  goal.random();
  goal->setXYZ(0.0, -10.0, 1.0);
//  goal->values[0] = 0.0;
//  goal->values[1] = 0.0;
//  goal->values[2] = 1.0;
  goal->rotation().setIdentity();

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);
  // set trajectory objective
  pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

  // create a planner for the defined space
  planner_ = std::make_shared<QuadPlanner::PlannerT>(si);

  // set the problem we are trying to solve for the planner
  planner_->setProblemDefinition(pdef);
  planner_->setRange(50 * octomap_ptr_->getResolution());

  // perform setup steps for the planner
  planner_->setup();


  ob::ScopedState<StateSpaceT> state1(si);
  state1->setXYZ(-4.73269, -0.200741, 1.3174);
  state1->rotation().setIdentity();
  ob::ScopedState<StateSpaceT> state2(si);
  state2->setXYZ(-6.88227, -0.0225858, 5.4814);
  state2->rotation().setIdentity();
  std::cout << "check: " << si->checkMotion(state1.get(), state2.get()) << std::endl;


  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner_->solveWithMinNumOfValidSamples(1000, 2.0);
//  ob::PlannerStatus solved = planner_->ob::Planner::solve(5.0);

  if (solved)
  {
    if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
      std::cout << "Found approximate solution:" << std::endl;
    }
    else
    {
      std::cout << "Found exact solution:" << std::endl;
      // get the goal representation from the problem definition (not the same as the goal state)
      // and inquire about the found path
      const ob::PathPtr path = pdef->getSolutionPath();
      const og::PathGeometric *geo_path = dynamic_cast<og::PathGeometric*>(path.get());

      // print the path to screen
      path->print(std::cout);

      std::ofstream out("octomap_trajectory.txt");
      geo_path->printAsMatrix(out);
      out.close();
    }
  }
  else
  {
    std::cout << "No solution found" << std::endl;
  }

  return pdef;
}
