#include "quad_planner/quad_planner.h"

#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

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

void QuadPlanner::run()
{
  octomap::point3d bbx_min(-0.1, -0.1, -1.2);
  octomap::point3d bbx_max(+0.1, +0.1, +0.1);
  for (auto it = octomap_ptr_->begin_leafs_bbx(bbx_min, bbx_max); it != octomap_ptr_->end_leafs_bbx(); ++it)
  {
    const octomap::point3d &coord = it.getCoordinate();
    octomap::OcTreeKey key = it.getKey();
    double size = it.getSize();
    double occupancy = it->getOccupancy();
    std::cout << "Leaf node at " << coord << " has occupancy " << occupancy << " [size=" << size << ", key=" << key << "]" << std::endl;
  }
  for (int i=-8; i < 8; ++i)
  {
    octomap::point3d point(0, 0, 0.05 + i * 0.1);
    auto node = octomap_ptr_->search(point, 0);
    std::cout << "node occupancy at " << point << ": " << node->getOccupancy() << std::endl;
    auto key = octomap_ptr_->coordToKey(point);
    std::cout << "node key: " << key << std::endl;
    auto it = octomap_ptr_->begin_leafs_bbx(key, key);
    std::cout << "node size: " << it.getSize() << " at coord: " << it.getCoordinate() << std::endl;
  }

  // construct the state space we are planning in
  auto space(boost::make_shared<StateSpaceT>());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-100);
  bounds.setHigh(+100);

  space->setBounds(bounds);

  // construct an instance of  space information from this state space
  auto si(boost::make_shared<ob::SpaceInformation>(space));

  // set state validity checking for this space
  si->setStateValidityChecker(boost::bind(&QuadPlanner::isStateValid, this, ::_1));

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
  auto pdef(boost::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

  // create a planner for the defined space
  planner_ = boost::make_shared<QuadPlanner::PlannerT>(si);

  // set the problem we are trying to solve for the planner
  planner_->setProblemDefinition(pdef);
  planner_->setRange(50 * octomap_ptr_->getResolution());

  // perform setup steps for the planner
  planner_->setup();


  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner_->solveWithMinNumOfValidSamples(10000);
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
}
