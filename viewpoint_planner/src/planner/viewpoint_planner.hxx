//==================================================
// viewpoint_planner.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 5, 2017
//==================================================

template <typename IteratorT>
std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPose(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
  IteratorT it = first + index;
//  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const Pose& pose = it->viewpoint.pose();
  return sampleSurroundingPose(pose);
}

template <typename IteratorT>
std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPoseFromEntries(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
  IteratorT it = first + index;
//  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const ViewpointEntryIndex viewpoint_index = *it;
  const Pose& pose = viewpoint_entries_[viewpoint_index].viewpoint.pose();
  return sampleSurroundingPose(pose);
}
