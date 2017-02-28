//==================================================
// viewpoint_planner_serialization.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 26, 2017
//==================================================

#pragma once

class VoxelMapSaver {
public:
  using FloatType = ViewpointPlanner::FloatType;
  using VoxelType = ViewpointPlanner::VoxelType;
  using VoxelMap = ViewpointPlanner::VoxelMap;

  VoxelMapSaver(const ViewpointPlannerData::OccupiedTreeType& bvh_tree) {
    // Compute consistent ordering of BVH nodes
    std::size_t voxel_index = 0;
    for (const ViewpointPlannerData::OccupiedTreeType::NodeType& node : bvh_tree) {
      voxel_index_map_.emplace(&node, voxel_index);
      ++voxel_index;
    }
  }

  template <typename Archive>
  void save(const VoxelMap& voxel_map, Archive& ar, const unsigned int version) const {
    ar & voxel_map.size();
    for (const auto& entry : voxel_map) {
      const VoxelType* voxel_ptr = entry.first.voxel;
      const FloatType information = entry.second;
      std::size_t voxel_index = voxel_index_map_.at(voxel_ptr);
      ar & voxel_index;
      ar & information;
    }
  }

private:
  std::unordered_map<const VoxelType*, std::size_t> voxel_index_map_;
};

class VoxelMapLoader {
public:
  using FloatType = ViewpointPlanner::FloatType;
  using VoxelType = ViewpointPlanner::VoxelType;
  using VoxelMap = ViewpointPlanner::VoxelMap;

  VoxelMapLoader(ViewpointPlannerData::OccupiedTreeType* bvh_tree) {
    // Compute consistent ordering of BVH nodes
    std::size_t voxel_index = 0;
    for (ViewpointPlannerData::OccupiedTreeType::NodeType& node : *bvh_tree) {
      index_voxel_map_.emplace(voxel_index, &node);
      ++voxel_index;
    }
  }

  template <typename Archive>
  void load(VoxelMap* voxel_map, Archive& ar, const unsigned int version) const {
    std::size_t num_voxels;
    ar & num_voxels;
    for (std::size_t i = 0; i < num_voxels; ++i) {
      std::size_t voxel_index;
      ar & voxel_index;
      VoxelType* voxel = index_voxel_map_.at(voxel_index);
      FloatType information;
      ar & information;
//        VoxelWithInformation voxel_with_information(voxel, information);
//        voxel_map->emplace(std::move(voxel_with_information));
      voxel_map->emplace(voxel, information);
    }
  }

private:
  std::unordered_map<std::size_t, VoxelType*> index_voxel_map_;
};

class VoxelWithInformationSetSaver {
public:
  using FloatType = ViewpointPlanner::FloatType;
  using VoxelType = ViewpointPlanner::VoxelType;
  using VoxelWithInformation = ViewpointPlanner::VoxelWithInformation;
  using VoxelWithInformationSet = ViewpointPlanner::VoxelWithInformationSet;

  VoxelWithInformationSetSaver(const ViewpointPlannerData::OccupiedTreeType& bvh_tree) {
    // Compute consistent ordering of BVH nodes
    std::size_t voxel_index = 0;
    for (const ViewpointPlannerData::OccupiedTreeType::NodeType& node : bvh_tree) {
      voxel_index_map_.emplace(&node, voxel_index);
      ++voxel_index;
    }
  }

  template <typename Archive>
  void save(const VoxelWithInformationSet& voxel_set, Archive& ar, const unsigned int version) const {
    ar & voxel_set.size();
    for (const VoxelWithInformation& voxel_with_information : voxel_set) {
      std::size_t voxel_index = voxel_index_map_.at(voxel_with_information.voxel);
      ar & voxel_index;
      ar & voxel_with_information.information;
    }
  }

private:
  std::unordered_map<const VoxelType*, std::size_t> voxel_index_map_;
};

class VoxelWithInformationSetLoader {
public:
  using FloatType = ViewpointPlanner::FloatType;
  using VoxelType = ViewpointPlanner::VoxelType;
  using VoxelWithInformation = ViewpointPlanner::VoxelWithInformation;
  using VoxelWithInformationSet = ViewpointPlanner::VoxelWithInformationSet;

  VoxelWithInformationSetLoader(ViewpointPlannerData::OccupiedTreeType* bvh_tree) {
    // Compute consistent ordering of BVH nodes
    std::size_t voxel_index = 0;
    for (ViewpointPlannerData::OccupiedTreeType::NodeType& node : *bvh_tree) {
      index_voxel_map_.emplace(voxel_index, &node);
      ++voxel_index;
    }
  }

  template <typename Archive>
  void load(VoxelWithInformationSet* voxel_set, Archive& ar, const unsigned int version) const {
    std::size_t num_voxels;
    ar & num_voxels;
    for (std::size_t i = 0; i < num_voxels; ++i) {
      std::size_t voxel_index;
      ar & voxel_index;
      VoxelType* voxel = index_voxel_map_.at(voxel_index);
      FloatType information;
      ar & information;
      VoxelWithInformation voxel_with_information(voxel, information);
      voxel_set->emplace(std::move(voxel_with_information));
    }
  }

private:
  std::unordered_map<std::size_t, VoxelType*> index_voxel_map_;
};

class ViewpointEntrySaver {
public:
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointEntryVector = ViewpointPlanner::ViewpointEntryVector;

  ViewpointEntrySaver(const ViewpointEntryVector& entries, const ViewpointPlannerData::OccupiedTreeType& bvh_tree)
  : entries_(entries), voxel_set_saver_(bvh_tree) {}

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) const {
    ar & entries_.size();
    for (const ViewpointEntry& entry : entries_) {
      // Save viewpoint
      ar & entry.viewpoint.pose();
      ar & entry.total_information;
      voxel_set_saver_.save(entry.voxel_set, ar, version);
    }
  }

  const ViewpointEntryVector& entries_;
  const VoxelWithInformationSetSaver voxel_set_saver_;
};

class ViewpointEntryLoader {
public:
  using Pose = ViewpointPlanner::Pose;
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointEntryVector = ViewpointPlanner::ViewpointEntryVector;

  ViewpointEntryLoader(ViewpointEntryVector* entries, ViewpointPlannerData::OccupiedTreeType* bvh_tree, const PinholeCamera* camera)
  : entries_(entries), voxel_set_loader_(bvh_tree), camera_(camera) {}

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    std::size_t num_entries;
    ar & num_entries;
    entries_->resize(num_entries);
    for (ViewpointEntry& entry : *entries_) {
      Pose pose;
      ar & pose;
      entry.viewpoint = Viewpoint(camera_, pose);
      ar & entry.total_information;
      voxel_set_loader_.load(&entry.voxel_set, ar, version);
    }
  }

  ViewpointEntryVector* entries_;
  const VoxelWithInformationSetLoader voxel_set_loader_;
  const PinholeCamera* camera_;
};

class ViewpointPathEntriesSaver {
public:
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;

  ViewpointPathEntriesSaver(const std::vector<ViewpointPathEntry>& path_entries)
  : path_entries_(path_entries) {}

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & path_entries_.size();
    for (const ViewpointPathEntry& path_entry : path_entries_) {
      ar & path_entry;
      if (version > 1) {
        ar & path_entry.viewpoint.pose();
      }
    }
  }

private:
  const std::vector<ViewpointPathEntry>& path_entries_;
};

class ViewpointPathSaver {
public:
  using ViewpointPath = ViewpointPlanner::ViewpointPath;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;
  using ViewpointPathComputationData = ViewpointPlanner::ViewpointPathComputationData;

  ViewpointPathSaver(const std::vector<ViewpointPath>& paths,
      const std::vector<ViewpointPathComputationData>& comp_datas,
      const ViewpointPlannerData::OccupiedTreeType& bvh_tree)
  : paths_(paths), comp_datas_(comp_datas), voxel_map_saver_(bvh_tree) {}

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) const {
    ar & paths_.size();
    for (std::size_t i = 0; i < paths_.size(); ++i) {
      const ViewpointPath& path = paths_[i];
      const ViewpointPathComputationData& comp_data = comp_datas_[i];
      if (boost::serialization::version<ViewpointPathEntry>::value > 1) {
        ar & path.entries;
      }
      else {
        ViewpointPathEntriesSaver vpes(path.entries);
        vpes.serialize(ar, boost::serialization::version<ViewpointPathEntry>::value);
      }
      ar & path.order;
      ar & path.acc_information;
      ar & path.acc_motion_distance;
      ar & path.acc_objective;
      voxel_map_saver_.save(path.observed_voxel_map, ar, version);
      //        voxel_set_saver_.save(path.observed_voxel_set, ar, version);
      ar & comp_data.sorted_new_informations;
    }
  }

  const std::vector<ViewpointPath>& paths_;
  const std::vector<ViewpointPathComputationData>& comp_datas_;
  const VoxelMapSaver voxel_map_saver_;
};

class ViewpointPathEntriesLoader {
public:
  using Pose = ViewpointPlanner::Pose;
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;

  ViewpointPathEntriesLoader(std::vector<ViewpointPathEntry>* path_entries,
                             const std::vector<ViewpointEntry>& viewpoint_entries,
                             const PinholeCamera* camera)
  : path_entries_(path_entries), viewpoint_entries_(viewpoint_entries), camera_(camera) {}

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    std::size_t num_of_entries;
    ar & num_of_entries;
    for (std::size_t i = 0; i < num_of_entries; ++i) {
      ViewpointPathEntry path_entry;
      ar & path_entry;
      if (version > 1) {
        Pose pose;
        ar & pose;
        path_entry.viewpoint = Viewpoint(camera_, pose);
      }
      else {
        AIT_ASSERT(path_entry.viewpoint_index < viewpoint_entries_.size());
        path_entry.viewpoint = viewpoint_entries_[path_entry.viewpoint_index].viewpoint;
      }
      path_entries_->push_back(path_entry);
    }
  }

private:
  std::vector<ViewpointPathEntry>* path_entries_;
  const std::vector<ViewpointEntry>& viewpoint_entries_;
  const PinholeCamera* camera_;
};

class ViewpointPathLoader {
public:
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;
  using ViewpointPath = ViewpointPlanner::ViewpointPath;
  using ViewpointPathComputationData = ViewpointPlanner::ViewpointPathComputationData;

  ViewpointPathLoader(std::vector<ViewpointPath>* paths,
      std::vector<ViewpointPathComputationData>* comp_datas,
      ViewpointPlannerData::OccupiedTreeType* bvh_tree,
      const std::vector<ViewpointEntry>& viewpoint_entries,
      const PinholeCamera* camera)
  : paths_(paths), comp_datas_(comp_datas), voxel_map_loader_(bvh_tree),
    viewpoint_entries_(viewpoint_entries), camera_(camera ){}

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    std::size_t num_paths;
    ar & num_paths;
    paths_->resize(num_paths);
    comp_datas_->resize(num_paths);
    for (std::size_t i = 0; i < paths_->size(); ++i) {
      ViewpointPath& path = (*paths_)[i];
      ViewpointPathComputationData& comp_data = (*comp_datas_)[i];
      if (boost::serialization::version<ViewpointPathEntry>::value > 1) {
        ar & path.entries;
      }
      else {
        ViewpointPathEntriesLoader vpel(&path.entries, viewpoint_entries_, camera_);
        vpel.serialize(ar, boost::serialization::version<ViewpointPathEntry>::value);
      }
      ar & path.order;
      ar & path.acc_information;
      ar & path.acc_motion_distance;
      ar & path.acc_objective;
      voxel_map_loader_.load(&path.observed_voxel_map, ar, version);
//        voxel_set_loader_.load(&path.observed_voxel_set, ar, version);
      ar & comp_data.sorted_new_informations;
    }
  }

private:
  std::vector<ViewpointPath>* paths_;
  std::vector<ViewpointPathComputationData>* comp_datas_;
  VoxelMapLoader voxel_map_loader_;
  const std::vector<ViewpointEntry>& viewpoint_entries_;
  const PinholeCamera* camera_;
};
