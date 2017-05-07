//==================================================
// merge_viewpoint_paths.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 03.05.17
//==================================================
//==================================================
// create_colmap_mvs_files.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 21.04.17
//==================================================

#include <iostream>
#include <memory>
#include <tuple>
#include <csignal>

#include <bh/boost.h>
#include <bh/filesystem.h>
#include <bh/string_utils.h>
#include <bh/config_options.h>
#include <bh/se3_transform.h>
#include <bh/vision/types.h>
#include <bh/vision/cameras.h>
#include <bh/vision/geometry.h>

#include "../planner/viewpoint_planner.h"
#include "../reconstruction/dense_reconstruction.h"

#pragma GCC optimize("O0")

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace boostfs = boost::filesystem;

using FloatType = float;
BH_USE_FIXED_EIGEN_TYPES(FloatType);

using BoundingBoxType = bh::BoundingBox3D<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using PointCloudIOType = ml::PointCloudIO<FloatType>;

class DummyVoxelMapLoader {
public:
  using FloatType = ViewpointPlanner::FloatType;
  using VoxelType = ViewpointPlanner::VoxelType;
  using VoxelMap = ViewpointPlanner::VoxelMap;

  DummyVoxelMapLoader() {}

  template <typename Archive>
  void load(Archive& ar, const unsigned int version) const {
    std::size_t num_voxels;
    ar & num_voxels;
    for (std::size_t i = 0; i < num_voxels; ++i) {
      std::size_t voxel_index;
      ar & voxel_index;
      FloatType information;
      ar & information;
    }
  }
};

class DummyVoxelMapSaver {
public:
  using FloatType = ViewpointPlanner::FloatType;
  using VoxelType = ViewpointPlanner::VoxelType;
  using VoxelMap = ViewpointPlanner::VoxelMap;

  DummyVoxelMapSaver() {}

  template <typename Archive>
  void save(const VoxelMap& voxel_map, Archive& ar, const unsigned int version) const {
    const size_t voxel_map_size = 0;
    ar & voxel_map_size;
  }
};

class DummyViewpointPathEntriesLoader {
public:
  using Pose = ViewpointPlanner::Pose;
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;

  DummyViewpointPathEntriesLoader(std::vector<ViewpointPathEntry>* path_entries,
                                  const PinholeCamera* camera)
          : path_entries_(path_entries), camera_(camera) {}

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    std::size_t num_of_entries;
    ar & num_of_entries;
    for (std::size_t i = 0; i < num_of_entries; ++i) {
      ViewpointPathEntry path_entry;
      ar & path_entry;
      BH_PRINT_VALUE(path_entry.viewpoint_index);
      if (version > 1) {
        Pose pose;
        ar & pose;
        BH_PRINT_VALUE(pose);
        path_entry.viewpoint = Viewpoint(camera_, pose);
      }
      else {
        std::cout << "WARNING: Loading viewpoint path entries from version <= 1" << std::endl;
      }
      path_entries_->push_back(path_entry);
    }
  }

private:
  std::vector<ViewpointPathEntry>* path_entries_;
  const PinholeCamera* camera_;
};

class DummyViewpointPathEntriesSaver {
public:
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;

  DummyViewpointPathEntriesSaver(const std::vector<ViewpointPathEntry>& path_entries)
      : path_entries_(path_entries) {}

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & path_entries_.size();
    for (const ViewpointPathEntry& path_entry : path_entries_) {
      ar & path_entry;
      BH_PRINT_VALUE(path_entry.viewpoint_index);
      if (version > 1) {
        ar & path_entry.viewpoint.pose();
        BH_PRINT_VALUE(path_entry.viewpoint.pose());
      }
    }
  }

private:
  const std::vector<ViewpointPathEntry>& path_entries_;
};

class DummyViewpointPathLoader {
public:
  using ViewpointEntry = ViewpointPlanner::ViewpointEntry;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;
  using ViewpointPath = ViewpointPlanner::ViewpointPath;
  using ViewpointPathComputationData = ViewpointPlanner::ViewpointPathComputationData;

  DummyViewpointPathLoader(std::vector<ViewpointPath>* paths,
                           std::vector<ViewpointPathComputationData>* comp_datas,
                           const reconstruction::PinholeCamera* camera)
          : paths_(paths), comp_datas_(comp_datas), camera_(camera ){}

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
        DummyViewpointPathEntriesLoader vpel(&path.entries, camera_);
        vpel.serialize(ar, boost::serialization::version<ViewpointPathEntry>::value);
      }
      else {
        ar & path.entries;
      }
      ar & path.order;
      ar & path.acc_information;
      ar & path.acc_motion_distance;
      ar & path.acc_objective;
      DummyVoxelMapLoader voxel_map_loader;
      voxel_map_loader.load(ar, version);
//        voxel_set_loader_.load(&path.observed_voxel_set, ar, version);
      ar & comp_data.sorted_new_informations;
    }
  }

private:
  std::vector<ViewpointPath>* paths_;
  std::vector<ViewpointPathComputationData>* comp_datas_;
  const PinholeCamera* camera_;
};

class DummyViewpointPathSaver {
public:
  using ViewpointPath = ViewpointPlanner::ViewpointPath;
  using ViewpointPathEntry = ViewpointPlanner::ViewpointPathEntry;
  using ViewpointPathComputationData = ViewpointPlanner::ViewpointPathComputationData;

  DummyViewpointPathSaver(const std::vector<ViewpointPath>& paths,
                          const std::vector<ViewpointPathComputationData>& comp_datas)
          : paths_(paths), comp_datas_(comp_datas) {}

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
        DummyViewpointPathEntriesSaver vpes(path.entries);
        vpes.serialize(ar, boost::serialization::version<ViewpointPathEntry>::value);
      }
      else {
        ar & path.entries;
      }
      ar & path.order;
      ar & path.acc_information;
      ar & path.acc_motion_distance;
      ar & path.acc_objective;
      DummyVoxelMapSaver voxel_map_saver;
      voxel_map_saver.save(path.observed_voxel_map, ar, version);
      //        voxel_set_saver_.save(path.observed_voxel_set, ar, version);
      ar & comp_data.sorted_new_informations;
    }
  }

  const std::vector<ViewpointPath>& paths_;
  const std::vector<ViewpointPathComputationData>& comp_datas_;
};

class MergeViewpointPaths {
public:
  using SE3Transform = bh::SE3Transform<FloatType>;
  using PoseType = bh::Pose<FloatType>;

  struct Options : bh::ConfigOptions {
    Options() {
      addOptionRequired<string>("viewpoint_path1", &viewpoint_path1);
      addOptionRequired<string>("viewpoint_path2", &viewpoint_path2);
      addOptionRequired<string>("out_viewpoint_path", &out_viewpoint_path);
    }

    ~Options() override {}

    string viewpoint_path1;
    string viewpoint_path2;
    string out_viewpoint_path;
  };

  static std::unique_ptr<bh::ConfigOptions> getConfigOptions() {
    std::unique_ptr<bh::ConfigOptions> options;
    options.reset(new Options());
    return std::move(options);
  }

  MergeViewpointPaths(
          const std::unique_ptr<bh::ConfigOptions>& options)
          : options_(*dynamic_cast<Options*>(options.get())) {}

  ~MergeViewpointPaths() {}

  ViewpointPlanner::ViewpointPath loadViewpointPath(const string& filename) const {
    std::ifstream ifs(filename);
    BH_ASSERT(ifs);
    boost::archive::binary_iarchive ia(ifs);
    std::vector<ViewpointPlanner::ViewpointPath> tmp_viewpoint_paths;
    std::vector<ViewpointPlanner::ViewpointPathComputationData> tmp_viewpoint_paths_data;
    const reconstruction::PinholeCamera camera = reconstruction::PinholeCamera::createSimple(100, 100, FloatType(1.0));

    DummyViewpointPathLoader vpl(&tmp_viewpoint_paths, &tmp_viewpoint_paths_data, &camera);
    ia >> vpl;
//    std::unordered_map<ViewpointPlanner::ViewpointEntryIndex, ViewpointPlanner::Pose> sparse_matching_viewpoint_poses;
//    ia >> sparse_matching_viewpoint_poses;
//    std::unordered_map<ViewpointPlanner::ViewpointIndexPair, ViewpointPlanner::ViewpointMotion, ViewpointPlanner::ViewpointIndexPair::Hash> local_viewpoint_path_motions;
//    ia >> local_viewpoint_path_motions;
    return tmp_viewpoint_paths.front();
  }

  void saveViewpointPath(const string& filename, const ViewpointPlanner::ViewpointPath& viewpoint_path) const {
    std::ofstream ofs(filename);
    BH_ASSERT(ofs);
    boost::archive::binary_oarchive oa(ofs);
    std::vector<ViewpointPlanner::ViewpointPath> tmp_viewpoint_paths;
    std::vector<ViewpointPlanner::ViewpointPathComputationData> tmp_viewpoint_paths_data;
    tmp_viewpoint_paths.push_back(viewpoint_path);
    tmp_viewpoint_paths_data.push_back(ViewpointPlanner::ViewpointPathComputationData());

    DummyViewpointPathSaver vps(tmp_viewpoint_paths, tmp_viewpoint_paths_data);
    oa << vps;

    std::unordered_map<ViewpointPlanner::ViewpointEntryIndex, PoseType> sparse_matching_viewpoint_poses;
    oa << sparse_matching_viewpoint_poses;

    std::unordered_map<ViewpointPlanner::ViewpointIndexPair, ViewpointPlanner::ViewpointMotion, ViewpointPlanner::ViewpointIndexPair::Hash> local_viewpoint_path_motions;
    oa << local_viewpoint_path_motions;
  }

  ViewpointPlanner::ViewpointPath mergeViewpointPaths(
          const ViewpointPlanner::ViewpointPath& viewpoint_path1,
          const ViewpointPlanner::ViewpointPath& viewpoint_path2) {
    ViewpointPlanner::ViewpointPath merged_viewpoint_path;
    for (const ViewpointPlanner::ViewpointPathEntry& entry : viewpoint_path1.entries) {
      merged_viewpoint_path.entries.push_back(entry);
    }
    for (const ViewpointPlanner::ViewpointPathEntry& entry : viewpoint_path2.entries) {
      merged_viewpoint_path.entries.push_back(entry);
    }
    for (const size_t index : viewpoint_path1.order) {
      const size_t merged_index = index;
      merged_viewpoint_path.order.push_back(merged_index);
    }
    for (const size_t index : viewpoint_path1.order) {
      const size_t merged_index = index + viewpoint_path1.entries.size();
      merged_viewpoint_path.order.push_back(merged_index);
    }
    merged_viewpoint_path.acc_information = viewpoint_path1.acc_information + viewpoint_path2.acc_information;
    merged_viewpoint_path.acc_motion_distance = viewpoint_path1.acc_motion_distance + viewpoint_path2.acc_motion_distance;
    merged_viewpoint_path.acc_objective = viewpoint_path1.acc_objective + viewpoint_path2.acc_objective;
    return merged_viewpoint_path;
  }

  bool run() {
    std::cout << "Loading viewpoint path 1 from " << options_.viewpoint_path1 << std::endl;
    const ViewpointPlanner::ViewpointPath viewpoint_path1 = loadViewpointPath(options_.viewpoint_path1);
    std::cout << "Loading viewpoint path 2 from " << options_.viewpoint_path2 << std::endl;
    const ViewpointPlanner::ViewpointPath viewpoint_path2 = loadViewpointPath(options_.viewpoint_path2);

    const ViewpointPlanner::ViewpointPath merged_viewpoint_path = mergeViewpointPaths(viewpoint_path1, viewpoint_path2);
    std::cout << "Saving merged viewpoint path to " << options_.out_viewpoint_path << std::endl;
    saveViewpointPath(options_.out_viewpoint_path, merged_viewpoint_path);

    return true;
  }

private:
  Options options_;
};

std::pair<bool, boost::program_options::variables_map> processOptions(
        int argc, char** argv, std::unique_ptr<bh::ConfigOptions>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
            ("help", "Produce help message")
            ("config-file", po::value<string>()->default_value("create_colmap_mvs_files.cfg"), "Config file.")
            ;

    po::options_description options;
    options.add(generic_options);
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help")) {
      cout << options << endl;
      return std::make_pair(false, vm);
    }
    po::notify(vm);

    po::options_description config_file_options;
    config_file_options.add(config_options->getBoostOptions());
    std::ifstream config_in(vm["config-file"].as<string>());
    if (!config_in) {
      throw BH_EXCEPTION("Unable to open config file");
    }
    else {
      po::store(parse_config_file(config_in, config_file_options), vm);
      notify(vm);
    }

    config_options->setVariablesMap(vm);

    return std::make_pair(true, vm);
  }
  catch (const po::required_option& err) {
    cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << endl;
    return std::make_pair(false, vm);
  }
  catch (const po::error& err) {
    cerr << "Error parsing command line: " << err.what() << endl;
    return std::make_pair(false, vm);
  }
}

bool ctrl_c_pressed = false;
void signalIntHandler(int sig) {
  ctrl_c_pressed = true;
}

void enableCtrlCHandler(void (*signalHandler)(int)) {
  std::signal(SIGINT, signalHandler);
}

void disableCtrlCHandler() {
  std::signal(SIGINT, SIG_DFL);
}

int main(int argc, char** argv)
{
  std::unique_ptr<bh::ConfigOptions> config_options = MergeViewpointPaths::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
          processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
    return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  MergeViewpointPaths merge_viewpoint_paths(config_options);

  if (merge_viewpoint_paths.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
