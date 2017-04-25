//==================================================
// create_colmap_mvs_files.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 21.04.17
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <bh/boost.h>
#include <bh/filesystem.h>
#include <bh/config_options.h>
#include <bh/se3_transform.h>
#include <bh/vision/types.h>
#include <bh/vision/cameras.h>
#include <bh/vision/geometry.h>

#include "../planner/viewpoint_planner.h"
#include "../reconstruction/dense_reconstruction.h"

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

class CreateColmapMVSFiles {
public:
  struct Options : bh::ConfigOptions {
    using ImageId = int;
    using CameraId = int;

    using SE3Transform = bh::SE3Transform<FloatType>;
    using PoseType = bh::Pose<FloatType>;

    Options() {
      addOptionRequired<string>("images_path", &images_path);
      addOptionRequired<string>("viewpoint_path_file", &viewpoint_path_file);
      addOption<string>("output_path", &output_path);
      addOption<bool>("use_patch_match_auto", &use_patch_match_auto);
      addOption<size_t>("patch_match_neighbor_overlap", &patch_match_neighbor_overlap);
      addOption<string>("patch_match_config", &patch_match_auto_config);
    }

    ~Options() override {}

    string images_path;
    string viewpoint_path_file;
    string output_path = "colmap_mvs";
    bool use_patch_match_auto = false;
    size_t patch_match_neighbor_overlap = 2;
    string patch_match_auto_config = "__auto__, 5";
  };

  static std::unique_ptr<bh::ConfigOptions> getConfigOptions() {
    std::unique_ptr<bh::ConfigOptions> options;
    options.reset(new Options());
    return std::move(options);
  }

  CreateColmapMVSFiles(
          const std::unique_ptr<bh::ConfigOptions>& options)
      : options_(*dynamic_cast<Options*>(options.get())) {}

  ~CreateColmapMVSFiles() {
  }

  // TODO: Extract to sparate file
  std::vector<string> getSortedImageFilenames() const {
    std::vector<string> image_filenames;
    boostfs::path images_path(options_.images_path);
    try {
      if (!boostfs::exists(images_path)) {
        throw bh::Error("Path to images does not exist");
      }
      if (!boostfs::is_directory(images_path)) {
        throw bh::Error("Path to images is not a directory");
      }
      for (auto it = boostfs::directory_iterator(images_path); it != boostfs::directory_iterator(); ++it) {
        if (boostfs::is_regular_file(it->status())) {
          image_filenames.push_back(bh::pathRelativeTo(images_path, it->path()).string());
        }
      }
    }
    catch (const boostfs::filesystem_error& ex) {
      throw bh::Error(ex.what());
    }
    std::sort(std::begin(image_filenames), std::end(image_filenames),
              [&](const string& a, const string& b) {
                return std::lexicographical_compare(std::begin(a), std::end(a), std::begin(b), std::end(b));
              });
    return image_filenames;
  }

  ViewpointPlanner::ViewpointPath readViewpointPath() const {
    std::cout << "Loading viewpoint paths from " << options_.viewpoint_path_file << std::endl;
    std::ifstream ifs(options_.viewpoint_path_file);
    boost::archive::binary_iarchive ia(ifs);
    std::vector<ViewpointPlanner::ViewpointPath> tmp_viewpoint_paths;
    std::vector<ViewpointPlanner::ViewpointPathComputationData> tmp_viewpoint_paths_data;
    const reconstruction::PinholeCamera camera = reconstruction::PinholeCamera::createSimple(100, 100, FloatType(1.0));
    // TODO
    DummyViewpointPathLoader vpl(&tmp_viewpoint_paths, &tmp_viewpoint_paths_data, &camera);
    ia >> vpl;
//    std::unordered_map<ViewpointPlanner::ViewpointEntryIndex, ViewpointPlanner::Pose> sparse_matching_viewpoint_poses;
//    ia >> sparse_matching_viewpoint_poses;
//    std::unordered_map<ViewpointPlanner::ViewpointIndexPair, ViewpointPlanner::ViewpointMotion, ViewpointPlanner::ViewpointIndexPair::Hash> local_viewpoint_path_motions;
//    ia >> local_viewpoint_path_motions;
    return tmp_viewpoint_paths.front();
  }

  std::vector<size_t> getMVSNeighborIndices(const ViewpointPlanner::ViewpointPath& viewpoint_path,
                                            const size_t order_index,
                                            const size_t neighbor_overlap) const {
    std::vector<size_t> mvs_neighbor_indices;
    size_t i = 0;
    // Left neighbors
    size_t left_order_index = order_index;
    while (i < neighbor_overlap) {
      left_order_index -= 1;
      if (left_order_index < 0) {
        left_order_index += viewpoint_path.order.size();
      }
      if (left_order_index == order_index) {
        break;
      }
      const size_t left_entry_index = viewpoint_path.order[left_order_index];
      if (viewpoint_path.entries[left_entry_index].mvs_viewpoint) {
        mvs_neighbor_indices.push_back(left_order_index);
      }
    }
    size_t right_order_index = order_index;
    while (i < neighbor_overlap) {
      right_order_index -= 1;
      if (right_order_index < 0) {
        right_order_index += viewpoint_path.order.size();
      }
      if (right_order_index == order_index) {
        break;
      }
      const size_t right_entry_index = viewpoint_path.order[right_order_index];
      if (viewpoint_path.entries[right_entry_index].mvs_viewpoint) {
        mvs_neighbor_indices.push_back(right_order_index);
      }
    }
    return mvs_neighbor_indices;
  }

  bool run() {
    // Read image filenames
    std::vector<string> image_filenames = getSortedImageFilenames();

    const ViewpointPlanner::ViewpointPath viewpoint_path = readViewpointPath();

    BH_ASSERT(image_filenames.size() == viewpoint_path.order.size());

    std::vector<string> mvs_image_filenames;
    for (auto it = viewpoint_path.order.begin(); it != viewpoint_path.order.end(); ++it) {
      if (viewpoint_path.entries[*it].mvs_viewpoint) {
        mvs_image_filenames.push_back(image_filenames[it - viewpoint_path.order.begin()]);
      }
    }

    if (!boostfs::exists(options_.output_path)) {
      boostfs::create_directory(options_.output_path);
    }
    BH_ASSERT(boostfs::is_directory(options_.output_path));

    {
      std::ofstream ofs(bh::joinPaths(options_.output_path, "fusion.cfg"));
      for (size_t i = 0; i < image_filenames.size(); ++i) {
        const size_t entry_index = viewpoint_path.order[i];
        const bool mvs_viewpoint = viewpoint_path.entries[entry_index].mvs_viewpoint;
        if (mvs_viewpoint) {
          ofs << image_filenames[i] << std::endl;
        }
      }
      ofs.close();
    }

    {
      std::ofstream ofs(bh::joinPaths(options_.output_path, "patch-match.cfg"));
      for (size_t i = 0; i < image_filenames.size(); ++i) {
        const size_t entry_index = viewpoint_path.order[i];
        const bool mvs_viewpoint = viewpoint_path.entries[entry_index].mvs_viewpoint;
        if (mvs_viewpoint) {
          ofs << image_filenames[i] << std::endl;
          if (options_.use_patch_match_auto) {
            ofs << options_.patch_match_auto_config << std::endl;
          }
          else {
            const size_t order_index = i;
            const std::vector<size_t> mvs_neighbor_indices
                    = getMVSNeighborIndices(viewpoint_path, order_index, options_.patch_match_neighbor_overlap);
            for (size_t j = 0; j < mvs_neighbor_indices.size(); ++j) {
              if (j > 0) {
                ofs << ", ";
              }
              const size_t mvs_neighbor_index = mvs_neighbor_indices[j];
              const string neighbor_mvs_filename = mvs_image_filenames[mvs_neighbor_index];
              ofs << neighbor_mvs_filename;
            }
            ofs << std::endl;
          }
        }
      }
      ofs.close();
    }

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
  std::unique_ptr<bh::ConfigOptions> config_options = CreateColmapMVSFiles::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
          processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
    return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  CreateColmapMVSFiles create_colmap_mvs_files_cmdline(config_options);

  if (create_colmap_mvs_files_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
