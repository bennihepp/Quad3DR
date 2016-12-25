/*
 * occupancy_map_from_colmap.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: bhepp
 */

#include <iostream>

#include <boost/program_options.hpp>

#include <octomap/octomap.h>

#include <ait/vision_utilities.h>
#include "../occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using std::cout;
using std::endl;
using std::string;
namespace oct = octomap;
using FloatType = double;

std::pair<bool, boost::program_options::variables_map> process_commandline(int argc, char** argv) {
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Allowed options");
    generic_options.add_options()
      ("help", "Produce help message")
      ("mvs-workspace", po::value<string>()->required(), "Colmap MVS workspace path")
      ("num-frames", po::value<size_t>(), "Number of frames to extract")
      ;

    po::options_description octomap_options("Octomap options");
    octomap_options.add_options()
      ("resolution", po::value<FloatType>()->default_value(0.1), "Octomap resolution")
      ("max-range", po::value<FloatType>()->default_value(std::numeric_limits<FloatType>().max()), "Max integration range")
      ("map-file", po::value<string>()->default_value("output_map.ot"), "Octomap output file")
      ("lazy-eval", po::bool_switch()->default_value(true), "Only update inner nodes once at the end")
      ("dense", po::bool_switch()->default_value(false), "Make a dense tree by inserting unknown nodes")
      ;

    po::options_description options;
    options.add(generic_options);
    options.add(octomap_options);
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help")) {
      std::cout << options << std::endl;
      return std::make_pair(false, vm);
    }

    po::notify(vm);

    return std::make_pair(true, vm);
  }
  catch (const po::required_option& err) {
    std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
    return std::make_pair(false, vm);
  }
  catch (const po::error& err) {
    std::cerr << "Error parsing command line: " << err.what() << std::endl;
    return std::make_pair(false, vm);
  }
}

int main(int argc, char** argv) {
  using OccupancyMapType = OccupancyMap<OccupancyNode>;

  namespace po = boost::program_options;

  // Handle command line
  std::pair<bool, boost::program_options::variables_map> cmdline_result = process_commandline(argc, argv);
  if (!cmdline_result.first) {
    return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  cout << "Loading dense reconstruction" << endl;
  DenseReconstruction reconstruction;
  reconstruction.read(vm["mvs-workspace"].as<string>());

  OccupancyMapType tree (vm["resolution"].as<FloatType>());

  FloatType max_range = vm["max-range"].as<FloatType>();

  std::size_t num_frames_to_extract = reconstruction.getImages().size();
  if (vm.count("num-frames") > 0) {
    num_frames_to_extract = std::min(num_frames_to_extract, vm["num-frames"].as<std::size_t>());
  }
  std::cout << "Total number of frames to integrate: " << num_frames_to_extract << std::endl;
  auto it = reconstruction.getImages().cbegin();
  for (std::size_t i = 0; i < num_frames_to_extract; ++i, ++it) {
    const ImageId image_id = it->first;
    const ImageColmap& image = it->second;
    const PinholeCameraColmap& camera = reconstruction.getCameras().at(image.camera_id());

    cout << "Integrating frame " << (i+1) << " of " << reconstruction.getImages().size() << endl;
    DenseReconstruction::DepthMap depth_map = reconstruction.readDepthMap(image_id, DenseReconstruction::DenseMapType::GEOMETRIC_FUSED);

    // Show depth maps for debugging
    {
      cv::Mat depth_img(depth_map.height(), depth_map.width(), CV_32F);
      for (std::size_t y = 0; y < depth_map.height(); ++y) {
        for (std::size_t x = 0; x < depth_map.width(); ++x) {
          FloatType depth = depth_map(y, x);
          depth_img.at<float>(y, x) = depth;
        }
      }
      double min, max;
      cv::minMaxIdx(depth_img, &min, &max);
      cout << "min=" << min << ", max=" << max << endl;
      cv::normalize(depth_img, depth_img, 0, 1, CV_MINMAX);
      cv::imshow("depth", depth_img);
      cv::waitKey(100);
    }

    const CameraMatrix& intrinsics = camera.intrinsics();
    FloatType depth_camera_scale = depth_map.width() / (FloatType)camera.height();
    const CameraMatrix depth_intrinsics = ait::getScaledIntrinsics(intrinsics, depth_camera_scale);
    const CameraMatrix inv_depth_intrinsics = depth_intrinsics.inverse();
    cout << "depth_intrinsics=" << depth_intrinsics << endl;
    cout << "inv_depth_intrinsics=" << inv_depth_intrinsics << endl;
    const ait::Pose::Matrix3x4 transform_image_to_world = image.pose().getTransformationImageToWorld();

    const ait::Pose::Vector3 sensor_pos = image.pose().getWorldPosition();
    oct::point3d sensor_origin(sensor_pos(0), sensor_pos(1), sensor_pos(2));
//    std::cout << "sensor_position=" << sensor_pos.transpose() << std::endl;

    oct::Pointcloud pc;
    for (size_t y = 0; y < depth_map.height(); ++y) {
      for (size_t x = 0; x < depth_map.width(); ++x) {
        DenseReconstruction::DepthMap::ValueType depth = depth_map(y, x);
        if (depth <= 0 || !std::isfinite(depth) || depth > max_range) {
          continue;
        }
//        cout << "x=" << x << ", y=" << y << ", depth=" << depth << endl;
        Eigen::Vector4d p4d = inv_depth_intrinsics * Eigen::Vector4d(x, y, 1, 1);
        Eigen::Vector3d p3d = depth * p4d.topRows(3);
        p3d = transform_image_to_world * p3d.homogeneous();
        oct::point3d p(p3d(0), p3d(1), p3d(2));
        pc.push_back(p);
      }
    }
    tree.insertPointCloud(pc, sensor_origin, max_range, vm["lazy-eval"].as<bool>());
  }

  if (vm["dense"].as<bool>()) {
    std::cout << "Octree has " << tree.getNumLeafNodes() << " leaf nodes and " << tree.size() << " total nodes" << std::endl;
    std::cout << "Filling unknown nodes" << std::endl;
    for (auto it = tree.begin_tree(); it != tree.end_tree(); ++it) {
      if (!it.isLeaf()) {
        for (size_t i = 0; i < 8; ++i) {
          if (!tree.nodeChildExists(&(*it), i)) {
            tree.createNodeChild(&(*it), i);
            it->setOccupancy(0.5f);
            it->setObservationCount(0);
          }
        }
      }
    }
  }

  if (vm["lazy-eval"].as<bool>()) {
    std::cout << "Updating inner nodes" << std::endl;
    tree.updateInnerOccupancy();
  }

  std::cout << "Octree has " << tree.getNumLeafNodes() << " leaf nodes and " << tree.size() << " total nodes" << std::endl;
  std::cout << "Metric extents:" << std::endl;
  double x, y, z;
  tree.getMetricSize(x, y, z);
  std::cout << "  size=(" << x << ", " << y << ", " << z << ")" << std::endl;
  tree.getMetricMin(x, y, z);
  std::cout << "   min=(" << x << ", " << y << ", " << z << ")" << std::endl;
  tree.getMetricMax(x, y, z);
  std::cout << "   max=(" << x << ", " << y << ", " << z << ")" << std::endl;


  tree.write(vm["map-file"].as<string>());
//    tree.writeBinary(vm["map-file"].as<string>() + ".bt");
}
