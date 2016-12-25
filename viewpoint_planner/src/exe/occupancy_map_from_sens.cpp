//==================================================
// depth_octomap.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 4, 2016
//==================================================

#include <iostream>

#include <boost/program_options.hpp>

#include <octomap/octomap.h>
#include "../occupancy_map.h"

#include <ait/mLib.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using std::cout;
using std::endl;
using std::string;
namespace oct = octomap;

std::pair<bool, boost::program_options::variables_map> process_commandline(int argc, char** argv)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Allowed options");
    generic_options.add_options()
      ("help", "Produce help message")
      ("sens-file", po::value<string>()->required(), "Sens-file to integrate into Octomap")
      ("num-frames", po::value<size_t>(), "Number of frames to extract")
      ;

    po::options_description octomap_options("Octomap options");
    octomap_options.add_options()
      ("resolution", po::value<double>()->default_value(0.05), "Octomap resolution")
      ("max-range", po::value<double>()->default_value(std::numeric_limits<double>().infinity()), "Max integration range")
      ("map-file", po::value<string>()->default_value("output_map.bt"), "Octomap output file")
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
  catch (const po::required_option& err)
  {
    std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
    return std::make_pair(false, vm);
  }
  catch (const po::error& err)
  {
    std::cerr << "Error parsing command line: " << err.what() << std::endl;
    return std::make_pair(false, vm);
  }
}

int main(int argc, char** argv)
{
  using OccupancyMapType = OccupancyMap<OccupancyNode>;

  namespace po = boost::program_options;

  // Handle command line
  std::pair<bool, boost::program_options::variables_map> cmdline_result = process_commandline(argc, argv);
  if (!cmdline_result.first) {
    return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  cout << "Loading sensor data" << endl;
  ml::SensorData sensor_data;
  sensor_data.loadFromFile(vm["sens-file"].as<string>());

  OccupancyMapType tree (vm["resolution"].as<double>());


  // insert some measurements of occupied cells

//  for (int x=-20; x<20; x++) {
//    for (int y=-20; y<20; y++) {
//      for (int z=-20; z<20; z++) {
//        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
//        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
//      }
//    }
//  }
//
//  // insert some measurements of free cells
//
//  for (int x=-30; x<30; x++) {
//    for (int y=-30; y<30; y++) {
//      for (int z=-30; z<30; z++) {
//        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
//        tree.updateNode(endpoint, false);  // integrate 'free' measurement
//      }
//    }
//  }

  double max_range = vm["max-range"].as<double>();
  ml::mat4f inv_depth_intrinsics = sensor_data.m_calibrationDepth.m_intrinsic.getInverse();
  cout << "depth_intrinsics=" << sensor_data.m_calibrationDepth.m_intrinsic << endl;
  cout << "inv_depth_intrinsics=" << inv_depth_intrinsics << endl;
  float* depth_data = new float[sensor_data.m_depthWidth * sensor_data.m_depthHeight];
  size_t num_frames_to_extract = sensor_data.m_frames.size();
  if (vm.count("num-frames") > 0) {
    num_frames_to_extract = std::min(num_frames_to_extract, vm["num-frames"].as<size_t>());
  }
  std::cout << "Total number of frames to integrate: " << num_frames_to_extract << std::endl;
  for (size_t i = 0; i < num_frames_to_extract; ++i) {
    cout << "Integrating frame " << (i+1) << " of " << sensor_data.m_frames.size() << endl;
    const ml::SensorData::RGBDFrame& frame = sensor_data.m_frames[i];
    ml::vec3f sensor_translation = frame.getCameraToWorld().getTranslation();
    unsigned short* depth_data_uint16 = sensor_data.decompressDepthAlloc(frame);
    for (size_t i = 0; i < sensor_data.m_depthWidth * sensor_data.m_depthHeight; ++i) {
      depth_data[i] = depth_data_uint16[i] / (float)sensor_data.m_depthShift;
    }
    std::free(depth_data_uint16);
    // Show depth maps for debugging
    {
      cv::Mat depth_img(sensor_data.m_depthHeight, sensor_data.m_depthWidth, CV_32F, depth_data);
      cv::Mat depth_img2;
      depth_img.copyTo(depth_img2);
      double min, max;
      cv::minMaxIdx(depth_img, &min, &max);
      cout << "min=" << min << ", max=" << max << endl;
      cv::normalize(depth_img2, depth_img2, 0, 1, CV_MINMAX);
      cv::imshow("depth", depth_img2);
      cv::waitKey(100);
    }
    oct::point3d sensor_origin(sensor_translation.x, sensor_translation.y, sensor_translation.z);
    //          std::cout << "sensor_position=" << sensor_translation << std::endl;
    oct::Pointcloud pc;
    for (size_t y = 0; y < sensor_data.m_depthHeight; ++y) {
      for (size_t x = 0; x < sensor_data.m_depthWidth; ++x) {
        float depth = depth_data[x + sensor_data.m_depthWidth * y];
        if (depth <= 0 || !std::isfinite(depth) || depth > max_range) {
          continue;
        }
        //                  cout << "x=" << x << ", y=" << y << ", depth=" << depth << endl;
        ml::vec4f p4d = inv_depth_intrinsics * ml::vec4f(x, y, 1, 1);
        //                  cout << "camera p4d=" << p4d << endl;
        ml::vec3f p3d = depth * p4d.getVec3();
        //                  cout << "camera p3d=" << p3d << endl;
        p3d = frame.getCameraToWorld() * p3d;
        //                  cout << "world p3d=" << p3d << endl;
        oct::point3d p(p3d.x, p3d.y, p3d.z);
        pc.push_back(p);
      }
    }
    tree.insertPointCloud(pc, sensor_origin, max_range, vm["lazy-eval"].as<bool>());
  }
  delete[] depth_data;

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
