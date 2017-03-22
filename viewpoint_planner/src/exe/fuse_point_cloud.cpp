//==================================================
// fuse_point_cloud.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 7, 2017
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <ait/boost.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <ait/common.h>
#include <ait/eigen.h>
#include <ait/utilities.h>
#include <ait/options.h>
#include <ait/math/geometry.h>

#include <ait/mLib.h>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = ait::BoundingBox3D<FloatType>;
using PointCloudType = ml::PointCloud<FloatType>;
using PointCloudIOType = ml::PointCloudIO<FloatType>;

class FusePointCloudCmdline {
public:
  FusePointCloudCmdline(
      const string& in_point_cloud_filename1,
      const string& in_point_cloud_filename2,
      const string& out_point_cloud_filename)
  : in_point_cloud_filename1_(in_point_cloud_filename1),
    in_point_cloud_filename2_(in_point_cloud_filename2),
    out_point_cloud_filename_(out_point_cloud_filename) {}

  ~FusePointCloudCmdline() {
  }

  void addPointCloud(const PointCloudType& src, PointCloudType* dest) {
    for (std::size_t i = 0; i < src.m_points.size(); ++i) {
      const ml::vec3<FloatType>& ml_point_coord = src.m_points[i];
      const Vector3 point_coord(ml_point_coord.x, ml_point_coord.y, ml_point_coord.z);
      dest->m_points.push_back(src.m_points[i]);
      if (src.hasColors()) {
        dest->m_colors.push_back(src.m_colors[i]);
      }
      if (src.hasNormals()) {
        dest->m_normals.push_back(src.m_normals[i]);
      }
      if (src.hasTexCoords()) {
        dest->m_texCoords.push_back(src.m_texCoords[i]);
      }
    }
  }

  bool run() {
    PointCloudType point_cloud1;
    PointCloudIOType::loadFromFile(in_point_cloud_filename1_, point_cloud1);
    cout << "Number of vertices in point cloud 1: " << point_cloud1.m_points.size() << endl;
    PointCloudType point_cloud2;
    PointCloudIOType::loadFromFile(in_point_cloud_filename2_, point_cloud2);
    cout << "Number of vertices in point cloud 2: " << point_cloud2.m_points.size() << endl;
    PointCloudType fused_point_cloud;
    addPointCloud(point_cloud1, &fused_point_cloud);
    addPointCloud(point_cloud2, &fused_point_cloud);
    cout << "Number of vertices in fused point cloud: " << fused_point_cloud.m_points.size() << endl;
    PointCloudIOType::saveToFile(out_point_cloud_filename_, fused_point_cloud);

    return true;
  }

private:
  string in_point_cloud_filename1_;
  string in_point_cloud_filename2_;
  string out_point_cloud_filename_;
};

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("in-point-cloud1", po::value<string>()->required(), "File to load the input point cloud 1 from.")
        ("in-point-cloud2", po::value<string>()->required(), "File to load the input point cloud 2 from.")
        ("out-point-cloud", po::value<string>()->required(), "File to save the fused point cloud to.")
        ;

    po::options_description options;
    options.add(generic_options);
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help")) {
      cout << options << endl;
      return std::make_pair(false, vm);
    }
    po::notify(vm);

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
  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  FusePointCloudCmdline fuse_cmdline(
      vm["in-point-cloud1"].as<string>(),
      vm["in-point-cloud2"].as<string>(),
      vm["out-point-cloud"].as<string>());

  if (fuse_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
