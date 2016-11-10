//==================================================
// sparse_stereo.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 19, 2016
//==================================================

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#if OPENCV_3
//  #include <opencv2/xfeatures2d/cuda.hpp>
#else
  #include <opencv2/gpu/gpu.hpp>
  #include <opencv2/gpu/gpumat.hpp>
#endif
#include <ait/stereo/sparse_stereo_matcher.h>
#include <ait/stereo/dense_stereo_matcher.h>
#include <ait/utilities.h>

namespace ait
{
namespace stereo
{

#if OPENCV_2_4
  namespace cv_cuda = cv::gpu;
  using HostMemType = cv::gpu::CudaMem;
  const int HOST_MEM_ALLOC_TYPE = HostMemType::ALLOC_ZEROCOPY;
#else
  namespace cv_cuda = cv::cuda;
  using HostMemType = cv::cuda::HostMem;
  const HostMemType::AllocType HOST_MEM_ALLOC_TYPE = HostMemType::PAGE_LOCKED;
#endif

struct StereoAndDepthImageData
{
  HostMemType left_mem;
  HostMemType right_mem;
  HostMemType left_mem_color;
  HostMemType right_mem_color;
  HostMemType depth_mem;

  cv::Mat left_img;
  cv::Mat right_img;
  cv::Mat left_img_color;
  cv::Mat right_img_color;
  cv::Mat depth_img;
  std::vector<cv::Point3d> point_cloud_points;
  std::vector<cv::Point3d> point_cloud_colors;

  StereoAndDepthImageData(int rows, int cols, int type, int color_type, int depth_type)
  : left_mem(rows, cols, type, HOST_MEM_ALLOC_TYPE),
    right_mem(rows, cols, type, HOST_MEM_ALLOC_TYPE),
    left_mem_color(rows, cols, color_type, HOST_MEM_ALLOC_TYPE),
    right_mem_color(rows, cols, color_type, HOST_MEM_ALLOC_TYPE),
    depth_mem(rows, cols, depth_type, HOST_MEM_ALLOC_TYPE)
  {
    left_img = left_mem.createMatHeader();
    right_img = right_mem.createMatHeader();
    left_img_color = left_mem_color.createMatHeader();
    right_img_color = right_mem_color.createMatHeader();
    depth_img = depth_mem.createMatHeader();
  }
};

template <typename T>
std::vector<cv::Point3d> sparseStereoMatching(
    const cv::Ptr<T> &matcher_ptr,
    cv::InputArray left_input_img, cv::InputArray right_input_img,
    std::vector<cv::Point2d> *left_image_points=nullptr,
    std::vector<cv::Point2d> *right_image_points=nullptr)
{
  ait::Timer  timer;
  bool verbose = true;
  std::vector<cv::Point3d> points_3d = matcher_ptr->match(left_input_img, right_input_img, left_image_points, right_image_points, verbose);
  timer.stopAndPrintTiming("sparse stereo matching");
  return points_3d;
}

template <typename T>
void stereoMatchingTest(
    cv::Ptr<T> matcher_ptr,
    const StereoAndDepthImageData &image_data,
    const stereo::StereoCameraCalibration &calib,
    bool save_pointclouds)
{
  static ml::PointCloud<float> sparse_pc;
//  static ml::PointCloud<float> dense_pc;
  try
  {
    std::vector<cv::Point2d> left_image_points;
    std::vector<cv::Point2d> right_image_points;
    std::vector<cv::Point3d> points_3d = sparseStereoMatching(
        matcher_ptr, image_data.left_img, image_data.right_img,
        &left_image_points, &right_image_points);

//    // For debugging
//    stereo::DenseStereoMatcher dense_matcher(calib);
//    cv::Mat disp_img = dense_matcher.match(image_data.left_img, image_data.right_img);
//    cv::Mat p3d_img;
//    cv::reprojectImageTo3D(disp_img, p3d_img, calib.disparity_to_depth_map);
//    for (int i = 0; i < points_3d.size(); ++i)
//    {
//      double depth_sparse = points_3d[i].z;
//      int left_x = static_cast<int>(left_image_points[i].x);
//      int left_y = static_cast<int>(left_image_points[i].y);
//      int right_x = static_cast<int>(right_image_points[i].x);
//      int right_y = static_cast<int>(right_image_points[i].y);
//      double depth_zed = image_data.depth_img.at<float>(left_y, left_x);
//      double depth_dense = p3d_img.at<cv::Point3f>(left_y, left_x).z;
//      double disp_sparse = left_x - right_x;
//      double disp_dense = disp_img.at<uchar>(left_y, left_x);
//      double disp_diff = disp_sparse - disp_dense;
//      if (disp_dense == 0)
//      {
//        continue;
//      }
//      std::cout << "disp_diff=" << disp_diff << ", disp_sparse=" << disp_sparse << ", disp_dense=" << disp_dense << ", depth_sparse=" << depth_sparse << std::endl;
//      if (!std::isnan(depth_zed))
//      {
//        double depth_diff = depth_sparse - depth_zed;
//        std::cout << "zed_depth_diff=" << depth_diff << ", depth_sparse=" << depth_sparse << ", depth_zed=" << depth_zed << std::endl;
//      }
////          if (!std::isnan(depth_dense))
////          {
////            double depth_diff = depth_sparse - depth_dense;
////            std::cout << "dense_depth_diff=" << depth_diff << ", depth_sparse=" << depth_sparse << ", depth_dense=" << depth_dense << std::endl;
////          }
//
//      cv::Mat mat(1, 1, CV_32FC3);
//      mat.at<cv::Point3f>(0, 0) = cv::Point3f(left_image_points[i].x, left_image_points[i].y, disp_sparse);
//      cv::Mat mat_out;
//      cv::perspectiveTransform(mat, mat_out, calib.disparity_to_depth_map);
//      const cv::Point3f p3df = mat_out.at<cv::Point3f>(0, 0);
//      cv::Point3d p3d(p3df.x, p3df.y, p3df.z);
////          if (disp_sparse < 4)
////          {
////            std::cout << " +++ disp_sparse=" << disp_sparse << std::endl;
////            std::cout << points_3d[i] << std::endl;
////            std::cout << p3d << std::endl;
////          }
//    }
//    cv::Mat disp_hist = ait::Utilities::drawHistogram(disp_img, 50);
//    cv::imshow("disp_hist", disp_hist);
//    cv::Mat disp_cmap_img = ait::Utilities::drawImageWithColormap(disp_img);
//    cv::imshow("disp_img", disp_cmap_img);
//    cv::Mat depth_hist = ait::Utilities::drawHistogram(ait::Utilities::getDepthImageFrom3DPointImage(p3d_img), 50);
//    cv::imshow("depth_hist", depth_hist);
//    cv::Mat depth_cmap_vis = ait::Utilities::draw3DPointImage(p3d_img);
//    cv::imshow("depth_img", depth_cmap_vis);

    if (save_pointclouds)
    {
    // For debugging
//      sparse_pc.clear();
      for (int i = 0; i < points_3d.size(); ++i)
      {
        const cv::Point3d &p = points_3d[i];
        const cv::Point2d &left_point_2d = left_image_points[i];
        const cv::Point2d &right_point_2d = right_image_points[i];
        const uchar *cv_color = image_data.left_img_color.at<uchar[4]>(left_point_2d);
        ml::vec4f c(cv_color[0], cv_color[1], cv_color[2], 255);
        // Scale sparse points to millimeters
        float point_scale = 1.0f;
        sparse_pc.m_points.push_back(ml::vec3f(p.x * point_scale, p.y * point_scale, p.z * point_scale));
        float color_scale = 1 / 255.0f;
        sparse_pc.m_colors.push_back(ml::vec4f(c.z * color_scale, c.y * color_scale, c.x * color_scale, c.w * color_scale));
      }
      std::cout << "Saving sparse pointcloud with " << sparse_pc.m_points.size() << " points" << std::endl;
      ml::PointCloudIO<float>::saveToFile("sparse_points.ply", sparse_pc);
      ait::Utilities::savePointCloudToOff("sparse_points.off", sparse_pc);
    }

//        // For debugging (point cloud from ZED point cloud)
//        dense_pc.clear();
//        for (int i = 0; i < image_data.point_cloud_points.size(); ++i)
//        {
//          const cv::Point3d &p = image_data.point_cloud_points[i];
//          const cv::Point3d &c = image_data.point_cloud_colors[i];
//          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
//          {
//            continue;
//          }
//          float point_scale = 1.0f;
//          dense_pc.m_points.push_back(ml::vec3f(p.x * point_scale, p.y * point_scale, p.z * point_scale));
//          float color_scale = 1 / 255.0f;
//          dense_pc.m_colors.push_back(ml::vec4f(c.x * color_scale, c.y * color_scale, c.z * color_scale, 1.0f));
//        }

//    // For debugging (point cloud from StereoSGBM depth image)
//    dense_pc.clear();
//    for (int i = 0; i < p3d_img.rows; ++i)
//    {
//      for (int j = 0; j < p3d_img.cols; ++j)
//      {
//        CV_Assert(p3d_img.type() == CV_32FC3);
//        const cv::Point3f &p = p3d_img.at<cv::Point3f>(i, j);
//        const uchar *img_data = image_data.left_img_color.at<uchar[4]>(i, j);
//        float disp = disp_img.at<float>(i, j);
////            std::cout << "i=" << i << ", j=" << j << std::endl;
////            std::cout << "p=" << p << ", disp=" << disp << std::endl;
//        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
//        {
//          continue;
//        }
//        ml::vec4f c(img_data[0], img_data[1], img_data[2], img_data[3]);
//        // Scale sparse points to millimeters
//        float point_scale = 1.0f;
//        dense_pc.m_points.push_back(ml::vec3f(p.x * point_scale, p.y * point_scale, p.z * point_scale));
//        float color_scale = 1 / 255.0f;
//        dense_pc.m_colors.push_back(ml::vec4f(c.z * color_scale, c.y * color_scale, c.x * color_scale, c.w * color_scale));
//      }
//    }

    // For debugging
    // Draw keypoints on image and put depth values as text
    cv::Mat img_with_text = image_data.left_img.clone();
    img_with_text = ait::Utilities::drawPoints(img_with_text, left_image_points, 5);
    cv::cvtColor(img_with_text, img_with_text, CV_GRAY2RGBA);
    for (int i = 0; i < points_3d.size(); ++i)
    {
      double depth_sparse = points_3d[i].z;
      int x = left_image_points[i].x;
      int y = left_image_points[i].y;
      std::ostringstream out;
      out.precision(2);
      out << depth_sparse;
      double font_scale = 1.0;
      cv::Scalar font_color(255, 0, 0, 255);
      cv::putText(img_with_text, out.str(), cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, font_scale, font_color);
    }
    cv::imshow("keypoints with depth text", img_with_text);
  }
  catch (const typename T::Error &err)
  {
    std::cerr << "Exception during stereo matching: " << err.what() << std::endl;
  }

//  if (save_pointclouds)
//  {
//    ml::PointCloudIO<float>::saveToFile("dense_points.ply", dense_pc);
//    ait::Utilities::savePointCloudToOff("dense_points.off", dense_pc);
//  }
}

}  // namespace stereo
}  // namespace ait
