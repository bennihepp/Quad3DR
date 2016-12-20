//==================================================
// cv_utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================
#pragma once

#include <opencv2/opencv.hpp>
#include <ait/stereo/stereo_calibration.h>

#if OPENCV_2_4
  namespace cv_cuda = cv::gpu;
#else
  namespace cv_cuda = cv::cuda;
#endif

namespace ait
{

class CVUtilities
{
public:
    CVUtilities() = delete;

  static cv::Mat convertToGrayscale(cv::InputArray img);

  static void convertToGrayscaleGpu(const cv_cuda::GpuMat &img, cv_cuda::GpuMat *img_grayscale_ptr, cv_cuda::Stream &stream);

  static cv::Mat drawImageWithColormap(cv::InputArray depth_img, cv::ColormapTypes cmap=cv::COLORMAP_HOT, bool show_range=true);

  static cv::Mat getDepthImageFrom3DPointImage(cv::InputArray p3d_img, double cutoff_threshold=1e4);

  static cv::Mat draw3DPointImage(cv::InputArray p3d_img, cv::ColormapTypes cmap=cv::COLORMAP_HOT);

  static cv::Mat drawHistogram(cv::InputArray img, int bins=100, bool show_range=true,  int width=1280, int height=720);

  static cv::Mat drawKeypoints(cv::InputArray img, const std::vector<cv::KeyPoint> &keypoints);

  static cv::Mat drawFeatureMatches(cv::InputArray left_img, const std::vector<cv::KeyPoint> &left_keypoints, cv::InputArray right_img, const std::vector<cv::KeyPoint> &right_keypoints, const std::vector<cv::DMatch> &matches);

  template <typename T>
  static cv::Mat drawPoints(cv::InputArray img, const std::vector<cv::Point_<T>> &points, int radius=4)
  {
    cv::Mat img_with_points = img.getMat().clone();
    for (int i = 0; i < points.size(); ++i)
    {
      double f = 255.0 * (double)(i) / (points.size() - 1);
      cv::Scalar color(255.0 - f, 0.5, 0.0 + f, 255.0);
      cv::circle(img_with_points, points[i], radius, color);
    }
    return img_with_points;
  }

  template <typename T, typename U>
  static cv::Mat drawPointMatches(
      cv::InputArray left_img,
      const std::vector<cv::Point_<T>> &left_points,
      cv::InputArray right_img,
      const std::vector<cv::Point_<U>> &right_points,
      int radius=4)
  {
    CV_Assert(left_img.size() == right_img.size());
    CV_Assert(left_points.size() == right_points.size());
    cv::Size image_size = left_img.size();
    cv::Mat img_with_matches;
    cv::hconcat(left_img, right_img, img_with_matches);
    for (int i = 0; i < left_points.size(); ++i)
    {
      double f = 255.0 * (double)(i) / (left_points.size() - 1);
      cv::Scalar color(255.0 - f, 0.5, 0.0 + f, 255.0);
      cv::Point_<T> left_point = left_points[i];
      cv::Point_<U> right_point = cv::Point_<U>(right_points[i].x + image_size.width, right_points[i].y);
      cv::circle(img_with_matches, left_point, radius, color);
      cv::circle(img_with_matches, right_point, radius, color);
      cv::line(img_with_matches, left_point, right_point, color);
    }
    return img_with_matches;
  }

  static void convertPointsToHomogeneous(const cv::Mat &points, cv::OutputArray hom_points)
  {
    CV_Assert(points.channels() == 1);
    if (points.type() == CV_32F || points.type() == CV_64F)
    {
      hom_points.create(points.rows, points.cols + 1, points.type());
      cv::Mat hom_points_mat = hom_points.getMat();
      points.copyTo(hom_points_mat(cv::Rect(0, 0, points.cols, points.rows)));
      for (int i = 0; i < hom_points_mat.rows; ++i)
      {
        if (points.type() == CV_64F)
        {
          hom_points_mat.at<double>(i, points.cols) = 1;
        }
        else // CV_32F
        {
          hom_points_mat.at<float>(i, points.cols) = 1;
        }
      }
      hom_points_mat(cv::Rect(points.cols, 0, 1, points.rows));
    }
    else
    {
      throw std::runtime_error("Unable to convert non-floating point array to homogeneous coordinates");
    }
  }

};

cv::Mat CVUtilities::convertToGrayscale(cv::InputArray img)
{
  CV_Assert(img.channels() == 1 || img.channels() == 3 || img.channels() == 4);
  CV_Assert(!img.empty());
  int code;
  if (img.channels() == 3)
  {
    code = CV_RGB2GRAY;
  }
  else if (img.channels() == 4)
  {
    code = CV_RGBA2GRAY;
  }
  else
  {
    return img.getMat();
  }
  cv::Mat img_grayscale;
  cv::cvtColor(img, img_grayscale, code);
  return img_grayscale;
}

void CVUtilities::convertToGrayscaleGpu(const cv_cuda::GpuMat &img, cv_cuda::GpuMat *img_grayscale_ptr, cv_cuda::Stream &stream)
{
  int code;
  if (img.channels() == 3)
  {
    code = CV_RGB2GRAY;
  }
  else if (img.channels() == 4)
  {
    code = CV_RGBA2GRAY;
  }
  else
  {
    throw std::runtime_error("Cannot convert image to grayscale");
  }
  int destination_channels = 0;
  cv_cuda::cvtColor(img, *img_grayscale_ptr, code, destination_channels, stream);
}

cv::Mat CVUtilities::drawImageWithColormap(cv::InputArray img, cv::ColormapTypes cmap, bool show_range)
{
  CV_Assert(img.channels() == 1);
  double min;
  double max;
  cv::minMaxIdx(img, &min, &max);
//  std::cout << "min=" << min << ", max=" << max << std::endl;
  cv::Mat vis;
  img.getMat().convertTo(vis, CV_8UC1, 255 / (max - min), -min);
//  std::cout << "min=" << min << ", max=" << max << std::endl;
  cv::Mat cmap_vis;
  cv::applyColorMap(vis, cmap_vis, cv::COLORMAP_HOT);
  if (show_range)
  {
    std::ostringstream out;
    out << "Min=" << min << ", " << "Max=" << max;
    double font_scale = 1.5;
    cv::Scalar color(200, 0, 0);
    cv::putText(cmap_vis, out.str(), cv::Point(10, cmap_vis.rows - 10), cv::FONT_HERSHEY_PLAIN, font_scale, color);
  }
  return cmap_vis;
}

cv::Mat CVUtilities::getDepthImageFrom3DPointImage(cv::InputArray p3d_img, double cutoff_threshold)
{
  cv::Mat depth_img;
  cv::extractChannel(p3d_img, depth_img, 2);
  cv::Mat invalid_mask = depth_img > cutoff_threshold;
  depth_img.setTo(0.0, invalid_mask);
  return depth_img;
}

cv::Mat CVUtilities::draw3DPointImage(cv::InputArray p3d_img, cv::ColormapTypes cmap)
{
  cv::Mat depth_img = getDepthImageFrom3DPointImage(p3d_img);
  return drawImageWithColormap(depth_img);
}

cv::Mat CVUtilities::drawHistogram(cv::InputArray img, int bins, bool show_range, int width, int height)
{
  // Constants
  const cv::Scalar foreground_color(50);
  const cv::Scalar background_color(200);
  const cv::Scalar text_color(230);
  const cv::Scalar grid_color(120);
  const int channels = 0;
  const int hist_dim = 1;;
  // Compute input min and max
  cv::Mat img_mat = img.getMat();
  double min, max;
  cv::minMaxIdx(img_mat, &min, &max);
  // Compute histogram
  float range[] = {static_cast<float>(min), static_cast<float>(max)};
  const float *ranges[] = {range};
  cv::Mat hist;
  cv::calcHist(&img_mat, 1, &channels, cv::noArray(), hist, hist_dim, &bins, ranges);
  CV_Assert(hist.type() == CV_32F || hist.type() == CV_8U);
  double hist_min, hist_max;
  cv::minMaxIdx(hist, &hist_min, &hist_max);
  // Draw histogram
  cv::Mat hist_img(height, width, CV_8UC1);
  hist_img.setTo(background_color);
  int padding_x = 10;
  int padding_y = 10;
  float scale_x = static_cast<float>(width - 2 * padding_x) / bins;
  float scale_y = static_cast<float>(height - 2 * padding_y) / hist_max;
  for(int i = 0; i < bins; i++)
  {
    float hist_value;
    if (hist.type() == CV_32F)
    {
      hist_value = hist.at<float>(i);
    }
    else
    {
      hist_value = hist.at<uchar>(i);
    }
    int num_pts = 5;
    cv::Point pt1(padding_x + i*scale_x, height - padding_y - 0);
    cv::Point pt2(padding_x + i*scale_x, height - padding_y - hist_value*scale_y);
    cv::Point pt3(padding_x + i*scale_x + scale_x, height - padding_y - hist_value*scale_y);
    cv::Point pt4(padding_x + i*scale_x + scale_x, height - padding_y - 0);
    std::vector<cv::Point> pts = {pt1, pt2, pt3, pt4, pt1};
    cv::fillConvexPoly(hist_img, pts, foreground_color);
    if (i > 0)
    {
      cv::Point line_pt1(padding_x + i*scale_x, 0);
      cv::Point line_pt2(padding_x + i*scale_x, height);
      cv::line(hist_img, line_pt1, line_pt2, grid_color);
    }
  }
  if (show_range)
  {
    std::ostringstream out;
    out << "Min=" << min << ", " << "Max=" << max;
    double font_scale = 1.5;
    cv::putText(hist_img, out.str(), cv::Point(padding_x + 10, hist_img.rows - padding_y - 10), cv::FONT_HERSHEY_PLAIN, font_scale, text_color);
  }
  return hist_img;
}

cv::Mat CVUtilities::drawKeypoints(cv::InputArray img, const std::vector<cv::KeyPoint> &keypoints)
{
  cv::Mat img_with_keypoints;
#ifdef OPENCV_2_4
  cv::drawKeypoints(img.getMat(), keypoints, img_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
#else
  cv::drawKeypoints(img, keypoints, img_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
#endif
  return img_with_keypoints;
}

cv::Mat CVUtilities::drawFeatureMatches(cv::InputArray left_img, const std::vector<cv::KeyPoint> &left_keypoints, cv::InputArray right_img, const std::vector<cv::KeyPoint> &right_keypoints, const std::vector<cv::DMatch> &matches)
{
  cv::Mat match_img;
#ifdef OPENCV_2_4
  cv::drawMatches(left_img.getMat(), left_keypoints, right_img.getMat(), right_keypoints, matches, match_img);
#else
  cv::drawMatches(left_img, left_keypoints, right_img, right_keypoints, matches, match_img);
#endif
  return match_img;
}

} /* namespace ait */
