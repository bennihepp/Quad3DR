//==================================================
// images.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 04.04.17
//==================================================
#pragma once

#include <opencv2/opencv.hpp>

#if OPENCV_2_4
namespace cv_cuda = cv::gpu;
#else
namespace cv_cuda = cv::cuda;
#endif

namespace bh {
namespace opencv {

class Images {
public:
  Images() = delete;

  static cv::Mat convertToGrayscale(cv::InputArray img);

  static void convertToGrayscaleGpu(
          const cv_cuda::GpuMat& img, cv_cuda::GpuMat* img_grayscale_ptr, cv_cuda::Stream& stream);

  static cv::Mat getDepthImageFrom3DPointImage(cv::InputArray p3d_img, double cutoff_threshold = 1e4);

  static void convertPointsToHomogeneous(const cv::Mat& points, cv::OutputArray hom_points) {
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

cv::Mat Images::convertToGrayscale(cv::InputArray img) {
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

void Images::convertToGrayscaleGpu(
        const cv_cuda::GpuMat& img, cv_cuda::GpuMat* img_grayscale_ptr, cv_cuda::Stream& stream)  {
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

cv::Mat Images::getDepthImageFrom3DPointImage(cv::InputArray p3d_img, double cutoff_threshold)  {
  cv::Mat depth_img;
  cv::extractChannel(p3d_img, depth_img, 2);
  cv::Mat invalid_mask = depth_img > cutoff_threshold;
  depth_img.setTo(0.0, invalid_mask);
  return depth_img;
}

} /* namespace opencv */
} /* namespace bh */
