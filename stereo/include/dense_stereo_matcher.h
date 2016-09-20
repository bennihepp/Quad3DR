//==================================================
// dense_stereo_matcher.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 16, 2016
//==================================================

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#if OPENCV_3_1
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/ximgproc.hpp>
  #include <opencv2/cudastereo.hpp>
#endif
#include "stereo_calibration.h"

namespace stereo
{

#if OPENCV_2_4
  namespace cv_cuda = cv::gpu;
#else
  namespace cv_cuda = cv::cuda;
#endif

class DenseStereoMatcher
{
  StereoCameraCalibration calib_;

public:
  class Error : public std::runtime_error
  {
  public:
    Error(const std::string &msg);
    ~Error() override;
  };

  DenseStereoMatcher(const StereoCameraCalibration &calib)
  : calib_(calib)
  {
  }

  virtual ~DenseStereoMatcher()
  {
  }

#if OPENCV_3_1
cv::Mat match(
    const cv::InputArray left_input_img, cv::InputArray right_input_img,
    bool verbose=true,
    cv_cuda::Stream &stream=cv_cuda::Stream::Null())
{
  stereo::Timer timer;
  double vis_mult = 1.0;

  cv::Mat left_img = stereo::Utilities::convertToGrayscale(left_input_img);
  cv::Mat right_img = stereo::Utilities::convertToGrayscale(right_input_img);

  int num_disp = 128;
  int block_size = 31;
  int iters = 8;
  int levels = 4;
  int nr_plane = 4;
  int min_disparity = 4;
  int P1 = 3;
  int P2 = 5;

  // StereoSGBM
  cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(min_disparity, num_disp, block_size, P1, P2);
  left_matcher->setMode(cv::StereoSGBM::MODE_HH);
//  left_matcher->setMinDisparity(min_disparity);
  cv::Mat left_disp;
  left_matcher->compute(left_img, right_img, left_disp);
  CV_Assert(left_disp.type() == CV_16S);
  timer.stopAndPrintTiming("Dense stereo matching");
//
  left_disp.convertTo(left_disp, CV_32F, 1 / 16.0);
  cv::Mat min_disp_mask = left_disp == min_disparity - 1;
  left_disp.setTo(0.0, min_disp_mask);

  cv::Mat left_disp_vis;
  left_disp.convertTo(left_disp_vis, CV_8U);
  cv::namedWindow("left disparity", cv::WINDOW_AUTOSIZE);
  cv::imshow("left disparity", left_disp_vis);

  return left_disp;

//  // CUDA StereoBM
////  cv::Ptr<cv::cuda::StereoBM> left_matcher = cv::cuda::createStereoBM(num_disp, block_size);
////  cv::Ptr<cv::cuda::StereoBeliefPropagation> left_matcher = cv::cuda::createStereoBeliefPropagation(num_disp, iters, levels);
//  cv::Ptr<cv::cuda::StereoConstantSpaceBP> left_matcher = cv::cuda::createStereoConstantSpaceBP(num_disp, iters, levels, nr_plane);
//  left_matcher->setMinDisparity(min_disparity);
//  cv_cuda::GpuMat left_img_gpu;
//  cv_cuda::GpuMat right_img_gpu;
//  left_img_gpu.upload(left_img, stream);
//  right_img_gpu.upload(right_img, stream);
//  timer.start();
//  cv_cuda::GpuMat left_disp_gpu;
//  left_matcher->compute(left_img_gpu, right_img_gpu, left_disp_gpu, stream);
//  CV_Assert(left_disp_gpu.type() == CV_16S);
//  timer.stopAndPrintTiming("Dense stereo matching");
//  cv::Mat left_disp;
//  left_disp_gpu.download(left_disp, stream);
//  CV_Assert(left_disp.type() == CV_16S);
////
//  cv::Mat left_disp_vis;
//  left_disp.convertTo(left_disp_vis, CV_8U);
//  cv::namedWindow("left disparity", cv::WINDOW_AUTOSIZE);
//  cv::imshow("left disparity", left_disp_vis);
//
//  left_disp.convertTo(left_disp, CV_32F);
//  return left_disp;
}
#endif

};

} /* namespace stereo */
