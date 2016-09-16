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
  double ratio_test_threshold_;
  double epipolar_constraint_threshold_;
  int match_norm_;
  cv::Ptr<cv::flann::IndexParams> flann_index_params_;
  cv::Ptr<cv::flann::SearchParams> flann_search_params_;

public:
  class Error : public std::runtime_error
  {
  public:
    Error(const std::string &msg);
    ~Error() override;
  };

  DenseStereoMatcher(const StereoCameraCalibration &calib);
  virtual ~DenseStereoMatcher();


#if OPENCV_3_1
cv::Mat match(
    const cv::InputArray left_input_img, cv::InputArray right_input_img,
    bool verbose=true)
{
  stereo::Timer timer;
  double vis_mult = 1.0;

  cv::Mat left_img = stereo::Utilities::convertToGrayscale(left_input_img);
  cv::Mat right_img = stereo::Utilities::convertToGrayscale(right_input_img);

  // CUDA StereoBM
//  cv::Ptr<cv::cuda::StereoBM> left_matcher = cv::cuda::createStereoBM(max_disp, wsize);

  // CUDA StereoBM
  int num_disp = 128;
  int block_size = 19;
  int iters = 8;
  int levels = 4;
  int nr_plane = 4;
//  cv::Ptr<cv::cuda::StereoBM> left_matcher = cv::cuda::createStereoBM(num_disp, block_size);
//  cv::Ptr<cv::cuda::StereoBeliefPropagation> left_matcher = cv::cuda::createStereoBeliefPropagation(num_disp, iters, levels);
  cv::Ptr<cv::cuda::StereoConstantSpaceBP> left_matcher = cv::cuda::createStereoConstantSpaceBP(num_disp, iters, levels, nr_plane);
  cv_cuda::GpuMat left_img_gpu;
  cv_cuda::GpuMat right_img_gpu;
  left_img_gpu.upload(left_img);
  right_img_gpu.upload(right_img);
  timer.start();
  cv_cuda::GpuMat left_disp_gpu;
  left_matcher->compute(left_img_gpu, right_img_gpu, left_disp_gpu);
  timer.stopAndPrintTiming("Dense stereo matching");
  cv::Mat left_disp;
  left_disp_gpu.download(left_disp);
//
  std::cout << left_disp.type() << ", " << CV_16S << std::endl;
  left_disp.convertTo(left_disp, CV_8U);
  cv::namedWindow("left disparity", cv::WINDOW_AUTOSIZE);
  cv::imshow("left disparity", left_disp);

  return left_disp;
}
#endif

};

} /* namespace stereo */

#include "sparse_stereo_matcher.hpp"

