//==================================================
// sparse_stereo_test.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#include <iostream>
#include <stdexcept>
#include <vector>
#include <tclap/CmdLine.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sparse_stereo_matcher.h>
#include <utilities.h>

template <typename T>
void sparse_stereo_matching(const stereo::SparseStereoMatcher<T> &matcher, const cv::InputArray left_img, cv::InputArray right_img)
{
  using Utilities = stereo::Utilities;

  std::cout << "channels: " << left_img.channels() << ", dtype: " << left_img.type() << ", continuous: " << left_img.getMat().isContinuous() << ", depth: " << left_img.depth() << std::endl;
  std::cout << "width: " << left_img.getMat().cols << ", height: " << left_img.getMat().rows << ", elementSize: " << left_img.getMat().elemSize1() << std::endl;

  auto match_result = matcher.match(left_img, right_img);

  auto left_img_with_keypoints = Utilities::drawKeypoints(left_img, match_result.left_keypoints);
  auto right_img_with_keypoints = Utilities::drawKeypoints(right_img, match_result.right_keypoints);
  cv::imshow("Left keypoints", left_img_with_keypoints);
  cv::imshow("Right keypoints", right_img_with_keypoints);

  auto match_img = Utilities::drawFeatureMatches(left_img, match_result.left_keypoints, right_img, match_result.right_keypoints, match_result.matches);
  cv::imshow("Keypoint matches", match_img);

  cv::waitKey(0);
}

static stereo::StereoCameraCalibration readStereoCalibration(const std::string &filename)
{
  stereo::StereoCameraCalibration calib;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    throw std::runtime_error("Unable to open calibration file");
  }

  fs["camera_matrix_left"] >> calib.left.camera_matrix;
  fs["distortion_coefficients_left:"] >> calib.left.dist_coeffs;

  fs["camera_matrix_right"] >> calib.right.camera_matrix;
  fs["distortion_coefficients_right:"] >> calib.right.dist_coeffs;

  fs["rotation"] >> calib.rotation;
  fs["translation"] >> calib.translation;
  fs["essential_matrix"] >> calib.essential_matrix;
  fs["fundamental_matrix"] >> calib.fundamental_matrix;

  calib.projection_matrix_left(cv::Rect(0, 0, 3, 3)) = calib.left.camera_matrix;
  calib.projection_matrix_right(cv::Rect(0, 0, 3, 3)) = calib.right.camera_matrix;
  calib.projection_matrix_right(cv::Rect(3, 0, 1, 3)) = -calib.translation;

  std::cout << calib.left.camera_matrix << std::endl;
  std::cout << calib.left.dist_coeffs << std::endl;
  std::cout << calib.projection_matrix_left << std::endl;

  return calib;
}

int main(int argc, char **argv)
{
  try
  {
    TCLAP::CmdLine cmd("Sparse stereo matching test", ' ', "0.1");
    TCLAP::ValueArg<std::string> left_img_arg("l", "left", "Left image file to use", true, "", "filename", cmd);
    TCLAP::ValueArg<std::string> right_img_arg("r", "right", "Right image file to use", true, "", "filename", cmd);
    TCLAP::ValueArg<std::string> calib_arg("c", "calib", "Calibration file to use", true, "", "filename", cmd);;
//    TCLAP::ValueArg<bool> show_arg("s", "show", "Show captured video", false, true, "boolean", cmd);

    cmd.parse(argc, argv);

    cv::Mat left_img = cv::imread(left_img_arg.getValue(), cv::IMREAD_COLOR);
    cv::Mat right_img = cv::imread(right_img_arg.getValue(), cv::IMREAD_COLOR);

    stereo::StereoCameraCalibration calib = readStereoCalibration(calib_arg.getValue());

    using FeatureType = cv::xfeatures2d::SURF;
    stereo::SparseStereoMatcher<FeatureType> matcher(calib);

    sparse_stereo_matching<>(matcher, left_img, right_img);
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
