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
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <sparse_stereo_matcher.h>
#include <utilities.h>

template <typename T>
void chessboardTriangulation(
    const stereo::SparseStereoMatcher<T> &matcher,
    const cv::InputArray left_input_img, cv::InputArray right_input_img,
    const cv::Size &board_size)
{
  using Utilities = stereo::Utilities;

  cv::Mat left_img = Utilities::convertToGrayscale(left_input_img);
  cv::Mat right_img = Utilities::convertToGrayscale(right_input_img);

  std::cout << "channels: " << left_img.channels() << ", dtype: " << left_img.type() << ", continuous: " << left_img.isContinuous() << ", depth: " << left_img.depth() << std::endl;
  std::cout << "width: " << left_img.cols << ", height: " << left_img.rows << ", elementSize: " << left_img.elemSize1() << std::endl;

  std::vector<cv::Point2f> left_points;
  std::vector<cv::Point2f> right_points;
  bool found_left = cv::findChessboardCorners(left_img, board_size, left_points,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
  bool found_right= cv::findChessboardCorners(right_img, board_size, right_points,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

  if (!found_left || !found_right)
  {
    throw std::runtime_error("Unable to find chessboard corners");
  }

  cv::cornerSubPix(left_img, left_points, cv::Size(11,11),
      cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));
  cv::cornerSubPix(right_img, right_points, cv::Size(11,11),
      cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));

  auto left_img_with_points = Utilities::drawPoints(left_input_img, left_points);
  auto right_img_with_points = Utilities::drawPoints(right_input_img, right_points);
  cv::imshow("Left points", left_img_with_points);
  cv::imshow("Right points", right_img_with_points);

  auto match_img = Utilities::drawPointMatches(left_input_img, left_points, right_input_img, right_points);
  cv::imshow("Point matches", match_img);

  std::vector<cv::Point3f> points_3d = matcher.triangulatePoints(left_points, right_points);

  cv::waitKey(0);
}

template <typename T>
void sparseStereoMatchingFull(const stereo::SparseStereoMatcher<T> &matcher, const cv::InputArray left_img, cv::InputArray right_img)
{
  using Utilities = stereo::Utilities;

  std::cout << "channels: " << left_img.channels() << ", dtype: " << left_img.type() << ", continuous: " << left_img.getMat().isContinuous() << ", depth: " << left_img.depth() << std::endl;
  std::cout << "width: " << left_img.getMat().cols << ", height: " << left_img.getMat().rows << ", elementSize: " << left_img.getMat().elemSize1() << std::endl;

  auto match_result = matcher.matchFull(left_img, right_img);

  auto left_img_with_keypoints = Utilities::drawKeypoints(left_img, match_result.left_keypoints);
  auto right_img_with_keypoints = Utilities::drawKeypoints(right_img, match_result.right_keypoints);
  cv::imshow("Left keypoints", left_img_with_keypoints);
  cv::imshow("Right keypoints", right_img_with_keypoints);

  auto match_img = Utilities::drawFeatureMatches(left_img, match_result.left_keypoints, right_img, match_result.right_keypoints, match_result.matches);
  cv::imshow("Keypoint matches", match_img);

  cv::waitKey(0);
}

template <typename T>
void profileSparseStereoMatching(
    const stereo::SparseStereoMatcher<T> &matcher,
    cv::InputArray left_input_img, cv::InputArray right_input_img)
{
  int num_of_iterations = 5;
  double matching_time = (double)cv::getTickCount();
  for (int i = 0; i < num_of_iterations; ++i)
  {
    std::vector<cv::Point2d> image_points;
    std::vector<cv::Point3d> points_3d = matcher.match(left_input_img, right_input_img, &image_points);
  }
  matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
  matching_time /= num_of_iterations;
  std::cout << "sparse matching time: " << matching_time << std::endl;
}

template <typename T>
void denseStereoMatching(
    const stereo::SparseStereoMatcher<T> &matcher,
    cv::InputArray left_input_img, cv::InputArray right_input_img)
{
  int max_disp = 16*7;
  int wsize = 19;
  double vis_mult = 1.0;

  cv::Mat left_img = stereo::Utilities::convertToGrayscale(left_input_img);
  cv::Mat right_img = stereo::Utilities::convertToGrayscale(right_input_img);

  cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(max_disp, wsize);
  cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

  cv::Mat left_disp;
  cv::Mat right_disp;
  double matching_time = (double)cv::getTickCount();
  left_matcher->compute(left_img, right_img, left_disp);
  right_matcher->compute(right_img, left_img, right_disp);
  matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();

  double lambda = 8000.0;
  double sigma = 1.5;
  wls_filter->setLambda(lambda);
  wls_filter->setSigmaColor(sigma);
  double filtering_time = (double)cv::getTickCount();
  cv::Mat filtered_disp;
  wls_filter->filter(left_disp, left_img, filtered_disp, right_disp);
  filtering_time = ((double)cv::getTickCount() - filtering_time) / cv::getTickFrequency();

  std::cout << "matching time: " << matching_time << std::endl;
  cv::Mat raw_disp_vis;
  cv::ximgproc::getDisparityVis(left_disp, raw_disp_vis, vis_mult);
  cv::namedWindow("raw disparity", cv::WINDOW_AUTOSIZE);
  cv::imshow("raw disparity", raw_disp_vis);

  std::cout << "filtering time: " << filtering_time << std::endl;
  cv::Mat filtered_disp_vis;
  cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
  cv::namedWindow("filtered disparity", cv::WINDOW_AUTOSIZE);
  cv::imshow("filtered disparity", filtered_disp_vis);

  cv::waitKey();
}

int main(int argc, char **argv)
{
  try
  {
    TCLAP::CmdLine cmd("Sparse stereo matching test", ' ', "0.1");
    TCLAP::ValueArg<std::string> left_img_arg("l", "left", "Left image file to use", true, "", "filename", cmd);
    TCLAP::ValueArg<std::string> right_img_arg("r", "right", "Right image file to use", true, "", "filename", cmd);
    TCLAP::ValueArg<std::string> calib_arg("c", "calib", "Calibration file to use", true, "", "filename", cmd);
    TCLAP::SwitchArg chessboard_arg("", "chessboard", "Use board pattern", cmd, false);
    TCLAP::ValueArg<int> board_width_arg("", "width", "Board width", false, 8, "integer", cmd);
    TCLAP::ValueArg<int> board_height_arg("", "height", "Board height", false, 6, "integer", cmd);

    cmd.parse(argc, argv);

    // TODO: For profiling (bug in oprofile eclipse plugin)
    chdir("/home/bhepp/Projects/Quad3DR/workspace/stereo");
    std::cout << "cwd: " << get_current_dir_name() << std::endl;

    cv::Mat left_img = cv::imread(left_img_arg.getValue(), cv::IMREAD_COLOR);
    if (left_img.data == nullptr)
    {
      throw std::runtime_error("Unable to read left image");
    }
    cv::Mat right_img = cv::imread(right_img_arg.getValue(), cv::IMREAD_COLOR);
    if (right_img.data == nullptr)
    {
      throw std::runtime_error("Unable to read right image");
    }

    stereo::StereoCameraCalibration calib = stereo::Utilities::readStereoCalibration(calib_arg.getValue());

    // SURF
//    using DetectorType = cv::xfeatures2d::SURF;
//    using DescriptorType = cv::xfeatures2d::SURF;
//    const int hessian_threshold = 400;
//    cv::Ptr<DetectorType> detector = DescriptorType::create(hessian_threshold);
//    cv::Ptr<DescriptorType> descriptor_computer = detector;

    // FREAK
//    using DetectorType = cv::FastFeatureDetector;
//    using DescriptorType = cv::xfeatures2d::FREAK;
//    cv::Ptr<DetectorType> detector = DetectorType::create();
//    cv::Ptr<DescriptorType> descriptor_computer = DescriptorType::create(true, true, 16, 2);

    // ORB
    using DetectorType = cv::ORB;
    using DescriptorType = cv::ORB;
    cv::Ptr<DetectorType> detector = DetectorType::create();
    cv::Ptr<DescriptorType> descriptor_computer = detector;

    using FeatureDetectorType = stereo::FeatureDetectorOpenCV<DetectorType, DescriptorType>;
    cv::Ptr<FeatureDetectorType> feature_detector = new FeatureDetectorType(detector, descriptor_computer);

    using SparseStereoMatcherType = stereo::SparseStereoMatcher<FeatureDetectorType>;
    SparseStereoMatcherType matcher(feature_detector, calib);

    // ORB
    matcher.setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
    matcher.setMatchNorm(cv::NORM_HAMMING2);

    // FREAK
//    matcher.setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
//    matcher.setMatchNorm(cv::NORM_HAMMING2);

//    denseStereoMatching(matcher, left_img, right_img);
    profileSparseStereoMatching(matcher, left_img, right_img);
//    if (chessboard_arg.getValue())
//    {
//      cv::Size board_size(board_width_arg.getValue(), board_height_arg.getValue());
//      chessboardTriangulation(matcher, left_img, right_img, board_size);
//    }
//    else
//    {
//      sparseStereoMatchingFull<>(matcher, left_img, right_img);
//    }
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
