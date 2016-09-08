//==================================================
// sparse_stereo_matcher.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "stereo_calibration.h"

namespace stereo
{

struct SparseMatchResult
{
  std::vector<cv::Point3d> points_3d;
  std::vector<cv::Point2d> left_points;
  std::vector<cv::Point2d> right_points;
  std::vector<cv::KeyPoint> left_keypoints;
  std::vector<cv::KeyPoint> right_keypoints;
  std::vector<cv::Mat> left_descriptors;
  std::vector<cv::Mat> right_descriptors;
  std::vector<cv::DMatch> matches;
  std::vector<double> epipolar_constraints;
};


template <typename T>
class SparseStereoMatcher
{
  StereoCameraCalibration calib_;

  cv::Ptr<T> detector_;

  std::vector<cv::KeyPoint> detectFeatureKeypoints(const cv::InputArray img, std::vector<cv::Point2d> *points_ptr) const;
  cv::Mat computeFeatureDescriptors(cv::InputArray img, std::vector<cv::KeyPoint> &keypoints) const;
  cv::Mat undistortPoints(cv::InputArray &points, cv::InputArray camera_matrix, cv::InputArray dist_coefficients) const;

  std::vector<cv::DMatch> filterMatches(const std::vector<cv::DMatch> &all_matches, double good_threshold_multiplier=5, double min_good_threshold=0.02) const;
  std::vector<cv::DMatch> matchFeaturesBf(cv::InputArray left_descriptors, cv::InputArray right_descriptors) const;
  std::vector<cv::DMatch> matchFeaturesFlann(cv::InputArray left_descriptors, cv::InputArray right_descriptors) const;
  std::vector<cv::DMatch> matchFeaturesFlannKnn2(cv::InputArray left_descriptors, cv::InputArray right_descriptors, double ratio_test_threshold=0.7) const;

public:
  SparseStereoMatcher(const StereoCameraCalibration &calib);
  virtual ~SparseStereoMatcher();

  SparseMatchResult match(const cv::InputArray left_color_img, cv::InputArray right_color_img) const;
};

} /* namespace stereo */

#include "sparse_stereo_matcher.hpp"

