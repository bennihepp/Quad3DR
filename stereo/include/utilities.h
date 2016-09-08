//==================================================
// utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace stereo
{

class Utilities
{
public:
  Utilities() = delete;

  static cv::Mat convertToGrayscale(cv::InputArray img);

  static cv::Mat drawKeypoints(cv::InputArray img, const std::vector<cv::KeyPoint> &keypoints);

  static cv::Mat drawFeatureMatches(cv::InputArray left_img, const std::vector<cv::KeyPoint> &left_keypoints, cv::InputArray right_img, const std::vector<cv::KeyPoint> &right_keypoints, const std::vector<cv::DMatch> &matches);

};

} /* namespace stereo */
