//==================================================
// utilities.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#include <utilities.h>

namespace stereo
{

cv::Mat Utilities::convertToGrayscale(cv::InputArray img)
{
  CV_Assert(img.channels() == 1 || img.channels() == 3);
  cv::Mat img_m = img.getMat();
  CV_Assert(!img_m.empty());
  if (img_m.channels() > 1)
  {
    cv::Mat grayscale_img;
    cv::cvtColor(img_m, grayscale_img, CV_RGB2GRAY);
    return grayscale_img;
  }
  else
  {
    return img_m;
  }
}

cv::Mat Utilities::drawKeypoints(cv::InputArray img, const std::vector<cv::KeyPoint> &keypoints)
{
  cv::Mat img_with_keypoints;
  cv::drawKeypoints(img, keypoints, img_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  return img_with_keypoints;
}

cv::Mat Utilities::drawFeatureMatches(cv::InputArray left_img, const std::vector<cv::KeyPoint> &left_keypoints, cv::InputArray right_img, const std::vector<cv::KeyPoint> &right_keypoints, const std::vector<cv::DMatch> &matches)
{
  cv::Mat match_img;
  cv::drawMatches(left_img, left_keypoints, right_img, right_keypoints, matches, match_img);
  return match_img;
}

} /* namespace stereo */
