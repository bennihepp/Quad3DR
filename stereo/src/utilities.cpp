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

StereoCameraCalibration Utilities::readStereoCalibration(const std::string &filename)
{
  stereo::StereoCameraCalibration calib;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    throw std::runtime_error("Unable to open calibration file");
  }

  fs["image_width"] >> calib.image_size.width;
  fs["image_height"] >> calib.image_size.height;

  fs["camera_matrix_left"] >> calib.left.camera_matrix;
  fs["distortion_coefficients_left"] >> calib.left.dist_coeffs;

  fs["camera_matrix_right"] >> calib.right.camera_matrix;
  fs["distortion_coefficients_right"] >> calib.right.dist_coeffs;

  fs["rotation"] >> calib.rotation;
  fs["translation"] >> calib.translation;
  fs["essential_matrix"] >> calib.essential_matrix;
  fs["fundamental_matrix"] >> calib.fundamental_matrix;

  calib.computeProjectionMatrices();

//  std::cout << calib.left.camera_matrix << std::endl;
//  std::cout << calib.left.dist_coeffs << std::endl;
//  std::cout << calib.projection_matrix_right << std::endl;
//  std::cout << calib.essential_matrix << std::endl;
//  std::cout << calib.fundamental_matrix << std::endl;

  return calib;
}

cv::Mat Utilities::convertToGrayscale(cv::InputArray img)
{
  CV_Assert(img.channels() == 1 || img.channels() == 3 || img.channels() == 4);
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
