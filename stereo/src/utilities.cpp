//==================================================
// utilities.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#include <utilities.h>
#if OPENCV_3
  #include <opencv2/cudaimgproc.hpp>
#endif

namespace stereo
{

#if OPENCV_2_4
  namespace cv_cuda = cv::gpu;
#else
  namespace cv_cuda = cv::cuda;
#endif

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

cv::Mat Utilities::drawKeypoints(cv::InputArray img, const std::vector<cv::KeyPoint> &keypoints)
{
  cv::Mat img_with_keypoints;
#ifdef OPENCV_2_4
  cv::drawKeypoints(img.getMat(), keypoints, img_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
#else
  cv::drawKeypoints(img, keypoints, img_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
#endif
  return img_with_keypoints;
}

cv::Mat Utilities::drawFeatureMatches(cv::InputArray left_img, const std::vector<cv::KeyPoint> &left_keypoints, cv::InputArray right_img, const std::vector<cv::KeyPoint> &right_keypoints, const std::vector<cv::DMatch> &matches)
{
  cv::Mat match_img;
#ifdef OPENCV_2_4
  cv::drawMatches(left_img.getMat(), left_keypoints, right_img.getMat(), right_keypoints, matches, match_img);
#else
  cv::drawMatches(left_img, left_keypoints, right_img, right_keypoints, matches, match_img);
#endif
  return match_img;
}

} /* namespace stereo */
