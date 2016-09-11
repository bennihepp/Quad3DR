//==================================================
// stereo_calibration.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 6, 2016
//==================================================

#include <stereo_calibration.h>

namespace stereo
{

CameraCalibration::CameraCalibration()
: camera_matrix(cv::Mat::eye(3, 3, CV_64FC1)),
  dist_coeffs(cv::Mat::zeros(5, 1, CV_64FC1))
{

}

CameraCalibration::~CameraCalibration()
{
}

StereoCameraCalibration::StereoCameraCalibration()
: image_size(),
  rotation(cv::Mat::eye(3, 3, CV_64FC1)),
  translation(cv::Mat::zeros(3, 1, CV_64FC1)),
  essential_matrix(cv::Mat::zeros(3, 3, CV_64FC1)),
  fundamental_matrix(cv::Mat::zeros(3, 3, CV_64FC1)),
  projection_matrix_left(),
  projection_matrix_right()
{
}

StereoCameraCalibration::~StereoCameraCalibration()
{
}

void StereoCameraCalibration::computeProjectionMatrices()
{
  cv::Mat R1, R2, Q;
  cv::stereoRectify(
      left.camera_matrix, left.dist_coeffs,
      right.camera_matrix, right.dist_coeffs,
      image_size,
      rotation, translation,
      R1, R2,
      projection_matrix_left, projection_matrix_right,
      Q
  );
  // For debugging
  std::cout << "projection_matrix_left: " << projection_matrix_left << std::endl;
  std::cout << "projection_matrix_right: " << projection_matrix_right << std::endl;
}

} /* namespace stereo */

