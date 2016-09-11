//==================================================
// stereo_calibration.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 6, 2016
//==================================================

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace stereo
{

struct CameraCalibration
{
  CameraCalibration();
  virtual ~CameraCalibration();

  // Matrices as returned from cv::calibrateCamera
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

struct StereoCameraCalibration
{
  StereoCameraCalibration();
  virtual ~StereoCameraCalibration();

  void computeProjectionMatrices();

  CameraCalibration left;
  CameraCalibration right;

  cv::Size image_size;

  // Matrices as returned from cv::stereoCalibrate
  cv::Mat rotation;
  cv::Mat translation;
  cv::Mat essential_matrix;
  cv::Mat fundamental_matrix;

  // 3x4 Projection matrix of left and right camera
  cv::Mat projection_matrix_left;
  cv::Mat projection_matrix_right;
};

} /* namespace stereo */

