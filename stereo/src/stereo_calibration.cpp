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
: rotation(cv::Mat::eye(3, 3, CV_64FC1)),
  translation(cv::Mat::zeros(3, 1, CV_64FC1)),
  essential_matrix(cv::Mat::zeros(3, 3, CV_64FC1)),
  fundamental_matrix(cv::Mat::zeros(3, 3, CV_64FC1)),
  projection_matrix_left(cv::Mat::eye(3, 4, CV_64FC1)),
  projection_matrix_right(cv::Mat::eye(3, 4, CV_64FC1))
{
}

StereoCameraCalibration::~StereoCameraCalibration()
{
}

} /* namespace stereo */

