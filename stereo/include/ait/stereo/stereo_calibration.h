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
#include <Eigen/Dense>
#if WITH_ZED
	#include <zed\Camera.hpp>
#endif

namespace ait
{
namespace stereo
{

struct CameraCalibration
{
  // Matrices as returned from cv::calibrateCamera
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;


  CameraCalibration();
  virtual ~CameraCalibration();

  Eigen::Matrix3d getCameraMatrixEigen() const;
  Eigen::VectorXd getDistCoeffEigen() const;
};

struct StereoCameraCalibration
{
#if WITH_ZED
	static CameraCalibration getCameraCalibrationFromZED(const sl::zed::CamParameters& params);
	static StereoCameraCalibration getStereoCalibrationFromZED(sl::zed::Camera* zed);
#endif
	static StereoCameraCalibration getStereoCalibrationFromOpenCV(const std::string &filename);
	static StereoCameraCalibration readStereoCalibration(const std::string &filename);

  StereoCameraCalibration();
  virtual ~StereoCameraCalibration();

  Eigen::Matrix3d getRotationEigen() const;
  Eigen::Vector3d getTranslationEigen() const;

  Eigen::Matrix4d getLeftExtrinsicsEigen() const;
  Eigen::Matrix4d getRightExtrinsicsEigen() const;

  void computeProjectionMatrices();

  CameraCalibration left;
  CameraCalibration right;

  cv::Size image_size;

  // Matrices as returned from cv::stereoCalibrate
  cv::Mat rotation;
  cv::Mat translation;
  cv::Mat essential_matrix;
  cv::Mat fundamental_matrix;

  // Matrices as returned from cv::stereoRectify
  //
  // 3x4 Projection matrices
  cv::Mat projection_matrix_left;
  cv::Mat projection_matrix_right;
  //
  // 3x3 Rectification matrices
  cv::Mat rectification_transform_left;
  cv::Mat rectification_transform_right;
  //
  // 4x4 Disparity to depth mapping (OpenCV)
  cv::Mat disparity_to_depth_map;
};

}  // namespace stereo
}  // namespace ait

