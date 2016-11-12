//==================================================
// stereo_calibration.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 6, 2016
//==================================================

#include <ait/stereo/stereo_calibration.h>
#include <opencv2/core/eigen.hpp>

namespace ait
{
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

Eigen::Matrix3d CameraCalibration::getCameraMatrixEigen() const
{
  Eigen::Matrix3d camera_matrix_eig;
  cv::cv2eigen(camera_matrix, camera_matrix_eig);
  return camera_matrix_eig;
}

Eigen::VectorXd CameraCalibration::getDistCoeffEigen() const
{
  Eigen::VectorXd dist_coeffs_eig;
  cv::cv2eigen(dist_coeffs, dist_coeffs_eig);
  return dist_coeffs_eig;
}


CameraCalibration StereoCameraCalibration::getCameraCalibrationFromZED(const sl::zed::CamParameters& params)
{
	CameraCalibration calib;
	calib.camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
	calib.camera_matrix.at<double>(0, 0) = params.fx;
	calib.camera_matrix.at<double>(1, 1) = params.fx;
	calib.camera_matrix.at<double>(2, 2) = 1;
	calib.camera_matrix.at<double>(0, 2) = params.cx;
	calib.camera_matrix.at<double>(1, 2) = params.cy;
	calib.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
	// ZED SDK returns undistorted images
	//  calib.dist_coeffs.at<double>(0, 0) = params.disto[0]; // k1
	//  calib.dist_coeffs.at<double>(1, 0) = params.disto[1]; // k2
	//  calib.dist_coeffs.at<double>(2, 0) = params.disto[3]; // r1 = p1
	//  calib.dist_coeffs.at<double>(3, 0) = params.disto[4]; // r2 = p2
	//  calib.dist_coeffs.at<double>(4, 0) = params.disto[2]; // k3
	return calib;
}

StereoCameraCalibration StereoCameraCalibration::getStereoCalibrationFromZED(sl::zed::Camera* zed)
{
	const sl::zed::StereoParameters* stereo_params = zed->getParameters();
	StereoCameraCalibration calib;

	calib.image_size.width = zed->getImageSize().width;
	calib.image_size.height = zed->getImageSize().height;

	calib.left = getCameraCalibrationFromZED(stereo_params->LeftCam);
	calib.right = getCameraCalibrationFromZED(stereo_params->RightCam);

	calib.translation = cv::Mat::zeros(3, 1, CV_64F);
	calib.translation.at<double>(0, 0) = -stereo_params->baseline;
	calib.translation.at<double>(1, 0) = -stereo_params->Ty;
	calib.translation.at<double>(2, 0) = -stereo_params->Tz;

	cv::Mat rotVecX = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat rotVecY = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat rotVecZ = cv::Mat::zeros(3, 1, CV_64F);
	rotVecX = stereo_params->Rx;
	rotVecY = stereo_params->convergence;
	rotVecZ = stereo_params->Rz;
	cv::Mat rotX;
	cv::Mat rotY;
	cv::Mat rotZ;
	cv::Rodrigues(rotVecX, rotX);
	cv::Rodrigues(rotVecY, rotY);
	cv::Rodrigues(rotVecZ, rotZ);
	calib.rotation = rotX * rotY * rotZ;

	cv::Mat translation_cross = cv::Mat::zeros(3, 3, CV_64F);
	translation_cross.at<double>(1, 2) = -calib.translation.at<double>(0, 0);
	translation_cross.at<double>(2, 1) = +calib.translation.at<double>(0, 0);
	translation_cross.at<double>(0, 2) = +calib.translation.at<double>(1, 0);
	translation_cross.at<double>(2, 0) = -calib.translation.at<double>(1, 0);
	translation_cross.at<double>(0, 1) = -calib.translation.at<double>(2, 0);
	translation_cross.at<double>(1, 0) = +calib.translation.at<double>(2, 0);

	calib.essential_matrix = translation_cross * calib.rotation;
	calib.fundamental_matrix = calib.right.camera_matrix.t().inv() * calib.essential_matrix * calib.left.camera_matrix.inv();
	calib.fundamental_matrix /= calib.fundamental_matrix.at<double>(2, 2);

	// For debugging
	//std::cout << "width: " << calib.image_size.width << std::endl;
	//std::cout << "height: " << calib.image_size.height << std::endl;
	//std::cout << "translation: " << calib.translation << std::endl;
	//std::cout << "rotation: " << calib.rotation << std::endl;
	//std::cout << "left.camera_matrix: " << calib.left.camera_matrix << std::endl;
	//std::cout << "left.dist_coeffs: " << calib.left.dist_coeffs << std::endl;
	//std::cout << "right.camera_matrix: " << calib.right.camera_matrix << std::endl;
	//std::cout << "right.dist_coeffs: " << calib.right.dist_coeffs << std::endl;
	//std::cout << "essential_matrix: " << calib.essential_matrix << std::endl;
	//std::cout << "fundamental_matrix: " << calib.fundamental_matrix << std::endl;

	calib.computeProjectionMatrices();

	return calib;
}

StereoCameraCalibration StereoCameraCalibration::getStereoCalibrationFromOpenCV(const std::string &filename)
{
	StereoCameraCalibration calib;
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
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

	// For debugging
	std::cout << "calib.left.camera_matrix: " << calib.left.camera_matrix << std::endl;
	std::cout << "calib.left.dist_coeffs: " << calib.left.dist_coeffs << std::endl;
	std::cout << "calib.right.camera_matrix: " << calib.right.camera_matrix << std::endl;
	std::cout << "calib.right.dist_coeffs: " << calib.right.dist_coeffs << std::endl;
	std::cout << "calib.projection_matrix_left: " << calib.projection_matrix_left << std::endl;
	std::cout << "calib.projection_matrix_right: " << calib.projection_matrix_right << std::endl;
	std::cout << "calib.essential_matrix: " << calib.essential_matrix << std::endl;
	std::cout << "calib.fundamental_matrix: " << calib.fundamental_matrix << std::endl;

	return calib;
}
StereoCameraCalibration StereoCameraCalibration::readStereoCalibration(const std::string &filename)
{
	return getStereoCalibrationFromOpenCV(filename);
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

Eigen::Matrix3d StereoCameraCalibration::getRotationEigen() const
{
  Eigen::Matrix3d rotation_eig;
  cv::cv2eigen(rotation, rotation_eig);
  return rotation_eig;
}

Eigen::Vector3d StereoCameraCalibration::getTranslationEigen() const
{
  Eigen::Vector3d translation_eig;
  cv::cv2eigen(translation, translation_eig);
  return translation_eig;
}


Eigen::Matrix4d StereoCameraCalibration::getLeftExtrinsicsEigen() const
{
	Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();
	return extrinsics;
}

Eigen::Matrix4d StereoCameraCalibration::getRightExtrinsicsEigen() const
{
	Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();
	extrinsics.block(0, 3, 3, 1) = getTranslationEigen();
	extrinsics.block(0, 0, 3, 3) = getRotationEigen();
	return extrinsics;
}

void StereoCameraCalibration::computeProjectionMatrices()
{
	cv::stereoRectify(
		left.camera_matrix, left.dist_coeffs,
		right.camera_matrix, right.dist_coeffs,
		image_size,
		rotation, translation,
		rectification_transform_left, rectification_transform_right,
		projection_matrix_left, projection_matrix_right,
		disparity_to_depth_map
		);
	// For debugging
	//std::cout << "projection_matrix_left: " << projection_matrix_left << std::endl;
	//std::cout << "projection_matrix_right: " << projection_matrix_right << std::endl;
}

}  // namespace stereo
}  // namespace ait

