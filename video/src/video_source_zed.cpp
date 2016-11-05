//==================================================
// video_source_zed.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 29, 2016
//==================================================

#include <ait/video/video_source_zed.h>
#include <algorithm>
#include <cuda_runtime_api.h>

namespace ait
{
namespace video
{

VideoSourceZED::VideoSourceZED(sl::zed::SENSING_MODE sensing_mode, bool compute_disparity, bool compute_measure, bool compute_pointcloud)
: camera_(nullptr),
  sensing_mode_(sensing_mode),
  compute_disparity_(compute_disparity),
  compute_measure_(compute_measure),
  compute_pointcloud_(compute_pointcloud),
  initialized_(false)
{
	init_params_.unit = sl::zed::UNIT::METER;
}

VideoSourceZED::~VideoSourceZED()
{
  close();
}

void VideoSourceZED::ensureInitialized() const
{
  if (!initialized_)
  {
    throw VideoSource::Error("Video source has not been initialized");
  }
}

const sl::zed::InitParams& VideoSourceZED::getInitParameters() const
{
  return init_params_;
}

sl::zed::InitParams& VideoSourceZED::getInitParameters()
{
  return init_params_;
}

ait::stereo::StereoCameraCalibration VideoSourceZED::getStereoCalibration() const
{
	const sl::zed::StereoParameters* stereo_params = camera_->getParameters();
	ait::stereo::StereoCameraCalibration calib;

	calib.image_size.width = camera_->getImageSize().width;
	calib.image_size.height = camera_->getImageSize().height;

	calib.left = ait::stereo::StereoCameraCalibration::getCameraCalibrationFromZED(stereo_params->LeftCam);
	calib.right = ait::stereo::StereoCameraCalibration::getCameraCalibrationFromZED(stereo_params->RightCam);

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

void VideoSourceZED::init()
{
  sl::zed::ERRCODE err = camera_->init(init_params_);
  if (err != sl::zed::SUCCESS)
  {
    std::cerr << "ZED init error code : " << sl::zed::errcode2str(err) << std::endl;
    throw Error("Unable to initialize ZED camera");
  }
  initialized_ = true;
}

void VideoSourceZED::open(sl::zed::ZEDResolution_mode mode)
{
  close();
  camera_ = new sl::zed::Camera(mode);
  init();
}

void VideoSourceZED::open(const std::string &svo_filename)
{
  close();
  camera_ = new sl::zed::Camera(svo_filename);
  init();
}

void VideoSourceZED::close()
{
  if (camera_ != nullptr)
  {
    if (initialized_)
    {
      // Grab frame to prevent error on destruction (Bug in SDK)
      camera_->grab(sensing_mode_, compute_measure_, compute_disparity_, compute_pointcloud_);
    }
    delete camera_;
    camera_ = nullptr;
    initialized_ = false;
  }
}

sl::zed::Camera* VideoSourceZED::getNativeCamera()
{
  return camera_;
}

const sl::zed::Camera* VideoSourceZED::getNativeCamera()const
{
  return camera_;
}

bool VideoSourceZED::loadParameters(const std::string &filename)
{
  return init_params_.load(filename);
}

void VideoSourceZED::saveParameters(const std::string &filename)
{
  init_params_.save(filename);
}

double VideoSourceZED::getFPS() const
{
  ensureInitialized();
  return camera_->getCurrentFPS();
}

bool VideoSourceZED::setFPS(double fps)
{
  ensureInitialized();
  return camera_->setFPS(static_cast<int>(fps));
}

bool VideoSourceZED::has_depth() const
{
  return compute_measure_;
}

bool VideoSourceZED::has_stereo() const
{
  return true;
}

int VideoSourceZED::getWidth() const
{
  ensureInitialized();
  return camera_->getImageSize().width;
}

int VideoSourceZED::getHeight() const
{
  ensureInitialized();
  return camera_->getImageSize().height;
}

bool VideoSourceZED::grab(bool block)
{
  ensureInitialized();
  if (block)
  {
    bool old_image = true;
    while (old_image)
    {
      old_image = camera_->grab(sensing_mode_, compute_measure_, compute_disparity_, compute_pointcloud_);
    }
    return true;
  }
  else
  {
    return camera_->grab(sensing_mode_, compute_measure_, compute_disparity_, compute_pointcloud_);
  }
}

void VideoSourceZED::copyZedMatToCvMat(const sl::zed::Mat &zed_mat, cv::Mat *cv_mat) const
{
  assert(zed_mat.getDataSize() == cv_mat->elemSize1());
  int data_size = zed_mat.width * zed_mat.height * zed_mat.channels * zed_mat.getDataSize();
  std::copy(zed_mat.data, zed_mat.data + data_size, cv_mat->data);
  // For debugging: Copy manually
//  for (int row=0; row < zed_mat.height; ++row)
//  {
//    int row_offset = row * zed_mat.width * zed_mat.channels;
//    for (int col=0; col < zed_mat.width; ++col)
//    {
//      int row_col_offset = row_offset + col * zed_mat.channels;
//      for (int channel=0; channel < zed_mat.channels; ++channel)
//      {
//        cv_mat->data[row_col_offset + channel] = zed_mat.data[row_col_offset + channel];
//      }
//    }
//  }
}

void VideoSourceZED::copyZedMatToCvMat(const sl::zed::Mat &zed_mat, cv_cuda::GpuMat *cv_mat) const
{
  assert(zed_mat.getDataSize() == cv_mat->elemSize1());
  int data_size = zed_mat.width * zed_mat.height * zed_mat.channels * zed_mat.getDataSize();
  cudaMemcpy(cv_mat->data, zed_mat.data, data_size, cudaMemcpyDeviceToDevice);
  // For debugging: Copy manually
//  for (int row=0; row < zed_mat.height; ++row)
//  {
//    int row_offset = row * zed_mat.width * zed_mat.channels;
//    for (int col=0; col < zed_mat.width; ++col)
//    {
//      int row_col_offset = row_offset + col * zed_mat.channels;
//      for (int channel=0; channel < zed_mat.channels; ++channel)
//      {
//        cv_mat->data[row_col_offset + channel] = zed_mat.data[row_col_offset + channel];
//      }
//    }
//  }
}

bool VideoSourceZED::retrieveSide(cv::Mat *cv_mat, sl::zed::SIDE side)
{
  ensureInitialized();
  sl::zed::Mat zed_mat = camera_->retrieveImage(side);
  if (cv_mat->empty())
  {
    *cv_mat = cv::Mat(zed_mat.height, zed_mat.width, CV_8UC4);
  }
  copyZedMatToCvMat(zed_mat, cv_mat);
  // ZED SDK function for converting ZED image to CV
  //  *cv_mat = sl::zed::slMat2cvMat(zed_mat);
  return true;
}

bool VideoSourceZED::retrieveNormalizedMeasure(cv::Mat *cv_mat, sl::zed::MEASURE measure)
{
	ensureInitialized();
	if (measure == sl::zed::MEASURE::DISPARITY)
	{
		if (!compute_disparity_)
		{
			throw Error("Disparity map was not computed");
		}
	}
	else if (!compute_measure_)
	{
		throw Error("Depth map measures were not computed");
	}

	sl::zed::Mat zed_mat = camera_->normalizeMeasure(measure);
	if (cv_mat->empty())
	{
		*cv_mat = cv::Mat(zed_mat.height, zed_mat.width, CV_8UC4);
	}
	copyZedMatToCvMat(zed_mat, cv_mat);
	return true;
}

bool VideoSourceZED::retrieveMeasure(cv::Mat *cv_mat, sl::zed::MEASURE measure)
{
  ensureInitialized();
  if (measure == sl::zed::MEASURE::DISPARITY)
  {
    if (!compute_disparity_)
    {
      throw Error("Disparity map was not computed");
    }
  }
  else if (measure == sl::zed::MEASURE::XYZ || measure == sl::zed::MEASURE::XYZRGBA)
  {
    if (!compute_pointcloud_)
    {
      throw Error("Point cloud was not computed");
    }
  }
  else if (!compute_measure_)
  {
    throw Error("Depth map measures were not computed");
  }

  sl::zed::Mat zed_mat = camera_->retrieveMeasure(measure);
  if (cv_mat->empty())
  {
    if (measure == sl::zed::MEASURE::XYZ || measure == sl::zed::MEASURE::XYZRGBA)
    {
      *cv_mat = cv::Mat(zed_mat.height, zed_mat.width, CV_32FC4);
    }
    else
    {
      *cv_mat = cv::Mat(zed_mat.height, zed_mat.width, CV_32FC1);
    }
  }
  copyZedMatToCvMat(zed_mat, cv_mat);
  return true;
}

bool VideoSourceZED::retrieveSideGpu(cv_cuda::GpuMat *cv_mat, sl::zed::SIDE side, bool copy)
{
  ensureInitialized();
  sl::zed::Mat zed_mat = camera_->retrieveImage_gpu(side);
  if (copy)
  {
    if (cv_mat->empty())
    {
      *cv_mat = cv_cuda::GpuMat(zed_mat.height, zed_mat.width, CV_8UC4);
    }
    copyZedMatToCvMat(zed_mat, cv_mat);
  }
  else
  {
    *cv_mat = cv_cuda::GpuMat(zed_mat.height, zed_mat.width, CV_8UC4, zed_mat.data);
  }
  // ZED SDK function for converting ZED image to CV
  //  *cv_mat = sl::zed::slMat2cvMat(zed_mat);
  return true;
}

bool VideoSourceZED::retrieveNormalizedMeasureGpu(cv_cuda::GpuMat *cv_mat, sl::zed::MEASURE measure, bool copy)
{
  ensureInitialized();
  if (measure == sl::zed::MEASURE::DISPARITY)
  {
    if (!compute_disparity_)
    {
      throw Error("Disparity map was not computed");
    }
  }
  else if (!compute_measure_)
  {
    throw Error("Depth map measures were not computed");
  }

  sl::zed::Mat zed_mat = camera_->normalizeMeasure_gpu(measure);
  if (copy)
  {
    if (cv_mat->empty())
    {
      *cv_mat = cv_cuda::GpuMat(zed_mat.height, zed_mat.width, CV_8UC4);
    }
    copyZedMatToCvMat(zed_mat, cv_mat);
  }
  else
  {
    *cv_mat = cv_cuda::GpuMat(zed_mat.height, zed_mat.width, CV_8UC4, zed_mat.data);
  }
  return true;
}

bool VideoSourceZED::retrieveMeasureGpu(cv_cuda::GpuMat *cv_mat, sl::zed::MEASURE measure, bool copy)
{
  ensureInitialized();
  if (measure == sl::zed::MEASURE::DISPARITY)
  {
    if (!compute_disparity_)
    {
      throw Error("Disparity map was not computed");
    }
  }
  else if (!compute_measure_)
  {
    throw Error("Depth map measures were not computed");
  }

  sl::zed::Mat zed_mat = camera_->retrieveMeasure_gpu(measure);
  int type;
  switch (zed_mat.channels)
  {
  case 1:
    type = CV_32FC1;
    break;
  case 2:
    type = CV_32FC2;
    break;
  case 3:
    type = CV_32FC3;
    break;
  case 4:
    type = CV_32FC4;
    break;
  default:
    throw std::runtime_error("ZED images with channels > 4 are not supported");
  }
  if (copy)
  {
    if (cv_mat->empty())
    {
      *cv_mat = cv_cuda::GpuMat(zed_mat.height, zed_mat.width, type);
    }
    copyZedMatToCvMat(zed_mat, cv_mat);
  }
  else
  {
    *cv_mat = cv_cuda::GpuMat(zed_mat.height, zed_mat.width, type, zed_mat.data);
  }
  return true;
}

bool VideoSourceZED::retrieveMono(cv::Mat *mat)
{
  return retrieveLeft(mat);
}

bool VideoSourceZED::retrieveLeft(cv::Mat *mat)
{
  return retrieveSide(mat, sl::zed::SIDE::LEFT);
}

bool VideoSourceZED::retrieveRight(cv::Mat *mat)
{
  return retrieveSide(mat, sl::zed::SIDE::RIGHT);
}

bool VideoSourceZED::retrieveDepth(cv::Mat *mat)
{
  return retrieveNormalizedMeasure(mat, sl::zed::MEASURE::DEPTH);
}

bool VideoSourceZED::retrieveDepthFloat(cv::Mat *mat)
{
	return retrieveMeasure(mat, sl::zed::MEASURE::DEPTH);
}

bool VideoSourceZED::retrieveDisparity(cv::Mat *mat)
{
	return retrieveNormalizedMeasure(mat, sl::zed::MEASURE::DISPARITY);
}

bool VideoSourceZED::retrieveDisparityFloat(cv::Mat *mat)
{
	return retrieveMeasure(mat, sl::zed::MEASURE::DISPARITY);
}

bool VideoSourceZED::retrieveConfidence(cv::Mat *mat)
{
	return retrieveNormalizedMeasure(mat, sl::zed::MEASURE::CONFIDENCE);
}

bool VideoSourceZED::retrieveConfidenceFloat(cv::Mat *mat)
{
	return retrieveMeasure(mat, sl::zed::MEASURE::CONFIDENCE);
}

bool VideoSourceZED::retrieveLeftGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveSideGpu(mat, sl::zed::SIDE::LEFT, copy);
}

bool VideoSourceZED::retrieveRightGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveSideGpu(mat, sl::zed::SIDE::RIGHT, copy);
}

bool VideoSourceZED::retrieveDepthGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveNormalizedMeasureGpu(mat, sl::zed::MEASURE::DEPTH, copy);
}

bool VideoSourceZED::retrieveDepthFloatGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveMeasureGpu(mat, sl::zed::MEASURE::DEPTH, copy);
}

bool VideoSourceZED::retrieveConfidenceGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveNormalizedMeasureGpu(mat, sl::zed::MEASURE::CONFIDENCE, copy);
}

bool VideoSourceZED::retrieveConfidenceFloatGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveMeasureGpu(mat, sl::zed::MEASURE::CONFIDENCE, copy);
}

bool VideoSourceZED::retrieveDisparityGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveNormalizedMeasureGpu(mat, sl::zed::MEASURE::DISPARITY, copy);
}

bool VideoSourceZED::retrieveDisparityFloatGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveMeasureGpu(mat, sl::zed::MEASURE::DISPARITY, copy);
}

bool VideoSourceZED::retrievePointCloud(cv::Mat *mat)
{
  return retrieveMeasure(mat, sl::zed::MEASURE::XYZRGBA);
}

bool VideoSourceZED::retrievePointCloudGpu(cv_cuda::GpuMat *mat, bool copy)
{
  return retrieveMeasureGpu(mat, sl::zed::MEASURE::XYZRGBA, copy);
}

}  // namespace video
}  // namespace ait
