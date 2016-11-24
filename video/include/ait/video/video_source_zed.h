//==================================================
// video_source_zed.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 29, 2016
//==================================================

#pragma once

// TODO
//#define OPENCV_3 1
//#define OPENCV_3_1 1

#include "video_source.h"
#include <ait/stereo/stereo_calibration.h>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>
#if OPENCV_3
  #include <opencv2/core/cuda.hpp>
#else
  #include <opencv2/gpu/gpu.hpp>
  #include <opencv2/gpu/gpumat.hpp>
#endif

namespace ait
{
namespace video
{

#if OPENCV_2_4
  namespace cv_cuda = cv::gpu;
#else
  namespace cv_cuda = cv::cuda;
#endif

// ZED camera modes:
//   width x height [fps] {mode}
//   1344 x 376 [15, 30, 60, 100] {sl::zed::VGA}
//   2560 x 720 [15, 30, 60] {sl::zed::HD720}
//   3840 x 1080 [15, 30] {sl::zed::HD1080}
//   4416 x 1242 [15] {sl::zed::HD2K}

class VideoSourceZED : public VideoSource
{
  sl::zed::Camera *camera_;
  sl::zed::InitParams init_params_;
  bool initialized_;

  sl::zed::SENSING_MODE sensing_mode_;
  bool compute_measure_;
  bool compute_disparity_;
  bool compute_pointcloud_;

  void ensureInitialized() const;

  void init();
  void copyZedMatToCvMat(const sl::zed::Mat &zed_mat, cv::Mat *cv_mat) const;
  void copyZedMatToCvMat(const sl::zed::Mat &zed_mat, cv_cuda::GpuMat *cv_mat) const;
  bool retrieveSide(cv::Mat *mat, sl::zed::SIDE side);
  bool retrieveNormalizedMeasure(cv::Mat *mat, sl::zed::MEASURE measure);
  bool retrieveMeasure(cv::Mat *mat, sl::zed::MEASURE measure);
  bool retrieveSideGpu(cv_cuda::GpuMat *cv_mat, sl::zed::SIDE side, bool copy=true);
  bool retrieveNormalizedMeasureGpu(cv_cuda::GpuMat *mat, sl::zed::MEASURE measure, bool copy=true);
  bool retrieveMeasureGpu(cv_cuda::GpuMat *mat, sl::zed::MEASURE measure, bool copy=true);

public:
  VideoSourceZED(
      sl::zed::SENSING_MODE sensing_mode=sl::zed::STANDARD,
      bool compute_disparity=true,
      bool compute_measure=true,
      bool compute_pointcloud=false);
  VideoSourceZED(const VideoSourceZED &video) = delete;
  ~VideoSourceZED() override;

  const sl::zed::InitParams& getInitParameters() const;
  sl::zed::InitParams& getInitParameters();

  ait::stereo::StereoCameraCalibration getStereoCalibration() const;

  void open(sl::zed::ZEDResolution_mode mode=sl::zed::HD720);
  void open(const std::string &svo_filename);
  void close();

  sl::zed::Camera* getNativeCamera();
  const sl::zed::Camera* getNativeCamera() const;

  bool loadParameters(const std::string &filename);
  void saveParameters(const std::string &filename);

  double getFPS() const;
  bool setFPS(double fps);

  bool has_depth() const override;
  bool has_stereo() const override;

  int getWidth() const override;
  int getHeight() const override;

  bool grab(bool block=true) override;
  bool retrieveMono(cv::Mat *mat) override;
  bool retrieveLeft(cv::Mat *mat) override;
  bool retrieveRight(cv::Mat *mat) override;
  bool retrieveDepth(cv::Mat *mat) override;
  bool retrieveDepthFloat(cv::Mat *mat);
  bool retrieveDisparity(cv::Mat *mat) override;
  bool retrieveDisparityFloat(cv::Mat *mat);
  bool retrieveConfidence(cv::Mat *mat) override;
  bool retrieveConfidenceFloat(cv::Mat *mat);

  bool retrieveLeftGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveRightGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveDepthGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveDepthFloatGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveDisparityGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveDisparityFloatGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveConfidenceGpu(cv_cuda::GpuMat *mat, bool copy=true);
  bool retrieveConfidenceFloatGpu(cv_cuda::GpuMat *mat, bool copy=true);

  bool retrievePointCloud(cv::Mat *mat);
  bool retrievePointCloudGpu(cv_cuda::GpuMat *mat, bool copy=true);

  template <typename T, typename U>
  bool retrievePointCloud(std::vector<cv::Point3_<T>> *points, std::vector<cv::Point3_<U>> *colors=nullptr)
  {
    cv::Mat pc_mat;
    if (!retrievePointCloud(&pc_mat))
    {
      return false;
    }
    points->clear();
    points->reserve(pc_mat.rows * pc_mat.cols);
    if (colors != nullptr)
    {
      colors->clear();
      colors->reserve(pc_mat.rows * pc_mat.cols);
    }
    for (int i = 0; i < pc_mat.rows; ++i)
    {
      for (int j = 0; j < pc_mat.cols; ++j)
      {
        const float *XYZ = pc_mat.at<const float[4]>(i, j);
        T x = static_cast<T>(XYZ[0]);
        T y = static_cast<T>(XYZ[1]);
        T z = static_cast<T>(XYZ[2]);
        points->push_back(std::move(cv::Point3_<T>(x, y, z)));
        if (colors != nullptr)
        {
          const uchar *RGB = reinterpret_cast<const uchar*>(&XYZ[3]);
          U r = static_cast<U>(RGB[0]);
          U g = static_cast<U>(RGB[1]);
          U b = static_cast<U>(RGB[2]);
          colors->push_back(std::move(cv::Point3_<U>(r, g, b)));
        }
      }
    }
  }
};

}  // namespace video
}  // namespace ait
