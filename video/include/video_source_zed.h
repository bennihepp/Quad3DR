//==================================================
// video_source_zed.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 29, 2016
//==================================================

#pragma once

#include "video_source.h"
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

namespace video
{

// ZED camera modes:
//   width x height [fps] {mode}
//   1344 x 376 [15, 30, 60, 100] {sl::zed::VGA}
//   2560 x 720 [15, 30, 60] {sl::zed::HD720}
//   3840 x 1080 [15, 30] {sl::zed::HD1080}
//   4416 x 1242 [15] {sl::zed::HD2K}

class VideoSourceZED : public VideoSource
{
  const bool compute_pointcloud_ = false;

  sl::zed::Camera *camera_;
  sl::zed::InitParams init_params_;
  bool initialized_;

  sl::zed::SENSING_MODE sensing_mode_;
  bool compute_measure_;
  bool compute_disparity_;

  void ensureInitialized() const;

  void init();
  void copyZedMatToCvMat(const sl::zed::Mat &zed_mat, cv::Mat *cv_mat) const;
  bool retrieveSide(cv::Mat *mat, sl::zed::SIDE side);
  bool retrieveNormalizedMeasure(cv::Mat *mat, sl::zed::MEASURE measure);
  bool retrieveMeasure(cv::Mat *mat, sl::zed::MEASURE measure);

public:
  VideoSourceZED(
      sl::zed::SENSING_MODE sensing_mode=sl::zed::STANDARD,
      bool compute_disparity=true,
      bool compute_measure=true);
  VideoSourceZED(const VideoSourceZED &video) = delete;
  ~VideoSourceZED() override;

  const sl::zed::InitParams& getInitParameters() const;
  sl::zed::InitParams& getInitParameters();

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
};

} /* namespace video */
