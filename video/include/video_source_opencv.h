//==================================================
// video_source_opencv.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 29, 2016
//==================================================

#pragma once

#include <video_source.h>

namespace video
{

class VideoSourceOpenCV : public VideoSource
{
  cv::VideoCapture capture_;

  void ensureOpened() const;

public:
  VideoSourceOpenCV();
  virtual ~VideoSourceOpenCV() override;

  void open(int device);
  void open(const std::string &filename);
  void close();

  bool setFrameWidth(int frame_width);
  bool setFrameHeight(int frame_height);
  double getFPS() const;
  bool setFPS(double fps);

  int getWidth() const override;
  int getHeight() const override;

  bool grab(bool block=true) override;
  bool retrieveMono(cv::Mat *mat) override;
};

} /* namespace video */
