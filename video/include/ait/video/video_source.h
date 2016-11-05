//==================================================
// video_source.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 19, 2016
//==================================================

#include <string>
#include <opencv2/opencv.hpp>

#pragma once

namespace ait
{
namespace video
{

class VideoSource
{
public:

  class Error : public std::runtime_error
  {
  public:
    Error(const std::string &str);
  };

  virtual ~VideoSource();

  virtual bool has_depth() const;
  virtual bool has_stereo() const;

  virtual int getWidth() const = 0;
  virtual int getHeight() const = 0;

  virtual bool grab(bool block=true) = 0;
  virtual bool retrieveMono(cv::Mat *mat) = 0;
  virtual bool retrieveLeft(cv::Mat *mat);
  virtual bool retrieveRight(cv::Mat *mat);
  virtual bool retrieveDepth(cv::Mat *mat);
  virtual bool retrieveDisparity(cv::Mat *mat);
  virtual bool retrieveConfidence(cv::Mat *mat);
};

}  // namespace video
}  // namespace ait
