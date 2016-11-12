//==================================================
// video_source.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 19, 2016
//==================================================

#include <ait/video/video_source.h>

namespace ait
{
namespace video
{

  VideoSource::Error::Error(const std::string &str)
  : std::runtime_error(str)
  {
  }

  VideoSource::~VideoSource()
  {
  }

  bool VideoSource::has_depth() const
  {
    return false;
  }

  bool VideoSource::has_stereo() const
  {
    return false;
  }

  bool VideoSource::retrieveLeft(cv::Mat *mat)
  {
    throw Error("Unable to grab right frame from mono camera");
  }

  bool VideoSource::retrieveRight(cv::Mat *mat)
  {
    throw Error("Unable to grab right frame from mono camera");
  }

  bool VideoSource::retrieveDepth(cv::Mat *mat)
  {
    throw Error("Unable to grab depth frame from non-depth camera");
  }

  bool VideoSource::retrieveDisparity(cv::Mat *mat)
  {
    throw Error("Unable to grab disparity frame from non-depth camera");
  }

  bool VideoSource::retrieveConfidence(cv::Mat *mat)
  {
    throw Error("Unable to grab confidence frame from non-depth camera");
  }

}
}
