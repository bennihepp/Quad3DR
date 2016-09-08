//==================================================
// video_source_opencv.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 29, 2016
//==================================================

#include <video_source_opencv.h>

namespace video
{

VideoSourceOpenCV::VideoSourceOpenCV()
{
}

VideoSourceOpenCV::~VideoSourceOpenCV()
{
  close();
}

void VideoSourceOpenCV::ensureOpened() const
{
  if (!capture_.isOpened())
  {
    throw VideoSource::Error("Video source has not been initialized");
  }
}

void VideoSourceOpenCV::open(int device)
{
  close();
  if (!capture_.open(device))
  {
    throw VideoSource::Error("Failed to open video device");
  }
}

void VideoSourceOpenCV::open(const std::string &filename)
{
  close();
  if (!capture_.open(filename))
  {
    throw VideoSource::Error("Failed to open video file");
  }
}

void VideoSourceOpenCV::close()
{
  if (capture_.isOpened())
  {
    capture_.release();
  }
}

bool VideoSourceOpenCV::setFrameWidth(int frame_width)
{
  ensureOpened();
  return capture_.set(CV_CAP_PROP_FRAME_WIDTH, frame_width);
}
bool VideoSourceOpenCV::setFrameHeight(int frame_height)
{
  ensureOpened();
  return capture_.set(CV_CAP_PROP_FRAME_HEIGHT, frame_height);
}

double VideoSourceOpenCV::getFPS() const
{
  ensureOpened();
  return const_cast<cv::VideoCapture*>(&capture_)->get(CV_CAP_PROP_FPS);
}

bool VideoSourceOpenCV::setFPS(double fps)
{
  ensureOpened();
  return capture_.set(CV_CAP_PROP_FPS, fps);
}

int VideoSourceOpenCV::getWidth() const
{
  ensureOpened();
  // const_cast is required for OpenCV 2.4
  return const_cast<cv::VideoCapture*>(&capture_)->get(CV_CAP_PROP_FRAME_WIDTH);
}

int VideoSourceOpenCV::getHeight() const
{
  ensureOpened();
  // const_cast is required for OpenCV 2.4
  return const_cast<cv::VideoCapture*>(&capture_)->get(CV_CAP_PROP_FRAME_HEIGHT);
}

bool VideoSourceOpenCV::grab(bool block)
{
  ensureOpened();
  return capture_.grab();
}

bool VideoSourceOpenCV::retrieveMono(cv::Mat *mat)
{
  ensureOpened();
  return capture_.retrieve(*mat);
}

} /* namespace video */
