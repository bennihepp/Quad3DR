#include <iostream>
#include <stdexcept>
#include <tclap/CmdLine.h>
#include <opencv2/opencv.hpp>
#include <video_source_opencv.h>


int main(int argc, char **argv)
{
  try
  {
    TCLAP::CmdLine cmd("Video capture tool", ' ', "0.1");
    TCLAP::ValueArg<int> device_arg("d", "device", "Device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> video_arg("v", "video", "Video file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> width_arg("", "width", "Frame width to capture", false, 0, "pixels", cmd);
    TCLAP::ValueArg<int> height_arg("", "height", "Frame height to capture", false, 0, "pixels", cmd);
    TCLAP::ValueArg<int> fps_arg("", "fps", "Frame-rate to capture", false, 0, "Hz", cmd);
    TCLAP::ValueArg<bool> show_arg("s", "show", "Show captured video", false, true, "boolean", cmd);

    cmd.parse(argc, argv);

    video::VideoSourceOpenCV video;

    if (video_arg.isSet())
    {
      video.open(video_arg.getValue());
    }
    else
    {
      video.open(device_arg.getValue());
    }
    if (width_arg.isSet())
    {
      if (!video.setFrameWidth(width_arg.getValue()))
      {
        std::cerr << "Setting frame width failed" << std::endl;
      }
    }
    if (height_arg.isSet())
    {
      if (!video.setFrameHeight(height_arg.getValue()))
      {
        std::cerr << "Setting frame height failed" << std::endl;
      }
    }
    if (fps_arg.isSet())
    {
      if (!video.setFPS(fps_arg.getValue()))
      {
        std::cerr << "Setting FPS failed" << std::endl;
      }
    }

    cv::Mat frame;
    int64_t start_ticks = cv::getTickCount();
    int frame_counter = 0;
    int key = -1;
    while (key != 27)
    {
      if (!video.grab())
      {
        throw std::runtime_error("Failed to grab next frame");
      }
      if (!video.retrieveMono(&frame))
      {
        throw std::runtime_error("Failed to retrieve mono frame");
      }

      if (show_arg.getValue())
      {
        cv::imshow("Camera", frame);
      }

      ++frame_counter;
      int64_t ticks = cv::getTickCount();
      double dt = double(ticks - start_ticks) / cv::getTickFrequency();
      double fps = frame_counter / dt;
      if (frame_counter > 30)
      {
//        std::cout << "Frame size: " << frame.cols << "x" << frame.rows << std::endl;
        std::cout << "Running with " << fps << std::endl;
        start_ticks = ticks;
        frame_counter = 0;
      }

      key = cv::waitKey(10);
    }
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
