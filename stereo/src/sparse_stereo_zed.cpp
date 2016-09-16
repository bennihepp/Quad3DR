//==================================================
// sparse_stereo_zed.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 2, 2016
//==================================================

#include <iostream>
#include <stdexcept>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <tclap/CmdLine.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#if OPENCV_3
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/sfm.hpp>
  #include <opencv2/cudafeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
  #include <opencv2/cudaimgproc.hpp>
#else
  #include <opencv2/gpu/gpu.hpp>
  #include <opencv2/gpu/gpumat.hpp>
#endif

#include <video_source_zed.h>
#include <sparse_stereo_matcher.h>
#include <dense_stereo_matcher.h>
#include <utilities.h>
#include <mLibInclude.h>


#if OPENCV_2_4
  namespace cv_cuda = cv::gpu;
  using HostMemType = cv::gpu::CudaMem;
  const int HOST_MEM_ALLOC_TYPE = HostMemType::ALLOC_ZEROCOPY;
#else
  namespace cv_cuda = cv::cuda;
  using HostMemType = cv::cuda::HostMem;
  const HostMemType::AllocType HOST_MEM_ALLOC_TYPE = HostMemType::PAGE_LOCKED;
#endif

template <typename T>
struct LockedQueue
{
  std::mutex mutex;
  std::deque<T> queue;
};

struct StereoAndDepthImage
{
  HostMemType left_mem;
  HostMemType right_mem;
  HostMemType left_mem_color;
  HostMemType right_mem_color;
  HostMemType depth_mem;

  cv::Mat left_img;
  cv::Mat right_img;
  cv::Mat left_img_color;
  cv::Mat right_img_color;
  cv::Mat depth_img;
  std::vector<cv::Point3d> point_cloud_points;
  std::vector<cv::Point3d> point_cloud_colors;

  StereoAndDepthImage(int rows, int cols, int type, int color_type, int depth_type)
  : left_mem(rows, cols, type, HOST_MEM_ALLOC_TYPE),
    right_mem(rows, cols, type, HOST_MEM_ALLOC_TYPE),
    left_mem_color(rows, cols, color_type, HOST_MEM_ALLOC_TYPE),
    right_mem_color(rows, cols, color_type, HOST_MEM_ALLOC_TYPE),
    depth_mem(rows, cols, depth_type, HOST_MEM_ALLOC_TYPE)
  {
    left_img = left_mem.createMatHeader();
    right_img = right_mem.createMatHeader();
    left_img_color = left_mem_color.createMatHeader();
    right_img_color = right_mem_color.createMatHeader();
    depth_img = depth_mem.createMatHeader();
  }
};

template <typename T>
void savePointCloudToOff(const std::string &filename, const ml::PointCloud<T> &pc, bool save_color=true)
{
  std::ofstream out(filename);
  if (!out.is_open())
  {
    throw std::runtime_error("Unable to open output file: " + filename);
  }
  out << "COFF" << std::endl;
  out << pc.m_points.size() << " " << 0 << " " << 0 << std::endl;
  for (int i = 0; i < pc.m_points.size(); ++i)
  {
    const ml::vec3<T> &p = pc.m_points[i];
    out << p.x << " " << p.y << " " << p.z;
    if (save_color)
    {
      const ml::vec4<T> &c = pc.m_colors[i];
      out << " " << c.r << " " << c.g << " " << c.b << " " << 255;
    }
    else
    {
      ml::vec4<T> c(1, 1, 1, 1);
      out << " " << c.r << " " << c.g << " " << c.b << " " << 255;
    }
    out << std::endl;
  }
  out.close();
}

template <typename T>
struct SparseStereoThreadData
{
  cv::Ptr<T> matcher_ptr;
  LockedQueue<StereoAndDepthImage> images_queue;
  std::condition_variable queue_filled_condition;
  bool stop = false;
};

template <typename T>
std::vector<cv::Point3d> sparseStereoMatching(
    const cv::Ptr<T> &matcher_ptr,
    cv::InputArray left_input_img, cv::InputArray right_input_img,
    std::vector<cv::Point2d> *image_points=nullptr)
{
  stereo::Timer  timer;
  bool verbose = false;
  std::vector<cv::Point3d> points_3d = matcher_ptr->match(left_input_img, right_input_img, image_points, verbose);
  timer.stopAndPrintTiming("sparse stereo matching");
  return points_3d;
}

template <typename T>
void runSparseStereoMatching(SparseStereoThreadData<T> &thread_data)
{
  const cv::Ptr<T> &matcher_ptr = thread_data.matcher_ptr;
  LockedQueue<StereoAndDepthImage> &images_queue = thread_data.images_queue;
  std::condition_variable &queue_filled_condition = thread_data.queue_filled_condition;
  bool &stop = thread_data.stop;

  int64_t start_ticks = cv::getTickCount();
  int frame_counter = 0;
  ml::PointCloud<float> dense_pc;
  ml::PointCloud<float> sparse_pc;
  do
  {
    std::unique_lock<std::mutex> lock(images_queue.mutex);
    if (images_queue.queue.empty())
    {
      queue_filled_condition.wait_for(lock, std::chrono::milliseconds(100));
    }
    if (!images_queue.queue.empty())
    {
      stereo::ProfilingTimer prof_timer;
      StereoAndDepthImage images(std::move(images_queue.queue.back()));
      images_queue.queue.pop_back();
      lock.unlock();
      prof_timer.stopAndPrintTiming("Popping from queue and moving");
      try
      {
        std::vector<cv::Point2d> image_points;
        std::vector<cv::Point3d> points_3d = sparseStereoMatching(matcher_ptr, images.left_img, images.right_img, &image_points);
        for (int i = 0; i < points_3d.size(); ++i)
        {
          const cv::Point3d &p = points_3d[i];
          const cv::Point2d &point_2d = image_points[i];
          const uchar *cv_color = images.left_img_color.at<uchar[4]>(point_2d);
          ml::vec4f c(cv_color[0], cv_color[1], cv_color[2], 255);
          // Scale sparse points to millimeters
          float point_scale = 1 / 1000.0f;
          sparse_pc.m_points.push_back(ml::vec3f(p.x * point_scale, p.y * point_scale, p.z * point_scale));
          float color_scale = 1 / 255.0f;
          sparse_pc.m_colors.push_back(ml::vec4f(c.x * color_scale, c.y * color_scale, c.z * color_scale, c.w * color_scale));
        }

        dense_pc.clear();
        for (int i = 0; i < images.point_cloud_points.size(); ++i)
        {
          const cv::Point3d &p = images.point_cloud_points[i];
          const cv::Point3d &c = images.point_cloud_colors[i];
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          {
            continue;
          }
          float point_scale = 1.0f;
          dense_pc.m_points.push_back(ml::vec3f(p.x * point_scale, p.y * point_scale, p.z * point_scale));
          float color_scale = 1 / 255.0f;
          dense_pc.m_colors.push_back(ml::vec4f(c.x * color_scale, c.y * color_scale, c.z * color_scale, 1.0f));
        }

//      for (int i = 0; i < points_3d.size(); ++i)
//      {
//        double depth = points_3d[i].z / 1000.0;
//        int x = static_cast<int>(image_points[i].x);
//        int y = static_cast<int>(image_points[i].y);
//        float depth_zed = images.depth_img.at<float>(y, x);
//        if (!std::isnan(depth_zed))
//        {
//          double diff = depth - depth_zed;
//          std::cout << "diff=" << diff << ", depth=" << depth << std::endl;
//        }
//      }
      }
      catch (const typename T::Error &err)
      {
        std::cerr << "Exception during stereo matching: " << err.what() << std::endl;
      }

      // Computing frame rate
      ++frame_counter;
      int64_t ticks = cv::getTickCount();
      double dt = double(ticks - start_ticks) / cv::getTickFrequency();
      double fps = frame_counter / dt;
      if (frame_counter % 10 == 0)
      {
//        std::cout << "Frame size: " << frame.cols << "x" << frame.rows << std::endl;
        std::cout << "Thread running with " << fps << std::endl;
        if (frame_counter > 30)
        {
          frame_counter = 0;
          start_ticks = ticks;
        }
      }

    }  // if (!images_queue.queue.empty())
  } while (!stop);
  ml::PointCloudIO<float>::saveToFile("sparse_points.ply", sparse_pc);
  ml::PointCloudIO<float>::saveToFile("dense_points.ply", dense_pc);
  savePointCloudToOff("sparse_points.off", sparse_pc);
  savePointCloudToOff("dense_points.off", dense_pc);
}

void convertToGrayscaleGpu(const cv_cuda::GpuMat &img, cv_cuda::GpuMat *img_grayscale_ptr, cv_cuda::Stream &stream)
{
  int code;
  if (img.channels() == 3)
  {
    code = CV_RGB2GRAY;
  }
  else if (img.channels() == 4)
  {
    code = CV_RGBA2GRAY;
  }
  else
  {
    throw std::runtime_error("Cannot convert image to grayscale");
  }
  int destination_channels = 0;
  cv_cuda::cvtColor(img, *img_grayscale_ptr, code, destination_channels, stream);
}

int main(int argc, char **argv)
{
  try
  {
    TCLAP::CmdLine cmd("Sparse stereo matching ZED", ' ', "0.1");
    TCLAP::ValueArg<int> device_arg("d", "device", "Device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> video_arg("v", "video", "Video device file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<std::string> svo_arg("", "svo", "SVO file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> mode_arg("", "mode", "ZED Resolution mode", false, 2, "mode", cmd);
    TCLAP::ValueArg<double> fps_arg("", "fps", "Frame-rate to capture", false, 0, "Hz", cmd);
    TCLAP::SwitchArg hide_arg("", "hide", "Hide captured video", cmd, false);
    TCLAP::ValueArg<int> draw_period_arg("", "draw-period", "Period of drawing frames", false, 5, "integer", cmd);
    TCLAP::ValueArg<std::string> calib_arg("c", "calib", "Calibration file to use", false, "camera_calibration_stereo.yml", "filename", cmd);
    TCLAP::ValueArg<std::string> zed_params_arg("", "zed-params", "ZED parameter file", false, "", "filename", cmd);

    cmd.parse(argc, argv);

    // TODO
    cv_cuda::printCudaDeviceInfo(cv_cuda::getDevice());
//    cv_cuda::printShortCudaDeviceInfo(cv_cuda::getDevice());

    video::VideoSourceZED video(sl::zed::STANDARD, true, true, true);

    if (zed_params_arg.isSet())
    {
      video.getInitParameters().load(zed_params_arg.getValue());
    }
//    video.getInitParameters().disableSelfCalib = false;
    if (svo_arg.isSet())
    {
      video.open(svo_arg.getValue());
    }
    else
    {
      video.open(static_cast<sl::zed::ZEDResolution_mode>(mode_arg.getValue()));
    }
    video.getInitParameters().save("MyParam");

    if (fps_arg.isSet())
    {
      if (video.setFPS(fps_arg.getValue()))
      {
        throw std::runtime_error("Unable to set ZED framerate");
      }
    }
    std::cout << "ZED framerate: " << video.getFPS() << std::endl;

    stereo::StereoCameraCalibration calib = stereo::Utilities::readStereoCalibration(calib_arg.getValue());

#if OPENCV_2_4
    // FREAK
    using DetectorType = cv::FastFeatureDetector;
    using DescriptorType = cv::FREAK;
    cv::Ptr<DetectorType> detector = cv::makePtr<DetectorType>(20, true);
    cv::Ptr<DetectorType> detector_2 = cv::makePtr<DetectorType>(20, true);
//    cv::Ptr<DescriptorType> descriptor_computer = cv::makePtr<DescriptorType>(true, true, 22, 4);
//    cv::Ptr<DescriptorType> descriptor_computer_2 = cv::makePtr<DescriptorType>(true, true, 22, 4);
//    cv::Ptr<DescriptorType> descriptor_computer = cv::makePtr<DescriptorType>(true, true, 16, 3);
//    cv::Ptr<DescriptorType> descriptor_computer_2 = cv::makePtr<DescriptorType>(true, true, 16, 3);
    cv::Ptr<DescriptorType> descriptor_computer = cv::makePtr<DescriptorType>(true, true, 14, 2);
    cv::Ptr<DescriptorType> descriptor_computer_2 = cv::makePtr<DescriptorType>(true, true, 14, 2);

    // ORB
//    using DetectorType = cv::ORB;
//    using DescriptorType = cv::ORB;
//    cv::Ptr<DetectorType> detector = cv::makePtr<DetectorType>();
//    cv::Ptr<DescriptorType> descriptor_computer = cv::makePtr<DescriptorType>();

    // ORB CUDA
//    using FeatureType = cv::gpu::ORB_GPU;
//    cv::Ptr<FeatureType> feature_computer = cv::makePtr<FeatureType>();

    // Create feature detector
//    cv::Ptr<DetectorType> detector_2 = detector;
//    cv::Ptr<DescriptorType> descriptor_computer_2 = descriptor_computer;
    using FeatureDetectorType = stereo::FeatureDetectorOpenCV<DetectorType, DescriptorType>;
    cv::Ptr<FeatureDetectorType> feature_detector = cv::makePtr<FeatureDetectorType>(detector, detector_2, descriptor_computer, descriptor_computer_2);
//      using FeatureDetectorType = stereo::FeatureDetectorOpenCVCuda<FeatureType>;
//      cv::Ptr<FeatureDetectorType> feature_detector = cv::makePtr<FeatureDetectorType>(feature_computer);

    // Create sparse matcher
    using SparseStereoMatcherType = stereo::SparseStereoMatcher<FeatureDetectorType>;
    SparseStereoMatcherType matcher(feature_detector, calib);

    // ORB and FREAK
    //    matcher.setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
    matcher.setMatchNorm(cv::NORM_HAMMING);
#else
    // SURF CUDA
//    using FeatureType = cv::cuda::SURF_CUDA;
//    cv::Ptr<FeatureType> feature_computer = cv::makePtr<FeatureType>();

    // SURF
//    using DetectorType = cv::xfeatures2d::SURF;
//    using DescriptorType = cv::xfeatures2d::SURF;
//    const int hessian_threshold = 400;
//    cv::Ptr<DescriptorType> detector = FeatureType::create(hessian_threshold);

    // ORB
    using DetectorType = cv::ORB;
    using DescriptorType = cv::ORB;
    const int num_features = 2000;
    cv::Ptr<DetectorType> detector = DetectorType::create();
    cv::Ptr<DescriptorType> descriptor_computer = detector;

    // ORB CUDA
//    using FeatureType = cv::cuda::ORB;
//    cv::Ptr<FeatureType> feature_computer = FeatureType::create(500, 1.2f, 8);

    // FREAK
//    using DetectorType = cv::FastFeatureDetector;
//    using DescriptorType = cv::xfeatures2d::FREAK;
//    cv::Ptr<DetectorType> detector = DetectorType::create(20, true);
//    cv::Ptr<DetectorType> detector_2 = DetectorType::create(20, true);
//    cv::Ptr<DescriptorType> descriptor_computer = DescriptorType::create();
//    cv::Ptr<DescriptorType> descriptor_computer_2 = DescriptorType::create();

    // Create feature detector
    cv::Ptr<DetectorType> detector_2 = detector;
    cv::Ptr<DescriptorType> descriptor_computer_2 = descriptor_computer;
    using FeatureDetectorType = stereo::FeatureDetectorOpenCV<DetectorType, DescriptorType>;
    cv::Ptr<FeatureDetectorType> feature_detector = cv::makePtr<FeatureDetectorType>(detector, detector_2, descriptor_computer, descriptor_computer_2);
//    using FeatureDetectorType = stereo::FeatureDetectorOpenCVSurfCuda<FeatureType>;
//    cv::Ptr<FeatureDetectorType> feature_detector = cv::makePtr<FeatureDetectorType>(feature_computer);
//      using FeatureDetectorType = stereo::FeatureDetectorOpenCVCuda<FeatureType>;
//      cv::Ptr<FeatureDetectorType> feature_detector = cv::makePtr<FeatureDetectorType>(feature_computer);

    // Create sparse matcher
    using SparseStereoMatcherType = stereo::SparseStereoMatcher<FeatureDetectorType>;
    cv::Ptr<SparseStereoMatcherType> matcher_ptr = cv::makePtr<SparseStereoMatcherType>(feature_detector, calib);

    // ORB
//    matcher_ptr->setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
//    matcher_ptr->setMatchNorm(cv::NORM_HAMMING2);

    // FREAK
    //    matcher->setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
    matcher_ptr->setMatchNorm(cv::NORM_HAMMING);
#endif

    // General
//    feature_detector->setMaxNumOfKeypoints(500);
    matcher_ptr->setRatioTestThreshold(1.0);
    matcher_ptr->setEpipolarConstraintThreshold(1.0);

    SparseStereoThreadData<SparseStereoMatcherType> sparse_stereo_thread_data;
    sparse_stereo_thread_data.matcher_ptr = matcher_ptr;
    // TODO
//    std::thread sparse_matching_thread([&] ()
//    {
//      runSparseStereoMatching(sparse_stereo_thread_data);
//    });

    int width = video.getWidth();
    int height = video.getHeight();

    cv::Size display_size(width, height);
    cv_cuda::GpuMat left_img_gpu;
    cv_cuda::GpuMat right_img_gpu;
    cv_cuda::GpuMat depth_img_gpu;
    cv_cuda::GpuMat depth_float_img_gpu;
    cv_cuda::GpuMat disparity_img_gpu;
    cv_cuda::GpuMat confidence_img_gpu;
    std::vector<cv::Point3d> pc_points;
    std::vector<cv::Point3d> pc_colors;

    bool opengl_supported = false;
    if (!hide_arg.getValue())
    {
      try
      {
        cv::namedWindow("left", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        cv::namedWindow("right", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        cv::namedWindow("depth", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        opengl_supported = true;
      }
      catch (const cv::Exception &err)
      {
        cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
      }
    }

    cv_cuda::Stream stream;
    int64_t start_ticks = cv::getTickCount();
    int frame_counter = 0;
    int key = -1;
    while (key != 27)
    {
      if (!video.grab())
      {
        throw std::runtime_error("Failed to grab frame from camera");
      }
      video.retrieveLeftGpu(&left_img_gpu, false);
      video.retrieveRightGpu(&right_img_gpu, false);
      video.retrieveDepthGpu(&depth_img_gpu, false);
      video.retrieveDepthFloatGpu(&depth_float_img_gpu, false);
      video.retrieveDisparityFloatGpu(&disparity_img_gpu, false);
      video.retrieveConfidenceFloatGpu(&confidence_img_gpu, false);
      // TODO
      video.retrievePointCloud(&pc_points, &pc_colors);
//      sl::writePointCloudAs(video.getNativeCamera(), sl::POINT_CLOUD_FORMAT::PLY, "dense_zed.ply", true, false);

      if (!hide_arg.getValue() && frame_counter % draw_period_arg.getValue() == 0)
      {
        if (opengl_supported)
        {
          cv::imshow("left", left_img_gpu);
          cv::imshow("right", right_img_gpu);
          cv::imshow("depth", depth_img_gpu);
        }
        else
        {
          cv::Mat left_img;
          cv::Mat right_img;
          cv::Mat depth_img;
#if OPENCV_2_4
          stream.enqueueDownload(left_img_gpu, left_img);
          stream.enqueueDownload(right_img_gpu, right_img);
          stream.enqueueDownload(depth_img_gpu, depth_img);
#else
          left_img_gpu.download(left_img, stream);
          right_img_gpu.download(right_img, stream);
          depth_img_gpu.download(depth_img, stream);
#endif
          stream.waitForCompletion();
          cv::imshow("left", left_img);
          cv::imshow("right", right_img);
          cv::imshow("depth", depth_img);
        }
      }

      stereo::ProfilingTimer timer;
      // Convert stereo image to grayscale
      cv_cuda::GpuMat left_img_grayscale_gpu;
      cv_cuda::GpuMat right_img_grayscale_gpu;
      if (left_img_gpu.channels() != 1 || right_img_gpu.channels() != 1)
      {
        timer = stereo::ProfilingTimer();
        convertToGrayscaleGpu(left_img_gpu, &left_img_grayscale_gpu, stream);
        convertToGrayscaleGpu(right_img_gpu, &right_img_grayscale_gpu, stream);
        timer.stopAndPrintTiming("Converting images to grayscale");
      }
      else
      {
        left_img_grayscale_gpu = left_img_gpu;
        right_img_grayscale_gpu = right_img_gpu;
      }

      // Download stereo image to Host memory
      StereoAndDepthImage images(
          left_img_grayscale_gpu.rows, left_img_grayscale_gpu.cols,
          left_img_grayscale_gpu.type(),
          left_img_gpu.type(),
          depth_float_img_gpu.type());
      timer = stereo::ProfilingTimer();
#if OPENCV_2_4
      stream.enqueueDownload(left_img_grayscale_gpu, images.left_img);
      stream.enqueueDownload(right_img_grayscale_gpu, images.right_img);
      stream.enqueueDownload(depth_float_img_gpu, images.depth_img);
      stream.enqueueDownload(left_img_gpu, images.left_img_color);
      stream.enqueueDownload(right_img_gpu, images.right_img_color);
#else
      left_img_grayscale_gpu.download(images.left_img, stream);
      right_img_grayscale_gpu.download(images.right_img, stream);
      depth_float_img_gpu.download(images.depth_img, stream);
      left_img_gpu.download(images.left_img_color, stream);
      right_img_gpu.download(images.right_img_color, stream);
#endif
      images.point_cloud_points = std::move(pc_points);
      images.point_cloud_colors = std::move(pc_colors);
      stream.waitForCompletion();
      timer.stopAndPrintTiming("Downloading images from GPU");
//
      // Push stereo image to queue and notify sparse matcher thread
      timer = stereo::ProfilingTimer();
      {
        std::lock_guard<std::mutex> lock(sparse_stereo_thread_data.images_queue.mutex);
        sparse_stereo_thread_data.images_queue.queue.clear();
        sparse_stereo_thread_data.images_queue.queue.push_front(std::move(images));
      }
      sparse_stereo_thread_data.queue_filled_condition.notify_one();
      timer.stopAndPrintTiming("Pushing to queue and notifying");

      // TODO
      sparse_stereo_thread_data.stop = true;
      runSparseStereoMatching(sparse_stereo_thread_data);

      // Computing frame rate
      ++frame_counter;
      int64_t ticks = cv::getTickCount();
      double dt = double(ticks - start_ticks) / cv::getTickFrequency();
      double fps = frame_counter / dt;
      if (frame_counter % 10 == 0)
      {
//        std::cout << "Frame size: " << frame.cols << "x" << frame.rows << std::endl;
        std::cout << "Running with " << fps << std::endl;
        if (frame_counter > 30)
        {
          frame_counter = 0;
          start_ticks = ticks;
        }
      }

      if (!hide_arg.getValue())
      {
        key = cv::waitKey(10);
      }
    }  // while (key != 27)

    sparse_stereo_thread_data.stop = true;
    // TODO
//    sparse_matching_thread.join();
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
