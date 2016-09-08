#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>
#include <chrono>
#include <thread>
#include <tclap/CmdLine.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <video_source_opencv.h>
#include <video_source_zed.h>

struct AppSrcData
{
  std::function<bool (cv::Mat*)> retrieve_frame_fn;
  GstClockTime timestamp;
  std::chrono::time_point<std::chrono::system_clock> start_time;
  double framerate;
  bool negotiated;
};

void pushNewFrame(GstAppSrc *appsrc, AppSrcData *data)
{
  cv::Mat grabbed_frame;
  if (!data->retrieve_frame_fn(&grabbed_frame))
  {
    throw std::runtime_error("Failed to retrieve next frame.");
  }
  cv::Mat frame;
  if (grabbed_frame.isContinuous())
  {
    frame = grabbed_frame;
  }
  else
  {
    std::cout << "Grabbed non-continuous frame. Copying" << std::endl;
    frame = cv::Mat(frame.rows, frame.cols, frame.type());
    grabbed_frame.copyTo(frame);
  }

  int buffer_size = frame.rows * frame.cols * frame.channels() * frame.elemSize1();
//  std::cout << "width: " << frame.cols << std::endl;
//  std::cout << "height: " << frame.rows << std::endl;
//  std::cout << "channels: " << frame.channels() << std::endl;
//  std::cout << "element size: " << frame.elemSize1() << std::endl;
//  std::cout << "buffer size: " << buffer_size << std::endl;
  switch (frame.depth())
  {
  case CV_8U:
//    std::cout << "depth: CV_8U" << std::endl;
    break;
  case CV_8S:
//    std::cout << "depth: CV_8S" << std::endl;
    break;
  case CV_16U:
//    std::cout << "depth: CV_16U" << std::endl;
    break;
  case CV_16S:
//    std::cout << "depth: CV_16S" << std::endl;
    break;
  default:
    throw std::runtime_error("pushNewFrame() failed: Unsupported image depth");
  }
  if (!frame.isContinuous())
  {
    throw std::runtime_error("pushNewFrame() failed: Non-continuous images are not supported");
  }

  if (!data->negotiated)
  {
    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "width", G_TYPE_INT, frame.cols,
        "height", G_TYPE_INT, frame.rows,
        "format", G_TYPE_STRING, "BGRA",
        "bpp", G_TYPE_INT, 8 * frame.elemSize1(),
        "framerate", GST_TYPE_FRACTION, static_cast<int>(data->framerate), 1,
        nullptr);
    gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
    data->negotiated = true;
  }

  GstBuffer *buffer = gst_buffer_new_and_alloc(buffer_size);
  if (buffer == nullptr)
  {
    throw std::runtime_error("pushNewFrame() failed: Unable to allocate buffer");
  }

  GstMapInfo info;
  gst_buffer_map(buffer, &info, GST_MAP_WRITE);
  if (info.size != buffer_size)
  {
    throw std::runtime_error("pushNewFrame() failed: Buffer has wrong size");
  }
  std::copy(frame.data, frame.data + buffer_size, info.data);
  gst_buffer_unmap(buffer, &info);

  // Set buffer timestamp and duration
  std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_duration = now - data->start_time;
  double gst_elapsed_seconds = (elapsed_duration.count()) * GST_SECOND;
//  std::cout << elapsed_seconds.count() * GST_SECOND << std::endl;
  GstClockTime cst_elapsed_time = static_cast<GstClockTime>(gst_elapsed_seconds);
  GST_BUFFER_PTS(buffer) = cst_elapsed_time;
//  GST_BUFFER_PTS(buffer) = data->timestamp;
  GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, data->framerate);
//  data->timestamp += GST_BUFFER_DURATION(buffer);
//  std::cout << "time: " << (GST_BUFFER_PTS(buffer) / ((double)GST_SECOND)) << std::endl;
//  std::cout << "duration" << (GST_BUFFER_DURATION(buffer) / ((double)GST_SECOND)) << std::endl;

//  std::cout << "Pushing frame" << std::endl;
  //  GstFlowReturn ret;
  //  g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
  GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
  if (ret != GST_FLOW_OK)
  {
    std::cerr << "pushNewFrame() failed: push-buffer signal returned error" << std::endl;
  }
  // gst_app_src_push_buffer takes ownership of buffer
//  gst_buffer_unref(buffer);
}

// TODO
//void appsrcNeedDataCallback(GstElement *source, guint unused_size, gpointer user_data)
//{
//  GstAppSrc *appsrc = GST_APP_SRC(source);
//  UserData *data = reinterpret_cast<UserData*>(user_data);
//  pushNewFrame(appsrc, data);
//}

int main(int argc, char **argv)
{
  GError *gst_err;
  gboolean gst_initialized = gst_init_check(&argc, &argv, &gst_err);
  if (gst_initialized == FALSE)
  {
    std::cerr << "gst_init_check failed: " << gst_err->message << std::endl;
    throw std::runtime_error("Unable to initialize gstreamer");
  }

  try
  {
    TCLAP::CmdLine cmd("Video capture tool", ' ', "0.1");
    TCLAP::ValueArg<bool> zed_arg("z", "use-zed", "Use ZED camera", false, true, "boolean", cmd);
    TCLAP::ValueArg<int> zed_mode_arg("", "zed-mode", "ZED camera mode", false, 3, "mode", cmd);
    TCLAP::ValueArg<int> cv_device_arg("d", "cv-device", "OpenCV device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> cv_video_arg("v", "cv-video", "OpenCV video device to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> cv_width_arg("", "cv-width", "Frame width to capture", false, 0, "pixels", cmd);
    TCLAP::ValueArg<int> cv_height_arg("", "cv-height", "Frame height to capture", false, 0, "pixels", cmd);
    TCLAP::ValueArg<int> camera_fps_arg("", "camera-fps", "Frame-rate to capture frames", false, 0, "Hz", cmd);
    TCLAP::ValueArg<bool> show_arg("s", "show", "Show captured video", false, true, "boolean", cmd);

    TCLAP::ValueArg<std::string> streaming_branch_arg("", "streaming-branch", "Streaming branch to use", false,
        "queue2 ! omxh264enc low-latency=0 control-rate=1 bitrate=10000000 ! h264parse ! avdec_h264 ! videoconvert ! xvimagesink sync=false async=false", "gstreamer branch", cmd);
    TCLAP::ValueArg<std::string> display_branch_arg("", "display-branch", "Display branch to use", false,
        "queue2 ! videoconvert ! xvimagesink", "gstreamer branch", cmd);
    TCLAP::ValueArg<std::string> preprocess_branch_arg("", "preprocess-branch", "Preprocess branch to use", false,
            "videoconvert", "gstreamer branch", cmd);

    cmd.parse(argc, argv);

    // Create source element
//    GstElement *source = gst_element_factory_make("videotestsrc", "source");
    GstElement *source = gst_element_factory_make("appsrc", "source");
    if (source == nullptr)
    {
      throw std::runtime_error("Unable to create video source element");
    }
    // Modify source properties
//    g_object_set(source, "pattern", 0, nullptr);
    g_object_set(
        source,
        "stream-type", 0,
        "format", GST_FORMAT_TIME,
        nullptr);

    // TODO
//    g_signal_connect(source, "need-data", G_CALLBACK(appsrcNeedDataCallback), &data);

    // Create preprocessing bin
    GstElement *preprocess_bin = gst_parse_bin_from_description(preprocess_branch_arg.getValue().c_str(), true, nullptr);
    if (preprocess_bin == nullptr)
    {
      throw std::runtime_error("Unable to create preprocess elements");
    }

    // Create splitting element
    GstElement *tee = gst_element_factory_make("tee", "tee");
    if (tee == nullptr)
    {
      throw std::runtime_error("Unable to create tee element");
    }

    // Create display branch
    GstElement *display_bin = gst_parse_bin_from_description(display_branch_arg.getValue().c_str(), true, nullptr);
    if (display_bin == nullptr)
    {
      throw std::runtime_error("Unable to create display elements");
    }

    // Create streaming branch
    GstElement *streaming_bin = gst_parse_bin_from_description(streaming_branch_arg.getValue().c_str(), true, nullptr);
    if (streaming_bin == nullptr)
    {
      throw std::runtime_error("Unable to create streaming elements");
    }

    GstElement *pipeline = gst_pipeline_new("Quad3DR_pipeline");
    if (pipeline == nullptr)
    {
      throw std::runtime_error("Unable to create gstreamer pipeline");
    }

    // Fill pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, preprocess_bin, tee, display_bin, streaming_bin, nullptr);
    if (gst_element_link_many(source, preprocess_bin, tee, nullptr) != true)
    {
      gst_object_unref(pipeline);
      throw std::runtime_error("Unable to link elements");
    }

    // Link splitting element with display branch
    GstPadTemplate *tee_src_pad_template = gst_element_class_get_pad_template(GST_ELEMENT_GET_CLASS(tee), "src_%u");
    if (tee_src_pad_template == nullptr)
    {
      throw std::runtime_error("Unable to obtain tee source pad template for display branch");
    }
    std::cout << "pad_template: " << tee_src_pad_template << std::endl;
    GstPad *tee_display_pad = gst_element_request_pad(tee, tee_src_pad_template, nullptr, nullptr);
    if (tee_display_pad == nullptr)
    {
      throw std::runtime_error("Unable to create display pad from tee element");
    }
    std::cout << "Obtained request pad " << gst_pad_get_name(tee_display_pad) << " for display branch.\n" << std::endl;
    GstPad *display_sink_pad = gst_element_get_static_pad(display_bin, "sink");
    if (display_sink_pad == nullptr)
    {
      throw std::runtime_error("Unable to get display sink pad");
    }
    if (gst_pad_link(tee_display_pad, display_sink_pad) != GST_PAD_LINK_OK)
    {
      throw std::runtime_error("Unable to link tee pad to display branch");
    }

    // Link splitting element with streaming branch
    tee_src_pad_template = gst_element_class_get_pad_template(GST_ELEMENT_GET_CLASS(tee), "src_%u");
    if (tee_src_pad_template == nullptr)
    {
      throw std::runtime_error("Unable to obtain tee source pad template for streaming branch");
    }
    std::cout << "pad_template: " << tee_src_pad_template << std::endl;
    GstPad *tee_streaming_pad = gst_element_request_pad(tee, tee_src_pad_template, nullptr, nullptr);
    if (tee_streaming_pad == nullptr)
    {
      throw std::runtime_error("Unable to create streaming pad from tee element");
    }
    std::cout << "Obtained request pad " << gst_pad_get_name(tee_display_pad) << " for streaming branch.\n" << std::endl;
    GstPad *streaming_sink_pad = gst_element_get_static_pad(streaming_bin, "sink");
    if (streaming_sink_pad == nullptr)
    {
      throw std::runtime_error("Unable to get streaming sink pad");
    }
    if (gst_pad_link(tee_streaming_pad, streaming_sink_pad) != GST_PAD_LINK_OK)
    {
      throw std::runtime_error("Unable to link tee pad to streaming branch");
    }

    // Start playing
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
      gst_object_unref(pipeline);
      throw std::runtime_error("Unable to start pipeline");
    }

    // Dump pipeline graph
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "video_streamer_pipeline");


    double camera_framerate;
    std::function<bool (cv::Mat*)> retrieve_frame_fn;
    video::VideoSource *video_ptr;

    if (zed_arg.getValue())
    {
      video::VideoSourceZED *zed_video_ptr = new video::VideoSourceZED();
      zed_video_ptr->open(static_cast<sl::zed::ZEDResolution_mode>(zed_mode_arg.getValue()));
      if (camera_fps_arg.isSet())
      {
        if (!zed_video_ptr->setFPS(camera_fps_arg.getValue()))
        {
          std::cerr << "Setting camera FPS failed" << std::endl;
        }
      }
      camera_framerate = zed_video_ptr->getFPS();
      video_ptr = zed_video_ptr;
      retrieve_frame_fn = [video_ptr](cv::Mat *frame_ptr) -> bool
          {
              cv::Mat left_grabbed_frame;
              cv::Mat right_grabbed_frame;
              cv::Mat depth_grabbed_frame;
            //  cv::Size display_size(data->video_ptr->getWidth(), data->video_ptr->getHeight());
            //  cv::Mat left_grabbed_frame(display_size, CV_8UC4);
            //  cv::Mat right_grabbed_frame(display_size, CV_8UC4);
            //  cv::Mat depth_grabbed_frame(display_size, CV_8UC4);
              if (!video_ptr->grab())
              {
                throw std::runtime_error("Failed to grab next frame.");
              }
              // Retrieve stereo and depth frames
              if (!video_ptr->retrieveLeft(&left_grabbed_frame))
              {
                throw std::runtime_error("Failed to retrieve left frame");
              }
              if (!video_ptr->retrieveRight(&right_grabbed_frame))
              {
                throw std::runtime_error("Failed to retrieve right frame");
              }
              if (!video_ptr->retrieveDepth(&depth_grabbed_frame))
              {
                throw std::runtime_error("Failed to retrieve depth frame");
              }
              // Make sure all frames have the same size and type
              if (left_grabbed_frame.rows != right_grabbed_frame.rows || left_grabbed_frame.rows != depth_grabbed_frame.rows)
              {
                throw std::runtime_error("Stereo and depth frames do not have the same height");
              }
              if (left_grabbed_frame.cols != right_grabbed_frame.cols || left_grabbed_frame.cols != depth_grabbed_frame.cols)
              {
                throw std::runtime_error("Stereo and depth frames do not have the same width");
              }
              if (left_grabbed_frame.type() != right_grabbed_frame.type() || left_grabbed_frame.type() != depth_grabbed_frame.type())
              {
                throw std::runtime_error("Stereo and depth frames do not have the same type");
              }
              int total_rows = left_grabbed_frame.rows;
              int total_cols = left_grabbed_frame.cols + right_grabbed_frame.cols + depth_grabbed_frame.cols;
              if (frame_ptr->empty()
                  || frame_ptr->rows != total_rows
                  || frame_ptr->cols != total_cols
                  || frame_ptr->type() != left_grabbed_frame.type())
              {
                (*frame_ptr) = cv::Mat(total_rows, total_cols, left_grabbed_frame.type());
              }
              left_grabbed_frame.copyTo(frame_ptr->colRange(cv::Range(0, left_grabbed_frame.cols)));
              right_grabbed_frame.copyTo(frame_ptr->colRange(cv::Range(left_grabbed_frame.cols, left_grabbed_frame.cols + depth_grabbed_frame.cols)));
              depth_grabbed_frame.copyTo(frame_ptr->colRange(cv::Range(left_grabbed_frame.cols + depth_grabbed_frame.cols, total_cols)));
              return true;
          };
    }
    else
    {
      video::VideoSourceOpenCV *cv_video_ptr = new video::VideoSourceOpenCV();
      if (cv_video_arg.isSet())
      {
        cv_video_ptr->open(cv_video_arg.getValue());
      }
      else
      {
        cv_video_ptr->open(cv_device_arg.getValue());
      }
      if (cv_width_arg.isSet())
      {
        if (!cv_video_ptr->setFrameWidth(cv_width_arg.getValue()))
        {
          std::cerr << "Setting frame width failed" << std::endl;
        }
      }
      if (cv_height_arg.isSet())
      {
        if (!cv_video_ptr->setFrameHeight(cv_height_arg.getValue()))
        {
          std::cerr << "Setting frame height failed" << std::endl;
        }
      }
      if (camera_fps_arg.isSet())
      {
        if (!cv_video_ptr->setFPS(camera_fps_arg.getValue()))
        {
          std::cerr << "Setting camera FPS failed" << std::endl;
        }
      }
      camera_framerate = cv_video_ptr->getFPS();
      video_ptr = cv_video_ptr;
      retrieve_frame_fn = [video_ptr](cv::Mat *frame_ptr) -> bool
          {
              if (!video_ptr->grab())
              {
                throw std::runtime_error("Failed to grab next frame.");
              }
              if (!video_ptr->retrieveMono(frame_ptr))
              {
                throw std::runtime_error("Failed to retrieve mono frame");
              }
              return true;
          };
    }
    std::cout << "Grabbing frames with " << camera_framerate << " Hz" << std::endl;

    AppSrcData data;
    data.timestamp = 0;
    data.start_time = std::chrono::system_clock::now();
    data.framerate = camera_framerate;
    data.negotiated = false;
    data.retrieve_frame_fn = retrieve_frame_fn;


    bool terminate = false;

    // Create thread for grabbing images from camera
    std::thread push_thread([source, &data, &terminate]()
      {
        int64_t start_ticks = cv::getTickCount();
        int frame_counter = 0;
        int key = -1;
        while (!terminate)
        {
          pushNewFrame(GST_APP_SRC(source), &data);

          ++frame_counter;
          int64_t ticks = cv::getTickCount();
          double dt = double(ticks - start_ticks) / cv::getTickFrequency();
          double fps = frame_counter / dt;
          if (frame_counter > 30)
          {
            std::cout << "Running with " << fps << std::endl;
            start_ticks = ticks;
            frame_counter = 0;
          }
        }
      }
    );

    // Wait until error or EOS
    GstBus *bus = gst_element_get_bus(pipeline);
    while (!terminate)
    {
      GstMessage *msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_STATE_CHANGED));

      if (msg != nullptr)
      {
        GError *err;
        gchar *debug_info;
        switch (GST_MESSAGE_TYPE(msg))
        {
        case GST_MESSAGE_ERROR:
          gst_message_parse_error(msg, &err, &debug_info);
          std::cerr << "Error received from element " << GST_OBJECT_NAME(msg->src) << ": " << err->message << std::endl;
          std::cerr << "Debugging information: " << (debug_info ? debug_info : "none") << std::endl;
          g_clear_error(&err);
          g_free(debug_info);
          terminate = true;
          break;
        case GST_MESSAGE_EOS:
          std::cout << "Stream finished." << std::endl;
          terminate = true;
          break;
        case GST_MESSAGE_STATE_CHANGED:
        {
          GstState old_state, new_state, pending_state;
          gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
          if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline))
          {
            if (new_state == GST_STATE_PLAYING)
            {
              std::cout << "resetting time" << std::endl;
              data.start_time = std::chrono::system_clock::now();
              data.timestamp = 0;
            }
            std::cout << "Pipeline state changed from " << gst_element_state_get_name(old_state)
                << " to " << gst_element_state_get_name(new_state) << std::endl;
          }
          break;
        }
        default:
          std::cerr << "Unexpected message received." << std::endl;
          break;
        }
        gst_message_unref(msg);
      }
    }

    // Wait for frame pushing thread to stop
    push_thread.join();

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    delete video_ptr;
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
