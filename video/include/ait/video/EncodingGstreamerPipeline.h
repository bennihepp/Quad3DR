//==================================================
// EncodingGstreamerPipeline.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#pragma once

#include "GstreamerPipeline.h"

#include <opencv2/core.hpp>

template <typename TUserData>
class EncodingGstreamerPipeline : public GstreamerPipeline<TUserData>
{
  using Base = GstreamerPipeline<TUserData>;

public:
	EncodingGstreamerPipeline()
		: frame_counter_(0), negotiated_(false) {
		pre_process_branch_str_ = "videoconvert";
		display_branch_str_ = "queue ! autovideosink";
		//display_branch_str_ = "queue ! videoconvert ! video/x-raw, format=RGBA ! videoconvert ! autovideosink";
		//encoder_branch_str_ = "identity";
		encoder_branch_str_ = "x264enc bitrate=4096 pass=qual quantizer=20 tune=zerolatency speed-preset=medium threads=4 key-int-max=10";
//		encoder_branch_str_ = "omxh264enc low-latency=0 control-rate=1 bitrate=1000000";
		//encoder_branch_str_ = "openh264enc bitrate=2048";
		//encoder_branch_str_ = "vp8enc target-bitrate=1024";
		//encoder_branch_str_ = "queue ! videoconvert ! video/x-raw, format=RGBA ! pngenc";
		//encoder_branch_str_ = "queue ! jpegenc";
	}

	void setPreProcessBranchStr(const std::string& pre_process_branch_str) {
		pre_process_branch_str_ = pre_process_branch_str;
	}

	void setEncoderBranchStr(const std::string& encoder_branch_str) {
		encoder_branch_str_ = encoder_branch_str;
	}

	void setDisplayBranchStr(const std::string& display_branch_str) {
		display_branch_str_ = display_branch_str;
	}

	void pushInput(const cv::Mat& frame_in, const TUserData& user_data)
	{
		cv::Mat frame;
		if (frame_in.isContinuous()) {
			frame = frame_in;
		}
		else {
			std::cout << "WARNING: Got non-continuous frame. Copying it." << std::endl;
			frame = cv::Mat(frame.rows, frame.cols, frame.type());
			frame_in.copyTo(frame);
		}

		size_t buffer_size = frame.rows * frame.cols * frame.channels() * frame.elemSize1();
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
		//case CV_16U:
		//	//    std::cout << "depth: CV_16U" << std::endl;
		//	break;
		//case CV_16S:
		//	//    std::cout << "depth: CV_16S" << std::endl;
		//	break;
		default:
			throw std::runtime_error("pushNewFrame() failed: Unsupported image depth");
		}
		// Sanity check
		if (!frame.isContinuous()) {
			throw std::runtime_error("pushNewFrame() failed: Non-continuous images are not supported");
		}

		if (!negotiated_) {
			GstCaps *caps = gst_caps_new_simple(
				"video/x-raw",
				"width", G_TYPE_INT, frame.cols,
				"height", G_TYPE_INT, frame.rows,
				"format", G_TYPE_STRING, "BGRA",
				"bpp", G_TYPE_INT, 8 * frame.elemSize1(),
				//"framerate", GST_TYPE_FRACTION, 10, 1,
				nullptr);
			gst_app_src_set_caps(GST_APP_SRC(Base::getNativeAppSrc()), caps);
			negotiated_ = true;
			//start_time_ = std::chrono::system_clock::now();
		}

		GstBuffer *gst_buffer = gst_buffer_new_and_alloc(buffer_size);
		if (gst_buffer == nullptr) {
			throw std::runtime_error("pushNewFrame() failed: Unable to allocate buffer");
		}
		// gst_app_src_push_buffer will take ownership of buffer at some point
		GstBufferWrapper buffer(gst_buffer);
		buffer.mapWritable();
		if (buffer.getSize() != buffer_size) {
			throw std::runtime_error("pushNewFrame() failed: Buffer has wrong size");
		}
		std::copy(frame.data, frame.data + buffer_size, buffer.getDataWritable());
		buffer.unmap();

		// Set buffer timestamp and duration
		//std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		//std::chrono::duration<double> elapsed_duration = now - start_time_;
		//double gst_elapsed_seconds = (elapsed_duration.count()) * GST_SECOND;
		//GstClockTime cst_elapsed_time = static_cast<GstClockTime>(gst_elapsed_seconds);
		//GST_BUFFER_PTS(buffer) = cst_elapsed_time;
		//GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, framerate_);
		//
		//GST_BUFFER_TIMESTAMP(buffer) = gst_util_uint64_scale(data->num_samples, GST_SECOND, SAMPLE_RATE);
		//GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(CHUNK_SIZE, GST_SECOND, SAMPLE_RATE);
		//data->timestamp += GST_BUFFER_DURATION(buffer);
		//std::cout << "time: " << (GST_BUFFER_PTS(buffer) / ((double)GST_SECOND)) << std::endl;
		//std::cout << "duration" << (GST_BUFFER_DURATION(buffer) / ((double)GST_SECOND)) << std::endl;

		// Overwrite timing information to make sure that pipeline runs through
		GstClock* clock = gst_pipeline_get_clock(Base::getNativePipeline());
		GstClockTime time_now = gst_clock_get_time(clock);
		static GstClockTime time_previous = gst_clock_get_time(clock);

		GST_BUFFER_PTS(buffer.get()) = time_now;
		GST_BUFFER_DTS(buffer.get()) = GST_CLOCK_TIME_NONE;
		GST_BUFFER_DURATION(buffer.get()) = time_now - time_previous;
		GST_BUFFER_OFFSET(buffer.get()) = frame_counter_;
		GST_BUFFER_OFFSET_END(buffer.get()) = GST_BUFFER_OFFSET_NONE;
		//std::cout << "time_now: " << time_now << std::endl;
		//std::cout << "duration: " << GST_BUFFER_DURATION(buffer.get()) << std::endl;

		++frame_counter_;
		time_previous = time_now;

		// Set timestamp and duration

		//std::cout << "Pushing buffer with offset=" << GST_BUFFER_OFFSET(buffer) << std::endl;
		// gst_app_src_push_buffer takes ownership of buffer
		//GstFlowReturn ret = gst_app_src_push_buffer(getNativeAppSrc(), buffer);
		//  g_signal_emit_by_name(getNativeAppSrc(), "push-buffer", buffer, &ret);
		//if (ret != GST_FLOW_OK) {
		//	std::cerr << "pushNewFrame() failed: push-buffer signal returned error" << std::endl;
		//}

		if (!Base::pushInput(buffer, user_data)) {
			std::cerr << "pushNewFrame() failed: push-buffer signal returned error" << std::endl;
		}
	}

protected:
	GstPipeline* createPipeline(GstAppSrc* appsrc, GstAppSink* appsink) const override
	{
#if SIMULATE_ZED
		GstElement *source = gst_parse_bin_from_description("videotestsrc ! video/x-raw, format=RGBA, framerate=15/1, width=1920, height=360 ! videoconvert", true, nullptr);
		//GstElement* source = gst_element_factory_make("videotestsrc", "source");
		if (source == nullptr) {
			throw std::runtime_error("Unable to create videotestsrc element");
		}
		appsrc = reinterpret_cast<GstAppSrc*>(source);
#endif

		GstElement* sink = GST_ELEMENT(appsink);

		// TODO
		//g_signal_connect(source, "need-data", G_CALLBACK(appsrcNeedDataCallback), &data);

		// Create preprocessing bin
		std::cout << "Preprocess branch: " << pre_process_branch_str_ << std::endl;
		GstElement *preprocess_bin = gst_parse_bin_from_description(pre_process_branch_str_.c_str(), true, nullptr);
		if (preprocess_bin == nullptr) {
			throw std::runtime_error("Unable to create preprocess elements");
		}

		// Create splitting element
		GstElement *tee = gst_element_factory_make("tee", "tee");
		if (tee == nullptr) {
			throw std::runtime_error("Unable to create tee element");
		}

		// Create display branch
    std::cout << "Display branch: " << display_branch_str_ << std::endl;
		GstElement *display_bin = gst_parse_bin_from_description(display_branch_str_.c_str(), true, nullptr);
		if (display_bin == nullptr) {
			throw std::runtime_error("Unable to create display elements");
		}

		// Create streaming branch
    std::cout << "Encoder branch: " << encoder_branch_str_ << std::endl;
		GstElement *encoder_bin = gst_parse_bin_from_description(encoder_branch_str_.c_str(), true, nullptr);
		if (encoder_bin == nullptr) {
			throw std::runtime_error("Unable to create streaming elements");
		}

		GstElement *pipeline = gst_pipeline_new("BundleFusion_pipeline");
		if (pipeline == nullptr) {
			throw std::runtime_error("Unable to create gstreamer pipeline");
		}

		// Fill pipeline
		gst_bin_add_many(GST_BIN(pipeline), GST_ELEMENT(appsrc), preprocess_bin, tee, display_bin, encoder_bin, sink, nullptr);
		if (gst_element_link_many(GST_ELEMENT(appsrc), preprocess_bin, tee, nullptr) != TRUE) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to link source, preprocess and tee elements");
		}

		// Link splitting element with display branch
		GstPadTemplate *tee_src_pad_template = gst_element_class_get_pad_template(GST_ELEMENT_GET_CLASS(tee), "src_%u");
		if (tee_src_pad_template == nullptr) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to obtain tee source pad template for display branch");
		}
		std::cout << "pad_template: " << tee_src_pad_template << std::endl;
		GstPad *tee_display_pad = gst_element_request_pad(tee, tee_src_pad_template, nullptr, nullptr);
		if (tee_display_pad == nullptr) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to create display pad from tee element");
		}
		std::cout << "Obtained request pad " << gst_pad_get_name(tee_display_pad) << " for display branch.\n" << std::endl;
		GstPad *display_sink_pad = gst_element_get_static_pad(display_bin, "sink");
		if (display_sink_pad == nullptr) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to get display sink pad");
		}
		if (gst_pad_link(tee_display_pad, display_sink_pad) != GST_PAD_LINK_OK) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to link tee pad to display branch");
		}

		// Link splitting element with encoder branch
		tee_src_pad_template = gst_element_class_get_pad_template(GST_ELEMENT_GET_CLASS(tee), "src_%u");
		if (tee_src_pad_template == nullptr) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to obtain tee source pad template for streaming branch");
		}
		std::cout << "pad_template: " << tee_src_pad_template << std::endl;
		GstPad *tee_encoder_pad = gst_element_request_pad(tee, tee_src_pad_template, nullptr, nullptr);
		if (tee_encoder_pad == nullptr) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to create streaming pad from tee element");
		}
		std::cout << "Obtained request pad " << gst_pad_get_name(tee_display_pad) << " for streaming branch.\n" << std::endl;
		GstPad *encoder_sink_pad = gst_element_get_static_pad(encoder_bin, "sink");
		if (encoder_sink_pad == nullptr) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to get streaming sink pad");
		}
		if (gst_pad_link(tee_encoder_pad, encoder_sink_pad) != GST_PAD_LINK_OK) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to link tee pad to streaming branch");
		}

		// Link encoder branch with sink
		if (gst_element_link_many(encoder_bin, GST_ELEMENT(appsink), nullptr) != TRUE) {
			gst_object_unref(pipeline);
			throw std::runtime_error("Unable to link encoder and sink elements");
		}

		return GST_PIPELINE(pipeline);
	}

private:
	guint64 frame_counter_;
	bool negotiated_;
	//std::chrono::time_point<std::chrono::system_clock> start_time_;

	std::string encoder_branch_str_;
	std::string pre_process_branch_str_;
	std::string display_branch_str_;
};
