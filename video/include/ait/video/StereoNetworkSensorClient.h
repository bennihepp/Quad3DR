//==================================================
// StereoNetworkSensorClient.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <ait/video/StereoNetworkSensorProtocol.h>
#if WITH_GSTREAMER
	#include <gst/gst.h>
	#include <ait/video/GstreamerPipeline.h>
#endif

#include <ait/common.h>

//#pragma comment(lib, "zlib64.lib")
//#if _DEBUG
//	#pragma comment(lib, "zlibstaticd.lib")
//#else
//	#pragma comment(lib, "zlibstatic.lib")
//#endif

namespace ait
{

namespace video
{

template <typename TNetworkClient, typename TUserData, typename TUserParameters>
class StereoNetworkSensorClient
{
public:
	StereoNetworkSensorClient(const std::shared_ptr<TNetworkClient>& network_client, const StereoClientType& client_type)
		: ack_frame_counter_(0), network_client_(network_client), client_type_(client_type) {
		//size_t color_frame_size = stereo_calibration_.color_image_width_left * stereo_calibration_.color_image_height_left * 4 * sizeof(uint8_t);
		//size_t depth_frame_size = stereo_calibration_.depth_image_width * stereo_calibration_.depth_image_height * sizeof(float);
		//left_compressed_frame_.resize(color_frame_size);
		//right_compressed_frame_.resize(color_frame_size);
		//depth_compressed_frame_.resize(color_frame_size);
//		depth_frame_short_ = cv::Mat(stereo_calibration_.m_DepthImageHeight, stereo_calibration_.m_DepthImageWidth, CV_16S);
	}

	~StereoNetworkSensorClient() {
	}

	void sendCalibration(const StereoCalibration& stereo_calibration) {
		StereoPacketHeader header;
		header.client_type = client_type_;
		header.packet_type = StereoPacketType::CLIENT_2_SERVER_CALIBRATION;
		header.packet_size = sizeof(StereoCalibration);
		header.packet_size_decompressed = sizeof(StereoCalibration);
		network_client_->sendDataBlocking(header);

		network_client_->sendDataBlocking(stereo_calibration);
	}

	void sendFrameData(const cv::Mat& left_frame, const cv::Mat& right_frame, const cv::Mat& depth_frame, bool use_compression) {
		throw ANNOTATE_EXC(ait::Error, "sendFrameData is not supported");
//		size_t left_size = left_frame.rows * left_frame.cols * left_frame.elemSize();
//		size_t right_size = right_frame.rows * right_frame.cols * right_frame.elemSize();
//		size_t depth_size = depth_frame.rows * depth_frame.cols * depth_frame.elemSize();
//
//		StereoPacketHeader packet_header;
//		packet_header.client_type = client_type_;
//		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_FRAME_DATA;
//		packet_header.packet_size = sizeof(StereoFrameHeader);
//		packet_header.packet_size_decompressed = sizeof(StereoFrameHeader);
//		network_client_->sendDataBlocking(packet_header);
//
//		if (use_compression) {
//#pragma omp parallel sections
//			{
//				{ ml::ZLibWrapper::CompressStreamToMemory(left_frame.data, left_size, left_compressed_frame_, false); }
//#pragma omp section
//				{ ml::ZLibWrapper::CompressStreamToMemory(right_frame.data, right_size, right_compressed_frame_, false); }
//#pragma omp section
//				{ ml::ZLibWrapper::CompressStreamToMemory(depth_frame.data, depth_size, depth_compressed_frame_, false); }
//			}
//
//			float left_compression_ratio = left_compressed_frame_.size() / static_cast<float>(left_size);
//			float right_compression_ratio = right_compressed_frame_.size() / static_cast<float>(right_size);
//			float depth_compression_ratio = depth_compressed_frame_.size() / static_cast<float>(depth_size);
//			std::cout << "Compression ratios:" << std::endl;
//			std::cout << "  left color: " << left_compression_ratio << std::endl;
//			std::cout << "  right color: " << right_compression_ratio << std::endl;
//			std::cout << "  depth: " << depth_compression_ratio << std::endl;
//		}
//
//		size_t left_compressed_size = left_compressed_frame_.size();
//		size_t right_compressed_size = right_compressed_frame_.size();
//		size_t depth_compressed_size = depth_compressed_frame_.size();
//
//		StereoFrameHeader frame_header;
//		frame_header.left_compressed_size = left_compressed_size;
//		frame_header.right_compressed_size = right_compressed_size;
//		frame_header.depth_compressed_size = depth_compressed_size;
//		frame_header.left_size = left_size;
//		frame_header.right_size = right_size;
//		frame_header.depth_size = depth_size;
//		frame_header.left_compressed = use_compression;
//		frame_header.right_compressed = use_compression;
//		frame_header.depth_compressed = use_compression;
//		network_client_->sendDataBlocking(frame_header);
//
//		if (use_compression) {
//			network_client_->sendDataBlocking(left_compressed_frame_);
//			network_client_->sendDataBlocking(right_compressed_frame_);
//			network_client_->sendDataBlocking(depth_compressed_frame_);
//		}
//		else {
//			network_client_->sendDataBlocking(left_frame.data, left_size);
//			network_client_->sendDataBlocking(right_frame.data, right_size);
//			network_client_->sendDataBlocking(depth_frame.data, depth_size);
//		}
//
//		network_client_->receiveDataBlocking(packet_header);
//		if (packet_header.packet_type == StereoPacketType::SERVER_2_CLIENT_PROCESSED) {
//			++ack_frame_counter_;
//		}
	}

	void sendDisconnect() {
		StereoPacketHeader packet_header;
		packet_header.client_type = client_type_;
		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_DISCONNECT;
		packet_header.packet_size = 0;
		packet_header.packet_size_decompressed = 0;
		network_client_->sendDataBlocking(packet_header);
	}

#if WITH_GSTREAMER
	void sendGstreamerData(GstBufferWrapper& buffer, const TUserData& user_data) {
		StereoPacketHeader packet_header;
		packet_header.client_type = client_type_;
		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_GSTREAMER_FRAME;
		packet_header.packet_size = (size_t)-1;
		packet_header.packet_size_decompressed = packet_header.packet_size;
		network_client_->sendDataBlocking(packet_header);

#if DEBUG_IMAGE_COMPRESSION
        network_client_->sendDataBlocking(user_data);
        network_client_->sendDataBlocking(user_data.left_frame.data, user_data.left_frame.rows * user_data.left_frame.cols * user_data.left_frame.elemSize());
        network_client_->sendDataBlocking(user_data.right_frame.data, user_data.right_frame.rows * user_data.right_frame.cols * user_data.right_frame.elemSize());
        network_client_->sendDataBlocking(user_data.depth_frame.data, user_data.depth_frame.rows * user_data.depth_frame.cols * user_data.depth_frame.elemSize());
#else
        network_client_->sendDataBlocking(user_data);
#endif

		GstreamerBufferInfo buffer_info;
		buffer_info.pts = GST_BUFFER_PTS(buffer.get());
		buffer_info.dts = GST_BUFFER_DTS(buffer.get());
		buffer_info.duration = GST_BUFFER_DURATION(buffer.get());
		buffer_info.offset = GST_BUFFER_OFFSET(buffer.get());
		buffer_info.offset_end = GST_BUFFER_OFFSET_END(buffer.get());
		buffer_info.size = buffer.getSize();
		network_client_->sendDataBlocking(buffer_info);

		network_client_->sendDataBlocking(buffer.getData(), buffer.getSize());
	}

	void sendGstreamerParameters(GstCapsWrapper gst_caps, const TUserParameters& user_parameters) {
		StereoPacketHeader packet_header;
		packet_header.client_type = client_type_;
		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_GSTREAMER_PARAMETERS;
		packet_header.packet_size = (size_t)-1;
		packet_header.packet_size_decompressed = packet_header.packet_size;
		network_client_->sendDataBlocking(packet_header);

		std::string gst_caps_string((char*)gst_caps.getString());
		network_client_->sendDataBlocking(gst_caps_string);
		network_client_->sendDataBlocking(user_parameters);
	}
#endif

private:
	size_t ack_frame_counter_;
	std::shared_ptr<TNetworkClient> network_client_;
	StereoClientType client_type_;
	// TODO
//	std::vector<uint8_t> left_compressed_frame_;
//	std::vector<uint8_t> right_compressed_frame_;
//	std::vector<uint8_t> depth_compressed_frame_;
//	cv::Mat depth_frame_short_;
};

}

}
