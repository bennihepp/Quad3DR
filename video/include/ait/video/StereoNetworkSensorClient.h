#pragma once

#include <ait/mLib.h>
#include <memory>
#include <opencv2/core.hpp>
#include <ait/video/StereoNetworkSensorProtocol.h>
#if WITH_GSTREAMER
	#include <gst/gst.h>
	#include <ait/video/GstreamerPipeline.h>
#endif

namespace ait
{

namespace video
{

template <typename TNetworkClient, typename TUserData>
class StereoNetworkSensorClient
{
public:
	StereoNetworkSensorClient(const std::shared_ptr<TNetworkClient>& network_client, const StereoCalibration& stereo_calibration, const StereoClientType& client_type)
		: ack_frame_counter_(0), network_client_(network_client), stereo_calibration_(stereo_calibration), client_type_(client_type) {
		size_t color_frame_size = stereo_calibration_.m_ColorImageWidthLeft * stereo_calibration_.m_ColorImageHeightLeft * 4 * sizeof(uint8_t);
		size_t depth_frame_size = stereo_calibration_.m_DepthImageWidth * stereo_calibration_.m_DepthImageHeight * sizeof(float);
		//left_compressed_frame_.resize(color_frame_size);
		//right_compressed_frame_.resize(color_frame_size);
		//depth_compressed_frame_.resize(color_frame_size);
		depth_frame_short_ = cv::Mat(stereo_calibration_.m_DepthImageHeight, stereo_calibration_.m_DepthImageWidth, CV_16S);
	}

	~StereoNetworkSensorClient() {
	}

	void sendCalibration() {
		StereoPacketHeader header;
		header.client_type = client_type_;
		header.packet_type = StereoPacketType::CLIENT_2_SERVER_CALIBRATION;
		header.packet_size = sizeof(StereoCalibration);
		header.packet_size_decompressed = sizeof(StereoCalibration);
		network_client_->sendDataBlocking(header);

		network_client_->sendDataBlocking(stereo_calibration_);
	}

	void sendFrameData(const cv::Mat& left_frame, const cv::Mat& right_frame, const cv::Mat& depth_frame, bool use_compression) {
		size_t left_size = left_frame.rows * left_frame.cols * left_frame.elemSize();
		size_t right_size = right_frame.rows * right_frame.cols * right_frame.elemSize();
		size_t depth_size = depth_frame.rows * depth_frame.cols * depth_frame.elemSize();

		StereoPacketHeader packet_header;
		packet_header.client_type = client_type_;
		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_FRAME_DATA;
		packet_header.packet_size = sizeof(StereoFrameHeader);
		packet_header.packet_size_decompressed = sizeof(StereoFrameHeader);
		network_client_->sendDataBlocking(packet_header);

		if (use_compression) {
#pragma omp parallel sections
		{
			{ ml::ZLibWrapper::CompressStreamToMemory(left_frame.data, left_size, left_compressed_frame_, false); }
#pragma omp section
			{ ml::ZLibWrapper::CompressStreamToMemory(right_frame.data, right_size, right_compressed_frame_, false); }
#pragma omp section
			{ ml::ZLibWrapper::CompressStreamToMemory(depth_frame.data, depth_size, depth_compressed_frame_, false); }
		}

		float left_compression_ratio = left_compressed_frame_.size() / static_cast<float>(left_size);
		float right_compression_ratio = right_compressed_frame_.size() / static_cast<float>(right_size);
		float depth_compression_ratio = depth_compressed_frame_.size() / static_cast<float>(depth_size);
		std::cout << "Compression ratios:" << std::endl;
		std::cout << "  left color: " << left_compression_ratio << std::endl;
		std::cout << "  right color: " << right_compression_ratio << std::endl;
		std::cout << "  depth: " << depth_compression_ratio << std::endl;
		}

		size_t left_compressed_size = left_compressed_frame_.size();
		size_t right_compressed_size = right_compressed_frame_.size();
		size_t depth_compressed_size = depth_compressed_frame_.size();

		StereoFrameHeader frame_header;
		frame_header.left_compressed_size = left_compressed_size;
		frame_header.right_compressed_size = right_compressed_size;
		frame_header.depth_compressed_size = depth_compressed_size;
		frame_header.left_size = left_size;
		frame_header.right_size = right_size;
		frame_header.depth_size = depth_size;
		frame_header.left_compressed = use_compression;
		frame_header.right_compressed = use_compression;
		frame_header.depth_compressed = use_compression;
		network_client_->sendDataBlocking(frame_header);

		if (use_compression) {
			network_client_->sendDataBlocking(left_compressed_frame_);
			network_client_->sendDataBlocking(right_compressed_frame_);
			network_client_->sendDataBlocking(depth_compressed_frame_);
		}
		else {
			network_client_->sendDataBlocking(left_frame.data, left_size);
			network_client_->sendDataBlocking(right_frame.data, right_size);
			network_client_->sendDataBlocking(depth_frame.data, depth_size);
		}

		network_client_->receiveDataBlocking(packet_header);
		if (packet_header.packet_type == StereoPacketType::SERVER_2_CLIENT_PROCESSED) {
			++ack_frame_counter_;
		}
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
	void sendGstreamerBuffer(const std::vector<uint8_t>& buffer_data, const GstreamerBufferInfo& buffer_info, const TUserData& user_data) {
		StereoPacketHeader packet_header;
		packet_header.client_type = client_type_;
		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_GSTREAMER_BUFFER;
		packet_header.packet_size = sizeof(TUserData) + sizeof(GstreamerBufferInfo) + buffer_data.size();
		packet_header.packet_size_decompressed = packet_header.packet_size;
		network_client_->sendDataBlocking(packet_header);

		network_client_->sendDataBlocking(user_data);

		network_client_->sendDataBlocking(buffer_info);

		network_client_->sendDataBlocking(buffer_data);
	}

	void sendGstreamerCaps(GstCapsWrapper gst_caps) {
		StereoPacketHeader packet_header;
		packet_header.client_type = client_type_;
		packet_header.packet_type = StereoPacketType::CLIENT_2_SERVER_GSTREAMER_CAPS;
		packet_header.packet_size = std::strlen(gst_caps.getString()) + 1;
		packet_header.packet_size_decompressed = packet_header.packet_size;
		network_client_->sendDataBlocking(packet_header);

		network_client_->sendDataBlocking((uint8_t*)gst_caps.getString(), packet_header.packet_size);
	}
#endif

private:
	size_t ack_frame_counter_;
	std::shared_ptr<TNetworkClient> network_client_;
	StereoCalibration stereo_calibration_;
	StereoClientType client_type_;
	std::vector<uint8_t> left_compressed_frame_;
	std::vector<uint8_t> right_compressed_frame_;
	std::vector<uint8_t> depth_compressed_frame_;
	cv::Mat depth_frame_short_;
};

}

}
