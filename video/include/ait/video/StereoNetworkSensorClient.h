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
#include <gst/gst.h>
#include <ait/video/GstreamerPipeline.h>

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
	}

	~StereoNetworkSensorClient() {
	}

	void sendInitialization(const StereoInitializationPacket& stereo_initialization) {
		StereoPacketHeader header;
		header.client_type = client_type_;
		header.packet_type = StereoPacketType::CLIENT_2_SERVER_INITIALIZATION;
		header.packet_size = sizeof(StereoInitializationPacket);
		header.packet_size_decompressed = sizeof(StereoInitializationPacket);
		network_client_->sendDataBlocking(header);

		network_client_->sendDataBlocking(stereo_initialization);
	}

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

private:
	size_t ack_frame_counter_;
	std::shared_ptr<TNetworkClient> network_client_;
	StereoClientType client_type_;
};

}

}
