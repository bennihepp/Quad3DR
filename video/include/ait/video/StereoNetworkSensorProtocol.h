#pragma once

#include <iostream>
#include <mLibCore.h>
#include <mLibDepthCamera.h>
#include <ext-depthcamera\calibratedSensorData.h>

#if WITH_GSTREAMER
#include <gst/gst.h>
#endif

enum class StereoClientType
{
	CLIENT_UNKNOWN = 0,

	CLIENT_ZED = 1,
};

enum class StereoPacketType
{
	UNKNOWN = 0,

	// packets from client to server
	CLIENT_2_SERVER_CALIBRATION = 1,
	CLIENT_2_SERVER_FRAME_DATA = 2,
	CLIENT_2_SERVER_DISCONNECT = 3,
#if WITH_GSTREAMER
	CLIENT_2_SERVER_GSTREAMER_CAPS = 512 + 1,
	CLIENT_2_SERVER_GSTREAMER_BUFFER = 512 + 2,
#endif

	// packets from server to client
	SERVER_2_CLIENT_PROCESSED = 1024 + 1,
	SERVER_2_CLIENT_RESET = 1024 + 2,
};

struct StereoPacketHeader
{
	StereoClientType client_type;
	StereoPacketType packet_type;
	size_t packet_size;
	size_t packet_size_decompressed;
};

struct StereoFrameHeader
{
	size_t left_compressed_size;
	size_t right_compressed_size;
	size_t depth_compressed_size;
	size_t left_size;
	size_t right_size;
	size_t depth_size;
	bool left_compressed;
	bool right_compressed;
	bool depth_compressed;
};

// TODO: Rename members to use lower case
struct StereoCalibration {
	unsigned int	m_DepthImageWidth;
	unsigned int	m_DepthImageHeight;
	unsigned int	m_ColorImageWidthLeft;
	unsigned int	m_ColorImageHeightLeft;
	unsigned int	m_ColorImageWidthRight;
	unsigned int	m_ColorImageHeightRight;
	ml::CalibrationData m_CalibrationDepth;
	ml::CalibrationData m_CalibrationColorLeft;
	ml::CalibrationData m_CalibrationColorRight;
};

struct StereoFrameUserData {
	bool inverse_depth;
	float min_depth;
	float max_depth;
};
