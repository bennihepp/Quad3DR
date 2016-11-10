#pragma once

#include <iostream>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include <ait/serializable.h>

#if WITH_GSTREAMER
#include <gst/gst.h>
#endif

using ait::Serializable;
using ait::Reader;
using ait::Writer;

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
	CLIENT_2_SERVER_GSTREAMER_PARAMETERS = 512 + 1,
	CLIENT_2_SERVER_GSTREAMER_FRAME = 512 + 2,
#endif

	// packets from server to client
	SERVER_2_CLIENT_PROCESSED = 1024 + 1,
	SERVER_2_CLIENT_RESET = 1024 + 2,
};

struct StereoPacketHeader
{
	//~StereoPacketHeader() override {
	//};

	StereoClientType client_type;
	StereoPacketType packet_type;
	size_t packet_size;
	size_t packet_size_decompressed;
};

struct StereoFrameHeader
{
	//~StereoFrameHeader() override {
	//};

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

class Calibration
{
public:
	//~Calibration() override {
	//};

	Calibration() {
		setIdentity();
	}

	void setIdentity() {
		intrinsics.setIdentity();
		extrinsics.setIdentity();
	}

	template <typename DerivedA, typename DerivedB>
	void setMatrices(const Eigen::DenseBase<DerivedA>& intrinsics, const Eigen::DenseBase<DerivedB>& extrinsics) {
		this->intrinsics = intrinsics;
		this->extrinsics = extrinsics;
	}

	//! Camera projection matrix (in camera coordinate system)
	Eigen::Matrix4f intrinsics;

	//! World to camera coordinate system
	Eigen::Matrix4f extrinsics;
};

struct StereoCalibration
{
	//~StereoCalibration() override {
	//};

	unsigned int	depth_image_width;
	unsigned int	depth_image_height;
	unsigned int	color_image_width_left;
	unsigned int	color_image_height_left;
	unsigned int	color_image_width_right;
	unsigned int	color_image_height_right;
	Calibration calibration_depth;
	Calibration calibration_color_left;
	Calibration calibration_color_right;
};

#if WITH_GSTREAMER
struct GstreamerBufferInfo
{
	//~GstreamerBufferInfo() override {
	//};

	GstClockTime pts;
	GstClockTime dts;
	GstClockTime duration;
	guint64 offset;
	guint64 offset_end;
	size_t size;
};

struct GstreamerParameters : public Serializable<GstreamerParameters>
{
	//~GstreamerParameters() override {
	//};

	std::string caps_string;

	size_t _write(Writer& writer) const override {
		return writer.write(caps_string);
	}

	size_t _read(Reader& reader) override {
		return reader.read(caps_string);
	}
};

enum class StereoImageSide
{
	LEFT = 0,
	RIGHT = 1,
	DEPTH = 2,
};

struct ValidationPixelPosition
{
	//~ValidationPixelPosition() override {
	//};

	StereoImageSide side;
	uint16_t x;
	uint16_t y;
};

struct StereoFrameUserParameters : public Serializable<StereoFrameUserParameters>
{
	//~StereoFrameUserParameters() override {
	//};

	std::vector<ValidationPixelPosition> validation_pixel_positions;

	size_t _write(Writer& writer) const override {
		return writer.write(validation_pixel_positions);
	}

	size_t _read(Reader& reader) override {
		return reader.read(validation_pixel_positions);
	}
};

struct StereoFrameUserData : public Serializable<StereoFrameUserData>
{
	//~StereoFrameUserData() override {
	//};

	std::uint8_t truncation_threshold;
	bool inverse_depth;
	float min_depth;
	float max_depth;
	std::vector<uint8_t> validation_pixel_values;
#if DEBUG_IMAGE_COMPRESSION
	unsigned int width;
	unsigned int height;
	cv::Mat left_frame;
	cv::Mat right_frame;
	cv::Mat depth_frame;
#endif

	size_t _write(Writer& writer) const override {
		size_t written = 0;
		written += writer.write(truncation_threshold);
		written += writer.write(inverse_depth);
		written += writer.write(min_depth);
		written += writer.write(max_depth);
		written += writer.write(validation_pixel_values);
#if DEBUG_IMAGE_COMPRESSION
		written += writer.write(width);
		written += writer.write(height);
		written += writer.write(left_frame);
		written += writer.write(right_frame);
		written += writer.write(depth_frame);
#endif
		return written;
	}

	size_t _read(Reader& reader) override {
		size_t read = 0;
		read += reader.read(truncation_threshold);
		read += reader.read(inverse_depth);
		read += reader.read(min_depth);
		read += reader.read(max_depth);
		read += reader.read(validation_pixel_values);
#if DEBUG_IMAGE_COMPRESSION
		read += reader.read(width);
		read += reader.read(height);
		read += reader.read(left_frame);
		read += reader.read(right_frame);
		read += reader.read(depth_frame);
#endif
		return read;
	}
};

#endif
