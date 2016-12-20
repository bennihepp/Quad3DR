#pragma once

#include <iostream>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include <ait/serializable.h>

#include <gst/gst.h>

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
	// packets from client to server
	CLIENT_2_SERVER_INITIALIZATION = 0,

	CLIENT_2_SERVER_GSTREAMER_PARAMETERS = 512 + 1,
	CLIENT_2_SERVER_GSTREAMER_FRAME = 512 + 2,
};

struct StereoPacketHeader
{
	StereoClientType client_type;
	StereoPacketType packet_type;
	size_t packet_size;
	size_t packet_size_decompressed;
};

class Calibration
{
public:
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

struct StereoInitializationPacket
{
	StereoClientType client_type;
	StereoCalibration calibration;
};

struct GstreamerBufferInfo : public Serializable<GstreamerBufferInfo>
{
	~GstreamerBufferInfo() override {
	};

	GstClockTime pts;
	GstClockTime dts;
	GstClockTime duration;
	guint64 offset;
	guint64 offset_end;
	size_t size;
};

struct GstreamerParameters : public Serializable<GstreamerParameters>
{
	~GstreamerParameters() override {
	};

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
	StereoImageSide side;
	uint16_t x;
	uint16_t y;
};

struct StereoFrameParameters : public Serializable<StereoFrameParameters>
{
	~StereoFrameParameters() override {
	};

	std::vector<ValidationPixelPosition> validation_pixel_positions;

	size_t _write(Writer& writer) const override {
		return writer.write(validation_pixel_positions);
	}

	size_t _read(Reader& reader) override {
		return reader.read(validation_pixel_positions);
	}
};

struct StereoFrameInfo : public Serializable<StereoFrameInfo>
{
	~StereoFrameInfo() override {
	};

	double timestamp;
	bool inverse_depth;
	std::uint8_t truncation_threshold;
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
		written += writer.write(timestamp);
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
		size_t bytes_read = 0;
		bytes_read += reader.read(truncation_threshold);
		bytes_read += reader.read(inverse_depth);
		bytes_read += reader.read(min_depth);
		bytes_read += reader.read(max_depth);
		bytes_read += reader.read(validation_pixel_values);
#if DEBUG_IMAGE_COMPRESSION
		bytes_read += reader.read(timestamp);
		bytes_read += reader.read(width);
		bytes_read += reader.read(height);
		bytes_read += reader.read(left_frame);
		bytes_read += reader.read(right_frame);
		bytes_read += reader.read(depth_frame);
#endif
		return bytes_read;
	}
};

struct StereoFrameLocationInfo : public Serializable<StereoFrameLocationInfo>
{
	StereoFrameLocationInfo() {
		this->timestamp = std::numeric_limits<double>::quiet_NaN();
		this->latitude = 0;
		this->longitude = 0;
		this->altitude = 0;
		this->attitude_quaternion.setZero();
		this->velocity.setZero();
		this->angular_velocity.setZero();
	};

	~StereoFrameLocationInfo() override {
	};

	// Timestamp in seconds
	double timestamp;

	// GPS coordinates in degrees
	float latitude;
	float longitude;

	// Altitude in meters
	float altitude;

	// Drone attitude (x, y, z, w)
	Eigen::Matrix<float, 4, 1> attitude_quaternion;

	// Linear velocity in meters/second
	Eigen::Vector3f velocity;
	// Angualr velocity in radians/second
	Eigen::Vector3f angular_velocity;

	size_t _write(Writer& writer) const override {
		size_t written = 0;
		written += writer.write(timestamp);
		written += writer.write(latitude);
		written += writer.write(longitude);
		written += writer.write(altitude);
		written += writer.write(attitude_quaternion);
		written += writer.write(velocity);
		written += writer.write(angular_velocity);
		return written;
	}

	size_t _read(Reader& reader) override {
		size_t bytes_read = 0;
		bytes_read += reader.read(timestamp);
		bytes_read += reader.read(latitude);
		bytes_read += reader.read(longitude);
		bytes_read += reader.read(altitude);
		bytes_read += reader.read(attitude_quaternion);
		bytes_read += reader.read(velocity);
		bytes_read += reader.read(angular_velocity);
		return bytes_read;
	}
};
