//==================================================
// StereoNetworkSensorManager.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#pragma once

#include <memory>
#include <random>

#include <gst/gst.h>

#include <ait/video/StereoNetworkSensorClient.h>
#include <ait/video/StereoNetworkSensorProtocol.h>
#include <ait/video/EncodingGstreamerPipeline.h>

namespace ait
{

	namespace video
	{
		template <typename TNetworkClient>
		class StereoNetworkSensorManager
		{
			using clock = std::chrono::system_clock;

			const std::chrono::seconds CONNECTION_ATTEMPT_TIMEOUT = std::chrono::seconds(2);
			const uint8_t DEPTH_UINT8_TRUNCATION_THRESHOLD = 5;

			const unsigned int NUMBER_OF_VALIDATION_PIXELS_PER_IMAGE = 1000;

            using PipelineUserDataType = std::tuple<StereoFrameInfo, StereoFrameLocationInfo>;

		public:
			StereoNetworkSensorManager(const StereoCalibration& stereo_calibration, const StereoClientType& client_type, const std::string& remote_ip, unsigned int remote_port)
				: pipeline_initialized_(false),
				stereo_calibration_(stereo_calibration),
				network_client_(std::make_shared<TNetworkClient>()),
				stereo_sensor_client_(network_client_, client_type),
				remote_ip_(remote_ip), remote_port_(remote_port) {
				terminate_ = false;
				pipeline_.setStateChangeCallback(std::bind(&StereoNetworkSensorManager::stateChangeCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
				initializeValidationPixelLocations();
			}

			virtual ~StereoNetworkSensorManager() {
			}

			void setUserParameters(const StereoFrameParameters& user_parameters) {
				user_parameters_ = user_parameters;
			}

			void setDepthTruncation(float trunc_depth_min, float trunc_depth_max) {
				trunc_depth_min_ = trunc_depth_min;
				trunc_depth_max_ = trunc_depth_max;
			}

			void setInverseDepth(bool inverse_depth) {
				inverse_depth_ = inverse_depth;
			}

			const EncodingGstreamerPipeline<PipelineUserDataType>& getPipeline() const {
				return pipeline_;
			}

			EncodingGstreamerPipeline<PipelineUserDataType>& getPipeline() {
				return pipeline_;
			}

			void setUseCompression(bool use_compression) {
				use_compression_ = use_compression;
			}

			void stateChangeCallback(GstState old_state, GstState new_state, GstState pending_state) {
				if (new_state == GST_STATE_PLAYING) {
					std::cout << "Pipeline playing... Starting pipeline output thread" << std::endl;
					if (pipeline_output_thread_.joinable()) {
						throw std::runtime_error("Network loop is already running");
					}
					terminate_ = false;
					pipeline_output_thread_ = std::thread([this]() {
						pipelineOutputLoop();
					});
				}
				else {
					if (pipeline_output_thread_.joinable()) {
						std::cout << "Pipeline not playing anymore. Stopping pipeline output thread" << std::endl;
						terminate_ = true;
						pipeline_output_thread_.join();
            std::cout << "Pipeline output thread stopped" << std::endl;
					}
				}
			}

			void start() {
				if (!pipeline_initialized_) {
					pipeline_.initialize();
					pipeline_initialized_ = true;
				}
				pipeline_.start();
			}

			void stop() {
				std::cout << "Stopping pipeline and pipeline output thread ..." << std::endl;
				if (pipeline_initialized_) {
					pipeline_.stop();
				}
				std::cout << "Pipeline stopped" << std::endl;
				terminate_ = true;
				if (pipeline_output_thread_.joinable()) {
					pipeline_output_thread_.join();
				}
				std::cout << "Pipeline output thread stopped" << std::endl;
			}

            //! Push a new stereo frame into the pipeline. Also save corresponding user data. Not thread-safe!
            void pushNewStereoFrame(double timestamp, const cv::Mat& left_frame, const cv::Mat& right_frame, const cv::Mat& depth_frame,
                    const StereoFrameLocationInfo& location_info = StereoFrameLocationInfo()) {
                //cv::cvtColor(left_frame, left_frame, CV_BGRA2RGBA);
                //cv::cvtColor(right_frame, right_frame, CV_BGRA2RGBA);

                StereoFrameInfo frame_info;
                frame_info.timestamp = timestamp;
                const cv::Mat& merged_frame = processFrames(left_frame, right_frame, depth_frame, frame_info);

                pipeline_.pushInput(merged_frame, std::make_tuple(frame_info, location_info));
            }

		protected:
			void pipelineOutputLoop() {
				while (!terminate_) {
					try {
					    // Ensure connection is closed
					    network_client_->close();
						std::cout << "Trying to establish connection ..." << std::endl;
						//network_client_->open(remote_ip_, remote_port_);
						clock::time_point connect_start = clock::now();
						std::future<bool> success = network_client_->asyncOpen(remote_ip_, remote_port_);
						std::future_status status;
						do {
							status = success.wait_for(std::chrono::milliseconds(500));
							if (clock::now() - connect_start >= CONNECTION_ATTEMPT_TIMEOUT) {
								break;
							}
						} while (!terminate_ && status != std::future_status::ready);
						if (status != std::future_status::ready) {
							network_client_->asyncOpenCancel();
							std::cout << "Connection attempt timed out. Trying again." << std::endl;
							continue;
						}
						else if (!success.get()) {
							std::cout << "Unable to establish connection. Trying again." << std::endl;
							continue;
						}

						std::cout << "Connection established" << std::endl;
						StereoInitializationPacket stereo_initilization;
						stereo_initilization.client_type = StereoClientType::CLIENT_ZED;
						stereo_initilization.calibration = stereo_calibration_;
						stereo_sensor_client_.sendInitialization(stereo_initilization);

						bool outputCapsSent = false;
						RateCounter frame_rate_counter;
						size_t byte_counter = 0;
						while (!terminate_ || pipeline_.hasOutput()) {
							std::unique_lock<std::mutex> lock(pipeline_.getMutex());
							pipeline_.getOutputCondition().wait_for(lock, std::chrono::milliseconds(100), [this]() { return pipeline_.hasOutput(); });
							if (pipeline_.hasOutput()) {
								std::tuple<GstBufferWrapper, PipelineUserDataType> output_tuple(std::move(pipeline_.popOutput(lock)));
								GstBufferWrapper& buffer = std::get<0>(output_tuple);
								const PipelineUserDataType& pipeline_user_data = std::get<1>(output_tuple);
								const StereoFrameInfo& frame_info = std::get<0>(pipeline_user_data);
//								const StereoFrameLocationInfo& location_info = std::get<1>(pipeline_user_data);
								lock.unlock();
								if (!pipeline_.isPlaying()) {
									continue;
								}
								try {
									if (!outputCapsSent) {
										GstCapsWrapper output_caps = pipeline_.getOutputCaps();
										std::cout << "Sending Gstreamer Caps: " << output_caps.getString() << std::endl;
										stereo_sensor_client_.sendGstreamerParameters(std::move(pipeline_.getOutputCaps()), user_parameters_);
										outputCapsSent = true;
									}

									frame_rate_counter.count();
									double rate;
									unsigned int frame_count = frame_rate_counter.getCount();
									byte_counter += sizeof(StereoPacketHeader) + sizeof(GstreamerBufferInfo) + sizeof(StereoFrameInfo) + sizeof(StereoFrameLocationInfo)
										+ frame_info.validation_pixel_values.size() + buffer.getSize();
									if (frame_rate_counter.reportRate(rate)) {
										double bandwidth = rate * byte_counter / static_cast<double>(frame_count) / 1024.0;
										byte_counter = 0;
										std::cout << "Sending Gstreamer buffers with " << rate << " Hz. Bandwidth: " << bandwidth << "kB/s" << std::endl;
									}
									stereo_sensor_client_.sendGstreamerData(buffer, pipeline_user_data);
								}
								catch (const typename TNetworkClient::Error& err) {
									std::cerr << "Network error occured: " << err.what() << std::endl;
									break;
								}
							}
						}

					}
					catch (const typename TNetworkClient::Error& err) {
						if (err.getBoostErrorCode().value() == boost::asio::error::connection_refused) {
							std::cout << "Connection refused. Trying to reconnect in a moment" << std::endl;
							std::this_thread::sleep_for(std::chrono::milliseconds(500));
						}
						else {
							std::cerr << "Network error: " << err.what() << std::endl;
						}
					}
					try {
						network_client_->close();
					}
					catch (const typename TNetworkClient::Error& err) {
						std::cerr << "Error when trying to close client: " << err.what() << std::endl;
					}
				}
				std::cout << "Exiting pipeline output thread" << std::endl;
			}

            virtual const cv::Mat processFrames(
                    const cv::Mat& left_frame, const cv::Mat& right_frame, const cv::Mat& depth_frame,
                    StereoFrameInfo& frame_info) {
//                std::cout << "Converting depth frame to RGBA" << std::endl;
                const cv::Mat& depth_frame_rgba = convertDepthFrameFloatToRGBA(depth_frame, frame_info, inverse_depth_);
//                std::cout << "Merging left, right and depth frame to single stereo frame" << std::endl;
                const cv::Mat& merged_frame = mergeStereoDepthFrames(left_frame, right_frame, depth_frame_rgba);
                AIT_ASSERT(merged_frame.isContinuous());

#if DEBUG_IMAGE_COMPRESSION
                frame_info.width = left_frame.cols;
                frame_info.height = left_frame.rows;
                left_frame.copyTo(frame_info.left_frame);
                right_frame.copyTo(frame_info.right_frame);
                depth_frame.copyTo(frame_info.depth_frame);
#endif

//                std::cout << "Reading validation pixels" << std::endl;
                frame_info.validation_pixel_values.resize(user_parameters_.validation_pixel_positions.size());
#pragma omp parallel for
                for (int i = 0; i < user_parameters_.validation_pixel_positions.size(); ++i) {
                    const ValidationPixelPosition& position = user_parameters_.validation_pixel_positions[i];
                    uint8_t value;
                    switch (position.side) {
                    case StereoImageSide::LEFT:
                    {
                        const cv::Vec4b& vec = left_frame.at<cv::Vec4b>(position.y, position.x);
                        value = static_cast<uint8_t>(std::round((vec(0) + vec(1) + vec(2)) / 3.0f));
                        break;
                    }
                    case StereoImageSide::RIGHT:
                    {
                        const cv::Vec4b& vec = right_frame.at<cv::Vec4b>(position.y, position.x);
                        value = static_cast<uint8_t>(std::round((vec(0) + vec(1) + vec(2)) / 3.0f));
                        break;
                    }
                    case StereoImageSide::DEPTH:
                    {
                        const cv::Vec4b& vec = depth_frame_rgba.at<cv::Vec4b>(position.y, position.x);
                        value = static_cast<uint8_t>(std::round((vec(0) + vec(1) + vec(2)) / 3.0f));
                        break;
                    }
                    }
                    frame_info.validation_pixel_values[i] = value;
                }

                return merged_frame;
            }

			const cv::Mat& convertDepthFrameFloatToUint16(const cv::Mat& depth_frame)
			{
				// Convert float to uint16_t depth image (scaled by 1000)
				static cv::Mat depth_frame_uint16;
				if (depth_frame_uint16.empty()
					|| depth_frame_uint16.rows != depth_frame.rows
					|| depth_frame_uint16.cols != depth_frame.cols) {
					depth_frame_uint16 = cv::Mat(depth_frame.rows, depth_frame.cols, CV_16S);
				}
#pragma omp parallel for
				for (int i = 0; i < depth_frame.rows * depth_frame.cols; ++i) {
					float depth = depth_frame.at<float>(i);
					if (std::isfinite(depth)) {
						depth_frame_uint16.at<uint16_t>(i) = static_cast<uint16_t>(depth * 1000);
					}
					else {
						depth_frame_uint16.at<uint16_t>(i) = 0.0;
					}
				}
				return depth_frame_uint16;
			}

			const cv::Mat& convertDepthFrameFloatToRGBA(const cv::Mat& depth_frame, StereoFrameInfo& frame_info, bool inverse_depth = true)
			{
                const uint8_t trunc_thres = DEPTH_UINT8_TRUNCATION_THRESHOLD;
				// Convert float to uint16_t depth image (scaled by 1000)
				static cv::Mat depth_frame_rgba;
				if (depth_frame_rgba.empty()
					|| depth_frame_rgba.rows != depth_frame.rows
					|| depth_frame_rgba.cols != depth_frame.cols) {
					depth_frame_rgba = cv::Mat(depth_frame.rows, depth_frame.cols, CV_8UC4);
				}
				float min_depth = std::numeric_limits<float>::infinity();
				float max_depth = 0;
#pragma omp parallel for
				for (int i = 0; i < depth_frame.rows * depth_frame.cols; ++i) {
					float depth = depth_frame.at<float>(i);
					if (std::isfinite(depth) && depth >= trunc_depth_min_ && depth <= trunc_depth_max_) {
						if (depth < min_depth) {
							min_depth = depth;
						}
						if (depth > max_depth) {
							max_depth = depth;
						}
					}
				}
				frame_info.min_depth = min_depth;
				frame_info.max_depth = max_depth;
				frame_info.truncation_threshold = trunc_thres;
				frame_info.inverse_depth = inverse_depth;
				float max_inv_depth = 1 / min_depth;
				float min_inv_depth = 1 / max_depth;
				const auto& convertInvDepthFloatToUint8 = [&](float inv_depth) { return trunc_thres + static_cast<uint8_t>(std::round((255 - trunc_thres) * (inv_depth - min_inv_depth) / (max_inv_depth - min_inv_depth))); };
				const auto& convertDepthFloatToUint8 = [&](float depth) { return trunc_thres + static_cast<uint8_t>(std::round((255 - trunc_thres) * (depth - min_depth) / (max_depth - min_depth))); };
				const auto& convertUint8ToInvDepthFloat = [&](uint8_t value) { return (value - trunc_thres) * (max_inv_depth - min_inv_depth) / (255.0f - trunc_thres) + min_inv_depth; };
				const auto& convertUint8ToDepthFloat = [&](uint8_t value) { return (value - trunc_thres) * (max_depth - min_depth) / (255.0f - trunc_thres) + min_depth; };

				float max_conv_error = 0;
#pragma omp parallel for
				for (int i = 0; i < depth_frame.rows * depth_frame.cols; ++i) {
					float depth = depth_frame.at<float>(i);
					uint8_t value;
					if (std::isfinite(depth) && depth >= trunc_depth_min_ && depth <= trunc_depth_max_) {
						// Debugging: Compute maximum depth error due to conversion
						float depth_conv;
						if (inverse_depth) {
							value = convertInvDepthFloatToUint8(1.0f / depth);
							depth_conv = 1.0f / convertUint8ToInvDepthFloat(value);
						}
						else {
							value = convertDepthFloatToUint8(depth);
							depth_conv = convertUint8ToDepthFloat(value);
						}
						AIT_ASSERT(value >= trunc_thres && value <= 255);
						float conv_error = std::abs(depth - depth_conv);
						if (conv_error > max_conv_error) {
							max_conv_error = conv_error;
						}
					}
					else {
						value = 0;
					}
					depth_frame_rgba.at<cv::Vec4b>(i) = { value, value, value, value };
				}
				std::cout << "Maximum conversion error: " << max_conv_error << std::endl;
				return depth_frame_rgba;
			}

			const cv::Mat& mergeStereoDepthFrames(const cv::Mat& left_frame, const cv::Mat& right_frame, const cv::Mat& depth_frame)
			{
				static cv::Mat merged_frame;
				// Make sure all frames have the same size and type
				if (left_frame.rows != right_frame.rows || left_frame.rows != depth_frame.rows)
				{
					throw std::runtime_error("Stereo and depth frames do not have the same height");
				}
				if (left_frame.cols != right_frame.cols || left_frame.cols != depth_frame.cols)
				{
					throw std::runtime_error("Stereo and depth frames do not have the same width");
				}
				if (left_frame.type() != right_frame.type() || left_frame.type() != depth_frame.type())
				{
					throw std::runtime_error("Stereo and depth frames do not have the same type");
				}
				int total_rows = left_frame.rows;
				int total_cols = left_frame.cols + right_frame.cols + depth_frame.cols;
				if (merged_frame.empty()
					|| merged_frame.rows != total_rows
					|| merged_frame.cols != total_cols
					|| merged_frame.type() != left_frame.type())
				{
					merged_frame = cv::Mat(total_rows, total_cols, left_frame.type());
					AIT_ASSERT(merged_frame.isContinuous());
				}
#pragma omp parallel sections
				{
					{ left_frame.copyTo(merged_frame.colRange(cv::Range(0, left_frame.cols))); }
#pragma omp section
					{ right_frame.copyTo(merged_frame.colRange(cv::Range(left_frame.cols, left_frame.cols + depth_frame.cols))); }
#pragma omp section
					{ depth_frame.copyTo(merged_frame.colRange(cv::Range(left_frame.cols + depth_frame.cols, total_cols))); }
				}

				return merged_frame;
			}

			void initializeValidationPixelLocations() {
			    AIT_ASSERT(stereo_calibration_.color_image_width_left > 0);
                AIT_ASSERT(stereo_calibration_.color_image_height_left > 0);
				std::mt19937_64 rng;
				std::uniform_int_distribution<unsigned int> width_dist(0, stereo_calibration_.color_image_width_left - 1);
				std::uniform_int_distribution<unsigned int> height_dist(0, stereo_calibration_.color_image_height_left - 1);
				const auto& generate_pixel_positions = [&](StereoImageSide side, unsigned int number_of_pixels, std::vector<ValidationPixelPosition>& positions) {
					for (unsigned int n = 0; n < NUMBER_OF_VALIDATION_PIXELS_PER_IMAGE; ) {
						ValidationPixelPosition position;
						position.side = side;
						position.x = width_dist(rng);
						position.y = height_dist(rng);
						bool duplicate = false;
						for (unsigned int j = 0; j < positions.size(); ++j) {
							const ValidationPixelPosition& other = positions[j];
							if (position.side == other.side && position.x == other.x && position.y == other.y) {
								duplicate = true;
								break;
							}
						}
						if (!duplicate) {
							positions.push_back(position);
							++n;
						}
					}
				};
				std::vector<ValidationPixelPosition> positions;
				generate_pixel_positions(StereoImageSide::LEFT, NUMBER_OF_VALIDATION_PIXELS_PER_IMAGE, user_parameters_.validation_pixel_positions);
				generate_pixel_positions(StereoImageSide::RIGHT, NUMBER_OF_VALIDATION_PIXELS_PER_IMAGE, user_parameters_.validation_pixel_positions);
				generate_pixel_positions(StereoImageSide::DEPTH, NUMBER_OF_VALIDATION_PIXELS_PER_IMAGE, user_parameters_.validation_pixel_positions);
			}

			EncodingGstreamerPipeline<PipelineUserDataType> pipeline_;
			bool pipeline_initialized_;
			std::atomic_bool terminate_;

			const std::shared_ptr<TNetworkClient> network_client_;
			const std::string remote_ip_;
			const unsigned int remote_port_;

			StereoCalibration stereo_calibration_;
			StereoFrameParameters user_parameters_;
			float trunc_depth_min_;
			float trunc_depth_max_;
			bool inverse_depth_;

			std::thread pipeline_output_thread_;
			StereoNetworkSensorClient<TNetworkClient, PipelineUserDataType, StereoFrameParameters> stereo_sensor_client_;

			bool use_compression_;
		};

	}

}
