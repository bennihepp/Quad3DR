#pragma once

#include <memory>
#include <ait/video/StereoNetworkSensorClient.h>
#include <ait/video/StereoNetworkSensorProtocol.h>
#if WITH_GSTREAMER
#include <gst/gst.h>
#include <ait/video/EncodingGstreamerPipeline.h>
#endif

namespace ait
{

	namespace video
	{
		template <typename TNetworkClient>
		class StereoNetworkSensorManager
		{
		public:
			StereoNetworkSensorManager(const StereoCalibration& stereo_calibration, const StereoClientType& client_type, const std::string& remote_ip, unsigned int remote_port)
				: network_client_(std::make_shared<TNetworkClient>()), stereo_sensor_client_(network_client_, stereo_calibration, client_type),
				remote_ip_(remote_ip), remote_port_(remote_port) {
#if WITH_GSTREAMER
				terminate_ = false;
				pipeline_.initialize();
				pipeline_.setStateChangeCallback(std::bind(&StereoNetworkSensorManager::stateChangeCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
#endif
			}

			~StereoNetworkSensorManager() {
			}

			void setDepthTruncation(double trunc_depth_min, double trunc_depth_max) {
				trunc_depth_min_ = trunc_depth_min;
				trunc_depth_max_ = trunc_depth_max;
			}

			void setInverseDepth(bool inverse_depth) {
				inverse_depth_ = inverse_depth;
			}

			const GstreamerPipeline<StereoFrameUserData>& getPipeline() const {
				return pipeline_;
			}

			void setUseCompression(bool use_compression) {
				use_compression_ = use_compression;
			}

#if WITH_GSTREAMER
			void stateChangeCallback(GstState old_state, GstState new_state, GstState pending_state) {
				if (new_state == GST_STATE_PLAYING) {
					std::cout << "Pipeline playing... Starting appsink thread" << std::endl;
					if (appsink_thread_.joinable()) {
						throw std::runtime_error("Network loop is already running");
					}
					terminate_ = false;
					appsink_thread_ = std::thread([this]() {
						appsinkLoop();
					});
				}
				else {
					if (appsink_thread_.joinable()) {
						std::cout << "Pipeline not playing anymore. Stopping appsink thread" << std::endl;
						terminate_ = true;
						appsink_thread_.join();
					}
				}
			}
#endif

			void start() {
#if WITH_GSTREAMER
				pipeline_.start();
#endif
			}

			void stop() {
#if WITH_GSTREAMER
				pipeline_.stop();
				terminate_ = true;
				if (appsink_thread_.joinable()) {
					appsink_thread_.join();
				}
#endif
			}

			void pushNewStereoFrame(const cv::Mat& left_frame, const cv::Mat& right_frame, const cv::Mat& depth_frame) {
				//cv::cvtColor(left_frame, left_frame, CV_BGRA2RGBA);
				//cv::cvtColor(right_frame, right_frame, CV_BGRA2RGBA);

#if WITH_GSTREAMER
				StereoFrameUserData user_data;
				const cv::Mat& depth_frame_rgba = convertDepthFrameFloatToRGBA(depth_frame, user_data, inverse_depth_);
				const cv::Mat& merged_frame = mergeStereoDepthFrames(left_frame, right_frame, depth_frame_rgba);
				MLIB_ASSERT(merged_frame.isContinuous());
				pipeline_.pushNewFrame(merged_frame, user_data);
#else
				const cv::Mat& depth_frame_uint16 = convertDepthFrameToUint16(left_frame, right_frame, depth_frame);
				stereo_sensor_client.sendFrameData(left_frame, right_frame, depth_frame_uint16, use_compression_);
#endif
			}

		private:
#if WITH_GSTREAMER
			void appsinkLoop() {
				while (!terminate_) {
					try {
						std::cout << "Trying to establish connection ..." << std::endl;
						network_client_->open(remote_ip_, remote_port_);
						std::cout << "Connection established" << std::endl;
						stereo_sensor_client_.sendCalibration();

						AppSinkQueue<StereoFrameUserData>& appsink_queue = pipeline_.getAppSinkQueue();
						bool appsinkCapsSent = false;
						RateCounter frame_rate_counter;
						while (!terminate_ || !appsink_queue.empty()) {
							std::unique_lock<std::mutex> lock(appsink_queue.getMutex());
							appsink_queue.getQueueFilledCondition().wait_for(lock, std::chrono::milliseconds(100), [&appsink_queue]() { return !appsink_queue.empty(); });
							if (!appsink_queue.empty()) {
								std::tuple<GstSampleWrapper, GstreamerBufferInfo, StereoFrameUserData> sample_tuple(std::move(appsink_queue.popFront(lock)));
								GstSampleWrapper& sample = std::get<0>(sample_tuple);
								const GstreamerBufferInfo& buffer_info = std::get<1>(sample_tuple);
								const StereoFrameUserData& user_data = std::get<2>(sample_tuple);
								lock.unlock();
								if (!pipeline_.isPlaying()) {
									continue;
								}
								GstBufferWrapper buffer(gst_sample_get_buffer(sample.get()), false);
								MLIB_ASSERT(GST_IS_BUFFER(buffer.get()));
								try {
									if (!appsinkCapsSent) {
										GstCapsWrapper appsink_caps = pipeline_.getAppSinkCaps();
										std::cout << "Sending Gstreamer Caps: " << appsink_caps.getString() << std::endl;
										stereo_sensor_client_.sendGstreamerCaps(std::move(pipeline_.getAppSinkCaps()));
										appsinkCapsSent = true;
									}

									// Set buffer_info data on buffer
									GST_BUFFER_PTS(buffer.get()) = buffer_info.pts;
									GST_BUFFER_DTS(buffer.get()) = buffer_info.dts;
									GST_BUFFER_DURATION(buffer.get()) = buffer_info.duration;
									GST_BUFFER_OFFSET(buffer.get()) = buffer_info.offset;
									GST_BUFFER_OFFSET_END(buffer.get()) = buffer_info.offset_end;

									frame_rate_counter.count();
									double rate;
									if (frame_rate_counter.reportRate(rate)) {
										std::cout << "Sending Gstreamer Buffer, offset=" << GST_BUFFER_OFFSET(buffer.get()) << ", pts=" << GST_BUFFER_PTS(buffer.get()) << ", size=" << buffer.getSize() << std::endl;
										std::cout << "Sending Gstreamer buffers with " << rate << " Hz" << std::endl;
									}
									stereo_sensor_client_.sendGstreamerBuffer(std::move(buffer), user_data);
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
				//runNetworkLoop(appsink_queue, stereo_sensor_client_);
				std::cout << "Exiting appsink thread" << std::endl;
			}
#endif

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

#if WITH_GSTREAMER
			const cv::Mat& convertDepthFrameFloatToRGBA(const cv::Mat& depth_frame, StereoFrameUserData& frame_data, bool inverse_depth = true)
			{
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
				frame_data.min_depth = min_depth;
				frame_data.max_depth = max_depth;
				frame_data.inverse_depth = inverse_depth;
				float max_inv_depth = 1 / min_depth;
				float min_inv_depth = 1 / max_depth;
				const auto& convertInvDepthFloatToUint8 = [&](float inv_depth) { return 1 + static_cast<uint8_t>(std::round(254 * (inv_depth - min_inv_depth) / (max_inv_depth - min_inv_depth))); };
				const auto& convertDepthFloatToUint8 = [&](float depth) { return 1 + static_cast<uint8_t>(std::round(254 * (depth - min_depth) / (max_depth - min_depth))); };
				const auto& convertUint8ToInvDepthFloat = [&](uint8_t value) { return (value - 1) * (max_inv_depth - min_inv_depth) / 254.0 + min_inv_depth; };
				const auto& convertUint8ToDepthFloat = [&](uint8_t value) { return (value - 1) * (max_depth - min_depth) / 254.0 + min_depth; };
				// TODO
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
						MLIB_ASSERT(value >= 0 && value <= 255);
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
					MLIB_ASSERT(merged_frame.isContinuous());
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
#endif

#if WITH_GSTREAMER
			EncodingGstreamerPipeline<StereoFrameUserData> pipeline_;
			std::atomic_bool terminate_;
#endif
			float trunc_depth_min_;
			float trunc_depth_max_;
			bool inverse_depth_;
			std::thread appsink_thread_;
			const std::shared_ptr<TNetworkClient> network_client_;
			StereoNetworkSensorClient<TNetworkClient, StereoFrameUserData> stereo_sensor_client_;
			const std::string remote_ip_;
			const unsigned int remote_port_;
			bool use_compression_;
		};

	}

}
