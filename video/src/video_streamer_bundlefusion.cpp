#define WITH_GSTREAMER 1
#define SIMULATE_ZED 0

#include <ait/mLib.h>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>
#include <csignal>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#if WITH_GSTREAMER
	#include <gst/gst.h>
	#include <gst/app/gstappsrc.h>
	#include <gst/app/gstappsink.h>
#endif
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <ait/video/video_source_zed.h>
#include <ait/BoostNetworkClientTCP.h>
#include <ait/BoostNetworkClientUDP.h>
#include <ait/video/StereoNetworkSensorManager.h>
#include <ait/video/StereoNetworkSensorClient.h>
#if WITH_GSTREAMER
#include <ait/video/EncodingGstreamerPipeline.h>
#endif

volatile bool g_abort;

void signalHandler(int sig)
{
	g_abort = true;
	std::cout << "Received CTRL-C. Aborting" << std::endl;
}

class PaceMaker
{
public:
	using clock = std::chrono::high_resolution_clock;
	PaceMaker(double desired_rate)
		: desired_period_(1 / desired_rate), time_ahead_(0) {
		last_time_ = clock::now();
	}

	void sleep() {
		std::chrono::time_point<clock> now = clock::now();
		std::chrono::duration<double> duration = now - last_time_;
		time_ahead_ = time_ahead_ + desired_period_ - duration;
		if (time_ahead_.count() < 0) {
			time_ahead_ = std::chrono::duration<double>(0);
		}
		std::chrono::milliseconds sleep_time_ms(static_cast<int64_t>(1000 * time_ahead_.count()));
		std::this_thread::sleep_for(sleep_time_ms);
		last_time_ = now;
	}

private:
	std::chrono::time_point<clock> last_time_;
	std::chrono::duration<double> desired_period_;
	std::chrono::duration<double> time_ahead_;
};
StereoCalibration convertStereoCalibration(const ait::stereo::StereoCameraCalibration& stereo_calibration)
{
	StereoCalibration stereo_sensor_calibration;
	// Image widths and heights
	stereo_sensor_calibration.m_ColorImageWidthLeft = stereo_calibration.image_size.width;
	stereo_sensor_calibration.m_ColorImageHeightLeft = stereo_calibration.image_size.height;
	stereo_sensor_calibration.m_ColorImageWidthRight = stereo_calibration.image_size.width;
	stereo_sensor_calibration.m_ColorImageHeightRight = stereo_calibration.image_size.height;
	stereo_sensor_calibration.m_DepthImageWidth = stereo_calibration.image_size.width;
	stereo_sensor_calibration.m_DepthImageHeight = stereo_calibration.image_size.height;
	// Left intrinsics and extrinsics
	ml::mat4f left_intrinsics = ml::mat4f::identity();
	left_intrinsics.setMatrix3x3(ml::eigenutil::EigenToMlib(stereo_calibration.left.getCameraMatrixEigen()));
	ml::mat4f left_extrinsics;
	left_extrinsics = ml::eigenutil::EigenToMlib(stereo_calibration.getLeftExtrinsicsEigen());
	stereo_sensor_calibration.m_CalibrationColorLeft.setMatrices(left_intrinsics, left_extrinsics);
	// Depth intrinsics and extrinsics
	stereo_sensor_calibration.m_CalibrationDepth.setMatrices(left_intrinsics, left_extrinsics);
	// Right intrinsics and extrinsics
	ml::mat4f right_intrinsics = ml::mat4f::identity();
	right_intrinsics.setMatrix3x3(ml::eigenutil::EigenToMlib(stereo_calibration.right.getCameraMatrixEigen()));
	ml::mat4f right_extrinsics;
	right_extrinsics = ml::eigenutil::EigenToMlib(stereo_calibration.getRightExtrinsicsEigen());
	stereo_sensor_calibration.m_CalibrationColorRight.setMatrices(right_intrinsics, right_extrinsics);

	return stereo_sensor_calibration;
}

int main(int argc, char **argv)
{
	namespace avo = ait::video;
	namespace po = boost::program_options;

#if WITH_GSTREAMER
	GError *gst_err;
	gboolean gst_initialized = gst_init_check(&argc, &argv, &gst_err);
	if (gst_initialized == FALSE) {
		std::cerr << "ERROR: gst_init_check failed: " << gst_err->message << std::endl;
		throw std::runtime_error("Unable to initialize gstreamer");
	}
#endif

	try
	{
		bool show = false;
		bool rotate = false;
		bool use_compression = false;
		std::string remote_ip;
		int remote_port;

		po::options_description generic_options("Allowed options");
		generic_options.add_options()
			("help", "Produce help message")
			("rotate", po::bool_switch(&rotate)->default_value(false), "Rotate stereo images")
			("show", po::bool_switch(&show)->default_value(false), "Render output")
			;

		po::options_description zed_options("ZED options");
		zed_options.add_options()
			("svo-file", po::value<std::string>(), "SVO file to use")
			("mode", po::value<int>()->default_value(3), "ZED resolution mode")
			("fps", po::value<double>()->default_value(15), "Frame-rate to capture")
			("zed-params", po::value<std::string>(), "ZED parameter file")
			("calib-file", po::value<std::string>()->default_value("camera_calibration_stereo.yml"), "Stereo calibration file.")
			;

		po::options_description network_options("Network options");
		network_options.add_options()
			("remote-ip", po::value<std::string>(&remote_ip)->default_value("127.0.0.1"), "Remote IP address")
			("remote-port", po::value<int>(&remote_port)->default_value(1337), "Remote port")
			("compress", po::bool_switch(&use_compression)->default_value(false), "Use compression")
			;

		po::options_description options;
		options.add(generic_options);
		options.add(zed_options);
		options.add(network_options);
		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
		if (vm.count("help"))
		{
			std::cout << options << std::endl;
			return 1;
		}

		po::notify(vm);

		// Initialize ZED camera
		double camera_framerate;
		std::function<bool (cv::Mat&, cv::Mat&, cv::Mat&)> retrieve_frame_fn;
		avo::VideoSourceZED *video_ptr = new avo::VideoSourceZED();
		std::cout << "Initializing ZED camera... " << std::flush;
#if SIMULATE_ZED
		camera_framerate = 15;
#else
#if _DEBUG
		video_ptr->open(static_cast<sl::zed::ZEDResolution_mode>(vm["mode"].as<int>()));
#else
		if (vm.count("svo-file")) {
			video_ptr->open(vm["svo-file"].as<std::string>());
		}
		else {
			video_ptr->open(static_cast<sl::zed::ZEDResolution_mode>(vm["mode"].as<int>()));
		}
#endif
		std::cout << "Done." << std::endl;
		if (vm.count("fps")) {
			if (!video_ptr->setFPS(vm["fps"].as<double>())) {
				std::cerr << "Setting camera FPS failed" << std::endl;
			}
		}
		camera_framerate = video_ptr->getFPS();
#endif
		video_ptr = video_ptr;
		retrieve_frame_fn = [video_ptr](cv::Mat& left_frame, cv::Mat& right_frame, cv::Mat& depth_frame) -> bool
		{
		//  cv::Size display_size(data->video_ptr->getWidth(), data->video_ptr->getHeight());
		//  cv::Mat left_grabbed_frame(display_size, CV_8UC4);
		//  cv::Mat right_grabbed_frame(display_size, CV_8UC4);
		//  cv::Mat depth_grabbed_frame(display_size, CV_8UC4);
			if (!video_ptr->grab()) {
				throw std::runtime_error("Failed to grab next frame.");
			}
			// Retrieve stereo and depth frames
			if (!video_ptr->retrieveLeft(&left_frame)) {
				throw std::runtime_error("Failed to retrieve left frame");
			}
			if (!video_ptr->retrieveRight(&right_frame)) {
				throw std::runtime_error("Failed to retrieve right frame");
			}
			if (!video_ptr->retrieveDepthFloat(&depth_frame)) {
				throw std::runtime_error("Failed to retrieve depth frame");
			}
			// Make sure all frames have the same size and type
			if (left_frame.rows != right_frame.rows || left_frame.rows != depth_frame.rows) {
				throw std::runtime_error("Stereo and depth frames do not have the same height");
			}
			if (left_frame.cols != right_frame.cols || left_frame.cols != depth_frame.cols) {
				throw std::runtime_error("Stereo and depth frames do not have the same width");
			}
			if (left_frame.type() != right_frame.type()) {
				throw std::runtime_error("Stereo frames do not have the same type");
			}
			if (depth_frame.type() != CV_32F) {
				throw std::runtime_error("Depth frame does not have single-precision floating point type");
			}
			return true;
		};
		std::cout << "Grabbing frames with " << camera_framerate << " Hz" << std::endl;


		std::atomic_bool terminate;
		terminate = false;

		std::signal(SIGINT, signalHandler);

		using NetworkClient = ait::BoostNetworkClientTCP;
		//using NetworkClient = ait::BoostNetworkClientUDP;

#if SIMULATE_ZED
		StereoCalibration stereo_sensor_calibration;
		memset(&stereo_sensor_calibration, 0, sizeof(stereo_sensor_calibration));
		stereo_sensor_calibration.m_ColorImageWidthLeft = 640;
		stereo_sensor_calibration.m_ColorImageWidthRight = 640;
		stereo_sensor_calibration.m_DepthImageWidth = 640;
		stereo_sensor_calibration.m_ColorImageHeightLeft = 360;
		stereo_sensor_calibration.m_ColorImageHeightRight = 360;
		stereo_sensor_calibration.m_DepthImageHeight = 360;
#else
		//// Read stereo calibration
		ait::stereo::StereoCameraCalibration stereo_calibration = video_ptr->getStereoCalibration();
		StereoCalibration stereo_sensor_calibration = convertStereoCalibration(stereo_calibration);
#endif
		ait::video::StereoNetworkSensorManager<NetworkClient> manager(stereo_sensor_calibration, StereoClientType::CLIENT_ZED, remote_ip, remote_port);
		manager.setUseCompression(use_compression);
		manager.setDepthTruncation(1.0f, 12.0f);
		manager.setInverseDepth(false);
		manager.start();

		double desired_framerate;
		if (vm.count("fps")) {
			desired_framerate = vm["fps"].as<double>();
		}
		else {
			desired_framerate = camera_framerate;
		}

		std::chrono::seconds wait_for_playing_timeout(1);
		cv::Mat left_frame, right_frame, depth_frame;
		auto start_time = std::chrono::high_resolution_clock::now();
		RateCounter frame_rate_counter;
		PaceMaker pace(desired_framerate);
		while (!terminate && !g_abort) {
			if (!manager.getPipeline().isPlaying()) {
				auto now = std::chrono::high_resolution_clock::now(); 
				if (now - start_time > wait_for_playing_timeout) {
					std::cerr << "ERROR: Pipeline did not start playing in due time" << std::endl;
					break;
				}
			}

			retrieve_frame_fn(left_frame, right_frame, depth_frame);

			if (rotate) {
				std::swap(left_frame, right_frame);
				const int flipCode = -1;
#pragma omp parallel sections
				{
					if (rotate) {
						cv::flip(left_frame, left_frame, flipCode);
					}
#pragma omp section
					if (rotate) {
						cv::flip(right_frame, right_frame, flipCode);
					}
#pragma omp section
					if (rotate) {
						cv::flip(depth_frame, depth_frame, flipCode);
					}
				}
			}

			manager.pushNewStereoFrame(left_frame, right_frame, depth_frame);

			frame_rate_counter.count();
			double rate;
			if (frame_rate_counter.reportRate(rate)) {
				std::cout << "Running with " << rate << " Hz" << std::endl;
			}

			if (show) {
				cv::imshow("left frame", left_frame);
				cv::imshow("right frame", right_frame);
				cv::imshow("depth frame", depth_frame);
				if (cv::waitKey(1) == 27) {
					terminate = true;
				}
			}

			pace.sleep();
		}
		manager.stop();

		//delete video_ptr;
	}
	catch (const po::required_option& err)
	{
		std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
	}
	catch (const po::error& err)
	{
		std::cerr << "Error parsing command line: " << err.what() << std::endl;
	}
	catch (const std::exception& err)
	{
		std::cerr << "Exception: << " << err.what() << std::endl;
		throw;
	}

	return 0;
}
