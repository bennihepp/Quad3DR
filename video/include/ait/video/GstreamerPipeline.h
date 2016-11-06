#pragma once

#include <iostream>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <gst\gst.h>
#include <gst\app\gstappsink.h>
#include <gst\app\gstappsrc.h>

#define ANNOTATE_EXC(type, s) type ## (std::string(FUNCTION_LINE_STRING).append(": ").append(s))

class RateCounter
{
public:
	using clock = std::chrono::high_resolution_clock;

	RateCounter(unsigned int report_count = 10)
		: report_count_(report_count) {
		reset();
	}

	void reset() {
		counter_ = 0;
		start_time_ = clock::now();
	}

	void count() {
		++counter_;
	}

	unsigned int getCount() const {
		return counter_;
	}

	double getRate() const {
		auto cur_time = clock::now();
		auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);
		double rate = counter_ / (0.001 * duration_ms.count());
		return rate;
	}

	bool reportRate(double& rate) {
		if (counter_ >= report_count_) {
			auto cur_time = clock::now();
			auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);
			rate = counter_ / (0.001 * duration_ms.count());
			start_time_ = cur_time;
			counter_ = 0;
			return true;
		}
		else {
			return false;
		}
	}

private:
	unsigned int counter_;
	unsigned int report_count_;
	std::chrono::time_point<clock> start_time_;
};

template <typename T>
class GstWrapper
{
public:
	GstWrapper(T* ptr)
		: ptr_(ptr) {
	}

	GstWrapper(const GstWrapper& other) = delete;
	void operator=(const GstWrapper& other) = delete;

	GstWrapper(GstWrapper&& other) {
		ptr_ = other.ptr_;
		other.ptr_ = nullptr;
	}

	void operator=(GstWrapper&& other) {
		unref();
		ptr_ = other.ptr_;
		other.ptr_ = nullptr;
	}

	virtual ~GstWrapper() {
	}

	const T* operator()() const {
		return ptr_;
	}

	T* operator()() {
		return ptr_;
	}

	const T* get() const {
		return ptr_;
	}

	T* get() {
		return ptr_;
	}

	//protected:
	virtual void unref() = 0;

private:
	T* ptr_;
};

class GstCapsWrapper : public GstWrapper<GstCaps>
{
public:
	GstCapsWrapper(GstCaps* ptr)
		: GstWrapper<GstCaps>(ptr), caps_string_(nullptr) {
	}

	GstCapsWrapper(GstCapsWrapper&& other)
		: GstWrapper(std::move(other)) {
		caps_string_ = other.caps_string_;
		other.caps_string_ = nullptr;
	}

	void operator=(GstCapsWrapper&& other) {
		GstWrapper::operator=(std::move(other));
		caps_string_ = other.caps_string_;
	}

	~GstCapsWrapper() override {
		unref();
	}

	const gchar* getString() {
		if (caps_string_ == nullptr) {
			caps_string_ = gst_caps_to_string(get());
		}
		return caps_string_;
	}

protected:
	void unref() override {
		if (caps_string_ != nullptr) {
			g_free(caps_string_);
		}
		if (get() != nullptr) {
			gst_caps_unref(get());
		}
	}

	gchar* caps_string_;
};

class GstBufferWrapper : public GstWrapper<GstBuffer>
{
public:
	class Error : public std::runtime_error
	{
	public:
		Error(const std::string &str)
			: std::runtime_error(str) {
		}
	};

	GstBufferWrapper(GstBuffer* ptr)
		: GstWrapper<GstBuffer>(ptr), mapped_(false), mapped_writable_(false), owns_buffer_(true) {
	}

	GstBufferWrapper(GstBuffer* ptr, bool owns_buffer)
		: GstWrapper<GstBuffer>(ptr), mapped_(false), mapped_writable_(false), owns_buffer_(owns_buffer) {
	}

	GstBufferWrapper(GstBufferWrapper&& other)
		: GstWrapper(std::move(other)){
		owns_buffer_ = other.owns_buffer_;
		info_ = other.info_;
		mapped_ = other.mapped_;
	}

	void operator=(GstBufferWrapper&& other) {
		GstWrapper::operator=(std::move(other));
		owns_buffer_ = other.owns_buffer_;
		info_ = other.info_;
		mapped_ = other.mapped_;
	}

	~GstBufferWrapper() override {
		unref();
	}

	gsize getSize() {
		ensureMapped();
		return info_.size;
	}

	const guint8* getData() {
		ensureMapped();
		return info_.data;
	}

	guint8* getDataWritable() {
		ensureMappedWritable();
		return info_.data;
	}

	const GstMapInfo& getMapInfo() {
		ensureMapped();
		return info_;
	}

	void map() {
		if (!mapped_) {
			if (gst_buffer_map(get(), &info_, GST_MAP_READ) != TRUE) {
				throw ANNOTATE_EXC(Error, std::string("Unable to map Gstreamer buffer"));
			}
			mapped_ = true;
		}
	}

	void mapWritable() {
		if (!mapped_writable_) {
			if (mapped_) {
				unmap();
			}
			if (gst_buffer_map(get(), &info_, GST_MAP_WRITE) != TRUE) {
				throw ANNOTATE_EXC(Error, std::string("Unable to map writable Gstreamer buffer"));
			}
			mapped_ = true;
			mapped_writable_ = true;
		}
	}

	void unmap() {
		gst_buffer_unmap(get(), &info_);
		mapped_ = false;
		mapped_writable_ = false;
	}

protected:
	void unref() override {
		if (get() != nullptr) {
			if (mapped_) {
				unmap();
			}
			if (owns_buffer_) {
				gst_buffer_unref(get());
			}
		}
	}

private:
	void ensureMapped() {
		if (!mapped_) {
			map();
		}
	}

	void ensureMappedWritable() {
		if (!mapped_writable_) {
			mapWritable();
		}
	}

	GstMapInfo info_;
	bool mapped_;
	bool mapped_writable_;
	bool owns_buffer_;
};

class GstSampleWrapper : public GstWrapper<GstSample>
{
public:
	GstSampleWrapper(GstSample* ptr)
		: GstWrapper<GstSample>(ptr) {
	}

	GstSampleWrapper(GstSampleWrapper&& other)
		: GstWrapper(std::move(other)) {
	}

	void operator=(GstSampleWrapper&& other) {
		GstWrapper::operator=(std::move(other));
	}

	~GstSampleWrapper() override {
		unref();
	}

	GstBufferWrapper getBuffer() {
		GstBuffer* gst_buffer = gst_sample_get_buffer(get());
		return GstBufferWrapper(gst_buffer, false);
	}

protected:
	void unref() override {
		if (get() != nullptr) {
			gst_sample_unref(get());
		}
	}
};

struct GstreamerBufferInfo {
	GstClockTime pts;
	GstClockTime dts;
	GstClockTime duration;
	guint64 offset;
	guint64 offset_end;
};

template <typename T>
class SPSCFixedQueue
{
public:
	SPSCFixedQueue(unsigned int max_queue_size = 5)
		: max_queue_size_(max_queue_size) {
	}

	unsigned int getMaxQueueSize() const {
		return max_queue_size_;
	}

	bool empty() const {
		return queue_.empty();
	}

	size_t size() const {
		return queue_.size();
	}

	template <typename TMutex>
	T popFront(const std::unique_lock<TMutex>& lock) {
		if (!lock) {
			throw ANNOTATE_EXC(std::runtime_error, "Lock has not been acquired");
		}
		return popFrontWithoutLocking();
	}

	template <typename TMutex>
	T popFront(const std::lock_guard<TMutex>& lock) {
		return popFrontWithoutLocking();
	}

	T popFront() {
		std::lock_guard<std::mutex> lock(mutex_);
		return popFrontWithoutLocking();
	}

	std::mutex& getMutex() {
		return mutex_;
	}

	std::condition_variable& getQueueFilledCondition() {
		return queue_filled_condition_;
	}

	bool pushBack(T& element) {
		if (queue_.size() < max_queue_size_) {
			{
				std::lock_guard<std::mutex> lock(mutex_);
				queue_.push_back(std::move(element));
			}
			queue_filled_condition_.notify_one();
			return true;
		}
		else {
			return false;
		}
	}

private:
	T popFrontWithoutLocking() {
		T element(std::move(queue_.front()));
		queue_.pop_front();
		return element;
	}

	std::deque<T> queue_;
	unsigned int max_queue_size_;
	std::mutex mutex_;
	std::condition_variable queue_filled_condition_;
};

template <typename TUserData>
class GstreamerPipeline;

template <typename TUserData>
class AppSinkQueue : public SPSCFixedQueue<std::tuple<std::vector<uint8_t>, GstreamerBufferInfo, TUserData>>
{
public:
	const unsigned int FRAME_DROP_REPORT_RATE = 10;

	AppSinkQueue(unsigned int max_queue_size = 5, unsigned int user_data_queu_size = 3)
		: SPSCFixedQueue(max_queue_size), user_data_queu_size_(user_data_queu_size), first_frame_removed_(false), src_overflow_counter_(0), sink_overflow_counter_(0) {
	}

	bool pushData(GstAppSrc* appsrc, GstBufferWrapper buffer, const GstreamerBufferInfo& buffer_info, const TUserData& user_data) {
		gst_buffer_ref(buffer.get());
		MLIB_ASSERT(GST_IS_BUFFER(buffer.get()));
		GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer.get());
		if (ret == GST_FLOW_OK) {
			std::lock_guard<std::mutex> lock(getMutex());
			if (user_data_queue_.size() >= user_data_queu_size_) {
				// This is kind of hacky to ensure the user data queue is not blocking new frames
				user_data_queue_.pop_front();
				++src_overflow_counter_;
				if (src_overflow_counter_ >= FRAME_DROP_REPORT_RATE) {
					std::cout << "WARNING: Appsrc user data queue is full. Dropped " << FRAME_DROP_REPORT_RATE << " user data entries" << std::endl;
					src_overflow_counter_ = 0;
				}
			}
			user_data_queue_.push_back(std::make_pair(buffer_info, user_data));
			return true;
		}
		else {
			return false;
		}
	}

private:
	friend class GstreamerPipeline<TUserData>;

	GstFlowReturn newSampleCallback(GstAppSink* appsink) {
		// Retrieve the sample
		GstSample* gst_sample = gst_app_sink_pull_sample(appsink);
		//g_signal_emit_by_name(appsink, "pull-sample", &gst_sample);
		if (gst_sample == nullptr) {
			if (gst_app_sink_is_eos(appsink) == TRUE) {
				std::cout << "Received EOS condition" << std::endl;
			}
			else {
				throw std::runtime_error("Unable to pull new sample from appsink");
			}
		}
		else {
			GstSampleWrapper sample(gst_sample);
			// Copy buffer
			std::vector<uint8_t> buffer_data;
			{
				GstBufferWrapper buffer = sample.getBuffer();
				buffer_data.resize(buffer.getSize());
				std::copy(buffer.getData(), buffer.getData() + buffer.getSize(), buffer_data.begin());
				// Delete sample as fast as possible so that appsink has free buffers available
				GstSampleWrapper deleted_sample(std::move(sample));
			}
			// TODO: Check if this can be guaranteed to work
			//if (!first_frame_removed_) {
			//	// The first frame will usually be lost in the Gstreamer pipeline. Here we delete the corresponding info and user data
			//	if (user_data_queue_.size() > 1) {
			//		user_data_queue_.pop_front();
			//		first_frame_removed_ = true;
			//	}
			//	else {
			//		std::cerr << "WARNING: No lost frame detected in Gstreamer pipeline" << std::endl;
			//	}
			//}
			if (user_data_queue_.size() > 10) {
				// Warn if we cannot ensure correspondence of frames and user data
				std::cerr << "WARNING: More userdata available than frames (" << user_data_queue_.size() << "). Either frames are coming in too fast or frames get dropped in the pipeline" << std::endl;
			}
			std::pair<GstreamerBufferInfo, TUserData> pair(std::move(user_data_queue_.front()));
			// This is kind of hacky to ensure the user data queue is never empty
			if (user_data_queue_.size() > 1) {
				user_data_queue_.pop_front();
			}
			const GstreamerBufferInfo& buffer_info = pair.first;
			const TUserData& user_data = pair.second;
			auto sample_tuple = std::make_tuple(std::move(buffer_data), buffer_info, user_data);
			if (!pushBack(sample_tuple)) {
				++sink_overflow_counter_;
				if (sink_overflow_counter_ >= FRAME_DROP_REPORT_RATE) {
					std::cout << "WARNING: Appsink sample queue is full. Dropped " << FRAME_DROP_REPORT_RATE << " frames" << std::endl;
					sink_overflow_counter_ = 0;
				}
			}
		}

		return GST_FLOW_OK;
	}

	std::deque<std::pair<GstreamerBufferInfo, TUserData>> user_data_queue_;
	unsigned int user_data_queu_size_;
	bool first_frame_removed_;
	unsigned int src_overflow_counter_;
	unsigned int sink_overflow_counter_;
};

template <typename TUserData>
class GstreamerPipeline
{
public:
	using clock = std::chrono::system_clock;
	const unsigned int WATCHDOG_RESET_COUNT = 10;
	const std::chrono::seconds WATCHDOG_TIMEOUT = std::chrono::seconds(2);

	GstreamerPipeline()
		: pipeline_(nullptr), pipeline_state(GST_STATE_NULL) {
	}

	GstreamerPipeline(const GstreamerPipeline&) = delete;
	void operator=(const GstreamerPipeline&) = delete;

	~GstreamerPipeline() {
		stop();
		gst_object_unref(pipeline_);
	}

	void initialize() {
		if (pipeline_ != nullptr) {
			throw ANNOTATE_EXC(std::runtime_error, "Pipeline was already initialized");
		}
		appsrc_ = GST_APP_SRC(gst_element_factory_make("appsrc", "source"));
		if (appsrc_ == nullptr) {
			throw std::runtime_error("Unable to create app source element");
		}
		g_object_set(appsrc_, "stream-type", GST_APP_STREAM_TYPE_STREAM, nullptr);
		//g_object_set(appsrc_, "format", GST_FORMAT_TIME, nullptr);
		g_object_set(appsrc_, "format", GST_FORMAT_BYTES, nullptr);
		//g_object_set(appsrc_, "format", GST_FORMAT_BUFFERS, nullptr);
		g_object_set(appsrc_, "block", TRUE, nullptr);
		g_object_set(appsrc_, "max-bytes", 200000, nullptr);

		appsink_ = GST_APP_SINK(gst_element_factory_make("appsink", "sink"));
		if (appsink_ == nullptr) {
			throw std::runtime_error("Unable to create app sink element");
		}
		g_object_set(appsink_, "emit-signals", TRUE, nullptr);
		g_object_set(appsink_, "sync", FALSE, nullptr);
		//g_object_set(appsink_, "drop", TRUE, nullptr);
		//g_object_set(appsink_, "max-buffers", 5, nullptr);
		// Connect new sample signal to AppSinkQueue
		g_signal_connect(appsink_, "new-sample", G_CALLBACK(GstreamerPipeline<TUserData>::newAppsinkSampleCallbackStatic), this);

		pipeline_ = createPipeline(appsrc_, appsink_);
	}

	GstPipeline* getNativePipeline() {
		ensureInitialized();
		return pipeline_;
	}

	GstAppSrc* getNativeAppSrc() {
		ensureInitialized();
		return appsrc_;
	}

	GstAppSink* getNativeAppSink() {
		ensureInitialized();
		return appsink_;
	}

	bool hasSamples() const {
		ensureInitialized();
		GstClockTime timeout = 0;
		GstSample* gst_sample = gst_app_sink_try_pull_sample(appsink_, timeout);
		if (gst_sample == nullptr) {
			if (gst_app_sink_is_eos(appsink_) == TRUE) {
			}
			return false;
		}
		return true;
	}

	AppSinkQueue<TUserData>& getAppSinkQueue() {
		ensureInitialized();
		return appsink_queue_;
	}

	GstCapsWrapper getAppSinkCaps() {
		GstPad *appsink_sink_pad = gst_element_get_static_pad(GST_ELEMENT(appsink_), "sink");
		if (appsink_sink_pad == nullptr) {
			throw std::runtime_error("Unable to get appsink sink pad");
		}
		GstCaps* gst_caps = gst_pad_get_current_caps(appsink_sink_pad);
		GstCapsWrapper appsink_caps(gst_caps);
		return appsink_caps;
	}

	bool setAppsrcCaps(const GstCapsWrapper& caps) {
		ensureInitialized();
		gst_app_src_set_caps(appsrc_, caps.get());
		return true;
	}

	bool pushBufferToAppsrc(GstBufferWrapper& buffer, const TUserData& user_data) {
		ensureInitialized();
		MLIB_ASSERT(GST_IS_BUFFER(buffer.get()));

		clock::time_point now = clock::now();
		if (now - last_appsink_sample_time_ >= WATCHDOG_TIMEOUT) {
			++watchdog_counter_;
			if (watchdog_counter_ >= WATCHDOG_RESET_COUNT) {
				std::cout << "WARNING: Pipeline watchdog activated. Restarting pipeline" << std::endl;
				stop();
				start();
				return false;
			}
		}
		else if (watchdog_counter_ > 0) {
			watchdog_counter_ = 0;
		}

		GstreamerBufferInfo buffer_info;
		buffer_info.pts = GST_BUFFER_PTS(buffer.get());
		buffer_info.dts = GST_BUFFER_DTS(buffer.get());
		buffer_info.duration = GST_BUFFER_DURATION(buffer.get());
		buffer_info.offset = GST_BUFFER_OFFSET(buffer.get());
		buffer_info.offset_end = GST_BUFFER_OFFSET_END(buffer.get());

		// TODO: get from appsrc caps
		double frame_period = 1 / 10.;
		// Overwrite timing information to make sure that pipeline runs through
		static guint64 frame_counter = 0;
		GstClock* clock = gst_pipeline_get_clock(getNativePipeline());
		GstClockTime time_now = gst_clock_get_time(clock);
		GstClockTime gst_frame_period = static_cast<guint64>(GST_SECOND * frame_period);
		static GstClockTime time_previous = time_now - gst_frame_period;

		if (time_now - time_previous <= gst_frame_period) {
			GST_BUFFER_PTS(buffer.get()) = time_previous + gst_frame_period;
		}
		else {
			GST_BUFFER_PTS(buffer.get()) = time_now;
		}
		GST_BUFFER_DTS(buffer.get()) = GST_CLOCK_TIME_NONE;
		GST_BUFFER_DURATION(buffer.get()) = static_cast<guint64>(GST_SECOND * frame_period);
		GST_BUFFER_OFFSET(buffer.get()) = frame_counter;
		GST_BUFFER_OFFSET_END(buffer.get()) = GST_BUFFER_OFFSET_NONE;

		// TODO: get from appsrc caps
		//double frame_period = 1 / 10.;
		//static GstClockTime time_start = gst_clock_get_time(clock);
		//GST_BUFFER_PTS(buffer.get()) = time_start + static_cast<guint64>(GST_SECOND * frame_counter * frame_period);
		//GST_BUFFER_DTS(buffer.get()) = GST_CLOCK_TIME_NONE;
		//GST_BUFFER_DURATION(buffer.get()) = static_cast<guint64>(GST_SECOND * frame_period);
		//GST_BUFFER_OFFSET(buffer.get()) = frame_counter;
		//GST_BUFFER_OFFSET_END(buffer.get()) = GST_BUFFER_OFFSET_NONE;

		++frame_counter;
		time_previous = time_now;

		// Disable timing information in buffer
		//static unsigned int buffer_counter = 0;
		//GST_BUFFER_PTS(buffer.get()) = GST_CLOCK_TIME_NONE;
		//GST_BUFFER_DTS(buffer.get()) = GST_CLOCK_TIME_NONE;
		//GST_BUFFER_DURATION(buffer.get()) = GST_CLOCK_TIME_NONE;
		//GST_BUFFER_OFFSET(buffer.get()) = buffer_counter;
		//GST_BUFFER_OFFSET_END(buffer.get()) = GST_BUFFER_OFFSET_NONE;
		//++buffer_counter;

		return appsink_queue_.pushData(appsrc_, std::move(buffer), buffer_info, user_data);
	}

	void start() {
		ensureInitialized();
		if (message_thread_.joinable()) {
			stop();
		}
		// Start playing
		GstStateChangeReturn ret = gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);
		if (ret == GST_STATE_CHANGE_FAILURE) {
			gst_object_unref(pipeline_);
			throw std::runtime_error("Unable to set pipeline state");
		}
		watchdog_counter_ = 0;
		last_appsink_sample_time_ = clock::now();
		terminate_ = false;
		message_thread_ = std::thread([this]() {
			this->gstreamerLoop();
		});
	}

	void stop() {
		ensureInitialized();
		gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_NULL);
		terminate_ = true;
		if (message_thread_.joinable()) {
			message_thread_.join();
		}
	}

	GstState getState() const {
		return pipeline_state;
	}

	bool isPlaying() const {
		return pipeline_state == GST_STATE_PLAYING;
	}

	void setStateChangeCallback(const std::function<void(GstState, GstState, GstState)> callback) {
		state_change_callback_ = callback;
	}

protected:
	virtual GstPipeline* createPipeline(GstAppSrc* appsrc, GstAppSink* appsink) const = 0;

	virtual void gstreamerLoop()
	{
		// Wait until error or EOS
		GstBus *bus = gst_element_get_bus(GST_ELEMENT(pipeline_));
		while (!terminate_) {
			GstMessage *msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_STATE_CHANGED));

			if (msg != nullptr) {
				GError *err;
				gchar *debug_info;
				switch (GST_MESSAGE_TYPE(msg)) {
				case GST_MESSAGE_ERROR:
					gst_message_parse_error(msg, &err, &debug_info);
					std::cerr << "Error received from element " << GST_OBJECT_NAME(msg->src) << ": " << err->message << std::endl;
					std::cerr << "Debugging information: " << (debug_info ? debug_info : "none") << std::endl;
					g_clear_error(&err);
					g_free(debug_info);
					terminate_ = true;
					break;
				case GST_MESSAGE_EOS:
					std::cout << "Stream finished." << std::endl;
					terminate_ = true;
					break;
				case GST_MESSAGE_STATE_CHANGED:
				{
					GstState old_state, new_state, pending_state;
					gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
					if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline_)) {
						//if (new_state == GST_STATE_PLAYING) {
						//	//std::cout << "Resetting time" << std::endl;
						//	//data.start_time = std::chrono::system_clock::now();
						//	//data.timestamp = 0;
						//}
						pipeline_state = new_state;
						std::cout << "Pipeline state changed from " << gst_element_state_get_name(old_state)
							<< " to " << gst_element_state_get_name(new_state) << std::endl;
						if (state_change_callback_) {
							state_change_callback_(old_state, new_state, pending_state);
						}
					}
					break;
				}
				default:
					std::cerr << "Unexpected message received." << std::endl;
					break;
				}
				gst_message_unref(msg);
			}
		}
		gst_object_unref(bus);
	}

private:
	void ensureInitialized() const {
		if (pipeline_ == nullptr) {
			throw ANNOTATE_EXC(std::runtime_error, "Pipeline was not initialized");
		}
	}

	static GstFlowReturn newAppsinkSampleCallbackStatic(GstElement* sink, GstreamerPipeline* data) {
		return data->newAppsinkSampleCallback(GST_APP_SINK(sink));
	}

	GstFlowReturn newAppsinkSampleCallback(GstAppSink* appsink) {
		MLIB_ASSERT(appsink == appsink_);
		last_appsink_sample_time_ = clock::now();
		return appsink_queue_.newSampleCallback(appsink_);
	}

	GstPipeline* pipeline_;
	GstState pipeline_state;
	GstAppSrc* appsrc_;
	GstAppSink* appsink_;
	AppSinkQueue<TUserData> appsink_queue_;
	using clock = std::chrono::system_clock;
	unsigned int watchdog_counter_;
	std::chrono::time_point<clock> last_appsink_sample_time_;
	std::atomic_bool terminate_;
	std::thread message_thread_;
	std::function<void(GstState, GstState, GstState)> state_change_callback_;
};
