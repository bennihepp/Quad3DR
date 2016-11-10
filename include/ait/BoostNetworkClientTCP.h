//==================================================
// BoostNetworkClientTCP.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#pragma once

#include <iostream>
#include <stdexcept>
#include <future>
#include <memory>
#include <boost/asio.hpp>
#include <ait/serializable.h>

#define FUNCTION_LINE_STRING (std::string(__FILE__) + " [" + std::string(__FUNCTION__) + ":" + std::to_string(__LINE__) + "]")
#define ANNOTATE_EXC(type, s) type (std::string(FUNCTION_LINE_STRING).append(": ").append(s))
#define ANNOTATE_EXC_BOOST(type, s, ec) type (std::string(FUNCTION_LINE_STRING).append(": ").append(s), ec)

namespace ait
{

class BoostNetworkClientTCP
{
public:
	using tcp = boost::asio::ip::tcp;

	class Error : public std::runtime_error
	{
	public:
		Error(const std::string &str)
			: std::runtime_error(str) {
		}

		Error(const std::string &str, const boost::system::error_code& ec)
			: std::runtime_error(str + ": " + ec.message()), ec_(ec) {
		}

		const boost::system::error_code& getBoostErrorCode() const {
			return ec_;
		}

	private:
		boost::system::error_code ec_;
	};

	class ReaderWriter : public ait::Writer, public ait::Reader
	{
	public:
		ReaderWriter(BoostNetworkClientTCP* connection)
			: connection_(connection) {
		}

		virtual size_t read(void* data, size_t size)  {
			return connection_->receiveDataBlocking((uint8_t*)data, size);
		}

		virtual size_t write(const void* data, size_t size)  {
			return connection_->sendDataBlocking((const uint8_t*)data, size);
		}

		template <typename T>
		size_t read(T& value) {
			return Reader::read(value);
		}

		template <typename T>
		size_t write(const T& value) {
			return Writer::write(value);
		}

	private:
		BoostNetworkClientTCP* connection_;
	};

	BoostNetworkClientTCP()
		: socket_(io_service_), connected_(false), reader_writer_(this) {
		io_service_work_ = std::make_shared<boost::asio::io_service::work>(io_service_);
		io_service_thread_ = std::thread([this]() {
			run();
		});
	}

	~BoostNetworkClientTCP() {
		io_service_work_.reset();
		close();
		io_service_.stop();
		io_service_thread_.join();
	}

	bool isConnected() const {
		return socket_.is_open();
	}

	//! Connects the socket to a remote end.
	void open(const std::string& address, int port) {
		if (isConnected()) {
			throw ANNOTATE_EXC(Error, "Client socket already open");
		}

		tcp::resolver resolver(io_service_);
		tcp::resolver::query query(address, std::to_string(port));
		tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
		boost::system::error_code ec;
		boost::asio::connect(socket_, endpoint_iterator, ec);
		if (ec) {
			throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to connect to server"), ec);
		}
		connected_ = true;
	}

	//! Opens a network socket and waits for an incoming connection.
	std::future<bool> asyncOpen(const std::string& address, unsigned int port) {
		if (isConnected()) {
			throw ANNOTATE_EXC(Error, "Client socket already open");
		}

		tcp::resolver resolver(io_service_);
		tcp::resolver::query query(address, std::to_string(port));
		tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
//		boost::system::error_code ec;
//		boost::asio::connect(socket_, endpoint_iterator, ec);
//		if (ec) {
//			throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to connect to server"), ec);
//		}
//		connected_ = true;

		std::cout << "Trying to connect to " << endpoint_iterator->endpoint().address().to_string() << std::endl;
		async_success_promise_ = std::promise<bool>();
		socket_.async_connect(endpoint_iterator->endpoint(), [this](const boost::system::error_code& ec) {
			if (ec) {
				//success_promise.set_value(false);
				throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to connect to server"), ec);
			}
			else {
				connected_ = true;
				async_success_promise_.set_value(true);
			}
		});
		return async_success_promise_.get_future();
	}

	void asyncOpenCancel() {
		boost::system::error_code ec;
		socket_.cancel(ec);
		if (ec) {
			throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to cancel async connect"), ec);
		}
	}

	//! returns the number of received bytes
	size_t receiveDataBlocking(uint8_t* data, size_t byteSize) {
		boost::system::error_code ec;
		size_t totalReceivedBytes = 0;
		while (totalReceivedBytes < byteSize) {
			size_t receivedBytes = boost::asio::read(socket_, boost::asio::buffer(data + totalReceivedBytes, byteSize - totalReceivedBytes), ec);
			if (ec && ec != boost::asio::error::message_size) {
				throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to receive data on socket"), ec);
			}
			totalReceivedBytes += receivedBytes;
		}
		return totalReceivedBytes;
	}

	//! Blocking call. Returns the number of sent bytes.
	template <typename T>
	size_t receiveDataBlocking(std::vector<T>& data) {
		return receiveDataBlocking(reinterpret_cast<uint8_t*>(data.data()), data.size() * sizeof(T));
	}

	//! Blocking call. Returns the number of sent bytes.
	template <typename T>
	size_t receiveDataBlocking(T& data) {
		return reader_writer_.read(data);
		//return receiveDataBlocking(reinterpret_cast<uint8_t*>(&data), sizeof(T));
	}

	//! Blocking call. Returns the number of sent bytes.
	size_t sendDataBlocking(const uint8_t* data, size_t byteSize) {
		boost::system::error_code ec;
		size_t totalSentBytes = 0;
		while (totalSentBytes < byteSize) {
			size_t sentBytes = boost::asio::write(socket_, boost::asio::buffer(data + totalSentBytes, byteSize - totalSentBytes), ec);
			if (ec) {
				throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to send data on socket"), ec);
			}
			totalSentBytes += sentBytes;
		}
		return totalSentBytes;
	}

	//! Blocking call. Returns the number of sent bytes.
	template <typename T>
	size_t sendDataBlocking(const std::vector<T>& data) {
		return sendDataBlocking(reinterpret_cast<const uint8_t*>(data.data()), data.size() * sizeof(T));
	}

	//! Blocking call. Returns the number of sent bytes.
	template <typename T>
	size_t sendDataBlocking(const T& data) {
		return reader_writer_.write(data);
		//return sendDataBlocking(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
	}

	//! Shuts down and closes the socket
	void close() {
		if (socket_.is_open()) {
			boost::system::error_code ec;
			if (connected_) {
				socket_.shutdown(tcp::socket::shutdown_both, ec);
				if (ec) {
					std::cout << "WARNING: Unable to shutdown connected socket" << std::endl;
				}
				connected_ = false;
			}
			socket_.close(ec);
			if (ec) {
				throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to close socket"), ec);
			}
		}
	}

private:
	void run() {
		for (;;) {
			try {
				io_service_.run();
				break;
			}
			catch (const boost::system::error_code& ec) {
				std::cerr << "ERROR: Boost exception occured in io service: " << ec.message() << std::endl;
			}
			catch (const std::exception& err) {
				std::cerr << "ERROR: Exception occured in io service: " << err.what() << std::endl;
			}
		}
	}

	boost::asio::io_service io_service_;
	std::shared_ptr<boost::asio::io_service::work> io_service_work_;
	std::thread io_service_thread_;
	tcp::socket socket_;
	bool connected_;
	std::promise<bool> async_success_promise_;

	ReaderWriter reader_writer_;
};

}
