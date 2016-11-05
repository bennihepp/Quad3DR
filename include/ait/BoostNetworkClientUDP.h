#pragma once

#include <iostream>
#include <boost/asio.hpp>

namespace ait
{

class BoostNetworkClientUDP
{
public:
	using udp = boost::asio::ip::udp;

	class Error : public std::runtime_error
	{
	public:
		Error(const std::string &str)
			: std::runtime_error(str) {
		}
	};

	BoostNetworkClientUDP()
		: socket_(io_service_) {
	}

	~BoostNetworkClientUDP() {
		close();
	}

	bool isConnected() const {
		return socket_.is_open() && endpoint_ != udp::endpoint();
	}

	//! Connects the socket to a remote end.
	void open(const std::string& address, int port) {
		if (isConnected()) {
			throw Error("Client socket already open");
		}

		udp::resolver resolver(io_service_);
		endpoint_ = *resolver.resolve({ udp::v4(), address, std::to_string(port) });
		if (endpoint_ == udp::endpoint()) {
			throw Error(std::string("Unable to resolve remote end: ") + address + ":" + std::to_string(port));
		}
		socket_ = udp::socket(io_service_, udp::endpoint(udp::v4(), 0));
	}

	//! returns the number of received bytes
	size_t receiveDataBlocking(uint8_t* data, size_t byteSize) {
		ensureConnected();
		boost::system::error_code ec;
		size_t totalReceivedBytes = 0;
		while (totalReceivedBytes < byteSize) {
			udp::endpoint remote_endpoint;
			size_t receivedBytes = socket_.receive_from(boost::asio::buffer(data + totalReceivedBytes, byteSize - totalReceivedBytes), remote_endpoint, 0, ec);
			if (remote_endpoint != endpoint_) {
				std::cout << "WARNING: Received unexpected data from " << remote_endpoint.address().to_string() << ":" << remote_endpoint.port() << std::endl;
				continue;
			}
			if (ec && ec != boost::asio::error::message_size) {
				throw Error(std::string("Unable to receive data on socket: ") + ec.message());
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
		return receiveDataBlocking(reinterpret_cast<uint8_t*>(&data), sizeof(T));
	}

	//! Blocking call. Returns the number of sent bytes.
	size_t sendDataBlocking(const uint8_t* data, size_t byteSize) {
		// TODO:
		if (byteSize > 1500) {
			byteSize = 1500;
		}
		ensureConnected();
		boost::system::error_code ec;
		size_t totalSentBytes = 0;
		while (totalSentBytes < byteSize) {
			size_t sentBytes = socket_.send_to(boost::asio::buffer(data + totalSentBytes, byteSize - totalSentBytes), endpoint_, 0, ec);
			if (ec) {
				throw Error(std::string("Unable to send data on socket: ") + ec.message());
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
		return sendDataBlocking(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
	}

	//! Shuts down and closes the socket
	void close() {
		if (socket_.is_open()) {
			boost::system::error_code ec;
			socket_.shutdown(udp::socket::shutdown_both, ec);
			if (ec) {
				throw Error(std::string("Unable to shutdown socket: ") + ec.message());
			}
			socket_.close(ec);
			if (ec) {
				throw Error(std::string("Unable to close socket: ") + ec.message());
			}
		}
	}

private:
	void ensureConnected() {
		if (!isConnected()) {
			throw Error(std::string("Socket is not connected"));
		}
	}

	boost::asio::io_service io_service_;
	udp::socket socket_;
	udp::endpoint endpoint_;
};

}
