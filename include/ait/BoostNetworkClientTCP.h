#pragma once

#include <iostream>
#include <boost/asio.hpp>

#define ANNOTATE_EXC(type, s) type ## (std::string(FUNCTION_LINE_STRING).append(": ").append(s))
#define ANNOTATE_EXC_BOOST(type, s, ec) type ## (std::string(FUNCTION_LINE_STRING).append(": ").append(s), ec)

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

	BoostNetworkClientTCP()
		: socket_(io_service_), connected_(false) {
	}

	~BoostNetworkClientTCP() {
		close();
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
		return receiveDataBlocking(reinterpret_cast<uint8_t*>(&data), sizeof(T));
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
		return sendDataBlocking(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
	}

	//! Shuts down and closes the socket
	void close() {
		if (socket_.is_open()) {
			boost::system::error_code ec;
			if (connected_) {
				socket_.shutdown(tcp::socket::shutdown_both, ec);
				if (ec) {
					throw ANNOTATE_EXC_BOOST(Error, std::string("Unable to shutdown socket"), ec);
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
	boost::asio::io_service io_service_;
	tcp::socket socket_;
	bool connected_;
};

}
