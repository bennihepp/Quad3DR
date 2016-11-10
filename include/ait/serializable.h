//==================================================
// serializable.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 9, 2016
//==================================================

#pragma once

#include <type_traits>
#include <vector>

namespace ait {

	class Reader;
	class Writer;

	class ISerializable
	{
	public:
		virtual ~ISerializable() {
		};

		virtual size_t _write(Writer& writer) const = 0;
		virtual size_t _read(Reader& reader) = 0;
	};

	class Reader {
	public:
		virtual ~Reader() {
		};

		virtual size_t read(void* data, size_t size) = 0;

		template <typename T, typename std::enable_if<!std::is_base_of<ISerializable, T>::value>::type* = nullptr>
		size_t read(T& value) {
			return read(&value, sizeof(T));
		}

		//template <typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
		//size_t read(T& value) {
		//	return read(&value, sizeof(T));
		//}

		template <typename T, typename std::enable_if<!std::is_base_of<ISerializable, T>::value>::type* = nullptr>
		size_t read(std::vector<T>& vector) {
			size_t read = 0;
			size_t size;
			read += this->read(size);
			vector.resize(size);
			read += this->read(vector.data(), vector.size() * sizeof(T));
			return read;
		}

		template <typename T, typename std::enable_if<std::is_base_of<ISerializable, T>::value>::type* = nullptr>
		size_t read(std::vector<T>& vector) {
			size_t read = 0;
			size_t size;
			read += this->read(size);
			vector.resize(size);
			for (T& value : vector) {
				read += this->read(value);
			}
			return read;
		}

		size_t read(ISerializable& value) {
			return value._read(*this);
		}
	};

	template <>
	inline size_t Reader::read<std::string>(std::string& value) {
		size_t read_bytes = 0;
		size_t length;
		read_bytes += this->read(length);
		std::vector<char> string_data(length + 1);
		read_bytes += read(string_data.data(), length);
		string_data[length] = '\0';
		value = (char*)string_data.data();
		return read_bytes;
	}

	class Writer {
	public:
		virtual ~Writer() {
		};

		virtual size_t write(const void* data, size_t size) = 0;

		template <typename T, typename std::enable_if<!std::is_base_of<ISerializable, T>::value>::type* = nullptr>
		size_t write(const T& value) {
			return write(&value, sizeof(T));
		}

		//template <typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
		//size_t write(const T& value) {
		//	return write(&value, sizeof(T));
		//}

		template <typename T, typename std::enable_if<!std::is_base_of<ISerializable, T>::value>::type* = nullptr>
		size_t write(const std::vector<T>& vector) {
			size_t written = 0;
			written += write(vector.size());
			written += this->write(vector.data(), vector.size() * sizeof(T));
			return written;
		}

		template <typename T, typename std::enable_if<std::is_base_of<ISerializable, T>::value>::type* = nullptr>
		size_t write(const std::vector<T>& vector) {
			size_t written = 0;
			written += write(vector.size());
			for (const T& value : vector) {
				written += this->write(value);
			}
			return written;
		}

		size_t write(const ISerializable& value) {
			return value._write(*this);
		}
	};

	template <>
	inline size_t Writer::write<std::string>(const std::string& value) {
		size_t written = 0;
		written += write(value.length());
		written += this->write(value.c_str(), value.length());
		return written;
	}

	template <typename Derived>
	class Serializable : public ISerializable
	{
	public:
		~Serializable() override {
		}

		size_t _write(Writer& writer) const override {
			return writer.write(this, sizeof(Derived));
		}

		size_t _read(Reader& reader) override {
			return reader.read(this, sizeof(Derived));
		}
	};
}
