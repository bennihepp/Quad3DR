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

	class Reader {
	public:
		virtual ~Reader() {
		};

		virtual size_t _read(void* data, size_t size) = 0;

		template <typename T>
		size_t read(T& value);
	};

	class Writer {
	public:
		virtual ~Writer() {
		};

		virtual size_t _write(const void* data, size_t size) = 0;

		template <typename T>
		size_t write(const T& value);
	};

	template<typename T>
	struct dependent_false : std::false_type {};

	template<class T>
	struct is_vector : public std::false_type {};

	template<class T, class Alloc>
	struct is_vector<std::vector<T, Alloc>> : public std::true_type{};

	class ISerializable
	{
	public:
		virtual ~ISerializable() {
		};

		virtual size_t _write(Writer& writer) const = 0;
		virtual size_t _read(Reader& reader) = 0;
	};

	template <typename T>
	struct IsClassSerializable : std::integral_constant<
		bool,
		std::is_base_of<ISerializable, T>::value
		|| std::is_base_of<std::string, T>::value
		|| is_vector<T>::value
	>
	{
	};

	template <typename T>
	struct IsBinarySerializable : std::integral_constant<
		bool,
		!IsClassSerializable<T>::value
		&& (std::is_standard_layout<T>::value || std::is_fundamental<T>::value)
	>
	{
	};

	template <typename T>
	struct IsSerializable : std::integral_constant<
		bool,
		IsClassSerializable<T>::value || IsBinarySerializable<T>::value
	>
	{
	};

	template <typename T, typename enable = void>
	struct ValueReader
	{
		static size_t read(Reader* reader, T& value) {
			static_assert(dependent_false<T>::value, "Type is not suppoted for serialization");
		}
	};

	template <typename T>
	struct ValueReader<T, typename std::enable_if<!IsSerializable<T>::value>::type>
	{
		static size_t read(Reader* reader, T& value) {
			static_assert(dependent_false<T>::value, "Type is not suppoted for serialization");
		}
	};

	template <typename T>
	struct ValueReader<T, typename std::enable_if<std::is_base_of<ISerializable, T>::value>::type>
	{
		static size_t read(Reader* reader, T& value) {
			return value._read(*reader);
		}
	};

	template <typename T>
	struct ValueReader<T, typename std::enable_if<IsBinarySerializable<T>::value>::type>
	{
		static size_t read(Reader* reader, T& value) {
			return reader->_read(&value, sizeof(T));
		}
	};

	template <typename T>
	struct ValueReader<T, typename std::enable_if<std::is_base_of<std::string, T>::value>::type>
	{
		static size_t read(Reader* reader, T& value) {
			size_t read_bytes = 0;
			size_t length;
			read_bytes += reader->read(length);
			std::vector<char> string_data(length + 1);
			read_bytes += reader->_read(string_data.data(), length);
			string_data[length] = '\0';
			value = (char*)string_data.data();
			return read_bytes;
		}
	};

	template <typename T>
	struct ValueReader<T, typename std::enable_if<is_vector<T>::value>::type>
	{
		static size_t read(Reader* reader, T& vector) {
			size_t read = 0;
			size_t size;
			read += reader->read(size);
			vector.resize(size);
			for (typename T::value_type& value : vector) {
				read += reader->read(value);
			}
			return read;
		}
	};

	template <typename T>
	inline size_t Reader::read(T& value) {
		return ValueReader<T>::read(this, value);
	}

	template <typename T, typename enable = void>
	struct ValueWriter
	{
		static size_t write(Writer* writer, const T& value) {
			static_assert(dependent_false<T>::value, "Type is not suppoted for serialization");
		}
	};

	template <typename T>
	struct ValueWriter<T, typename std::enable_if<!IsSerializable<T>::value>::type>
	{
		static size_t write(Writer* writer, const T& value) {
			static_assert(dependent_false<T>::value, "Type is not suppoted for serialization");
		}
	};

	template <typename T>
	struct ValueWriter<T, typename std::enable_if<std::is_base_of<ISerializable, T>::value>::type>
	{
		static size_t write(Writer* writer, const T& value) {
			return value._write(*writer);
		}
	};

	template <typename T>
	struct ValueWriter<T, typename std::enable_if<IsBinarySerializable<T>::value>::type>
	{
		static size_t write(Writer* writer, const T& value) {
			return writer->_write(&value, sizeof(T));
		}
	};

	template <typename T>
	struct ValueWriter<T, typename std::enable_if<std::is_base_of<std::string, T>::value>::type>
	{
		static size_t write(Writer* writer, const T& value) {
			size_t written = 0;
			written += writer->write(value.length());
			written += writer->_write(value.c_str(), value.length());
			return written;
		}
	};

	template <typename T>
	struct ValueWriter<T, typename std::enable_if<is_vector<T>::value>::type>
	{
		static size_t write(Writer* writer, const T& vector) {
			size_t written = 0;
			written += writer->write(vector.size());
			for (const typename T::value_type& value : vector) {
				written += writer->write(value);
			}
			return written;
		}
	};

	template <typename T>
	inline size_t Writer::write(const T& value) {
		return ValueWriter<T>::write(this, value);
	}

	template <typename Derived>
	class Serializable : public ISerializable
	{
	public:
		~Serializable() override {
		}

		size_t _write(Writer& writer) const override {
			return writer._write(this, sizeof(Derived));
		}

		size_t _read(Reader& reader) override {
			return reader._read(this, sizeof(Derived));
		}
	};
}
