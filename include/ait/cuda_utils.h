//==================================================
// cuda.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 16, 2017
//==================================================
#pragma once

#include <string>
#include <exception>

#include <ait/common.h>

#include <cuda.h>
#include <cuda_runtime.h>
#ifdef __CDT_PARSER__
  #define __global__
  #define __device__
  #define __shared__
  #define __host__
#endif

#define CUDA_DEVICE_SYNCHRONIZE()   CUDA_SAFE_CALL(cudaDeviceSynchronize());

#define CUDA_SAFE_CALL(call) { \
    cudaError err = call; \
    if (cudaSuccess != err) { \
      fprintf(stderr, "CUDA error in file '%s' in line %i: %s\n", \
          __FILE__, __LINE__, cudaGetErrorString(err)); \
      AIT_DEBUG_BREAK; \
      AIT_EXCEPTION(std::string("CUDA error in file ") + __FILE__ + ":" + std::to_string(__LINE__) + ": " + cudaGetErrorString(err)); \
    } \
}

#define CUDA_CHECK_ERROR() { \
    cudaError_t err = cudaGetLastError(); \
    if (cudaSuccess != err) { \
      fprintf(stderr, "CUDA error in file '%s' in line %i: %s\n", \
          __FILE__, __LINE__, cudaGetErrorString(err)); \
      AIT_DEBUG_BREAK; \
      AIT_EXCEPTION(std::string("CUDA error in file ") + __FILE__ + ":" + std::to_string(__LINE__) + ": " + cudaGetErrorString(err)); \
    } \
}

namespace ait {

class CudaError : public std::exception {
public:
  CudaError(const cudaError_t err) {
    msg_ = cudaGetErrorString(err);
  }

  ~CudaError() _GLIBCXX_USE_NOEXCEPT override {}

  const char* what() const _GLIBCXX_USE_NOEXCEPT override {
    return msg_.c_str();
  }

private:
  std::string msg_;
};

class CudaManager {
public:
  static void deviceReset() {
    CUDA_SAFE_CALL(cudaDeviceReset());
  }

  static int getDeviceCount() {
    int device_count;
    CUDA_SAFE_CALL(cudaGetDeviceCount(&device_count));
    return device_count;
  }

  static int getDevice() {
    int device;
    CUDA_SAFE_CALL(cudaGetDevice(&device));
    return device;
  }

  static void setDevice(const int device) {
    CUDA_SAFE_CALL(cudaSetDevice(device));
  }

  static std::size_t getStackSize() {
    std::size_t stack_size;
    CUDA_SAFE_CALL(cudaDeviceGetLimit(&stack_size, cudaLimitStackSize));
    return stack_size;
  }

  static void setStackSize(const std::size_t stack_size) {
    CUDA_SAFE_CALL(cudaDeviceSetLimit(cudaLimitStackSize, stack_size));
  }
};

class CudaUtils {
public:
  template <typename T>
  static T* allocate() {
    T* ptr;
    CUDA_SAFE_CALL(cudaMalloc(&ptr, sizeof(T)));
    return ptr;
  }

  template <typename T>
  static T* allocate(const std::size_t count) {
    T* ptr;
    CUDA_SAFE_CALL(cudaMalloc(&ptr, count * sizeof(T)));
    return ptr;
  }

  template <typename T>
  static void deallocate(T* ptr) {
    CUDA_SAFE_CALL(cudaFree(ptr));
  }

  template <typename T>
  static void deallocate(T** ptr) {
    CUDA_SAFE_CALL(cudaFree(*ptr));
    *ptr = nullptr;
  }

  template <typename T>
  static T copyFromDevice(const T* device_data) {
    uint8_t host_data[sizeof(T)];
    std::size_t size = sizeof(T);
    CUDA_SAFE_CALL(cudaMemcpy(host_data, device_data, size, cudaMemcpyDeviceToHost));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
    return T(*reinterpret_cast<T*>(host_data));
  }

  template <typename T>
  static void copyFromDevice(const T* device_data, T* host_data) {
    std::size_t size = sizeof(T);
    CUDA_SAFE_CALL(cudaMemcpy(host_data, device_data, size, cudaMemcpyDeviceToHost));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
  }

  template <typename T>
  static std::vector<T> copyArrayFromDevice(const T* device_data, std::size_t count) {
    std::vector<T> host_data(count);
    std::size_t size = sizeof(T) * count;
    CUDA_SAFE_CALL(cudaMemcpy(host_data.data(), device_data, size, cudaMemcpyDeviceToHost));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
    return host_data;
  }

  template <typename T>
  static void copyArrayFromDevice(const T* device_data, std::vector<T>* host_data) {
    std::size_t size = sizeof(T) * host_data->size();
    CUDA_SAFE_CALL(cudaMemcpy(host_data->data(), device_data, size, cudaMemcpyDeviceToHost));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
  }

  template <typename T>
  static void copyArrayFromDevice(const T* device_data, std::vector<T>* host_data, std::size_t count) {
    host_data->resize(count);
    std::size_t size = sizeof(T) * count;
    CUDA_SAFE_CALL(cudaMemcpy(host_data->data(), device_data, size, cudaMemcpyDeviceToHost));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
  }

  template <typename T>
  static void copyToDevice(const T& host_data, T* device_data) {
    std::size_t size = sizeof(T);
    CUDA_SAFE_CALL(cudaMemcpy(device_data, &host_data, size, cudaMemcpyHostToDevice));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
  }

  template <typename T>
  static void copyArrayToDevice(const std::vector<T>& host_data, T* device_data) {
    std::size_t size = sizeof(T) * host_data.size();
    CUDA_SAFE_CALL(cudaMemcpy(device_data, host_data.data(), size, cudaMemcpyHostToDevice));

#if !NO_CHECK_CUDA_CALLS
    CUDA_DEVICE_SYNCHRONIZE();
    CUDA_CHECK_ERROR();
#endif
  }
};

}
