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

#include <bh/common.h>

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
      BH_DEBUG_BREAK; \
      BH_EXCEPTION(std::string("CUDA error in file ") + __FILE__ + ":" + std::to_string(__LINE__) + ": " + cudaGetErrorString(err)); \
    } \
}

#define CUDA_CHECK_ERROR() { \
    cudaError_t err = cudaGetLastError(); \
    if (cudaSuccess != err) { \
      fprintf(stderr, "CUDA error in file '%s' in line %i: %s\n", \
          __FILE__, __LINE__, cudaGetErrorString(err)); \
      BH_DEBUG_BREAK; \
      BH_EXCEPTION(std::string("CUDA error in file ") + __FILE__ + ":" + std::to_string(__LINE__) + ": " + cudaGetErrorString(err)); \
    } \
}

namespace bh {

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
  static int getDeviceCount() {
    int device_count;
    CUDA_SAFE_CALL(cudaGetDeviceCount(&device_count));
    return device_count;
  }

  static int getActiveGpuId() {
    int gpu_id;
    CUDA_SAFE_CALL(cudaGetDevice(&gpu_id));
    return gpu_id;
  }

  static void setActiveGpuId(const int gpu_id) {
    CUDA_SAFE_CALL(cudaSetDevice(gpu_id));
  }

  static void deviceReset() {
    CUDA_SAFE_CALL(cudaDeviceReset());
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

class CudaDevice {
public:
  CudaDevice(const int gpu_id, const bool do_activate = true)
  : gpu_id_(gpu_id) {
    if (do_activate) {
      activate();
    }
  }

  int getId() const {
    return gpu_id_;
  }

  void activate() {
    CUDA_SAFE_CALL(cudaSetDevice(gpu_id_));
  }

  void reset() {
    activate();
    CUDA_SAFE_CALL(cudaDeviceReset());
  }

  std::size_t getStackSize() {
    activate();
    std::size_t stack_size;
    CUDA_SAFE_CALL(cudaDeviceGetLimit(&stack_size, cudaLimitStackSize));
    return stack_size;
  }

  void setStackSize(const std::size_t stack_size) {
    activate();
    CUDA_SAFE_CALL(cudaDeviceSetLimit(cudaLimitStackSize, stack_size));
  }

  static CudaDevice getActivateDevice(const bool do_activate = true) {
    return CudaDevice(CudaManager::getActiveGpuId(), do_activate);
  }

private:
  int gpu_id_;
};

class CudaUtils {
public:

  static std::pair<size_t, size_t> computeGridAndBlockSize(const size_t num_threads, const size_t threads_per_block) {
    return std::make_pair(computeGridSize(num_threads, threads_per_block),
                          computeBlockSize(num_threads, threads_per_block));
  }

  static size_t computeGridSize(const size_t num_threads, const size_t threads_per_block) {
    const std::size_t grid_size = (num_threads + threads_per_block - 1) / threads_per_block;
    return grid_size;
  }

  static size_t computeBlockSize(const size_t num_threads, const size_t threads_per_block) {
    const std::size_t block_size = std::min(num_threads, threads_per_block);
    return block_size;
  }

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
