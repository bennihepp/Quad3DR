/*
 * thread.h
 *
 *  Created on: Dec 28, 2016
 *      Author: bhepp
 */

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <bh/common.h>

namespace bh {

class Thread {
public:
  static std::size_t getHardwareConcurrency() {
    return std::thread::hardware_concurrency();
  }

  Thread(bool start_now = false)
  : keep_running_(true), finished_(false) {
    if (start_now) {
      start();
    }
  }

  virtual ~Thread() {
    finish();
  }

  Thread(const Thread& thread) = delete;
  Thread(Thread&& thread) = delete;
//    Thread(Thread&& thread)
//    : thread_(std::move(thread.thread_)), keep_running_(std::move(thread.keep_running_)), finished_(std::move(thread.finished_)) {}

//    template <class Function, class... Args>
//    explicit Thread(Function&& f, Args&& args)
//    : thread_(std::forward(f), std::forward(args)), keep_running_(true), finished_(false) {}

  void setFinishedCallback(std::function<void()> finished_callback) {
    finished_callback_ = finished_callback;
  }

  virtual void reset() {
    finish();
    thread_ = std::thread();
    finished_ = false;
    keep_running_ = true;
  }

  void start() {
    BH_ASSERT(!isFinished());
    thread_ = std::thread(std::bind(&Thread::runInternal, this));
  }

  bool isRunning() const {
    return isJoinable() && !finished_;
  }

  bool isFinished() const {
    return finished_;
  }

  bool isJoinable() const {
    return thread_.joinable();
  }

  void join() {
    return thread_.join();
  }

  virtual void signalStop() {
    keep_running_ = false;
  }

  void finish() {
    if (isJoinable()) {
      signalStop();
      join();
    }
  }

  const std::thread& nativeThread() const {
    return thread_;
  }

  std::thread& navtiveThread() {
    return thread_;
  }

protected:
  virtual void run() = 0;

  bool shouldKeepRunning() const {
    return keep_running_;
  }

  bool shouldStop() const {
    return !keep_running_;
  }

private:
  void runInternal() {
    run();
    finished_ = true;
    if (finished_callback_) {
      finished_callback_();
    }
  }

  std::thread thread_;
  std::atomic<bool> keep_running_;
  std::atomic<bool> finished_;
  std::function<void()> finished_callback_;
};

class WorkerThread : public Thread {
public:
  enum Result {
    CONTINUE,
    STOP,
  };

  WorkerThread() {}

  ~WorkerThread() override {}

protected:
  virtual Result runIteration() = 0;

  void run() override {
    Result result = CONTINUE;
    while (shouldKeepRunning() && result == CONTINUE) {
      result = runIteration();
    }
  }
};

class PausableThread : public Thread {
public:
  enum Result {
    CONTINUE,
    STOP,
    PAUSE,
  };

  PausableThread(bool start_paused = false)
  : paused_(start_paused), verbose_(false) {}

  ~PausableThread() override {}

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

  void setPausedCallback(std::function<void()> paused_callback) {
    paused_callback_ = paused_callback;
  }

  void reset() override {
    paused_ = false;
    Thread::reset();
  }

  bool isPaused() const {
    return paused_;
  }

  void signalStop() override {
    Thread::signalStop();
    std::unique_lock<std::mutex> lock(mutex_);
    pause_cond_.notify_one();
  }

  void signalPause() {
    paused_ = true;
  }

  void signalContinue() {
    paused_ = false;
    std::unique_lock<std::mutex> lock(mutex_);
    pause_cond_.notify_one();
  }

protected:
  virtual Result runIteration() = 0;

  void run() override {
    Result result = CONTINUE;
    while (shouldKeepRunning() && result == CONTINUE) {
      waitIfPaused();
      // Stop could have been signaled while waiting for condition
      if (shouldStop()) {
        if (verbose_) {
          std::cout << "Stopping thread" << std::endl;
        }
        break;
      }
      Result iteration_result = runIteration();
      if (iteration_result == PAUSE) {
        paused_ = true;
      }
      else {
        result = iteration_result;
      }
    }
  }

  bool shouldPause() const {
    return paused_;
  }

  bool shouldContinue() const {
    return !paused_;
  }

  void waitIfPaused() {
    if (shouldPause()) {
      if (paused_callback_) {
        paused_callback_();
      }
      if (verbose_) {
        std::cout << "Pausing thread and waiting for condition" << std::endl;
      }
      waitForContinue();
      if (verbose_) {
        std::cout << "Thread was woken up" << std::endl;
      }
    }
  }

  void waitForContinue() {
    std::unique_lock<std::mutex> lock(mutex_);
    pause_cond_.wait(lock, [this]() {
      return shouldContinue() || shouldStop();
    });
  }

private:
  std::mutex mutex_;
  std::condition_variable pause_cond_;
  std::atomic<bool> paused_;
  std::function<void()> paused_callback_;
  bool verbose_;
};

}
