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
#include <ait/common.h>

namespace ait {

class Thread {
public:
  static std::size_t getHardwareConcurrency();

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

  void start() {
    AIT_ASSERT(!isFinished());
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
  }

  std::thread thread_;
  std::atomic<bool> keep_running_;
  std::atomic<bool> finished_;
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
  };

  PausableThread(bool start_paused = false)
  : paused_(start_paused) {}

  ~PausableThread() override {}

  bool isPaused() const {
    return paused_;
  }

  void signalStop() override {
    Thread::signalStop();
    pause_cond_.notify_one();
  }

  void signalPause() {
    paused_ = true;
  }

  void signalContinue() {
    paused_ = false;
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
        break;
      }
      result = runIteration();
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
      waitForContinue();
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
};

}
