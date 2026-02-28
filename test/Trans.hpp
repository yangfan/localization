#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

#include <glog/logging.h>

template <typename T> class Trans {
public:
  static Trans &instance() {
    static Trans ins;
    return ins;
  }

  void produce(const T &content) {
    std::unique_lock<std::mutex> plock(data_mtx_);
    not_full_.wait(plock, [this]() { return data_.size() < cap_; });
    data_.emplace_back(content);
    plock.unlock();
    not_empty_.notify_one();
  }
  T consume() {
    std::unique_lock<std::mutex> clock(data_mtx_);
    not_empty_.wait(clock, [this]() { return !data_.empty(); });
    T output = data_.front();
    data_.pop_front();
    clock.unlock();
    not_full_.notify_one();
    return output;
  }

private:
  Trans() : cap_(100) {}
  size_t cap_ = 0;
  std::deque<T> data_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::mutex data_mtx_;
};
