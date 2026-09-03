/**
 * @file concurrent_queue.hpp
 * @brief Thread-safe queue for FreeRTOS
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-06
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <queue>
#include <deque>
#include "../freertospp/mutex.h"
#include "../freertospp/semphr.h"

namespace utils {

template <typename T, typename Container = std::deque<T>>
class concurrent_queue {
 public:
  typedef typename std::queue<T, Container>::size_type size_type;
  typedef typename std::queue<T, Container>::reference reference;
  typedef typename std::queue<T, Container>::const_reference const_reference;

  void push(T const& e) {
    mutex_.lock();
    queue_.push(e);
    mutex_.unlock();
    sem_.give();
  }

  template <typename... _Args>
  void emplace(_Args&&... __args) {
    mutex_.lock();
    queue_.emplace(std::forward<_Args>(__args)...);
    mutex_.unlock();
    sem_.give();
  }

  bool empty() {
    mutex_.lock();
    bool res = queue_.empty();
    mutex_.unlock();
    return res;
  }

  void pop() {
    mutex_.lock();
    if (!queue_.empty())
      queue_.pop();
    mutex_.unlock();
  }

  void front_pop(T& ret) {
    sem_.take();
    mutex_.lock();
    ret = queue_.front();
    queue_.pop();
    mutex_.unlock();
  }

  size_type size() {
    mutex_.lock();
    size_type s = queue_.size();
    mutex_.unlock();
    return s;
  }

  reference front() {
    sem_.take();
    mutex_.lock();
    auto& ref = queue_.front();
    mutex_.unlock();
    return ref;
  }

  reference back() {
    mutex_.lock();
    auto& ref = queue_.back();
    mutex_.unlock();
    return ref;
  }

  void clear() {
    mutex_.lock();
    while (!queue_.empty()) queue_.pop();
    while (sem_.take(0)) {}
    mutex_.unlock();
  }

 protected:
  std::queue<T, Container> queue_;
  freertospp::Mutex mutex_;
  freertospp::Semaphore sem_;
};

}  // namespace utils
