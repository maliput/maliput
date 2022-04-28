// Copyright 2020 Toyota Research Institute
#include "maliput/utility/thread_pool.h"

namespace maliput {
namespace utility {

ThreadPool::ThreadPool(std::size_t n) {
  MALIPUT_THROW_UNLESS(n != 0);
  for (std::size_t i = 0; i < n; ++i) {
    futures_.push_back(std::async(std::launch::async, [this] { DoWork(); }));
  }
};

ThreadPool::~ThreadPool() { Finish(); }

void ThreadPool::Start() {
  MALIPUT_THROW_UNLESS(!is_running_);
  MALIPUT_THROW_UNLESS(!is_finished_);
  std::unique_lock<std::mutex> l_s(start_mutex_);
  is_running_ = true;
  start_.notify_all();
}

void ThreadPool::Finish() {
  is_finished_ = true;
  {
    std::unique_lock<std::mutex> l_t(tasks_mutex_);
    available_.notify_all();
  }
  for (const auto& future : futures_) {
    future.wait();
  }
  futures_.clear();
  is_running_ = false;
}

void ThreadPool::cancel_pending() {
  std::unique_lock<std::mutex> lock(tasks_mutex_);
  tasks_.clear();
}

void ThreadPool::DoWork() {
  {
    std::unique_lock<std::mutex> l_s(start_mutex_);
    if (!is_running_) {
      start_.wait(l_s, [&] { return is_running_.load(); });
    }
  }
  while (true) {
    std::packaged_task<void()> f;
    {  // Picks a task.
      std::unique_lock<std::mutex> l_t(tasks_mutex_);
      if (tasks_.empty()) {
        if (is_finished_) {
          break;
        } else {
          available_.wait(l_t);
        }
      } else {
        f = std::move(tasks_.front());
        tasks_.pop_front();
      }
    }
    if (f.valid()) {
      f();
    }
  }
}

}  // namespace utility
}  // namespace maliput
