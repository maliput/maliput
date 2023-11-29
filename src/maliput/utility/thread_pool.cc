// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
