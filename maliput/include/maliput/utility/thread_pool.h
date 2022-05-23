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
#pragma once

#include <atomic>
#include <deque>
#include <future>
#include <vector>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace utility {

/// Creates a pool of threads and a pool of tasks to be executed by the threads simultaneously.
/// Expected use of this class follows:
///
/// - Create an instance indicating the number of threads.
/// - Load the tasks to run via ThreadPool::Queue() and save the return std::future value.
/// - Run ThreadPool::Start().
/// - Optionally, run ThreadPool::Queue() if needed.
/// - Run ThreadPool::Finish().
/// - Optionally ThreadPool::cancel_pending() to hint the finalization of running tasks.
/// - Query the std::future results for consumption.
/// - After calling ThreadPool::Finish() neither ThreadPool::Start() nor ThreadPool::Queue() methods should be called.
///
/// @note The API is not guaranteed to be thread safe. This class is provided to speed up
/// build and load processes among others which are necessary but not part of the API.
/// Use it with caution.
class ThreadPool {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ThreadPool);

  /// Creates a pool of `n` threads.
  /// @param n Is the number of threads to be managed.
  /// @throw When N is equal to zero.
  explicit ThreadPool(std::size_t n);

  ~ThreadPool();

  /// Adds tasks to the queue.
  /// @tparam F Is the functor type.
  /// @param f Is the task.
  /// @tparam R Is the functor`s return type.
  /// @return A std::future<R> that will hold the result of `f`.
  /// @throws maliput::common::assertion_error When Finish() method was already called.
  template <class F, class R = std::result_of_t<F&()>>
  std::future<R> Queue(F&& f) {
    MALIPUT_THROW_UNLESS(!is_finished_);
    // Wrap the function object into a packaged task, splitting
    // execution from the return value.
    std::packaged_task<R()> packaged_task(std::forward<F>(f));

    // Get the future return value before we hand off the task.
    auto return_value = packaged_task.get_future();
    {
      std::unique_lock<std::mutex> l(tasks_mutex_);
      // store the task<R()> as a task<void()>
      tasks_.emplace_back(std::move(packaged_task));
    }
    // Wake a thread to work on the task.
    available_.notify_one();
    return return_value;
  }

  /// Starts the threads to work on the tasks.
  /// The main thread will not perform any task and will continue the execution.
  /// @throw maliput::common::assertion_error When this method
  /// is called consecutively without finishing the threads.
  void Start();

  /// Finishes the threads.
  /// Waits for all tasks previously added to finish before destroying the threads.
  void Finish();

  /// Cancels all non-started tasks.
  void cancel_pending();

  /// @returns True when the threads are already started.
  bool is_running() { return is_running_.load(); }

 private:
  // Loop that every thread runs.
  // When the thread picks up an invalid task it ends.
  void DoWork();

  // Mutex used to avoid race conditions when inserting and dispatching tasks.
  std::mutex tasks_mutex_;

  // Mutex used to avoid race conditions when inserting and dispatching tasks.
  std::mutex start_mutex_;

  // Condition variable used to notify when a task is available.
  std::condition_variable available_;

  // Condition variable used to notify when the threads should start.
  std::condition_variable start_;

  // Queue to hold all the tasks.
  // Note that a packaged_task<void()> can store a packaged_task<R>.
  std::deque<std::packaged_task<void()>> tasks_;

  // Holds futures representing the threads being done:
  std::vector<std::future<void>> futures_;

  // Indicates whether the threads are started.
  std::atomic<bool> is_running_{false};

  // Indicates whether the threads are finished.
  std::atomic<bool> is_finished_{false};
};

}  // namespace utility
}  // namespace maliput
