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

#include <algorithm>
#include <atomic>
#include <set>
#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace utility {
namespace test {
namespace {

// Functor that returns the id of the thread after the condition
// variable is free.
struct WaitForCondition {
  // @param m Is a pointer to the mutex that is manages by the condition variable.
  // @param c Is a pointer to the condition variable.
  // @param counter Is a pointer to a counter that needs to be increased.
  WaitForCondition(std::mutex* m, std::condition_variable* c, std::atomic<std::size_t>* counter)
      : m(m), c(c), counter(counter) {
    MALIPUT_THROW_UNLESS(c != nullptr);
    MALIPUT_THROW_UNLESS(m != nullptr);
    MALIPUT_THROW_UNLESS(counter != nullptr);
  }

  std::thread::id operator()() {
    std::unique_lock<std::mutex> l(*m);
    (*counter)++;
    c->wait(l);
    return std::this_thread::get_id();
  }

  std::mutex* m{nullptr};
  std::condition_variable* c{nullptr};
  std::atomic<std::size_t>* counter{nullptr};
};

// Functor that returns the id of the thread after a configurable time has passed.
struct SleepAndIdentify {
  std::thread::id operator()() {
    std::this_thread::sleep_for(timeout);
    return std::this_thread::get_id();
  }
  std::chrono::duration<double, std::milli> timeout;
};

// Simple tasks pool that shows a typical usecase of this class.
GTEST_TEST(ThreadPoolTest, BasicTaskPool) {
  const std::size_t kNumberOfThreads = 8;
  const std::size_t kNumberOfTasks = 50;

  ThreadPool dut(kNumberOfThreads);
  std::vector<std::future<std::size_t>> future_results;
  for (std::size_t i = 0; i < kNumberOfTasks; i++) {
    future_results.push_back(dut.Queue([i]() { return i * i; }));
  }
  dut.Start();
  dut.Finish();
  for (std::size_t i = 0; i < kNumberOfTasks; i++) {
    EXPECT_EQ(i * i, future_results[i].get());
  }
  // Once Finish() is called, Queue() and Start() aren't allowed anymore.
  EXPECT_THROW(dut.Queue([]() { return "InvalidOperation"; }), maliput::common::assertion_error);
  EXPECT_THROW(dut.Start(), maliput::common::assertion_error);
}

using namespace std::chrono_literals;

// Verifies that the tasks were solved using the number of threads requested.
// For doing this a sleep time was introduced in each task, allowing all the threads to pick at least one task.
GTEST_TEST(ThreadPoolTest, TasksWithDifferentExecutionTimes) {
  const std::size_t kNumberOfThreads = 4;
  const std::size_t kNumberOfTasks = 8;
  const std::chrono::milliseconds kSleepTime = 10ms;

  ThreadPool dut(kNumberOfThreads);
  std::vector<std::future<std::thread::id>> future_results;

  // Blocking tasks are first inserted in order to use all the available threads and keep them busy.
  std::mutex mutex{};
  std::condition_variable condition_variable{};
  std::atomic<std::size_t> counter{0};
  for (std::size_t i = 0; i < kNumberOfThreads; i++) {
    future_results.push_back(dut.Queue(WaitForCondition{&mutex, &condition_variable, &counter}));
  }
  // The next tasks should be on hold until the blocking tasks are completed.
  for (std::size_t i = 0; i < kNumberOfTasks - kNumberOfThreads; i++) {
    future_results.push_back(dut.Queue(SleepAndIdentify{kSleepTime}));
  }
  dut.Start();
  // Check that all the blocking tasks are executed.
  const int max_iterations{5};
  for (int i = 0; counter.load() < kNumberOfThreads; ++i) {
    std::this_thread::sleep_for(kSleepTime);
    // The test is aborted if the condition isn't met after 5 iterations.
    ASSERT_LE(i, max_iterations);
  }

  // Unblocks the first tasks.
  condition_variable.notify_all();
  std::this_thread::sleep_for(kSleepTime);
  dut.Finish();
  ASSERT_TRUE(std::all_of(future_results.begin(), future_results.end(),
                          [](const std::future<std::thread::id>& future_result) { return future_result.valid(); }));

  std::set<std::thread::id> workers_ids;
  for (auto& future_result : future_results) {
    workers_ids.insert(future_result.get());
  }
  EXPECT_TRUE(workers_ids.find(std::this_thread::get_id()) == workers_ids.end());
  EXPECT_EQ(kNumberOfTasks, future_results.size());
  EXPECT_EQ(kNumberOfThreads, workers_ids.size());
}

// Adds a blocking task for each thread to the queue. Also, it adds
// more tasks to the queue. When the pool thread starts each thread will be blocked at their first task.
// After cancel_pending method is called, the tasks are unblocked and the threads finish their work.
GTEST_TEST(ThreadPoolTest, CancelPendingTasks) {
  const std::size_t kNumberOfThreads = 4;
  const std::size_t kNumberOfTasksToFinish = kNumberOfThreads;
  const std::size_t kNumberOfTasksToCancel = 8;
  const std::chrono::milliseconds kSleepTime = 10ms;

  std::mutex mutex{};
  std::condition_variable condition_variable{};
  std::atomic<std::size_t> counter{0};
  ThreadPool dut(kNumberOfThreads);
  std::vector<std::future<std::thread::id>> future_results;
  // Creates tasks that will wait for a condition variable.
  for (std::size_t i = 0; i < kNumberOfTasksToFinish; i++) {
    future_results.push_back(dut.Queue(WaitForCondition{&mutex, &condition_variable, &counter}));
  }
  // Creates tasks that won't be able to be processed because there is no free thread.
  for (std::size_t i = 0; i < kNumberOfTasksToCancel; i++) {
    future_results.push_back(dut.Queue(SleepAndIdentify{kSleepTime}));
  }
  dut.Start();
  // Check that all the blocking tasks are executed.
  const int max_iterations{5};
  for (int i = 0; counter.load() < kNumberOfThreads; ++i) {
    std::this_thread::sleep_for(kSleepTime);
    // The test is aborted if the condition isn't met after 5 iterations.
    ASSERT_LE(i, max_iterations);
  }

  dut.cancel_pending();

  condition_variable.notify_all();

  dut.Finish();

  std::size_t invalid_tasks_result_counter{};
  for (auto& future_result : future_results) {
    // TODO(#395): Remove try catch clause. The std::future::valid() method should
    // return false when the task is deleted.
    try {
      future_result.get();
    } catch (const std::future_error& e) {
      invalid_tasks_result_counter++;
    } catch (const std::exception& e) {
      ASSERT_TRUE(false) << "Unexpected exception thrown: " << e.what();
    }
  }
  EXPECT_EQ(kNumberOfTasksToCancel, invalid_tasks_result_counter);
}

}  // namespace
}  // namespace test
}  // namespace utility
}  // namespace maliput
