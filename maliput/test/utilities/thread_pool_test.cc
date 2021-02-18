// Copyright 2020 Toyota Research Institute
#include "maliput/utilities/thread_pool.h"

#include <algorithm>
#include <atomic>
#include <set>
#include <stdexcept>

#include <gtest/gtest.h>

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
  WaitForCondition(std::mutex* m, std::condition_variable* c, std::atomic<int>* counter)
      : m(m), c(c), counter(counter) {
    MALIPUT_THROW_UNLESS(c != nullptr);
    MALIPUT_THROW_UNLESS(m != nullptr);
    MALIPUT_THROW_UNLESS(counter != nullptr);
  }

  std::thread::id operator()() {
    std::unique_lock<std::mutex> l(*m);
    counter->store(counter->load() + 1);
    c->wait(l);
    return std::this_thread::get_id();
  }

  std::mutex* m;
  std::condition_variable* c;
  std::atomic<int>* counter;
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
  std::atomic<int> counter{0};
  for (std::size_t i = 0; i < kNumberOfThreads; i++) {
    future_results.push_back(dut.Queue(WaitForCondition{&mutex, &condition_variable, &counter}));
  }
  dut.Start();
  // The next tasks should be on hold until the blocking tasks are completed.
  for (std::size_t i = 0; i < kNumberOfTasks - kNumberOfThreads; i++) {
    future_results.push_back(dut.Queue(SleepAndIdentify{kSleepTime}));
  }
  // Check that all the blocking tasks are executed.
  while (counter.load() < static_cast<int>(kNumberOfThreads)) {
    std::this_thread::sleep_for(kSleepTime);
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
  std::atomic<int> counter{0};
  ThreadPool dut(kNumberOfThreads);
  std::vector<std::future<std::thread::id>> future_results;
  // Creates tasks that will wait for a condition variable.
  for (std::size_t i = 0; i < kNumberOfTasksToFinish; i++) {
    future_results.push_back(dut.Queue(WaitForCondition{&mutex, &condition_variable, &counter}));
  }
  dut.Start();
  // Creates tasks that won't be able to be processed because there is no free thread.
  for (std::size_t i = 0; i < kNumberOfTasksToCancel; i++) {
    future_results.push_back(dut.Queue(SleepAndIdentify{kSleepTime}));
  }
  // Check that all the blocking tasks are executed.
  while (counter.load() < static_cast<int>(kNumberOfThreads)) {
    std::this_thread::sleep_for(kSleepTime);
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
