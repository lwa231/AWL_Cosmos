#pragma once
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace cosmos {

// Small fixed-size std::thread pool (plan section 4.5). The only operation is run(): execute fn(0) .. fn(tasks - 1) and
// return when all are done. The calling thread takes part, so a pool of N threads owns N - 1 workers and a pool of 1
// runs everything inline on the caller without any synchronisation.
//
// Which thread executes which task is not specified, so a caller that wants a result independent of the thread count
// must make each task a pure function of its index and write only to storage owned by that index (the Simulation does:
// task k advances a fixed contiguous range of shuttles into its own event buffer). run() is not reentrant and must be
// called from one thread at a time. An exception thrown by a task is rethrown from run() after every task finished
// (the first one, by task index order of detection).
class ThreadPool {
public:
    static constexpr int kMaxThreads = 16;

    // threads is clamped to [1, kMaxThreads].
    explicit ThreadPool(int threads = 1);
    ~ThreadPool();
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int threads() const { return threads_; }
    // Joins the workers and starts new ones. Not allowed while run() is executing.
    void resize(int threads);

    void run(std::size_t tasks, const std::function<void(std::size_t)>& fn);

private:
    void start(int threads);
    void stop();
    void workerLoop();
    void execute(std::size_t task);

    int threads_ = 1;
    std::vector<std::thread> workers_;

    std::mutex m_;
    std::condition_variable work_;
    std::condition_variable done_;
    const std::function<void(std::size_t)>* fn_ = nullptr;
    std::size_t tasks_ = 0;
    std::size_t next_ = 0;
    std::size_t finished_ = 0;
    bool stop_ = false;
    std::exception_ptr error_;
};

} // namespace cosmos
