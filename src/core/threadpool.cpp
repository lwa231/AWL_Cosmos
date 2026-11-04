#include "core/threadpool.h"
#include <algorithm>

namespace cosmos {

ThreadPool::ThreadPool(int threads) { start(threads); }

ThreadPool::~ThreadPool() { stop(); }

void ThreadPool::start(int threads)
{
    threads_ = std::clamp(threads, 1, kMaxThreads);
    stop_ = false;
    workers_.reserve(static_cast<std::size_t>(threads_ - 1));
    for (int i = 1; i < threads_; ++i) workers_.emplace_back([this] { workerLoop(); });
}

void ThreadPool::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_);
        stop_ = true;
    }
    work_.notify_all();
    for (std::thread& t : workers_) t.join();
    workers_.clear();
}

void ThreadPool::resize(int threads)
{
    const int wanted = std::clamp(threads, 1, kMaxThreads);
    if (wanted == threads_) return;
    stop();
    start(wanted);
}

// Runs one claimed task; a throwing task is recorded (the first one wins) and the pool carries on.
void ThreadPool::execute(std::size_t task)
{
    try {
        (*fn_)(task);
    } catch (...) {
        std::lock_guard<std::mutex> lock(m_);
        if (!error_) error_ = std::current_exception();
    }
}

void ThreadPool::workerLoop()
{
    std::unique_lock<std::mutex> lock(m_);
    for (;;) {
        work_.wait(lock, [this] { return stop_ || next_ < tasks_; });
        if (stop_) return;
        const std::size_t task = next_++;
        lock.unlock();
        execute(task);
        lock.lock();
        if (++finished_ == tasks_) done_.notify_all();
    }
}

void ThreadPool::run(std::size_t tasks, const std::function<void(std::size_t)>& fn)
{
    if (tasks == 0) return;
    if (threads_ == 1) {
        // Same contract as with workers: every task runs, the first exception is rethrown at the end
        std::exception_ptr first;
        for (std::size_t k = 0; k < tasks; ++k) {
            try {
                fn(k);
            } catch (...) {
                if (!first) first = std::current_exception();
            }
        }
        if (first) std::rethrow_exception(first);
        return;
    }

    std::unique_lock<std::mutex> lock(m_);
    fn_ = &fn;
    tasks_ = tasks;
    next_ = 0;
    finished_ = 0;
    error_ = nullptr;
    work_.notify_all();

    // The caller works too
    while (next_ < tasks_) {
        const std::size_t task = next_++;
        lock.unlock();
        execute(task);
        lock.lock();
        ++finished_;
    }
    done_.wait(lock, [this] { return finished_ == tasks_; });

    tasks_ = 0;
    next_ = 0;
    fn_ = nullptr;
    const std::exception_ptr e = error_;
    error_ = nullptr;
    lock.unlock();
    if (e) std::rethrow_exception(e);
}

} // namespace cosmos
