#include <doctest.h>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>
#include "core/threadpool.h"

using namespace cosmos;

// The std::thread pool of the shuttle pass (plan 4.5).

TEST_SUITE("simulation thread pool")
{
    TEST_CASE("run() executes every task exactly once, for any thread count and task count")
    {
        for (int threads : {1, 2, 3, 8, 16}) {
            ThreadPool pool(threads);
            CHECK(pool.threads() == threads);
            for (std::size_t tasks : {std::size_t(0), std::size_t(1), std::size_t(5), std::size_t(64), std::size_t(1000)}) {
                std::vector<std::atomic<int>> hits(tasks);
                for (auto& h : hits) h = 0;
                pool.run(tasks, [&](std::size_t k) { hits[k].fetch_add(1); });
                for (std::size_t k = 0; k < tasks; ++k) CHECK(hits[k].load() == 1);
            }
        }
    }

    TEST_CASE("the thread count is clamped to 1..16 and can be changed")
    {
        ThreadPool pool(0);
        CHECK(pool.threads() == 1);
        pool.resize(100);
        CHECK(pool.threads() == ThreadPool::kMaxThreads);
        pool.resize(-3);
        CHECK(pool.threads() == 1);
        pool.resize(4);
        CHECK(pool.threads() == 4);
        std::atomic<int> sum{0};
        pool.run(10, [&](std::size_t k) { sum.fetch_add(static_cast<int>(k)); });
        CHECK(sum.load() == 45);
    }

    TEST_CASE("many runs back to back (no lost wake-up, no deadlock), work really spreads over threads")
    {
        ThreadPool pool(4);
        std::atomic<long> total{0};
        for (int rep = 0; rep < 3000; ++rep) pool.run(7, [&](std::size_t k) { total.fetch_add(static_cast<long>(k) + 1); });
        CHECK(total.load() == 3000L * 28);

        // With enough slow tasks more than one thread takes part (checked by thread id)
        std::vector<std::thread::id> who(64);
        pool.run(64, [&](std::size_t k) {
            who[k] = std::this_thread::get_id();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        });
        std::size_t distinct = 0;
        for (std::size_t k = 0; k < who.size(); ++k) {
            bool seen = false;
            for (std::size_t j = 0; j < k; ++j) seen = seen || who[j] == who[k];
            if (!seen) ++distinct;
        }
        CHECK(distinct >= 2);
    }

    TEST_CASE("an exception in a task is rethrown from run() after all tasks finished; the pool stays usable")
    {
        for (int threads : {1, 4}) {
            ThreadPool pool(threads);
            std::atomic<int> done{0};
            CHECK_THROWS_AS(pool.run(20, [&](std::size_t k) {
                                if (k == 7) throw std::runtime_error("boom");
                                done.fetch_add(1);
                            }),
                            std::runtime_error);
            CHECK(done.load() == 19);   // the others all ran
            std::atomic<int> again{0};
            pool.run(20, [&](std::size_t) { again.fetch_add(1); });
            CHECK(again.load() == 20);
        }
    }
}
