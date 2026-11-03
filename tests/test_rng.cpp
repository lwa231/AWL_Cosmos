#include <doctest.h>
#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <set>
#include "core/rng.h"

using namespace cosmos;

// Counter-based SplitMix64 (plan 4.6, 7.2).

TEST_SUITE("rng")
{
    TEST_CASE("SplitMix64 reference stream from seed 0 and seed 1234567")
    {
        // Published SplitMix64 outputs for seed 0 (first three), and values computed independently in Python for 1234567.
        Rng r0(0);
        CHECK(r0.next() == 0xE220A8397B1DCDAFull);
        CHECK(r0.next() == 0x6E789E6AA1B965F4ull);
        CHECK(r0.next() == 0x06C45D188009454Full);
        Rng r1(1234567);
        CHECK(r1.next() == 0x599ED017FB08FC85ull);
        CHECK(r1.next() == 0x2C73F08458540FA5ull);
    }

    TEST_CASE("hash(id, tick) is a pure function, not symmetric, and forTick seeds with it")
    {
        CHECK(hashIdTick(7, 3) == 0x27FC3D62819D71A4ull); // computed independently in Python
        CHECK(hashIdTick(0, 0) == 0x48218226FF3CD4BFull);
        CHECK(hashIdTick(7, 3) == hashIdTick(7, 3));
        CHECK(hashIdTick(7, 3) != hashIdTick(3, 7));
        CHECK(Rng::forTick(7, 3).state() == hashIdTick(7, 3));
        Rng a = Rng::forTick(42, 100), b = Rng::forTick(42, 100);
        for (int k = 0; k < 100; ++k) CHECK(a.next() == b.next());
    }

    TEST_CASE("different (id, tick) pairs give different seeds (no collisions over a 200 x 5000 grid)")
    {
        std::set<std::uint64_t> seen;
        for (std::uint64_t id = 1; id <= 200; ++id)
            for (std::uint64_t tick = 0; tick < 5000; ++tick) seen.insert(hashIdTick(id, tick));
        CHECK(seen.size() == 200u * 5000u);
    }

    TEST_CASE("avalanche: flipping one input bit flips about half of the seed bits")
    {
        // Average Hamming distance over 64 id bits x 64 tick bits flipped; ideal 32.
        double sum = 0.0;
        int n = 0;
        for (int bit = 0; bit < 64; ++bit) {
            for (std::uint64_t base = 1; base <= 20; ++base) {
                const std::uint64_t h0 = hashIdTick(base * 1000003ull, base * 7919ull);
                const std::uint64_t h1 = hashIdTick(base * 1000003ull ^ (1ull << bit), base * 7919ull);
                const std::uint64_t h2 = hashIdTick(base * 1000003ull, base * 7919ull ^ (1ull << bit));
                sum += std::bitset<64>(h0 ^ h1).count() + std::bitset<64>(h0 ^ h2).count();
                n += 2;
            }
        }
        const double mean = sum / n;
        CHECK(mean > 31.0);
        CHECK(mean < 33.0);
    }

    TEST_CASE("uniform() is in [0, 1) with the right mean and variance")
    {
        Rng r = Rng::forTick(5, 9);
        const int N = 200000;
        double s = 0.0, s2 = 0.0, lo = 1.0, hi = 0.0;
        for (int k = 0; k < N; ++k) {
            const double u = r.uniform();
            s += u;
            s2 += u * u;
            lo = std::min(lo, u);
            hi = std::max(hi, u);
        }
        const double mean = s / N, var = s2 / N - mean * mean;
        CHECK(lo >= 0.0);
        CHECK(hi < 1.0);
        // sigma of the mean = sqrt(1/12 / N) = 6.5e-4; 4 sigma = 2.6e-3
        CHECK(std::fabs(mean - 0.5) < 2.6e-3);
        CHECK(var == doctest::Approx(1.0 / 12.0).epsilon(2e-2));
        // uniform(lo, hi) maps linearly
        Rng a(99), b(99);
        CHECK(a.uniform(-2.0, 6.0) == doctest::Approx(-2.0 + 8.0 * b.uniform()).epsilon(1e-15));
    }

    TEST_CASE("the first draw of neighbouring ticks of one shuttle is evenly spread (chi-square, 16 buckets)")
    {
        const int N = 16000;
        int count[16] = {};
        for (int tick = 0; tick < N; ++tick) ++count[Rng::forTick(7, static_cast<std::uint64_t>(tick)).next() >> 60];
        double chi2 = 0.0;
        for (int c : count) chi2 += (c - N / 16.0) * (c - N / 16.0) / (N / 16.0);
        CHECK(chi2 < 37.7); // 99.9th percentile of chi-square with 15 degrees of freedom
    }

    TEST_CASE("below(n) stays in range and covers it")
    {
        Rng r(3);
        std::set<std::uint64_t> seen;
        for (int k = 0; k < 2000; ++k) {
            const std::uint64_t v = r.below(10);
            CHECK(v < 10u);
            seen.insert(v);
        }
        CHECK(seen.size() == 10u);
        CHECK(r.below(0) == 0u);
        CHECK(r.below(1) == 0u);
    }
}
