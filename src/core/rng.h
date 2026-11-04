#pragma once
#include <cstdint>

namespace cosmos {

// Counter-based random numbers (plan section 4.6, 7.2): SplitMix64 seeded from hash(shuttleId, tick).
// There is no global state: a generator is a single 64-bit word that the caller owns, and the same (id, tick) always
// gives the same stream on every thread and under every warp schedule.

// The SplitMix64 output function (Steele, Lea & Flood 2014): a bijection on 64-bit words with full avalanche.
inline constexpr std::uint64_t splitmix64Mix(std::uint64_t z)
{
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

// hash(id, tick): mix the id, add the tick, mix again. Not symmetric in its arguments, and neighbouring ticks of one
// shuttle (or equal ticks of neighbouring shuttles) give unrelated seeds.
inline constexpr std::uint64_t hashIdTick(std::uint64_t id, std::uint64_t tick)
{
    return splitmix64Mix(splitmix64Mix(id + 0x9E3779B97F4A7C15ull) + tick);
}

class Rng {
public:
    explicit constexpr Rng(std::uint64_t seed = 0) : state_(seed) {}

    // The stream for one shuttle at one controller tick.
    static constexpr Rng forTick(std::uint64_t shuttleId, std::uint64_t tick) { return Rng(hashIdTick(shuttleId, tick)); }

    // Next 64 random bits (the reference SplitMix64 step).
    constexpr std::uint64_t next()
    {
        state_ += 0x9E3779B97F4A7C15ull;
        return splitmix64Mix(state_);
    }

    // Uniform in [0, 1) with 53 random bits (every value is a multiple of 2^-53).
    double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }

    // Uniform in [lo, hi).
    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

    // Uniform integer in [0, n) for n >= 1 (n = 0 returns 0). The modulo bias is below n / 2^64, invisible for the
    // small n used here.
    std::uint64_t below(std::uint64_t n) { return n == 0 ? 0 : next() % n; }

    constexpr std::uint64_t state() const { return state_; }

private:
    std::uint64_t state_;
};

} // namespace cosmos
