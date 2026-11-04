#pragma once
// Trails (plan 9.5): a per-object ring buffer of TIMESTAMPED samples in inertial coordinates, thinned while recording,
// re-expressed in the chosen reference frame at draw time. GL-free.
//
// Recording (one sample per object per frame, at the display time):
//   - the latest observation is always kept as the trail's HEAD, so the drawn line always ends at the object;
//   - a sample enters the ring only when the object has moved at least `minSpacing` (arc length since the last stored
//     sample; the caller passes a few pixels at the current zoom, so it is pixel-scale aware) or its velocity has turned
//     by more than the turn angle (the curvature criterion, independent of zoom). A straight, steady stretch costs two
//     samples, an orbit about 120 per revolution. The trade-off of a zoom-dependent spacing: history recorded while
//     zoomed in stays denser than needed (the ring then holds fewer revolutions); it never gets coarser than it was.
//   - each sample keeps the velocity, so positions between samples come from a cubic Hermite (exact for straight motion
//     at any speed profile, fourth order for curves).
//   - a frame spans many body steps at high warp; the trail is sampled at frame times (the dense output of older steps is
//     gone), so at extreme warp a fast orbit is a polygon with a vertex per frame.
//
// Drawing in a frame (appendTrail): the line vertex for time t is  p_obj(t) - p_origin(t)  plus a per-frame offset, where
// p_origin(t) is the frame origin's own trail evaluated at t (Hermite). The vertex times are the union of the object's
// and the origin's sample times, so the polyline is accurate in the moving frame even when the object itself is nearly
// straight (a star seen from a planet) - a body circling its moving parent is drawn as a closed loop.
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "app/overlay_arrows.h"
#include "app/render_frame.h"

namespace cosmos {
namespace app {

struct TrailSample {
    double t = 0.0;            // display time, s
    double x = 0.0, y = 0.0;   // inertial position, m
    double vx = 0.0, vy = 0.0; // inertial velocity, m/s
};

constexpr std::size_t kTrailCapacity = 4096;           // stored samples per object
constexpr double kTrailTurnRad = 3.0 * 3.14159265358979323846 / 180.0;
constexpr double kTrailSpacingPx = 6.0;                // arc-length spacing used by the overlay builder

// Cubic Hermite position at t between a and b (t clamped to [a.t, b.t]).
void hermiteXY(const TrailSample& a, const TrailSample& b, double t, double& x, double& y);

class Trail {
public:
    bool empty() const { return !haveHead_; }
    // Forget everything but keep the buffer (no reallocation when it fills again).
    void clear() { count_ = 0; start_ = 0; haveHead_ = false; }

    // Observe the object at time s.t. Returns true if the sample entered the ring. A sample not later than the newest
    // stored one only refreshes the head.
    bool record(const TrailSample& s, double minSpacing, double turnRad = kTrailTurnRad);

    std::size_t storedCount() const { return count_; }
    const TrailSample& stored(std::size_t k) const { return buf_[(start_ + k) % buf_.size()]; }   // 0 = oldest
    const TrailSample& head() const { return head_; }

    // Stored samples plus the head when it is newer than the newest stored one: the polyline the trail draws.
    std::size_t sampleCount() const { return count_ + (headExtra() ? 1 : 0); }
    const TrailSample& sample(std::size_t k) const { return k < count_ ? stored(k) : head_; }
    double firstTime() const { return sample(0).t; }
    double lastTime() const { return head_.t; }

private:
    bool headExtra() const { return haveHead_ && (count_ == 0 || head_.t > stored(count_ - 1).t); }
    void store(const TrailSample& s);

    std::vector<TrailSample> buf_;
    std::size_t start_ = 0, count_ = 0;
    TrailSample head_;
    bool haveHead_ = false;
};

// Evaluates a trail at non-decreasing times (a forward-only cursor: O(1) amortised per call).
class TrailCursor {
public:
    explicit TrailCursor(const Trail* trail) : trail_(trail) {}
    void eval(double t, double& x, double& y);

private:
    const Trail* trail_;
    std::size_t seg_ = 0;
};

struct TrailDrawParams {
    double now = 0.0;            // display time: the trail ends here
    double tailTime = -1e300;    // the oldest time to draw (now - trail length), -huge for the whole ring
    double offX = 0.0, offY = 0.0;   // added to (object - origin): the origin's current position minus the camera centre
    double metresPerPixel = 1.0;
    Rgba colour;                 // alpha is the opacity at the head; it fades linearly to ~0 at the tail
    float widthPx = 1.5f;
    double minStepPx = 1.0;      // vertices closer than this to the previous one are dropped
};

// Appends one strip to the batch. `origin` is the frame origin's trail (null for the inertial frame; the offsets then
// carry -camera centre). Returns the number of vertices added (0 if fewer than two).
std::size_t appendTrail(const Trail& object, const Trail* origin, const TrailDrawParams& p, LineBatch& batch);

// All trails of a scene by object ID (bodies and shuttles share one ID line). The key kBarycentreKey holds the system
// barycentre, the origin of the barycentre frame.
constexpr std::uint64_t kBarycentreKey = ~std::uint64_t(0);

class TrailStore {
public:
    Trail& touch(std::uint64_t id, std::uint64_t frameStamp);
    const Trail* find(std::uint64_t id) const;
    void erase(std::uint64_t id) { trails_.erase(id); }
    void clear() { trails_.clear(); }
    std::size_t size() const { return trails_.size(); }
    // Drops every trail not touched with this stamp (objects that no longer exist).
    void pruneUntouched(std::uint64_t frameStamp);

private:
    struct Entry {
        Trail trail;
        std::uint64_t stamp = 0;
    };
    std::unordered_map<std::uint64_t, Entry> trails_;
};

} // namespace app
} // namespace cosmos
