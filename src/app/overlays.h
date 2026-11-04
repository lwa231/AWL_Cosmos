#pragma once
// OverlayBuilder: the CPU-side line geometry of the plan 9.5 overlays that are drawn with lines (M7):
//   velocity and acceleration arrows, trails, osculating orbits, predicted paths.
// GL-free and read-only: it consumes the DisplayState (and the Simulation for shuttle masses and thrust) and fills
// RenderFrame::underlay (trails, orbits, predicted path; plan 9.4 steps 4) and RenderFrame::overlay (arrows; step 9) in
// camera-relative float metres (plan 9.3). It never changes the simulation, and the predicted path runs on a COPY in a
// worker thread (overlay_predict.h).
//
// Per frame, after the camera has been updated for the display time:
//     overlays.build(driver, runControl.generation(), OverlayView::fromCamera(camera), frame);
// build() only APPENDS to the two batches (RenderFrame::clearObjects empties them). Hooks: onEvent / onObjectRemoved for
// the events the app hands out (dueEvents), reset() for Stop / rebuild / loading a scene. A different `generation`
// (RunControl::generation()) or a display time that went backwards also resets, so the hooks are belt and braces: the
// trails of objects that no longer appear in the state are dropped anyway.
//
// Cost: the arrows are O(bodies^2) for the accelerations (one gravity pass, the same as the physics), trails O(samples)
// per object with a pixel-based decimation, orbits one closest-pair hierarchy per frame (buildHierarchy allocates a few
// small vectors; everything else - scratch buffers, the trail rings, the frame batches - keeps its capacity, so a steady
// frame allocates nothing else). Everything is skipped while its toggle is off; trail history is dropped while trails
// are off unless keepTrailsWhileHidden is set.
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "app/camera.h"
#include "app/overlay_arrows.h"
#include "app/overlay_orbits.h"
#include "app/overlay_predict.h"
#include "app/overlay_trails.h"
#include "app/palette.h"
#include "app/render_frame.h"
#include "app/sim_driver.h"
#include "core/hierarchy.h"

namespace cosmos {
namespace app {

// What the overlays need from the camera.
struct OverlayView {
    double metresPerPixel = 1.0;
    Vec3 centre;                                  // camera centre, world (simulation) coordinates, z ignored
    FrameMode frameMode = FrameMode::Inertial;
    std::uint64_t frameBody = 0;
    Follow follow;                                // the followed object (default prediction target)

    static OverlayView fromCamera(const Camera& c);
};

struct OverlaySettings {
    // Toggles (the View window: ViewOverlays).
    bool velocityArrows = false;
    bool accelerationArrows = false;
    bool trails = false;
    bool osculatingOrbits = false;
    bool predictedPath = false;
    // Which kinds of objects the per-object overlays cover.
    bool onBodies = true;
    bool onShuttles = true;

    // Arrows: lengthPx is the slider, the references are the magnitudes that get the full length (0 = the largest on
    // screen this frame, so the biggest arrow is always lengthPx long).
    ArrowScale arrowMode = ArrowScale::Sqrt;
    double arrowLengthPx = 80.0;
    double velocityReference = 0.0;       // m/s
    double accelerationReference = 0.0;   // m/s^2

    // Trails: length in sim seconds (0 = the whole ring), opacity at the head.
    double trailSeconds = 0.0;
    float trailAlpha = 0.85f;
    bool keepTrailsWhileHidden = false;   // keep recording while the toggle is off (costs a little per frame)

    // Orbits.
    std::size_t orbitSegments = kOrbitSegments;
    float orbitAlpha = 0.6f;

    // Predicted path: sim seconds ahead (0 = automatic: 1.1 osculating periods of the target, capped).
    double predictSeconds = 0.0;
    double predictMaxSeconds = 3.2e8;     // cap of the automatic horizon, about 10 years
    int predictSamples = 512;
    double predictRefresh = 0.25;         // new request when this fraction of the horizon has passed (and the worker is idle)
    double predictMinRealGap = 0.25;      // real seconds between requests (RenderFrame::time)
};

struct OverlayStats {
    std::size_t trailObjects = 0;
    std::size_t trailVertices = 0;
    std::size_t orbits = 0;
    std::size_t arrows = 0;
    std::size_t predictedVertices = 0;
    bool predictionPending = false;       // a worker job is running or queued
    bool predictionDrawn = false;
    // Wall time of this build() per part, ms (the benchmark's profile; negligible cost: five clock reads).
    double trailRecordMs = 0.0, trailDrawMs = 0.0, orbitsMs = 0.0, predictMs = 0.0, arrowsMs = 0.0;
};

class OverlayBuilder {
public:
    OverlaySettings settings;

    OverlayBuilder();
    OverlayBuilder(const OverlayBuilder&) = delete;
    OverlayBuilder& operator=(const OverlayBuilder&) = delete;

    // Shuttle colours (trails, orbits, arrows use the first sector colour). Null = a fixed light blue. Not owned.
    void setShuttlePalette(const ShuttlePalette* palette) { palette_ = palette; }
    // Integrator settings of the predictor's copy (RunOptions::sim), so the prediction matches the live physics.
    void setPredictorOptions(const SimulationOptions& options) { predictorOptions_ = options; }
    // The object whose path is predicted; None = the camera's follow target.
    void setPredictionTarget(TargetKind kind, std::uint64_t id) { predTarget_ = {kind, id}; }

    void build(const SimDriver& driver, std::uint64_t generation, const OverlayView& view, RenderFrame& frame);
    // The same from its parts (the driver's state() and sim()).
    void build(const DisplayState& state, const Simulation& sim, std::uint64_t generation, const OverlayView& view,
              RenderFrame& frame);

    // An object is gone (merged away or destroyed): drop its trail (and the prediction if it was the target).
    void onObjectRemoved(std::uint64_t id);
    // Merge (the absorbed body) and ShuttleDestroyed (the shuttle) remove objects; other events are ignored.
    void onEvent(const Event& e);
    // Stop / rebuild / new scene: forget every trail, the cached orbits and the prediction.
    void reset();

    const OverlayStats& stats() const { return stats_; }
    PathPredictor& predictor() { return predictor_; }
    const TrailStore& trailStore() const { return trails_; }

private:
    struct FrameOrigin {
        FrameMode mode = FrameMode::Inertial;   // the mode actually used (a vanished frame body falls back to Inertial)
        std::uint64_t body = 0;
        Vec3 pos, vel;
    };
    struct ArrowItem {
        double x = 0.0, y = 0.0;   // tail, camera-relative
        double dx = 0.0, dy = 0.0; // vector (frame velocity or acceleration)
        double mag = 0.0;
        Rgba colour;
    };

    FrameOrigin frameOrigin(const DisplayState& st, const OverlayView& view) const;
    Rgba objectColour(const DisplayBody& b, float alpha) const;
    Rgba objectColour(const DisplayShuttle& s, float alpha) const;
    void ensureHierarchy(const DisplayState& st);

    void recordTrails(const DisplayState& st, const OverlayView& view);
    void drawTrails(const DisplayState& st, const OverlayView& view, const FrameOrigin& o, RenderFrame& frame);
    void drawOrbits(const DisplayState& st, const OverlayView& view, RenderFrame& frame);
    void updatePrediction(const DisplayState& st, const Simulation& sim, const OverlayView& view, const FrameOrigin& o,
                          RenderFrame& frame);
    void drawArrows(const DisplayState& st, const Simulation& sim, const OverlayView& view, const FrameOrigin& o,
                    RenderFrame& frame);
    void appendOrbit(const Vec3& parentPos, const Vec3& parentVel, double muTotal, const Vec3& r, const Vec3& v,
                     const Rgba& colour, const OverlayView& view, RenderFrame& frame);
    double predictionHorizon(const DisplayState& st, const PredictRequest& target);   // target: kind and id

    const ShuttlePalette* palette_ = nullptr;
    SimulationOptions predictorOptions_;
    TrailStore trails_;
    PathPredictor predictor_;
    OverlayStats stats_;

    std::uint64_t generation_ = ~std::uint64_t(0);
    std::uint64_t stamp_ = 0;
    double lastT_ = 0.0;
    bool haveTime_ = false;
    bool trailsWereOn_ = false;

    // Hierarchy of the bodies of the current frame (built on demand).
    Hierarchy hier_;
    bool haveHier_ = false;
    std::vector<Vec3> hr_, hv_;
    std::vector<double> hmu_;

    // Prediction bookkeeping.
    struct Target {
        TargetKind kind = TargetKind::None;
        std::uint64_t id = 0;
    };
    Target predTarget_;
    Target reqTarget_;
    std::uint64_t reqGeneration_ = ~std::uint64_t(0);
    std::uint64_t reqNumber_ = 0;
    double reqT0_ = 0.0, reqHorizon_ = 0.0, reqReal_ = -1e300;
    bool reqLive_ = false;

    // Scratch (capacity kept between frames).
    OrbitCurve curve_;
    std::vector<ArrowItem> arrowItems_;
    BodyState accState_;
    std::vector<double> accMu_, accRadius_;
    Accel acc_;
};

} // namespace app
} // namespace cosmos
