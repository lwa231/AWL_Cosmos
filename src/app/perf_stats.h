#pragma once
// Frame-time statistics for the release benchmark (plan 14 "Performance (M8)", 17): a series of per-frame samples with its
// summary (average, percentiles, maximum), and the per-phase breakdown of a frame. GL-free, header and one .cpp.
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace cosmos {
namespace app {

struct SeriesSummary {
    std::size_t count = 0;
    double avg = 0.0, p50 = 0.0, p95 = 0.0, p99 = 0.0, max = 0.0;
};

// Nearest-rank percentile (q in [0, 100]) of an ASCENDING sorted series: the smallest sample such that at least q percent of
// the samples are not above it. 0 for an empty series.
double percentileSorted(const std::vector<double>& sorted, double q);

SeriesSummary summarize(std::vector<double> samples);

// The phases of one real frame, in the order the main loop runs them.
enum class Phase : std::size_t {
    Advance,      // RunControl::advance: clock, physics ahead, state at the display time (includes Physics)
    Bookkeeping,  // lookups, events, explosions registry, slow-down, camera update
    FillFrame,    // RenderFrame bodies and shuttles
    FillOverlays, // wells, atmospheres, circles, trails, orbits, predictor, arrows
    Explosions,   // explosion system update
    RenderFill,   // renderer.beginFrame: upload into the mapped buffer, clear
    RenderDraw,   // the draw calls and endFrame (CPU side)
    Ui,           // ImGui frame, labels, ImGui render
    Swap,         // glfwSwapBuffers (with vsync off: the wait for the GPU / driver queue)
    Poll,         // glfwPollEvents
    Count
};
inline constexpr std::size_t kPhaseCount = static_cast<std::size_t>(Phase::Count);
const char* phaseName(Phase p);

// Samples of the measured frames (after the warm-up): wall frame time and the phases, all in ms.
struct PerfSeries {
    std::vector<double> frame, cpuWork, physics, gpu;
    std::array<std::vector<double>, kPhaseCount> phase;
    // Finer physics split (ms per frame, from the simulation's own timers) and overlay split.
    std::vector<double> bodiesIas15, shuttlePass, driverState;
    std::vector<double> ovTrailRecord, ovTrailDraw, ovOrbits, ovPredict, ovArrows;
    std::vector<double> ovWellsEtc;

    void clear();
};

// A text report of the series: the summary lines and the table of phases. `label` heads it.
std::string perfReport(const PerfSeries& s, const std::string& label);

// One CSV line per frame (header first) for plotting / finding the spikes: frame, cpu, physics, gpu, then the phases.
std::string perfCsv(const PerfSeries& s);

} // namespace app
} // namespace cosmos
