#include "app/perf_stats.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace cosmos {
namespace app {

double percentileSorted(const std::vector<double>& sorted, double q)
{
    if (sorted.empty()) return 0.0;
    q = std::min(100.0, std::max(0.0, q));
    const double rank = std::ceil(q / 100.0 * static_cast<double>(sorted.size()));
    const std::size_t i = rank < 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
    return sorted[std::min(i, sorted.size() - 1)];
}

SeriesSummary summarize(std::vector<double> v)
{
    SeriesSummary s;
    s.count = v.size();
    if (v.empty()) return s;
    double sum = 0.0;
    for (double x : v) sum += x;
    s.avg = sum / static_cast<double>(v.size());
    std::sort(v.begin(), v.end());
    s.p50 = percentileSorted(v, 50.0);
    s.p95 = percentileSorted(v, 95.0);
    s.p99 = percentileSorted(v, 99.0);
    s.max = v.back();
    return s;
}

const char* phaseName(Phase p)
{
    switch (p) {
    case Phase::Advance: return "advance (physics+state)";
    case Phase::Bookkeeping: return "lookups/events/camera";
    case Phase::FillFrame: return "fillFrame";
    case Phase::FillOverlays: return "fillOverlays";
    case Phase::Explosions: return "explosions update";
    case Phase::RenderFill: return "renderer.beginFrame";
    case Phase::RenderDraw: return "renderer draw+end";
    case Phase::Ui: return "ImGui + labels";
    case Phase::Swap: return "swapBuffers";
    case Phase::Poll: return "pollEvents";
    case Phase::Count: break;
    }
    return "?";
}

void PerfSeries::clear()
{
    frame.clear(); cpuWork.clear(); physics.clear(); gpu.clear();
    for (auto& p : phase) p.clear();
    bodiesIas15.clear(); shuttlePass.clear(); driverState.clear();
    ovTrailRecord.clear(); ovTrailDraw.clear(); ovOrbits.clear(); ovPredict.clear(); ovArrows.clear(); ovWellsEtc.clear();
}

namespace {

void line(std::string& out, const char* name, const std::vector<double>& v)
{
    if (v.empty()) return;
    const SeriesSummary s = summarize(v);
    char buf[200];
    std::snprintf(buf, sizeof buf, "  %-26s avg %7.3f  p50 %7.3f  p95 %7.3f  p99 %7.3f  max %8.3f\n", name, s.avg, s.p50, s.p95, s.p99, s.max);
    out += buf;
}

} // namespace

std::string perfReport(const PerfSeries& s, const std::string& label)
{
    std::string out;
    char buf[200];
    const SeriesSummary f = summarize(s.frame);
    std::snprintf(buf, sizeof buf, "PERF %s: %zu frames measured\n", label.c_str(), f.count);
    out += buf;
    std::snprintf(buf, sizeof buf, "  frame ms (wall): avg %.3f p50 %.3f p95 %.3f p99 %.3f max %.3f  (avg %.1f fps)\n", f.avg, f.p50, f.p95, f.p99,
                  f.max, f.avg > 0.0 ? 1000.0 / f.avg : 0.0);
    out += buf;
    out += "  per frame, ms:\n";
    line(out, "frame (wall)", s.frame);
    line(out, "CPU work", s.cpuWork);
    line(out, "GPU (whole frame)", s.gpu);
    line(out, "physics (driver)", s.physics);
    line(out, "  bodies IAS15 + swept", s.bodiesIas15);
    line(out, "  shuttle pass", s.shuttlePass);
    line(out, "  driver buildState", s.driverState);
    out += "  phases of the main loop:\n";
    for (std::size_t i = 0; i < kPhaseCount; ++i) line(out, phaseName(static_cast<Phase>(i)), s.phase[i]);
    out += "  overlay builder:\n";
    line(out, "  wells/atmo/circles", s.ovWellsEtc);
    line(out, "  trails record", s.ovTrailRecord);
    line(out, "  trails draw", s.ovTrailDraw);
    line(out, "  osculating orbits", s.ovOrbits);
    line(out, "  predicted path", s.ovPredict);
    line(out, "  arrows", s.ovArrows);
    return out;
}

std::string perfCsv(const PerfSeries& s)
{
    std::string out = "frame_ms,cpu_ms,physics_ms,gpu_ms,shuttle_pass_ms,bodies_ms";
    for (std::size_t i = 0; i < kPhaseCount; ++i) {
        out += ",";
        out += phaseName(static_cast<Phase>(i));
    }
    out += "\n";
    char buf[64];
    auto at = [](const std::vector<double>& v, std::size_t i) { return i < v.size() ? v[i] : 0.0; };
    for (std::size_t k = 0; k < s.frame.size(); ++k) {
        std::snprintf(buf, sizeof buf, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", s.frame[k], at(s.cpuWork, k), at(s.physics, k), at(s.gpu, k),
                      at(s.shuttlePass, k), at(s.bodiesIas15, k));
        out += buf;
        for (std::size_t i = 0; i < kPhaseCount; ++i) {
            std::snprintf(buf, sizeof buf, ",%.4f", at(s.phase[i], k));
            out += buf;
        }
        out += "\n";
    }
    return out;
}

} // namespace app
} // namespace cosmos
