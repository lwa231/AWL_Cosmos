#include "ui/stats_panel.h"
#include <algorithm>
#include <cmath>
#include "imgui.h"
#include "implot.h"
#include "ui/theme.h"

namespace cosmos {
namespace ui {

namespace {
constexpr double kFloor = 1e-18;   // the log axis cannot show 0: an exact zero drift is drawn here
constexpr double kDay = 86400.0;

ImPlotSpec lineSpec(const ImVec4& colour, float weight, int offset = 0)
{
    ImPlotSpec spec;
    spec.LineColor = colour;
    spec.LineWeight = weight;
    spec.Offset = offset;
    return spec;
}
} // namespace

double StatsPanel::relative(double delta, double scale)
{
    if (!std::isfinite(delta)) return 0.0;
    return scale > 0.0 ? delta / scale : delta;   // no scale (an empty or motionless system): report G times the true value
}

void StatsPanel::reset()
{
    t_.clear();
    dE_.clear();
    dP_.clear();
    dL_.clear();
    haveReference_ = false;
    latest_ = Drift{};
    stride_ = 1;
    recorded_ = 0;
}

void StatsPanel::recordConserved(double simTime, const Conserved& c, double momentumScale)
{
    if (haveReference_ && simTime < lastT_) reset();   // time went backwards: a snapshot was restored
    if (!haveReference_) {
        reference_ = c;
        t0_ = simTime;
        haveReference_ = true;
    }
    lastT_ = simTime;

    Drift d;
    d.energy = relative(std::fabs(c.energyG - reference_.energyG), std::fabs(reference_.energyG));
    d.momentum = relative(norm(c.momentumG - reference_.momentumG), momentumScale);
    d.angular = relative(norm(c.angularMomentumG - reference_.angularMomentumG), norm(reference_.angularMomentumG));
    latest_ = d;

    if (recorded_++ % stride_ != 0) return;
    if (t_.size() >= kMaxConserved) {   // halve the history, keep the whole run visible
        auto thin = [](std::vector<double>& v) {
            std::size_t w = 0;
            for (std::size_t r = 0; r < v.size(); r += 2) v[w++] = v[r];
            v.resize(w);
        };
        thin(t_); thin(dE_); thin(dP_); thin(dL_);
        stride_ *= 2;
    }
    t_.push_back((simTime - t0_) / kDay);
    dE_.push_back(std::max(d.energy, kFloor));
    dP_.push_back(std::max(d.momentum, kFloor));
    dL_.push_back(std::max(d.angular, kFloor));
}

void StatsPanel::recordFrame(double frameMs, double physicsMs)
{
    if (frameX_.empty()) {
        frameX_.assign(kFrameHistory, 0.0f);
        frameMs_.assign(kFrameHistory, 0.0f);
        physicsMs_.assign(kFrameHistory, 0.0f);
    }
    frameX_[frameHead_] = static_cast<float>(frameIndex_);
    frameMs_[frameHead_] = static_cast<float>(frameMs);
    physicsMs_[frameHead_] = static_cast<float>(physicsMs);
    frameHead_ = (frameHead_ + 1) % kFrameHistory;
    frameIndex_ += 1.0;
    frameCount_ = std::min(frameCount_ + 1, kFrameHistory);

    double sum = 0.0, mx = 0.0;
    for (std::size_t i = 0; i < frameCount_; ++i) {
        sum += frameMs_[i];
        mx = std::max(mx, static_cast<double>(frameMs_[i]));
    }
    frameAvg_ = frameCount_ ? sum / static_cast<double>(frameCount_) : 0.0;
    frameMax_ = mx;
}

bool StatsPanel::windowRect(float out[4]) const
{
    if (!rectValid_) return false;
    for (int i = 0; i < 4; ++i) out[i] = rect_[i];
    return true;
}

void StatsPanel::draw(bool* open)
{
    rectValid_ = false;
    if (!open || !*open) return;
    const Palette& pal = palette();
    const ImGuiIO& io = ImGui::GetIO();
    const float s = ImGui::GetFontSize() / 16.0f;   // follows the font (DPI) scale

    ImGui::SetNextWindowSize(ImVec2(470.0f * s, 525.0f * s), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 16.0f * s, 16.0f * s), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.88f);
    if (!ImGui::Begin("Stats", open, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }

    ImGui::TextColored(pal.textDim, "Conservation drift (integrator error: merge losses added back)");
    const ImPlotFlags plotFlags = ImPlotFlags_NoMouseText;
    if (ImPlot::BeginPlot("##drift", ImVec2(-1.0f, 210.0f * s), plotFlags)) {
        ImPlot::SetupAxes("simulated time (days)", nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxisFormat(ImAxis_Y1, "%.0e");
        ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, kFloor, 1e12);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_None);
        if (!t_.empty()) {
            const int n = static_cast<int>(t_.size());
            ImPlot::PlotLine("|dE/E0|", t_.data(), dE_.data(), n, lineSpec(pal.accent, 1.8f * s));
            ImPlot::PlotLine("|dP|/sum mu|v|", t_.data(), dP_.data(), n, lineSpec(pal.good, 1.8f * s));
            ImPlot::PlotLine("|dL/L0|", t_.data(), dL_.data(), n, lineSpec(pal.warn, 1.8f * s));
        }
        ImPlot::EndPlot();
    }
    ImGui::Text("now:  dE/E0 %.2e   dP %.2e   dL/L0 %.2e", latest_.energy, latest_.momentum, latest_.angular);

    ImGui::Spacing();
    ImGui::TextColored(pal.textDim, "Frame time");
    if (ImPlot::BeginPlot("##frame", ImVec2(-1.0f, 150.0f * s), plotFlags)) {
        ImPlot::SetupAxes(nullptr, "ms", ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, 0.0, 1000.0);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_None);
        if (frameCount_ > 1) {
            const int n = static_cast<int>(frameCount_);
            const int offset = frameCount_ == kFrameHistory ? static_cast<int>(frameHead_) : 0;
            ImPlot::PlotLine("frame", frameX_.data(), frameMs_.data(), n, lineSpec(pal.accent, 1.6f * s, offset));
            ImPlot::PlotLine("physics", frameX_.data(), physicsMs_.data(), n, lineSpec(pal.warn, 1.6f * s, offset));
        }
        ImPlot::EndPlot();
    }
    const double fps = frameAvg_ > 0.0 ? 1000.0 / frameAvg_ : 0.0;
    ImGui::Text("avg %.2f ms (%.0f fps)   worst %.2f ms   last %.2f ms", frameAvg_, fps, frameMax_,
                frameCount_ ? static_cast<double>(frameMs_[(frameHead_ + kFrameHistory - 1) % kFrameHistory]) : 0.0);
    {
        const ImVec2 p = ImGui::GetWindowPos(), z = ImGui::GetWindowSize();
        rect_[0] = p.x; rect_[1] = p.y; rect_[2] = p.x + z.x; rect_[3] = p.y + z.y;
        rectValid_ = true;
    }
    ImGui::End();
}

} // namespace ui
} // namespace cosmos
