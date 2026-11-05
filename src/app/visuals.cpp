#include "app/visuals.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

namespace {

// Blackbody display colours (sRGB, brightest channel 1), the usual published 2-degree values rounded to 8 bits.
struct BbEntry {
    float kelvin;
    float r, g, b;
};
constexpr BbEntry kBlackbody[] = {
    {1000.0f, 255, 56, 0},     {1500.0f, 255, 109, 0},    {2000.0f, 255, 137, 18},   {2500.0f, 255, 161, 72},
    {3000.0f, 255, 180, 107},  {3500.0f, 255, 196, 137},  {4000.0f, 255, 209, 163},  {4500.0f, 255, 219, 186},
    {5000.0f, 255, 228, 206},  {5500.0f, 255, 236, 224},  {6000.0f, 255, 243, 239},  {6500.0f, 255, 249, 253},
    {7000.0f, 245, 243, 255},  {8000.0f, 227, 233, 255},  {9000.0f, 214, 225, 255},  {10000.0f, 204, 219, 255},
    {12000.0f, 191, 211, 255}, {15000.0f, 179, 204, 255}, {20000.0f, 168, 197, 255}, {30000.0f, 159, 191, 255},
    {40000.0f, 155, 188, 255},
};
constexpr int kBbCount = static_cast<int>(sizeof(kBlackbody) / sizeof(kBlackbody[0]));

RenderColour normalised(const RenderColour& c)
{
    const float m = std::max(std::max(c.r, c.g), std::max(c.b, 1e-6f));
    return {c.r / m, c.g / m, c.b / m};
}

float hash01(std::uint64_t x)
{
    x ^= x >> 33; x *= 0xff51afd7ed558ccdull;
    x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ull;
    x ^= x >> 33;
    return static_cast<float>(x >> 40) / static_cast<float>(1u << 24);
}

RenderColour airTint(const std::array<float, 3>& c)   // as overlay_fill.cpp's bubbles: the body colour pulled toward sky blue
{
    return {0.6f * c[0] + 0.4f * 0.45f, 0.6f * c[1] + 0.4f * 0.70f, 0.6f * c[2] + 0.4f * 1.0f};
}

} // namespace

RenderColour blackbodyColour(float kelvin)
{
    if (!(kelvin > kBlackbody[0].kelvin)) kelvin = kBlackbody[0].kelvin;
    if (kelvin >= kBlackbody[kBbCount - 1].kelvin) {
        const BbEntry& e = kBlackbody[kBbCount - 1];
        return {e.r / 255.0f, e.g / 255.0f, e.b / 255.0f};
    }
    int i = 0;
    while (i + 2 < kBbCount && kBlackbody[i + 1].kelvin <= kelvin) ++i;
    const BbEntry& a = kBlackbody[i];
    const BbEntry& b = kBlackbody[i + 1];
    const float t = std::log(kelvin / a.kelvin) / std::log(b.kelvin / a.kelvin);
    return {(a.r + (b.r - a.r) * t) / 255.0f, (a.g + (b.g - a.g) * t) / 255.0f, (a.b + (b.b - a.b) * t) / 255.0f};
}

float temperatureFromColour(const RenderColour& colour)
{
    const RenderColour c = normalised(colour);
    float best = 6500.0f, bestErr = 1e30f;
    // 1000 K .. 40000 K in 1 % steps
    for (float k = 1000.0f; k <= 40000.0f; k *= 1.01f) {
        const RenderColour b = blackbodyColour(k);
        const float dr = b.r - c.r, dg = b.g - c.g, db = b.b - c.b;
        const float err = dr * dr + dg * dg + db * db;
        if (err < bestErr) { bestErr = err; best = k; }
    }
    return best;
}

float starTemperature(BodyKind kind, const RenderColour& colour)
{
    if (kind == BodyKind::NeutronStar) return 1.0e6f;
    if (kind == BodyKind::Star) return temperatureFromColour(colour);
    return 0.0f;
}

bool isPlanetClass(BodyKind kind)
{
    switch (kind) {
    case BodyKind::Planet: case BodyKind::GasGiant: case BodyKind::DwarfPlanet: case BodyKind::Moon: case BodyKind::Asteroid:
        return true;
    default:
        return false;
    }
}

void applyBodyVisual(const DisplayBody& body, RenderBody& rb, double displayTime)
{
    const std::array<float, 3>& c = body.colour;
    rb.seed = hash01(body.id * 0x9e3779b97f4a7c15ull + 1);
    rb.spinPhase = static_cast<float>(std::fmod(body.spinRate * displayTime, 6.283185307179586));
    const double rho0 = body.atmosphere ? body.atmosphere->rho0 : 0.0;
    rb.atmosphereTint = airTint(c);
    rb.atmosphereRim = rho0 > 0.0 ? static_cast<float>(std::min(1.0, std::max(0.25, 0.55 + 0.2 * std::log10(rho0)))) : 0.0f;
    rb.cloudCover = 0.0f;
    rb.temperature = 0.0f;

    switch (body.kind) {
    case BodyKind::Star:
    case BodyKind::NeutronStar:
        rb.style = BodyStyle::Star;
        rb.temperature = starTemperature(body.kind, rb.colour);
        rb.colour = blackbodyColour(rb.temperature);   // coloured by temperature
        return;
    case BodyKind::BlackHole:
        rb.style = BodyStyle::BlackHole;
        return;
    case BodyKind::GasGiant:
        rb.style = BodyStyle::GasGiant;
        rb.atmosphereRim = std::max(rb.atmosphereRim, 0.45f);
        return;
    default:
        break;
    }
    const bool bluish = c[2] > c[0] + 0.15f && c[2] >= c[1];
    const bool pale = std::min(std::min(c[0], c[1]), c[2]) > 0.7f && c[2] >= c[0];
    if (body.kind == BodyKind::Planet && bluish) {
        rb.style = BodyStyle::Ocean;                 // a blue planet: oceans, continents and weather
        rb.cloudCover = 0.5f;
        rb.atmosphereRim = std::max(rb.atmosphereRim, 0.6f);
    } else {
        rb.style = pale ? BodyStyle::Icy : BodyStyle::Rocky;
        if (rho0 >= 3.0) rb.cloudCover = 0.95f;       // a Venus: a closed deck tinted like the body
        else if (rho0 >= 0.15) rb.cloudCover = 0.35f;
        else if (rho0 > 0.0) rb.cloudCover = 0.1f;    // a Mars: thin haze
    }
}

int pickDetailBody(const std::vector<DisplayBody>& bodies, const std::vector<RenderBody>& rendered, float metresPerPixel,
                   int viewportWidth, int viewportHeight, float minRadiusPx)
{
    const std::size_t n = std::min(bodies.size(), rendered.size());
    const double mpp = metresPerPixel > 0.0f ? metresPerPixel : 1.0;
    const double hw = 0.5 * viewportWidth * mpp, hh = 0.5 * viewportHeight * mpp;
    int best = -1;
    double bestMu = -1.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!isPlanetClass(bodies[i].kind)) continue;
        const RenderBody& r = rendered[i];
        const double radius = r.radius;
        if (radius / mpp < minRadiusPx) continue;
        if (std::abs(r.x) > hw + radius || std::abs(r.y) > hh + radius) continue;
        if (bodies[i].mu > bestMu) { bestMu = bodies[i].mu; best = static_cast<int>(i); }
    }
    return best;
}

void diskNormal(const std::vector<DisplayBody>& bodies, std::size_t hole, float out[3])
{
    out[0] = 0.0f; out[1] = 0.0f; out[2] = 1.0f;
    if (hole >= bodies.size()) return;
    const DisplayBody& h = bodies[hole];
    double lx = 0.0, ly = 0.0, lz = 0.0;
    for (std::size_t j = 0; j < bodies.size(); ++j) {
        if (j == hole) continue;
        const double rx = bodies[j].r.x - h.r.x, ry = bodies[j].r.y - h.r.y, rz = bodies[j].r.z - h.r.z;
        const double vx = bodies[j].v.x - h.v.x, vy = bodies[j].v.y - h.v.y, vz = bodies[j].v.z - h.v.z;
        const double r2 = rx * rx + ry * ry + rz * rz;
        if (!(r2 > 0.0)) continue;
        double cx = ry * vz - rz * vy, cy = rz * vx - rx * vz, cz = rx * vy - ry * vx;   // specific angular momentum
        const double cl = std::sqrt(cx * cx + cy * cy + cz * cz);
        if (!(cl > 0.0)) continue;
        const double w = bodies[j].mu / r2 / cl;   // unit direction, weighted by the pull of body j toward the hole
        lx += cx * w; ly += cy * w; lz += cz * w;
    }
    const double l = std::sqrt(lx * lx + ly * ly + lz * lz);
    if (!(l > 0.0)) return;
    out[0] = static_cast<float>(lx / l);
    out[1] = static_cast<float>(ly / l);
    out[2] = static_cast<float>(lz / l);
}

} // namespace app
} // namespace cosmos
