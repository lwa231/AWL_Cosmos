#include "core/kinds.h"
#include "core/constants.h"

namespace cosmos {

const char* kindName(BodyKind kind)
{
    switch (kind) {
    case BodyKind::Star: return "Star";
    case BodyKind::Planet: return "Planet";
    case BodyKind::GasGiant: return "Gas Giant";
    case BodyKind::DwarfPlanet: return "Dwarf Planet";
    case BodyKind::Moon: return "Moon";
    case BodyKind::Asteroid: return "Asteroid";
    case BodyKind::NeutronStar: return "Neutron Star";
    case BodyKind::BlackHole: return "Black Hole";
    }
    return "Unknown";
}

BodyDesc defaultBodyDesc(BodyKind kind)
{
    BodyDesc d;
    d.kind = kind;
    switch (kind) {
    case BodyKind::Star:
        d.mu = kSolarMu;
        d.radius = kSolarRadius;
        d.colour = {1.0f, 0.92f, 0.65f};
        break;
    case BodyKind::Planet: // Earth
        d.mu = 5.972e24 * constants::G;
        d.radius = 6.371e6;
        d.colour = {0.30f, 0.50f, 0.85f};
        break;
    case BodyKind::GasGiant: // Jupiter
        d.mu = 1.898e27 * constants::G;
        d.radius = 7.1492e7;
        d.colour = {0.85f, 0.70f, 0.50f};
        break;
    case BodyKind::DwarfPlanet: // Ceres-like, not in table 12.1
        d.mu = 9.38e20 * constants::G;
        d.radius = 4.73e5;
        d.colour = {0.70f, 0.65f, 0.60f};
        break;
    case BodyKind::Moon: // the Moon
        d.mu = 7.35e22 * constants::G;
        d.radius = 1.737e6;
        d.colour = {0.72f, 0.72f, 0.72f};
        break;
    case BodyKind::Asteroid:
        d.mu = 1.0e15 * constants::G;
        d.radius = 5.0e3;
        d.colour = {0.50f, 0.45f, 0.40f};
        break;
    case BodyKind::NeutronStar:
        d.mu = 1.4 * kSolarMu;
        d.radius = 1.2e4;
        d.colour = {0.70f, 0.85f, 1.00f};
        break;
    case BodyKind::BlackHole:
        d.mu = 10.0 * kSolarMu;
        d.radius = 2.0 * d.mu / (constants::c * constants::c);
        d.colour = {0.05f, 0.05f, 0.08f};
        break;
    }
    return d;
}

std::string NameCounter::next(BodyKind kind)
{
    std::uint64_t& n = counts[static_cast<std::size_t>(kind)];
    ++n;
    return std::string(kindName(kind)) + " " + std::to_string(n);
}

BodyDesc newBodyDesc(BodyKind kind, NameCounter& counter)
{
    BodyDesc d = defaultBodyDesc(kind);
    d.name = counter.next(kind);
    return d;
}

BodyKind mergedKind(BodyKind heavier, bool heavierLocked, BodyKind lighter, double muMerged)
{
    if (heavierLocked) return heavier;
    if (heavier == BodyKind::BlackHole || lighter == BodyKind::BlackHole) return BodyKind::BlackHole;
    if (heavier == BodyKind::NeutronStar && muMerged > kNeutronStarMaxMu) return BodyKind::BlackHole;
    return heavier;
}

} // namespace cosmos
