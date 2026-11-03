#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "core/bodies.h"

namespace cosmos {

// Body kinds, defaults and merge reclassification (plan sections 12.1 and 12.2).

inline constexpr double kSolarMu = 1.32712440018e20;               // GM of the Sun, m^3/s^2
inline constexpr double kSolarRadius = 6.957e8;                    // m (IAU nominal)
inline constexpr double kNeutronStarMaxMu = 2.3 * kSolarMu;        // above this a neutron star is a Black Hole

// Display name of a kind: "Star", "Planet", "Gas Giant", "Dwarf Planet", "Moon", "Asteroid", "Neutron Star", "Black Hole".
const char* kindName(BodyKind kind);

// Default parameters for a new body of the given kind (table 12.1): mu, radius, colour, k = 0.4, no spin, no
// atmosphere, kind set, kindLocked false, name empty. Masses given in kg in the table become mu = G * kg; the Star
// and the compact objects use the solar mu directly. A Black Hole gets R = 2 mu / c^2.
// Dwarf Planet is a Lucerna display kind and not in the table; it gets Ceres-like values.
BodyDesc defaultBodyDesc(BodyKind kind);

// Generic names "<Kind> <n>" (plan 12.2). One counter per kind, starting at 1, only ever incremented, so a number
// is never reused even after the body is deleted or renamed. Template bodies are named explicitly and never call
// next(), so they consume no counter.
class NameCounter {
public:
    // Returns "<Kind> <n>" and advances that kind's counter.
    std::string next(BodyKind kind);

    // Last number handed out for the kind (0 = none yet). For saving and restoring scene state.
    std::uint64_t issued(BodyKind kind) const { return counts[static_cast<std::size_t>(kind)]; }
    void setIssued(BodyKind kind, std::uint64_t n) { counts[static_cast<std::size_t>(kind)] = n; }

private:
    std::array<std::uint64_t, 8> counts{};
};

// defaultBodyDesc(kind) with name = counter.next(kind).
BodyDesc newBodyDesc(BodyKind kind, NameCounter& counter);

// Kind of the body that results from merging (plan 12.2), upward only. `heavier` is the survivor's kind,
// `muMerged` the combined mu:
//   survivor kindLocked                            -> heavier (never reclassified)
//   either body a Black Hole                       -> Black Hole
//   result a Neutron Star with mu > 2.3 solar      -> Black Hole
//   otherwise                                      -> heavier
BodyKind mergedKind(BodyKind heavier, bool heavierLocked, BodyKind lighter, double muMerged);

} // namespace cosmos
