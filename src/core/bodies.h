#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include "core/vec3.h"

namespace cosmos {

using BodyId = std::uint64_t;

enum class BodyKind { Star, Planet, GasGiant, DwarfPlanet, Moon, Asteroid, NeutronStar, BlackHole };

struct Atmosphere {
    double rho0 = 0.0;
    double scaleHeight = 0.0;
};

struct BodyDesc {
    std::string name;
    BodyKind kind = BodyKind::Planet;
    bool kindLocked = false;
    bool fictionalDensity = false;
    double mu = 0.0;
    double radius = 0.0;
    double spinRate = 0.0;
    double inertiaFactor = 0.4;
    std::optional<Atmosphere> atmosphere;
    std::array<float, 3> colour = {1.0f, 1.0f, 1.0f};
    Vec3 position;
    Vec3 velocity;
};

struct BodyState {
    std::vector<double> x, y, z, vx, vy, vz;
};

class Bodies {
public:
    BodyId add(const BodyDesc& d);
    bool remove(BodyId id);
    std::optional<std::size_t> find(BodyId id) const;
    std::size_t size() const { return ids.size(); }
    bool consistent() const;

    BodyState state;
    std::vector<BodyId> ids;
    std::vector<std::string> name;
    std::vector<BodyKind> kind;
    std::vector<std::uint8_t> kindLocked;
    std::vector<std::uint8_t> fictionalDensity;
    std::vector<double> mu;
    std::vector<double> radius;
    std::vector<double> spinRate;
    std::vector<double> inertiaFactor;
    std::vector<std::optional<Atmosphere>> atmosphere;
    std::vector<std::array<float, 3>> colour;

private:
    BodyId nextId = 1;
    std::unordered_map<BodyId, std::size_t> index;
};

} // namespace cosmos
