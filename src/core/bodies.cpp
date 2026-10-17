#include "core/bodies.h"

namespace cosmos {

namespace {
template <typename T>
void eraseAt(std::vector<T>& v, std::size_t i)
{
    v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
}
}

BodyId Bodies::add(const BodyDesc& d)
{
    const BodyId id = nextId++;
    index[id] = ids.size();

    ids.push_back(id);
    name.push_back(d.name);
    kind.push_back(d.kind);
    kindLocked.push_back(d.kindLocked ? 1 : 0);
    fictionalDensity.push_back(d.fictionalDensity ? 1 : 0);
    mu.push_back(d.mu);
    radius.push_back(d.radius);
    spinRate.push_back(d.spinRate);
    inertiaFactor.push_back(d.inertiaFactor);
    atmosphere.push_back(d.atmosphere);
    colour.push_back(d.colour);

    state.x.push_back(d.position.x);
    state.y.push_back(d.position.y);
    state.z.push_back(d.position.z);
    state.vx.push_back(d.velocity.x);
    state.vy.push_back(d.velocity.y);
    state.vz.push_back(d.velocity.z);
    return id;
}

bool Bodies::remove(BodyId id)
{
    auto it = index.find(id);
    if (it == index.end()) return false;
    const std::size_t i = it->second;
    index.erase(it);

    eraseAt(ids, i);
    eraseAt(name, i);
    eraseAt(kind, i);
    eraseAt(kindLocked, i);
    eraseAt(fictionalDensity, i);
    eraseAt(mu, i);
    eraseAt(radius, i);
    eraseAt(spinRate, i);
    eraseAt(inertiaFactor, i);
    eraseAt(atmosphere, i);
    eraseAt(colour, i);

    eraseAt(state.x, i);
    eraseAt(state.y, i);
    eraseAt(state.z, i);
    eraseAt(state.vx, i);
    eraseAt(state.vy, i);
    eraseAt(state.vz, i);

    for (std::size_t j = i; j < ids.size(); ++j) index[ids[j]] = j;
    return true;
}

std::optional<std::size_t> Bodies::find(BodyId id) const
{
    auto it = index.find(id);
    if (it == index.end()) return std::nullopt;
    return it->second;
}

bool Bodies::consistent() const
{
    const std::size_t n = ids.size();
    if (name.size() != n || kind.size() != n || kindLocked.size() != n ||
        fictionalDensity.size() != n || mu.size() != n || radius.size() != n ||
        spinRate.size() != n || inertiaFactor.size() != n || atmosphere.size() != n ||
        colour.size() != n) return false;
    if (state.x.size() != n || state.y.size() != n || state.z.size() != n ||
        state.vx.size() != n || state.vy.size() != n || state.vz.size() != n) return false;
    if (index.size() != n) return false;
    for (std::size_t j = 0; j < n; ++j) {
        auto it = index.find(ids[j]);
        if (it == index.end() || it->second != j) return false;
    }
    return true;
}

} // namespace cosmos
