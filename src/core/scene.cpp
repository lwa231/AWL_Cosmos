#include "core/scene.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_set>
#include "core/kepler.h"
#include "core/placement.h"

namespace cosmos {

namespace {

const double kPi = 3.141592653589793238462643383279502884;
const double kDegToRad = kPi / 180.0;

// Largest jump in store IDs that resolveScene bridges with placeholder bodies to keep a scene ID
constexpr std::uint64_t kMaxIdGap = 10000;

bool finite(double v) { return std::isfinite(v); }
bool finite(const Vec3& v) { return finite(v.x) && finite(v.y) && finite(v.z); }

std::string quoted(const std::string& s) { return "\"" + s + "\""; }

std::string whoBody(const SceneBody& b)
{
    return "body " + quoted(b.name) + " (id " + std::to_string(b.id) + ")";
}

std::string whoShuttle(const SceneShuttle& s)
{
    return "shuttle " + quoted(s.name) + " (id " + std::to_string(s.id) + ")";
}

// Empty string when fine, otherwise what is wrong with the orbit itself (the parent is checked by the caller)
std::string orbitProblem(const SceneOrbit& o)
{
    if (!finite(o.a) || !finite(o.e) || !finite(o.argPeriapsis_deg) || !finite(o.meanAnomaly_deg))
        return "orbit has a non-finite value";
    if (!(o.a > 0.0)) return "orbit semi-major axis a must be > 0 (bound orbits only)";
    if (o.e < 0.0 || o.e >= 1.0) return "orbit eccentricity e must be in [0, 1) (bound orbits only)";
    return {};
}

Elements toElements(const SceneOrbit& o)
{
    Elements el;
    el.a = o.a;
    el.e = o.e;
    el.omega = o.argPeriapsis_deg * kDegToRad;
    el.M = o.meanAnomaly_deg * kDegToRad;
    el.retrograde = o.retrograde;
    return el;
}

} // namespace

int findSceneBody(const Scene& scene, std::uint64_t id)
{
    for (std::size_t i = 0; i < scene.bodies.size(); ++i)
        if (scene.bodies[i].id == id) return static_cast<int>(i);
    return -1;
}

bool validateScene(const Scene& scene, std::string& error)
{
    if (!finite(scene.epoch_s)) { error = "epoch_s is not finite"; return false; }
    if (!finite(scene.settings.controllerTick_s) || !(scene.settings.controllerTick_s > 0.0)) {
        error = "settings.controllerTick_s must be > 0";
        return false;
    }

    const std::size_t n = scene.bodies.size();
    std::unordered_set<std::uint64_t> used;

    for (const SceneBody& b : scene.bodies) {
        if (b.id == 0) { error = whoBody(b) + ": id must be nonzero"; return false; }
        if (!used.insert(b.id).second) { error = whoBody(b) + ": duplicate id"; return false; }
        if (!finite(b.mu) || !(b.mu > 0.0)) { error = whoBody(b) + ": mu must be > 0"; return false; }
        if (!finite(b.radius) || b.radius < 0.0) { error = whoBody(b) + ": radius must be >= 0"; return false; }
        if (!finite(b.spinPeriod_h)) { error = whoBody(b) + ": spinPeriod_h is not finite"; return false; }
        if (!finite(b.inertiaFactor) || b.inertiaFactor < 0.0) {
            error = whoBody(b) + ": inertiaFactor must be >= 0";
            return false;
        }
        if (b.atmosphere) {
            if (!finite(b.atmosphere->rho0) || b.atmosphere->rho0 < 0.0 ||
                !finite(b.atmosphere->scaleHeight) || !(b.atmosphere->scaleHeight > 0.0)) {
                error = whoBody(b) + ": atmosphere needs rho0 >= 0 and scaleHeight > 0";
                return false;
            }
        }
        for (float c : b.colour) {
            if (!std::isfinite(c)) { error = whoBody(b) + ": colour is not finite"; return false; }
        }
        if (b.orbit) {
            const std::string p = orbitProblem(*b.orbit);
            if (!p.empty()) { error = whoBody(b) + ": " + p; return false; }
        } else if (!finite(b.position) || !finite(b.velocity)) {
            error = whoBody(b) + ": position/velocity is not finite";
            return false;
        }
    }

    // Parents exist and the orbit graph has no cycle (every chain of parents must end at a body without an orbit)
    for (const SceneBody& b : scene.bodies) {
        if (!b.orbit) continue;
        if (b.orbit->parent == b.id) { error = whoBody(b) + ": orbits itself"; return false; }
        if (findSceneBody(scene, b.orbit->parent) < 0) {
            error = whoBody(b) + ": orbit parent id " + std::to_string(b.orbit->parent) + " does not exist";
            return false;
        }
        const SceneBody* cur = &b;
        std::size_t hops = 0;
        while (cur->orbit) {
            cur = &scene.bodies[static_cast<std::size_t>(findSceneBody(scene, cur->orbit->parent))];
            if (++hops > n) { error = whoBody(b) + ": orbit parents form a cycle"; return false; }
        }
    }

    std::unordered_set<std::uint64_t> shuttleIds;
    for (const SceneShuttle& s : scene.shuttles) {
        if (s.id == 0) { error = whoShuttle(s) + ": id must be nonzero"; return false; }
        if (used.count(s.id) != 0 || !shuttleIds.insert(s.id).second) {
            error = whoShuttle(s) + ": id is already used by another body or shuttle";
            return false;
        }
        const double fields[] = {s.dryMass, s.fuelMass, s.maxThrust, s.exhaustVelocity, s.cdA, s.radius, s.maxTorque, s.inertia};
        for (double f : fields) {
            if (!finite(f) || f < 0.0) { error = whoShuttle(s) + ": mass, thrust, drag, radius and inertia values must be finite and >= 0"; return false; }
        }
        if (s.orbit) {
            const std::string p = orbitProblem(*s.orbit);
            if (!p.empty()) { error = whoShuttle(s) + ": " + p; return false; }
            if (findSceneBody(scene, s.orbit->parent) < 0) {
                error = whoShuttle(s) + ": orbit parent id " + std::to_string(s.orbit->parent) + " is not a body";
                return false;
            }
        } else if (!finite(s.position) || !finite(s.velocity)) {
            error = whoShuttle(s) + ": position/velocity is not finite";
            return false;
        }
    }
    return true;
}

bool resolveScene(const Scene& scene, ResolvedScene& out, std::string& error, const ResolveOptions& opts)
{
    out = ResolvedScene{};
    if (!validateScene(scene, error)) return false;

    const std::size_t n = scene.bodies.size();

    // Parent and tree root of every body (indices into scene.bodies)
    std::vector<int> parent(n, -1), rootOf(n, -1);
    bool anyOrbit = false;
    for (std::size_t i = 0; i < n; ++i) {
        if (scene.bodies[i].orbit) {
            parent[i] = findSceneBody(scene, scene.bodies[i].orbit->parent);
            anyOrbit = true;
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t r = i;
        while (parent[r] >= 0) r = static_cast<std::size_t>(parent[r]);
        rootOf[i] = static_cast<int>(r);
    }

    // Per tree: Jacobi placement about the tree barycentre, then moved so the root body sits at its given state
    std::vector<Vec3> pos(n), vel(n);
    for (std::size_t r = 0; r < n; ++r) {
        if (parent[r] >= 0) continue;

        std::vector<std::size_t> members{r};
        std::vector<int> local(n, -1);
        local[r] = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (i != r && rootOf[i] == static_cast<int>(r)) {
                local[i] = static_cast<int>(members.size());
                members.push_back(i);
            }
        }

        std::vector<PlacementBody> pb(members.size());
        for (std::size_t k = 0; k < members.size(); ++k) {
            const SceneBody& b = scene.bodies[members[k]];
            pb[k].mu = b.mu;
            if (b.orbit) {
                pb[k].parent = local[static_cast<std::size_t>(parent[members[k]])];
                const Elements el = toElements(*b.orbit);
                pb[k].orbit = {el.a, el.e, el.omega, el.M, el.retrograde};
            }
        }
        const std::vector<Placed> placed = placeJacobi(pb);
        if (placed.size() != members.size()) {
            error = "Jacobi placement failed for the system of " + whoBody(scene.bodies[r]);
            return false;
        }

        const Vec3 shiftR = scene.bodies[r].position - placed[0].r;
        const Vec3 shiftV = scene.bodies[r].velocity - placed[0].v;
        for (std::size_t k = 0; k < members.size(); ++k) {
            pos[members[k]] = placed[k].r + shiftR;
            vel[members[k]] = placed[k].v + shiftV;
        }
    }

    // Shuttles: explicit state, or Kepler elements about the parent body's resolved state (massless, so mu = mu_parent)
    std::vector<Vec3> shPos(scene.shuttles.size()), shVel(scene.shuttles.size());
    for (std::size_t s = 0; s < scene.shuttles.size(); ++s) {
        const SceneShuttle& sh = scene.shuttles[s];
        if (sh.orbit) {
            const std::size_t p = static_cast<std::size_t>(findSceneBody(scene, sh.orbit->parent));
            const RelState rel = elementsToState(toElements(*sh.orbit), scene.bodies[p].mu);
            shPos[s] = pos[p] + rel.r;
            shVel[s] = vel[p] + rel.v;
        } else {
            shPos[s] = sh.position;
            shVel[s] = sh.velocity;
        }
    }

    // Store order is ascending scene ID (stable)
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t x, std::size_t y) { return scene.bodies[x].id < scene.bodies[y].id; });

    // Barycentre shift (plan 5.9 step 4), summed in store order
    Vec3 shiftR, shiftV;
    if (n > 0 && (opts.recentre == Recentre::Always || (opts.recentre == Recentre::Auto && anyOrbit))) {
        double total = 0.0;
        Vec3 r, v;
        for (std::size_t i : order) {
            total += scene.bodies[i].mu;
            r += scene.bodies[i].mu * pos[i];
            v += scene.bodies[i].mu * vel[i];
        }
        shiftR = r / total;
        shiftV = v / total;
    }

    out.bodyIds.assign(n, 0);
    BodyId next = 1;   // the next ID a fresh store will hand out
    for (std::size_t i : order) {
        const SceneBody& sb = scene.bodies[i];

        BodyDesc d;
        d.name = sb.name;
        d.kind = sb.kind;
        d.kindLocked = sb.kindLocked;
        d.fictionalDensity = sb.fictionalDensity;
        d.mu = sb.mu;
        d.radius = sb.radius;
        d.spinRate = sb.spinPeriod_h != 0.0 ? 2.0 * kPi / (sb.spinPeriod_h * 3600.0) : 0.0;
        d.inertiaFactor = sb.inertiaFactor;
        d.atmosphere = sb.atmosphere;
        d.colour = sb.colour;
        d.position = pos[i] - shiftR;
        d.velocity = vel[i] - shiftV;

        // Bridge to the wanted ID with placeholders that are removed again (store IDs are never reused)
        if (sb.id > next && sb.id - next <= kMaxIdGap) {
            while (next < sb.id) {
                next = out.bodies.add(BodyDesc{}) + 1;
                out.bodies.remove(next - 1);
            }
        }
        const BodyId got = out.bodies.add(d);
        next = got + 1;
        out.bodyIds[i] = got;
        if (got != sb.id) {
            out.idsPreserved = false;
            out.warnings.push_back(whoBody(sb) + " could not keep its id; the store gave it id " + std::to_string(got));
        }
    }

    out.shuttles.resize(scene.shuttles.size());
    for (std::size_t s = 0; s < scene.shuttles.size(); ++s) {
        out.shuttles[s].desc = scene.shuttles[s];
        out.shuttles[s].position = shPos[s] - shiftR;
        out.shuttles[s].velocity = shVel[s] - shiftV;
    }
    return true;
}

} // namespace cosmos
