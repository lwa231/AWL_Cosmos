#include "app/run_control.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include "app/presets.h"

namespace cosmos {
namespace app {

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
}

const char* runStateName(RunState s)
{
    switch (s) {
    case RunState::Stopped: return "Stopped";
    case RunState::Running: return "Running";
    case RunState::Paused: return "Paused";
    }
    return "";
}

Scene captureScene(const Simulation& sim, const std::string& name, const std::string& units)
{
    Scene s;
    s.name = name;
    s.units = units;
    s.epoch_s = sim.time();
    s.settings.infiniteFuel = sim.infiniteFuel();
    s.settings.controllerTick_s = sim.controllerTick();

    const Bodies& b = sim.bodies();
    s.bodies.reserve(b.size());
    for (std::size_t i = 0; i < b.size(); ++i) {
        SceneBody o;
        o.id = b.ids[i];
        o.name = b.name[i];
        o.kind = b.kind[i];
        o.kindLocked = b.kindLocked[i] != 0;
        o.fictionalDensity = b.fictionalDensity[i] != 0;
        o.mu = b.mu[i];
        o.radius = b.radius[i];
        o.spinPeriod_h = b.spinRate[i] != 0.0 ? 2.0 * kPi / (b.spinRate[i] * 3600.0) : 0.0;
        o.inertiaFactor = b.inertiaFactor[i];
        o.atmosphere = b.atmosphere[i];
        o.colour = b.colour[i];
        o.position = {b.state.x[i], b.state.y[i], b.state.z[i]};
        o.velocity = {b.state.vx[i], b.state.vy[i], b.state.vz[i]};
        s.bodies.push_back(std::move(o));
    }

    const Shuttles& sh = sim.shuttles();
    for (std::size_t i = 0; i < sh.size(); ++i) {
        if (sh.alive[i] == 0) continue;   // destroyed shuttles are not kept
        SceneShuttle o;
        o.id = sh.ids[i];
        o.name = sh.name[i];
        o.controller = controllerName(sh.controller[i]);
        o.position = {sh.state.x[i], sh.state.y[i], sh.state.z[i]};
        o.velocity = {sh.state.vx[i], sh.state.vy[i], sh.state.vz[i]};
        o.dryMass = sh.dryMass[i];
        o.fuelMass = sh.fuelMass[i];
        o.maxThrust = sh.maxThrust[i];
        o.exhaustVelocity = sh.exhaustVelocity[i];
        o.cdA = sh.cdA[i];
        o.radius = sh.radius[i];
        o.maxTorque = sh.maxTorque[i];
        o.inertia = sh.inertia[i];
        s.shuttles.push_back(std::move(o));
    }
    return s;
}

std::unique_ptr<SimDriver> buildDriver(const Scene& scene, const ResolveOptions& resolve, SimulationOptions sim,
                                       const DriverOptions& driver, std::string& error, std::vector<std::string>* warnings)
{
    ResolvedScene res;
    if (!resolveScene(scene, res, error, resolve)) return nullptr;
    sim.startTime = scene.epoch_s;
    std::unique_ptr<SimDriver> d;
    try {
        d = std::make_unique<SimDriver>(res, scene.settings, sim, driver);
    } catch (const std::exception& e) {
        error = std::string("cannot build the simulation: ") + e.what();
        return nullptr;
    }
    if (warnings) {
        *warnings = res.warnings;
        for (const std::string& w : d->sim().warnings()) warnings->push_back(w);
    }
    return d;
}

// ---- RunControl --------------------------------------------------------------------------------------------------------

RunControl::RunControl(const RunOptions& options) : options_(options)
{
    editor_.setScene(emptyScene());
    std::string error;
    driver_ = buildDriver(editor_.scene(), editor_.resolveOptions(), options_.sim, options_.driver, error, &buildWarnings_);
    builtRevision_ = editor_.revision();
    generation_ = 1;
    driver_->setPaused(true);
}

bool RunControl::rebuild(const Scene& scene, const ResolveOptions& resolve, std::string& error)
{
    std::vector<std::string> warnings;
    std::unique_ptr<SimDriver> d = buildDriver(scene, resolve, options_.sim, options_.driver, error, &warnings);
    if (!d) return false;
    if (driver_) d->warp() = driver_->warp();   // the warp setting (and its Resume memory) survives rebuilds
    d->setPaused(true);
    driver_ = std::move(d);
    buildWarnings_ = std::move(warnings);
    ++generation_;
    return true;
}

std::uint64_t RunControl::highestId() const
{
    std::uint64_t top = 0;
    const Simulation& sim = driver_->sim();
    for (std::uint64_t id : sim.bodies().ids) top = std::max(top, id);
    for (std::uint64_t id : sim.shuttles().ids) top = std::max(top, id);   // dead shuttles stay in the store
    return top;
}

bool RunControl::loadScene(Scene scene, std::string& error)
{
    ResolveOptions autoResolve;
    autoResolve.recentre = Recentre::Auto;
    std::vector<std::string> warnings;
    std::unique_ptr<SimDriver> d = buildDriver(scene, autoResolve, options_.sim, options_.driver, error, &warnings);
    if (!d) {
        lastError_ = error;
        return false;
    }
    if (driver_) {
        d->warp() = driver_->warp();
        d->warp().clearResume();
    }
    d->setPaused(true);
    driver_ = std::move(d);
    buildWarnings_ = std::move(warnings);
    ++generation_;

    editor_.setLocked(false);
    editor_.setScene(std::move(scene));
    builtRevision_ = editor_.revision();
    state_ = RunState::Stopped;
    haveSnapshot_ = false;
    snapshot_ = Scene{};
    lastError_.clear();
    return true;
}

EditResult RunControl::applyEdits()
{
    if (state_ == RunState::Running) return EditResult::failure(kLockedMessage);
    if (!dirty()) return EditResult::success();
    std::string error;
    if (!rebuild(editor_.scene(), editor_.resolveOptions(), error)) {
        lastError_ = error;
        return EditResult::failure(error);
    }
    builtRevision_ = editor_.revision();
    lastError_.clear();
    EditResult r = EditResult::success();
    r.warnings = buildWarnings_;
    return r;
}

EditResult RunControl::start()
{
    if (state_ == RunState::Running) return EditResult::success();
    lastError_.clear();

    if (state_ == RunState::Stopped) {
        if (editor_.scene().bodies.empty()) {
            lastError_ = "Add at least one body before pressing Start.";
            return EditResult::failure(lastError_);
        }
        std::string error;
        if (!rebuild(editor_.scene(), editor_.resolveOptions(), error)) {
            lastError_ = error;
            return EditResult::failure(error);
        }
        builtRevision_ = editor_.revision();
        snapshot_ = editor_.scene();
        haveSnapshot_ = true;
    } else {
        EditResult r = applyEdits();   // no-op without edits: the same driver continues
        if (!r) return r;
    }

    driver_->setPaused(false);
    editor_.setLocked(true);
    state_ = RunState::Running;
    return EditResult::success();
}

EditResult RunControl::pause()
{
    if (state_ != RunState::Running) return EditResult::failure("The simulation is not running.");
    lastError_.clear();
    driver_->setPaused(true);

    const std::string name = haveSnapshot_ ? snapshot_.name : editor_.scene().name;
    Scene captured = captureScene(driver_->sim(), name);
    const std::uint64_t top = highestId();
    editor_.setRecentre(Recentre::Never);
    editor_.restoreScene(std::move(captured), top);
    editor_.setLocked(false);
    builtRevision_ = editor_.revision();   // the driver IS this scene: nothing to rebuild until the user edits
    state_ = RunState::Paused;
    return EditResult::success();
}

EditResult RunControl::stop()
{
    if (state_ == RunState::Stopped) return EditResult::success();
    lastError_.clear();
    if (!haveSnapshot_) {   // cannot happen (Start takes it); keep the state machine total
        state_ = RunState::Stopped;
        editor_.setLocked(false);
        return EditResult::success();
    }

    const std::uint64_t top = highestId();
    std::string error;
    ResolveOptions autoResolve;
    autoResolve.recentre = Recentre::Auto;
    if (!rebuild(snapshot_, autoResolve, error)) {
        lastError_ = error;
        return EditResult::failure(error);
    }
    driver_->warp().clearResume();
    editor_.setLocked(false);
    editor_.setRecentre(Recentre::Auto);
    editor_.restoreScene(snapshot_, top);
    builtRevision_ = editor_.revision();
    state_ = RunState::Stopped;
    return EditResult::success();
}

bool RunControl::advance(double frameDt)
{
    if (state_ != RunState::Running) {
        if (dirty()) applyEdits();
        return false;
    }
    driver_->advance(frameDt);
    if (driver_->frameInfo().failed) {
        pause();
        lastError_ = "The integrator failed; the simulation was paused where it stopped.";
    }
    return true;
}

bool RunControl::captureCurrent(Scene& out, const std::string& name)
{
    if (state_ == RunState::Stopped) {
        out = editor_.scene();
        if (!name.empty()) out.name = name;
        return true;
    }
    if (state_ == RunState::Paused) {
        const EditResult r = applyEdits();
        if (!r) return false;
    }
    const std::string base = haveSnapshot_ ? snapshot_.name : editor_.scene().name;
    out = captureScene(driver_->sim(), name.empty() ? base + " (saved)" : name);
    return true;
}

bool RunControl::saveCurrent(const std::string& path, std::string& error, const std::string& name)
{
    Scene s;
    if (!captureCurrent(s, name)) {
        error = lastError_.empty() ? "cannot capture the current state" : lastError_;
        return false;
    }
    return saveSceneToPath(path, s, error);
}

std::vector<ObjectRow> RunControl::listObjects() const
{
    if (state_ != RunState::Running) return editor_.objects();
    std::vector<ObjectRow> rows;
    const Simulation& sim = driver_->sim();
    const Bodies& b = sim.bodies();
    for (std::size_t i = 0; i < b.size(); ++i) {
        ObjectRow r;
        r.id = b.ids[i];
        r.type = ObjectType::Body;
        r.name = b.name[i];
        r.kind = b.kind[i];
        rows.push_back(std::move(r));
    }
    const Shuttles& s = sim.shuttles();
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s.alive[i] == 0) continue;
        ObjectRow r;
        r.id = s.ids[i];
        r.type = ObjectType::Shuttle;
        r.name = s.name[i];
        r.controller = controllerName(s.controller[i]);
        rows.push_back(std::move(r));
    }
    return rows;
}

} // namespace app
} // namespace cosmos
