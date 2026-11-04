#include "app/overlay_predict.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <thread>

namespace cosmos {
namespace app {

namespace {

constexpr int kChunksPerSample = 4;   // advanceTo calls per sample: how often a superseded job notices (never changes a result)

Vec3 nanPoint() { return {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}; }

// Position of body `id` at tk: the dense output of the latest step when tk lies in it, else the store.
bool bodyAt(const Simulation& sim, std::uint64_t id, double tk, Vec3& out)
{
    if (sim.hasBodyStep() && tk >= sim.bodyStepStart() && tk <= sim.bodyStepEnd()) return sim.bodyPositionAt(id, tk, out);
    const auto i = sim.bodies().find(id);
    if (!i) return false;
    out = {sim.bodies().state.x[*i], sim.bodies().state.y[*i], sim.bodies().state.z[*i]};
    return true;
}

} // namespace

int PredictedPath::bodyColumn(std::uint64_t bodyId) const
{
    for (std::size_t j = 0; j < bodyIds.size(); ++j)
        if (bodyIds[j] == bodyId) return static_cast<int>(j);
    return -1;
}

bool makeSnapshot(const Simulation& sim, TargetKind kind, std::uint64_t id, PredictSnapshot& out)
{
    PredictSnapshot s;
    s.t0 = sim.time();
    s.bodies = sim.bodies();
    s.settings.infiniteFuel = sim.infiniteFuel();
    s.settings.controllerTick_s = sim.controllerTick();
    if (kind == TargetKind::Body) {
        if (!s.bodies.find(id)) return false;
    } else if (kind == TargetKind::Shuttle) {
        const Shuttles& sh = sim.shuttles();
        const auto found = sh.find(id);
        if (!found || sh.alive[*found] == 0) return false;
        const std::size_t i = *found;
        ShuttleDesc d;
        d.id = sh.ids[i];
        d.name = sh.name[i];
        d.controller = sh.controller[i];
        d.position = {sh.state.x[i], sh.state.y[i], sh.state.z[i]};
        d.velocity = {sh.state.vx[i], sh.state.vy[i], sh.state.vz[i]};
        d.heading = sh.heading[i];
        d.spin = sh.spin[i];
        d.dryMass = sh.dryMass[i];
        d.fuelMass = sh.fuelMass[i];
        d.maxThrust = sh.maxThrust[i];
        d.exhaustVelocity = sh.exhaustVelocity[i];
        d.cdA = sh.cdA[i];
        d.radius = sh.radius[i];
        d.maxTorque = sh.maxTorque[i];
        d.inertia = sh.inertia[i];
        s.hasShuttle = true;
        s.shuttle = d;
    } else {
        return false;
    }
    out = std::move(s);
    return true;
}

void runPrediction(const PredictSnapshot& snap, const PredictRequest& req, PredictedPath& out, bool (*abort)(const void*),
                   const void* abortArg)
{
    out = PredictedPath{};
    out.kind = req.kind;
    out.id = req.id;
    out.t0 = snap.t0;
    const int n = std::max(2, req.samples);
    if (!(req.horizon_s > 0.0) || !std::isfinite(req.horizon_s)) return;
    out.dt = req.horizon_s / n;

    ResolvedScene scene;
    scene.bodies = snap.bodies;
    scene.bodyIds = snap.bodies.ids;
    SimulationOptions opt = req.sim;
    opt.threads = 1;
    opt.startTime = snap.t0;
    opt.recordShuttlePaths = true;
    Simulation sim(scene, snap.settings, opt);
    if (req.kind == TargetKind::Shuttle && (!snap.hasShuttle || sim.addShuttle(snap.shuttle) == 0)) return;
    const std::optional<std::size_t> shuttleIdx = req.kind == TargetKind::Shuttle ? sim.shuttles().find(req.id)
                                                                                  : std::nullopt;

    out.bodyIds = snap.bodies.ids;
    const std::size_t nb = out.bodyIds.size();
    out.target.reserve(static_cast<std::size_t>(n) + 1);
    out.bodies.reserve((static_cast<std::size_t>(n) + 1) * nb);
    out.barycentre.reserve(static_cast<std::size_t>(n) + 1);

    for (int k = 0; k <= n; ++k) {
        const double tk = snap.t0 + out.dt * k;
        if (k > 0) {
            const double tPrev = snap.t0 + out.dt * (k - 1);
            bool ok = true;
            for (int c = 1; c <= kChunksPerSample && ok; ++c) {
                if (abort && abort(abortArg)) {
                    out.truncated = true;
                    return;
                }
                if (static_cast<std::size_t>(sim.stats().bodySteps) > req.maxBodySteps) {
                    out.truncated = true;
                    return;
                }
                ok = sim.advanceTo(c == kChunksPerSample ? tk : tPrev + (tk - tPrev) * c / kChunksPerSample);
            }
            if (!ok) {
                out.truncated = true;
                return;
            }
        }

        // The target.
        Vec3 pt;
        bool ended = false;
        if (req.kind == TargetKind::Body) {
            if (!bodyAt(sim, req.id, tk, pt)) {
                out.targetEnded = true;   // absorbed: the previous sample was its last
                return;
            }
        } else {
            const std::size_t i = *shuttleIdx;
            pt = sim.shuttlePositionAt(i, tk);   // clamped to the contact point for a shuttle that died
            if (sim.shuttles().alive[i] == 0) {
                // A body step can span many samples: the shuttle is dead in the simulation, but only ended at the sample
                // times from its contact on.
                const std::vector<ShuttleSample>& path = sim.shuttlePath(i);
                const double deathT = path.empty() ? sim.shuttleStep(i).to.t : path.back().t;
                ended = tk >= deathT;
            }
        }

        // Every body, and the barycentre of those still present.
        double m = 0.0, bx = 0.0, by = 0.0;
        for (std::size_t j = 0; j < nb; ++j) {
            Vec3 p;
            if (bodyAt(sim, out.bodyIds[j], tk, p) && sim.bodies().find(out.bodyIds[j])) {
                out.bodies.push_back(p);
                const double mu = sim.bodies().mu[*sim.bodies().find(out.bodyIds[j])];
                m += mu;
                bx += mu * p.x;
                by += mu * p.y;
            } else {
                out.bodies.push_back(nanPoint());
            }
        }
        out.target.push_back(pt);
        out.barycentre.push_back(m > 0.0 ? Vec3{bx / m, by / m, 0.0} : Vec3{});
        out.count = out.target.size();
        if (ended) {
            out.targetEnded = true;
            return;
        }
    }
}

PredictedPath predictNow(const Simulation& sim, const PredictRequest& req)
{
    PredictedPath out;
    PredictSnapshot snap;
    if (makeSnapshot(sim, req.kind, req.id, snap)) runPrediction(snap, req, out);
    out.requestId = 1;
    return out;
}

// ---------------------------------------------------------------------------------------------------------------
// The worker

struct PathPredictor::Impl {
    struct Job {
        std::uint64_t serial = 0;
        PredictSnapshot snap;
        PredictRequest req;
    };

    std::mutex m;
    std::condition_variable cv;        // wakes the worker
    std::condition_variable idleCv;    // wakes waitIdle
    std::thread worker;
    std::optional<Job> pending;
    bool running = false;
    bool stop = false;
    std::atomic<std::uint64_t> latest{0};   // number of the newest request: a job with a smaller one is stale
    PredictedPath ready;
    bool hasReady = false;

    struct AbortCtx {
        const std::atomic<std::uint64_t>* latest;
        std::uint64_t mine;
    };
    static bool shouldAbort(const void* p)
    {
        const AbortCtx* c = static_cast<const AbortCtx*>(p);
        return c->latest->load(std::memory_order_relaxed) != c->mine;
    }

    void loop()
    {
        std::unique_lock<std::mutex> lock(m);
        for (;;) {
            cv.wait(lock, [&] { return stop || pending.has_value(); });
            if (stop) return;
            Job job = std::move(*pending);
            pending.reset();
            running = true;
            lock.unlock();

            PredictedPath path;
            const AbortCtx ctx{&latest, job.serial};
            runPrediction(job.snap, job.req, path, &Impl::shouldAbort, &ctx);
            path.requestId = job.serial;

            lock.lock();
            running = false;
            if (job.serial == latest.load()) {
                // A superseded job is dropped; a current one - even a truncated one - is handed over.
                ready = std::move(path);
                hasReady = true;
            }
            idleCv.notify_all();
        }
    }
};

PathPredictor::PathPredictor() : impl_(new Impl) {}

PathPredictor::~PathPredictor()
{
    {
        std::lock_guard<std::mutex> lock(impl_->m);
        impl_->stop = true;
        impl_->latest.store(~std::uint64_t(0));   // abort a running job at its next chunk
    }
    impl_->cv.notify_all();
    if (impl_->worker.joinable()) impl_->worker.join();
}

std::uint64_t PathPredictor::request(const Simulation& sim, const PredictRequest& req)
{
    Impl::Job job;
    if (!makeSnapshot(sim, req.kind, req.id, job.snap)) return 0;
    job.req = req;
    {
        std::lock_guard<std::mutex> lock(impl_->m);
        job.serial = impl_->latest.load() + 1;
        impl_->latest.store(job.serial);        // the running job (if any) sees it is stale
        impl_->pending = std::move(job);
        impl_->hasReady = false;                // a result of an older request is not wanted any more
        if (!impl_->worker.joinable()) impl_->worker = std::thread([this] { impl_->loop(); });
    }
    impl_->cv.notify_one();
    return impl_->latest.load();
}

bool PathPredictor::poll()
{
    std::unique_lock<std::mutex> lock(impl_->m, std::try_to_lock);
    if (!lock.owns_lock() || !impl_->hasReady) return false;
    std::swap(latest_, impl_->ready);
    impl_->hasReady = false;
    return true;
}

std::uint64_t PathPredictor::latestRequest() const
{
    const std::uint64_t v = impl_->latest.load();
    return v == ~std::uint64_t(0) ? 0 : v;
}

bool PathPredictor::busy() const
{
    std::lock_guard<std::mutex> lock(impl_->m);
    return impl_->running || impl_->pending.has_value();
}

void PathPredictor::cancel()
{
    {
        std::lock_guard<std::mutex> lock(impl_->m);
        impl_->latest.store(impl_->latest.load() + 1);   // nothing running or queued matches any more
        impl_->pending.reset();
        impl_->hasReady = false;
    }
    latest_ = PredictedPath{};
}

bool PathPredictor::waitIdle(double timeoutSeconds)
{
    std::unique_lock<std::mutex> lock(impl_->m);
    return impl_->idleCv.wait_for(lock, std::chrono::duration<double>(timeoutSeconds),
                                  [&] { return !impl_->running && !impl_->pending.has_value(); });
}

} // namespace app
} // namespace cosmos
