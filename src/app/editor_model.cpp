#include "app/editor_model.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <utility>
#include "core/constants.h"

namespace cosmos {
namespace app {

namespace {

constexpr double kDominanceFraction = 0.10;   // plan 10.1
constexpr std::size_t kMaxNameLength = 64;
// Significant digits of the text in a field. Four keep a default Earth at "1 M_earth" and a radius at "6371 km"; the stored
// value is not rounded by showing it (an untouched field is never re-applied, see setFieldText).
constexpr int kFieldSig = 4;

std::string fq(double value, Quantity q, const char* unit = nullptr) { return formatQuantity(value, q, unit, kFieldSig); }

std::string trimmed(std::string_view s)
{
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return std::string(s.substr(b, e - b));
}

bool finite(double v) { return std::isfinite(v); }

std::string quoted(const std::string& s) { return "\"" + s + "\""; }

// n of "<prefix><n>" when the rest is 1..9 digits, else 0
std::uint64_t genericNumber(const std::string& name, const std::string& prefix)
{
    if (name.size() <= prefix.size() || name.compare(0, prefix.size(), prefix) != 0) return 0;
    const std::size_t digits = name.size() - prefix.size();
    if (digits > 9) return 0;
    std::uint64_t n = 0;
    for (std::size_t i = prefix.size(); i < name.size(); ++i) {
        if (name[i] < '0' || name[i] > '9') return 0;
        n = n * 10 + static_cast<std::uint64_t>(name[i] - '0');
    }
    return n;
}

bool checkName(const std::string& name, std::string& error)
{
    if (name.empty()) { error = "The name cannot be empty."; return false; }
    if (name.size() > kMaxNameLength) { error = "The name is too long (at most 64 characters)."; return false; }
    for (unsigned char c : name) {
        if (c < 0x20 || c == 0x7F) { error = "The name contains a control character."; return false; }
    }
    return true;
}

bool checkBody(const SceneBody& b, std::string& error)
{
    if (!checkName(b.name, error)) return false;
    if (!finite(b.mu) || !(b.mu > 0.0)) { error = "The mass must be greater than zero."; return false; }
    if (!finite(b.radius) || !(b.radius > 0.0)) { error = "The radius must be greater than zero."; return false; }
    if (!finite(b.spinPeriod_h)) { error = "The spin period must be a finite number."; return false; }
    for (float c : b.colour) {
        if (!std::isfinite(c) || c < 0.0f || c > 1.0f) { error = "Each colour component must be between 0 and 1."; return false; }
    }
    if (b.atmosphere) {
        if (!finite(b.atmosphere->rho0) || b.atmosphere->rho0 < 0.0) { error = "The atmosphere density rho0 cannot be negative."; return false; }
        if (!finite(b.atmosphere->scaleHeight) || !(b.atmosphere->scaleHeight > 0.0)) {
            error = "The atmosphere scale height must be greater than zero.";
            return false;
        }
    }
    return true;
}

bool checkShuttle(const SceneShuttle& s, std::string& error)
{
    if (!checkName(s.name, error)) return false;
    ControllerKind k;
    if (!parseControllerKind(s.controller, k)) { error = "Unknown controller " + quoted(s.controller) + "."; return false; }
    if (!finite(s.dryMass) || !(s.dryMass > 0.0)) { error = "The dry mass must be greater than zero."; return false; }
    if (!finite(s.fuelMass) || s.fuelMass < 0.0) { error = "The fuel mass cannot be negative."; return false; }
    if (!finite(s.maxThrust) || s.maxThrust < 0.0) { error = "The maximum thrust cannot be negative."; return false; }
    if (!finite(s.exhaustVelocity) || s.exhaustVelocity < 0.0) { error = "The exhaust velocity cannot be negative."; return false; }
    if (!finite(s.cdA) || s.cdA < 0.0) { error = "Cd*A cannot be negative."; return false; }
    if (!finite(s.radius) || !(s.radius > 0.0)) { error = "The collision radius must be greater than zero."; return false; }
    if (!finite(s.maxTorque) || s.maxTorque < 0.0) { error = "The maximum torque cannot be negative."; return false; }
    if (!finite(s.inertia) || !(s.inertia > 0.0)) { error = "The moment of inertia must be greater than zero."; return false; }
    return true;
}

bool finiteVec(const Vec3& v) { return finite(v.x) && finite(v.y) && finite(v.z); }

std::string lengthText(double m) { return formatQuantity(m, Quantity::Length, nullptr, 3); }

// State of a scene body or shuttle in a resolved scene
bool stateIn(const Scene& scene, const ResolvedScene& res, std::uint64_t id, Vec3& position, Vec3& velocity)
{
    const int bi = findSceneBody(scene, id);
    if (bi >= 0) {
        const auto idx = res.bodies.find(res.bodyIds[static_cast<std::size_t>(bi)]);
        if (!idx) return false;
        const Bodies& b = res.bodies;
        position = {b.state.x[*idx], b.state.y[*idx], b.state.z[*idx]};
        velocity = {b.state.vx[*idx], b.state.vy[*idx], b.state.vz[*idx]};
        return true;
    }
    for (std::size_t i = 0; i < scene.shuttles.size(); ++i) {
        if (scene.shuttles[i].id != id) continue;
        position = res.shuttles[i].position;
        velocity = res.shuttles[i].velocity;
        return true;
    }
    return false;
}

} // namespace

// ---- field table -------------------------------------------------------------------------------------------------------

FieldInfo fieldInfo(Field f)
{
    switch (f) {
    case Field::Name: return {"Name", Quantity::Number, "", true, true};
    case Field::Mass: return {"Mass", Quantity::BodyMass, "kg", true, false};
    case Field::Radius: return {"Radius", Quantity::Length, "m", true, false};
    case Field::SpinPeriod: return {"Spin period", Quantity::Time, "h", true, false};
    case Field::AtmoRho0: return {"Surface density rho0", Quantity::Density, "kg/m^3", true, false};
    case Field::AtmoScaleHeight: return {"Scale height H", Quantity::Length, "m", true, false};
    case Field::PosX: return {"Position x", Quantity::Length, "m", true, true};
    case Field::PosY: return {"Position y", Quantity::Length, "m", true, true};
    case Field::VelX: return {"Velocity x", Quantity::Speed, "m/s", true, true};
    case Field::VelY: return {"Velocity y", Quantity::Speed, "m/s", true, true};
    case Field::OrbitA: return {"Semi-major axis a", Quantity::Length, "m", true, true};
    case Field::OrbitE: return {"Eccentricity e", Quantity::Number, "", true, true};
    case Field::OrbitOmega: return {"Argument of periapsis", Quantity::Angle, "deg", true, true};
    case Field::OrbitMeanAnomaly: return {"Mean anomaly", Quantity::Angle, "deg", true, true};
    case Field::DryMass: return {"Dry mass", Quantity::Mass, "kg", false, true};
    case Field::FuelMass: return {"Fuel mass", Quantity::Mass, "kg", false, true};
    case Field::MaxThrust: return {"Max thrust", Quantity::Force, "N", false, true};
    case Field::ExhaustVelocity: return {"Exhaust velocity", Quantity::Speed, "m/s", false, true};
    case Field::CdA: return {"Cd*A", Quantity::Area, "m^2", false, true};
    case Field::CollisionRadius: return {"Collision radius", Quantity::Length, "m", false, true};
    case Field::MaxTorque: return {"Max torque", Quantity::Torque, "N m", false, true};
    case Field::Inertia: return {"Moment of inertia", Quantity::Inertia, "kg m^2", false, true};
    }
    return {"", Quantity::Number, "", false, false};
}

bool fieldApplies(ObjectType type, Field f)
{
    const FieldInfo i = fieldInfo(f);
    return (type == ObjectType::Body && i.forBody) || (type == ObjectType::Shuttle && i.forShuttle);
}

// ---- document ----------------------------------------------------------------------------------------------------------

EditorModel::EditorModel()
{
    scene_.name = "Untitled";
}

void EditorModel::bump()
{
    ++revision_;
    cache_.valid = false;
}

void EditorModel::noteNames(const Scene& s)
{
    static const BodyKind kinds[] = {BodyKind::Star,     BodyKind::Planet,      BodyKind::GasGiant,   BodyKind::DwarfPlanet,
                                     BodyKind::Moon,     BodyKind::Asteroid,    BodyKind::NeutronStar, BodyKind::BlackHole};
    for (const SceneBody& b : s.bodies) {
        for (BodyKind k : kinds) {
            const std::uint64_t n = genericNumber(b.name, std::string(kindName(k)) + " ");
            if (n > names_.issued(k)) names_.setIssued(k, n);
        }
    }
    for (const SceneShuttle& sh : s.shuttles) shuttleNames_ = std::max(shuttleNames_, genericNumber(sh.name, "Shuttle "));
}

void EditorModel::noteIds(const Scene& s, std::uint64_t reserve)
{
    std::uint64_t top = reserve;
    for (const SceneBody& b : s.bodies) top = std::max(top, b.id);
    for (const SceneShuttle& sh : s.shuttles) top = std::max(top, sh.id);
    nextId_ = std::max(nextId_, top + 1);
}

void EditorModel::setScene(Scene scene)
{
    scene_ = std::move(scene);
    selected_ = 0;
    recentre_ = Recentre::Auto;
    names_ = NameCounter{};
    shuttleNames_ = 0;
    nextId_ = 1;
    noteNames(scene_);
    noteIds(scene_, 0);
    bump();
}

void EditorModel::restoreScene(Scene scene, std::uint64_t reserveIdsUpTo)
{
    scene_ = std::move(scene);
    if (typeOf(selected_) == ObjectType::None) selected_ = 0;
    noteNames(scene_);
    noteIds(scene_, reserveIdsUpTo);
    bump();
}

void EditorModel::reset()
{
    Scene s;
    s.name = "Untitled";
    setScene(std::move(s));
}

void EditorModel::setRecentre(Recentre r)
{
    if (r == recentre_) return;
    recentre_ = r;
    cache_.valid = false;
}

ResolveOptions EditorModel::resolveOptions() const
{
    ResolveOptions o;
    o.recentre = recentre_;
    return o;
}

// ---- queries -----------------------------------------------------------------------------------------------------------

int EditorModel::shuttleIndex(std::uint64_t id) const
{
    for (std::size_t i = 0; i < scene_.shuttles.size(); ++i)
        if (scene_.shuttles[i].id == id) return static_cast<int>(i);
    return -1;
}

ObjectType EditorModel::typeOf(std::uint64_t id) const
{
    if (id == 0) return ObjectType::None;
    if (bodyIndex(id) >= 0) return ObjectType::Body;
    if (shuttleIndex(id) >= 0) return ObjectType::Shuttle;
    return ObjectType::None;
}

const SceneBody* EditorModel::body(std::uint64_t id) const
{
    const int i = bodyIndex(id);
    return i < 0 ? nullptr : &scene_.bodies[static_cast<std::size_t>(i)];
}

const SceneShuttle* EditorModel::shuttle(std::uint64_t id) const
{
    const int i = shuttleIndex(id);
    return i < 0 ? nullptr : &scene_.shuttles[static_cast<std::size_t>(i)];
}

bool EditorModel::select(std::uint64_t id)
{
    if (typeOf(id) == ObjectType::None) return false;
    selected_ = id;
    return true;
}

std::vector<ObjectRow> EditorModel::objects() const
{
    std::vector<ObjectRow> rows;
    rows.reserve(scene_.bodies.size() + scene_.shuttles.size());
    for (const SceneBody& b : scene_.bodies) {
        ObjectRow r;
        r.id = b.id;
        r.type = ObjectType::Body;
        r.name = b.name;
        r.kind = b.kind;
        rows.push_back(std::move(r));
    }
    for (const SceneShuttle& s : scene_.shuttles) {
        ObjectRow r;
        r.id = s.id;
        r.type = ObjectType::Shuttle;
        r.name = s.name;
        r.controller = s.controller;
        rows.push_back(std::move(r));
    }
    return rows;
}

bool EditorModel::inOrbitMode(std::uint64_t id) const
{
    if (const SceneBody* b = body(id)) return b->orbit.has_value();
    if (const SceneShuttle* s = shuttle(id)) return s->orbit.has_value();
    return false;
}

const ResolvedScene* EditorModel::resolved() const
{
    if (!cache_.valid || cache_.revision != revision_ || cache_.recentre != recentre_) {
        std::string err;
        cache_.res = ResolvedScene{};
        cache_.ok = resolveScene(scene_, cache_.res, err, resolveOptions());
        cache_.valid = true;
        cache_.revision = revision_;
        cache_.recentre = recentre_;
    }
    return cache_.ok ? &cache_.res : nullptr;
}

bool EditorModel::resolvedState(std::uint64_t id, Vec3& position, Vec3& velocity) const
{
    const ResolvedScene* res = resolved();
    return res != nullptr && stateIn(scene_, *res, id, position, velocity);
}

bool EditorModel::validate(std::string& error) const
{
    if (!validateScene(scene_, error)) return false;
    ResolvedScene res;
    return resolveScene(scene_, res, error, resolveOptions());
}

double EditorModel::suggestedOrbitDistance(std::uint64_t parentId, ObjectType forType) const
{
    const SceneBody* p = body(parentId);
    if (!p) return 0.0;
    if (forType == ObjectType::Shuttle) {
        double top = 0.0;   // plan 5.5: the atmosphere ends where rho = 1e-6 kg/m^3
        if (p->atmosphere && p->atmosphere->rho0 > 1.0e-6) top = p->atmosphere->scaleHeight * std::log(p->atmosphere->rho0 / 1.0e-6);
        return p->radius + std::max(0.25 * p->radius, 1.25 * top);
    }
    return 4.0 * p->radius;
}

// ---- warnings ----------------------------------------------------------------------------------------------------------

const SceneBody* EditorModel::heaviestOther(std::uint64_t id) const
{
    const SceneBody* heavy = nullptr;
    for (const SceneBody& o : scene_.bodies) {
        if (o.id == id) continue;
        if (!heavy || o.mu > heavy->mu) heavy = &o;
    }
    return heavy;
}

bool EditorModel::dominates(std::uint64_t id, double mu) const
{
    const SceneBody* heavy = heaviestOther(id);
    return heavy != nullptr && mu > kDominanceFraction * heavy->mu;
}

std::string EditorModel::dominanceWarning(std::uint64_t id) const
{
    const SceneBody* b = body(id);
    if (!b) return {};
    const SceneBody* heavy = heaviestOther(id);
    if (!heavy || !(b->mu > kDominanceFraction * heavy->mu)) return {};
    const double ratio = b->mu / heavy->mu;
    if (ratio < 1.0)
        return quoted(b->name) + " has " + formatNumber(100.0 * ratio, 3) + "% of the mass of " + quoted(heavy->name) +
               ", the most massive body (warning above 10%): it will strongly disturb the system.";
    return quoted(b->name) + " is " + formatNumber(ratio, 3) + " times as massive as " + quoted(heavy->name) +
           ", the most massive body: it will dominate the scene.";
}

std::vector<std::string> EditorModel::placementWarnings(std::uint64_t id) const
{
    std::vector<std::string> out;
    const ObjectType type = typeOf(id);
    if (type == ObjectType::None) return out;
    const std::string myName = type == ObjectType::Body ? body(id)->name : shuttle(id)->name;

    // Names used twice
    for (const SceneBody& b : scene_.bodies)
        if (b.id != id && b.name == myName) { out.push_back("The name " + quoted(myName) + " is also used by another object."); break; }
    if (out.empty()) {
        for (const SceneShuttle& s : scene_.shuttles)
            if (s.id != id && s.name == myName) { out.push_back("The name " + quoted(myName) + " is also used by another object."); break; }
    }

    // Periapsis inside the parent
    const std::optional<SceneOrbit>& orbit = type == ObjectType::Body ? body(id)->orbit : shuttle(id)->orbit;
    if (orbit) {
        if (const SceneBody* p = body(orbit->parent)) {
            const double own = type == ObjectType::Body ? body(id)->radius : 0.0;
            const double peri = orbit->a * (1.0 - orbit->e);
            if (peri < p->radius + own)
                out.push_back("The periapsis (" + lengthText(peri) + " from the centre) is inside " + quoted(p->name) +
                              " (radius " + lengthText(p->radius) + ").");
        }
    }

    // Overlap at the start
    Vec3 r, v;
    if (resolvedState(id, r, v)) {
        for (const SceneBody& o : scene_.bodies) {
            if (o.id == id) continue;
            Vec3 ro, vo;
            if (!resolvedState(o.id, ro, vo)) continue;
            const double d = norm(r - ro);
            if (type == ObjectType::Body) {
                if (d < body(id)->radius + o.radius)
                    out.push_back(quoted(myName) + " overlaps " + quoted(o.name) + " at the start: they would merge immediately.");
            } else if (d < o.radius) {
                out.push_back(quoted(myName) + " starts inside " + quoted(o.name) + ".");
            }
        }
    }
    return out;
}

void EditorModel::finish(EditResult& r, std::uint64_t id, bool dominance) const
{
    if (dominance) {
        const std::string d = dominanceWarning(id);
        if (!d.empty()) r.warnings.push_back(d);
    }
    for (std::string& w : placementWarnings(id)) r.warnings.push_back(std::move(w));
}

// ---- adding and deleting -----------------------------------------------------------------------------------------------

EditResult EditorModel::addBody(BodyKind kind, Vec3 position, Vec3 velocity)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    if (!finiteVec(position) || !finiteVec(velocity)) return EditResult::failure("The position and velocity must be finite numbers.");

    const BodyDesc d = defaultBodyDesc(kind);
    SceneBody b;
    b.id = takeId();
    b.name = names_.next(kind);
    b.kind = d.kind;
    b.kindLocked = d.kindLocked;
    b.fictionalDensity = d.fictionalDensity;
    b.mu = d.mu;
    b.radius = d.radius;
    b.spinPeriod_h = 0.0;
    b.inertiaFactor = d.inertiaFactor;
    b.atmosphere = d.atmosphere;
    b.colour = d.colour;
    b.position = position;
    b.velocity = velocity;
    scene_.bodies.push_back(std::move(b));
    selected_ = scene_.bodies.back().id;
    bump();
    ++edits_;

    EditResult r = EditResult::success(selected_);
    finish(r, selected_, true);
    return r;
}

EditResult EditorModel::addShuttle(ControllerKind controller, Vec3 position, Vec3 velocity)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    if (!finiteVec(position) || !finiteVec(velocity)) return EditResult::failure("The position and velocity must be finite numbers.");

    SceneShuttle s;   // the defaults are the table of plan 7.1
    s.id = takeId();
    s.name = "Shuttle " + std::to_string(++shuttleNames_);
    s.controller = controllerName(controller);
    s.position = position;
    s.velocity = velocity;
    scene_.shuttles.push_back(std::move(s));
    selected_ = scene_.shuttles.back().id;
    bump();
    ++edits_;

    EditResult r = EditResult::success(selected_);
    finish(r, selected_, false);
    return r;
}

EditResult EditorModel::remove(std::uint64_t id)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    const ObjectType type = typeOf(id);
    if (type == ObjectType::None) return EditResult::failure("There is no object with id " + std::to_string(id) + ".");

    if (type == ObjectType::Body) {
        // Whatever orbits this body keeps its place and becomes an explicit state
        struct Frozen {
            std::uint64_t id;
            bool shuttle;
            Vec3 r, v;
        };
        std::vector<Frozen> frozen;
        for (const SceneBody& b : scene_.bodies)
            if (b.orbit && b.orbit->parent == id) frozen.push_back({b.id, false, {}, {}});
        for (const SceneShuttle& s : scene_.shuttles)
            if (s.orbit && s.orbit->parent == id) frozen.push_back({s.id, true, {}, {}});
        if (!frozen.empty()) {
            // The state they were placed at, before the recentring of 5.9 step 4, so the geometry of the system does not change
            ResolvedScene res;
            std::string err;
            ResolveOptions never;
            never.recentre = Recentre::Never;
            if (!resolveScene(scene_, res, err, never))
                return EditResult::failure("Cannot delete: the scene cannot be resolved, so the satellites of this body could not keep their place (" + err + ").");
            for (Frozen& f : frozen) stateIn(scene_, res, f.id, f.r, f.v);
        }
        for (const Frozen& f : frozen) {
            if (f.shuttle) {
                SceneShuttle& s = scene_.shuttles[static_cast<std::size_t>(shuttleIndex(f.id))];
                s.orbit.reset();
                s.position = f.r;
                s.velocity = f.v;
            } else {
                SceneBody& b = scene_.bodies[static_cast<std::size_t>(bodyIndex(f.id))];
                b.orbit.reset();
                b.position = f.r;
                b.velocity = f.v;
            }
        }
        scene_.bodies.erase(scene_.bodies.begin() + bodyIndex(id));
    } else {
        scene_.shuttles.erase(scene_.shuttles.begin() + shuttleIndex(id));
    }
    if (selected_ == id) selected_ = 0;
    bump();
    ++edits_;
    return EditResult::success(id);
}

std::string EditorModel::copyName(const std::string& name) const
{
    auto used = [this](const std::string& n) {
        for (const SceneBody& b : scene_.bodies)
            if (b.name == n) return true;
        for (const SceneShuttle& s : scene_.shuttles)
            if (s.name == n) return true;
        return false;
    };
    for (int k = 1;; ++k) {
        const std::string suffix = k == 1 ? " copy" : " copy " + std::to_string(k);
        std::string base = name.substr(0, std::min<std::size_t>(name.size(), 64 - suffix.size()));
        while (!base.empty() && base.back() == ' ') base.pop_back();
        const std::string candidate = base + suffix;
        if (!used(candidate)) return candidate;
    }
}

EditResult EditorModel::duplicate(std::uint64_t id)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    const ObjectType type = typeOf(id);
    if (type == ObjectType::None) return EditResult::failure("There is no object with id " + std::to_string(id) + ".");
    auto halfOrbitOn = [](SceneOrbit& o) {
        o.meanAnomaly_deg = std::fmod(o.meanAnomaly_deg + 180.0, 360.0);
        if (o.meanAnomaly_deg < 0.0) o.meanAnomaly_deg += 360.0;
    };
    const Scene before = scene_;
    std::uint64_t newId = 0;
    if (type == ObjectType::Body) {
        SceneBody b = *body(id);
        b.id = takeId();
        b.name = copyName(b.name);
        if (b.orbit) halfOrbitOn(*b.orbit);
        else b.position = b.position + Vec3{4.0 * b.radius, 0.0, 0.0};
        newId = b.id;
        scene_.bodies.push_back(std::move(b));
    } else {
        SceneShuttle sh = *shuttle(id);
        sh.id = takeId();
        sh.name = copyName(sh.name);
        if (sh.orbit) halfOrbitOn(*sh.orbit);
        else sh.position = sh.position + Vec3{1000.0, 0.0, 0.0};
        newId = sh.id;
        scene_.shuttles.push_back(std::move(sh));
    }
    std::string err;
    if (!validateScene(scene_, err)) {
        scene_ = before;
        return EditResult::failure(err);
    }
    selected_ = newId;
    bump();
    ++edits_;
    EditResult r = EditResult::success(newId);
    finish(r, newId, type == ObjectType::Body);
    return r;
}

// ---- editing -----------------------------------------------------------------------------------------------------------

EditResult EditorModel::commitBody(std::uint64_t id, const std::function<void(SceneBody&)>& fn)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    const int i = bodyIndex(id);
    if (i < 0) return EditResult::failure("There is no body with id " + std::to_string(id) + ".");

    SceneBody& b = scene_.bodies[static_cast<std::size_t>(i)];
    const SceneBody old = b;
    const bool wasDominant = dominates(id, old.mu);
    fn(b);
    b.id = old.id;
    std::string err;
    if (!checkBody(b, err) || !validateScene(scene_, err)) {
        b = old;
        return EditResult::failure(err);
    }
    bump();
    ++edits_;
    EditResult r = EditResult::success(id);
    finish(r, id, b.mu != old.mu && !wasDominant);   // only when this edit makes it dominant
    return r;
}

EditResult EditorModel::commitShuttle(std::uint64_t id, const std::function<void(SceneShuttle&)>& fn)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    const int i = shuttleIndex(id);
    if (i < 0) return EditResult::failure("There is no shuttle with id " + std::to_string(id) + ".");

    SceneShuttle& s = scene_.shuttles[static_cast<std::size_t>(i)];
    const SceneShuttle old = s;
    fn(s);
    s.id = old.id;
    std::string err;
    if (!checkShuttle(s, err) || !validateScene(scene_, err)) {
        s = old;
        return EditResult::failure(err);
    }
    bump();
    ++edits_;
    EditResult r = EditResult::success(id);
    finish(r, id, false);
    return r;
}

EditResult EditorModel::updateBody(std::uint64_t id, const std::function<void(SceneBody&)>& fn) { return commitBody(id, fn); }
EditResult EditorModel::updateShuttle(std::uint64_t id, const std::function<void(SceneShuttle&)>& fn) { return commitShuttle(id, fn); }

EditResult EditorModel::setName(std::uint64_t id, std::string_view name)
{
    const std::string n = trimmed(name);
    if (typeOf(id) == ObjectType::Shuttle) return commitShuttle(id, [&](SceneShuttle& s) { s.name = n; });
    return commitBody(id, [&](SceneBody& b) { b.name = n; });
}

EditResult EditorModel::setKind(std::uint64_t id, BodyKind kind, bool applyDefaults)
{
    return commitBody(
        id,
        [&](SceneBody& b) {
            b.kind = kind;
            if (applyDefaults) {
                const BodyDesc d = defaultBodyDesc(kind);
                b.mu = d.mu;
                b.radius = d.radius;
                b.colour = d.colour;
            }
        });
}

EditResult EditorModel::setMu(std::uint64_t id, double mu)
{
    return commitBody(id, [&](SceneBody& b) { b.mu = mu; });
}

EditResult EditorModel::setMassKg(std::uint64_t id, double kg) { return setMu(id, muFromKg(kg)); }

EditResult EditorModel::setRadius(std::uint64_t id, double radius)
{
    return commitBody(id, [&](SceneBody& b) { b.radius = radius; });
}

EditResult EditorModel::setColour(std::uint64_t id, const std::array<float, 3>& rgb)
{
    return commitBody(id, [&](SceneBody& b) { b.colour = rgb; });
}

EditResult EditorModel::setSpinPeriodHours(std::uint64_t id, double hours)
{
    return commitBody(id, [&](SceneBody& b) { b.spinPeriod_h = hours; });
}

EditResult EditorModel::setAtmosphere(std::uint64_t id, const std::optional<Atmosphere>& atmosphere)
{
    return commitBody(id, [&](SceneBody& b) { b.atmosphere = atmosphere; });
}

EditResult EditorModel::enableAtmosphere(std::uint64_t id, bool on)
{
    const SceneBody* b = body(id);
    if (b && b->atmosphere.has_value() == on) return locked_ ? EditResult::failure(kLockedMessage) : EditResult::success(id);
    Atmosphere a;
    a.rho0 = 1.225;
    a.scaleHeight = 8500.0;
    return setAtmosphere(id, on ? std::optional<Atmosphere>(a) : std::nullopt);
}

EditResult EditorModel::setState(std::uint64_t id, const Vec3& position, const Vec3& velocity)
{
    if (!finiteVec(position) || !finiteVec(velocity)) return EditResult::failure("The position and velocity must be finite numbers.");
    if (typeOf(id) == ObjectType::Shuttle)
        return commitShuttle(id, [&](SceneShuttle& s) { s.orbit.reset(); s.position = position; s.velocity = velocity; });
    return commitBody(id, [&](SceneBody& b) { b.orbit.reset(); b.position = position; b.velocity = velocity; });
}

EditResult EditorModel::setOrbit(std::uint64_t id, const SceneOrbit& orbit)
{
    if (!finite(orbit.a) || !(orbit.a > 0.0)) return EditResult::failure("The semi-major axis a must be greater than zero.");
    if (!finite(orbit.e) || orbit.e < 0.0 || orbit.e >= 1.0)
        return EditResult::failure("The eccentricity e must be at least 0 and below 1 (bound orbits only).");
    if (!finite(orbit.argPeriapsis_deg) || !finite(orbit.meanAnomaly_deg)) return EditResult::failure("The angles must be finite numbers.");
    if (id == orbit.parent) return EditResult::failure("An object cannot orbit itself.");
    if (typeOf(orbit.parent) != ObjectType::Body) return EditResult::failure("The parent must be a body of the scene.");
    if (typeOf(id) == ObjectType::Shuttle) return commitShuttle(id, [&](SceneShuttle& s) { s.orbit = orbit; });
    return commitBody(id, [&](SceneBody& b) { b.orbit = orbit; });
}

EditResult EditorModel::setOrbitAround(std::uint64_t id, std::uint64_t parentId, double distance)
{
    if (!finite(distance) || !(distance > 0.0)) return EditResult::failure("The distance must be greater than zero.");
    SceneOrbit o;
    const std::optional<SceneOrbit>* existing = nullptr;
    if (const SceneBody* b = body(id)) existing = &b->orbit;
    else if (const SceneShuttle* s = shuttle(id)) existing = &s->orbit;
    if (existing && existing->has_value() && (*existing)->parent == parentId) o = **existing;
    o.parent = parentId;
    o.a = distance;
    return setOrbit(id, o);
}

EditResult EditorModel::bakeState(std::uint64_t id)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    if (typeOf(id) == ObjectType::None) return EditResult::failure("There is no object with id " + std::to_string(id) + ".");
    if (!inOrbitMode(id)) return EditResult::success(id);
    ResolvedScene res;
    std::string err;
    ResolveOptions never;
    never.recentre = Recentre::Never;
    Vec3 r, v;
    if (!resolveScene(scene_, res, err, never) || !stateIn(scene_, res, id, r, v))
        return EditResult::failure("The scene cannot be resolved, so the position and velocity are unknown.");
    return setState(id, r, v);
}

EditResult EditorModel::setController(std::uint64_t id, ControllerKind kind)
{
    if (typeOf(id) != ObjectType::Shuttle && !locked_) return EditResult::failure("Only shuttles have a controller.");
    return commitShuttle(id, [&](SceneShuttle& s) { s.controller = controllerName(kind); });
}

// ---- text fields -------------------------------------------------------------------------------------------------------

std::string EditorModel::fieldText(std::uint64_t id, Field f) const
{
    const ObjectType type = typeOf(id);
    if (!fieldApplies(type, f)) return {};
    const SceneBody* b = body(id);
    const SceneShuttle* s = shuttle(id);
    const std::optional<SceneOrbit>& orbit = b ? b->orbit : s->orbit;

    if (f == Field::Name) return b ? b->name : s->name;

    // Position and velocity: the scene value in state mode, the resolved value (read-only) in orbit mode
    if (f == Field::PosX || f == Field::PosY || f == Field::VelX || f == Field::VelY) {
        Vec3 r = b ? b->position : s->position;
        Vec3 v = b ? b->velocity : s->velocity;
        if (orbit && !resolvedState(id, r, v)) return {};
        switch (f) {
        case Field::PosX: return fq(r.x, Quantity::Length);
        case Field::PosY: return fq(r.y, Quantity::Length);
        case Field::VelX: return fq(v.x, Quantity::Speed);
        default: return fq(v.y, Quantity::Speed);
        }
    }

    switch (f) {
    case Field::OrbitA: return orbit ? fq(orbit->a, Quantity::Length) : std::string();
    case Field::OrbitE: return orbit ? formatNumber(orbit->e, kFieldSig) : std::string();
    case Field::OrbitOmega: return orbit ? fq(orbit->argPeriapsis_deg, Quantity::Angle, "deg") : std::string();
    case Field::OrbitMeanAnomaly: return orbit ? fq(orbit->meanAnomaly_deg, Quantity::Angle, "deg") : std::string();
    default: break;
    }

    if (b) {
        switch (f) {
        case Field::Mass: return fq(b->mu, Quantity::BodyMass);
        case Field::Radius: return fq(b->radius, Quantity::Length);
        case Field::SpinPeriod: return fq(b->spinPeriod_h * 3600.0, Quantity::Time, "h");
        case Field::AtmoRho0: return b->atmosphere ? fq(b->atmosphere->rho0, Quantity::Density) : std::string();
        case Field::AtmoScaleHeight: return b->atmosphere ? fq(b->atmosphere->scaleHeight, Quantity::Length) : std::string();
        default: return {};
        }
    }
    switch (f) {
    case Field::DryMass: return fq(s->dryMass, Quantity::Mass);
    case Field::FuelMass: return fq(s->fuelMass, Quantity::Mass);
    case Field::MaxThrust: return fq(s->maxThrust, Quantity::Force);
    case Field::ExhaustVelocity: return fq(s->exhaustVelocity, Quantity::Speed);
    case Field::CdA: return fq(s->cdA, Quantity::Area);
    case Field::CollisionRadius: return fq(s->radius, Quantity::Length);
    case Field::MaxTorque: return fq(s->maxTorque, Quantity::Torque);
    case Field::Inertia: return fq(s->inertia, Quantity::Inertia);
    default: return {};
    }
}

EditResult EditorModel::setFieldText(std::uint64_t id, Field f, std::string_view textIn)
{
    if (locked_) return EditResult::failure(kLockedMessage);
    const ObjectType type = typeOf(id);
    if (type == ObjectType::None) return EditResult::failure("There is no object with id " + std::to_string(id) + ".");
    const FieldInfo info = fieldInfo(f);
    if (!fieldApplies(type, f))
        return EditResult::failure(std::string(info.label) + " does not apply to a " + (type == ObjectType::Body ? "body." : "shuttle."));

    const std::string text = trimmed(textIn);
    if (f == Field::Name) {
        if (text == fieldText(id, f)) return EditResult::success(id);
        return setName(id, text);
    }
    if (text == fieldText(id, f)) return EditResult::success(id);   // untouched field: do not round the stored value

    const std::string label = info.label;
    const ParseResult p = parseQuantity(text, info.quantity, info.defaultUnit);
    if (!p) return EditResult::failure(label + ": " + p.error);
    const double v = p.value;
    const auto bad = [&](const char* why) { return EditResult::failure(label + " " + why); };

    const bool isPosVel = f == Field::PosX || f == Field::PosY || f == Field::VelX || f == Field::VelY;
    const bool isOrbit = f == Field::OrbitA || f == Field::OrbitE || f == Field::OrbitOmega || f == Field::OrbitMeanAnomaly;
    if (isPosVel && inOrbitMode(id))
        return EditResult::failure("This object is placed by an orbit; edit the orbit elements, or convert it to a position and velocity first.");
    if (isOrbit && !inOrbitMode(id))
        return EditResult::failure("This object is placed by position and velocity; choose a parent to orbit first.");

    if (isPosVel) {
        const auto set = [&](Vec3& pos, Vec3& vel) {
            switch (f) {
            case Field::PosX: pos.x = v; break;
            case Field::PosY: pos.y = v; break;
            case Field::VelX: vel.x = v; break;
            default: vel.y = v; break;
            }
        };
        if (type == ObjectType::Shuttle) return commitShuttle(id, [&](SceneShuttle& s) { set(s.position, s.velocity); });
        return commitBody(id, [&](SceneBody& b) { set(b.position, b.velocity); });
    }
    if (isOrbit) {
        if (f == Field::OrbitA && !(v > 0.0)) return bad("must be greater than zero.");
        if (f == Field::OrbitE && (v < 0.0 || v >= 1.0)) return bad("must be at least 0 and below 1 (bound orbits only).");
        const auto set = [&](SceneOrbit& o) {
            switch (f) {
            case Field::OrbitA: o.a = v; break;
            case Field::OrbitE: o.e = v; break;
            case Field::OrbitOmega: o.argPeriapsis_deg = v; break;
            default: o.meanAnomaly_deg = v; break;
            }
        };
        if (type == ObjectType::Shuttle) return commitShuttle(id, [&](SceneShuttle& s) { set(*s.orbit); });
        return commitBody(id, [&](SceneBody& b) { set(*b.orbit); });
    }

    switch (f) {
    case Field::Mass:
        if (!(v > 0.0)) return bad("must be greater than zero.");
        return setMu(id, v);
    case Field::Radius:
        if (!(v > 0.0)) return bad("must be greater than zero.");
        return setRadius(id, v);
    case Field::SpinPeriod: return setSpinPeriodHours(id, v / 3600.0);
    case Field::AtmoRho0:
    case Field::AtmoScaleHeight: {
        const SceneBody* b = body(id);
        if (!b->atmosphere) return EditResult::failure("Switch the atmosphere on first.");
        if (f == Field::AtmoRho0 && v < 0.0) return bad("cannot be negative.");
        if (f == Field::AtmoScaleHeight && !(v > 0.0)) return bad("must be greater than zero.");
        Atmosphere a = *b->atmosphere;
        (f == Field::AtmoRho0 ? a.rho0 : a.scaleHeight) = v;
        return setAtmosphere(id, a);
    }
    case Field::DryMass:
        if (!(v > 0.0)) return bad("must be greater than zero.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.dryMass = v; });
    case Field::FuelMass:
        if (v < 0.0) return bad("cannot be negative.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.fuelMass = v; });
    case Field::MaxThrust:
        if (v < 0.0) return bad("cannot be negative.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.maxThrust = v; });
    case Field::ExhaustVelocity:
        if (v < 0.0) return bad("cannot be negative.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.exhaustVelocity = v; });
    case Field::CdA:
        if (v < 0.0) return bad("cannot be negative.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.cdA = v; });
    case Field::CollisionRadius:
        if (!(v > 0.0)) return bad("must be greater than zero.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.radius = v; });
    case Field::MaxTorque:
        if (v < 0.0) return bad("cannot be negative.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.maxTorque = v; });
    case Field::Inertia:
        if (!(v > 0.0)) return bad("must be greater than zero.");
        return commitShuttle(id, [&](SceneShuttle& s) { s.inertia = v; });
    default: break;
    }
    return EditResult::failure("Unhandled field.");
}

} // namespace app
} // namespace cosmos
