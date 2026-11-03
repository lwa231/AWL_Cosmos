#include "core/scene_json.h"
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <unordered_set>
#include <nlohmann/json.hpp>

namespace cosmos {

namespace {

using json = nlohmann::json;
using ojson = nlohmann::ordered_json;

// ---------------------------------------------------------------------------------------------------------------
// Loading. Parse helpers throw ParseError internally; every public function catches it, so nothing escapes.
// ---------------------------------------------------------------------------------------------------------------

struct ParseError {
    std::string msg;
};

[[noreturn]] void fail(const std::string& path, const std::string& what)
{
    throw ParseError{path.empty() ? what : path + ": " + what};
}

std::string child(const std::string& path, const char* key) { return path.empty() ? key : path + "." + key; }
std::string child(const std::string& path, std::size_t i) { return path + "[" + std::to_string(i) + "]"; }

const json* member(const json& o, const char* key)
{
    const auto it = o.find(key);
    return it == o.end() ? nullptr : &*it;
}

const json& needObject(const json& j, const std::string& path)
{
    if (!j.is_object()) fail(path, std::string("expected an object, found ") + j.type_name());
    return j;
}

double toNumber(const json& v, const std::string& path)
{
    if (!v.is_number()) fail(path, std::string("expected a number, found ") + v.type_name());
    const double d = v.get<double>();
    if (!std::isfinite(d)) fail(path, "number is out of range");
    return d;
}

double number(const json& o, const char* key, const std::string& path, std::optional<double> def = std::nullopt)
{
    const json* v = member(o, key);
    if (v == nullptr) {
        if (!def) fail(child(path, key), "required field is missing");
        return *def;
    }
    return toNumber(*v, child(path, key));
}

std::uint64_t toUint(const json& v, const std::string& path)
{
    if (!v.is_number_unsigned()) fail(path, "expected a non-negative integer");
    return v.get<std::uint64_t>();
}

bool boolean(const json& o, const char* key, const std::string& path, bool def)
{
    const json* v = member(o, key);
    if (v == nullptr) return def;
    if (!v->is_boolean()) fail(child(path, key), std::string("expected true or false, found ") + v->type_name());
    return v->get<bool>();
}

std::string text(const json& o, const char* key, const std::string& path, const std::string& def)
{
    const json* v = member(o, key);
    if (v == nullptr) return def;
    if (!v->is_string()) fail(child(path, key), std::string("expected a string, found ") + v->type_name());
    return v->get<std::string>();
}

std::array<double, 3> triple(const json& v, const std::string& path)
{
    if (!v.is_array() || v.size() != 3) fail(path, "expected an array of 3 numbers");
    std::array<double, 3> t{};
    for (std::size_t i = 0; i < 3; ++i) t[i] = toNumber(v[i], child(path, i));
    return t;
}

std::string squash(const std::string& s)
{
    std::string out;
    for (char c : s) {
        if (c == ' ' || c == '_' || c == '-') continue;
        out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

struct KindName {
    const char* squashed;
    const char* canonical;
    BodyKind kind;
};
const KindName kKinds[] = {
    {"star", "Star", BodyKind::Star},
    {"planet", "Planet", BodyKind::Planet},
    {"gasgiant", "GasGiant", BodyKind::GasGiant},
    {"dwarfplanet", "DwarfPlanet", BodyKind::DwarfPlanet},
    {"moon", "Moon", BodyKind::Moon},
    {"asteroid", "Asteroid", BodyKind::Asteroid},
    {"neutronstar", "NeutronStar", BodyKind::NeutronStar},
    {"blackhole", "BlackHole", BodyKind::BlackHole},
};

BodyKind parseKind(const json& o, const std::string& path)
{
    const json* v = member(o, "kind");
    if (v == nullptr) return BodyKind::Planet;
    if (!v->is_string()) fail(child(path, "kind"), std::string("expected a string, found ") + v->type_name());
    const std::string s = squash(v->get<std::string>());
    for (const KindName& k : kKinds)
        if (s == k.squashed) return k.kind;
    fail(child(path, "kind"), "unknown kind \"" + v->get<std::string>() +
                              "\" (use Star, Planet, GasGiant, DwarfPlanet, Moon, Asteroid, NeutronStar or BlackHole)");
}

const char* kindName(BodyKind k)
{
    for (const KindName& n : kKinds)
        if (n.kind == k) return n.canonical;
    return "Planet";
}

SceneOrbit parseOrbit(const json& j, const std::string& path)
{
    needObject(j, path);
    SceneOrbit o;
    const json* parent = member(j, "parent");
    if (parent == nullptr) fail(child(path, "parent"), "required field is missing");
    o.parent = toUint(*parent, child(path, "parent"));
    o.a = number(j, "a", path);
    o.e = number(j, "e", path, 0.0);
    o.argPeriapsis_deg = number(j, "argPeriapsis_deg", path, 0.0);
    o.meanAnomaly_deg = number(j, "meanAnomaly_deg", path, 0.0);
    o.retrograde = boolean(j, "retrograde", path, false);
    return o;
}

// Shared by bodies and shuttles: exactly one of "orbit" or ("position" and "velocity")
void parsePlacement(const json& j, const std::string& path, std::optional<SceneOrbit>& orbit, Vec3& pos, Vec3& vel)
{
    const json* o = member(j, "orbit");
    const json* p = member(j, "position");
    const json* v = member(j, "velocity");
    if (o != nullptr) {
        if (p != nullptr || v != nullptr) fail(path, "has both an \"orbit\" and a \"position\"/\"velocity\"; use only one of them");
        orbit = parseOrbit(*o, child(path, "orbit"));
        return;
    }
    if (p == nullptr || v == nullptr) fail(path, "needs either an \"orbit\" or both a \"position\" and a \"velocity\"");
    const auto a = triple(*p, child(path, "position"));
    const auto b = triple(*v, child(path, "velocity"));
    pos = {a[0], a[1], a[2]};
    vel = {b[0], b[1], b[2]};
}

SceneBody parseBody(const json& j, const std::string& path)
{
    needObject(j, path);
    SceneBody b;
    if (const json* id = member(j, "id")) {
        b.id = toUint(*id, child(path, "id"));
        if (b.id == 0) fail(child(path, "id"), "id must be nonzero (leave it out to get one assigned)");
    }
    b.name = text(j, "name", path, "");
    b.kind = parseKind(j, path);
    b.kindLocked = boolean(j, "kindLocked", path, false);
    b.fictionalDensity = boolean(j, "fictionalDensity", path, false);
    b.mu = number(j, "mu", path);
    b.radius = number(j, "radius", path);
    b.spinPeriod_h = number(j, "spinPeriod_h", path, 0.0);
    b.inertiaFactor = number(j, "inertiaFactor", path, 0.4);
    if (const json* c = member(j, "colour")) {
        const auto t = triple(*c, child(path, "colour"));
        b.colour = {static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2])};
    }
    if (const json* a = member(j, "atmosphere")) {
        const std::string ap = child(path, "atmosphere");
        needObject(*a, ap);
        Atmosphere at;
        at.rho0 = number(*a, "rho0", ap);
        at.scaleHeight = number(*a, "scaleHeight", ap);
        b.atmosphere = at;
    }
    parsePlacement(j, path, b.orbit, b.position, b.velocity);
    return b;
}

SceneShuttle parseShuttle(const json& j, const std::string& path)
{
    needObject(j, path);
    SceneShuttle s;
    const json* id = member(j, "id");
    if (id == nullptr) fail(child(path, "id"), "required field is missing");
    s.id = toUint(*id, child(path, "id"));
    s.name = text(j, "name", path, "");
    s.controller = text(j, "controller", path, s.controller);
    s.dryMass = number(j, "dryMass", path, s.dryMass);
    s.fuelMass = number(j, "fuelMass", path, s.fuelMass);
    s.maxThrust = number(j, "maxThrust", path, s.maxThrust);
    s.exhaustVelocity = number(j, "exhaustVelocity", path, s.exhaustVelocity);
    s.cdA = number(j, "cdA", path, s.cdA);
    s.radius = number(j, "radius", path, s.radius);
    s.maxTorque = number(j, "maxTorque", path, s.maxTorque);
    s.inertia = number(j, "inertia", path, s.inertia);
    parsePlacement(j, path, s.orbit, s.position, s.velocity);
    return s;
}

Scene parseScene(const json& root)
{
    needObject(root, "");
    Scene sc;
    sc.name = text(root, "name", "", "");
    sc.units = text(root, "units", "", "SI");
    if (sc.units != "SI") fail("units", "only \"SI\" is supported, found \"" + sc.units + "\"");
    sc.epoch_s = number(root, "epoch_s", "", 0.0);

    const json* bodies = member(root, "bodies");
    if (bodies == nullptr) fail("bodies", "required field is missing (use an empty array for an empty scene)");
    if (!bodies->is_array()) fail("bodies", std::string("expected an array, found ") + bodies->type_name());
    for (std::size_t i = 0; i < bodies->size(); ++i) sc.bodies.push_back(parseBody((*bodies)[i], child("bodies", i)));

    // Bodies without an id get the smallest unused ones, in file order
    std::unordered_set<std::uint64_t> used;
    for (const SceneBody& b : sc.bodies)
        if (b.id != 0) used.insert(b.id);
    std::uint64_t nextFree = 1;
    for (SceneBody& b : sc.bodies) {
        if (b.id != 0) continue;
        while (used.count(nextFree) != 0) ++nextFree;
        b.id = nextFree;
        used.insert(nextFree);
    }

    if (const json* shuttles = member(root, "shuttles")) {
        if (!shuttles->is_array()) fail("shuttles", std::string("expected an array, found ") + shuttles->type_name());
        for (std::size_t i = 0; i < shuttles->size(); ++i)
            sc.shuttles.push_back(parseShuttle((*shuttles)[i], child("shuttles", i)));
    }

    if (const json* st = member(root, "settings")) {
        needObject(*st, "settings");
        sc.settings.infiniteFuel = boolean(*st, "infiniteFuel", "settings", sc.settings.infiniteFuel);
        sc.settings.controllerTick_s = number(*st, "controllerTick_s", "settings", sc.settings.controllerTick_s);
    }
    return sc;
}

// ---------------------------------------------------------------------------------------------------------------
// Saving
// ---------------------------------------------------------------------------------------------------------------

// A float colour as the shortest decimal that reads back as the same float (0.95, not 0.949999988079071)
double shortestFloat(float f)
{
    char buf[40];
    for (int p = 1; p <= 9; ++p) {
        std::snprintf(buf, sizeof buf, "%.*g", p, static_cast<double>(f));
        if (std::strtof(buf, nullptr) == f) break;
    }
    return std::strtod(buf, nullptr);
}

ojson vec3Json(const Vec3& v) { return ojson::array({v.x, v.y, v.z}); }

ojson orbitJson(const SceneOrbit& o)
{
    ojson j = ojson::object();
    j["parent"] = o.parent;
    j["a"] = o.a;
    j["e"] = o.e;
    j["argPeriapsis_deg"] = o.argPeriapsis_deg;
    j["meanAnomaly_deg"] = o.meanAnomaly_deg;
    j["retrograde"] = o.retrograde;
    return j;
}

ojson bodyJson(const SceneBody& b)
{
    ojson j = ojson::object();
    j["id"] = b.id;
    j["name"] = b.name;
    j["kind"] = kindName(b.kind);
    j["kindLocked"] = b.kindLocked;
    j["fictionalDensity"] = b.fictionalDensity;
    j["mu"] = b.mu;
    j["radius"] = b.radius;
    if (b.spinPeriod_h != 0.0) j["spinPeriod_h"] = b.spinPeriod_h;
    if (b.inertiaFactor != 0.4) j["inertiaFactor"] = b.inertiaFactor;
    j["colour"] = ojson::array({shortestFloat(b.colour[0]), shortestFloat(b.colour[1]), shortestFloat(b.colour[2])});
    if (b.atmosphere) {
        ojson a = ojson::object();
        a["rho0"] = b.atmosphere->rho0;
        a["scaleHeight"] = b.atmosphere->scaleHeight;
        j["atmosphere"] = a;
    }
    if (b.orbit) {
        j["orbit"] = orbitJson(*b.orbit);
    } else {
        j["position"] = vec3Json(b.position);
        j["velocity"] = vec3Json(b.velocity);
    }
    return j;
}

ojson shuttleJson(const SceneShuttle& s)
{
    ojson j = ojson::object();
    j["id"] = s.id;
    j["name"] = s.name;
    j["controller"] = s.controller;
    if (s.orbit) {
        j["orbit"] = orbitJson(*s.orbit);
    } else {
        j["position"] = vec3Json(s.position);
        j["velocity"] = vec3Json(s.velocity);
    }
    j["dryMass"] = s.dryMass;
    j["fuelMass"] = s.fuelMass;
    j["maxThrust"] = s.maxThrust;
    j["exhaustVelocity"] = s.exhaustVelocity;
    j["cdA"] = s.cdA;
    j["radius"] = s.radius;
    j["maxTorque"] = s.maxTorque;
    j["inertia"] = s.inertia;
    return j;
}

ojson sceneJson(const Scene& sc)
{
    ojson j = ojson::object();
    j["name"] = sc.name;
    j["units"] = sc.units;
    j["epoch_s"] = sc.epoch_s;
    ojson bodies = ojson::array();
    for (const SceneBody& b : sc.bodies) bodies.push_back(bodyJson(b));
    j["bodies"] = bodies;
    ojson shuttles = ojson::array();
    for (const SceneShuttle& s : sc.shuttles) shuttles.push_back(shuttleJson(s));
    j["shuttles"] = shuttles;
    ojson st = ojson::object();
    st["infiniteFuel"] = sc.settings.infiniteFuel;
    st["controllerTick_s"] = sc.settings.controllerTick_s;
    j["settings"] = st;
    return j;
}

// One-line form: { "a": 1, "b": [1, 2] }
std::string inlineText(const ojson& j)
{
    if (j.is_object()) {
        if (j.empty()) return "{}";
        std::string s = "{ ";
        bool first = true;
        for (const auto& kv : j.items()) {
            if (!first) s += ", ";
            first = false;
            s += ojson(kv.key()).dump() + ": " + inlineText(kv.value());
        }
        return s + " }";
    }
    if (j.is_array()) {
        std::string s = "[";
        bool first = true;
        for (const auto& v : j) {
            if (!first) s += ", ";
            first = false;
            s += inlineText(v);
        }
        return s + "]";
    }
    return j.dump();
}

// Readable layout: short values stay on one line, long objects and arrays get one entry per line
void pretty(const ojson& j, int depth, std::string& out)
{
    const std::string one = inlineText(j);
    const bool container = j.is_object() || j.is_array();
    if (!container || j.empty() || (depth > 0 && one.size() <= 140)) {
        out += one;
        return;
    }
    const std::string pad(static_cast<std::size_t>(depth + 1) * 2, ' ');
    const std::string padEnd(static_cast<std::size_t>(depth) * 2, ' ');
    out += j.is_object() ? "{\n" : "[\n";
    std::size_t i = 0;
    const std::size_t n = j.size();
    if (j.is_object()) {
        for (const auto& kv : j.items()) {
            out += pad + ojson(kv.key()).dump() + ": ";
            pretty(kv.value(), depth + 1, out);
            out += (++i < n) ? ",\n" : "\n";
        }
    } else {
        for (const auto& v : j) {
            out += pad;
            pretty(v, depth + 1, out);
            out += (++i < n) ? ",\n" : "\n";
        }
    }
    out += padEnd + (j.is_object() ? "}" : "]");
}

} // namespace

bool sceneFromJson(const std::string& textIn, Scene& out, std::string& error)
{
    try {
        json root;
        try {
            root = json::parse(textIn);
        } catch (const json::parse_error& e) {
            error = std::string("not valid JSON: ") + e.what();
            return false;
        }
        Scene sc = parseScene(root);
        if (!validateScene(sc, error)) return false;
        out = std::move(sc);
        return true;
    } catch (const ParseError& e) {
        error = e.msg;
    } catch (const std::exception& e) {
        error = std::string("scene could not be read: ") + e.what();
    } catch (...) {
        error = "scene could not be read: unknown error";
    }
    return false;
}

bool sceneToJson(const Scene& scene, std::string& out, std::string& error)
{
    try {
        if (!validateScene(scene, error)) return false;
        std::string s;
        pretty(sceneJson(scene), 0, s);
        s += "\n";
        out = std::move(s);
        return true;
    } catch (const std::exception& e) {
        error = std::string("scene could not be written: ") + e.what();
    } catch (...) {
        error = "scene could not be written: unknown error";
    }
    return false;
}

bool loadSceneFile(const std::string& path, Scene& out, std::string& error)
{
    try {
        std::ifstream f(path, std::ios::binary);
        if (!f) {
            error = "cannot open scene file \"" + path + "\"";
            return false;
        }
        const std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        if (f.bad()) {
            error = "cannot read scene file \"" + path + "\"";
            return false;
        }
        if (!sceneFromJson(content, out, error)) {
            error = path + ": " + error;
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        error = std::string("cannot read scene file: ") + e.what();
    } catch (...) {
        error = "cannot read scene file: unknown error";
    }
    return false;
}

bool saveSceneFile(const std::string& path, const Scene& scene, std::string& error)
{
    try {
        std::string content;
        if (!sceneToJson(scene, content, error)) return false;
        std::ofstream f(path, std::ios::binary | std::ios::trunc);
        if (!f) {
            error = "cannot open \"" + path + "\" for writing";
            return false;
        }
        f.write(content.data(), static_cast<std::streamsize>(content.size()));
        f.flush();
        if (!f) {
            error = "writing \"" + path + "\" failed";
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        error = std::string("cannot write scene file: ") + e.what();
    } catch (...) {
        error = "cannot write scene file: unknown error";
    }
    return false;
}

} // namespace cosmos
