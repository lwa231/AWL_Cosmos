#pragma once
// The scene editor model (plan sections 3 Q8/Q14/Q25, 5.9, 10.1, 12.1, 12.2). No OpenGL, no ImGui: the left panel only
// reads and calls this.
//
// What it holds: an editable core Scene (the document), the selection, and the counters that make the names and the IDs
// of new objects. Every change goes through a method that checks the result and either commits it or leaves the scene
// exactly as it was and returns an error string, so the scene in the model always passes validateScene.
//
// Enter = apply: a field edit is applied to the scene when the panel calls the setter (on Enter or when the field loses
// focus); the setter's result carries the error to show or the warnings to display. The paused/stopped simulation is then
// brought in line by RunControl::applyEdits (run_control.h), which rebuilds it from this scene.
//
// Run lock (plan Q19): while the model is locked (RunControl locks it while the simulation Runs) every mutating method
// fails with kLockedMessage and changes nothing. Selection is not an edit and always works.
//
// Names (plan 12.2, Q25): a new body is "<Kind> <n>" and a new shuttle "Shuttle <n>". The counters only ever count up, so a
// number is not reused after the object is deleted or renamed, nor after Stop discards a body added mid-run. Loading or
// restoring a scene raises the counters past every generic name it already contains ("Planet 3" makes the next Planet
// "Planet 4"). A template name ("Domus") matches no pattern and consumes nothing. Identity is the permanent ID.
//
// IDs: bodies and shuttles share one ID line (plan 4.3). New objects get the next number after every ID the model has seen;
// like the name counters this only grows, so an ID of a deleted object is not given out again within a session.
//
// Placement. A body or shuttle is either in STATE mode (position and velocity) or in ORBIT mode (parent, a, e, omega, M,
// prograde/retrograde). "Orbit around X at distance d" is orbit mode with a = d: resolveScene places it in Jacobi
// coordinates (plan 5.9), so the period is the one for mu_parent + mu_body (+ the inner moons of the same parent). The
// scene is resolved with resolveOptions(): Recentre::Auto for a design-time scene (plan 5.9 step 4), Recentre::Never for a
// scene captured from a run (RunControl sets that), so editing a paused simulation never moves the other bodies.
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>
#include "app/units.h"
#include "core/controllers.h"
#include "core/kinds.h"
#include "core/scene.h"

namespace cosmos {
namespace app {

inline constexpr const char* kLockedMessage = "Editing is only possible while the simulation is paused or stopped.";

struct EditResult {
    bool ok = true;
    std::string error;                  // when !ok: what to show next to the field
    std::vector<std::string> warnings;  // advisory, the change was made (dominance, overlap, periapsis inside the parent, ...)
    std::uint64_t id = 0;               // the object that was added or edited, 0 if none

    explicit operator bool() const { return ok; }
    static EditResult success(std::uint64_t id = 0)
    {
        EditResult r;
        r.id = id;
        return r;
    }
    static EditResult failure(std::string message)
    {
        EditResult r;
        r.ok = false;
        r.error = std::move(message);
        return r;
    }
};

enum class ObjectType { None, Body, Shuttle };

// A row of the object list.
struct ObjectRow {
    std::uint64_t id = 0;
    ObjectType type = ObjectType::Body;
    std::string name;
    BodyKind kind = BodyKind::Planet;   // bodies
    std::string controller;             // shuttles
};

// The text fields of the panel (typed with units, plan 10.1/10.2). Orbit fields apply in orbit mode, position and velocity in
// state mode.
enum class Field {
    Name,
    // bodies
    Mass, Radius, SpinPeriod, AtmoRho0, AtmoScaleHeight,
    // bodies and shuttles
    PosX, PosY, VelX, VelY,
    OrbitA, OrbitE, OrbitOmega, OrbitMeanAnomaly,
    // shuttles
    DryMass, FuelMass, MaxThrust, ExhaustVelocity, CdA, CollisionRadius, MaxTorque, Inertia
};

struct FieldInfo {
    const char* label;          // "Mass", "Semi-major axis a", ...
    Quantity quantity;          // what the text is parsed as (Number for Name)
    const char* defaultUnit;    // unit of a bare number: SI base units, except the spin period (h) and angles (deg)
    bool forBody;
    bool forShuttle;
};
FieldInfo fieldInfo(Field f);
// Whether the field exists for this kind of object (the panel shows only those)
bool fieldApplies(ObjectType type, Field f);

class EditorModel {
public:
    EditorModel();

    // ---- The document -------------------------------------------------------------------------------------------
    const Scene& scene() const { return scene_; }
    // Increases on every committed change of the scene (and on setScene/restoreScene/reset); selection does not count.
    std::uint64_t revision() const { return revision_; }
    // Increases on every committed USER edit only (add, delete, duplicate, any setter): unlike revision() it does not move on
    // setScene / restoreScene / reset, so a run (Pause captures the simulation, Stop restores the snapshot) is not an edit.
    // The app compares it with the value at the last load or save to know whether there are unsaved edits (V1.1 step 1).
    std::uint64_t editCount() const { return edits_; }

    // A new document: replaces the scene, clears the selection, recomputes the name counters and the ID counter from the
    // scene (they restart; use restoreScene to keep them), sets Recentre::Auto. Works even when locked (it is not an edit of
    // the running simulation; RunControl stops the run first).
    void setScene(Scene scene);
    // The same document in another state (Stop restores the snapshot, Pause captures the run): replaces the scene, keeps the
    // selection if that ID still exists, and never lowers the counters. reserveIdsUpTo: IDs up to and including this one are
    // not given out again (IDs of shuttles that were destroyed during a run are no longer in the scene).
    void restoreScene(Scene scene, std::uint64_t reserveIdsUpTo = 0);
    // Empty scene, counters back to zero.
    void reset();

    // How the scene is resolved to states (see the file comment)
    Recentre recentre() const { return recentre_; }
    void setRecentre(Recentre r);
    ResolveOptions resolveOptions() const;

    // Locked = mutators fail (kLockedMessage).
    void setLocked(bool locked) { locked_ = locked; }
    bool locked() const { return locked_; }

    // ---- Selection ----------------------------------------------------------------------------------------------
    std::uint64_t selected() const { return selected_; }
    ObjectType selectedType() const { return typeOf(selected_); }
    bool select(std::uint64_t id);   // false (selection unchanged) when no object has this ID
    void clearSelection() { selected_ = 0; }

    // ---- Queries ------------------------------------------------------------------------------------------------
    ObjectType typeOf(std::uint64_t id) const;
    const SceneBody* body(std::uint64_t id) const;
    const SceneShuttle* shuttle(std::uint64_t id) const;
    std::vector<ObjectRow> objects() const;   // bodies in scene order, then shuttles
    bool inOrbitMode(std::uint64_t id) const;
    // Position and velocity the object will start with (resolved: orbit placed, scene recentred per resolveOptions()).
    // false when the id is unknown or the scene cannot be resolved.
    bool resolvedState(std::uint64_t id, Vec3& position, Vec3& velocity) const;
    // validateScene plus a resolve; true when the scene can be started
    bool validate(std::string& error) const;
    // A distance for "orbit around X": 4 radii of the parent for a body, just above the atmosphere (1.25 radii if none) for
    // a shuttle. 0 when the parent is not a body.
    double suggestedOrbitDistance(std::uint64_t parentId, ObjectType forType) const;
    // Last number handed out for the kind / for shuttles (see Names above)
    std::uint64_t namesIssued(BodyKind kind) const { return names_.issued(kind); }
    std::uint64_t shuttleNamesIssued() const { return shuttleNames_; }
    std::uint64_t nextId() const { return nextId_; }

    // ---- Warnings (advisory; also returned in the EditResult of the change that caused them) -------------------------
    // Plan 10.1: the body's mu is more than 10% of the most massive OTHER body's mu. Empty if not, or if it is alone. (For
    // the heaviest body this compares with the second heaviest, so it is nearly always true there; the EditResult of an edit
    // carries the warning only when the edit made the body dominant: a new body, or a mass change from below the line to above.)
    std::string dominanceWarning(std::uint64_t id) const;
    // Overlapping at the start (bodies: centres closer than the radii sum; shuttles: inside a body); periapsis of the orbit
    // inside the parent; a name used twice. Dominance is not included.
    std::vector<std::string> placementWarnings(std::uint64_t id) const;

    // ---- Adding and deleting ------------------------------------------------------------------------------------
    // Defaults of table 12.1 (kinds.h), generic name, selected afterwards. State mode, at the position with the velocity.
    // Result id = the new ID; warnings carry the dominance and overlap checks.
    EditResult addBody(BodyKind kind, Vec3 position = {}, Vec3 velocity = {});
    // Defaults of table 7.1, name "Shuttle <n>", state mode, selected afterwards.
    EditResult addShuttle(ControllerKind controller = ControllerKind::Coast, Vec3 position = {}, Vec3 velocity = {});
    // Deletes a body or a shuttle. Whatever orbited a deleted body (bodies and shuttles in orbit mode with it as parent)
    // keeps the position and velocity it was placed at (before the recentring of 5.9 step 4) and switches to state mode, so
    // the geometry of the system is unchanged. Clears the selection if it was the object.
    EditResult remove(std::uint64_t id);
    // A copy of a body or shuttle with a new ID and the name "<name> copy" ("<name> copy 2", ... when taken), selected
    // afterwards. In orbit mode the copy has the same elements half an orbit further on (mean anomaly + 180 deg), so it does not
    // start on top of the original; in state mode it starts beside it (a body 4 radii away along +x, a shuttle 1 km) with
    // the same velocity. Satellites of the original are not copied. Result id = the new ID, warnings as for addBody.
    EditResult duplicate(std::uint64_t id);
    // The name a duplicate of `name` gets (see duplicate): unused by any body or shuttle, at most 64 characters.
    std::string copyName(const std::string& name) const;

    // ---- Editing ------------------------------------------------------------------------------------------------
    EditResult setName(std::uint64_t id, std::string_view name);       // trimmed, 1..64 characters
    // Changing the kind keeps mass and radius unless applyDefaults, which also takes the table-12.1 mass, radius and colour.
    EditResult setKind(std::uint64_t id, BodyKind kind, bool applyDefaults = false);
    EditResult setMu(std::uint64_t id, double mu);                     // m^3/s^2 (stored value)
    EditResult setMassKg(std::uint64_t id, double kg);                 // mu = G * kg
    EditResult setRadius(std::uint64_t id, double radius);             // m, > 0
    EditResult setColour(std::uint64_t id, const std::array<float, 3>& rgb);   // each in [0, 1]
    EditResult setSpinPeriodHours(std::uint64_t id, double hours);     // signed; negative = retrograde, 0 = no spin
    EditResult setAtmosphere(std::uint64_t id, const std::optional<Atmosphere>& atmosphere);   // rho0 >= 0, H > 0; nullopt = none
    // Toggle: on = Earth-like defaults (1.225 kg/m^3, 8500 m) unless it already has one
    EditResult enableAtmosphere(std::uint64_t id, bool on);

    // State mode: position and velocity (drops the orbit)
    EditResult setState(std::uint64_t id, const Vec3& position, const Vec3& velocity);
    // Orbit mode with these elements (a > 0, 0 <= e < 1, parent a body other than the object and not one of its own
    // satellites)
    EditResult setOrbit(std::uint64_t id, const SceneOrbit& orbit);
    // "Orbit around X at distance d" (plan 5.9): a = distance. If the object already orbits this parent the other elements
    // stay, otherwise e = 0, omega = 0, M = 0, prograde.
    EditResult setOrbitAround(std::uint64_t id, std::uint64_t parentId, double distance);
    // Orbit mode -> state mode, keeping the state it was placed at (before the recentring of 5.9 step 4, so the geometry
    // of the system is unchanged). No-op in state mode.
    EditResult bakeState(std::uint64_t id);
    EditResult setController(std::uint64_t id, ControllerKind kind);   // shuttles

    // Any other change: fn edits a copy of the body (or shuttle); the copy is validated and, only if it is fine, committed.
    // The id of the object cannot be changed by fn.
    EditResult updateBody(std::uint64_t id, const std::function<void(SceneBody&)>& fn);
    EditResult updateShuttle(std::uint64_t id, const std::function<void(SceneShuttle&)>& fn);

    // ---- Text fields (units grammar, plan 10.2) -----------------------------------------------------------------------
    // The text a field shows now ("" when the field does not apply to the object or is not set: orbit fields in state mode,
    // atmosphere fields without an atmosphere). In orbit mode the position and velocity fields show the resolved state.
    std::string fieldText(std::uint64_t id, Field f) const;
    // Parses the text and applies it. A text identical to fieldText(id, f) changes nothing (so pressing Enter in an untouched
    // field never rounds the value). Errors: unknown object, field does not apply, unit/number errors, out-of-range value,
    // the object is in the other placement mode.
    EditResult setFieldText(std::uint64_t id, Field f, std::string_view text);

private:
    struct ResolvedCache {
        bool valid = false;
        std::uint64_t revision = 0;
        Recentre recentre = Recentre::Auto;
        bool ok = false;
        ResolvedScene res;
    };

    int bodyIndex(std::uint64_t id) const { return findSceneBody(scene_, id); }
    int shuttleIndex(std::uint64_t id) const;
    const ResolvedScene* resolved() const;
    void bump();
    void noteNames(const Scene& s);
    void noteIds(const Scene& s, std::uint64_t reserve);
    std::uint64_t takeId() { return nextId_++; }
    EditResult commitBody(std::uint64_t id, const std::function<void(SceneBody&)>& fn);
    EditResult commitShuttle(std::uint64_t id, const std::function<void(SceneShuttle&)>& fn);
    void finish(EditResult& r, std::uint64_t id, bool dominance) const;
    const SceneBody* heaviestOther(std::uint64_t id) const;
    bool dominates(std::uint64_t id, double mu) const;   // mu is more than 10% of the heaviest body other than id

    Scene scene_;
    std::uint64_t revision_ = 1;
    std::uint64_t edits_ = 0;
    std::uint64_t selected_ = 0;
    bool locked_ = false;
    Recentre recentre_ = Recentre::Auto;
    NameCounter names_;
    std::uint64_t shuttleNames_ = 0;
    std::uint64_t nextId_ = 1;
    mutable ResolvedCache cache_;
};

} // namespace app
} // namespace cosmos
