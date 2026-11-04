#pragma once
// Camera for the 2D view (plan 9.3, 9.4, 4.3 and M4). Pure logic: no GL, no GLFW, no ImGui.
//
// Conventions
//  - World positions are double metres in the view plane (x right, y up; z is ignored). The camera centre is double
//    too. The renderer gets p_cam = float(p_world - centreWorld) (toCameraRelative), so there is no jitter at 1e11 m
//    (plan 9.3): the subtraction happens in double and only the small difference is rounded to float.
//  - "Window pixels" are the GLFW cursor convention: origin top-left, x right, y DOWN. "Centred pixels" have the origin
//    at the viewport centre and y UP, which matches camera-relative metres divided by metresPerPixel.
//  - metresPerPixel = exp(zoomLog), clamped to [kMinMetresPerPixel, kMaxMetresPerPixel] (1 mm/px to 1e12 m/px).
//  - Positions of followed objects come from CameraSources callbacks evaluated at the render time t (the caller
//    interpolates, plan 6.6). Body and shuttle IDs share one allocator (plan 4.3), so an ID is unique across both.
#include "core/vec3.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace cosmos {
namespace app {

constexpr double kMinMetresPerPixel = 1e-3;     // closer than a 2 m shuttle needs
constexpr double kMaxMetresPerPixel = 1e12;     // a whole planetary system fits in a window
constexpr double kDefaultMetresPerPixel = 1e6;
constexpr double kZoomStepFactor = 1.25;        // one wheel notch changes metresPerPixel by this factor
constexpr double kMarkerDiameterPx = 3.0;       // plan 9.3: a body smaller than this is a minimum-size marker
constexpr double kMinPickRadiusPx = 8.0;        // plan 9.3: every object can be picked within this many pixels

enum class TargetKind { None, Body, Shuttle };

// Where objects and frame origins come from. Every callback may be empty (then it reports "not available").
// Position callbacks return false when the object does not exist at time t (merged away, destroyed).
struct CameraSources {
    std::function<bool(std::uint64_t id, double t, Vec3& out)> bodyPosition;
    std::function<bool(std::uint64_t id, double t, Vec3& out)> shuttlePosition;
    std::function<bool(std::uint64_t id, double t, Vec3& out)> bodyVelocity;     // for the follow-body frame velocity
    std::function<bool(double t, Vec3& out)> barycentre;                         // system barycentre position
    std::function<bool(double t, Vec3& out)> barycentreVelocity;
    // The body that absorbed `id` in a merge (plan 4.3), or 0 if `id` was not merged. May be followed transitively.
    std::function<std::uint64_t(std::uint64_t id)> mergeSurvivor;
};

// Reference frames (plan 9.3; V1 is translation only): the origin is the frame's zero point.
enum class FrameMode { Inertial, Barycentre, FollowBody };

struct Viewport {
    int width = 1280;       // pixels
    int height = 720;
};

struct Follow {
    TargetKind kind = TargetKind::None;
    std::uint64_t id = 0;
    bool active() const { return kind != TargetKind::None; }
};

// What Camera::update changed this call, so the UI can show a notice.
struct CameraUpdate {
    bool followTransferred = false;     // the target merged: now following its survivor (see follow())
    bool followLost = false;            // the target vanished with no survivor: no longer following
    bool frameTransferred = false;      // the follow-body frame's body merged: the frame now uses the survivor
    bool frameFellBack = false;         // the follow-body frame's body vanished with no survivor: frame is Inertial
};

class Camera {
public:
    // ---- zoom (logarithmic) ----
    double zoomLog() const { return zoomLog_; }
    double metresPerPixel() const;                  // exp(zoomLog), clamped (exact at the limits)
    double pixelsPerMetre() const { return 1.0 / metresPerPixel(); }
    void setZoomLog(double z);                      // clamped
    void setMetresPerPixel(double mpp);             // clamped
    // Wheel zoom: `steps` notches (positive = zoom in = fewer metres per pixel). The world point under the cursor
    // stays under the cursor. Returns false (and changes nothing) if the zoom was already at the limit.
    bool zoomAtWindow(double steps, double cursorX, double cursorY, const Viewport& vp);
    bool zoomAtCentred(double steps, double cursorX, double cursorY);

    // ---- centre and pan ----
    const Vec3& centreWorld() const { return centreWorld_; }   // as of the last update()/pan/set (z is 0)
    // Moves the camera so the centre is at `p` (absolute world). When following, this changes the pan offset.
    void setCentreWorld(const Vec3& p);
    void panWorld(double dx, double dy);            // move the centre by (dx, dy) metres
    // Drag: the cursor moved by (dxPx, dyPx) window pixels; the scene follows the cursor.
    void panByWindowPixels(double dxPx, double dyPx);
    const Vec3& panOffset() const { return panOffset_; }       // centre - target while following (world metres)

    // ---- mapping ----
    // Window pixels <-> world. The viewport centre is at (width/2, height/2).
    void worldToWindow(const Vec3& world, const Viewport& vp, double& px, double& py) const;
    Vec3 windowToWorld(double px, double py, const Viewport& vp) const;
    // Centred pixels (y up) <-> world.
    void worldToCentred(const Vec3& world, double& px, double& py) const;
    Vec3 centredToWorld(double px, double py) const;
    // Camera-relative float metres for the renderer (plan 9.3): the subtraction is done in double.
    struct Rel { float x = 0.0f, y = 0.0f; };
    Rel toCameraRelative(const Vec3& world) const;
    // Window pixels -> camera-relative metres (double), the space picking works in.
    void windowToRelative(double px, double py, const Viewport& vp, double& rx, double& ry) const;

    // ---- follow (plan 4.3, 9.3) ----
    const Follow& following() const { return follow_; }
    // Start following. The camera snaps so the target is at the centre (pan offset zero).
    void follow(TargetKind kind, std::uint64_t id);
    // Start following without moving the view: the pan offset becomes (current centre - targetPos).
    void followKeepingView(TargetKind kind, std::uint64_t id, const Vec3& targetPos);
    void stopFollowing();                           // the centre stays where it is

    // ---- reference frame (plan 9.3) ----
    FrameMode frameMode() const { return frameMode_; }
    std::uint64_t frameBody() const { return frameBody_; }
    // Switch frame without moving the view. FrameMode::FollowBody needs `bodyId` (not 0), else the call is ignored
    // and false is returned. Evaluated at time t.
    bool setFrame(FrameMode mode, std::uint64_t bodyId, double t, const CameraSources& src);
    // Frame origin (position and velocity) at time t, for re-expressing trails and velocity arrows (plan 9.5).
    // Returns false if the origin is unavailable (follow-body frame whose body does not exist at t); out is then 0.
    bool frameOrigin(double t, const CameraSources& src, Vec3& out) const;
    bool frameVelocity(double t, const CameraSources& src, Vec3& out) const;
    // A world point as seen in this frame at time t: p - origin(t).
    Vec3 toFrame(const Vec3& world, double t, const CameraSources& src) const;

    // ---- per-frame update ----
    // Call once per rendered frame with the render time t. Resolves the follow target (and the frame body), handles a
    // vanished target (survivor or stop) and recomputes centreWorld.
    CameraUpdate update(double t, const CameraSources& src);

private:
    bool targetPosition(TargetKind kind, std::uint64_t id, double t, const CameraSources& src, Vec3& out) const;
    void shiftCentre(double dx, double dy);

    double zoomLog_ = 13.815510557964274;           // log(kDefaultMetresPerPixel)
    Vec3 centreWorld_;                              // last resolved centre
    Vec3 panOffset_;                                // follow: centre - target
    Vec3 centreFrame_;                              // not following: centre - frame origin
    Vec3 lastOrigin_;                               // frame origin at the last update()
    Follow follow_;
    FrameMode frameMode_ = FrameMode::Inertial;
    std::uint64_t frameBody_ = 0;
};

// ---- picking (plan 9.3: on the CPU, against camera-relative positions) ----
struct PickCandidate {
    TargetKind kind = TargetKind::Body;
    std::uint64_t id = 0;
    float x = 0.0f, y = 0.0f;       // camera-relative position [m]
    float radius = 0.0f;            // physical radius [m] (0 for a point)
};

struct PickResult {
    bool hit = false;
    TargetKind kind = TargetKind::None;
    std::uint64_t id = 0;
    double distancePx = 0.0;        // cursor to object centre
};

// The nearest candidate whose pick disc contains the cursor. The pick radius is max(radius in px, minPickRadiusPx),
// so tiny bodies and 2 m shuttles stay clickable. "Nearest" is by distance from the cursor to the centre; ties go to
// the smaller pick radius, then the smaller id, so the result does not depend on candidate order.
// (relX, relY) is the cursor in camera-relative metres (Camera::windowToRelative).
PickResult pickNearest(const std::vector<PickCandidate>& candidates, double relX, double relY,
                       double metresPerPixel, double minPickRadiusPx = kMinPickRadiusPx);

// Minimum-size marker rule (plan 9.3): true if a body of this radius has an on-screen diameter below `thresholdPx`.
// The renderer applies the same rule (RenderFrame::bodyMinPixels); labels use it to decide when to draw a name.
bool isMarkerSized(double radiusMetres, double metresPerPixel, double thresholdPx = kMarkerDiameterPx);
// On-screen disc radius in pixels after the minimum-size rule (marker discs have radius thresholdPx / 2).
double drawnRadiusPx(double radiusMetres, double metresPerPixel, double thresholdPx = kMarkerDiameterPx);

} // namespace app
} // namespace cosmos
