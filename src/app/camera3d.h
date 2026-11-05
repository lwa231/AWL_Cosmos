#pragma once
// 3D view on top of the 2D Camera (V1.1 plan step 5). GL-free and unit-tested.
//
// The 2D Camera (camera.h) keeps owning what it already does well: the look-at point (centre, follow, frames, pan), the
// zoom (metres per pixel) and picking. View3D adds an orientation (yaw, pitch), a projection (perspective or
// orthographic), an animated switch to top-down (T), a WASD free-fly offset and the projection of world points to the
// screen. The app projects on the CPU and hands the renderer screen-space positions, so the 2D renderer and every overlay
// pass draw a 3D scene unchanged (painter's order by depth). In top-down orthographic the projection is the identity of
// the 2D view, so the old behaviour is kept exactly (the app then skips the projection altogether, see isFlat()).
//
// Conventions
//  - World: x, y in the ecliptic plane, z up (right-handed). Points are given RELATIVE TO THE LOOK-AT POINT (double), so
//    1e11 m coordinates never meet float.
//  - The eye sits at distance D from the look-at point along d = (cos p cos y, cos p sin y, sin p). Screen right is
//    r = (-sin y, cos y, 0), screen up is u = d x r. Top-down is p = 90 deg, y = -90 deg: r = +x, u = +y, like the 2D view.
//  - Scale: D is chosen so that the plane through the look-at point, facing the eye, has exactly metresPerPixel, so
//    switching projection or tilting never jumps the zoom at the centre.
//  - Screen output is centred pixels (origin at the viewport centre, y up), the space Camera::worldToCentred uses.
#include <cstdint>
#include "core/vec3.h"

namespace cosmos {
namespace app {

enum class Projection { Orthographic, Perspective };

struct Projected {
    bool visible = false;        // in front of the near plane (perspective); always true in orthographic
    double x = 0.0, y = 0.0;     // centred pixels, y up
    double depth = 0.0;          // distance from the eye along the view direction [m] (larger = further away)
    double pixelsPerMetre = 0.0; // local scale at that depth (constant in orthographic)
};

struct Ray {
    Vec3 origin;                 // relative to the look-at point [m]
    Vec3 dir;                    // unit
};

class View3D {
public:
    static constexpr double kTopPitch = 1.5707963267948966;    // 90 deg
    static constexpr double kTopYaw = -1.5707963267948966;     // -90 deg
    static constexpr double kMinPitch = -1.5690509975429023;   // -89.9 deg
    static constexpr double kDefaultFovY = 0.8726646259971648; // 50 deg
    static constexpr double kTiltDefaultPitch = 0.6108652381980153;   // 35 deg: where the first 3D tilt goes

    // ---- orientation ----
    double yaw() const { return yaw_; }
    double pitch() const { return pitch_; }
    void setOrientation(double yaw, double pitch);   // pitch clamped to [kMinPitch, kTopPitch]; cancels an animation
    // Right-drag: dx, dy window pixels (x right, y down). Dragging right turns the scene with the cursor, dragging down
    // raises the eye towards top-down (like pulling the near edge of the scene down). `radPerPixel` is the sensitivity.
    void orbitByPixels(double dx, double dy, double radPerPixel = 0.005);

    // ---- projection ----
    Projection projection() const { return projection_; }
    void setProjection(Projection p) { projection_ = p; }
    void toggleProjection() { projection_ = projection_ == Projection::Perspective ? Projection::Orthographic : Projection::Perspective; }
    double fovY() const { return fovY_; }
    void setFovY(double radians);                    // clamped to [10, 120] deg

    // True when the view is exactly the 2D one: top-down, orthographic, not animating, no fly offset. The app then uses
    // the 2D path unchanged.
    bool isFlat() const;
    bool isTopDown() const;                          // pitch at 90 deg and yaw at -90 deg (any projection)

    // ---- T: animated top-down switch ----
    // From any tilt: animate to top-down (remembering the tilt). From top-down: animate back to the remembered tilt (the
    // default 35 deg tilt the first time). `seconds` is the animation length.
    void toggleTopDown(double seconds = 0.45);
    bool animating() const { return animT_ < animLen_; }
    // Advance the animation by real seconds (call once per frame). Returns true while the view changed.
    bool update(double dtReal);

    // ---- free fly (WASD) ----
    bool flying() const { return flying_; }
    void setFlying(bool on);
    // Offset of the eye from the orbit pivot, applied on top of the look-at point [m]. Zero when not flying.
    const Vec3& flyOffset() const { return flyOffset_; }
    void clearFlyOffset() { flyOffset_ = {}; }
    // forward/right/up in -1..1 (keys W-S, D-A, E-Q), over dtReal seconds. Speed = `metresPerPixel * viewportHeight` per
    // second (one screen height per second at the current zoom), so it works from a moon to the whole system.
    void fly(double forward, double right, double up, double dtReal, double metresPerPixel, int viewportHeight);

    // ---- basis and projection ----
    Vec3 viewDir() const;    // d: from the look-at point towards the eye (unit)
    Vec3 right() const;      // r
    Vec3 up() const;         // u
    // Eye distance for this zoom and viewport height (see Scale above).
    double eyeDistance(double metresPerPixel, int viewportHeight) const;
    // Project a point relative to the look-at point. Near plane: 1e-4 of the eye distance.
    Projected project(const Vec3& rel, double metresPerPixel, int viewportHeight) const;
    // The ray through a centred-pixel position (cx, cy), relative to the look-at point.
    Ray rayThrough(double cx, double cy, double metresPerPixel, int viewportHeight) const;
    // Where that ray meets the plane z = planeZ (relative to the look-at point's z, i.e. 0 = the plane through it).
    // False when the ray is parallel to the plane or the hit is behind the eye.
    bool pickPlane(double cx, double cy, double metresPerPixel, int viewportHeight, double planeZ, Vec3& out) const;

private:
    double yaw_ = kTopYaw;
    double pitch_ = kTopPitch;
    double fovY_ = kDefaultFovY;
    Projection projection_ = Projection::Orthographic;
    // remembered tilt for T
    double savedYaw_ = kTopYaw + 0.5235987755982988;   // 30 deg round from top-down's heading
    double savedPitch_ = kTiltDefaultPitch;
    // animation
    double fromYaw_ = 0.0, fromPitch_ = 0.0, toYaw_ = 0.0, toPitch_ = 0.0;
    double animT_ = 0.0, animLen_ = 0.0;
    bool flying_ = false;
    Vec3 flyOffset_;
};

// Shortest signed angle from a to b, in (-pi, pi].
double angleDelta(double a, double b);

} // namespace app
} // namespace cosmos
