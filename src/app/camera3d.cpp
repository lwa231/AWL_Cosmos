#include "app/camera3d.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kNearFraction = 1e-4;

double smooth(double t) { return t * t * (3.0 - 2.0 * t); }
} // namespace

double angleDelta(double a, double b)
{
    double d = std::fmod(b - a, 2.0 * kPi);
    if (d <= -kPi) d += 2.0 * kPi;
    if (d > kPi) d -= 2.0 * kPi;
    return d;
}

void View3D::setOrientation(double yaw, double pitch)
{
    yaw_ = std::isfinite(yaw) ? std::remainder(yaw, 2.0 * kPi) : yaw_;
    if (std::isfinite(pitch)) pitch_ = std::clamp(pitch, kMinPitch, kTopPitch);
    animT_ = animLen_ = 0.0;
}

void View3D::orbitByPixels(double dx, double dy, double radPerPixel)
{
    setOrientation(yaw_ - dx * radPerPixel, pitch_ + dy * radPerPixel);
}

void View3D::setFovY(double radians)
{
    if (std::isfinite(radians)) fovY_ = std::clamp(radians, 10.0 * kPi / 180.0, 120.0 * kPi / 180.0);
}

bool View3D::isTopDown() const
{
    return pitch_ >= kTopPitch - 1e-12 && std::fabs(angleDelta(yaw_, kTopYaw)) < 1e-12;
}

bool View3D::isFlat() const
{
    return isTopDown() && projection_ == Projection::Orthographic && !animating() && flyOffset_.x == 0.0 &&
           flyOffset_.y == 0.0 && flyOffset_.z == 0.0;
}

void View3D::toggleTopDown(double seconds)
{
    // The target of a running animation counts as where we are, so pressing T twice quickly goes back.
    const bool atTop = animating() ? (toPitch_ >= kTopPitch - 1e-12) : isTopDown();
    fromYaw_ = yaw_;
    fromPitch_ = pitch_;
    if (atTop) {
        toYaw_ = savedYaw_;
        toPitch_ = savedPitch_;
    } else {
        if (!animating()) {
            savedYaw_ = yaw_;
            savedPitch_ = pitch_;
        }
        toYaw_ = kTopYaw;
        toPitch_ = kTopPitch;
    }
    animT_ = 0.0;
    animLen_ = seconds > 0.0 ? seconds : 0.0;
    if (animLen_ == 0.0) {
        yaw_ = toYaw_;
        pitch_ = toPitch_;
    }
}

bool View3D::update(double dtReal)
{
    if (!animating()) return false;
    animT_ = std::min(animLen_, animT_ + std::max(0.0, dtReal));
    const double s = smooth(animLen_ > 0.0 ? animT_ / animLen_ : 1.0);
    yaw_ = fromYaw_ + angleDelta(fromYaw_, toYaw_) * s;
    pitch_ = fromPitch_ + (toPitch_ - fromPitch_) * s;
    if (!animating()) {   // land exactly
        yaw_ = toYaw_;
        pitch_ = toPitch_;
    }
    return true;
}

void View3D::setFlying(bool on)
{
    flying_ = on;
    if (!on) flyOffset_ = {};
}

void View3D::fly(double forward, double rightAmt, double upAmt, double dtReal, double metresPerPixel, int viewportHeight)
{
    if (!flying_ || !(dtReal > 0.0)) return;
    const double speed = metresPerPixel * static_cast<double>(std::max(1, viewportHeight));
    const Vec3 f = viewDir() * -1.0;
    flyOffset_ += (f * forward + right() * rightAmt + Vec3{0.0, 0.0, 1.0} * upAmt) * (speed * dtReal);
}

Vec3 View3D::viewDir() const
{
    const double cp = std::cos(pitch_), sp = std::sin(pitch_);
    return {cp * std::cos(yaw_), cp * std::sin(yaw_), sp};
}

Vec3 View3D::right() const { return {-std::sin(yaw_), std::cos(yaw_), 0.0}; }

Vec3 View3D::up() const { return cross(viewDir(), right()); }

double View3D::eyeDistance(double metresPerPixel, int viewportHeight) const
{
    const double halfH = 0.5 * static_cast<double>(std::max(1, viewportHeight));
    return metresPerPixel * halfH / std::tan(0.5 * fovY_);
}

Projected View3D::project(const Vec3& rel, double metresPerPixel, int viewportHeight) const
{
    Projected out;
    const Vec3 p = rel - flyOffset_;
    const Vec3 d = viewDir(), r = right(), u = up();
    const double cx = dot(p, r), cy = dot(p, u), cz = dot(p, d);
    const double D = eyeDistance(metresPerPixel, viewportHeight);
    out.depth = D - cz;
    if (projection_ == Projection::Orthographic) {
        out.visible = true;
        out.pixelsPerMetre = 1.0 / metresPerPixel;
        out.x = cx * out.pixelsPerMetre;
        out.y = cy * out.pixelsPerMetre;
        return out;
    }
    if (!(out.depth > kNearFraction * D)) return out;
    out.visible = true;
    out.pixelsPerMetre = (D / metresPerPixel) / out.depth;   // focal length in px is D / mpp
    out.x = cx * out.pixelsPerMetre;
    out.y = cy * out.pixelsPerMetre;
    return out;
}

Ray View3D::rayThrough(double cx, double cy, double metresPerPixel, int viewportHeight) const
{
    const Vec3 d = viewDir(), r = right(), u = up();
    const double D = eyeDistance(metresPerPixel, viewportHeight);
    Ray ray;
    if (projection_ == Projection::Orthographic) {
        ray.origin = flyOffset_ + d * D + r * (cx * metresPerPixel) + u * (cy * metresPerPixel);
        ray.dir = d * -1.0;
        return ray;
    }
    ray.origin = flyOffset_ + d * D;
    // The point (cx, cy) px on the focal plane through the look-at point
    const Vec3 onPlane = flyOffset_ + r * (cx * metresPerPixel) + u * (cy * metresPerPixel);
    const Vec3 dir = onPlane - ray.origin;
    ray.dir = dir / norm(dir);
    return ray;
}

bool View3D::pickPlane(double cx, double cy, double metresPerPixel, int viewportHeight, double planeZ, Vec3& out) const
{
    const Ray ray = rayThrough(cx, cy, metresPerPixel, viewportHeight);
    if (std::fabs(ray.dir.z) < 1e-9) return false;
    const double t = (planeZ - ray.origin.z) / ray.dir.z;
    if (!(t > 0.0)) return false;
    out = ray.origin + ray.dir * t;
    return true;
}

} // namespace app
} // namespace cosmos
