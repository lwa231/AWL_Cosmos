#include "app/camera.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

namespace {

// log of the zoom limits, computed once; the limits themselves are returned exactly (no exp round-off at the ends).
const double kZoomLogMin = std::log(kMinMetresPerPixel);
const double kZoomLogMax = std::log(kMaxMetresPerPixel);

constexpr int kMaxSurvivorChain = 16;   // a survivor may itself merge later; stop on a (corrupt) cycle

Vec3 planar(const Vec3& v) { return {v.x, v.y, 0.0}; }

// Find the live body for `id`, following merge survivors (plan 4.3). `outId` is the body finally found.
bool resolveBody(std::uint64_t id, double t, const CameraSources& src, std::uint64_t& outId, Vec3& pos)
{
    for (int hop = 0; hop < kMaxSurvivorChain; ++hop) {
        if (src.bodyPosition && src.bodyPosition(id, t, pos)) {
            outId = id;
            return true;
        }
        const std::uint64_t s = src.mergeSurvivor ? src.mergeSurvivor(id) : 0;
        if (s == 0 || s == id) return false;
        id = s;
    }
    return false;
}

} // namespace

// ---------------------------------------------------------------- zoom

double Camera::metresPerPixel() const
{
    if (zoomLog_ <= kZoomLogMin) return kMinMetresPerPixel;
    if (zoomLog_ >= kZoomLogMax) return kMaxMetresPerPixel;
    return std::exp(zoomLog_);
}

void Camera::setZoomLog(double z)
{
    if (!std::isfinite(z)) return;
    zoomLog_ = std::min(std::max(z, kZoomLogMin), kZoomLogMax);
}

void Camera::setMetresPerPixel(double mpp)
{
    if (!(mpp > 0.0) || !std::isfinite(mpp)) return;
    setZoomLog(std::log(mpp));
}

bool Camera::zoomAtCentred(double steps, double cursorX, double cursorY)
{
    if (steps == 0.0 || !std::isfinite(steps)) return false;
    const double before = zoomLog_;
    const double mppBefore = metresPerPixel();
    // Zooming in (positive steps) lowers log(mpp) by steps * log(factor).
    setZoomLog(zoomLog_ - steps * std::log(kZoomStepFactor));
    if (zoomLog_ == before) return false;
    // The world point under the cursor is centre + cursor * mpp. Keep it fixed: centre += cursor * (mppOld - mppNew).
    const double k = mppBefore - metresPerPixel();
    shiftCentre(cursorX * k, cursorY * k);
    return true;
}

bool Camera::zoomAtWindow(double steps, double cursorX, double cursorY, const Viewport& vp)
{
    return zoomAtCentred(steps, cursorX - 0.5 * vp.width, 0.5 * vp.height - cursorY);
}

// ---------------------------------------------------------------- centre and pan

void Camera::shiftCentre(double dx, double dy)
{
    // The same displacement goes into whichever offset the centre is defined by (plan 9.3: pan offset relative to
    // the target while following), and into the cached centre.
    Vec3& off = follow_.active() ? panOffset_ : centreFrame_;
    off.x += dx;
    off.y += dy;
    centreWorld_.x += dx;
    centreWorld_.y += dy;
}

void Camera::setCentreWorld(const Vec3& p)
{
    const double dx = p.x - centreWorld_.x;
    const double dy = p.y - centreWorld_.y;
    Vec3& off = follow_.active() ? panOffset_ : centreFrame_;
    off.x += dx;
    off.y += dy;
    centreWorld_ = planar(p);
}

void Camera::panWorld(double dx, double dy)
{
    if (!std::isfinite(dx) || !std::isfinite(dy)) return;
    shiftCentre(dx, dy);
}

void Camera::panByWindowPixels(double dxPx, double dyPx)
{
    const double mpp = metresPerPixel();
    // Dragging right moves the scene right, so the camera moves left; window y is down, world y is up.
    panWorld(-dxPx * mpp, dyPx * mpp);
}

// ---------------------------------------------------------------- mapping

void Camera::worldToCentred(const Vec3& world, double& px, double& py) const
{
    const double inv = 1.0 / metresPerPixel();
    px = (world.x - centreWorld_.x) * inv;
    py = (world.y - centreWorld_.y) * inv;
}

Vec3 Camera::centredToWorld(double px, double py) const
{
    const double mpp = metresPerPixel();
    return {centreWorld_.x + px * mpp, centreWorld_.y + py * mpp, 0.0};
}

void Camera::worldToWindow(const Vec3& world, const Viewport& vp, double& px, double& py) const
{
    double cx = 0.0, cy = 0.0;
    worldToCentred(world, cx, cy);
    px = 0.5 * vp.width + cx;
    py = 0.5 * vp.height - cy;
}

Vec3 Camera::windowToWorld(double px, double py, const Viewport& vp) const
{
    return centredToWorld(px - 0.5 * vp.width, 0.5 * vp.height - py);
}

Camera::Rel Camera::toCameraRelative(const Vec3& world) const
{
    // Subtract in double, round to float afterwards (plan 9.3).
    return {static_cast<float>(world.x - centreWorld_.x), static_cast<float>(world.y - centreWorld_.y)};
}

void Camera::windowToRelative(double px, double py, const Viewport& vp, double& rx, double& ry) const
{
    const double mpp = metresPerPixel();
    rx = (px - 0.5 * vp.width) * mpp;
    ry = (0.5 * vp.height - py) * mpp;
}

// ---------------------------------------------------------------- follow

void Camera::follow(TargetKind kind, std::uint64_t id)
{
    if (kind == TargetKind::None || id == 0) {
        stopFollowing();
        return;
    }
    follow_ = {kind, id};
    panOffset_ = {};    // the next update() puts the target at the centre
}

void Camera::followKeepingView(TargetKind kind, std::uint64_t id, const Vec3& targetPos)
{
    if (kind == TargetKind::None || id == 0) {
        stopFollowing();
        return;
    }
    follow_ = {kind, id};
    panOffset_ = {centreWorld_.x - targetPos.x, centreWorld_.y - targetPos.y, 0.0};
}

void Camera::stopFollowing()
{
    if (!follow_.active()) return;
    follow_ = {};
    panOffset_ = {};
    centreFrame_ = {centreWorld_.x - lastOrigin_.x, centreWorld_.y - lastOrigin_.y, 0.0};
}

// ---------------------------------------------------------------- reference frame

bool Camera::frameOrigin(double t, const CameraSources& src, Vec3& out) const
{
    out = {};
    Vec3 p;
    switch (frameMode_) {
    case FrameMode::Inertial:
        return true;
    case FrameMode::Barycentre:
        if (!src.barycentre || !src.barycentre(t, p)) return false;
        break;
    case FrameMode::FollowBody:
        if (!src.bodyPosition || !src.bodyPosition(frameBody_, t, p)) return false;
        break;
    }
    out = planar(p);
    return true;
}

bool Camera::frameVelocity(double t, const CameraSources& src, Vec3& out) const
{
    out = {};
    Vec3 v;
    switch (frameMode_) {
    case FrameMode::Inertial:
        return true;
    case FrameMode::Barycentre:
        if (!src.barycentreVelocity || !src.barycentreVelocity(t, v)) return false;
        break;
    case FrameMode::FollowBody:
        if (!src.bodyVelocity || !src.bodyVelocity(frameBody_, t, v)) return false;
        break;
    }
    out = planar(v);
    return true;
}

Vec3 Camera::toFrame(const Vec3& world, double t, const CameraSources& src) const
{
    Vec3 o;
    frameOrigin(t, src, o);
    return {world.x - o.x, world.y - o.y, 0.0};
}

bool Camera::setFrame(FrameMode mode, std::uint64_t bodyId, double t, const CameraSources& src)
{
    if (mode == FrameMode::FollowBody && bodyId == 0) return false;
    const FrameMode oldMode = frameMode_;
    const std::uint64_t oldBody = frameBody_;
    frameMode_ = mode;
    frameBody_ = mode == FrameMode::FollowBody ? bodyId : 0;
    Vec3 origin;
    if (mode == FrameMode::FollowBody && !frameOrigin(t, src, origin)) {   // the body does not exist: keep the old frame
        frameMode_ = oldMode;
        frameBody_ = oldBody;
        return false;
    }
    frameOrigin(t, src, origin);
    // Re-express the centre in the new frame so the view does not move.
    centreFrame_ = {centreWorld_.x - origin.x, centreWorld_.y - origin.y, 0.0};
    lastOrigin_ = origin;
    return true;
}

// ---------------------------------------------------------------- update

bool Camera::targetPosition(TargetKind kind, std::uint64_t id, double t, const CameraSources& src, Vec3& out) const
{
    if (kind == TargetKind::Body) return src.bodyPosition && src.bodyPosition(id, t, out);
    if (kind == TargetKind::Shuttle) return src.shuttlePosition && src.shuttlePosition(id, t, out);
    return false;
}

CameraUpdate Camera::update(double t, const CameraSources& src)
{
    CameraUpdate res;

    // The frame body may have merged or vanished (plan 4.3: frame references transfer to the survivor).
    bool frameChanged = false;
    if (frameMode_ == FrameMode::FollowBody) {
        std::uint64_t found = 0;
        Vec3 p;
        if (resolveBody(frameBody_, t, src, found, p)) {
            if (found != frameBody_) {
                frameBody_ = found;
                res.frameTransferred = true;
                frameChanged = true;
            }
        } else {
            frameMode_ = FrameMode::Inertial;
            frameBody_ = 0;
            res.frameFellBack = true;
            frameChanged = true;
        }
    }

    Vec3 origin;
    frameOrigin(t, src, origin);
    if (frameChanged) centreFrame_ = {centreWorld_.x - origin.x, centreWorld_.y - origin.y, 0.0};   // no jump

    if (follow_.active()) {
        Vec3 tp;
        bool ok = false;
        if (follow_.kind == TargetKind::Body) {
            std::uint64_t found = 0;
            ok = resolveBody(follow_.id, t, src, found, tp);
            if (ok && found != follow_.id) {
                // Followed body merged into `found`: keep following it, without moving the view.
                follow_.id = found;
                panOffset_ = {centreWorld_.x - tp.x, centreWorld_.y - tp.y, 0.0};
                res.followTransferred = true;
            }
        } else {
            ok = targetPosition(follow_.kind, follow_.id, t, src, tp);   // shuttles are destroyed, never merged
        }
        if (ok) {
            centreWorld_ = {tp.x + panOffset_.x, tp.y + panOffset_.y, 0.0};
        } else {
            // Target gone and no survivor: stop following, the centre stays where it was.
            follow_ = {};
            panOffset_ = {};
            centreFrame_ = {centreWorld_.x - origin.x, centreWorld_.y - origin.y, 0.0};
            res.followLost = true;
        }
    } else {
        centreWorld_ = {origin.x + centreFrame_.x, origin.y + centreFrame_.y, 0.0};
    }
    lastOrigin_ = origin;
    return res;
}

// ---------------------------------------------------------------- picking and marker rule

PickResult pickNearest(const std::vector<PickCandidate>& candidates, double relX, double relY,
                       double metresPerPixel, double minPickRadiusPx)
{
    PickResult best;
    if (!(metresPerPixel > 0.0)) return best;
    double bestRadiusPx = 0.0;
    const double inv = 1.0 / metresPerPixel;
    for (const PickCandidate& c : candidates) {
        const double dx = (static_cast<double>(c.x) - relX) * inv;
        const double dy = (static_cast<double>(c.y) - relY) * inv;
        const double d = std::sqrt(dx * dx + dy * dy);
        const double radiusPx = std::max(static_cast<double>(c.radius) * inv, minPickRadiusPx);
        if (!(d <= radiusPx)) continue;     // also rejects NaN
        bool better = !best.hit || d < best.distancePx;
        if (best.hit && d == best.distancePx) {
            better = radiusPx < bestRadiusPx || (radiusPx == bestRadiusPx && c.id < best.id);
        }
        if (better) {
            best = {true, c.kind, c.id, d};
            bestRadiusPx = radiusPx;
        }
    }
    return best;
}

bool isMarkerSized(double radiusMetres, double metresPerPixel, double thresholdPx)
{
    return 2.0 * radiusMetres / metresPerPixel < thresholdPx;
}

double drawnRadiusPx(double radiusMetres, double metresPerPixel, double thresholdPx)
{
    return std::max(radiusMetres / metresPerPixel, 0.5 * thresholdPx);
}

} // namespace app
} // namespace cosmos
