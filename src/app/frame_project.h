#pragma once
// 3D views of a RenderFrame (V1.1 plan step 5). GL-free.
//
// The app fills the frame exactly as in 2D (camera-relative metres, z in the new z fields, 0 while the core is 2D) and,
// when the View3D is not flat, calls projectFrame: every position is projected on the CPU and written back as
// "screen metres" (centred pixels times metresPerPixel), every radius and pixel-sized thing is scaled by the local
// perspective factor, bodies are sorted back to front (painter's order: the renderer has no depth buffer), circles
// become projected line loops, and things behind the eye are dropped. The 2D renderer then draws the result unchanged.
//
// Also here: the ecliptic reference grid and the warped grid sheet that replaces the gravity-well rings in tilted views.
#include <cstdint>
#include <vector>
#include "app/camera3d.h"
#include "app/render_frame.h"
#include "core/vec3.h"

namespace cosmos {
namespace app {

struct ProjectContext {
    const View3D* view = nullptr;
    double metresPerPixel = 1.0;
    int viewportWidth = 1280;
    int viewportHeight = 720;
};

// Screen metres of a camera-relative point; false when it is behind the eye.
bool projectToScreen(const ProjectContext& c, const Vec3& rel, double& sx, double& sy, double& scale, double& depth);

// The whole frame, in place. `bodyOrder` (optional) receives, for each body now in frame.bodies, its index before the
// projection (so labels and picking can map back).
void projectFrame(RenderFrame& frame, const ProjectContext& c, std::vector<std::uint32_t>* bodyOrder = nullptr);

// Ecliptic reference grid: lines of the plane z = planeZ (camera-relative) around the look-at point, spaced at a power of
// ten that suits the zoom, snapped to world multiples of the spacing (lookAtWorld is the look-at point in world metres, so
// the grid does not swim when the camera pans). Appended to `out` in camera-relative metres, before projection.
void appendEclipticGrid(LineBatch& out, const ProjectContext& c, const Vec3& lookAtWorld, double planeZ = 0.0);

// Gravity wells as a warped sheet: a grid in the plane z = 0 displaced downwards by the depth of the potential of
// `wells` (log scale, Phi = -sum mu / max(r, R) like the ring pass). Appended in camera-relative metres, before projection.
void appendWellSheet(LineBatch& out, const std::vector<RenderWell>& wells, const ProjectContext& c, const Vec3& lookAtWorld,
                     int lines = 41);

// Sheet depth at a camera-relative point for those wells (exposed for tests): <= 0, 0 far from every mass, and
// at most `depthMax` deep. uEdge is the log-potential at which the sheet starts to dip.
double wellSheetDepth(const std::vector<RenderWell>& wells, double x, double y, double uEdge, double depthMax);

} // namespace app
} // namespace cosmos
