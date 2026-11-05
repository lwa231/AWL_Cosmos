#pragma once
// GL-free helpers for the procedural body looks (V1.1 step 3): star temperatures and colours, the surface style of a
// body, which body in view gets the extra detail, and the accretion-disk orientation of a black hole. Visual only: the
// results go into the RenderFrame and never back into the simulation.
#include <cstdint>
#include <vector>
#include "app/render_frame.h"
#include "app/sim_driver.h"
#include "core/bodies.h"

namespace cosmos {
namespace app {

// Display colour of a black body at `kelvin` (sRGB-encoded, brightest channel 1). Interpolated in log T over a table of
// blackbody colours from 1000 K to 40000 K; clamped outside it.
RenderColour blackbodyColour(float kelvin);

// The temperature whose blackbody colour best matches `colour` (by chromaticity: each colour divided by its brightest
// channel). Lets a scene's authored star colour stand for its temperature; a white or grey colour gives about 6500 K.
float temperatureFromColour(const RenderColour& colour);

// Effective temperature of a star of kind `kind` with the given colour: neutron stars are hot (1e6 K, blue-white), other
// stars take the temperature of their colour. 0 for the non-emitting kinds.
float starTemperature(BodyKind kind, const RenderColour& colour);

// Fills the step-3 look of one body (style, temperature, seed, cloud cover, atmosphere rim) from its kind and data.
// `rb.colour` must already be set. Does not touch position, radius or the light flags.
void applyBodyVisual(const DisplayBody& body, RenderBody& rb, double displayTime);

// True for the kinds that can be the "hero" body that gets the most detail (planets, gas giants, dwarf planets, moons,
// asteroids: not stars or compact objects).
bool isPlanetClass(BodyKind kind);

// Index of the most massive planet-class body whose disc is in view and at least `minRadiusPx` across in radius, or -1.
// `rendered` holds the camera-relative positions of `bodies` in the same order (RenderFrame::bodies after fillFrame).
int pickDetailBody(const std::vector<DisplayBody>& bodies, const std::vector<RenderBody>& rendered, float metresPerPixel,
                   int viewportWidth, int viewportHeight, float minRadiusPx = 2.0f);

// Unit normal of a black hole's accretion disk: the direction of the angular momentum of the bodies around it, each
// weighted by its pull towards the hole (mu_j / r^2) so the nearby matter decides. +z (face-on, counter-clockwise) when
// nothing orbits it.
void diskNormal(const std::vector<DisplayBody>& bodies, std::size_t hole, float out[3]);

} // namespace app
} // namespace cosmos
