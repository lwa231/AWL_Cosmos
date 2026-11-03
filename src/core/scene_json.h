#pragma once
#include <string>
#include "core/scene.h"

namespace cosmos {

// Scene JSON I/O (plan section 11.1) with nlohmann/json. None of these throw: on failure they return false and put a
// readable message (with the JSON path of the offending field, for example "bodies[3].orbit.a: expected a number") in error.
//
// Schema notes beyond the plan's example:
//  - "kind" is one of Star, Planet, GasGiant, DwarfPlanet, Moon, Asteroid, NeutronStar, BlackHole (matched ignoring case,
//    spaces, '_' and '-', so "Gas Giant" also loads). Saving writes the identifier form. Default: Planet.
//  - "spinPeriod_h" is signed: negative means a retrograde (clockwise) spin. Absent or 0 means no spin.
//  - A body or shuttle has either "orbit" or both "position" and "velocity"; giving both, or neither, is an error.
//    "orbit" needs "parent" and "a"; "e", "argPeriapsis_deg", "meanAnomaly_deg" and "retrograde" default to 0 / false.
//  - "id" on a body is optional on load: bodies without one get the smallest unused IDs, counting up from 1 in file order.
//    A shuttle's "id" is required (bodies and shuttles share one ID namespace).
//  - Unknown keys are ignored. Loaded scenes are checked with validateScene.

bool sceneFromJson(const std::string& text, Scene& out, std::string& error);
bool sceneToJson(const Scene& scene, std::string& out, std::string& error);

bool loadSceneFile(const std::string& path, Scene& out, std::string& error);
bool saveSceneFile(const std::string& path, const Scene& scene, std::string& error);

} // namespace cosmos
