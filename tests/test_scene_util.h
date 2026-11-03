#pragma once
#include <doctest.h>
#include <cstddef>
#include <string>
#include "core/scene.h"
#include "core/scene_json.h"

#ifndef COSMOS_SOURCE_DIR
#error "COSMOS_SOURCE_DIR must be defined by CMake (CMakeLists.txt, cosmos_tests target)"
#endif

namespace scene_test {

inline std::string sceneFile(const std::string& name)
{
    return std::string(COSMOS_SOURCE_DIR) + "/scenes/" + name;
}

// scenes/lucerna.json, loaded and resolved. Failing to do so fails the calling test case.
struct Lucerna {
    cosmos::Scene scene;
    cosmos::ResolvedScene res;

    Lucerna()
    {
        std::string err;
        REQUIRE_MESSAGE(cosmos::loadSceneFile(sceneFile("lucerna.json"), scene, err), err);
        REQUIRE_MESSAGE(cosmos::resolveScene(scene, res, err), err);
    }

    // Index into scene.bodies
    std::size_t sceneIndex(const std::string& name) const
    {
        for (std::size_t i = 0; i < scene.bodies.size(); ++i)
            if (scene.bodies[i].name == name) return i;
        FAIL("no body named " << name);
        return 0;
    }

    // Index into res.bodies (the SoA store)
    std::size_t storeIndex(const std::string& name) const
    {
        const auto idx = res.bodies.find(res.bodyIds[sceneIndex(name)]);
        REQUIRE(idx.has_value());
        return *idx;
    }

    const cosmos::SceneBody& body(const std::string& name) const { return scene.bodies[sceneIndex(name)]; }
};

} // namespace scene_test
