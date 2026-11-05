cmake_minimum_required(VERSION 3.21)
project(AWL_Cosmos VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to a Debug build when none is chosen (the "default" preset also sets this)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# GLAD: build glad.c into a small static library
add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

# Dear ImGui (core + GLFW and OpenGL3 backends) and ImPlot, vendored in external/ (plan 16). Third-party code: built
# without our warning flags. The OpenGL3 backend uses its own bundled loader, so it does not depend on GLAD. The demo
# windows are left out (not compiled, ShowDemoWindow compiled out).
add_library(imgui STATIC
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
    external/implot/implot.cpp
    external/implot/implot_items.cpp)
target_include_directories(imgui PUBLIC external/imgui external/implot external/icons)
target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_DEMO_WINDOWS)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# cosmos_core: physics library. No OpenGL/GLFW here, so it can run headless (tests).
# Every .cpp in src/core is part of the library (CONFIGURE_DEPENDS picks up new files on the next build)
file(GLOB COSMOS_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/core/*.cpp)
add_library(cosmos_core STATIC ${COSMOS_CORE_SOURCES})
target_include_directories(cosmos_core PUBLIC src)
# nlohmann/json is vendored (header-only); scene_json.cpp uses it, so <nlohmann/json.hpp> resolves for core consumers
target_include_directories(cosmos_core PUBLIC external/json)
# PUBLIC so header-inline math in every consumer is compiled with the same float rules (determinism)
target_compile_options(cosmos_core PUBLIC -ffp-contract=off)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cosmos_core PRIVATE -Wall -Wextra)
endif()

# cosmos_app: clock, warp, simulation driver, interpolation, camera math (plan 4.1 app/). No GL, so tests can use it.
file(GLOB COSMOS_APP_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/app/*.cpp)
add_library(cosmos_app STATIC ${COSMOS_APP_SOURCES})
target_link_libraries(cosmos_app PUBLIC cosmos_core)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cosmos_app PRIVATE -Wall -Wextra)
endif()

# Headless tests: link only against core (no window, no GPU)
enable_testing()
file(GLOB COSMOS_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/tests/*.cpp)
# The developer's 3D core spec tests (V1.1 step 5, docs/V1_1_3D_CORE.md) call functions that are only declared in
# src/core/*3d.h until the developer writes them, so they are built only on request. COSMOS_3D_CORE_SPECS picks
# which spec files to build (by the part after "test_3d_core_", e.g. "kepler;placement"), so the core can be written
# and checked one header at a time; "all" builds every one. Every case in them is [3d-core] and skipped by default.
option(COSMOS_3D_CORE "Build the developer's 3D core spec tests" OFF)
set(COSMOS_3D_CORE_SPECS "all" CACHE STRING "3D core spec files to build when COSMOS_3D_CORE is ON (all, or a list like kepler;placement;merge;impact;hierarchy;swept)")
list(FILTER COSMOS_TEST_SOURCES EXCLUDE REGEX "/test_3d_core_[^/]*\\.cpp$")
if(COSMOS_3D_CORE)
    if(COSMOS_3D_CORE_SPECS STREQUAL "all")
        file(GLOB COSMOS_3D_CORE_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/tests/test_3d_core_*.cpp)
    else()
        set(COSMOS_3D_CORE_TEST_SOURCES "")
        foreach(spec ${COSMOS_3D_CORE_SPECS})
            list(APPEND COSMOS_3D_CORE_TEST_SOURCES ${CMAKE_SOURCE_DIR}/tests/test_3d_core_${spec}.cpp)
        endforeach()
    endif()
    list(APPEND COSMOS_TEST_SOURCES ${COSMOS_3D_CORE_TEST_SOURCES})
endif()
add_executable(cosmos_tests ${COSMOS_TEST_SOURCES})
if(COSMOS_3D_CORE)
    target_compile_definitions(cosmos_tests PRIVATE COSMOS_3D_CORE=1)
endif()
target_include_directories(cosmos_tests PRIVATE external/doctest)
# Tests find scenes/ through this absolute path (plan section 11)
target_compile_definitions(cosmos_tests PRIVATE COSMOS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(cosmos_tests PRIVATE cosmos_app cosmos_core)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cosmos_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME cosmos_tests COMMAND cosmos_tests)

# App executable: src/*.cpp plus the render/ui modules (plan section 4.1). GL lives here only.
file(GLOB AWL_APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/render/*.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/*.cpp)
add_executable(AWL_Cosmos ${AWL_APP_SOURCES})
target_include_directories(AWL_Cosmos PRIVATE src)

# Warnings for our own code only (not the bundled GLAD)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(AWL_Cosmos PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(AWL_Cosmos PRIVATE /W4)
endif()

# Copy each shader next to the exe. Every file is its own build output that DEPENDS on its source,
# so editing only a .vert/.frag re-copies it on the next build (no relink needed).
# CONFIGURE_DEPENDS re-checks the glob each build, so new shader files are picked up automatically.
file(GLOB SHADER_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/shaders/*)
set(SHADER_OUTPUTS "")
foreach(shader ${SHADER_SOURCES})
    get_filename_component(shader_name ${shader} NAME)
    set(shader_out ${CMAKE_BINARY_DIR}/shaders/${shader_name})
    add_custom_command(
        OUTPUT ${shader_out}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${shader} ${shader_out}
        DEPENDS ${shader}
        COMMENT "Copying shader ${shader_name}"
    )
    list(APPEND SHADER_OUTPUTS ${shader_out})
endforeach()
add_custom_target(copy_shaders ALL DEPENDS ${SHADER_OUTPUTS})
add_dependencies(AWL_Cosmos copy_shaders)

# Scenes next to the exe too (the app loads scenes/lucerna.json from its working folder, like shaders/)
file(GLOB SCENE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/scenes/*.json)
set(SCENE_OUTPUTS "")
foreach(scene ${SCENE_SOURCES})
    get_filename_component(scene_name ${scene} NAME)
    set(scene_out ${CMAKE_BINARY_DIR}/scenes/${scene_name})
    add_custom_command(
        OUTPUT ${scene_out}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${scene} ${scene_out}
        DEPENDS ${scene}
        COMMENT "Copying scene ${scene_name}"
    )
    list(APPEND SCENE_OUTPUTS ${scene_out})
endforeach()
add_custom_target(copy_scenes ALL DEPENDS ${SCENE_OUTPUTS})
add_dependencies(AWL_Cosmos copy_scenes)

# Fonts next to the exe too (Roboto + Font Awesome Solid, loaded from assets/fonts/ by the UI)
file(GLOB FONT_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/assets/fonts/*.ttf ${CMAKE_SOURCE_DIR}/assets/fonts/*.otf)
set(FONT_OUTPUTS "")
foreach(font ${FONT_SOURCES})
    get_filename_component(font_name ${font} NAME)
    set(font_out ${CMAKE_BINARY_DIR}/assets/fonts/${font_name})
    add_custom_command(
        OUTPUT ${font_out}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${font} ${font_out}
        DEPENDS ${font}
        COMMENT "Copying font ${font_name}"
    )
    list(APPEND FONT_OUTPUTS ${font_out})
endforeach()
add_custom_target(copy_fonts ALL DEPENDS ${FONT_OUTPUTS})
add_dependencies(AWL_Cosmos copy_fonts)

target_link_libraries(AWL_Cosmos PRIVATE cosmos_app cosmos_core imgui glad glfw OpenGL::GL)
if(WIN32)
    target_link_libraries(AWL_Cosmos PRIVATE winmm)   # timeBeginPeriod for the frame-rate cap
endif()

# Build and run from the exe's folder so shaders/ resolves:  cmake --build build --target run
add_custom_target(run
    COMMAND AWL_Cosmos
    WORKING_DIRECTORY $<TARGET_FILE_DIR:AWL_Cosmos>
    DEPENDS AWL_Cosmos
)
