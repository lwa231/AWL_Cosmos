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

# Headless tests: link only against core (no window, no GPU)
enable_testing()
file(GLOB COSMOS_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/tests/*.cpp)
add_executable(cosmos_tests ${COSMOS_TEST_SOURCES})
target_include_directories(cosmos_tests PRIVATE external/doctest)
# Tests find scenes/ through this absolute path (plan section 11)
target_compile_definitions(cosmos_tests PRIVATE COSMOS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(cosmos_tests PRIVATE cosmos_core)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cosmos_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME cosmos_tests COMMAND cosmos_tests)

# App: src/*.cpp plus the app/render/ui modules (plan section 4.1)
file(GLOB AWL_APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/app/*.cpp
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

target_link_libraries(AWL_Cosmos PRIVATE cosmos_core glad glfw OpenGL::GL)

# Build and run from the exe's folder so shaders/ resolves:  cmake --build build --target run
add_custom_target(run
    COMMAND AWL_Cosmos
    WORKING_DIRECTORY $<TARGET_FILE_DIR:AWL_Cosmos>
    DEPENDS AWL_Cosmos
)
