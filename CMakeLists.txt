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

add_executable(AWL_Cosmos
    src/main.cpp
    src/Shader.cpp
    src/GraphicsObData.cpp
    src/SimulationLogic.cpp
)
target_include_directories(AWL_Cosmos PRIVATE src)

# Warnings for our own code only (not the bundled GLAD)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(AWL_Cosmos PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(AWL_Cosmos PRIVATE /W4)
endif()

# Copy shaders next to the exe after every build
add_custom_command(TARGET AWL_Cosmos POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders $<TARGET_FILE_DIR:AWL_Cosmos>/shaders
)

target_link_libraries(AWL_Cosmos PRIVATE glad glfw OpenGL::GL)

# Build and run from the exe's folder so shaders/ resolves:  cmake --build build --target run
add_custom_target(run
    COMMAND AWL_Cosmos
    WORKING_DIRECTORY $<TARGET_FILE_DIR:AWL_Cosmos>
    DEPENDS AWL_Cosmos
)
