cmake_minimum_required(VERSION 3.10.0)
project(AWL_Cosmos VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# GLAD: build glad.c into a small static library
add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

add_executable(AWL_Cosmos
    src/main.cpp
    src/Shader.cpp
    src/GraphicsObData.cpp
)
target_include_directories(AWL_Cosmos PRIVATE src)

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
