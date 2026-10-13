# AWL Cosmos

## Goal

A real-time space simulation and renderer built from scratch in C++ and OpenGL. Stars, planets and moons attract each other with N-body gravity and are drawn from shared template meshes with sphere-lit shading. A controllable shuttle, richer shaders (starfield, glow) and more celestial bodies are planned.

## Requirements

| Requirement | Version | Notes |
|---|---|---|
| Windows | 10 | Developed and tested here |
| MSYS2 (UCRT64) | current | Provides the toolchain and GLFW |
| GCC / g++ | 16.2 | C++17 or newer |
| CMake | 3.21+ | 4.4 used |
| Ninja | 1.13 | Build backend |
| GLFW | 3.x | Window and input |
| OpenGL | 3.3 core | Needs a GPU and driver that support it |
| GLAD | bundled | Included in `external/glad`, nothing to install |

## Setup

1. Install the toolchain and GLFW from an MSYS2 UCRT64 shell:

   ```bash
   pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-glfw
   ```

2. Add `C:\msys64\ucrt64\bin` to your Windows `PATH`.

3. Configure the project (first time, or after editing `CMakeLists.txt`):

   ```bash
   cmake --preset default
   ```

4. Build:

   ```bash
   cmake --build --preset default
   ```

5. Run (launches from the exe's folder so `shaders/` is found):

   ```bash
   cmake --build build --target run
   ```
