# AWL Cosmos
3D Cosmos Renderer (currently a 2D OpenGL prototype: planets, moons and shuttles drawn from shared template meshes).

## Requirements

| Requirement | Version used | Notes |
|---|---|---|
| Windows | 10 | Developed on Windows 10 |
| MSYS2 (UCRT64 environment) | current | Provides the toolchain and GLFW |
| GCC / g++ | 16.2 (C++17 or newer) | `mingw-w64-ucrt-x86_64-gcc` |
| CMake | 3.10+ (4.4 used) | `mingw-w64-ucrt-x86_64-cmake` |
| Ninja | 1.13 | `mingw-w64-ucrt-x86_64-ninja` |
| GLFW | 3.x | `mingw-w64-ucrt-x86_64-glfw`, found via `find_package(glfw3)` |
| OpenGL | 3.3 core profile | Needs a GPU/driver that supports it |
| GLAD | OpenGL 3.3 core loader | Bundled in `external/glad` (no install needed) |

Install the toolchain and GLFW from an MSYS2 UCRT64 shell:

```bash
pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-glfw
```

Add `C:\msys64\ucrt64\bin` to your Windows `PATH` so `cmake`, `ninja`, `g++` and the GLFW DLL are found from PowerShell or an IDE.

## Build and run

Configure (first time, or after editing `CMakeLists.txt`):

```bash
cmake --preset default
```

Build:

```bash
cmake --build --preset default
```

Build and run (launches from the exe's folder so `shaders/` resolves):

```bash
cmake --build build --target run
```

## Project layout

```
CMakeLists.txt        build definition (also the `run` target)
CMakePresets.json     "default" preset: Ninja, Debug, ./build
external/glad/        bundled OpenGL loader
shaders/              GLSL files (copied next to the exe on every build)
src/main.cpp          window, loop, uniforms
src/GraphicsObData.*  GOD: shared GPU buffers for template shapes
src/Shader.*          shader file loading, compiling and linking
```
