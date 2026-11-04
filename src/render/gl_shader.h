#pragma once
// GLSL program loading for the OpenGL renderer.
#include <string>

namespace cosmos {
namespace render {

// Reads `<dir>/<vertFile>` and `<dir>/<fragFile>`, expands `#include "file"` lines (relative to dir, so the shared
// frame-uniform block lives in one place), compiles and links. Returns the GL program, or 0 after printing the
// compiler/linker log to stderr.
unsigned int buildProgram(const std::string& dir, const std::string& vertFile, const std::string& fragFile);

} // namespace render
} // namespace cosmos
