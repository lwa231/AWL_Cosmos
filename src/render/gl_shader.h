#pragma once
// GLSL program loading for the OpenGL renderer.
#include <string>

namespace cosmos {
namespace render {

// Reads `<dir>/<vertFile>` and `<dir>/<fragFile>`, expands `#include "file"` lines (relative to dir, so the shared
// frame-uniform block lives in one place), compiles and links. Returns the GL program, or 0 after printing the
// compiler/linker log to stderr.
unsigned int buildProgram(const std::string& dir, const std::string& vertFile, const std::string& fragFile);

// Reads, expands and compiles one compute shader `<dir>/<file>` and links it into a program (0 on failure, log on stderr).
unsigned int buildComputeProgram(const std::string& dir, const std::string& file);

} // namespace render
} // namespace cosmos
