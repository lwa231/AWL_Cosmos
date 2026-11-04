#pragma once
// Minimal 24-bit BMP writer for developer screenshots (--screenshot). No dependencies, GL-free.
#include <cstdint>
#include <string>

namespace cosmos {
namespace app {

// rgb: width*height*3 bytes, top row first, R,G,B per pixel. Returns false if the file cannot be written.
bool writeBmp24(const std::string& path, int width, int height, const std::uint8_t* rgb);

} // namespace app
} // namespace cosmos
