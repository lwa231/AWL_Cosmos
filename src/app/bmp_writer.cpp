#include "app/bmp_writer.h"
#include <cstdio>
#include <vector>

namespace cosmos {
namespace app {

namespace {
void put16(std::vector<std::uint8_t>& o, std::uint32_t v)
{
    o.push_back(static_cast<std::uint8_t>(v & 0xFF));
    o.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
}
void put32(std::vector<std::uint8_t>& o, std::uint32_t v)
{
    put16(o, v & 0xFFFF);
    put16(o, v >> 16);
}
} // namespace

bool writeBmp24(const std::string& path, int width, int height, const std::uint8_t* rgb)
{
    if (width <= 0 || height <= 0 || !rgb) return false;
    const std::uint32_t rowBytes = (static_cast<std::uint32_t>(width) * 3 + 3) & ~3u;   // rows padded to 4 bytes
    const std::uint32_t imageBytes = rowBytes * static_cast<std::uint32_t>(height);

    std::vector<std::uint8_t> out;
    out.reserve(54 + imageBytes);
    out.push_back(0x42);           // "BM"
    out.push_back(0x4D);
    put32(out, 54 + imageBytes);   // file size
    put32(out, 0);                 // reserved
    put32(out, 54);                // pixel data offset
    put32(out, 40);                // BITMAPINFOHEADER
    put32(out, static_cast<std::uint32_t>(width));
    put32(out, static_cast<std::uint32_t>(height));   // positive = rows stored bottom-up
    put16(out, 1);                 // planes
    put16(out, 24);                // bits per pixel
    put32(out, 0);                 // BI_RGB
    put32(out, imageBytes);
    put32(out, 2835);              // 72 dpi
    put32(out, 2835);
    put32(out, 0);
    put32(out, 0);

    for (int y = height - 1; y >= 0; --y) {
        const std::uint8_t* row = rgb + static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 3;
        for (int x = 0; x < width; ++x) {
            out.push_back(row[x * 3 + 2]);   // BMP stores B,G,R
            out.push_back(row[x * 3 + 1]);
            out.push_back(row[x * 3 + 0]);
        }
        for (std::uint32_t p = static_cast<std::uint32_t>(width) * 3; p < rowBytes; ++p) out.push_back(0);
    }

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
    std::fclose(f);
    return ok;
}

} // namespace app
} // namespace cosmos
