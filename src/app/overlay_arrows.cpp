#include "app/overlay_arrows.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

double arrowShape(ArrowScale mode, double x)
{
    if (!(x > 0.0) || !std::isfinite(x)) return 0.0;
    switch (mode) {
    case ArrowScale::Linear: return x;
    case ArrowScale::Sqrt: return std::sqrt(x);
    case ArrowScale::Log: return std::max(0.0, 1.0 + std::log10(x) / kArrowLogDecades);
    }
    return 0.0;
}

double arrowLengthPx(ArrowScale mode, double magnitude, double reference, double lengthPx)
{
    if (!(reference > 0.0) || !(lengthPx > 0.0)) return 0.0;
    return std::min(lengthPx * arrowShape(mode, magnitude / reference), kArrowMaxFactor * lengthPx);
}

bool appendArrow(LineBatch& batch, double x, double y, double dirX, double dirY, double lengthPx, double metresPerPixel,
                 const Rgba& colour, float widthPx)
{
    const double dn = std::hypot(dirX, dirY);
    if (!(dn > 0.0) || !(lengthPx >= kArrowMinPx) || !(metresPerPixel > 0.0)) return false;
    const double ux = dirX / dn, uy = dirY / dn;
    const double len = lengthPx * metresPerPixel;
    const double tipX = x + ux * len, tipY = y + uy * len;

    // Head: two barbs 25 degrees off the shaft, 30% of the arrow but between 4 and 12 px (and never longer than the shaft).
    const double headPx = std::min(lengthPx, std::clamp(0.3 * lengthPx, 4.0, 12.0));
    const double head = headPx * metresPerPixel;
    constexpr double kCos = 0.9063077870366499, kSin = 0.42261826174069944;   // 25 degrees
    const double bx = -ux * head, by = -uy * head;                             // from the tip back along the shaft
    LineVertex tail{static_cast<float>(x), static_cast<float>(y), colour.r, colour.g, colour.b, colour.a};
    LineVertex tip{static_cast<float>(tipX), static_cast<float>(tipY), colour.r, colour.g, colour.b, colour.a};
    LineVertex b1 = tip, b2 = tip;
    b1.x = static_cast<float>(tipX + bx * kCos - by * kSin);
    b1.y = static_cast<float>(tipY + bx * kSin + by * kCos);
    b2.x = static_cast<float>(tipX + bx * kCos + by * kSin);
    b2.y = static_cast<float>(tipY - bx * kSin + by * kCos);
    batch.addSegment(tail, tip, widthPx);
    batch.addSegment(tip, b1, widthPx);
    batch.addSegment(tip, b2, widthPx);
    return true;
}

} // namespace app
} // namespace cosmos
