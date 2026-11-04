#include <doctest.h>
#include "app/bmp_writer.h"
#include "app/palette.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

using namespace cosmos::app;

namespace {

// Independent inverse used to check the forward conversion: linear sRGB -> OKLab (Ottosson's M1 and M2 matrices).
std::array<double, 3> linearSrgbToOklab(double r, double g, double b)
{
    const double l = std::cbrt(0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b);
    const double m = std::cbrt(0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b);
    const double s = std::cbrt(0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b);
    return {0.2104542553 * l + 0.7936177850 * m - 0.0040720468 * s,
            1.9779984951 * l - 2.4285922050 * m + 0.4505937099 * s,
            0.0259040371 * l + 0.7827717662 * m - 0.8086757660 * s};
}

std::size_t tripleKey(const HueTriple& t) { return (t.hue[0] * 24u + t.hue[1]) * 24u + t.hue[2]; }

} // namespace

TEST_SUITE("palette")
{
    TEST_CASE("OKLCH of the sRGB primaries converts to the primaries (published values)")
    {
        // L, C, h of sRGB red, green, blue in OKLCH (Ottosson / CSS Color 4), 5 significant digits
        const auto red = oklchToLinearSrgbUnclipped(0.62796, 0.25768, 29.2339);
        CHECK(red[0] == doctest::Approx(1.0).epsilon(2e-3));
        CHECK(red[1] == doctest::Approx(0.0).scale(1.0).epsilon(2e-3));
        CHECK(red[2] == doctest::Approx(0.0).scale(1.0).epsilon(2e-3));

        const auto green = oklchToLinearSrgbUnclipped(0.86644, 0.29483, 142.4953);
        CHECK(green[0] == doctest::Approx(0.0).scale(1.0).epsilon(2e-3));
        CHECK(green[1] == doctest::Approx(1.0).epsilon(2e-3));
        CHECK(green[2] == doctest::Approx(0.0).scale(1.0).epsilon(2e-3));

        const auto blue = oklchToLinearSrgbUnclipped(0.45201, 0.31321, 264.0520);
        CHECK(blue[0] == doctest::Approx(0.0).scale(1.0).epsilon(2e-3));
        CHECK(blue[1] == doctest::Approx(0.0).scale(1.0).epsilon(2e-3));
        CHECK(blue[2] == doctest::Approx(1.0).epsilon(2e-3));
    }

    TEST_CASE("white, black and grey")
    {
        const auto white = oklchToLinearSrgbUnclipped(1.0, 0.0, 0.0);
        for (double c : white) CHECK(c == doctest::Approx(1.0).epsilon(1e-8));
        const auto black = oklchToLinearSrgbUnclipped(0.0, 0.0, 123.0);
        for (double c : black) CHECK(std::fabs(c) < 1e-12);

        // L = 0.5, C = 0: linear 0.5^3 = 0.125, which is sRGB 0.38858 (1.055 * 0.125^(1/2.4) - 0.055)
        const SrgbColour grey = oklchToSrgb(0.5, 0.0, 0.0);
        CHECK(grey.r == doctest::Approx(0.38858).epsilon(1e-4));
        CHECK(grey.g == doctest::Approx(0.38858).epsilon(1e-4));
        CHECK(grey.b == doctest::Approx(0.38858).epsilon(1e-4));
    }

    TEST_CASE("sRGB transfer function reference points")
    {
        CHECK(linearToSrgb(0.0) == 0.0);
        CHECK(linearToSrgb(1.0) == doctest::Approx(1.0).epsilon(1e-12));
        CHECK(linearToSrgb(0.0031308) == doctest::Approx(0.04045).epsilon(1e-4));   // the toe/curve junction
        CHECK(srgbToLinear(0.5) == doctest::Approx(0.214041).epsilon(1e-5));        // well-known mid-grey value
        CHECK(linearToSrgb(0.214041) == doctest::Approx(0.5).epsilon(1e-5));
        for (double v = 0.0; v <= 1.0; v += 0.05) CHECK(linearToSrgb(srgbToLinear(v)) == doctest::Approx(v).scale(1.0).epsilon(1e-9));
    }

    TEST_CASE("gamut clipping reduces chroma and keeps lightness and hue")
    {
        double used = 0.0;
        const SrgbColour c = oklchToSrgb(0.62796, 0.40, 29.2339, &used);   // far outside the sRGB gamut
        CHECK(used < 0.40);
        CHECK(used > 0.20);
        for (float v : {c.r, c.g, c.b}) { CHECK(v >= 0.0f); CHECK(v <= 1.0f); }
        const auto lab = linearSrgbToOklab(srgbToLinear(c.r), srgbToLinear(c.g), srgbToLinear(c.b));
        CHECK(lab[0] == doctest::Approx(0.62796).epsilon(2e-3));
        CHECK(std::hypot(lab[1], lab[2]) == doctest::Approx(used).epsilon(2e-3));
        CHECK(std::atan2(lab[2], lab[1]) * 180.0 / 3.14159265358979 == doctest::Approx(29.2339).epsilon(5e-3));

        double unchanged = 0.0;
        oklchToSrgb(0.7, 0.05, 200.0, &unchanged);   // comfortably inside: untouched
        CHECK(unchanged == 0.05);
    }

    TEST_CASE("24 hues: evenly spaced, one lightness, readable on black, all distinct")
    {
        const auto& hues = paletteHues();
        std::vector<std::array<double, 2>> ab;
        int clipped = 0;
        for (int k = 0; k < kPaletteHues; ++k) {
            const double h = kPaletteHueOffsetDeg + 360.0 * k / kPaletteHues;
            double used = 0.0;
            const SrgbColour expect = oklchToSrgb(kPaletteLightness, kPaletteChroma, h, &used);
            CHECK(hues[k].r == expect.r);
            CHECK(hues[k].g == expect.g);
            CHECK(hues[k].b == expect.b);
            if (used < kPaletteChroma - 1e-9) ++clipped;

            const auto lab = linearSrgbToOklab(srgbToLinear(hues[k].r), srgbToLinear(hues[k].g), srgbToLinear(hues[k].b));
            CHECK(lab[0] == doctest::Approx(kPaletteLightness).epsilon(3e-3));   // one lightness for all hues
            CHECK(lab[0] > 0.7);                                                // bright enough on black
            ab.push_back({lab[1], lab[2]});
        }
        // Distinct: no two palette colours closer than 0.02 in OKLab (neighbours are ~0.039 apart at full chroma)
        double minDist = 1e9;
        for (int i = 0; i < kPaletteHues; ++i)
            for (int j = i + 1; j < kPaletteHues; ++j)
                minDist = std::min(minDist, std::hypot(ab[i][0] - ab[j][0], ab[i][1] - ab[j][1]));
        CHECK(minDist > 0.02);
        CHECK(clipped < kPaletteHues);   // chroma is only reduced where the gamut forces it
    }

    TEST_CASE("decodeTriple enumerates every ordered triple of different hues exactly once")
    {
        CHECK(kTripleCount == 12144);
        std::set<std::size_t> keys;
        for (std::size_t i = 0; i < kTripleCount; ++i) {
            const HueTriple t = decodeTriple(i);
            REQUIRE(t.hue[0] < 24);
            REQUIRE(t.hue[1] < 24);
            REQUIRE(t.hue[2] < 24);
            REQUIRE(t.hue[0] != t.hue[1]);
            REQUIRE(t.hue[0] != t.hue[2]);
            REQUIRE(t.hue[1] != t.hue[2]);
            keys.insert(tripleKey(t));
        }
        CHECK(keys.size() == kTripleCount);
    }

    TEST_CASE("shuffled assignment: unique for the first 12144 shuttle IDs, repeats after")
    {
        const ShuttlePalette p(1234);
        std::set<std::size_t> keys;
        for (std::uint64_t id = 0; id < kTripleCount; ++id) keys.insert(tripleKey(p.tripleFor(id)));
        CHECK(keys.size() == kTripleCount);
        // Any window of IDs shorter than kTripleCount is unique too (IDs start at 1 in practice)
        keys.clear();
        for (std::uint64_t id = 1; id <= 5000; ++id) keys.insert(tripleKey(p.tripleFor(id)));
        CHECK(keys.size() == 5000);
        // After one full cycle the sequence repeats
        CHECK(tripleKey(p.tripleFor(7)) == tripleKey(p.tripleFor(7 + kTripleCount)));
    }

    TEST_CASE("shuffle is deterministic per seed and differs between seeds")
    {
        const ShuttlePalette a(42), b(42), c(43);
        int differ = 0;
        for (std::uint64_t id = 0; id < 500; ++id) {
            CHECK(tripleKey(a.tripleFor(id)) == tripleKey(b.tripleFor(id)));
            if (tripleKey(a.tripleFor(id)) != tripleKey(c.tripleFor(id))) ++differ;
        }
        CHECK(differ > 450);   // an unrelated order: a match is a 1-in-12144 coincidence
        CHECK(a.seed() == 42);
    }

    TEST_CASE("the shuffle is really shuffled (neighbouring IDs are not neighbouring triples)")
    {
        const ShuttlePalette p(0);
        int sameFirstHue = 0;
        for (std::uint64_t id = 0; id < 480; ++id)
            if (p.tripleFor(id).hue[0] == p.tripleFor(id + 1).hue[0]) ++sameFirstHue;
        // Unshuffled order would share the first hue for 479 of 480 neighbours; random is ~1/24 (20)
        CHECK(sameFirstHue < 60);
    }

    TEST_CASE("coloursFor returns the palette colours of the triple")
    {
        const ShuttlePalette p(7);
        const auto cols = p.coloursFor(99);
        const HueTriple t = p.tripleFor(99);
        for (int s = 0; s < 3; ++s) {
            CHECK(cols[s].r == paletteHues()[t.hue[s]].r);
            CHECK(cols[s].g == paletteHues()[t.hue[s]].g);
            CHECK(cols[s].b == paletteHues()[t.hue[s]].b);
        }
    }

    TEST_CASE("writeBmp24 produces a valid 24-bit bottom-up BMP")
    {
        const std::uint8_t rgb[2 * 2 * 3] = {255, 0, 0, 0, 255, 0,     // top row: red, green
                                             0, 0, 255, 10, 20, 30};   // bottom row: blue, (10,20,30)
        const char* path = "test_bmp_writer_tmp.bmp";
        REQUIRE(writeBmp24(path, 2, 2, rgb));
        std::FILE* f = std::fopen(path, "rb");
        REQUIRE(f != nullptr);
        std::vector<unsigned char> d(200);
        const std::size_t n = std::fread(d.data(), 1, d.size(), f);
        std::fclose(f);
        std::remove(path);
        REQUIRE(n == 54 + 2 * 8);                    // 54-byte header + 2 rows of (6 bytes padded to 8)
        CHECK(d[0] == 'B');
        CHECK(d[1] == 'M');
        CHECK(d[10] == 54);                          // pixel data offset
        CHECK(d[18] == 2);                           // width
        CHECK(d[22] == 2);                           // height (positive: bottom-up)
        CHECK(d[28] == 24);                          // bits per pixel
        CHECK(d[54] == 255); CHECK(d[55] == 0);   CHECK(d[56] == 0);     // first stored row = bottom row; blue pixel as BGR
        CHECK(d[57] == 30);  CHECK(d[58] == 20);  CHECK(d[59] == 10);    // (10,20,30) as BGR
        CHECK(d[62 + 2] == 255);                     // second stored row = top row: red pixel, R is the third byte
        CHECK(!writeBmp24(path, 0, 2, rgb));
    }
}
