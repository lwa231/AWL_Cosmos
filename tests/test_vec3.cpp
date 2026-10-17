#include <doctest.h>
#include <cmath>
#include "core/vec3.h"

using namespace cosmos;

TEST_SUITE("vec3")
{
    TEST_CASE("default is the zero vector")
    {
        Vec3 v{};
        CHECK(v.x == 0.0);
        CHECK(v.y == 0.0);
        CHECK(v.z == 0.0);
    }

    TEST_CASE("add and subtract")
    {
        Vec3 a{1, 2, 3}, b{4, -5, 6};
        Vec3 s = a + b;
        Vec3 d = a - b;
        CHECK(s.x == 5.0);  CHECK(s.y == -3.0); CHECK(s.z == 9.0);
        CHECK(d.x == -3.0); CHECK(d.y == 7.0);  CHECK(d.z == -3.0);
    }

    TEST_CASE("scalar multiply and divide")
    {
        Vec3 a{1, 2, 3};
        Vec3 l = a * 2.0;
        Vec3 r = 2.0 * a;
        Vec3 q = a / 2.0;
        CHECK(l.x == 2.0); CHECK(l.y == 4.0); CHECK(l.z == 6.0);
        CHECK(r.x == 2.0); CHECK(r.y == 4.0); CHECK(r.z == 6.0);
        CHECK(q.x == 0.5); CHECK(q.y == 1.0); CHECK(q.z == 1.5);
    }

    TEST_CASE("compound assignment")
    {
        Vec3 a{1, 2, 3}, b{4, -5, 6};
        Vec3 t = a;
        t += b;
        CHECK(t.x == 5.0); CHECK(t.y == -3.0); CHECK(t.z == 9.0);
        t -= b;
        CHECK(t.x == 1.0); CHECK(t.y == 2.0); CHECK(t.z == 3.0);
    }

    TEST_CASE("dot")
    {
        Vec3 x{1, 0, 0}, y{0, 1, 0};
        CHECK(dot(x, x) == 1.0);
        CHECK(dot(x, y) == 0.0);
        CHECK(dot(Vec3{1, 2, 3}, Vec3{4, -5, 6}) == 12.0);
    }

    TEST_CASE("cross follows the right-hand rule")
    {
        Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
        Vec3 xy = cross(x, y);
        Vec3 yz = cross(y, z);
        Vec3 zx = cross(z, x);
        CHECK(xy.x == 0.0); CHECK(xy.y == 0.0); CHECK(xy.z == 1.0);
        CHECK(yz.x == 1.0); CHECK(yz.y == 0.0); CHECK(yz.z == 0.0);
        CHECK(zx.x == 0.0); CHECK(zx.y == 1.0); CHECK(zx.z == 0.0);
    }

    TEST_CASE("cross is antisymmetric and perpendicular to both inputs")
    {
        Vec3 a{1, 2, 3}, b{4, -5, 6};
        Vec3 ab = cross(a, b);
        Vec3 ba = cross(b, a);
        CHECK(ab.x == -ba.x);
        CHECK(ab.y == -ba.y);
        CHECK(ab.z == -ba.z);
        CHECK(dot(ab, a) == 0.0);
        CHECK(dot(ab, b) == 0.0);
    }

    TEST_CASE("Lagrange identity")
    {
        Vec3 a{1.3, -2.7, 0.4}, b{-0.9, 1.1, 3.6};
        double lhs = norm2(cross(a, b));
        double rhs = norm2(a) * norm2(b) - dot(a, b) * dot(a, b);
        CHECK(lhs == doctest::Approx(rhs).epsilon(1e-12).scale(0));
    }

    TEST_CASE("2D z component matches h = rx*vy - ry*vx")
    {
        Vec3 r{2.5, -1.5, 0}, v{0.5, 3.0, 0};
        CHECK(cross(r, v).z == r.x * v.y - r.y * v.x);
    }

    TEST_CASE("norm and norm2")
    {
        Vec3 a{1, 2, 3};
        CHECK(norm2(a) == dot(a, a));
        CHECK(norm2(a) == 14.0);
        CHECK(norm(Vec3{3, 4, 0}) == 5.0);
        CHECK(norm(Vec3{}) == 0.0);
    }

    TEST_CASE("usable at compile time")
    {
        constexpr Vec3 k = Vec3{1, 2, 3} + Vec3{1, 1, 1};
        static_assert(k.x == 2.0 && k.y == 3.0 && k.z == 4.0, "constexpr add");
        CHECK(k.x == 2.0);
    }
}
