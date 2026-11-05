// Procedural noise for the V1.1 step 3 looks (own implementation): integer hashing, value noise, fBm and cellular noise.
// Deterministic across runs and GPUs that implement 32-bit integer arithmetic (all GL 4.5 ones).

uint hashU(uint x)
{
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}
uint hash3u(ivec3 i) { return hashU(uint(i.x) + hashU(uint(i.y) + hashU(uint(i.z) ^ 0x5bd1e995u))); }
uint hash2u(ivec2 i) { return hashU(uint(i.x) + hashU(uint(i.y) ^ 0x27d4eb2du)); }
float u2f(uint h) { return float(h >> 8) * (1.0 / 16777216.0); }   // 0..1

// Value noise in 3D, 0..1, smooth (quintic fade)
float vnoise3(vec3 p)
{
    ivec3 i = ivec3(floor(p));
    vec3 f = fract(p);
    vec3 w = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float a = u2f(hash3u(i)),               b = u2f(hash3u(i + ivec3(1, 0, 0)));
    float c = u2f(hash3u(i + ivec3(0, 1, 0))), d = u2f(hash3u(i + ivec3(1, 1, 0)));
    float e = u2f(hash3u(i + ivec3(0, 0, 1))), g = u2f(hash3u(i + ivec3(1, 0, 1)));
    float h = u2f(hash3u(i + ivec3(0, 1, 1))), k = u2f(hash3u(i + ivec3(1, 1, 1)));
    return mix(mix(mix(a, b, w.x), mix(c, d, w.x), w.y), mix(mix(e, g, w.x), mix(h, k, w.x), w.y), w.z);
}

float vnoise2(vec2 p)
{
    ivec2 i = ivec2(floor(p));
    vec2 f = fract(p);
    vec2 w = f * f * (3.0 - 2.0 * f);
    float a = u2f(hash2u(i)), b = u2f(hash2u(i + ivec2(1, 0)));
    float c = u2f(hash2u(i + ivec2(0, 1))), d = u2f(hash2u(i + ivec2(1, 1)));
    return mix(mix(a, b, w.x), mix(c, d, w.x), w.y);
}

// fBm, 0..1. `octaves` may be fractional: the last octave fades in (no popping as the level of detail changes).
float fbm3(vec3 p, float octaves)
{
    float sum = 0.0, amp = 0.5, norm = 0.0;
    for (int o = 0; o < 8; ++o) {
        float w = clamp(octaves - float(o), 0.0, 1.0);
        if (w <= 0.0) break;
        sum += amp * w * vnoise3(p);
        norm += amp * w;
        amp *= 0.5;
        p = p * 2.03 + vec3(17.1, 5.3, 11.7);
    }
    return norm > 0.0 ? sum / norm : 0.5;
}

float fbm2(vec2 p, int octaves)
{
    float sum = 0.0, amp = 0.5, norm = 0.0;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * vnoise2(p);
        norm += amp;
        amp *= 0.5;
        p = p * 2.07 + vec2(13.7, 7.1);
    }
    return sum / norm;
}

// Cellular noise in 3D: x = distance to the nearest feature point, y = to the second nearest (cell units)
vec2 cellular3(vec3 p)
{
    ivec3 i = ivec3(floor(p));
    vec3 f = fract(p);
    float f1 = 8.0, f2 = 8.0;
    for (int z = -1; z <= 1; ++z)
    for (int y = -1; y <= 1; ++y)
    for (int x = -1; x <= 1; ++x) {
        ivec3 c = i + ivec3(x, y, z);
        uint h = hash3u(c);
        vec3 o = vec3(u2f(h), u2f(hashU(h)), u2f(hashU(h ^ 0x9e3779b9u)));
        vec3 d = vec3(x, y, z) + o - f;
        float dd = dot(d, d);
        if (dd < f1) { f2 = f1; f1 = dd; } else if (dd < f2) { f2 = dd; }
    }
    return sqrt(vec2(f1, f2));
}
