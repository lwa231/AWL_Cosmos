#version 450 core
// Sphere-impostor shading. V1: the lit hemisphere gets light from every light plus ambient, stars are flat, black holes a
// dark disc with a thin ring. V1.1 step 3 adds procedural looks by style (no textures):
//  - stars: limb darkening, cellular granulation, spots, a streamered corona and an additive bloom, coloured by temperature;
//  - planets: gas-giant bands with turbulence and a storm, rocky and icy terrain, oceans with continents, clouds and an
//    atmosphere rim on the lit limb;
//  - black holes: the shadow, a photon ring, a Doppler-brightened accretion disk tilted along its angular momentum, and
//    the lensed image of the far side of the disk.
// uVisual.x (0 Low .. 2 High) sets the octaves and the effects; the body with detail = 1 gets two extra octaves, and the
// octaves are capped by the disc size in pixels, so a small disc costs little.
#include "common.glsl"
#include "bodies.glsl"
#include "noise.glsl"

layout(location = 0) flat in vec2 vCentre;
layout(location = 1) flat in float vDrawRadius;
layout(location = 2) flat in uint  vBody;
layout(location = 3) flat in float vMarker;

layout(location = 0) out vec4 fragColour;

const float LZ = 0.4;              // light tilt towards the camera
const float RING_PX = 1.5;         // black-hole ring thickness (plain style)
const float PI = 3.14159265;

float quality() { return uVisual.x; }

// Octaves for a surface of `radiusPx`: base by quality, +2 for the detail body, capped so the finest octave stays above ~3 px
float surfaceOctaves(float radiusPx, float detail)
{
    float base = 2.0 + 2.0 * quality() + 2.0 * detail;
    float cap = log2(max(radiusPx, 1.0) / 3.0);
    return clamp(min(base, cap), 1.0, 8.0);
}

// ---- stars ----------------------------------------------------------------------------------------------------------

vec4 shadeStar(Body b, vec2 rel, float R)
{
    float dist = length(rel);
    float d = dist / R;
    vec3 base = vec3(b.r, b.g, b.b);
    vec3 limbTint = base * vec3(1.0, 0.78, 0.55);                 // the cooler, redder limb
    vec3 glowTint = mix(base, vec3(1.0), 0.25);
    float q = quality();
    float t = uScale.z;
    float coverage = clamp(R - dist + 0.5, 0.0, 1.0);

    vec3 disc = vec3(0.0);
    if (coverage > 0.0) {
        float mu = sqrt(max(0.0, 1.0 - d * d));
        float limb = 1.0 - 0.55 * (1.0 - mu) - 0.25 * (1.0 - mu) * (1.0 - mu);   // quadratic limb darkening
        vec3 s = vec3(rel / R, mu);
        float gran = 1.0;
        if (vMarker < 0.5) {
            // granulation: bright cells with dark lanes, boiling slowly (the slice moves through 3D noise)
            float cells = q >= 2.0 ? 55.0 : 32.0;
            float visible = smoothstep(1.5, 5.0, R / cells);       // cells smaller than ~2 px fade out
            if (q >= 1.0 && visible > 0.0) {
                vec2 c = cellular3(s * cells + vec3(b.seed * 91.0, 0.0, t * 0.08));
                float lane = 1.0 - smoothstep(0.0, 0.35, c.y - c.x);
                gran -= visible * (0.28 * lane - 0.06 * (1.0 - c.x));
                if (q >= 2.0) {                                    // supergranulation network
                    vec2 c2 = cellular3(s * cells * 0.22 + vec3(3.7, b.seed * 13.0, t * 0.02));
                    gran -= visible * 0.10 * (1.0 - smoothstep(0.0, 0.2, c2.y - c2.x));
                }
            } else {
                float vis = smoothstep(1.5, 5.0, R / 12.0);
                gran -= vis * 0.18 * (fbm3(s * 12.0 + vec3(b.seed * 31.0, 0.0, t * 0.05), 2.0) - 0.5);
            }
            // spots and bright faculae
            float sv = smoothstep(1.0, 6.0, R / 6.0);
            if (q >= 1.0 && sv > 0.0) {
                float spot = fbm3(s * 3.0 + vec3(b.seed * 57.0, 1.3, t * 0.002), 3.0);
                gran *= 1.0 - sv * 0.65 * smoothstep(0.75, 0.80, spot);
                gran += sv * 0.12 * smoothstep(0.6, 0.7, spot) * (1.0 - mu);
            }
        }
        vec3 col = mix(limbTint, base, sqrt(mu)) * limb * gran * 1.7;
        disc = 1.0 - exp(-col * 1.5);                              // soft tone curve: a white-hot centre, a coloured limb
    }

    // corona (radii) and bloom (pixels), additive
    float h = max(d - 1.0, 0.0);
    float dpx = max(dist - R, 0.0);
    float B = starBloomPx(R);
    float glow = 0.35 * exp(-dpx / B);
    if (q >= 1.0) {
        glow += 0.12 * exp(-dpx / (4.0 * B));
        if (vMarker < 0.5 && R > 4.0) {
            vec2 dir = rel / max(dist, 1e-3);
            float streak = fbm3(vec3(dir * 3.5, b.seed * 7.0 + t * 0.01), q >= 2.0 ? 4.0 : 2.0);
            float corona = (0.45 * exp(-h * 4.0) + 0.12 * exp(-h * 1.3)) * (0.4 + 1.3 * streak);
            corona *= 1.0 - smoothstep(0.5 * (STAR_CORONA - 1.0), STAR_CORONA - 1.0, h);
            glow += corona;
        }
    }
    vec3 halo = glowTint * glow;
    return vec4(disc * coverage + halo * (1.0 - coverage), coverage);
}

// ---- black holes ----------------------------------------------------------------------------------------------------

// Emission of the disk at radius r (horizon radii) and disk-plane direction a (unit, in the e_u/e_s frame), 0..~1.5
vec3 diskEmission(float r, vec2 a, float spinSign, float q, float seed)
{
    if (r < BH_DISK_IN || r > BH_DISK_OUT) return vec3(0.0);
    float x = BH_DISK_IN / r;
    float flux = 4.2 * x * x * x * (1.0 - sqrt(x)) * 6.0;         // thin-disk profile, peaks just outside the inner edge
    flux *= smoothstep(BH_DISK_IN, BH_DISK_IN + 0.25, r) * (1.0 - smoothstep(BH_DISK_OUT * 0.6, BH_DISK_OUT, r));
    if (q >= 1.0) {
        // Keplerian shear: the pattern winds up, the inner rings faster
        float omega = 0.9 * spinSign * pow(BH_DISK_IN / r, 1.5);
        float ang = atan(a.y, a.x) - omega * uScale.z;
        vec3 p = vec3(cos(ang) * 2.5, sin(ang) * 2.5, log(r) * 9.0 + seed * 40.0);
        float turb = fbm3(p, q >= 2.0 ? 5.0 : 3.0);
        flux *= 0.35 + 1.3 * turb;
    }
    float heat = clamp(flux, 0.0, 1.5);
    vec3 col = mix(vec3(0.85, 0.28, 0.06), vec3(1.0, 0.78, 0.45), smoothstep(0.1, 0.9, heat));
    col = mix(col, vec3(1.0, 0.97, 0.9), smoothstep(0.9, 1.5, heat));
    return col * flux;
}

vec4 shadeBlackHole(Body b, vec2 rel, float radiusPx)
{
    float u = bhUnitPx(radiusPx);
    vec2 p = rel / u;                                              // horizon radii
    float rho = length(p);
    float q = quality();

    // Disk orientation. The view is top-down, so a disk in the orbital plane is face-on and shows no Doppler asymmetry:
    // the displayed tilt is at least MIN_TILT so the beaming reads.
    // TODO(3D): with a free camera, use the true angle between the view direction and the disk normal.
    const float MIN_TILT = 0.42;                                   // rad (~24 degrees)
    vec3 n = normalize(vec3(b.p0, b.p1, b.p2) + vec3(0.0, 0.0, 1e-6));
    float spinSign = n.z >= 0.0 ? 1.0 : -1.0;                      // counter-clockwise seen from the viewer when +
    vec2 nxy = length(n.xy) > 1e-4 ? normalize(n.xy) : vec2(0.0, 1.0);
    float tilt = max(acos(clamp(abs(n.z), 0.0, 1.0)), MIN_TILT);
    vec3 nd = vec3(nxy * sin(tilt), cos(tilt) * spinSign);         // displayed normal
    vec3 eu = vec3(-nd.y, nd.x, 0.0) / max(length(nd.xy), 1e-6);   // line of nodes (in the screen plane)
    vec3 es = cross(nd, eu);                                       // in-plane, toward / away from the viewer
    float c2 = max(dot(es.xy, es.xy), 1e-4);

    // Direct image: the disk point under this pixel
    float du = dot(p, eu.xy);
    float ds = dot(p, es.xy) / c2;
    vec3 D = du * eu + ds * es;
    float rd = length(vec2(du, ds));
    float z = ds * es.z;                                           // toward the viewer when > 0

    float shadowCov = clamp((BH_SHADOW - rho) * u + 0.5, 0.0, 1.0);
    vec3 col = vec3(0.0);

    // Lensed image of the far half of the disk, bent over and under the shadow (a compressed copy hugging the photon ring)
    if (q >= 1.0 && rho > BH_SHADOW) {
        float rl = BH_DISK_IN + (rho - BH_SHADOW) * 2.6;
        vec2 dir = p / rho;
        float wu = dot(dir, eu.xy);
        float ws = dot(dir, normalize(es.xy));
        float farSign = es.z >= 0.0 ? -1.0 : 1.0;                 // disk-plane s of the far half
        vec2 al = normalize(vec2(wu, farSign * (abs(ws) + 1e-3)));
        vec3 Dl = al.x * eu + al.y * es;                           // unit: eu and es are orthonormal
        float side = ws * farSign > 0.0 ? 1.0 : 0.45;             // the far side shows above the shadow, a thin image below
        vec3 vdir = cross(nd, Dl);
        float beta = 0.5 * sqrt(BH_DISK_IN / rl);
        float boost = pow(1.0 / max(1.0 - beta * vdir.z, 0.2), 3.0) * (1.0 - beta * beta);
        col += diskEmission(rl, normalize(al), spinSign, q, b.seed) * boost * side * 0.75;
    }

    // Photon ring
    float ringW = max(0.06, 0.7 / u);
    col += vec3(1.0, 0.82, 0.6) * 0.9 * exp(-pow((rho - BH_SHADOW) / ringW, 2.0));

    // Black shadow over the lensed light, unless the near half of the disk passes in front of it
    col *= 1.0 - shadowCov;
    bool hidden = z < 0.0 && rho < BH_SHADOW;
    if (!hidden) {
        vec2 a = rd > 0.0 ? vec2(du, ds) / rd : vec2(1.0, 0.0);
        vec3 vdir = cross(nd, rd > 0.0 ? D / rd : eu);              // orbital velocity direction
        float beta = 0.5 * sqrt(BH_DISK_IN / max(rd, BH_DISK_IN));
        float delta = 1.0 / max(1.0 - beta * vdir.z, 0.2) * sqrt(1.0 - beta * beta);
        float boost = delta * delta * delta;                        // relativistic beaming
        vec3 e = diskEmission(rd, a, spinSign, q, b.seed) * boost;
        e *= mix(vec3(1.25, 0.85, 0.65), vec3(0.8, 0.95, 1.25), smoothstep(0.75, 1.35, delta));   // red- / blue-shift tint
        col += e;
    }

    // Faint glow around the whole thing so a distant hole still reads
    col += vec3(0.9, 0.55, 0.3) * 0.05 * exp(-max(rho - BH_SHADOW, 0.0) / 3.0);
    col = 1.0 - exp(-col * 1.3);
    float alpha = max(shadowCov, 0.0);
    return vec4(col, alpha);                                       // the light is additive, the shadow covers
}

// ---- planets --------------------------------------------------------------------------------------------------------

// Body-fixed point on the unit sphere for the view normal n: the pole leans 25 degrees toward the viewer so the bands and
// the spin read from above, then the surface turns about the pole by the spin phase.
vec3 bodyFixed(vec3 n, float spin)
{
    const vec3 pole = vec3(0.0, 0.906, 0.423);
    const vec3 ex = vec3(1.0, 0.0, 0.0);
    const vec3 ez = vec3(0.0, -0.423, 0.906);
    vec3 p = vec3(dot(n, ex), dot(n, pole), dot(n, ez));
    float c = cos(spin), s = sin(spin);
    return vec3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z);
}

// Albedo of the surface (before clouds); `water` 1 where an ocean is (for the specular glint)
vec3 surfaceAlbedo(Body b, vec3 p, float oct, out float water)
{
    vec3 base = vec3(b.r, b.g, b.b);
    vec3 sd = vec3(b.seed * 37.0, b.seed * 11.0, b.seed * 23.0);
    water = 0.0;
    if (b.style == STYLE_GAS) {
        float turb = fbm3(p * 3.0 + sd, oct);
        float y = p.y + 0.10 * (turb - 0.5);
        float bands = 0.5 + 0.5 * sin(y * PI * (6.0 + 3.0 * b.seed) + b.seed * 6.0);
        bands = mix(bands, 0.5 + 0.5 * sin(y * PI * 17.0 + 1.3), 0.25);
        vec3 zone = mix(base, vec3(1.0, 0.97, 0.9), 0.35);
        vec3 belt = base * vec3(0.72, 0.6, 0.5);
        vec3 col = mix(belt, zone, smoothstep(0.25, 0.75, bands));
        col *= 0.9 + 0.2 * fbm3(vec3(p.x * 2.0, p.y * 40.0, p.z * 2.0) + sd, max(oct - 2.0, 1.0));   // fine streaks
        // a great storm (detail body, or High quality)
        if (b.detail > 0.5 || quality() >= 2.0) {
            vec2 ll = vec2(atan(p.z, p.x), asin(clamp(p.y, -1.0, 1.0)));
            vec2 o = vec2(ll.x - (b.seed * 6.0 - 3.0), ll.y + 0.38);
            o.x = mod(o.x + PI, 2.0 * PI) - PI;
            float e = length(o / vec2(0.28, 0.12));
            float swirl = fbm3(vec3(o * 12.0, 1.0) + sd, 3.0);
            col = mix(col, vec3(0.78, 0.38, 0.24) * (0.8 + 0.4 * swirl), (1.0 - smoothstep(0.7, 1.0, e)) * 0.85);
        }
        return col;
    }
    float e = fbm3(p * 1.8 + sd, oct);
    if (b.style == STYLE_OCEAN) {
        float land = smoothstep(0.53, 0.56, e);
        vec3 deep = base * vec3(0.45, 0.6, 0.85) * 0.75;
        vec3 shallow = base * vec3(0.7, 0.95, 1.05);
        vec3 sea = mix(deep, shallow, smoothstep(0.42, 0.53, e));
        float veg = fbm3(p * 4.0 + sd.zxy, max(oct - 1.0, 1.0));
        vec3 ground = mix(vec3(0.24, 0.36, 0.16), vec3(0.58, 0.49, 0.33), smoothstep(0.4, 0.65, veg));
        ground = mix(ground, vec3(0.75, 0.72, 0.68), smoothstep(0.7, 0.8, e));   // mountains
        vec3 col = mix(sea, ground, land);
        float ice = smoothstep(0.80, 0.86, abs(p.y) + 0.08 * (e - 0.5));
        water = (1.0 - land) * (1.0 - ice);
        return mix(col, vec3(0.93, 0.95, 0.98), ice);
    }
    if (b.style == STYLE_ICY) {
        float crack = 1.0 - smoothstep(0.0, 0.025, abs(fbm3(p * 3.0 + sd.yzx, oct) - 0.5));
        return base * (0.92 + 0.16 * (e - 0.5)) * (1.0 - 0.3 * crack) * vec3(0.95, 0.97, 1.0);
    }
    // rocky: highlands and darker maria, plus ridged detail
    float maria = smoothstep(0.48, 0.6, fbm3(p * 1.2 + sd.zyx, min(oct, 3.0)));
    float ridge = 1.0 - abs(2.0 * fbm3(p * 5.0 + sd, max(oct - 1.0, 1.0)) - 1.0);
    vec3 col = base * (0.78 + 0.5 * (e - 0.5)) * (1.0 - 0.25 * maria) * (0.9 + 0.2 * ridge);
    return col;
}

void main()
{
    Body b = bodies[vBody];
    vec3 base = vec3(b.r, b.g, b.b);

    // Position relative to the disc centre from gl_FragCoord, not from an interpolated varying: the rim distance is
    // then exact in pixels even when the radius is millions of pixels.
    vec2 rel = gl_FragCoord.xy - 0.5 * uViewport.xy - vCentre;

    if (b.style == STYLE_STAR) {
        vec4 c = shadeStar(b, rel, vDrawRadius);
        if (c.a <= 0.0 && dot(c.rgb, vec3(1.0)) < 1.0 / 255.0) discard;
        fragColour = c;
        return;
    }
    if (b.style == STYLE_BH) {
        vec4 c = shadeBlackHole(b, rel, b.radius * uScale.y);
        if (c.a <= 0.0 && dot(c.rgb, vec3(1.0)) < 1.0 / 255.0) discard;
        fragColour = c;
        return;
    }

    float insidePx = vDrawRadius - length(rel);          // distance inside the rim in pixels
    vec2 vLocal = rel / vDrawRadius;                     // disc radius 1
    float d = length(vLocal);
    float coverage = clamp(insidePx + 0.5, 0.0, 1.0);    // smooth 1-px edge
    if (coverage <= 0.0) discard;

    vec3 colour;
    if ((b.flags & BODY_BLACK_HOLE) != 0u) {
        float ring = 1.0 - clamp(insidePx - RING_PX + 0.5, 0.0, 1.0);   // 1 on the outer RING_PX pixels
        colour = base * ring;                                           // dark inside, bright ring
    } else if ((b.flags & BODY_FLAT) != 0u || vMarker > 0.5) {
        colour = base;                                   // stars, unlit bodies and minimum-size markers: flat
    } else {
        vec3 n = vec3(vLocal, sqrt(max(0.0, 1.0 - d * d)));
        bool textured = b.style != STYLE_PLAIN && vDrawRadius >= 4.0;
        vec3 albedo = base;
        float water = 0.0, cloud = 0.0;
        if (textured) {
            float oct = surfaceOctaves(vDrawRadius, b.detail);
            vec3 p = bodyFixed(n, b.spinPhase);
            albedo = surfaceAlbedo(b, p, oct, water);
            if (b.cloudCover > 0.0) {
                vec3 pc = bodyFixed(n, b.spinPhase * 1.08 + uScale.z * 0.004);
                float c = fbm3(pc * 2.6 + vec3(b.seed * 5.0, 7.0, 3.0), oct);
                float thr = 1.0 - b.cloudCover;
                cloud = smoothstep(thr - 0.08, thr + 0.12, c + 0.25 * (b.cloudCover - 0.5));
                vec3 cloudCol = b.cloudCover > 0.9 ? mix(base, vec3(1.0, 0.98, 0.92), 0.25) : vec3(0.96, 0.97, 1.0);
                albedo = mix(albedo, cloudCol, cloud);
            }
        }
        vec3 light = vec3(0.0), wrapLight = vec3(0.0), spec = vec3(0.0);
        for (uint i = 0u; i < uCounts.y; ++i) {
            Light l = lights[i];
            vec2 toLight = vec2(l.x - b.x, l.y - b.y);
            float m = max(abs(toLight.x), abs(toLight.y));   // scale first so squaring 1e11 m cannot overflow
            vec2 dir = (m > 0.0) ? normalize(toLight / m) : vec2(0.0);
            vec3 L = normalize(vec3(dir, LZ));
            vec3 lc = vec3(l.r, l.g, l.b) * l.intensity;
            float ndl = dot(n, L);
            light += lc * max(ndl, 0.0);
            if (textured) {
                wrapLight += lc * max(ndl + 0.35, 0.0) / 1.35;
                if (ndl > 0.0) spec += lc * pow(max(dot(reflect(-L, n), vec3(0.0, 0.0, 1.0)), 0.0), 40.0);
            }
        }
        colour = albedo * (b.ambient + light);
        if (textured) {
            colour += spec * water * (1.0 - cloud) * 0.45;
            float rim = pow(1.0 - n.z, 2.2) * b.atmRim;            // the air seen edge-on, lit side only
            colour += vec3(b.p0, b.p1, b.p2) * rim * wrapLight * 0.9;
        }
        colour = min(colour, vec3(1.0));
    }
    fragColour = vec4(colour * coverage, coverage);       // premultiplied
}
