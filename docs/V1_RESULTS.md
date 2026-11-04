# AWL Cosmos V1 — measured results

The plan's M8 asks for "this document updated with the measured results". The plan file itself was left untouched by request, so the results live here. Branch `v1-agents`, Release build (`cmake --preset release`), Windows 10, MSYS2 UCRT64 GCC, RTX 2070 SUPER.

Run everything:

```
export PATH=/c/msys64/ucrt64/bin:$PATH
cmake --preset release && cmake --build out/release
./out/release/cosmos_tests.exe            # full suite, ~30 s
./out/release/cosmos_tests.exe -s | grep "\[validation\]\|\[lucerna50\]"   # the measured values below
```

Suite status at the time of writing: **652 test cases, 1,145,916 assertions, all passing** (Release, no compiler warnings).

## Plan §14 validation suite

| Row | Pass condition (plan) | Measured | Result |
|---|---|---|---|
| Kepler e = 0, 0.5, 0.9 | energy and period error < 1e-10 over 1,000 orbits (IAS15) | max \|dE/E\| 8.7e-15 / 6.7e-15 / 2.8e-14; period error 0 / 3.1e-15 / 1.8e-14 | PASS |
| Leapfrog / Yoshida order | error ratio ≈ 4 (KDK), ≈ 16 (Yoshida) when dt halves | KDK 4.0002, 4.0000; Yoshida 16.0008 | PASS |
| Kepler utilities | round trip < 1e-12 relative, e 0–0.95 and hyperbolic | 2,000 random orbits (400 hyperbolic): worst state 1.6e-14, a 2.3e-14, e 2.0e-15 | PASS |
| Figure-eight | stays on the choreography for 100 periods | max return deviation 4e-6, \|dE/E\| 1.6e-15 (also via `Simulation`, point-mass mode) | PASS |
| Pythagorean three-body | μ=3 escapes; 4 and 5 leave as a binary; last close approach t ≈ 59.4 | E(μ3 vs binary) +2.356 (unbound), E(binary) −8.153 (bound); last close approach t = 59.42 | PASS |
| Sun–Jupiter–Saturn | Great Inequality period 880–940 yr; tighter-tolerance rerun within 0.1 % | 937.43 yr at ε = 1e-9 and 1e-11 (difference 1.3e-11) | PASS |
| Lucerna 50-year | all moons bound, Gelu < 0.4 Hill, energy error < 1e-9 | 981,244 steps (2.4 s); energy error 4.3e-10; Gelu 0.330 Hill; all 7 moons bound, Exul stays retrograde | PASS |
| Merge conservation | P and L (incl. spin and logged breakup ΔL) to 1e-14; one case below and one above ω_break; ΔE logged | below: P, L error 0; above: L error 0 with logged ΔL added back; 200 random merges: worst P 1.3e-16, L 4.8e-16 | PASS |
| Drag | v(t) = v0 / (1 + k v0 t) to 1e-12 | exact drag step 0; through `propagateShuttle` (20,001 steps) 1.1e-13 | PASS |
| Rocket | Δv = v_e ln(m0/m_dry) to 1e-9 | exact burn 0; through `propagateShuttle` 6.8e-13 | PASS |
| Swept collision | fast shuttle, thin body, huge dt: detected | detected (`test_collisions`, `test_impact_swept`, `test_simulation_shuttles`) | PASS |
| Determinism | Lucerna + 200 shuttles, 1/4/16 threads, two warp schedules: bit-identical | bit-identical (`test_determinism`) | PASS |
| Real solar system (M8) | Horizons at T0, compare at T0 + 1 yr: Earth within ~1,000 km; Mercury ~24 km/yr | Earth **86.9 km**; Mercury **31.9 km** (see below) | PASS |
| Performance (M8) | Release, vsync off, Lucerna + 200 shuttles, every overlay on: ≤ 6.9 ms at 1× and 100,000× | avg 1.35 ms at 1×, 2.16 ms at 100,000× (1920×1080) | PASS |

## Real solar system preset (plan §11.2)

- `scenes/solar_system.json`, preset "Real solar system": Sun, Mercury, Venus, Earth, Moon, Mars, Jupiter, Saturn, Uranus, Neptune, Pluto.
- JPL Horizons API, retrieved 2026-11-04: body-centre vectors (ids 10, 199 … 999), centre `500@0`, ecliptic J2000 / ICRF, km and km/s converted to SI, real z kept. Epoch T0 = JD 2451545.0 TDB (J2000). Body GMs and mean radii from the Horizons headers (DE440/DE441 family). Provenance in `tests/data/README.md`.
- 1-year comparison (T0 + 365.25 d, 515 IAS15 steps, 0.02 s):

| Body | Position error | Body | Position error |
|---|---|---|---|
| Sun | 25.8 km | Jupiter | 2,732 km |
| Mercury | 31.9 km | Saturn | 40,992 km |
| Venus | 104.1 km | Uranus | 8,117 km |
| Earth | 86.9 km | Neptune | 28,960 km |
| Moon | 98.9 km | Pluto | 763,791 km |
| Mars | 77.0 km | | |

- Mercury's 31.9 km is the same order as the plan's ~24 km/yr from missing relativity (1PN is "Later"); the remainder is plausibly missing asteroids and the moon systems below.
- The outer planets are large **by data choice, not integrator error**: the plan asks for bodies, and the body centres of Jupiter–Pluto orbit their system barycentres with their moons (≈ 1 m/s for the giants, ≈ 27 m/s for Pluto–Charon), which a point mass integrates as drift. Using system barycentres 5–9 with system GMs would remove it; left as specified.

## Performance (plan §14 row, §17)

Benchmark command (Release, from `out/release`):

```
./AWL_Cosmos.exe --size 1920 1080 --no-vsync --overlays all --open view --bench-shuttles 200 --bench-pace 144 \
                 --follow Domus --mpp 2000 --warp 1 --do start@3 --frames 2500        # and --warp 100000
```

- `--bench-shuttles 200` adds 200 shuttles to Lucerna (209 in total), cycling through the autopilots.
- `--overlays all` turns on every View overlay: arrows, trails, osculating orbits, predicted path, gravity wells, Hill/SOI/Roche and atmospheres, with labels and UI visible.
- `--bench-pace 144` makes every frame simulate 1/144 s of real time × warp. That is the physics load per frame at the 144 fps target, however fast the frames are actually drawn. The first 120 frames are a warm-up and are not counted.

Measured frame times (wall clock, ms) on an RTX 2070 SUPER, 16 threads:

| Warp | avg | p50 | p95 | p99 | max | physics avg | target |
|---|---|---|---|---|---|---|---|
| 1× | **1.35** | 1.21 | 2.40 | 3.19 | 16.2 | 0.002 | ≤ 6.9 |
| 100,000× | **2.16** | 2.33 | 3.22 | 3.85 | 8.1 | 0.80 | ≤ 6.9 |

- Before tuning, the same scene ran at an average of 3.82 ms (1×) and 3.80 ms (100,000×), unpaced at 1600×900. The biggest cost was the osculating-orbit overlay, at 2.85 ms per frame.
- Tuning made the trail and orbit geometry parallel and culled to the view, and added a Hill-radius table for shuttle parents. The osculating-orbit cost fell to 0.27 ms. Integrator tolerances and the physics are unchanged, and the determinism and accuracy tests still pass.
- The gravity-well ring pass costs 0.14 ms of GPU time at 1080p with Lucerna's 14 bodies. With 64 sources it costs 0.57 ms, inside the plan's 0.3–0.6 ms range.
- Single-frame spikes remain. The 16 ms frame at 1× is a one-off during start-up, such as a buffer growth. At 100,000×, rare frames that take a long body step reach about 8 ms, and the physics max is 35 ms when the clock catches up. The average and p99 are well inside the budget.

## Milestone summary

| Milestone | Delivered |
|---|---|
| M0–M1 | Baseline, `cosmos_core` library, doctest + CTest, build presets, shader copy target |
| M2 | SI/μ units, gravity with overlap field, KDK, Yoshida-4, IAS15 (fixed, adaptive, dense output), Kepler utilities, Jacobi placement, hierarchy, swept collisions, merges, events, kinds, scene JSON, Lucerna |
| M3 | Shuttles: DOPRI5 against body dense output, atmospheres with exact drag, rocket equation, PD autopilots, counter-based RNG, impact prediction, threaded deterministic `Simulation` |
| M4 | Clock (Neumaier), warp ladder, `SimDriver` interpolation at t_target, camera (log zoom, follow, frames) |
| M5 | OpenGL 4.5 renderer: UBO/SSBO, instancing, persistent mapped triple-buffered streams with fences, camera-relative floats |
| M6 | ImGui/ImPlot UI: scene editor with units grammar, Start/Pause/Stop/Save, presets, bottom bar, labels, stats charts |
| M7 | Velocity/acceleration arrows, timestamped trails, osculating orbits, predicted path (worker), gravity-well rings, Hill/SOI/Roche circles, atmosphere bubbles, anchored GPU explosions, warp step-down and Resume warp, "any shuttle" switch |
| M8 | Real solar system preset and 1-year test, §14 audit, performance benchmark and tuning, this document |

M7 done-when, checked with screenshots: with every overlay on, each toggle draws its overlay. The Lucerna Retrograde shuttle (Shuttle 6), started at 100,000×, steps the warp down one level at a time, reaching 1× about 3 real seconds before entering Domus's atmosphere. It then burns in the air for about a minute, is destroyed, and explodes as a visible spark cloud. After Resume warp, the cloud moves round the limb with Domus's spin while it fades.

## Known limitations

- Explosion clouds are magnified on screen to a minimum size of about 34 px, so a 100 m cloud stays readable at planetary zoom. The magnification is visual only.
- The predicted path matches a continuing run to about a millimetre, not bit for bit, because the copy restarts IAS15.
- 1PN relativity, rotating reference frames and the other "Later" items from plan §18 are not part of V1.
