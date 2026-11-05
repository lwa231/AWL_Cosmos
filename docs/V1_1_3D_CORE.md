# V1.1 step 5: the 3D physics core (developer guide)

The signatures below are agreed and declared in `src/core/*3d.h`, and each one is documented there in full: inputs, units, frame, angle ranges, degenerate cases and how it reduces to 2D. You write the bodies in new files, `src/core/kepler3d.cpp`, `placement3d.cpp`, `merge3d.cpp` and `impact3d.cpp`. CMake globs every file in `src/core/*.cpp`, so nothing else needs to change. The spec is the doctest files `tests/test_3d_core_*.cpp`. Every case in them is tagged `[3d-core]` and skipped by default.

The frame is the ecliptic x-y plane with z up, right-handed. Units are SI, and mu = GM.

## Functions to implement (suggested order)

| Header | Signature | Spec file: what it checks |
|---|---|---|
| kepler3d.h | `Elements3D stateToElements3D(const Vec3& r, const Vec3& v, double mu)` | kepler: Curtis ex. 4.3 and Vallado ex. 2-5, hand-made polar/inclined cases, angle ranges, the degenerate rules (i = 0 / pi, e = 0, near-equatorial, parabolic speed), the reduction to `stateToElements` at z = 0, and that rotating about z shifts only Omega |
| kepler3d.h | `RelState elementsToState3D(const Elements3D& el, double mu)` | kepler: round trips over i, Omega, omega, e (ellipses and hyperbolas) and M; z == 0 exactly for i = 0 / pi; the reduction to `elementsToState`; |h| and vis-viva |
| kepler3d.h | `Elements3D toElements3D(const Elements& el2)` | kepler: the 2D to 3D mapping (i = 0 or pi, omega -> -omega when retrograde) |
| kepler3d.h | `Vec3 orbitNormal(const Elements3D& el)` | kepler: equals h/\|h\|; exact at i = 0 / pi |
| placement3d.h | `std::vector<Placed> placeJacobi3D(const std::vector<PlacementBody3D>& bodies)` | placement: the barycentre and momentum are zero in 3D; each Jacobi relative state gives back its elements; a planar list reduces to `placeJacobi` with z == 0 exactly; two-body plane; invalid input |
| placement3d.h | `OrbitSpec3D toOrbitSpec3D(const OrbitSpec& s)` | (used through toPlacement3D) |
| placement3d.h | `std::vector<PlacementBody3D> toPlacement3D(const std::vector<PlacementBody>& bodies)` | placement: the reduction test |
| merge3d.h | `Vec3 spinAngularMomentumG(const SpinBody3D& b)` | merge: k mu R^2 spin |
| merge3d.h | `Vec3 angularMomentumG(const SpinBody3D& b)` | merge: mu r x v + spin |
| merge3d.h | `Merge3D mergeSpin3D(const SpinBody3D& survivor, const SpinBody3D& absorbed, double mergedRadius)` | merge: mass, momentum, centre of mass and total 3D L (about any point) in an off-plane collision; the breakup clamp along the spin axis, with the excess logged; point masses; the reduction to `mergeBodies` in plane; off plane, the vector spin keeps what 2D logs as lost |
| impact3d.h | `ImpactPrediction predictParentImpact3D(const Vec3& r, const Vec3& v, double mu, double rTrig)` | impact: the reduction to `predictParentImpact`; rotation invariance (the point rotates, nothing else changes); radial fall along z; an inclined flyby that hits in projection but misses in 3D |
| impact3d.h | `ImpactPrediction predictShuttleImpact3D(rShuttle, vShuttle, rParent, vParent, muParent, parentRadius, parentAtmosphere)` | impact: wrappers |
| impact3d.h | `ImpactPrediction predictPairImpact3D(pa, va, muA, radiusA, pb, vb, muB, radiusB)` | impact: wrappers |

**Already 3D: nothing to write.** The hierarchy, Hill radius and shuttle parent (`hierarchy.h`), and the swept and closing tests (`collisions.h`, `impact.h`), already use full 3D norms. `test_3d_core_hierarchy.cpp` and `test_3d_core_swept.cpp` pin that behaviour out of plane, and they pass today.

**Spin choice.** Spin is the angular velocity vector in rad/s. The 2D `spinRate` is its z component. Wiring a `Vec3` spin into `Bodies` and `mergeBodies` comes after the spec passes.

## Suggested approach and reading

- **Kepler.** Use the h, node and eccentricity vectors (RV2COE / COE2RV). Read Vallado, *Fundamentals of Astrodynamics and Applications*, sections 2.5 and 2.6, and Curtis, *Orbital Mechanics for Engineering Students*, chapter 4, algorithms 4.2 and 4.5.
  - Reuse the 2D anomaly code (`kepler.cpp`) inside the orbit plane.
  - Treat the equatorial case explicitly, so that sin i = 0 holds exactly.
- **Placement.** Copy the structure of `placement.cpp` and change only the elements-to-state call.
- **Merge.** Murray & Dermott, *Solar System Dynamics*, chapter 2, covers the two-body angular momentum. The rest is the 2D `merge.cpp` with `Vec3` spin.
- **Impact.** Rotate (r, v) into the orbit plane: take x along r, and z along h, or any perpendicular vector when h = 0. Run the 2D predictor there, then rotate the point back. Alternatively, generalise the formulas with |r| and r.v.

Do not copy code from GPL sources.

## Build and run

```
cmake -S . -B out/core3d -G Ninja -DCMAKE_BUILD_TYPE=Release -DCOSMOS_3D_CORE=ON
# one header at a time: -DCOSMOS_3D_CORE_SPECS="kepler;hierarchy;swept"
cmake --build out/core3d -j 8 --target cosmos_tests
./out/core3d/cosmos_tests.exe --no-skip --test-case=*3d-core*          # or *3d-core*kepler*
```

The default build (`COSMOS_3D_CORE` OFF) does not compile these files.
