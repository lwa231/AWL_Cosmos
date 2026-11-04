# AWL Cosmos V1.1 plan

Agreed in the V1.1 Q&A. The V1 baseline is `main` = `v1` = 64b32c4 on GitHub.

## Ground rules
- Claude works on `v1.1-agents` (worktree `N:\MyCode\AWL_Cosmos_v11`) with worktrees and subagents, commits locally only and never pushes. Claude gives the push and PR commands.
- **No `Co-Authored-By` or other AI attribution lines in commit messages.** GitHub turns them into a contributor icon. Claude is credited in the Settings → Credits tab instead.
- **Split:**
  - The developer writes the 3D physics core in `src/core`, with tutoring. Claude writes the core tests first, as the spec.
  - Claude writes everything else: GUI, rendering and glue.
- **Every scene is fully editable.** Presets are just starting points; nothing is locked.
- **Frame budget stays 6.9 ms:**
  - Release build, vsync off.
  - Lucerna + 200 shuttles, every overlay and the new visuals on.
  - Command: `--bench-shuttles 200 --bench-pace 144 --overlays all`, at 1× and 100,000×.
- Claude says when each step becomes testable.

## Steps, in order

### 1. Menus and settings
- **Start screen:**
  - Cards for Lucerna, Real solar system and Empty sandbox.
  - **Load scene file…**, **Settings** and **Quit**.
  - Picking a card loads that scene into today's Stopped state.
  - `--scene` skips the menu, so scripted runs and screenshots keep working.
- **Esc menu** while running: pauses, and offers Resume, Settings and Return to menu.
- **Settings tabs:**
  - **Graphics:** vsync, MSAA, UI scale, frame-rate cap, star/skybox quality.
  - **Simulation defaults:** threads, default warp, auto slow-down, slow down for any shuttle, controller tick.
  - **Display & units:** SI / km / AU; kg / Earth / solar masses; date format; overlays on at startup.
  - **Controls:** a read-only key and mouse list.
  - **Credits**, last:
    - libraries: Dear ImGui, ImPlot, nlohmann/json, doctest, GLFW, GLAD;
    - fonts: Roboto, Font Awesome (CC BY attribution);
    - data: JPL Horizons;
    - development assistance: Claude (Anthropic). The developer decides the wording.
- Settings are saved to `settings.json` next to the exe.
- **New files:** `src/ui/main_menu.*`, `src/ui/settings_panel.*`, `src/app/settings.*` (GL-free, tested).

### 2. Warp snapping and the object table
- **Warp stops:** 1, 2, 5, 10, 30 s; 1, 2, 5, 10, 30 min; 1, 2, 6, 12 h; 1, 2, 5, 10 days of simulated time per real second.
  - Hold **Ctrl** while dragging for a smooth value.
  - The **, / .** keys step through the same list.
  - Code: `src/app/warp.*`.
- **Object table** replaces the Objects list:
  - Columns: Name | Kind | Parent | Distance to parent | Speed. Sortable, with a search filter.
  - A **▾ menu on each row:** Focus, Rename, Duplicate, Orbit around…, Delete. Deleting a body that has satellites asks for confirmation.
  - Reuses `EditorModel::objects`, `remove` and `setOrbitAround`.

### 3. Visuals
Procedural GLSL only, no texture files. A Graphics quality setting controls the detail.
- **Stars:** granulation, limb darkening, corona and bloom, coloured by temperature.
- **Planets:** noise bands or continents, clouds, an atmosphere rim. The most massive object gets the most detail.
- **Black holes:** a dark shadow, a lensing ring, and an accretion disk brightened on one side (Doppler). The disk is tilted to match the angular momentum of the matter orbiting it.
- **Skybox:** a procedural starfield of faint white, yellow, red, green and blue specks, plus a subtle nebula haze.

### 4. Drag-to-place, 2D first
Works while the run is Stopped or Paused.
- **Drag** an existing body, or a new one from **+ Body**:
  - The SOI and Hill rings of nearby masses are shown.
  - The body's parent snaps to whichever sphere it is inside, and it gets a circular-orbit velocity automatically.
  - A live predicted path (the first few orbits) updates on the worker thread as you drag. Reuses `PathPredictor` in `src/app/overlay_predict.*`.
- **Shift-drag** from the body to set a custom velocity arrow. The prediction shows the resulting eccentric or escape path.

### 5. 3D
The function signatures are agreed first, so both sides can work in parallel.
- **Developer (core):**
  - 3D Kepler elements, adding inclination i and node Ω. The conversions must be safe at i = 0 and e = 0.
  - 3D Jacobi placement.
  - Vector spin and full 3D angular momentum in merges.
  - 3D versions of the hierarchy, Hill radius, impact prediction and swept tests.
- **Claude (everything else):**
  - **Orbit camera:** right-drag orbits around the focused object, the wheel zooms (log scale), middle-drag pans, clicking a body focuses on it.
  - **Camera options:** **T** for an animated switch to top-down; perspective and orthographic; an ecliptic reference grid; a WASD free-fly mode as a toggle.
  - **3D renderer:** depth and 3D picking.
  - **Overlays:** every overlay in 3D. Gravity wells become a warped grid sheet, falling back to the contour rings in top-down.
  - **Placement:** 3D placement UI.

## Verification for every step
- Release build with the full doctest suite green, determinism included.
- `--shot` screenshots of the new UI.
- The 6.9 ms benchmark at 1× and 100,000×.

## Status (handoff)
- **Step 1, menus and settings: done** (commit "V1.1 step 1", 667 tests passing in Release, 0 GL errors).
  - Start screen shows without `--scene`; `--menu` forces it.
  - Esc: the first press stops following, the second opens the pause menu.
  - Settings has five tabs; Controls and Credits text lives in `src/ui/info_text.cpp`.
  - `settings.json` is written next to the exe and is gitignored.
- **Loose ends from step 1:**
  - The "Load scene file…" popup has not been checked with a screenshot.
  - Return to menu and Quit don't warn about unsaved edits.
  - The 6.9 ms benchmark was only spot-checked (about 0.65 ms avg at 1×), not formally re-run.
- **Next: step 2**, warp snapping and the object table.
- **Environment:** building and testing the GUI needs Windows with MSYS2 UCRT64 and an OpenGL 4.5 GPU. A Linux cloud session can edit code and run the GL-free `cosmos_tests`, but the exe's screenshots need the Windows machine.
