# Smart Print assistant

The Smart Print assistant chooses material and strength settings from what a part is used for, and
shows how the part copes with realistic loads. It is opened from the main menu or an object's context
menu, works on one object, and writes its result as per-object settings.

## Pieces

| Where | What |
|---|---|
| `src/libslic3r/SmartPrint/Materials.*` | Mechanical properties of printed material families (`resources/smartprint/materials.json`), matched to Orca's `filament_type`. |
| `src/libslic3r/SmartPrint/PrintStructure.*` | Walls, shells and infill as they affect stiffness and strength. Infill properties scale as density^n per pattern (Gibson-Ashby). |
| `src/libslic3r/SmartPrint/StructuralModel.*` | Linear-elastic finite elements on a voxel grid. |
| `src/libslic3r/SmartPrint/Propulsion.*` | Motor / propeller / battery library (`resources/smartprint/propulsion.json`) and a static thrust model. |
| `src/libslic3r/SmartPrint/Advisor.*` | Use cases (`resources/smartprint/usecases.json`), material ranking, the Light / Balanced / Strong search, drone layout and load cases. |
| `src/libslic3r/SmartPrint/Session.*` | JSON request/response protocol used by the GUI. |
| `src/slic3r/GUI/SmartPrintDialog.*` | Web dialog host: reads the object, printer, filaments and settings, runs the session on a worker thread, applies the result. |
| `resources/web/dialog/SmartPrint/` | The page: wizard, option comparison and the three.js stress view. |

## Structural model

The object's model parts, in printer coordinates (Z is the build direction), are voxelized. Each voxel's
fill fraction comes from vertical rays (3 x 3 per column) and its distances to the side, top and bottom
surfaces from a distance transform. Those distances decide how much of the voxel is wall or shell for
the given wall count, line width and shell layers; the rest is sparse infill. The voxel's stiffness and
strength mix the two.

Elements are 8-node bricks with Wilson incompatible modes, condensed once for the unit cube. They bend
correctly with two elements through a wall: a cantilever's tip deflection, bending stress and first
natural frequency are within 2% of beam theory (see `tests/libslic3r/test_smartprint.cpp`).

The failure check is strain based for each constituent (walls and infill strands) and separates
in-plane failure (von Mises against XY strength) from layers separating (Z tension and interlayer shear
against Z strength), because FDM parts are weakest across layers.

## Speed

The system is solved with conjugate gradients preconditioned by geometric multigrid on the voxel node
grid (Galerkin coarse levels, damped Jacobi smoothing, direct solve on the coarsest level). Cost grows
linearly with the element count, which a direct factorization of a chunky 3D part does not.

Every solve is the response to one unit load (a unit force or torque on a region, or the part's own
weight per unit density and acceleration) and is cached. A load case is a weighted sum of responses, so
changing a motor, propeller, battery, force magnitude or material needs no new solve. Changing walls or
infill clears the cache but keeps the old responses as starting guesses. Independent responses are
solved in parallel.

Vibration uses Rayleigh's method on the static deflection under the part's own weight plus attached
masses, which is within a few percent of the first mode for arms and beams and is nearly free.

## Options search

The search for Light / Balanced / Strong runs on a coarse model (about 5,000 elements): safety under the
governing load case over walls 2-6 and infill 10/30/100%, interpolated in log-log space for the lightest
setting meeting each target. Each pick is then checked on the model the user sees and re-aimed with the
measured ratio between the two, so the numbers on the options match the stress map. When no setting
reaches a target, the reply says how much thicker the weak section must be (strength in bending scales
with thickness squared).

## Drones

Motor mounts are found as the outermost points in `n` directions around the frame centre; the centre is
the support. Load cases are a full-throttle punch-out (thrust and motor torque on every mount, times a
style factor), a hard landing (acceleration on the frame and motor masses) and a crash onto one arm.
The thrust model balances motor torque `Kt (I - I0)` against propeller torque `Cp rho n^2 D^5 / 2 pi` with
battery sag; a bench-measured thrust overrides it.

## Applying

Settings are written as per-object overrides (`wall_loops`, `sparse_infill_density`,
`sparse_infill_pattern`, `top_shell_layers`, `bottom_shell_layers`, and the shell thicknesses set to 0 so
the simulated layer counts are what gets printed) inside one undo snapshot. The chosen filament is used
from the slot that holds it, or put into the object's slot.

## Strings

The page's English text lives in `resources/web/dialog/SmartPrint/strings.js`.
`scripts/smartprint_strings.py` generates `src/slic3r/GUI/SmartPrintStrings.cpp` from it, so every page
string goes through the .po catalogs with the context "Smart Print".
