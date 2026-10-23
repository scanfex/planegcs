# Native PlaneGCS build

This directory adds a plain C ABI DLL around the PlaneGCS sources that already
live in `../planegcs`. It does not use Emscripten, JavaScript or Node.js.

The build was verified with:

* Windows 10/11 x64
* Visual Studio 2026 Insiders (MSVC 19.51)
* CMake 4.3.1 (bundled with Visual Studio)

## Layout

```
native/
  CMakeLists.txt          native DLL + smoke test
  build.ps1               bootstrap + configure + build
  include/
    planegcs_native.h     public C API
    Console.h             native replacement for the Emscripten logging shim
    planegcs_prelude.h    headers that MSVC does not include transitively
  patches/
    remove-boost.patch    the Boost removal applied to the upstream sources
  src/
    planegcs_native.cpp   C ABI implementation
  smoke/
    smoke_test.cpp        native end-to-end check

third_party/eigen/        vendored Eigen 3.4.0 headers (MPL2)
```

Build outputs go to `native/out/<Configuration>/`:

* `planegcs_native.dll` - the solver library;
* `planegcs_native.lib` - import library;
* `planegcs_native_smoke.exe` - native smoke test.

## Build

From the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\native\build.ps1
```

Useful options:

```powershell
# Debug build, clean build directory, copy the DLL into a consuming application
.\native\build.ps1 -Configuration Debug -Clean -CopyTo C:\path\to\app\native

# Only build, do not run the smoke test
.\native\build.ps1 -SkipSmoke
```

The script:

1. finds Visual Studio through `vswhere` (including prerelease installs);
2. installs the Visual Studio Build Tools with the C++ workload through
   `winget` if no C++ toolset is found;
3. uses the CMake bundled with Visual Studio, then `cmake` from `PATH`, then
   installs CMake with `winget`;
4. downloads Eigen 3.4.0 into `third_party/eigen` if the headers are missing;
5. configures and builds the DLL and the smoke test.

## C API

`native/include/planegcs_native.h` is self-contained. The ABI is C, so it can be
used through P/Invoke, ctypes, Rust FFI, etc.

The main flow is:

```c
pgcs_context* ctx = pgcs_create();

int x1 = pgcs_add_param(ctx, 0.0, 1);   /* value, is_fixed */
int y1 = pgcs_add_param(ctx, 0.0, 1);
int x2 = pgcs_add_param(ctx, 10.0, 0);
int y2 = pgcs_add_param(ctx, 10.0, 0);

int p1 = pgcs_add_point(ctx, x1, y1);
int p2 = pgcs_add_point(ctx, x2, y2);

pgcs_add_coincident(ctx, p1, p2, 1, 1, 1.0); /* tag, driving, scale */

int status = pgcs_solve(ctx, PGCS_ALGORITHM_DOG_LEG);
pgcs_apply_solution(ctx);
pgcs_diagnose(ctx, PGCS_ALGORITHM_DOG_LEG);
int dof = pgcs_diagnose_full(ctx);

pgcs_destroy(ctx);
```

Important points:

* Parameters are the actual solver unknowns. `is_fixed = 1` excludes a
  parameter from the unknown list.
* Lines are defined by two point handles. Arcs need center/start/end points,
  start/end angle parameters and a radius parameter.
* An arc also needs `pgcs_add_arc_rules()` so the endpoints follow the angles.
* Constraint tags are supplied by the caller and are returned by the diagnostic
  functions (`pgcs_get_conflicting`, `pgcs_get_redundant`,
  `pgcs_get_partially_redundant`).
* `pgcs_solve` returns `0` (Success), `1` (Converged) or `2` (Failed); negative
  values are errors. `pgcs_last_error` describes the last error.
* `pgcs_diagnose` recomputes DoF, conflicting and redundant tags on the current
  parameter values. Call it after `pgcs_apply_solution()` so the diagnostics
  describe the solved geometry; `pgcs_dof` returns the last computed diagnosis.
* `pgcs_diagnose_full` recomputes DoF from the rank of the full unreduced
  Jacobian on the current parameter values and refreshes redundant tags. Use it
  after `pgcs_apply_solution()` for the final status: reduced QR may hide
  nonlinear dependencies, while the full dense rank sees them.
* Algorithm values match `GCS::Algorithm`: BFGS = 0, Levenberg-Marquardt = 1,
  DogLeg = 2.

## Why Boost is not needed

The upstream build installed Boost because of three tiny usages. They were
replaced with standard C++:

* `Geo.h` included `boost/math/constants/constants.hpp` but did not use it;
* `Constraints.cpp` included `boost/graph/graph_concepts.hpp` but did not use it;
* `GCS.cpp` used `boost::adjacency_list` + `boost::connected_components` to split
  the system into independent components. This is now a small union-find in the
  same function (`System::initSolution`).

The change is stored as `native/patches/remove-boost.patch`. If you re-run
`npm run update-freecad` (or otherwise replace the sources with a newer FreeCAD
snapshot), re-apply the patch:

```powershell
git apply native/patches/remove-boost.patch
```

The patch is portable: the browser/WASM build can use the same sources.

`native/include/planegcs_prelude.h` is force-included with `/FI` for MSVC. The
upstream code relied on libstdc++/Emscripten to pull in `<cmath>`, `<cassert>`,
etc. transitively; MSVC does not.

The library is silent by default. Pass `-DPLANEGCS_NATIVE_VERBOSE=ON` to CMake
(or `-DPLANEGCS_NATIVE_VERBOSE=ON` after configuring) to mirror the PlaneGCS
diagnostic messages to stderr. Solver diagnostics (DOF, conflicting and
redundant tags) are always available through the API.

## Managed wrapper

The C# P/Invoke wrapper is in the AutoConstraint repository:

```
src/Planegcs.Native/         PlanegcsContext, enums, PlanegcsException
src/Planegcs.Native.Demo/    console smoke test
```

`Planegcs.Native.dll` (managed) has no reference to AutoConstraint types. Copy
`planegcs_native.dll` next to the consuming application, or pass
`-CopyTo <dir>` to `build.ps1`.

The native DLL links the Visual C++ runtime dynamically. Install the
Microsoft Visual C++ Redistributable (x64) on machines that do not have Visual
Studio installed.

## Licensing

The PlaneGCS sources are LGPL-2.0-or-later (inherited from FreeCAD). Eigen is
MPL2 (see `third_party/eigen/COPYING.MPL2`). Keep the license notices when
redistributing the DLL and the vendored headers.
