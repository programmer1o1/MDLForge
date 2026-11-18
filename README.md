# MDLForge

Standalone, cross-platform build of `studiomdl` with minimal runtime dependencies.

## Build

### Windows

Requirements: CMake 3.16+ and Visual Studio.

```powershell
cmake -S . -B build -A Win32
cmake --build build --config Release
```

Output: `build/utils/studiomdl/Release/studiomdl.exe`

Defaults to **32-bit** (`STUDIOMDL_REQUIRE_32BIT=ON`) on Windows. Pass `-DSTUDIOMDL_REQUIRE_32BIT=OFF` and `-A x64` for a 64-bit build — output is fully compatible with Source engines either way.

`-DSTUDIOMDL_MSVC_STATIC_RUNTIME=ON` links the MSVC runtime statically for redistribution.

### Linux / macOS

Requirements: CMake 3.16+ and a C++17 compiler toolchain.

```sh
cmake -S . -B build -DSTUDIOMDL_REQUIRE_32BIT=OFF
cmake --build build
```

64-bit builds produce on-disk MDL/VVD/VTX/PHY files that match Source's 32-bit layout exactly — pointer fields in the on-disk structs are pinned to 32 bits regardless of host bitness, so a 64-bit `studiomdl` on Linux or macOS produces files the (32-bit) Source engine loads correctly.

On Apple Silicon Macs, build for x86_64 and run under Rosetta:

```sh
cmake -S . -B build -DSTUDIOMDL_REQUIRE_32BIT=OFF -DCMAKE_OSX_ARCHITECTURES=x86_64
cmake --build build
arch -x86_64 build/utils/studiomdl/studiomdl <args>
```

### CI

CI build scripts live in `.ci/` and are used by `.github/workflows/ci.yml`:

- Windows: `.ci/build.ps1`
- Linux/macOS: `.ci/build.sh`

## Install

```powershell
cmake --install build --config Release --prefix <install-dir>
```

For single-config generators (Ninja/Makefiles), omit `--config`.

## Run

```text
studiomdl <path/to/model.qc> -game <path/to/moddir>
```

`-game` must point to the directory that contains `gameinfo.txt`.

Useful flags (in addition to stock `studiomdl` options):

- `-nosw` — skip writing `.sw.vtx` (software-renderer LOD, unused by modern Source games)
- `-noxbox` — skip writing `.xbox.vtx` (original Xbox LOD)

## Notes

### vphysics (collision compilation)

`$collisionjoints`/`$collisionmodel` compilation requires an `IPhysicsCollision` / `IPhysicsSurfaceProps` implementation loadable at runtime. MDLForge ships a bundled module under `vendor/vphysics/` that builds on all three platforms and is placed next to the `studiomdl` binary so the loader finds it automatically.

**Origin and licensing.** The implementation is MIT-licensed. Most of it (format structures, volume / centroid / inertia math, axis remap, ledge layout, half-edge graph construction) was developed clean-room from public IVP format documentation and Valve's public SDK headers. The specific convention for IVP's `compact_edge.opposite_index` field — a signed offset in 4-byte slot units, *not* a linear edge index — was derived by reading `nillerusr/source-engine`'s vphysics tree as a format reference; the code here is original but the bit-level format choice was confirmed against that reference. All collision logic, ray-casts, and data flow are our own code.

The bundled module:

- Is enabled by default via `-DMDLFORGE_BUILD_VPHYSICS=ON`. Set `OFF` to skip it if you want to provide your own `libvphysics.{so,dylib,dll}`.
- Produces `.phy` files that match Valve's stock output field-by-field on well-authored physics meshes. Validated against shipped reference models — per-solid `massCenter`, `upperLimitRadius`, ledgetree node bounds, and compact-surface/ledge layout all agree to four decimals. Ragdolls collide and settle normally in Source engines.
- Implements the IVP wire format correctly: inches→meters scaling, Source→IVP axis remap (`(x,y,z)→(x,-z,y)`, a +90° rotation about X), 16-byte-aligned ledge layout (ledges before tree nodes), volume-weighted centroid via tetrahedral decomposition, real per-axis inertia moments, half-edge adjacency with signed-slot `opposite_index`, and ray-cast `pierce_index` for penetration-depth queries.
- Uses [akuukka/quickhull](https://github.com/akuukka/quickhull) (public domain) for convex hull construction.

The loader (`utils/common/physdll.cpp`) probes per-OS filenames:

- Windows: `vphysics.dll`
- Linux: `libvphysics.so`, `vphysics.so`
- macOS: `libvphysics.dylib`, `vphysics.dylib`

Search order: `<exe_dir>/` → `<exe_dir>/bin/` → bare filename (standard loader paths) → `<gamedir>/bin/` → `<gameroot>/bin/`. The exe-dir probe (resolved via `_NSGetExecutablePath` on macOS, `/proc/self/exe` on Linux) is what makes co-located `libvphysics.*` work without setting `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH`. To use a user-supplied vphysics instead of the bundled one, drop it into any of those locations — the first found wins.

### Tests

With the vphysics module enabled, CTest targets are registered:

```sh
ctest --test-dir build --output-on-failure
```

Runs four phase tests:
- `phase4a_smoke` — dlopen + `CreateInterface` factory
- `phase4b_convex` — `ConvexFromVerts` against analytical cube / tetrahedron
- `phase4d_serialize` — round-trip `.phy` structure
- `phase4e_surfaceprops` — KV1 surfaceproperties parser

Disable with `-DMDLFORGE_VPHYSICS_TESTS=OFF`.

A `.phy` dumper lives at `vendor/vphysics/tools/phy_dump.cpp` for inspecting any `.phy` file's structure — useful when diagnosing model loading issues in-game.

## studiomdl changes

- **`$jigglebone`** — physics jiggle bones, same QC syntax and defaults as the Source SDK 2013
  studiomdl: `is_flexible`, `is_rigid`, `has_base_spring` and `is_boing` blocks (`length`,
  `tip_mass`, `pitch_/yaw_/along_stiffness` and `_damping`, `angle_constraint`, `yaw_constraint`,
  `pitch_constraint`, `allow_length_flex`, `base_mass`, `left_/up_/forward_constraint`, …).
  Written as `mstudiojigglebone_t` (`STUDIO_PROC_JIGGLE`, `BONE_ALWAYS_PROCEDURAL`). Works with
  `$include`, so a model can keep its jiggle rules in a separate `.qci`.
- **Models over 32,767 vertices** — the LOD vertex reorder (`write.cpp`) kept each mesh's map into
  the flattened vertex list as `short`, so every vertex past 32,767 got a wrapped index and the
  VTX referenced garbage. Meshes after that point rendered as nothing in game. The map is now
  `int`, and it's built in one pass instead of an O(n²) scan.
- **Fast SMD loading** — vertex unification (`lookup_index`) uses a hash on (material, position,
  texcoord) instead of a linear scan over every vertex so far. The output is identical, and a
  65k-triangle SMD compiles in under 2 minutes instead of about 13.
- **More source triangles** — `MAXSTUDIOTRIANGLES` raised from 65,536 to 262,144. The SMD loader
  kept faces in fixed arrays of that size and silently overran them (memory corruption, crash
  later), and it now stops with a clear error if a model ever exceeds the limit. The engine's real
  limits (per-mesh vertices/indices, 53 bones per strip) are unaffected.
- **`MDLFORGE_DEBUG_MESHES=1`** — prints each source mesh's vertex and face ranges and flags any
  face index outside its mesh.
