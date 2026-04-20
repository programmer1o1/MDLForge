# MDLForge

Standalone, cross-platform build of `studiomdl` with minimal runtime dependencies.

## Build

### Windows (recommended)

Requirements: CMake 3.16+ and Visual Studio.

```powershell
cmake -S . -B build -A Win32
cmake --build build --config Release
```

Output: `build/utils/studiomdl/Release/studiomdl.exe`

This project defaults to **32-bit** builds (`STUDIOMDL_REQUIRE_32BIT=ON`) because Source 1 toolchain modules (notably `vphysics`) are typically 32-bit.

To try a 64-bit build, pass `-DSTUDIOMDL_REQUIRE_32BIT=OFF` and select an x64 toolchain (for example, `-A x64` with Visual Studio generators).

On MSVC, you can also pass `-DSTUDIOMDL_MSVC_STATIC_RUNTIME=ON` to link the runtime statically for easier redistribution.

### Linux / macOS

Requirements: CMake 3.16+ and a C++17 compiler toolchain.

```sh
cmake -S . -B build -DSTUDIOMDL_REQUIRE_32BIT=OFF
cmake --build build
```

For Source 1 `vphysics` compatibility you will generally need a 32-bit toolchain (`-m32`) and 32-bit system dependencies.

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

## Notes

### vphysics (collision compilation)

`$collisionjoints`/`$collisionmodel` compilation requires an `IPhysicsCollision` / `IPhysicsSurfaceProps` implementation loadable at runtime. MDLForge ships a bundled clean-room module under `vendor/vphysics/` that builds on all three platforms and is placed next to the `studiomdl` binary so the loader finds it automatically. No leaked Valve source is used; the implementation is MIT-licensed.

The bundled module:

- Is enabled by default via `-DMDLFORGE_BUILD_VPHYSICS=ON`. Set `OFF` to skip it if you want to provide your own `libvphysics.{so,dylib,dll}`.
- Produces `.phy` files that are structurally compatible with Valve's format (VPHY/IVPS magic, correct compact-surface + ledgetree + compactledge layout). Byte-for-byte equivalence with Valve's output is **not** a goal — convex-hull ordering and LOD/drag fields differ — but Source games should load the output without issue for all typical collision shapes. Competitive-precision use cases (surf/bhop movement servers) may notice small differences in convex silhouettes.
- Uses [akuukka/quickhull](https://github.com/akuukka/quickhull) (public domain) for convex hull construction.

The loader (`utils/common/physdll.cpp`) probes per-OS filenames:

- Windows: `vphysics.dll`
- Linux: `libvphysics.so`, `vphysics.so`
- macOS: `libvphysics.dylib`, `vphysics.dylib`

Search order remains: bare filename (standard loader paths) → `<gamedir>/bin/` → `<gameroot>/bin/`. To use a user-supplied vphysics instead of the bundled one, drop it into any of those locations — the first found wins.

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
