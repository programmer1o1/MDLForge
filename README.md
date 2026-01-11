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

- `$collisionjoints`/`$collisionmodel` compilation requires the `vphysics` module to be loadable; the loader tries the mod's `bin` and the parent `bin` (common Source layout).
