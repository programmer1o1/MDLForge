# vendor/vphysics — Architecture

Clean-room implementation of the Valve Source `IPhysicsCollision` and
`IPhysicsSurfaceProps` interfaces, sufficient for `studiomdl` to compile
`$collisionmodel` and `$collisionjoints` blocks into `.phy` files on any
platform. No code from Valve's unreleased `vphysics.dll` or from any leaked
Valve source tree was used.

## High-level picture

```
            studiomdl (the tool)
                 │
   utils/studiomdl/collisionmodel.cpp
                 │  calls IPhysicsCollision::*
                 ▼
   utils/common/physdll.cpp  ── dlopen ──▶  libvphysics.{dylib,so,dll}
                                             │
                                             ▼
                            vendor/vphysics/src/
                            ├── vphysics_module.cpp   ← factory + vtables
                            ├── cphysconvex.{h,cpp}   ← hull geometry
                            ├── cphyscollide.{h,cpp}  ← aggregate
                            ├── phy_format.h          ← binary layout
                            ├── phy_serialize.cpp     ← CollideWrite
                            └── surfaceprops.{h,cpp}  ← KV1 parser
                                             │
                                             ▼
                            vendor/quickhull/*        ← hull algorithm
```

Studiomdl talks to this module via two C++ interfaces resolved at runtime
through `dlopen` + a C symbol called `CreateInterface`. The module load path
is what Phase 1 fixed; everything inside the library is Phase 4.

---

## 1. The loader — `utils/common/physdll.cpp`

The original had one line that broke everything non-Windows:

```cpp
pPhysicsModule = Sys_LoadModule( "VPHYSICS.DLL" );
```

On Linux/macOS `dlopen("VPHYSICS.DLL")` just returns `NULL` — no such file.
The fix is a per-OS filename table:

```cpp
static const char *const kVPhysicsModuleNames[] = {
#if defined( _WIN32 )
    "vphysics.dll",
#elif defined( __APPLE__ )
    "libvphysics.dylib", "vphysics.dylib",
#else
    "libvphysics.so", "vphysics.so",
#endif
    NULL
};
```

`TryLoadInDir(dir, subdir)` iterates the table against each search root
(`<gamedir>/bin/`, `<gameroot>/bin/`). Three probe paths × N filename
candidates, first hit wins.

**Key design choice.** We don't try to change the DLL search behavior
inside `Sys_LoadModule` (that's deep in `tier1/interface.cpp` and has
platform assumptions). We just hand it the right filename up front and let
its existing logic (which already handles `dlopen` on POSIX) do its thing.

---

## 2. The factory — `vendor/vphysics/src/vphysics_module.cpp`

This file is the whole public surface. It exports exactly one C symbol:

```cpp
extern "C" MDLFORGE_VPHYSICS_EXPORT void *
CreateInterface( const char *pName, int *pReturnCode )
{
    if ( std::strcmp( pName, VPHYSICS_COLLISION_INTERFACE_VERSION ) == 0 )
        return static_cast<IPhysicsCollision *>( &g_Collision );
    if ( std::strcmp( pName, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION ) == 0 )
        return static_cast<IPhysicsSurfaceProps *>( &g_SurfaceProps );
    return nullptr;
}
```

Two singletons, no heap allocations at load time. The
`MDLFORGE_VPHYSICS_EXPORT` macro expands to `__declspec(dllexport)` on
Windows and `__attribute__((visibility("default")))` on POSIX — required
because the CMake target sets `CXX_VISIBILITY_PRESET hidden` so only
intentionally-exported symbols appear in the dynsym table.

`CMDLForgePhysicsCollision` subclasses `IPhysicsCollision` and overrides
**all ~40 virtual methods** even though studiomdl only calls 11.
Why? Because the vtable layout must match — C++ ABI puts methods in
declaration order, and skipping one means callers invoke the wrong slot.
The 29 unused methods are nop stubs (`return 0`, `return nullptr`, `{}`).

The interesting overrides delegate to internal types via opaque casts:

```cpp
CPhysConvex *ConvexFromVerts( Vector **ppVerts, int vertCount ) override
{
    return AsOpaque( CPhysConvex_Impl::BuildFromVerts( ppVerts, vertCount ) );
}
```

`CPhysConvex` is forward-declared in Valve's public header — from
studiomdl's perspective it's an opaque pointer. We give it meaning as
`CPhysConvex_Impl` in our tree only; two helpers (`AsImpl`, `AsOpaque`)
keep the `reinterpret_cast` in one place.

---

## 3. The convex hull — `cphysconvex.{h,cpp}`

`CPhysConvex_Impl` is the unit of geometry — one convex hull:

```cpp
class CPhysConvex_Impl {
public:
    std::vector<Vector>   points;     // hull vertices
    std::vector<uint16_t> triangles;  // 3 indices per triangle, CCW
    Vector                aabbMin, aabbMax;
    float                 volume;
    float                 surfaceArea;
    unsigned int          gameData;
};
```

`BuildFromVerts` is where quickhull does the real work:

```cpp
quickhull::QuickHull<float> qh;
auto hull = qh.getConvexHull( &cloud[0].x, cloud.size(),
                              /*CCW*/true, /*useOriginalIndices*/false );
```

Two flags matter:

- **`CCW=true`** — triangles wind counterclockwise when viewed from
  outside. IVP expects this for correct inside/outside tests.
- **`useOriginalIndices=false`** — quickhull produces a compact vertex
  buffer containing only points that actually survived on the hull.
  Cleaner output, smaller `.phy`.

After hull construction we compute three derived quantities:

### Volume — via the divergence theorem

Each triangle contributes `(1/6) · a · (b × c)` to a signed sum; `abs()`
makes it origin-independent:

```cpp
sixV += DotProduct( a, CrossProduct( b, c ) );
return std::fabs( sixV ) / 6.0f;
```

This works because any closed triangulated surface encloses a volume equal
to the signed sum of tetrahedra formed with any reference point (we
implicitly use the origin).

### Surface area

Straight sum of `0.5 · |cross(b-a, c-a)|` per triangle. No tricks.

### AABB

Per-axis min/max scan over all hull points.

The `< 4` vertex check and `< 0xFFFF` upper bound are explicit: quickhull
needs at least 4 non-coplanar points for a 3D hull, and studiomdl
traditionally uses 16-bit point indices in the `.phy` (see the edge
bit-packing in section 6).

---

## 4. The aggregate — `cphyscollide.{h,cpp}`

`CPhysCollide_Impl` owns an array of convexes plus whole-model metadata:

```cpp
class CPhysCollide_Impl {
    std::vector<CPhysConvex_Impl *> convexes;  // owned
    Vector massCenter, orthographicAreas;
    Vector aabbMin, aabbMax;
    ~CPhysCollide_Impl() { for (auto *c : convexes) delete c; }
};
```

### Ownership transfer

The Valve header comment on `ConvertConvexToCollide` says _"this deletes
the convex elements"_. So after studiomdl calls it, the old
`CPhysConvex*` pointers are invalid — they've been consumed into the new
`CPhysCollide*`. Our destructor honors that by deleting all the convexes.
Studiomdl never calls `DestroyCollide` in the one-shot compile flow (it
just exits), so we effectively leak — acceptable for a one-shot tool.

### Mass center

Valve's formula is proper volume-integrated centroid; we approximate with
volume-weighted AABB centers:

```cpp
const Vector center = 0.5f * ( c->aabbMin + c->aabbMax );
accum    += center * c->volume;
totalVol += c->volume;
massCenter = accum / totalVol;
```

Cosmetically different from Valve's output but studiomdl always overrides
via the `$masscenter` QC keyword anyway. For the unit cube both
approximations give `(0.5, 0.5, 0.5)`.

### `CollideGetAABB`

Takes origin + angles and returns a world-space AABB. Computed by
transforming all 8 corners of the local AABB (not just min/max — that's
wrong under rotation) and taking the world-space per-axis bounds:

```cpp
matrix3x4_t xform;
AngleMatrix( angles, origin, xform );
for ( int i = 0; i < 8; ++i ) {
    Vector local(
        (i & 1) ? aabbMax.x : aabbMin.x,
        (i & 2) ? aabbMax.y : aabbMin.y,
        (i & 4) ? aabbMax.z : aabbMin.z );
    VectorTransform( local, xform, corners[i] );
}
// then min/max over corners
```

Cheap and exact for oriented-bounding transformation.

---

## 5. The binary format — `phy_format.h`

Every struct has a `static_assert` on its size so ABI drift breaks loudly:

```cpp
static_assert( sizeof( CompactSurfaceHeader )  == 28, "..." );
static_assert( sizeof( IVPCompactSurface )     == 48, "..." );
static_assert( sizeof( IVPCompactLedgeNode )   == 28, "..." );
static_assert( sizeof( IVPCompactLedge )       == 16, "..." );
static_assert( sizeof( IVPCompactTriangle )    == 16, "..." );
```

All bitfield-heavy values get **hand-packed** via `constexpr` helpers
rather than using C++ bitfield syntax (which has
implementation-defined layout):

```cpp
static inline uint32_t PackLedgeFlagsAndSize(
    uint32_t hasChildren, uint32_t isCompact,
    uint32_t dummy, uint32_t sizeDiv16 )
{
    return ( hasChildren & 0x3u )
         | ( ( isCompact & 0x3u ) << 2 )
         | ( ( dummy     & 0xFu ) << 4 )
         | ( ( sizeDiv16 & 0x00FFFFFFu ) << 8 );
}
```

Hand-packing means the layout is identical across compilers and platforms
— GCC/Clang/MSVC all produce the same wire bytes. The alternative
(`uint32_t hasChildren:2; uint32_t isCompact:2; ...`) has MSVC ordering
that can differ from GCC.

Magic constants are defined as 4-byte little-endian encodings of ASCII:

```cpp
static constexpr uint32_t kVPhysicsID_VPHY =
    'V' | ('P'<<8) | ('H'<<16) | ('Y'<<24);
static constexpr uint32_t kIVPS =
    'I' | ('V'<<8) | ('P'<<16) | ('S'<<24);
```

---

## 6. The serializer — `phy_serialize.cpp`

Output layout per collide:

```
offset 0    CompactSurfaceHeader  (28 bytes) ── VPHY, version, modelType,
                                                surfaceSize, drag, axisMapSize
offset 28   IVPCompactSurface     (48 bytes) ── massCenter, rotInertia,
                                                upperLimitRadius,
                                                offsetLedgetreeRoot, IVPS
offset 76   ledgetree nodes       (28 each)  ── 2N-1 for N convexes,
                                                left-spine layout
            ledges + triangles + points      ── per-convex blocks
```

### The ledgetree layout

For N convexes we need N terminal nodes + (N-1) internal nodes = 2N-1
total. We build a **left-spine binary tree** where the first leaf goes to
the root's right child, and the remaining leaves form the left subtree
recursively:

```cpp
size_t PlanSubtree( size_t firstLeaf, size_t leafCount,
                    std::vector<NodeMeta> &nodes, ... )
{
    if ( leafCount == 1 ) { /* emit terminal */ return idx; }
    size_t rootIdx = nodes.size();
    nodes.push_back( {} );                                // reserve slot
    size_t leftIdx  = PlanSubtree( firstLeaf + 1,
                                    leafCount - 1, ... ); // recurse
    size_t rightIdx = PlanSubtree( firstLeaf, 1, ... );   // single leaf
    nodes[rootIdx].leftNodeOffset  = ...;
    nodes[rootIdx].rightNodeOffset = ...;
}
```

Why left-spine? Because IVP's convention is **left child immediately
follows the internal node** in memory — the offset is implicit (+28). The
right child's offset is explicit (`offsetRightNode`). So by recursing left
first and writing the right child after the entire left subtree is laid
out, we satisfy the invariant "left child follows parent" naturally.

Planning is two-pass:

1. **Pass 1** walks the tree assigning node offsets into a `NodeMeta`
   vector (no bytes written yet).
2. **Pass 2** iterates leaf nodes and assigns ledge-block offsets after
   all nodes.

Then a third pass emits actual bytes: write the header and surface, write
each node (computing leaf bounding spheres from the convex AABB), then
walk nodes in **reverse** to compute internal-node bounding spheres from
their children — reverse because an internal node's sphere encloses its
children, so we want the children's spheres written first:

```cpp
for ( size_t i = nodes.size(); i-- > 0; )
{
    if ( nodes[i].terminal ) continue;
    SetInternalNodeBounds( *node, *left, *right );
}
```

### The leaf block

For each convex: one ledge + its triangles + its points, contiguous:

```cpp
ledge->cPointOffset = kLedgeSize + nTri * kTriSize;
ledge->flagsAndSizeDiv16 = PackLedgeFlagsAndSize(
    /*has_children*/ 0,
    /*is_compact*/   1,
    /*dummy*/        0,
    /*size_div_16*/  ledgeBytes / 16 );
```

Triangles use IVP's bit-packed triangle header (tri index, pierce index,
material, virtual flag) and three 4-byte edges encoding start-point,
opposite-edge-index, and virtual flag. We emit `opposite_index = 0` for
all edges — this is adjacency information used for ray-cast
acceleration; omitting it means ray-casts are slower but still correct.
Studiomdl doesn't care — studiomdl is a compiler, not a runtime.

Points are padded to 16 bytes (`float xyz[4]` with `xyz[3]=0`) — IVP
loads them as SSE vectors so they need alignment.

### The bug caught via niktoper

The first version had a 32-byte `CompactSurfaceHeader` with a leading
`int size` field. Some public references (bspfuzz) include this field
because they fold studiomdl's externally-written `int solidSize` prefix
into the struct. But `CollideWrite` itself shouldn't emit that byte —
studiomdl writes it separately in `write.cpp`. A hexdump of a real
ragdoll `.phy` showed the `VPHY` magic at offset 0 of the `CollideWrite`
block, not offset 4. Off-by-4. The fix: remove the `size` field and
adjust `kHeaderSize` from 32 to 28, shifting every downstream offset.

**This is the class of bug unit tests cannot catch** — our tests
validated internal self-consistency, but the spec itself was wrong. Only
diffing against a real file reveals it.

---

## 7. The KV1 parser — `surfaceprops.{h,cpp}`

Two files: a hand-rolled lexer and the table.

`KV1Lexer::Next` returns one token at a time — a quoted string, a brace,
or a bare identifier — skipping whitespace and both `//` line and
`/* */` block comments. `Parse` runs a small state machine: expect a
name, expect `{`, consume key/value pairs until `}`, repeat.

The table keeps an `std::unordered_map<std::string, int>` for
name → index and a `std::vector<SurfacePropEntry>` for entries
themselves. Each entry records which fields came from KV1 explicitly vs.
defaulted, so `GetEffective` can walk the `base` chain for fields the
immediate entry didn't set:

```cpp
while ( cur && (!haveD || !haveT || !haveF || !haveE) && depth++ < 8 ) {
    int bi = IndexOf( cur->base.c_str() );
    cur = At( bi );
    if ( !haveD && cur->hasDensity ) { d = cur->density; haveD = true; }
    // ...
}
```

Cycle protection via `depth < 8`. Studiomdl only reads `density` and
`thickness`; the other two are parsed for completeness.

---

## 8. The tests

Four CTest-registered executables live in `vendor/vphysics/tests/`. They
all follow the same pattern: `dlopen` the library as a black box,
`dlsym("CreateInterface")`, declare a minimum `IPhysicsCollision` vtable
prefix, cast the factory result to it, and exercise.

The tests **never link against our library** — they're pure runtime
integration tests that validate the dlopen path works end-to-end, the
vtable layout is right, and the behavior matches studiomdl's expectations.

- **`phase4a_smoke`** — asserts `CreateInterface("VPhysicsCollision007")`
  and `CreateInterface("VPhysicsSurfaceProps001")` return non-null and
  that an unknown name returns null.
- **`phase4b_convex`** — analytical verification: unit cube has
  volume = 1 and surface area = 6; regular tetrahedron at (±1,±1,±1)
  (even parity) has volume = 8/3 and surface area = 8√3. Both within
  1e-3 tolerance.
- **`phase4d_serialize`** — round-trips a cube through the factory,
  re-parses the output bytes using replica layout structs, and asserts
  magic values, field values, and ledge flag bits.
- **`phase4e_surfaceprops`** — parses three entries with `base`
  inheritance and asserts the inheritance chain works correctly.

Run all four with:

```sh
ctest --test-dir build --output-on-failure
```

---

## 9. The diagnostic tool

`vendor/vphysics/tools/phy_dump.cpp` reads any `.phy` file, walks the
outer `phyheader_t`, iterates solids, and for each one parses the
`CompactSurfaceHeader` + `IVPCompactSurface` + ledgetree. Recursive walk
with bounds checks so malformed files don't crash.

This is what caught the Phase 4d header bug — running it on a real
ragdoll `.phy` showed garbage, hexdumping showed the 4-byte offset,
fixing the layout made the dumper produce sensible output on the next
run.

Compile:

```sh
c++ -std=c++17 vendor/vphysics/tools/phy_dump.cpp -o phy_dump
./phy_dump path/to/any/model.phy
```

---

## 10. The CMake glue

Three knobs in top-level `CMakeLists.txt`:

- **`STUDIOMDL_REQUIRE_32BIT`** — pre-existing, defaults ON for
  Source-compat.
- **`MDLFORGE_BUILD_VPHYSICS`** — defaults ON, gates
  `add_subdirectory(vendor/vphysics)`.
- **`MDLFORGE_VPHYSICS_TESTS`** — defaults ON, enables CTest + test
  executables.

The vphysics CMakeLists does one clever thing: it co-locates the built
library next to the `studiomdl` binary via generator expressions, so the
Phase 1 loader's `<exe_dir>` probe finds it with zero install steps:

```cmake
set_target_properties(mdlforge_vphysics PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY $<TARGET_FILE_DIR:studiomdl>
    RUNTIME_OUTPUT_DIRECTORY $<TARGET_FILE_DIR:studiomdl>
)
```

It also links against `studiomdl_mathlib` for `AngleMatrix` /
`VectorTransform` (used in `CollideGetAABB`) and pulls in
`tier0_stubs.cpp` because mathlib transitively references `HushAsserts`
and `GetCPUInformation`.

---

## End-to-end data flow

A studiomdl invocation with `$collisionmodel` in the QC:

1. Studiomdl parses QC, reads SMD mesh verts.
2. `DoCollisionModel()` calls `PhysicsDLLPath("VPHYSICS.DLL")` → falls
   through to `LoadPhysicsModule()` → probes `libvphysics.dylib` in exe
   dir → `dlopen` succeeds.
3. `Sys_GetFactory` → `dlsym("CreateInterface")` → returns our factory.
4. Studiomdl calls `CreateInterface("VPhysicsCollision007")` → we return
   `&g_Collision`.
5. For each mesh/bone, studiomdl calls `ConvexFromVerts(verts, count)` →
   quickhull runs → we return a `CPhysConvex_Impl *` cast to opaque.
6. Studiomdl may call `ConvexVolume`, `ConvexSurfaceArea`,
   `SetConvexGameData`.
7. Studiomdl calls `ConvertConvexToCollideParams(convexes, N, params)` →
   we build a `CPhysCollide_Impl` that owns the convexes.
8. Studiomdl calls `CollideGetAABB`, `CollideSetMassCenter`.
9. At write time: `CollideSize(c)` returns our computed total → studiomdl
   `stackalloc`s that many bytes → `CollideWrite(buf, c)` emits the IVP
   binary.
10. Studiomdl writes its own outer `phyheader_t` + `int solidSize`
    prefix + our bytes + KV1 text trailer to `model.phy`.
11. Process exits. No cleanup needed.

Every step is testable from either side — via the dlopen tests against
our library, or via `phy_dump` against the `.phy` output.

---

## Licensing

| Component | License | Provenance |
|---|---|---|
| `vendor/vphysics/src/*` | MIT (our code) | Clean-room — structured from Valve public SDK headers + public format docs |
| `vendor/vphysics/tests/*` | MIT | Our tests |
| `vendor/vphysics/tools/*` | MIT | Our tools |
| `vendor/quickhull/*` | Public Domain | Vendored from [akuukka/quickhull](https://github.com/akuukka/quickhull) @ 4ef66c6 |

See `vendor/vphysics/LICENSE` and `vendor/quickhull/UPSTREAM.txt` for full text.
