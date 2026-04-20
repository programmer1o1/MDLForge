cmake_minimum_required(VERSION 3.16)

project(MDLForge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(
    STUDIOMDL_REQUIRE_32BIT
    "Require 32-bit builds (recommended for Source 1 toolchain + vphysics)"
    ON
)

option(
    STUDIOMDL_MSVC_STATIC_RUNTIME
    "Link the MSVC runtime statically (/MT) for easier redistribution"
    OFF
)

option(
    MDLFORGE_BUILD_VPHYSICS
    "Build the bundled clean-room vphysics module (vendor/vphysics)"
    ON
)

option(
    MDLFORGE_VPHYSICS_TESTS
    "Build + register CTest targets for the vphysics phase tests"
    ON
)

# When the vphysics shim is built, studiomdl_mathlib/tier1 get linked into
# a shared library, so they must be position-independent.
if(MDLFORGE_BUILD_VPHYSICS)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(STUDIOMDL_REQUIRE_32BIT AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(
        FATAL_ERROR
        "MDLForge currently targets 32-bit (x86) builds. "
        "On Windows use -A Win32. On GCC/Clang use -m32. "
        "Set -DSTUDIOMDL_REQUIRE_32BIT=OFF to try a 64-bit build."
    )
endif()

if(MDLFORGE_VPHYSICS_TESTS)
    enable_testing()
endif()

# Build the studiomdl tool.
add_subdirectory(utils/studiomdl)

# Bundled clean-room vphysics. Must come after studiomdl — it uses
# $<TARGET_FILE_DIR:studiomdl> to colocate its output.
if(MDLFORGE_BUILD_VPHYSICS)
    add_subdirectory(vendor/vphysics)
endif()
