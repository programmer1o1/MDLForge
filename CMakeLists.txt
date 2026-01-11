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
if(STUDIOMDL_REQUIRE_32BIT AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(
        FATAL_ERROR
        "MDLForge currently targets 32-bit (x86) builds. "
        "On Windows use -A Win32. On GCC/Clang use -m32. "
        "Set -DSTUDIOMDL_REQUIRE_32BIT=OFF to try a 64-bit build."
    )
endif()

# Build only the studiomdl tool.
add_subdirectory(utils/studiomdl)
