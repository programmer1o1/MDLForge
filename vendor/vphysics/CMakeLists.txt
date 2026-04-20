cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Clean-room vphysics shim. Output name "vphysics" → vphysics.dll on Windows,
# libvphysics.{so,dylib} on Linux/macOS, matching the names the Phase 1 loader
# probes (see utils/common/physdll.cpp).
add_library(mdlforge_vphysics SHARED
    src/vphysics_module.cpp
    src/cphysconvex.cpp
    src/cphyscollide.cpp
    src/phy_serialize.cpp
    src/surfaceprops.cpp
    # Tier0 stub implementations that mathlib references. Shared with the
    # studiomdl tree to avoid symbol duplication.
    ${PROJECT_SOURCE_DIR}/utils/studiomdl/tier0_stubs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../quickhull/QuickHull.cpp
)

set_target_properties(mdlforge_vphysics PROPERTIES
    OUTPUT_NAME       "vphysics"
    PREFIX            "lib"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# On Windows the import/export is via __declspec and the loader looks for
# vphysics.dll (no "lib" prefix). Everywhere else, the POSIX loader expects
# the "lib" prefix.
if(WIN32)
    set_target_properties(mdlforge_vphysics PROPERTIES PREFIX "")
endif()

target_include_directories(mdlforge_vphysics PRIVATE
    ${PROJECT_SOURCE_DIR}/public
    ${PROJECT_SOURCE_DIR}/public/mathlib
    ${PROJECT_SOURCE_DIR}/public/tier0
    ${PROJECT_SOURCE_DIR}/public/tier1
    ${PROJECT_SOURCE_DIR}/mathlib
    ${PROJECT_SOURCE_DIR}/vendor/quickhull
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Mirror the same platform defines studiomdl uses so that shared headers
# (mathlib/vector.h, tier0/platform.h) compile identically.
set(VPHYSICS_DEFINES STATIC_TIER0)
if(WIN32)
    list(APPEND VPHYSICS_DEFINES NOMINMAX)
    if(MSVC)
        list(APPEND VPHYSICS_DEFINES _CRT_SECURE_NO_WARNINGS)
    endif()
else()
    list(APPEND VPHYSICS_DEFINES POSIX GNUC NO_MALLOC_OVERRIDE)
    if(APPLE)
        list(APPEND VPHYSICS_DEFINES OSX _LINUX)
    else()
        list(APPEND VPHYSICS_DEFINES LINUX _LINUX)
    endif()
    list(APPEND VPHYSICS_DEFINES
        stricmp=strcasecmp
        _stricmp=strcasecmp
        strnicmp=strncasecmp
        _strnicmp=strncasecmp
        _snprintf=snprintf
        _vsnprintf=vsnprintf
        _alloca=alloca
        strcmpi=strcasecmp
    )
endif()

target_compile_definitions(mdlforge_vphysics PRIVATE ${VPHYSICS_DEFINES})

# Pull in AngleMatrix/VectorTransform from studiomdl's mathlib static lib.
target_link_libraries(mdlforge_vphysics PRIVATE studiomdl_mathlib)

# Place the library next to studiomdl in the build tree so the Phase 1
# loader's <exe_dir> search root picks it up without any install step.
set_target_properties(mdlforge_vphysics PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY $<TARGET_FILE_DIR:studiomdl>
    RUNTIME_OUTPUT_DIRECTORY $<TARGET_FILE_DIR:studiomdl>
)

include(GNUInstallDirs)
install(TARGETS mdlforge_vphysics
    LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ---- Tests --------------------------------------------------------------
# Each test is a standalone executable that dlopens libvphysics at runtime
# and validates one phase of the port. They require no linkage against the
# library — the factory plug is discovered by dlsym at test time.

if(BUILD_TESTING OR MDLFORGE_VPHYSICS_TESTS)
    enable_testing()

    foreach(phase_test
        phase4a_smoke
        phase4b_convex
        phase4d_serialize
        phase4e_surfaceprops
    )
        add_executable(${phase_test} tests/${phase_test}.cpp)
        set_target_properties(${phase_test} PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )
        if(NOT WIN32)
            target_link_libraries(${phase_test} PRIVATE ${CMAKE_DL_LIBS})
        endif()
        add_dependencies(${phase_test} mdlforge_vphysics)
        add_test(
            NAME ${phase_test}
            COMMAND ${phase_test} $<TARGET_FILE:mdlforge_vphysics>
        )
    endforeach()
endif()
