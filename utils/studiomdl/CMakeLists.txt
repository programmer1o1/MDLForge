cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(STUDIOMDL_ROOT ${CMAKE_CURRENT_LIST_DIR})

# Core studiomdl sources.
set(STUDIOMDL_SOURCES
    bmpread.cpp
    collisionmodel.cpp
    fs_globals.cpp
    hardwarematrixstate.cpp
    hardwarevertexcache.cpp
    matsys.cpp
    mrmsupport.cpp
    objsupport.cpp
    optimize.cpp
    perfstats.cpp
    simplify.cpp
    studiomdl.cpp
    tier0_stubs.cpp
    tristrip.cpp
    unifylods.cpp
    v1support.cpp
    write.cpp
    # nvtristrip helper library (local copy)
    ../nvtristriplib/nvtristrip.cpp
    ../nvtristriplib/nvtristripobjects.cpp
    # Shared utils
    ../common/cmdlib.cpp
    ../common/filesystem_tools.cpp
    ../common/mstristrip.cpp
    ../common/physdll.cpp
    ../common/scriplib.cpp
    ../../public/bone_setup.cpp
    ../../public/collisionutils.cpp
    ../../public/filesystem_helpers.cpp
    ../../public/filesystem_init.cpp
    ../../public/studio.cpp
)

# tier1 subset needed by the tool.
set(TIER1_SOURCES
    ${PROJECT_SOURCE_DIR}/tier1/bitbuf.cpp
    ${PROJECT_SOURCE_DIR}/tier1/byteswap.cpp
    ${PROJECT_SOURCE_DIR}/tier1/characterset.cpp
    ${PROJECT_SOURCE_DIR}/tier1/checksum_crc.cpp
    ${PROJECT_SOURCE_DIR}/tier1/checksum_md5.cpp
    ${PROJECT_SOURCE_DIR}/tier1/convar.cpp
    ${PROJECT_SOURCE_DIR}/tier1/datamanager.cpp
    ${PROJECT_SOURCE_DIR}/tier1/generichash.cpp
    ${PROJECT_SOURCE_DIR}/tier1/interface.cpp
    ${PROJECT_SOURCE_DIR}/tier1/keyvalues.cpp
    ${PROJECT_SOURCE_DIR}/tier1/mempool.cpp
    ${PROJECT_SOURCE_DIR}/tier1/memstack.cpp
    ${PROJECT_SOURCE_DIR}/tier1/netadr.cpp
    ${PROJECT_SOURCE_DIR}/tier1/rangecheckedvar.cpp
    ${PROJECT_SOURCE_DIR}/tier1/stringpool.cpp
    ${PROJECT_SOURCE_DIR}/tier1/strtools.cpp
    ${PROJECT_SOURCE_DIR}/tier1/tier1.cpp
    ${PROJECT_SOURCE_DIR}/tier1/tokenreader.cpp
    ${PROJECT_SOURCE_DIR}/tier1/undiff.cpp
    ${PROJECT_SOURCE_DIR}/tier1/utlbuffer.cpp
    ${PROJECT_SOURCE_DIR}/tier1/utlstring.cpp
    ${PROJECT_SOURCE_DIR}/tier1/utlsymbol.cpp
)

set(MATHLIB_SOURCES
    ${PROJECT_SOURCE_DIR}/mathlib/halton.cpp
    ${PROJECT_SOURCE_DIR}/mathlib/lightdesc.cpp
    ${PROJECT_SOURCE_DIR}/mathlib/mathlib_base.cpp
    ${PROJECT_SOURCE_DIR}/mathlib/powsse.cpp
    ${PROJECT_SOURCE_DIR}/mathlib/sseconst.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/3dnow.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/almostequal.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/anorms.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/bumpvects.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/color_conversion.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/IceKey.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/imagequant.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/polyhedron.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/quantize.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/sparse_convolution_noise.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/spherical.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/sse.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/ssenoise.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/vector.cpp
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib/vmatrix.cpp
)

add_library(studiomdl_tier1 STATIC ${TIER1_SOURCES})
target_include_directories(studiomdl_tier1 PUBLIC
    ${PROJECT_SOURCE_DIR}/public
    ${PROJECT_SOURCE_DIR}/public/tier0
    ${PROJECT_SOURCE_DIR}/public/tier1
    ${PROJECT_SOURCE_DIR}/tier1
)

set(STUDIOMDL_COMMON_DEFINES STATIC_TIER0)
if(WIN32)
    list(APPEND STUDIOMDL_COMMON_DEFINES NOMINMAX)
    if(MSVC)
        list(APPEND STUDIOMDL_COMMON_DEFINES _CRT_SECURE_NO_WARNINGS)
    endif()
else()
    list(APPEND STUDIOMDL_COMMON_DEFINES POSIX)
    if(APPLE)
        list(APPEND STUDIOMDL_COMMON_DEFINES OSX _LINUX)
    else()
        list(APPEND STUDIOMDL_COMMON_DEFINES LINUX _LINUX)
    endif()
    list(APPEND STUDIOMDL_COMMON_DEFINES GNUC NO_MALLOC_OVERRIDE)

    list(APPEND STUDIOMDL_COMMON_DEFINES
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

if(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    list(APPEND STUDIOMDL_COMMON_DEFINES COMPILER_MSVC64)
endif()

target_compile_definitions(studiomdl_tier1 PRIVATE ${STUDIOMDL_COMMON_DEFINES})

add_library(studiomdl_mathlib STATIC ${MATHLIB_SOURCES})
target_include_directories(studiomdl_mathlib PUBLIC
    ${PROJECT_SOURCE_DIR}/public
    ${PROJECT_SOURCE_DIR}/public/mathlib
    ${PROJECT_SOURCE_DIR}/public/tier0
    ${PROJECT_SOURCE_DIR}/mathlib
    ${PROJECT_SOURCE_DIR}/__src2013/src/mathlib
)
target_compile_definitions(studiomdl_mathlib PRIVATE ${STUDIOMDL_COMMON_DEFINES})

add_executable(studiomdl ${STUDIOMDL_SOURCES})
target_include_directories(studiomdl PRIVATE
    ${STUDIOMDL_ROOT}
    ${PROJECT_SOURCE_DIR}/utils/nvtristriplib
    ${PROJECT_SOURCE_DIR}/public
    ${PROJECT_SOURCE_DIR}/public/mathlib
    ${PROJECT_SOURCE_DIR}/public/appframework
    ${PROJECT_SOURCE_DIR}/public/datacache
    ${PROJECT_SOURCE_DIR}/public/tier0
    ${PROJECT_SOURCE_DIR}/public/tier1
    ${PROJECT_SOURCE_DIR}/public/vstdlib
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/utils/common
    ${PROJECT_SOURCE_DIR}/mathlib
    ${PROJECT_SOURCE_DIR}/game_shared
)

target_compile_definitions(studiomdl PRIVATE STUDIOMDL ${STUDIOMDL_COMMON_DEFINES})

target_link_libraries(studiomdl PRIVATE
    studiomdl_tier1
    studiomdl_mathlib
)

if(MSVC)
    target_link_libraries(studiomdl PRIVATE legacy_stdio_definitions.lib)       
endif()

if(MSVC AND STUDIOMDL_MSVC_STATIC_RUNTIME)
    foreach(target studiomdl studiomdl_tier1 studiomdl_mathlib)
        set_property(
            TARGET ${target}
            PROPERTY MSVC_RUNTIME_LIBRARY
            "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        )
    endforeach()
endif()

find_package(Threads REQUIRED)
target_link_libraries(studiomdl PRIVATE Threads::Threads ${CMAKE_DL_LIBS})      

# Installation (optional)
include(GNUInstallDirs)
install(TARGETS studiomdl RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
