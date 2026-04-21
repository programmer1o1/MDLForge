//========= Copyright © 1996-2005, Valve Corporation, All rights reserved. ============//
//
// Purpose: 
//
// $NoKeywords: $
//
//=============================================================================//
#include <stdio.h>
#include "physdll.h"
#include "filesystem_tools.h"
#include "tier1/interface.h"
#include "tier1/strtools.h"

#if defined( __APPLE__ )
#  include <mach-o/dyld.h>
#  include <limits.h>
#elif defined( __linux__ )
#  include <unistd.h>
#  include <limits.h>
#endif

static CSysModule *pPhysicsModule = NULL;

// Resolve the directory containing the current executable. Needed on
// POSIX because dlopen does not search the executable's directory for
// bare filenames the way LoadLibrary does on Windows — the engine's
// Sys_LoadModule inherits that limitation, so we have to probe
// <exe_dir> explicitly for a colocated libvphysics.
static bool GetExecutableDir( char *out, size_t outSize )
{
#if defined( _WIN32 )
	// Not needed: Sys_LoadModule already probes <exe_dir> on Windows.
	out[0] = '\0';
	return false;
#elif defined( __APPLE__ )
	char buf[PATH_MAX];
	uint32_t sz = sizeof( buf );
	if ( _NSGetExecutablePath( buf, &sz ) != 0 )
		return false;
	char resolved[PATH_MAX];
	if ( !realpath( buf, resolved ) )
		Q_strncpy( resolved, buf, sizeof( resolved ) );
	Q_strncpy( out, resolved, outSize );
	Q_StripFilename( out );
	return out[0] != '\0';
#else
	char buf[PATH_MAX];
	ssize_t n = readlink( "/proc/self/exe", buf, sizeof( buf ) - 1 );
	if ( n <= 0 )
		return false;
	buf[n] = '\0';
	Q_strncpy( out, buf, outSize );
	Q_StripFilename( out );
	return out[0] != '\0';
#endif
}

// Candidate module filenames in priority order. Valve stock is "vphysics.dll"
// on Windows; community cross-platform forks typically ship as
// "libvphysics.so"/"libvphysics.dylib", and some as "vphysics.so"/"vphysics.dylib".
static const char *const kVPhysicsModuleNames[] = {
#if defined( _WIN32 )
	"vphysics.dll",
#elif defined( __APPLE__ )
	"libvphysics.dylib",
	"vphysics.dylib",
#else
	"libvphysics.so",
	"vphysics.so",
#endif
	NULL
};

static CSysModule *TryLoadInDir( const char *dir, const char *subdir )
{
	char candidate[1024];
	for ( int i = 0; kVPhysicsModuleNames[i] != NULL; ++i )
	{
		if ( subdir && subdir[0] )
			Q_snprintf( candidate, sizeof( candidate ), "%s/%s/%s", dir, subdir, kVPhysicsModuleNames[i] );
		else
			Q_snprintf( candidate, sizeof( candidate ), "%s/%s", dir, kVPhysicsModuleNames[i] );
		Q_FixSlashes( candidate );
		CSysModule *mod = Sys_LoadModule( candidate );
		if ( mod )
			return mod;
	}
	return NULL;
}

static CSysModule *LoadPhysicsModule()
{
	if ( pPhysicsModule )
		return pPhysicsModule;

	// POSIX dlopen does not search the executable's own directory for a
	// bare filename. Probe <exe_dir> first so a studiomdl distribution
	// with libvphysics shipped alongside the binary "just works" without
	// requiring LD_LIBRARY_PATH or DYLD_LIBRARY_PATH.
	char exeDir[1024];
	if ( GetExecutableDir( exeDir, sizeof( exeDir ) ) )
	{
		pPhysicsModule = TryLoadInDir( exeDir, NULL );
		if ( pPhysicsModule )
			return pPhysicsModule;
		pPhysicsModule = TryLoadInDir( exeDir, "bin" );
		if ( pPhysicsModule )
			return pPhysicsModule;
	}

	// Bare filename — let the dynamic loader search standard paths (and, on
	// Windows, Sys_LoadModule itself probes <exe_dir> and <exe_dir>/bin).
	for ( int i = 0; kVPhysicsModuleNames[i] != NULL; ++i )
	{
		pPhysicsModule = Sys_LoadModule( kVPhysicsModuleNames[i] );
		if ( pPhysicsModule )
			return pPhysicsModule;
	}

	// For most shipped Source games, the engine DLLs live in <game root>/bin,
	// while -game points at the mod directory (e.g. "<game root>/hl2").
	if ( gamedir[0] )
	{
		char modDir[1024];
		Q_strncpy( modDir, gamedir, sizeof( modDir ) );
		Q_StripTrailingSlash( modDir );

		// Some mods ship their own bin directory.
		pPhysicsModule = TryLoadInDir( modDir, "bin" );
		if ( pPhysicsModule )
			return pPhysicsModule;

		// Try parent/bin (e.g. "<game root>/bin").
		char gameRoot[1024];
		Q_strncpy( gameRoot, modDir, sizeof( gameRoot ) );
		if ( Q_StripLastDir( gameRoot, sizeof( gameRoot ) ) )
		{
			Q_StripTrailingSlash( gameRoot );
			pPhysicsModule = TryLoadInDir( gameRoot, "bin" );
			if ( pPhysicsModule )
				return pPhysicsModule;
		}
	}

	return NULL;
}

CreateInterfaceFn GetPhysicsFactory( void )
{
	if ( !LoadPhysicsModule() )
		return NULL;

	return Sys_GetFactory( pPhysicsModule );
}

void PhysicsDLLPath( const char *pPathname )
{
        if ( !pPhysicsModule )
        {
                pPhysicsModule = Sys_LoadModule( pPathname );
				if ( !pPhysicsModule && pPathname && !Q_IsAbsolutePath( pPathname ) )
				{
					// Allow fallback search for bare filenames.
					LoadPhysicsModule();
				}
        }
}
