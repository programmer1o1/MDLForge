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

static CSysModule *pPhysicsModule = NULL;

static CSysModule *LoadPhysicsModule()
{
	if ( pPhysicsModule )
		return pPhysicsModule;

	pPhysicsModule = Sys_LoadModule( "VPHYSICS.DLL" );
	if ( pPhysicsModule )
		return pPhysicsModule;

	// For most shipped Source games, the engine DLLs live in <game root>/bin,
	// while -game points at the mod directory (e.g. "<game root>/hl2").
	if ( gamedir[0] )
	{
		char modDir[1024];
		Q_strncpy( modDir, gamedir, sizeof( modDir ) );
		Q_StripTrailingSlash( modDir );

		char candidate[1024];

		// Some mods ship their own bin directory.
		Q_snprintf( candidate, sizeof( candidate ), "%s/bin/VPHYSICS.DLL", modDir );
		Q_FixSlashes( candidate );
		pPhysicsModule = Sys_LoadModule( candidate );
		if ( pPhysicsModule )
			return pPhysicsModule;

		// Try parent/bin (e.g. "<game root>/bin").
		char gameRoot[1024];
		Q_strncpy( gameRoot, modDir, sizeof( gameRoot ) );
		if ( Q_StripLastDir( gameRoot, sizeof( gameRoot ) ) )
		{
			Q_StripTrailingSlash( gameRoot );
			Q_snprintf( candidate, sizeof( candidate ), "%s/bin/VPHYSICS.DLL", gameRoot );
			Q_FixSlashes( candidate );
			pPhysicsModule = Sys_LoadModule( candidate );
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
