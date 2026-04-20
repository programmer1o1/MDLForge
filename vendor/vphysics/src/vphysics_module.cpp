// Clean-room implementation of Valve's vphysics module, scoped to the narrow
// subset studiomdl requires for .phy file generation.
//
// This file provides only factory/vtable scaffolding — every method is a
// stub returning a sentinel value. Real geometry lives in later phases.
//
// The interface declarations come from the public Source SDK headers. No
// code from Valve's unreleased vphysics.dll or from leaked source trees has
// been used.

#include "vphysics_interface.h"
#include "tier1/interface.h"
#include "mathlib/mathlib.h"

#include "cphysconvex.h"
#include "cphyscollide.h"
#include "phy_serialize.h"
#include "surfaceprops.h"

// Disable Valve's min/max macros before using STL algorithm functions.
#include "tier0/valve_minmax_off.h"

#include <algorithm>
#include <cstring>

using mdlforge_vphysics::CPhysConvex_Impl;
using mdlforge_vphysics::CPhysCollide_Impl;

static CPhysConvex_Impl *AsImpl( CPhysConvex *p )
{
	return reinterpret_cast<CPhysConvex_Impl *>( p );
}

static CPhysConvex *AsOpaque( CPhysConvex_Impl *p )
{
	return reinterpret_cast<CPhysConvex *>( p );
}

static CPhysCollide_Impl *AsImpl( CPhysCollide *p )
{
	return reinterpret_cast<CPhysCollide_Impl *>( p );
}

static const CPhysCollide_Impl *AsImpl( const CPhysCollide *p )
{
	return reinterpret_cast<const CPhysCollide_Impl *>( p );
}

static CPhysCollide *AsOpaque( CPhysCollide_Impl *p )
{
	return reinterpret_cast<CPhysCollide *>( p );
}

namespace
{

// ---------------------------------------------------------------------------
// IPhysicsCollision stub
// ---------------------------------------------------------------------------
class CMDLForgePhysicsCollision : public IPhysicsCollision
{
public:
	~CMDLForgePhysicsCollision() override {}

	CPhysConvex *ConvexFromVerts( Vector **ppVerts, int vertCount ) override
	{
		return AsOpaque( CPhysConvex_Impl::BuildFromVerts( ppVerts, vertCount ) );
	}
	CPhysConvex *ConvexFromPlanes( float *, int, float ) override { return nullptr; }
	float ConvexVolume( CPhysConvex *p ) override
	{
		auto *c = AsImpl( p );
		return c ? c->volume : 0.0f;
	}

	CPhysCollide *ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount ) override
	{
		if ( pConvex == nullptr || convexCount <= 0 )
			return nullptr;
		// Caller transfers ownership; consume.
		std::vector<CPhysConvex_Impl *> items;
		items.reserve( convexCount );
		for ( int i = 0; i < convexCount; ++i )
			items.push_back( AsImpl( pConvex[i] ) );
		return AsOpaque( CPhysCollide_Impl::Build( items.data(), static_cast<int>( items.size() ) ) );
	}

	float ConvexSurfaceArea( CPhysConvex *p ) override
	{
		auto *c = AsImpl( p );
		return c ? c->surfaceArea : 0.0f;
	}
	void SetConvexGameData( CPhysConvex *p, unsigned int gameData ) override
	{
		if ( auto *c = AsImpl( p ) )
			c->gameData = gameData;
	}
	void ConvexFree( CPhysConvex *p ) override
	{
		delete AsImpl( p );
	}

	CPhysPolysoup *PolysoupCreate() override { return nullptr; }
	void PolysoupDestroy( CPhysPolysoup * ) override {}
	void PolysoupAddTriangle( CPhysPolysoup *, const Vector &, const Vector &, const Vector &, int ) override {}
	CPhysCollide *ConvertPolysoupToCollide( CPhysPolysoup *, bool ) override { return nullptr; }

	int CollideSize( CPhysCollide *pCollide ) override
	{
		return mdlforge_vphysics::ComputeCollideSize( AsImpl( pCollide ) );
	}
	int CollideWrite( char *pDest, CPhysCollide *pCollide ) override
	{
		if ( pDest == nullptr )
			return 0;
		return mdlforge_vphysics::SerializeCollide( reinterpret_cast<uint8_t *>( pDest ), AsImpl( pCollide ) );
	}

	void DestroyCollide( CPhysCollide *pCollide ) override
	{
		delete AsImpl( pCollide );
	}
	float CollideVolume( CPhysCollide * ) override { return 0.0f; }
	float CollideSurfaceArea( CPhysCollide * ) override { return 0.0f; }

	Vector CollideGetExtent( const CPhysCollide *, const Vector &origin, const QAngle &, const Vector & ) override { return origin; }
	void CollideGetAABB( Vector &mins, Vector &maxs, const CPhysCollide *pCollide, const Vector &origin, const QAngle &angles ) override
	{
		const auto *c = AsImpl( pCollide );
		if ( c == nullptr )
		{
			mins.Init();
			maxs.Init();
			return;
		}
		// Compute rotated+translated AABB via the 8-corner transform. Cheap
		// and always correct.
		matrix3x4_t xform;
		AngleMatrix( angles, origin, xform );

		Vector corners[8];
		for ( int i = 0; i < 8; ++i )
		{
			Vector local(
				( i & 1 ) ? c->aabbMax.x : c->aabbMin.x,
				( i & 2 ) ? c->aabbMax.y : c->aabbMin.y,
				( i & 4 ) ? c->aabbMax.z : c->aabbMin.z );
			VectorTransform( local, xform, corners[i] );
		}
		mins = maxs = corners[0];
		for ( int i = 1; i < 8; ++i )
		{
			mins.x = std::min( mins.x, corners[i].x );
			mins.y = std::min( mins.y, corners[i].y );
			mins.z = std::min( mins.z, corners[i].z );
			maxs.x = std::max( maxs.x, corners[i].x );
			maxs.y = std::max( maxs.y, corners[i].y );
			maxs.z = std::max( maxs.z, corners[i].z );
		}
	}

	CPhysCollide *BBoxToCollide( const Vector &, const Vector & ) override { return nullptr; }

	void TraceBox( const Vector &, const Vector &, const Vector &, const Vector &, const CPhysCollide *, const Vector &, const QAngle &, trace_t * ) override {}
	void TraceBox( const Ray_t &, const CPhysCollide *, const Vector &, const QAngle &, trace_t * ) override {}

	void TraceCollide( const Vector &, const Vector &, const CPhysCollide *, const QAngle &, const CPhysCollide *, const Vector &, const QAngle &, trace_t * ) override {}

	void VCollideLoad( vcollide_t *pOutput, int, const char *, int ) override
	{
		if ( pOutput )
			std::memset( pOutput, 0, sizeof( *pOutput ) );
	}
	void VCollideUnload( vcollide_t * ) override {}

	IVPhysicsKeyParser *VPhysicsKeyParserCreate( const char * ) override { return nullptr; }
	void VPhysicsKeyParserDestroy( IVPhysicsKeyParser * ) override {}

	int CreateDebugMesh( CPhysCollide const *, Vector ** ) override { return 0; }
	void DestroyDebugMesh( int, Vector * ) override {}

	ICollisionQuery *CreateQueryModel( CPhysCollide * ) override { return nullptr; }
	void DestroyQueryModel( ICollisionQuery * ) override {}

	IPhysicsCollision *ThreadContextCreate() override { return this; }
	void ThreadContextDestroy( IPhysicsCollision * ) override {}

	unsigned int ReadStat( int ) override { return 0; }

	void TraceBox( const Ray_t &, unsigned int, IConvexInfo *, const CPhysCollide *, const Vector &, const QAngle &, trace_t * ) override {}

	void CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter ) override
	{
		if ( !pOutMassCenter )
			return;
		const auto *c = AsImpl( pCollide );
		*pOutMassCenter = c ? c->massCenter : Vector( 0.0f, 0.0f, 0.0f );
	}
	void CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter ) override
	{
		if ( auto *c = AsImpl( pCollide ) )
			c->massCenter = massCenter;
	}

	int CollideIndex( const CPhysCollide * ) override { return 0; }

	CPhysCollide *ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &params ) override
	{
		if ( pConvex == nullptr || convexCount <= 0 )
			return nullptr;
		std::vector<CPhysConvex_Impl *> items;
		items.reserve( convexCount );
		for ( int i = 0; i < convexCount; ++i )
			items.push_back( AsImpl( pConvex[i] ) );
		CPhysCollide_Impl *collide = CPhysCollide_Impl::Build( items.data(), static_cast<int>( items.size() ) );
		if ( collide != nullptr && params.buildDragAxisAreas )
		{
			// Proper drag-axis computation would project each convex onto the
			// three OBB axes and sum the projected areas. Valve's default when
			// the flag is off is (1,1,1); we keep that as a conservative
			// fallback and leave precise projection for a future pass.
			collide->orthographicAreas.Init( 1.0f, 1.0f, 1.0f );
		}
		return AsOpaque( collide );
	}
	CPhysConvex *BBoxToConvex( const Vector &, const Vector & ) override { return nullptr; }

	Vector CollideGetOrthographicAreas( const CPhysCollide *pCollide ) override
	{
		const auto *c = AsImpl( pCollide );
		return c ? c->orthographicAreas : Vector( 1.0f, 1.0f, 1.0f );
	}

	void OutputDebugInfo( const CPhysCollide * ) override {}

	bool IsBoxIntersectingCone( const Vector &, const Vector &, const truncatedcone_t & ) override { return false; }
	CPhysCollide *CreateVirtualMesh( const virtualmeshparams_t & ) override { return nullptr; }
	void DumpVirtualCollideStats() override {}

	CPhysConvex *ConvexFromConvexPolyhedron( const CPolyhedron & ) override { return nullptr; }

	bool SupportsVirtualMesh() override { return false; }

	CPhysCollide *UnserializeCollide( char *, int, int ) override { return nullptr; }
	void CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas ) override
	{
		if ( auto *c = AsImpl( pCollide ) )
			c->orthographicAreas = areas;
	}

	bool GetBBoxCacheSize( int *pCachedSize, int *pCachedCount ) override
	{
		if ( pCachedSize ) *pCachedSize = 0;
		if ( pCachedCount ) *pCachedCount = 0;
		return false;
	}

#if defined( USE_PHX_FILES )
	const char *PackVCollideText( const char *, int *, bool, bool ) override { return nullptr; }
	void DestroyVCollideText( const char * ) override {}
	CPackedPhysicsDescription *CreatePackedDesc( const char *, int ) override { return nullptr; }
	void DestroyPackedDesc( CPackedPhysicsDescription * ) override {}
#endif
};

// ---------------------------------------------------------------------------
// IPhysicsSurfaceProps — parses surfaceproperties.txt into a name→properties
// table. Only the subset studiomdl actually reads (density, thickness) is
// returned with non-default values; the other surfacedata_t fields stay
// zero-initialized since studiomdl never inspects them.
// ---------------------------------------------------------------------------
class CMDLForgePhysicsSurfaceProps : public IPhysicsSurfaceProps
{
public:
	~CMDLForgePhysicsSurfaceProps() override {}

	int ParseSurfaceData( const char * /*pFilename*/, const char *pText ) override
	{
		return m_table.Parse( pText );
	}
	int SurfacePropCount() override { return m_table.Count(); }

	int GetSurfaceIndex( const char *name ) override { return m_table.IndexOf( name ); }

	void GetPhysicsProperties( int index, float *density, float *thickness, float *friction, float *elasticity ) override
	{
		if ( !m_table.GetEffective( index, density, thickness, friction, elasticity ) )
		{
			// Unknown index → zeros, matching Valve's "not found" behaviour.
			if ( density )    *density = 0.0f;
			if ( thickness )  *thickness = 0.0f;
			if ( friction )   *friction = 0.0f;
			if ( elasticity ) *elasticity = 0.0f;
		}
	}

	surfacedata_t *GetSurfaceData( int ) override { return nullptr; }
	const char *GetString( unsigned short ) override { return ""; }

	const char *GetPropName( int index ) override
	{
		const auto *e = m_table.At( index );
		return e ? e->name.c_str() : "";
	}

	void SetWorldMaterialIndexTable( int *, int ) override {}

	void GetPhysicsParameters( int index, surfacephysicsparams_t *pParamsOut ) override
	{
		if ( !pParamsOut )
			return;
		std::memset( pParamsOut, 0, sizeof( *pParamsOut ) );
		float density = 0.0f, thickness = 0.0f, friction = 0.0f, elasticity = 0.0f;
		if ( m_table.GetEffective( index, &density, &thickness, &friction, &elasticity ) )
		{
			pParamsOut->density    = density;
			pParamsOut->thickness  = thickness;
			pParamsOut->friction   = friction;
			pParamsOut->elasticity = elasticity;
		}
	}

private:
	mdlforge_vphysics::SurfacePropsTable m_table;
};

CMDLForgePhysicsCollision     g_Collision;
CMDLForgePhysicsSurfaceProps  g_SurfaceProps;

} // namespace

// ---------------------------------------------------------------------------
// Factory export — the symbol Sys_GetFactory()/dlsym/GetProcAddress looks up.
// ---------------------------------------------------------------------------
#if defined( _WIN32 )
#  define MDLFORGE_VPHYSICS_EXPORT __declspec(dllexport)
#else
#  define MDLFORGE_VPHYSICS_EXPORT __attribute__((visibility("default")))
#endif

extern "C" MDLFORGE_VPHYSICS_EXPORT void *CreateInterface( const char *pName, int *pReturnCode )
{
	if ( pReturnCode )
		*pReturnCode = 0;

	if ( pName == nullptr )
		return nullptr;

	if ( std::strcmp( pName, VPHYSICS_COLLISION_INTERFACE_VERSION ) == 0 )
		return static_cast<IPhysicsCollision *>( &g_Collision );

	if ( std::strcmp( pName, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION ) == 0 )
		return static_cast<IPhysicsSurfaceProps *>( &g_SurfaceProps );

	if ( pReturnCode )
		*pReturnCode = 1;
	return nullptr;
}
