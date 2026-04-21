#include "cphyscollide.h"

// Turn off Valve's min/max macros before using std::min/std::max (see
// tier0/valve_minmax_on.h). Required for MSVC builds to compile.
#include "tier0/valve_minmax_off.h"

#include <algorithm>

namespace mdlforge_vphysics
{

CPhysCollide_Impl::~CPhysCollide_Impl()
{
	for ( CPhysConvex_Impl *c : convexes )
		delete c;
}

void CPhysCollide_Impl::RecomputeUnionAABB()
{
	if ( convexes.empty() )
	{
		aabbMin.Init();
		aabbMax.Init();
		return;
	}
	aabbMin = convexes[0]->aabbMin;
	aabbMax = convexes[0]->aabbMax;
	for ( size_t i = 1; i < convexes.size(); ++i )
	{
		const Vector &mn = convexes[i]->aabbMin;
		const Vector &mx = convexes[i]->aabbMax;
		aabbMin.x = std::min( aabbMin.x, mn.x );
		aabbMin.y = std::min( aabbMin.y, mn.y );
		aabbMin.z = std::min( aabbMin.z, mn.z );
		aabbMax.x = std::max( aabbMax.x, mx.x );
		aabbMax.y = std::max( aabbMax.y, mx.y );
		aabbMax.z = std::max( aabbMax.z, mx.z );
	}
}

void CPhysCollide_Impl::RecomputeMassCenter()
{
	// Volume-weighted centroid across all convexes. Each convex has its true
	// centroid pre-computed via tetrahedral decomposition in
	// CPhysConvex_Impl::RecomputeDerived. Using the AABB center here instead
	// would drift on asymmetric hulls and cause the engine to seat the rigid
	// body at a visibly wrong offset from its bone. Calls to
	// CollideSetMassCenter still override this when $masscenter is set in QC.
	float totalVol = 0.0f;
	Vector accum( 0.0f, 0.0f, 0.0f );
	for ( const CPhysConvex_Impl *c : convexes )
	{
		accum   += c->centroid * c->volume;
		totalVol += c->volume;
	}
	if ( totalVol > 0.0f )
		massCenter = accum / totalVol;
	else
		massCenter = 0.5f * ( aabbMin + aabbMax );
}

CPhysCollide_Impl *CPhysCollide_Impl::Build( CPhysConvex_Impl **convex, int count )
{
	if ( convex == nullptr || count <= 0 )
		return nullptr;

	auto *out = new CPhysCollide_Impl();
	out->convexes.reserve( count );
	for ( int i = 0; i < count; ++i )
	{
		if ( convex[i] != nullptr )
			out->convexes.push_back( convex[i] );
	}
	if ( out->convexes.empty() )
	{
		delete out;
		return nullptr;
	}
	out->RecomputeUnionAABB();
	out->RecomputeMassCenter();
	return out;
}

} // namespace mdlforge_vphysics
