// Convex-hull construction and geometric-property computation for the
// clean-room vphysics shim. Uses akuukka/quickhull (public domain) as the
// hull builder; everything else (volume, surface area, AABB) is our code.

#include "cphysconvex.h"

// Valve's tier0 headers (pulled in by mathlib/vector.h) define min/max as
// macros via valve_minmax_on.h, which breaks std::min/std::max with MSVC.
// This header undoes that before we use STL.
#include "tier0/valve_minmax_off.h"

#include "QuickHull.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mdlforge_vphysics
{

namespace
{

// Volume of a closed triangulated surface via the divergence theorem. Taking
// abs() so the value is correct regardless of whether the origin lies inside
// the hull. Reference: https://en.wikipedia.org/wiki/Polyhedron#Volume
float ComputeVolume( const std::vector<Vector> &pts, const std::vector<uint16_t> &tris )
{
	float sixV = 0.0f;
	for ( size_t i = 0; i + 2 < tris.size(); i += 3 )
	{
		const Vector &a = pts[ tris[i + 0] ];
		const Vector &b = pts[ tris[i + 1] ];
		const Vector &c = pts[ tris[i + 2] ];
		sixV += DotProduct( a, CrossProduct( b, c ) );
	}
	return std::fabs( sixV ) / 6.0f;
}

float ComputeSurfaceArea( const std::vector<Vector> &pts, const std::vector<uint16_t> &tris )
{
	float area = 0.0f;
	for ( size_t i = 0; i + 2 < tris.size(); i += 3 )
	{
		const Vector &a = pts[ tris[i + 0] ];
		const Vector &b = pts[ tris[i + 1] ];
		const Vector &c = pts[ tris[i + 2] ];
		area += 0.5f * CrossProduct( b - a, c - a ).Length();
	}
	return area;
}

void ComputeAABB( const std::vector<Vector> &pts, Vector &outMin, Vector &outMax )
{
	if ( pts.empty() )
	{
		outMin.Init();
		outMax.Init();
		return;
	}
	outMin = outMax = pts[0];
	for ( size_t i = 1; i < pts.size(); ++i )
	{
		const Vector &p = pts[i];
		outMin.x = std::min( outMin.x, p.x );
		outMin.y = std::min( outMin.y, p.y );
		outMin.z = std::min( outMin.z, p.z );
		outMax.x = std::max( outMax.x, p.x );
		outMax.y = std::max( outMax.y, p.y );
		outMax.z = std::max( outMax.z, p.z );
	}
}

} // namespace

void CPhysConvex_Impl::RecomputeDerived()
{
	ComputeAABB( points, aabbMin, aabbMax );
	volume      = ComputeVolume( points, triangles );
	surfaceArea = ComputeSurfaceArea( points, triangles );
}

CPhysConvex_Impl *CPhysConvex_Impl::BuildFromVerts( Vector **ppVerts, int vertCount )
{
	if ( ppVerts == nullptr || vertCount < 4 )
		return nullptr;

	std::vector< quickhull::Vector3<float> > cloud;
	cloud.reserve( vertCount );
	for ( int i = 0; i < vertCount; ++i )
	{
		if ( ppVerts[i] == nullptr )
			continue;
		const Vector &v = *ppVerts[i];
		cloud.emplace_back( v.x, v.y, v.z );
	}
	if ( cloud.size() < 4 )
		return nullptr;

	quickhull::QuickHull<float> qh;
	// CCW=true so triangle winding is outward-facing; useOriginalIndices=false
	// so we get a compact vertex buffer containing only hull points.
	auto hull = qh.getConvexHull( &cloud[0].x, cloud.size(), /*CCW*/true, /*useOriginalIndices*/false );

	const auto &vb = hull.getVertexBuffer();
	const auto &ib = hull.getIndexBuffer();

	if ( vb.size() < 4 || ib.size() < 12 || ( ib.size() % 3 ) != 0 )
		return nullptr;
	// Studiomdl historically uses 16-bit indices; refuse hulls above that.
	if ( vb.size() > 0xFFFF )
		return nullptr;

	auto *c = new CPhysConvex_Impl();
	c->points.reserve( vb.size() );
	for ( const auto &p : vb )
		c->points.emplace_back( p.x, p.y, p.z );

	c->triangles.reserve( ib.size() );
	for ( size_t idx : ib )
		c->triangles.push_back( static_cast<uint16_t>( idx ) );

	c->RecomputeDerived();
	return c;
}

} // namespace mdlforge_vphysics
