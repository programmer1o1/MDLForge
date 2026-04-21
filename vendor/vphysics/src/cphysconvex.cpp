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
#include <unordered_map>

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

// Diagonal of the inertia tensor about the centroid, per unit mass, in the
// same units as the input points. For a triangulated convex surface we
// integrate (y²+z², x²+z², x²+y²) over the enclosed volume via signed
// tetrahedra from origin, then shift to the centroid using the parallel
// axis theorem. Off-diagonal terms are dropped — callers assume the input
// coordinate frame is close enough to principal axes for typical ragdoll
// solids that the diagonal is a reasonable approximation; a full
// eigendecomposition would be more accurate but Valve's shipped .phy
// files use the same diagonal-only convention.
//
// Closed-form integrals for a tet with vertices (0, a, b, c):
//   V_tet     = (a · (b × c)) / 6
//   ∫ p² dV   = V_tet / 10 * (pa² + pb² + pc² + pa·pb + pa·pc + pb·pc)
// where pa, pb, pc are the same component (x/y/z) of a/b/c.
void ComputeInertiaDiagonal( const std::vector<Vector> &pts,
                             const std::vector<uint16_t> &tris,
                             const Vector &centroid, float volume,
                             float &Ixx, float &Iyy, float &Izz )
{
	double intXX = 0.0, intYY = 0.0, intZZ = 0.0;
	for ( size_t i = 0; i + 2 < tris.size(); i += 3 )
	{
		const Vector &a = pts[ tris[i + 0] ];
		const Vector &b = pts[ tris[i + 1] ];
		const Vector &c = pts[ tris[i + 2] ];
		const double vTet6 =
			(double) a.x * ( (double) b.y * c.z - (double) b.z * c.y ) +
			(double) a.y * ( (double) b.z * c.x - (double) b.x * c.z ) +
			(double) a.z * ( (double) b.x * c.y - (double) b.y * c.x );
		auto sq = []( double pa, double pb, double pc ) {
			return pa * pa + pb * pb + pc * pc + pa * pb + pa * pc + pb * pc;
		};
		// intP² = V_tet * sq(pa,pb,pc) / 10, and V_tet = vTet6 / 6, so the
		// coefficient is vTet6 / 60.
		const double sx = sq( a.x, b.x, c.x );
		const double sy = sq( a.y, b.y, c.y );
		const double sz = sq( a.z, b.z, c.z );
		intXX += vTet6 * ( sy + sz );   // ∫(y²+z²) dV * 60
		intYY += vTet6 * ( sx + sz );
		intZZ += vTet6 * ( sx + sy );
	}
	// Divide by 60 to finalize the integrals; by volume to get per-unit-mass.
	// fabs handles the case where quickhull emits winding that makes our
	// signed tet integrals come out negative — the magnitudes are still
	// correct, the sign just depends on whether the triangles wind CCW
	// from the perspective our formula assumes.
	const double inv = 1.0 / ( 60.0 * std::max( (double) volume, 1e-10 ) );
	double ixxOrig = std::fabs( intXX * inv );
	double iyyOrig = std::fabs( intYY * inv );
	double izzOrig = std::fabs( intZZ * inv );
	// Parallel axis: I_cm = I_origin - (cy²+cz², cx²+cz², cx²+cy²).
	Ixx = static_cast<float>( ixxOrig - ( centroid.y * centroid.y + centroid.z * centroid.z ) );
	Iyy = static_cast<float>( iyyOrig - ( centroid.x * centroid.x + centroid.z * centroid.z ) );
	Izz = static_cast<float>( izzOrig - ( centroid.x * centroid.x + centroid.y * centroid.y ) );
	// Clamp against floating-point drift on nearly-degenerate hulls.
	if ( Ixx < 1e-6f ) Ixx = 1e-6f;
	if ( Iyy < 1e-6f ) Iyy = 1e-6f;
	if ( Izz < 1e-6f ) Izz = 1e-6f;
}

// Volume-weighted centroid of the enclosed region. Each triangle (v0, v1, v2)
// forms a signed tetrahedron with the origin; the tet's centroid is
// (v0 + v1 + v2) / 4 and its signed volume is (v0 · (v1 × v2)) / 6. Summing
// vol*centroid across all triangles and dividing by total signed volume gives
// the exact centroid of the closed convex region — not the AABB center,
// which drifts on asymmetric hulls. Matches what Valve's stock vphysics
// writes into the .phy mass-center field.
Vector ComputeCentroid( const std::vector<Vector> &pts, const std::vector<uint16_t> &tris )
{
	double sixV = 0.0;
	double accX = 0.0, accY = 0.0, accZ = 0.0;
	for ( size_t i = 0; i + 2 < tris.size(); i += 3 )
	{
		const Vector &a = pts[ tris[i + 0] ];
		const Vector &b = pts[ tris[i + 1] ];
		const Vector &c = pts[ tris[i + 2] ];
		const double tet6V =
			(double) a.x * ( (double) b.y * c.z - (double) b.z * c.y ) +
			(double) a.y * ( (double) b.z * c.x - (double) b.x * c.z ) +
			(double) a.z * ( (double) b.x * c.y - (double) b.y * c.x );
		// Tet centroid contribution, multiplied by 6 to match the sixV scale;
		// the common factor of 1/24 (from /4 centroid × /6 volume) cancels in
		// the final division.
		accX += tet6V * ( (double) a.x + b.x + c.x );
		accY += tet6V * ( (double) a.y + b.y + c.y );
		accZ += tet6V * ( (double) a.z + b.z + c.z );
		sixV += tet6V;
	}
	if ( std::fabs( sixV ) < 1e-10 )
		return Vector( 0.0f, 0.0f, 0.0f );
	const double inv4V = 1.0 / ( 4.0 * sixV );
	return Vector(
		static_cast<float>( accX * inv4V ),
		static_cast<float>( accY * inv4V ),
		static_cast<float>( accZ * inv4V ) );
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

// Build the half-edge → opposite-half-edge map for a closed convex hull.
// Each triangle contributes three directed half-edges (a→b, b→c, c→a). In
// a closed manifold, every half-edge (a→b) is matched by exactly one
// half-edge (b→a) in an adjacent triangle. IVP stores the linear index
// (tri*3 + slot) of that twin inside each compact edge's opposite_index
// field — without it, the engine can't walk across triangle boundaries
// during contact resolution and the rigid body tunnels through surfaces.
std::vector<uint16_t> BuildOppositeEdges( const std::vector<uint16_t> &tris )
{
	const size_t nEdges = tris.size();
	std::vector<uint16_t> opp( nEdges, 0 );
	// Key = (from_vertex << 16) | to_vertex. Value = linear edge index.
	std::unordered_map<uint32_t, uint16_t> halfEdges;
	halfEdges.reserve( nEdges );
	for ( size_t t = 0; t + 2 < nEdges; t += 3 )
	{
		for ( size_t e = 0; e < 3; ++e )
		{
			const uint16_t from = tris[t + e];
			const uint16_t to   = tris[t + ( ( e + 1 ) % 3 )];
			const uint32_t key  = ( static_cast<uint32_t>( from ) << 16 ) | to;
			halfEdges[ key ] = static_cast<uint16_t>( t + e );
		}
	}
	for ( size_t t = 0; t + 2 < nEdges; t += 3 )
	{
		for ( size_t e = 0; e < 3; ++e )
		{
			const uint16_t from = tris[t + e];
			const uint16_t to   = tris[t + ( ( e + 1 ) % 3 )];
			const uint32_t twinKey = ( static_cast<uint32_t>( to ) << 16 ) | from;
			auto it = halfEdges.find( twinKey );
			opp[t + e] = ( it != halfEdges.end() ) ? it->second : static_cast<uint16_t>( t + e );
		}
	}
	return opp;
}

// Drop vertices that sit absurdly far from the rest of the cloud. Physics
// SMDs with cross-bone triangles sometimes hand us a handful of points from
// an adjacent bone that got dragged in via a shared triangle; without
// filtering, the hull blows out to encompass them and the resulting rigid
// body sits several inches off the bone it's supposed to follow. Valve's
// stock vphysics.dll clearly does something like this — its shipped .phy
// files stay tight to their bones even when the physics mesh is messy.
//
// Heuristic: after computing the centroid of all input verts, reject any
// vert whose distance from the centroid is greater than kOutlierRatio
// times the MEDIAN distance. Median is robust to a few extreme values in
// a way the mean isn't; if most verts cluster and a few drift far, the
// median reflects the cluster and the drifted ones get cut. We iterate
// until stable so that dropping far outliers tightens the centroid enough
// to expose any remaining ones.
void FilterOutlierVerts( std::vector<quickhull::Vector3<float>> &cloud )
{
	constexpr float kOutlierRatio = 5.0f;
	constexpr int   kMaxIters     = 4;
	for ( int iter = 0; iter < kMaxIters; ++iter )
	{
		if ( cloud.size() < 8 )
			return;
		float cx = 0.0f, cy = 0.0f, cz = 0.0f;
		for ( const auto &p : cloud ) { cx += p.x; cy += p.y; cz += p.z; }
		const float inv = 1.0f / static_cast<float>( cloud.size() );
		cx *= inv; cy *= inv; cz *= inv;

		std::vector<float> dists;
		dists.reserve( cloud.size() );
		for ( const auto &p : cloud )
		{
			const float dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
			dists.push_back( std::sqrt( dx * dx + dy * dy + dz * dz ) );
		}
		std::vector<float> sorted = dists;
		std::nth_element( sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end() );
		const float median = sorted[ sorted.size() / 2 ];
		if ( median < 1e-4f )
			return;
		const float threshold = kOutlierRatio * median;

		const size_t before = cloud.size();
		std::vector<quickhull::Vector3<float>> kept;
		kept.reserve( cloud.size() );
		for ( size_t i = 0; i < cloud.size(); ++i )
		{
			if ( dists[i] <= threshold )
				kept.push_back( cloud[i] );
		}
		if ( kept.size() == before )
			return; // stable
		if ( kept.size() < 4 )
			return; // too few left to form a hull; keep original
		cloud.swap( kept );
	}
}

} // namespace

void CPhysConvex_Impl::RecomputeDerived()
{
	ComputeAABB( points, aabbMin, aabbMax );
	volume      = ComputeVolume( points, triangles );
	surfaceArea = ComputeSurfaceArea( points, triangles );
	centroid    = ComputeCentroid( points, triangles );
	ComputeInertiaDiagonal(
		points, triangles, centroid, volume,
		inertiaDiag.x, inertiaDiag.y, inertiaDiag.z );
	oppositeEdges = BuildOppositeEdges( triangles );
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

	FilterOutlierVerts( cloud );
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

	// Normalize winding so every triangle's outward normal points AWAY from
	// the hull's centroid. IVP's collision detection treats the triangles
	// as oriented surfaces — if the winding is flipped, the engine thinks
	// the solid region is everywhere outside the hull and collision tests
	// pass straight through. quickhull's `CCW=true` is relative to whichever
	// face the algorithm picks first, which in practice can come out either
	// way depending on input point order; we rewind here from first
	// principles.
	{
		Vector centroid( 0.0f, 0.0f, 0.0f );
		for ( const Vector &p : c->points )
			centroid += p;
		centroid /= static_cast<float>( c->points.size() );
		for ( size_t t = 0; t + 2 < c->triangles.size(); t += 3 )
		{
			const Vector &a = c->points[ c->triangles[t + 0] ];
			const Vector &b = c->points[ c->triangles[t + 1] ];
			const Vector &dd = c->points[ c->triangles[t + 2] ];
			// Triangle normal × (triangle-vertex - centroid). If the normal
			// points inward, swap two indices to flip it outward.
			const Vector n = CrossProduct( b - a, dd - a );
			if ( DotProduct( n, a - centroid ) < 0.0f )
				std::swap( c->triangles[t + 1], c->triangles[t + 2] );
		}
	}

	c->RecomputeDerived();
	return c;
}

} // namespace mdlforge_vphysics
