// Internal definition of CPhysConvex — the opaque type studiomdl gets back
// from IPhysicsCollision::ConvexFromVerts. Callers only ever see a pointer,
// so this definition stays out of public headers.

#pragma once

#include "mathlib/vector.h"

#include <cstdint>
#include <vector>

namespace mdlforge_vphysics
{

class CPhysConvex_Impl
{
public:
	std::vector<Vector>   points;     // hull vertices
	std::vector<uint16_t> triangles;  // 3 indices per triangle, CCW from outside
	// Per-edge linear index of the twin edge in the adjacent triangle that
	// shares this half-edge. Indexed the same way as `triangles` — three
	// entries per triangle, one per edge. IVP's contact walker follows
	// these to traverse the hull during collision resolution; without
	// them, contacts that span triangle boundaries fail and the rigid
	// body partially penetrates surfaces.
	// Per-edge signed 15-bit value destined for IVP's compact_edge
	// opposite_index field: the distance (in 4-byte slot units) from this
	// edge to its twin half-edge in an adjacent triangle. See
	// BuildOppositeEdgeOffsets in cphysconvex.cpp for the exact layout
	// convention.
	std::vector<int16_t>  oppositeEdges;
	// Per-triangle "pierce" index: for triangle T, the index of the
	// triangle on the far side of the hull that a ray from T's centroid
	// along T's inward normal exits through. IVP uses this during
	// penetration-depth queries to walk from the contact face to the
	// opposite hull face; the query loops forever when it's unset (zero)
	// and adjacency is populated.
	std::vector<uint16_t> pierceIndices;
	Vector                aabbMin{ 0.0f, 0.0f, 0.0f };
	Vector                aabbMax{ 0.0f, 0.0f, 0.0f };
	// True volume-weighted centroid — matches what Valve's vphysics uses for
	// mass-center output. Computed via tetrahedral decomposition; for hulls
	// whose AABB center drifts from the true centroid (asymmetric hulls)
	// this is what the engine lines the rigid body up against at runtime.
	Vector                centroid{ 0.0f, 0.0f, 0.0f };
	// Principal moments of inertia about the centroid, per unit mass, in
	// the same units as `points`. The three values are the (x,y,z)
	// diagonal of the inertia tensor; off-diagonal terms are neglected,
	// matching Valve's .phy convention of storing only three values.
	Vector                inertiaDiag{ 1.0f, 1.0f, 1.0f };
	float                 volume       = 0.0f;
	float                 surfaceArea  = 0.0f;
	unsigned int          gameData     = 0;

	// Build a convex hull from an array of point pointers (as studiomdl passes).
	// Returns nullptr on failure (too few points / degenerate input).
	static CPhysConvex_Impl *BuildFromVerts( Vector **ppVerts, int vertCount );

	// Compute cached volume, surfaceArea, AABB from points + triangles.
	void RecomputeDerived();
};

} // namespace mdlforge_vphysics
