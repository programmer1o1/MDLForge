// Internal definition of CPhysCollide — the opaque compiled collision model
// that studiomdl serializes into the .phy file.

#pragma once

#include "cphysconvex.h"

#include "mathlib/vector.h"

#include <vector>

namespace mdlforge_vphysics
{

class CPhysCollide_Impl
{
public:
	// Convex leaves — owned; destructor deletes them.
	std::vector<CPhysConvex_Impl *> convexes;

	// Computed on construction, adjustable via CollideSetMassCenter.
	Vector massCenter{ 0.0f, 0.0f, 0.0f };

	// Drag-axis-area fractions. Valve defaults to (1,1,1) when
	// buildDragAxisAreas is false.
	Vector orthographicAreas{ 1.0f, 1.0f, 1.0f };

	// Union AABB across all contained convexes (in hull-local space).
	Vector aabbMin{ 0.0f, 0.0f, 0.0f };
	Vector aabbMax{ 0.0f, 0.0f, 0.0f };

	~CPhysCollide_Impl();

	// Wraps an array of CPhysConvex_Impl pointers and takes ownership. Computes
	// mass center as the volume-weighted centroid of the contained convexes.
	static CPhysCollide_Impl *Build( CPhysConvex_Impl **convex, int count );

	void RecomputeUnionAABB();
	void RecomputeMassCenter();
};

} // namespace mdlforge_vphysics
